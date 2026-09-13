/** @file car_main.c
 *
 * @brief Vehicle controller. Creates the tasks and owns mission state.
 *
 * This is the only file that includes all five subsystem headers. Anything
 * one subsystem needs from another flows through here, or through the
 * types in car_types.h, never by one module including another.
 *
 * NOTE: One task per periodic subsystem, each pacing itself with
 * tk_dly_tsk(). The kernel tick is CNF_TIMER_PERIOD in config/config.h,
 * 10 ms as shipped, and a delay rounds up to the next tick. Lower the tick
 * if a loop must run faster than that.
 *
 * NOTE: Every public getter below is called from a task other than the one
 * that updates its module, so each module must make its getters safe: copy
 * the value inside a short DI()/EI() section or under a mutex. The
 * telemetry struct in this file is protected by g_telemetry_mutex as the
 * worked example. Kernel types (INT, ID, ER, UB) appear only where the
 * kernel API requires them.
 *
 * Owner: the team. This is the integration surface and has no single buddy.
 * The state transitions are filled in together once the subsystems below them
 * work.
 */

#include <stdbool.h>
#include <stdint.h>

/* No <stddef.h> here: the kernel typedefs its own size_t and the two
 * collide. NULL comes from the kernel headers. */
#include <tk/tkernel.h>

#include "car_config.h"
#include "car_log.h"
#include "car_types.h"
#include "comms.h"
#include "imu_terrain.h"
#include "line_barcode.h"
#include "motion.h"
#include "scanning.h"

/* Priorities: a lower number runs first. Motion outranks everything because
 * a late PID update shows up as a wobble at the wheels. */
#define CAR_PRI_MOTION         5
#define CAR_PRI_IMU            6
#define CAR_PRI_LINE           6
#define CAR_PRI_MISSION        7
#define CAR_PRI_COMMS          8
#define CAR_TASK_STACK_BYTES   4096

static car_mission_state_t g_state           = CAR_STATE_INIT;
static car_telemetry_t     g_telemetry       = { 0 };
static ID                  g_telemetry_mutex = 0;

/* Single value stores, one writer each, so no lock. line_task writes the
 * error, and both line_task and the remote command handler write the
 * pending command; the last whole value written wins, by design. */
static volatile int16_t           g_line_error          = 0;
static volatile car_nav_command_t g_pending_nav_command = CAR_NAV_NONE;

static void motion_task (INT stacd, void * p_exinf);
static void imu_task (INT stacd, void * p_exinf);
static void line_task (INT stacd, void * p_exinf);
static void mission_task (INT stacd, void * p_exinf);
static void comms_task (INT stacd, void * p_exinf);
static bool start_task (T_CTSK const * p_ctsk, char const * p_name);
static void handle_remote_command (car_nav_command_t command);
static void run_state_machine (void);
static void publish_telemetry (void);

static T_CTSK const g_ctsk_motion =
{
    .exinf   = NULL,
    .tskatr  = TA_HLNG | TA_RNG3,
    .task    = motion_task,
    .itskpri = CAR_PRI_MOTION,
    .stksz   = CAR_TASK_STACK_BYTES,
};

static T_CTSK const g_ctsk_imu =
{
    .exinf   = NULL,
    .tskatr  = TA_HLNG | TA_RNG3,
    .task    = imu_task,
    .itskpri = CAR_PRI_IMU,
    .stksz   = CAR_TASK_STACK_BYTES,
};

static T_CTSK const g_ctsk_line =
{
    .exinf   = NULL,
    .tskatr  = TA_HLNG | TA_RNG3,
    .task    = line_task,
    .itskpri = CAR_PRI_LINE,
    .stksz   = CAR_TASK_STACK_BYTES,
};

static T_CTSK const g_ctsk_mission =
{
    .exinf   = NULL,
    .tskatr  = TA_HLNG | TA_RNG3,
    .task    = mission_task,
    .itskpri = CAR_PRI_MISSION,
    .stksz   = CAR_TASK_STACK_BYTES,
};

static T_CTSK const g_ctsk_comms =
{
    .exinf   = NULL,
    .tskatr  = TA_HLNG | TA_RNG3,
    .task    = comms_task,
    .itskpri = CAR_PRI_COMMS,
    .stksz   = CAR_TASK_STACK_BYTES,
};

/**
 * @brief Kernel entry point. Runs as the initial task once the kernel is up.
 *
 * @return Never returns. The kernel shuts down if the initial task returns.
 */
INT usermain (void)
{
    T_CMTX const cmtx =
    {
        .exinf   = NULL,
        .mtxatr  = TA_INHERIT,
        .ceilpri = 0,
    };

    CAR_LOG(CAR_LOG_INFO, "car firmware starting\n");
    g_telemetry_mutex = tk_cre_mtx(&cmtx);

    /* Bring up every subsystem. Any failure halts before the wheels move. */
    if ((0 >= g_telemetry_mutex) || (CAR_OK != motion_init())
        || (CAR_OK != line_init()) || (CAR_OK != imu_init())
        || (CAR_OK != scan_init()) || (CAR_OK != comms_init()))
    {
        CAR_LOG(CAR_LOG_ERROR, "subsystem init failed, halting\n");
        g_state = CAR_STATE_HALTED;
    }

    (void)comms_set_command_handler(handle_remote_command);

    (void)start_task(&g_ctsk_motion, "motion");
    (void)start_task(&g_ctsk_imu, "imu");
    (void)start_task(&g_ctsk_line, "line");
    (void)start_task(&g_ctsk_mission, "mission");
    (void)start_task(&g_ctsk_comms, "comms");

    /* The initial task must never return: the kernel shuts down if it does. */
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}

/**
 * @brief Run the PID at a fixed rate.
 *
 * ponytail: tk_dly_tsk() drifts by up to one tick per loop. Switch to a
 * cyclic handler that wakes this task with tk_wup_tsk() if the PID needs a
 * period that does not wander.
 */
static void motion_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        (void)motion_tick();
        (void)tk_dly_tsk(MOTION_TICK_PERIOD_MSEC);
    }
}

/**
 * @brief Sample and filter the IMU at a fixed rate.
 */
static void imu_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        (void)imu_update();
        (void)tk_dly_tsk(IMU_SAMPLE_PERIOD_MSEC);
    }
}

/**
 * @brief Own every read of the line sensors, so the module has one caller.
 */
static void line_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        int16_t           error   = 0;
        car_nav_command_t command = CAR_NAV_NONE;

        if (CAR_OK == line_get_position(&error))
        {
            g_line_error = error;
        }

        if (CAR_OK == barcode_poll(&command))
        {
            g_pending_nav_command = command;
        }

        (void)tk_dly_tsk(LINE_SAMPLE_PERIOD_MSEC);
    }
}

/**
 * @brief Advance the mission state machine at a fixed rate.
 */
static void mission_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        run_state_machine();
        (void)tk_dly_tsk(CAR_MISSION_PERIOD_MSEC);
    }
}

/**
 * @brief Service the network and publish telemetry and heartbeat.
 *
 * NOTE: lwIP runs with NO_SYS=1, which means every call into the comms
 * module must come from this one task. Never publish from elsewhere.
 */
static void comms_task (INT stacd, void * p_exinf)
{
    uint32_t since_telemetry_msec = 0u;
    uint32_t since_heartbeat_msec = 0u;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        (void)comms_poll();

        if (since_telemetry_msec >= CAR_TELEMETRY_PERIOD_MSEC)
        {
            publish_telemetry();
            since_telemetry_msec = 0u;
        }

        if (since_heartbeat_msec >= COMMS_HEARTBEAT_PERIOD_MSEC)
        {
            (void)comms_publish_heartbeat();
            since_heartbeat_msec = 0u;
        }

        since_telemetry_msec += COMMS_POLL_PERIOD_MSEC;
        since_heartbeat_msec += COMMS_POLL_PERIOD_MSEC;
        (void)tk_dly_tsk(COMMS_POLL_PERIOD_MSEC);
    }
}

/**
 * @brief Create and start one task, logging the name on failure.
 *
 * @param[in] p_ctsk Task creation packet.
 * @param[in] p_name Short name for the log line.
 *
 * @return true if the task is running.
 */
static bool start_task (T_CTSK const * p_ctsk, char const * p_name)
{
    ID   tskid     = tk_cre_tsk(p_ctsk);
    bool b_started = false;

    if (0 < tskid)
    {
        b_started = (E_OK == tk_sta_tsk(tskid, 0));
    }

    if (!b_started)
    {
        CAR_LOG(CAR_LOG_ERROR, "task %s failed to start\n", p_name);
    }

    return b_started;
}

/**
 * @brief Receive a navigation command from MQTT. Runs in comms_task.
 *
 * @param[in] command The decoded command.
 */
static void handle_remote_command (car_nav_command_t command)
{
    g_pending_nav_command = command;
}

/**
 * @brief Advance the mission by one step.
 */
static void run_state_machine (void)
{
    switch (g_state)
    {
        case CAR_STATE_INIT:
            // TODO: Go to FOLLOW_LINE once line_calibrate() and
            //       imu_calibrate() both return CAR_OK.
        break;

        case CAR_STATE_FOLLOW_LINE:
            // TODO: Turn g_line_error into a steering correction each
            //       tick. Go to DECODE_BARCODE when g_pending_nav_command
            //       is not CAR_NAV_NONE, to AVOID_OBSTACLE when
            //       scan_coarse() reports a valid profile, to HALTED on
            //       imu_is_collision_detected().
        break;

        case CAR_STATE_DECODE_BARCODE:
            // TODO: Take g_pending_nav_command, clear it, store it in
            //       g_telemetry.last_nav_command under the mutex, and go
            //       to EXECUTE_TURN, or back to FOLLOW_LINE for STRAIGHT.
        break;

        case CAR_STATE_EXECUTE_TURN:
            // TODO: Issue motion_turn_left(), motion_turn_right() or a
            //       180 degree turn once, then go to FOLLOW_LINE when
            //       motion_is_busy() goes false.
        break;

        case CAR_STATE_AVOID_OBSTACLE:
            // TODO: scan_fine() around the coarse bearing, store the
            //       profile in g_telemetry.last_obstacle under the mutex,
            //       act on scan_plan_avoidance(), then go to RECOVER_LINE.
        break;

        case CAR_STATE_RECOVER_LINE:
            // TODO: Call scan_recover_line() each tick. Go to FOLLOW_LINE
            //       on CAR_OK, to HALTED on CAR_ERR_TIMEOUT.
        break;

        case CAR_STATE_HALTED:
            (void)motion_stop();
        break;

        default:
            g_state = CAR_STATE_HALTED;
        break;
    }
}

/**
 * @brief Gather one snapshot from every subsystem and publish it.
 */
static void publish_telemetry (void)
{
    motion_state_t motion = { 0 };
    car_hump_t     hump   = { 0 };

    (void)motion_get_state(&motion);
    (void)imu_get_peak_hump(&hump);

    if (E_OK == tk_loc_mtx(g_telemetry_mutex, TMO_FEVR))
    {
        (void)line_get_sensor_mask(&g_telemetry.line_sensor_mask);

        g_telemetry.mission_state       = g_state;
        g_telemetry.speed_mm_per_sec    = motion.speed_mm_per_sec;
        g_telemetry.encoder_count_left  = motion.encoder_count_left;
        g_telemetry.encoder_count_right = motion.encoder_count_right;
        g_telemetry.total_distance_mm   = motion.distance_mm;
        g_telemetry.peak_hump_height_mm = hump.peak_height_mm;

        (void)comms_publish_telemetry(&g_telemetry);
        (void)tk_unl_mtx(g_telemetry_mutex);
    }
}

/*** end of file ***/

