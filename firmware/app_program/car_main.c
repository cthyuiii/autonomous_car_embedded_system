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
 * NOTE: A subsystem whose init reports CAR_ERR_NOT_IMPLEMENTED is treated
 * as absent, not broken: the car drives without it. Only motion and the
 * line sensors are mandatory. Any other failure halts before the wheels
 * move.
 *
 * Owner: the team. This is the integration surface and has no single buddy.
 */

#include <stdbool.h>
#include <stdint.h>

/* No <stddef.h> here: the kernel typedefs its own size_t and the two
 * collide. NULL comes from the kernel headers. */
#include <tk/tkernel.h>

#include "car_config.h"
#include "car_hw.h"
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

#define CAR_STRAIGHT_AHEAD_DEG SCAN_CENTRE_ANGLE_DEG
#define CAR_TURN_DEG           90u
#define CAR_DETOUR_STEPS       7u

/** One leg of the box detour around an obstacle. */
typedef struct
{
    bool     b_turn;        /* true turns, false drives forward */
    bool     b_away;        /* turn away from the obstacle, else toward */
    uint16_t amount;        /* degrees or mm */
} detour_step_t;

/* Turn away, step sideways, turn back, pass the obstacle, turn in, step
 * back, turn straight: a box that ends parallel to the original heading. */
/* Sized from the obstacle at decision time by plan_detour(), falling
 * back to these when the fine scan measured no width. The shape never
 * changes, only the two distances. */
static detour_step_t g_detour[CAR_DETOUR_STEPS] =
{
    { true,  true,  SCAN_DETOUR_TURN_DEG },
    { false, false, SCAN_DETOUR_SIDE_MM },
    { true,  false, SCAN_DETOUR_TURN_DEG },
    { false, false, SCAN_DETOUR_DEPTH_MM },
    { true,  false, SCAN_DETOUR_TURN_DEG },
    { false, false, SCAN_DETOUR_SIDE_MM },
    { true,  true,  SCAN_DETOUR_TURN_DEG },
};

static car_mission_state_t g_state           = CAR_STATE_INIT;
static car_telemetry_t     g_telemetry       = { 0 };
static ID                  g_telemetry_mutex = 0;

/* Which subsystems answered at init. Motion and line are mandatory. */
static bool g_b_has_imu   = false;
static bool g_b_has_scan  = false;
static bool g_b_has_comms = false;

/* Single value stores, one writer each, so no lock. line_task writes the
 * error, the seen flag and the junction flag; both line_task and the remote
 * command handler write the pending command; the last whole value written
 * wins, by design. */
static volatile int16_t           g_line_error          = 0;
static volatile bool              g_b_line_seen         = false;
static volatile bool              g_b_junction          = false;
static volatile car_nav_command_t g_pending_nav_command = CAR_NAV_NONE;

/* Line following law state, mission task only. */
static int16_t  g_prev_error       = 0;
static int16_t  g_last_side        = 0;    // Last non zero error, to lean to
static uint32_t g_lost_since_mm    = 0u;
static uint32_t g_last_sonar_msec  = 0u;
static uint16_t g_seen_samples     = 0u;

/* Turn execution state, mission task only. */
static car_nav_command_t g_active_command    = CAR_NAV_NONE;
static uint32_t          g_turn_search_start = 0u;
static bool              g_b_turn_issued     = false;

/* Obstacle avoidance state, mission task only. */
static bool               g_b_scan_done      = false;
static bool               g_b_reversing      = false;
static uint8_t            g_detour_index     = 0u;
static car_avoid_action_t g_detour_side      = CAR_AVOID_LEFT;

/* Backing out of a lane that turned out to be blocked. The legs already
 * driven are undone in reverse order, which puts the car back where it
 * started so the other side can be tried from the same place. */
static bool               g_b_retracing      = false;
static uint8_t            g_retrace_index    = 0u;
static car_avoid_action_t g_probe_side       = CAR_AVOID_LEFT;

/* A detour that ends where it started leaves the same obstacle ahead, so the
 * car would go round it again forever. Count the attempts and stop going
 * round once they run out. Cleared only by CAR_DETOUR_CLEAR_MM of following,
 * which is the one thing that proves the obstacle is actually behind us. */
static uint8_t            g_detour_attempts  = 0u;
static uint32_t           g_follow_start_mm  = 0u;

/* Set once per mission tick by the shared ranging, so every state sees the
 * same answer without any of them spending a ranging of its own. An
 * obstacle is just as solid while turning or searching as while following,
 * which is why this is checked outside the switch rather than inside one
 * state. */
static bool               g_b_obstacle       = false;

/* Coarse sweep while driving. The sides are early warning only: nothing
 * steers or plans from them, they go in the log and the telemetry so the
 * decision the fine scan later makes can be read against what the car
 * already knew. */
static uint8_t            g_sweep_index      = 0u;
static uint16_t           g_range_left_mm    = SONAR_MAX_RANGE_MM;
static uint16_t           g_range_right_mm   = SONAR_MAX_RANGE_MM;

/* Collision handling. A hit outranks every other state, so it is checked
 * outside the switch and can interrupt a detour leg, a turn or a search
 * mid move. The count is never cleared: three hits in one run means the
 * car is somewhere it cannot drive, and reversing again will not help. */
static uint8_t            g_collisions       = 0u;
static uint32_t           g_collision_msec   = 0u;
static bool               g_b_collision_done = false;

/* True once a detour leg has moved the car off the line. Every way out of
 * avoidance then has to go through the search, because the brief allows the
 * bypass to leave the line but requires the line to be reacquired after it.
 * Cleared on entering avoidance, so a plain reverse never sets it. */
static bool               g_b_off_line       = false;

static void motion_task (INT stacd, void * p_exinf);
static void imu_task (INT stacd, void * p_exinf);
static void line_task (INT stacd, void * p_exinf);
static void mission_task (INT stacd, void * p_exinf);
static void comms_task (INT stacd, void * p_exinf);
static bool start_task (T_CTSK const * p_ctsk, char const * p_name);
static bool bring_up (char const * p_name, car_status_t status,
                      bool b_mandatory, bool * p_present);
static void handle_remote_command (car_nav_command_t command);
static void run_state_machine (void);
static void enter_state (car_mission_state_t state);
static void state_follow_line (void);
static void state_decode_barcode (void);
static void state_execute_turn (void);
static void state_avoid_obstacle (void);
static void state_recover_line (void);
static void state_collision (void);
static bool steer_along_line (void);
static bool obstacle_ahead (void);
static bool path_blocked (void);
static uint16_t terrain_speed (uint16_t level_speed);
static void issue_detour_step (detour_step_t const * p_step);
static void undo_detour_step (detour_step_t const * p_step);
static void plan_detour (car_obstacle_profile_t const * p_profile);
static uint16_t clamp_mm (uint32_t value, uint16_t low, uint16_t high);
static void retrace_step (void);
static car_avoid_action_t other_side (car_avoid_action_t side);
static uint32_t distance_now (void);
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
 * Brings every subsystem up, calibrates the two that need it while no other
 * task is running, then starts the tasks. Calibration happens here on
 * purpose: the IMU calibration reads the bus itself, and the driver does
 * not serialise two tasks.
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
    bool b_ok      = true;
    bool b_present = false;

    CAR_LOG(CAR_LOG_INFO, "car firmware starting\n");
    g_telemetry_mutex = tk_cre_mtx(&cmtx);
    b_ok = (0 < g_telemetry_mutex);

    b_ok = bring_up("motion", motion_init(), true, &b_present) && b_ok;
    b_ok = bring_up("line", line_init(), true, &b_present) && b_ok;
    b_ok = bring_up("imu", imu_init(), false, &g_b_has_imu) && b_ok;
    b_ok = bring_up("scan", scan_init(), false, &g_b_has_scan) && b_ok;
    b_ok = bring_up("comms", comms_init(), false, &g_b_has_comms) && b_ok;

    if (b_ok)
    {
        if (CAR_OK != line_calibrate())
        {
            CAR_LOG(CAR_LOG_ERROR, "line calibrate failed\n");
        }

        if (g_b_has_imu && (CAR_OK != imu_calibrate()))
        {
            /* A tilted or moving start is not fatal, just less accurate. */
            CAR_LOG(CAR_LOG_ERROR, "imu calibrate failed, continuing\n");
        }
    }
    else
    {
        CAR_LOG(CAR_LOG_ERROR, "subsystem init failed, halting\n");
        g_state = CAR_STATE_HALTED;
    }

    (void)comms_set_command_handler(handle_remote_command);

    (void)start_task(&g_ctsk_motion, "motion");
    (void)start_task(&g_ctsk_line, "line");
    (void)start_task(&g_ctsk_mission, "mission");

    if (g_b_has_imu)
    {
        (void)start_task(&g_ctsk_imu, "imu");
    }

    if (g_b_has_comms)
    {
        (void)start_task(&g_ctsk_comms, "comms");
    }

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
        if (CAR_ERR_HARDWARE == motion_tick())
        {
            CAR_LOG(CAR_LOG_ERROR, "encoder stalled, halting\n");
            g_state = CAR_STATE_HALTED;
        }

        (void)tk_dly_tsk(MOTION_TICK_PERIOD_MSEC);
    }
}

/**
 * @brief Feed the IMU the wheel data, then sample and filter it.
 */
static void imu_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        motion_state_t motion = { 0 };

        if (CAR_OK == motion_get_state(&motion))
        {
            (void)imu_feed_odometry(motion.distance_mm, motion.left_mm_per_sec,
                                    motion.right_mm_per_sec);
        }

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
        car_status_t      status  = line_get_position(&error);

        if (CAR_OK == status)
        {
            g_line_error = error;
        }

        g_b_line_seen = (CAR_OK == status);
        g_b_junction  = line_is_at_junction();

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
    uint32_t last_telemetry_msec = 0u;
    uint32_t last_heartbeat_msec = 0u;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        /* Timed off the clock, not by adding up the delay: comms_poll()
         * takes about as long again as the delay itself, so counting
         * nominal periods ran both publishes at half the rate asked for. */
        uint32_t now = car_hw_msec();

        (void)comms_poll();

        if ((now - last_telemetry_msec) >= CAR_TELEMETRY_PERIOD_MSEC)
        {
            publish_telemetry();
            last_telemetry_msec = now;
        }

        if ((now - last_heartbeat_msec) >= COMMS_HEARTBEAT_PERIOD_MSEC)
        {
            (void)comms_publish_heartbeat();
            last_heartbeat_msec = now;
        }

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
 * @brief Interpret one subsystem's init result.
 *
 * @param[in]  p_name      Name for the log.
 * @param[in]  status      What its init returned.
 * @param[in]  b_mandatory true if the car cannot run without it.
 * @param[out] p_present   Set true if the subsystem is usable.
 *
 * @return false if the car must halt.
 */
static bool bring_up (char const * p_name, car_status_t status,
                      bool b_mandatory, bool * p_present)
{
    bool b_ok = true;

    *p_present = (CAR_OK == status);

    if (CAR_OK == status)
    {
        CAR_LOG(CAR_LOG_INFO, "%s ready\n", p_name);
    }
    else if ((CAR_ERR_NOT_IMPLEMENTED == status) && !b_mandatory)
    {
        CAR_LOG(CAR_LOG_INFO, "%s absent, continuing without it\n", p_name);
    }
    else
    {
        CAR_LOG(CAR_LOG_ERROR, "%s init failed with %d\n", p_name, status);
        b_ok = false;
    }

    return b_ok;
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
    /* The scan phase of avoidance is about to sweep the whole arc, so it
     * would only be paying for a reading it is about to take anyway, and a
     * halted car is not going to drive into anything. */
    bool b_own_look = ((CAR_STATE_AVOID_OBSTACLE == g_state) && !g_b_scan_done);

    /* Before anything else, and regardless of what the car was doing.
     * Stopping is the only response that is right whatever was hit, and
     * it has to happen in the same tick the hit is seen. */
    if (g_b_has_imu && imu_is_collision_detected()
        && (CAR_STATE_COLLISION != g_state)
        && (CAR_STATE_HALTED != g_state))
    {
        (void)motion_stop();
        enter_state(CAR_STATE_COLLISION);
    }

    if (!b_own_look && (CAR_STATE_HALTED != g_state)
        && (CAR_STATE_COLLISION != g_state))
    {
        g_b_obstacle = obstacle_ahead();
    }

    switch (g_state)
    {
        case CAR_STATE_INIT:
            // TODO: Go to FOLLOW_LINE once line_calibrate() and
            //       imu_calibrate() both return CAR_OK.
            /* Both calibrations ran in usermain() before any task existed,
             * so there is nothing left to wait for. */
            (void)motion_set_speed(CAR_FOLLOW_SPEED_MM_PER_SEC);
            enter_state(CAR_STATE_FOLLOW_LINE);
        break;

        case CAR_STATE_FOLLOW_LINE:
            // TODO: Turn g_line_error into a steering correction each
            //       tick. Go to DECODE_BARCODE when g_pending_nav_command
            //       is not CAR_NAV_NONE, to AVOID_OBSTACLE when
            //       scan_coarse() reports a valid profile, to HALTED on
            //       imu_is_collision_detected().
            state_follow_line();
        break;

        case CAR_STATE_DECODE_BARCODE:
            // TODO: Take g_pending_nav_command, clear it, store it in
            //       g_telemetry.last_nav_command under the mutex, and go
            //       to EXECUTE_TURN, or back to FOLLOW_LINE for STRAIGHT.
            state_decode_barcode();
        break;

        case CAR_STATE_EXECUTE_TURN:
            // TODO: Issue motion_turn_left(), motion_turn_right() or a
            //       180 degree turn once, then go to FOLLOW_LINE when
            //       motion_is_busy() goes false.
            state_execute_turn();
        break;

        case CAR_STATE_AVOID_OBSTACLE:
            // TODO: scan_fine() around the coarse bearing, store the
            //       profile in g_telemetry.last_obstacle under the mutex,
            //       act on scan_plan_avoidance(), then go to RECOVER_LINE.
            state_avoid_obstacle();
        break;

        case CAR_STATE_RECOVER_LINE:
            // TODO: Call scan_recover_line() each tick. Go to FOLLOW_LINE
            //       on CAR_OK, to HALTED on CAR_ERR_TIMEOUT.
            state_recover_line();
        break;

        case CAR_STATE_HALTED:
            (void)motion_stop();
        break;

        case CAR_STATE_COLLISION:
            state_collision();
        break;

        default:
            g_state = CAR_STATE_HALTED;
        break;
    }
}

/**
 * @brief Switch state, resetting whatever the new state needs fresh.
 *
 * @param[in] state Destination.
 */
static void enter_state (car_mission_state_t state)
{
#if CAR_SKIP_LINE_RECOVERY
    /* Every route into the search comes through here, so one redirect
     * covers losing the line, a bypass and the end of a detour alike. */
    if (CAR_STATE_RECOVER_LINE == state)
    {
        state = CAR_STATE_FOLLOW_LINE;
    }
#endif

    CAR_LOG(CAR_LOG_INFO, "state %d -> %d\n", g_state, state);

    switch (state)
    {
        case CAR_STATE_FOLLOW_LINE:
            g_prev_error      = 0;
            g_lost_since_mm   = distance_now();
            g_follow_start_mm = distance_now();
            g_seen_samples    = 0u;
            (void)motion_set_speed(CAR_BARCODE_SPEED_MM_PER_SEC);
        break;

        case CAR_STATE_EXECUTE_TURN:
            g_turn_search_start = distance_now();
            g_b_turn_issued     = false;
        break;

        case CAR_STATE_AVOID_OBSTACLE:
            g_b_scan_done      = false;
            g_b_reversing      = false;
            g_b_off_line       = false;
            g_b_retracing      = false;
            g_retrace_index    = 0u;
            g_probe_side       = CAR_AVOID_LEFT;
            g_detour_index     = 0u;
            (void)motion_stop();
        break;

        case CAR_STATE_HALTED:
            (void)motion_stop();
        break;

        case CAR_STATE_COLLISION:
            (void)motion_stop();
            g_collisions++;
            g_collision_msec   = car_hw_msec();
            g_b_collision_done = false;
            CAR_LOG(CAR_LOG_ERROR, "collision %u, stopping dead\n",
                    g_collisions);
        break;

        default:
            /* Nothing to reset. */
        break;
    }

    g_state = state;
}

/**
 * @brief Follow the line, watching for a collision, an obstacle, a barcode
 *        command, and the line being lost for good.
 */
static void state_follow_line (void)
{
    if ((0u != g_detour_attempts)
        && ((distance_now() - g_follow_start_mm) >= CAR_DETOUR_CLEAR_MM))
    {
        /* Clear road since the last detour: whatever it was, it is behind
         * us, and the next obstacle deserves a fresh set of attempts. */
        g_detour_attempts = 0u;
    }

    if (g_b_obstacle)
    {
        enter_state(CAR_STATE_AVOID_OBSTACLE);
    }
    else if (CAR_NAV_NONE != g_pending_nav_command)
    {
        enter_state(CAR_STATE_DECODE_BARCODE);
    }
    else if (!steer_along_line())
    {
        (void)motion_stop();
        (void)scan_recover_start(g_last_side < 0);
        enter_state(CAR_STATE_RECOVER_LINE);
    }
    else
    {
        /* Still on the line. */
    }
}

/**
 * @brief Take the pending command and decide whether it needs a turn.
 */
static void state_decode_barcode (void)
{
    car_nav_command_t command = g_pending_nav_command;

    g_pending_nav_command = CAR_NAV_NONE;
    CAR_LOG(CAR_LOG_INFO, "command %d\n", command);

    if (E_OK == tk_loc_mtx(g_telemetry_mutex, TMO_FEVR))
    {
        g_telemetry.last_nav_command = command;
        (void)tk_unl_mtx(g_telemetry_mutex);
    }

    if ((CAR_NAV_LEFT == command) || (CAR_NAV_RIGHT == command)
        || (CAR_NAV_UTURN == command))
    {
        g_active_command = command;
        enter_state(CAR_STATE_EXECUTE_TURN);
    }
    else
    {
        enter_state(CAR_STATE_FOLLOW_LINE);
    }
}

/**
 * @brief Keep following to the next junction, then make the turn once.
 *
 * NOTE: A turn command is taken to mean "at the next junction". If no
 * junction shows up within CAR_TURN_SEARCH_MM the turn is made anyway.
 * TODO: confirm against the course write-up whether turns are immediate.
 */
static void state_execute_turn (void)
{
    if (g_b_obstacle && !g_b_turn_issued)
    {
        /* Put the command back so the turn still happens once the way is
         * clear, rather than being lost to the detour. */
        CAR_LOG(CAR_LOG_INFO, "obstacle before the turn, avoiding first\n");
        g_pending_nav_command = g_active_command;
        enter_state(CAR_STATE_AVOID_OBSTACLE);
    }
    else if (!g_b_turn_issued)
    {
        bool b_search_over = ((distance_now() - g_turn_search_start)
                              >= CAR_TURN_SEARCH_MM);

        if (g_b_junction || b_search_over)
        {
            (void)motion_stop();
            (void)motion_set_speed(CAR_FOLLOW_SPEED_MM_PER_SEC);

            if (CAR_NAV_LEFT == g_active_command)
            {
                (void)motion_turn_left(CAR_TURN_DEG);
            }
            else if (CAR_NAV_RIGHT == g_active_command)
            {
                (void)motion_turn_right(CAR_TURN_DEG);
            }
            else
            {
                (void)motion_turn_right(CAR_UTURN_DEG);
            }

            g_b_turn_issued = true;
        }
        else if (!steer_along_line())
        {
            /* Lost the line on the way to the junction: turn where we are. */
            g_b_junction = true;
        }
        else
        {
            /* Still approaching. */
        }
    }
    else if (!motion_is_busy())
    {
        enter_state(CAR_STATE_FOLLOW_LINE);
    }
    else
    {
        /* Turning. */
    }
}

/**
 * @brief Profile the obstacle once, then drive the detour step by step.
 */
static void state_avoid_obstacle (void)
{
    if (motion_is_busy())
    {
        /* g_b_scan_done means a detour leg is running. A reverse is the
         * other busy case, and an obstacle ahead during one is the thing
         * being backed away from, so it is left alone. */
        if (g_b_obstacle && g_b_scan_done)
        {
            CAR_LOG(CAR_LOG_INFO, "obstacle during leg %u, replanning\n",
                    g_detour_index);
            (void)motion_stop();
            g_b_scan_done  = false;
            g_detour_index = 0u;
        }

        return;
    }

    if (g_b_retracing)
    {
        retrace_step();
        return;
    }

    if (g_b_reversing)
    {
        /* Back at the origin. Probe the chosen side for real rather than
         * ranging again from 100 mm further back, which only ever repeats
         * the answer that sent the car backwards in the first place. */
        g_b_reversing  = false;
        g_b_scan_done  = true;
        g_detour_index = 0u;
        CAR_LOG(CAR_LOG_INFO, "probing %s\n",
                (CAR_AVOID_LEFT == g_detour_side) ? "left" : "right");
    }

    if (!g_b_scan_done)
    {
        car_obstacle_profile_t profile  = { 0 };
        car_avoid_action_t     action   = CAR_AVOID_STOP;
        uint16_t               range_mm = 0u;

        (void)scan_coarse(&profile);

        if (profile.b_is_valid)
        {
            int16_t  centre = (int16_t)((int16_t)CAR_STRAIGHT_AHEAD_DEG
                                        + profile.bearing_deg);
            int16_t  half   = (int16_t)SCAN_FINE_HALF_SPAN_DEG;
            int16_t  start  = (int16_t)(centre - half);
            int16_t  end    = (int16_t)(centre + half);

            if (start < 0)
            {
                start = 0;
            }

            if (end > (int16_t)SERVO_TRAVEL_DEG)
            {
                end = (int16_t)SERVO_TRAVEL_DEG;
            }

            (void)scan_fine((uint16_t)start, (uint16_t)end, &profile);
        }

        /* Leave the servo looking ahead for the forward ping later. */
        (void)scan_measure(CAR_STRAIGHT_AHEAD_DEG, &range_mm);

        if (E_OK == tk_loc_mtx(g_telemetry_mutex, TMO_FEVR))
        {
            g_telemetry.last_obstacle = profile;
            (void)tk_unl_mtx(g_telemetry_mutex);
        }

        (void)scan_plan_avoidance(&profile, &action);

        /* The same obstacle coming back means the last detour did not clear
         * it, so repeating it will not either. Back off instead. */
        if (((CAR_AVOID_LEFT == action) || (CAR_AVOID_RIGHT == action))
            && (g_detour_attempts >= CAR_MAX_DETOUR_ATTEMPTS))
        {
            CAR_LOG(CAR_LOG_ERROR,
                    "detour %u did not clear it, reversing instead\n",
                    g_detour_attempts);
            action = CAR_AVOID_REVERSE;
        }

        CAR_LOG(CAR_LOG_INFO, "obstacle bearing %d range %u action %d\n",
                profile.bearing_deg, profile.closest_range_mm, action);

        switch (action)
        {
            case CAR_AVOID_LEFT:
            case CAR_AVOID_RIGHT:
                plan_detour(&profile);
                g_detour_attempts++;
                g_detour_side  = action;
                g_detour_index = 0u;
                g_b_scan_done  = true;
            break;

            case CAR_AVOID_REVERSE:
                if (g_detour_attempts < CAR_MAX_DETOUR_ATTEMPTS)
                {
                    /* Nothing measured clear, but the sonar only sees the
                     * mouth of each lane. Back off and drive into one to
                     * find out, alternating sides each time round. */
                    g_detour_attempts++;
                    g_detour_side = g_probe_side;
                    g_probe_side  = other_side(g_probe_side);
                    g_b_reversing = true;
                    (void)motion_move_backward(SCAN_REVERSE_MM);
                }
                else
                {
                    CAR_LOG(CAR_LOG_ERROR, "both sides blocked, halting\n");
                    enter_state(CAR_STATE_HALTED);
                }
            break;

            case CAR_AVOID_CONTINUE:
                if (g_b_off_line)
                {
                    /* The way is clear but the detour already left the
                     * line, so search rather than pretending it is under
                     * the sensors. */
                    (void)scan_recover_start(CAR_AVOID_RIGHT == g_detour_side);
                    enter_state(CAR_STATE_RECOVER_LINE);
                }
                else
                {
                    enter_state(CAR_STATE_FOLLOW_LINE);
                }
            break;

            case CAR_AVOID_STOP:
            default:
                enter_state(CAR_STATE_HALTED);
            break;
        }
    }
    else if (g_detour_index < CAR_DETOUR_STEPS)
    {
        detour_step_t const * p_step = &g_detour[g_detour_index];

        if ((g_detour_index >= CAR_DETOUR_PAST_INDEX) && g_b_line_seen)
        {
            /* Round the obstacle and back over the line already. Driving
             * the rest of the box would only leave it again, and the line
             * is what the mission is actually about. */
            CAR_LOG(CAR_LOG_INFO, "line back at leg %u, detour done\n",
                    g_detour_index);
            enter_state(CAR_STATE_FOLLOW_LINE);
        }
        /* Only the driving legs can run into anything; a turn on the spot
         * cannot, and pinging before one would just cost 60 ms. */
        else if (!p_step->b_turn && path_blocked())
        {
            /* This lane is blocked too. Undo the legs driven so far so the
             * other side is tried from the same place, not from wherever
             * this attempt happened to end. */
            CAR_LOG(CAR_LOG_INFO, "leg %u blocked, backing out\n",
                    g_detour_index);
            g_b_retracing   = true;
            g_retrace_index = g_detour_index;
        }
        else
        {
            issue_detour_step(p_step);
            g_detour_index++;
            g_b_off_line = true;
        }
    }
    else
    {
        /* Back on the original heading, offset toward the detour side, so
         * the line lies on the other side. */
        (void)scan_recover_start(CAR_AVOID_RIGHT == g_detour_side);
        enter_state(CAR_STATE_RECOVER_LINE);
    }
}

/**
 * @brief Execute the search pattern until the line sensors report a hit.
 */
static void state_recover_line (void)
{
    if (g_b_obstacle)
    {
        /* Searching for the line is no reason to drive into something. */
        CAR_LOG(CAR_LOG_INFO, "obstacle during the search, avoiding\n");
        (void)motion_stop();
        enter_state(CAR_STATE_AVOID_OBSTACLE);
    }
    else if (!motion_is_busy())
    {
        car_status_t status = CAR_ERR_NO_DATA;

        (void)scan_recover_report(g_b_line_seen);
        status = scan_recover_line();

        if (CAR_OK == status)
        {
            enter_state(CAR_STATE_FOLLOW_LINE);
        }
        else if (CAR_ERR_TIMEOUT == status)
        {
            CAR_LOG(CAR_LOG_ERROR, "line not found\n");
            enter_state(CAR_STATE_HALTED);
        }
        else
        {
            car_avoid_action_t action = CAR_AVOID_STOP;
            uint16_t           amount = 0u;

            (void)scan_recover_get_step(&action, &amount);

            if (CAR_AVOID_LEFT == action)
            {
                (void)motion_turn_left(amount);
            }
            else if (CAR_AVOID_RIGHT == action)
            {
                (void)motion_turn_right(amount);
            }
            else if (CAR_AVOID_CONTINUE == action)
            {
                (void)motion_move_forward(amount);
            }
            else
            {
                /* Nothing to do this step. */
            }
        }
    }
}

/**
 * @brief Stop, let the chassis settle, back straight off, then look.
 *
 * NOTE on why it backs off blind. The accelerometer says a hit happened
 * and roughly how hard, but nothing says where: the sonar was pointing
 * wherever it was pointing, and the line sensors look at the floor. So
 * the only safe move is straight back along the path the car just drove,
 * which is the one direction known to have been clear a moment ago.
 * Scanning happens afterwards, from far enough away for the sonar's
 * minimum range to be out of the way.
 */
static void state_collision (void)
{
    if (g_collisions > CAR_MAX_COLLISIONS)
    {
        CAR_LOG(CAR_LOG_ERROR, "hit %u times, halting\n", g_collisions);
        enter_state(CAR_STATE_HALTED);
    }
    else if ((car_hw_msec() - g_collision_msec) < CAR_COLLISION_SETTLE_MSEC)
    {
        /* Held still while the impact rings out of the accelerometer.
         * Reading anything during this is reading the crash, not the
         * world. */
        (void)motion_stop();
    }
    else if (!g_b_collision_done)
    {
        g_b_collision_done = true;
        (void)motion_move_backward(CAR_COLLISION_BACKOFF_MM);
    }
    else if (!motion_is_busy())
    {
        /* Far enough back to see. Hand over to avoidance, which scans and
         * decides, and which will search for the line afterwards. */
        enter_state(CAR_STATE_AVOID_OBSTACLE);
    }
    else
    {
        /* Still backing off. */
    }
}

/**
 * @brief One tick of the line following law.
 *
 * Proportional plus derivative on the sensor error while the line is seen.
 * When it is lost, drive straight for the gap and barcode distance, then
 * lean toward the side it was last seen on, and finally give up.
 *
 * NOTE: The speed only rises to the follow speed after the line has been
 * seen for CAR_LINE_SEEN_SAMPLES in a row. Inside a barcode the centre
 * sensor flips every bar, and holding the reading speed through the whole
 * symbol keeps every element's width on the same scale for the decoder.
 *
 * @return false once the line has been lost for longer than the limit.
 */
static bool steer_along_line (void)
{
    bool     b_following = true;
    int32_t  steer       = 0;
    uint32_t distance    = distance_now();

    if (g_b_line_seen)
    {
        int16_t error = g_line_error;

        steer = (CAR_STEER_KP_PERMILLE * error)
                + (CAR_STEER_KD_PERMILLE * (error - g_prev_error));
        g_prev_error    = error;
        g_lost_since_mm = distance;

        if (0 != error)
        {
            g_last_side = error;
        }

        if (g_seen_samples < CAR_LINE_SEEN_SAMPLES)
        {
            g_seen_samples++;
        }
        else
        {
            (void)motion_set_speed(
                terrain_speed(CAR_FOLLOW_SPEED_MM_PER_SEC));
        }
    }
    else
    {
        uint32_t lost_mm = distance - g_lost_since_mm;

        g_seen_samples = 0u;
        (void)motion_set_speed(
            terrain_speed(CAR_BARCODE_SPEED_MM_PER_SEC));

        if (lost_mm < CAR_LINE_LOST_STRAIGHT_MM)
        {
            /* The gap and the barcode: hold course, slow enough to read. */
            steer = 0;
        }
        else if (lost_mm < CAR_LINE_LOST_LIMIT_MM)
        {
            steer = (g_last_side < 0) ? -CAR_STEER_LOST_PERMILLE
                                      : CAR_STEER_LOST_PERMILLE;
        }
        else
        {
#if CAR_SKIP_LINE_RECOVERY
            steer = 0;                  /* Hold course, never give up. */
#else
            b_following = false;
#endif
        }
    }

    if (b_following)
    {
        (void)motion_drive_steer((int16_t)steer);
    }

    return b_following;
}

/**
 * @brief Ping straight ahead at most once per sonar cycle.
 *
 * @return true if something is inside SCAN_OBSTACLE_RANGE_MM.
 */
static bool obstacle_ahead (void)
{
    bool b_blocked = false;

    if (g_b_has_scan)
    {
        uint32_t now = car_hw_msec();

        if ((now - g_last_sonar_msec) >= CAR_SONAR_CHECK_PERIOD_MSEC)
        {
            g_last_sonar_msec = now;

#if SCAN_SWEEP_WHILE_MOVING
            /* Centre, right, centre, left, and any of them being close
             * enough stops the car. A side return is not in the path the
             * way a centre one is, but the fine scan that follows decides
             * that properly and answers CONTINUE when there is really
             * nothing in the way. An unnecessary stop is the cheap error
             * here; driving into the corner of something is not. */
            if (0u == (g_sweep_index & 1u))
            {
                b_blocked = path_blocked();
            }
            else
            {
                uint16_t range_mm = SONAR_MAX_RANGE_MM;
                uint16_t angle    = (1u == g_sweep_index)
                                    ? SCAN_MIN_ANGLE_DEG
                                    : SCAN_MAX_ANGLE_DEG;

                if (CAR_OK == scan_measure(angle, &range_mm))
                {
                    if (1u == g_sweep_index)
                    {
                        g_range_right_mm = range_mm;
                    }
                    else
                    {
                        g_range_left_mm = range_mm;
                    }

                    if (range_mm < SCAN_OBSTACLE_RANGE_MM)
                    {
                        CAR_LOG(CAR_LOG_INFO,
                                "obstacle at %u mm, %u deg off centre\n",
                                range_mm, angle);
                        b_blocked = true;
                    }
                }
            }

            g_sweep_index = (uint8_t)((g_sweep_index + 1u) & 3u);
#else
            b_blocked = path_blocked();
#endif
        }
    }

    return b_blocked;
}

/**
 * @brief One forward ranging, no rate limit, no state.
 *
 * NOTE: Costs SONAR_MIN_CYCLE_MSEC standing still, which is why
 * obstacle_ahead() rations it while following the line. The detour calls it
 * directly instead, because a leg about to be driven blind is worth the wait.
 *
 * @return true if something is inside SCAN_OBSTACLE_RANGE_MM ahead.
 */
static bool path_blocked (void)
{
    uint16_t range_mm  = SONAR_MAX_RANGE_MM;
    bool     b_blocked = false;

    if (g_b_has_scan
        && (CAR_OK == scan_measure(CAR_STRAIGHT_AHEAD_DEG, &range_mm))
        && (range_mm < SCAN_OBSTACLE_RANGE_MM))
    {
        CAR_LOG(CAR_LOG_INFO, "obstacle at %u mm\n", range_mm);
        b_blocked = true;
    }

    return b_blocked;
}

/**
 * @brief Start one leg of the detour.
 *
 * @param[in] p_step Leg to run, with turns resolved against g_detour_side.
 */
static void issue_detour_step (detour_step_t const * p_step)
{
    if (p_step->b_turn)
    {
        bool b_left = (CAR_AVOID_LEFT == g_detour_side);

        if (!p_step->b_away)
        {
            b_left = !b_left;
        }

        if (b_left)
        {
            (void)motion_turn_left(p_step->amount);
        }
        else
        {
            (void)motion_turn_right(p_step->amount);
        }
    }
    else
    {
        (void)motion_move_forward(p_step->amount);
    }
}

/**
 * @brief Size the detour legs from the obstacle the fine scan measured.
 *
 * A fixed box either clips a wide obstacle or wastes floor on a narrow
 * one. The sideways leg has to clear half the obstacle plus the car's own
 * half width plus a margin, and the car covers that diagonally at
 * SCAN_DETOUR_TURN_DEG, so the straight line distance is divided by
 * sin 45.
 *
 * NOTE: Nothing measures how deep an obstacle is from the front, only how
 * wide it looks. Depth is therefore the width plus a margin, which is a
 * guess that holds for boxes and fails for walls. The mid detour ping is
 * what catches the failure.
 *
 * @param[in] p_profile Result of the fine scan. A zero width falls back
 *                      to the fixed legs.
 */
static void plan_detour (car_obstacle_profile_t const * p_profile)
{
    uint16_t side_mm  = SCAN_DETOUR_SIDE_MM;
    uint16_t depth_mm = SCAN_DETOUR_DEPTH_MM;

    if ((NULL != p_profile) && (0u != p_profile->width_mm))
    {
        uint32_t clear = ((uint32_t)p_profile->width_mm / 2u)
                         + SCAN_DETOUR_MARGIN_MM;

        side_mm  = clamp_mm((clear * SCAN_DETOUR_SIN45_RECIP) / 1000u,
                            SCAN_DETOUR_SIDE_MIN_MM,
                            SCAN_DETOUR_SIDE_MAX_MM);
        /* Width stands in for depth, because the sonar cannot see how far
         * back an obstacle goes. CAR_LENGTH_MM is on top of it: the back
         * of the car is still beside the obstacle when the bumper is past
         * it, and turning in there clips it. */
        depth_mm = clamp_mm((uint32_t)p_profile->width_mm + CAR_LENGTH_MM
                            + SCAN_DETOUR_MARGIN_MM,
                            SCAN_DETOUR_DEPTH_MIN_MM,
                            SCAN_DETOUR_DEPTH_MAX_MM);
    }

    g_detour[1].amount = side_mm;
    g_detour[3].amount = depth_mm;
    g_detour[5].amount = side_mm;

    CAR_LOG(CAR_LOG_INFO, "detour sized for %u mm wide: side %u depth %u\n",
            (NULL != p_profile) ? p_profile->width_mm : 0u, side_mm,
            depth_mm);
}

/**
 * @brief Hold a value inside a range.
 *
 * @param[in] value What to clamp.
 * @param[in] low   Lowest allowed.
 * @param[in] high  Highest allowed.
 *
 * @return The clamped value.
 */
static uint16_t clamp_mm (uint32_t value, uint16_t low, uint16_t high)
{
    uint32_t held = value;

    if (held < (uint32_t)low)
    {
        held = (uint32_t)low;
    }
    else if (held > (uint32_t)high)
    {
        held = (uint32_t)high;
    }
    else
    {
        /* Inside the range. */
    }

    return (uint16_t)held;
}

/**
 * @brief Drive one leg backwards, undoing what issue_detour_step() did.
 *
 * A turn is undone by turning the same amount the other way, a drive by
 * driving the same distance in reverse. Walked in reverse order by
 * retrace_step(), this puts the car back where the detour began.
 *
 * @param[in] p_step Leg to undo, resolved against the current side.
 */
static void undo_detour_step (detour_step_t const * p_step)
{
    if (p_step->b_turn)
    {
        bool b_left = (CAR_AVOID_LEFT == g_detour_side);

        if (!p_step->b_away)
        {
            b_left = !b_left;
        }

        /* The other way round from issue_detour_step(). */
        if (b_left)
        {
            (void)motion_turn_right(p_step->amount);
        }
        else
        {
            (void)motion_turn_left(p_step->amount);
        }
    }
    else
    {
        (void)motion_move_backward(p_step->amount);
    }
}

/**
 * @brief Undo one leg, or start down the other side once none are left.
 *
 * NOTE: The attempt counter is shared with the detour proper, so probing
 * left, probing right and going round all draw on the same budget and the
 * car cannot shuffle between lanes forever.
 */
static void retrace_step (void)
{
    if (0u != g_retrace_index)
    {
        g_retrace_index--;
        undo_detour_step(&g_detour[g_retrace_index]);
    }
    else if (g_detour_attempts < CAR_MAX_DETOUR_ATTEMPTS)
    {
        g_b_retracing  = false;
        g_detour_attempts++;
        g_detour_side  = other_side(g_detour_side);
        g_probe_side   = other_side(g_detour_side);
        g_detour_index = 0u;
        CAR_LOG(CAR_LOG_INFO, "back at the start, trying %s\n",
                (CAR_AVOID_LEFT == g_detour_side) ? "left" : "right");
    }
    else
    {
        g_b_retracing = false;
        CAR_LOG(CAR_LOG_ERROR, "no lane free after %u tries, halting\n",
                g_detour_attempts);
        enter_state(CAR_STATE_HALTED);
    }
}

/**
 * @brief The opposite side, for alternating probes.
 *
 * @param[in] side CAR_AVOID_LEFT or CAR_AVOID_RIGHT.
 *
 * @return The other one.
 */
static car_avoid_action_t other_side (car_avoid_action_t side)
{
    return (CAR_AVOID_LEFT == side) ? CAR_AVOID_RIGHT : CAR_AVOID_LEFT;
}

/**
 * @brief Adjust a level ground speed for what the IMU says underfoot.
 *
 * Climbing needs more than level ground or the car stalls on the face of
 * the hump; descending needs less or it runs away and lands hard. Rough
 * ground gets the descent speed whichever way it is pointing, because the
 * wheels are not reliably in contact.
 *
 * NOTE: Pitch is frozen while the accelerometer cannot be believed, and a
 * car climbing a bump is exactly when that happens. The climb is
 * therefore detected from the motion event, which survives the gate,
 * rather than from the pitch angle.
 *
 * @param[in] level_speed What the controller wanted on the flat.
 *
 * @return Speed to command.
 */
static uint16_t terrain_speed (uint16_t level_speed)
{
    static uint16_t    s_reported = 0u;
    uint16_t           speed = level_speed;
    car_motion_event_t event = CAR_MOTION_STATIONARY;

    if (g_b_has_imu && (CAR_OK == imu_get_event(&event)))
    {
        if (CAR_MOTION_CLIMBING == event)
        {
            speed = CAR_CLIMB_SPEED_MM_PER_SEC;
        }
        else if ((CAR_MOTION_DESCENDING == event)
                 || !imu_is_terrain_stable())
        {
            speed = CAR_DESCEND_SPEED_MM_PER_SEC;
        }
        else
        {
            /* Level and stable, so whatever was asked for. */
        }
    }

    /* Without this the override is invisible on a console: a car that
     * never switches and a car with no IMU drive identically. */
    if (speed != s_reported)
    {
        CAR_LOG(CAR_LOG_INFO, "terrain speed %u, event %d, pitch ok %d\n",
                speed, (int)event, imu_is_pitch_trusted());
        s_reported = speed;
    }

    return speed;
}

/**
 * @brief Ground distance travelled so far, from the motion snapshot.
 *
 * @return Millimetres.
 */
static uint32_t distance_now (void)
{
    motion_state_t motion = { 0 };

    (void)motion_get_state(&motion);

    return motion.distance_mm;
}

/**
 * @brief Gather one snapshot from every subsystem and publish it.
 */
static void publish_telemetry (void)
{
    motion_state_t     motion        = { 0 };
    car_hump_t         hump          = { 0 };
    int16_t            pitch_deg     = 0;
    int16_t            heading_deg   = 0;
    int16_t            rate_dps      = 0;
    uint16_t           rough_milli_g = 0u;
    car_motion_event_t event         = CAR_MOTION_STATIONARY;
    bool               b_trusted     = false;
    bool               b_stable      = true;

    (void)motion_get_state(&motion);

    if (g_b_has_imu)
    {
        (void)imu_get_peak_hump(&hump);
        (void)imu_get_orientation(&pitch_deg, &heading_deg);
        (void)imu_get_turn_rate_dps(&rate_dps);
        (void)imu_get_event(&event);
        (void)imu_get_terrain_roughness(&rough_milli_g);
        b_trusted = imu_is_pitch_trusted();
        b_stable  = imu_is_terrain_stable();
    }

    if (E_OK == tk_loc_mtx(g_telemetry_mutex, TMO_FEVR))
    {
        (void)line_get_sensor_mask(&g_telemetry.line_sensor_mask);

        g_telemetry.mission_state       = g_state;
        g_telemetry.speed_mm_per_sec    = motion.speed_mm_per_sec;
        g_telemetry.encoder_count_left  = motion.encoder_count_left;
        g_telemetry.encoder_count_right = motion.encoder_count_right;
        g_telemetry.total_distance_mm   = motion.distance_mm;
        g_telemetry.peak_hump_height_mm = hump.peak_height_mm;
        g_telemetry.pitch_deg             = pitch_deg;
        g_telemetry.b_pitch_trusted       = b_trusted;
        g_telemetry.heading_deg           = heading_deg;
        g_telemetry.turn_rate_dps         = rate_dps;
        g_telemetry.motion_event          = event;
        g_telemetry.b_terrain_stable      = b_stable;
        g_telemetry.terrain_rough_milli_g = rough_milli_g;
        /* Latched by the collision state so a hit still shows in the
         * next message even though reading it in the IMU clears it. */
        g_telemetry.b_collision           = (CAR_STATE_COLLISION == g_state);

        (void)comms_publish_telemetry(&g_telemetry);
        (void)tk_unl_mtx(g_telemetry_mutex);
    }
}

/*** end of file ***/
