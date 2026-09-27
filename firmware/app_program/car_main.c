/** @file car_main.c
 *
 * @brief Vehicle controller. Creates the tasks and owns mission state.
 *
 * This is the only file that includes all five subsystem headers. Anything
 * one subsystem needs from another flows through here, or through the
 * types in car.h, never by one module including another.
 *
 * NOTE: One task per periodic subsystem. The four control loops run once
 * per kernel tick, 10 ms, woken by car_time_wait_tick(); a kernel delay
 * would run them every 20 ms. Comms paces itself off the clock.
 *
 * NOTE: Every public getter below is called from a task other than the one
 * that updates its module, so each module must make its getters safe: copy
 * the value inside a short DI()/EI() section or under a mutex. The
 * telemetry struct in this file is protected by gh_telemetry_mutex as the
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
#include "car_log.h"
#include "car_time.h"
#include "car.h"
#include "comms.h"
#include "imu.h"
#include "line.h"
#include "motion.h"
#include "scan.h"

/* Priorities: a lower number runs first. Motion outranks everything because
 * a late PID update shows up as a wobble at the wheels. */
#define CAR_PRI_MOTION         5
#define CAR_PRI_IMU            6
#define CAR_PRI_LINE           6
#define CAR_PRI_MISSION        7
#define CAR_PRI_COMMS          8
#define CAR_TASK_STACK_BYTES   4096

/* Every control loop runs once per kernel tick, see car_time.h, so the
 * periods the modules count in must all be that tick. */
#if (MOTION_TICK_PERIOD_MSEC != CAR_TIME_TICK_MSEC)                         \
    || (IMU_SAMPLE_PERIOD_MSEC != CAR_TIME_TICK_MSEC)                       \
    || (LINE_SAMPLE_PERIOD_MSEC != CAR_TIME_TICK_MSEC)                      \
    || (CAR_MISSION_PERIOD_MSEC != CAR_TIME_TICK_MSEC)
#error "A control loop's period differs from the kernel tick"
#endif

#define CAR_TURN_DEG           90u

static car_mission_state_t g_state           = CAR_STATE_INIT;
/* Terrain reads stable until the IMU says otherwise, which with no IMU
 * fitted is never. */
static car_telemetry_t     g_telemetry       = { .b_terrain_stable = true };
static ID                  gh_telemetry_mutex = 0;

/* Which subsystems answered at init. Motion and line are mandatory. */
static bool gb_has_imu   = false;
static bool gb_has_scan  = false;
static bool gb_has_comms = false;

/* Single value stores, one writer each, so no lock. line_task writes the
 * error, the sensor mask, the seen flag and the junction flag; both
 * line_task and the remote command handler write the pending command; the
 * last whole value written wins, by design. */
static volatile int16_t           g_line_error          = 0;
static volatile uint8_t           g_line_mask           = 0u;
static volatile bool              gb_line_seen         = false;
static volatile bool              gb_junction          = false;
static volatile car_nav_command_t g_pending_nav_command = CAR_NAV_NONE;

/* Line following law state, mission task only. */
static int16_t  g_last_side        = 0;    // Last non zero error, to lean to
static uint32_t g_lost_since_mm    = 0u;
static uint32_t g_last_sonar_msec  = 0u;
static uint16_t g_seen_samples     = 0u;

/* Coming back onto the line from off it, mission task only. See
 * reenter_line(). */
typedef enum
{
    REENTRY_OFF = 0,        /* On the line, the normal law steers */
    REENTRY_SEEK,           /* Off it, heading for it */
    REENTRY_CROSS,          /* One sensor touched it, straight on */
    REENTRY_CREEP,          /* Both did, wheels forward over it */
    REENTRY_SPIN            /* On the spot until it sits between them */
} reentry_phase_t;

static reentry_phase_t g_reentry          = REENTRY_OFF;
static uint32_t        g_touch_mm         = 0u;    // Where the first touched
static bool            gb_spin_left       = false; // Toward the second sensor
static bool            gb_last_turn_left  = false; // For a square on arrival

/* Junction handling, mission task only. The active command is the last
 * one read since the previous junction; the junction acts on it. */
typedef enum
{
    JUNCTION_WAIT = 0,      /* Stopped, waiting for a command to arrive */
    JUNCTION_CREEP,         /* Moving the wheels over the crossing */
    JUNCTION_TURN           /* Turning onto the new branch */
} junction_phase_t;

static car_nav_command_t g_active_command    = CAR_NAV_NONE;
static junction_phase_t  g_junction_phase    = JUNCTION_WAIT;
static uint32_t          g_junction_msec     = 0u;
/* Cleared on stopping at a junction and set again once both sensors have
 * left it, so the car does not stop twice at the same crossing. */
static bool              gb_junction_armed   = true;

/* Obstacle avoidance state, mission task only. */
static bool               gb_scan_done       = false;
static bool               gb_reversing       = false;
static uint8_t            g_detour_index     = 0u;
static car_avoid_action_t g_detour_side      = CAR_AVOID_LEFT;

/* Backing out of a lane that turned out to be blocked. The legs already
 * driven are undone in reverse order, which puts the car back where it
 * started so the other side can be tried from the same place. */
static bool               gb_retracing       = false;
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
static bool               gb_obstacle        = false;

/* Coarse sweep while driving: which of centre, right, centre, left the
 * next ranging looks at. */
static uint8_t            g_sweep_index      = 0u;

/* Collision handling. A hit outranks every other state, so it is checked
 * outside the switch and can interrupt a detour leg, a turn or a search
 * mid move. The count is never cleared: three hits in one run means the
 * car is somewhere it cannot drive, and reversing again will not help. */
static uint8_t            g_collisions       = 0u;
static uint32_t           g_collision_msec   = 0u;
static bool               gb_collision_done  = false;

/* What the car is doing right now. Worked out by the mission task every
 * tick, read by the comms task for telemetry. */
static volatile car_action_t g_action        = CAR_ACTION_STARTING;

/* Set by the motion task when a driven wheel has stopped turning for
 * MOTION_STALL_FAULT_MSEC: the car is pushing against something, perhaps
 * too close or too low for the sonar to see. Handled as a hit. */
static volatile bool      gb_stuck           = false;

/* True once a detour leg has moved the car off the line. Every way out of
 * avoidance then has to go through the search, because the brief allows the
 * bypass to leave the line but requires the line to be reacquired after it.
 * Cleared on entering avoidance, so a plain reverse never sets it. */
static bool               gb_off_line        = false;

static void motion_task (INT stacd, void * p_exinf);
static void imu_task (INT stacd, void * p_exinf);
static void line_task (INT stacd, void * p_exinf);
static void mission_task (INT stacd, void * p_exinf);
static void comms_task (INT stacd, void * p_exinf);
static bool start_task (T_CTSK const * p_ctsk, char const * p_name);
static bool bring_up (char const * p_name, car_status_t status,
                      bool b_mandatory, bool * p_present);
static void handle_remote_command (car_nav_command_t command);
static car_nav_command_t take_command (void);
static void run_state_machine (void);
static void enter_state (car_mission_state_t state);
static void state_follow_line (void);
static void state_decode_barcode (void);
static void state_execute_turn (void);
static void leave_junction (void);
static void state_avoid_obstacle (void);
static void watch_detour_leg (void);
static void scan_and_decide (void);
static void fine_scan_around (car_obstacle_profile_t * p_profile);
static void act_on_plan (car_avoid_action_t action,
                         car_obstacle_profile_t const * p_profile);
static void next_detour_leg (void);
static void state_recover_line (void);
static void take_step (car_avoid_action_t action, uint16_t amount);
static void state_collision (void);
static bool steer_along_line (void);
static bool reenter_line (uint32_t distance);
static void first_touch (uint8_t line, uint32_t distance);
static int32_t steer_on_line (uint32_t distance);
static bool steer_off_line (uint32_t distance, int32_t * p_steer);
static bool obstacle_ahead (void);
#if SCAN_SWEEP_WHILE_MOVING
static bool side_blocked (void);
#endif
static bool path_blocked (void);
static uint16_t terrain_speed (uint16_t level_speed);
static void retrace_step (void);
static car_avoid_action_t other_side (car_avoid_action_t side);
static uint32_t distance_now (void);
static void publish_telemetry (void);
static void publish_terrain (void);
static car_action_t current_action (void);

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
    gh_telemetry_mutex = tk_cre_mtx(&cmtx);
    b_ok = (0 < gh_telemetry_mutex);

    b_ok = (bring_up("motion", motion_init(), true, &b_present)) && b_ok;
    b_ok = (bring_up("line", line_init(), true, &b_present)) && b_ok;
    b_ok = (bring_up("imu", imu_init(), false, &gb_has_imu)) && b_ok;
    b_ok = (bring_up("scan", scan_init(), false, &gb_has_scan)) && b_ok;
    b_ok = (bring_up("comms", comms_init(), false, &gb_has_comms)) && b_ok;

    if (b_ok)
    {
        if (CAR_OK != line_calibrate())
        {
            uint8_t mask = 0u;

            (void)line_get_sensor_mask(&mask);
            CAR_LOG(CAR_LOG_ERROR, "line sensors %x read dark at the start "
                    "(1 left, 2 barcode, 4 right): start with the line "
                    "between left and right, or set that trimpot until "
                    "its LED is steady over floor\n", mask);
        }

        if (gb_has_imu && (CAR_OK != imu_calibrate()))
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

    if (gb_has_imu)
    {
        (void)start_task(&g_ctsk_imu, "imu");
    }

    if (gb_has_comms)
    {
        (void)start_task(&g_ctsk_comms, "comms");
    }

    /* The initial task must never return: the kernel shuts down if it does. */
    (void)tk_slp_tsk(TMO_FEVR);

    return 0;
}

/**
 * @brief Run the PID once per kernel tick.
 */
static void motion_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;
    (void)car_time_start_ticks();

    for (;;)
    {
        if (CAR_ERR_HARDWARE == motion_tick())
        {
            gb_stuck = true;
        }

        car_time_wait_tick();
    }
}

/**
 * @brief Feed the IMU the wheel data, then sample and filter it.
 */
static void imu_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;
    (void)car_time_start_ticks();

    for (;;)
    {
        motion_state_t motion = { 0 };

        if (CAR_OK == motion_get_state(&motion))
        {
            (void)imu_feed_odometry(motion.distance_mm, motion.left_mm_per_sec,
                                    motion.right_mm_per_sec);
        }

        (void)imu_update();
        car_time_wait_tick();
    }
}

/**
 * @brief Own every read of the line sensors, so the module has one caller.
 */
static void line_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;
    (void)car_time_start_ticks();

    for (;;)
    {
        int16_t           error   = 0;
        uint8_t           mask    = 0u;
        car_nav_command_t command = CAR_NAV_NONE;
        car_status_t      status  = line_get_position(&error);

        if (CAR_OK == status)
        {
            g_line_error = error;
        }

        (void)line_get_sensor_mask(&mask);
        g_line_mask  = mask;
        gb_line_seen = (CAR_OK == status);
        gb_junction  = line_is_at_junction();

        if (CAR_OK == line_poll_barcode(&command))
        {
            g_pending_nav_command = command;
        }

        car_time_wait_tick();
    }
}

/**
 * @brief Advance the mission state machine once per kernel tick.
 */
static void mission_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;
    (void)car_time_start_ticks();

    for (;;)
    {
        run_state_machine();
        g_action = current_action();
        car_time_wait_tick();
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
    uint32_t last_terrain_msec   = 0u;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        /* Timed off the clock, not by adding up the delay: comms_poll()
         * takes about as long again as the delay itself, so counting
         * nominal periods ran both publishes at half the rate asked for. */
        uint32_t now = car_time_msec();

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

        if ((now - last_terrain_msec) >= CAR_TERRAIN_PERIOD_MSEC)
        {
            publish_terrain();
            last_terrain_msec = now;
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
    ID   h_task     = tk_cre_tsk(p_ctsk);
    bool b_started = false;

    if (0 < h_task)
    {
        b_started = (E_OK == tk_sta_tsk(h_task, 0));
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
    else if ((CAR_ERR_NOT_IMPLEMENTED == status) && (!b_mandatory))
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
    bool b_own_look = ((CAR_STATE_AVOID_OBSTACLE == g_state)
                       && (!gb_scan_done));

    /* Before anything else, and regardless of what the car was doing.
     * Stopping is the only response that is right whatever was hit, and
     * it has to happen in the same tick the hit is seen. */
    {
        bool b_hit   = gb_has_imu && (imu_is_collision_detected());
        bool b_stuck = gb_stuck;

        gb_stuck = false;

        if ((b_hit || b_stuck) && (CAR_STATE_COLLISION != g_state)
            && (CAR_STATE_HALTED != g_state))
        {
            /* Stuck is treated as a slow hit: stop, back off far enough
             * for the sonar to see what the car was pushing on, and let
             * avoidance deal with it, rather than push on blindly. */
            if (b_stuck)
            {
                CAR_LOG(CAR_LOG_ERROR,
                        "wheels driven but not turning, backing off\n");
            }

            (void)motion_stop();
            enter_state(CAR_STATE_COLLISION);
        }
    }

    if ((!b_own_look) && (CAR_STATE_HALTED != g_state)
        && (CAR_STATE_COLLISION != g_state))
    {
        gb_obstacle = obstacle_ahead();
    }

    switch (g_state)
    {
        case CAR_STATE_INIT:
            /* Both calibrations ran in usermain() before any task existed,
             * so there is nothing left to wait for. */
            (void)motion_set_speed(CAR_FOLLOW_SPEED_MM_PER_SEC);
            enter_state(CAR_STATE_FOLLOW_LINE);
        break;

        case CAR_STATE_FOLLOW_LINE:
            state_follow_line();
        break;

        case CAR_STATE_DECODE_BARCODE:
            state_decode_barcode();
        break;

        case CAR_STATE_EXECUTE_TURN:
            state_execute_turn();
        break;

        case CAR_STATE_AVOID_OBSTACLE:
            state_avoid_obstacle();
        break;

        case CAR_STATE_RECOVER_LINE:
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
    CAR_LOG(CAR_LOG_INFO, "state %d -> %d\n", g_state, state);

    switch (state)
    {
        case CAR_STATE_FOLLOW_LINE:
            /* A detour or a search left the line, so the next touch is a
             * re-entry that may be at a steep angle. A junction turn spins
             * on the crossing and lands on the new branch, and going
             * straight on never left the line. */
            g_reentry         = ((CAR_STATE_AVOID_OBSTACLE == g_state)
                                 || (CAR_STATE_RECOVER_LINE == g_state))
                                ? REENTRY_SEEK : REENTRY_OFF;
            g_lost_since_mm   = distance_now();
            g_follow_start_mm = distance_now();
            g_seen_samples    = 0u;
            (void)motion_set_speed(CAR_BARCODE_SPEED_MM_PER_SEC);
        break;

        case CAR_STATE_EXECUTE_TURN:
            (void)motion_stop();
            g_junction_phase   = JUNCTION_WAIT;
            g_junction_msec    = car_time_msec();
            gb_junction_armed  = false;
        break;

        case CAR_STATE_AVOID_OBSTACLE:
            gb_scan_done       = false;
            gb_reversing       = false;
            gb_off_line        = false;
            gb_retracing       = false;
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
            g_collision_msec   = car_time_msec();
            gb_collision_done  = false;
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

    if (!gb_junction)
    {
        gb_junction_armed = true;
    }

    if (gb_obstacle)
    {
        enter_state(CAR_STATE_AVOID_OBSTACLE);
    }
    else if (CAR_NAV_NONE != g_pending_nav_command)
    {
        enter_state(CAR_STATE_DECODE_BARCODE);
    }
    else if (gb_junction && gb_junction_armed && (REENTRY_OFF == g_reentry))
    {
        /* Not while coming back onto the line: crossing it square on reads
         * as a junction for a moment too. */
        CAR_LOG(CAR_LOG_INFO, "junction, stopping\n");
        enter_state(CAR_STATE_EXECUTE_TURN);
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
 * @brief Take the pending command, log it and record it for telemetry.
 *
 * @return The command, CAR_NAV_NONE if none was pending.
 */
static car_nav_command_t take_command (void)
{
    car_nav_command_t command = g_pending_nav_command;

    g_pending_nav_command = CAR_NAV_NONE;
    CAR_LOG(CAR_LOG_INFO, "command %d\n", command);

    if (E_OK == tk_loc_mtx(gh_telemetry_mutex, TMO_FEVR))
    {
        g_telemetry.last_nav_command = command;
        (void)tk_unl_mtx(gh_telemetry_mutex);
    }

    return command;
}

/**
 * @brief Hold the command for the next junction and carry on following.
 *
 * NOTE: Barcodes sit anywhere along the track, so the command waits for
 * the next cross however far away it is; turning where the barcode was
 * read would leave the line. A later barcode replaces an earlier one.
 */
static void state_decode_barcode (void)
{
    g_active_command = take_command();
    enter_state(CAR_STATE_FOLLOW_LINE);
}

/**
 * @brief Stopped at a junction: wait for a command, then turn or go on.
 *
 * Waits CAR_JUNCTION_WAIT_MSEC with the wheels stopped, taking any command
 * that arrives meanwhile. Left and right then creep CAR_JUNCTION_CREEP_MM so
 * the car spins about the crossing, and turn. A U-turn spins where it
 * stands. Straight, or no command since the last junction, drives on.
 */
static void state_execute_turn (void)
{
    if (JUNCTION_WAIT == g_junction_phase)
    {
        if (CAR_NAV_NONE != g_pending_nav_command)
        {
            g_active_command = take_command();
        }

        if ((car_time_msec() - g_junction_msec) >= CAR_JUNCTION_WAIT_MSEC)
        {
            leave_junction();
        }
    }
    else if (motion_is_busy())
    {
        /* Creeping or turning. */
    }
    else if (JUNCTION_CREEP == g_junction_phase)
    {
        if (CAR_NAV_LEFT == g_active_command)
        {
            (void)motion_turn_left(CAR_TURN_DEG);
        }
        else
        {
            (void)motion_turn_right(CAR_TURN_DEG);
        }

        g_junction_phase = JUNCTION_TURN;
    }
    else
    {
        /* Turned onto the new branch. */
        enter_state(CAR_STATE_FOLLOW_LINE);
        g_active_command = CAR_NAV_NONE;
    }
}

/**
 * @brief The wait at a junction is over: start whatever it leads to.
 */
static void leave_junction (void)
{
    (void)motion_set_speed(CAR_FOLLOW_SPEED_MM_PER_SEC);

    if ((CAR_NAV_LEFT == g_active_command)
        || (CAR_NAV_RIGHT == g_active_command))
    {
        (void)motion_move_forward(CAR_JUNCTION_CREEP_MM);
        g_junction_phase = JUNCTION_CREEP;
    }
    else if (CAR_NAV_UTURN == g_active_command)
    {
        (void)motion_turn_right(CAR_UTURN_DEG);
        g_junction_phase = JUNCTION_TURN;
    }
    else
    {
        CAR_LOG(CAR_LOG_INFO, "no turn for this junction, straight on\n");
        g_active_command = CAR_NAV_NONE;
        enter_state(CAR_STATE_FOLLOW_LINE);
    }
}

/**
 * @brief Profile the obstacle once, then drive the detour step by step.
 */
static void state_avoid_obstacle (void)
{
    if (motion_is_busy())
    {
        watch_detour_leg();
    }
    else if (gb_retracing)
    {
        retrace_step();
    }
    else
    {
        if (gb_reversing)
        {
            /* Back at the origin. Probe the chosen side for real rather
             * than ranging again from 100 mm further back, which only ever
             * repeats the answer that sent the car backwards. */
            gb_reversing   = false;
            gb_scan_done   = true;
            g_detour_index = 0u;
            CAR_LOG(CAR_LOG_INFO, "probing %s\n",
                    (CAR_AVOID_LEFT == g_detour_side) ? "left" : "right");
        }

        if (!gb_scan_done)
        {
            scan_and_decide();
        }
        else if (g_detour_index < SCAN_DETOUR_LEGS)
        {
            next_detour_leg();
        }
        else
        {
            /* Back on the original heading, offset toward the detour side,
             * so the line lies on the other side. */
            (void)scan_recover_start(CAR_AVOID_RIGHT == g_detour_side);
            enter_state(CAR_STATE_RECOVER_LINE);
        }
    }
}

/**
 * @brief While a leg or a reverse runs, watch for what should cut it short.
 *
 * gb_scan_done means a detour leg is running. A reverse is the other busy
 * case, and an obstacle ahead during one is the thing being backed away
 * from, so it is left alone.
 */
static void watch_detour_leg (void)
{
    if (gb_obstacle && gb_scan_done)
    {
        CAR_LOG(CAR_LOG_INFO, "obstacle during leg %u, replanning\n",
                g_detour_index);
        (void)motion_stop();
        gb_scan_done   = false;
        g_detour_index = 0u;
    }
    else if (gb_scan_done && (!gb_retracing)
             && (g_detour_index > CAR_DETOUR_PAST_INDEX) && gb_line_seen)
    {
        /* Past the obstacle and the line came under a sensor partway
         * through a leg: stop on it, rather than finish the leg and leave
         * it behind. */
        CAR_LOG(CAR_LOG_INFO, "line back during leg %u, detour done\n",
                g_detour_index - 1u);
        (void)motion_stop();
        enter_state(CAR_STATE_FOLLOW_LINE);
    }
    else
    {
        /* Leg still running. */
    }
}

/**
 * @brief Profile the obstacle, record it, and act on the avoidance plan.
 */
static void scan_and_decide (void)
{
    car_obstacle_profile_t profile  = { 0 };
    car_avoid_action_t     action   = CAR_AVOID_STOP;
    uint16_t               range_mm = 0u;

    (void)scan_coarse(&profile);

    if (profile.b_is_valid)
    {
        fine_scan_around(&profile);
    }

    /* Leave the servo looking ahead for the forward ping later. */
    (void)scan_measure(SCAN_CENTRE_ANGLE_DEG, &range_mm);

    if (E_OK == tk_loc_mtx(gh_telemetry_mutex, TMO_FEVR))
    {
        g_telemetry.last_obstacle = profile;
        (void)tk_unl_mtx(gh_telemetry_mutex);
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
    act_on_plan(action, &profile);
}

/**
 * @brief Fine scan the arc around the bearing the coarse scan found.
 *
 * @param[in,out] p_profile Coarse result in, fine result out.
 */
static void fine_scan_around (car_obstacle_profile_t * p_profile)
{
    /* Casts: servo angles are 0 to 180 and the bearing within ±90, so
     * every sum here fits an int16_t, and start and end are clamped to
     * 0 to SERVO_TRAVEL_DEG before they go back into a uint16_t. */
    int16_t centre = (int16_t)((int16_t)SCAN_CENTRE_ANGLE_DEG
                               + p_profile->bearing_deg);
    int16_t start  = (int16_t)(centre - (int16_t)SCAN_FINE_HALF_SPAN_DEG);
    int16_t end    = (int16_t)(centre + (int16_t)SCAN_FINE_HALF_SPAN_DEG);

    if (start < 0)
    {
        start = 0;
    }

    /* Casts: as above, all within 0 to 180 by now. */
    if (end > (int16_t)SERVO_TRAVEL_DEG)
    {
        end = (int16_t)SERVO_TRAVEL_DEG;
    }

    (void)scan_fine((uint16_t)start, (uint16_t)end, p_profile); /* 0..180 */
}

/**
 * @brief Start what the avoidance planner chose.
 *
 * @param[in] action    The plan.
 * @param[in] p_profile The obstacle it was made from, to size a detour.
 */
static void act_on_plan (car_avoid_action_t action,
                         car_obstacle_profile_t const * p_profile)
{
    switch (action)
    {
        case CAR_AVOID_LEFT:
        case CAR_AVOID_RIGHT:
            (void)scan_detour_plan(p_profile);
            g_detour_attempts++;
            g_detour_side  = action;
            g_detour_index = 0u;
            gb_scan_done   = true;
        break;

        case CAR_AVOID_REVERSE:
            if (g_detour_attempts < CAR_MAX_DETOUR_ATTEMPTS)
            {
                /* Nothing measured clear, but the sonar only sees the mouth
                 * of each lane. Back off and drive into one to find out,
                 * alternating sides each time round. */
                g_detour_attempts++;
                (void)scan_detour_plan(p_profile);
                g_detour_side = g_probe_side;
                g_probe_side  = other_side(g_probe_side);
                gb_reversing  = true;
                (void)motion_move_backward(SCAN_REVERSE_MM);
            }
            else
            {
                CAR_LOG(CAR_LOG_ERROR, "both sides blocked, halting\n");
                enter_state(CAR_STATE_HALTED);
            }
        break;

        case CAR_AVOID_CONTINUE:
            if (gb_off_line)
            {
                /* The way is clear but the detour already left the line,
                 * so search rather than pretend it is under the sensors. */
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

/**
 * @brief Drive the next leg of the box, unless the detour is already over.
 */
static void next_detour_leg (void)
{
    car_avoid_action_t action = CAR_AVOID_STOP;
    uint16_t           amount = 0u;

    (void)scan_detour_get_leg(g_detour_index, g_detour_side, false, &action,
                              &amount);

    if ((g_detour_index >= CAR_DETOUR_PAST_INDEX) && gb_line_seen)
    {
        /* Round the obstacle and back over the line already. Driving the
         * rest of the box would only leave it again, and the line is what
         * the mission is actually about. */
        CAR_LOG(CAR_LOG_INFO, "line back at leg %u, detour done\n",
                g_detour_index);
        enter_state(CAR_STATE_FOLLOW_LINE);
    }
    /* Only the driving legs can run into anything; a turn on the spot
     * cannot, and pinging before one would just cost 60 ms. */
    else if ((CAR_AVOID_CONTINUE == action) && (path_blocked()))
    {
        /* This lane is blocked too. Undo the legs driven so far so the
         * other side is tried from the same place, not from wherever this
         * attempt happened to end. */
        CAR_LOG(CAR_LOG_INFO, "leg %u blocked, backing out\n",
                g_detour_index);
        gb_retracing    = true;
        g_retrace_index = g_detour_index;
    }
    else
    {
        take_step(action, amount);
        g_detour_index++;
        gb_off_line = true;
    }
}

/**
 * @brief Execute the search pattern until the line sensors report a hit.
 */
static void state_recover_line (void)
{
    if (gb_obstacle)
    {
        /* Searching for the line is no reason to drive into something. */
        CAR_LOG(CAR_LOG_INFO, "obstacle during the search, avoiding\n");
        (void)motion_stop();
        enter_state(CAR_STATE_AVOID_OBSTACLE);
    }
    else if ((motion_is_busy()) && gb_line_seen)
    {
        /* The line passed under a sensor partway through a search step.
         * Stop on it now; finishing the step would sweep straight past. */
        CAR_LOG(CAR_LOG_INFO, "line found mid step\n");
        (void)motion_stop();
        enter_state(CAR_STATE_FOLLOW_LINE);
    }
    else if (!motion_is_busy())
    {
        car_status_t status = CAR_ERR_NO_DATA;

        (void)scan_recover_report(gb_line_seen);
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
            take_step(action, amount);
        }
    }
    else
    {
        /* A search step is still running. */
    }
}

/**
 * @brief Start one move the scanning module handed out.
 *
 * @param[in] action CAR_AVOID_LEFT or CAR_AVOID_RIGHT to turn,
 *                   CAR_AVOID_CONTINUE to drive forward, CAR_AVOID_REVERSE
 *                   to drive back. Anything else does nothing.
 * @param[in] amount Degrees for a turn, mm for a drive.
 */
static void take_step (car_avoid_action_t action, uint16_t amount)
{
    if (CAR_AVOID_LEFT == action)
    {
        gb_last_turn_left = true;
        (void)motion_turn_left(amount);
    }
    else if (CAR_AVOID_RIGHT == action)
    {
        gb_last_turn_left = false;
        (void)motion_turn_right(amount);
    }
    else if (CAR_AVOID_CONTINUE == action)
    {
        (void)motion_move_forward(amount);
    }
    else if (CAR_AVOID_REVERSE == action)
    {
        (void)motion_move_backward(amount);
    }
    else
    {
        /* Nothing to do this step. */
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
    else if ((car_time_msec() - g_collision_msec) < CAR_COLLISION_SETTLE_MSEC)
    {
        /* Held still while the impact rings out of the accelerometer.
         * Reading anything during this is reading the crash, not the
         * world. */
        (void)motion_stop();
    }
    else if (!gb_collision_done)
    {
        gb_collision_done = true;
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
 * Proportional on the sensor error while the line is seen. When it is
 * lost, drive straight for CAR_LINE_LOST_STRAIGHT_MM, then lean toward the
 * side it was last seen on, and finally give up. Coming back onto the line
 * from off it is reenter_line()'s job.
 *
 * NOTE: The speed only rises to the follow speed after the line has been
 * seen for CAR_LINE_SEEN_SAMPLES in a row, so a car that has just found
 * the line, or keeps losing it, stays slow until it has settled on it.
 *
 * @return false once the line has been lost for longer than the limit.
 */
static bool steer_along_line (void)
{
    bool     b_following = true;
    int32_t  steer       = 0;
    uint32_t distance    = distance_now();

    if (reenter_line(distance))
    {
        /* The re-entry drives the car this tick. */
    }
    else if (gb_line_seen)
    {
        steer = steer_on_line(distance);
        /* Cast: KP times an error of at most 2, far inside int16_t. */
        (void)motion_drive_steer((int16_t)steer);
    }
    else
    {
        b_following = steer_off_line(distance, &steer);

        if (b_following)
        {
            /* Cast: the lean is at most 1000 either way. */
            (void)motion_drive_steer((int16_t)steer);
        }
    }

    return b_following;
}

/**
 * @brief One tick of coming back onto the line from off it.
 *
 * NOTE on why the normal law cannot do this. The sensors straddle the
 * line, so they only work while the car runs roughly along it. Arriving at
 * an angle, the first sensor to touch the line is the one on the side the
 * car is heading, and steering toward it turns the car further across.
 * Steering hard the other way fails too, because the sensors sit ahead of
 * the wheels: turning where they are swings them off the line again.
 *
 * So the car does what it does at a junction. It drives straight on until
 * the second sensor touches the line, creeps until its wheels are over the
 * line, and turns on the spot toward the second sensor until that sensor
 * touches the line again. The normal law finishes from there.
 *
 * NOTE on where the spin stops. With the sensors this far ahead of the
 * wheels, the space between them is only about 3 degrees of spin wide, and
 * a spin that has reached full speed coasts further than that after the
 * stop, carrying the line past both sensors. Stopping at the first touch
 * leaves the whole sensor spacing, about 10 degrees, for the coast, and
 * the line ends under one sensor or the other with the car within about
 * 10 degrees of straight.
 *
 * The creep comes from the crossing itself. The two touches are some
 * distance apart along the path, and the wheels, CAR_JUNCTION_CREEP_MM
 * behind the sensors, still have that creep less half the gap to go when
 * the second sensor touches, whatever the angle. TRACK_LINE_WIDTH_MM on
 * top puts them just past the line's centre at the angles this happens
 * at.
 * A gap of twice that or more is an angle under about 5 degrees, which the
 * normal law steers out of on its own.
 *
 * @param[in] distance Ground distance now, mm.
 *
 * @return true while the re-entry is driving the car.
 */
static bool reenter_line (uint32_t distance)
{
    uint32_t const reach   = CAR_JUNCTION_CREEP_MM + TRACK_LINE_WIDTH_MM;
    /* Cast: masked to the two line sensor bits. */
    uint8_t const  line    = (uint8_t)(g_line_mask & LINE_BITS_LINE);
    uint8_t const  second  = gb_spin_left ? LINE_BIT_LEFT : LINE_BIT_RIGHT;
    uint32_t const crossed = distance - g_touch_mm;
    bool           b_busy  = true;

    switch (g_reentry)
    {
        case REENTRY_SEEK:
            if (0u == line)
            {
                b_busy = false;     /* steer_off_line() heads for it */
            }
            else
            {
                first_touch(line, distance);
            }
        break;

        case REENTRY_CROSS:
            if ((0u != (line & second)) && (reach > (crossed / 2u)))
            {
                /* Cast: at most reach, well under 1000 mm. */
                uint16_t creep = (uint16_t)(reach - (crossed / 2u));

                CAR_LOG(CAR_LOG_INFO, "crossed in %u mm, creeping %u mm\n",
                        crossed, creep);
                (void)motion_move_forward(creep);
                g_reentry = REENTRY_CREEP;
            }
            else if ((0u != (line & second)) || (crossed >= (2u * reach)))
            {
                /* Shallow, or it never crossed: the normal law copes. */
                g_reentry = REENTRY_OFF;
                b_busy    = false;
            }
            else
            {
                (void)motion_drive_steer(0);
            }
        break;

        case REENTRY_CREEP:
            if (!motion_is_busy())
            {
                g_reentry = REENTRY_SPIN;

                if (gb_spin_left)
                {
                    (void)motion_turn_left(CAR_REENTRY_SPIN_MAX_DEG);
                }
                else
                {
                    (void)motion_turn_right(CAR_REENTRY_SPIN_MAX_DEG);
                }
            }
        break;

        case REENTRY_SPIN:
            if (0u != (line & second))
            {
                CAR_LOG(CAR_LOG_INFO, "back on the line\n");
                (void)motion_stop();
                g_reentry = REENTRY_OFF;
                b_busy    = false;
            }
            else if (!motion_is_busy())
            {
                /* The whole arc and never met it: off the line again. */
                g_reentry       = REENTRY_SEEK;
                g_lost_since_mm = distance;
                b_busy          = false;
            }
            else
            {
                /* Still turning toward it. */
            }
        break;

        case REENTRY_OFF:
        default:
            b_busy = false;
        break;
    }

    return b_busy;
}

/**
 * @brief The line has come under a sensor: start crossing it.
 *
 * The spin at the end goes away from the sensor that touched first. Both
 * at once is square on: the spin then turns back against the turn that
 * brought the car here.
 *
 * @param[in] line     The line sensor bits that are dark.
 * @param[in] distance Ground distance now, mm.
 */
static void first_touch (uint8_t line, uint32_t distance)
{
    if (LINE_BITS_LINE == line)
    {
        gb_spin_left = !gb_last_turn_left;
    }
    else
    {
        gb_spin_left = (LINE_BIT_RIGHT == line);
    }

    g_touch_mm = distance;
    g_reentry  = REENTRY_CROSS;
    CAR_LOG(CAR_LOG_INFO, "line touched, crossing it\n");
    (void)motion_drive_steer(0);
}

/**
 * @brief Steering while a line sensor sees the line.
 *
 * @param[in] distance Ground distance now, mm.
 *
 * @return Steer, per mille.
 */
static int32_t steer_on_line (uint32_t distance)
{
    int16_t error = g_line_error;

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
        (void)motion_set_speed(terrain_speed(CAR_FOLLOW_SPEED_MM_PER_SEC));
    }

    return CAR_STEER_KP_PERMILLE * error;
}

/**
 * @brief Steering while no line sensor sees the line.
 *
 * @param[in]  distance Ground distance now, mm.
 * @param[out] p_steer  Steer, per mille, written while still following.
 *
 * @return false once the line has been lost for longer than the limit.
 */
static bool steer_off_line (uint32_t distance, int32_t * p_steer)
{
    bool     b_following = true;
    uint32_t lost_mm     = distance - g_lost_since_mm;

    g_seen_samples = 0u;
    (void)motion_set_speed(terrain_speed(CAR_BARCODE_SPEED_MM_PER_SEC));

    if (lost_mm >= CAR_LINE_LOST_LIMIT_MM)
    {
        b_following = false;
    }
    else if (lost_mm < CAR_LINE_LOST_STRAIGHT_MM)
    {
        /* Briefly out of view on a straight: hold course. */
        *p_steer = 0;
    }
    else
    {
        /* Really off the line now, so the next touch is a re-entry. */
        g_reentry         = REENTRY_SEEK;
        gb_last_turn_left = (g_last_side < 0);
        *p_steer          = gb_last_turn_left ? -CAR_STEER_LOST_PERMILLE
                                              : CAR_STEER_LOST_PERMILLE;
    }

    return b_following;
}

/**
 * @brief Ping at most once per sonar cycle.
 *
 * @return true if something is inside SCAN_OBSTACLE_RANGE_MM.
 */
static bool obstacle_ahead (void)
{
    bool     b_blocked = false;
    uint32_t now       = car_time_msec();

    if (gb_has_scan
        && ((now - g_last_sonar_msec) >= CAR_SONAR_CHECK_PERIOD_MSEC))
    {
        g_last_sonar_msec = now;

#if SCAN_SWEEP_WHILE_MOVING
        /* Centre, right, centre, left, and any of them being close enough
         * stops the car. A side return is not in the path the way a centre
         * one is, but the fine scan that follows decides that properly and
         * answers CONTINUE when there is really nothing in the way. An
         * unnecessary stop is the cheap error here; driving into the
         * corner of something is not. */
        b_blocked = (0u == (g_sweep_index & 1u)) ? path_blocked()
                                                 : side_blocked();
        g_sweep_index = (uint8_t)((g_sweep_index + 1u) & 3u);
#else
        b_blocked = path_blocked();
#endif
    }

    return b_blocked;
}

#if SCAN_SWEEP_WHILE_MOVING
/**
 * @brief Range the side the sweep is on.
 *
 * @return true if something is inside SCAN_OBSTACLE_RANGE_MM that side.
 */
static bool side_blocked (void)
{
    bool     b_blocked = false;
    uint16_t range_mm  = SONAR_MAX_RANGE_MM;
    uint16_t angle     = (1u == g_sweep_index) ? SCAN_MIN_ANGLE_DEG
                                               : SCAN_MAX_ANGLE_DEG;

    if ((CAR_OK == scan_measure(angle, &range_mm))
        && (range_mm < SCAN_OBSTACLE_RANGE_MM))
    {
        CAR_LOG(CAR_LOG_INFO, "obstacle at %u mm, %u deg off centre\n",
                range_mm, angle);
        b_blocked = true;
    }

    return b_blocked;
}
#endif

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

    if (gb_has_scan
        && (CAR_OK == scan_measure(SCAN_CENTRE_ANGLE_DEG, &range_mm))
        && (range_mm < SCAN_OBSTACLE_RANGE_MM))
    {
        CAR_LOG(CAR_LOG_INFO, "obstacle at %u mm\n", range_mm);
        b_blocked = true;
    }

    return b_blocked;
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
        car_avoid_action_t action = CAR_AVOID_STOP;
        uint16_t           amount = 0u;

        g_retrace_index--;
        (void)scan_detour_get_leg(g_retrace_index, g_detour_side, true,
                                  &action, &amount);
        take_step(action, amount);
    }
    else if (g_detour_attempts < CAR_MAX_DETOUR_ATTEMPTS)
    {
        gb_retracing   = false;
        g_detour_attempts++;
        g_detour_side  = other_side(g_detour_side);
        g_probe_side   = other_side(g_detour_side);
        g_detour_index = 0u;
        CAR_LOG(CAR_LOG_INFO, "back at the start, trying %s\n",
                (CAR_AVOID_LEFT == g_detour_side) ? "left" : "right");
    }
    else
    {
        gb_retracing = false;
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
 * Climbing keeps the asked speed: the speed loop adds the power the slope
 * needs as it slows the wheels. Descending needs less or the car runs
 * away and lands hard. Rough ground gets the descent speed whichever way
 * it is pointing, because the wheels are not reliably in contact.
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

    if (gb_has_imu && (CAR_OK == imu_get_event(&event)))
    {
        if (CAR_MOTION_CLIMBING == event)
        {
            /* Keep the asked speed, see above. */
        }
        else if ((CAR_MOTION_DESCENDING == event)
                 || (!imu_is_terrain_stable()))
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
                speed, (int32_t)event, imu_is_pitch_trusted());
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
    motion_state_t motion = { 0 };

    (void)motion_get_state(&motion);

    if (E_OK == tk_loc_mtx(gh_telemetry_mutex, TMO_FEVR))
    {
        (void)line_get_sensor_mask(&g_telemetry.line_sensor_mask);
        (void)line_get_last_barcode(&g_telemetry.last_barcode,
                                    &g_telemetry.barcode_count);

        g_telemetry.mission_state       = g_state;
        g_telemetry.speed_mm_per_sec    = motion.speed_mm_per_sec;
        g_telemetry.encoder_count_left  = motion.encoder_count_left;
        g_telemetry.encoder_count_right = motion.encoder_count_right;
        g_telemetry.total_distance_mm   = motion.distance_mm;
        g_telemetry.action              = g_action;
        /* Latched by the collision state so a hit still shows in the
         * next message even though reading it in the IMU clears it. */
        g_telemetry.b_collision         = (CAR_STATE_COLLISION == g_state);

        if (gb_has_imu)
        {
            (void)imu_get_peak_hump(&g_telemetry.peak_hump_height_mm);
            (void)imu_get_orientation(&g_telemetry.pitch_deg,
                                      &g_telemetry.heading_deg);
            (void)imu_get_turn_rate_dps(&g_telemetry.turn_rate_dps);
            (void)imu_get_event(&g_telemetry.motion_event);
            (void)imu_get_terrain_roughness(
                &g_telemetry.terrain_rough_milli_g);
            (void)imu_get_last_hump(&g_telemetry.last_hump_height_mm);
            (void)imu_get_hump_count(&g_telemetry.hump_count);
            (void)imu_get_accel_magnitude(&g_telemetry.accel_milli_g);
            g_telemetry.b_pitch_trusted  = imu_is_pitch_trusted();
            g_telemetry.b_terrain_stable = imu_is_terrain_stable();
            g_telemetry.b_on_hump        = imu_is_hump_detected();
        }

        (void)comms_publish_telemetry(&g_telemetry);
        (void)tk_unl_mtx(gh_telemetry_mutex);
    }
}

/**
 * @brief Publish the terrain status from the latest telemetry snapshot.
 *
 * NOTE: publish_telemetry() refreshes the snapshot every
 * CAR_TELEMETRY_PERIOD_MSEC, so this reads nothing from the IMU itself.
 */
static void publish_terrain (void)
{
    if (E_OK == tk_loc_mtx(gh_telemetry_mutex, TMO_FEVR))
    {
        (void)comms_publish_terrain(&g_telemetry);
        (void)tk_unl_mtx(gh_telemetry_mutex);
    }
}

/**
 * @brief What the car is doing right now, for telemetry.
 *
 * @return The action, from the state and what that state is part way
 *         through.
 */
static car_action_t current_action (void)
{
    car_action_t action = CAR_ACTION_HALTED;

    switch (g_state)
    {
        case CAR_STATE_INIT:
            action = CAR_ACTION_STARTING;
        break;

        case CAR_STATE_FOLLOW_LINE:
            if (REENTRY_OFF != g_reentry)
            {
                action = CAR_ACTION_ACQUIRING;
            }
            else if (gb_line_seen)
            {
                action = CAR_ACTION_FOLLOWING;
            }
            else
            {
                action = CAR_ACTION_LINE_LOST;
            }
        break;

        case CAR_STATE_DECODE_BARCODE:
            action = CAR_ACTION_READING_BARCODE;
        break;

        case CAR_STATE_EXECUTE_TURN:
            if (JUNCTION_WAIT == g_junction_phase)
            {
                action = CAR_ACTION_AT_JUNCTION;
            }
            else if (CAR_NAV_LEFT == g_active_command)
            {
                action = CAR_ACTION_TURNING_LEFT;
            }
            else if (CAR_NAV_RIGHT == g_active_command)
            {
                action = CAR_ACTION_TURNING_RIGHT;
            }
            else
            {
                action = CAR_ACTION_U_TURN;
            }
        break;

        case CAR_STATE_AVOID_OBSTACLE:
            if (gb_reversing)
            {
                action = CAR_ACTION_REVERSING;
            }
            else if (!gb_scan_done)
            {
                action = CAR_ACTION_SCANNING;
            }
            else if (gb_retracing)
            {
                action = CAR_ACTION_BACKING_OUT;
            }
            else
            {
                action = CAR_ACTION_DETOUR;
            }
        break;

        case CAR_STATE_RECOVER_LINE:
            action = CAR_ACTION_SEARCHING;
        break;

        case CAR_STATE_COLLISION:
            action = CAR_ACTION_BACKING_OFF;
        break;

        case CAR_STATE_HALTED:
        default:
            action = CAR_ACTION_HALTED;
        break;
    }

    return action;
}

/*** end of file ***/
