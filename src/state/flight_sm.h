/* Flight state machine with apogee detection and parachute-deploy logic.
 *
 * IDLE -> ARMED -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED, plus a SAFE
 * fault state entered on watchdog trip. Pure C11, no RTOS dependency, so it is
 * fully unit-testable on the host. */
#ifndef AEROPILOT_FLIGHT_SM_H
#define AEROPILOT_FLIGHT_SM_H

#include "aeropilot.h"

typedef struct
{
    float launch_accel_thresh; /* Vertical accel for liftoff detect (m/s^2). */
    int   boost_debounce_n;    /* Samples above threshold to confirm boost. */
    float burnout_accel_thresh;/* Vertical accel below which motor burned out.*/
    int   burnout_debounce_n;  /* Samples to confirm burnout. */
    float apogee_vel_thresh;   /* Velocity at/below which we are at apogee. */
    int   apogee_debounce_n;   /* Samples to confirm apogee (anti-noise). */
    float min_apogee_alt;      /* Minimum altitude to allow apogee/deploy (m).*/
    float landed_alt_thresh;   /* Altitude below which we may be landed (m). */
    float landed_vel_thresh;   /* Speed below which we may be landed (m/s). */
    int   landed_debounce_n;   /* Samples to confirm landing. */
} flight_sm_config_t;

typedef struct
{
    flight_state_t state;
    flight_sm_config_t cfg;

    int   deployed;      /* Non-zero once the deploy event has fired. */
    float deploy_time;   /* Sim time of the deploy event (s). */
    float max_altitude;  /* Peak altitude observed by the estimator (m). */
    float apogee_time;   /* Sim time apogee was detected (s). */

    int   boost_count;
    int   burnout_count;
    int   apogee_count;
    int   landed_count;
    int   faulted;       /* Latched once SAFE is entered. */
} flight_sm_t;

/* Fill cfg with defaults tuned for the reference rocket_sim trajectory. */
void flight_sm_default_config(flight_sm_config_t *cfg);

void flight_sm_init(flight_sm_t *sm, const flight_sm_config_t *cfg);

/* Move IDLE -> ARMED. No effect once flight has begun or after a fault. */
void flight_sm_arm(flight_sm_t *sm);

/* Advance the state machine with the latest fused estimate and the current
 * world-frame vertical acceleration. Returns the (possibly new) state. */
flight_state_t flight_sm_update(flight_sm_t *sm,
                                const state_estimate_t *est,
                                float a_vert);

/* Force the SAFE fault state (called by the watchdog). Latches: subsequent
 * updates keep the vehicle in SAFE and inhibit any further deploy. */
void flight_sm_fault(flight_sm_t *sm);

int flight_sm_deploy_fired(const flight_sm_t *sm);

#endif /* AEROPILOT_FLIGHT_SM_H */
