#include "state/flight_sm.h"

#include <math.h>

void flight_sm_default_config(flight_sm_config_t *cfg)
{
    cfg->launch_accel_thresh = 20.0f;
    cfg->boost_debounce_n = 3;
    cfg->burnout_accel_thresh = 0.0f;
    cfg->burnout_debounce_n = 3;
    cfg->apogee_vel_thresh = 0.0f;
    cfg->apogee_debounce_n = 5;
    cfg->min_apogee_alt = 10.0f;
    cfg->landed_alt_thresh = 2.0f;
    cfg->landed_vel_thresh = 1.0f;
    cfg->landed_debounce_n = 20;
}

void flight_sm_init(flight_sm_t *sm, const flight_sm_config_t *cfg)
{
    sm->state = FLIGHT_IDLE;
    sm->cfg = *cfg;
    sm->deployed = 0;
    sm->deploy_time = 0.0f;
    sm->max_altitude = 0.0f;
    sm->apogee_time = 0.0f;
    sm->boost_count = 0;
    sm->burnout_count = 0;
    sm->apogee_count = 0;
    sm->landed_count = 0;
    sm->faulted = 0;
}

void flight_sm_arm(flight_sm_t *sm)
{
    if (!sm->faulted && sm->state == FLIGHT_IDLE)
    {
        sm->state = FLIGHT_ARMED;
    }
}

void flight_sm_fault(flight_sm_t *sm)
{
    sm->faulted = 1;
    sm->state = FLIGHT_SAFE;
}

int flight_sm_deploy_fired(const flight_sm_t *sm)
{
    return sm->deployed;
}

flight_state_t flight_sm_update(flight_sm_t *sm,
                                const state_estimate_t *est,
                                float a_vert)
{
    if (sm->faulted)
    {
        return FLIGHT_SAFE;
    }

    if (est->altitude > sm->max_altitude)
    {
        sm->max_altitude = est->altitude;
    }

    switch (sm->state)
    {
        case FLIGHT_IDLE:
            /* Requires an explicit arm command. */
            break;

        case FLIGHT_ARMED:
            if (a_vert >= sm->cfg.launch_accel_thresh)
            {
                if (++sm->boost_count >= sm->cfg.boost_debounce_n)
                {
                    sm->state = FLIGHT_BOOST;
                }
            }
            else
            {
                sm->boost_count = 0;
            }
            break;

        case FLIGHT_BOOST:
            /* Motor burnout: sustained non-positive vertical acceleration. */
            if (a_vert <= sm->cfg.burnout_accel_thresh)
            {
                if (++sm->burnout_count >= sm->cfg.burnout_debounce_n)
                {
                    sm->state = FLIGHT_COAST;
                }
            }
            else
            {
                sm->burnout_count = 0;
            }
            break;

        case FLIGHT_COAST:
            /* Apogee: filtered velocity has fallen to/through zero, with
             * debounce and a minimum-altitude guard against baro noise. */
            if (est->velocity <= sm->cfg.apogee_vel_thresh &&
                est->altitude >= sm->cfg.min_apogee_alt)
            {
                if (++sm->apogee_count >= sm->cfg.apogee_debounce_n)
                {
                    sm->state = FLIGHT_APOGEE;
                    if (!sm->deployed)
                    {
                        sm->deployed = 1;
                        sm->deploy_time = est->t;
                    }
                    sm->apogee_time = est->t;
                }
            }
            else
            {
                sm->apogee_count = 0;
            }
            break;

        case FLIGHT_APOGEE:
            /* Once clearly descending, move to DESCENT. */
            if (est->velocity < -sm->cfg.landed_vel_thresh)
            {
                sm->state = FLIGHT_DESCENT;
            }
            break;

        case FLIGHT_DESCENT:
            if (est->altitude <= sm->cfg.landed_alt_thresh &&
                fabsf(est->velocity) <= sm->cfg.landed_vel_thresh)
            {
                if (++sm->landed_count >= sm->cfg.landed_debounce_n)
                {
                    sm->state = FLIGHT_LANDED;
                }
            }
            else
            {
                sm->landed_count = 0;
            }
            break;

        case FLIGHT_LANDED:
        case FLIGHT_SAFE:
        default:
            break;
    }

    return sm->state;
}
