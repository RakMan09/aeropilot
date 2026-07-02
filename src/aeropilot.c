#include "aeropilot.h"

#include <math.h>

const char *flight_state_name(flight_state_t state)
{
    switch (state)
    {
        case FLIGHT_IDLE:    return "IDLE";
        case FLIGHT_ARMED:   return "ARMED";
        case FLIGHT_BOOST:   return "BOOST";
        case FLIGHT_COAST:   return "COAST";
        case FLIGHT_APOGEE:  return "APOGEE";
        case FLIGHT_DESCENT: return "DESCENT";
        case FLIGHT_LANDED:  return "LANDED";
        case FLIGHT_SAFE:    return "SAFE";
        default:             return "UNKNOWN";
    }
}

float baro_pressure_to_altitude(float pressure_pa, float ground_pressure_pa)
{
    /* International barometric formula (troposphere), referenced to the
     * pressure measured on the launch pad so that altitude starts at 0. */
    const float exponent = 1.0f / 5.255f;
    float ground_alt = 44330.0f * (1.0f - powf(ground_pressure_pa /
                                               AEROPILOT_SEA_LEVEL_PRESSURE_PA,
                                               exponent));
    float abs_alt = 44330.0f * (1.0f - powf(pressure_pa /
                                            AEROPILOT_SEA_LEVEL_PRESSURE_PA,
                                            exponent));
    return abs_alt - ground_alt;
}
