/* Shared data model for AeroPilot: sensor samples, state estimates and the
 * flight state machine enumeration. These types are pure C11 with no RTOS
 * dependency so they compile into both the firmware and the host tests.
 */
#ifndef AEROPILOT_H
#define AEROPILOT_H

#include <stdint.h>

/* Sea-level reference used by the barometric altitude model. */
#define AEROPILOT_SEA_LEVEL_PRESSURE_PA 101325.0f
#define AEROPILOT_GRAVITY_MS2           9.80665f

/* A raw sensor sample as produced by the physics simulation / sensor task.
 * Accelerations are in m/s^2 (body frame), gyro rates in rad/s, pressure in
 * pascals. */
typedef struct
{
    float t;        /* Sample timestamp in seconds. */
    float ax;       /* Body X acceleration (m/s^2). */
    float ay;       /* Body Y acceleration (m/s^2). */
    float az;       /* Body Z acceleration, +up along the airframe (m/s^2). */
    float gx;       /* Roll rate (rad/s). */
    float gy;       /* Pitch rate (rad/s). */
    float gz;       /* Yaw rate (rad/s). */
    float pressure; /* Static pressure (Pa). */
} sensor_sample_t;

/* The fused vehicle state estimate. */
typedef struct
{
    float t;        /* Estimate timestamp in seconds. */
    float altitude; /* Estimated altitude above launch (m). */
    float velocity; /* Estimated vertical velocity (m/s, +up). */
    float pitch;    /* Estimated pitch angle (rad). */
    float roll;     /* Estimated roll angle (rad). */
} state_estimate_t;

/* Flight state machine states. */
typedef enum
{
    FLIGHT_IDLE = 0,
    FLIGHT_ARMED = 1,
    FLIGHT_BOOST = 2,
    FLIGHT_COAST = 3,
    FLIGHT_APOGEE = 4,
    FLIGHT_DESCENT = 5,
    FLIGHT_LANDED = 6,
    FLIGHT_SAFE = 7 /* Fault state entered by the watchdog. */
} flight_state_t;

const char *flight_state_name(flight_state_t state);

/* Convert a barometric pressure reading to altitude above the launch site,
 * given the pressure recorded on the pad. Uses the international barometric
 * formula. */
float baro_pressure_to_altitude(float pressure_pa, float ground_pressure_pa);

#endif /* AEROPILOT_H */
