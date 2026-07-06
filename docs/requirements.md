# AeroPilot Software Requirements (SRS)

DO-178C-flavored requirements for the AeroPilot flight software. Each
requirement has a stable identifier used in the design and verification
traceability matrices. This is a demonstration project, not a certified
artifact.

## High-level requirements (HLR)

| ID | Requirement |
|----|-------------|
| HLR-1 | The system shall execute deterministic, fixed-priority preemptive real-time tasks on an RTOS. |
| HLR-2 | The system shall estimate altitude, vertical velocity and attitude by fusing IMU and barometer data. |
| HLR-3 | The system shall sequence through the flight phases IDLE, ARMED, BOOST, COAST, APOGEE, DESCENT, LANDED. |
| HLR-4 | The system shall detect apogee and command parachute deployment. |
| HLR-5 | The system shall detect a stalled task and transition the vehicle to a safe state within a bounded time. |
| HLR-6 | The system shall transmit CRC-protected telemetry over a simulated avionics data bus. |
| HLR-7 | A ground station shall decode telemetry, reject corrupted frames, and reconstruct the flight. |

## Low-level requirements (LLR)

### Scheduling / determinism (traces HLR-1)
- SRS-SCHED-1: The watchdog task shall run at the highest application priority.
- SRS-SCHED-2: The sensor task shall execute at a fixed period (10 ms) using an absolute-delay primitive (`vTaskDelayUntil`).
- SRS-SCHED-3: Telemetry shall be scheduled at a higher priority than the fusion and state-machine tasks so that a stall in those tasks does not suppress telemetry.
- SRS-SCHED-4: The realised period of the sensor task shall be measured and reported.

### Sensor simulation (traces HLR-2)
- SRS-SIM-1: The physics model shall integrate thrust, aerodynamic drag and gravity to produce a vertical trajectory.
- SRS-SIM-2: The model shall synthesise IMU (specific force = kinematic accel + gravity) and barometric pressure samples.
- SRS-SIM-3: Noise generation shall be deterministic for a given seed.

### Fusion (traces HLR-2)
- SRS-FUSE-1: A 1-D Kalman filter shall estimate altitude and vertical velocity from accelerometer input and barometric altitude.
- SRS-FUSE-2: A complementary filter shall provide an alternate altitude/velocity estimate.
- SRS-FUSE-3: An attitude complementary filter shall estimate pitch and roll.
- SRS-FUSE-4: The Kalman estimator shall track the reference trajectory with peak altitude error < 15 m under nominal sensor noise.

### Flight state machine (traces HLR-3, HLR-4)
- SRS-SM-1: The state machine shall start in IDLE and require an explicit ARM command to proceed.
- SRS-SM-2: Liftoff (ARMED->BOOST) shall be detected by sustained vertical acceleration above threshold, debounced.
- SRS-SM-3: Burnout (BOOST->COAST) shall be detected by sustained non-positive vertical acceleration, debounced.
- SRS-SM-4: Apogee (COAST->APOGEE) shall be detected by filtered vertical velocity crossing zero, debounced, above a minimum altitude.
- SRS-SM-5: Parachute deploy shall be commanded exactly once, at the apogee transition.
- SRS-SM-6: Landing (DESCENT->LANDED) shall be detected by low altitude and low speed, debounced.

### Fault handling (traces HLR-5)
- SRS-FAULT-1: The watchdog shall declare a task stalled after it fails to advance its check-in counter for a bounded number of consecutive periods.
- SRS-FAULT-2: On a stall, the system shall transition to SAFE, inhibit further deploy commands, and hold last-good state.
- SRS-FAULT-3: The SAFE condition shall be reported in telemetry.
- SRS-FAULT-4: SAFE shall latch (no recovery back into the nominal sequence).

### Protocol / bus (traces HLR-6, HLR-7)
- SRS-PROTO-1: Telemetry shall be serialised into a fixed-size 18-byte frame with a sync word and a CRC-16/CCITT.
- SRS-PROTO-2: Frames shall be transported as ARINC-429 words (two payload bytes per word) with odd parity and a telemetry label.
- SRS-PROTO-3: The decoder shall reject any word failing parity/label and any frame failing sync/CRC.
- SRS-PROTO-4: The ground station shall re-synchronise on the sync word after corruption.
