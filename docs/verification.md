# AeroPilot Verification & Traceability

Verification is performed by (a) host-side Unity unit/scenario tests over the
portable core, (b) full-flight integration on QEMU with telemetry decoded by
the ground station, and (c) static analysis with cppcheck. All three run in CI.

## Requirement -> test traceability

| Requirement | Verified by |
|-------------|-------------|
| SRS-SCHED-1..3 | Firmware runs to LANDED on QEMU (`FLIGHT COMPLETE`); watchdog fault-injection run reports `SAFE STATE ENTERED` while telemetry continues |
| SRS-SCHED-4 | Firmware `[metrics]` line (sensor period min/max/jitter over N samples) |
| SRS-SIM-1..3 | `test_sim.c` (climb, apogee bound, pressure vs altitude, determinism) |
| SRS-FUSE-1, SRS-FUSE-4 | `test_kalman.c`, `test_fusion.c` (constant-velocity/accel tracking, apogee accuracy, noise robustness) |
| SRS-FUSE-2 | `test_complementary.c`, `test_fusion.c` |
| SRS-FUSE-3 | `test_complementary.c` (attitude level / gyro integration / tilt) |
| SRS-SM-1 | `test_flight_sm.c::test_starts_idle`, `test_idle_ignores_motion_until_armed` |
| SRS-SM-2 | `test_flight_sm.c::test_boost_requires_debounce` |
| SRS-SM-3 | `test_flight_sm.c::test_reaches_coast` |
| SRS-SM-4 | `test_flight_sm.c::test_apogee_deploys`, `test_no_early_apogee_below_min_altitude` |
| SRS-SM-5 | `test_flight_sm.c::test_apogee_deploys`; `test_scenario.c` (deploy once, within window of true apogee) |
| SRS-SM-6 | `test_flight_sm.c::test_full_sequence_to_landed`; `test_scenario.c` |
| SRS-FAULT-1 | `test_watchdog.c` (trips after max_missed, recovers before trip, lowest-index) |
| SRS-FAULT-2, SRS-FAULT-4 | `test_flight_sm.c::test_fault_latches_safe`, `test_arm_ignored_after_fault` |
| SRS-FAULT-2, SRS-FAULT-3 | QEMU fault-injection run: `SAFE STATE ENTERED`, ground station decodes `SAFE` flag |
| SRS-PROTO-1 | `test_crc16.c`, `test_frame.c` (round trip, sync/CRC rejection, all-bit-flip detection) |
| SRS-PROTO-2 | `test_arinc429.c`, `test_bus.c` (parity, label round trip, payload round trip) |
| SRS-PROTO-3 | `test_frame.c`, `test_arinc429.c`, `test_bus.c`; ground station corruption run |
| SRS-PROTO-4 | Ground station re-sync verified against a bit-corrupted stream |

## Test inventory (host)

| Suite | Focus |
|-------|-------|
| test_crc16 | CRC-16/CCITT check value + error detection |
| test_frame | Frame encode/decode, sync/CRC rejection, exhaustive payload bit-flips |
| test_arinc429 | Word parity, label bit-reversal, field extraction, single-bit-flip detection |
| test_bus | Frame<->word transport round trip, corruption counting |
| test_kalman | Filter convergence, tracking, noise rejection |
| test_complementary | Vertical + attitude complementary filters |
| test_fusion | End-to-end fusion vs reference trajectory, apogee accuracy |
| test_flight_sm | All state transitions, deploy logic, SAFE latching |
| test_watchdog | Stall detection, recovery, priority of report |
| test_sim | Physics model sanity + determinism |
| test_scenario | Full sim->fusion->SM integration; ordered states; deploy window |

## Static analysis

`cppcheck --enable=warning,performance,portability --std=c11 --error-exitcode=2`
runs clean over `src/`. Style/`unusedFunction` classes are excluded because the
core exposes a library API exercised by the host tests rather than by any single
translation unit.

## How to reproduce

See the repository `README.md` (build firmware, run on QEMU, decode telemetry,
run host tests, run cppcheck) or `.github/workflows/ci.yml`.
