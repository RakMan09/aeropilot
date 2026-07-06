# AeroPilot Metrics

Representative results from the reference flight configuration
(`rocket_sim_default_params`), captured on QEMU `mps2-an385` @ 25 MHz,
1 kHz tick.

## Flight profile
- True apogee: ~106.6 m at t ~ 4.67 s.
- State sequence: `IDLE -> ARMED -> BOOST -> COAST -> APOGEE -> DESCENT -> LANDED`.
- Deploy: commanded once at the APOGEE transition.

## Apogee-detection accuracy
- Ground-station observed apogee vs true apogee: **+0.0 m** error on the
  reference run (Kalman fusion, nominal noise).
- Scenario unit test bound: deploy altitude within 15 m and deploy time within
  0.75 s of true apogee (`test_scenario.c`).

## Scheduler determinism
- Sensor task target period: 10 ticks (10 ms).
- Measured period: min = 10, max = 10 ticks; **worst-case jitter = 0 ticks**
  over ~1500 samples per flight (fixed-priority + `vTaskDelayUntil`).
- Note: sub-tick jitter would require the DWT cycle counter, which the QEMU
  `mps2-an385` model does not implement; on real STM32 hardware this counter
  gives nanosecond-resolution jitter figures.

## Fault detection -> safe state
- Watchdog period: 100 ms; trip after 3 consecutive missed check-ins.
- Bounded fault-detection-to-SAFE time: **<= ~300 ms** (3 x watchdog period).
- Demonstrated by the fault-injection build (`-DAEROPILOT_INJECT_HANG`), which
  hangs the fusion task in flight; the watchdog trips to SAFE and the ground
  station decodes the SAFE flag.

## Bus integrity
- Two independent error-detection layers: ARINC-429 odd word parity + frame
  CRC-16/CCITT.
- Under an injected 300-bit corruption of a nominal stream, the ground station
  rejected all corrupted words/frames on parity/CRC while still reconstructing
  the flight from the surviving frames.

## Verification (quantified)
- Host unit/scenario tests: **70 test cases across 11 suites**, all passing
  (individual assertions run into the thousands via exhaustive bit-flip/parity
  loops, e.g. all 4,096 ARINC-429 data words are parity-checked).
- **Code coverage (gcov over the portable core): 96.7% lines, 98.0% functions,
  85.6% branches** (439/454 lines). Reproduce with:
  ```bash
  cmake -B build-cov -DAEROPILOT_BUILD=host_tests -DAEROPILOT_COVERAGE=ON -DCMAKE_C_COMPILER=gcc
  cmake --build build-cov && ctest --test-dir build-cov
  gcovr --root . --filter 'src/' --exclude 'src/app/' --exclude 'src/tasks/' --print-summary
  ```
- Static analysis: cppcheck clean (warning/performance/portability), 0 warnings.
- Firmware C source: ~1,800 lines across `src/`.

## Per-module coverage (gcov)

| Module | Lines | Coverage |
|--------|-------|----------|
| `fusion/kalman.c` | 46 | 100% |
| `fusion/complementary.c` | 34 | 100% |
| `fusion/fusion.c` | 33 | 100% |
| `state/flight_sm.c` | 80 | 100% |
| `proto/crc16.c` | 9 | 100% |
| `proto/frame.c` | 42 | 100% |
| `proto/arinc429.c` | 39 | 100% |
| `proto/bus.c` | 36 | 97% |
| `sim/rocket_sim.c` | 94 | 97% |
| `watchdog/watchdog.c` | 25 | 96% |
| **Total** | **454** | **96.7%** |
