#!/usr/bin/env bash
# One-command AeroPilot demo: build the firmware, fly it on QEMU, and decode
# the telemetry on the ground. Intended for Codespaces / a fresh checkout.
set -euo pipefail
cd "$(dirname "$0")/.."

echo "==> Initialising submodules (FreeRTOS-Kernel, Unity)"
git submodule update --init --recursive >/dev/null 2>&1 || true

echo "==> Building flight firmware (arm-none-eabi)"
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -DAEROPILOT_BUILD=firmware >/dev/null
cmake --build build >/dev/null

echo "==> Flying on QEMU (mps2-an385)"
qemu-system-arm -M mps2-an385 -cpu cortex-m3 -kernel build/aeropilot.elf \
  -display none -monitor none -serial stdio -serial file:telemetry.bin -semihosting

echo
echo "==> Decoding telemetry on the ground station"
python3 groundstation/decode.py --file telemetry.bin --true-apogee 106.6 \
  --plot flight.png --check

echo
echo "Done. See flight.png for the decoded flight plot."
