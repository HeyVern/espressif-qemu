#!/usr/bin/env bash
# Build the pinned, vendored SX1262 chip library independently of QEMU.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export OUT="${OUT:-$here/build/virtual-sx1262}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
export OUT
cd "$here/files/hw/xtensa/bramble/virtual-sx1262"
exec sh ./build.sh "${1:-shared}"
