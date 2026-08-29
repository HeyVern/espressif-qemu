#!/usr/bin/env bash
# Configure and build one QEMU target from a bootstrapped tree.
set -euo pipefail

target="${1:-xtensa-softmmu}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
src="${2:-$here/build/qemu-esp}"

mkdir -p "$src/build"
cd "$src/build"
[ -f build.ninja ] || ../configure --target-list="$target" \
  --disable-slirp --disable-werror --disable-gnutls --disable-docs
ninja
