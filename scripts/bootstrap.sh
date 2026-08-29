#!/usr/bin/env bash
# Produce a buildable tree: pinned espressif/qemu, our model sources copied in,
# our patches applied. Idempotent only on a fresh dir.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$here/qemu.lock"
dest="${1:-$here/build/qemu-esp}"

if [ ! -d "$dest/.git" ]; then
  git clone --branch "$QEMU_TAG" --depth 1 "$QEMU_REPO" "$dest"
fi

# Model sources first, mirroring the upstream root; patches then edit only files
# upstream already owns.
tar -C "$here/files" -cf - . | tar -C "$dest" -xf -

# upstream fixes, then anything both machines share, then per-machine wiring.
for p in "$here"/patches/upstream/*.patch \
         "$here"/patches/common/*.patch \
         "$here"/patches/esp32s3/*.patch \
         "$here"/patches/esp32c3/*.patch; do
  [ -e "$p" ] || continue
  echo "applying $(basename "$p")"
  git -C "$dest" apply --whitespace=nowarn "$p"
done

echo "tree ready: $dest"
