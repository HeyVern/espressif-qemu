# expressif-qemu

Pre-built QEMU for emulating MobMesh firmware images. Upstream espressif/qemu does
not model the peripherals a MeshCore node needs -- there is no LoRa radio, no I2C
controller on the S3 machine, and the USB serial console is a stub -- so this repo
carries the missing device models and the fixes, and publishes built binaries as
releases.

Nothing here is a fork. A pinned upstream clone is created at build time, our model
sources are copied in, and our patches are applied on top; the same `files/` +
`patches/` arrangement the firmware repo uses for MeshCore.

## Layout

    qemu.lock            upstream repo, tag and base commit
    files/               device model sources, mirroring the upstream root
    patches/upstream/    fixes bound for espressif/qemu
    patches/common/      device models and build wiring both machines share
    patches/esp32s3/     esp32s3 machine wiring
    patches/esp32c3/     esp32c3 machine wiring
    boards/              per-board run-time facts, consumed by the boot checks
    scripts/             bootstrap and build

## Building

    scripts/bootstrap.sh          # clone the pinned tag, copy files/, apply patches/
    scripts/build.sh xtensa-softmmu

Needs `ninja`, `meson`, glib and pixman headers. On a host without them, `uv venv`
supplies ninja and meson.

## What the binaries give you

`qemu-system-xtensa` covers esp32 and esp32s3; `qemu-system-riscv32` covers esp32c3.
Two binaries cover every board -- boards differ in run-time configuration, not in
build, which is what `boards/*.json` records.

With this build a stock `heltec_v4/repeater` image boots, initialises its SX1262
through RadioLib, mounts SPIFFS, reaches `loop()` in about six seconds, and answers
its CLI over the emulated USB console:

    ver     -> v1.17.1 (14 Aug 2026) + ota
    clock   -> 10:53 - 15/5/2024 UTC
    advert  -> OK - Advert sent

Run it with the USB console on stdio (serial index 2; 0 and 1 are the UARTs):

    qemu-system-xtensa -display none -monitor none -machine esp32s3 -m 8M \
      -L pc-bios -drive file=<flash_image.bin>,if=mtd,format=raw \
      -serial null -serial null -serial stdio

`-m 8M` is required for heltec_v4: the app probes for PSRAM.

## The models

Board differences that are not run-time configuration -- the radio's chip-select GPIO,
the SPI controller it hangs off -- are still compile-time constants in the models. A
second board needs those promoted to qdev properties fed from `boards/*.json`; the
directory layout does not help with that.

`files/hw/xtensa/bramble/` comes from justinlindh/bramble (MIT) -- GPSPI2, an SX1262
radio, a GPIO overlay, a SAR ADC and an SSD1680 display -- with our changes on top.

## The patches

`patches/upstream/` holds one fix that is not ours to keep: the ESP32-S3 SPI
controller counts quad-I/O dummy cycles as one bit each instead of four, so every
`esp_partition_read` returns data shifted by two bytes and SPIFFS can never mount.
It belongs upstream once it has a repro that does not depend on our images.

`patches/common/` builds the models and replaces the USB-serial-JTAG stub with a real
console. That model is shared: `hw/misc/esp32c3_jtag.c` backs the block on both
machines, so the console work is not esp32s3-specific even though only the esp32s3
machine wires it up today.

`patches/esp32s3/` is that machine's own wiring -- attaching the models, connecting
the console's chardev and interrupt, adding the two I2C controllers, and driving the
interrupt matrix combinationally.

Two things to know before touching the machine patches:

- **Interrupt-matrix inputs are numbered by map-register offset, not by the `ETS_*`
  enum.** The two orderings diverge partway down the list: USB-serial-JTAG is input
  96 where the enum says 92, and I2C_EXT0 is 42 where the enum says 39. An interrupt
  wired from the enum is asserted on a line nothing reads, and fails silently.
- **The I2C buses are left empty on purpose.** Empty means the firmware's 128-address
  probe NACKs immediately, which is the difference between a 200-second and a
  6-second boot. A device that ACKs gets initialised for real, and espressif/qemu#110
  then faults the guest on the first data phase.
