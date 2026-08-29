# espressif-qemu

Pre-built QEMU for emulating ESP32 LoRa boards. Upstream espressif/qemu leaves out
peripherals such a board needs -- there is no LoRa radio, the esp32s3 machine has no
I2C controller, and the USB serial console is a stub -- so this repo carries the
missing device models and the fixes, and publishes built binaries as releases.

A pinned upstream clone is created at build time, the model sources copied in and the
patches applied on top, so upstream stays upstream and our changes stay reviewable.

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

## Running

One artifact per MCU. Board wiring is set at run time, so the same binary serves
every board on an architecture.

    qemu-system-xtensa -display none -monitor none -machine esp32s3 -m 8M \
      -L pc-bios -drive file=<flash_image.bin>,if=mtd,format=raw \
      -global driver=bramble.gpspi2,property=cs-gpio,value=8 \
      -global driver=bramble.gpspi2,property=spi-base,value=0x60025000 \
      -serial null -serial null -serial stdio

`boards/*.json` carries those arguments per board, along with the machine, memory
size and serial layout. Two things that are easy to get wrong:

- **Use the long `-global` form.** The driver name contains a dot and `-global a.b=c`
  splits at the first one, so the short form is silently ignored ("invalid class
  name" on stderr, then default behaviour).
- **Memory size is not advisory.** A board with PSRAM whose firmware probes for it
  wedges during init if `-m` is too small.

The USB console is serial index 2; 0 and 1 are the UARTs. A firmware built for
hardware CDC puts its console there.

## What this gets you

Enough emulated hardware for a real firmware image to run: a stock heltec_v4 LoRa
image boots on this build, brings up its SX1262 through RadioLib, mounts SPIFFS,
reaches its main loop in about six seconds and answers commands over the emulated
USB console.

## The models

`files/hw/xtensa/bramble/` comes from justinlindh/bramble (MIT) -- GPSPI2, an SX1262
radio, a GPIO overlay, a SAR ADC and an SSD1680 display -- with our changes on top.

The GPSPI2 model takes two qdev properties, which is what makes one binary serve
every board:

    cs-gpio    the pin the firmware drives as the radio's soft chip-select   (default 8)
    spi-base   the SPI controller window the overlay claims       (default 0x60025000)

Both are load-bearing: a wrong `cs-gpio` gives `radio init failed: -2`, and a wrong
`spi-base` puts the radio on a bus the firmware never talks to.

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
  6-second boot. A device that ACKs gets initialised for real, and the upstream I2C
  model then faults the guest on the first data phase.
