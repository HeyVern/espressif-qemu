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
    patches/nrf52840/    nrf52840 machine build wiring
    boards/              per-board run-time facts, consumed by the boot checks
    scripts/             bootstrap and build

## Building

    scripts/bootstrap.sh          # clone the pinned tag, copy files/, apply patches/
    scripts/build.sh xtensa-softmmu

Needs `ninja`, `meson`, glib and pixman headers. On a host without them, `uv venv`
supplies ninja and meson.

## Verifying a download

GitHub records a digest for each release asset, computed from what it stored:

    gh api repos/<owner>/espressif-qemu/releases/tags/<tag> \
      --jq '.assets[] | select(.name|endswith(".tar.zst")) | .digest'

Pin that value where the release is consumed, so a replaced asset fails loudly.

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

Enough emulated hardware for a real firmware image to run. On esp32s3, a stock
heltec_v4 LoRa image boots, brings up its SX1262 through RadioLib, mounts SPIFFS,
reaches its main loop in about six seconds and answers commands over the emulated
USB console. On esp32c3, a stock XIAO image does the same. A stock Waveshare ESP32-S3-LR1121 image
(upstream v1.18.0) brings up its LR1121 the same way.

A release build sends its console to USB, so UART0 is silent and a log can look like
a hang when the firmware is running fine. Building the image with the console on
UART0 and RadioLib's SPI tracing on is the fastest way to see what a guest is really
doing.

## The models

`files/hw/xtensa/bramble/` comes from justinlindh/bramble (MIT) -- GPSPI2, an SX1262
radio, a GPIO overlay, a SAR ADC and an SSD1680 display -- with our changes on top.

The GPSPI2 model takes two qdev properties, which is what makes one binary serve
every board:

    cs-gpio    the pin the firmware drives as the radio's soft chip-select   (default 8)
    spi-base   the SPI controller window the overlay claims       (default 0x60025000)

Both are load-bearing: a wrong `cs-gpio` gives `radio init failed: -2`, and a wrong
`spi-base` puts the radio on a bus the firmware never talks to.

A third property picks the radio model on the bus:

    radio      sx1262 (default) or lr1121

The LR1121 model (`bramble_lr1121.c`) answers RadioLib's LR11x0 driver: 16-bit opcodes, answers
read back in a second all-NOP transaction, GetStatus by NOPs, and its interrupt on DIO9, set with
`-global driver=bramble.lr1121,property=dio9-gpio,value=<pin>`. TX and RX ride the same emu-link
events as the SX1262. `BRAMBLE_LR_TRACE=1` logs each command it executes.

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
- **The I2C buses are empty unless a peripheral is requested.** Unattached
  addresses NACK immediately. `bramble.oled` attaches a real SSD1306 controller
  on I2C0 for display-enabled firmware.

## Optional SX1262 chip library

The built-in SX1262 model is the default. An optional MIT-licensed chip library
provides additional modem settings, receive timing, IRQ diagnostics and fault
injection. Its source, tests, license and pinned revision are under
`files/hw/xtensa/bramble/virtual-sx1262/`.

Build and verify the library with a C++17 compiler:

    STRICT=1 scripts/build-radio.sh test
    scripts/build-radio.sh shared

Select it explicitly on any machine using `bramble.sx1262`:

    -global driver=bramble.sx1262,property=chip-library,value=/absolute/path/libvirtualsx1262.so

Release archives include the library beside the emulator. No library is loaded
unless `chip-library` is set. Missing symbols, an unreadable library, or an ABI
other than the pinned 1.9 fail startup with an error. The LR1121 uses its existing
model independently.

Additional properties on `bramble.sx1262`:

| Property | Default | Meaning |
| --- | --- | --- |
| `noise-seed` | 0 | Zero selects guest entropy; a nonzero value makes radio noise reproducible. |
| `strict-receive` | false | Require reception throughout the modeled preamble and frame. |
| `stuck-irq-ms` | 0 | Inject recurring stale detection IRQs; zero disables injection. |
| `rf-lockstep` | false | Advance a paused, icount-enabled guest to each binary RF engine tick. |
| `chip-counters` | read only | QOM JSON string with IRQ, CAD, drop, sync-word and receive-error counters. |

SPI, DIO1 and the existing emu-link broker remain the interfaces to the board
and channel. The adapter runs under the BQL, advances the chip in QEMU virtual
time, and defers IRQ assertions raised inside an SPI command until after that
command. System reset recreates the chip and clears its timers and counters.

Transmissions include preamble length, header mode, CRC, configured LDRO, IQ
inversion and sync word. A compatible broker must preserve these fields and use
the modem settings for airtime. Older brokers ignore them and retain their own
airtime defaults. `chip_toa_ms` is the chip's rounded estimate; the broker remains
responsible for channel scheduling and `txdone`.

Incoming `rx` messages can include `sync_word` and `failure` (1–6, as defined in
the vendored C ABI) to exercise rejection and receive-error handling. Missing
metadata retains compatibility with existing replay messages. Frequency, SF,
bandwidth and IQ inversion are checked when supplied. Ordinary broker loss does
not automatically become a CRC error.

This is a chip model, not MeshBench's waveform or propagation engine. BUSY stays
low, receive duty cycling records its periods without sleeping, and strict RX
can expose clock drift between independently running emulator guests. Leave
strict RX off for ordinary emu-link regression runs.

## RF engine and device controls

A chardev named `meshbench-radio` selects the binary RF protocol used by the
MeshBench channel engine. The local chip adapter handles radio frames, TXDONE,
busy state, failure injection, radio statistics and tick acknowledgements. Use
`rf-lockstep=on`, `-S` and `-icount shift=3,sleep=off,align=off` together. The
guest runs to each requested RF tick and pauses before acknowledging it. QOM
`chip-counters` includes engine and virtual clock measurements.

The sibling bench suite `suites/meshbench/` supplies a launcher translating the
engine's machine properties into these local models. It selects the bundled
radio library and preserves the standalone emu-link path.

The GPIO model exposes QOM `gpio-state` for input/output masks and write-only
`gpio-input` accepting `pin:level` for pins 0–48. This uses the existing input
and interrupt mechanism. A launcher can map held button events to these writes.

`-device bramble.oled,path=/path/to/panel.sock,address=60,column-offset=0`
attaches an SSD1306 panel to I2C0. It sends the controller's actual 128×64 page
framebuffer and power state using the engine's MBF2 panel protocol. The model
is derived from MeshBench's GPL-2.0-or-later SSD1306 implementation at revision
`922e9b4bc37dc480a06f9f7bc2e96eefc233c9d6`; its copyright and license notice are
retained in `bramble_ssd1306.c`. Firmware without display support produces no
pixels. This panel does not change the LR1121 or nRF52840 peripheral paths.
