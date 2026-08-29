The esp32c3 machine needs the same two things the esp32s3 needed and does not
have yet: a chardev and an interrupt on the USB-serial-JTAG block it already
instantiates (hw/riscv/esp32c3.c, realized and mapped but not wired), and an I2C
controller so the firmware sensor scan NACKs instead of running to timeout.

Its interrupt numbering is its own -- check the map-register offsets, not the
ETS_* enum. See the README.
