/* Bramble ESP32-S3 USB OTG device controller with a built-in CDC host. See hw/xtensa/bramble/bramble_usbotg.c. */

#ifndef HW_XTENSA_BRAMBLE_USBOTG_H
#define HW_XTENSA_BRAMBLE_USBOTG_H

#include "exec/memory.h"
#include "hw/qdev-core.h"

/* Maps the OTG core and USB_WRAP, raises the USB interrupt source on intc, and bridges CDC to serial_hd(3). */
void bramble_usbotg_attach(MemoryRegion *sys_mem, DeviceState *intc);

#endif
