// nRF52840 machine that boots MeshCore Adafruit-core images without Nordic's SoftDevice.
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/guest-random.h"
#include "exec/address-spaces.h"
#include "exec/memory.h"
#include "exec/cpu-common.h"
#include "hw/core/cpu.h"
#include "hw/arm/boot.h"
#include "hw/arm/armv7m.h"
#include "hw/boards.h"
#include "hw/clock.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "nrf52840_cc310.h"
#include "hw/xtensa/bramble_gpio.h"
#include "hw/xtensa/bramble_sx1262.h"
#include "hw/xtensa/bramble_emulink.h"
#include "chardev/char-fe.h"
#include "sysemu/runstate.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"

#define TYPE_NRF52840_MACHINE MACHINE_TYPE_NAME("nrf52840-mobmesh")
OBJECT_DECLARE_SIMPLE_TYPE(Nrf52840MachineState, NRF52840_MACHINE)

#define FLASH_SIZE (1024 * 1024)
#define SRAM_SIZE (256 * 1024)
#define PAGE_SIZE_NRF 4096
#define APP_BASE 0x26000
#define STUB_BASE 0x1000
#define SVC_VECTOR_OFFSET 0x2c
#define STUB_APP_SLOT 0x28

#define FICR_BASE 0x10000000
#define UICR_BASE 0x10001000
#define SRAM_BASE 0x20000000
#define APB_BASE 0x40000000
#define APB_SIZE 0x10000000
#define GPIO_BASE 0x50000000
#define SD_DEV_ID 0xff0

#define NUM_PERIPH 64
#define NUM_RTC 3
#define RTC_HZ 32768ULL
#define RTC_MASK 0xffffffu

#define ID_POWER 0x00
#define ID_UARTE0 0x02
#define ID_SERIAL0 0x03
#define ID_SERIAL1 0x04
#define ID_GPIOTE 0x06
#define ID_SAADC 0x07
#define ID_TEMP 0x0c
#define ID_RNG 0x0d
#define ID_RTC0 0x0b
#define ID_RTC1 0x11
#define ID_NVMC 0x1e
#define ID_SPIM2 0x23
#define ID_RTC2 0x24
#define ID_USBD 0x27
#define ID_UARTE1 0x28
#define ID_SPIM3 0x2f

#define R_EVENTS 0x100
#define R_INTEN 0x300
#define R_INTENSET 0x304
#define R_INTENCLR 0x308
#define R_ENABLE 0x500
#define ENABLE_TWIM 6
#define ENABLE_SPIM 7

#define USB_PKT_MAX 64
#define USB_CTRL_MAX 512
#define USB_STEP_NS (50 * SCALE_US)
#define USB_RESET_DELAY_NS (5 * SCALE_MS)
#define USB_GAP_NS (2 * SCALE_MS)
#define USB_CAUSE_READY (1u << 11)
#define USB_R_CAUSE 0x400
#define USB_R_DATASTATUS 0x46c
#define USB_R_USBADDR 0x470
#define USB_R_BMREQ 0x480
#define USB_R_SIZE_OUT 0x4a0
#define USB_R_PULLUP 0x504
#define USB_EV_RESET 0x100
#define USB_EV_ENDIN(n) (0x108 + (n) * 4)
#define USB_EV_EP0DONE 0x128
#define USB_EV_ENDOUT(n) (0x130 + (n) * 4)
#define USB_EV_SETUP 0x15c
#define USB_EV_EPDATA 0x160

#define NRF_SUCCESS 0
#define NRF_ERROR_NOT_SUPPORTED 6
#define NRF_ERROR_NOT_FOUND 5

// Hand-assembled SVC trampoline: SVC 0 goes to the app, every other number to the stub device.
static const uint8_t svc_stub[48] = {
    0x1e, 0xf0, 0x04, 0x0f, 0x0c, 0xbf, 0xef, 0xf3, 0x08, 0x80, 0xef, 0xf3,
    0x09, 0x80, 0x81, 0x69, 0x11, 0xf8, 0x02, 0x1c, 0x09, 0xb9, 0x04, 0x4a,
    0x10, 0x47, 0x04, 0x4a, 0x10, 0x60, 0xc2, 0xf8, 0x08, 0xe0, 0x51, 0x60,
    0x70, 0x47, 0x00, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x4f,
};

typedef struct {
    uint32_t regs[1024];
} Periph;

typedef struct {
    int id;
    QEMUTimer *timer;
    int64_t start_ns;
    uint32_t base;
    uint32_t last;
    bool running;
} NrfRtc;

struct Nrf52840MachineState {
    MachineState parent;
    ARMv7MState armv7m;
    Clock *sysclk;
    Clock *refclk;
    MemoryRegion flash;
    MemoryRegion sram;
    MemoryRegion ficr;
    MemoryRegion uicr;
    MemoryRegion partid;
    MemoryRegion cryptocell;
    MemoryRegion apb;
    MemoryRegion gpio;
    uint8_t *flash_ptr;
    char *flash_file;
    int flash_fd;
    Periph periph[NUM_PERIPH];
    NrfRtc rtc[NUM_RTC];
    uint32_t gpio_out[2];
    uint32_t gpio_dir[2];
    uint32_t gpio_latch[2];
    uint32_t gpio_detect[2];
    uint32_t gpio_cnf[2][32];
    uint32_t gpio_ext[2];
    QEMUTimer *usb_timer;
    CharBackend usb_chr;
    int usb_step;
    uint8_t usb_ctrl[USB_CTRL_MAX];
    uint32_t usb_ctrl_len;
    uint8_t usb_out[USB_PKT_MAX + 8];
    uint32_t usb_out_len;
    uint32_t usb_in_pending;
    bool usb_pwrrdy_pending;
    bool usb_start_pending;
    bool usb_ctrl_in_pending;
    bool usb_ctrl_out_pending;
    bool usb_next_pending;
    bool usb_accept_pending;
    bool usb_ready;
    bool usb_out_armed[8];
    uint8_t usb_rx[USB_PKT_MAX];
    uint32_t usb_rx_len;
    int usb_cdc_if;
    int usb_cdc_in;
    int usb_cdc_out;
    uint16_t usb_cfg_total;
    uint32_t sd_frame;
    uint32_t sd_exc_return;
    CharBackend uart0;
    SSIBus *spi;
    qemu_irq radio_cs_line;
    uint32_t radio_cs;
    uint32_t radio_dio1;
    uint32_t radio_busy;
    bool radio_selected;
};

static inline uint32_t *reg(Nrf52840MachineState *s, int id, hwaddr off)
{
    return &s->periph[id].regs[off >> 2];
}

static void periph_update(Nrf52840MachineState *s, int id)
{
    uint32_t inten = *reg(s, id, R_INTEN);
    bool level = false;

    for (int bit = 0; bit < 32; bit++) {
        if ((inten & (1u << bit)) && *reg(s, id, R_EVENTS + bit * 4)) {
            level = true;
            break;
        }
    }
    qemu_set_irq(qdev_get_gpio_in(DEVICE(&s->armv7m), id), level);
}

static void event_set(Nrf52840MachineState *s, int id, hwaddr off)
{
    *reg(s, id, off) = 1;
    periph_update(s, id);
}

static NrfRtc *rtc_for(Nrf52840MachineState *s, int id)
{
    for (int i = 0; i < NUM_RTC; i++) {
        if (s->rtc[i].id == id) {
            return &s->rtc[i];
        }
    }
    return NULL;
}

static inline uint64_t rtc_period_ns(Nrf52840MachineState *s, NrfRtc *r)
{
    uint32_t prescaler = *reg(s, r->id, 0x508) & 0xfff;

    return (NANOSECONDS_PER_SECOND * (prescaler + 1)) / RTC_HZ;
}

static uint32_t rtc_count(Nrf52840MachineState *s, NrfRtc *r)
{
    if (!r->running) {
        return r->base;
    }
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - r->start_ns;

    return (r->base + (uint32_t)(elapsed / rtc_period_ns(s, r))) & RTC_MASK;
}

static inline uint32_t rtc_enabled(Nrf52840MachineState *s, NrfRtc *r)
{
    return *reg(s, r->id, R_INTEN) | *reg(s, r->id, 0x340);
}

static void rtc_schedule(Nrf52840MachineState *s, NrfRtc *r)
{
    timer_del(r->timer);
    if (!r->running) {
        return;
    }
    uint32_t now = rtc_count(s, r);
    uint32_t enabled = rtc_enabled(s, r);
    uint32_t best = 0x1000000;

    if (enabled & 1) {
        best = 1;
    }
    if (enabled & 2) {
        best = MIN(best, ((RTC_MASK - now) & RTC_MASK) + 1);
    }
    for (int n = 0; n < 4; n++) {
        if (enabled & (1u << (16 + n))) {
            uint32_t delta = (*reg(s, r->id, 0x540 + n * 4) - now) & RTC_MASK;

            best = MIN(best, delta ? delta : 0x1000000);
        }
    }
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - r->start_ns;
    uint64_t period = rtc_period_ns(s, r);
    int64_t next = r->start_ns + (elapsed / period + best) * period;

    timer_mod_ns(r->timer, next);
}

static void rtc_poll(Nrf52840MachineState *s, NrfRtc *r)
{
    uint32_t now = rtc_count(s, r);
    uint32_t enabled = rtc_enabled(s, r);
    uint32_t delta = (now - r->last) & RTC_MASK;

    if (!delta) {
        return;
    }
    if ((enabled & 1)) {
        event_set(s, r->id, 0x100);
    }
    if ((enabled & 2) && now < r->last) {
        event_set(s, r->id, 0x104);
    }
    for (int n = 0; n < 4; n++) {
        uint32_t cc = *reg(s, r->id, 0x540 + n * 4) & RTC_MASK;

        if ((enabled & (1u << (16 + n))) &&
            ((cc - r->last - 1) & RTC_MASK) < delta) {
            event_set(s, r->id, 0x140 + n * 4);
        }
    }
    r->last = now;
}

static void rtc_tick(void *opaque)
{
    NrfRtc *r = opaque;
    Nrf52840MachineState *s = NRF52840_MACHINE(qdev_get_machine());

    rtc_poll(s, r);
    rtc_schedule(s, r);
}

static void rtc_task(Nrf52840MachineState *s, NrfRtc *r, hwaddr off)
{
    rtc_poll(s, r);
    switch (off) {
    case 0x00:
        if (!r->running) {
            r->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            r->running = true;
        }
        break;
    case 0x04:
        r->base = rtc_count(s, r);
        r->running = false;
        break;
    case 0x08:
        r->base = 0;
        r->last = 0;
        r->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    case 0x0c:
        r->base = 0xfffff0;
        r->last = 0xfffff0;
        r->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    default:
        break;
    }
    rtc_schedule(s, r);
}

static void spim_start(Nrf52840MachineState *s, int id)
{
    uint32_t tx_len = *reg(s, id, 0x548) & 0xffff;
    uint32_t rx_ptr = *reg(s, id, 0x534), rx_len = *reg(s, id, 0x538) & 0xffff;
    uint32_t tx_ptr = *reg(s, id, 0x544), orc = *reg(s, id, 0x5c0) & 0xff;
    uint32_t n = MAX(tx_len, rx_len);

    event_set(s, id, 0x14c);
    for (uint32_t i = 0; i < n; i++) {
        uint8_t out = orc, in = 0;

        if (i < tx_len) {
            cpu_physical_memory_read(tx_ptr + i, &out, 1);
        }
        if (s->radio_selected) {
            in = ssi_transfer(s->spi, out);
        }
        if (i < rx_len) {
            cpu_physical_memory_write(rx_ptr + i, &in, 1);
        }
    }
    *reg(s, id, 0x53c) = rx_len;
    *reg(s, id, 0x54c) = tx_len;
    event_set(s, id, 0x110);
    event_set(s, id, 0x120);
    event_set(s, id, 0x118);
}

static void twim_task(Nrf52840MachineState *s, int id, hwaddr off)
{
    // No I2C devices are attached, so every transfer ends in an address NACK.
    switch (off) {
    case 0x00:
        event_set(s, id, 0x14c);
        *reg(s, id, 0x4c4) |= 2;
        event_set(s, id, 0x124);
        event_set(s, id, 0x104);
        break;
    case 0x08:
        event_set(s, id, 0x150);
        *reg(s, id, 0x4c4) |= 2;
        event_set(s, id, 0x124);
        event_set(s, id, 0x104);
        break;
    case 0x14:
        event_set(s, id, 0x104);
        break;
    default:
        break;
    }
}

static void uarte_task(Nrf52840MachineState *s, int id, hwaddr off)
{
    switch (off) {
    case 0x00:
        event_set(s, id, 0x14c);
        break;
    case 0x04:
        *reg(s, id, 0x53c) = 0;
        event_set(s, id, 0x110);
        event_set(s, id, 0x144);
        break;
    case 0x08: {
        uint32_t ptr = *reg(s, id, 0x544), len = *reg(s, id, 0x548) & 0xffff;
        g_autofree uint8_t *buf = g_malloc(len + 1);

        event_set(s, id, 0x150);
        cpu_physical_memory_read(ptr, buf, len);
        if (id == ID_UARTE0) {
            qemu_chr_fe_write_all(&s->uart0, buf, len);
        }
        *reg(s, id, 0x54c) = len;
        event_set(s, id, 0x120);
        break;
    }
    case 0x0c:
        event_set(s, id, 0x158);
        break;
    default:
        break;
    }
}

static void saadc_task(Nrf52840MachineState *s, hwaddr off)
{
    switch (off) {
    case 0x00:
        event_set(s, ID_SAADC, 0x100);
        break;
    case 0x04: {
        int16_t value = 2048;
        uint32_t ptr = *reg(s, ID_SAADC, 0x62c);
        uint32_t maxcnt = *reg(s, ID_SAADC, 0x630);

        for (uint32_t i = 0; i < maxcnt; i++) {
            cpu_physical_memory_write(ptr + i * 2, &value, 2);
        }
        *reg(s, ID_SAADC, 0x634) = maxcnt;
        event_set(s, ID_SAADC, 0x10c);
        event_set(s, ID_SAADC, 0x108);
        event_set(s, ID_SAADC, 0x104);
        break;
    }
    case 0x08:
        event_set(s, ID_SAADC, 0x114);
        break;
    case 0x0c:
        event_set(s, ID_SAADC, 0x110);
        break;
    default:
        break;
    }
}

static void power_task(Nrf52840MachineState *s, hwaddr off)
{
    switch (off) {
    case 0x00:
        *reg(s, ID_POWER, 0x40c) = 0x10001;
        *reg(s, ID_POWER, 0x408) = 1;
        event_set(s, ID_POWER, 0x100);
        break;
    case 0x04:
        *reg(s, ID_POWER, 0x40c) = 0;
        break;
    case 0x08:
        *reg(s, ID_POWER, 0x418) = 0x10000 | (*reg(s, ID_POWER, 0x518) & 3);
        *reg(s, ID_POWER, 0x414) = 1;
        event_set(s, ID_POWER, 0x104);
        break;
    case 0x10:
        event_set(s, ID_POWER, 0x10c);
        break;
    default:
        break;
    }
}

// Scripted USB host: enumerates the CDC function, then bridges its data endpoints to a chardev.
static void usb_kick(Nrf52840MachineState *s, int64_t delay)
{
    timer_mod_ns(s->usb_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay);
}

static void usb_send_setup(Nrf52840MachineState *s, const uint8_t setup[8],
                           const uint8_t *data, uint32_t len)
{
    for (int i = 0; i < 8; i++) {
        *reg(s, ID_USBD, USB_R_BMREQ + i * 4) = setup[i];
    }
    s->usb_ctrl_len = 0;
    s->usb_out_len = len;
    if (len) {
        memcpy(s->usb_out, data, len);
    }
    event_set(s, ID_USBD, USB_EV_SETUP);
}

static void usb_parse_config(Nrf52840MachineState *s)
{
    uint32_t i = 0;

    while (i + 2 <= s->usb_ctrl_len && s->usb_ctrl[i]) {
        const uint8_t *d = s->usb_ctrl + i;

        if (d[1] == 4 && d[5] == 2) {
            s->usb_cdc_if = d[2];
        } else if (d[1] == 5 && (d[3] & 3) == 2) {
            if (d[2] & 0x80) {
                s->usb_cdc_in = d[2] & 0xf;
            } else {
                s->usb_cdc_out = d[2] & 0xf;
            }
        }
        i += d[0];
    }
}

static void usb_next_request(Nrf52840MachineState *s)
{
    static const uint8_t line_coding[7] = { 0x00, 0xc2, 0x01, 0x00, 0, 0, 8 };
    uint8_t setup[8] = { 0 };
    int step = s->usb_step++;

    switch (step) {
    case 0:
        memcpy(setup, "\x80\x06\x00\x01\x00\x00\x12\x00", 8);
        usb_send_setup(s, setup, NULL, 0);
        break;
    case 1:
        memcpy(setup, "\x00\x05\x01\x00\x00\x00\x00\x00", 8);
        usb_send_setup(s, setup, NULL, 0);
        *reg(s, ID_USBD, USB_R_USBADDR) = 1;
        s->usb_next_pending = true;
        usb_kick(s, USB_GAP_NS);
        break;
    case 2:
        memcpy(setup, "\x80\x06\x00\x02\x00\x00\x09\x00", 8);
        usb_send_setup(s, setup, NULL, 0);
        break;
    case 3:
        s->usb_cfg_total = MIN(lduw_le_p(s->usb_ctrl + 2), USB_CTRL_MAX);
        memcpy(setup, "\x80\x06\x00\x02\x00\x00", 6);
        stw_le_p(setup + 6, s->usb_cfg_total);
        usb_send_setup(s, setup, NULL, 0);
        break;
    case 4:
        usb_parse_config(s);
        memcpy(setup, "\x00\x09\x01\x00\x00\x00\x00\x00", 8);
        usb_send_setup(s, setup, NULL, 0);
        break;
    case 5:
        memcpy(setup, "\x21\x20\x00\x00", 4);
        stw_le_p(setup + 4, s->usb_cdc_if);
        setup[6] = sizeof(line_coding);
        usb_send_setup(s, setup, line_coding, sizeof(line_coding));
        break;
    case 6:
        memcpy(setup, "\x21\x22\x01\x00", 4);
        stw_le_p(setup + 4, s->usb_cdc_if);
        usb_send_setup(s, setup, NULL, 0);
        break;
    default:
        s->usb_ready = true;
        s->usb_accept_pending = true;
        usb_kick(s, USB_STEP_NS);
        break;
    }
}

static void usb_timer_cb(void *opaque)
{
    Nrf52840MachineState *s = opaque;

    if (s->usb_pwrrdy_pending) {
        s->usb_pwrrdy_pending = false;
        event_set(s, ID_POWER, 0x124);
    }
    if (s->usb_start_pending) {
        s->usb_start_pending = false;
        s->usb_step = 0;
        event_set(s, ID_USBD, USB_EV_RESET);
        s->usb_next_pending = true;
        usb_kick(s, USB_GAP_NS);
        return;
    }
    if (s->usb_ctrl_in_pending) {
        s->usb_ctrl_in_pending = false;
        event_set(s, ID_USBD, USB_EV_EP0DONE);
    }
    if (s->usb_ctrl_out_pending) {
        s->usb_ctrl_out_pending = false;
        *reg(s, ID_USBD, USB_R_SIZE_OUT) = s->usb_out_len;
        event_set(s, ID_USBD, USB_EV_EP0DONE);
    }
    if (s->usb_in_pending) {
        *reg(s, ID_USBD, USB_R_DATASTATUS) |= s->usb_in_pending;
        s->usb_in_pending = 0;
        event_set(s, ID_USBD, USB_EV_EPDATA);
    }
    if (s->usb_next_pending) {
        s->usb_next_pending = false;
        usb_next_request(s);
    }
    if (s->usb_accept_pending) {
        s->usb_accept_pending = false;
        qemu_chr_fe_accept_input(&s->usb_chr);
    }
}

static void usb_start_in(Nrf52840MachineState *s, int n)
{
    uint32_t base = 0x600 + n * 0x14;
    uint32_t ptr = *reg(s, ID_USBD, base);
    uint32_t len = MIN(*reg(s, ID_USBD, base + 4), USB_PKT_MAX);
    uint8_t buf[USB_PKT_MAX];

    cpu_physical_memory_read(ptr, buf, len);
    *reg(s, ID_USBD, base + 8) = len;
    if (n == 0) {
        len = MIN(len, USB_CTRL_MAX - s->usb_ctrl_len);
        memcpy(s->usb_ctrl + s->usb_ctrl_len, buf, len);
        s->usb_ctrl_len += len;
        s->usb_ctrl_in_pending = true;
    } else {
        if (n == s->usb_cdc_in && len) {
            qemu_chr_fe_write_all(&s->usb_chr, buf, len);
        }
        s->usb_in_pending |= 1u << n;
    }
    event_set(s, ID_USBD, USB_EV_ENDIN(n));
    usb_kick(s, USB_STEP_NS);
}

static void usb_start_out(Nrf52840MachineState *s, int n)
{
    uint32_t base = 0x700 + n * 0x14;
    uint32_t ptr = *reg(s, ID_USBD, base);
    uint32_t len = *reg(s, ID_USBD, base + 4);
    const uint8_t *src = n == 0 ? s->usb_out : s->usb_rx;

    len = MIN(len, n == 0 ? s->usb_out_len : s->usb_rx_len);
    cpu_physical_memory_write(ptr, src, len);
    *reg(s, ID_USBD, base + 8) = len;
    if (n != 0) {
        s->usb_rx_len = 0;
        s->usb_out_armed[n] = true;
        s->usb_accept_pending = true;
        usb_kick(s, USB_STEP_NS);
    }
    event_set(s, ID_USBD, USB_EV_ENDOUT(n));
}

static void usbd_task(Nrf52840MachineState *s, hwaddr off)
{
    if (off >= 0x004 && off <= 0x020) {
        usb_start_in(s, (off - 0x004) / 4);
    } else if (off >= 0x028 && off <= 0x044) {
        usb_start_out(s, (off - 0x028) / 4);
    } else if (off == 0x04c) {
        s->usb_ctrl_out_pending = true;
        usb_kick(s, USB_STEP_NS);
    } else if (off == 0x050 || off == 0x054) {
        s->usb_next_pending = true;
        usb_kick(s, USB_GAP_NS);
    }
}

// Registers whose writes have side effects; returns true when the write was consumed.
static bool usbd_write(Nrf52840MachineState *s, hwaddr off, uint32_t value)
{
    if (off == USB_R_CAUSE || off == USB_R_DATASTATUS) {
        *reg(s, ID_USBD, off) &= ~value;
        return true;
    }
    if (off >= USB_R_SIZE_OUT && off < USB_R_SIZE_OUT + 32) {
        s->usb_out_armed[(off - USB_R_SIZE_OUT) / 4] = true;
        s->usb_accept_pending = true;
        usb_kick(s, USB_STEP_NS);
        return true;
    }
    if (off == R_ENABLE && value) {
        *reg(s, ID_USBD, USB_R_CAUSE) |= USB_CAUSE_READY;
        // Delayed like the real regulator, so the handler runs after HFCLK has been requested.
        s->usb_pwrrdy_pending = true;
        usb_kick(s, USB_GAP_NS);
    }
    if (off == USB_R_PULLUP && value && !*reg(s, ID_USBD, off)) {
        s->usb_start_pending = true;
        s->usb_ready = false;
        usb_kick(s, USB_RESET_DELAY_NS);
    }
    return false;
}

static int usb_can_receive(void *opaque)
{
    Nrf52840MachineState *s = opaque;

    if (!s->usb_ready || !s->usb_out_armed[s->usb_cdc_out] || s->usb_rx_len) {
        return 0;
    }
    return USB_PKT_MAX;
}

static void usb_receive(void *opaque, const uint8_t *buf, int size)
{
    Nrf52840MachineState *s = opaque;
    int n = s->usb_cdc_out;

    size = MIN(size, USB_PKT_MAX);
    memcpy(s->usb_rx, buf, size);
    s->usb_rx_len = size;
    s->usb_out_armed[n] = false;
    *reg(s, ID_USBD, USB_R_SIZE_OUT + n * 4) = size;
    *reg(s, ID_USBD, USB_R_DATASTATUS) |= 1u << (16 + n);
    event_set(s, ID_USBD, USB_EV_EPDATA);
}

static void periph_task(Nrf52840MachineState *s, int id, hwaddr off)
{
    uint32_t enable = *reg(s, id, R_ENABLE);
    NrfRtc *r = rtc_for(s, id);

    if (r) {
        rtc_task(s, r, off);
        return;
    }
    switch (id) {
    case ID_USBD:
        usbd_task(s, off);
        break;
    case ID_POWER:
        power_task(s, off);
        break;
    case ID_UARTE0:
    case ID_UARTE1:
        uarte_task(s, id, off);
        break;
    case ID_SERIAL0:
    case ID_SERIAL1:
        if (enable == ENABLE_TWIM) {
            twim_task(s, id, off);
        } else if (enable == ENABLE_SPIM && off == 0x10) {
            spim_start(s, id);
        }
        break;
    case ID_SPIM2:
    case ID_SPIM3:
        if (off == 0x10) {
            spim_start(s, id);
        }
        break;
    case ID_SAADC:
        saadc_task(s, off);
        break;
    case ID_TEMP:
        if (off == 0x00) {
            *reg(s, ID_TEMP, 0x508) = 100;
            event_set(s, ID_TEMP, 0x100);
        }
        break;
    case ID_RNG:
        if (off == 0x00) {
            event_set(s, ID_RNG, 0x100);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "nrf52840: task 0x%x on peripheral 0x%x\n",
                      (unsigned)off, id);
        break;
    }
}

static uint32_t guest_read32(uint32_t addr)
{
    uint32_t value = 0;

    cpu_physical_memory_read(addr, &value, 4);
    return value;
}

static void guest_write32(uint32_t addr, uint32_t value)
{
    cpu_physical_memory_write(addr, &value, 4);
}

static void flash_modify(Nrf52840MachineState *s, uint32_t addr, uint32_t len)
{
    memory_region_set_dirty(&s->flash, addr, len);
    if (s->flash_fd >= 0 && pwrite(s->flash_fd, s->flash_ptr + addr, len, addr) != len) {
        error_report("nrf52840-mobmesh: flash file write failed");
    }
}

// Answers the SoftDevice calls that remain once Bluetooth is out of scope.
static void sd_call(Nrf52840MachineState *s, uint32_t frame, uint32_t number)
{
    uint32_t r0 = guest_read32(frame), r1 = guest_read32(frame + 4);
    uint32_t r2 = guest_read32(frame + 8);
    uint32_t result = NRF_SUCCESS;

    if (nrf52840_cc310_call(frame, number, s->sd_exc_return, &result)) {
        guest_write32(frame, result);
        return;
    }
    switch (number) {
    case 0x10:
        result = NRF_ERROR_NOT_SUPPORTED;
        break;
    case 0x12: {
        uint8_t off = 0;

        cpu_physical_memory_write(r0, &off, 1);
        break;
    }
    case 0x28:
        if (r0 * PAGE_SIZE_NRF + PAGE_SIZE_NRF <= FLASH_SIZE) {
            memset(s->flash_ptr + r0 * PAGE_SIZE_NRF, 0xff, PAGE_SIZE_NRF);
            flash_modify(s, r0 * PAGE_SIZE_NRF, PAGE_SIZE_NRF);
        }
        break;
    case 0x29:
        if (r0 + r2 * 4 <= FLASH_SIZE) {
            g_autofree uint8_t *buf = g_malloc(r2 * 4);

            cpu_physical_memory_read(r1, buf, r2 * 4);
            for (uint32_t i = 0; i < r2 * 4; i++) {
                s->flash_ptr[r0 + i] &= buf[i];
            }
            flash_modify(s, r0, r2 * 4);
        }
        break;
    case 0x4b:
        result = NRF_ERROR_NOT_FOUND;
        break;
    case 0x4c:
        guest_write32(r0, 100);
        break;
    case 0x11:
    case 0x13:
    case 0x33:
    case 0x35:
    case 0x3c:
    case 0x3d:
    case 0x3f:
    case 0x41:
    case 0x42:
    case 0x43:
    case 0x44:
    case 0x46:
    case 0x4d:
    case 0x4e:
    case 0x4f:
    case 0x50:
        break;
    default:
        result = NRF_ERROR_NOT_SUPPORTED;
        qemu_log_mask(LOG_UNIMP, "nrf52840: unhandled SVC 0x%x\n", number);
        break;
    }
    guest_write32(frame, result);
}

static uint64_t apb_read(void *opaque, hwaddr addr, unsigned size)
{
    Nrf52840MachineState *s = opaque;
    int id = (addr >> 12) & 0xfff;
    hwaddr off = addr & 0xffc;

    if (id >= NUM_PERIPH) {
        return 0;
    }
    NrfRtc *r = rtc_for(s, id);

    if (off == R_INTENSET || off == R_INTENCLR) {
        return *reg(s, id, R_INTEN);
    }
    if (r && off == 0x504) {
        rtc_poll(s, r);
        return rtc_count(s, r);
    }
    if (id == ID_RNG && off == 0x508) {
        uint8_t value;

        qemu_guest_getrandom_nofail(&value, 1);
        *reg(s, ID_RNG, 0x100) = 1;
        return value;
    }
    if (id == ID_NVMC && (off == 0x400 || off == 0x408)) {
        return 1;
    }
    return *reg(s, id, off);
}

static void apb_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    Nrf52840MachineState *s = opaque;
    int id = (addr >> 12) & 0xfff;
    hwaddr off = addr & 0xffc;

    if (id == SD_DEV_ID) {
        if (off == 0) {
            s->sd_frame = value;
        } else if (off == 8) {
            s->sd_exc_return = value;
        } else if (off == 4) {
            sd_call(s, s->sd_frame, value);
        }
        return;
    }
    if (id >= NUM_PERIPH) {
        return;
    }
    NrfRtc *r = rtc_for(s, id);

    if (id == ID_USBD && usbd_write(s, off, value)) {
        return;
    }
    if (id == ID_POWER && off == 0x500 && (value & 1)) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
    if (off == R_INTENSET) {
        *reg(s, id, R_INTEN) |= value;
    } else if (off == R_INTENCLR) {
        *reg(s, id, R_INTEN) &= ~value;
    } else if (r && off == 0x344) {
        *reg(s, id, 0x340) |= value;
    } else if (r && off == 0x348) {
        *reg(s, id, 0x340) &= ~value;
    } else if (off < R_EVENTS) {
        if (value) {
            periph_task(s, id, off);
        }
        return;
    } else {
        if (r) {
            rtc_poll(s, r);
        }
        *reg(s, id, off) = value;
    }
    periph_update(s, id);
    if (r) {
        rtc_schedule(s, r);
    }
}

static const MemoryRegionOps apb_ops = {
    .read = apb_read,
    .write = apb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void gpio_outputs_changed(Nrf52840MachineState *s)
{
    int pin = s->radio_cs;
    uint32_t bit = 1u << (pin & 31);
    bool high = s->gpio_out[pin >> 5] & bit;

    if (high == !s->radio_selected) {
        return;
    }
    s->radio_selected = !high;
    qemu_set_irq(s->radio_cs_line, high);
}

// Called by the radio model to drive an input pin, numbered 0..47 as the firmware numbers it.
void bramble_gpio_set_input(int pin, bool level)
{
    Nrf52840MachineState *s = NRF52840_MACHINE(qdev_get_machine());
    int port = pin >> 5;
    uint32_t bit = 1u << (pin & 31);
    bool was = s->gpio_ext[port] & bit;

    if (port > 1 || was == level) {
        return;
    }
    s->gpio_ext[port] = level ? s->gpio_ext[port] | bit : s->gpio_ext[port] & ~bit;
    for (int ch = 0; ch < 8; ch++) {
        uint32_t cnf = *reg(s, ID_GPIOTE, 0x510 + ch * 4);
        uint32_t polarity = (cnf >> 16) & 3;

        if ((cnf & 3) == 1 && ((cnf >> 8) & 0x3f) == pin &&
            (polarity == 3 || polarity == (level ? 1 : 2))) {
            event_set(s, ID_GPIOTE, 0x100 + ch * 4);
        }
    }
}

static uint32_t gpio_in_value(Nrf52840MachineState *s, int port)
{
    uint32_t in = 0;

    for (int pin = 0; pin < 32; pin++) {
        uint32_t bit = 1u << pin;
        uint32_t pull = (s->gpio_cnf[port][pin] >> 2) & 3;

        if (s->gpio_dir[port] & bit) {
            in |= s->gpio_out[port] & bit;
        } else if (port * 32 + pin == s->radio_busy ||
                   port * 32 + pin == s->radio_dio1) {
            in |= s->gpio_ext[port] & bit;
        } else if (pull != 1) {
            // Boards pull unconnected inputs such as buttons high externally.
            in |= bit;
        }
    }
    return in;
}

static uint64_t gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    Nrf52840MachineState *s = opaque;
    int port = addr >= 0x800 ? 1 : 0;
    hwaddr off = addr - port * 0x300;

    if (off >= 0x700 && off < 0x780) {
        return s->gpio_cnf[port][(off - 0x700) >> 2];
    }
    switch (off) {
    case 0x504:
    case 0x508:
    case 0x50c:
        return s->gpio_out[port];
    case 0x510:
        return gpio_in_value(s, port);
    case 0x514:
    case 0x518:
    case 0x51c:
        return s->gpio_dir[port];
    case 0x520:
        return s->gpio_latch[port];
    case 0x524:
        return s->gpio_detect[port];
    default:
        return 0;
    }
}

static void gpio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    Nrf52840MachineState *s = opaque;
    int port = addr >= 0x800 ? 1 : 0;
    hwaddr off = addr - port * 0x300;

    if (off >= 0x700 && off < 0x780) {
        int pin = (off - 0x700) >> 2;

        s->gpio_cnf[port][pin] = value;
        s->gpio_dir[port] = deposit32(s->gpio_dir[port], pin, 1, value & 1);
        return;
    }
    switch (off) {
    case 0x504:
        s->gpio_out[port] = value;
        break;
    case 0x508:
        s->gpio_out[port] |= value;
        break;
    case 0x50c:
        s->gpio_out[port] &= ~value;
        break;
    case 0x514:
        s->gpio_dir[port] = value;
        break;
    case 0x518:
        s->gpio_dir[port] |= value;
        break;
    case 0x51c:
        s->gpio_dir[port] &= ~value;
        break;
    case 0x520:
        s->gpio_latch[port] &= ~value;
        break;
    case 0x524:
        s->gpio_detect[port] = value;
        break;
    default:
        break;
    }
    gpio_outputs_changed(s);
}

static const MemoryRegionOps gpio_ops = {
    .read = gpio_read,
    .write = gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void machine_reset_cb(void *opaque)
{
    Nrf52840MachineState *s = opaque;

    memset(s->periph, 0, sizeof(s->periph));
    memset(s->gpio_out, 0, sizeof(s->gpio_out));
    memset(s->gpio_dir, 0, sizeof(s->gpio_dir));
    memset(s->gpio_latch, 0, sizeof(s->gpio_latch));
    memset(s->gpio_detect, 0, sizeof(s->gpio_detect));
    memset(s->gpio_cnf, 0, sizeof(s->gpio_cnf));
    memset(s->gpio_ext, 0, sizeof(s->gpio_ext));
    s->radio_selected = false;
    timer_del(s->usb_timer);
    s->usb_step = 0;
    s->usb_ctrl_len = s->usb_out_len = s->usb_rx_len = 0;
    s->usb_in_pending = 0;
    s->usb_start_pending = s->usb_ctrl_in_pending = false;
    s->usb_pwrrdy_pending = false;
    s->usb_ctrl_out_pending = s->usb_next_pending = false;
    s->usb_accept_pending = s->usb_ready = false;
    memset(s->usb_out_armed, 0, sizeof(s->usb_out_armed));
    s->usb_cdc_if = 0;
    s->usb_cdc_in = s->usb_cdc_out = 0;
    // VBUS is always present, as if the board were plugged into a host.
    *reg(s, ID_POWER, 0x438) = 3;
    for (int i = 0; i < NUM_RTC; i++) {
        timer_del(s->rtc[i].timer);
        s->rtc[i].running = false;
        s->rtc[i].base = 0;
        s->rtc[i].last = 0;
    }
    cpu_reset(first_cpu);
}

// A private backing file keeps a guest's filesystem and identity across runs; the app is always reloaded.
static void open_flash_file(Nrf52840MachineState *s)
{
    struct stat st;

    s->flash_fd = -1;
    if (!s->flash_file) {
        return;
    }
    s->flash_fd = open(s->flash_file, O_RDWR | O_CREAT, 0644);
    if (s->flash_fd < 0 || fstat(s->flash_fd, &st) < 0) {
        error_report("nrf52840-mobmesh: cannot open %s", s->flash_file);
        exit(1);
    }
    if (st.st_size == FLASH_SIZE) {
        if (pread(s->flash_fd, s->flash_ptr, FLASH_SIZE, 0) != FLASH_SIZE) {
            error_report("nrf52840-mobmesh: cannot read %s", s->flash_file);
            exit(1);
        }
    } else if (pwrite(s->flash_fd, s->flash_ptr, FLASH_SIZE, 0) != FLASH_SIZE) {
        error_report("nrf52840-mobmesh: cannot initialise %s", s->flash_file);
        exit(1);
    }
}

static void load_app(Nrf52840MachineState *s, const char *path)
{
    g_autofree char *data = NULL;
    gsize length = 0;
    GError *err = NULL;

    if (!path) {
        error_report("nrf52840-mobmesh: -kernel <app.bin> is required");
        exit(1);
    }
    if (!g_file_get_contents(path, &data, &length, &err)) {
        error_report("nrf52840-mobmesh: %s", err->message);
        exit(1);
    }
    if (length > FLASH_SIZE - APP_BASE) {
        error_report("nrf52840-mobmesh: image does not fit in flash");
        exit(1);
    }
    memset(s->flash_ptr, 0xff, FLASH_SIZE);
    open_flash_file(s);
    memcpy(s->flash_ptr + APP_BASE, data, length);
    nrf52840_cc310_patch(s->flash_ptr + APP_BASE, length);

    // The vector table stays at the app; only its SVC entry is redirected through the stub.
    uint32_t app_svc = ldl_le_p(s->flash_ptr + APP_BASE + SVC_VECTOR_OFFSET);

    memcpy(s->flash_ptr + STUB_BASE, svc_stub, sizeof(svc_stub));
    stl_le_p(s->flash_ptr + STUB_BASE + STUB_APP_SLOT, app_svc);
    stl_le_p(s->flash_ptr + APP_BASE + SVC_VECTOR_OFFSET, STUB_BASE | 1);
}

static void init_ficr_uicr(Nrf52840MachineState *s)
{
    MemoryRegion *sysmem = get_system_memory();
    uint8_t *p;

    memory_region_init_ram(&s->ficr, NULL, "nrf52840.ficr", 0x1000,
                           &error_fatal);
    p = memory_region_get_ram_ptr(&s->ficr);
    memset(p, 0xff, 0x1000);
    stl_le_p(p + 0x060, 0x4d6f624d);
    stl_le_p(p + 0x064, 0x65736801);
    stl_le_p(p + 0x0a4, 0x4d6f624d);
    stl_le_p(p + 0x0a8, 0xc000);
    memory_region_set_readonly(&s->ficr, true);
    memory_region_add_subregion(sysmem, FICR_BASE, &s->ficr);

    memory_region_init_ram(&s->uicr, NULL, "nrf52840.uicr", 0x1000,
                           &error_fatal);
    p = memory_region_get_ram_ptr(&s->uicr);
    memset(p, 0xff, 0x1000);
    stl_le_p(p + 0x304, 5);
    memory_region_add_subregion(sysmem, UICR_BASE, &s->uicr);

    // Part identification words read by the Nordic errata checks in SystemInit.
    memory_region_init_ram(&s->partid, NULL, "nrf52840.partid", 0x1000,
                           &error_fatal);
    p = memory_region_get_ram_ptr(&s->partid);
    memset(p, 0, 0x1000);
    stl_le_p(p + 0xfe0, 0x08);
    memory_region_set_readonly(&s->partid, true);
    memory_region_add_subregion(sysmem, 0xf0000000, &s->partid);

    // CryptoCell register block: plain RAM; the library calls that would use it are replaced in nrf52840_cc310.c.
    memory_region_init_ram(&s->cryptocell, NULL, "nrf52840.cryptocell", 0x2000,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0x5002a000, &s->cryptocell);
}

static void nrf52840_init(MachineState *machine)
{
    Nrf52840MachineState *s = NRF52840_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    static const int rtc_ids[NUM_RTC] = { ID_RTC0, ID_RTC1, ID_RTC2 };
    DeviceState *armv7m;

    s->sysclk = clock_new(OBJECT(machine), "sysclk");
    clock_set_hz(s->sysclk, 64000000);
    s->refclk = clock_new(OBJECT(machine), "refclk");
    clock_set_hz(s->refclk, 32768);

    memory_region_init_ram(&s->flash, NULL, "nrf52840.flash",
                           FLASH_SIZE, &error_fatal);
    memory_region_set_readonly(&s->flash, true);
    s->flash_ptr = memory_region_get_ram_ptr(&s->flash);
    memory_region_add_subregion(sysmem, 0, &s->flash);
    memory_region_init_ram(&s->sram, NULL, "nrf52840.sram",
                           SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, SRAM_BASE, &s->sram);
    init_ficr_uicr(s);

    memory_region_init_io(&s->apb, OBJECT(machine), &apb_ops, s, "nrf52840.apb",
                          APB_SIZE);
    memory_region_add_subregion(sysmem, APB_BASE, &s->apb);
    memory_region_init_io(&s->gpio, OBJECT(machine), &gpio_ops, s,
                          "nrf52840.gpio", 0x1000);
    memory_region_add_subregion(sysmem, GPIO_BASE, &s->gpio);

    object_initialize_child(OBJECT(machine), "armv7m", &s->armv7m, TYPE_ARMV7M);
    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", NUM_PERIPH);
    qdev_prop_set_uint8(armv7m, "num-prio-bits", 3);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    qdev_prop_set_uint32(armv7m, "init-nsvtor", APP_BASE);
    qdev_prop_set_bit(armv7m, "enable-bitband", false);
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    object_property_set_link(OBJECT(armv7m), "memory", OBJECT(sysmem),
                             &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(armv7m), &error_fatal);

    for (int i = 0; i < NUM_RTC; i++) {
        s->rtc[i].id = rtc_ids[i];
        s->rtc[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rtc_tick, &s->rtc[i]);
    }
    // The bus hangs off the CPU container because a machine is not a qdev device.
    s->spi = ssi_create_bus(armv7m, "spi");
    DeviceState *radio = qdev_new(TYPE_BRAMBLE_SX1262);

    qdev_prop_set_uint8(radio, "cs", 0);
    qdev_prop_set_uint32(radio, "dio1-gpio", s->radio_dio1);
    qdev_prop_set_bit(radio, "noise-reg", true);
    ssi_realize_and_unref(radio, s->spi, &error_fatal);
    s->radio_cs_line = qdev_get_gpio_in_named(radio, SSI_GPIO_CS, 0);
    bramble_emulink_attach();

    if (serial_hd(0)) {
        qemu_chr_fe_init(&s->uart0, serial_hd(0), &error_fatal);
    }
    s->usb_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, usb_timer_cb, s);
    if (serial_hd(1)) {
        qemu_chr_fe_init(&s->usb_chr, serial_hd(1), &error_fatal);
        qemu_chr_fe_set_handlers(&s->usb_chr, usb_can_receive, usb_receive, NULL,
                                 NULL, s, NULL, true);
    }
    load_app(s, machine->kernel_filename);
    qemu_register_reset(machine_reset_cb, s);
}

static char *get_flash_file(Object *obj, Error **errp)
{
    return g_strdup(NRF52840_MACHINE(obj)->flash_file);
}

static void set_flash_file(Object *obj, const char *value, Error **errp)
{
    Nrf52840MachineState *s = NRF52840_MACHINE(obj);

    g_free(s->flash_file);
    s->flash_file = g_strdup(value);
}

static void nrf52840_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    object_class_property_add_str(oc, "flash-file", get_flash_file, set_flash_file);

    mc->desc = "nRF52840 for MeshCore images, SoftDevice replaced by a stub";
    mc->init = nrf52840_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4");
    mc->ignore_memory_transaction_failures = false;
}

static void nrf52840_instance_init(Object *obj)
{
    Nrf52840MachineState *s = NRF52840_MACHINE(obj);

    s->radio_cs = 43;
    s->radio_dio1 = 31;
    s->radio_busy = 29;
    object_property_add_uint32_ptr(obj, "radio-busy", &s->radio_busy,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "radio-cs", &s->radio_cs,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "radio-dio1", &s->radio_dio1,
                                   OBJ_PROP_FLAG_READWRITE);
}

static const TypeInfo nrf52840_info = {
    .name = TYPE_NRF52840_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Nrf52840MachineState),
    .instance_init = nrf52840_instance_init,
    .class_init = nrf52840_class_init,
};

static void nrf52840_register(void)
{
    type_register_static(&nrf52840_info);
}

type_init(nrf52840_register);
