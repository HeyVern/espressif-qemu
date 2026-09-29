/* Bramble ESP32-S3 USB OTG (DWC2, slave mode) device controller plus a scripted host that enumerates a CDC-ACM device and bridges it to a chardev. */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/qdev-core.h"
#include "qom/object.h"
#include "exec/address-spaces.h"
#include "chardev/char-fe.h"
#include "sysemu/sysemu.h"
#include "hw/xtensa/bramble_usbotg.h"
#include "hw/xtensa/bramble_scaffold.h"

#define USBOTG_BASE        0x60080000
#define USBOTG_WINDOW      0x20000
#define USBWRAP_BASE       0x60039000
#define USBWRAP_WINDOW     0x1000
#define USB_INTR_SOURCE    38   /* INTERRUPT_CORE0_USB_INTR_MAP_REG offset 0x098 / 4 */

#define NUM_EP             7
#define STEP_NS            (20 * SCALE_US)
#define HOST_GAP_NS        (2 * SCALE_MS)
#define RXQ_LEN            32
#define PKT_MAX            64
#define HOST_RX_MAX        512

/* Global register offsets */
#define R_GOTGCTL    0x000
#define R_GOTGINT    0x004
#define R_GAHBCFG    0x008
#define R_GUSBCFG    0x00c
#define R_GRSTCTL    0x010
#define R_GINTSTS    0x014
#define R_GINTMSK    0x018
#define R_GRXSTSR    0x01c
#define R_GRXSTSP    0x020
#define R_GRXFSIZ    0x024
#define R_GNPTXFSIZ  0x028
#define R_GNPTXSTS   0x02c
#define R_GSNPSID    0x040
#define R_GHWCFG1    0x044
#define R_GHWCFG2    0x048
#define R_GHWCFG3    0x04c
#define R_GHWCFG4    0x050
#define R_DCFG       0x800
#define R_DCTL       0x804
#define R_DSTS       0x808
#define R_DIEPMSK    0x810
#define R_DOEPMSK    0x814
#define R_DAINT      0x818
#define R_DAINTMSK   0x81c
#define R_DIEPEMPMSK 0x834
#define R_INEP_BASE  0x900
#define R_OUTEP_BASE 0xb00
#define R_EP_STRIDE  0x20
#define R_FIFO_BASE  0x1000
#define R_FIFO_END   0x11000

/* GINTSTS bits */
#define GINT_RXFLVL     (1u << 4)
#define GINT_NPTXFEMP   (1u << 5)
#define GINT_GINNAKEFF  (1u << 6)
#define GINT_GOUTNAKEFF (1u << 7)
#define GINT_USBRST     (1u << 12)
#define GINT_ENUMDONE   (1u << 13)
#define GINT_IEPINT     (1u << 18)
#define GINT_OEPINT     (1u << 19)
#define GINT_PTXFEMP    (1u << 26)
#define GINT_DERIVED    (GINT_RXFLVL | GINT_NPTXFEMP | GINT_GINNAKEFF | GINT_GOUTNAKEFF | GINT_IEPINT | GINT_OEPINT | GINT_PTXFEMP | 1u)

#define GRST_CSFTRST    (1u << 0)
#define GRST_RXFFLSH    (1u << 4)
#define GRST_TXFFLSH    (1u << 5)
#define GRST_AHBIDL     (1u << 31)

#define DCTL_SFTDISCON  (1u << 1)
#define DCTL_SGNPINNAK  (1u << 7)
#define DCTL_CGNPINNAK  (1u << 8)
#define DCTL_SGOUTNAK   (1u << 9)
#define DCTL_CGOUTNAK   (1u << 10)
#define DCTL_WO         (DCTL_SGNPINNAK | DCTL_CGNPINNAK | DCTL_SGOUTNAK | DCTL_CGOUTNAK)

#define EPCTL_NAKSTS    (1u << 17)
#define EPCTL_STALL     (1u << 21)
#define EPCTL_CNAK      (1u << 26)
#define EPCTL_SNAK      (1u << 27)
#define EPCTL_SETD0PID  (1u << 28)
#define EPCTL_SETD1PID  (1u << 29)
#define EPCTL_EPDIS     (1u << 30)
#define EPCTL_EPENA     (1u << 31)
#define EPCTL_WO        (EPCTL_CNAK | EPCTL_SNAK | EPCTL_SETD0PID | EPCTL_SETD1PID)

#define EPINT_XFERCOMPL (1u << 0)
#define EPINT_EPDISBLD  (1u << 1)
#define DIEPINT_TXFEMP  (1u << 7)
#define DOEPINT_SETUP   (1u << 3)
#define DOEPINT_STUPRCV (1u << 15)

#define PKTSTS_OUT_DATA  2
#define PKTSTS_OUT_DONE  3
#define PKTSTS_SETUP_DONE 4
#define PKTSTS_SETUP_DATA 6

#define TYPE_BRAMBLE_USBOTG "bramble.usbotg"
OBJECT_DECLARE_SIMPLE_TYPE(BrambleUsbOtgState, BRAMBLE_USBOTG)

typedef struct {
    uint32_t status;
    uint8_t data[PKT_MAX];
    uint16_t len;
} RxEntry;

typedef enum {
    HOST_DETACHED,
    HOST_RESET,
    HOST_ENUMDONE,
    HOST_CONTROL,
    HOST_CONFIGURED,
    HOST_FAILED,
} HostState;

typedef enum {
    CTRL_IDLE,
    CTRL_DATA_IN,
    CTRL_DATA_OUT,
    CTRL_STATUS_IN,
    CTRL_STATUS_OUT,
    CTRL_WAIT_OUT_DONE,
    CTRL_DONE,
} CtrlPhase;

struct BrambleUsbOtgState {
    DeviceState parent_obj;

    MemoryRegion iomem;
    MemoryRegion wrap;
    qemu_irq irq;
    CharBackend chr;
    QEMUTimer *timer;

    uint32_t gotgint, gahbcfg, gusbcfg, grstctl, gintsts, gintmsk, grxfsiz, gnptxfsiz;
    uint32_t dcfg, dctl, diepmsk, doepmsk, daintmsk, diepempmsk;
    uint32_t dieptxf[4];
    uint32_t diepctl[NUM_EP], diepint[NUM_EP], dieptsiz[NUM_EP];
    uint32_t doepctl[NUM_EP], doepint[NUM_EP], doeptsiz[NUM_EP];
    uint32_t wrap_regs[USBWRAP_WINDOW / 4];
    bool goutnak, ginnak;

    uint8_t tx[NUM_EP][PKT_MAX + 4];
    uint16_t txlen[NUM_EP];

    RxEntry rxq[RXQ_LEN];
    int rx_head, rx_count;
    RxEntry cur;
    uint16_t cur_pos;
    bool out_pending[NUM_EP];

    HostState host;
    int64_t host_wait_until;
    int step;
    CtrlPhase phase;
    uint8_t setup[8];
    uint8_t out_data[PKT_MAX];
    uint16_t out_len;
    uint8_t in_buf[512];
    uint16_t in_len;
    uint16_t cfg_total;
    uint8_t cfg_value;
    uint8_t cdc_iface;
    uint8_t bulk_in, bulk_out;
    uint16_t bulk_out_mps;

    uint8_t host_rx[HOST_RX_MAX];
    uint16_t host_rx_len;
    bool trace;
};

static void usbotg_kick(BrambleUsbOtgState *s)
{
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + STEP_NS);
}

static void usbotg_log(BrambleUsbOtgState *s, const char *fmt, ...) G_GNUC_PRINTF(2, 3);
static void usbotg_log(BrambleUsbOtgState *s, const char *fmt, ...)
{
    if (!s->trace) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "bramble-usbotg: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

static unsigned ep_mps(BrambleUsbOtgState *s, int n, bool in)
{
    uint32_t ctl = in ? s->diepctl[n] : s->doepctl[n];
    if (n == 0) {
        static const unsigned ep0[4] = { 64, 32, 16, 8 };
        return ep0[ctl & 3];
    }
    unsigned mps = ctl & 0x7ff;
    return mps ? mps : 64;
}

static uint32_t tsiz_xfer(uint32_t v) { return v & 0x7ffff; }
static uint32_t tsiz_pkt(uint32_t v) { return (v >> 19) & 0x3ff; }
static uint32_t tsiz_make(uint32_t pkt, uint32_t xfer, uint32_t keep) { return (keep & (3u << 29)) | (pkt << 19) | xfer; }

static uint32_t daint_value(BrambleUsbOtgState *s)
{
    uint32_t v = 0;
    for (int n = 0; n < NUM_EP; n++) {
        uint32_t in = s->diepint[n] & (s->diepmsk & ~DIEPINT_TXFEMP);
        if ((s->diepempmsk & (1u << n)) && s->txlen[n] == 0) {
            in |= DIEPINT_TXFEMP;
        }
        if (in) {
            v |= 1u << n;
        }
        if (s->doepint[n] & s->doepmsk) {
            v |= 1u << (16 + n);
        }
    }
    return v;
}

static uint32_t gintsts_value(BrambleUsbOtgState *s)
{
    uint32_t v = s->gintsts & ~GINT_DERIVED;
    uint32_t daint = daint_value(s) & s->daintmsk;
    if (s->rx_count) {
        v |= GINT_RXFLVL;
    }
    v |= GINT_NPTXFEMP | GINT_PTXFEMP;
    if (s->ginnak) {
        v |= GINT_GINNAKEFF;
    }
    if (s->goutnak) {
        v |= GINT_GOUTNAKEFF;
    }
    if (daint & 0xffff) {
        v |= GINT_IEPINT;
    }
    if (daint & 0xffff0000) {
        v |= GINT_OEPINT;
    }
    return v;
}

static void usbotg_update_irq(BrambleUsbOtgState *s)
{
    for (int n = 0; n < NUM_EP; n++) {
        if (s->txlen[n] == 0) {
            s->diepint[n] |= DIEPINT_TXFEMP;
        } else {
            s->diepint[n] &= ~DIEPINT_TXFEMP;
        }
    }
    bool level = (s->gahbcfg & 1) && (gintsts_value(s) & s->gintmsk);
    qemu_set_irq(s->irq, level);
}

static bool rxq_push(BrambleUsbOtgState *s, int ep, int pktsts, const uint8_t *data, uint16_t len)
{
    if (s->rx_count >= RXQ_LEN) {
        return false;
    }
    RxEntry *e = &s->rxq[(s->rx_head + s->rx_count) % RXQ_LEN];
    e->status = (uint32_t)ep | ((uint32_t)len << 4) | ((uint32_t)pktsts << 17);
    e->len = len;
    if (len) {
        memcpy(e->data, data, len);
    }
    s->rx_count++;
    return true;
}

static void core_reset(BrambleUsbOtgState *s)
{
    s->gotgint = s->gahbcfg = s->gusbcfg = s->gintsts = s->gintmsk = 0;
    s->grxfsiz = 0x100;
    s->gnptxfsiz = 0x01000100;
    s->dcfg = 0;
    s->dctl = DCTL_SFTDISCON;
    s->diepmsk = s->doepmsk = s->daintmsk = s->diepempmsk = 0;
    memset(s->dieptxf, 0, sizeof(s->dieptxf));
    memset(s->diepctl, 0, sizeof(s->diepctl));
    memset(s->diepint, 0, sizeof(s->diepint));
    memset(s->dieptsiz, 0, sizeof(s->dieptsiz));
    memset(s->doepctl, 0, sizeof(s->doepctl));
    memset(s->doepint, 0, sizeof(s->doepint));
    memset(s->doeptsiz, 0, sizeof(s->doeptsiz));
    memset(s->txlen, 0, sizeof(s->txlen));
    memset(s->out_pending, 0, sizeof(s->out_pending));
    s->rx_head = s->rx_count = 0;
    s->cur_pos = 0;
    s->cur.len = 0;
    s->goutnak = s->ginnak = false;
    s->host = HOST_DETACHED;
    s->host_rx_len = 0;
}

/* Host side: consumes every IN packet the device sends. */
static void host_in_packet(BrambleUsbOtgState *s, int ep, const uint8_t *data, uint16_t len)
{
    if (ep == 0) {
        if (s->phase == CTRL_DATA_IN) {
            uint16_t room = sizeof(s->in_buf) - s->in_len;
            uint16_t take = len < room ? len : room;
            memcpy(s->in_buf + s->in_len, data, take);
            s->in_len += take;
            uint16_t want = s->setup[6] | (s->setup[7] << 8);
            if (len < PKT_MAX || s->in_len >= want) {
                s->phase = CTRL_STATUS_OUT;
            }
        } else if (s->phase == CTRL_STATUS_IN && len == 0) {
            s->phase = CTRL_DONE;
        }
        return;
    }
    if (s->trace && ep && len) {
        fprintf(stderr, "bramble-usbotg: IN ep%d \"", ep);
        for (int i = 0; i < len; i++) {
            fprintf(stderr, (data[i] >= 32 && data[i] < 127) ? "%c" : "\\x%02x", data[i]);
        }
        fprintf(stderr, "\"\n");
    }
    if (s->host == HOST_CONFIGURED && ep == s->bulk_in && len) {
        qemu_chr_fe_write_all(&s->chr, data, len);
    }
}

static void in_ep_progress(BrambleUsbOtgState *s, int n)
{
    if (!(s->diepctl[n] & EPCTL_EPENA)) {
        return;
    }
    uint32_t xfer = tsiz_xfer(s->dieptsiz[n]);
    uint32_t pkt = tsiz_pkt(s->dieptsiz[n]);
    unsigned mps = ep_mps(s, n, true);
    if (xfer == 0) {
        host_in_packet(s, n, NULL, 0);
        s->dieptsiz[n] = 0;
        s->diepctl[n] &= ~EPCTL_EPENA;
        s->diepint[n] |= EPINT_XFERCOMPL;
        return;
    }
    unsigned plen = xfer < mps ? xfer : mps;
    if (s->txlen[n] < plen) {
        return;
    }
    host_in_packet(s, n, s->tx[n], plen);
    s->txlen[n] = 0;
    xfer -= plen;
    pkt = pkt ? pkt - 1 : 0;
    s->dieptsiz[n] = tsiz_make(pkt, xfer, 0);
    if (xfer == 0) {
        s->diepctl[n] &= ~EPCTL_EPENA;
        s->diepint[n] |= EPINT_XFERCOMPL;
    }
}

static bool out_ep_ready(BrambleUsbOtgState *s, int n)
{
    return (s->doepctl[n] & EPCTL_EPENA) && !(s->doepctl[n] & EPCTL_STALL) && !s->out_pending[n];
}

static void out_deliver(BrambleUsbOtgState *s, int n, const uint8_t *data, uint16_t len)
{
    if (n) {
        usbotg_log(s, "OUT ep%d deliver %u", n, len);
    }
    rxq_push(s, n, PKTSTS_OUT_DATA, data, len);
    rxq_push(s, n, PKTSTS_OUT_DONE, NULL, 0);
    s->out_pending[n] = true;
}

static void host_send_setup(BrambleUsbOtgState *s, const uint8_t setup[8], const uint8_t *data, uint16_t len)
{
    memcpy(s->setup, setup, 8);
    s->out_len = len;
    if (len) {
        memcpy(s->out_data, data, len);
    }
    s->in_len = 0;
    s->diepctl[0] &= ~EPCTL_EPENA;
    s->txlen[0] = 0;
    rxq_push(s, 0, PKTSTS_SETUP_DATA, setup, 8);
    rxq_push(s, 0, PKTSTS_SETUP_DONE, NULL, 0);
    uint16_t wlen = setup[6] | (setup[7] << 8);
    if (setup[0] & 0x80) {
        s->phase = wlen ? CTRL_DATA_IN : CTRL_STATUS_IN;
    } else {
        s->phase = wlen ? CTRL_DATA_OUT : CTRL_STATUS_IN;
    }
    usbotg_log(s, "setup %02x %02x %02x%02x %02x%02x %02x%02x", setup[0], setup[1], setup[3], setup[2],
               setup[5], setup[4], setup[7], setup[6]);
}

static void parse_config(BrambleUsbOtgState *s)
{
    uint16_t pos = 0;
    int cur_class = -1;
    s->cfg_value = s->in_len > 5 ? s->in_buf[5] : 1;
    while (pos + 2 <= s->in_len) {
        uint8_t blen = s->in_buf[pos], type = s->in_buf[pos + 1];
        if (blen < 2 || pos + blen > s->in_len) {
            break;
        }
        if (type == 4 && blen >= 9) {
            cur_class = s->in_buf[pos + 5];
            if (cur_class == 0x02) {
                s->cdc_iface = s->in_buf[pos + 2];
            }
        } else if (type == 5 && blen >= 7 && cur_class == 0x0a && (s->in_buf[pos + 3] & 3) == 2) {
            uint8_t addr = s->in_buf[pos + 2];
            if (addr & 0x80) {
                s->bulk_in = addr & 0x0f;
            } else {
                s->bulk_out = addr & 0x0f;
                s->bulk_out_mps = s->in_buf[pos + 4] | (s->in_buf[pos + 5] << 8);
            }
        }
        pos += blen;
    }
    usbotg_log(s, "config value %u, cdc iface %u, bulk in %u out %u", s->cfg_value, s->cdc_iface,
               s->bulk_in, s->bulk_out);
}

/* Returns true when the enumeration script has another request to issue. */
static bool host_next_request(BrambleUsbOtgState *s)
{
    uint8_t setup[8] = { 0 };
    static const uint8_t line_coding[7] = { 0x00, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x08 };
    switch (s->step) {
    case 0: /* GET_DESCRIPTOR device */
        memcpy(setup, (uint8_t[]){ 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00 }, 8);
        break;
    case 1: /* SET_ADDRESS 1 */
        memcpy(setup, (uint8_t[]){ 0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 }, 8);
        break;
    case 2: /* GET_DESCRIPTOR config header */
        memcpy(setup, (uint8_t[]){ 0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x09, 0x00 }, 8);
        break;
    case 3: /* GET_DESCRIPTOR config full */
        memcpy(setup, (uint8_t[]){ 0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00 }, 8);
        setup[6] = s->cfg_total & 0xff;
        setup[7] = s->cfg_total >> 8;
        break;
    case 4: /* SET_CONFIGURATION */
        memcpy(setup, (uint8_t[]){ 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 8);
        setup[2] = s->cfg_value;
        break;
    case 5: /* CDC SET_LINE_CODING 115200 8N1 */
        memcpy(setup, (uint8_t[]){ 0x21, 0x20, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00 }, 8);
        setup[4] = s->cdc_iface;
        host_send_setup(s, setup, line_coding, sizeof(line_coding));
        return true;
    case 6: /* CDC SET_CONTROL_LINE_STATE DTR on, RTS off */
        memcpy(setup, (uint8_t[]){ 0x21, 0x22, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 }, 8);
        setup[4] = s->cdc_iface;
        break;
    default:
        return false;
    }
    host_send_setup(s, setup, NULL, 0);
    return true;
}

static void host_request_done(BrambleUsbOtgState *s)
{
    if (s->step == 2 && s->in_len >= 4) {
        s->cfg_total = s->in_buf[2] | (s->in_buf[3] << 8);
        if (s->cfg_total > sizeof(s->in_buf)) {
            s->cfg_total = sizeof(s->in_buf);
        }
    } else if (s->step == 3) {
        parse_config(s);
        if (!s->bulk_in || !s->bulk_out) {
            fprintf(stderr, "bramble-usbotg: no CDC data interface in the configuration descriptor\n");
            s->host = HOST_FAILED;
            return;
        }
    }
    s->step++;
    s->phase = CTRL_IDLE;
    s->host_wait_until = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + HOST_GAP_NS;
}

static void host_run(BrambleUsbOtgState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool connected = !(s->dctl & DCTL_SFTDISCON) && (s->gintmsk & GINT_USBRST);

    if (!connected) {
        if (s->host != HOST_DETACHED) {
            usbotg_log(s, "device disconnected");
        }
        s->host = HOST_DETACHED;
        return;
    }
    if (now < s->host_wait_until) {
        return;
    }
    switch (s->host) {
    case HOST_DETACHED:
        usbotg_log(s, "bus reset");
        s->gintsts |= GINT_USBRST;
        s->host = HOST_RESET;
        s->host_wait_until = now + HOST_GAP_NS;
        break;
    case HOST_RESET:
        s->gintsts |= GINT_ENUMDONE;
        s->host = HOST_ENUMDONE;
        s->host_wait_until = now + 5 * HOST_GAP_NS;
        break;
    case HOST_ENUMDONE:
        s->step = 0;
        s->phase = CTRL_IDLE;
        s->cfg_total = 9;
        s->bulk_in = s->bulk_out = 0;
        s->host = HOST_CONTROL;
        break;
    case HOST_CONTROL:
        switch (s->phase) {
        case CTRL_IDLE:
            if (!host_next_request(s)) {
                fprintf(stderr, "bramble-usbotg: CDC enumerated, DTR set; console bridged\n");
                s->host = HOST_CONFIGURED;
            }
            break;
        case CTRL_DATA_OUT:
            if (out_ep_ready(s, 0)) {
                out_deliver(s, 0, s->out_data, s->out_len);
                s->phase = CTRL_STATUS_IN;
            }
            break;
        case CTRL_STATUS_OUT:
            if (out_ep_ready(s, 0)) {
                out_deliver(s, 0, NULL, 0);
                s->phase = CTRL_WAIT_OUT_DONE;
            }
            break;
        case CTRL_WAIT_OUT_DONE:
            if (!s->out_pending[0]) {
                s->phase = CTRL_DONE;
            }
            break;
        case CTRL_DONE:
            host_request_done(s);
            break;
        default:
            break;
        }
        break;
    case HOST_CONFIGURED:
        if (s->host_rx_len && out_ep_ready(s, s->bulk_out)) {
            uint16_t mps = s->bulk_out_mps ? s->bulk_out_mps : PKT_MAX;
            uint16_t len = s->host_rx_len < mps ? s->host_rx_len : mps;
            out_deliver(s, s->bulk_out, s->host_rx, len);
            memmove(s->host_rx, s->host_rx + len, s->host_rx_len - len);
            s->host_rx_len -= len;
            qemu_chr_fe_accept_input(&s->chr);
        }
        break;
    default:
        break;
    }
}

static void usbotg_step(void *opaque)
{
    BrambleUsbOtgState *s = opaque;
    for (int n = 0; n < NUM_EP; n++) {
        in_ep_progress(s, n);
    }
    host_run(s);
    usbotg_update_irq(s);
    if (s->host != HOST_DETACHED && s->host != HOST_FAILED) {
        usbotg_kick(s);
    }
}

static uint64_t usbotg_read(void *opaque, hwaddr addr, unsigned size)
{
    BrambleUsbOtgState *s = opaque;
    uint32_t v = 0;

    if (addr >= R_FIFO_BASE && addr < R_FIFO_END) {
        for (int i = 0; i < 4; i++) {
            uint8_t b = s->cur_pos < s->cur.len ? s->cur.data[s->cur_pos] : 0;
            v |= (uint32_t)b << (8 * i);
            s->cur_pos++;
        }
        return v;
    }
    if (addr >= R_INEP_BASE && addr < R_INEP_BASE + NUM_EP * R_EP_STRIDE) {
        int n = (addr - R_INEP_BASE) / R_EP_STRIDE;
        switch ((addr - R_INEP_BASE) % R_EP_STRIDE) {
        case 0x00: return s->diepctl[n];
        case 0x08: usbotg_update_irq(s); return s->diepint[n];
        case 0x10: return s->dieptsiz[n];
        case 0x18: return 0x100;
        default: return 0;
        }
    }
    if (addr >= R_OUTEP_BASE && addr < R_OUTEP_BASE + NUM_EP * R_EP_STRIDE) {
        int n = (addr - R_OUTEP_BASE) / R_EP_STRIDE;
        switch ((addr - R_OUTEP_BASE) % R_EP_STRIDE) {
        case 0x00: return s->doepctl[n];
        case 0x08: return s->doepint[n];
        case 0x10: return s->doeptsiz[n];
        default: return 0;
        }
    }
    if (addr >= 0x104 && addr < 0x114) {
        return s->dieptxf[(addr - 0x104) / 4];
    }
    switch (addr) {
    case R_GOTGCTL:   return (1u << 16) | (1u << 18) | (1u << 19);
    case R_GOTGINT:   return s->gotgint;
    case R_GAHBCFG:   return s->gahbcfg;
    case R_GUSBCFG:   return s->gusbcfg;
    case R_GRSTCTL:   return s->grstctl | GRST_AHBIDL;
    case R_GINTSTS:   return gintsts_value(s);
    case R_GINTMSK:   return s->gintmsk;
    case R_GRXSTSR:   return s->rx_count ? s->rxq[s->rx_head].status : 0;
    case R_GRXSTSP:
        if (!s->rx_count) {
            return 0;
        } else {
            RxEntry *e = &s->rxq[s->rx_head];
            int ep = e->status & 0xf;
            int pktsts = (e->status >> 17) & 0xf;
            s->cur = *e;
            s->cur_pos = 0;
            s->rx_head = (s->rx_head + 1) % RXQ_LEN;
            s->rx_count--;
            if (pktsts == PKTSTS_SETUP_DONE) {
                s->doepint[ep] |= DOEPINT_SETUP | DOEPINT_STUPRCV;
                s->doepctl[ep] &= ~EPCTL_EPENA;
            } else if (pktsts == PKTSTS_OUT_DONE) {
                uint32_t xfer = tsiz_xfer(s->doeptsiz[ep]);
                uint32_t pkt = tsiz_pkt(s->doeptsiz[ep]);
                s->doeptsiz[ep] = tsiz_make(pkt ? pkt - 1 : 0, xfer, s->doeptsiz[ep]);
                s->doepctl[ep] &= ~EPCTL_EPENA;
                s->doepint[ep] |= EPINT_XFERCOMPL;
                s->out_pending[ep] = false;
            }
            usbotg_kick(s);
            return e->status;
        }
    case R_GRXFSIZ:   return s->grxfsiz;
    case R_GNPTXFSIZ: return s->gnptxfsiz;
    case R_GNPTXSTS:  return 0x00080100;
    case R_GSNPSID:   return 0x4f54400a;
    case R_GHWCFG1:   return 0;
    case R_GHWCFG2:   return 0x224dd930;
    case R_GHWCFG3:   return 0x00c804b5;
    case R_GHWCFG4:   return 0xd3f0a030;
    case R_DCFG:      return s->dcfg;
    case R_DCTL:      return s->dctl;
    case R_DSTS:      return (3u << 1) | ((uint32_t)((qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) & 0x3fff) << 8));
    case R_DIEPMSK:   return s->diepmsk;
    case R_DOEPMSK:   return s->doepmsk;
    case R_DAINT:     return daint_value(s);
    case R_DAINTMSK:  return s->daintmsk;
    case R_DIEPEMPMSK: return s->diepempmsk;
    default:          return 0;
    }
}

static void usbotg_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    BrambleUsbOtgState *s = opaque;
    uint32_t v = value;

    if (addr >= R_FIFO_BASE && addr < R_FIFO_END) {
        int n = (addr - R_FIFO_BASE) / 0x1000;
        if (n < NUM_EP && s->txlen[n] + 4 <= sizeof(s->tx[n])) {
            memcpy(&s->tx[n][s->txlen[n]], &v, 4);
            s->txlen[n] += 4;
        }
        usbotg_kick(s);
        return;
    }
    if (addr >= R_INEP_BASE && addr < R_INEP_BASE + NUM_EP * R_EP_STRIDE) {
        int n = (addr - R_INEP_BASE) / R_EP_STRIDE;
        switch ((addr - R_INEP_BASE) % R_EP_STRIDE) {
        case 0x00:
            if (v & EPCTL_SNAK) {
                s->diepctl[n] |= EPCTL_NAKSTS;
            }
            if (v & EPCTL_CNAK) {
                s->diepctl[n] &= ~EPCTL_NAKSTS;
            }
            s->diepctl[n] = (v & ~(EPCTL_WO | EPCTL_NAKSTS | EPCTL_EPDIS)) | (s->diepctl[n] & EPCTL_NAKSTS);
            if (v & EPCTL_EPDIS) {
                s->diepctl[n] &= ~EPCTL_EPENA;
                s->diepint[n] |= EPINT_EPDISBLD;
            }
            if (v & EPCTL_EPENA) {
                s->txlen[n] = 0;
            }
            break;
        case 0x08: s->diepint[n] &= ~(v & ~DIEPINT_TXFEMP); break;
        case 0x10: s->dieptsiz[n] = v; break;
        default: break;
        }
        usbotg_kick(s);
        return;
    }
    if (addr >= R_OUTEP_BASE && addr < R_OUTEP_BASE + NUM_EP * R_EP_STRIDE) {
        int n = (addr - R_OUTEP_BASE) / R_EP_STRIDE;
        switch ((addr - R_OUTEP_BASE) % R_EP_STRIDE) {
        case 0x00:
            if (v & EPCTL_SNAK) {
                s->doepctl[n] |= EPCTL_NAKSTS;
            }
            if (v & EPCTL_CNAK) {
                s->doepctl[n] &= ~EPCTL_NAKSTS;
            }
            s->doepctl[n] = (v & ~(EPCTL_WO | EPCTL_NAKSTS | EPCTL_EPDIS)) | (s->doepctl[n] & EPCTL_NAKSTS);
            if (v & EPCTL_EPDIS) {
                s->doepctl[n] &= ~EPCTL_EPENA;
                s->doepint[n] |= EPINT_EPDISBLD;
            }
            break;
        case 0x08: s->doepint[n] &= ~v; break;
        case 0x10: s->doeptsiz[n] = v; break;
        default: break;
        }
        usbotg_kick(s);
        return;
    }
    if (addr >= 0x104 && addr < 0x114) {
        s->dieptxf[(addr - 0x104) / 4] = v;
        return;
    }
    switch (addr) {
    case R_GOTGINT:   s->gotgint &= ~v; break;
    case R_GAHBCFG:   s->gahbcfg = v; break;
    case R_GUSBCFG:   s->gusbcfg = v; break;
    case R_GRSTCTL:
        if (v & GRST_CSFTRST) {
            core_reset(s);
        }
        if (v & GRST_RXFFLSH) {
            s->rx_count = 0;
        }
        if (v & GRST_TXFFLSH) {
            memset(s->txlen, 0, sizeof(s->txlen));
        }
        s->grstctl = v & ~(GRST_CSFTRST | GRST_RXFFLSH | GRST_TXFFLSH | GRST_AHBIDL);
        break;
    case R_GINTSTS:   s->gintsts &= ~(v & ~GINT_DERIVED); break;
    case R_GINTMSK:   s->gintmsk = v; break;
    case R_GRXFSIZ:   s->grxfsiz = v; break;
    case R_GNPTXFSIZ: s->gnptxfsiz = v; break;
    case R_DCFG:      s->dcfg = v; break;
    case R_DCTL:
        if (v & DCTL_SGOUTNAK) {
            s->goutnak = true;
        }
        if (v & DCTL_CGOUTNAK) {
            s->goutnak = false;
        }
        if (v & DCTL_SGNPINNAK) {
            s->ginnak = true;
        }
        if (v & DCTL_CGNPINNAK) {
            s->ginnak = false;
        }
        s->dctl = v & ~DCTL_WO;
        break;
    case R_DIEPMSK:   s->diepmsk = v; break;
    case R_DOEPMSK:   s->doepmsk = v; break;
    case R_DAINTMSK:  s->daintmsk = v; break;
    case R_DIEPEMPMSK: s->diepempmsk = v; break;
    default: break;
    }
    usbotg_kick(s);
}

static const MemoryRegionOps usbotg_ops = {
    .read = usbotg_read,
    .write = usbotg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t usbwrap_read(void *opaque, hwaddr addr, unsigned size)
{
    BrambleUsbOtgState *s = opaque;
    return s->wrap_regs[(addr & (USBWRAP_WINDOW - 1)) / 4];
}

static void usbwrap_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    BrambleUsbOtgState *s = opaque;
    s->wrap_regs[(addr & (USBWRAP_WINDOW - 1)) / 4] = value;
}

static const MemoryRegionOps usbwrap_ops = {
    .read = usbwrap_read,
    .write = usbwrap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static int usbotg_can_receive(void *opaque)
{
    BrambleUsbOtgState *s = opaque;
    return s->host == HOST_CONFIGURED ? HOST_RX_MAX - s->host_rx_len : 0;
}

static void usbotg_receive(void *opaque, const uint8_t *buf, int size)
{
    BrambleUsbOtgState *s = opaque;
    int take = MIN(size, HOST_RX_MAX - s->host_rx_len);
    memcpy(s->host_rx + s->host_rx_len, buf, take);
    s->host_rx_len += take;
    usbotg_kick(s);
}

static void usbotg_instance_init(Object *obj)
{
    BrambleUsbOtgState *s = BRAMBLE_USBOTG(obj);
    memory_region_init_io(&s->iomem, obj, &usbotg_ops, s, TYPE_BRAMBLE_USBOTG, USBOTG_WINDOW);
    memory_region_init_io(&s->wrap, obj, &usbwrap_ops, s, "bramble.usbwrap", USBWRAP_WINDOW);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, usbotg_step, s);
    s->trace = getenv("BRAMBLE_USBOTG_TRACE") != NULL;
    core_reset(s);
}

static const TypeInfo usbotg_info = {
    .name = TYPE_BRAMBLE_USBOTG,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(BrambleUsbOtgState),
    .instance_init = usbotg_instance_init,
};

static void usbotg_register_types(void)
{
    type_register_static(&usbotg_info);
}

type_init(usbotg_register_types)

void bramble_usbotg_attach(MemoryRegion *sys_mem, DeviceState *intc)
{
    Object *obj = object_new(TYPE_BRAMBLE_USBOTG);
    BrambleUsbOtgState *s = BRAMBLE_USBOTG(obj);

    bramble_overlay_attach(obj, "bramble-usbotg", &s->iomem, sys_mem, USBOTG_BASE,
                           "bramble-usbotg: OTG device controller + CDC host");
    memory_region_add_subregion_overlap(sys_mem, USBWRAP_BASE, &s->wrap, 1);
    if (intc) {
        s->irq = qdev_get_gpio_in(intc, USB_INTR_SOURCE);
    }
    Chardev *chr = serial_hd(3);
    if (chr) {
        qemu_chr_fe_init(&s->chr, chr, &error_abort);
        qemu_chr_fe_set_handlers(&s->chr, usbotg_can_receive, usbotg_receive, NULL, NULL, s, NULL, true);
    }
    usbotg_kick(s);
}
