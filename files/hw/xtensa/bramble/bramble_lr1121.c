/* Bramble LR1121 radio for RadioLib's LR11x0 driver; framing per ternmesh PR #72 and README "The models". */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-core.h"
#include "qom/object.h"
#include "hw/ssi/ssi.h"
#include "qemu/base64.h"
#include "qapi/qmp/qdict.h"
#include "hw/xtensa/bramble_lr1121.h"
#include "hw/xtensa/bramble_gpio.h"
#include "hw/xtensa/bramble_emulink.h"

/* Opcodes RadioLib's LR11x0 driver sends (LR11x0_commands.h). */
#define LR_GET_STATUS          0x0100
#define LR_GET_VERSION         0x0101
#define LR_READ_REG_MEM        0x0106
#define LR_WRITE_BUFFER        0x0109
#define LR_READ_BUFFER         0x010A
#define LR_CLEAR_RX_BUFFER     0x010B
#define LR_GET_ERRORS          0x010D
#define LR_CLEAR_ERRORS        0x010E
#define LR_SET_DIO_IRQ_PARAMS  0x0113
#define LR_CLEAR_IRQ           0x0114
#define LR_GET_VBAT            0x0119
#define LR_GET_TEMP            0x011A
#define LR_SET_SLEEP           0x011B
#define LR_SET_STANDBY         0x011C
#define LR_SET_FS              0x011D
#define LR_GET_RANDOM_NUMBER   0x0120
#define LR_GET_STATS           0x0201
#define LR_GET_PACKET_TYPE     0x0202
#define LR_GET_RX_BUFFER_STATUS 0x0203
#define LR_GET_PACKET_STATUS   0x0204
#define LR_GET_RSSI_INST       0x0205
#define LR_SET_RX              0x0209
#define LR_SET_TX              0x020A
#define LR_SET_RF_FREQUENCY    0x020B
#define LR_SET_PACKET_TYPE     0x020E
#define LR_SET_MODULATION_PARAMS 0x020F
#define LR_SET_PACKET_PARAMS   0x0210
#define LR_SET_TX_PARAMS       0x0211
#define LR_SET_CAD             0x0218
#define LR_SET_RX_BOOSTED      0x0227
#define LR_GET_LORA_RX_HEADER_INFOS 0x0230

#define LR_IRQ_TX_DONE         (1u << 2)
#define LR_IRQ_RX_DONE         (1u << 3)

/* Stat2 chip modes (manual table 3-2). */
#define LR_MODE_SLEEP          0
#define LR_MODE_STBY_RC        1
#define LR_MODE_FS             3
#define LR_MODE_RX             4
#define LR_MODE_TX             5

#define LR_STAT1_CMD_OK        (2u << 1)
#define LR_DEVICE_LR1121       0x03
#define LR_FW_VERSION          0x0103

#define LR_ARGS_MAX            300
#define LR_BUFFER_SIZE         256
#define LR_RX_FIFO_DEPTH       8
#define LR_DIO9_GPIO           38
/* Deferred RX presentation as in bramble_sx1262.c: RX_DONE never rises inside SetRx. */
#define LR_PRESENT_DELAY_NS    1000000

#define TRACE(fmt, ...) do { \
    if (trace_enabled()) { \
        fprintf(stderr, "bramble-lr1121: " fmt "\n", ##__VA_ARGS__); \
    } \
} while (0)

static bool trace_enabled(void)
{
    static int enabled = -1;
    if (enabled < 0) {
        const char *value = getenv("BRAMBLE_LR_TRACE");
        enabled = (value && *value && *value != '0') ? 1 : 0;
    }
    return enabled;
}

typedef struct {
    uint8_t data[255];
    uint8_t len;
    uint8_t rssi_raw;   /* GetPacketStatus byte: rssi = -raw/2 */
    int8_t snr_raw;     /* GetPacketStatus byte: snr = raw/4 */
} Lr1121RxFrame;

OBJECT_DECLARE_SIMPLE_TYPE(BrambleLr1121State, BRAMBLE_LR1121)

struct BrambleLr1121State {
    SSIPeripheral parent_obj;

    uint8_t mode;
    uint32_t irq_status;
    uint32_t dio9_mask;
    uint8_t packet_type;
    uint8_t buffer[LR_BUFFER_SIZE];
    uint8_t tx_len;
    uint8_t rx_len;

    int freq_mhz;
    int sf;
    int bw_hz;
    int cr;
    int power;

    /* The current transaction: opcode and argument bytes in, or an answer/status out. */
    uint32_t byte_idx;
    bool is_command;
    uint16_t opcode;
    uint8_t args[LR_ARGS_MAX];
    uint32_t nargs;

    /* The answer the next all-NOP transaction clocks out after Stat1. */
    uint8_t answer[LR_ARGS_MAX];
    uint32_t answer_len;
    bool answer_pending;

    Lr1121RxFrame rx_fifo[LR_RX_FIFO_DEPTH];
    int rx_fifo_head;
    int rx_fifo_count;
    Lr1121RxFrame rx_cur;
    bool rx_active;
    bool dio9_level;
    uint32_t dio9_gpio;
    QEMUTimer *present_timer;
    uint32_t random_state;
};

static uint8_t lr_stat1(BrambleLr1121State *s)
{
    return (uint8_t)(LR_STAT1_CMD_OK | ((s->irq_status & s->dio9_mask) ? 1 : 0));
}

static uint8_t lr_stat2(BrambleLr1121State *s)
{
    return (uint8_t)((s->mode << 1) | 1);
}

static void lr_update_dio9(BrambleLr1121State *s)
{
    bool level = (s->irq_status & s->dio9_mask) != 0;
    if (level != s->dio9_level) {
        s->dio9_level = level;
        bramble_gpio_set_input(s->dio9_gpio, level);
    }
}

static void lr_raise(BrambleLr1121State *s, uint32_t flags)
{
    s->irq_status |= flags;
    lr_update_dio9(s);
}

static void lr_answer(BrambleLr1121State *s, const uint8_t *bytes, uint32_t len)
{
    memset(s->answer, 0, sizeof(s->answer));
    if (bytes && len) {
        memcpy(s->answer, bytes, MIN(len, (uint32_t)sizeof(s->answer)));
    }
    s->answer_len = len;
    s->answer_pending = true;
}

static uint32_t lr_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/* ---- RX presentation, mirroring bramble_sx1262.c ---------------------------------------------- */

static void lr_rx_present(BrambleLr1121State *s)
{
    if (s->mode != LR_MODE_RX || s->rx_active || s->rx_fifo_count == 0) {
        return;
    }
    s->rx_cur = s->rx_fifo[s->rx_fifo_head];
    s->rx_fifo_head = (s->rx_fifo_head + 1) % LR_RX_FIFO_DEPTH;
    s->rx_fifo_count--;
    memcpy(s->buffer, s->rx_cur.data, s->rx_cur.len);
    s->rx_len = s->rx_cur.len;
    s->rx_active = true;
    TRACE("present: len=%u", s->rx_len);
    lr_raise(s, LR_IRQ_RX_DONE);
}

static void lr_rx_rearm(BrambleLr1121State *s)
{
    if (s->rx_active) {
        if (!(s->irq_status & LR_IRQ_RX_DONE)) {
            memcpy(s->buffer, s->rx_cur.data, s->rx_cur.len);
            s->rx_len = s->rx_cur.len;
            lr_raise(s, LR_IRQ_RX_DONE);
        }
    } else {
        lr_rx_present(s);
    }
}

static void lr_present_deferred(void *opaque)
{
    lr_rx_rearm((BrambleLr1121State *)opaque);
}

static void lr_on_rx(QDict *msg, void *ctx)
{
    BrambleLr1121State *s = ctx;
    const char *payload = qdict_get_try_str(msg, "payload");
    if (!payload) {
        return;
    }
    size_t n = 0;
    g_autofree uint8_t *decoded = qbase64_decode(payload, -1, &n, NULL);
    if (!decoded || n == 0 || n > sizeof(s->rx_fifo[0].data)) {
        return;
    }
    int rssi = (int)qdict_get_try_int(msg, "rssi", -100);
    int snr = (int)qdict_get_try_int(msg, "snr", 0);
    if (s->rx_fifo_count == LR_RX_FIFO_DEPTH) {
        fprintf(stderr, "bramble-lr1121: RX FIFO full (%d), dropping oldest frame\n",
                LR_RX_FIFO_DEPTH);
        s->rx_fifo_head = (s->rx_fifo_head + 1) % LR_RX_FIFO_DEPTH;
        s->rx_fifo_count--;
    }
    Lr1121RxFrame *f = &s->rx_fifo[(s->rx_fifo_head + s->rx_fifo_count) % LR_RX_FIFO_DEPTH];
    memcpy(f->data, decoded, n);
    f->len = (uint8_t)n;
    f->rssi_raw = (uint8_t)MIN(MAX(-2 * rssi, 0), 255);
    f->snr_raw = (int8_t)MIN(MAX(4 * snr, -128), 127);
    s->rx_fifo_count++;
    lr_rx_present(s);
}

static void lr_on_txdone(QDict *msg, void *ctx)
{
    (void)msg;
    lr_raise((BrambleLr1121State *)ctx, LR_IRQ_TX_DONE);
}

/* ---- command execution (CS rising edge) ------------------------------------------------------- */

static void lr_execute(BrambleLr1121State *s)
{
    const uint8_t *a = s->args;
    uint32_t n = s->nargs;
    uint8_t out[LR_ARGS_MAX];

    {
        static uint8_t seen[0x400];
        if (s->opcode < sizeof(seen) * 8 && !(seen[s->opcode / 8] & (1 << (s->opcode % 8)))) {
            seen[s->opcode / 8] |= 1 << (s->opcode % 8);
            qemu_log("LR1121 newop=0x%04x nargs=%u\n", s->opcode, n);
        }
    }
    TRACE("cmd 0x%04x nargs=%u", s->opcode, n);

    switch (s->opcode) {
    case LR_GET_STATUS:
        out[0] = lr_stat2(s);
        out[1] = s->irq_status >> 24;
        out[2] = s->irq_status >> 16;
        out[3] = s->irq_status >> 8;
        out[4] = s->irq_status;
        lr_answer(s, out, 5);
        break;
    case LR_GET_VERSION:
        out[0] = 0x22;
        out[1] = LR_DEVICE_LR1121;
        out[2] = LR_FW_VERSION >> 8;
        out[3] = LR_FW_VERSION & 0xff;
        lr_answer(s, out, 4);
        break;
    case LR_READ_REG_MEM:
        /* [addr32][words]: registers read back as zero. */
        lr_answer(s, NULL, n >= 5 ? 4u * a[4] : 0);
        break;
    case LR_GET_ERRORS:
        lr_answer(s, NULL, 2);
        break;
    case LR_GET_VBAT:
        out[0] = 0xC0;
        lr_answer(s, out, 1);
        break;
    case LR_GET_TEMP:
        out[0] = 0x02;
        out[1] = 0x80;
        lr_answer(s, out, 2);
        break;
    case LR_GET_RANDOM_NUMBER:
        s->random_state = s->random_state * 1664525u + 1013904223u;
        out[0] = s->random_state >> 24;
        out[1] = s->random_state >> 16;
        out[2] = s->random_state >> 8;
        out[3] = s->random_state;
        lr_answer(s, out, 4);
        break;
    case LR_GET_STATS:
        lr_answer(s, NULL, 8);
        break;
    case LR_GET_PACKET_TYPE:
        out[0] = s->packet_type;
        lr_answer(s, out, 1);
        break;
    case LR_GET_RX_BUFFER_STATUS:
        out[0] = s->rx_len;
        out[1] = 0;
        lr_answer(s, out, 2);
        break;
    case LR_GET_PACKET_STATUS:
        out[0] = s->rx_cur.rssi_raw;
        out[1] = (uint8_t)s->rx_cur.snr_raw;
        out[2] = s->rx_cur.rssi_raw;
        lr_answer(s, out, 3);
        /* The last read of an RX drain: hand over the next queued frame. */
        if (s->rx_active) {
            s->rx_active = false;
            lr_rx_present(s);
        }
        break;
    case LR_GET_RSSI_INST:
        out[0] = 220; /* -110 dBm: a quiet channel */
        lr_answer(s, out, 1);
        break;
    case LR_GET_LORA_RX_HEADER_INFOS:
        out[0] = 0x10; /* CRC on */
        lr_answer(s, out, 1);
        break;
    case LR_READ_BUFFER:
        /* [offset][length] */
        if (n >= 2) {
            uint32_t off = a[0], len = a[1];
            for (uint32_t i = 0; i < len; i++) {
                out[i] = s->buffer[(off + i) % LR_BUFFER_SIZE];
            }
            lr_answer(s, out, len);
        }
        break;
    case LR_WRITE_BUFFER:
        memcpy(s->buffer, a, MIN(n, (uint32_t)LR_BUFFER_SIZE));
        break;
    case LR_CLEAR_RX_BUFFER:
        memset(s->buffer, 0, sizeof(s->buffer));
        break;
    case LR_SET_DIO_IRQ_PARAMS:
        if (n >= 4) {
            s->dio9_mask = lr_be32(a);
            lr_update_dio9(s);
        }
        break;
    case LR_CLEAR_IRQ:
        if (n >= 4) {
            s->irq_status &= ~lr_be32(a);
            lr_update_dio9(s);
        }
        break;
    case LR_SET_SLEEP:
        s->mode = LR_MODE_SLEEP;
        break;
    case LR_SET_STANDBY:
        s->mode = LR_MODE_STBY_RC;
        break;
    case LR_SET_FS:
        s->mode = LR_MODE_FS;
        break;
    case LR_SET_PACKET_TYPE:
        if (n >= 1) {
            s->packet_type = a[0];
        }
        break;
    case LR_SET_RF_FREQUENCY:
        if (n >= 4) {
            s->freq_mhz = (int)((lr_be32(a) + 500000u) / 1000000u);
        }
        break;
    case LR_SET_MODULATION_PARAMS:
        /* [sf][bw][cr][ldro]; bw codes per the user manual, 0x0D-0x0F being the 2.4 GHz widths. */
        if (n >= 3) {
            static const int bw[16] = { 0, 15625, 31250, 62500, 125000, 250000, 500000, 0,
                                        10417, 20833, 41667, 0, 0, 203125, 406250, 812500 };
            s->sf = a[0];
            s->bw_hz = bw[a[1] & 0x0f] ? bw[a[1] & 0x0f] : 125000;
            s->cr = a[2];
        }
        break;
    case LR_SET_PACKET_PARAMS:
        /* LoRa: [preamble16][header][payload length][crc][iq] */
        if (n >= 4) {
            s->tx_len = a[3];
        }
        break;
    case LR_SET_TX_PARAMS:
        if (n >= 1) {
            s->power = (int8_t)a[0];
        }
        break;
    case LR_SET_TX:
        s->mode = LR_MODE_TX;
        if (s->tx_len > 0) {
            emulink_send_tx(s->buffer, s->tx_len, s->freq_mhz, s->sf, s->bw_hz, s->cr, s->power);
        }
        break;
    case LR_SET_RX:
    case LR_SET_CAD:
        s->mode = LR_MODE_RX;
        timer_mod(s->present_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + LR_PRESENT_DELAY_NS);
        break;
    default:
        /* Configuration the model does not need to keep: accepted. */
        break;
    }
}

/* ---- SSI ---------------------------------------------------------------------------------------- */

static int bramble_lr1121_set_cs(SSIPeripheral *dev, bool level)
{
    BrambleLr1121State *s = BRAMBLE_LR1121(dev);
    if (!level) {
        s->byte_idx = 0;
        s->nargs = 0;
        s->is_command = false;
    } else if (s->byte_idx > 0) {
        if (s->is_command) {
            s->answer_pending = false;
            lr_execute(s);
        } else if (s->answer_pending && s->byte_idx > 1) {
            s->answer_pending = false;
        }
    }
    return 0;
}

static uint32_t bramble_lr1121_transfer(SSIPeripheral *dev, uint32_t val)
{
    BrambleLr1121State *s = BRAMBLE_LR1121(dev);
    uint8_t in = val & 0xff;
    uint32_t idx = s->byte_idx++;

    if (idx == 0) {
        /* A command's opcode never has a zero high byte; NOPs start an answer or status read. */
        s->is_command = in != 0;
        s->opcode = (uint16_t)in << 8;
        return lr_stat1(s);
    }
    if (s->is_command) {
        if (idx == 1) {
            s->opcode |= in;
            return lr_stat2(s);
        }
        if (s->nargs < LR_ARGS_MAX) {
            s->args[s->nargs++] = in;
        }
        return 0;
    }
    if (s->answer_pending) {
        return idx - 1 < s->answer_len ? s->answer[idx - 1] : 0;
    }
    /* GetStatus by NOPs: Stat2, then the IRQ flags most significant first. */
    switch (idx) {
    case 1: return lr_stat2(s);
    case 2: return (s->irq_status >> 24) & 0xff;
    case 3: return (s->irq_status >> 16) & 0xff;
    case 4: return (s->irq_status >> 8) & 0xff;
    case 5: return s->irq_status & 0xff;
    default: return 0;
    }
}

static void bramble_lr1121_realize(SSIPeripheral *dev, Error **errp)
{
    BrambleLr1121State *s = BRAMBLE_LR1121(dev);
    (void)errp;
    s->mode = LR_MODE_STBY_RC;
    s->packet_type = 0;
    s->freq_mhz = 915;
    s->sf = 7;
    s->bw_hz = 125000;
    s->cr = 1;
    s->power = 22;
    s->random_state = 0x1121u;
    s->present_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lr_present_deferred, s);
    emulink_on("txdone", lr_on_txdone, s);
    emulink_on("rx", lr_on_rx, s);
}

static Property bramble_lr1121_properties[] = {
    DEFINE_PROP_UINT32("dio9-gpio", BrambleLr1121State, dio9_gpio, LR_DIO9_GPIO),
    DEFINE_PROP_END_OF_LIST(),
};

static void bramble_lr1121_class_init(ObjectClass *klass, void *data)
{
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    device_class_set_props(DEVICE_CLASS(klass), bramble_lr1121_properties);
    k->realize = bramble_lr1121_realize;
    k->transfer = bramble_lr1121_transfer;
    k->set_cs = bramble_lr1121_set_cs;
    k->cs_polarity = SSI_CS_LOW;
}

static const TypeInfo bramble_lr1121_info = {
    .name = TYPE_BRAMBLE_LR1121,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(BrambleLr1121State),
    .class_init = bramble_lr1121_class_init,
};

static void bramble_lr1121_register_types(void)
{
    type_register_static(&bramble_lr1121_info);
}

type_init(bramble_lr1121_register_types)
