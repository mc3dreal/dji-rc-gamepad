/* Enable POSIX + BSD extensions */
#define _GNU_SOURCE

/*
 * DJI RC (RM330) Network Joystick Driver
 *
 * Reads stick/wheel data from a DJI RC smart controller over WiFi
 * by connecting to its internal TCP service on port 40007.
 * Exposes the controller as a virtual joystick via Linux uinput.
 *
 * Protocol:
 *   - TCP connection to port 40007 on the RC's IP
 *   - Controller sends DJI TCP-framed DUML packets on connect
 *   - RC channel data: cmd_set=0x06, cmd_id=0xAE
 *   - Axis data uses byte-interleaved encoding across 16-bit channel boundaries:
 *     position = high byte of ch[N], wrap counter = low byte of ch[N+1]
 *     true_value = (wrap - 4) * 256 + position, range ±660
 *   - Driver maintains a persistent TCP connection with DUML keepalive
 *     commands sent periodically to sustain data flow (~9 Hz vs ~5 Hz passive)
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <poll.h>
#include <linux/uinput.h>
#include <ncurses.h>

/* ── Protocol constants ───────────────────────────────────────────── */

/* TCP transport frame magic bytes */
static const uint8_t TCP_MAGIC[] = { 0x55, 0xCC, 0x30, 0x75 };

/* Port the RM330 streams RC data on */
#define RC_DATA_PORT    40007

/* DUML command identifiers for RC channel data */
#define RC_CMD_SET      0x06
#define RC_CMD_ID       0xAE

/* Maximum channels in a packet */
#define MAX_CHANNELS    16

/* Send keepalive after this many idle read cycles (~20ms each) */
#define KEEPALIVE_INTERVAL 2

/* ── DUML CRC tables (from dji-firmware-tools) ──────────────────── */

/* CRC-8 lookup table, initial value 0x77 */
static const uint8_t CRC8_TAB[256] = {
    0x00,0x5e,0xbc,0xe2,0x61,0x3f,0xdd,0x83,0xc2,0x9c,0x7e,0x20,0xa3,0xfd,0x1f,0x41,
    0x9d,0xc3,0x21,0x7f,0xfc,0xa2,0x40,0x1e,0x5f,0x01,0xe3,0xbd,0x3e,0x60,0x82,0xdc,
    0x23,0x7d,0x9f,0xc1,0x42,0x1c,0xfe,0xa0,0xe1,0xbf,0x5d,0x03,0x80,0xde,0x3c,0x62,
    0xbe,0xe0,0x02,0x5c,0xdf,0x81,0x63,0x3d,0x7c,0x22,0xc0,0x9e,0x1d,0x43,0xa1,0xff,
    0x46,0x18,0xfa,0xa4,0x27,0x79,0x9b,0xc5,0x84,0xda,0x38,0x66,0xe5,0xbb,0x59,0x07,
    0xdb,0x85,0x67,0x39,0xba,0xe4,0x06,0x58,0x19,0x47,0xa5,0xfb,0x78,0x26,0xc4,0x9a,
    0x65,0x3b,0xd9,0x87,0x04,0x5a,0xb8,0xe6,0xa7,0xf9,0x1b,0x45,0xc6,0x98,0x7a,0x24,
    0xf8,0xa6,0x44,0x1a,0x99,0xc7,0x25,0x7b,0x3a,0x64,0x86,0xd8,0x5b,0x05,0xe7,0xb9,
    0x8c,0xd2,0x30,0x6e,0xed,0xb3,0x51,0x0f,0x4e,0x10,0xf2,0xac,0x2f,0x71,0x93,0xcd,
    0x11,0x4f,0xad,0xf3,0x70,0x2e,0xcc,0x92,0xd3,0x8d,0x6f,0x31,0xb2,0xec,0x0e,0x50,
    0xaf,0xf1,0x13,0x4d,0xce,0x90,0x72,0x2c,0x6d,0x33,0xd1,0x8f,0x0c,0x52,0xb0,0xee,
    0x32,0x6c,0x8e,0xd0,0x53,0x0d,0xef,0xb1,0xf0,0xae,0x4c,0x12,0x91,0xcf,0x2d,0x73,
    0xca,0x94,0x76,0x28,0xab,0xf5,0x17,0x49,0x08,0x56,0xb4,0xea,0x69,0x37,0xd5,0x8b,
    0x57,0x09,0xeb,0xb5,0x36,0x68,0x8a,0xd4,0x95,0xcb,0x29,0x77,0xf4,0xaa,0x48,0x16,
    0xe9,0xb7,0x55,0x0b,0x88,0xd6,0x34,0x6a,0x2b,0x75,0x97,0xc9,0x4a,0x14,0xf6,0xa8,
    0x74,0x2a,0xc8,0x96,0x15,0x4b,0xa9,0xf7,0xb6,0xe8,0x0a,0x54,0xd7,0x89,0x6b,0x35,
};

/* CRC-16 lookup table, initial value 0x3692 */
static const uint16_t CRC16_TAB[256] = {
    0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
    0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
    0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
    0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
    0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
    0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
    0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
    0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
    0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
    0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
    0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
    0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
    0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
    0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
    0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
    0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
    0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
    0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
    0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
    0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
    0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
    0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
    0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
    0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
    0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
    0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
    0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
    0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
    0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
    0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
    0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
    0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78,
};

static uint8_t duml_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x77;
    for (size_t i = 0; i < len; i++)
        crc = CRC8_TAB[(crc ^ data[i]) & 0xFF];
    return crc;
}

static uint16_t duml_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0x3692;
    for (size_t i = 0; i < len; i++)
        crc = (crc >> 8) ^ CRC16_TAB[(crc ^ data[i]) & 0xFF];
    return crc;
}

/*
 * Build a DUML keepalive packet wrapped in TCP transport frame.
 * Sends a General Version Query (cmd_set=0x00, cmd_id=0x01) which
 * is harmless but triggers the RC to push more data packets.
 * Returns frame length written to buf.
 */
#define KEEPALIVE_FRAME_LEN  21  /* 8 TCP header + 13 DUML (11 hdr + 2 CRC) */

static int build_keepalive(uint8_t *buf, size_t buf_len, uint16_t seq)
{
    if (buf_len < KEEPALIVE_FRAME_LEN) return -1;

    /* DUML packet: 11 header bytes + 2 CRC bytes = 13 total */
    uint16_t total_len = 13;
    uint8_t byte1 = total_len & 0xFF;
    uint8_t byte2 = ((total_len >> 8) & 0x03) | (1 << 2);  /* version=1 */

    uint8_t hdr3[3] = { 0x55, byte1, byte2 };
    uint8_t hdr_crc = duml_crc8(hdr3, 3);

    uint8_t pkt[13];
    pkt[0]  = 0x55;          /* SOF */
    pkt[1]  = byte1;         /* length low */
    pkt[2]  = byte2;         /* length high + version */
    pkt[3]  = hdr_crc;       /* header CRC-8 */
    pkt[4]  = 2;             /* sender: type=2 (App), idx=0 */
    pkt[5]  = 6;             /* receiver: type=6 (RC), idx=0 */
    pkt[6]  = seq & 0xFF;    /* sequence low */
    pkt[7]  = (seq >> 8) & 0xFF;  /* sequence high */
    pkt[8]  = 0;             /* cmd_type=0, ack_type=0, encrypt=0 */
    pkt[9]  = 0x00;          /* cmd_set: General */
    pkt[10] = 0x01;          /* cmd_id: Version Query */

    uint16_t crc = duml_crc16(pkt, 11);
    pkt[11] = crc & 0xFF;
    pkt[12] = (crc >> 8) & 0xFF;

    /* TCP transport frame: magic + LE length + payload */
    memcpy(buf, TCP_MAGIC, 4);
    buf[4] = 13; buf[5] = 0; buf[6] = 0; buf[7] = 0;  /* LE uint32 = 13 */
    memcpy(buf + 8, pkt, 13);

    return KEEPALIVE_FRAME_LEN;
}

/* Output range for uinput ABS axes (symmetric around 0) */
#define OUT_MIN         -32767
#define OUT_MAX          32767


/*
 * Axis encoding: position and wrap counter are interleaved across
 * the 16-bit channel boundaries.
 *   position  = high byte of ch[N]       (0-255, the fast-moving byte)
 *   wrap_count = low byte of ch[N+1]     (center=4, counts position wraps)
 *   true_value = (wrap_count - 4) * 256 + position
 * Observed range: ±660 at full stick deflection.
 */
#define WRAP_CENTER     4
#define AXIS_RANGE      660.0


/* ── Channel mapping ──────────────────────────────────────────────── */

/*
 * Payload layout from cmd 0x06/0xAE (17 bytes):
 *
 *   Bytes 0-3: buttons and mode (parsed as int16 channels 0-1)
 *     Ch[0] (bytes 0-1): Button bitmask:
 *            32  = RTH / Flight Pause
 *            128 = Camera button half-press
 *            192 = Camera button full-press (128+64)
 *            256 = Record button
 *     Ch[1] (bytes 2-3): Mode + back buttons (additive bitmask):
 *            0/1/2 = Sport/Normal/Cinema mode switch
 *            +4    = Left back button (C1)
 *            +8    = Right back button (C2)
 *
 *   Bytes 4-16: axis data (byte-interleaved encoding)
 *     Each axis spans TWO adjacent bytes that cross int16 channel boundaries:
 *       position  = payload[2*ch + 1]  (high byte of ch[N], 0-255)
 *       wrap_count = payload[2*ch + 2] (low byte of ch[N+1], center=4)
 *       true_value = (wrap_count - 4) * 256 + position
 *     Range: ±660 at full deflection.
 *
 *     Ch[2]: Right stick horizontal  (pos=byte5,  wrap=byte6)
 *     Ch[3]: Right stick vertical    (pos=byte7,  wrap=byte8)
 *     Ch[4]: Left stick vertical     (pos=byte9,  wrap=byte10)
 *     Ch[5]: Left stick horizontal   (pos=byte11, wrap=byte12)
 *     Ch[6]: Left back wheel         (pos=byte13, wrap=byte14)
 *     Ch[7]: Right back wheel        (pos=byte15, wrap=byte16)
 */

/* Axis channel indices within the 0x06/0xAE payload */
#define CH_LH    5   /* Left Horizontal   -> ABS_X  */
#define CH_LV    4   /* Left Vertical     -> ABS_Y  */
#define CH_RH    2   /* Right Horizontal  -> ABS_RX */
#define CH_RV    3   /* Right Vertical    -> ABS_RY */
#define CH_LW    6   /* Left back wheel   -> ABS_Z  */
#define CH_RW    7   /* Right back wheel  -> ABS_RZ */

/* Button channel */
#define BTN_CHANNEL      0
#define MODE_CHANNEL     1

/* Button bitmasks on Ch[0] */
#define BTN_RTH_MASK     0x0020   /* 32  */
#define BTN_CAM_H_MASK   0x0080   /* 128 - half press */
#define BTN_CAM_F_MASK   0x0040   /* 64  - full press adds this */
#define BTN_REC_MASK     0x0100   /* 256 */

/* Back button bitmasks on Ch[1] (above mode bits 0-1) */
#define BTN_C1_MASK      0x0004   /* 4 - left back button */
#define BTN_C2_MASK      0x0008   /* 8 - right back button */

/* ── Global state ─────────────────────────────────────────────────── */

static volatile sig_atomic_t g_running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ── Network reader ──────────────────────────────────────────────── */

/*
 * Persistent TCP connection to the RC.
 * Connect once, read continuously. Reconnect on error/disconnect.
 */
typedef struct {
    char      ip[64];
    int       port;
    struct sockaddr_in addr;
    int       fd;           /* persistent socket, -1 when disconnected */
} rc_conn_t;

static void rc_conn_init(rc_conn_t *c, const char *ip, int port)
{
    memset(c, 0, sizeof(*c));
    snprintf(c->ip, sizeof(c->ip), "%s", ip);
    c->port = port;
    c->fd = -1;
    c->addr.sin_family = AF_INET;
    c->addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &c->addr.sin_addr);
}

/*
 * Establish a persistent TCP connection to the RC.
 * Non-blocking connect with poll(). Stores fd in rc_conn_t.
 * Returns 0 on success, -1 on failure.
 */
static int rc_connect(rc_conn_t *c)
{
    if (c->fd >= 0) return 0;  /* already connected */

    int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd < 0) return -1;

    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    int rc = connect(fd, (struct sockaddr *)&c->addr, sizeof(c->addr));
    if (rc < 0 && errno != EINPROGRESS) {
        close(fd);
        return -1;
    }

    struct pollfd pfd = { .fd = fd, .events = POLLOUT };
    rc = poll(&pfd, 1, 100);  /* 100ms timeout for connect */
    if (rc <= 0) { close(fd); return -1; }

    int err = 0;
    socklen_t elen = sizeof(err);
    getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &elen);
    if (err != 0) { close(fd); return -1; }

    c->fd = fd;
    return 0;
}

/*
 * Close the persistent connection.
 */
static void rc_disconnect(rc_conn_t *c)
{
    if (c->fd >= 0) {
        close(c->fd);
        c->fd = -1;
    }
}

/*
 * Read from the persistent connection using poll().
 * Returns bytes read (> 0), 0 on timeout (no data), -1 on error/disconnect.
 * On error, the connection is closed (fd set to -1).
 */
static int rc_read(rc_conn_t *c, uint8_t *buf, size_t buf_len, int timeout_ms)
{
    if (c->fd < 0) return -1;

    struct pollfd pfd = { .fd = c->fd, .events = POLLIN };
    int rc = poll(&pfd, 1, timeout_ms);

    if (rc < 0)
        return (errno == EINTR) ? 0 : -1;
    if (rc == 0)
        return 0;  /* timeout, no data */

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        rc_disconnect(c);
        return -1;
    }

    ssize_t n = recv(c->fd, buf, buf_len, MSG_DONTWAIT);
    if (n <= 0) {
        rc_disconnect(c);
        return -1;
    }
    return (int)n;
}

/*
 * Send a DUML keepalive command on the persistent connection.
 * Returns 0 on success, -1 on error (connection closed).
 */
static int rc_send_keepalive(rc_conn_t *c, uint16_t seq)
{
    if (c->fd < 0) return -1;

    uint8_t frame[KEEPALIVE_FRAME_LEN];
    build_keepalive(frame, sizeof(frame), seq);

    ssize_t n = send(c->fd, frame, KEEPALIVE_FRAME_LEN, MSG_NOSIGNAL);
    if (n < 0) {
        rc_disconnect(c);
        return -1;
    }
    return 0;
}

/* ── DUML packet parser ───────────────────────────────────────────── */

/*
 * Parse TCP-framed DUML packets and extract RC channel data.
 * Scans the entire buffer and returns the LAST (most recent) RC packet,
 * so stale data in the ring buffer doesn't cause lag.
 * Returns number of channels found, or 0 if no RC packet.
 */
static int parse_rc_channels(const uint8_t *data, size_t len,
                             int16_t *channels, int max_ch,
                             size_t *consumed,
                             uint8_t *raw_out, size_t raw_out_size,
                             size_t *raw_out_len)
{
    size_t offset = 0;
    size_t last_frame_end = 0;  /* byte after last complete frame */
    int best_num_ch = 0;

    while (offset + 8 <= len) {
        /* Find TCP frame magic */
        if (memcmp(data + offset, TCP_MAGIC, 4) != 0) {
            offset++;
            continue;
        }

        /* Read payload length (LE uint32) */
        uint32_t plen = data[offset + 4] |
                        (data[offset + 5] << 8) |
                        (data[offset + 6] << 16) |
                        (data[offset + 7] << 24);

        size_t frame_end = offset + 8 + plen;
        if (frame_end > len) {
            /* Incomplete frame — keep these bytes for next read */
            break;
        }

        if (plen >= 13) {
            const uint8_t *pkt = data + offset + 8;

            /* Verify DUML SOF */
            if (pkt[0] == 0x55) {
                uint8_t cmd_set = pkt[9];
                uint8_t cmd_id  = pkt[10];

                if (cmd_set == RC_CMD_SET && cmd_id == RC_CMD_ID) {
                    const uint8_t *payload = pkt + 11;
                    size_t payload_len = plen - 13;

                    int num_ch = 0;
                    for (size_t i = 0; i + 1 < payload_len && num_ch < max_ch; i += 2) {
                        channels[num_ch++] = (int16_t)(payload[i] | (payload[i + 1] << 8));
                    }
                    best_num_ch = num_ch;

                    /* Save raw payload for axis decoding / hex dump */
                    if (raw_out && raw_out_len) {
                        size_t copy_len = payload_len < raw_out_size ? payload_len : raw_out_size;
                        memcpy(raw_out, payload, copy_len);
                        *raw_out_len = copy_len;
                    }
                }
            }
        }

        offset = frame_end;
        last_frame_end = frame_end;
    }

    if (consumed)
        *consumed = last_frame_end;
    return best_num_ch;
}

/* ── Axis decoding ────────────────────────────────────────────────── */

/*
 * Decode an axis from the raw payload using byte-interleaved encoding.
 * ch_idx is the channel index (2–7) assigned to this axis.
 *   position   = payload[2*ch_idx + 1]  (high byte of ch[N])
 *   wrap_count = payload[2*ch_idx + 2]  (low byte of ch[N+1], or byte 16 for ch[7])
 *   true_value = (wrap_count - WRAP_CENTER) * 256 + position
 * Returns scaled value in [-32767, +32767].
 */
static int decode_axis(int ch_idx, const uint8_t *payload, size_t payload_len)
{
    size_t pos_byte  = (size_t)(2 * ch_idx + 1);
    size_t wrap_byte = (size_t)(2 * ch_idx + 2);
    if (wrap_byte >= payload_len)
        return 0;

    int value = ((int)payload[wrap_byte] - WRAP_CENTER) * 256
              + (int)payload[pos_byte];
    value = -value;  /* DJI convention is inverted vs standard joystick axes */

    double scaled = (double)value * 32767.0 / AXIS_RANGE;
    if (scaled >  32767.0) scaled =  32767.0;
    if (scaled < -32767.0) scaled = -32767.0;
    return (int)scaled;
}


/* ── uinput helpers ───────────────────────────────────────────────── */

static int uinput_setup_abs(int fd, int code, int min, int max)
{
    struct uinput_abs_setup abs = {0};
    abs.code = code;
    abs.absinfo.minimum = min;
    abs.absinfo.maximum = max;
    return ioctl(fd, UI_ABS_SETUP, &abs);
}

static int uinput_create(void)
{
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("open /dev/uinput");
        return -1;
    }

    ioctl(fd, UI_SET_EVBIT,  EV_ABS);
    ioctl(fd, UI_SET_EVBIT,  EV_KEY);

    /* 4 stick axes + 2 wheel axes */
    ioctl(fd, UI_SET_ABSBIT, ABS_X);
    ioctl(fd, UI_SET_ABSBIT, ABS_Y);
    ioctl(fd, UI_SET_ABSBIT, ABS_RX);
    ioctl(fd, UI_SET_ABSBIT, ABS_RY);
    ioctl(fd, UI_SET_ABSBIT, ABS_Z);        /* left back wheel */
    ioctl(fd, UI_SET_ABSBIT, ABS_RZ);       /* right back wheel */

    /* Flight mode as axis (0=Sport, 1=Normal, 2=Cinema) */
    ioctl(fd, UI_SET_ABSBIT, ABS_MISC);

    /*
     * Buttons — these are just btn 0-5 in most software:
     *   btn 0 = RTH/Pause,  btn 1 = Record
     *   btn 2 = Cam half,   btn 3 = Cam full  (same button, paired)
     *   btn 4 = C1 back-L,  btn 5 = C2 back-R (back buttons, paired)
     */
    ioctl(fd, UI_SET_KEYBIT, BTN_TRIGGER);    /* btn 0: RTH / Pause */
    ioctl(fd, UI_SET_KEYBIT, BTN_THUMB);      /* btn 1: Record */
    ioctl(fd, UI_SET_KEYBIT, BTN_THUMB2);     /* btn 2: Camera half-press */
    ioctl(fd, UI_SET_KEYBIT, BTN_TOP);        /* btn 3: Camera full-press */
    ioctl(fd, UI_SET_KEYBIT, BTN_TOP2);       /* btn 4: C1 left back */
    ioctl(fd, UI_SET_KEYBIT, BTN_PINKIE);     /* btn 5: C2 right back */
    ioctl(fd, UI_SET_KEYBIT, BTN_JOYSTICK);

    uinput_setup_abs(fd, ABS_X,    OUT_MIN, OUT_MAX);
    uinput_setup_abs(fd, ABS_Y,    OUT_MIN, OUT_MAX);
    uinput_setup_abs(fd, ABS_RX,   OUT_MIN, OUT_MAX);
    uinput_setup_abs(fd, ABS_RY,   OUT_MIN, OUT_MAX);
    uinput_setup_abs(fd, ABS_Z,    OUT_MIN, OUT_MAX);  /* left wheel */
    uinput_setup_abs(fd, ABS_RZ,   OUT_MIN, OUT_MAX);  /* right wheel */
    uinput_setup_abs(fd, ABS_MISC, 0, 2);              /* mode switch */

    struct uinput_setup setup = {0};
    snprintf(setup.name, UINPUT_MAX_NAME_SIZE, "DJI RC RM330");
    setup.id.bustype = BUS_VIRTUAL;
    setup.id.vendor  = 0x2CA3;
    setup.id.product = 0x1023;
    setup.id.version = 1;

    if (ioctl(fd, UI_DEV_SETUP, &setup) < 0 ||
        ioctl(fd, UI_DEV_CREATE) < 0) {
        perror("uinput setup");
        close(fd);
        return -1;
    }

    usleep(200000);
    return fd;
}

static void uinput_emit(int fd, int type, int code, int value)
{
    struct input_event ev = {0};
    ev.type  = type;
    ev.code  = code;
    ev.value = value;
    if (write(fd, &ev, sizeof(ev)) < 0)
        perror("uinput write");
}

static void uinput_sync(int fd)
{
    uinput_emit(fd, EV_SYN, SYN_REPORT, 0);
}

/* ── Network auto-discovery ──────────────────────────────────────── */

/*
 * Scan a /24 subnet for hosts with RC_DATA_PORT open that speak DUML.
 * Scans in batches of BATCH_SIZE to avoid overwhelming the network.
 * Returns 1 if found (writes IP to out_ip), 0 if not found.
 */
#define SCAN_BATCH_SIZE 64

static int scan_subnet(uint32_t net_addr, uint32_t mask, int port,
                       char *out_ip, size_t out_len)
{
    uint32_t hostmask = ~ntohl(mask);
    int num_hosts = (int)(hostmask - 1);
    if (num_hosts <= 0 || num_hosts > 1022)  /* up to /22 */
        num_hosts = 1022;

    uint32_t base = ntohl(net_addr) & ntohl(mask);

    /* Skip our own IP */
    uint32_t self = ntohl(net_addr);
    int self_host = (int)(self - base);

    /* Scan high-to-low: DHCP typically assigns from the top of the range */
    for (int batch_end = num_hosts; batch_end >= 1; batch_end -= SCAN_BATCH_SIZE) {
        int batch_start = batch_end - SCAN_BATCH_SIZE + 1;
        if (batch_start < 1) batch_start = 1;
        struct pollfd fds[SCAN_BATCH_SIZE];
        uint32_t     addrs[SCAN_BATCH_SIZE];
        int count = 0;

        for (int i = batch_start; i <= batch_end; i++) {
            if (i == self_host) continue;

            uint32_t host_ip = htonl(base + (uint32_t)i);

            int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
            if (fd < 0) continue;

            struct sockaddr_in addr = {0};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(port);
            addr.sin_addr.s_addr = host_ip;

            int rc = connect(fd, (struct sockaddr *)&addr, sizeof(addr));
            if (rc == 0) {
                /* Immediate connect (unlikely but handle it) */
                fds[count].fd = fd;
                fds[count].events = POLLIN;
                fds[count].revents = POLLOUT;
                addrs[count] = host_ip;
                count++;
                continue;
            }
            if (errno != EINPROGRESS) {
                close(fd);
                continue;
            }

            fds[count].fd = fd;
            fds[count].events = POLLOUT;
            addrs[count] = host_ip;
            count++;
        }

        if (count == 0) continue;

        /* Wait for connects in this batch (300ms — plenty for local network) */
        int ready = poll(fds, (nfds_t)count, 300);

        if (ready > 0) {
            for (int i = 0; i < count; i++) {
                if (!(fds[i].revents & POLLOUT)) continue;

                int err = 0;
                socklen_t elen = sizeof(err);
                getsockopt(fds[i].fd, SOL_SOCKET, SO_ERROR, &err, &elen);
                if (err != 0) continue;

                /* Port is open — read and check for DUML magic */
                struct in_addr a = { .s_addr = addrs[i] };
                fprintf(stderr, "    Port %d open on %s, checking DUML...\n",
                        port, inet_ntoa(a));

                struct pollfd rpfd = { .fd = fds[i].fd, .events = POLLIN };
                if (poll(&rpfd, 1, 500) > 0) {
                    uint8_t buf[256];
                    ssize_t n = recv(fds[i].fd, buf, sizeof(buf), MSG_DONTWAIT);
                    if (n >= 8 && memcmp(buf, TCP_MAGIC, 4) == 0) {
                        snprintf(out_ip, out_len, "%s", inet_ntoa(a));
                        /* Clean up all sockets in this batch */
                        for (int j = 0; j < count; j++)
                            close(fds[j].fd);
                        return 1;
                    }
                    fprintf(stderr, "    Got %zd bytes but no DUML magic\n", n);
                } else {
                    fprintf(stderr, "    No data received (port open but silent)\n");
                }
            }
        }

        for (int i = 0; i < count; i++)
            close(fds[i].fd);
    }

    return 0;
}

/*
 * Quick-check a single IP: connect to port and verify DUML magic.
 * Returns 1 if it's a DJI RC, 0 otherwise.
 */
static int rc_check_ip(const char *ip, int port)
{
    int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd < 0) return 0;

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &addr.sin_addr);

    int rc = connect(fd, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0 && errno != EINPROGRESS) { close(fd); return 0; }

    struct pollfd pfd = { .fd = fd, .events = POLLOUT };
    rc = poll(&pfd, 1, 500);
    if (rc <= 0) { close(fd); return 0; }

    int err = 0;
    socklen_t elen = sizeof(err);
    getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &elen);
    if (err != 0) { close(fd); return 0; }

    /* Connected — verify DUML */
    pfd.events = POLLIN;
    if (poll(&pfd, 1, 500) <= 0) { close(fd); return 0; }

    uint8_t buf[128];
    ssize_t n = recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
    close(fd);

    return (n >= 8 && memcmp(buf, TCP_MAGIC, 4) == 0);
}

/* Known default DJI RC addresses to try first */
static const char *known_rc_ips[] = {
    "192.168.7.251",   /* common DHCP assignment */
    "192.168.7.253",   /* common DHCP assignment on /22 networks */
    "192.168.42.2",    /* USB tethering default */
    "192.168.137.2",   /* Windows ICS default */
    NULL
};

/*
 * Auto-discover a DJI RC on the local network.
 * Strategy:
 *   1. Try known default DJI RC addresses (fast, covers most cases)
 *   2. Scan all local interface subnets
 * Returns 1 if found, 0 if not.
 */
static int rc_discover(int port, char *out_ip, size_t out_len)
{
    /* Phase 1: try known defaults */
    fprintf(stderr, "  Trying known DJI RC addresses...\n");
    for (int i = 0; known_rc_ips[i]; i++) {
        fprintf(stderr, "    %s ... ", known_rc_ips[i]);
        if (rc_check_ip(known_rc_ips[i], port)) {
            fprintf(stderr, "found!\n");
            snprintf(out_ip, out_len, "%s", known_rc_ips[i]);
            return 1;
        }
        fprintf(stderr, "no\n");
    }

    /* Phase 2: scan all local subnets */
    fprintf(stderr, "  Scanning local subnets...\n");

    struct ifaddrs *ifas, *ifa;
    if (getifaddrs(&ifas) < 0) {
        perror("getifaddrs");
        return 0;
    }

    int found = 0;

    for (ifa = ifas; ifa && !found; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET)
            continue;
        if (ifa->ifa_flags & IFF_LOOPBACK)
            continue;
        if (!(ifa->ifa_flags & IFF_UP))
            continue;

        struct sockaddr_in *sin = (struct sockaddr_in *)ifa->ifa_addr;
        struct sockaddr_in *nmask = (struct sockaddr_in *)ifa->ifa_netmask;

        /* Skip link-local (169.254.x.x) */
        uint32_t ip_host = ntohl(sin->sin_addr.s_addr);
        if ((ip_host >> 16) == 0xA9FE)
            continue;

        int cidr = __builtin_popcount(ntohl(nmask->sin_addr.s_addr));
        if (cidr < 16) continue;

        char net_str[INET_ADDRSTRLEN];
        uint32_t net = sin->sin_addr.s_addr & nmask->sin_addr.s_addr;
        inet_ntop(AF_INET, &net, net_str, sizeof(net_str));

        fprintf(stderr, "    %s/%d on %s ...\n", net_str, cidr, ifa->ifa_name);

        found = scan_subnet(sin->sin_addr.s_addr, nmask->sin_addr.s_addr,
                            port, out_ip, out_len);
    }

    freeifaddrs(ifas);
    return found;
}

/* ── Controller state ─────────────────────────────────────────────── */

typedef struct {
    int lh, lv, rh, rv;       /* stick axes (scaled to ±32767) */
    int lw, rw;                /* back wheel axes */
    int mode;                  /* flight mode: 0=Sport, 1=Normal, 2=Cinema */
    int b_rth, b_rec;          /* RTH and Record buttons */
    int b_cam_h, b_cam_f;     /* Camera half/full press */
    int b_c1, b_c2;           /* C1/C2 back buttons */
    double hz;                 /* update rate */
    unsigned long errors;
    const char *ip;
} rc_state_t;

/* ── ncurses visualizer ──────────────────────────────────────────── */

/*
 * Map a signed axis value [-32767..+32767] to a position within [0..size-1].
 */
static int axis_to_pos(int val, int size)
{
    int pos = (val + 32767) * (size - 1) / 65534;
    if (pos < 0) pos = 0;
    if (pos >= size) pos = size - 1;
    return pos;
}

/*
 * Draw a horizontal bar: filled portion based on val [-32767..+32767].
 */
static void draw_bar(int row, int col, int width, int val, const char *label)
{
    int fill = axis_to_pos(val, width);
    mvprintw(row, col, "%s ", label);
    int bar_col = col + (int)strlen(label) + 1;
    for (int i = 0; i < width; i++) {
        if (i == width / 2)
            mvaddch(row, bar_col + i, (i <= fill) ? ACS_BLOCK | A_BOLD : ACS_VLINE);
        else if (i <= fill)
            mvaddch(row, bar_col + i, ACS_BLOCK);
        else
            mvaddch(row, bar_col + i, ACS_BULLET);
    }
}

/*
 * Draw a stick box with a moving crosshair.
 * h,v are axis values [-32767..+32767].
 */
static void draw_stick(int top, int left, int box_w, int box_h,
                       int h_val, int v_val, const char *label)
{
    /* Border */
    for (int r = 0; r < box_h; r++) {
        for (int c = 0; c < box_w; c++) {
            int ch = ' ';
            if (r == 0 && c == 0)           ch = ACS_ULCORNER;
            else if (r == 0 && c == box_w-1) ch = ACS_URCORNER;
            else if (r == box_h-1 && c == 0) ch = ACS_LLCORNER;
            else if (r == box_h-1 && c == box_w-1) ch = ACS_LRCORNER;
            else if (r == 0 || r == box_h-1) ch = ACS_HLINE;
            else if (c == 0 || c == box_w-1) ch = ACS_VLINE;
            else ch = ' ';
            mvaddch(top + r, left + c, ch);
        }
    }

    /* Center crosshair lines (faint) */
    int mid_r = top + box_h / 2;
    int mid_c = left + box_w / 2;
    for (int c = left + 1; c < left + box_w - 1; c++)
        mvaddch(mid_r, c, ACS_HLINE | A_DIM);
    for (int r = top + 1; r < top + box_h - 1; r++)
        mvaddch(r, mid_c, ACS_VLINE | A_DIM);
    mvaddch(mid_r, mid_c, ACS_PLUS | A_DIM);

    /* Cursor position */
    int cx = axis_to_pos(h_val, box_w - 2) + left + 1;
    int cy = axis_to_pos(v_val, box_h - 2) + top + 1;
    mvaddch(cy, cx, ACS_DIAMOND | A_BOLD);

    /* Label below */
    mvprintw(top + box_h, left + (box_w - (int)strlen(label)) / 2, "%s", label);
}

/*
 * Draw a button indicator: highlighted when pressed.
 */
static void draw_button(int row, int col, const char *label, int pressed)
{
    if (pressed) {
        attron(A_REVERSE | A_BOLD);
        mvprintw(row, col, " %s ", label);
        attroff(A_REVERSE | A_BOLD);
    } else {
        attron(A_DIM);
        mvprintw(row, col, " %s ", label);
        attroff(A_DIM);
    }
}

/*
 * Full controller visualization.
 */
static void draw_controller(const rc_state_t *s)
{
    erase();

    int top = 1;
    int left = 2;

    /* Title */
    attron(A_BOLD);
    mvprintw(top, left, "DJI RC RM330");
    attroff(A_BOLD);
    mvprintw(top, left + 14, "(%s)", s->ip);

    top += 2;

    /* Mode switch: C / N / S  (Cinema left, Normal center, Sport right) */
    const char *labels[] = {"C", "N", "S"};
    int mode_values[]    = { 2,   1,   0 };
    mvprintw(top, left + 16, "Mode:");
    for (int i = 0; i < 3; i++) {
        int active = (s->mode == mode_values[i]);
        if (active) attron(A_REVERSE | A_BOLD);
        else attron(A_DIM);
        mvprintw(top, left + 22 + i * 4, " %s ", labels[i]);
        if (active) attroff(A_REVERSE | A_BOLD);
        else attroff(A_DIM);
    }
    top += 2;

    /* Sticks */
    int stick_w = 17;
    int stick_h = 9;
    int gap = 8;

    draw_stick(top, left, stick_w, stick_h, -s->lh, s->lv, "LEFT STICK");
    draw_stick(top, left + stick_w + gap, stick_w, stick_h, -s->rh, s->rv, "RIGHT STICK");

    top += stick_h + 2;

    /* Wheels */
    int bar_w = 16;
    draw_bar(top, left, bar_w, s->lw, "LW");
    draw_bar(top, left + stick_w + gap, bar_w, s->rw, "RW");
    top += 2;

    /* Buttons row */
    int bcol = left;
    draw_button(top, bcol, "RTH", s->b_rth);       bcol += 6;
    draw_button(top, bcol, "REC", s->b_rec);        bcol += 6;
    draw_button(top, bcol, "CAM", s->b_cam_h);      bcol += 6;
    draw_button(top, bcol, "CAM!", s->b_cam_f);     bcol += 7;
    draw_button(top, bcol, "C1", s->b_c1);          bcol += 5;
    draw_button(top, bcol, "C2", s->b_c2);
    top += 2;

    /* Status line */
    attron(A_DIM);
    mvprintw(top, left, "%.0f Hz  |  errors: %lu  |  Ctrl+C to quit", s->hz, s->errors);
    attroff(A_DIM);

    refresh();
}

/* ── Usage ────────────────────────────────────────────────────────── */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [OPTIONS] [RC_IP]\n"
        "\n"
        "DJI RC (RM330) network joystick driver.\n"
        "Connects to the RC over WiFi and creates a virtual joystick.\n"
        "If no IP is given, auto-scans the local network for the RC.\n"
        "\n"
        "Arguments:\n"
        "  RC_IP              IP address of the DJI RC (auto-detected if omitted)\n"
        "\n"
        "Options:\n"
        "  -p, --port PORT    TCP port [default: 40007]\n"
        "  -d, --debug        Print live axis values\n"
        "  -t, --test         Test mode: print values without creating joystick\n"
        "  -R, --raw          Raw mode: show ALL channel values (for mapping)\n"
        "  -V, --visual       Visual mode: ncurses controller display\n"
        "  -h, --help         Show this help\n"
        "\n"
        "Examples:\n"
        "  %s                         # auto-detect RC, create joystick\n"
        "  %s 192.168.7.251           # specify IP\n"
        "  %s -V                      # visual mode\n"
        "  %s -d 192.168.7.251        # debug output\n"
        "  %s -t 192.168.7.251        # test without uinput\n",
        prog, prog, prog, prog, prog, prog);
}

/* ── Main ─────────────────────────────────────────────────────────── */

int main(int argc, char *argv[])
{
    const char *rc_ip = NULL;
    int port = RC_DATA_PORT;
    bool debug = false;
    bool test_mode = false;
    bool raw_mode = false;
    bool visual_mode = false;
    static struct option long_opts[] = {
        {"port",   required_argument, NULL, 'p'},
        {"debug",  no_argument,       NULL, 'd'},
        {"test",   no_argument,       NULL, 't'},
        {"raw",    no_argument,       NULL, 'R'},
        {"visual", no_argument,       NULL, 'V'},
        {"help",   no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "p:dtRVh", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'p': port = atoi(optarg); break;
        case 'd': debug = true; break;
        case 't': test_mode = true; debug = true; break;
        case 'R': raw_mode = true; test_mode = true; debug = true; break;
        case 'V': visual_mode = true; test_mode = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }
    static char discovered_ip[64];

    if (optind < argc) {
        rc_ip = argv[optind];
    } else {
        fprintf(stderr, "No IP specified — scanning local network for DJI RC...\n");
        if (rc_discover(port, discovered_ip, sizeof(discovered_ip))) {
            rc_ip = discovered_ip;
            fprintf(stderr, "Found DJI RC at %s\n\n", rc_ip);
        } else {
            fprintf(stderr, "No DJI RC found on the local network.\n");
            fprintf(stderr, "Make sure the RC is powered on and connected to WiFi.\n");
            fprintf(stderr, "You can also specify the IP manually:\n");
            fprintf(stderr, "  %s 192.168.7.253\n", argv[0]);
            return 1;
        }
    }

    struct sigaction sa = {0};
    sa.sa_handler = signal_handler;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* Create virtual joystick (unless test mode) */
    int uinput_fd = -1;
    if (!test_mode) {
        fprintf(stderr, "Creating virtual joystick...\n");
        uinput_fd = uinput_create();
        if (uinput_fd < 0) {
            fprintf(stderr, "Failed to create uinput device.\n");
            return 1;
        }
        fprintf(stderr, "Virtual joystick 'DJI RC RM330' created.\n");
    }

    fprintf(stderr, "Connecting to DJI RC at %s:%d\n", rc_ip, port);
    fprintf(stderr, "Ctrl+C to stop.\n\n");

    /* Init ncurses if visual mode */
    if (visual_mode) {
        initscr();
        noecho();
        curs_set(0);
        nodelay(stdscr, TRUE);
        keypad(stdscr, TRUE);
        clear();
        refresh();
    }

    /* Connection state */
    rc_conn_t conn;
    rc_conn_init(&conn, rc_ip, port);

    int16_t channels[MAX_CHANNELS];
    uint8_t raw_payload[64];
    size_t  raw_payload_len = 0;
    unsigned long polls = 0, reads = 0, errors = 0;

    /* Stream accumulation buffer for persistent connection */
    uint8_t stream_buf[8192];
    size_t  stream_len = 0;

    /* Keepalive state */
    uint16_t keepalive_seq = 1;
    int idle_count = 0;

    struct timespec t_start;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    while (g_running) {
        /* Ensure we have a connection */
        if (conn.fd < 0) {
            stream_len = 0;  /* reset buffer on reconnect */
            idle_count = 0;
            if (rc_connect(&conn) < 0) {
                if (visual_mode) {
                    int key = getch();
                    if (key == 'q' || key == 'Q')
                        g_running = 0;
                }
                usleep(100000);  /* 100ms backoff before retry */
                continue;
            }
            /* Send first keepalive immediately on connect to trigger data */
            rc_send_keepalive(&conn, keepalive_seq++);
        }

        polls++;

        /* Read data with short timeout for fast response */
        int n = rc_read(&conn, stream_buf + stream_len,
                        sizeof(stream_buf) - stream_len, 20);
        if (n < 0) {
            errors++;
            stream_len = 0;
            continue;  /* fd already closed by rc_read */
        }
        if (n == 0) {
            /* No data — send keepalive to prod the RC */
            idle_count++;
            if (idle_count >= KEEPALIVE_INTERVAL) {
                if (rc_send_keepalive(&conn, keepalive_seq++) < 0) {
                    errors++;
                    continue;
                }
                idle_count = 0;
            }
            if (visual_mode) {
                int key = getch();
                if (key == 'q' || key == 'Q')
                    g_running = 0;
            }
            continue;
        }
        stream_len += (size_t)n;
        idle_count = 0;

        /* Drain any additional data available right now */
        for (int i = 0; i < 3 && stream_len < sizeof(stream_buf) - 512; i++) {
            int more = rc_read(&conn, stream_buf + stream_len,
                               sizeof(stream_buf) - stream_len, 5);
            if (more <= 0) break;
            stream_len += (size_t)more;
        }

        /* Parse all complete packets from the accumulation buffer */
        size_t consumed = 0;
        int num_ch = parse_rc_channels(stream_buf, stream_len,
                                       channels, MAX_CHANNELS, &consumed,
                                       raw_payload, sizeof(raw_payload),
                                       &raw_payload_len);

        /* Compact buffer: remove consumed bytes */
        if (consumed > 0 && consumed < stream_len) {
            memmove(stream_buf, stream_buf + consumed, stream_len - consumed);
            stream_len -= consumed;
        } else if (consumed >= stream_len) {
            stream_len = 0;
        }

        /* Prevent buffer from growing unbounded with unparseable data */
        if (stream_len > sizeof(stream_buf) - 512) {
            stream_len = 0;  /* flush if nearly full with junk */
        }

        if (num_ch == 0)
            continue;

        reads++;

        /* Decode into state struct */
        rc_state_t st = {0};
        st.lh = decode_axis(CH_LH, raw_payload, raw_payload_len);
        st.lv = decode_axis(CH_LV, raw_payload, raw_payload_len);
        st.rh = decode_axis(CH_RH, raw_payload, raw_payload_len);
        st.rv = decode_axis(CH_RV, raw_payload, raw_payload_len);
        st.lw = decode_axis(CH_LW, raw_payload, raw_payload_len);
        st.rw = decode_axis(CH_RW, raw_payload, raw_payload_len);

        uint16_t btns = (BTN_CHANNEL < num_ch) ? (uint16_t)channels[BTN_CHANNEL] : 0;
        st.b_rth   = !!(btns & BTN_RTH_MASK);
        st.b_rec   = !!(btns & BTN_REC_MASK);
        st.b_cam_h = !!(btns & BTN_CAM_H_MASK);
        st.b_cam_f = !!(btns & BTN_CAM_F_MASK);

        uint16_t mode_raw = (MODE_CHANNEL < num_ch) ? (uint16_t)channels[MODE_CHANNEL] : 0;
        st.mode  = mode_raw & 0x03;
        st.b_c1  = !!(mode_raw & BTN_C1_MASK);
        st.b_c2  = !!(mode_raw & BTN_C2_MASK);

        struct timespec t_now;
        clock_gettime(CLOCK_MONOTONIC, &t_now);
        double elapsed = (t_now.tv_sec - t_start.tv_sec)
                       + (t_now.tv_nsec - t_start.tv_nsec) / 1e9;
        st.hz     = (elapsed > 0.1) ? reads / elapsed : 0;
        st.errors = errors;
        st.ip     = rc_ip;

        /* Emit joystick events */
        if (uinput_fd >= 0) {
            uinput_emit(uinput_fd, EV_ABS, ABS_X,    st.lh);
            uinput_emit(uinput_fd, EV_ABS, ABS_Y,    st.lv);
            uinput_emit(uinput_fd, EV_ABS, ABS_RX,   st.rh);
            uinput_emit(uinput_fd, EV_ABS, ABS_RY,   st.rv);
            uinput_emit(uinput_fd, EV_ABS, ABS_Z,    st.lw);
            uinput_emit(uinput_fd, EV_ABS, ABS_RZ,   st.rw);
            uinput_emit(uinput_fd, EV_ABS, ABS_MISC, st.mode);

            uinput_emit(uinput_fd, EV_KEY, BTN_TRIGGER, st.b_rth);
            uinput_emit(uinput_fd, EV_KEY, BTN_THUMB,   st.b_rec);
            uinput_emit(uinput_fd, EV_KEY, BTN_THUMB2,  st.b_cam_h);
            uinput_emit(uinput_fd, EV_KEY, BTN_TOP,     st.b_cam_f);
            uinput_emit(uinput_fd, EV_KEY, BTN_TOP2,    st.b_c1);
            uinput_emit(uinput_fd, EV_KEY, BTN_PINKIE,  st.b_c2);

            uinput_sync(uinput_fd);
        }

        /* Display */
        if (visual_mode) {
            static struct timespec last_draw = {0};
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long ms_since = (now.tv_sec - last_draw.tv_sec) * 1000
                          + (now.tv_nsec - last_draw.tv_nsec) / 1000000;
            if (ms_since >= 33) {
                draw_controller(&st);
                last_draw = now;
            }
            int key = getch();
            if (key == 'q' || key == 'Q')
                g_running = 0;
        } else if (debug) {
            if (raw_mode) {
                fprintf(stderr, "\rRAW: ");
                for (size_t i = 0; i < raw_payload_len; i++)
                    fprintf(stderr, "%3d ", raw_payload[i]);
                fprintf(stderr, "\n INT: ");
                for (int i = 0; i < num_ch; i++)
                    fprintf(stderr, "[%d]%+6d ", i, channels[i]);
                fprintf(stderr, "\n DEC: RH:%+4d RV:%+4d LV:%+4d LH:%+4d LW:%+4d RW:%+4d",
                        st.rh, st.rv, st.lv, st.lh, st.lw, st.rw);
                fprintf(stderr, "  %.0fHz   \033[2A", st.hz);
            } else {
                fprintf(stderr,
                    "\rLH:%+6d LV:%+6d RH:%+6d RV:%+6d LW:%+6d RW:%+6d  "
                    "BTN[%s%s%s%s%s%s] M:%d  "
                    "%.0fHz err:%lu  ",
                    st.lh, st.lv, st.rh, st.rv, st.lw, st.rw,
                    st.b_rth   ? "RTH "  : "",
                    st.b_rec   ? "REC "  : "",
                    st.b_cam_h ? "CAMh " : "",
                    st.b_cam_f ? "CAMf " : "",
                    st.b_c1    ? "C1 "   : "",
                    st.b_c2    ? "C2 "   : "",
                    st.mode, st.hz, st.errors);
            }
        }
    }

    /* Cleanup ncurses */
    if (visual_mode)
        endwin();

    if (debug) fprintf(stderr, "\n");

    struct timespec t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_end);
    double total_secs = (t_end.tv_sec - t_start.tv_sec)
                      + (t_end.tv_nsec - t_start.tv_nsec) / 1e9;
    double final_hz = (total_secs > 0.1) ? reads / total_secs : 0;
    fprintf(stderr, "Shutting down... (%.1fs, %.0f Hz, %lu polls, %lu reads, %lu errors)\n",
            total_secs, final_hz, polls, reads, errors);

    rc_disconnect(&conn);

    if (uinput_fd >= 0) {
        ioctl(uinput_fd, UI_DEV_DESTROY);
        close(uinput_fd);
    }

    return 0;
}
