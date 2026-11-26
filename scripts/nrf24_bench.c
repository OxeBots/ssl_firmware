/*
 * nrf24_bench.c — minimal standalone C benchmark for the ESP32 Base Station.
 *
 * Speaks COBS + CRC-8 (poly 0x07) + 0x00 framing over UART0 @ 921600 baud
 * matching proto/base_station_protocol.bitproto and proto/ssl_robot_protocol.bitproto.
 *
 * Build:  gcc -O2 -Wall -Wextra -Werror -o nrf24_bench nrf24_bench.c
 * Usage:  ./nrf24_bench -p /dev/ttyUSB0 [-b 921600] [-r 0] [-n 500] [-I 5] [-t 25]
 */
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

enum
{
    BS_MSG_PING = 0x80,
    BS_MSG_RF_TX = 0x84,
    BS_MSG_INFO = 0x88,
    BS_MSG_RF_TX_STATUS = 0x89,
    BS_MSG_RF_RX = 0x8A,
    BS_TX_FLAG_EXPECT_REPLY = 2,
    BS_TX_STATUS_OK = 0,
    MSG_COMMAND = 0,
    MSG_TELEMETRY = 2,
    MSG_TELEMETRY_REQ = 3
};

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

static uint32_t wall_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
}

static uint8_t crc8(const uint8_t * data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}

static size_t cobs_encode(const uint8_t * in, size_t len, uint8_t * out)
{
    size_t w = 1, code_idx = 0;
    uint8_t code = 1;
    for (size_t i = 0; i < len; ++i)
    {
        if (in[i] != 0)
        {
            out[w++] = in[i];
            if (++code == 0xFF)
            {
                out[code_idx] = 0xFF;
                code = 1;
                code_idx = w++;
            }
        }
        else
        {
            out[code_idx] = code;
            code = 1;
            code_idx = w++;
        }
    }
    out[code_idx] = code;
    return w;
}

static size_t cobs_decode(const uint8_t * in, size_t len, uint8_t * out, size_t cap)
{
    size_t w = 0, i = 0;
    while (i < len)
    {
        uint8_t code = in[i++];
        if (code == 0)
            return 0;
        for (uint8_t j = 1; j < code; ++j)
        {
            if (i >= len || w >= cap)
                return 0;
            out[w++] = in[i++];
        }
        if (code < 0xFF && i < len)
        {
            if (w >= cap)
                return 0;
            out[w++] = 0;
        }
    }
    return w;
}

static int send_frame(int fd, uint8_t msg_type, const uint8_t * payload, size_t len)
{
    uint8_t staged[256];
    uint8_t encoded[264];
    if (len + 2 > sizeof(staged))
        return -1;
    staged[0] = msg_type;
    memcpy(staged + 1, payload, len);
    staged[len + 1] = crc8(staged, len + 1);
    size_t n = cobs_encode(staged, len + 2, encoded);
    encoded[n++] = 0x00;
    return (write(fd, encoded, n) == (ssize_t)n) ? 0 : -1;
}

typedef struct
{
    uint8_t accum[264];
    size_t accum_len;
    int overflow;
} FrameDec;

static int read_next_frame(int fd, FrameDec * dec, uint8_t * out_type, uint8_t * out_payload, size_t * out_len, int timeout_ms)
{
    double deadline = now_ms() + timeout_ms;
    while (now_ms() < deadline)
    {
        int rem = (int)(deadline - now_ms());
        if (rem <= 0)
            break;
        struct pollfd pfd = {fd, POLLIN, 0};
        if (poll(&pfd, 1, rem) <= 0)
            break;
        uint8_t buf[256];
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n <= 0)
            continue;
        for (ssize_t i = 0; i < n; ++i)
        {
            uint8_t b = buf[i];
            if (b == 0)
            {
                if (dec->overflow)
                {
                    dec->overflow = 0;
                    dec->accum_len = 0;
                }
                else if (dec->accum_len > 0)
                {
                    uint8_t raw[264];
                    size_t rlen = cobs_decode(dec->accum, dec->accum_len, raw, sizeof(raw));
                    dec->accum_len = 0;
                    if (rlen >= 3 && crc8(raw, rlen - 1) == raw[rlen - 1])
                    {
                        *out_type = raw[0];
                        *out_len = rlen - 2;
                        memcpy(out_payload, raw + 1, *out_len);
                        return 1;
                    }
                }
            }
            else if (!dec->overflow)
            {
                if (dec->accum_len >= sizeof(dec->accum))
                {
                    dec->accum_len = 0;
                    dec->overflow = 1;
                }
                else
                {
                    dec->accum[dec->accum_len++] = b;
                }
            }
        }
    }
    return 0;
}

static speed_t speed_from_int(int baud)
{
    switch (baud)
    {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B921600;
    }
}

static int open_port(const char * path, int baud)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        return -1;
    int bits = 0;
    if (ioctl(fd, TIOCMGET, &bits) == 0)
    {
        bits &= ~(TIOCM_DTR | TIOCM_RTS);
        ioctl(fd, TIOCMSET, &bits);
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);
    speed_t sp = speed_from_int(baud);
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);
    tty.c_cflag = (tty.c_cflag & ~(PARENB | CSTOPB | CSIZE | CRTSCTS)) | CS8 | CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static int cmp_double(const void * a, const void * b)
{
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

int main(int argc, char ** argv)
{
    const char * port = "/dev/ttyUSB0";
    int baud = 921600;
    int robot_id = 0;
    int count = 500;
    int tel_interval = 5;
    int timeout_ms = 25;

    int opt;
    while ((opt = getopt(argc, argv, "p:b:r:n:I:t:")) != -1)
    {
        if (opt == 'p') port = optarg;
        else if (opt == 'b') baud = atoi(optarg);
        else if (opt == 'r') robot_id = atoi(optarg);
        else if (opt == 'n') count = atoi(optarg);
        else if (opt == 'I') tel_interval = atoi(optarg);
        else if (opt == 't') timeout_ms = atoi(optarg);
    }

    int fd = open_port(port, baud);
    if (fd < 0)
    {
        perror("open_port");
        return 1;
    }

    FrameDec dec = {0};
    uint8_t ping[4] = {1, 0, 0, 0};
    send_frame(fd, BS_MSG_PING, ping, sizeof(ping));
    uint8_t ftype = 0, fpay[256];
    size_t flen = 0;
    if (!read_next_frame(fd, &dec, &ftype, fpay, &flen, 400) || ftype != BS_MSG_INFO)
    {
        fprintf(stderr, "No BaseInfo handshake on %s @ %d baud\n", port, baud);
        close(fd);
        return 1;
    }
    printf("Connected to ESP32 Base Station (%s @ %d) ch=%u ready=%u\n", port, baud, fpay[2], fpay[1]);

    double * rtts = (double *)calloc((size_t)count + 1, sizeof(double));
    int rtt_n = 0, tx_ok = 0, retries_sum = 0, tel_req = 0;
    uint16_t seq = 1;
    double t_start = now_ms();

    for (int i = 0; i < count; ++i)
    {
        uint8_t rf_tx[36] = {0};
        uint32_t ts = wall_ms();
        uint16_t s = seq++;
        rf_tx[0] = (uint8_t)(s & 0xFF);
        rf_tx[1] = (uint8_t)(s >> 8);
        rf_tx[2] = (uint8_t)robot_id;
        rf_tx[3] = 0;
        rf_tx[4] = 30;
        rf_tx[5] = (uint8_t)((robot_id << 4) | MSG_COMMAND);
        memcpy(&rf_tx[6], &ts, 4);

        send_frame(fd, BS_MSG_RF_TX, rf_tx, sizeof(rf_tx));
        if (read_next_frame(fd, &dec, &ftype, fpay, &flen, timeout_ms) && ftype == BS_MSG_RF_TX_STATUS)
        {
            if (fpay[3] == BS_TX_STATUS_OK)
            {
                tx_ok++;
                retries_sum += fpay[4];
            }
        }

        if (tel_interval > 0 && ((i + 1) % tel_interval == 0))
        {
            tel_req++;
            uint8_t req_tx[36] = {0};
            uint16_t s2 = seq++;
            uint32_t ts2 = wall_ms();
            req_tx[0] = (uint8_t)(s2 & 0xFF);
            req_tx[1] = (uint8_t)(s2 >> 8);
            req_tx[2] = (uint8_t)robot_id;
            req_tx[3] = BS_TX_FLAG_EXPECT_REPLY;
            req_tx[4] = 5;
            req_tx[5] = (uint8_t)((robot_id << 4) | MSG_TELEMETRY_REQ);
            memcpy(&req_tx[6], &ts2, 4);

            double t0 = now_ms();
            send_frame(fd, BS_MSG_RF_TX, req_tx, sizeof(req_tx));
            double dl = t0 + timeout_ms;
            while (now_ms() < dl)
            {
                if (!read_next_frame(fd, &dec, &ftype, fpay, &flen, (int)(dl - now_ms())))
                    break;
                if (ftype == BS_MSG_RF_RX && flen >= 6 + 22 && (fpay[6] & 0x0F) == MSG_TELEMETRY &&
                    (fpay[6] >> 4) == robot_id)
                {
                    rtts[rtt_n++] = now_ms() - t0;
                    break;
                }
            }
        }
    }

    double elapsed_s = (now_ms() - t_start) / 1000.0;
    printf("Done in %.2fs (%.1f cmds/s) | TX ACK: %d/%d (%.1f%%, avg retries=%.2f)\n",
           elapsed_s,
           count / elapsed_s,
           tx_ok,
           count,
           100.0 * tx_ok / count,
           tx_ok ? (double)retries_sum / tx_ok : 0.0);
    if (rtt_n > 0)
    {
        qsort(rtts, (size_t)rtt_n, sizeof(double), cmp_double);
        double sum = 0;
        for (int i = 0; i < rtt_n; ++i) sum += rtts[i];
        printf("Telemetry: %d/%d (%.1f%%) | RTT avg=%.2fms min=%.2fms p95=%.2fms max=%.2fms\n",
               rtt_n,
               tel_req,
               100.0 * rtt_n / tel_req,
               sum / rtt_n,
               rtts[0],
               rtts[(int)(0.95 * (rtt_n - 1))],
               rtts[rtt_n - 1]);
    }
    free(rtts);
    close(fd);
    return 0;
}
