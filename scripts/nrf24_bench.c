/*
 * nrf24_bench.c — minimal C round-trip benchmark for one SSL robot.
 *
 * Talks to the nRF24 USB bridge directly (no Python, no pyserial, no
 * bitproto): sends fire-and-forget 30-byte RobotCommand frames (synced on the
 * dongle's 6-byte TX status) and explicit 5-byte RobotTelemetryRequest frames
 * every tel_interval commands (-I, default 1), measuring the 22-byte
 * RobotTelemetry round-trip distribution.
 *
 * Wire layout (bitproto, little-endian):
 *   cmd[0]      = (robot_id << 4) | MSG_TYPE_COMMAND(0)
 *   cmd[1..4]   = uint32 timestamp, ms
 *   cmd[5..16]  = Pose {x,y,angle,x_v,y_v,angular_vel}, int16 each (all 0)
 *   cmd[17..28] = vision_pose (unused)
 *   cmd[29]     = kick_velocity
 *   req[0]      = (robot_id << 4) | MSG_TYPE_TELEMETRY_REQ(3)
 *   req[1..4]   = uint32 timestamp, ms
 *   tel[0]      = (robot_id << 4) | MSG_TYPE_TELEMETRY(2)
 *   tel[1..4]   = echoed timestamp
 *
 * Build:  gcc -O2 -Wall -Wextra -o nrf24_bench nrf24_bench.c
 * Usage:  ./nrf24_bench [-p /dev/ttyUSB1] [-r 1] [-n 500] [-I 5] [-w 0] [-t 100] [-i]
 */
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define CMD_LEN 30
#define TEL_REQ_LEN 5
#define TEL_LEN 22
#define STATUS_LEN 6
#define MAX_REPLY 256

enum
{
    MSG_COMMAND = 0,
    MSG_TELEMETRY = 2,
    MSG_TELEMETRY_REQ = 3
};

static const uint8_t STATUS_BYTES[STATUS_LEN] = {0x02, 0, 0, 0, 0, 0};

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

static uint32_t wall_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
}

static int speed_from_int(int baud)
{
    switch (baud)
    {
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            return -1;
    }
}

static int open_port(const char * path, int baud)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0)
    {
        fprintf(stderr, "tcgetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tio.c_cflag |= CS8;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    int sp = speed_from_int(baud);
    if (sp < 0)
    {
        fprintf(stderr, "unsupported baud %d\n", baud);
        close(fd);
        return -1;
    }
    cfsetispeed(&tio, (speed_t)sp);
    cfsetospeed(&tio, (speed_t)sp);

    if (tcsetattr(fd, TCSANOW, &tio) != 0)
    {
        fprintf(stderr, "tcsetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static void build_cmd(uint8_t out[CMD_LEN], uint8_t robot_id, uint32_t ts)
{
    memset(out, 0, CMD_LEN);
    out[0] = (uint8_t)((robot_id << 4) | MSG_COMMAND);
    out[1] = (uint8_t)(ts);
    out[2] = (uint8_t)(ts >> 8);
    out[3] = (uint8_t)(ts >> 16);
    out[4] = (uint8_t)(ts >> 24);
}

static void build_tel_req(uint8_t out[TEL_REQ_LEN], uint8_t robot_id, uint32_t ts)
{
    out[0] = (uint8_t)((robot_id << 4) | MSG_TELEMETRY_REQ);
    out[1] = (uint8_t)(ts);
    out[2] = (uint8_t)(ts >> 8);
    out[3] = (uint8_t)(ts >> 16);
    out[4] = (uint8_t)(ts >> 24);
}

static void write_all(int fd, const uint8_t * data, size_t len)
{
    size_t off = 0;
    while (off < len)
    {
        ssize_t w = write(fd, data + off, len - off);
        if (w > 0)
        {
            off += (size_t)w;
        }
        else if (w < 0 && (errno == EAGAIN || errno == EINTR))
        {
            struct pollfd pfd = {.fd = fd, .events = POLLOUT, .revents = 0};
            poll(&pfd, 1, 5);
        }
        else
        {
            break;
        }
    }
    tcdrain(fd);
}

/*
 * Wait for the USB dongle (CH340 + STC15) to emit its 6-byte TX-complete
 * status frame (02 00 00 00 00 00) before sending the next UART packet.
 */
static void wait_dongle_tx_status(int fd, int max_wait_ms)
{
    const double deadline = now_ms() + (double)max_wait_ms;
    int rcvd = 0;
    uint8_t dump[32];

    while (rcvd < STATUS_LEN)
    {
        double left = deadline - now_ms();
        if (left <= 0)
            break;

        struct pollfd pfd = {.fd = fd, .events = POLLIN, .revents = 0};
        int rc = poll(&pfd, 1, (int)left);
        if (rc <= 0 || !(pfd.revents & POLLIN))
            break;

        ssize_t n = read(fd, dump, sizeof(dump));
        if (n > 0)
            rcvd += (int)n;
    }
    usleep(1500);
    tcflush(fd, TCIFLUSH);
}

static uint32_t read_echo(const uint8_t * p)
{
    return (uint32_t)p[1] | ((uint32_t)p[2] << 8) | ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 24);
}

/*
 * Waits up to timeout_ms for a RobotTelemetry frame. Returns 1 and fills
 * *echo on success, 0 on timeout. Strips the (vestigial) 6-byte bridge
 * status code and rejects frames whose msg_type is not telemetry.
 */
static int wait_telemetry(int fd, int timeout_ms, uint32_t * echo)
{
    uint8_t buf[MAX_REPLY];
    size_t len = 0;
    const double deadline = now_ms() + (double)timeout_ms;

    for (;;)
    {
        double left = deadline - now_ms();
        if (left <= 0)
            return 0;

        struct pollfd pfd = {.fd = fd, .events = POLLIN, .revents = 0};
        int rc = poll(&pfd, 1, (int)left);
        if (rc < 0)
        {
            if (errno == EINTR)
                continue;
            return 0;
        }
        if (rc == 0)
            return 0;
        if (!(pfd.revents & POLLIN))
            return 0;

        size_t room = sizeof(buf) - len;
        if (room == 0)
        {
            memmove(buf, buf + 1, --len);
            room = 1;
        }
        ssize_t n = read(fd, buf + len, room);
        if (n <= 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
            return 0;
        }
        len += (size_t)n;

        for (;;)
        {
            if (len >= STATUS_LEN && memcmp(buf, STATUS_BYTES, STATUS_LEN) == 0)
            {
                memmove(buf, buf + STATUS_LEN, len - STATUS_LEN);
                len -= STATUS_LEN;
                continue;
            }
            if (len < TEL_LEN)
                break;
            if ((buf[0] & 0x0F) == MSG_TELEMETRY)
            {
                *echo = read_echo(buf);
                return 1;
            }
            memmove(buf, buf + 1, --len);
        }
    }
}

static int cmp_double(const void * a, const void * b)
{
    double x = *(const double *)a;
    double y = *(const double *)b;
    return (x > y) - (x < y);
}

static void send_rate_setup(int fd)
{
    static const char at[] = "AT+RATE=3\r\n";
    ssize_t ignored = write(fd, at, sizeof(at) - 1);
    (void)ignored;
    usleep(150000);
    tcflush(fd, TCIFLUSH);
}

int main(int argc, char ** argv)
{
    const char * port = "/dev/ttyUSB1";
    int robot = 1;
    int count = 500;
    int tel_interval = 1;
    int gap_ms = 0;
    int timeout_ms = 50;
    int baud = 115200;
    int do_init = 0;
    int no_flush = 0;

    int opt;
    while ((opt = getopt(argc, argv, "p:r:n:I:w:t:b:iFh")) != -1)
    {
        switch (opt)
        {
            case 'p':
                port = optarg;
                break;
            case 'r':
                robot = atoi(optarg);
                break;
            case 'n':
                count = atoi(optarg);
                break;
            case 'I':
                tel_interval = atoi(optarg);
                break;
            case 'w':
                gap_ms = atoi(optarg);
                break;
            case 't':
                timeout_ms = atoi(optarg);
                break;
            case 'b':
                baud = atoi(optarg);
                break;
            case 'i':
                do_init = 1;
                break;
            case 'F':
                no_flush = 1;
                break;
            default:
                printf(
                  "usage: %s [-p port] [-r robot] [-n count] [-I tel_interval] [-w gap_ms] "
                  "[-t timeout_ms] [-b baud] [-i] [-F]\n",
                  argv[0]);
                return 1;
        }
    }
    if (count <= 0 || timeout_ms <= 0 || tel_interval < 0 || robot < 0 || robot > 14)
    {
        fprintf(stderr, "invalid arguments\n");
        return 1;
    }

    int fd = open_port(port, baud);
    if (fd < 0)
        return 1;

    if (do_init)
        send_rate_setup(fd);
    usleep(50000);
    tcflush(fd, TCIFLUSH);

    double * rtt = calloc((size_t)count, sizeof(double));
    if (!rtt)
    {
        fprintf(stderr, "out of memory\n");
        close(fd);
        return 1;
    }

    int tel_reqs = 0;
    int ok = 0;
    int first_ok = -1;
    const double start = now_ms();

    for (int i = 0; i < count; ++i)
    {
        uint8_t cmd[CMD_LEN];
        build_cmd(cmd, (uint8_t)robot, wall_ms());

        if (!no_flush)
            tcflush(fd, TCIFLUSH);

        write_all(fd, cmd, CMD_LEN);
        wait_dongle_tx_status(fd, 18);

        if (tel_interval > 0 && ((i + 1) % tel_interval == 0))
        {
            uint8_t req[TEL_REQ_LEN];
            const uint32_t ts = wall_ms();
            build_tel_req(req, (uint8_t)robot, ts);

            if (!no_flush)
                tcflush(fd, TCIFLUSH);

            const double t0 = now_ms();
            write_all(fd, req, TEL_REQ_LEN);
            tel_reqs++;

            uint32_t echo = 0;
            if (wait_telemetry(fd, timeout_ms, &echo) && echo == ts)
            {
                if (first_ok < 0)
                    first_ok = i;
                rtt[ok++] = now_ms() - t0;
                usleep(1500);
            }
            tcflush(fd, TCIFLUSH);
        }

        if (gap_ms > 0)
            usleep((useconds_t)gap_ms * 1000);
    }

    const double elapsed = now_ms() - start;
    close(fd);

    printf("\n=== nRF24 C round-trip benchmark ===\n");
    printf("port %s @ %d, robot %d, %d cmds, tel every %d cmd(s), gap %d ms\n",
           port,
           baud,
           robot,
           count,
           tel_interval,
           gap_ms);
    printf("elapsed    : %.2f s\n", elapsed / 1000.0);
    printf("cmd rate   : %.1f cmds/s\n", (double)count * 1000.0 / elapsed);

    if (tel_reqs == 0)
    {
        free(rtt);
        return 0;
    }
    if (ok == 0)
    {
        printf("tel replies: 0/%d (0.00%%)\n", tel_reqs);
        free(rtt);
        return 2;
    }

    qsort(rtt, (size_t)ok, sizeof(double), cmp_double);
    double sum = 0.0;
    for (int i = 0; i < ok; ++i) sum += rtt[i];

    const double p50 = rtt[ok / 2];
    const double p90 = rtt[(int)((double)ok * 0.90)];
    const double p99 = rtt[(int)((double)ok * 0.99) < ok ? (int)((double)ok * 0.99) : ok - 1];

    printf("tel replies: %d/%d (%.2f%%)\n", ok, tel_reqs, 100.0 * ok / tel_reqs);
    printf("tel rate   : %.1f replies/s\n", (double)ok * 1000.0 / elapsed);
    printf("RTT min    : %.2f ms\n", rtt[0]);
    printf("RTT p50    : %.2f ms\n", p50);
    printf("RTT p90    : %.2f ms\n", p90);
    printf("RTT p99    : %.2f ms\n", p99);
    printf("RTT max    : %.2f ms\n", rtt[ok - 1]);
    printf("RTT mean   : %.2f ms\n", sum / (double)ok);
    printf("first ok   : cmd %d\n", first_ok);

    free(rtt);
    return ok == tel_reqs ? 0 : 3;
}
