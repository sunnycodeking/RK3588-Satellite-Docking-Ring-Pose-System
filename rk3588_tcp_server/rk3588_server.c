#define _XOPEN_SOURCE 600

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* ====================== 检测跟踪程序路径 ====================== */
#define DETECT_WORKDIR "/hzy_sta/rknn_yolov8_ellipse_demo"
#define DETECT_EXE     "./rknn_yolov8_ellipse_demo"
#define DETECT_MODEL   "./model/yolov8_ellipse.rknn"

/* ====================== 单次双目位姿测量程序路径 ======================
 * 位姿程序运行方式：
 *   cd POSE_WORKDIR
 *   POSE_EXE POSE_MODEL /tmp/stereo_pose_input.jpg
 *
 * 位姿程序会在 POSE_WORKDIR 下生成：
 *   result.txt
 *   result.jpg
 */
#define POSE_WORKDIR       "/hzy_sta/rknn_yolov8_pose_demo"
#define POSE_EXE           "./rknn_yolov8_pose_demo"
#define POSE_MODEL         "./model/yolov8_ellipse.rknn"
#define POSE_INPUT_IMAGE   "/tmp/stereo_pose_input.jpg"
#define POSE_RESULT_TXT    "result.txt"
#define POSE_RESULT_JPG    "result.jpg"
#define POSE_IMAGE_PORT_DEFAULT 5002
#define DETECT_CMD_FIFO    "/tmp/detect_track_cmd_fifo"
#define POSE_WAIT_SNAPSHOT_MS 5000


/* ====================== BH1750 直接 I2C 用户态配置 ====================== */
#define BH1750_I2C_DEV "/dev/i2c-4"
#define BH1750_ADDR    0x23

#define BH1750_POWER_DOWN       0x00
#define BH1750_POWER_ON         0x01
#define BH1750_RESET            0x07
#define BH1750_ONETIME_H_RES    0x20

/* ====================== RK3588 平台温度节点 ====================== */
#define THERMAL_ZONE0_TEMP "/sys/class/thermal/thermal_zone0/temp"

/* ====================== 服务器配置 ====================== */
#define DEFAULT_LISTEN_PORT 8888
#define HEARTBEAT_INTERVAL_SEC 5
#define RECV_BUF_SIZE 4096
#define CMD_LINE_SIZE 8192
#define LOG_LINE_SIZE 4096
#define JSON_BUF_SIZE 16384
#define LISTEN_BACKLOG 5
#define STOP_WAIT_MS 1500


#define DETECT_STATUS_PREFIX "@@DETECT_STATUS@@ "

/* ====================== 手动舵机控制 PWM 配置 ====================== */
#define PWM_PATH0 "/sys/class/pwm/pwmchip0/pwm0/"   /* Y轴舵机路径：角度控制 */
#define PWM_PATH1 "/sys/class/pwm/pwmchip1/pwm0/"   /* X轴舵机路径：速度控制 */
#define PWM_PERIOD_NS 4000000
#define MIN_DUTY_X_NS 2150000
#define MAX_DUTY_X_NS 2750000
#define MID_DUTY_X_NS 2450000
#define MIN_DUTY_Y_NS 1600000
#define MAX_DUTY_Y_NS 3200000
#define MID_DUTY_Y_NS 2450000
#define MANUAL_Y_STEP_NS 100000



typedef enum {
    MODE_IDLE = 0,
    MODE_VIDEO_PREVIEW = 1,
    MODE_DETECT_TRACK = 2
} SystemMode;

typedef struct {
    pid_t pid;
    bool running;
    long long start_ms;
    int log_parent_fd;
    char log_line[LOG_LINE_SIZE];
    int log_len;
} DetectService;

typedef struct {
    pid_t pid;
    bool running;
    long long start_ms;
} PreviewService;

typedef struct {
    pthread_t thread;
    pthread_mutex_t mutex;
    bool thread_running;
    bool stop_requested;
    int period_ms;

    bool valid;
    double light_lux;
    double platform_temp_c;
    char error[256];

    unsigned long seq;
} SensorService;

static volatile sig_atomic_t g_running = 1;
static DetectService g_detect = { -1, false, 0, -1, {0}, 0 };
static PreviewService g_preview = { -1, false, 0 };
static SensorService g_sensor;
static SystemMode g_mode = MODE_IDLE;
static char g_current_client_ip[INET_ADDRSTRLEN] = {0};
static int g_manual_duty_x = MID_DUTY_X_NS;
static int g_manual_duty_y = MID_DUTY_Y_NS;

static void on_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

static long long now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000LL + tv.tv_usec / 1000;
}

static void log_info(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    time_t t = time(NULL);
    struct tm tm_info;
    localtime_r(&t, &tm_info);

    char timebuf[32];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", &tm_info);

    fprintf(stdout, "[%s] ", timebuf);
    vfprintf(stdout, fmt, ap);
    fprintf(stdout, "\n");
    fflush(stdout);

    va_end(ap);
}

static const char *mode_to_string(void)
{
    switch (g_mode) {
        case MODE_VIDEO_PREVIEW:
            return "video_preview";
        case MODE_DETECT_TRACK:
            return "detect_track";
        case MODE_IDLE:
        default:
            return "idle";
    }
}

static void close_fd(int *fd)
{
    if (*fd >= 0) {
        close(*fd);
        *fd = -1;
    }
}

static void set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

/* ====================== TCP / JSON 工具 ====================== */

static int send_all(int fd, const char *buf, size_t len)
{
    if (fd < 0) {
        return -1;
    }

    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(fd, buf + sent, len - sent, 0);
        if (n > 0) {
            sent += (size_t)n;
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        return -1;
    }

    return 0;
}


static int send_raw_json_line(int client_fd, const char *json)
{
    if (client_fd < 0 || !json || json[0] == '\0') {
        return -1;
    }

    /*
     * json 已经是完整 JSON 对象字符串，这里只补 '\n'，
     * 保持和原有“一行一个 JSON”的 TCP 协议一致。
     */
    size_t len = strlen(json);
    if (send_all(client_fd, json, len) < 0) {
        return -1;
    }
    return send_all(client_fd, "\n", 1);
}


static int send_json_line(int fd, const char *fmt, ...)
{
    if (fd < 0) {
        return -1;
    }

    char payload[JSON_BUF_SIZE];

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(payload, sizeof(payload), fmt, ap);
    va_end(ap);

    if (n < 0 || n >= (int)sizeof(payload)) {
        log_info("[TCP] send_json_line too long");
        return -1;
    }

    char line[JSON_BUF_SIZE + 4];
    int m = snprintf(line, sizeof(line), "%s\n", payload);
    if (m < 0 || m >= (int)sizeof(line)) {
        return -1;
    }

    return send_all(fd, line, (size_t)m);
}

static void json_escape(const char *src, char *dst, size_t dst_size)
{
    if (!dst || dst_size == 0) {
        return;
    }

    size_t j = 0;
    for (size_t i = 0; src && src[i] != '\0' && j + 1 < dst_size; ++i) {
        unsigned char c = (unsigned char)src[i];

        if (c == '"' || c == '\\') {
            if (j + 2 >= dst_size) break;
            dst[j++] = '\\';
            dst[j++] = (char)c;
        } else if (c == '\n') {
            if (j + 2 >= dst_size) break;
            dst[j++] = '\\';
            dst[j++] = 'n';
        } else if (c == '\r') {
            if (j + 2 >= dst_size) break;
            dst[j++] = '\\';
            dst[j++] = 'r';
        } else if (c == '\t') {
            if (j + 2 >= dst_size) break;
            dst[j++] = '\\';
            dst[j++] = 't';
        } else if (c < 0x20) {
            if (j + 6 >= dst_size) break;
            snprintf(dst + j, dst_size - j, "\\u%04x", c);
            j += 6;
        } else {
            dst[j++] = (char)c;
        }
    }

    dst[j] = '\0';
}

static int send_log_json(int client_fd, const char *source, const char *stream, const char *message)
{
    if (!message || message[0] == '\0') {
        return 0;
    }

    char escaped[LOG_LINE_SIZE * 2];
    json_escape(message, escaped, sizeof(escaped));

    return send_json_line(client_fd,
                          "{\"type\":\"LOG\",\"source\":\"%s\",\"stream\":\"%s\",\"message\":\"%s\"}",
                          source, stream, escaped);
}

// static void forward_log_line(int client_fd, const char *source, const char *stream, const char *message)
// {
//     if (!message || message[0] == '\0') {
//         return;
//     }

//     /*
//      * 检测跟踪程序会周期性输出：
//      *   @@DETECT_STATUS@@ {"type":"DETECT_STATUS","state":"TRACKING","duty_x":2450000,"duty_y":2450000}
//      *
//      * 这一类信息不是普通调试日志，而是给 Qt 系统状态区使用的结构化状态。
//      * 因此这里识别前缀后，直接把后面的 JSON 发给 Qt，不再包装为 LOG。
//      */
//     if (strcmp(source, "detect_track") == 0 &&
//         strncmp(message, DETECT_STATUS_PREFIX, strlen(DETECT_STATUS_PREFIX)) == 0) {
//         const char *json = message + strlen(DETECT_STATUS_PREFIX);
//         send_raw_json_line(client_fd, json);
//         return;
//     }

//     /*
//      * 其他输出仍然作为板端调试日志转发给 Qt 的“板端调试输出”窗口。
//      */
//     log_info("[%s/%s] %s", source, stream, message);
//     send_log_json(client_fd, source, stream, message);
// }
/*
 * 从 text 中提取第一个完整 JSON 对象。
 * 例如：
 *   {"type":"DETECT_STATUS","state":"TRACKING"}[FSM] xxx
 * 提取：
 *   {"type":"DETECT_STATUS","state":"TRACKING"}
 *
 * 返回：
 *   0 成功
 *  -1 失败
 *
 * trailing_start 可用于返回 JSON 后面的剩余文本位置。
 */
static int extract_first_json_object(const char *text,
                                     char *json_out,
                                     size_t json_out_size,
                                     const char **trailing_start)
{
    if (trailing_start) {
        *trailing_start = NULL;
    }

    if (!text || !json_out || json_out_size == 0) {
        return -1;
    }

    const char *start = strchr(text, '{');
    if (!start) {
        return -1;
    }

    int depth = 0;
    int in_string = 0;
    int escape = 0;

    const char *p = start;
    for (; *p; ++p) {
        char c = *p;

        if (in_string) {
            if (escape) {
                escape = 0;
            } else if (c == '\\') {
                escape = 1;
            } else if (c == '"') {
                in_string = 0;
            }
            continue;
        }

        if (c == '"') {
            in_string = 1;
        } else if (c == '{') {
            depth++;
        } else if (c == '}') {
            depth--;
            if (depth == 0) {
                size_t len = (size_t)(p - start + 1);
                if (len >= json_out_size) {
                    return -1;
                }

                memcpy(json_out, start, len);
                json_out[len] = '\0';

                if (trailing_start) {
                    *trailing_start = p + 1;
                }

                return 0;
            }
        }
    }

    return -1;
}
static void forward_log_line(int client_fd, const char *source, const char *stream, const char *message)
{
    if (!message || message[0] == '\0') {
        return;
    }

    if (strcmp(source, "detect_track") == 0 &&
        strncmp(message, DETECT_STATUS_PREFIX, strlen(DETECT_STATUS_PREFIX)) == 0) {

        const char *payload = message + strlen(DETECT_STATUS_PREFIX);

        char json[1024];
        const char *trailing = NULL;

        if (extract_first_json_object(payload, json, sizeof(json), &trailing) == 0) {
            /*
             * 只把完整 JSON 对象发给 Qt。
             * 这样 Qt 收到的一定是一行干净 JSON：
             * {"type":"DETECT_STATUS","state":"TRACKING","duty_x":...,"duty_y":...}
             */
            send_raw_json_line(client_fd, json);

            /*
             * 如果 JSON 后面还混有 [FSM] 文本，把尾部作为普通调试 LOG 发给 Qt。
             * 如果你嫌调试窗口刷屏，也可以注释掉这一段。
             */
            if (trailing) {
                while (*trailing == ' ' || *trailing == '\t') {
                    trailing++;
                }

                if (*trailing != '\0') {
                    log_info("[detect_track/%s] %s", stream, trailing);
                    send_log_json(client_fd, source, stream, trailing);
                }
            }

            return;
        }

        /*
         * 如果提取失败，不要把坏 JSON 发给 Qt。
         * 作为普通日志转发，方便调试。
         */
        log_info("[detect_status_parse_failed] %s", message);
        send_log_json(client_fd, source, stream, message);
        return;
    }

    /*
     * 普通检测程序调试输出仍然走 LOG。
     */
    log_info("[%s/%s] %s", source, stream, message);
    send_log_json(client_fd, source, stream, message);
}

static bool cmd_is(const char *line, const char *cmd)
{
    char p1[128], p2[128];
    snprintf(p1, sizeof(p1), "\"cmd\":\"%s\"", cmd);
    snprintf(p2, sizeof(p2), "\"cmd\": \"%s\"", cmd);
    return strstr(line, p1) || strstr(line, p2);
}

static bool mode_is(const char *line, const char *mode)
{
    char p1[128], p2[128];
    snprintf(p1, sizeof(p1), "\"mode\":\"%s\"", mode);
    snprintf(p2, sizeof(p2), "\"mode\": \"%s\"", mode);
    return strstr(line, p1) || strstr(line, p2);
}

static int json_int_value(const char *line, const char *key, int default_value)
{
    char pattern[128];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *p = strstr(line, pattern);
    if (!p) {
        return default_value;
    }

    p = strchr(p, ':');
    if (!p) {
        return default_value;
    }

    p++;
    while (*p == ' ' || *p == '\t') {
        p++;
    }

    int value = atoi(p);
    return value > 0 ? value : default_value;
}

static int create_server_socket(int listen_port)
{
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        log_info("socket failed: %s", strerror(errno));
        return -1;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons((uint16_t)listen_port);

    if (bind(listen_fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        log_info("bind failed on port %d: %s", listen_port, strerror(errno));
        close(listen_fd);
        return -1;
    }

    if (listen(listen_fd, LISTEN_BACKLOG) < 0) {
        log_info("listen failed: %s", strerror(errno));
        close(listen_fd);
        return -1;
    }

    log_info("RK3588 server listening on 0.0.0.0:%d", listen_port);
    return listen_fd;
}

/* ====================== BH1750 直接 I2C 用户态读取 ====================== */

static int bh1750_i2c_open(void)
{
    int fd = open(BH1750_I2C_DEV, O_RDWR);
    if (fd < 0) {
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, BH1750_ADDR) < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

static int bh1750_write_cmd(int fd, uint8_t cmd)
{
    return write(fd, &cmd, 1) == 1 ? 0 : -1;
}

static int bh1750_read_lux(double *lux_out, char *err, size_t err_size)
{
    if (!lux_out) {
        return -1;
    }

    int fd = bh1750_i2c_open();
    if (fd < 0) {
        if (err) {
            snprintf(err, err_size, "open/ioctl %s addr 0x%02x failed: %s",
                     BH1750_I2C_DEV, BH1750_ADDR, strerror(errno));
        }
        return -1;
    }

    if (bh1750_write_cmd(fd, BH1750_POWER_ON) < 0) {
        if (err) snprintf(err, err_size, "BH1750 POWER_ON failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    if (bh1750_write_cmd(fd, BH1750_RESET) < 0) {
        if (err) snprintf(err, err_size, "BH1750 RESET failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    usleep(10000);

    if (bh1750_write_cmd(fd, BH1750_ONETIME_H_RES) < 0) {
        if (err) snprintf(err, err_size, "BH1750 ONETIME_H_RES failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    usleep(180000);

    uint8_t buf[2] = {0};
    if (read(fd, buf, 2) != 2) {
        if (err) snprintf(err, err_size, "BH1750 read failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    *lux_out = (double)raw / 1.2;

    bh1750_write_cmd(fd, BH1750_POWER_DOWN);
    close(fd);

    return 0;
}

/* ====================== RK3588 平台温度读取 ====================== */

static int read_platform_temperature(double *temp_out, char *err, size_t err_size)
{
    if (!temp_out) {
        return -1;
    }

    FILE *fp = fopen(THERMAL_ZONE0_TEMP, "r");
    if (!fp) {
        if (err) snprintf(err, err_size, "open %s failed: %s", THERMAL_ZONE0_TEMP, strerror(errno));
        return -1;
    }

    long value = 0;
    if (fscanf(fp, "%ld", &value) != 1) {
        if (err) snprintf(err, err_size, "read %s failed", THERMAL_ZONE0_TEMP);
        fclose(fp);
        return -1;
    }

    fclose(fp);
    *temp_out = (double)value / 1000.0;
    return 0;
}

/* ====================== 传感器线程 ====================== */

static void sensor_service_init(void)
{
    memset(&g_sensor, 0, sizeof(g_sensor));
    pthread_mutex_init(&g_sensor.mutex, NULL);
    g_sensor.period_ms = 1000;
}

static void sensor_service_update_data(bool valid,
                                       double lux,
                                       double temp,
                                       const char *error)
{
    pthread_mutex_lock(&g_sensor.mutex);

    g_sensor.valid = valid;
    g_sensor.light_lux = lux;
    g_sensor.platform_temp_c = temp;

    if (error) {
        snprintf(g_sensor.error, sizeof(g_sensor.error), "%s", error);
    } else {
        g_sensor.error[0] = '\0';
    }

    g_sensor.seq++;

    pthread_mutex_unlock(&g_sensor.mutex);
}

static void *sensor_thread_func(void *arg)
{
    (void)arg;

    while (1) {
        pthread_mutex_lock(&g_sensor.mutex);
        bool stop = g_sensor.stop_requested;
        int period_ms = g_sensor.period_ms;
        pthread_mutex_unlock(&g_sensor.mutex);

        if (stop) {
            break;
        }

        double lux = 0.0;
        double temp = 0.0;
        char light_err[256] = {0};
        char temp_err[256] = {0};
        char combined_err[512] = {0};

        int light_ok = bh1750_read_lux(&lux, light_err, sizeof(light_err));
        int temp_ok = read_platform_temperature(&temp, temp_err, sizeof(temp_err));

        bool valid = (light_ok == 0 && temp_ok == 0);

        if (!valid) {
            if (light_ok != 0 && temp_ok != 0) {
                snprintf(combined_err, sizeof(combined_err), "light: %s; temp: %s", light_err, temp_err);
            } else if (light_ok != 0) {
                snprintf(combined_err, sizeof(combined_err), "light: %s", light_err);
            } else {
                snprintf(combined_err, sizeof(combined_err), "temp: %s", temp_err);
            }
        }

        sensor_service_update_data(valid, lux, temp, valid ? NULL : combined_err);

        int slept = 0;
        while (slept < period_ms) {
            pthread_mutex_lock(&g_sensor.mutex);
            stop = g_sensor.stop_requested;
            pthread_mutex_unlock(&g_sensor.mutex);

            if (stop) {
                break;
            }

            usleep(100 * 1000);
            slept += 100;
        }
    }

    return NULL;
}

static int sensor_service_start(int period_ms)
{
    if (period_ms < 500) {
        period_ms = 500;
    }
    if (period_ms > 10000) {
        period_ms = 10000;
    }

    pthread_mutex_lock(&g_sensor.mutex);

    if (g_sensor.thread_running) {
        g_sensor.period_ms = period_ms;
        pthread_mutex_unlock(&g_sensor.mutex);
        return 1;
    }

    g_sensor.stop_requested = false;
    g_sensor.period_ms = period_ms;
    g_sensor.valid = false;
    g_sensor.error[0] = '\0';

    pthread_mutex_unlock(&g_sensor.mutex);

    if (pthread_create(&g_sensor.thread, NULL, sensor_thread_func, NULL) != 0) {
        return -1;
    }

    pthread_mutex_lock(&g_sensor.mutex);
    g_sensor.thread_running = true;
    pthread_mutex_unlock(&g_sensor.mutex);

    return 0;
}

static void sensor_service_stop(void)
{
    pthread_mutex_lock(&g_sensor.mutex);

    if (!g_sensor.thread_running) {
        pthread_mutex_unlock(&g_sensor.mutex);
        return;
    }

    g_sensor.stop_requested = true;
    pthread_t tid = g_sensor.thread;

    pthread_mutex_unlock(&g_sensor.mutex);

    pthread_join(tid, NULL);

    pthread_mutex_lock(&g_sensor.mutex);
    g_sensor.thread_running = false;
    g_sensor.stop_requested = false;
    pthread_mutex_unlock(&g_sensor.mutex);
}

static int send_sensor_snapshot(int client_fd, bool running)
{
    bool valid = false;
    double lux = 0.0;
    double temp = 0.0;
    unsigned long seq = 0;
    char err[256] = {0};

    pthread_mutex_lock(&g_sensor.mutex);
    valid = g_sensor.valid;
    lux = g_sensor.light_lux;
    temp = g_sensor.platform_temp_c;
    seq = g_sensor.seq;
    snprintf(err, sizeof(err), "%s", g_sensor.error);
    pthread_mutex_unlock(&g_sensor.mutex);

    if (valid) {
        return send_json_line(client_fd,
                              "{\"type\":\"SENSOR\",\"source\":\"bh1750_i2c_and_thermal\","
                              "\"valid\":true,\"running\":%s,\"light_lux\":%.2f,"
                              "\"platform_temp_c\":%.2f,\"temperature\":%.2f,\"seq\":%lu}",
                              running ? "true" : "false",
                              lux,
                              temp,
                              temp,
                              seq);
    }

    char escaped[512];
    json_escape(err, escaped, sizeof(escaped));

    return send_json_line(client_fd,
                          "{\"type\":\"SENSOR\",\"source\":\"bh1750_i2c_and_thermal\","
                          "\"valid\":false,\"running\":%s,\"error\":\"%s\",\"seq\":%lu}",
                          running ? "true" : "false",
                          escaped,
                          seq);
}

static int read_and_send_sensor_once(int client_fd)
{
    double lux = 0.0;
    double temp = 0.0;
    char light_err[256] = {0};
    char temp_err[256] = {0};
    char combined_err[512] = {0};

    int light_ok = bh1750_read_lux(&lux, light_err, sizeof(light_err));
    int temp_ok = read_platform_temperature(&temp, temp_err, sizeof(temp_err));

    bool valid = (light_ok == 0 && temp_ok == 0);

    if (!valid) {
        if (light_ok != 0 && temp_ok != 0) {
            snprintf(combined_err, sizeof(combined_err), "light: %s; temp: %s", light_err, temp_err);
        } else if (light_ok != 0) {
            snprintf(combined_err, sizeof(combined_err), "light: %s", light_err);
        } else {
            snprintf(combined_err, sizeof(combined_err), "temp: %s", temp_err);
        }
    }

    sensor_service_update_data(valid, lux, temp, valid ? NULL : combined_err);
    return send_sensor_snapshot(client_fd, false);
}


static void reap_detect_child(int client_fd);

/* ====================== 原始视频预览服务 ====================== */

static void reap_preview_child(int client_fd)
{
    if (!g_preview.running || g_preview.pid <= 0) {
        return;
    }

    int status = 0;
    pid_t r = waitpid(g_preview.pid, &status, WNOHANG);
    if (r == g_preview.pid) {
        log_info("[PREVIEW] exited, pid=%d, status=%d", g_preview.pid, status);
        g_preview.pid = -1;
        g_preview.running = false;
        if (g_mode == MODE_VIDEO_PREVIEW) {
            g_mode = MODE_IDLE;
        }

        if (client_fd >= 0) {
            send_log_json(client_fd, "server", "info", "video_preview process exited");
            send_json_line(client_fd,
                           "{\"type\":\"STATUS\",\"mode\":\"idle\","
                           "\"preview_running\":false,\"preview_pid\":-1,"
                           "\"detect_running\":%s,\"detect_pid\":%d}",
                           g_detect.running ? "true" : "false",
                           g_detect.pid);
        }
    }
}

static int start_video_preview(int client_fd, const char *client_ip, int video_port)
{
    reap_preview_child(client_fd);
    reap_detect_child(client_fd);

    if (g_detect.running && g_detect.pid > 0) {
        forward_log_line(client_fd, "server", "stderr",
                         "cannot start video_preview: detect_track is running");
        return -10;
    }

    if (g_preview.running && g_preview.pid > 0) {
        forward_log_line(client_fd, "server", "info", "video_preview already running");
        return 1;
    }

    if (!client_ip || client_ip[0] == '\0') {
        forward_log_line(client_fd, "server", "stderr", "cannot start video_preview: empty client IP");
        return -1;
    }

    if (video_port <= 0 || video_port > 65535) {
        video_port = 5000;
    }

    char host_arg[128];
    char port_arg[64];
    snprintf(host_arg, sizeof(host_arg), "host=%s", client_ip);
    snprintf(port_arg, sizeof(port_arg), "port=%d", video_port);

    pid_t pid = fork();
    if (pid < 0) {
        forward_log_line(client_fd, "server", "stderr", "fork video_preview failed");
        return -2;
    }

    if (pid == 0) {
        setpgid(0, 0);
        setenv("GST_DEBUG_NO_COLOR", "1", 1);

        execlp("gst-launch-1.0", "gst-launch-1.0", "-q",
               "v4l2src", "device=/dev/video21", "!",
               "videoconvert", "!",
               "video/x-raw,format=I420,width=1280,height=480,framerate=10/1", "!",
               "jpegenc", "quality=85", "!",
               "jpegparse", "!",
               "rtpjpegpay", "pt=26", "!",
               "udpsink", host_arg, port_arg, "sync=false", "async=false",
               (char *)NULL);

        perror("execlp gst-launch video_preview failed");
        _exit(127);
    }

    g_preview.pid = pid;
    g_preview.running = true;
    g_preview.start_ms = now_ms();
    g_mode = MODE_VIDEO_PREVIEW;

    char msg[512];
    snprintf(msg, sizeof(msg),
             "video_preview started, pid=%d, stream: /dev/video21 -> rtp/jpeg %s:%d",
             pid, client_ip, video_port);
    forward_log_line(client_fd, "server", "info", msg);

    return 0;
}

static int stop_video_preview(int client_fd)
{
    reap_preview_child(client_fd);

    if (!g_preview.running || g_preview.pid <= 0) {
        g_preview.pid = -1;
        g_preview.running = false;
        if (g_mode == MODE_VIDEO_PREVIEW) {
            g_mode = MODE_IDLE;
        }
        return 0;
    }

    pid_t pid = g_preview.pid;
    forward_log_line(client_fd, "server", "info", "stopping video_preview");
    log_info("[PREVIEW] stopping, pid=%d", pid);

    kill(-pid, SIGTERM);

    long long start = now_ms();
    while (now_ms() - start < STOP_WAIT_MS) {
        int status = 0;
        pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) {
            g_preview.pid = -1;
            g_preview.running = false;
            if (g_mode == MODE_VIDEO_PREVIEW) {
                g_mode = MODE_IDLE;
            }
            forward_log_line(client_fd, "server", "info", "video_preview stopped by SIGTERM");
            return 0;
        }
        usleep(100 * 1000);
    }

    forward_log_line(client_fd, "server", "stderr", "SIGTERM timeout, force killing video_preview");
    kill(-pid, SIGKILL);
    waitpid(pid, NULL, 0);

    g_preview.pid = -1;
    g_preview.running = false;
    if (g_mode == MODE_VIDEO_PREVIEW) {
        g_mode = MODE_IDLE;
    }

    forward_log_line(client_fd, "server", "info", "video_preview force killed");
    return 0;
}

/* ====================== 检测跟踪日志转发与进程管理 ====================== */

static int create_log_channel(int *parent_read_fd, int *child_write_fd, bool *is_pty)
{
    *parent_read_fd = -1;
    *child_write_fd = -1;
    *is_pty = false;

    int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master >= 0) {
        if (grantpt(master) == 0 && unlockpt(master) == 0) {
            char *slave_name = ptsname(master);
            if (slave_name) {
                int slave = open(slave_name, O_RDWR | O_NOCTTY);
                if (slave >= 0) {
                    *parent_read_fd = master;
                    *child_write_fd = slave;
                    *is_pty = true;
                    return 0;
                }
            }
        }
        close(master);
    }

    int pipefd[2];
    if (pipe(pipefd) != 0) {
        return -1;
    }

    *parent_read_fd = pipefd[0];
    *child_write_fd = pipefd[1];
    *is_pty = false;
    return 0;
}

static void flush_detect_log_line(int client_fd)
{
    if (g_detect.log_len <= 0) {
        return;
    }

    g_detect.log_line[g_detect.log_len] = '\0';
    forward_log_line(client_fd, "detect_track", "stdout", g_detect.log_line);
    g_detect.log_len = 0;
}

static void read_detect_log_fd(int client_fd)
{
    if (g_detect.log_parent_fd < 0) {
        return;
    }

    char buf[1024];

    while (1) {
        ssize_t n = read(g_detect.log_parent_fd, buf, sizeof(buf));
        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                char c = buf[i];

                if (c == '\r') {
                    continue;
                }

                if (c == '\n') {
                    flush_detect_log_line(client_fd);
                    continue;
                }

                if (g_detect.log_len < LOG_LINE_SIZE - 1) {
                    g_detect.log_line[g_detect.log_len++] = c;
                } else {
                    flush_detect_log_line(client_fd);
                    g_detect.log_line[g_detect.log_len++] = c;
                }
            }
            continue;
        }

        if (n == 0) {
            flush_detect_log_line(client_fd);
            return;
        }

        if (errno == EIO) {
            flush_detect_log_line(client_fd);
            return;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return;
        }

        return;
    }
}

static void reap_detect_child(int client_fd)
{
    if (!g_detect.running || g_detect.pid <= 0) {
        return;
    }

    int status = 0;
    pid_t r = waitpid(g_detect.pid, &status, WNOHANG);

    if (r == g_detect.pid) {
        read_detect_log_fd(client_fd);
        flush_detect_log_line(client_fd);
        close_fd(&g_detect.log_parent_fd);

        log_info("[DETECT] exited, pid=%d, status=%d", g_detect.pid, status);

        g_detect.pid = -1;
        g_detect.running = false;
        g_detect.log_len = 0;
        g_mode = MODE_IDLE;

        if (client_fd >= 0) {
            send_log_json(client_fd, "server", "info", "detect_track process exited");
            send_json_line(client_fd,
                           "{\"type\":\"STATUS\",\"mode\":\"idle\","
                           "\"preview_running\":false,\"preview_pid\":-1,"
                           "\"detect_running\":false,\"detect_pid\":-1}");
        }
    }
}

static int start_detect_track(int client_fd)
{
    reap_detect_child(client_fd);
    reap_preview_child(client_fd);

    if (g_preview.running && g_preview.pid > 0) {
        forward_log_line(client_fd, "server", "stderr",
                         "cannot start detect_track: video_preview is running");
        return -10;
    }

    if (g_detect.running && g_detect.pid > 0) {
        forward_log_line(client_fd, "server", "info", "detect_track already running");
        return 1;
    }

    // char exe_abs_path[512];
    // snprintf(exe_abs_path, sizeof(exe_abs_path), "%s/rknn_yolov8_camera_demo", DETECT_WORKDIR);

    // if (access(DETECT_WORKDIR, R_OK | X_OK) != 0) {
    //     forward_log_line(client_fd, "server", "stderr", "DETECT_WORKDIR not accessible");
    //     return -1;
    // }

    // if (access(exe_abs_path, X_OK) != 0) {
    //     forward_log_line(client_fd, "server", "stderr", "detect executable not accessible");
    //     return -2;
    // }


    //补丁
    char exe_abs_path[512];
    char model_abs_path[512];

    if (DETECT_EXE[0] == '/') {
        snprintf(exe_abs_path, sizeof(exe_abs_path), "%s", DETECT_EXE);
    } else if (strncmp(DETECT_EXE, "./", 2) == 0) {
        snprintf(exe_abs_path, sizeof(exe_abs_path), "%s/%s", DETECT_WORKDIR, DETECT_EXE + 2);
    } else {
        snprintf(exe_abs_path, sizeof(exe_abs_path), "%s/%s", DETECT_WORKDIR, DETECT_EXE);
    }

    if (DETECT_MODEL[0] == '/') {
        snprintf(model_abs_path, sizeof(model_abs_path), "%s", DETECT_MODEL);
    } else if (strncmp(DETECT_MODEL, "./", 2) == 0) {
        snprintf(model_abs_path, sizeof(model_abs_path), "%s/%s", DETECT_WORKDIR, DETECT_MODEL + 2);
    } else {
        snprintf(model_abs_path, sizeof(model_abs_path), "%s/%s", DETECT_WORKDIR, DETECT_MODEL);
    }

    if (access(DETECT_WORKDIR, R_OK | X_OK) != 0) {
        char msg[512];
        snprintf(msg, sizeof(msg),
                "DETECT_WORKDIR not accessible: %s, error=%s",
                DETECT_WORKDIR, strerror(errno));
        forward_log_line(client_fd, "server", "stderr", msg);
        return -1;
    }

    if (access(exe_abs_path, X_OK) != 0) {
        char msg[512];
        snprintf(msg, sizeof(msg),
                "detect executable not accessible: %s, error=%s",
                exe_abs_path, strerror(errno));
        forward_log_line(client_fd, "server", "stderr", msg);
        return -2;
    }

    if (access(model_abs_path, R_OK) != 0) {
        char msg[512];
        snprintf(msg, sizeof(msg),
                "detect model not accessible: %s, error=%s",
                model_abs_path, strerror(errno));
        forward_log_line(client_fd, "server", "stderr", msg);
        return -3;
    }
    //到这里


    int parent_read_fd = -1;
    int child_write_fd = -1;
    bool is_pty = false;

    if (create_log_channel(&parent_read_fd, &child_write_fd, &is_pty) != 0) {
        forward_log_line(client_fd, "server", "stderr", "failed to create log channel");
        return -3;
    }

    pid_t pid = fork();
    if (pid < 0) {
        close_fd(&parent_read_fd);
        close_fd(&child_write_fd);
        forward_log_line(client_fd, "server", "stderr", "fork failed");
        return -4;
    }

    if (pid == 0) {
        setpgid(0, 0);

        close(parent_read_fd);
        dup2(child_write_fd, STDOUT_FILENO);
        dup2(child_write_fd, STDERR_FILENO);

        if (child_write_fd > STDERR_FILENO) {
            close(child_write_fd);
        }

        setenv("TERM", "dumb", 1);
        setenv("GST_DEBUG_NO_COLOR", "1", 1);

        if (chdir(DETECT_WORKDIR) != 0) {
            perror("chdir DETECT_WORKDIR failed");
            _exit(126);
        }

        execl(DETECT_EXE, DETECT_EXE, DETECT_MODEL, (char *)NULL);
        perror("execl detect failed");
        _exit(127);
    }

    close_fd(&child_write_fd);
    set_nonblocking(parent_read_fd);

    g_detect.pid = pid;
    g_detect.running = true;
    g_detect.start_ms = now_ms();
    g_detect.log_parent_fd = parent_read_fd;
    g_detect.log_len = 0;
    g_mode = MODE_DETECT_TRACK;

    char msg[512];
    snprintf(msg, sizeof(msg),
             "detect_track started, pid=%d, log_channel=%s, cmd: cd %s && %s %s",
             pid, is_pty ? "pty" : "pipe", DETECT_WORKDIR, DETECT_EXE, DETECT_MODEL);

    forward_log_line(client_fd, "server", "info", msg);
    return 0;
}

static int stop_detect_track(int client_fd)
{
    reap_detect_child(client_fd);

    if (!g_detect.running || g_detect.pid <= 0) {
        g_detect.pid = -1;
        g_detect.running = false;
        g_detect.log_len = 0;
        close_fd(&g_detect.log_parent_fd);
        g_mode = MODE_IDLE;
        return 0;
    }

    pid_t pid = g_detect.pid;
    forward_log_line(client_fd, "server", "info", "stopping detect_track");
    log_info("[DETECT] stopping, pid=%d", pid);

    kill(-pid, SIGTERM);

    long long start = now_ms();
    while (now_ms() - start < STOP_WAIT_MS) {
        read_detect_log_fd(client_fd);

        int status = 0;
        pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) {
            read_detect_log_fd(client_fd);
            flush_detect_log_line(client_fd);
            close_fd(&g_detect.log_parent_fd);

            g_detect.pid = -1;
            g_detect.running = false;
            g_detect.log_len = 0;
            g_mode = MODE_IDLE;

            forward_log_line(client_fd, "server", "info", "detect_track stopped by SIGTERM");
            return 0;
        }

        usleep(100 * 1000);
    }

    forward_log_line(client_fd, "server", "stderr", "SIGTERM timeout, force killing detect_track");
    kill(-pid, SIGKILL);

    int status = 0;
    waitpid(pid, &status, 0);

    read_detect_log_fd(client_fd);
    flush_detect_log_line(client_fd);
    close_fd(&g_detect.log_parent_fd);

    g_detect.pid = -1;
    g_detect.running = false;
    g_detect.log_len = 0;
    g_mode = MODE_IDLE;

    forward_log_line(client_fd, "server", "info", "detect_track force killed");
    return 0;
}


/* ====================== 单次双目位姿测量服务 ====================== */

static void build_path_from_workdir(const char *workdir,
                                    const char *name,
                                    char *out,
                                    size_t out_size)
{
    if (!out || out_size == 0) {
        return;
    }

    if (!name || name[0] == '\0') {
        snprintf(out, out_size, "%s", workdir ? workdir : "");
        return;
    }

    if (name[0] == '/') {
        snprintf(out, out_size, "%s", name);
    } else if (strncmp(name, "./", 2) == 0) {
        snprintf(out, out_size, "%s/%s", workdir, name + 2);
    } else {
        snprintf(out, out_size, "%s/%s", workdir, name);
    }
}

static char *json_escape_alloc(const char *src)
{
    if (!src) {
        char *empty = (char *)malloc(1);
        if (empty) empty[0] = '\0';
        return empty;
    }

    size_t len = strlen(src);
    size_t cap = len * 6 + 1;
    char *dst = (char *)malloc(cap);
    if (!dst) {
        return NULL;
    }

    size_t j = 0;
    for (size_t i = 0; src[i] != '\0' && j + 1 < cap; ++i) {
        unsigned char c = (unsigned char)src[i];

        if (c == '"' || c == '\\') {
            dst[j++] = '\\';
            dst[j++] = (char)c;
        } else if (c == '\n') {
            dst[j++] = '\\';
            dst[j++] = 'n';
        } else if (c == '\r') {
            dst[j++] = '\\';
            dst[j++] = 'r';
        } else if (c == '\t') {
            dst[j++] = '\\';
            dst[j++] = 't';
        } else if (c < 0x20) {
            snprintf(dst + j, cap - j, "\\u%04x", c);
            j += 6;
        } else {
            dst[j++] = (char)c;
        }
    }

    dst[j] = '\0';
    return dst;
}

static char *read_text_file_alloc(const char *path)
{
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (size < 0) {
        fclose(fp);
        return NULL;
    }

    char *buf = (char *)malloc((size_t)size + 1);
    if (!buf) {
        fclose(fp);
        return NULL;
    }

    size_t n = fread(buf, 1, (size_t)size, fp);
    buf[n] = '\0';
    fclose(fp);
    return buf;
}

static int send_pose_result_json(int client_fd, bool valid, const char *text, const char *error)
{
    char *esc_text = json_escape_alloc(text ? text : "");
    char *esc_error = json_escape_alloc(error ? error : "");
    if (!esc_text || !esc_error) {
        free(esc_text);
        free(esc_error);
        return -1;
    }

    const char *prefix = "{\"type\":\"POSE_RESULT\",\"valid\":";
    const char *valid_str = valid ? "true" : "false";

    send_all(client_fd, prefix, strlen(prefix));
    send_all(client_fd, valid_str, strlen(valid_str));
    send_all(client_fd, ",\"text\":\"", strlen(",\"text\":\""));
    send_all(client_fd, esc_text, strlen(esc_text));
    send_all(client_fd, "\",\"error\":\"", strlen("\",\"error\":\""));
    send_all(client_fd, esc_error, strlen(esc_error));
    send_all(client_fd, "\"}\n", 3);

    free(esc_text);
    free(esc_error);
    return 0;
}

static int request_detect_snapshot(int client_fd, const char *snapshot_path)
{
    if (!g_detect.running || g_detect.pid <= 0) {
        send_pose_result_json(client_fd, false, "", "detect_track is not running; please start detection tracking first");
        return -1;
    }

    unlink(snapshot_path);

    int fd = open(DETECT_CMD_FIFO, O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "open detect command FIFO failed: %s, error=%s",
                 DETECT_CMD_FIFO, strerror(errno));
        send_pose_result_json(client_fd, false, "", msg);
        return -2;
    }

    char cmd[512];
    int n = snprintf(cmd, sizeof(cmd), "SNAPSHOT %s\n", snapshot_path);
    if (write(fd, cmd, (size_t)n) != n) {
        close(fd);
        send_pose_result_json(client_fd, false, "", "write snapshot command to detect process failed");
        return -3;
    }
    close(fd);

    long long start = now_ms();
    while (now_ms() - start < POSE_WAIT_SNAPSHOT_MS) {
        read_detect_log_fd(client_fd);
        reap_detect_child(client_fd);

        struct stat st;
        if (stat(snapshot_path, &st) == 0 && st.st_size > 0) {
            return 0;
        }

        usleep(50 * 1000);
    }

    send_pose_result_json(client_fd, false, "", "timeout waiting for detect_track to save raw stereo snapshot");
    return -4;
}

static int run_pose_measure_program(int client_fd)
{
    char exe_abs[512];
    char model_abs[512];
    char result_txt_abs[512];
    char result_jpg_abs[512];

    build_path_from_workdir(POSE_WORKDIR, POSE_EXE, exe_abs, sizeof(exe_abs));
    build_path_from_workdir(POSE_WORKDIR, POSE_MODEL, model_abs, sizeof(model_abs));
    build_path_from_workdir(POSE_WORKDIR, POSE_RESULT_TXT, result_txt_abs, sizeof(result_txt_abs));
    build_path_from_workdir(POSE_WORKDIR, POSE_RESULT_JPG, result_jpg_abs, sizeof(result_jpg_abs));

    if (access(POSE_WORKDIR, R_OK | X_OK) != 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "POSE_WORKDIR not accessible: %s, error=%s", POSE_WORKDIR, strerror(errno));
        send_pose_result_json(client_fd, false, "", msg);
        return -1;
    }
    if (access(exe_abs, X_OK) != 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "pose executable not accessible: %s, error=%s", exe_abs, strerror(errno));
        send_pose_result_json(client_fd, false, "", msg);
        return -2;
    }
    if (access(model_abs, R_OK) != 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "pose model not accessible: %s, error=%s", model_abs, strerror(errno));
        send_pose_result_json(client_fd, false, "", msg);
        return -3;
    }
    if (access(POSE_INPUT_IMAGE, R_OK) != 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "pose input image not accessible: %s, error=%s", POSE_INPUT_IMAGE, strerror(errno));
        send_pose_result_json(client_fd, false, "", msg);
        return -4;
    }

    unlink(result_txt_abs);
    unlink(result_jpg_abs);

    pid_t pid = fork();
    if (pid < 0) {
        send_pose_result_json(client_fd, false, "", "fork pose measure process failed");
        return -5;
    }

    if (pid == 0) {
        if (chdir(POSE_WORKDIR) != 0) {
            perror("chdir POSE_WORKDIR failed");
            _exit(126);
        }

        execl(POSE_EXE, POSE_EXE, POSE_MODEL, POSE_INPUT_IMAGE, (char *)NULL);
        perror("execl pose measure failed");
        _exit(127);
    }

    int status = 0;
    waitpid(pid, &status, 0);

    char *txt = read_text_file_alloc(result_txt_abs);
    if (!txt) {
        char msg[512];
        snprintf(msg, sizeof(msg), "pose measurement finished but result.txt not found: %s", result_txt_abs);
        send_pose_result_json(client_fd, false, "", msg);
        return -6;
    }

    bool ok = strstr(txt, "Status: Success") != NULL;
    send_pose_result_json(client_fd, ok, txt, ok ? "" : "pose measurement failed, see result text");
    free(txt);

    return ok ? 0 : -7;
}

static int start_pose_image_stream(int client_fd, int image_port)
{
    char result_jpg_abs[512];
    build_path_from_workdir(POSE_WORKDIR, POSE_RESULT_JPG, result_jpg_abs, sizeof(result_jpg_abs));

    if (access(result_jpg_abs, R_OK) != 0) {
        char msg[512];
        snprintf(msg, sizeof(msg), "result.jpg not accessible: %s, error=%s", result_jpg_abs, strerror(errno));
        send_log_json(client_fd, "server", "stderr", msg);
        return -1;
    }

    const char *host = g_current_client_ip[0] ? g_current_client_ip : "127.0.0.1";

    /*
     * 重要修改：
     * 你的 RK3588 当前 GStreamer 环境没有 imagefreeze 插件。
     * 已验证可用的图片发送管道是：
     *
     * filesrc location=/hzy_sta/rknn_yolov8_pose_demo/result.jpg !
     * jpegparse !
     * rtpjpegpay pt=26 !
     * udpsink host=<Ubuntu_IP> port=5002 sync=false async=false
     *
     * 单次发送通常 10ms 内结束，Qt 或 autovideosink 可能来不及显示。
     * 因此这里重复发送多次，相当于把静态 result.jpg 连续推送约 3 秒。
     */
    const int send_count = 6;
    const int send_interval_us = 500 * 1000;
    const int duration_ms = send_count * send_interval_us / 1000;

    send_json_line(client_fd,
                   "{\"type\":\"POSE_IMAGE_STREAM\",\"port\":%d,\"format\":\"rtp_jpeg\","
                   "\"duration_ms\":%d,\"file\":\"%s\"}",
                   image_port,
                   duration_ms,
                   result_jpg_abs);

    char cmd[2048];
    snprintf(cmd, sizeof(cmd),
             "gst-launch-1.0 -q "
             "filesrc location='%s' ! "
             "jpegparse ! "
             "rtpjpegpay pt=26 ! "
             "udpsink host=%s port=%d sync=false async=false",
             result_jpg_abs,
             host,
             image_port);

    char msg[768];
    snprintf(msg, sizeof(msg),
             "pose result image stream start: host=%s port=%d file=%s count=%d interval_ms=%d",
             host,
             image_port,
             result_jpg_abs,
             send_count,
             send_interval_us / 1000);
    send_log_json(client_fd, "server", "info", msg);
    log_info("[POSE_IMAGE] command: %s", cmd);

    int ok_count = 0;
    int last_ret = 0;

    for (int i = 0; i < send_count; ++i) {
        last_ret = system(cmd);

        snprintf(msg, sizeof(msg),
                 "pose image send try=%d/%d ret=%d",
                 i + 1,
                 send_count,
                 last_ret);
        send_log_json(client_fd, "server", last_ret == 0 ? "info" : "stderr", msg);
        log_info("[POSE_IMAGE] %s", msg);

        if (last_ret == 0) {
            ok_count++;
        }

        usleep(send_interval_us);
    }

    if (ok_count <= 0) {
        snprintf(msg, sizeof(msg),
                 "pose result image stream failed: all gst-launch attempts failed, last_ret=%d",
                 last_ret);
        send_log_json(client_fd, "server", "stderr", msg);
        send_json_line(client_fd,
                       "{\"type\":\"POSE_IMAGE_STREAM_DONE\",\"result\":\"failed\",\"port\":%d,"
                       "\"success_count\":0,\"last_ret\":%d}",
                       image_port,
                       last_ret);
        return -2;
    }

    snprintf(msg, sizeof(msg),
             "pose result image stream done: success_count=%d/%d",
             ok_count,
             send_count);
    send_log_json(client_fd, "server", "info", msg);

    send_json_line(client_fd,
                   "{\"type\":\"POSE_IMAGE_STREAM_DONE\",\"result\":\"ok\",\"port\":%d,"
                   "\"success_count\":%d}",
                   image_port,
                   ok_count);

    return 0;
}


static int handle_measure_pose_command(int client_fd, int image_port)
{
    if (image_port <= 0 || image_port > 65535) {
        image_port = POSE_IMAGE_PORT_DEFAULT;
    }

    send_json_line(client_fd, "{\"type\":\"ACK\",\"cmd\":\"MEASURE_POSE\",\"result\":\"started\"}");

    int ret = request_detect_snapshot(client_fd, POSE_INPUT_IMAGE);
    if (ret != 0) {
        return ret;
    }

    send_log_json(client_fd, "server", "info", "raw stereo snapshot saved, starting pose measurement");

    ret = run_pose_measure_program(client_fd);

    /*
     * 即使位姿解算失败，只要 result.jpg 存在，也发送图像用于诊断。
     */
    start_pose_image_stream(client_fd, image_port);

    return ret;
}



/* ====================== 手动舵机控制 ====================== */

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static bool action_is(const char *line, const char *action)
{
    char p1[128], p2[128];
    snprintf(p1, sizeof(p1), "\"action\":\"%s\"", action);
    snprintf(p2, sizeof(p2), "\"action\": \"%s\"", action);
    return strstr(line, p1) || strstr(line, p2);
}

static int write_text_file(const char *path, const char *text)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        return -1;
    }

    ssize_t len = (ssize_t)strlen(text);
    ssize_t n = write(fd, text, (size_t)len);
    close(fd);
    return n == len ? 0 : -1;
}

static int pwm_init_one(const char *export_path, const char *pwm_path)
{
    if (access(pwm_path, F_OK) != 0) {
        int fd = open(export_path, O_WRONLY);
        if (fd >= 0) {
            write(fd, "0", 1);
            close(fd);
            usleep(100 * 1000);
        }
    }

    char period_path[256];
    char enable_path[256];
    char duty_path[256];
    char period_buf[32];

    snprintf(period_path, sizeof(period_path), "%speriod", pwm_path);
    snprintf(enable_path, sizeof(enable_path), "%senable", pwm_path);
    snprintf(duty_path, sizeof(duty_path), "%sduty_cycle", pwm_path);
    snprintf(period_buf, sizeof(period_buf), "%d", PWM_PERIOD_NS);

    /*
     * 如果 PWM 通道已经被之前的程序 enable，部分内核不允许在线修改 period。
     * 手动控制所需的 period 与检测程序一致，因此这里不把 period 写入失败视为致命错误，
     * 只要 duty_cycle 节点存在，后续直接写 duty 即可。
     */
    (void)write_text_file(period_path, period_buf);

    if (access(duty_path, W_OK) != 0) {
        return -1;
    }

    if (write_text_file(enable_path, "1") != 0) {
        /* 已经 enable 时重复写 1 通常没问题；若失败但 duty 可写，仍允许继续。 */
        if (access(duty_path, W_OK) != 0) {
            return -1;
        }
    }

    return 0;
}

static int pwm_set_duty(const char *pwm_path, int duty_ns)
{
    duty_ns = clamp_int(duty_ns, 0, PWM_PERIOD_NS);

    char duty_path[256];
    char duty_buf[32];
    snprintf(duty_path, sizeof(duty_path), "%sduty_cycle", pwm_path);
    snprintf(duty_buf, sizeof(duty_buf), "%d", duty_ns);

    return write_text_file(duty_path, duty_buf);
}

static int manual_pwm_init(void)
{
    int ret_y = pwm_init_one("/sys/class/pwm/pwmchip0/export", PWM_PATH0);
    int ret_x = pwm_init_one("/sys/class/pwm/pwmchip1/export", PWM_PATH1);
    return (ret_y == 0 && ret_x == 0) ? 0 : -1;
}

static int manual_apply_pwm(void)
{
    int ret_y = pwm_set_duty(PWM_PATH0, g_manual_duty_y);
    int ret_x = pwm_set_duty(PWM_PATH1, g_manual_duty_x);
    return (ret_y == 0 && ret_x == 0) ? 0 : -1;
}

static int handle_servo_manual_command(int client_fd, const char *line)
{
    if (g_detect.running && g_detect.pid > 0) {
        send_json_line(client_fd,
                       "{\"type\":\"ERROR\",\"cmd\":\"SERVO_MANUAL\",\"code\":409,"
                       "\"message\":\"manual servo disabled while detect_track is running\"}");
        return -1;
    }

    const char *action = "unknown";

    if (action_is(line, "left_press")) {
        action = "left_press";
        g_manual_duty_x = MIN_DUTY_X_NS;
    } else if (action_is(line, "left_release")) {
        action = "left_release";
        g_manual_duty_x = MID_DUTY_X_NS;
    } else if (action_is(line, "right_press")) {
        action = "right_press";
        g_manual_duty_x = MAX_DUTY_X_NS;
    } else if (action_is(line, "right_release")) {
        action = "right_release";
        g_manual_duty_x = MID_DUTY_X_NS;
    } else if (action_is(line, "down_step")) {
        action = "down_step";
        g_manual_duty_y = clamp_int(g_manual_duty_y - MANUAL_Y_STEP_NS,
                                    MIN_DUTY_Y_NS, MAX_DUTY_Y_NS);
    } else if (action_is(line, "up_step")) {
        action = "up_step";
        g_manual_duty_y = clamp_int(g_manual_duty_y + MANUAL_Y_STEP_NS,
                                    MIN_DUTY_Y_NS, MAX_DUTY_Y_NS);
    } else if (action_is(line, "center")) {
        action = "center";
        /* X轴是速度控制，没有绝对角度反馈；归中时只让 X 停止，并让 Y 回中。 */
        g_manual_duty_x = MID_DUTY_X_NS;
        g_manual_duty_y = MID_DUTY_Y_NS;
    } else {
        send_json_line(client_fd,
                       "{\"type\":\"ERROR\",\"cmd\":\"SERVO_MANUAL\",\"code\":400,"
                       "\"message\":\"unknown servo manual action\"}");
        return -2;
    }

    if (manual_pwm_init() != 0 || manual_apply_pwm() != 0) {
        char msg[512];
        snprintf(msg, sizeof(msg),
                 "manual servo pwm write failed, action=%s, duty_x=%d, duty_y=%d, errno=%s",
                 action, g_manual_duty_x, g_manual_duty_y, strerror(errno));
        forward_log_line(client_fd, "server", "stderr", msg);
        send_json_line(client_fd,
                       "{\"type\":\"ERROR\",\"cmd\":\"SERVO_MANUAL\",\"code\":500,"
                       "\"message\":\"manual servo pwm write failed\"}");
        return -3;
    }

    send_json_line(client_fd,
                   "{\"type\":\"ACK\",\"cmd\":\"SERVO_MANUAL\",\"result\":\"ok\","
                   "\"action\":\"%s\",\"duty_x\":%d,\"duty_y\":%d}",
                   action, g_manual_duty_x, g_manual_duty_y);

    send_json_line(client_fd,
                   "{\"type\":\"SERVO_STATUS\",\"manual\":true,"
                   "\"action\":\"%s\",\"duty_x\":%d,\"duty_y\":%d}",
                   action, g_manual_duty_x, g_manual_duty_y);

    return 0;
}

/* ====================== TCP 命令处理 ====================== */

static int send_sensor_if_new(int client_fd, unsigned long *last_sent_seq)
{
    unsigned long seq = 0;

    pthread_mutex_lock(&g_sensor.mutex);
    seq = g_sensor.seq;
    bool running = g_sensor.thread_running;
    pthread_mutex_unlock(&g_sensor.mutex);

    if (running && seq != 0 && seq != *last_sent_seq) {
        *last_sent_seq = seq;
        return send_sensor_snapshot(client_fd, true);
    }

    return 0;
}

static void handle_client_command(int client_fd, const char *line)
{
    log_info("RX: %s", line);

    if (cmd_is(line, "HELLO")) {
        pthread_mutex_lock(&g_sensor.mutex);
        bool sensor_running = g_sensor.thread_running;
        int period_ms = g_sensor.period_ms;
        pthread_mutex_unlock(&g_sensor.mutex);

        send_json_line(client_fd,
                       "{\"type\":\"HELLO_ACK\",\"server\":\"rk3588\",\"status\":\"ready\"," 
                       "\"version\":\"2.3.0\",\"mode\":\"%s\"," 
                       "\"sensor\":\"bh1750_i2c_plus_platform_temp\"," 
                       "\"sensor_running\":%s,\"sensor_period_ms\":%d}",
                       mode_to_string(),
                       sensor_running ? "true" : "false",
                       period_ms);
    } else if (cmd_is(line, "START_MODE") && mode_is(line, "video_preview")) {
        int video_port = json_int_value(line, "video_port", 5000);
        int ret = start_video_preview(client_fd, g_current_client_ip, video_port);
        if (ret >= 0) {
            send_json_line(client_fd,
                           "{\"type\":\"ACK\",\"cmd\":\"START_MODE\",\"mode\":\"video_preview\"," 
                           "\"result\":\"ok\",\"pid\":%d,\"video_port\":%d}",
                           g_preview.pid, video_port);
            send_json_line(client_fd,
                           "{\"type\":\"STATUS\",\"mode\":\"video_preview\"," 
                           "\"preview_running\":true,\"preview_pid\":%d,"
                           "\"detect_running\":false,\"detect_pid\":-1}",
                           g_preview.pid);
        } else if (ret == -10) {
            send_json_line(client_fd,
                           "{\"type\":\"ERROR\",\"cmd\":\"START_MODE\",\"mode\":\"video_preview\"," 
                           "\"code\":409,\"message\":\"detect_track is running\"}");
        } else {
            send_json_line(client_fd,
                           "{\"type\":\"ERROR\",\"cmd\":\"START_MODE\",\"mode\":\"video_preview\"," 
                           "\"code\":500,\"message\":\"start video_preview failed\"}");
        }
    } else if (cmd_is(line, "STOP_MODE") && mode_is(line, "video_preview")) {
        stop_video_preview(client_fd);
        send_json_line(client_fd,
                       "{\"type\":\"ACK\",\"cmd\":\"STOP_MODE\",\"mode\":\"video_preview\",\"result\":\"ok\"}");
        send_json_line(client_fd,
                       "{\"type\":\"STATUS\",\"mode\":\"idle\"," 
                       "\"preview_running\":false,\"preview_pid\":-1,"
                       "\"detect_running\":false,\"detect_pid\":-1}");
    } else if (cmd_is(line, "START_MODE") && mode_is(line, "detect_track")) {
        int ret = start_detect_track(client_fd);
        if (ret >= 0) {
            send_json_line(client_fd,
                           "{\"type\":\"ACK\",\"cmd\":\"START_MODE\",\"mode\":\"detect_track\"," 
                           "\"result\":\"ok\",\"pid\":%d}", g_detect.pid);
            send_json_line(client_fd,
                           "{\"type\":\"STATUS\",\"mode\":\"detect_track\"," 
                           "\"preview_running\":false,\"preview_pid\":-1,"
                           "\"detect_running\":true,\"detect_pid\":%d}", g_detect.pid);
        } else if (ret == -10) {
            send_json_line(client_fd,
                           "{\"type\":\"ERROR\",\"cmd\":\"START_MODE\",\"mode\":\"detect_track\"," 
                           "\"code\":409,\"message\":\"video_preview is running\"}");
        } else {
            send_json_line(client_fd,
                           "{\"type\":\"ERROR\",\"cmd\":\"START_MODE\",\"mode\":\"detect_track\"," 
                           "\"code\":500,\"message\":\"start detect_track failed\"}");
        }
    } else if (cmd_is(line, "STOP_MODE") && mode_is(line, "detect_track")) {
        stop_detect_track(client_fd);
        send_json_line(client_fd,
                       "{\"type\":\"ACK\",\"cmd\":\"STOP_MODE\",\"mode\":\"detect_track\",\"result\":\"ok\"}");
        send_json_line(client_fd,
                       "{\"type\":\"STATUS\",\"mode\":\"idle\"," 
                       "\"preview_running\":false,\"preview_pid\":-1,"
                       "\"detect_running\":false,\"detect_pid\":-1}");
    } else if (cmd_is(line, "SERVO_MANUAL")) {
        handle_servo_manual_command(client_fd, line);
    } else if (cmd_is(line, "MEASURE_POSE")) {
        int image_port = json_int_value(line, "image_port", POSE_IMAGE_PORT_DEFAULT);
        handle_measure_pose_command(client_fd, image_port);
    } else if (cmd_is(line, "GET_SENSOR")) {
        read_and_send_sensor_once(client_fd);
    } else if (cmd_is(line, "SUBSCRIBE_SENSOR")) {
        int period_ms = json_int_value(line, "period_ms", 1000);

        int ret = sensor_service_start(period_ms);

        if (ret >= 0) {
            send_json_line(client_fd,
                           "{\"type\":\"ACK\",\"cmd\":\"SUBSCRIBE_SENSOR\",\"result\":\"ok\"," 
                           "\"sensor\":\"bh1750_i2c_plus_platform_temp\",\"period_ms\":%d}",
                           period_ms < 500 ? 500 : (period_ms > 10000 ? 10000 : period_ms));
        } else {
            send_json_line(client_fd,
                           "{\"type\":\"ERROR\",\"cmd\":\"SUBSCRIBE_SENSOR\",\"code\":520,"
                           "\"message\":\"start sensor thread failed\"}");
        }
    } else if (cmd_is(line, "UNSUBSCRIBE_SENSOR")) {
        sensor_service_stop();
        send_json_line(client_fd,
                       "{\"type\":\"ACK\",\"cmd\":\"UNSUBSCRIBE_SENSOR\",\"result\":\"ok\"," 
                       "\"sensor\":\"bh1750_i2c_plus_platform_temp\"}");
    } else if (cmd_is(line, "GET_STATUS")) {
        reap_preview_child(client_fd);
        reap_detect_child(client_fd);

        pthread_mutex_lock(&g_sensor.mutex);
        bool sensor_running = g_sensor.thread_running;
        int period_ms = g_sensor.period_ms;
        pthread_mutex_unlock(&g_sensor.mutex);

        send_json_line(client_fd,
                       "{\"type\":\"STATUS\",\"mode\":\"%s\"," 
                       "\"preview_running\":%s,\"preview_pid\":%d,"
                       "\"detect_running\":%s,\"detect_pid\":%d,"
                       "\"sensor_running\":%s,\"sensor_period_ms\":%d}",
                       mode_to_string(),
                       g_preview.running ? "true" : "false",
                       g_preview.pid,
                       g_detect.running ? "true" : "false",
                       g_detect.pid,
                       sensor_running ? "true" : "false",
                       period_ms);
    } else if (cmd_is(line, "STOP_ALL")) {
        sensor_service_stop();
        stop_video_preview(client_fd);
        stop_detect_track(client_fd);

        /* 一键停止全部时，额外确保 X 轴停止、Y 轴回中。 */
        g_manual_duty_x = MID_DUTY_X_NS;
        g_manual_duty_y = MID_DUTY_Y_NS;
        if (manual_pwm_init() == 0 && manual_apply_pwm() == 0) {
            send_json_line(client_fd,
                           "{\"type\":\"SERVO_STATUS\",\"manual\":true,"
                           "\"action\":\"stop_all_center\",\"duty_x\":%d,\"duty_y\":%d}",
                           g_manual_duty_x, g_manual_duty_y);
        }

        send_json_line(client_fd,
                       "{\"type\":\"ACK\",\"cmd\":\"STOP_ALL\",\"result\":\"ok\"}");
        send_json_line(client_fd,
                       "{\"type\":\"STATUS\",\"mode\":\"idle\"," 
                       "\"preview_running\":false,\"preview_pid\":-1,"
                       "\"detect_running\":false,\"detect_pid\":-1,\"sensor_running\":false}");
    } else if (cmd_is(line, "PING")) {
        send_json_line(client_fd, "{\"type\":\"PONG\",\"time_ms\":%lld}", now_ms());
    } else {
        send_json_line(client_fd,
                       "{\"type\":\"ERROR\",\"code\":400,\"message\":\"unknown command\"}");
    }
}

static int process_received_data(int client_fd,
                                 const char *data,
                                 int data_len,
                                 char *line_buf,
                                 int *line_len)
{
    for (int i = 0; i < data_len; ++i) {
        char c = data[i];

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            line_buf[*line_len] = '\0';
            if (*line_len > 0) {
                handle_client_command(client_fd, line_buf);
            }
            *line_len = 0;
            continue;
        }

        if (*line_len < CMD_LINE_SIZE - 1) {
            line_buf[*line_len] = c;
            (*line_len)++;
        } else {
            *line_len = 0;
            return -1;
        }
    }

    return 0;
}

static int handle_one_client(int client_fd, const char *client_ip, int client_port)
{
    char recv_buf[RECV_BUF_SIZE];
    char cmd_line[CMD_LINE_SIZE];
    int cmd_len = 0;
    long long last_heartbeat_ms = 0;
    unsigned long last_sensor_seq_sent = 0;

    log_info("client connected: %s:%d", client_ip, client_port);
    snprintf(g_current_client_ip, sizeof(g_current_client_ip), "%s", client_ip);

    pthread_mutex_lock(&g_sensor.mutex);
    bool sensor_running = g_sensor.thread_running;
    int sensor_period_ms = g_sensor.period_ms;
    pthread_mutex_unlock(&g_sensor.mutex);

    send_json_line(client_fd,
                   "{\"type\":\"HELLO\",\"device\":\"rk3588\",\"role\":\"server\","
                   "\"version\":\"2.3.0\",\"mode\":\"%s\","
                   "\"sensor\":\"bh1750_i2c_plus_platform_temp\","
                   "\"sensor_running\":%s,\"sensor_period_ms\":%d}",
                   mode_to_string(),
                   sensor_running ? "true" : "false",
                   sensor_period_ms);

    while (g_running) {
        reap_preview_child(client_fd);
        reap_detect_child(client_fd);

        fd_set readfds;
        FD_ZERO(&readfds);

        FD_SET(client_fd, &readfds);
        int maxfd = client_fd;

        if (g_detect.log_parent_fd >= 0) {
            FD_SET(g_detect.log_parent_fd, &readfds);
            if (g_detect.log_parent_fd > maxfd) {
                maxfd = g_detect.log_parent_fd;
            }
        }

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 200 * 1000;

        int ret = select(maxfd + 1, &readfds, NULL, NULL, &timeout);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            log_info("select failed: %s", strerror(errno));
            return -1;
        }

        if (ret > 0) {
            if (FD_ISSET(client_fd, &readfds)) {
                ssize_t n = recv(client_fd, recv_buf, sizeof(recv_buf), 0);
                if (n > 0) {
                    if (process_received_data(client_fd, recv_buf, (int)n, cmd_line, &cmd_len) < 0) {
                        return -1;
                    }
                } else if (n == 0) {
                    log_info("client disconnected: %s:%d", client_ip, client_port);
                    sensor_service_stop();
                    stop_video_preview(client_fd);
                    stop_detect_track(client_fd);
                    return 0;
                } else if (errno != EINTR) {
                    log_info("recv failed: %s", strerror(errno));
                    return -1;
                }
            }

            if (g_detect.log_parent_fd >= 0 && FD_ISSET(g_detect.log_parent_fd, &readfds)) {
                read_detect_log_fd(client_fd);
            }
        }

        long long t = now_ms();
        if (t - last_heartbeat_ms >= HEARTBEAT_INTERVAL_SEC * 1000LL) {
            last_heartbeat_ms = t;

            pthread_mutex_lock(&g_sensor.mutex);
            bool sensor_thread_running = g_sensor.thread_running;
            pthread_mutex_unlock(&g_sensor.mutex);

            send_json_line(client_fd,
                           "{\"type\":\"HEARTBEAT\",\"device\":\"rk3588\",\"time_ms\":%lld,"
                           "\"mode\":\"%s\",\"preview_running\":%s,"
                           "\"detect_running\":%s,\"sensor_running\":%s}",
                           t,
                           mode_to_string(),
                           g_preview.running ? "true" : "false",
                           g_detect.running ? "true" : "false",
                           sensor_thread_running ? "true" : "false");
        }

        send_sensor_if_new(client_fd, &last_sensor_seq_sent);
    }

    return 0;
}

int main(int argc, char *argv[])
{
    int listen_port = DEFAULT_LISTEN_PORT;
    if (argc >= 2) {
        listen_port = atoi(argv[1]);
        if (listen_port <= 0 || listen_port > 65535) {
            fprintf(stderr, "invalid port: %s\n", argv[1]);
            return 1;
        }
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    sensor_service_init();

    int listen_fd = create_server_socket(listen_port);
    if (listen_fd < 0) {
        return 1;
    }

    while (g_running) {
        struct sockaddr_in cliaddr;
        socklen_t clilen = sizeof(cliaddr);

        int client_fd = accept(listen_fd, (struct sockaddr *)&cliaddr, &clilen);
        if (client_fd < 0) {
            if (errno == EINTR) {
                continue;
            }
            log_info("accept failed: %s", strerror(errno));
            break;
        }

        char client_ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &cliaddr.sin_addr, client_ip, sizeof(client_ip));
        int client_port = ntohs(cliaddr.sin_port);

        handle_one_client(client_fd, client_ip, client_port);
        close(client_fd);
    }

    sensor_service_stop();

    if (g_preview.running) {
        stop_video_preview(-1);
    }

    if (g_detect.running) {
        stop_detect_track(-1);
    }

    close(listen_fd);
    log_info("server exit");
    return 0;
}
