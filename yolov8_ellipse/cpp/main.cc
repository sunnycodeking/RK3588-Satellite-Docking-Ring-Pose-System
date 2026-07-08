// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//双目跟踪卫星程序(传输图片版本)

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <turbojpeg.h>
#include <glib-unix.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/stat.h>
#include <math.h>
#include <errno.h>

#include "yolov8.h"
#include "image_utils.h"
#include "image_drawing.h"
#include "kcf_tracker.h"
#include "opencv2/opencv.hpp"
#include <vector>
#if defined(RV1106_1103)
#include "dma_alloc.hpp"
#endif

/*-------------------------------------------
                  Globals
-------------------------------------------*/
static rknn_app_context_t rknn_app_ctx;
static gboolean gst_initialized = FALSE;
static GMainLoop *main_loop = NULL;

// 显示管道全局变量
static GstElement *display_pipeline = NULL;
static GstElement *display_appsrc = NULL;
static int display_width = 1280;
static int display_height = 480;

// 双目同步摄像头 /dev/video21 输出的已经是左右拼接图像：1280x480
#define STEREO_WIDTH 1280
#define STEREO_HEIGHT 480
#define EYE_WIDTH 640
#define EYE_HEIGHT 480

/* ====================== 位姿测量快照配置 ======================
 * RK3588 Server 通过命名管道发送：
 *   SNAPSHOT /tmp/stereo_pose_input.jpg
 *
 * 检测跟踪程序在下一帧回调中保存“未绘制任何检测框/文字的原始 1280x480 双目拼接图”。
 */
#define POSE_CMD_FIFO "/tmp/detect_track_cmd_fifo"
#define POSE_DEFAULT_SNAPSHOT_PATH "/tmp/stereo_pose_input.jpg"

static int pose_cmd_fd = -1;
static int pose_cmd_dummy_fd = -1;
static volatile int pose_snapshot_requested = 0;
static char pose_snapshot_path[256] = POSE_DEFAULT_SNAPSHOT_PATH;


// 舵机控制参数
#define PWM_PATH0 "/sys/class/pwm/pwmchip0/pwm0/"
#define PWM_PATH1 "/sys/class/pwm/pwmchip1/pwm0/"
#define PWM_PERIOD_NS 4000000
#define MIN_DUTY_X_NS 2150000
#define MAX_DUTY_X_NS 2750000
#define MID_DUTY_X_NS 2450000
#define MIN_DUTY_Y_NS 1600000
#define MAX_DUTY_Y_NS 3200000
#define MID_DUTY_Y_NS 2450000

// 三状态机参数
#define TRACK_CONF_HIGH               0.50f
#define IOU_MATCH_THRESH              0.50f
#define DETECTION_INTERVAL_TRACKING   20
#define PREDICT_ENTER_FRAMES          5
#define DETECTING_RETURN_FRAMES       60
#define TIMEOUT_NS (5 * G_TIME_SPAN_SECOND)

// DETECTING状态下 X 轴慢速扫描参数
#define SEARCH_SCAN_STEP_NS           6000      // 每次扫描步长，尽量小，保证慢速移动
#define SEARCH_SCAN_MARGIN_NS         30000     // 留出边界，避免撞到机械限位
#define SEARCH_SCAN_FRAME_INTERVAL    4         // 每4帧移动一次，降低扫描速度

// 目标类别定义
#define PERSON_CLASS_ID 0
static const char *PERSON_CLASS_NAME = "adapter_ring";

// 全局占空比变量
static int current_duty_x = MID_DUTY_X_NS;
static int current_duty_y = MID_DUTY_Y_NS;
static int last_duty_y = MID_DUTY_Y_NS;
static gint64 last_detection_time = 0;

/*-------------------------------------------
               状态机全局变量
-------------------------------------------*/
// 左目作为主控相机，沿用原三状态机和原 kcf_tracker
static KCFTrackerHandle kcf_tracker = NULL;
static TrackerState tracker_state = TRACKER_STATE_DETECTING;

// 右目只用于辅助检测/跟踪和画面绘制，不参与 PID 舵机控制
static KCFTrackerHandle right_kcf_tracker = NULL;
static TrackerState right_tracker_state = TRACKER_STATE_DETECTING;
static int right_lost_frames = 0;
static int right_stable_track_frames = 0;

static int frame_count = 0;
static int lost_frames = 0;
static int stable_track_frames = 0;

// X轴扫描状态
static int search_duty_x = MID_DUTY_X_NS;
static int search_scan_dir = 1; // 1: 向右, -1: 向左

/*-------------------------------------------
                 PID控制器结构体
-------------------------------------------*/
typedef struct {
    double kp;
    double ki;
    double kd;
    double integral;
    double prev_error;
    gint64 prev_time;
    double output_min;
    double output_max;
} PIDController;

typedef struct {
    int x;
    int y;
} offset_t;

typedef struct {
    bool found;
    int x;
    int y;
    int w;
    int h;
    float conf;
} DetectionResult;

typedef struct {
    bool found;
    int x;
    int y;
    int w;
    int h;
    float conf;
} TrackResult;

typedef struct {
    bool valid;
    int x;
    int y;
    int w;
    int h;
    float conf;
} TargetResult;

// 全局PID控制器
static PIDController pid_x = {0};
static PIDController pid_y = {0};

/*-------------------------------------------
          运行状态上报：跟踪状态 / 舵机占空比
-------------------------------------------*/
#define STATUS_REPORT_INTERVAL_US (500 * 1000)  // 500ms 上报一次

static gint64 status_last_report_time = 0;

static const char* tracker_state_name(TrackerState state) {
    switch (state) {
        case TRACKER_STATE_TRACKING:
            return "TRACKING";
        case TRACKER_STATE_PREDICTING:
            return "PREDICTING";
        case TRACKER_STATE_DETECTING:
        default:
            return "DETECTING";
    }
}

/*
 * 输出一行专门给 RK3588 服务器解析的最小状态信息。
 *
 * Qt 上位机只显示三类信息：
 *   1. 视频接收帧率：由 Qt 端按收到的帧计算；
 *   2. 跟踪状态：由本函数输出 state；
 *   3. 舵机占空比：由本函数输出 duty_x / duty_y。
 *
 * 这一行以 @@DETECT_STATUS@@ 开头，服务器识别后会把后面的 JSON 直接发给 Qt。
 */
static void emit_runtime_status(gint64 current_time,
                                const offset_t *offset,
                                bool target_valid,
                                float confidence) {
    (void)offset;
    (void)target_valid;
    (void)confidence;

    if (status_last_report_time == 0) {
        status_last_report_time = current_time;
        return;
    }

    gint64 dt_us = current_time - status_last_report_time;
    if (dt_us < STATUS_REPORT_INTERVAL_US) {
        return;
    }

    printf("@@DETECT_STATUS@@ "
           "{\"type\":\"DETECT_STATUS\","
           "\"state\":\"%s\","
           "\"duty_x\":%d,"
           "\"duty_y\":%d}\n",
           tracker_state_name(tracker_state),
           current_duty_x,
           current_duty_y);

    fflush(stdout);
    status_last_report_time = current_time;
}



/*-------------------------------------------
          位姿测量：命名管道控制与原始帧保存
-------------------------------------------*/
static void pose_snapshot_set_request(const char *path)
{
    if (path && path[0] != '\0') {
        snprintf(pose_snapshot_path, sizeof(pose_snapshot_path), "%s", path);
    } else {
        snprintf(pose_snapshot_path, sizeof(pose_snapshot_path), "%s", POSE_DEFAULT_SNAPSHOT_PATH);
    }

    pose_snapshot_requested = 1;
    printf("@@POSE_SNAPSHOT@@ {\"type\":\"POSE_SNAPSHOT\",\"event\":\"requested\",\"path\":\"%s\"}\n",
           pose_snapshot_path);
    fflush(stdout);
}

static void init_pose_command_fifo(void)
{
    unlink(POSE_CMD_FIFO);
    if (mkfifo(POSE_CMD_FIFO, 0666) != 0 && errno != EEXIST) {
        perror("mkfifo POSE_CMD_FIFO failed");
        return;
    }

    pose_cmd_fd = open(POSE_CMD_FIFO, O_RDONLY | O_NONBLOCK);
    if (pose_cmd_fd < 0) {
        perror("open POSE_CMD_FIFO read failed");
        return;
    }

    /*
     * 保持一个写端，避免没有外部 writer 时 read() 反复 EOF。
     */
    pose_cmd_dummy_fd = open(POSE_CMD_FIFO, O_WRONLY | O_NONBLOCK);

    printf("[POSE] Command FIFO ready: %s\n", POSE_CMD_FIFO);
    fflush(stdout);
}

static void cleanup_pose_command_fifo(void)
{
    if (pose_cmd_fd >= 0) {
        close(pose_cmd_fd);
        pose_cmd_fd = -1;
    }
    if (pose_cmd_dummy_fd >= 0) {
        close(pose_cmd_dummy_fd);
        pose_cmd_dummy_fd = -1;
    }
    unlink(POSE_CMD_FIFO);
}

static void poll_pose_command_fifo(void)
{
    if (pose_cmd_fd < 0) {
        return;
    }

    char buf[512];
    while (1) {
        ssize_t n = read(pose_cmd_fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';

            char *p = strstr(buf, "SNAPSHOT");
            if (p) {
                p += strlen("SNAPSHOT");
                while (*p == ' ' || *p == '\t') {
                    ++p;
                }

                char path[256] = {0};
                int i = 0;
                while (*p && *p != '\n' && *p != '\r' && i < (int)sizeof(path) - 1) {
                    path[i++] = *p++;
                }
                path[i] = '\0';

                pose_snapshot_set_request(path[0] ? path : POSE_DEFAULT_SNAPSHOT_PATH);
            }

            continue;
        }

        if (n == 0) {
            return;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return;
        }

        return;
    }
}

static int save_raw_stereo_snapshot_rgb(const unsigned char *rgb_data,
                                        size_t rgb_size,
                                        const char *path)
{
    if (!rgb_data || !path || path[0] == '\0') {
        return -1;
    }

    const size_t expected = (size_t)STEREO_WIDTH * STEREO_HEIGHT * 3;
    if (rgb_size < expected) {
        return -2;
    }

    cv::Mat rgb(STEREO_HEIGHT, STEREO_WIDTH, CV_8UC3, (void*)rgb_data);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(95);

    if (!cv::imwrite(path, bgr, params)) {
        return -3;
    }

    return 0;
}

static void save_snapshot_if_requested(const unsigned char *rgb_data, size_t rgb_size)
{
    if (!pose_snapshot_requested) {
        return;
    }

    char path[256];
    snprintf(path, sizeof(path), "%s", pose_snapshot_path);

    int ret = save_raw_stereo_snapshot_rgb(rgb_data, rgb_size, path);
    if (ret == 0) {
        printf("@@POSE_SNAPSHOT@@ "
               "{\"type\":\"POSE_SNAPSHOT\",\"valid\":true,\"path\":\"%s\",\"width\":%d,\"height\":%d}\n",
               path, STEREO_WIDTH, STEREO_HEIGHT);
    } else {
        printf("@@POSE_SNAPSHOT@@ "
               "{\"type\":\"POSE_SNAPSHOT\",\"valid\":false,\"path\":\"%s\",\"error\":\"save raw stereo snapshot failed\",\"code\":%d}\n",
               path, ret);
    }

    fflush(stdout);
    pose_snapshot_requested = 0;
}


/*-------------------------------------------
                PID控制器初始化
-------------------------------------------*/
static void pid_init(PIDController *pid, double kp, double ki, double kd,
                     double output_min, double output_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->prev_time = g_get_monotonic_time();
    pid->output_min = output_min;
    pid->output_max = output_max;
}

/*-------------------------------------------
                PID计算函数
-------------------------------------------*/
static double pid_calculate(PIDController *pid, double error) {
    gint64 current_time = g_get_monotonic_time();
    double dt = (current_time - pid->prev_time) / 1000000.0;

    if (dt <= 0) {
        dt = 0.01;
    }

    double proportional = pid->kp * error;
    pid->integral += error * dt;

    double integral_max = (pid->ki != 0)
                            ? ((pid->output_max - pid->output_min) / pid->ki)
                            : INFINITY;
    if (pid->integral > integral_max) pid->integral = integral_max;
    if (pid->integral < -integral_max) pid->integral = -integral_max;

    double integral = pid->ki * pid->integral;
    double derivative = 0.0;
    if (pid->prev_time > 0) {
        derivative = pid->kd * (error - pid->prev_error) / dt;
    }

    double output = proportional + integral + derivative;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    pid->prev_error = error;
    pid->prev_time = current_time;
    return output;
}

/*-------------------------------------------
                PID控制器重置
-------------------------------------------*/
static void pid_reset(PIDController *pid) {
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->prev_time = g_get_monotonic_time();
}

/*-------------------------------------------
                  PWM控制函数
-------------------------------------------*/
static void set_pwm_duty(int duty_ns, const char *pwm_path) {
    static int initialized0 = 0;
    static int initialized1 = 0;
    char buffer[32];
    int fd;

    if (strcmp(pwm_path, PWM_PATH0) == 0 && !initialized0) {
        fd = open("/sys/class/pwm/pwmchip0/export", O_WRONLY);
        if (fd >= 0) {
            write(fd, "0", 1);
            close(fd);
        }

        fd = open(PWM_PATH0 "period", O_WRONLY);
        if (fd >= 0) {
            snprintf(buffer, sizeof(buffer), "%d", PWM_PERIOD_NS);
            write(fd, buffer, strlen(buffer));
            close(fd);
        }

        fd = open(PWM_PATH0 "enable", O_WRONLY);
        if (fd >= 0) {
            write(fd, "1", 1);
            close(fd);
        }
        initialized0 = 1;
    } else if (strcmp(pwm_path, PWM_PATH1) == 0 && !initialized1) {
        fd = open("/sys/class/pwm/pwmchip1/export", O_WRONLY);
        if (fd >= 0) {
            write(fd, "0", 1);
            close(fd);
        }

        fd = open(PWM_PATH1 "period", O_WRONLY);
        if (fd >= 0) {
            snprintf(buffer, sizeof(buffer), "%d", PWM_PERIOD_NS);
            write(fd, buffer, strlen(buffer));
            close(fd);
        }

        fd = open(PWM_PATH1 "enable", O_WRONLY);
        if (fd >= 0) {
            write(fd, "1", 1);
            close(fd);
        }
        initialized1 = 1;
    }

    duty_ns = duty_ns < 0 ? 0 : duty_ns;
    duty_ns = duty_ns > PWM_PERIOD_NS ? PWM_PERIOD_NS : duty_ns;

    char duty_path[256];
    snprintf(duty_path, sizeof(duty_path), "%sduty_cycle", pwm_path);
    fd = open(duty_path, O_WRONLY);
    if (fd >= 0) {
        snprintf(buffer, sizeof(buffer), "%d", duty_ns);
        write(fd, buffer, strlen(buffer));
        close(fd);
    } else {
        perror("Failed to set PWM duty");
    }
}

/*-------------------------------------------
               禁用PWM输出函数
-------------------------------------------*/
static void disable_pwm() {
    int fd;
    const char *disable_value = "0";

    fd = open("/sys/class/pwm/pwmchip0/pwm0/enable", O_WRONLY);
    if (fd >= 0) {
        write(fd, disable_value, 1);
        close(fd);
        printf("[PWM] Disabled pwmchip0/pwm0\n");
    } else {
        perror("Failed to disable pwmchip0/pwm0");
    }

    fd = open("/sys/class/pwm/pwmchip1/pwm0/enable", O_WRONLY);
    if (fd >= 0) {
        write(fd, disable_value, 1);
        close(fd);
    } else {
        perror("Failed to disable pwmchip1/pwm0");
    }
}

/*-------------------------------------------
                信号处理函数
-------------------------------------------*/
static gboolean handle_signal(gpointer user_data) {
    GMainLoop *loop = (GMainLoop *)user_data;
    g_print("\nStopping gracefully...\n");

    set_pwm_duty(MID_DUTY_Y_NS, PWM_PATH0);
    set_pwm_duty(MID_DUTY_X_NS, PWM_PATH1);
    disable_pwm();
    cleanup_pose_command_fifo();

    if (kcf_tracker) {
        kcf_tracker_destroy(kcf_tracker);
        kcf_tracker = NULL;
    }

    if (right_kcf_tracker) {
        kcf_tracker_destroy(right_kcf_tracker);
        right_kcf_tracker = NULL;
    }

    g_main_loop_quit(loop);
    return TRUE;
}

/*-------------------------------------------
              GStreamer Callbacks
-------------------------------------------*/
static gboolean bus_callback(GstBus *bus, GstMessage *msg, gpointer data) {
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err = NULL;
            gchar *debug_info = NULL;
            gst_message_parse_error(msg, &err, &debug_info);
            g_printerr("Error: %s\nDebug info: %s\n", err->message, debug_info);
            g_clear_error(&err);
            g_free(debug_info);
            g_main_loop_quit(main_loop);
            break;
        }
        case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            g_main_loop_quit(main_loop);
            break;
        default:
            break;
    }
    return TRUE;
}

/*-------------------------------------------
              JPEG 内存解码函数
-------------------------------------------*/
static int decode_jpeg_memory(const unsigned char *jpeg_data, unsigned long jpeg_size, image_buffer_t *image) {
    int width, height;
    int origin_width, origin_height;
    int subsample, colorspace;
    unsigned char *rgb_buf = NULL;
    tjhandle handle = NULL;
    int flags = 0;
    int ret = 0;

    handle = tjInitDecompress();
    if (!handle) {
        g_printerr("tjInitDecompress failed: %s\n", tjGetErrorStr());
        return -1;
    }

    ret = tjDecompressHeader3(handle, jpeg_data, jpeg_size, &origin_width, &origin_height, &subsample, &colorspace);
    if (ret < 0) {
        g_printerr("JPEG header error: %s\n", tjGetErrorStr());
        tjDestroy(handle);
        return -1;
    }

    width = origin_width;
    height = origin_height;
    unsigned long rgb_size = width * height * 3;
    rgb_buf = (unsigned char *)malloc(rgb_size);
    if (!rgb_buf) {
        g_printerr("Memory allocation failed for RGB buffer\n");
        tjDestroy(handle);
        return -1;
    }

    ret = tjDecompress2(handle, jpeg_data, jpeg_size, rgb_buf,
                        width, 0, height, TJPF_RGB, flags);
    if (ret < 0) {
        g_printerr("JPEG decompress failed: %s\n", tjGetErrorStr());
        free(rgb_buf);
        tjDestroy(handle);
        return -1;
    }

    image->width = width;
    image->height = height;
    image->format = IMAGE_FORMAT_RGB888;
    image->virt_addr = rgb_buf;
    image->size = rgb_size;

    tjDestroy(handle);
    return 0;
}

/*-------------------------------------------
              创建显示管道
-------------------------------------------*/
int create_display_pipeline(int width, int height, const char *host, int port) {
    display_width = width;
    display_height = height;

    display_pipeline = gst_pipeline_new("rtp-stream-pipeline");
    display_appsrc = gst_element_factory_make("appsrc", "display-source");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "convert");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "jpeg-caps");
    GstCaps *jpeg_caps = gst_caps_new_simple("video/x-raw",
                                             "format", G_TYPE_STRING, "I420",
                                             NULL);

    GstElement *jpegenc = gst_element_factory_make("jpegenc", "jpeg-encoder");
    GstElement *rtpjpegpay = gst_element_factory_make("rtpjpegpay", "rtp-payload");
    GstElement *udpsink = gst_element_factory_make("udpsink", "udp-sink");

    if (!display_pipeline || !display_appsrc || !videoconvert || !jpegenc || !rtpjpegpay || !udpsink) {
        g_printerr("Failed to create RTP pipeline elements\n");
        return -1;
    }

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "RGB",
                                        "width", G_TYPE_INT, width,
                                        "height", G_TYPE_INT, height,
                                        "framerate", GST_TYPE_FRACTION, 15, 1,
                                        NULL);
    g_object_set(display_appsrc,
                 "caps", caps,
                 "format", GST_FORMAT_TIME,
                 "block", TRUE,
                 "stream-type", 0,
                 "is-live", TRUE,
                 NULL);
    gst_caps_unref(caps);

    g_object_set(jpegenc,
                 "quality", 85,
                 "idct-method", 0,
                 NULL);

    g_object_set(capsfilter, "caps", jpeg_caps, NULL);
    gst_caps_unref(jpeg_caps);
    g_object_set(rtpjpegpay, "pt", 26, NULL);

    g_object_set(udpsink,
                 "host", host,
                 "port", port,
                 "sync", FALSE,
                 "async", FALSE,
                 NULL);

    gst_bin_add_many(GST_BIN(display_pipeline),
                     display_appsrc, videoconvert, capsfilter, jpegenc, rtpjpegpay, udpsink, NULL);
    if (!gst_element_link_many(display_appsrc, videoconvert, capsfilter, jpegenc, rtpjpegpay, udpsink, NULL)) {
        g_printerr("Failed to link elements!\n");
        gst_object_unref(display_pipeline);
        return -1;
    }

    GstStateChangeReturn ret = gst_element_set_state(display_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Failed to start RTP pipeline: %s\n",
                   gst_element_state_change_return_get_name(ret));
        gst_object_unref(display_pipeline);
        return -1;
    }

    return 0;
}

/*-------------------------------------------
              推送图像到显示管道
-------------------------------------------*/
static void push_frame_to_display(const image_buffer_t *image) {
    if (!display_appsrc) {
        g_printerr("Error: display_appsrc is NULL!\n");
        return;
    }

    GstBuffer *buffer = gst_buffer_new_allocate(NULL, image->size, NULL);
    if (!buffer) {
        g_printerr("Failed to allocate buffer!\n");
        return;
    }

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        memcpy(map.data, image->virt_addr, image->size);
        gst_buffer_unmap(buffer, &map);

        GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(
            g_get_monotonic_time(), GST_SECOND, G_USEC_PER_SEC);

        GstFlowReturn flow_ret;
        g_signal_emit_by_name(display_appsrc, "push-buffer", buffer, &flow_ret);
        if (flow_ret != GST_FLOW_OK) {
            g_printerr("Failed to push buffer: %s\n", gst_flow_get_name(flow_ret));
        }
        gst_buffer_unref(buffer);
    } else {
        g_printerr("Failed to map buffer for writing!\n");
        gst_buffer_unref(buffer);
    }
}

/*-------------------------------------------
              查找最高概率的目标
-------------------------------------------*/
static object_detect_result* find_best_person(object_detect_result_list *results) {
    object_detect_result* best_person = NULL;
    float max_prob = 0.5f;

    for (int i = 0; i < results->count; i++) {
        object_detect_result *det_result = &(results->results[i]);
        if (det_result->cls_id == PERSON_CLASS_ID) {
            if (det_result->prop > max_prob) {
                max_prob = det_result->prop;
                best_person = det_result;
            }
        }
    }

    return best_person;
}

/*-------------------------------------------
              计算两个矩形的IOU
-------------------------------------------*/
static float calculate_iou(int x1, int y1, int w1, int h1,
                           int x2, int y2, int w2, int h2) {
    int inter_left = (x1 > x2) ? x1 : x2;
    int inter_top = (y1 > y2) ? y1 : y2;
    int inter_right = (x1 + w1 < x2 + w2) ? (x1 + w1) : (x2 + w2);
    int inter_bottom = (y1 + h1 < y2 + h2) ? (y1 + h1) : (y2 + h2);

    if (inter_right <= inter_left || inter_bottom <= inter_top) {
        return 0.0f;
    }

    int inter_area = (inter_right - inter_left) * (inter_bottom - inter_top);
    int area1 = w1 * h1;
    int area2 = w2 * h2;
    int union_area = area1 + area2 - inter_area;
    if (union_area <= 0) return 0.0f;
    return (float)inter_area / union_area;
}

/*-------------------------------------------
        计算中心偏移量
-------------------------------------------*/
static offset_t calculate_offset(int center_x, int center_y,
                                 int screen_width, int screen_height) {
    offset_t offset = {0, 0};
    offset.x = center_x - (screen_width / 2);
    offset.y = center_y - (screen_height / 2);
    return offset;
}

/*-------------------------------------------
        DETECTING状态：X轴慢速扫描函数
-------------------------------------------*/
static void update_detecting_scan_x(void) {
    if (frame_count % SEARCH_SCAN_FRAME_INTERVAL != 0) {
        return;
    }

    const int scan_min = MIN_DUTY_X_NS + SEARCH_SCAN_MARGIN_NS;
    const int scan_max = MAX_DUTY_X_NS - SEARCH_SCAN_MARGIN_NS;

    search_duty_x += search_scan_dir * SEARCH_SCAN_STEP_NS;

    if (search_duty_x >= scan_max) {
        search_duty_x = scan_max;
        search_scan_dir = -1;
    } else if (search_duty_x <= scan_min) {
        search_duty_x = scan_min;
        search_scan_dir = 1;
    }

    current_duty_x = search_duty_x;
    set_pwm_duty(current_duty_x, PWM_PATH1);
}

/*-------------------------------------------
        云台归中函数
-------------------------------------------*/
static void center_gimbal(void) {
    current_duty_x = MID_DUTY_X_NS;
    current_duty_y = MID_DUTY_Y_NS;
    last_duty_y = MID_DUTY_Y_NS;
    search_duty_x = MID_DUTY_X_NS;
    search_scan_dir = 1;
    set_pwm_duty(current_duty_x, PWM_PATH1);
    set_pwm_duty(current_duty_y, PWM_PATH0);
}

/*-------------------------------------------
        PID控制舵机输出占空比
-------------------------------------------*/
static void calculate_dutyset_pid(offset_t offset) {
    int dead_zone_x = 30;
    int dead_zone_y = 30;

    if (abs(offset.x) > dead_zone_x) {
        double error_x = (double)offset.x / (display_width / 2.0);
        double output_x = pid_calculate(&pid_x, error_x);
        current_duty_x = MID_DUTY_X_NS + (int)(output_x * 100000);
        if (current_duty_x > MAX_DUTY_X_NS) current_duty_x = MAX_DUTY_X_NS;
        if (current_duty_x < MIN_DUTY_X_NS) current_duty_x = MIN_DUTY_X_NS;
        set_pwm_duty(current_duty_x, PWM_PATH1);
    } else {
        pid_reset(&pid_x);
        current_duty_x = MID_DUTY_X_NS;
        set_pwm_duty(current_duty_x, PWM_PATH1);
    }

    if (abs(offset.y) > dead_zone_y) {
        double error_y = (double)offset.y / (display_height / 2.0);
        double output_y = pid_calculate(&pid_y, error_y);
        current_duty_y = last_duty_y + (int)(-1 * output_y * 100000);
        if (current_duty_y > MAX_DUTY_Y_NS) current_duty_y = MAX_DUTY_Y_NS;
        if (current_duty_y < MIN_DUTY_Y_NS) current_duty_y = MIN_DUTY_Y_NS;
        set_pwm_duty(current_duty_y, PWM_PATH0);
        last_duty_y = current_duty_y;
    } else {
        pid_reset(&pid_y);
        set_pwm_duty(last_duty_y, PWM_PATH0);
    }
}

/*-------------------------------------------
        绘制屏幕中心十字线和PID信息
-------------------------------------------*/
static void draw_center_and_pid_info(image_buffer_t *image, const offset_t *offset) {
    int center_x = image->width / 2;
    int center_y = image->height / 2;

    int cross_size = 20;
    draw_line(image, center_x - cross_size, center_y, center_x + cross_size, center_y, COLOR_GREEN, 2);
    draw_line(image, center_x, center_y - cross_size, center_x, center_y + cross_size, COLOR_GREEN, 2);

    char offset_text[128];
    sprintf(offset_text, "Offset: X=%d, Y=%d", offset->x, offset->y);
    draw_text(image, offset_text, 10, 30, COLOR_GREEN, 12);

    char dutyset_text[128];
    sprintf(dutyset_text, "Duty: X=%d, Y=%d", current_duty_x, current_duty_y);
    draw_text(image, dutyset_text, 10, 50, COLOR_GREEN, 12);

    char pid_text[128];
    sprintf(pid_text, "PID_X: P=%.2f I=%.2f D=%.2f", pid_x.kp, pid_x.ki, pid_x.kd);
    draw_text(image, pid_text, 10, 70, COLOR_ORANGE, 10);

    sprintf(pid_text, "PID_Y: P=%.2f I=%.2f D=%.2f", pid_y.kp, pid_y.ki, pid_y.kd);
    draw_text(image, pid_text, 10, 85, COLOR_ORANGE, 10);
}

/*-------------------------------------------
             绘制跟踪信息
-------------------------------------------*/
static void draw_tracking_info(image_buffer_t *image,
                               int track_x, int track_y, int track_w, int track_h,
                               TrackerState state, float confidence) {
    unsigned int color;
    char state_text[32];

    switch (state) {
        case TRACKER_STATE_TRACKING:
            color = COLOR_GREEN;
            sprintf(state_text, "TRACKING");
            break;
        case TRACKER_STATE_PREDICTING:
            color = COLOR_YELLOW;
            sprintf(state_text, "PREDICTING");
            break;
        case TRACKER_STATE_DETECTING:
        default:
            color = COLOR_BLUE;
            sprintf(state_text, "DETECTING");
            break;
    }

    draw_rectangle(image, track_x, track_y, track_w, track_h, color, 3);

    char text[128];
    sprintf(text, "KCF: %s (%.1f%%)", state_text, confidence * 100);
    draw_text(image, text, track_x, track_y - 20, color, 10);
}

/*-------------------------------------------
             虚线矩形绘制函数
-------------------------------------------*/
static void draw_rectangle_dashed(image_buffer_t *image, int x, int y, int width, int height,
                                  unsigned int color, int thickness, int dash_length) {
    for (int i = x; i < x + width; i += dash_length * 2) {
        int end = i + dash_length;
        if (end > x + width) end = x + width;
        draw_line(image, i, y, end, y, color, thickness);
    }

    for (int i = x; i < x + width; i += dash_length * 2) {
        int end = i + dash_length;
        if (end > x + width) end = x + width;
        draw_line(image, i, y + height, end, y + height, color, thickness);
    }

    for (int i = y; i < y + height; i += dash_length * 2) {
        int end = i + dash_length;
        if (end > y + height) end = y + height;
        draw_line(image, x, i, x, end, color, thickness);
    }

    for (int i = y; i < y + height; i += dash_length * 2) {
        int end = i + dash_length;
        if (end > y + height) end = y + height;
        draw_line(image, x + width, i, x + width, end, color, thickness);
    }
}

/*-------------------------------------------
              状态策略判定
-------------------------------------------*/
static void decide_run_policy(bool *run_yolo, bool *run_kcf) {
    *run_yolo = false;
    *run_kcf = false;

    switch (tracker_state) {
        case TRACKER_STATE_DETECTING:
            *run_yolo = true;
            *run_kcf = false;
            break;
        case TRACKER_STATE_TRACKING:
            *run_yolo = (frame_count % DETECTION_INTERVAL_TRACKING == 0);
            *run_kcf = true;
            break;
        case TRACKER_STATE_PREDICTING:
            *run_yolo = true;
            *run_kcf = true;
            break;
        default:
            *run_yolo = true;
            *run_kcf = false;
            tracker_state = TRACKER_STATE_DETECTING;
            break;
    }
}

/*-------------------------------------------
              执行YOLO检测
-------------------------------------------*/
static DetectionResult run_detection_if_needed(image_buffer_t *src_image, bool run_yolo) {
    DetectionResult det = {0};
    if (!run_yolo) {
        return det;
    }

    object_detect_result_list od_results;
    memset(&od_results, 0, sizeof(object_detect_result_list));

    int ret = inference_yolov8_model(&rknn_app_ctx, src_image, &od_results);
    if (ret != 0) {
        g_printerr("Inference failed: %d\n", ret);
        return det;
    }

    object_detect_result *best_person = find_best_person(&od_results);
    if (best_person) {
        det.found = true;
        det.x = best_person->box.left;
        det.y = best_person->box.top;
        det.w = best_person->box.right - best_person->box.left;
        det.h = best_person->box.bottom - best_person->box.top;
        det.conf = best_person->prop;
    }

    return det;
}

/*-------------------------------------------
              执行KCF更新
-------------------------------------------*/
static TrackResult run_tracking_if_needed(image_buffer_t *src_image, bool run_kcf) {
    TrackResult trk = {0};
    if (!run_kcf || !kcf_tracker) {
        return trk;
    }

    int x, y, w, h;
    float conf = 0.0f;
    if (kcf_tracker_update(kcf_tracker,
                           src_image->virt_addr,
                           src_image->width,
                           src_image->height,
                           &x, &y, &w, &h, &conf)) {
        if (conf >= TRACK_CONF_HIGH) {
            trk.found = true;
            trk.x = x;
            trk.y = y;
            trk.w = w;
            trk.h = h;
            trk.conf = conf;
        }
    }

    return trk;
}

/*-------------------------------------------
              DETECTING状态处理
-------------------------------------------*/
static TargetResult handle_detecting_state(image_buffer_t *src_image, DetectionResult det) {
    TargetResult target = {0};

    // DETECTING下维持Y中位，X轴慢速扫描
    set_pwm_duty(MID_DUTY_Y_NS, PWM_PATH0);
    last_duty_y = MID_DUTY_Y_NS;
    current_duty_y = MID_DUTY_Y_NS;
    //update_detecting_scan_x();
    current_duty_x = MID_DUTY_X_NS;
    set_pwm_duty(current_duty_x, PWM_PATH1);


    if (det.found) {
        if (kcf_tracker_init(kcf_tracker,
                             src_image->virt_addr,
                             src_image->width,
                             src_image->height,
                             det.x, det.y, det.w, det.h)) {
            target.valid = true;
            target.x = det.x;
            target.y = det.y;
            target.w = det.w;
            target.h = det.h;
            target.conf = det.conf;

            lost_frames = 0;
            stable_track_frames = 0;
            tracker_state = TRACKER_STATE_TRACKING;
            last_detection_time = g_get_monotonic_time();
            g_print("[STATE] DETECTING -> TRACKING\n");
        }
    }

    return target;
}

/*-------------------------------------------
              TRACKING状态处理
-------------------------------------------*/
static TargetResult handle_tracking_state(image_buffer_t *src_image,
                                          DetectionResult det,
                                          TrackResult trk) {
    TargetResult target = {0};

    if (det.found && trk.found) {
        float iou = calculate_iou(det.x, det.y, det.w, det.h,
                                  trk.x, trk.y, trk.w, trk.h);
        if (iou >= IOU_MATCH_THRESH) {
            target.valid = true;
            target.x = trk.x;
            target.y = trk.y;
            target.w = trk.w;
            target.h = trk.h;
            target.conf = trk.conf;
            lost_frames = 0;
            stable_track_frames++;
        } else {
            kcf_tracker_init(kcf_tracker,
                             src_image->virt_addr,
                             src_image->width,
                             src_image->height,
                             det.x, det.y, det.w, det.h);
            target.valid = true;
            target.x = det.x;
            target.y = det.y;
            target.w = det.w;
            target.h = det.h;
            target.conf = det.conf;
            lost_frames = 0;
            stable_track_frames = 0;
        }
        return target;
    }

    if (det.found && !trk.found) {
        kcf_tracker_init(kcf_tracker,
                         src_image->virt_addr,
                         src_image->width,
                         src_image->height,
                         det.x, det.y, det.w, det.h);
        target.valid = true;
        target.x = det.x;
        target.y = det.y;
        target.w = det.w;
        target.h = det.h;
        target.conf = det.conf;
        lost_frames = 0;
        stable_track_frames = 0;
        return target;
    }

    if (!det.found && trk.found) {
        target.valid = true;
        target.x = trk.x;
        target.y = trk.y;
        target.w = trk.w;
        target.h = trk.h;
        target.conf = trk.conf;
        lost_frames = 0;
        stable_track_frames++;
        return target;
    }

    lost_frames++;
    current_duty_x = MID_DUTY_X_NS;
    set_pwm_duty(current_duty_x, PWM_PATH1);
    if (lost_frames > PREDICT_ENTER_FRAMES) {
        tracker_state = TRACKER_STATE_PREDICTING;
        g_print("[STATE] TRACKING -> PREDICTING\n");
    }
    return target;
}

/*-------------------------------------------
              PREDICTING状态处理
-------------------------------------------*/
static TargetResult handle_predicting_state(image_buffer_t *src_image,
                                            DetectionResult det,
                                            TrackResult trk) {
    TargetResult target = {0};

    if (det.found) {
        kcf_tracker_init(kcf_tracker,
                         src_image->virt_addr,
                         src_image->width,
                         src_image->height,
                         det.x, det.y, det.w, det.h);
        target.valid = true;
        target.x = det.x;
        target.y = det.y;
        target.w = det.w;
        target.h = det.h;
        target.conf = det.conf;
        lost_frames = 0;
        stable_track_frames = 0;
        tracker_state = TRACKER_STATE_TRACKING;
        last_detection_time = g_get_monotonic_time();
        g_print("[STATE] PREDICTING -> TRACKING\n");
        return target;
    }

    if (!det.found && trk.found) {
        target.valid = true;
        target.x = trk.x;
        target.y = trk.y;
        target.w = trk.w;
        target.h = trk.h;
        target.conf = trk.conf;
        lost_frames++;
        return target;
    }
    current_duty_y = last_duty_y;
    set_pwm_duty(current_duty_y, PWM_PATH0);

    //update_detecting_scan_x();
    current_duty_x = MID_DUTY_X_NS;
    set_pwm_duty(current_duty_x, PWM_PATH1);
    lost_frames++;
    if (lost_frames > DETECTING_RETURN_FRAMES) {
        kcf_tracker_reset(kcf_tracker);
        tracker_state = TRACKER_STATE_DETECTING;
        // current_duty_x = MID_DUTY_X_NS;
        // set_pwm_duty(current_duty_x, PWM_PATH1);
        //search_duty_x = MID_DUTY_X_NS;
        // if (search_duty_x < MIN_DUTY_X_NS + SEARCH_SCAN_MARGIN_NS ||
        //     search_duty_x > MAX_DUTY_X_NS - SEARCH_SCAN_MARGIN_NS) {
        //     search_duty_x = MID_DUTY_X_NS;
        // }
        g_print("[STATE] PREDICTING -> DETECTING\n");
    }
    return target;
}

/*-------------------------------------------
              绘制检测框
-------------------------------------------*/
static void draw_detection_result(image_buffer_t *src_image, DetectionResult det) {
    if (!det.found) return;

    draw_rectangle_dashed(src_image, det.x, det.y, det.w, det.h, COLOR_BLUE, 2, 5);
    char det_text[128];
    sprintf(det_text, "DET: %s %.1f%%", PERSON_CLASS_NAME, det.conf * 100.0f);
    draw_text(src_image, det_text, det.x, det.y - 40, COLOR_BLUE, 10);
}

/*-------------------------------------------
          双目拼接图像处理辅助函数
-------------------------------------------*/
static int copy_eye_from_stereo_rgb(const unsigned char *stereo_rgb,
                                    int stereo_w,
                                    int stereo_h,
                                    int eye_x,
                                    image_buffer_t *eye)
{
    if (!stereo_rgb || !eye || stereo_w <= 0 || stereo_h <= 0) {
        return -1;
    }

    memset(eye, 0, sizeof(image_buffer_t));
    eye->width = EYE_WIDTH;
    eye->height = EYE_HEIGHT;
    eye->format = IMAGE_FORMAT_RGB888;
    eye->size = EYE_WIDTH * EYE_HEIGHT * 3;
    eye->virt_addr = (unsigned char *)malloc(eye->size);
    if (!eye->virt_addr) {
        return -1;
    }

    for (int y = 0; y < EYE_HEIGHT; ++y) {
        const unsigned char *src_row = stereo_rgb + ((y * stereo_w + eye_x) * 3);
        unsigned char *dst_row = eye->virt_addr + (y * EYE_WIDTH * 3);
        memcpy(dst_row, src_row, EYE_WIDTH * 3);
    }

    return 0;
}

static int compose_stereo_rgb(const image_buffer_t *left,
                              const image_buffer_t *right,
                              image_buffer_t *stereo)
{
    if (!left || !right || !stereo || !left->virt_addr || !right->virt_addr) {
        return -1;
    }

    memset(stereo, 0, sizeof(image_buffer_t));
    stereo->width = STEREO_WIDTH;
    stereo->height = STEREO_HEIGHT;
    stereo->format = IMAGE_FORMAT_RGB888;
    stereo->size = STEREO_WIDTH * STEREO_HEIGHT * 3;
    stereo->virt_addr = (unsigned char *)malloc(stereo->size);
    if (!stereo->virt_addr) {
        return -1;
    }

    for (int y = 0; y < STEREO_HEIGHT; ++y) {
        unsigned char *dst_row = stereo->virt_addr + y * STEREO_WIDTH * 3;
        const unsigned char *left_row = left->virt_addr + y * EYE_WIDTH * 3;
        const unsigned char *right_row = right->virt_addr + y * EYE_WIDTH * 3;
        memcpy(dst_row, left_row, EYE_WIDTH * 3);
        memcpy(dst_row + EYE_WIDTH * 3, right_row, EYE_WIDTH * 3);
    }

    return 0;
}

static void draw_eye_name(image_buffer_t *image, const char *name, unsigned int color)
{
    if (!image || !name) return;
    draw_text(image, name, 10, 18, color, 12);
}

static void draw_simple_center_cross(image_buffer_t *image, unsigned int color)
{
    if (!image) return;
    int cx = image->width / 2;
    int cy = image->height / 2;
    int cross_size = 18;
    draw_line(image, cx - cross_size, cy, cx + cross_size, cy, color, 2);
    draw_line(image, cx, cy - cross_size, cx, cy + cross_size, color, 2);
}

/*-------------------------------------------
          右目辅助检测/跟踪，不参与舵机控制
-------------------------------------------*/
static void decide_right_run_policy(bool *run_yolo, bool *run_kcf)
{
    *run_yolo = false;
    *run_kcf = false;

    switch (right_tracker_state) {
        case TRACKER_STATE_DETECTING:
            *run_yolo = true;
            *run_kcf = false;
            break;
        case TRACKER_STATE_TRACKING:
            *run_yolo = (frame_count % DETECTION_INTERVAL_TRACKING == 0);
            *run_kcf = true;
            break;
        case TRACKER_STATE_PREDICTING:
            *run_yolo = true;
            *run_kcf = true;
            break;
        default:
            right_tracker_state = TRACKER_STATE_DETECTING;
            *run_yolo = true;
            *run_kcf = false;
            break;
    }
}

static TrackResult run_right_tracking_if_needed(image_buffer_t *src_image, bool run_kcf)
{
    TrackResult trk = {0};
    if (!run_kcf || !right_kcf_tracker) {
        return trk;
    }

    int x, y, w, h;
    float conf = 0.0f;
    if (kcf_tracker_update(right_kcf_tracker,
                           src_image->virt_addr,
                           src_image->width,
                           src_image->height,
                           &x, &y, &w, &h, &conf)) {
        if (conf >= TRACK_CONF_HIGH) {
            trk.found = true;
            trk.x = x;
            trk.y = y;
            trk.w = w;
            trk.h = h;
            trk.conf = conf;
        }
    }

    return trk;
}

static TargetResult handle_right_eye_state(image_buffer_t *src_image,
                                           DetectionResult det,
                                           TrackResult trk)
{
    TargetResult target = {0};

    switch (right_tracker_state) {
        case TRACKER_STATE_DETECTING:
            if (det.found && right_kcf_tracker) {
                if (kcf_tracker_init(right_kcf_tracker,
                                     src_image->virt_addr,
                                     src_image->width,
                                     src_image->height,
                                     det.x, det.y, det.w, det.h)) {
                    target.valid = true;
                    target.x = det.x;
                    target.y = det.y;
                    target.w = det.w;
                    target.h = det.h;
                    target.conf = det.conf;
                    right_lost_frames = 0;
                    right_stable_track_frames = 0;
                    right_tracker_state = TRACKER_STATE_TRACKING;
                    g_print("[RIGHT_STATE] DETECTING -> TRACKING\n");
                }
            }
            break;

        case TRACKER_STATE_TRACKING:
            if (det.found && trk.found) {
                float iou = calculate_iou(det.x, det.y, det.w, det.h,
                                          trk.x, trk.y, trk.w, trk.h);
                if (iou >= IOU_MATCH_THRESH) {
                    target.valid = true;
                    target.x = trk.x;
                    target.y = trk.y;
                    target.w = trk.w;
                    target.h = trk.h;
                    target.conf = trk.conf;
                    right_lost_frames = 0;
                    right_stable_track_frames++;
                } else if (right_kcf_tracker) {
                    kcf_tracker_init(right_kcf_tracker,
                                     src_image->virt_addr,
                                     src_image->width,
                                     src_image->height,
                                     det.x, det.y, det.w, det.h);
                    target.valid = true;
                    target.x = det.x;
                    target.y = det.y;
                    target.w = det.w;
                    target.h = det.h;
                    target.conf = det.conf;
                    right_lost_frames = 0;
                    right_stable_track_frames = 0;
                }
            } else if (det.found && right_kcf_tracker) {
                kcf_tracker_init(right_kcf_tracker,
                                 src_image->virt_addr,
                                 src_image->width,
                                 src_image->height,
                                 det.x, det.y, det.w, det.h);
                target.valid = true;
                target.x = det.x;
                target.y = det.y;
                target.w = det.w;
                target.h = det.h;
                target.conf = det.conf;
                right_lost_frames = 0;
                right_stable_track_frames = 0;
            } else if (trk.found) {
                target.valid = true;
                target.x = trk.x;
                target.y = trk.y;
                target.w = trk.w;
                target.h = trk.h;
                target.conf = trk.conf;
                right_lost_frames = 0;
                right_stable_track_frames++;
            } else {
                right_lost_frames++;
                if (right_lost_frames > PREDICT_ENTER_FRAMES) {
                    right_tracker_state = TRACKER_STATE_PREDICTING;
                    g_print("[RIGHT_STATE] TRACKING -> PREDICTING\n");
                }
            }
            break;

        case TRACKER_STATE_PREDICTING:
            if (det.found && right_kcf_tracker) {
                kcf_tracker_init(right_kcf_tracker,
                                 src_image->virt_addr,
                                 src_image->width,
                                 src_image->height,
                                 det.x, det.y, det.w, det.h);
                target.valid = true;
                target.x = det.x;
                target.y = det.y;
                target.w = det.w;
                target.h = det.h;
                target.conf = det.conf;
                right_lost_frames = 0;
                right_stable_track_frames = 0;
                right_tracker_state = TRACKER_STATE_TRACKING;
                g_print("[RIGHT_STATE] PREDICTING -> TRACKING\n");
            } else if (trk.found) {
                target.valid = true;
                target.x = trk.x;
                target.y = trk.y;
                target.w = trk.w;
                target.h = trk.h;
                target.conf = trk.conf;
                right_lost_frames++;
            } else {
                right_lost_frames++;
                if (right_lost_frames > DETECTING_RETURN_FRAMES) {
                    if (right_kcf_tracker) {
                        kcf_tracker_reset(right_kcf_tracker);
                    }
                    right_tracker_state = TRACKER_STATE_DETECTING;
                    right_lost_frames = 0;
                    g_print("[RIGHT_STATE] PREDICTING -> DETECTING\n");
                }
            }
            break;

        default:
            right_tracker_state = TRACKER_STATE_DETECTING;
            break;
    }

    return target;
}


/*-------------------------------------------
                主回调函数
-------------------------------------------*/
static GstFlowReturn new_sample_callback(GstAppSink *appsink, gpointer user_data) {
    (void)user_data;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        g_printerr("Failed to pull sample from appsink\n");
        return GST_FLOW_ERROR;
    }

    gint64 current_time = g_get_monotonic_time();

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        g_printerr("Failed to get buffer from sample\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        g_printerr("Failed to map buffer\n");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    const size_t expected_size = STEREO_WIDTH * STEREO_HEIGHT * 3;
    if (map.size < expected_size) {
        g_printerr("Stereo RGB buffer too small: got %lu, expected %lu\n",
                   (unsigned long)map.size, (unsigned long)expected_size);
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    /*
     * 位姿测量快照必须在任何检测框、文字、中心线绘制之前保存，
     * 使用 /dev/video21 的原始 RGB 双目拼接帧。
     */
    poll_pose_command_fifo();
    save_snapshot_if_requested(map.data, map.size);

    image_buffer_t left_image;
    image_buffer_t right_image;
    image_buffer_t stereo_display_image;
    memset(&left_image, 0, sizeof(left_image));
    memset(&right_image, 0, sizeof(right_image));
    memset(&stereo_display_image, 0, sizeof(stereo_display_image));

    if (copy_eye_from_stereo_rgb(map.data, STEREO_WIDTH, STEREO_HEIGHT, 0, &left_image) != 0 ||
        copy_eye_from_stereo_rgb(map.data, STEREO_WIDTH, STEREO_HEIGHT, EYE_WIDTH, &right_image) != 0) {
        g_printerr("Failed to split stereo frame into left/right images\n");
        if (left_image.virt_addr) free(left_image.virt_addr);
        if (right_image.virt_addr) free(right_image.virt_addr);
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    frame_count++;

    if (!kcf_tracker) {
        kcf_tracker = kcf_tracker_create();
        if (!kcf_tracker) {
            g_printerr("Failed to create LEFT KCF tracker\n");
        } else {
            g_print("LEFT KCF tracker created successfully\n");
        }
    }

    if (!right_kcf_tracker) {
        right_kcf_tracker = kcf_tracker_create();
        if (!right_kcf_tracker) {
            g_printerr("Failed to create RIGHT KCF tracker\n");
        } else {
            g_print("RIGHT KCF tracker created successfully\n");
        }
    }

    /* ---------------- 左目：主控检测跟踪 + PID 舵机控制 ---------------- */
    bool left_run_yolo = false;
    bool left_run_kcf = false;
    decide_run_policy(&left_run_yolo, &left_run_kcf);

    DetectionResult left_det = run_detection_if_needed(&left_image, left_run_yolo);
    TrackResult left_trk = run_tracking_if_needed(&left_image, left_run_kcf);
    TargetResult left_target = {0};
    offset_t left_offset = {0, 0};

    draw_detection_result(&left_image, left_det);

    switch (tracker_state) {
        case TRACKER_STATE_DETECTING:
            left_target = handle_detecting_state(&left_image, left_det);
            break;
        case TRACKER_STATE_TRACKING:
            left_target = handle_tracking_state(&left_image, left_det, left_trk);
            break;
        case TRACKER_STATE_PREDICTING:
            left_target = handle_predicting_state(&left_image, left_det, left_trk);
            break;
        default:
            tracker_state = TRACKER_STATE_DETECTING;
            left_target = handle_detecting_state(&left_image, left_det);
            break;
    }

    if (left_target.valid) {
        int target_center_x = left_target.x + left_target.w / 2;
        int target_center_y = left_target.y + left_target.h / 2;
        left_offset = calculate_offset(target_center_x, target_center_y,
                                       left_image.width, left_image.height);

        calculate_dutyset_pid(left_offset);
        draw_tracking_info(&left_image,
                           left_target.x, left_target.y,
                           left_target.w, left_target.h,
                           tracker_state, left_target.conf);

        last_detection_time = current_time;

        printf("[LEFT_FSM] %s | Conf: %.1f%% | Pos:(%d,%d) | Offset:(%d,%d) | Duty:(%d,%d)\n",
               tracker_state == TRACKER_STATE_TRACKING ? "TRACKING" :
               tracker_state == TRACKER_STATE_PREDICTING ? "PREDICTING" : "DETECTING",
               left_target.conf * 100.0f,
               target_center_x, target_center_y,
               left_offset.x, left_offset.y,
               current_duty_x, current_duty_y);
    } else {
        pid_reset(&pid_x);
        pid_reset(&pid_y);

        if (last_detection_time > 0 &&
            (current_time - last_detection_time) > TIMEOUT_NS) {
            g_print("LEFT timeout reached, center gimbal and reset KCF\n");
            last_detection_time = 0;
            center_gimbal();
            if (kcf_tracker) {
                kcf_tracker_reset(kcf_tracker);
            }
            tracker_state = TRACKER_STATE_DETECTING;
        }
    }

    draw_eye_name(&left_image, "LEFT MAIN CONTROL", COLOR_GREEN);
    draw_center_and_pid_info(&left_image, &left_offset);

    char left_state_text[128];
    const char *left_state_str = tracker_state == TRACKER_STATE_DETECTING ? "DETECTING" :
                                 tracker_state == TRACKER_STATE_TRACKING ? "TRACKING" : "PREDICTING";
    sprintf(left_state_text, "Left State: %s | Frame: %d | Lost: %d", left_state_str, frame_count, lost_frames);
    draw_text(&left_image, left_state_text, 10, 110, COLOR_GREEN, 10);

    /* ---------------- 右目：辅助检测跟踪 + 绘制，不参与 PID ---------------- */
    bool right_run_yolo = false;
    bool right_run_kcf = false;
    decide_right_run_policy(&right_run_yolo, &right_run_kcf);

    DetectionResult right_det = run_detection_if_needed(&right_image, right_run_yolo);
    TrackResult right_trk = run_right_tracking_if_needed(&right_image, right_run_kcf);
    TargetResult right_target = handle_right_eye_state(&right_image, right_det, right_trk);

    draw_detection_result(&right_image, right_det);
    draw_eye_name(&right_image, "RIGHT AUX TRACK", COLOR_ORANGE);
    draw_simple_center_cross(&right_image, COLOR_ORANGE);

    if (right_target.valid) {
        draw_tracking_info(&right_image,
                           right_target.x, right_target.y,
                           right_target.w, right_target.h,
                           right_tracker_state, right_target.conf);
    }

    char right_state_text[128];
    const char *right_state_str = right_tracker_state == TRACKER_STATE_DETECTING ? "DETECTING" :
                                  right_tracker_state == TRACKER_STATE_TRACKING ? "TRACKING" : "PREDICTING";
    sprintf(right_state_text, "Right State: %s | Lost: %d", right_state_str, right_lost_frames);
    draw_text(&right_image, right_state_text, 10, 110, COLOR_ORANGE, 10);

    if (compose_stereo_rgb(&left_image, &right_image, &stereo_display_image) != 0) {
        g_printerr("Failed to compose stereo display image\n");
        free(left_image.virt_addr);
        free(right_image.virt_addr);
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    draw_line(&stereo_display_image, EYE_WIDTH, 0, EYE_WIDTH, STEREO_HEIGHT - 1, COLOR_ORANGE, 2);

    /* 只给上位机状态区发送最小信息：跟踪状态 + 舵机占空比；视频 FPS 由 Qt 接收端自己计算 */
    emit_runtime_status(current_time, &left_offset, left_target.valid, left_target.conf);

    push_frame_to_display(&stereo_display_image);

    free(left_image.virt_addr);
    free(right_image.virt_addr);
    free(stereo_display_image.virt_addr);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

/*-------------------------------------------
              Model Initialization
-------------------------------------------*/
int init_detection_model(const char *model_path) {
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

    init_post_process();

    int ret = init_yolov8_model(model_path, &rknn_app_ctx);
    if (ret != 0) {
        g_printerr("Model initialization failed: %d\n", ret);
        return -1;
    }

    return 0;
}

/*-------------------------------------------
            Pipeline Initialization
-------------------------------------------*/
GstElement* create_capture_pipeline(const char* device, int width, int height, int fps) {
    (void)fps;

    /*
     * 双目同步摄像头已经在 /dev/video21 输出左右拼接画面。
     * 已验证可工作的采集链路等价于：
     * v4l2src device=/dev/video21 ! videoconvert ! video/x-raw,format=RGB,width=1280,height=480 ! appsink
     */
    GstElement *pipeline = gst_pipeline_new("stereo-capture-pipeline");
    GstElement *src = gst_element_factory_make("v4l2src", "source");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "convert");
    GstElement *capsfilter = gst_element_factory_make("capsfilter", "caps");
    GstElement *appsink = gst_element_factory_make("appsink", "sink");

    if (!pipeline || !src || !videoconvert || !capsfilter || !appsink) {
        g_printerr("Failed to create stereo capture pipeline elements\n");
        if (pipeline) gst_object_unref(pipeline);
        return NULL;
    }

    g_object_set(src,
                 "device", device,
                 NULL);

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "RGB",
        "width", G_TYPE_INT, width,
        "height", G_TYPE_INT, height,
        NULL);
    g_object_set(capsfilter, "caps", caps, NULL);
    gst_caps_unref(caps);

    g_object_set(appsink,
        "emit-signals", TRUE,
        "sync", FALSE,
        "max-buffers", 1,
        "drop", TRUE,
        NULL);

    g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample_callback), NULL);

    gst_bin_add_many(GST_BIN(pipeline),
        src, videoconvert, capsfilter, appsink, NULL);

    if (!gst_element_link_many(src, videoconvert, capsfilter, appsink, NULL)) {
        g_printerr("Failed to link stereo capture pipeline elements\n");
        gst_object_unref(pipeline);
        return NULL;
    }

    return pipeline;
}

/*-------------------------------------------
                  清理函数
-------------------------------------------*/
void cleanup_display_pipeline() {
    center_gimbal();
    disable_pwm();

    if (kcf_tracker) {
        kcf_tracker_destroy(kcf_tracker);
        kcf_tracker = NULL;
    }

    if (right_kcf_tracker) {
        kcf_tracker_destroy(right_kcf_tracker);
        right_kcf_tracker = NULL;
    }

    if (display_pipeline) {
        gst_element_set_state(display_pipeline, GST_STATE_NULL);
        gst_object_unref(display_pipeline);
        display_pipeline = NULL;
        display_appsrc = NULL;
    }
}

/*-------------------------------------------
              Main Function
-------------------------------------------*/
int main(int argc, char *argv[]) {
    /*
     * 让 stdout/stderr 尽量实时输出，便于 RK3588 server 捕获并转发给 Qt。
     */
    setvbuf(stdout, NULL, _IOLBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    if (argc != 2) {
        printf("Usage: %s <model_path>\n", argv[0]);
        return -1;
    }

    gst_init(&argc, &argv);
    gst_initialized = TRUE;

    if (init_detection_model(argv[1]) != 0) {
        return -1;
    }

    pid_init(&pid_x, 5.5, 0.1, 0.25, -200000, 200000);
    pid_init(&pid_y, 0.3, 0, 0.15, -200000, 200000);

    printf("PID Controllers Initialized:\n");
    printf("X-axis: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", pid_x.kp, pid_x.ki, pid_x.kd);
    printf("Y-axis: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", pid_y.kp, pid_y.ki, pid_y.kd);

    last_detection_time = g_get_monotonic_time();
    center_gimbal();
    init_pose_command_fifo();

    int out_width = STEREO_WIDTH;
    int out_height = STEREO_HEIGHT;
    const char *client_ip = "192.168.43.76";
    int client_port = 5000;
    if (create_display_pipeline(out_width, out_height, client_ip, client_port) != 0) {
        g_printerr("Failed to create RTP pipeline\n");
        return -1;
    }

    main_loop = g_main_loop_new(NULL, FALSE);
    g_unix_signal_add(SIGINT, handle_signal, main_loop);
    g_unix_signal_add(SIGTERM, handle_signal, main_loop);
    g_unix_signal_add(SIGQUIT, handle_signal, main_loop);

    const char *camera_device = "/dev/video21";
    int width = STEREO_WIDTH;
    int height = STEREO_HEIGHT;
    int fps = 30;

    GstElement *capture_pipeline = create_capture_pipeline(camera_device, width, height, fps);
    if (!capture_pipeline) {
        g_printerr("Failed to create capture pipeline\n");
        cleanup_display_pipeline();
        return -1;
    }

    GstBus *bus = gst_element_get_bus(capture_pipeline);
    gst_bus_add_watch(bus, bus_callback, main_loop);
    gst_object_unref(bus);

    GstStateChangeReturn ret = gst_element_set_state(capture_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Failed to start capture pipeline\n");
        gst_object_unref(capture_pipeline);
        cleanup_display_pipeline();
        return -1;
    }

    g_print("Starting stereo stage-1 detection-tracking system: LEFT controls gimbal, RIGHT auxiliary tracking...\n");
    g_print("Press Ctrl+C to exit\n");

    g_main_loop_run(main_loop);

    g_print("Stopping pipelines...\n");
    gst_element_set_state(capture_pipeline, GST_STATE_NULL);
    gst_object_unref(capture_pipeline);

    cleanup_display_pipeline();
    release_yolov8_model(&rknn_app_ctx);
    deinit_post_process();

    g_main_loop_unref(main_loop);
    g_print("Application exited cleanly\n");
    return 0;
}
















