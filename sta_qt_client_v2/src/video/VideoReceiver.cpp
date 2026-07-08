#include "VideoReceiver.h"

#include <QMetaObject>
#include <QTimer>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

static GstFlowReturn on_new_sample(GstAppSink* sink, gpointer user_data)
{
    auto* receiver = static_cast<VideoReceiver*>(user_data);
    if (!receiver) {
        return GST_FLOW_ERROR;
    }

    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) {
        return GST_FLOW_ERROR;
    }

    GstCaps* caps = gst_sample_get_caps(sample);
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (!caps || !buffer) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstVideoInfo info;
    if (!gst_video_info_from_caps(&info, caps)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    const int width = GST_VIDEO_INFO_WIDTH(&info);
    const int height = GST_VIDEO_INFO_HEIGHT(&info);
    const int stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);

    if (width <= 0 || height <= 0 || stride <= 0) {
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    QImage image(reinterpret_cast<const uchar*>(map.data),
                 width,
                 height,
                 stride,
                 QImage::Format_RGB888);

    /*
     * 必须 copy()，因为 gst_buffer_unmap 后 map.data 就失效。
     * 必须 QueuedConnection，因为该回调在 GStreamer 线程中，不应直接刷新 Qt UI。
     */
    QImage frameCopy = image.copy();

    QMetaObject::invokeMethod(receiver,
                              "deliverFrame",
                              Qt::QueuedConnection,
                              Q_ARG(QImage, frameCopy));

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

VideoReceiver::VideoReceiver(QObject* parent)
    : QObject(parent)
    , m_busTimer(new QTimer(this))
{
    static bool gstInitialized = false;
    if (!gstInitialized) {
        gst_init(nullptr, nullptr);
        gstInitialized = true;
    }

    connect(m_busTimer, &QTimer::timeout, this, &VideoReceiver::pollBus);
}

VideoReceiver::~VideoReceiver()
{
    stop();
}

bool VideoReceiver::isRunning() const
{
    return m_running;
}

QString VideoReceiver::pipelineDescription(int port) const
{
    /*
     * 对应终端命令：
     * gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, encoding-name=JPEG, payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink
     *
     * 页面内显示版本：
     * autovideosink 改为 appsink，Qt 从 appsink 中取 RGB 帧显示。
     *
     * 这里不要强制 width/height，先让它自动匹配 RK3588 发来的真实视频流分辨率。
     */
    return QString("udpsrc port=%1 caps=application/x-rtp,media=video,encoding-name=JPEG,payload=26,clock-rate=90000 "
                   "! rtpjpegdepay "
                   "! jpegdec "
                   "! videoconvert "
                   "! video/x-raw,format=RGB "
                   "! appsink name=qt_sink emit-signals=false sync=false max-buffers=2 drop=true")
        .arg(port);
}

bool VideoReceiver::startRtpJpeg(int port)
{
    if (m_running) {
        emit logMessage("视频接收已经在运行");
        return true;
    }

    GError* error = nullptr;
    const QByteArray pipelineBytes = pipelineDescription(port).toUtf8();

    m_pipeline = gst_parse_launch(pipelineBytes.constData(), &error);
    if (!m_pipeline || error) {
        const QString msg = QString("创建 GStreamer 接收管道失败：%1")
            .arg(error ? QString::fromUtf8(error->message) : QString("unknown"));
        if (error) {
            g_error_free(error);
        }
        cleanupPipeline(false);
        emit errorMessage(msg);
        return false;
    }

    m_appsink = gst_bin_get_by_name(GST_BIN(m_pipeline), "qt_sink");
    if (!m_appsink) {
        cleanupPipeline(false);
        emit errorMessage("无法获取 appsink：qt_sink");
        return false;
    }

    /*
     * 关键修复：
     * GstAppSinkCallbacks 必须使用 {} 初始化。
     * 否则某些 GStreamer 版本中结构体新增字段会包含随机函数指针，收到事件时可能直接崩溃。
     */
    GstAppSinkCallbacks callbacks = {};
    callbacks.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(m_appsink), &callbacks, this, nullptr);

    const GstStateChangeReturn ret = gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        cleanupPipeline(false);
        emit errorMessage("GStreamer 接收管道启动失败");
        return false;
    }

    m_running = true;
    m_busTimer->start(200);

    emit logMessage(QString("页面内视频接收已启动，UDP/RTP 端口：%1").arg(port));
    emit runningChanged(true);
    emit started();

    return true;
}

void VideoReceiver::stop()
{
    if (!m_running && !m_pipeline) {
        return;
    }

    emit logMessage("正在停止页面内视频接收 ...");
    cleanupPipeline(true);
}

void VideoReceiver::cleanupPipeline(bool notifyStopped)
{
    if (m_busTimer) {
        m_busTimer->stop();
    }

    if (m_appsink) {
        /*
         * 先清空回调，避免停止/析构过程中 GStreamer 线程继续回调已经释放的对象。
         */
        GstAppSinkCallbacks callbacks = {};
        gst_app_sink_set_callbacks(GST_APP_SINK(m_appsink), &callbacks, nullptr, nullptr);
    }

    if (m_pipeline) {
        gst_element_set_state(m_pipeline, GST_STATE_NULL);
    }

    if (m_appsink) {
        gst_object_unref(GST_OBJECT(m_appsink));
        m_appsink = nullptr;
    }

    if (m_pipeline) {
        gst_object_unref(GST_OBJECT(m_pipeline));
        m_pipeline = nullptr;
    }

    const bool wasRunning = m_running;
    m_running = false;

    if (wasRunning || notifyStopped) {
        emit runningChanged(false);
        emit stopped();
    }
}

void VideoReceiver::pollBus()
{
    if (!m_pipeline) {
        return;
    }

    GstBus* bus = gst_element_get_bus(m_pipeline);
    if (!bus) {
        return;
    }

    GstMessage* msg = nullptr;
    while ((msg = gst_bus_pop(bus)) != nullptr) {
        switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError* error = nullptr;
            gchar* debug = nullptr;
            gst_message_parse_error(msg, &error, &debug);

            const QString text = QString("GStreamer 错误：%1%2")
                .arg(error ? QString::fromUtf8(error->message) : QString("unknown"))
                .arg(debug ? QString("；debug=%1").arg(QString::fromUtf8(debug)) : QString());

            if (error) {
                g_error_free(error);
            }
            if (debug) {
                g_free(debug);
            }

            emit errorMessage(text);
            gst_message_unref(msg);
            gst_object_unref(bus);
            cleanupPipeline(true);
            return;
        }
        case GST_MESSAGE_EOS:
            emit logMessage("GStreamer 视频流结束");
            gst_message_unref(msg);
            gst_object_unref(bus);
            cleanupPipeline(true);
            return;
        default:
            break;
        }

        gst_message_unref(msg);
    }

    gst_object_unref(bus);
}

void VideoReceiver::deliverFrame(const QImage& frame)
{
    emit frameReady(frame);
}
