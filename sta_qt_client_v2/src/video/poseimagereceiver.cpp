#include "poseimagereceiver.h"

#include <QMetaObject>
#include <QTimer>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

static GstFlowReturn on_pose_new_sample(GstAppSink* sink, gpointer user_data)
{
    auto* receiver = static_cast<PoseImageReceiver*>(user_data);
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

    if (width > 0 && height > 0 && stride > 0) {
        QImage image(reinterpret_cast<const uchar*>(map.data),
                     width,
                     height,
                     stride,
                     QImage::Format_RGB888);

        QImage frameCopy = image.copy();
        QMetaObject::invokeMethod(receiver,
                                  "deliverFrame",
                                  Qt::QueuedConnection,
                                  Q_ARG(QImage, frameCopy));
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

PoseImageReceiver::PoseImageReceiver(QObject* parent)
    : QObject(parent)
    , m_busTimer(new QTimer(this))
{
    static bool gstInitialized = false;
    if (!gstInitialized) {
        gst_init(nullptr, nullptr);
        gstInitialized = true;
    }

    connect(m_busTimer, &QTimer::timeout, this, &PoseImageReceiver::pollBus);
}

PoseImageReceiver::~PoseImageReceiver()
{
    stop();
}

bool PoseImageReceiver::isRunning() const
{
    return m_running;
}

QString PoseImageReceiver::pipelineDescription(int port) const
{
    return QString("udpsrc port=%1 caps=application/x-rtp,media=video,encoding-name=JPEG,payload=26,clock-rate=90000 "
                   "! rtpjpegdepay "
                   "! jpegdec "
                   "! videoconvert "
                   "! video/x-raw,format=RGB "
                   "! appsink name=pose_sink emit-signals=false sync=false max-buffers=2 drop=true")
        .arg(port);
}

bool PoseImageReceiver::startRtpJpeg(int port)
{
    if (m_running) {
        emit logMessage("位姿结果图像接收已经在运行");
        return true;
    }

    GError* error = nullptr;
    const QByteArray desc = pipelineDescription(port).toUtf8();

    m_pipeline = gst_parse_launch(desc.constData(), &error);
    if (!m_pipeline || error) {
        const QString msg = QString("创建位姿图像接收管道失败：%1")
            .arg(error ? QString::fromUtf8(error->message) : QString("unknown"));
        if (error) {
            g_error_free(error);
        }
        cleanupPipeline(false);
        emit errorMessage(msg);
        return false;
    }

    m_appsink = gst_bin_get_by_name(GST_BIN(m_pipeline), "pose_sink");
    if (!m_appsink) {
        cleanupPipeline(false);
        emit errorMessage("无法获取位姿图像 appsink：pose_sink");
        return false;
    }

    GstAppSinkCallbacks callbacks = {};
    callbacks.new_sample = on_pose_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(m_appsink), &callbacks, this, nullptr);

    const GstStateChangeReturn ret = gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        cleanupPipeline(false);
        emit errorMessage("位姿图像接收管道启动失败");
        return false;
    }

    m_running = true;
    m_busTimer->start(200);

    emit logMessage(QString("位姿结果图像接收已启动，UDP/RTP 端口：%1").arg(port));
    emit started();
    return true;
}

void PoseImageReceiver::stop()
{
    if (!m_running && !m_pipeline) {
        return;
    }

    emit logMessage("正在停止位姿结果图像接收 ...");
    cleanupPipeline(true);
}

void PoseImageReceiver::cleanupPipeline(bool notifyStopped)
{
    if (m_busTimer) {
        m_busTimer->stop();
    }

    if (m_appsink) {
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
        emit stopped();
    }
}

void PoseImageReceiver::pollBus()
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

            const QString text = QString("位姿图像 GStreamer 错误：%1%2")
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
            emit logMessage("位姿图像流结束");
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

void PoseImageReceiver::deliverFrame(const QImage& frame)
{
    emit frameReady(frame);
}
