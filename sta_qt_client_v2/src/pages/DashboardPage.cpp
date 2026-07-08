#include "DashboardPage.h"

#include "app/AppState.h"
#include "common/Protocol.h"
#include "network/TcpClientManager.h"
#include "pages/DebugLogWindow.h"
#include "video/VideoReceiver.h"
#include "video/poseimagereceiver.h"

#include <QDateTime>
#include <QPlainTextEdit>
#include <QEvent>
#include <QDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QTabWidget>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTextEdit>
#include <QVBoxLayout>

DashboardPage::DashboardPage(TcpClientManager* tcpClient,
                             VideoReceiver* videoReceiver,
                             QWidget* parent)
    : QWidget(parent)
    , m_tcpClient(tcpClient)
    , m_videoReceiver(videoReceiver)
    , m_debugLogWindow(new DebugLogWindow(this))
    , m_poseImageReceiver(new PoseImageReceiver(this))
    , m_mode(SystemMode::Disconnected)
{
    buildUi();
    updateButtonState();

    connect(m_startVideoButton, &QPushButton::clicked, this, &DashboardPage::startVideoOnly);
    connect(m_stopVideoButton, &QPushButton::clicked, this, &DashboardPage::stopVideoOnly);
    connect(m_startDetectButton, &QPushButton::clicked, this, &DashboardPage::startDetectTrack);
    connect(m_stopDetectButton, &QPushButton::clicked, this, &DashboardPage::stopDetectTrack);
    connect(m_getSensorButton, &QPushButton::clicked, this, &DashboardPage::getSensorOnce);
    connect(m_subscribeSensorButton, &QPushButton::clicked, this, &DashboardPage::startSensorSubscribe);
    connect(m_unsubscribeSensorButton, &QPushButton::clicked, this, &DashboardPage::stopSensorSubscribe);
    connect(m_servoLeftButton, &QPushButton::pressed, this, &DashboardPage::servoLeftPressed);
    connect(m_servoLeftButton, &QPushButton::released, this, &DashboardPage::servoLeftReleased);
    connect(m_servoRightButton, &QPushButton::pressed, this, &DashboardPage::servoRightPressed);
    connect(m_servoRightButton, &QPushButton::released, this, &DashboardPage::servoRightReleased);
    connect(m_servoUpButton, &QPushButton::clicked, this, &DashboardPage::servoUpStep);
    connect(m_servoDownButton, &QPushButton::clicked, this, &DashboardPage::servoDownStep);
    connect(m_servoCenterButton, &QPushButton::clicked, this, &DashboardPage::servoCenter);
    connect(m_stopAllButton, &QPushButton::clicked, this, &DashboardPage::stopAll);
    connect(m_disconnectButton, &QPushButton::clicked, this, &DashboardPage::disconnectRequested);
    connect(m_logoutButton, &QPushButton::clicked, this, &DashboardPage::logoutRequested);
    connect(m_manualSendButton, &QPushButton::clicked, this, &DashboardPage::sendManualCommand);
    connect(m_manualCommandEdit, &QLineEdit::returnPressed, this, &DashboardPage::sendManualCommand);
    connect(m_openDebugWindowButton, &QPushButton::clicked, this, &DashboardPage::openDebugLogWindow);
    connect(m_clearLogsButton, &QPushButton::clicked, this, &DashboardPage::clearLogs);

    connect(m_poseMeasureButton, &QPushButton::clicked, this, &DashboardPage::measurePoseOnce);

    if (m_poseImageReceiver) {
        connect(m_poseImageReceiver, &PoseImageReceiver::frameReady,
                this, &DashboardPage::onPoseImageFrameReady);
        connect(m_poseImageReceiver, &PoseImageReceiver::logMessage,
                this, &DashboardPage::appendLog);
        connect(m_poseImageReceiver, &PoseImageReceiver::errorMessage,
                this, &DashboardPage::appendError);
    }

    if (m_videoReceiver) {
        connect(m_videoReceiver, &VideoReceiver::frameReady,
                this, &DashboardPage::onVideoFrameReady);
    }
}

void DashboardPage::setVideoPort(int port)
{
    m_videoPort = port;
}

void DashboardPage::setConnectionInfo(const QString& host, quint16 port)
{
    m_connectionLabel->setText(QString("服务器：%1:%2    视频端口：%3")
                               .arg(host).arg(port).arg(m_videoPort));
}

void DashboardPage::appendText(QTextEdit* edit, const QString& text, const QColor& color)
{
    if (!edit) {
        return;
    }

    QTextCursor cursor = edit->textCursor();
    cursor.movePosition(QTextCursor::End);

    QTextCharFormat fmt;
    fmt.setForeground(color);

    cursor.insertText(text, fmt);
    cursor.insertBlock();

    edit->setTextCursor(cursor);
    edit->ensureCursorVisible();
}

void DashboardPage::appendLog(const QString& message)
{
    const QString time = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    appendText(m_logEdit, QString("[%1] %2").arg(time, message), Qt::black);
}

void DashboardPage::appendError(const QString& message)
{
    const QString time = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    appendText(m_logEdit, QString("[%1] ERROR: %2").arg(time, message), QColor(176, 0, 32));
}

void DashboardPage::appendBoardDebug(const QString& source,
                                     const QString& stream,
                                     const QString& message)
{
    const QString time = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    const QString line = QString("[%1] [%2/%3] %4")
        .arg(time, source, stream, message);

    const bool isError = stream.compare("stderr", Qt::CaseInsensitive) == 0;
    appendText(m_boardLogEdit, line, isError ? QColor(176, 0, 32) : Qt::darkBlue);

    if (m_debugLogWindow) {
        m_debugLogWindow->appendLine(QString("[%1/%2] %3").arg(source, stream, message), isError);
    }
}

void DashboardPage::onServerMessage(const QJsonObject& object)
{
    const QString type = object.value("type").toString();
    const QString cmd = object.value("cmd").toString();
    const QString mode = object.value("mode").toString();
    const QString result = object.value("result").toString();

    if (type == "LOG") {
        appendBoardDebug(object.value("source").toString("rk3588"),
                         object.value("stream").toString("stdout"),
                         object.value("message").toString());
        return;
    }

    if (type == "DETECT_STATUS") {
        handleDetectStatusMessage(object);
        return;
    }

    if (type == "SERVO_STATUS" || (type == "ACK" && cmd == "SERVO_MANUAL")) {
        handleServoStatusMessage(object);
        return;
    }

    if (type == "POSE_RESULT") {
        handlePoseResultMessage(object);
        return;
    }

    if (type == "POSE_IMAGE_STREAM") {
        handlePoseImageStreamMessage(object);
        return;
    }

    if (type == "STATUS") {
        handleStatusMessage(object);
        return;
    }

    if (type == "SENSOR") {
        handleSensorMessage(object);
        return;
    }

    if (type == "FAULT") {
        handleFaultMessage(object);
        return;
    }

    if (type == "ACK" && result == "ok" && cmd == "START_MODE" && mode == "detect_track") {
        updateMode(SystemMode::DetectTrack);
        appendLog(QString("RK3588 已启动检测跟踪程序，pid=%1").arg(object.value("pid").toInt()));
        return;
    }

    if (type == "ACK" && result == "ok" && cmd == "STOP_MODE" && mode == "detect_track") {
        updateMode(SystemMode::Idle);
        if (m_videoReceiver && m_videoReceiver->isRunning()) {
            m_videoReceiver->stop();
        }
        appendLog("RK3588 已停止检测跟踪程序");
        return;
    }

    if (type == "ACK" && result == "ok" && cmd == "START_MODE" && mode == "video_preview") {
        updateMode(SystemMode::VideoOnly);
        appendLog(QString("RK3588 已启动原始视频预览，pid=%1").arg(object.value("pid").toInt()));
        return;
    }

    if (type == "ACK" && result == "ok" && cmd == "STOP_MODE" && mode == "video_preview") {
        updateMode(SystemMode::Idle);
        if (m_videoReceiver && m_videoReceiver->isRunning()) {
            m_videoReceiver->stop();
        }
        appendLog("RK3588 已停止原始视频预览");
        return;
    }

    if (type == "ACK" && result == "ok" && cmd == "START_MODE" && mode == "patrol") {
        updateMode(SystemMode::Patrol);
        appendLog(QString("RK3588 已启动巡检模式，pid=%1").arg(object.value("pid").toInt()));
        return;
    }

    if (type == "ACK" && result == "ok" && cmd == "STOP_MODE" && mode == "patrol") {
        updateMode(SystemMode::Idle);
        if (m_videoReceiver && m_videoReceiver->isRunning()) {
            m_videoReceiver->stop();
        }
        appendLog("RK3588 已停止巡检模式");
        return;
    }

    if (type == "ACK" && result == "ok" && cmd == "STOP_ALL") {
        updateMode(SystemMode::Idle);
        if (m_videoReceiver && m_videoReceiver->isRunning()) {
            m_videoReceiver->stop();
        }
        appendLog("RK3588 已停止全部任务");
        return;
    }


    if (type == "ACK" && result == "started" && cmd == "MEASURE_POSE") {
        if (m_poseStatusLabel) {
            m_poseStatusLabel->setText("状态：测量中，等待板端返回结果...");
        }
        appendLog("RK3588 已开始单次位姿测量");
        return;
    }

    if (type == "ERROR") {
        appendError(QString("RK3588 错误: %1")
                    .arg(QString::fromUtf8(QJsonDocument(object).toJson(QJsonDocument::Compact))));

        if (cmd == "START_MODE" &&
            (mode == "detect_track" || mode == "video_preview") &&
            m_videoReceiver && m_videoReceiver->isRunning()) {
            m_videoReceiver->stop();
        }
        if (cmd == "START_MODE" && (mode == "detect_track" || mode == "video_preview")) {
            updateMode(SystemMode::Idle);
        }
        return;
    }

    if (type == "HEARTBEAT") {
        if (object.contains("mode")) {
            handleStatusMessage(object);
        }
        return;
    }
}

void DashboardPage::onTcpConnected()
{
    updateMode(SystemMode::Idle);
    m_connectionLabel->setText(QString("服务器：%1:%2    视频端口：%3")
                               .arg(m_tcpClient->host()).arg(m_tcpClient->port()).arg(m_videoPort));
    appendLog("TCP 已连接");
}

void DashboardPage::onTcpDisconnected()
{
    updateMode(SystemMode::Disconnected);
    m_connectionLabel->setText("服务器：未连接");
    if (m_videoReceiver && m_videoReceiver->isRunning()) {
        m_videoReceiver->stop();
    }
    if (m_poseImageReceiver && m_poseImageReceiver->isRunning()) {
        m_poseImageReceiver->stop();
    }
    appendLog("TCP 已断开");
}

void DashboardPage::onVideoStarted()
{
    resetVideoReceiveFps();
    m_videoLabel->setText(QString("视频接收：运行中，UDP/RTP 端口 %1").arg(m_videoPort));
    m_videoView->setText("正在接收视频流 ...");
    updateButtonState();
}

void DashboardPage::onVideoStopped()
{
    resetVideoReceiveFps();
    m_fpsLabel->setText("视频接收帧率：-- FPS");
    m_videoLabel->setText("视频接收：未运行");
    m_lastFrame = QImage();
    m_videoView->clear();
    m_videoView->setText("视频未启动");
    updateButtonState();
}

void DashboardPage::onVideoFrameReady(const QImage& frame)
{
    m_lastFrame = frame;
    updateVideoReceiveFps();
    refreshVideoPixmap();
}

void DashboardPage::refreshVideoPixmap()
{
    if (!m_videoView || m_lastFrame.isNull()) {
        return;
    }

    const QPixmap pix = QPixmap::fromImage(m_lastFrame)
        .scaled(m_videoView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_videoView->setPixmap(pix);
}

void DashboardPage::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    refreshVideoPixmap();
    refreshPosePixmap();
}

void DashboardPage::startVideoOnly()
{
    if (!m_tcpClient || !m_tcpClient->isConnected()) {
        appendError("TCP 未连接，无法启动视频预览");
        return;
    }

    if (m_mode != SystemMode::Idle) {
        appendError("当前不是空闲状态，无法启动原始视频预览");
        return;
    }

    /*
     * 先启动 Qt 端 UDP/RTP 5000 接收，再通知 RK3588 启动原始预览推流，
     * 避免板端已经开始发送而 Qt 还没有进入接收状态。
     */
    if (m_videoReceiver && !m_videoReceiver->isRunning()) {
        if (!m_videoReceiver->startRtpJpeg(m_videoPort)) {
            appendError("视频接收启动失败，未发送视频预览启动命令");
            return;
        }
    }

    sendCommand(Protocol::startVideoPreview(m_videoPort));
    updateMode(SystemMode::VideoOnly);
    appendLog("已发送启动原始视频预览命令，等待 RK3588 返回 ACK ...");
}

void DashboardPage::stopVideoOnly()
{
    if (!m_tcpClient || !m_tcpClient->isConnected()) {
        if (m_videoReceiver) {
            m_videoReceiver->stop();
        }
        updateMode(SystemMode::Idle);
        return;
    }

    sendCommand(Protocol::stopVideoPreview());
    appendLog("已发送停止原始视频预览命令，等待 RK3588 返回 ACK ...");
}


void DashboardPage::startDetectTrack()
{
    if (!m_tcpClient || !m_tcpClient->isConnected()) {
        appendError("TCP 未连接，无法启动检测跟踪");
        return;
    }

    if (m_mode != SystemMode::Idle) {
        appendError("当前不是空闲状态，无法启动检测跟踪。请先停止视频预览或其他任务。");
        return;
    }

    if (!m_videoReceiver->isRunning()) {
        if (!m_videoReceiver->startRtpJpeg(m_videoPort)) {
            appendError("视频接收启动失败，未发送检测跟踪启动命令");
            return;
        }
    }

    QJsonObject cmd;
    cmd["cmd"] = "START_MODE";
    cmd["mode"] = "detect_track";
    cmd["target"] = "satellite_fault";
    cmd["video_port"] = m_videoPort;

    sendCommand(cmd);
    appendLog("已发送启动检测跟踪命令，等待 RK3588 返回 ACK ...");
}

void DashboardPage::stopDetectTrack()
{
    QJsonObject cmd;
    cmd["cmd"] = "STOP_MODE";
    cmd["mode"] = "detect_track";

    sendCommand(cmd);
    appendLog("已发送停止检测跟踪命令，等待 RK3588 返回 ACK ...");
}

void DashboardPage::getSensorOnce()
{
    sendCommand(Protocol::getSensor("all"));
}

void DashboardPage::startSensorSubscribe()
{
    sendCommand(Protocol::subscribeSensor(1000));
}

void DashboardPage::stopSensorSubscribe()
{
    sendCommand(Protocol::unsubscribeSensor());
}

void DashboardPage::sendServoManualAction(const QString& action)
{
    if (!m_tcpClient || !m_tcpClient->isConnected()) {
        appendError("TCP 未连接，无法手动控制舵机");
        return;
    }

    if (m_mode == SystemMode::DetectTrack) {
        appendError("检测跟踪运行中，手动舵机控制已禁用");
        return;
    }

    sendCommand(Protocol::servoManual(action));
}

void DashboardPage::servoLeftPressed()
{
    sendServoManualAction("left_press");
}

void DashboardPage::servoLeftReleased()
{
    sendServoManualAction("left_release");
}

void DashboardPage::servoRightPressed()
{
    sendServoManualAction("right_press");
}

void DashboardPage::servoRightReleased()
{
    sendServoManualAction("right_release");
}

void DashboardPage::servoUpStep()
{
    sendServoManualAction("up_step");
}

void DashboardPage::servoDownStep()
{
    sendServoManualAction("down_step");
}

void DashboardPage::servoCenter()
{
    sendServoManualAction("center");
}

void DashboardPage::stopAll()
{
    sendCommand(Protocol::simpleCommand("STOP_ALL"));
    if (m_videoReceiver && m_videoReceiver->isRunning()) {
        m_videoReceiver->stop();
    }
    if (m_poseImageReceiver && m_poseImageReceiver->isRunning()) {
        m_poseImageReceiver->stop();
    }
    updateMode(SystemMode::Idle);
}

void DashboardPage::sendManualCommand()
{
    const QString text = m_manualCommandEdit->text().trimmed();
    if (text.isEmpty()) {
        return;
    }

    if (text.startsWith('{')) {
        QJsonParseError error;
        const QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8(), &error);
        if (error.error != QJsonParseError::NoError || !doc.isObject()) {
            QMessageBox::warning(this, "JSON 格式错误", error.errorString());
            return;
        }
        sendCommand(doc.object());
    } else {
        sendCommand(Protocol::simpleCommand(text.toUpper()));
    }

    m_manualCommandEdit->clear();
}

void DashboardPage::openDebugLogWindow()
{
    if (!m_debugLogWindow) {
        m_debugLogWindow = new DebugLogWindow(this);
    }
    m_debugLogWindow->show();
    m_debugLogWindow->raise();
    m_debugLogWindow->activateWindow();
}

void DashboardPage::clearLogs()
{
    m_logEdit->clear();
    m_boardLogEdit->clear();
    if (m_debugLogWindow) {
        m_debugLogWindow->clearLog();
    }
}


void DashboardPage::measurePoseOnce()
{
    if (!m_tcpClient || !m_tcpClient->isConnected()) {
        appendError("TCP 未连接，无法进行位姿测量");
        return;
    }

    if (m_mode != SystemMode::DetectTrack) {
        appendError("请先启动检测跟踪程序，再进行单次位姿测量");
        return;
    }

    if (m_poseImageReceiver && !m_poseImageReceiver->isRunning()) {
        if (!m_poseImageReceiver->startRtpJpeg(m_poseImagePort)) {
            appendError("位姿结果图像接收启动失败");
            return;
        }
    }

    if (m_poseStatusLabel) {
        m_poseStatusLabel->setText("状态：测量中...");
    }
    if (m_poseTextEdit) {
        m_poseTextEdit->setPlainText("正在请求 RK3588 保存原始双目图像并执行位姿测量...");
    }
    if (m_poseMeasureButton) {
        m_poseMeasureButton->setEnabled(false);
    }

    sendCommand(Protocol::measurePose(m_poseImagePort));
    appendLog(QString("已发送单次位姿测量命令，位姿图像端口：%1").arg(m_poseImagePort));
}

void DashboardPage::handlePoseResultMessage(const QJsonObject& object)
{
    const bool valid = object.value("valid").toBool(false);
    const QString text = object.value("text").toString();
    const QString error = object.value("error").toString();

    if (m_poseStatusLabel) {
        m_poseStatusLabel->setText(valid ? "状态：测量成功" : "状态：测量失败");
    }

    if (m_poseTextEdit) {
        QString content;
        if (!error.isEmpty()) {
            content += QString("Error: %1\n\n").arg(error);
        }
        content += text.isEmpty() ? QString("未收到 result.txt 内容") : text;
        m_poseTextEdit->setPlainText(content);
    }

    if (valid) {
        appendLog("位姿测量完成：成功");
    } else {
        appendError("位姿测量完成：失败，详情见位姿信息框");
    }

    if (m_poseMeasureButton) {
        m_poseMeasureButton->setEnabled(m_tcpClient && m_tcpClient->isConnected() && m_mode == SystemMode::DetectTrack);
    }
}

void DashboardPage::handlePoseImageStreamMessage(const QJsonObject& object)
{
    const int port = object.value("port").toInt(m_poseImagePort);
    const int durationMs = object.value("duration_ms").toInt(3000);
    appendLog(QString("RK3588 正在发送位姿结果图像：UDP/RTP 端口 %1，持续 %2 ms")
              .arg(port)
              .arg(durationMs));
}

void DashboardPage::onPoseImageFrameReady(const QImage& frame)
{
    m_lastPoseImage = frame;
    refreshPosePixmap();

    if (m_poseStatusLabel && m_poseStatusLabel->text().contains("测量中")) {
        m_poseStatusLabel->setText("状态：已收到结果图像，等待文本结果...");
    }
}

void DashboardPage::refreshPosePixmap()
{
    if (!m_poseImageView || m_lastPoseImage.isNull()) {
        return;
    }

    const QPixmap pix = QPixmap::fromImage(m_lastPoseImage)
        .scaled(m_poseImageView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    m_poseImageView->setPixmap(pix);
}

void DashboardPage::showPoseImageDialog()
{
    if (m_lastPoseImage.isNull()) {
        return;
    }

    auto* dialog = new QDialog(this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle("位姿测量结果图像");

    auto* layout = new QVBoxLayout(dialog);
    auto* label = new QLabel(dialog);
    label->setAlignment(Qt::AlignCenter);
    label->setPixmap(QPixmap::fromImage(m_lastPoseImage)
                     .scaled(1100, 520, Qt::KeepAspectRatio, Qt::SmoothTransformation));

    layout->addWidget(label);
    dialog->resize(1150, 580);
    dialog->show();
}

bool DashboardPage::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == m_poseImageView && event->type() == QEvent::MouseButtonDblClick) {
        showPoseImageDialog();
        return true;
    }

    return QWidget::eventFilter(watched, event);
}


void DashboardPage::buildUi()
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(14, 14, 14, 14);
    root->setSpacing(10);

    auto* titleLine = new QHBoxLayout;
    auto* title = new QLabel("智能在轨卫星跟踪与位姿测量系统 - 客户端", this);
    QFont titleFont = title->font();
    titleFont.setPointSize(15);
    titleFont.setBold(true);
    title->setFont(titleFont);

    m_openDebugWindowButton = new QPushButton("板端调试窗口", this);
    m_clearLogsButton = new QPushButton("清空日志", this);
    m_disconnectButton = new QPushButton("断开连接", this);
    m_logoutButton = new QPushButton("退出登录", this);
    titleLine->addWidget(title);
    titleLine->addStretch();
    titleLine->addWidget(m_openDebugWindowButton);
    titleLine->addWidget(m_clearLogsButton);
    titleLine->addWidget(m_disconnectButton);
    titleLine->addWidget(m_logoutButton);

    auto* statusGroup = new QGroupBox("系统状态", this);
    auto* statusLayout = new QGridLayout(statusGroup);
    m_connectionLabel = new QLabel("服务器：未连接", statusGroup);
    m_modeLabel = new QLabel("当前模式：未连接", statusGroup);
    m_videoLabel = new QLabel("视频接收：未运行", statusGroup);
    m_lightLabel = new QLabel("光照强度：-- lux", statusGroup);
    m_temperatureLabel = new QLabel("平台温度：-- ℃", statusGroup);
    m_fpsLabel = new QLabel("视频接收帧率：-- FPS", statusGroup);
    m_servoLabel = new QLabel("舵机占空比：X=-- ns，Y=-- ns", statusGroup);
    m_alarmLabel = new QLabel("跟踪状态：--", statusGroup);
    statusLayout->addWidget(m_connectionLabel, 0, 0, 1, 2);
    statusLayout->addWidget(m_modeLabel, 1, 0);
    statusLayout->addWidget(m_videoLabel, 1, 1);
    statusLayout->addWidget(m_lightLabel, 2, 0);
    statusLayout->addWidget(m_temperatureLabel, 2, 1);
    statusLayout->addWidget(m_fpsLabel, 3, 0);
    statusLayout->addWidget(m_servoLabel, 3, 1);
    statusLayout->addWidget(m_alarmLabel, 4, 0, 1, 2);

    auto* controlGroup = new QGroupBox("功能控制", this);
    auto* controlLayout = new QGridLayout(controlGroup);
    controlLayout->setColumnStretch(0, 1);
    controlLayout->setColumnStretch(1, 1);
    controlLayout->setColumnStretch(2, 1);

    m_startVideoButton = new QPushButton("开始视频预览", controlGroup);
    m_stopVideoButton = new QPushButton("停止视频预览", controlGroup);
    m_startDetectButton = new QPushButton("开始检测跟踪", controlGroup);
    m_stopDetectButton = new QPushButton("停止检测跟踪", controlGroup);
    m_getSensorButton = new QPushButton("获取一次传感器", controlGroup);
    m_subscribeSensorButton = new QPushButton("开始传感器刷新", controlGroup);
    m_unsubscribeSensorButton = new QPushButton("停止传感器刷新", controlGroup);
    m_stopAllButton = new QPushButton("一键停止全部", controlGroup);

    controlLayout->addWidget(m_startVideoButton, 0, 0);
    controlLayout->addWidget(m_stopVideoButton, 0, 1);
    controlLayout->addWidget(m_startDetectButton, 1, 0);
    controlLayout->addWidget(m_stopDetectButton, 1, 1);
    controlLayout->addWidget(m_getSensorButton, 2, 0);
    controlLayout->addWidget(m_subscribeSensorButton, 2, 1);
    controlLayout->addWidget(m_unsubscribeSensorButton, 2, 2);

    auto* servoPanel = new QWidget(controlGroup);
    auto* servoLayout = new QGridLayout(servoPanel);
    servoLayout->setContentsMargins(0, 0, 0, 0);
    servoLayout->setHorizontalSpacing(6);
    servoLayout->setVerticalSpacing(6);
    servoLayout->setColumnStretch(0, 1);
    servoLayout->setColumnStretch(1, 1);
    servoLayout->setColumnStretch(2, 1);

    m_servoUpButton = new QPushButton("向上", servoPanel);
    m_servoLeftButton = new QPushButton("向左", servoPanel);
    m_servoCenterButton = new QPushButton("归中", servoPanel);
    m_servoRightButton = new QPushButton("向右", servoPanel);
    m_servoDownButton = new QPushButton("向下", servoPanel);

    m_servoLeftButton->setToolTip("按住向左转动，松开后 X 轴停止");
    m_servoRightButton->setToolTip("按住向右转动，松开后 X 轴停止");
    m_servoUpButton->setToolTip("Y 轴向上步进一个角度");
    m_servoDownButton->setToolTip("Y 轴向下步进一个角度");
    m_servoCenterButton->setToolTip("X 轴停止，Y 轴回中；X 轴不做绝对归中");

    m_servoUpButton->setAutoRepeat(true);
    m_servoUpButton->setAutoRepeatDelay(350);
    m_servoUpButton->setAutoRepeatInterval(180);
    m_servoDownButton->setAutoRepeat(true);
    m_servoDownButton->setAutoRepeatDelay(350);
    m_servoDownButton->setAutoRepeatInterval(180);

    servoLayout->addWidget(m_servoUpButton, 0, 1);
    servoLayout->addWidget(m_servoLeftButton, 1, 0);
    servoLayout->addWidget(m_servoCenterButton, 1, 1);
    servoLayout->addWidget(m_servoRightButton, 1, 2);
    servoLayout->addWidget(m_servoDownButton, 2, 1);

    auto* servoGroup = new QGroupBox("手动舵机控制", controlGroup);
    auto* servoGroupLayout = new QVBoxLayout(servoGroup);
    servoGroupLayout->setContentsMargins(8, 8, 8, 8);
    servoGroupLayout->addWidget(servoPanel);

    controlLayout->addWidget(servoGroup, 3, 0, 1, 3);
    controlLayout->addWidget(m_stopAllButton, 4, 0, 1, 3);

    auto* manualGroup = new QGroupBox("手动命令调试", this);
    auto* manualLayout = new QHBoxLayout(manualGroup);
    m_manualCommandEdit = new QLineEdit(manualGroup);
    m_manualCommandEdit->setPlaceholderText("输入 JSON 命令，或输入简单命令，例如 GET_STATUS / STOP_ALL");
    m_manualSendButton = new QPushButton("发送", manualGroup);
    manualLayout->addWidget(m_manualCommandEdit);
    manualLayout->addWidget(m_manualSendButton);

    auto* poseGroup = new QGroupBox("位姿测量", this);
    poseGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* poseLayout = new QVBoxLayout(poseGroup);
    poseLayout->setContentsMargins(8, 8, 8, 8);
    poseLayout->setSpacing(6);

    auto* poseTopLine = new QHBoxLayout;
    m_poseMeasureButton = new QPushButton("单次位姿测量", poseGroup);
    m_poseStatusLabel = new QLabel("状态：未测量", poseGroup);
    poseTopLine->addWidget(m_poseMeasureButton);
    poseTopLine->addWidget(m_poseStatusLabel);
    poseTopLine->addStretch();

    m_poseImageView = new QLabel("暂无位姿结果图像\n双击可查看大图", poseGroup);
    m_poseImageView->setAlignment(Qt::AlignCenter);
    m_poseImageView->setMinimumSize(360, 155);
    m_poseImageView->setMaximumHeight(240);
    m_poseImageView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_poseImageView->setStyleSheet("QLabel { background-color: #111; color: #ddd; border: 1px solid #666; }");
    m_poseImageView->installEventFilter(this);

    m_poseTextEdit = new QTextEdit(poseGroup);
    m_poseTextEdit->setReadOnly(true);
    m_poseTextEdit->setMinimumHeight(130);
    m_poseTextEdit->setMaximumHeight(QWIDGETSIZE_MAX);
    m_poseTextEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_poseTextEdit->setLineWrapMode(QTextEdit::NoWrap);
    m_poseTextEdit->setPlaceholderText("位姿测量结果将在这里显示...");

    poseLayout->addLayout(poseTopLine, 0);
    poseLayout->addWidget(m_poseImageView, 1);
    poseLayout->addWidget(m_poseTextEdit, 2);

    auto* videoGroup = new QGroupBox("视频画面（页面内显示）", this);
    auto* videoLayout = new QVBoxLayout(videoGroup);
    m_videoView = new QLabel("视频未启动", videoGroup);
    m_videoView->setAlignment(Qt::AlignCenter);
    m_videoView->setMinimumSize(640, 480);
    m_videoView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_videoView->setStyleSheet("QLabel { background-color: #111; color: #ddd; border: 1px solid #444; }");
    videoLayout->addWidget(m_videoView);

    m_logTabs = new QTabWidget(this);
    m_logEdit = new QTextEdit(m_logTabs);
    m_boardLogEdit = new QTextEdit(m_logTabs);
    m_logEdit->setReadOnly(true);
    m_boardLogEdit->setReadOnly(true);
    m_logEdit->setLineWrapMode(QTextEdit::NoWrap);
    m_boardLogEdit->setLineWrapMode(QTextEdit::NoWrap);
    m_logTabs->addTab(m_logEdit, "系统日志");
    m_logTabs->addTab(m_boardLogEdit, "板端调试输出");
    m_logTabs->setMinimumHeight(230);
    m_logTabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    statusGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    controlGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    manualGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

    auto* leftPanel = new QWidget(this);
    leftPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    auto* leftColumn = new QVBoxLayout(leftPanel);
    leftColumn->setContentsMargins(0, 0, 0, 0);
    leftColumn->setSpacing(10);
    leftColumn->addWidget(statusGroup, 0);
    leftColumn->addWidget(controlGroup, 0);
    leftColumn->addWidget(manualGroup, 0);
    leftColumn->addWidget(poseGroup, 1);

    auto* rightPanel = new QWidget(this);
    rightPanel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* rightColumn = new QVBoxLayout(rightPanel);
    rightColumn->setContentsMargins(0, 0, 0, 0);
    rightColumn->setSpacing(10);
    rightColumn->addWidget(videoGroup, 5);
    rightColumn->addWidget(m_logTabs, 2);

    auto* mainLine = new QHBoxLayout;
    mainLine->setSpacing(10);
    mainLine->addWidget(leftPanel, 2);
    mainLine->addWidget(rightPanel, 5);

    root->addLayout(titleLine, 0);
    root->addLayout(mainLine, 1);
}

void DashboardPage::updateMode(SystemMode mode)
{
    m_mode = mode;
    m_modeLabel->setText(QString("当前模式：%1").arg(systemModeToString(mode)));
    updateButtonState();
}

void DashboardPage::updateButtonState()
{
    const bool connected = m_tcpClient && m_tcpClient->isConnected();
    const bool idle = (m_mode == SystemMode::Idle || m_mode == SystemMode::SensorMonitor);
    const bool previewMode = (m_mode == SystemMode::VideoOnly);
    const bool patrolMode = (m_mode == SystemMode::Patrol);
    const bool detectMode = (m_mode == SystemMode::DetectTrack);

    /*
     * 视频预览与检测跟踪互斥：
     * - 空闲：允许启动视频预览/检测跟踪；
     * - 视频预览：只允许停止视频预览，禁止检测跟踪；
     * - 检测跟踪：只允许停止检测跟踪，禁止视频预览。
     */
    m_startVideoButton->setEnabled(connected && idle);
    m_stopVideoButton->setEnabled(connected && previewMode);

    m_startDetectButton->setEnabled(connected && idle);
    m_stopDetectButton->setEnabled(connected && detectMode);

    m_getSensorButton->setEnabled(connected);
    m_subscribeSensorButton->setEnabled(connected);
    m_unsubscribeSensorButton->setEnabled(connected);

    const bool manualServoEnabled = connected && !detectMode;
    m_servoLeftButton->setEnabled(manualServoEnabled);
    m_servoRightButton->setEnabled(manualServoEnabled);
    m_servoUpButton->setEnabled(manualServoEnabled);
    m_servoDownButton->setEnabled(manualServoEnabled);
    m_servoCenterButton->setEnabled(manualServoEnabled);

    m_stopAllButton->setEnabled(connected);
    m_manualCommandEdit->setEnabled(connected);
    m_manualSendButton->setEnabled(connected);
    if (m_poseMeasureButton) {
        m_poseMeasureButton->setEnabled(connected && detectMode);
    }
    m_disconnectButton->setEnabled(connected);
}

void DashboardPage::sendCommand(const QJsonObject& object)
{
    if (!m_tcpClient || !m_tcpClient->sendJson(object)) {
        return;
    }
}

void DashboardPage::handleStatusMessage(const QJsonObject& object)
{
    const QString mode = object.value("mode").toString();
    if (!mode.isEmpty()) {
        const SystemMode newMode = systemModeFromServerString(mode);
        updateMode(newMode);

        if (newMode == SystemMode::Idle &&
            m_videoReceiver && m_videoReceiver->isRunning()) {
            m_videoReceiver->stop();
        }
    }

    /*
     * 注意：
     * 1. 视频接收帧率不再从 RK3588 STATUS 中读取，而是在 onVideoFrameReady() 按 Qt 实际收到的帧计算。
     * 2. 舵机占空比不再从普通 STATUS 中读取，而是从 DETECT_STATUS 中读取。
     */
}

void DashboardPage::handleDetectStatusMessage(const QJsonObject& object)
{
    const QString state = object.value("state").toString("--");

    const int dutyX = object.contains("duty_x")
        ? object.value("duty_x").toInt()
        : object.value("servo_x").toInt(0);

    const int dutyY = object.contains("duty_y")
        ? object.value("duty_y").toInt()
        : object.value("servo_y").toInt(0);

    if (m_alarmLabel) {
        m_alarmLabel->setText(QString("跟踪状态：%1").arg(state));
    }

    if (m_servoLabel) {
        m_servoLabel->setText(QString("舵机占空比：X=%1 ns，Y=%2 ns")
                              .arg(dutyX)
                              .arg(dutyY));
    }
}


void DashboardPage::handleServoStatusMessage(const QJsonObject& object)
{
    const QString action = object.value("action").toString();
    const int dutyX = object.value("duty_x").toInt(-1);
    const int dutyY = object.value("duty_y").toInt(-1);

    if (dutyX >= 0 && dutyY >= 0 && m_servoLabel) {
        m_servoLabel->setText(QString("舵机占空比：X=%1 ns，Y=%2 ns")
                              .arg(dutyX)
                              .arg(dutyY));
    }

    if (!action.isEmpty()) {
        appendLog(QString("手动舵机控制：%1，X=%2 ns，Y=%3 ns")
                  .arg(action)
                  .arg(dutyX)
                  .arg(dutyY));
    }
}

void DashboardPage::updateVideoReceiveFps()
{
    if (!m_videoFpsTimer.isValid()) {
        m_videoFpsTimer.start();
        m_videoFrameCount = 0;
    }

    ++m_videoFrameCount;

    const qint64 elapsedMs = m_videoFpsTimer.elapsed();
    if (elapsedMs >= 1000) {
        const double fps = m_videoFrameCount * 1000.0 / static_cast<double>(elapsedMs > 0 ? elapsedMs : 1);
        if (m_fpsLabel) {
            m_fpsLabel->setText(QString("视频接收帧率：%1 FPS").arg(fps, 0, 'f', 1));
        }

        m_videoFrameCount = 0;
        m_videoFpsTimer.restart();
    }
}

void DashboardPage::resetVideoReceiveFps()
{
    m_videoFrameCount = 0;
    m_videoFpsTimer.invalidate();
}

void DashboardPage::handleSensorMessage(const QJsonObject& object)
{
    if (object.contains("light_lux")) {
        m_lightLabel->setText(QString("光照强度：%1 lux").arg(object.value("light_lux").toDouble(), 0, 'f', 2));
    }
    if (object.contains("temperature")) {
        m_temperatureLabel->setText(QString("平台温度：%1 ℃").arg(object.value("temperature").toDouble(), 0, 'f', 1));
    }
}

void DashboardPage::handleFaultMessage(const QJsonObject& object)
{
    const QString cls = object.value("class").toString("unknown");
    const double conf = object.value("conf").toDouble(0.0);
    m_alarmLabel->setText(QString("告警状态：检测到 %1，置信度 %2").arg(cls).arg(conf, 0, 'f', 2));
    appendError(QString("故障告警: class=%1, conf=%2").arg(cls).arg(conf, 0, 'f', 2));
}
