#pragma once

#include <QColor>
#include <QElapsedTimer>
#include <QImage>
#include <QJsonObject>
#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;
class QTabWidget;
class QEvent;
class TcpClientManager;
class VideoReceiver;
class PoseImageReceiver;
class DebugLogWindow;

enum class SystemMode;

class DashboardPage : public QWidget
{
    Q_OBJECT

public:
    explicit DashboardPage(TcpClientManager* tcpClient,
                           VideoReceiver* videoReceiver,
                           QWidget* parent = nullptr);

    void setVideoPort(int port);
    void setConnectionInfo(const QString& host, quint16 port);

signals:
    void logoutRequested();
    void disconnectRequested();

public slots:
    void appendLog(const QString& message);
    void appendError(const QString& message);
    void appendBoardDebug(const QString& source,
                          const QString& stream,
                          const QString& message);
    void onServerMessage(const QJsonObject& object);
    void onTcpConnected();
    void onTcpDisconnected();
    void onVideoStarted();
    void onVideoStopped();
    void onVideoFrameReady(const QImage& frame);

private slots:
    void startVideoOnly();
    void stopVideoOnly();
    void startDetectTrack();
    void stopDetectTrack();
    void getSensorOnce();
    void startSensorSubscribe();
    void stopSensorSubscribe();
    void servoLeftPressed();
    void servoLeftReleased();
    void servoRightPressed();
    void servoRightReleased();
    void servoUpStep();
    void servoDownStep();
    void servoCenter();
    void stopAll();
    void sendManualCommand();
    void openDebugLogWindow();
    void clearLogs();
    void measurePoseOnce();
    void onPoseImageFrameReady(const QImage& frame);

protected:
    void resizeEvent(QResizeEvent* event) override;
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void buildUi();
    void updateMode(SystemMode mode);
    void updateButtonState();
    void sendCommand(const QJsonObject& object);
    void sendServoManualAction(const QString& action);
    void handleStatusMessage(const QJsonObject& object);
    void handleSensorMessage(const QJsonObject& object);
    void handleFaultMessage(const QJsonObject& object);
    void handleDetectStatusMessage(const QJsonObject& object);
    void handleServoStatusMessage(const QJsonObject& object);
    void updateVideoReceiveFps();
    void resetVideoReceiveFps();
    void appendText(QTextEdit* edit, const QString& text, const QColor& color = Qt::black);
    void refreshVideoPixmap();
    void handlePoseResultMessage(const QJsonObject& object);
    void handlePoseImageStreamMessage(const QJsonObject& object);
    void refreshPosePixmap();
    void showPoseImageDialog();

private:
    TcpClientManager* m_tcpClient = nullptr;
    VideoReceiver* m_videoReceiver = nullptr;
    DebugLogWindow* m_debugLogWindow = nullptr;
    PoseImageReceiver* m_poseImageReceiver = nullptr;

    int m_poseImagePort = 5002;
    QImage m_lastPoseImage;

    int m_videoPort = 5000;
    SystemMode m_mode;
    QImage m_lastFrame;

    QElapsedTimer m_videoFpsTimer;
    int m_videoFrameCount = 0;

    QLabel* m_connectionLabel = nullptr;
    QLabel* m_modeLabel = nullptr;
    QLabel* m_videoLabel = nullptr;
    QLabel* m_lightLabel = nullptr;
    QLabel* m_temperatureLabel = nullptr;
    QLabel* m_fpsLabel = nullptr;
    QLabel* m_servoLabel = nullptr;
    QLabel* m_alarmLabel = nullptr;
    QLabel* m_videoView = nullptr;
    QPushButton* m_poseMeasureButton = nullptr;
    QLabel* m_poseStatusLabel = nullptr;
    QLabel* m_poseImageView = nullptr;
    QTextEdit* m_poseTextEdit = nullptr;

    QPushButton* m_startVideoButton = nullptr;
    QPushButton* m_stopVideoButton = nullptr;
    QPushButton* m_startDetectButton = nullptr;
    QPushButton* m_stopDetectButton = nullptr;
    QPushButton* m_getSensorButton = nullptr;
    QPushButton* m_subscribeSensorButton = nullptr;
    QPushButton* m_unsubscribeSensorButton = nullptr;
    QPushButton* m_servoLeftButton = nullptr;
    QPushButton* m_servoRightButton = nullptr;
    QPushButton* m_servoUpButton = nullptr;
    QPushButton* m_servoDownButton = nullptr;
    QPushButton* m_servoCenterButton = nullptr;
    QPushButton* m_stopAllButton = nullptr;
    QPushButton* m_disconnectButton = nullptr;
    QPushButton* m_logoutButton = nullptr;
    QPushButton* m_openDebugWindowButton = nullptr;
    QPushButton* m_clearLogsButton = nullptr;

    QLineEdit* m_manualCommandEdit = nullptr;
    QPushButton* m_manualSendButton = nullptr;
    QTabWidget* m_logTabs = nullptr;
    QTextEdit* m_logEdit = nullptr;
    QTextEdit* m_boardLogEdit = nullptr;
};
