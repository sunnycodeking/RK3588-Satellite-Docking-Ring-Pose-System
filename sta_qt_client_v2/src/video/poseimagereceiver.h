#pragma once

#include <QImage>
#include <QObject>
#include <QString>

class QTimer;
typedef struct _GstElement GstElement;

class PoseImageReceiver : public QObject
{
    Q_OBJECT

public:
    explicit PoseImageReceiver(QObject* parent = nullptr);
    ~PoseImageReceiver() override;

    bool isRunning() const;

public slots:
    bool startRtpJpeg(int port = 5002);
    void stop();

signals:
    void frameReady(const QImage& frame);
    void logMessage(const QString& message);
    void errorMessage(const QString& message);
    void started();
    void stopped();

private slots:
    void pollBus();
    void deliverFrame(const QImage& frame);

private:
    QString pipelineDescription(int port) const;
    void cleanupPipeline(bool notifyStopped);

private:
    GstElement* m_pipeline = nullptr;
    GstElement* m_appsink = nullptr;
    QTimer* m_busTimer = nullptr;
    bool m_running = false;
};
