#pragma once

#include <QJsonArray>
#include <QJsonObject>
#include <QString>

namespace Protocol {

static constexpr int kDefaultControlPort = 8888;
static constexpr int kDefaultVideoPort = 5000;
static constexpr int kDefaultPoseImagePort = 5002;
static constexpr const char* kClientName = "ubuntu_qt_client";
static constexpr const char* kClientVersion = "2.0.0";

inline QJsonObject hello(int videoPort)
{
    QJsonObject obj;
    obj["cmd"] = "HELLO";
    obj["client"] = kClientName;
    obj["version"] = kClientVersion;
    obj["video_port"] = videoPort;
    return obj;
}

inline QJsonObject simpleCommand(const QString& cmd)
{
    QJsonObject obj;
    obj["cmd"] = cmd;
    return obj;
}

inline QJsonObject startMode(const QString& mode, int videoPort, const QString& target = QString())
{
    QJsonObject obj;
    obj["cmd"] = "START_MODE";
    obj["mode"] = mode;
    obj["video_port"] = videoPort;
    if (!target.isEmpty()) {
        obj["target"] = target;
    }
    return obj;
}

inline QJsonObject stopMode(const QString& mode)
{
    QJsonObject obj;
    obj["cmd"] = "STOP_MODE";
    obj["mode"] = mode;
    return obj;
}

inline QJsonObject startVideoPreview(int videoPort = kDefaultVideoPort)
{
    QJsonObject obj;
    obj["cmd"] = "START_MODE";
    obj["mode"] = "video_preview";
    obj["video_port"] = videoPort;
    return obj;
}

inline QJsonObject stopVideoPreview()
{
    QJsonObject obj;
    obj["cmd"] = "STOP_MODE";
    obj["mode"] = "video_preview";
    return obj;
}

inline QJsonObject servoManual(const QString& action)
{
    QJsonObject obj;
    obj["cmd"] = "SERVO_MANUAL";
    obj["action"] = action;
    return obj;
}

inline QJsonObject getSensor(const QString& type = "all")
{
    QJsonObject obj;
    obj["cmd"] = "GET_SENSOR";
    obj["type"] = type;
    return obj;
}

inline QJsonObject subscribeSensor(int periodMs = 1000)
{
    QJsonArray items;
    items.append("light");
    items.append("temperature");

    QJsonObject obj;
    obj["cmd"] = "SUBSCRIBE_SENSOR";
    obj["items"] = items;
    obj["period_ms"] = periodMs;
    return obj;
}

inline QJsonObject unsubscribeSensor()
{
    QJsonArray items;
    items.append("light");
    items.append("temperature");

    QJsonObject obj;
    obj["cmd"] = "UNSUBSCRIBE_SENSOR";
    obj["items"] = items;
    return obj;
}


inline QJsonObject measurePose(int imagePort = kDefaultPoseImagePort)
{
    QJsonObject obj;
    obj["cmd"] = "MEASURE_POSE";
    obj["image_port"] = imagePort;
    return obj;
}

} // namespace Protocol
