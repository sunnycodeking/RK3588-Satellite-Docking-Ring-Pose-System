#pragma once

#include <QString>

enum class SystemMode {
    Disconnected,
    Idle,
    VideoOnly,
    Patrol,
    DetectTrack,
    SensorMonitor,
    Error
};

inline QString systemModeToString(SystemMode mode)
{
    switch (mode) {
    case SystemMode::Disconnected: return "未连接";
    case SystemMode::Idle: return "空闲";
    case SystemMode::VideoOnly: return "视频预览";
    case SystemMode::Patrol: return "巡检模式";
    case SystemMode::DetectTrack: return "检测跟踪";
    case SystemMode::SensorMonitor: return "传感器监测";
    case SystemMode::Error: return "异常";
    }
    return "未知";
}

inline SystemMode systemModeFromServerString(const QString& mode)
{
    if (mode == "idle") return SystemMode::Idle;
    if (mode == "detect_track") return SystemMode::DetectTrack;
    if (mode == "patrol") return SystemMode::Patrol;
    if (mode == "video") return SystemMode::VideoOnly;
    if (mode == "video_preview") return SystemMode::VideoOnly;
    if (mode == "sensor") return SystemMode::SensorMonitor;
    if (mode == "error") return SystemMode::Error;
    return SystemMode::Idle;
}
