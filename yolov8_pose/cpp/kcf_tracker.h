#ifndef KCF_TRACKER_H
#define KCF_TRACKER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*-------------------------------------------
              跟踪器状态定义
-------------------------------------------*/
typedef enum {
    TRACKER_STATE_INVALID = 0,    // 无效状态
    TRACKER_STATE_DETECTING,      // 检测模式（无目标）
    TRACKER_STATE_TRACKING,       // 跟踪模式
    TRACKER_STATE_PREDICTING,     // 预测模式（目标短暂丢失）
    TRACKER_STATE_LOST            // 目标丢失
} TrackerState;

/*-------------------------------------------
              KCF跟踪器句柄
-------------------------------------------*/
typedef void* KCFTrackerHandle;

/*-------------------------------------------
             跟踪器配置参数
-------------------------------------------*/
typedef struct {
    // 检测参数
    int detection_interval;    // 检测间隔帧数
    float detection_threshold; // 检测置信度阈值
    
    // 跟踪参数
    int max_predict_frames;    // 最大预测帧数
    float track_confidence_threshold; // 跟踪置信度阈值
    
    // 匹配参数
    float iou_threshold;       // IOU匹配阈值
    int stable_frames_needed;  // 稳定跟踪所需帧数
} TrackerConfig;

/*-------------------------------------------
             跟踪器管理函数
-------------------------------------------*/
// 创建跟踪器
KCFTrackerHandle kcf_tracker_create(void);

// 销毁跟踪器
void kcf_tracker_destroy(KCFTrackerHandle handle);

// 初始化跟踪器（使用检测结果）
bool kcf_tracker_init(KCFTrackerHandle handle,
                     const uint8_t* image_data,
                     int width, int height,
                     int x, int y, int w, int h);

// 更新跟踪器
bool kcf_tracker_update(KCFTrackerHandle handle,
                       const uint8_t* image_data,
                       int width, int height,
                       int* out_x, int* out_y, int* out_w, int* out_h,
                       float* confidence);

// 重置跟踪器
void kcf_tracker_reset(KCFTrackerHandle handle);

// 获取跟踪器状态
TrackerState kcf_tracker_get_state(KCFTrackerHandle handle);

// 设置跟踪器参数
void kcf_tracker_set_config(KCFTrackerHandle handle, const TrackerConfig* config);

#ifdef __cplusplus
}
#endif

#endif /* KCF_TRACKER_H */