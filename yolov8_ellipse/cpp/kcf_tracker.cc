#include "kcf_tracker.h"

/* 包含您本地的OpenCV头文件 */
#include "opencv2/opencv.hpp"
#include "opencv2/tracking/tracking.hpp"

#include <stdlib.h>
#include <string.h>
#include <cmath> // 添加 fabs 支持
#include <algorithm> // 添加 clamp 支持

/*-------------------------------------------
          KCF跟踪器内部实现结构体
-------------------------------------------*/
typedef struct {
    cv::Ptr<cv::Tracker> tracker;  // OpenCV跟踪器智能指针
    cv::Rect2d bbox;               // 当前跟踪框
    bool is_initialized;           // 是否已初始化
    float confidence;              // 跟踪置信度
    int lost_frames;               // 连续丢失帧数
    TrackerState state;            // 当前状态
    
    // 历史位置平滑（可选）
    double smooth_x, smooth_y, smooth_w, smooth_h;
    double alpha; // 平滑因子
} KCFTrackerImpl;

/*-------------------------------------------
               创建跟踪器
-------------------------------------------*/
KCFTrackerHandle kcf_tracker_create(void) {
    KCFTrackerImpl* impl = (KCFTrackerImpl*)malloc(sizeof(KCFTrackerImpl));
    if (!impl) return NULL;
    
    memset(impl, 0, sizeof(KCFTrackerImpl));
    
    // 创建OpenCV的KCF跟踪器实例
    impl->tracker = cv::TrackerKCF::create();
    if (impl->tracker.empty()) {
        free(impl);
        return NULL;
    }
    
    impl->is_initialized = false;
    impl->confidence = 0.0f;
    impl->lost_frames = 0;
    impl->state = TRACKER_STATE_DETECTING;
    impl->alpha = 0.3; // 平滑系数
    
    return (KCFTrackerHandle)impl;
}

/*-------------------------------------------
               销毁跟踪器
-------------------------------------------*/
void kcf_tracker_destroy(KCFTrackerHandle handle) {
    if (!handle) return;
    
    KCFTrackerImpl* impl = (KCFTrackerImpl*)handle;
    
    // OpenCV的Ptr会自动释放，我们只需要释放自己的结构体
    free(impl);
}

/*-------------------------------------------
               初始化跟踪器
-------------------------------------------*/


bool kcf_tracker_init(KCFTrackerHandle handle,
                     const uint8_t* image_data,
                     int width, int height,
                     int x, int y, int w, int h) {
    if (!handle || !image_data || w <= 0 || h <= 0) {
        return false;
    }
    KCFTrackerImpl* impl = (KCFTrackerImpl*)handle;
    if (x < 0 || y < 0 || w <= 0 || h <= 0 || 
        (x + w) > width || (y + h) > height) {
        fprintf(stderr, "Invalid ROI: (%d,%d,%d,%d) for image %dx%d\n", 
                x, y, w, h, width, height);
        impl->is_initialized = false;
        return false;
    }

    // 完全释放旧跟踪器
    if (impl->tracker) {
        impl->tracker.release();  // 显式释放资源
        impl->tracker = nullptr;  // 置空指针
    }

    try {
        // 修复：OpenCV 4.5.4中init返回void，不返回bool
        // 直接调用，不检查返回值
        impl->tracker = cv::TrackerKCF::create();
        if (impl->tracker.empty()) {
            fprintf(stderr, "Failed to create new KCF tracker\n");
            return false;
        }
        
        // 转换图像格式
        cv::Mat rgb_frame(height, width, CV_8UC3, (void*)image_data);
        cv::Mat bgr_frame;
        cv::cvtColor(rgb_frame, bgr_frame, cv::COLOR_RGB2BGR);
        
        // 设置初始边界框
        cv::Rect init_rect(x, y, w, h);
        impl->tracker->init(bgr_frame, init_rect);
        // 更新内部状态
        impl->bbox = cv::Rect2d(init_rect);
        impl->is_initialized = true;
        impl->smooth_x = x + w/2.0;
        impl->smooth_y = y + h/2.0;
        impl->smooth_w = w;
        impl->smooth_h = h;

        impl->confidence = 0.9f; // 初始高置信度
        impl->lost_frames = 0;
        impl->state = TRACKER_STATE_TRACKING;
        
        
        return true;
    } catch (const cv::Exception& e) {
        fprintf(stderr, "KCF init failed: %s\n", e.what());
    }
    
    return false;
}

/*-------------------------------------------
               更新跟踪器
-------------------------------------------*/

bool kcf_tracker_update(KCFTrackerHandle handle,
                       const uint8_t* image_data,
                       int width, int height,
                       int* out_x, int* out_y, int* out_w, int* out_h,
                       float* confidence) {
    // 输入验证
    if (!handle || !image_data || !out_x || !out_y || !out_w || !out_h ||
        width <= 0 || height <= 0) {
        return false;
    }
    
    KCFTrackerImpl* impl = (KCFTrackerImpl*)handle;
    if (!impl->is_initialized) {
        return false;
    }
    
    // 创建图像矩阵（添加边界检查）
    cv::Mat rgb_frame;
    try {
        rgb_frame = cv::Mat(height, width, CV_8UC3, (void*)image_data);
        if (rgb_frame.empty()) {
            fprintf(stderr, "Error: Failed to create RGB frame\n");
            return false;
        }
    } catch (const cv::Exception& e) {
        fprintf(stderr, "Error creating RGB frame: %s\n", e.what());
        return false;
    }
    
    cv::Mat bgr_frame;
    try {
        cv::cvtColor(rgb_frame, bgr_frame, cv::COLOR_RGB2BGR);
    } catch (const cv::Exception& e) {
        fprintf(stderr, "Color conversion failed: %s\n", e.what());
        return false;
    }
    
    bool success = false;
    cv::Rect tracked_rect;  // 使用整数矩形接收结果
    



    try {
        // 修复：使用整数矩形接收结果
        success = impl->tracker->update(bgr_frame, tracked_rect);
        
        if (success && tracked_rect.width > 0 && tracked_rect.height > 0) {
            // 将整数矩形转换为浮点矩形用于计算
            cv::Rect2d new_bbox(tracked_rect);
            
            impl->bbox = new_bbox;
            impl->lost_frames = 0;
            
            // 修复：使用 fabs 代替 abs 处理浮点数
            double width_diff = new_bbox.width - impl->smooth_w;
            double height_diff = new_bbox.height - impl->smooth_h;
            double box_change = (std::fabs(width_diff) / impl->smooth_w) + 
                              (std::fabs(height_diff) / impl->smooth_h);
            
            impl->confidence = static_cast<float>(std::fmax(0.0, 1.0 - box_change * 2.0));
            
            // 应用指数平滑
            double center_x = new_bbox.x + new_bbox.width/2.0;
            double center_y = new_bbox.y + new_bbox.height/2.0;
            
            impl->smooth_x = impl->alpha * center_x + (1 - impl->alpha) * impl->smooth_x;
            impl->smooth_y = impl->alpha * center_y + (1 - impl->alpha) * impl->smooth_y;
            impl->smooth_w = impl->alpha * new_bbox.width + (1 - impl->alpha) * impl->smooth_w;
            impl->smooth_h = impl->alpha * new_bbox.height + (1 - impl->alpha) * impl->smooth_h;
            
            // 修复：确保坐标在有效范围内
            *out_x = std::clamp(static_cast<int>(impl->smooth_x - impl->smooth_w/2), 0, width - 1);
            *out_y = std::clamp(static_cast<int>(impl->smooth_y - impl->smooth_h/2), 0, height - 1);
            *out_w = std::clamp(static_cast<int>(impl->smooth_w), 1, width - *out_x);
            *out_h = std::clamp(static_cast<int>(impl->smooth_h), 1, height - *out_y);
            
            impl->state = (impl->confidence > 0.3f) ? 
                         TRACKER_STATE_TRACKING : TRACKER_STATE_PREDICTING;
        } else {
            impl->lost_frames++;
            impl->confidence = 0.0f;
            
            // 使用最后已知位置（带边界检查）
            *out_x = std::clamp(static_cast<int>(impl->smooth_x - impl->smooth_w/2), 0, width - 1);
            *out_y = std::clamp(static_cast<int>(impl->smooth_y - impl->smooth_h/2), 0, height - 1);
            *out_w = std::clamp(static_cast<int>(impl->smooth_w), 1, width - *out_x);
            *out_h = std::clamp(static_cast<int>(impl->smooth_h), 1, height - *out_y);
            
            if (impl->lost_frames > 5) {
                impl->state = TRACKER_STATE_PREDICTING;
            }
            if (impl->lost_frames > 15) {
                impl->state = TRACKER_STATE_LOST;
            }
            
            success = false;
        }
    } catch (const cv::Exception& e) {
        fprintf(stderr, "KCF update failed: %s\n", e.what());
        success = false;
    }
    
    if (confidence) {
        *confidence = impl->confidence;
    }
    
    return success;
}
/*-------------------------------------------
               重置跟踪器
-------------------------------------------*/
void kcf_tracker_reset(KCFTrackerHandle handle) {
    if (!handle) return;
    
    KCFTrackerImpl* impl = (KCFTrackerImpl*)handle;
    
    // 重新创建跟踪器实例
    impl->tracker = cv::TrackerKCF::create();
    impl->is_initialized = false;
    impl->confidence = 0.0f;
    impl->lost_frames = 0;
    impl->state = TRACKER_STATE_DETECTING;
    
    // 重置平滑值
    impl->smooth_x = impl->smooth_y = impl->smooth_w = impl->smooth_h = 0;
}

/*-------------------------------------------
             获取跟踪器状态
-------------------------------------------*/
TrackerState kcf_tracker_get_state(KCFTrackerHandle handle) {
    if (!handle) return TRACKER_STATE_INVALID;
    
    KCFTrackerImpl* impl = (KCFTrackerImpl*)handle;
    return impl->state;
}

/*-------------------------------------------
             设置跟踪器配置
-------------------------------------------*/
void kcf_tracker_set_config(KCFTrackerHandle handle, const TrackerConfig* config) {
    if (!handle || !config) return;
    
    KCFTrackerImpl* impl = (KCFTrackerImpl*)handle;
    
    // 这里可以根据配置调整跟踪器参数
    // 注意：OpenCV的TrackerKCF参数需要通过create()设置
    // 所以如果需要调整参数，需要重新创建跟踪器
    
    // 示例：调整平滑系数
    impl->alpha = config->track_confidence_threshold * 0.3f;
    if (impl->alpha < 0.1f) impl->alpha = 0.1f;
    if (impl->alpha > 0.9f) impl->alpha = 0.9f;
}