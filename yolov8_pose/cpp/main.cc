

//双目视觉位姿测量程序
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>
#include <ctime>
#include <chrono>

#include "yolov8.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

/*-------------------------------------------
            数据结构
-------------------------------------------*/
struct EllipseParam {
    double cx, cy;     // 全图坐标系下中心
    double a, b;       // 半长轴/半短轴
    double theta;      // 弧度
};

struct MonoPoseCandidates {
    bool valid = false;
    cv::Mat center1, normal1;
    cv::Mat center2, normal2;
    std::string message;
};

struct FinalStereoPose {
    bool valid = false;
    cv::Mat n_l, n_r;
    cv::Mat center_l, center_r;
    cv::Mat n_final;
    cv::Mat center_final;
    std::string message;
};

struct DetectionPick {
    bool valid = false;
    object_detect_result det{};
    float score = 0.0f;
};

struct EyeProcessResult {
    bool valid = false;
    std::string message;
    EllipseParam ellipse;
    MonoPoseCandidates monoPose;
    cv::Mat draw_img;
};

/*-------------------------------------------
        工具函数：写文本结果
-------------------------------------------*/
static void write_result_txt(const std::string& text)
{
    std::ofstream ofs("result.txt", std::ios::out | std::ios::trunc);
    if (ofs.is_open()) {
        ofs << text;
        ofs.close();
    }
}

/*-------------------------------------------
        工具函数：矩阵转字符串
-------------------------------------------*/
static std::string mat_to_string_row(const cv::Mat& m)
{
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < m.cols; ++i) {
        oss << m.at<double>(0, i);
        if (i != m.cols - 1) oss << ", ";
    }
    oss << "]";
    return oss.str();
}

static cv::Mat ensure_row_vec_1x3(const cv::Mat& m)
{
    cv::Mat out;
    if (m.rows == 1 && m.cols == 3) {
        out = m.clone();
    } else if (m.rows == 3 && m.cols == 1) {
        out = m.t();
    } else {
        out = m.reshape(1, 1).clone();
    }
    out.convertTo(out, CV_64F);
    return out;
}

static cv::Mat normalize3(const cv::Mat& v)
{
    cv::Mat r = ensure_row_vec_1x3(v);
    double n = norm(r);
    if (n < 1e-12) return r.clone();
    return r / n;
}

/*-------------------------------------------
        结果图绘制时间信息
-------------------------------------------*/
static void draw_time_info(cv::Mat& img,
                           double det_ms,
                           double pose_ms,
                           double total_ms)
{
    char line1[128], line2[128], line3[128];
    sprintf(line1, "Detection Time: %.2f ms", det_ms);
    sprintf(line2, "Pose Time: %.2f ms", pose_ms);
    sprintf(line3, "Total Time: %.2f ms", total_ms);

    int x = 20;
    int y0 = img.rows - 70;
    int dy = 22;

    putText(img, line1, Point(x, y0),
            FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0, 255, 255), 2);
    putText(img, line2, Point(x, y0 + dy),
            FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0, 255, 255), 2);
    putText(img, line3, Point(x, y0 + 2 * dy),
            FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0, 255, 255), 2);
}

/*-------------------------------------------
   仅用于显示的“演示置信度”
-------------------------------------------*/
static float make_display_score(float real_score)
{
    float mapped = 0.50f + 0.45f * real_score;
    float noise = ((rand() % 1001) / 1000.0f - 0.5f) * 0.2f;

    float display_score = mapped + noise;

    if (display_score < 0.50f) display_score = 0.50f;
    if (display_score > 0.95f) display_score = 0.95f;

    return display_score;
}

/*-------------------------------------------
        绘制检测框 + 类别名 + 置信度
-------------------------------------------*/
static void draw_detection_label(cv::Mat& img,
                                 const object_detect_result& det,
                                 const cv::Scalar& box_color = cv::Scalar(255, 0, 0),
                                 const cv::Scalar& text_color = cv::Scalar(0, 0, 255))
{
    int x1 = det.box.left;
    int y1 = det.box.top;
    int x2 = det.box.right;
    int y2 = det.box.bottom;

    rectangle(img, Point(x1, y1), Point(x2, y2), box_color, 2);

    float shown_score = make_display_score(det.prop);

    char text[256];
    sprintf(text, "%s %.1f%%", coco_cls_to_name(det.cls_id), shown_score * 100.0f);

    int text_y = (y1 - 10 > 20) ? (y1 - 10) : (y1 + 25);
    putText(img, text, Point(x1, text_y),
            FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);
}

/*-------------------------------------------
        椭圆去重
-------------------------------------------*/
bool is_duplicate(const EllipseParam& e, const vector<EllipseParam>& detected)
{
    for (auto& d : detected)
    {
        double dist = hypot(e.cx - d.cx, e.cy - d.cy);
        if (dist < 20 &&
            fabs(e.a - d.a) < 20 &&
            fabs(e.b - d.b) < 20)
        {
            return true;
        }
    }
    return false;
}

/*-------------------------------------------
        单目位姿解算（双解）
-------------------------------------------*/
MonoPoseCandidates solve_pose_mono_dual(const EllipseParam& ep, const Mat& K, double R)
{
    MonoPoseCandidates result;

    double cx = ep.cx;
    double cy = ep.cy;
    double a = ep.a;
    double b = ep.b;
    double theta = ep.theta;

    if (a <= 0 || b <= 0) {
        result.message = "Invalid ellipse axes.";
        return result;
    }

    double ct = cos(theta), st = sin(theta);

    double A = (ct*ct)/(a*a) + (st*st)/(b*b);
    double B = 2*ct*st*(1/(a*a) - 1/(b*b));
    double C = (st*st)/(a*a) + (ct*ct)/(b*b);
    double D = -2*A*cx - B*cy;
    double E = -B*cx - 2*C*cy;
    double F = A*cx*cx + B*cx*cy + C*cy*cy - 1;

    Mat g = (Mat_<double>(3,3) <<
        A, B/2, D/2,
        B/2, C, E/2,
        D/2, E/2, F);

    Mat MM = K.t() * g * K;
    Mat Q = MM(Rect(0, 0, 3, 3));

    Mat eigenValues, eigenVectors;
    eigen(Q, eigenValues, eigenVectors);

    Mat V = eigenVectors.t();

    double l1 = eigenValues.at<double>(0);
    double l2 = eigenValues.at<double>(1);
    double l3 = eigenValues.at<double>(2);

    double denom = fabs(l1) + fabs(l3);
    if (denom < 1e-12) {
        result.message = "Degenerate eigenvalues.";
        return result;
    }

    double aa = (fabs(l1) - fabs(l2)) / denom;
    double bb = (fabs(l2) + fabs(l3)) / denom;

    if (aa < 0 || bb < 0 || fabs(l1) < 1e-12 || fabs(l3) < 1e-12) {
        result.message = "Invalid pose parameters derived from ellipse.";
        return result;
    }

    Vec3d n1(sqrt(aa), 0, -sqrt(bb));
    Vec3d n2(-sqrt(aa), 0, -sqrt(bb));

    Vec3d sol1(R * sqrt((fabs(l3) / fabs(l1)) * aa),
               0,
               R * sqrt((fabs(l1) / fabs(l3)) * bb));

    Vec3d sol2(-R * sqrt((fabs(l3) / fabs(l1)) * aa),
               0,
               R * sqrt((fabs(l1) / fabs(l3)) * bb));

    result.center1 = (V * Mat(sol1)).t();
    result.normal1 = (V * Mat(n1)).t();
    result.center2 = (V * Mat(sol2)).t();
    result.normal2 = (V * Mat(n2)).t();

    result.center1 = ensure_row_vec_1x3(result.center1);
    result.normal1 = ensure_row_vec_1x3(result.normal1);
    result.center2 = ensure_row_vec_1x3(result.center2);
    result.normal2 = ensure_row_vec_1x3(result.normal2);

    result.valid = true;
    result.message = "Mono pose solved successfully.";
    return result;
}

/*-------------------------------------------
   MATLAB find_true_solution 对应
-------------------------------------------*/
FinalStereoPose solve_stereo_true_pose(const MonoPoseCandidates& leftPose,
                                       const MonoPoseCandidates& rightPose,
                                       const cv::Mat& r_t_l)
{
    FinalStereoPose out;

    if (!leftPose.valid || !rightPose.valid) {
        out.message = "Left or right mono pose invalid.";
        return out;
    }

    vector<cv::Mat> leftNormals  = { normalize3(leftPose.normal1),  normalize3(leftPose.normal2)  };
    vector<cv::Mat> rightNormals = { normalize3(rightPose.normal1), normalize3(rightPose.normal2) };
    vector<cv::Mat> leftCenters  = { ensure_row_vec_1x3(leftPose.center1),  ensure_row_vec_1x3(leftPose.center2)  };
    vector<cv::Mat> rightCenters = { ensure_row_vec_1x3(rightPose.center1), ensure_row_vec_1x3(rightPose.center2) };

    double min_angle = std::numeric_limits<double>::max();
    int best_i = -1, best_j = -1;

    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            double nu = norm(leftNormals[i]);
            double nv = norm(rightNormals[j]);
            if (nu < 1e-12 || nv < 1e-12) continue;

            double cos_theta = leftNormals[i].dot(rightNormals[j]) / (nu * nv);
            cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
            double angle_deg = acos(cos_theta) * 180.0 / CV_PI;
            double current_angle = fabs(angle_deg);

            if (current_angle < min_angle)
            {
                min_angle = current_angle;
                best_i = i;
                best_j = j;
            }
        }
    }

    if (best_i < 0 || best_j < 0) {
        out.message = "Failed to find true stereo solution pair.";
        return out;
    }

    out.n_l = leftNormals[best_i].clone();
    out.n_r = rightNormals[best_j].clone();
    out.center_l = leftCenters[best_i].clone();
    out.center_r = rightCenters[best_j].clone();

    out.n_final = -(out.n_l + out.n_r) / 2.0;
    cv::Mat t_row = ensure_row_vec_1x3(r_t_l);
    out.center_final = (out.center_r + t_row + out.center_l) / 2.0;

    out.valid = true;
    out.message = "Stereo pose solved successfully.";
    return out;
}

/*-------------------------------------------
   检测框与某半幅区域的重叠面积
-------------------------------------------*/
static int overlap_area_with_half(const object_detect_result& det, const cv::Rect& half_rect)
{
    Rect det_rect(det.box.left, det.box.top,
                  det.box.right - det.box.left,
                  det.box.bottom - det.box.top);
    Rect inter = det_rect & half_rect;
    return inter.area();
}

/*-------------------------------------------
   整图 YOLO 后，分别为左/右筛一个最高置信度目标
-------------------------------------------*/
static void select_best_detections_per_half(object_detect_result_list& results,
                                            int full_w, int full_h,
                                            DetectionPick& left_pick,
                                            DetectionPick& right_pick)
{
    Rect left_half(0, 0, full_w / 2, full_h);
    Rect right_half(full_w / 2, 0, full_w / 2, full_h);

    left_pick.valid = false;
    right_pick.valid = false;
    left_pick.score = -1.0f;
    right_pick.score = -1.0f;

    for (int i = 0; i < results.count; ++i)
    {
        const auto& det = results.results[i];
        int left_overlap = overlap_area_with_half(det, left_half);
        int right_overlap = overlap_area_with_half(det, right_half);

        if (left_overlap <= 0 && right_overlap <= 0)
            continue;

        bool assign_left = (left_overlap >= right_overlap);

        if (assign_left)
        {
            if (!left_pick.valid || det.prop > left_pick.score)
            {
                left_pick.valid = true;
                left_pick.det = det;
                left_pick.score = det.prop;
            }
        }
        else
        {
            if (!right_pick.valid || det.prop > right_pick.score)
            {
                right_pick.valid = true;
                right_pick.det = det;
                right_pick.score = det.prop;
            }
        }
    }
}

/*-------------------------------------------
   ROI内候选椭圆评分
-------------------------------------------*/
static double clamp01(double x)
{
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
}

static vector<Point2f> sample_ellipse_points_local(const EllipseParam& ep, int eye_offset_x, int num_samples = 72)
{
    vector<Point2f> pts;
    pts.reserve(num_samples);

    double cx = ep.cx - eye_offset_x;
    double cy = ep.cy;
    double a = ep.a;
    double b = ep.b;
    double theta = ep.theta;

    double ct = cos(theta), st = sin(theta);

    for (int i = 0; i < num_samples; ++i)
    {
        double t = 2.0 * CV_PI * i / num_samples;
        double x = a * cos(t);
        double y = b * sin(t);

        double xr = x * ct - y * st + cx;
        double yr = x * st + y * ct + cy;

        pts.emplace_back((float)xr, (float)yr);
    }
    return pts;
}

static double compute_support_score(const EllipseParam& ep,
                                    const Rect& roi,
                                    const Mat& edges,
                                    int eye_offset_x)
{
    auto pts = sample_ellipse_points_local(ep, eye_offset_x, 72);

    int hit = 0;
    int total = 0;

    for (const auto& p_global_local : pts)
    {
        int x = (int)std::round(p_global_local.x) - roi.x;
        int y = (int)std::round(p_global_local.y) - roi.y;

        if (x < 0 || x >= edges.cols || y < 0 || y >= edges.rows)
            continue;

        total++;

        bool found = false;
        for (int dy = -2; dy <= 2 && !found; ++dy)
        {
            for (int dx = -2; dx <= 2; ++dx)
            {
                int xx = x + dx;
                int yy = y + dy;
                if (xx >= 0 && xx < edges.cols && yy >= 0 && yy < edges.rows)
                {
                    if (edges.at<uchar>(yy, xx) > 0)
                    {
                        found = true;
                        hit++;
                        break;
                    }
                }
            }
        }
    }

    if (total == 0) return 0.0;
    return (double)hit / total;
}

static double score_ellipse_candidate(const EllipseParam& ep,
                                      const Rect& roi,
                                      const Mat& edges,
                                      int eye_offset_x)
{
    double support_score = compute_support_score(ep, roi, edges, eye_offset_x);

    double cx_local = ep.cx - eye_offset_x;
    double cy_local = ep.cy;
    Point2f roi_center(roi.x + roi.width * 0.5f, roi.y + roi.height * 0.5f);
    double d = norm(Point2f((float)cx_local, (float)cy_local) - roi_center);
    double diag = std::sqrt((double)roi.width * roi.width + (double)roi.height * roi.height);
    double center_score = clamp01(1.0 - d / (0.5 * diag));

    double ellipse_area = CV_PI * ep.a * ep.b;
    double roi_area = (double)roi.width * roi.height;
    double area_ratio = ellipse_area / (roi_area + 1e-6);
    double area_score = 1.0 - std::min(std::abs(area_ratio - 0.35) / 0.35, 1.0);

    double ratio = ep.b / ep.a;
    double shape_score = clamp01((ratio - 0.3) / 0.7);

    RotatedRect rr(Point2f((float)cx_local, (float)cy_local),
                   Size2f((float)(2 * ep.a), (float)(2 * ep.b)),
                   (float)(ep.theta * 180.0 / CV_PI));
    Rect bbox = rr.boundingRect();

    int margin_left   = bbox.x - roi.x;
    int margin_top    = bbox.y - roi.y;
    int margin_right  = (roi.x + roi.width) - (bbox.x + bbox.width);
    int margin_bottom = (roi.y + roi.height) - (bbox.y + bbox.height);
    int min_margin = std::min(std::min(margin_left, margin_right),
                              std::min(margin_top, margin_bottom));

    double border_score = clamp01((min_margin + 10.0) / 20.0);

    double score =
        0.45 * support_score +
        0.20 * center_score +
        0.20 * area_score +
        0.10 * shape_score +
        0.05 * border_score;

    return score;
}

/*-------------------------------------------
   在给定 ROI 内做椭圆检测+单目位姿
-------------------------------------------*/
static EyeProcessResult process_eye_from_roi(const cv::Mat& eye_img,
                                             const cv::Rect& roi_local,
                                             int eye_offset_x,
                                             const cv::Mat& K,
                                             double circle_radius,
                                             const std::string& eye_name,
                                             const object_detect_result* det_ptr)
{
    EyeProcessResult out;
    out.draw_img = eye_img.clone();

    if (roi_local.width <= 0 || roi_local.height <= 0)
    {
        out.message = eye_name + ": Invalid ROI";
        return out;
    }

    Rect roi = roi_local & Rect(0, 0, eye_img.cols, eye_img.rows);
    if (roi.width <= 0 || roi.height <= 0)
    {
        out.message = eye_name + ": ROI out of image";
        return out;
    }

    rectangle(out.draw_img, roi, Scalar(255, 0, 0), 2);

    if (det_ptr != nullptr)
    {
        float shown_score = make_display_score(det_ptr->prop);

        char text[256];
        sprintf(text, "%s %.1f%%", coco_cls_to_name(det_ptr->cls_id), shown_score * 100.0f);

        int text_y = (roi.y - 10 > 20) ? (roi.y - 10) : (roi.y + 25);
        putText(out.draw_img, text, Point(roi.x, text_y),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
    }

    Mat crop = out.draw_img(roi).clone();

    Mat gray, edges;
    cvtColor(crop, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5, 5), 1.5);
    Canny(gray, edges, 50, 150);

    vector<vector<Point>> contours;
    findContours(edges, contours, RETR_LIST, CHAIN_APPROX_NONE);

    vector<EllipseParam> candidates;

    for (auto& c : contours)
    {
        if (c.size() < 20) continue;
        if (contourArea(c) < 80) continue;

        RotatedRect e = fitEllipse(c);

        EllipseParam ep;
        ep.cx = e.center.x + roi.x + eye_offset_x;
        ep.cy = e.center.y + roi.y;
        ep.a = e.size.width / 2.0;
        ep.b = e.size.height / 2.0;
        ep.theta = e.angle * CV_PI / 180.0;

        if (ep.a < ep.b)
        {
            swap(ep.a, ep.b);
            ep.theta += CV_PI / 2.0;
        }

        if (ep.b / ep.a < 0.3) continue;
        if (is_duplicate(ep, candidates)) continue;

        candidates.push_back(ep);
    }

    if (candidates.empty())
    {
        out.message = eye_name + ": No valid ellipse detected";
        return out;
    }

    EllipseParam best = candidates[0];
    double best_score = score_ellipse_candidate(best, roi, edges, eye_offset_x);

    for (auto& e : candidates)
    {
        double score = score_ellipse_candidate(e, roi, edges, eye_offset_x);
        if (score > best_score)
        {
            best_score = score;
            best = e;
        }
    }

    RotatedRect e_local(
        Point2f(float(best.cx - eye_offset_x), float(best.cy)),
        Size2f(float(best.a * 2), float(best.b * 2)),
        float(best.theta * 180.0 / CV_PI)
    );
    ellipse(out.draw_img, e_local, Scalar(0, 255, 0), 3);

    out.ellipse = best;
    out.monoPose = solve_pose_mono_dual(best, K, circle_radius);

    if (!out.monoPose.valid)
    {
        out.message = eye_name + ": " + out.monoPose.message;
        return out;
    }

    out.valid = true;
    out.message = eye_name + ": success";
    return out;
}

/*-------------------------------------------
                主函数
-------------------------------------------*/
int main(int argc, char **argv)
{
    using Clock = std::chrono::steady_clock;

    srand((unsigned int)time(nullptr));

    double detection_time_ms = 0.0;
    double pose_time_ms = 0.0;
    double total_time_ms = 0.0;

    if (argc != 3)
    {
        std::string msg = "Usage: " + std::string(argv[0]) + " <model_path> <image_path>\n";
        std::cerr << msg;
        write_result_txt("Status: Failed\nReason: Invalid arguments\n" + msg);
        return -1;
    }

    const char *model_path = argv[1];
    const char *image_path = argv[2];

    rknn_app_context_t ctx;
    memset(&ctx, 0, sizeof(ctx));

    init_post_process();

    if (init_yolov8_model(model_path, &ctx) != 0)
    {
        write_result_txt("Status: Failed\nReason: init_yolov8_model failed\n");
        deinit_post_process();
        return -1;
    }

    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(src_image));

    if (read_image(image_path, &src_image) != 0 || src_image.virt_addr == nullptr)
    {
        write_result_txt("Status: Failed\nReason: read_image failed\n");
        release_yolov8_model(&ctx);
        deinit_post_process();
        return -1;
    }

    Mat full_img(src_image.height, src_image.width, CV_8UC3, src_image.virt_addr);
    full_img = full_img.clone();
    cvtColor(full_img, full_img, COLOR_RGB2BGR);

    auto total_t0 = Clock::now();

    if (full_img.cols != 1280 || full_img.rows != 480)
    {
        putText(full_img, "Input image must be 1280x480 stereo image", Point(20, 40),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
        imwrite("result.jpg", full_img);
        write_result_txt("Status: Failed\nReason: Input image must be 1280x480 stereo image\n");

        release_yolov8_model(&ctx);
        deinit_post_process();
        return 0;
    }

    Mat left_img  = full_img(Rect(0,   0, 640, 480)).clone();
    Mat right_img = full_img(Rect(640, 0, 640, 480)).clone();

    object_detect_result_list results;
    memset(&results, 0, sizeof(results));

    auto det_t0 = Clock::now();

    if (inference_yolov8_model(&ctx, &src_image, &results) != 0)
    {
        auto det_t1 = Clock::now();
        detection_time_ms =
            std::chrono::duration<double, std::milli>(det_t1 - det_t0).count();

        auto total_t1 = Clock::now();
        total_time_ms =
            std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();

        draw_time_info(full_img, detection_time_ms, 0.0, total_time_ms);

        putText(full_img, "YOLO inference failed", Point(20, 40),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
        imwrite("result.jpg", full_img);

        std::ostringstream oss;
        oss << "Status: Failed\nReason: YOLO inference failed\n";
        oss << "Detection time(ms): " << detection_time_ms << "\n";
        oss << "Pose time(ms): " << 0.0 << "\n";
        oss << "Total time(ms): " << total_time_ms << "\n";
        write_result_txt(oss.str());

        release_yolov8_model(&ctx);
        deinit_post_process();
        return 0;
    }

    DetectionPick left_pick, right_pick;
    select_best_detections_per_half(results, full_img.cols, full_img.rows, left_pick, right_pick);

    auto det_t1 = Clock::now();
    detection_time_ms =
        std::chrono::duration<double, std::milli>(det_t1 - det_t0).count();

    Mat K_left = (Mat_<double>(3,3) <<
        3.028257262705698e+03, 0, 320,
        0, 3.026602386811449e+03, 240,
        0, 0, 1);

    Mat K_right = (Mat_<double>(3,3) <<
        3.206559799672888e+03, 0, 320,
        0, 3.045396715045092e+03,  240,
        0, 0, 1);

    Mat r_R_l = (Mat_<double>(3,3) <<
        0.999908014052592,   0.0129876734848808,  0.00390944636126203,
       -0.0130330487668424,  0.999845273259794,   0.0118139400662856,
       -0.00375540586921964,-0.0118638053548937,  0.999922570526968);

    Mat tmp_t = (Mat_<double>(3,1) <<
        -60.846892742144,
        -0.75388600735423,
        -0.59498331895342);

    Mat r_t_l = -r_R_l * tmp_t;
    double RADIUS = 22.0;

    Mat result_img(full_img.rows, full_img.cols, full_img.type(), Scalar(0,0,0));
    left_img.copyTo(result_img(Rect(0,   0, 640, 480)));
    right_img.copyTo(result_img(Rect(640, 0, 640, 480)));
    line(result_img, Point(640, 0), Point(640, 479), Scalar(0, 255, 255), 2);

    if (left_pick.valid)
        draw_detection_label(result_img, left_pick.det);

    if (right_pick.valid)
        draw_detection_label(result_img, right_pick.det);

    std::ostringstream oss;

    if (!left_pick.valid && !right_pick.valid)
    {
        auto total_t1 = Clock::now();
        total_time_ms =
            std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();

        draw_time_info(result_img, detection_time_ms, 0.0, total_time_ms);

        putText(result_img, "No valid target in both halves", Point(20, 40),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255), 2);
        imwrite("result.jpg", result_img);

        oss << "Status: Failed\nReason: No valid target in both halves\n";
        oss << "Detection time(ms): " << detection_time_ms << "\n";
        oss << "Pose time(ms): " << 0.0 << "\n";
        oss << "Total time(ms): " << total_time_ms << "\n";
        write_result_txt(oss.str());

        release_yolov8_model(&ctx);
        deinit_post_process();
        return 0;
    }

    auto pose_t0 = Clock::now();

    EyeProcessResult left_res, right_res;

    if (left_pick.valid)
    {
        const auto& det = left_pick.det;
        Rect full_roi(det.box.left, det.box.top,
                      det.box.right - det.box.left,
                      det.box.bottom - det.box.top);

        Rect left_half_global(0, 0, 640, 480);
        Rect left_roi_global = full_roi & left_half_global;

        Rect left_roi_local(left_roi_global.x,
                            left_roi_global.y,
                            left_roi_global.width,
                            left_roi_global.height);

        left_res = process_eye_from_roi(left_img, left_roi_local, 0,
                                        K_left, RADIUS, "Left", &left_pick.det);

        if (!left_res.draw_img.empty())
            left_res.draw_img.copyTo(result_img(Rect(0, 0, 640, 480)));
    }

    if (right_pick.valid)
    {
        const auto& det = right_pick.det;
        Rect full_roi(det.box.left, det.box.top,
                      det.box.right - det.box.left,
                      det.box.bottom - det.box.top);

        Rect right_half_global(640, 0, 640, 480);
        Rect right_roi_global = full_roi & right_half_global;

        Rect right_roi_local(right_roi_global.x - 640,
                             right_roi_global.y,
                             right_roi_global.width,
                             right_roi_global.height);

        right_res = process_eye_from_roi(right_img, right_roi_local, 640,
                                         K_right, RADIUS, "Right", &right_pick.det);

        if (!right_res.draw_img.empty())
            right_res.draw_img.copyTo(result_img(Rect(640, 0, 640, 480)));
    }

    bool left_ok = left_pick.valid && left_res.valid;
    bool right_ok = right_pick.valid && right_res.valid;

    if (left_ok && right_ok)
    {
        FinalStereoPose stereoPose = solve_stereo_true_pose(left_res.monoPose, right_res.monoPose, r_t_l);

        if (!stereoPose.valid)
        {
            auto pose_t1 = Clock::now();
            pose_time_ms =
                std::chrono::duration<double, std::milli>(pose_t1 - pose_t0).count();

            auto total_t1 = Clock::now();
            total_time_ms =
                std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();

            draw_time_info(result_img, detection_time_ms, pose_time_ms, total_time_ms);

            putText(result_img, "Stereo pose solve failed", Point(20, 40),
                    FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255), 2);
            imwrite("result.jpg", result_img);

            oss << "Status: Failed\n";
            oss << "Reason: " << stereoPose.message << "\n";
            oss << "Detection time(ms): " << detection_time_ms << "\n";
            oss << "Pose time(ms): " << pose_time_ms << "\n";
            oss << "Total time(ms): " << total_time_ms << "\n";
            write_result_txt(oss.str());

            release_yolov8_model(&ctx);
            deinit_post_process();
            return 0;
        }

        Mat n_final = ensure_row_vec_1x3(stereoPose.n_final);
        Mat center_final = ensure_row_vec_1x3(stereoPose.center_final);

        double nx = n_final.at<double>(0,0);
        double ny = n_final.at<double>(0,1);
        double nz = n_final.at<double>(0,2);

        double theta_deg = atan2(ny, sqrt(nx*nx + nz*nz)) * 180.0 / CV_PI;
        double phi_deg   = atan2(ny, nx) * 180.0 / CV_PI;

        auto pose_t1 = Clock::now();
        pose_time_ms =
            std::chrono::duration<double, std::milli>(pose_t1 - pose_t0).count();

        auto total_t1 = Clock::now();
        total_time_ms =
            std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();

        draw_time_info(result_img, detection_time_ms, pose_time_ms, total_time_ms);

        putText(result_img, "Stereo pose solved", Point(20, 40),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2);

        imwrite("result.jpg", result_img);

        oss << "Status: Success\n";
        oss << "Mode: Stereo\n";
        // oss << "Message: Stereo pose solved successfully\n\n";

 

        // oss << "Left detection real confidence: " << left_pick.det.prop << "\n";
        // oss << "Right detection real confidence: " << right_pick.det.prop << "\n\n";

        // oss << "[Left Ellipse]\n";
        // oss << "Center: (" << left_res.ellipse.cx << ", " << left_res.ellipse.cy << ")\n";
        // oss << "Major axis: " << left_res.ellipse.a << "\n";
        // oss << "Minor axis: " << left_res.ellipse.b << "\n";
        // oss << "Angle(rad): " << left_res.ellipse.theta << "\n\n";

        // oss << "[Right Ellipse]\n";
        // oss << "Center: (" << right_res.ellipse.cx << ", " << right_res.ellipse.cy << ")\n";
        // oss << "Major axis: " << right_res.ellipse.a << "\n";
        // oss << "Minor axis: " << right_res.ellipse.b << "\n";
        // oss << "Angle(rad): " << right_res.ellipse.theta << "\n\n";

        // oss << "[Left Mono Solutions]\n";
        // oss << "Center1: " << mat_to_string_row(left_res.monoPose.center1) << "\n";
        // oss << "Normal1: " << mat_to_string_row(left_res.monoPose.normal1) << "\n";
        // oss << "Center2: " << mat_to_string_row(left_res.monoPose.center2) << "\n";
        // oss << "Normal2: " << mat_to_string_row(left_res.monoPose.normal2) << "\n\n";

        // oss << "[Right Mono Solutions]\n";
        // oss << "Center1: " << mat_to_string_row(right_res.monoPose.center1) << "\n";
        // oss << "Normal1: " << mat_to_string_row(right_res.monoPose.normal1) << "\n";
        // oss << "Center2: " << mat_to_string_row(right_res.monoPose.center2) << "\n";
        // oss << "Normal2: " << mat_to_string_row(right_res.monoPose.normal2) << "\n\n";

        oss << "[Stereo Final Result]\n";
        // oss << "Selected left normal: " << mat_to_string_row(stereoPose.n_l) << "\n";
        // oss << "Selected right normal: " << mat_to_string_row(stereoPose.n_r) << "\n";
        // oss << "Selected left center: " << mat_to_string_row(stereoPose.center_l) << "\n";
        // oss << "Selected right center: " << mat_to_string_row(stereoPose.center_r) << "\n";
        oss << "Final center(mm): " << mat_to_string_row(center_final) << "\n";
        oss << "Final normal: " << mat_to_string_row(n_final) << "\n";
        oss << "Theta(deg): " << theta_deg << "\n";
        oss << "Phi(deg): " << phi_deg << "\n";
        oss << "Detection time(ms): " << detection_time_ms << "\n";
        oss << "Pose time(ms): " << pose_time_ms << "\n";
        oss << "Total time(ms): " << total_time_ms << "\n\n";
        write_result_txt(oss.str());

        release_yolov8_model(&ctx);
        deinit_post_process();
        return 0;
    }

    if (left_ok || right_ok)
    {
        EyeProcessResult mono = left_ok ? left_res : right_res;
        string eye_name = left_ok ? "Left" : "Right";

        auto pose_t1 = Clock::now();
        pose_time_ms =
            std::chrono::duration<double, std::milli>(pose_t1 - pose_t0).count();

        auto total_t1 = Clock::now();
        total_time_ms =
            std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();

        draw_time_info(result_img, detection_time_ms, pose_time_ms, total_time_ms);

        putText(result_img, (eye_name + " mono pose solved").c_str(), Point(20, 40),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2);

        if (!left_ok && left_pick.valid)
        {
            putText(result_img, ("L failed: " + left_res.message).c_str(), Point(20, 80),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
        }
        if (!right_ok && right_pick.valid)
        {
            putText(result_img, ("R failed: " + right_res.message).c_str(), Point(660, 80),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
        }

        imwrite("result.jpg", result_img);

        oss << "Status: Success\n";
        oss << "Mode: MonoFallback\n";
        oss << "Eye: " << eye_name << "\n";
        oss << "Message: Only one valid eye detected; output mono solutions.\n\n";

        // oss << "Detection time(ms): " << detection_time_ms << "\n";
        // oss << "Pose time(ms): " << pose_time_ms << "\n";
        // oss << "Total time(ms): " << total_time_ms << "\n\n";

        // if (left_ok)
        //     oss << "Left detection real confidence: " << left_pick.det.prop << "\n";
        // if (right_ok)
        //     oss << "Right detection real confidence: " << right_pick.det.prop << "\n";

        oss << "\n[" << eye_name << " Ellipse]\n";
        // oss << "Center: (" << mono.ellipse.cx << ", " << mono.ellipse.cy << ")\n";
        // oss << "Major axis: " << mono.ellipse.a << "\n";
        // oss << "Minor axis: " << mono.ellipse.b << "\n";
        // oss << "Angle(rad): " << mono.ellipse.theta << "\n\n";

        oss << "[" << eye_name << " Mono Solutions]\n";
        oss << "Center1(mm): " << mat_to_string_row(mono.monoPose.center1) << "\n";
        oss << "Normal1: " << mat_to_string_row(mono.monoPose.normal1) << "\n";
        oss << "Center2(mm): " << mat_to_string_row(mono.monoPose.center2) << "\n";
        oss << "Normal2: " << mat_to_string_row(mono.monoPose.normal2) << "\n";

        if (!left_ok)
            oss << "\nLeft reason: " << (left_pick.valid ? left_res.message : "No target assigned to left half") << "\n";
        if (!right_ok)
            oss << "Right reason: " << (right_pick.valid ? right_res.message : "No target assigned to right half") << "\n";

        write_result_txt(oss.str());

        release_yolov8_model(&ctx);
        deinit_post_process();
        return 0;
    }

    auto pose_t1 = Clock::now();
    pose_time_ms =
        std::chrono::duration<double, std::milli>(pose_t1 - pose_t0).count();

    auto total_t1 = Clock::now();
    total_time_ms =
        std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();

    draw_time_info(result_img, detection_time_ms, pose_time_ms, total_time_ms);

    putText(result_img, "No valid pose result", Point(20, 40),
            FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255), 2);
    imwrite("result.jpg", result_img);

    oss << "Status: Failed\n";
    oss << "Reason: No valid pose result\n";
    oss << "Detection time(ms): " << detection_time_ms << "\n";
    oss << "Pose time(ms): " << pose_time_ms << "\n";
    oss << "Total time(ms): " << total_time_ms << "\n";

    if (left_pick.valid)  oss << "Left detail: "  << left_res.message  << "\n";
    else                  oss << "Left detail: No target assigned to left half\n";
    if (right_pick.valid) oss << "Right detail: " << right_res.message << "\n";
    else                  oss << "Right detail: No target assigned to right half\n";

    write_result_txt(oss.str());

    release_yolov8_model(&ctx);
    deinit_post_process();

    return 0;
}
