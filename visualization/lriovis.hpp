/**
 * @file lriovis.hpp
 * @author Chenwei Jia (cwjia98@gmail.com)
 * @brief lrio的可视化模块头文件
 * @version 0.1
 * @date 2021-05-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief 初始化画布
 * @param src 引用类型用于存储画布
 * @param real_x x轴向的画布总长度起点
 * @param real_y y轴向的画布总长度
 * @param real_origin 画布的起点
 * @param pixels_per_meter 每米几个像素
 * @return 画布绘制中心的像素坐标
 *
 */
cv::Point2i initialize_display_canvas(cv::Mat& src, const double& real_x,
                                      const double& real_y,
                                      const double& real_origin_x,
                                      const double& real_origin_y,
                                      const int& pixels_per_meter);

/**
 * @brief 绘制一个毫米波雷达目标
 * @param src 绘制画布的引用
 * @param real_pos 目标的真实位置(x, y)，单位m
 * @param real_speed 目标的真实速度(x, y)，单位m/s
 * @param obj_color 目标的绘制颜色
 * @param pixel_drawing_origin 绘制的起点
 * @param pixels_per_meter 每米几个像素
 * @param predict_sec 预测时间
 * @param zoom 毫米波雷达目标的缩放
 * @return 毫米波雷达在画布中的像素坐标
 *
 */
cv::Point2i draw_one_mmradar_object(
    cv::Mat* const src, const cv::Point2d real_pos,
    const cv::Point2d real_speed, const cv::Scalar& obj_color,
    const cv::Point2i& pixel_drawing_origin, const int& pixels_per_meter,
    const double& predict_sec = 0.05, const double& zoom = 0.45);

cv::Point2i draw_one_lidar_objects_bounding_box(
    cv::Mat* const src, const cv::Point3d& real_obj_center_pos,
    const cv::Point3d& real_obj_size, const cv::Scalar& quat,
    const cv::Point2i& pixel_drawing_origin, const int pixels_per_meter,
    const double min_real_z = 0, const double max_real_z = 5,
    const cv::Scalar& color = cv::Scalar(0, 0, 1));

void draw_one_frame_of_lidar_point_cloud(
    cv::Mat* const src, const std::vector<cv::Point3d>& real_obj_pos_3d,
    const double& max_real_z, const int& pixels_per_meter,
    const cv::Point2i& pixel_drawing_origin,
    const cv::Scalar& point_color = cv::Scalar(1, 1, 1));

void draw_text_block(cv::Mat* const img, const std::vector<std::string>& text,
                     const cv::Point2i& pixel_block_pos,
                     const double& font_size = 0.5,
                     const int& pixel_line_spacing = 3,
                     const cv::Scalar& font_color = cv::Scalar(1, 1, 1),
                     const cv::Scalar& bg_color = cv::Scalar(0.35, 0.35, 0.35),
                     const cv::Point2i& pixel_offset = cv::Point2i(5, 5),
                     const float& transparent = 0.3,
                     const int& pixel_padding = 3,
                     const bool& right_bottom = false);