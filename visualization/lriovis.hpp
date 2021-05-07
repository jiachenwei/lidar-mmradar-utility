#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Point2i initialize_display_canvas(
    cv::Mat& src, const double& real_width = 24, const double& real_length = 52,
    const cv::Point2i& real_origin = cv::Point2f(12, 0),
    const int& pixels_per_meter = 18);

cv::Point2i draw_one_mmradar_object(
    cv::Mat* const src, const cv::Point2d& real_obj_pos,
    const cv::Point2d& real_speed, const cv::Scalar& obj_color,
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