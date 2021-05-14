/**
 * @file lriovis.cpp
 * @brief
 * @author Chenwei Jia (cwjia98@gmail.com)
 * @version 1.0
 * @date 2021-05-14
 */

#include "lriovis.hpp"

using namespace cv;

/**
 * @brief 初始化画布
 * @param src              引用类型用于存储画布
 * @param real_x           x轴向的画布总长度起点
 * @param real_y           y轴向的画布总长度
 * @param real_origin      画布的起点
 * @param pixels_per_meter 每米几个像素
 * @return cv::Point2i 画布绘制中心的像素坐标
 *
 */
Point2i initialize_display_canvas(Mat& src, const double& real_x,
                                  const double& real_y,
                                  const double& real_origin_x,
                                  const double& real_origin_y,
                                  const int& pixels_per_meter) {
    const Point2i _real_origin(real_origin_x, real_origin_y);
    // 画布宽度，单位：像素
    const int pixel_canvas_width = std::round(real_x * pixels_per_meter);
    // 画布高度，单位：像素
    const int pixel_canvas_height = std::round(real_y * pixels_per_meter);

    // 绘图中心点, wh, xy
    Point2i pixel_drawing_origin(
        std::round(_real_origin.x * pixels_per_meter),
        std::round(pixel_canvas_height - _real_origin.y * pixels_per_meter));

    // 创建初始画布
    src = Mat::zeros(Size(pixel_canvas_width, pixel_canvas_height), CV_32FC3);

    // 画布背景辅助线颜色
    Scalar bg_guide_line_color(0, 0.5, 0);

    // 绘制距离同心圆与数字
    int real_circle_interval_distance = 10;  // 真实同心圆间隔距离,单位：m
    int pixel_circle_interval_distance =
        std::round((double(real_circle_interval_distance) / double(real_y)) *
                   pixel_canvas_height);  // 画布中同心圆间隔距离，单位：像素

    int baseline = 0;
    int thickness = 1;
    int text_spacing = 7;

    circle(src, pixel_drawing_origin, (0.5 * pixel_circle_interval_distance),
           bg_guide_line_color);

    Point2i text_org =
        pixel_drawing_origin +
        Point2i(+text_spacing,
                -0.5 * pixel_circle_interval_distance - text_spacing);
    Size text_size =
        getTextSize("5m", FONT_HERSHEY_SIMPLEX, 0.7, thickness, &baseline);

    rectangle(src, text_org + Point2i(-1, -text_size.height - 1),
              text_org + Point2i(text_size.width - 5, 3), bg_guide_line_color,
              1);
    putText(src, "5m", text_org, FONT_HERSHEY_SIMPLEX, 0.6, bg_guide_line_color,
            thickness);

    for (int i = 1; i <= std::round((real_y - _real_origin.y) /
                                    real_circle_interval_distance);
         i++) {
        circle(src, pixel_drawing_origin, (i * pixel_circle_interval_distance),
               bg_guide_line_color);

        std::string str =
            std::to_string(i * real_circle_interval_distance) + "m";
        Point2i text_org =
            pixel_drawing_origin +
            Point2i(text_spacing,
                    -i * pixel_circle_interval_distance - text_spacing);
        Size text_size =
            getTextSize(str, FONT_HERSHEY_SIMPLEX, 0.6, thickness, &baseline);

        rectangle(src, text_org + Point2i(-1, -text_size.height - 2),
                  text_org + Point2i(text_size.width, 3), bg_guide_line_color);
        putText(src, str, text_org, FONT_HERSHEY_SIMPLEX, 0.6,
                bg_guide_line_color, thickness);
    }

    Point tmp_p1(pixel_drawing_origin.x, 0);
    Point tmp_p2(pixel_drawing_origin.x, pixel_canvas_height);

    line(src, tmp_p2, tmp_p1, bg_guide_line_color, 1);
    line(src, tmp_p2 + Point2i(-5 * pixels_per_meter, 0),
         tmp_p1 + Point2i(-5 * pixels_per_meter, 0), bg_guide_line_color, 1);
    line(src, tmp_p2 + Point2i(5 * pixels_per_meter, 0),
         tmp_p1 + Point2i(5 * pixels_per_meter, 0), bg_guide_line_color, 1);
    line(src, pixel_drawing_origin + Point2i(-pixel_canvas_width / 2, 0),
         pixel_drawing_origin + Point2i(pixel_canvas_width / 2, 0),
         bg_guide_line_color, 1);

    return pixel_drawing_origin;
}

/**
 * @brief 绘制一个毫米波雷达目标
 * @param src              绘制画布的指针
 * @param real_pos         目标的真实位置(x, y)，单位m
 * @param real_speed       目标的真实速度(x, y)，单位m/s
 * @param obj_color        目标的绘制颜色
 * @param pixel_drawing_origin 绘制的起点
 * @param pixels_per_meter 每米几个像素
 * @param predict_sec      预测时间
 * @param zoom             毫米波雷达目标的缩放
 * @return cv::Point2i 毫米波雷达在画布中的像素坐标
 *
 */
Point2i draw_one_mmradar_object(Mat* const src, const Point2d real_pos,
                                const Point2d real_speed,
                                const Scalar& obj_color,
                                const Point2i& pixel_drawing_origin,
                                const int& pixels_per_meter,
                                const double& predict_sec, const double& zoom) {
    Point2i pixel_obj_point(std::round(real_pos.x * pixels_per_meter),
                            -std::round(real_pos.y * pixels_per_meter));

    circle(*src, pixel_drawing_origin + pixel_obj_point,
           std::round(pixels_per_meter * zoom), obj_color, 1);
    circle(*src, pixel_drawing_origin + pixel_obj_point, 1, obj_color, -1);

    const double real_next_x = predict_sec * real_speed.x;
    const double real_next_y = predict_sec * real_speed.y;

    if (real_next_x != 0 || real_next_y != 0) {
        Point2i end_point(std::round(real_next_x * pixels_per_meter),
                          -std::round(real_next_y * pixels_per_meter));
        const double real_distance =
            sqrtf(powf(real_next_x, 2) + powf(real_next_y, 2));
        Point2i spacing(
            std::round(pixels_per_meter * real_next_x / real_distance),
            -std::round(pixels_per_meter * real_next_y / real_distance));
        line(
            *src, pixel_drawing_origin + pixel_obj_point + spacing * zoom,
            pixel_drawing_origin + pixel_obj_point + spacing * zoom + end_point,
            obj_color);
    }

    return pixel_drawing_origin + pixel_obj_point;
}

/**
 * @brief 四元数转旋转矩阵
 * @param  w                w
 * @param  x                x
 * @param  y                y
 * @param  z                z
 * @return Eigen::Matrix3d 旋转矩阵
 */
Eigen::Matrix3d quat_to_rot_mat(const double& w, const double& x,
                                const double& y, const double& z) {
    Eigen::Quaterniond q;
    q.w() = w;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    return q.normalized().toRotationMatrix();
}

/**
 * @brief 四元数转旋转矩阵
 * @param  quat             cv::Scalar 类封装的四元数{w,x,y,z}
 * @return Eigen::Matrix3d 旋转矩阵
 */
Eigen::Matrix3d quat_to_rot_mat(const Scalar& quat) {
    Eigen::Quaterniond q;
    q.w() = quat[0];
    q.x() = quat[1];
    q.y() = quat[2];
    q.z() = quat[3];
    return q.normalized().toRotationMatrix();
}

/**
 * @brief
 * 旋转矩阵叉乘坐标，如果目标绕远点旋转则目标不需要修正，如果目标绕的不是原点旋转，则目标的坐标需要减旋转点的坐标作出修正。
 * @param  rot_mat          旋转矩阵
 * @param  p 目标点的坐标。
 */
void rot_mat_mul_point(const Eigen::Matrix3d& rot_mat, Point3d& p) {
    Eigen::Vector3d pos(p.x, p.y, p.z);
    pos = rot_mat * pos;
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
}

/**
 * @brief 旋转矩阵叉乘坐标
 * @param  rot_mat          旋转矩阵
 * @param  p                目标点
 * @param  o                目标点所绕的旋转点
 */
void rot_mat_mul_poin_2(const Eigen::Matrix3d& rot_mat, Point3d& p,
                        const Point3d& o) {
    Eigen::Vector3d pos((p.x - o.x), (p.y - o.y), (p.z - o.z));
    pos = rot_mat * pos;
    p.x = pos.x() + o.x;
    p.y = pos.y() + o.y;
    p.z = pos.z() + o.z;
}

/**
 * @brief 绘制一个激光雷达包围框
 * @param  src              绘制画布的指针
 * @param  real_obj_center_pos 包围框中心的真实坐标
 * @param  real_obj_size    包围框的真实尺寸
 * @param  quat             包围框的旋转四元数
 * @param  pixel_drawing_origin 绘制画布的像素中心
 * @param  pixels_per_meter 每米几个像素
 * @param  min_real_z       需要显示的最小高度
 * @param  max_real_z       需要显示的最大高度
 * @param  color            包围框的颜色
 * @return cv::Point2i 激光雷达包围框的像素坐标
 */
Point2i draw_one_lidar_objects_bounding_box(
    Mat* const src, const Point3d& real_obj_center_pos,
    const Point3d& real_obj_size, const Scalar& quat,
    const Point2i& pixel_drawing_origin, const int pixels_per_meter,
    const double min_real_z, const double max_real_z, const Scalar& color) {
    Scalar obj_color = color;
    if (real_obj_center_pos.z <= min_real_z)
        obj_color *= 0.5;
    else if (real_obj_center_pos.z < max_real_z)
        obj_color *= (0.4 + 0.6 * (real_obj_center_pos.z - min_real_z) /
                                (max_real_z - min_real_z));

    Point3i pixel_obj_center_pos_3d(
        (real_obj_center_pos.x * pixels_per_meter) + pixel_drawing_origin.x,
        -(real_obj_center_pos.y * pixels_per_meter) + pixel_drawing_origin.y,
        (real_obj_center_pos.z * pixels_per_meter));

    Point3d real_left__front_top(-real_obj_size.x / 2, +real_obj_size.y / 2,
                                 +real_obj_size.z / 2);
    Point3d real_left__rear__top(-real_obj_size.x / 2, -real_obj_size.y / 2,
                                 +real_obj_size.z / 2);
    Point3d real_right_front_top(+real_obj_size.x / 2, +real_obj_size.y / 2,
                                 +real_obj_size.z / 2);
    Point3d real_right_rear__top(+real_obj_size.x / 2, -real_obj_size.y / 2,
                                 +real_obj_size.z / 2);
    Point3d real_left__front_btm(-real_obj_size.x / 2, +real_obj_size.y / 2,
                                 -real_obj_size.z / 2);
    Point3d real_left__rear__btm(-real_obj_size.x / 2, -real_obj_size.y / 2,
                                 -real_obj_size.z / 2);
    Point3d real_right_front_btm(+real_obj_size.x / 2, +real_obj_size.y / 2,
                                 -real_obj_size.z / 2);
    Point3d real_right_rear__btm(+real_obj_size.x / 2, -real_obj_size.y / 2,
                                 -real_obj_size.z / 2);

    Eigen::Matrix3d rot_mat = quat_to_rot_mat(quat);

    rot_mat_mul_point(rot_mat, real_left__front_top);
    rot_mat_mul_point(rot_mat, real_left__rear__top);
    rot_mat_mul_point(rot_mat, real_right_front_top);
    rot_mat_mul_point(rot_mat, real_right_rear__top);
    rot_mat_mul_point(rot_mat, real_left__front_btm);
    rot_mat_mul_point(rot_mat, real_left__rear__btm);
    rot_mat_mul_point(rot_mat, real_right_front_btm);
    rot_mat_mul_point(rot_mat, real_right_rear__btm);

    const double& ppm = pixels_per_meter;
    const Point3i& p_obj = pixel_obj_center_pos_3d;

    Point2i pixel_left__front_top_2d(p_obj.x + real_left__front_top.x * ppm,
                                     p_obj.y - real_left__front_top.y * ppm);
    Point2i pixel_left__rear__top_2d(p_obj.x + real_left__rear__top.x * ppm,
                                     p_obj.y - real_left__rear__top.y * ppm);
    Point2i pixel_right_front_top_2d(p_obj.x + real_right_front_top.x * ppm,
                                     p_obj.y - real_right_front_top.y * ppm);
    Point2i pixel_right_rear__top_2d(p_obj.x + real_right_rear__top.x * ppm,
                                     p_obj.y - real_right_rear__top.y * ppm);
    Point2i pixel_left__front_btm_2d(p_obj.x + real_left__front_btm.x * ppm,
                                     p_obj.y - real_left__front_btm.y * ppm);
    Point2i pixel_left__rear__btm_2d(p_obj.x + real_left__rear__btm.x * ppm,
                                     p_obj.y - real_left__rear__btm.y * ppm);
    Point2i pixel_right_front_btm_2d(p_obj.x + real_right_front_btm.x * ppm,
                                     p_obj.y - real_right_front_btm.y * ppm);
    Point2i pixel_right_rear__btm_2d(p_obj.x + real_right_rear__btm.x * ppm,
                                     p_obj.y - real_right_rear__btm.y * ppm);

    std::vector<Point2i> pts_btm(
        {pixel_left__front_btm_2d, pixel_left__rear__btm_2d,
         pixel_right_rear__btm_2d, pixel_right_front_btm_2d

        });

    std::vector<Point2i> pts_top(
        {pixel_left__front_top_2d, pixel_left__rear__top_2d,
         pixel_right_rear__top_2d, pixel_right_front_top_2d

        });

    Mat obj_mask = Mat::zeros(src->size(), src->type());

    fillPoly(obj_mask, pts_btm, 0.8 * obj_color);

    addWeighted(*src, 1, obj_mask, 0.5, 0, *src);

    polylines(*src, pts_btm, true, 0.8 * obj_color);

    line(*src, pixel_left__front_btm_2d, pixel_right_rear__btm_2d,
         0.8 * obj_color);
    line(*src, pixel_right_front_btm_2d, pixel_left__rear__btm_2d,
         0.8 * obj_color);

    //////////////////////////////////////////////////////////////

    line(*src, pixel_left__front_top_2d, pixel_left__front_btm_2d, obj_color);
    line(*src, pixel_left__rear__top_2d, pixel_left__rear__btm_2d, obj_color);
    line(*src, pixel_right_front_top_2d, pixel_right_front_btm_2d, obj_color);
    line(*src, pixel_right_rear__top_2d, pixel_right_rear__btm_2d, obj_color);

    ////////////////////////////////////////////////////////////////

    polylines(*src, pts_top, true, 1.3 * obj_color);

    Point2i pt1 = (pixel_left__front_top_2d + pixel_right_front_top_2d +
                   pixel_left__front_btm_2d + pixel_right_front_btm_2d) /
                  4;
    circle(*src, pt1, 3, color, -1);

    return Point2i(pixel_obj_center_pos_3d.x, pixel_obj_center_pos_3d.y);
}

/**
 * @brief 绘制一帧点云
 * @param  src              绘制画布的指针
 * @param  real_obj_pos_3d  真实坐标的点云
 * @param  max_real_z       最大显示的高度
 * @param  pixels_per_meter 每米几个像素
 * @param  pixel_drawing_origin 绘制画布的像素中心
 * @param  point_color      绘制点云的颜色
 */
void draw_one_frame_of_lidar_point_cloud(
    Mat* const src, const std::vector<Point3d>& real_obj_pos_3d,
    const double& max_real_z, const int& pixels_per_meter,
    const Point2i& pixel_drawing_origin, const Scalar& point_color) {
    for (int i = 0; i < real_obj_pos_3d.size(); i++) {
        Point2i pixel_obj_point(real_obj_pos_3d[i].x * pixels_per_meter,
                                -real_obj_pos_3d[i].y * pixels_per_meter);
        if (real_obj_pos_3d[i].z > max_real_z) {
            circle(*src, pixel_drawing_origin + pixel_obj_point, 1,
                   point_color);
        } else {
            circle(*src, pixel_drawing_origin + pixel_obj_point, 1,
                   0.5 * point_color * (real_obj_pos_3d[i].z / max_real_z + 1));
        }
    }
}

/**
 * @brief 图像透明相加
 * @param  dst              目标图像
 * @param  img              需要放置的图像
 * @param  pos              需要放置的位置
 * @param  transparent      img的透明度
 */
void add_transparent(cv::Mat& dst, cv::Mat& img, const Point2i& pos,
                     const double& transparent) {
    for (int i = 0; i < img.rows && i + pos.y < dst.rows; i++) {
        if (i + pos.y < 0) continue;
        for (int j = 0; j < img.cols && j + pos.x < dst.cols; j++) {
            if (j + pos.x < 0) continue;
            dst.at<Point3f>(pos.y + i, pos.x + j) =
                dst.at<Point3f>(pos.y + i, pos.x + j) * transparent +
                img.at<Point3f>(i, j) * (1 - transparent);
        }
    }
}

/**
 * @brief 绘制文本块
 * @param  src              绘制画布的指针
 * @param  text             需要绘制的文本，没一行需要单独加入vector中
 * @param  pixel_block_pos  绘制文本块的左上角位置
 * @param  font_size        字体大小
 * @param  pixel_line_spacing 行间距
 * @param  font_color       字体颜色
 * @param  bg_color         背景颜色
 * @param  pixel_offset     位置偏移
 * @param  transparent      文本块的透明度
 * @param  pixel_padding    文本的页边距
 * @param  right_bottom     如果置为true，则文本框位置变为右下角位置
 */
void draw_text_block(cv::Mat* const src, const std::vector<std::string>& texts,
                     const cv::Point2i& pixel_block_pos,
                     const double& font_scale, const int& pixel_line_spacing,
                     const cv::Scalar& font_color, const cv::Scalar& bg_color,
                     const cv::Point2i& pixel_offset, const float& transparent,
                     const int& pixel_padding, const bool& right_bottom) {
    /**
     * @brief
     *
     */
    Size2i total_size(0, 0);
    int baseline = 0;
    for (int i = 0; i < texts.size(); i++) {
        Size tmp = getTextSize(texts[i], CV_FONT_HERSHEY_SIMPLEX, font_scale, 1,
                               &baseline);
        total_size.height += (tmp.height + baseline);
        if (tmp.width > total_size.width) {
            total_size.width = tmp.width;
        }
    }

    total_size.height += pixel_line_spacing * (texts.size() - 1);

    total_size.width += pixel_padding * 2;
    total_size.height += pixel_padding * 2;

    Mat text_block = Mat::zeros(total_size, src->type());
    rectangle(text_block, Point2i(0, 0),
              Point2i(total_size.width, total_size.height), bg_color, -1);

    Point2i pixel_text_drawing_start(pixel_padding, pixel_padding);

    for (int i = 0; i < texts.size(); i++) {
        pixel_text_drawing_start.y +=
            getTextSize(texts[i], CV_FONT_HERSHEY_SIMPLEX, font_scale, 1,
                        &baseline)
                .height;
        putText(text_block, texts[i], pixel_text_drawing_start,
                CV_FONT_HERSHEY_SIMPLEX, font_scale, font_color);
        pixel_text_drawing_start.y += (pixel_line_spacing + baseline);
    }

    if (right_bottom == false) {
        add_transparent(*src, text_block, pixel_offset + pixel_block_pos,
                        transparent);
        circle(*src, pixel_block_pos + pixel_offset, 3, font_color, -1);
    }

    if (right_bottom == true) {
        add_transparent(*src, text_block,
                        pixel_block_pos - pixel_offset -
                            Point2i(total_size.width, total_size.height),
                        transparent);
        circle(*src, pixel_block_pos - pixel_offset, 3, font_color, -1);
    }
}