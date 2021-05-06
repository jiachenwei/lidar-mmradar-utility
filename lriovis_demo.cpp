#include <iostream>
#include "lriovis.hpp"

using namespace cv;
using namespace std;

int main(){
    Mat src;
    const int pixels_per_meter = 18;
    Point2i pixel_drawing_origin = initialize_display_canvas(src, 24, 55, Point2d(12, 3), pixels_per_meter);

    draw_one_lidar_objects_bounding_box(&src,
                                        Point3d(6, 32, 4),
                                        Point3d(1.9, 4.9, 2),
                                        Scalar(0.99, 0.10, 0.13, 0),
                                        pixel_drawing_origin,
                                        pixels_per_meter,
                                        0,
                                        5,
                                        Scalar(0, 0, 1));

    draw_one_lidar_objects_bounding_box(&src,
                                        Point3d(2, 32, 3),
                                        Point3d(1.9, 4.9, 2),
                                        Scalar(0.99, 0.10, 0.13, 0),
                                        pixel_drawing_origin,
                                        pixels_per_meter,
                                        0,
                                        5,
                                        Scalar(0, 0, 1));
    

    draw_one_lidar_objects_bounding_box(&src,
                                        Point3d(-2, 32, 2),
                                        Point3d(1.9, 4.9, 2),
                                        Scalar(0.99, 0.10, 0.13, 0),
                                        pixel_drawing_origin,
                                        pixels_per_meter,
                                        0,
                                        5,
                                        Scalar(0, 0, 1));
    
    draw_one_lidar_objects_bounding_box(&src,
                                        Point3d(-6, 32, 1),
                                        Point3d(1.9, 4.9, 2),
                                        Scalar(0.99, 0.10, 0.13, 0),
                                        pixel_drawing_origin,
                                        pixels_per_meter,
                                        0,
                                        5,
                                        Scalar(0, 0, 1));
    
    
    draw_one_lidar_objects_bounding_box(&src,
                                        Point3d(-3, 44, 1),
                                        Point3d(1, 1, 1.7),
                                        Scalar(0.99, -0.13, -0.09, 0),
                                        pixel_drawing_origin,
                                        pixels_per_meter,
                                        0,
                                        5,
                                        Scalar(1, 0, 0));
    
    draw_one_lidar_objects_bounding_box(&src,
                                        Point3d(2.8, 22, 0.9),
                                        Point3d(1, 1, 1.7),
                                        Scalar(0.99, -0.13, -0.09, 0),
                                        pixel_drawing_origin,
                                        pixels_per_meter,
                                        0,
                                        5,
                                        Scalar(1, 0, 0));

    Scalar robj_color(1, 0.5, 1);

    draw_one_mmradar_object(&src, Point2d(-6, 16), Point2d(2, 2), robj_color/7, pixel_drawing_origin, pixels_per_meter);
    draw_one_mmradar_object(&src, Point2d(-4, 16), Point2d(4, 4), robj_color/7*2, pixel_drawing_origin, pixels_per_meter);
    draw_one_mmradar_object(&src, Point2d(-2, 16), Point2d(6, 6), robj_color/7*3, pixel_drawing_origin, pixels_per_meter);
    draw_one_mmradar_object(&src, Point2d(0, 16), Point2d(8, 8), robj_color/7*4, pixel_drawing_origin, pixels_per_meter);
    draw_one_mmradar_object(&src, Point2d(2, 16), Point2d(10, 10), robj_color/7*5, pixel_drawing_origin, pixels_per_meter);
    draw_one_mmradar_object(&src, Point2d(4, 16), Point2d(12, 12), robj_color/7*6, pixel_drawing_origin, pixels_per_meter);
    draw_one_mmradar_object(&src, Point2d(6, 16), Point2d(14, 14), robj_color, pixel_drawing_origin, pixels_per_meter);

    draw_one_mmradar_object(&src, Point2d(-6, 32), Point2d(-20, 2), robj_color, pixel_drawing_origin, pixels_per_meter);
    draw_one_mmradar_object(&src, Point2d(-8, 44), Point2d(0, 0), robj_color, pixel_drawing_origin, pixels_per_meter);

    Point2i p1 = draw_one_mmradar_object(&src, Point2d(3, 22), Point2d(0, 0), robj_color, pixel_drawing_origin, pixels_per_meter);
    draw_text_block(&src, 
                    vector<string>({"mm_radar_obj", "x=3, y=22", "vx=0, vy=0", "prob=7"}), 
                    p1, 
                    0.5, 
                    3, 
                    Scalar(1, 1, 1), 
                    Scalar(0.35, 0.35, 0.35));

    Point2i p2 = draw_one_mmradar_object(&src, Point2d(-2, 43), Point2d(10, -10), robj_color/7*5, pixel_drawing_origin, pixels_per_meter);
    draw_text_block(&src, 
                    vector<string>({"mm_radar_obj", "x=-2, y=43", "vx=10, vy=-10", "prob=5"}), 
                    p2, 
                    0.5, 
                    3, 
                    Scalar(1, 1, 1), 
                    Scalar(0.35, 0.35, 0.35), 
                    Point2i(5, 5),
                    0.3,
                    3,
                    true);

    namedWindow("LRIOVIS Demo", CV_WINDOW_AUTOSIZE);
    imshow("LRIOVIS Demo", src);
    waitKey(0);

    destroyWindow("LRIOVIS Demo");

    return 0;
}


// g++ -o test test.cpp `pkg-config opencv --cflags --libs`