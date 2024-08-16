#ifndef RADAR_UTILS_H
#define RADAR_UTILS_H
#include <iostream>
#include <string>
#include "vector"
#include <opencv2/opencv.hpp>

namespace tdt_radar
{
    class Parser_Points
    {
    public:
        public:
        Parser_Points(const std::string &points_name);
        float return_height(cv::Point2f &input_point);
        void Update();
        void World2Camera();
        std::vector<cv::Point3f> ReadPoints(const std::string &points_name);
        std::vector<cv::Point> Float2Int(std::vector<cv::Point2f> &FloatPoint);
        std::vector<cv::Point3f> Points_3D;
        std::vector<cv::Point> Points_2D;
        float Height=0;
    private:
        cv::Mat world_rvec;
        cv::Mat world_tvec;
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
    };
    class parser
    {
    public:
        parser();
        void Change_Matrix();
        cv::Point2f parse(cv::Point2f &input_point);
        void draw_ui(cv::Mat &img);
        float get_height(cv::Point2f &input_point);
        cv::Point2f get_2d(cv::Point2f &input_point,float height);
        cv::Mat world_rvec;
        cv::Mat world_tvec;
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
        std::map <std::string,Parser_Points*> points_map;
    };
}
#endif //RADAR_UTILS_H
