//
// Created by kawa on 10/6/18.
//
#ifndef CONVERTPCL_CONVERT_H
#define CONVERTPCL_CONVERT_H

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

class ConvertPCL{

private:

    //Buffer関係
    rs2::pipeline pipeline;
    rs2::pipeline_profile pipeline_profile;
    rs2::frameset frameset;
    rs2::frame color_frame;
    rs2::frame depth_frame;
    rs2::colorizer color_map;

    // ポイントクラウドとテクスチャマッピングを計算するためのポイントクラウドオブジェクトの宣言
    rs2::pointcloud pc;
    // ポイントオブジェクトを永続化して、フレームがドロップしたときに最後のクラウドを表示できるようにします
    rs2::points points;

    cv::Mat color_mat_;
    cv::Mat depth_mat_;
    int color_width_ = 640;
    int color_height_ = 480;
    int depth_width_ = 640;
    int depth_height_ = 480;
    int width_ = 640;
    int height_ = 480;
    int fps_ = 30;
    int count_=0;

    //colorとdepthのframe合わせ
    rs2::frameset aligned_frames;

public:

    ConvertPCL();
    ~ConvertPCL();
    void run();

private:

    inline void startSenser();
    void updateFrame();
    void draw();
    void show();
    void pointToPCL();
    void setAligned_frames();
    void setCloud();
    void writePCDfile();
    void writeAngleFile();

};

#endif //CONVERTPCL_CONVERT_H
