//
// Created by kawa on 9/30/18.
//
#include "convert.hpp"

ConvertPCL::ConvertPCL(){
    startSenser();
}

ConvertPCL::~ConvertPCL(){
    pipeline.stop();
}

void ConvertPCL::startSenser(){

    // デバイスを見つける
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        std::cout << "No device detected. Is it plugged in?" << std::endl;
    rs2::device dev = list.front();

    // Set Device Config
    rs2::config config;
    config.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width_, color_height_, rs2_format::RS2_FORMAT_BGR8, fps_);
    config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width_, depth_height_, rs2_format::RS2_FORMAT_Z16, fps_);

    pipeline_profile = pipeline.start(config);
}

void ConvertPCL::updateFrame() {
    // センサーからframeを得る
    frameset = pipeline.wait_for_frames();

    color_frame = frameset.get_color_frame();
    depth_frame = frameset.get_depth_frame().apply_filter(color_map);

    // Retrive Frame Size
    color_width_ = color_frame.as<rs2::video_frame>().get_width();
    color_height_ = color_frame.as<rs2::video_frame>().get_height();
    depth_width_ = depth_frame.as<rs2::video_frame>().get_width();
    depth_height_ = depth_frame.as<rs2::video_frame>().get_height();
}

void ConvertPCL::draw() {

    color_mat_ = cv::Mat(color_height_, color_width_, CV_8UC3, const_cast<void *>(color_frame.get_data()));
    depth_mat_ = cv::Mat(depth_height_, depth_width_, CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
}

void ConvertPCL::show() {

    cv::imshow("color_test", color_mat_);
    cv::imshow("depth_test", depth_mat_);
}

void ConvertPCL::writeAngleFile() {


    color_frame = aligned_frames.get_color_frame();
    depth_frame = aligned_frames.get_depth_frame();
    points = pc.calculate(depth_frame);
    auto colorArray = static_cast<uint8_t *>(const_cast<void *>(color_frame.get_data()));

}

void ConvertPCL::run(){

    while(true){
        updateFrame();
        draw();
        show();

        const int key = cv::waitKey(20);
        if(key=='s'){
            writeAngleFile();
            count_++;
        }
        else if(key==27){
            cv::destroyAllWindows();
            break;
        }
    }
}

int main(int argc, char *argv[]){

    ConvertPCL *convert = new ConvertPCL();
    convert->run();
    delete convert;
}