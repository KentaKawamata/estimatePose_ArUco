//
// Created by kawa on 9/30/18.
//
#include "convert.hpp"

ConvertPCL::ConvertPCL() : 
    camera_matrix (cv::Mat_<float>(3, 3)),
    dist_coeffs (cv::Mat_<float>(1, 5)),
    degree (180/3.1415926),
    count (0)
    {
    
    load = new loadParams();
    setupParams();
    startSenser();
    setupArUco();
}

ConvertPCL::~ConvertPCL(){
    pipeline.stop();
    delete load;
}

void ConvertPCL::startSenser(){

    // デバイスを見つける
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0) {
        std::cout << "No device detected. Is it plugged in?" << std::endl;
    }

    

    rs2::device dev = list.front();

    // Set Device Config
    rs2::config config;
    config.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width_, color_height_, rs2_format::RS2_FORMAT_BGR8, fps_);
    //config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width_, depth_height_, rs2_format::RS2_FORMAT_Z16, fps_);

    pipeline_profile = pipeline.start(config);
}

void ConvertPCL::setupParams() {

    load->loadCameraParams();

    for(int i; i<3; i++){
        for(int j; j<3; j++){
            camera_matrix.at<float>(i,j) = 0;
        }
    }

    camera_matrix.at<float>(0,0) = load->focal[0];
    camera_matrix.at<float>(1,1) = load->focal[1];
    camera_matrix.at<float>(0,3) = load->principal[0];
    camera_matrix.at<float>(1,3) = load->principal[1];

    for(int i; i<load->dist.size(); i++){

        dist_coeffs.at<float>(0,i) = load->dist[i];
    }
}

void ConvertPCL::setupArUco() {

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
    //cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
    
    grid_board = cv::aruco::GridBoard::create(markers_x, markers_y, marker_length, marker_separation,dictionary);
    board = grid_board.staticCast<cv::aruco::Board>();
}

void ConvertPCL::updateFrame() {
    // センサーからframeを得る
    frameset = pipeline.wait_for_frames();

    color_frame = frameset.get_color_frame();

    // Retrive Frame Size
    color_width_ = color_frame.as<rs2::video_frame>().get_width();
    color_height_ = color_frame.as<rs2::video_frame>().get_height();
}

void ConvertPCL::detectArUco() {

    cv::aruco::detectMarkers(color_mat_, dictionary, corners, ids);
    //cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);
            
    if(ids.size() > 0) {


        cv::aruco::drawDetectedMarkers(color_mat_, corners, ids);

        cv::aruco::estimatePoseBoard(corners,ids,board,camera_matrix,dist_coeffs,rvecs,tvecs);
        cv::aruco::drawAxis(color_mat_, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1);
    }
}

void ConvertPCL::draw() {

    color_mat_ = cv::Mat(color_height_, color_width_, CV_8UC3, const_cast<void *>(color_frame.get_data()));
}

void ConvertPCL::show() {

    cv::imshow("color_test", color_mat_);
}

void ConvertPCL::writeImage() {

    cv::imwrite("./image_name.jpg", color_mat_);
}

void ConvertPCL::writeRotation() {

    cv::Mat R;
    cv::Rodrigues(rvecs, R);

    R = R.t();
    cv::Rodrigues(R, rvec_aruco_to_cam);

    FILE *fp;
    std::cout << " x: " << rvec_aruco_to_cam[0]*degree 
              << " y: " << rvec_aruco_to_cam[1]*degree 
              << " z: " << rvec_aruco_to_cam[2]*degree << std::endl;
    
    if( (fp=fopen("save.csv","a")) != NULL){
        
        fprintf(fp,"step %d,\n",count);
        fprintf(fp,"%f,%f,%f\n",rvecs[0],rvecs[1],rvecs[2]);
        fclose(fp);
    }
}

void ConvertPCL::run(){

    while(true){
        updateFrame();
        draw();
        detectArUco();
        show();

        const int key = cv::waitKey(20);

        if(key=='s'){
            writeImage();
            writeRotation();
            count++;
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
