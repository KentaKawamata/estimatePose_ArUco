//
// Created by kawa on 9/30/18.
//
#include "convert.hpp"

ConvertPCL::ConvertPCL() : 
    camera_matrix (cv::Mat_<double>(3, 3)),
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
    //config.enable_stream(RS2_STREAM_ACCEL);
    config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width_, depth_height_, rs2_format::RS2_FORMAT_Z16, fps_);

    pipeline_profile = pipeline.start(config);
}

void ConvertPCL::setupParams() {

    load->loadCameraParams();

    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            camera_matrix.at<float>(i,j) = 0.00;
        }
    }

    load->focal[0] = load->focal[0] * color_width_ / calib_width;
    load->focal[1] = load->focal[1] * color_height_ / calib_height;
    load->principal[0] = load->principal[0] * color_width_ / calib_width;
    load->principal[1] = load->principal[1] * color_height_ / calib_height;

    camera_matrix.at<double>(0,0) = load->focal[0];
    camera_matrix.at<double>(1,1) = load->focal[1];
    camera_matrix.at<double>(0,2) = load->principal[0];
    camera_matrix.at<double>(1,2) = load->principal[1];
    camera_matrix.at<double>(2,2) = 1.0;


    for(int i=0; i<3; i++){
        std::cout << camera_matrix.at<double>(i,0) << " " 
                  << camera_matrix.at<double>(i,1) << " " 
                  << camera_matrix.at<double>(i,2) << std::endl;
    }



    for(int i=0; i<load->dist.size(); i++){

        dist_coeffs.at<float>(0,i) = load->dist[i];
    }
}

void ConvertPCL::setupArUco() {

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
    //cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
    
    grid_board = cv::aruco::GridBoard::create(markers_x, markers_y, marker_length, marker_separation, dictionary);
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
            
    if(ids.size() > 0) {

        cv::aruco::drawDetectedMarkers(color_mat_, corners, ids);
        //cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);

        cv::aruco::estimatePoseBoard(corners, ids, grid_board, camera_matrix, dist_coeffs, rvecs, tvecs);
        cv::aruco::drawAxis(color_mat_, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1);
 
        /** 
        for (int i=0; i<ids.size(); i++) {
            cv::aruco::drawAxis(color_mat_, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
        }
        **/

       setAligned_frames();
       rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
       distance = depth_frame.get_distance(corners[0][0].x, corners[0][0].y );
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

    /**
    if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
    {
        rs2_vector gyro_sample = gyro_frame.get_motion_data();
        float theta_x = std::atan(gyro_sample.z/gyro_sample.y); 
        std::cout << " IMU_x_R: " << theta_x*degree+90 << std::endl;
    }
    **/

    cv::Mat R;
    cv::Rodrigues(rvecs, R);

    R = R.t();
    cv::Rodrigues(R, rvec_aruco_to_cam);

    FILE *fp;
    std::cout << " x_R: " << rvec_aruco_to_cam[0]*degree 
              << " y_R: " << rvec_aruco_to_cam[1]*degree 
              << " z_R: " << rvec_aruco_to_cam[2]*degree << std::endl;

    std::cout << " x_t: " << -tvecs[0] 
              << " y_t: " << -tvecs[1] 
              << " z_t: " << -tvecs[2] << std::endl;

    std::cout << " distance: " << distance << std::endl; 
    
    if( (fp=fopen("save.csv","a")) != NULL){
        
        fprintf(fp,"step %d,\n", count);
        fprintf(fp,"%f,%f,%f\n", rvec_aruco_to_cam[0], rvec_aruco_to_cam[1], rvec_aruco_to_cam[2]);
        fprintf(fp,"%f,%f,%f\n", -tvecs[0], -tvecs[1], -tvecs[2]);
        fclose(fp);
    }
}

void ConvertPCL::setAligned_frames() {

    // color_frameとdepth_frameの辻褄合わせ
    rs2::align align(rs2_stream::RS2_STREAM_COLOR);

    aligned_frames = align.process(frameset);
    if( !aligned_frames.size() ) {
        std::cout << "NO" << std::endl;
        exit(0);
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
