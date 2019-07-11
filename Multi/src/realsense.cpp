#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include "realsense.h"

// Constructor
RealSense::RealSense( const std::string serial_number, const std::string friendly_name ) :
      serial_number( serial_number ),
      friendly_name( friendly_name ),
      camera_matrix (cv::Mat_<float>(3, 3)),
      dist_coeffs (cv::Mat_<float>(1, 5)),
      degree (180/3.1415926),
      count (0)
{
    setupParams();
    initialize();
    setupArUco();
}

// Destructor
RealSense::~RealSense()
{
    // Finalize
    finalize();
}

// Initialize
void RealSense::initialize()
{
    cv::setUseOptimized( true );

    initializeSensor();
}

void RealSense::loadCameraParams(std::string serial_number){

    std::string filename = "./src/" + serial_number + ".xml";

    boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml(filename, pt);

    BOOST_FOREACH (const boost::property_tree::ptree::value_type& child, 
                                        pt.get_child("Config.FocalLengthRGB")) {

        focal.push_back( boost::lexical_cast<double>(child.second.data()) );
    }
 
    BOOST_FOREACH (const boost::property_tree::ptree::value_type& child, 
                                        pt.get_child("Config.PrincipalPointRGB")) {

        principal.push_back( boost::lexical_cast<double>(child.second.data()) );
    }
  
    BOOST_FOREACH (const boost::property_tree::ptree::value_type& child, 
                                        pt.get_child("Config.DistortionRGB")) {

        dist.push_back( boost::lexical_cast<double>(child.second.data()) );
    }
    
}

// Initialize Sensor
inline void RealSense::initializeSensor()
{
    // Set Device Config
    rs2::config config;
    config.enable_device( serial_number );
    config.enable_stream( rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps );

    // Start Pipeline
    pipeline_profile = pipeline.start( config );
}

void RealSense::setupParams() {

    loadCameraParams(serial_number);

    for(int i; i<3; i++){
        for(int j; j<3; j++){
            camera_matrix.at<float>(i,j) = 0;
        }
    }
            
    camera_matrix.at<float>(0,0) = focal[0];
    camera_matrix.at<float>(1,1) = focal[1];
    camera_matrix.at<float>(0,3) = principal[0];
    camera_matrix.at<float>(1,3) = principal[1];
    
    for(int i; i<dist.size(); i++){
    
        dist_coeffs.at<float>(0,i) = dist[i];
    }
}

void RealSense::setupArUco() {

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
    //cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
    
    grid_board = cv::aruco::GridBoard::create(markers_x, markers_y, marker_length, marker_separation,dictionary);
    board = grid_board.staticCast<cv::aruco::Board>();
}

void RealSense::detectArUco() {

    cv::aruco::detectMarkers(color_mat, dictionary, corners, ids);
    //cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);
            
    if(ids.size() > 0) {


        cv::aruco::drawDetectedMarkers(color_mat, corners, ids);

        cv::aruco::estimatePoseBoard(corners,ids,board,camera_matrix,dist_coeffs,rvecs,tvecs);
        cv::aruco::drawAxis(color_mat, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1);
    }
}

// Finalize
void RealSense::finalize()
{
    // Close Windows
    cv::destroyAllWindows();

    // Stop Pipline
    pipeline.stop();
}

// Update Data
void RealSense::update()
{
    // Update Frame
    updateFrame();

    // Update Color
    updateColor();
}

// Update Frame
inline void RealSense::updateFrame()
{
    // Update Frame
    frameset = pipeline.wait_for_frames();
}

// Update Color
inline void RealSense::updateColor()
{
    // Retrieve Color Flame
    color_frame = frameset.get_color_frame();

    // Retrive Frame Size
    color_width = color_frame.as<rs2::video_frame>().get_width();
    color_height = color_frame.as<rs2::video_frame>().get_height();
}


// Draw Data
void RealSense::draw()
{
    // Draw Color
    drawColor();
}

// Draw Color
inline void RealSense::drawColor()
{
    // Create cv::Mat form Color Frame
    color_mat = cv::Mat( color_height, color_width, CV_8UC3, const_cast<void*>( color_frame.get_data() ) );
}

// Show Data
void RealSense::show()
{
    // Show Color
    showColor();
}

// Show Color
inline void RealSense::showColor()
{
    if( color_mat.empty() ){
        return;
    }

    // Show Color Image
    cv::imshow( "Color - " + friendly_name + " (" + serial_number + ")", color_mat );
}

void RealSense::writeImage() {

    cv::imwrite("color_" + serial_number, color_mat);
}

void RealSense::writeRotation() {

    if (ids.size() == 0){
        std::cout << "Could not detect markers" << std::endl;

    } else {
        FILE *fp;
        std::cout <<"x: " << rvecs[0]*degree << " y: " << rvecs[1]*degree << " z: "<< rvecs[2]*degree <<std::endl;
    
        std::string tmp(serial_number + ".csv");
        const char* filename = tmp.c_str();
        if( (fp=fopen(filename, "a")) != NULL){
        
            fprintf(fp,"step %d,\n",count);
            fprintf(fp,"%f,%f,%f\n",rvecs[0],rvecs[1],rvecs[2]);
            fclose(fp);
        }
    }
}
