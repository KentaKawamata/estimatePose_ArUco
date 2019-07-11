#ifndef __REALSENSE__H
#define __REALSENSE__H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>

class RealSense
{
private:
    // RealSense
    rs2::pipeline pipeline;
    rs2::pipeline_profile pipeline_profile;
    rs2::frameset frameset;
    std::string serial_number;
    std::string friendly_name;

    // Color Buffer
    rs2::frame color_frame;
    cv::Mat color_mat;
    uint32_t color_width = 640;
    uint32_t color_height = 480;
    uint32_t color_fps = 30;

    float degree;
    int count;

    /** loadparams ************************************/
    std::vector<double> focal;
    std::vector<double> principal;
    std::vector<double> dist;

    void loadCameraParams(std::string serial_number);

    /** ArUco ****************************************************************** **/
    const int markers_x           = 3;      // x方向に配置するマーカー数
    const int markers_y           = 2;      // y方向に配置するマーカー数
    const float marker_length     = 0.2;    // マーカーのサイズ
    const float marker_separation = 0.07;   // マーカー間の間隔

    std::vector<int*> ids;
    cv::Vec3d rvecs, tvecs;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    std::ostringstream vector_to_marker;
    float actual_marker_length = 0.15;
    //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Ptr<cv::aruco::GridBoard> grid_board;
    cv::Ptr<cv::aruco::Board> board;
    /** ************************************************************************* **/

public:
    // Constructor
    RealSense( const std::string serial_number, const std::string friendly_name = "" );

    // Destructor
    ~RealSense();

    // Update Data
    void update();

    // Draw Data
    void draw();

    // Show Data
    void show();

    void writeImage();
    void writeRotation();
    void detectArUco();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    inline void initializeSensor();

    // Finalize
    void finalize();

    // Update Frame
    inline void updateFrame();

    // Update Color
    inline void updateColor();

    // Draw Color
    inline void drawColor();

    // Show Color
    inline void showColor();

    void setupParams();
    void setupArUco();
};

#endif // __REALSENSE__H
