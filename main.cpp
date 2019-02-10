#include <librealsense2/rs.hpp>
#include <algorithm>            // std::min, std::max
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <chrono>

#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>



#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

typedef pcl::PointXYZRGB P_pcl;
typedef pcl::PointCloud<P_pcl> point_cloud;
typedef point_cloud::Ptr ptr_cloud;
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);


void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

inline void pass_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0,9.0);
    pass.filter(*cloud);
}

inline void planeDetect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double threshold) {
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);//モデル
    seg.setMethodType(pcl::SAC_RANSAC);//検出手法
    seg.setDistanceThreshold(threshold); //閾値 0.5とか
    seg.segment(*inliers, *coefficients);
}

inline void planeRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative) {
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(negative);
    extract.filter(*cloud);
}




// Get RGB values based on normals - texcoords, normals value [u v]
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
    
    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}


auto points_to_pcl(const rs2::points& points, const rs2::video_frame& color){

    // OpenCV Mat for showing the rgb color image, just as part of processing
    cv::Mat colorr(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    //namedWindow("Display Image", WINDOW_AUTOSIZE );
    //imshow("Display Image", colorr);
        
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    ptr_cloud cloud(new point_cloud);
    
    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();

    // Iterating through all points and setting XYZRGB coordinates
    // and RGB values
    for (int i = 0; i < points.size(); ++i)
    {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;
	    
        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color, tex_coords[i]);

        // Reversed order- 2-1-0 because of BGR model used in camera
        cloud->points[i].r = std::get<2>(current_color);
        cloud->points[i].g = std::get<1>(current_color);
        cloud->points[i].b = std::get<0>(current_color);
        
    }
    
   return cloud;
}


int main() {
    FILE *fp;
    int count= 0;
    const int markers_x         = 3;   // x方向に配置するマーカー数
    const int markers_y         = 2;   // y方向に配置するマーカー数
    const float marker_length     = 0.2; // マーカーのサイズ
    const float marker_separation = 0.07;  // マーカー間の間隔

    float degree = 180/3.141592;
    int wide = 1280;
    int height = 720;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    pcl::io::loadPLYFile("model.ply", *model);
    //aruco
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;
    float actual_marker_length = 0.15;
    //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
    cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    cv::Ptr<cv::aruco::GridBoard> grid_board = cv::aruco::GridBoard::create(markers_x, markers_y, marker_length, marker_separation,dictionary);
    cv::Ptr<cv::aruco::Board> board = grid_board.staticCast<cv::aruco::Board>();
    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;


    rs2::pointcloud pc;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, wide, height, RS2_FORMAT_BGR8, 30);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, wide, height, RS2_FORMAT_Z16, 30);
   
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    std::chrono::system_clock::time_point start, end;
    while(1){

        start = std::chrono::system_clock::now();
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    auto colored_frame = frames.get_color_frame();


    // Order here is crucial! 
    // map_to() color frame has to be done befor point calculation
    // otherwise texture won't be mapped

    //pc.map_to(colored_frame);
    auto points = pc.calculate(depth);
    // Actual calling of conversion and saving XYZRGBRGB cloud to file

    ptr_cloud cloud = points_to_pcl(points, colored_frame);
    std::cout << cloud->empty() << std::endl;

    if(cloud->size() != false) {

        cv::Mat image(cv::Size(wide, height), CV_8UC3, (void*)colored_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            //std::vector<cv::Vec3d> rvecs, tvecs;
            cv::Vec3d rvecs, tvecs;
            //cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);
            // draw axis for each marker
            for(int i=0; i < ids.size(); i++)
            {
                cv::aruco::estimatePoseBoard(corners,ids,board,camera_matrix,dist_coeffs,rvecs,tvecs);
                cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1);
                //cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
               // std::cout <<"x: " << rvecs[i][0]*degree << " y: " << rvecs[i][1]*degree << " z: "<< rvecs[i][2]*degree <<std::endl;
                 std::cout <<"x: " << rvecs[0]*degree << " y: " << rvecs[1]*degree << " z: "<< rvecs[2]*degree <<std::endl;
                if((fp=fopen("save.csv","a"))!=NULL){
                    fprintf(fp,"step %d,\n",count);
                    fprintf(fp,"%f,%f,%f\n",rvecs[0],rvecs[1],rvecs[2]);
                    fclose(fp);
                }
    }


    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();

    cv::waitKey(0);
    count++;
    }
  
    return 1;
}

