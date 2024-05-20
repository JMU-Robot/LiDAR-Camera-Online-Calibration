#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include "ros/ros.h"
#include "livox_ros_driver/CustomMsg.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define max(a, b) ((a) > (b) ? (a) : (b))

using namespace std;

typedef pcl::PointXYZINormal PointType;

//ros::Publisher pub_calib_params, pub_merge_image;

ros::Publisher rgb_cloud_pub;

float fx_, fy_, cx_, cy_, k1_, k2_, k3_, p1_, p2_;
int min_depth = 2;
int max_depth = 100;
int width_, height_;
cv::Mat camera_matrix_, dist_coeffs_, init_extrinsic_;
cv::Mat rot_vec;
cv::Mat bias_0;
cv::Mat adjust_0;

double bias_yaw = 0.0;
double bias_pitch = 0.0;
double bias_roll = 0.0;

double bias_x = 0.0;
double bias_y = 0.0;
double bias_z = 0.0;

double window_size = 0.0;
double adjust_angle = 0.0;
double adjust_trans = 0.0;
double error_value = 0.0;

Eigen::Matrix3d init_rotation_matrix_;
Eigen::Vector3d init_translation_vector_;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
Vector6d calib_params;
Vector6d calib_params_estimate;

bool loadCameraConfig(const std::string &camera_file)
{
  cv::FileStorage cameraSettings(camera_file, cv::FileStorage::READ);
  if (!cameraSettings.isOpened())
  {
    std::cerr << "Failed to open camera params file at " << camera_file
              << std::endl;
    exit(-1);
  }
  else
  {
    ROS_INFO("Sucessfully load camera params file");
  }
  width_ = cameraSettings["Camera.width"];
  height_ = cameraSettings["Camera.height"];
  cameraSettings["CameraMat"] >> camera_matrix_;
  cameraSettings["DistCoeffs"] >> dist_coeffs_;
  fx_ = camera_matrix_.at<double>(0, 0);
  cx_ = camera_matrix_.at<double>(0, 2);
  fy_ = camera_matrix_.at<double>(1, 1);
  cy_ = camera_matrix_.at<double>(1, 2);
  k1_ = dist_coeffs_.at<double>(0, 0);
  k2_ = dist_coeffs_.at<double>(0, 1);
  p1_ = dist_coeffs_.at<double>(0, 2);
  p2_ = dist_coeffs_.at<double>(0, 3);
  k3_ = dist_coeffs_.at<double>(0, 4);
  std::cout << "Camera Matrix: " << std::endl
            << camera_matrix_ << std::endl;
  std::cout << "Distortion Coeffs: " << std::endl
            << dist_coeffs_ << std::endl;
  return true;
}



static float _int_as_float(uint32_t i)
{
  union
  {
    float f;
    uint32_t i;
  } conv{};

  conv.i = i;
  return conv.f;
}



std::queue<livox_ros_driver::CustomMsgConstPtr> _lidarMsgQueue;
std::queue<sensor_msgs::ImageConstPtr> _cameraMsgQueue;
std::mutex _mutexLidarQueue;
std::mutex _mutexCameraQueue;
bool newcamFlag = false;
bool newlidarFlag = false;

void sync_callback_lidar_cam(const sensor_msgs::ImageConstPtr &cameraMsgIn,
                             const livox_ros_driver::CustomMsgConstPtr &livox_msg_in)
{
  std::unique_lock<std::mutex> lock_cam(_mutexCameraQueue);
  _cameraMsgQueue.push(cameraMsgIn);
  newcamFlag = true;

  std::unique_lock<std::mutex> lock_lidar(_mutexLidarQueue);
  _lidarMsgQueue.push(livox_msg_in);
  newlidarFlag = true;
}

void pose_callback(const nav_msgs::OdometryConstPtr &calib_params_sub)
{
  calib_params_estimate[0]=calib_params_sub->pose.pose.orientation.x;
  calib_params_estimate[1]=calib_params_sub->pose.pose.orientation.y;
  calib_params_estimate[2]=calib_params_sub->pose.pose.orientation.z;
  
  calib_params_estimate[3]=calib_params_sub->pose.pose.position.x;
  calib_params_estimate[4]=calib_params_sub->pose.pose.position.y;
  calib_params_estimate[5]=calib_params_sub->pose.pose.position.z;
}




double project_2d(const Vector6d &extrinsic_params,
                  const pcl::PointCloud<PointType>::Ptr &lidar_cloud,
                  const cv::Mat &edge_img){
  double proj_score = 0;
  std::vector<cv::Point3f> pts_3d;
  

  // [1]. load point cloud into cv format
  for(size_t i =0;i<lidar_cloud->size();i++){
    PointType point_3d = lidar_cloud->points[i];
    float depth = point_3d.x; //notice here
    if(depth>min_depth && depth <max_depth){
      pts_3d.emplace_back(cv::Point3f(point_3d.x,point_3d.y,point_3d.z));
    }
  }
  // [2]. load intrinsic
  cv::Mat camera_matrix = (cv::Mat_<double>(3,3)<<fx_,0.0,cx_,0.0, fy_,cy_,0.0,0.0,1.0);
  cv::Mat distortion_coeff = (cv::Mat_<double>(1,5)<<k1_,k2_,p1_,p2_,k3_);
  cv::Mat r_vec =(cv::Mat_<double>(3,1)<<extrinsic_params[0],extrinsic_params[1],extrinsic_params[2]);
  cv::Mat t_vec =(cv::Mat_<double>(3,1)<<extrinsic_params[3],extrinsic_params[4],extrinsic_params[5]);

  // [3]. project 3d points into 2d image view
  std::vector<cv::Point2f> pts_2d;
  cv::projectPoints(pts_3d,r_vec,t_vec,camera_matrix,distortion_coeff,pts_2d);

  // [4]. caculate projection score
  for(size_t i = 0; i<pts_2d.size();++i){
    cv::Point2f point_2d = pts_2d[i];
    if(point_2d.x<=0||point_2d.x>=width_||point_2d.y<=0||point_2d.y>=height_){
      continue;
    } else{
      proj_score += (double)edge_img.at<uchar>(point_2d.y,point_2d.x);
    }
  }
 // [5]. normalize
  proj_score = proj_score/((double)lidar_cloud->size()*255.0);

  return proj_score;

}

// Color the point cloud by rgb image using given extrinsic
void colorCloud(
    const Vector6d &extrinsic_params,
    //const cv::Mat &input_image,
    const sensor_msgs::ImageConstPtr image_msg_in,
    //const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud,
    const livox_ros_driver::CustomMsgConstPtr &livox_msg_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_cloud) {
  cv::Mat input_image;
  cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image_msg_in, sensor_msgs::image_encodings::BGR8);
  img_ptr->image.copyTo(input_image);

  cv::Mat rgb_img;
  if (input_image.type() == CV_8UC3) {
    rgb_img = input_image;
  } else if (input_image.type() == CV_8UC1) {
    cv::cvtColor(input_image, rgb_img, cv::COLOR_GRAY2BGR);
  }
  std::vector<cv::Point3f> pts_3d;
  for (size_t i = 0; i < livox_msg_in->point_num; ++i) {
    pcl::PointXYZI point;
    point.x = livox_msg_in->points[i].x;
    point.y = livox_msg_in->points[i].y;
    point.z = livox_msg_in->points[i].z;
    //float depth = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    //if (depth > 2 && depth < 50 &&
        //point.intensity >= color_intensity_threshold_) {
    if(point.x>1 && point.x<50)
      pts_3d.emplace_back(cv::Point3f(point.x, point.y, point.z));
    
  }
 /* Eigen::AngleAxisd rotation_vector3;
  rotation_vector3 =
      Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
      */
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cv::Mat distortion_coeff =
      (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
      /*
  cv::Mat r_vec =
      (cv::Mat_<double>(3, 1)
           << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
       rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
       */
  cv::Mat r_vec =(cv::Mat_<double>(3,1)<<extrinsic_params[0],extrinsic_params[1],extrinsic_params[2]);
  cv::Mat t_vec = (cv::Mat_<double>(3, 1) << extrinsic_params[3],
                   extrinsic_params[4], extrinsic_params[5]);
  std::vector<cv::Point2f> pts_2d;
  cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
                    pts_2d);
  int image_rows = rgb_img.rows;
  int image_cols = rgb_img.cols;
  color_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (size_t i = 0; i < pts_2d.size(); i++) {
    if (pts_2d[i].x >= 0 && pts_2d[i].x < image_cols && pts_2d[i].y >= 0 &&
        pts_2d[i].y < image_rows) {
      cv::Scalar color =
          rgb_img.at<cv::Vec3b>((int)pts_2d[i].y, (int)pts_2d[i].x);
      if (color[0] == 0 && color[1] == 0 && color[2] == 0) {
        continue;
      }
      if (pts_3d[i].x > 100) {
        continue;
      }
      pcl::PointXYZRGB p;
      p.x = pts_3d[i].x;
      p.y = pts_3d[i].y;
      p.z = pts_3d[i].z;
      // p.a = 255;
      p.b = color[0];
      p.g = color[1];
      p.r = color[2];
      color_cloud->points.push_back(p);
    }
  }
  color_cloud->width = color_cloud->points.size();
  color_cloud->height = 1;
}


void online_display_process(){

  vector<cv::Mat> feature_img_vec;
  vector<pcl::PointCloud<PointType>::Ptr> feature_cloud_vec;
  vector<double> score_vec;
  double best_score = 0;

  //start online calib process here
  while(ros::ok()){
    double time_curr_cam = 0.0;
    double time_curr_lidar = 0.0;
    clock_t start_t, end_t;

    if(newlidarFlag && newcamFlag){
      livox_ros_driver::CustomMsgConstPtr livox_msg_sycho;
      sensor_msgs::ImageConstPtr image_msg_sycho;

      // ----------- get new camera data msg ------------ //
      std::unique_lock<std::mutex> lock_camera(_mutexCameraQueue);
      if(!_cameraMsgQueue.empty()){
        time_curr_cam = _cameraMsgQueue.front()->header.stamp.toSec();
        image_msg_sycho = _cameraMsgQueue.front();
        _cameraMsgQueue.pop();
      }
      lock_camera.unlock();

      // ----------- get new lidar data msg ------------ //
      std::unique_lock<std::mutex> lock_lidar(_mutexLidarQueue);
      if(!_lidarMsgQueue.empty()){
        time_curr_lidar = _lidarMsgQueue.front()->header.stamp.toSec();
        livox_msg_sycho = _lidarMsgQueue.front();
        _lidarMsgQueue.pop();
      }
      lock_lidar.unlock();

      newlidarFlag =  false;
      newcamFlag = false;

      if(abs(time_curr_lidar - time_curr_cam)<0.1){
        //start process
        //cout<<"sycho: "<<time_curr_lidar<<" "<<time_curr_cam<<endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        sensor_msgs::PointCloud2 rgb_cloud_msg;
        //cout<<calib_params_estimate<<endl;
        colorCloud(calib_params_estimate,image_msg_sycho,livox_msg_sycho,rgb_cloud);
        pcl::toROSMsg(*rgb_cloud, rgb_cloud_msg);
        rgb_cloud_msg.header.frame_id = "world";
        // publish
        rgb_cloud_pub.publish(rgb_cloud_msg);
    
       
      }


    }

  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "online display");
  ros::NodeHandle nh;

  srand(time(NULL));
  ros::Duration(0.5).sleep();
  ROS_INFO("\033[1;32m--->\033[0m ======================================");

  ROS_INFO("\033[1;32m--->\033[0m online display node start.");

  ROS_INFO("\033[1;32m--->\033[0m ======================================");

  
  message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/strobe_image1", 100);

  message_filters::Subscriber<livox_ros_driver::CustomMsg> lidar_sub(nh, "/livox/lidar", 100);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, livox_ros_driver::CustomMsg> Sync_LiDAR_Cam;

  message_filters::Synchronizer<Sync_LiDAR_Cam> sync(Sync_LiDAR_Cam(20), img_sub, lidar_sub);
  sync.registerCallback(boost::bind(&sync_callback_lidar_cam, _1, _2));

  const std::string CameraConfigPath = std::string(argv[1]);
  const std::string CalibConfigPath = std::string(argv[2]);

  ROS_INFO("load camera params");
  loadCameraConfig(CameraConfigPath);

  ros::Subscriber pose_sub = nh.subscribe("/Calib_params", 5, pose_callback);

  rgb_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rgb_cloud", 50);

  std::thread thread_process{online_display_process};


  ros::spin();
  return 0;
}
