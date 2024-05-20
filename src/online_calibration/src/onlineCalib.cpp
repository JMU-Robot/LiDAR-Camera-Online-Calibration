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

#define max(a, b) ((a) > (b) ? (a) : (b))

using namespace std;

typedef pcl::PointXYZINormal PointType;

ros::Publisher pub_calib_params, pub_merge_image;


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

double cur_score_display = 0.0;

Eigen::Matrix3d init_rotation_matrix_;
Eigen::Vector3d init_translation_vector_;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
Vector6d calib_params;
Vector6d calib_params_estimate;

Vector6d calib_params_out;

vector<double> score_result_v;

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

bool loadCalibConfig(const std::string &config_file)
{

  cv::FileStorage fSettings(config_file, cv::FileStorage::READ);
  if (!fSettings.isOpened())
  {
    std::cerr << "Failed to open calib settings file at: " << config_file
              << std::endl;
    exit(-1);
  }
  else
  {
    ROS_INFO("Sucessfully load calib config file");
  }

  fSettings["ExtrinsicMat"] >> init_extrinsic_;

  init_rotation_matrix_ << init_extrinsic_.at<double>(0, 0),
      init_extrinsic_.at<double>(0, 1), init_extrinsic_.at<double>(0, 2),
      init_extrinsic_.at<double>(1, 0), init_extrinsic_.at<double>(1, 1),
      init_extrinsic_.at<double>(1, 2), init_extrinsic_.at<double>(2, 0),
      init_extrinsic_.at<double>(2, 1), init_extrinsic_.at<double>(2, 2);
  init_translation_vector_ << init_extrinsic_.at<double>(0, 3),
      init_extrinsic_.at<double>(1, 3), init_extrinsic_.at<double>(2, 3);

  fSettings["rotVector"] >> rot_vec;

  calib_params[0] = rot_vec.at<double>(0, 0);
  calib_params[1] = rot_vec.at<double>(0, 1);
  calib_params[2] = rot_vec.at<double>(0, 2);

  calib_params[3] = rot_vec.at<double>(0, 3);
  calib_params[4] = rot_vec.at<double>(0, 4);
  calib_params[5] = rot_vec.at<double>(0, 5);

  fSettings["biasVector"] >> bias_0;

  bias_yaw = bias_0.at<double>(0, 0);
  bias_pitch = bias_0.at<double>(0, 1);
  bias_roll = bias_0.at<double>(0, 2);

  bias_x = bias_0.at<double>(0, 3);
  bias_y = bias_0.at<double>(0, 4);
  bias_z = bias_0.at<double>(0, 5);

  fSettings["adjustVector"] >> adjust_0;

  adjust_angle = adjust_0.at<double>(0, 0);
  adjust_trans = adjust_0.at<double>(0, 1);
  window_size  = adjust_0.at<double>(0, 2);
  error_value  = adjust_0.at<double>(0, 3);

  std::cout<<"init vector: "<<calib_params<<std::endl;
  std::cout<<"adjust angle: "<<adjust_angle<<std::endl;
  std::cout<<"adjust_trans: "<<adjust_trans<<std::endl;
  std::cout<<"window_size : "<<window_size<<std::endl;
  std::cout<<"error_value : "<<error_value<<std::endl;
  
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

float intensityRatio(float intensity_a, float intensity_b)
{
  float max_0 = 0.0;
  float min_0 = 0.0;
  float ratio = 0.0;

  max_0 = ((intensity_a) > (intensity_b) ? (intensity_a) : (intensity_b));
  min_0 = ((intensity_a) < (intensity_b) ? (intensity_a) : (intensity_b));
  return min_0 / max_0;
}

void detectFeaturePoint(pcl::PointCloud<PointType>::Ptr &cloud_in,
                        pcl::PointCloud<PointType>::Ptr &cloud_out)
{

  int cloudSize = cloud_in->points.size();
  int count_num = 0;
  for (int i = 5; i < cloudSize - 5; i++)
  {
    float diff_intensity_l =
        intensityRatio((float)cloud_in->points[i + 1].intensity,
                       (float)cloud_in->points[i].intensity);
    float diff_intensity_r =
        intensityRatio((float)cloud_in->points[i - 1].intensity,
                       (float)cloud_in->points[i].intensity);
    float diff_intensity_l_l =
        intensityRatio((float)cloud_in->points[i + 2].intensity,
                       (float)cloud_in->points[i].intensity);
    float diff_intensity_r_r =
        intensityRatio((float)cloud_in->points[i - 2].intensity,
                       (float)cloud_in->points[i].intensity);

    if ((((diff_intensity_l < 0.4) && (diff_intensity_l_l < 0.4)) ||
         ((diff_intensity_r < 0.4) && (diff_intensity_r_r < 0.4))) &&
        cloud_in->points[i].x < 50)
    {
      cloud_out->push_back(cloud_in->points[i]);
    }
  }
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

cv::Mat inverse_distance_transform(const cv::Mat &input_img, int kernel_size)
{
  double alpha = 0.3;
  double gamma = 0.98;
  cv::Mat output_img = input_img.clone();

  for (int y = kernel_size; y < input_img.rows - kernel_size; ++y)
  {
    for (int x = kernel_size; x < input_img.cols - kernel_size; ++x)
    {

      double max_e = 0;
      double max_E = 0;
      int dis = 0;

      for (int j = y - kernel_size; j < y + kernel_size; ++j)
      {
        for (int i = x - kernel_size; i < x + kernel_size; ++i)
        {
          int dis = max(abs(i - x), abs(j - y));
          max_e = (double)output_img.at<uchar>(j, i);

          if (max_E < max_e)
          {
            max_E = max_e;
            dis = max(abs(i - x), abs(j - y));
          }
        }
      }

      output_img.at<uchar>(y, x) = alpha * (double)output_img.at<uchar>(y, x) +
                                   (1 - alpha) * (double)max_E * pow(gamma, dis);
    }
  }
  return output_img;
}


double pca_filter(pcl::PointCloud<PointType>::Ptr &cloud_raw,
                  pcl::PointCloud<PointType>::Ptr &cloud_feature,
                  pcl::PointCloud<PointType>::Ptr &cloud_feature_out)
{
  Eigen::Matrix3f convariance;
  Eigen::Vector4f centroid;

  int count = 0;
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(cloud_raw);
  int K = 10;
  std::vector<int> pointIdx(K);
  std::vector<float> pointDis(K);

  //for every feature point, search the neighbor and caculate the noise by its convariance
  for(int i=0;i<cloud_feature->points.size();i++){
    pcl::PointCloud<PointType>::Ptr cloud_neighbor(new pcl::PointCloud<PointType>);
    if(kdtree.nearestKSearch(cloud_feature->points[i],K,pointIdx,pointDis)>0){
      for(int j=0;j<pointIdx.size();j++)
       cloud_neighbor->points.push_back(cloud_raw->points[pointIdx[j]]);
    }

    pcl::compute3DCentroid(*cloud_neighbor, centroid);
    pcl::computeCovarianceMatrix(*cloud_neighbor, centroid,convariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(convariance);

    if(eigen_solver.eigenvalues()[0]!=0 &&
       eigen_solver.eigenvalues()[2]!=0){
         if(eigen_solver.eigenvalues()[0]/eigen_solver.eigenvalues()[2]<0.1){
           cloud_feature_out->points.push_back(cloud_feature->points[i]);
           count+=1;
       }
    }
    
  }
  //noise ratio of feature frame
  return (double)count/(double)cloud_feature->points.size(); 
        
}


double get_feature_cloud(const livox_ros_driver::CustomMsgConstPtr &livox_msg_in,
pcl::PointCloud<PointType>::Ptr &feature_point)
{
  pcl::PointCloud<PointType>::Ptr pcl_in(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr feature_cloud_raw(new pcl::PointCloud<PointType>());

  pcl::PointCloud<PointType>::Ptr line_0(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr line_1(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr line_2(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr line_3(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr line_4(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr line_5(new pcl::PointCloud<PointType>());

  pcl_in.reset(new pcl::PointCloud<PointType>());

  double timeSpan = ros::Time().fromNSec(livox_msg_in->points.back().offset_time).toSec();

  for(unsigned  int i=0;i<livox_msg_in->point_num;++i){
    int line_num = (int)livox_msg_in->points[i].line;

    pcl::PointXYZINormal pt;
    pt.x = livox_msg_in->points[i].x;
    pt.y = livox_msg_in->points[i].y;
    pt.z = livox_msg_in->points[i].z;

    pt.intensity = livox_msg_in->points[i].reflectivity;

    pt.normal_x = ros::Time().fromNSec(livox_msg_in->points.back().offset_time).toSec()/timeSpan;
    pt.normal_y = _int_as_float(line_num);

    pcl_in->push_back(pt);

    switch(line_num){
      case 0: line_0->push_back(pt);break;
      case 1: line_1->push_back(pt);break;
      case 2: line_2->push_back(pt);break;
      case 3: line_3->push_back(pt);break;
      case 4: line_4->push_back(pt);break;
      case 5: line_5->push_back(pt);break;
      default:break;
    }
  }

  detectFeaturePoint(line_0,feature_cloud_raw);
  detectFeaturePoint(line_1,feature_cloud_raw);
  detectFeaturePoint(line_2,feature_cloud_raw);
  detectFeaturePoint(line_3,feature_cloud_raw);
  detectFeaturePoint(line_4,feature_cloud_raw);
  detectFeaturePoint(line_5,feature_cloud_raw);

  double noise_ratio = 0;
  if(pcl_in->points.size()>0)
    noise_ratio = pca_filter(pcl_in, feature_cloud_raw,feature_point);
    
  return noise_ratio;

}

double project_score_normalize(const Vector6d &extrinsic_params,
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
      proj_score += ((double)edge_img.at<uchar>(point_2d.y,point_2d.x))*(1.0/(double)pts_3d[i].x); //add weight here
    }
  }
 // [5]. normalize
  proj_score = proj_score/((double)lidar_cloud->size()*255.0);


  return proj_score;

}

void project_2d(const Vector6d &extrinsic_params,
                const pcl::PointCloud<PointType>::Ptr &lidar_cloud,
                const cv::Mat &edge_img,
                cv::Mat &project_2d_img){
  
  std::vector<cv::Point3f> pts_3d;

  cv::Mat project_2d = cv::Mat::zeros(height_,width_,CV_16UC1);// 16 bit depth map
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
      project_2d.at<ushort>(point_2d.y, point_2d.x)= 65535; // pointcloud project to 2d image
    }
    
  }
 

  project_2d.convertTo(project_2d, CV_8UC1, 1/256.0);

  project_2d_img = project_2d.clone();

 

}

bool display_text(cv::Mat &input_img, cv::Point origin, string text)
{
  int font = cv::FONT_HERSHEY_DUPLEX;
  double scale = 1.0;
  int thickness = 2;
  cv::putText(input_img, text, origin, font, scale, cv::Scalar(255,255,255),thickness, 8 ,0);
  return 1;
}

bool display_calib_status(cv::Mat &input_img){
  cv::Point origin_0;
  origin_0.x = 20;
  origin_0.y = 30;
  string text_0 = "  [ Online calibration ]  ";
  display_text(input_img, origin_0, text_0);

  cv::Point origin_1;
  origin_1.x = 20;
  origin_1.y = 70;
  string text_1;
  text_1 = "cur_score: [ " + to_string(cur_score_display) + " ]";
  display_text(input_img, origin_1, text_1);

  return 1;
}

//计算socre的方差
double stdev_score(vector<double> &score_result_v){
  double sum = std::accumulate(std::begin(score_result_v),
                               std::end(score_result_v),0.0);
  double mean  = sum/score_result_v.size();
  double accum = 0.0;
  std::for_each(std::begin(score_result_v),
                std::end(score_result_v),
                [&](const double d){accum += (d-mean)*(d-mean);});
  return sqrt(accum/(score_result_v.size()-1));
}

bool get_feature_image(const sensor_msgs::ImageConstPtr &cameraMsgIn,
                      cv::Mat &feature_image_out){
   cv::Mat rgb_img;
   cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(cameraMsgIn, sensor_msgs::image_encodings::BGR8);
   img_ptr->image.copyTo(rgb_img);

   cv::Mat cut_grey_image_;
   cv::Mat edge_corner_image_;
   cv::Mat result;

   cv::cvtColor(rgb_img,cut_grey_image_,cv::COLOR_BGR2GRAY);   

   cv::Mat XBorder, YBorder, XYBorder;
   Sobel(cut_grey_image_,XBorder,CV_16S,1,0,3,1.0,0);   
   Sobel(cut_grey_image_,YBorder,CV_16S,1,0,3,1.0,0);  
   convertScaleAbs(XBorder,XBorder);
   convertScaleAbs(YBorder,YBorder);

   addWeighted(XBorder,0.5,YBorder,0.5,0,XYBorder);

   XYBorder.convertTo(XYBorder,CV_8UC1); //255 max

   feature_image_out = inverse_distance_transform(XYBorder, 2); 
   
   return 1;              

}

double estimate_calib_params_multi(const vector<cv::Mat> &feature_img_vec,
const vector<pcl::PointCloud<PointType>::Ptr> &feature_cloud_vec,
Vector6d init_calib_params, Vector6d &output_calib_params)
{
  double calib_score = 0.0;
  cv::Mat project_2d_img;

  if(feature_cloud_vec.size()!=0&&feature_cloud_vec.size()==feature_img_vec.size()){
    double score = 0.0;
    for(int i=0;i<feature_cloud_vec.size();i++){
      score+=project_score_normalize(init_calib_params, feature_cloud_vec[i],feature_img_vec[i]);
    }

    //current_score_display = score;

    int find_better_count = 0;
    double max_score = score;
    Vector6d calib_params_best_tmp = init_calib_params;

    for(int angle_0 = -1; angle_0<2; angle_0+=1){
      for(int angle_1 = -1; angle_1<2; angle_1+=1){
        for(int angle_2 = -1; angle_2<2; angle_2+=1){
          //for speed up
          //for(int trans_0 = -1; trans_0<2; trans_0+=1){
            //for(int trans_1 = -1; trans_1<2; trans_1+=1){
              //for(int trans_2 = -1; trans_2<2; trans_2+=1){
                Vector6d calib_params_tmp;

                calib_params_tmp[0] = init_calib_params[0]+(double)angle_0*adjust_angle+
                (double)angle_0*((double)(rand()%10)/10000.0);
                calib_params_tmp[1] = init_calib_params[1]+(double)angle_1*adjust_angle+
                (double)angle_1*((double)(rand()%10)/10000.0);
                calib_params_tmp[2] = init_calib_params[2]+(double)angle_2*adjust_angle+
                (double)angle_2*((double)(rand()%10)/10000.0);
                
                //for speed up
                /*
                calib_params_tmp[3] = init_calib_params[3]+(double)trans_0*adjust_angle+
                (double)trans_0*((double)(rand()%10)/10000.0);
                calib_params_tmp[4] = init_calib_params[4]+(double)trans_1*adjust_angle+
                (double)trans_1*((double)(rand()%10)/10000.0);
                calib_params_tmp[5] = init_calib_params[5]+(double)trans_2*adjust_angle+
                (double)trans_2*((double)(rand()%10)/10000.0);
                */
                double temp_score = 0.0;

                for(int i=0; i<feature_cloud_vec.size();i++){
                  temp_score+=project_score_normalize(calib_params_tmp, feature_cloud_vec[i],feature_img_vec[i]);
                }

                if(temp_score>score){
                  find_better_count+=1;

                  if(temp_score>max_score){
                    max_score=temp_score;
                    calib_params_best_tmp = calib_params_tmp;
                    //calib_params_best_tmp = init_calib_params;

                    //after_score_display = max_score; 
                    
                  }
                }

              //}
            //}
          //}
        }
      }
    }// end of for

    output_calib_params = calib_params_best_tmp;
    //calib_score = (double)(729-find_better_count)/729.0;
    calib_score = (double)(27-find_better_count)/27.0;
    //calib_score_display = calib_score;

  } else{cout<<"sycho data error!"<<endl;}

  cv::Mat depth_img;
  cv::Mat merge_image;
  //projection(output_calib_params, feature_cloud_vec[0],depth_img);
  project_2d(output_calib_params, feature_cloud_vec[0],feature_img_vec[0],depth_img);
  cv::addWeighted(feature_img_vec[0],0.5, depth_img, 0.5, 0, merge_image);
  display_calib_status(merge_image);

  //publish merge feature image
  sensor_msgs::ImagePtr processed_image = cv_bridge::CvImage(std_msgs::Header(), 
  sensor_msgs::image_encodings::MONO8, merge_image).toImageMsg();

  pub_merge_image.publish(*processed_image);
  
  return calib_score;

}

void pubcalibParams(const Vector6d &calib_params)
{
  nav_msgs::Odometry calib_params_pub;
  calib_params_pub.header.frame_id = "/livox_frame";
  calib_params_pub.header.stamp = ros::Time().now(); 
  calib_params_pub.pose.pose.orientation.x = calib_params[0];
  calib_params_pub.pose.pose.orientation.y = calib_params[1];
  calib_params_pub.pose.pose.orientation.z = calib_params[2];
  calib_params_pub.pose.pose.orientation.w = 0;
  calib_params_pub.pose.pose.position.x = calib_params[3];
  calib_params_pub.pose.pose.position.y = calib_params[4];
  calib_params_pub.pose.pose.position.z = calib_params[5];

  pub_calib_params.publish(calib_params_pub);
}

void online_calib_process(){

  vector<cv::Mat> feature_img_vec;
  vector<pcl::PointCloud<PointType>::Ptr> feature_cloud_vec;
  vector<double> score_vec;
  

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
        cv::Mat feature_img;
        get_feature_image(image_msg_sycho, feature_img);
       // cv::imshow("feature",feature_img);
       // cv::waitKey(1);
        pcl::PointCloud<PointType>::Ptr feature_point(new pcl::PointCloud<PointType>());
        double noise_ratio = get_feature_cloud(livox_msg_sycho, feature_point);

        feature_img_vec.push_back(feature_img);
        feature_cloud_vec.push_back(feature_point);

        double score = estimate_calib_params_multi(feature_img_vec,feature_cloud_vec,
                                                calib_params, calib_params_estimate);
        //std::cout<<"score "<<score<<std::endl;

        cur_score_display = score;

        
        calib_params = calib_params_estimate;

        //if(score==1){
        //calib_params_out=(calib_params_out+calib_params)/2.0;
        //calib_params = calib_params_out;
        
        //}
        
         
        

        //cout<<calib_params_out[0]<<" "<<calib_params_out[1]<<" "<<calib_params_out[2]<<" "<<score<<endl;
        if(stdev_score(score_result_v)==0){
        //cout<<calib_params[0]<<" "<<calib_params[1]<<" "<<calib_params[2]<<" "<<score<<" "<<stdev_score(score_result_v)<<endl;
        cout<<calib_params[0]<<" "<<calib_params[1]<<" "<<calib_params[2]<<" "<<score<<endl;
        calib_params_out = calib_params;
        }
        else
        cout<<calib_params_out[0]<<" "<<calib_params_out[1]<<" "<<calib_params_out[2]<<" "<<score<<endl;

        score_result_v.push_back(score);
        pubcalibParams(calib_params_out);

        if(feature_img_vec.size()>(int)(window_size-1))
        feature_img_vec.erase(feature_img_vec.begin(),
                              feature_img_vec.begin()+
                              feature_img_vec.size()-
                              (int)(window_size-1));
        if(feature_cloud_vec.size()>(int)(window_size-1))
        feature_cloud_vec.erase(feature_cloud_vec.begin(),
                              feature_cloud_vec.begin()+
                              feature_cloud_vec.size()-
                              (int)(window_size-1));

        if(score_result_v.size()>(int)(20))
        score_result_v.erase(score_result_v.begin(),
                              score_result_v.begin()+
                              score_result_v.size()-
                              (int)(20));
      }


    }

  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "online cali");
  ros::NodeHandle nh;

  srand(time(NULL));
  ROS_INFO("\033[1;32m--->\033[0m ======================================");
  ROS_INFO("\033[1;32m--->\033[0m online calibration node start.");
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

  ROS_INFO("load calib config");
  loadCalibConfig(CalibConfigPath);

  calib_params[0]+=bias_yaw;
  calib_params[1]+=bias_pitch;
  calib_params[2]+=bias_roll;

  calib_params[3]+=bias_x;
  calib_params[4]+=bias_y;
  calib_params[5]+=bias_z;

  calib_params_out = calib_params;

  //pub_calib_params = calib_params;

  pub_calib_params = nh.advertise<nav_msgs::Odometry>("/Calib_params", 5);
  pub_merge_image = nh.advertise<sensor_msgs::Image>("/merge_img", 100);
  //rgb_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rgb_cloud", 50);

  std::thread thread_process{online_calib_process};


  ros::spin();
  return 0;
}
