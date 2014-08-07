#ifndef CASCADE_CLASSIFIER_H_
#define CASCADE_CLASSIFIER_H_

//#define USE_GPU 1

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#ifdef USE_GPU
#include <opencv2/gpu/gpu.hpp>
#endif

#include <time.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/shared_ptr.hpp>

#include <tf/transform_listener.h>
#include <XmlRpcException.h>

namespace vision{

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace boost;


#ifdef USE_GPU
 typedef cv::gpu::CascadeClassifier_GPU classifier_t;
#else
 typedef cv::CascadeClassifier classifier_t;
#endif


 class training_definition;
 typedef boost::shared_ptr<training_definition> training_definition_ptr;

 typedef struct {
   cv::Rect rect;
   cv::Point2d detection_center;
   cv::Point2d disparity_center;
   float disp_val;
   cv::Point3d projected_position;
 } detection_object;

 typedef struct {
   training_definition_ptr definition;
   std::vector<detection_object> objects;
   double object_radius_;
 } detection_set;
  
 class training_definition{
 private:
   std::string cascade_path_;
   classifier_t cascade_classifier_;

   int id_;
   std::string label_;
   double object_radius_;
   double scale_factor_;
   int min_neighbors_;
   cv::Size min_size_;
   cv::Size max_size_;
   float max_disparity_for_detection_;
 public:
 training_definition(int id, std::string label, std::string cascade_path, double object_radius, double scale_factor, int min_neighbors, cv::Size min_size, cv::Size max_size, float max_disparity_for_detection) : id_(id), label_(label), cascade_path_(cascade_path), object_radius_(object_radius), scale_factor_(scale_factor), min_neighbors_(min_neighbors), min_size_(min_size), max_size_(max_size), max_disparity_for_detection_(max_disparity_for_detection){
     if (!cascade_classifier_.load(cascade_path_)) {
       ROS_ERROR("Error loading cascade classifier: %s", cascade_path_.c_str());
     }
     ROS_INFO_STREAM("Loaded training definition: "<<id_<<":"<<label_<<", "<<cascade_path_);
   }
   int id(){return id_;}
   std::string label(){return label_;}
   double object_radius(){return object_radius_;}
   std::string cascade_path(){return cascade_path_;}
   float max_disparity_for_detection(){return max_disparity_for_detection_;}
    
   detection_set detect(cv::Mat image_gray, training_definition_ptr self){
#ifdef USE_GPU
     cv::gpu::GpuMat image_gray_gpu(image_gray);
     cv::gpu::GpuMat gpu_detections_mat;
     int num_objects = cascade_classifier_.detectMultiScale(image_gray_gpu, gpu_detections_mat, scale_factor_, min_neighbors_, min_size_);
     cv::Mat detections_mat;
     gpu_detections_mat.colRange(0, num_objects).download(detections_mat);
     cv::Rect* object_rects = detections_mat.ptr<cv::Rect>();
#else
     std::vector<cv::Rect> object_rects;
     cascade_classifier_.detectMultiScale(image_gray, object_rects, scale_factor_, min_neighbors_, 0, min_size_, max_size_);
     int num_objects = object_rects.size();
#endif
     detection_set detection;
     for(int i = 0; i < num_objects; ++i){
       detection_object object;
       object.rect = object_rects[i];
       detection.objects.push_back(object);
     }
     detection.definition = self;
     return detection;
   }
 };




class CascadeClassifier{

  typedef ApproximateTime<Image, DisparityImage> ImageSyncPolicy;
  typedef message_filters::Synchronizer<ImageSyncPolicy> ImageSynchronizer;

  typedef ApproximateTime<CameraInfo, CameraInfo> InfoSyncPolicy;
  typedef message_filters::Synchronizer<InfoSyncPolicy> InfoSynchronizer;

 public:
  CascadeClassifier(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~CascadeClassifier();

 private:
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter sub_l_image_;
  message_filters::Subscriber<DisparityImage> sub_d_image_;
  shared_ptr<ImageSynchronizer> image_synchronizer_;

  message_filters::Subscriber<CameraInfo> sub_l_info_;
  message_filters::Subscriber<CameraInfo> sub_r_info_;
  shared_ptr<InfoSynchronizer> info_synchronizer_;

  boost::shared_mutex stereo_model_mutex_;
  bool stereo_model_init_;
  image_geometry::StereoCameraModel stereo_model_;

  tf::TransformListener tf_listener_;

  ros::Publisher object_location_pub_;
  image_transport::Publisher disparity_pub_;
  image_transport::Publisher image_pub_;

  std::vector<training_definition_ptr> training_definitions_;
  std::map<int, training_definition_ptr> training_definitions_by_id_;

  bool show_windows_;

  std::string robot_frame_;

  void imageCb(const ImageConstPtr& l_image_msg,
	       const DisparityImageConstPtr& d_image_msg);

  void infoCb(const CameraInfoConstPtr& l_info_msg,
	      const CameraInfoConstPtr& r_info_msg);

  void detect_and_publish(const std::string& l_camera_frame, const ros::Time& time, cv::Mat& l_mat, cv::Mat& d_image);
  float average_disparity(const cv::Mat& disp, const cv::Point2d& pt, int width, int height);

  void putTextCenter(cv::Mat& img, const std::string& text, cv::Point org, int font_face, double scale, cv::Scalar color, int thickness = 1);

  std::vector<training_definition_ptr> parse_training_definitions(XmlRpc::XmlRpcValue& training_descriptions);
  training_definition_ptr parse_training_definition(XmlRpc::XmlRpcValue& training_description);
  cv::Size parse_size(XmlRpc::XmlRpcValue& size);
};


}

#endif
