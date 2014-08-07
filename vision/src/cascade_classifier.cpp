#include <vision/cascade_classifier.h>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>
#include <omp.h>

namespace vision{
  using namespace message_filters::sync_policies;

  CascadeClassifier::CascadeClassifier(ros::NodeHandle nh, ros::NodeHandle pnh): it_(nh), stereo_model_init_(false), show_windows_(false){
    pnh.getParam("show_windows", show_windows_);
    pnh.getParam("robot_frame", robot_frame_);

    robot_frame_ = "aero/base_footprint";


    XmlRpc::XmlRpcValue training_xml;
    if(!pnh.getParam("training", training_xml)){
      ROS_WARN("No training definitions specified");
    }
    try{
      training_definitions_ = parse_training_definitions(training_xml);
      BOOST_FOREACH(training_definition_ptr def, training_definitions_){
        training_definitions_by_id_[def->id()] = def;
      }
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading training definitions: "<<e.getMessage());
    }



#ifdef USE_GPU
    ROS_INFO("Using GPU Detector");
#else
    ROS_INFO("Using CPU Detector");
#endif



    image_synchronizer_.reset(new ImageSynchronizer(ImageSyncPolicy(10), sub_l_image_, sub_d_image_));
    image_synchronizer_->registerCallback(boost::bind(&CascadeClassifier::imageCb,
					    this, _1, _2));

    info_synchronizer_.reset(new InfoSynchronizer(InfoSyncPolicy(6), sub_l_info_, sub_r_info_));
    info_synchronizer_->registerCallback(boost::bind(&CascadeClassifier::infoCb,
					    this, _1, _2));

    object_location_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("object_detection", 5);
    disparity_pub_ = it_.advertise("object_detection/disparity", 1);
    image_pub_ = it_.advertise("object_detection/image", 1);

    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh);
    sub_l_image_.subscribe(it_, "left/image_rect", 1, hints);
    sub_d_image_.subscribe(nh, "disparity", 1);

    sub_l_info_.subscribe(nh, "left/camera_info", 1);
    sub_r_info_.subscribe(nh, "right/camera_info", 1);


    if(show_windows_){
      cv::namedWindow("image");
      cv::namedWindow("disparity");
    }
  }

  CascadeClassifier::~CascadeClassifier(){
    if(show_windows_){
      cv::destroyWindow("image");
      cv::destroyWindow("disparity");
    }
  }


  void CascadeClassifier::imageCb(const ImageConstPtr& l_image_msg,
				  const DisparityImageConstPtr& d_image_msg){
    ROS_DEBUG("Got Matching Image and Disparity delay=%fs", (ros::Time::now()-l_image_msg->header.stamp).toSec());
    cv_bridge::CvImagePtr l_image_ptr;
    cv_bridge::CvImagePtr d_image_ptr;
    try{
      l_image_ptr = cv_bridge::toCvCopy(l_image_msg, sensor_msgs::image_encodings::BGR8);
      d_image_ptr = cv_bridge::toCvCopy(d_image_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    detect_and_publish(l_image_msg->header.frame_id, l_image_msg->header.stamp, l_image_ptr->image, d_image_ptr->image);
  }

  void CascadeClassifier::infoCb(const CameraInfoConstPtr& l_info_msg,
				 const CameraInfoConstPtr& r_info_msg){
    
    boost::unique_lock<boost::shared_mutex> lock(stereo_model_mutex_);
    stereo_model_.fromCameraInfo(l_info_msg, r_info_msg);
    stereo_model_init_ = true;
  }

  void CascadeClassifier::detect_and_publish(const std::string& l_camera_frame, const ros::Time& time, cv::Mat& l_mat, cv::Mat& d_image){
    ros::Time pre_start = ros::Time::now();
    cv::Mat image_gray;
    cv::cvtColor(l_mat, image_gray, CV_RGB2GRAY);
    cv::equalizeHist(image_gray, image_gray);

    cv::Mat disparity_color;
    cv::convertScaleAbs(d_image, disparity_color, 100, 0.0);
    cv::cvtColor(disparity_color, disparity_color, CV_GRAY2RGB);

    std::vector<detection_set> detections;

    {//check that the model has been initialized yet, if not just drop the frame
      boost::lock_guard<boost::shared_mutex> lock(stereo_model_mutex_);
      if(!stereo_model_init_)
	return;
    }

#pragma omp parallel for shared(detections)
    for(int i = 0; i<training_definitions_.size(); ++i){
      training_definition_ptr definition = training_definitions_[i];

      ros::Time classifier_start = ros::Time::now();
      detection_set detection = definition->detect(image_gray, definition);
      ROS_DEBUG("Classification done: %fs", (ros::Time::now()-classifier_start).toSec());

      {
	boost::lock_guard<boost::shared_mutex> lock(stereo_model_mutex_);

	BOOST_FOREACH(detection_object& object, detection.objects){
	  object.detection_center = cv::Point2d(object.rect.x + object.rect.width / 2,
						object.rect.y + object.rect.height / 2);

	  object.disparity_center = cv::Point2d(object.detection_center.x,
						object.detection_center.y+30);

	  object.disp_val = average_disparity(d_image, object.disparity_center, 20, 40);

	  stereo_model_.projectDisparityTo3d(object.detection_center, object.disp_val, object.projected_position);
	  object.projected_position.z += definition->object_radius();
	}
      }

#pragma omp critical
      {
        detections.push_back(detection);
      }
    }

    BOOST_FOREACH(detection_set& detection, detections){
      BOOST_FOREACH(detection_object& object, detection.objects){

	cv::Scalar color;
	if(object.disp_val<0){
          ROS_DEBUG("No disparity for detection: In left camera at (%d, %d)", (int)object.detection_center.x, (int)object.detection_center.y);
	  color = cv::Scalar(0, 0, 255);
	}
	else if(object.disp_val > detection.definition->max_disparity_for_detection()){
          ROS_DEBUG("Disparity above max for detection: In left camera at (%d, %d), disp = %f, (%f m)", (int)object.detection_center.x, (int)object.detection_center.y, object.disp_val, object.projected_position.z);
	  color = cv::Scalar(0, 255, 0);
	}
	else{
	  color = cv::Scalar(255, 0, 0);

	  tf::Point position_tf(object.projected_position.x, object.projected_position.y, object.projected_position.z);
	  geometry_msgs::PointStamped camera_point, robot_point;
	  camera_point.header.frame_id = l_camera_frame;
	  camera_point.header.stamp = time;
	  tf::pointTFToMsg(position_tf, camera_point.point);

          ROS_DEBUG("Got detection: In left camera at (%d, %d), disp = %f (%f m)", (int)object.detection_center.x, (int)object.detection_center.y, object.disp_val, object.projected_position.z);

	  geometry_msgs::PoseWithCovarianceStamped msg;
	  msg.header = camera_point.header;
	  msg.pose.pose.position = camera_point.point;
	  msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	  msg.pose.covariance = boost::assign::list_of(0.2) (0)   (0)  (0)  (0)  (0)
	    (0)  (0.2)  (0)  (0)  (0)  (0)
	    (0)   (0)  (0.3) (0)  (0)  (0)
	    (0)   (0)   (0)  (0)  (0)  (0)
	    (0)   (0)   (0)  (0)  (0)  (0)
	    (0)   (0)   (0)  (0)  (0)  (0) ;
	  object_location_pub_.publish(msg);
	}

	cv::ellipse(l_mat, object.detection_center,
		    cv::Size(object.rect.width / 2, object.rect.height / 2),
		    0, 0, 360, color, 2, 8, 0);

	putTextCenter(l_mat, detection.definition->label(), object.detection_center,
		      CV_FONT_HERSHEY_SIMPLEX, 1,
		      cv::Scalar(39, 100, 206), 2);

	cv::rectangle(disparity_color,
		      cv::Point(object.disparity_center.x - 10 / 2,
                                object.disparity_center.y - 20 / 2),
		      cv::Point(object.disparity_center.x + 10 / 2,
                                object.disparity_center.y + 20 / 2),
		      color, 2);


      }
    }


    cv_bridge::CvImage disparity_msg;
    disparity_msg.header.stamp   = time;
    disparity_msg.header.frame_id   = l_camera_frame;
    disparity_msg.encoding = sensor_msgs::image_encodings::BGR8;
    disparity_msg.image    = disparity_color;
    disparity_pub_.publish(disparity_msg.toImageMsg());
    cv_bridge::CvImage image_msg;
    image_msg.header.stamp   = time;
    image_msg.header.frame_id   = l_camera_frame;
    image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    image_msg.image    = l_mat;
    image_pub_.publish(image_msg.toImageMsg());

    if(show_windows_){
      static boost::mutex display_mutex;
      boost::lock_guard<boost::mutex> lock(display_mutex);
      cv::imshow("image", l_mat);
      cv::imshow("disparity", disparity_color);
      cv::waitKey(3);
    }

  }

  float CascadeClassifier::average_disparity(const cv::Mat& disp, const cv::Point2d& pt, int width, int height) {
    int startx = pt.x - width/2;
    int starty = pt.y - height/2;
    int ctr = 0;
    float sum = 0.0;
    for (int i = 0; i < width; i++) {
      for (int j = 0; j < height; j++) {
	float value = disp.at<float>(starty + i, startx + j);
	if (value > 0.0){
	  sum = sum + value;
	  ctr++;
	}
      }
    }
    if(ctr == 0)
      return -1;
    return sum / (float) ctr;
  }


  void CascadeClassifier::putTextCenter(cv::Mat& img, const std::string& text, cv::Point org, int font_face, double scale, cv::Scalar color, int thickness){
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, font_face, scale, thickness, &baseline);
    cv::Point text_org(org.x - text_size.width/2,
		  org.y + text_size.height/2);
    cv::putText(img, text, text_org, font_face, scale, color, thickness);
  }



  std::vector<training_definition_ptr> CascadeClassifier::parse_training_definitions(XmlRpc::XmlRpcValue& training_descriptions){
    std::vector<training_definition_ptr> definitions;
    ROS_ASSERT(training_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < training_descriptions.size(); ++i) {
      XmlRpc::XmlRpcValue& training_description = training_descriptions[i];
      definitions.push_back(parse_training_definition(training_description));
    }
    return definitions;
  }

  training_definition_ptr CascadeClassifier::parse_training_definition(XmlRpc::XmlRpcValue& training_description){
    ROS_ASSERT(training_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(training_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(training_description["label"].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(training_description["cascade_path"].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(training_description["object_radius"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(training_description["scale_factor"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(training_description["min_neighbors"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(training_description["max_disparity_for_detection"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    return training_definition_ptr(new training_definition(
	(int)training_description["id"],
	(std::string)training_description["label"],
	(std::string)training_description["cascade_path"],
	(double)training_description["object_radius"],
	(double)training_description["scale_factor"],
	(int)training_description["min_neighbors"],
	parse_size(training_description["min_size"]),
	parse_size(training_description["max_size"]),
        (float)(double)training_description["max_disparity_for_detection"] ));
  }

  cv::Size CascadeClassifier::parse_size(XmlRpc::XmlRpcValue& size){
    ROS_ASSERT(size.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(size["width"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(size["height"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    return cv::Size((double)size["width"], (double)size["height"]);
  }


}
