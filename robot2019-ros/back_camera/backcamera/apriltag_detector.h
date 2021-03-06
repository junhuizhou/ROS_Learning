#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <image_transport/image_transport.h>

#include <AprilTags/TagDetector.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace apriltags_ros{

class AprilTagDescription{
 public:
  AprilTagDescription(int id, double size, std::string &frame_name):id_(id), size_(size), frame_name_(frame_name){}
  double size(){return size_;}
  int id(){return id_;} 
  std::string& frame_name(){return frame_name_;} 
 private:
  int id_;
  double size_;
  std::string frame_name_;
};


class AprilTagDetector{
 public:

  AprilTagDetector(ros::NodeHandle& nh);
  ~AprilTagDetector();
  image_transport::CameraSubscriber image_sub_;
  image_transport::ImageTransport it_;
  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
  
  
  ros::Publisher initialpose_pub_; 
  ros::Subscriber amcl_pose_sub_;
  tf::TransformListener listener;
  int tag_id;
  int tag_detect_amount_;
  unsigned int enemy_color_;
  bool is_pose_publish_;
 
 private:
 // void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
  std::map<int, AprilTagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);

 private:
  std::map<int, AprilTagDescription> descriptions_;
  std::string sensor_frame_id_;
  //image_transport::ImageTransport it_;
  //image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher detections_pub_;
  ros::Publisher pose_pub_;
  //ros::Publisher initialpose_pub_;
  ros::Publisher camera_info_pub;
  tf::TransformBroadcaster tf_pub_;
  boost::shared_ptr<AprilTags::TagDetector> tag_detector_;
  bool projected_optics_;

  public:
  double delta_x ; 
  double delta_y ; 
  double delta_theta ;

};



}


#endif
