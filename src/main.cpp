#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "bbox_position/ImageBoundingBox.h"

using namespace std;

#define humanFrameID "human_detected"
#define fixedFrameID "base_link"

string bbox_topic;

ros::Time bbox_time;

class PoseEstimator {

  ros::NodeHandle nh_;
  ros::Subscriber hog_sub_;
  ros::Subscriber pcd_sub_;
  ros::Publisher pose_pub_;
  bool ValidPose = false;
  int u_px, v_px;
  
public:
  PoseEstimator() {
    hog_sub_ = nh_.subscribe(bbox_topic, 1000, &PoseEstimator::hogCallback, this);
    pcd_sub_ = nh_.subscribe("/kinect2/qhd/points", 1000, &PoseEstimator::pcdCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pedestrian/position", 1);
  }
  void hogCallback(const bbox_position::ImageBoundingBox& bbox_msg) {
    if (isnan(bbox_msg.cornerPoints[0].u) == false) {
      u_px = bbox_msg.cornerPoints[0].u + bbox_msg.width/2;
      v_px = bbox_msg.cornerPoints[0].v + bbox_msg.height/2;
      if (u_px!=0 && v_px!=0)
	ValidPose = true;
      else
	ValidPose = false;
    }
  }

  void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& pCloud) {

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;
    
    if (ValidPose == true) {
      int arrayPosition = v_px*pCloud->row_step + u_px*pCloud->point_step;
      int arrayPosX = arrayPosition + pCloud->fields[0].offset;
      int arrayPosY = arrayPosition + pCloud->fields[1].offset;
      int arrayPosZ = arrayPosition + pCloud->fields[2].offset;

      memcpy(&X, &pCloud->data[arrayPosX], sizeof(float));
      memcpy(&Y, &pCloud->data[arrayPosY], sizeof(float));
      memcpy(&Z, &pCloud->data[arrayPosZ], sizeof(float));

      static tf2_ros::TransformBroadcaster br;
      geometry_msgs::TransformStamped transformStamped;
      geometry_msgs::PoseStamped humanPose;

      ros::Time nau = ros::Time::now();
      transformStamped.header.stamp = nau;
      transformStamped.header.frame_id = fixedFrameID; 
      transformStamped.child_frame_id = humanFrameID;
      transformStamped.transform.translation.x = X;
      transformStamped.transform.translation.y = Z;
      transformStamped.transform.translation.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();
      br.sendTransform(transformStamped);

      humanPose.header.stamp = bbox_time;
      humanPose.header.frame_id = fixedFrameID; 
      humanPose.pose.position.x = X; 
      humanPose.pose.position.y = Z; 
      humanPose.pose.position.z = 0; 
      humanPose.pose.orientation.x = q.x();
      humanPose.pose.orientation.y = q.y();
      humanPose.pose.orientation.z = q.z();
      humanPose.pose.orientation.w = q.w();
      pose_pub_.publish(humanPose);
      ValidPose = false;
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;
  nh.getParam("/bbox_topic", bbox_topic);
  PoseEstimator ic;
  ros::spin ();
  return 0;
}
