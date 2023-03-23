#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "custom_uv_msgs/ImageBoundingBox.h"
custom_uv_msgs::ImageBoundingBox bbox_msg;

using namespace std;

/* Global variables */
#define humanFrameID "human_detected"
#define fixedFrameID "base_link"

ros::Subscriber bbox_sub;
ros::Subscriber pcd_sub;
ros::Publisher pose_pub;
ros::Publisher base_pub;

vector<vector<float>> VectPose;
bool ValidPose = false;
int u_px, v_px;
ros::Time bbox_time;

void bboxCallback(const custom_uv_msgs::ImageBoundingBox& msg_bbox) { 
  if (isnan(msg_bbox.cornerPoints[0].u) == false) {
    u_px = msg_bbox.cornerPoints[0].u + msg_bbox.width/2;
    v_px = msg_bbox.cornerPoints[0].v + msg_bbox.height/2;
    bbox_time = msg_bbox.header.stamp;
    ValidPose = true;
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
    transformStamped.header.stamp = bbox_time;
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
    pose_pub.publish(humanPose);

  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;

  bbox_sub = nh.subscribe("/humanBBox",1000,bboxCallback);
  pcd_sub = nh.subscribe("/kinect2/qhd/points", 1000, pcdCallback);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/human/position", 1);
  base_pub = nh.advertise<geometry_msgs::PoseStamped>("/human/test_pose",1);
  while (ros::ok()) {
    ros::spinOnce();
  }
}
