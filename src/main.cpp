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

#include "pose_est/ImageBoundingBox.h"
pose_est::ImageBoundingBox bbox_msg;

using namespace std;

/* Global variables */
#define humanFrameID "human_detected"
#define fixedFrameID "camera_link"

ros::Subscriber bbox_sub;
ros::Subscriber pcd_sub;
ros::Publisher pose_pub;

vector<vector<float>> VectPose;

class HumanParameters {
public:
  float Xposition;
  float Yposition;
  float Zposition;
  float Velocity;
  float Time;
  pose_est::ImageBoundingBox Bbox;

  void addBbox(const pose_est::ImageBoundingBox bbox_msg) {
    Bbox = bbox_msg;
  }
  void addPose(const sensor_msgs::PointCloud2ConstPtr& pCloud, const ros::Time& RosTime) {
    /* Pixel coordinates in the point cloud */   
    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;    
    int u_px = Bbox.center.u + Bbox.width/2;
    int v_px = Bbox.center.v + Bbox.height/2;
    int arrayPosition = v_px*pCloud->row_step + u_px*pCloud->point_step;
    int arrayPosX = arrayPosition + pCloud->fields[0].offset;
    int arrayPosY = arrayPosition + pCloud->fields[1].offset;
    int arrayPosZ = arrayPosition + pCloud->fields[2].offset;

    memcpy(&X, &pCloud->data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud->data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud->data[arrayPosZ], sizeof(float));

    /* Pointcloud to plane projection */
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = RosTime;
    transformStamped.header.frame_id = fixedFrameID;
    transformStamped.child_frame_id = humanFrameID;
    transformStamped.transform.translation.x = X;
    transformStamped.transform.translation.y = Y;
    transformStamped.transform.translation.z = Z;
    tf2::Quaternion q;
    q.setRPY(-M_PI/2.0, M_PI/2.0, M_PI);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);

    Xposition = X;
    Yposition = Y;
    Zposition = Z;
    Time = RosTime.toSec();
  }
  
  void addVelocity(vector<vector<float>> VectPose) {
    /* Point velocity */
    float x1 = VectPose[0][0];
    float y1 = VectPose[0][1];
    float z1 = VectPose[0][2];
    float x2 = VectPose[1][0];
    float y2 = VectPose[1][1];
    float z2 = VectPose[1][2];
    float deltaT = VectPose[1][3] - VectPose[0][3]; 
    Velocity = sqrt(pow(x2-x1,2) + pow(y2-y1,2) + pow(z2-z1,2))/deltaT;
  }
};

void publish_point(const float X, const float Y, const float Z, const ros::Time& rosTime, const float Time, const float Velocity) {
  geometry_msgs::PoseStamped humanPose;
  humanPose.header.stamp = rosTime;
  humanPose.header.frame_id = humanFrameID;
  humanPose.pose.position.x = X; //red
  humanPose.pose.position.y = Y; //green
  humanPose.pose.position.z = Z; //blue
  
  /* Publish position */
  pose_pub.publish(humanPose);
  
}

void bboxCallback(const pose_est::ImageBoundingBox& msg_bbox) {
  HumanParameters Hp;
  Hp.addBbox(msg_bbox);
}

void pcdCallback(const sensor_msgs::PointCloud2ConstPtr& pCloud) { 
  HumanParameters Hp;
  ros::Time rostime = ros::Time::now();
  Hp.addPose(pCloud, rostime); 
  VectPose.push_back({Hp.Xposition, Hp.Yposition, Hp.Zposition, Hp.Time});
  if (VectPose.size() == 2) {
    Hp.addVelocity(VectPose);
      publish_point(Hp.Xposition, Hp.Yposition, Hp.Zposition, rostime, Hp.Time, Hp.Velocity);
      VectPose.erase(VectPose.begin());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;

  bbox_sub = nh.subscribe("/humanBBox",1000,bboxCallback);
  pcd_sub = nh.subscribe("/camera/depth_registered/points", 1000, pcdCallback);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("human/position", 1);
  
  while (ros::ok()) {
    ros::spinOnce();
  }
}
