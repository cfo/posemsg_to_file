/*
 * record.cpp
 *
 *  Created on: Jan 15, 2013
 *      Author: cforster
 */

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sys/time.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <vikit/params_helper.h>
#include <vikit/timer.h>

namespace {

class Recorder 
{
public:
  std::ofstream ofs_;   
  int n_msgs_received_;  
  bool invert_pose_;     
  tf::TransformListener tf_listener_;
  Eigen::Quaterniond q_;  
  Eigen::Vector3d p_;    
  double stamp_;          

  Recorder(std::string filename, bool invert_pose) :
    n_msgs_received_(0),
    invert_pose_(invert_pose),
    tf_listener_(ros::Duration(100))
  {
    ofs_.open(filename.c_str());
    if(ofs_.fail())
      throw std::runtime_error("Could not create tracefile. Does folder exist?");

    ofs_ << "# format: timestamp tx ty tz qx qy qz qw" << std::endl;
  }

  ~Recorder() {}

  void write()
  {
    if(invert_pose_)
    {
      Eigen::Matrix3d R = q_.toRotationMatrix().transpose();
      p_ = -R*p_;
      q_ = Eigen::Quaterniond(R);
    }

    ofs_.precision(15);
    ofs_.setf(std::ios::fixed, std::ios::floatfield );
    ofs_ << stamp_ << " ";
    ofs_.precision(6);
    ofs_ << p_.x() << " " << p_.y() << " " << p_.z() << " "
         << q_.x() << " " << q_.y() << " " << q_.z() << " " << q_.w() << std::endl;

    if(++n_msgs_received_ % 50 == 0)
      printf("received %i pose messages.\n", n_msgs_received_);
  }

  void poseCallback(const geometry_msgs::PoseStampedPtr& msg)
  {
    q_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                            msg->pose.orientation.y, msg->pose.orientation.z);
    p_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    stamp_ = msg->header.stamp.toSec();
    write();
  }

  void poseCovCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
  {
    q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    p_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    stamp_ = msg->header.stamp.toSec();
    write();
  }

  void transformStampedCallback(const geometry_msgs::TransformStampedPtr& msg)
  {
    q_ = Eigen::Quaterniond(msg->transform.rotation.w, msg->transform.rotation.x,
                            msg->transform.rotation.y, msg->transform.rotation.z);
    p_ = Eigen::Vector3d(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
    stamp_ = msg->header.stamp.toSec();
    write();
  }

  void tfCallback(const std::string& topic, const std::string& topic_ref)
  {
    tf::StampedTransform tf_transform;
    ros::Time now(ros::Time::now());
    try
    {
      tf_listener_.waitForTransform(topic, topic_ref, now, ros::Duration(2.0));
      tf_listener_.lookupTransform(topic, topic_ref, now, tf_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("tfCallback: %s", ex.what());
    }

    Eigen::Affine3d eigen_transform;
    tf::transformTFToEigen(tf_transform, eigen_transform);
    q_ = Eigen::Quaterniond(eigen_transform.rotation());
    p_ = eigen_transform.translation();
    stamp_ = now.toSec();
    write();
  }
};

} // namespace

int main(int argc, char** argv)
{
  // create ros node
  ros::init(argc, argv, "trajectory_recorder");
  ros::NodeHandle nh;

  // get parameters to subscribe
  std::string topic(vk::getParam<std::string>("posemsg_to_file/topic"));
  std::string topic_type(vk::getParam<std::string>("posemsg_to_file/topic_type"));
  std::string topic_ref(vk::getParam<std::string>("posemsg_to_file/topic_ref", ""));
  bool invert_pose(vk::getParam<bool>("posemsg_to_file/invert_pose", false));

  // generate filename
  std::string topic_name(topic);
  replace(topic_name.begin(), topic_name.end(), '/', '_');
  std::string filename(ros::package::getPath("posemsg_to_file")+"/logs/"+topic_name+".txt");

  // start recorder
  Recorder recorder(filename, invert_pose);

  // subscribe to topic
  ros::Subscriber sub;
  if(topic_type == std::string("PoseWithCovarianceStamped"))
    sub = nh.subscribe(topic, 10, &Recorder::poseCovCallback, &recorder);
  else if (topic_type == std::string("PoseStamped"))
    sub = nh.subscribe(topic, 10, &Recorder::poseCallback, &recorder);
  else if (topic_type == std::string("TransformStamped"))
    sub = nh.subscribe(topic, 10, &Recorder::transformStampedCallback, &recorder);
  else if (topic_type == std::string("tf")) {
    if(topic_ref.empty())
      throw std::runtime_error("no tf reference topic specified.");
  }
  else
    throw std::runtime_error("specified topic_type is not supported.");

  // spin
  ros::Rate r(500);
  while(ros::ok())
  {
    ros::spinOnce();
    if(topic_type == std::string("tf"))
      recorder.tfCallback(topic, topic_ref);
    r.sleep();
  }
  return 0;
}
