//=================================================================================================
// Copyright (c) 2015, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

class NavCollisionChecker
{
public:

  NavCollisionChecker()
  {
    ros::NodeHandle nh_;

    virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_x");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_y");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_z");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_x");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_y");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_z");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_w");

    virtual_link_joint_states_.position.resize(7);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    if (!robot_model_.get()){
      ROS_ERROR("Couldn't load robot model, exiting!");
      exit(0);
    }

    robot_state_.reset(new robot_state::RobotState(robot_model_));

    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

    //Only check for collisions between robot and environment, not for self collisions
    collision_detection::AllowedCollisionMatrix& acm = planning_scene_->getAllowedCollisionMatrixNonConst();
    acm.setEntry(robot_model_->getLinkModelNames(), robot_model_->getLinkModelNames(), true);

    octomap_sub_ = nh_.subscribe("octomap", 2, &NavCollisionChecker::octomapCallback, this);
    robot_pose_sub_ = nh_.subscribe("robot_pose", 1, &NavCollisionChecker::robotPoseCallback, this);
    joint_state_sub_ = nh_.subscribe("joint_states", 5, &NavCollisionChecker::jointStatesCallback, this);
    desired_twist_sub_ = nh_.subscribe("cmd_vel_raw", 1, &NavCollisionChecker::twistCallback, this);

    safe_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_safe", 1, false);

    /*
    prior_roll_angle_ = 0.0;

    point_cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out",10,false);
    prox_warning_pub_ = nh_.advertise<std_msgs::Bool>("proximity_warning",10,false);

    scan_sub_ = nh_.subscribe("cloud", 10, &NavCollisionChecker::cloudCallback, this);


    crop_box_.reset(new pcl::CropBox<pcl::PointXYZ>());

    ros::NodeHandle pnh_("~");

    double x_max, x_min, y_max, y_min, z_max, z_min;
    pnh_.param("x_max", x_max, 1.0);
    pnh_.param("x_min", x_min, 1.0);
    pnh_.param("y_max", y_max, 1.0);
    pnh_.param("y_min", y_min, 1.0);
    pnh_.param("z_max", z_max, 1.0);
    pnh_.param("z_min", z_min, 1.0);

    //filter_chain_.configure("scan_filter_chain", pnh_);

    pnh_.param("target_frame", p_target_frame_, std::string("NO_TARGET_FRAME_SPECIFIED"));

    */

  }

  void octomapCallback(const octomap_msgs::OctomapConstPtr msg)
  {
    ros::WallTime start_octo_update_time = ros::WallTime::now();
    planning_scene_->processOctomapMsg(*msg);
    ROS_DEBUG("Octomap update took %f seconds", (ros::WallTime::now()-start_octo_update_time).toSec());
  }

  void robotPoseCallback(const geometry_msgs::PoseStampedConstPtr msg)
  {
    robot_pose_ptr_ = msg;
  }

  void jointStatesCallback(const sensor_msgs::JointStateConstPtr msg)
  {
    moveit::core::jointStateToRobotState(*msg, *robot_state_);
  }

  void twistCallback(const geometry_msgs::TwistConstPtr msg)
  {
    geometry_msgs::Twist twist_out = *msg;

    if (!robot_pose_ptr_.get()){
      ROS_WARN_THROTTLE(3.0, "Cannot get robot pose. Forwarding velocity command without safety check! This message is throttled.");
      safe_twist_pub_.publish(twist_out);
      return;
    }

    double step_time = 0.5;

    Eigen::Affine3d test_pose;
    tf::poseMsgToEigen(robot_pose_ptr_->pose, test_pose);

    Eigen::Affine3d pose_change = this->integrateTwist(*msg, step_time);

    for (size_t i = 0; i < 3; ++i){
      test_pose = pose_change * test_pose;
      bool in_collision = isInCollision(test_pose);

      if (in_collision){
        twist_out.linear.x = 0.0;
        twist_out.linear.y = 0.0;
        twist_out.linear.z = 0.0;

        twist_out.angular.x = 0.0;
        twist_out.angular.y = 0.0;
        twist_out.angular.z = 0.0;

        safe_twist_pub_.publish(twist_out);
        return;
      }
    }

    safe_twist_pub_.publish(twist_out);
  }

  bool isInCollision(const Eigen::Affine3d& pose)
  {
    virtual_link_joint_states_.position[0] = pose.translation().x();
    virtual_link_joint_states_.position[1] = pose.translation().y();
    virtual_link_joint_states_.position[2] = pose.translation().z();

    Eigen::Quaterniond quat(pose.rotation());

    virtual_link_joint_states_.position[3] = quat.x();
    virtual_link_joint_states_.position[4] = quat.y();
    virtual_link_joint_states_.position[5] = quat.z();
    virtual_link_joint_states_.position[6] = quat.w();

    moveit::core::jointStateToRobotState(virtual_link_joint_states_, *robot_state_);

    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_request.max_contacts = 100;
    collision_request.distance = true;
    collision_detection::CollisionResult collision_result;

    planning_scene_->checkCollision(collision_request, collision_result, *robot_state_, planning_scene_->getAllowedCollisionMatrix());
    ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " self collision. Distance: " << collision_result.distance);
  }

  //For now, this assumes only angular rate in z and linear vel in x
  Eigen::Affine3d integrateTwist(const geometry_msgs::Twist& msg, double step_time)
  {
    //This holds x,y and theta
    Eigen::Vector3d int_vec(Eigen::Vector3d::Zero());

    if (std::abs(msg.angular.z) < 0.0001){
      int_vec.x() = msg.linear.x * step_time;
    }else{

      double dist_change = msg.linear.x * step_time;
      double angle_change = msg.angular.z * step_time;

      double arc_radius = dist_change / angle_change;

      int_vec.x() = std::sin(angle_change) * arc_radius;
      int_vec.y() = arc_radius - std::cos(angle_change) * arc_radius;
      int_vec.z() = angle_change;
    }

    return Eigen::AngleAxisd(int_vec.z(), Eigen::Vector3d::UnitZ()) *
                             Eigen::Translation3d(int_vec.x(),
                                                  int_vec.y(),
                                                  0.0);
  }

protected:
  ros::Subscriber octomap_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber desired_twist_sub_;
  ros::Subscriber joint_state_sub_;

  ros::Publisher safe_twist_pub_;

  boost::shared_ptr<tf::TransformListener> tfl_;
  ros::Duration wait_duration_;


  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  planning_scene::PlanningScenePtr planning_scene_;

  geometry_msgs::PoseStampedConstPtr robot_pose_ptr_;

  sensor_msgs::JointState virtual_link_joint_states_;

  //bool p_use_high_fidelity_projection_;
  //std::string p_target_frame_;

  //double prior_roll_angle_;


  //sensor_msgs::PointCloud2 cloud2_;

  //std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > > cloud_agg_;

  //boost::shared_ptr<pcl::CropBox<pcl::PointXYZ> > crop_box_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_collision_checker_node");

  NavCollisionChecker ls;

  ros::spin();
}
