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

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/crop_box.h>

//#include <pcl_conversions/pcl_conversions.h>

//#include <pcl_ros/transforms.h>

//#include <filters/filter_chain.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Twist.h>

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

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    if (!robot_model_.get()){
      ROS_ERROR("Couldn't load robot model, exiting!");
      exit(0);
    }

    robot_state_.reset(new robot_state::RobotState(robot_model_));

    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

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

    isInCollision(*robot_pose_ptr_);

    safe_twist_pub_.publish(twist_out);
  }

  bool isInCollision(const geometry_msgs::PoseStamped& pose)
  {
    virtual_link_joint_states_.position[0] = pose.pose.position.x;
    virtual_link_joint_states_.position[1] = pose.pose.position.y;
    virtual_link_joint_states_.position[2] = pose.pose.position.z;
    virtual_link_joint_states_.position[3] = pose.pose.orientation.x;
    virtual_link_joint_states_.position[4] = pose.pose.orientation.y;
    virtual_link_joint_states_.position[5] = pose.pose.orientation.z;
    virtual_link_joint_states_.position[6] = pose.pose.orientation.w;

    moveit::core::jointStateToRobotState(virtual_link_joint_states_, *robot_state_);

    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_request.max_contacts = 100;
    collision_request.distance = true;
    collision_detection::CollisionResult collision_result;

    planning_scene_->checkCollision(collision_request, collision_result, *robot_state_, planning_scene_->getAllowedCollisionMatrix());
    ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
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
