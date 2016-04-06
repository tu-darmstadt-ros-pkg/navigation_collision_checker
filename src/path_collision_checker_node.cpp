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



#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Twist.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/server.h>
#include <navigation_collision_checker/PathCollisionCheckerConfig.h>

#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_proc/grid_map_polygon_tools.h>

#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Path.h>
#include <hector_worldmodel_msgs/AddObject.h>
#include <hector_worldmodel_msgs/PosePercept.h>

class PathCollisionChecker
{
public:

  PathCollisionChecker()
  {
    ros::NodeHandle nh_;

    dyn_rec_server_.setCallback(boost::bind(&PathCollisionChecker::dynRecParamCallback, this, _1, _2));

    //grid_map_polygon_tools::setFootprintPoly(0.2, 0.2, this->footprint_poly_);

    grid_map_polygon_tools::printPolyInfo(this->footprint_poly_);

    //traversability_map_sub_ = nh_.subscribe("/dynamic_map", 2, &PathCollisionChecker::traversabilityMapCallback, this);

    //safe_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_safe", 1, false);

    ros::NodeHandle pnh("~");
    marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("path_collision_check_markers", 1,false);

    debug_pose_ = pnh.advertise<geometry_msgs::PoseStamped>("debug_pose",5, false);

    pose_sub_ = pnh.subscribe("/robot_pose", 1, &PathCollisionChecker::poseCallback, this);

    local_elevation_map_sub_ = pnh.subscribe("/elevation_mapping/elevation_map", 1, &PathCollisionChecker::localElevationMapCallback, this);

    path_sub_ = pnh.subscribe("/smooth_path", 1, &PathCollisionChecker::pathCallback, this);


    add_object_client_ = pnh.serviceClient<hector_worldmodel_msgs::AddObject>("/worldmodel/add_object", false);
    pose_percept_publisher_ = pnh.advertise<hector_worldmodel_msgs::PosePercept>("/worldmodel/pose_percept", 5, false);
  }


  void traversabilityMapCallback(const nav_msgs::OccupancyGrid& msg)
  {
    // Update typically takes around 20ms on a i7 running sim+onboard
    ros::WallTime start_trav_update_time = ros::WallTime::now();
    grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "occupancy", traversability_map_);
    ROS_DEBUG("Traversability map update took %f seconds", (ros::WallTime::now()-start_trav_update_time).toSec());
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    robot_pose_ = msg;
  }

  void localElevationMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
  {
      grid_map::GridMapRosConverter::fromMessage(*msg, local_elevation_map_);
  }

  void pathCallback(const nav_msgs::Path& path)
  {
    
    if (p_path_check_enabled_){
      ros::WallTime start = ros::WallTime::now();

      if (!robot_pose_.get()){
        ROS_WARN("Robot pose not available, skipping path checking.");
        return;
      }

      geometry_msgs::PoseWithCovariance obstacle_pose;
      double robot_elevation = robot_pose_->pose.position.z + p_pose_height_offset_;
      double elevation_threshold = p_obstacle_diff_threshold_;

      bool is_in_collision = grid_map_polygon_tools::isPathInCollisionElevation(footprint_poly_,
                                                                                local_elevation_map_,
                                                                                path,
                                                                                robot_elevation,
                                                                                elevation_threshold,
                                                                                obstacle_pose.pose,
                                                                                p_path_min_travel_dist_,
                                                                                p_path_max_travel_dist_,
                                                                                0.3,
                                                                                "elevation");

      ROS_INFO("Collision checking took %f seconds", (ros::WallTime::now() - start).toSec());

      if (!is_in_collision){
        ROS_INFO("Path not in collision.");
        return;
      }

      if (debug_pose_.getNumSubscribers() > 0){
        geometry_msgs::PoseStamped pose_debug;
        pose_debug.pose = obstacle_pose.pose;

      }

      /*
    hector_worldmodel_msgs::AddObject add_obj_srv;

    add_obj_srv.request.object.header.frame_id = "world";
    add_obj_srv.request.object.header.stamp = ros::Time::now();

    add_obj_srv.request.object.info.class_id = "obstacle";
    add_obj_srv.request.object.state.state = hector_worldmodel_msgs::ObjectState::PENDING;


    add_obj_srv.request.object.pose = obstacle_pose;

    if (add_object_client_.call(add_obj_srv)){
      ROS_INFO("Added collision object!");
    }else{
      ROS_ERROR("Failed to add collision object!");
    }
    */

      hector_worldmodel_msgs::PosePercept pose_percept;

      pose_percept.header.frame_id = "world";
      pose_percept.header.stamp = ros::Time::now();
      pose_percept.info.class_id = "obstacle";
      pose_percept.info.object_support = 1.0;
      pose_percept.pose = obstacle_pose;

      pose_percept_publisher_.publish(pose_percept);
    }

  }

  
  
  void addMarker(const Eigen::Affine3d& pose, size_t count)
  {
    visualization_msgs::Marker marker;
    //marker.header.stamp = req.point.header.stamp;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;
    marker.id = count;

    marker.ns ="nav_coll_check";
    //marker.pose.orientation.w = 1.0;

    //tf::quaternionEigenToMsg(Eigen::Quaterniond(pose.rotation()), marker.pose.orientation);
    tf::poseEigenToMsg(pose, marker.pose);

    marker_array_.markers.push_back(marker);
  }


  void dynRecParamCallback(navigation_collision_checker::PathCollisionCheckerConfig &config, uint32_t level)
  {
    p_obstacle_diff_threshold_ = config.elev_diff_threshold;
    p_pose_height_offset_      = config.pose_height_offset;

    grid_map_polygon_tools::setFootprintPoly(config.footprint_x, config.footprint_y, this->footprint_poly_);

    p_path_min_travel_dist_ = config.path_min_travel_dist;
    p_path_max_travel_dist_ = config.path_max_travel_dist;
    
    p_path_check_enabled_ = config.path_check_enabled;
  }

protected:
  
  ros::Subscriber traversability_map_sub_;

  ros::Publisher marker_pub_;
  ros::Publisher collision_state_pub_;

  ros::Publisher debug_pose_;

  ros::Publisher debug_cloud_pub_;

  ros::Publisher cmd_vel_pub_;

  ros::Subscriber local_elevation_map_sub_;
  ros::Subscriber path_sub_;



  ros::Subscriber pose_sub_;

  //Worldmodel interface
  ros::ServiceClient add_object_client_;
  ros::Publisher pose_percept_publisher_;

  //boost::shared_ptr<tf::TransformListener> tfl_;
  


  ros::Timer check_timer_;


  //! Fused elevation map as grid map.
  grid_map::GridMap local_elevation_map_;

  geometry_msgs::PoseStampedConstPtr robot_pose_;



  grid_map::GridMap traversability_map_;
  grid_map::Polygon footprint_poly_;

  visualization_msgs::MarkerArray marker_array_;

  dynamic_reconfigure::Server<navigation_collision_checker::PathCollisionCheckerConfig> dyn_rec_server_;

  double p_obstacle_diff_threshold_;
  double p_pose_height_offset_;
  double p_footprint_x;
  double p_footprint_y;
  double p_path_min_travel_dist_;
  double p_path_max_travel_dist_;
  bool p_path_check_enabled_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_collision_checker_node");

  PathCollisionChecker ls;

  ros::spin();
}
