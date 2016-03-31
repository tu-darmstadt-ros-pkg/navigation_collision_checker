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
#include <moveit_msgs/DisplayRobotState.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/server.h>
#include <navigation_collision_checker/NavCollisionCheckerConfig.h>

#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_proc/grid_map_polygon_tools.h>


#include <vigir_worldmodel_server/point_cloud/point_cloud_aggregator.h>


class NavCollisionChecker
{
public:

  NavCollisionChecker()
    : estimated_state_in_collision_(false)
  {
    ros::NodeHandle nh_;

    dyn_rec_server_.setCallback(boost::bind(&NavCollisionChecker::dynRecParamCallback, this, _1, _2));

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

    grid_map_polygon_tools::setFootprintPoly(0.4, 0.3, this->footprint_poly_);

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


    //desired_twist_sub_ = nh_.subscribe("cmd_vel_raw", 1, &NavCollisionChecker::twistCallback, this);





    traversability_map_sub_ = nh_.subscribe("/dynamic_map", 2, &NavCollisionChecker::traversabilityMapCallback, this);

    safe_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_safe", 1, false);

    ros::NodeHandle pnh("~");
    marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("nav_collision_check_markers", 1,false);
    collision_state_pub_ = pnh.advertise<moveit_msgs::DisplayRobotState>("in_collision_state", 1, false);
    debug_cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("fast_coll_debug_cloud", 1, false);


    check_timer_ = pnh.createTimer(ros::Duration(0.05), &NavCollisionChecker::checkTimerCallback, this, false);


    tfl_.reset(new tf::TransformListener());

    cloud_aggregator_.reset(new vigir_worldmodel::PointCloudAggregator<pcl::PointXYZI>(tfl_, 500));

    cloud_sub_ = nh_.subscribe("/scan_cloud_filtered", 30, &NavCollisionChecker::filteredCloudCallback, this);

    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>("cmd_vel_raw", 1,
          boost::bind(&NavCollisionChecker::twistCallback, this, _1),
          ros::VoidPtr(), &queue_);

    desired_twist_sub_ = nh_.subscribe(so);

    this->callback_queue_thread_ =
      boost::thread(boost::bind(&NavCollisionChecker::QueueThread, this));
  }

  void octomapCallback(const octomap_msgs::OctomapConstPtr msg)
  {
    // Octomap update typically takes around 5ms on a i7 running sim+onboard
    ros::WallTime start_octo_update_time = ros::WallTime::now();
    planning_scene_->processOctomapMsg(*msg);
    ROS_DEBUG("Octomap update took %f seconds", (ros::WallTime::now()-start_octo_update_time).toSec());
  }

  void filteredCloudCallback(const sensor_msgs::PointCloud2& msg)
  {
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > pc_ (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(msg, *pc_);
    cloud_aggregator_->addCloud(pc_);
    //ROS_INFO("cloud agg size: %d", (int)cloud_aggregator_->size());

  }

  void robotPoseCallback(const geometry_msgs::PoseStampedConstPtr msg)
  {
    robot_pose_ptr_ = msg;
  }

  void jointStatesCallback(const sensor_msgs::JointStateConstPtr msg)
  {
    moveit::core::jointStateToRobotState(*msg, *robot_state_);
  }

  void traversabilityMapCallback(const nav_msgs::OccupancyGrid& msg)
  {
    // Update typically takes around 20ms on a i7 running sim+onboard
    ros::WallTime start_trav_update_time = ros::WallTime::now();
    grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "occupancy", traversability_map_);
    ROS_DEBUG("Traversability map update took %f seconds", (ros::WallTime::now()-start_trav_update_time).toSec());
  }

  void QueueThread() {
      static const double timeout = 0.01;

      while (ros::ok()) {
        queue_.callAvailable(ros::WallDuration(timeout));
      }
    }

  void twistCallback(const geometry_msgs::TwistConstPtr msg)
  {
    bool in_collision = false;

    {
      boost::mutex::scoped_lock scoped_lock(collision_state_lock_);
      in_collision = estimated_state_in_collision_;
      latest_twist_ = msg;
    }


    if (p_pass_through_){
      geometry_msgs::Twist twist_out = *msg;
      safe_twist_pub_.publish(twist_out);
      return;
    }


    if (in_collision){
      geometry_msgs::Twist twist_out;
      twist_out.linear.x = 0.0;
      twist_out.linear.y = 0.0;
      twist_out.linear.z = 0.0;

      twist_out.angular.x = 0.0;
      twist_out.angular.y = 0.0;
      twist_out.angular.z = 0.0;

      safe_twist_pub_.publish(twist_out);

    }else{
      geometry_msgs::Twist twist_out = *msg;
      safe_twist_pub_.publish(twist_out);
    }


  }


  void checkTimerCallback(const ros::TimerEvent& event)
  {

    if (!robot_pose_ptr_.get()){
      ROS_WARN_THROTTLE(3.0, "Cannot get robot pose. Cannot compute collision check. This message is throttled.");
      return;
    }

    geometry_msgs::Twist latest_twist_cpy;

    {
      boost::mutex::scoped_lock scoped_lock(collision_state_lock_);
      if (!latest_twist_.get()){
        ROS_WARN_THROTTLE(3.0, "Cannot get latest twist msg. Cannot compute collision check. This message is throttled.");
        return;
      }
      latest_twist_cpy = *latest_twist_;
    }

    double step_time = p_roll_out_step_time_;

    Eigen::Affine3d test_pose;
    tf::poseMsgToEigen(robot_pose_ptr_->pose, test_pose);

    Eigen::Affine3d pose_change (this->integrateTwist(latest_twist_cpy, step_time));

    marker_array_.markers.clear();

    if (p_use_filtered_cloud_collision_avoidance){
      if (this->isInCollisionFilteredCloud(latest_twist_cpy)){
        {
          boost::mutex::scoped_lock scoped_lock(collision_state_lock_);
          estimated_state_in_collision_ = true;
        }
        return;
      }
    }

    for (size_t i = 0; i < p_roll_out_steps_; ++i){
      test_pose = test_pose * pose_change;
      this->addMarker(test_pose, i);
      //bool in_collision = isInCollisionOcto(test_pose);
      bool in_collision = false;

      if (p_use_traversability_map_collision_avoidance){
        in_collision = isInCollisionTraversabilityMap(test_pose);
      }

      if (in_collision){

        marker_pub_.publish(marker_array_);

        {
          boost::mutex::scoped_lock scoped_lock(collision_state_lock_);
          estimated_state_in_collision_ = true;
        }

        return;
      }
    }

    {
      boost::mutex::scoped_lock scoped_lock(collision_state_lock_);
      estimated_state_in_collision_ = false;
    }


    marker_pub_.publish(marker_array_);
    //safe_twist_pub_.publish(twist_out);
  }

  bool isInCollisionOcto(const Eigen::Affine3d& pose)
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
    //ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " self collision. Distance: " << collision_result.distance);

    if (collision_result.collision){
      if (collision_state_pub_.getNumSubscribers() > 0){
        moveit_msgs::DisplayRobotState collision_robot_state;

        moveit::core::robotStateToRobotStateMsg(*robot_state_, collision_robot_state.state);
        collision_state_pub_.publish(collision_robot_state);
      }

      collision_detection::CollisionResult::ContactMap& contacts = collision_result.contacts;
      ROS_INFO_THROTTLE(1.0, "Detected %d collisions. This message is throttled.", (int)contacts.size());
      return true;
    }

    return false;
  }

  bool isInCollisionTraversabilityMap(const Eigen::Affine3d& pose)
  {
    if (!traversability_map_.exists("occupancy")){
      ROS_WARN("Occupancy layer of traversability map not (yet) available, skipping check.");
      return false;
    }

    const grid_map::Polygon pose_footprint = grid_map_polygon_tools::getTransformedPoly(footprint_poly_, pose);

    grid_map::Matrix& traversability_data = traversability_map_["occupancy"];

    for (grid_map::PolygonIterator poly_iterator(traversability_map_, pose_footprint); !poly_iterator.isPastEnd(); ++poly_iterator) {
      const grid_map::Index index(*poly_iterator);

      //std::cout << "idx: " << index(0) << " " << index(1) << "\n";

      if (traversability_data(index(0), index(1)) > 50.0){
        return true;
      }
    }
    return false;
  }


  bool isInCollisionFilteredCloud(const geometry_msgs::Twist& twist)
  {
    geometry_msgs::Point point_min;
    geometry_msgs::Point point_max;


    point_min.y = -p_reactive_lidar_avoid_y_side;
    point_min.z =  p_reactive_lidar_avoid_z_min;

    point_max.y = -point_min.y;
    point_max.z =  point_min.z + p_reactive_lidar_avoid_z_range;
    
    if (twist.linear.x > 0.0){
      point_min.x =  p_reactive_lidar_avoid_x_min;
      point_max.x =  p_reactive_lidar_avoid_x_min + p_reactive_lidar_avoid_x_range;
    }else if (twist.linear.x < 0.0){
      point_max.x = -p_reactive_lidar_avoid_x_min;
      point_min.x = -p_reactive_lidar_avoid_x_min - p_reactive_lidar_avoid_x_range;
    }else{
      //No forward speed, return immediately
      return false;
    }



    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > cloud_base_link (new pcl::PointCloud<pcl::PointXYZI>());

    cloud_aggregator_->getAggregateCloudBbxFiltered(cloud_base_link, "base_link", point_min, point_max, 0.0, p_reactive_aggregation_size);

    //cloud_aggregator_->getAggregateCloud(cloud_base_link, "base_link", p_reactive_aggregation_size);

    // Publish cloud for debugging if requested
    if (debug_cloud_pub_.getNumSubscribers() > 0){
      sensor_msgs::PointCloud2 cloud_out;
      pcl::toROSMsg(*cloud_base_link, cloud_out);

      cloud_out.header.stamp = ros::Time::now();
      cloud_out.header.frame_id = "base_link";

      debug_cloud_pub_.publish (cloud_out);
    }

    if (cloud_base_link->size() > p_reactive_min_number_for_obstacles){
      return true;
    }

    return false;
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

    //std::cout << "\n" << int_vec << "\n";

    return Eigen::AngleAxisd(int_vec.z(), Eigen::Vector3d::UnitZ()) *
                             Eigen::Translation3d(int_vec.x(),
                                                  int_vec.y(),
                                                  0.0);
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

  void dynRecParamCallback(navigation_collision_checker::NavCollisionCheckerConfig &config, uint32_t level)
  {
    p_roll_out_step_time_ = config.roll_out_step_time;
    p_roll_out_steps_ = config.roll_out_steps;

    p_reactive_lidar_avoid_x_min    = config.reactive_lidar_avoid_x_min;
    p_reactive_lidar_avoid_x_range  = config.reactive_lidar_avoid_x_range;
    p_reactive_lidar_avoid_z_min    = config.reactive_lidar_avoid_z_min;
    p_reactive_lidar_avoid_z_range  = config.reactive_lidar_avoid_z_range;
    p_reactive_lidar_avoid_y_side   = config.reactive_lidar_avoid_y_side;

    p_reactive_aggregation_size = config.reactive_aggregation_size;
    p_reactive_min_number_for_obstacles = config.reactive_min_number_for_obstacles;

    p_use_traversability_map_collision_avoidance = config.use_traversability_map_collision_avoidance;
    p_use_filtered_cloud_collision_avoidance = config.use_filtered_cloud_collision_avoidance;

    ROS_INFO("Set ranges x: %f x_r: %f, z: %f z_r: %f y: %f",
             p_reactive_lidar_avoid_x_min,
             p_reactive_lidar_avoid_x_range,
             p_reactive_lidar_avoid_z_min,
             p_reactive_lidar_avoid_z_range,
             p_reactive_lidar_avoid_y_side);

    p_pass_through_ = config.pass_through;
  }

protected:
  ros::Subscriber octomap_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber desired_twist_sub_;
  ros::Subscriber joint_state_sub_;

  ros::Subscriber traversability_map_sub_;

  ros::Publisher safe_twist_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher collision_state_pub_;

  ros::Publisher debug_cloud_pub_;

  boost::shared_ptr<tf::TransformListener> tfl_;
  ros::Duration wait_duration_;

  boost::shared_ptr<vigir_worldmodel::PointCloudAggregator<pcl::PointXYZI> > cloud_aggregator_;

  ros::Subscriber cloud_sub_;

  ros::Timer check_timer_;

  geometry_msgs::TwistConstPtr latest_twist_;


  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  planning_scene::PlanningScenePtr planning_scene_;

  geometry_msgs::PoseStampedConstPtr robot_pose_ptr_;
  sensor_msgs::JointState virtual_link_joint_states_;

  grid_map::GridMap traversability_map_;
  grid_map::Polygon footprint_poly_;

  visualization_msgs::MarkerArray marker_array_;

  dynamic_reconfigure::Server<navigation_collision_checker::NavCollisionCheckerConfig> dyn_rec_server_;

  double p_roll_out_step_time_;
  int p_roll_out_steps_;
  bool p_pass_through_;

  double p_reactive_lidar_avoid_x_min;
  double p_reactive_lidar_avoid_x_range;
  double p_reactive_lidar_avoid_z_min;
  double p_reactive_lidar_avoid_z_range;
  double p_reactive_lidar_avoid_y_side;

  int p_reactive_aggregation_size;
  int p_reactive_min_number_for_obstacles;

  bool p_use_traversability_map_collision_avoidance;
  bool p_use_filtered_cloud_collision_avoidance;


  bool estimated_state_in_collision_;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  boost::mutex collision_state_lock_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_collision_checker_node");

  NavCollisionChecker ls;

  ros::spin();
}
