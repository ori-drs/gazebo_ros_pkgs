/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to move a robot on
 *       the xy plane.
 * Author: Piyush Khandelwal
 * Date: 29 July 2013
 */

#include <gazebo_plugins/gazebo_ros_planar_move.h>
#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo
{

  GazeboRosPlanarMove::GazeboRosPlanarMove() {}

  GazeboRosPlanarMove::~GazeboRosPlanarMove() {}

  // Load the controller
  void GazeboRosPlanarMove::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf)
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace"))
    {
      ROS_INFO_NAMED("planar_move", "PlanarMovePlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else
    {
      robot_namespace_ =
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), command_topic_.c_str());
    }
    else
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_topic_.c_str());
    }
    else
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    }
    else
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    }
    else
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    }
    cmd_timeout_ = -1;
    if (!sdf->HasElement("cmdTimeout"))
    {
      ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <cmdTimeout>, "
          "defaults to %f",
          robot_namespace_.c_str(), cmd_timeout_);
    }
    else
    {
      cmd_timeout_ = sdf->GetElement("cmdTimeout")->Get<double>();
    }

#if GAZEBO_MAJOR_VERSION >= 8
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
#else
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
#endif
#if GAZEBO_MAJOR_VERSION >= 8
    last_odom_pose_ = parent_->WorldPose();
#else
    last_odom_pose_ = parent_->GetWorldPose().Ign();
#endif
    x_ = 0;
    y_ = 0;
    rot_ = 0;
    linear_cmd_ = ignition::math::Vector3d();
    angular_cmd_ = ignition::math::Vector3d();
    last_linear_cmd_ = ignition::math::Vector3d();
    last_angular_cmd_ = ignition::math::Vector3d();
    alive_ = true;
    recover_roll_velocity_p_gain_  = 100; /*10.0;*/
    recover_pitch_velocity_p_gain_ = 100; /*10.0;*/
    recover_z_velocity_p_gain_ = 1000; /*1.0;*/

    if (sdf->HasElement("recover_roll_velocity_p_gain"))
      (sdf->GetElement("recover_roll_velocity_p_gain")->GetValue()->Get(recover_roll_velocity_p_gain_));
    if (sdf->HasElement("recover_pitch_velocity_p_gain"))
      (sdf->GetElement("recover_pitch_velocity_p_gain")->GetValue()->Get(recover_pitch_velocity_p_gain_));
    if (sdf->HasElement("recover_z_velocity_p_gain"))
      (sdf->GetElement("recover_z_velocity_p_gain")->GetValue()->Get(recover_z_velocity_p_gain_));
    ROS_INFO("PlanarMovePlugin (ns = %s) recover_roll_velocity_p_gain_ = %f", robot_namespace_.c_str(), recover_roll_velocity_p_gain_);
    ROS_INFO("PlanarMovePlugin (ns = %s) recover_pitch_velocity_p_gain_ = %f", robot_namespace_.c_str(), recover_pitch_velocity_p_gain_);
    ROS_INFO("PlanarMovePlugin (ns = %s) recover_z_velocity_p_gain_ = %f", robot_namespace_.c_str(), recover_z_velocity_p_gain_);

    x_velocity_limit_max_ = 1.0;
    x_velocity_limit_min_ =-1.0;
    y_velocity_limit_max_ = 1.0;
    y_velocity_limit_min_ =-1.0;
    rot_velocity_limit_max_ = 1.0;
    rot_velocity_limit_min_ =-1.0;
    x_acceleration_limit_max_ = 1.0;
    x_acceleration_limit_min_ =-1.0;
    y_acceleration_limit_max_ = 1.0;
    y_acceleration_limit_min_ =-1.0;
    rot_acceleration_limit_max_ = 1.0;
    rot_acceleration_limit_min_ =-1.0;

    if (sdf->HasElement("x_velocity_limit_max"))
      (sdf->GetElement("x_velocity_limit_max")->GetValue()->Get(x_velocity_limit_max_));
    if (sdf->HasElement("x_velocity_limit_min"))
      (sdf->GetElement("x_velocity_limit_min")->GetValue()->Get(x_velocity_limit_min_));
    if (sdf->HasElement("y_velocity_limit_max"))
      (sdf->GetElement("y_velocity_limit_max")->GetValue()->Get(y_velocity_limit_max_));
    if (sdf->HasElement("y_velocity_limit_min"))
      (sdf->GetElement("y_velocity_limit_min")->GetValue()->Get(y_velocity_limit_min_));
    if (sdf->HasElement("rot_velocity_limit_max"))
      (sdf->GetElement("rot_velocity_limit_max")->GetValue()->Get(rot_velocity_limit_max_));
    if (sdf->HasElement("rot_velocity_limit_min"))
      (sdf->GetElement("rot_velocity_limit_min")->GetValue()->Get(rot_velocity_limit_min_));
    if (sdf->HasElement("x_acceleration_limit_max"))
      (sdf->GetElement("x_acceleration_limit_max")->GetValue()->Get(x_acceleration_limit_max_));
    if (sdf->HasElement("x_acceleration_limit_min"))
      (sdf->GetElement("x_acceleration_limit_min")->GetValue()->Get(x_acceleration_limit_min_));
    if (sdf->HasElement("y_acceleration_limit_max"))
      (sdf->GetElement("y_acceleration_limit_max")->GetValue()->Get(y_acceleration_limit_max_));
    if (sdf->HasElement("y_acceleration_limit_min"))
      (sdf->GetElement("y_acceleration_limit_min")->GetValue()->Get(y_acceleration_limit_min_));
    if (sdf->HasElement("rot_acceleration_limit_max"))
      (sdf->GetElement("rot_acceleration_limit_max")->GetValue()->Get(rot_acceleration_limit_max_));
    if (sdf->HasElement("rot_acceleration_limit_min"))
      (sdf->GetElement("rot_acceleration_limit_min")->GetValue()->Get(rot_acceleration_limit_min_));

    ROS_INFO("PlanarMovePlugin (ns = %s) x_velocity_limit_max_ = %f", robot_namespace_.c_str(), x_velocity_limit_max_);
    ROS_INFO("PlanarMovePlugin (ns = %s) x_velocity_limit_min_ = %f", robot_namespace_.c_str(), x_velocity_limit_min_);
    ROS_INFO("PlanarMovePlugin (ns = %s) y_velocity_limit_max_ = %f", robot_namespace_.c_str(), y_velocity_limit_max_);
    ROS_INFO("PlanarMovePlugin (ns = %s) y_velocity_limit_min_ = %f", robot_namespace_.c_str(), y_velocity_limit_min_);
    ROS_INFO("PlanarMovePlugin (ns = %s) rot_velocity_limit_max_ = %f", robot_namespace_.c_str(), rot_velocity_limit_max_);
    ROS_INFO("PlanarMovePlugin (ns = %s) rot_velocity_limit_min_ = %f", robot_namespace_.c_str(), rot_velocity_limit_min_);
    ROS_INFO("PlanarMovePlugin (ns = %s) x_acceleration_limit_max_ = %f", robot_namespace_.c_str(), x_acceleration_limit_max_);
    ROS_INFO("PlanarMovePlugin (ns = %s) x_acceleration_limit_min_ = %f", robot_namespace_.c_str(), x_acceleration_limit_min_);
    ROS_INFO("PlanarMovePlugin (ns = %s) y_acceleration_limit_max_ = %f", robot_namespace_.c_str(), y_acceleration_limit_max_);
    ROS_INFO("PlanarMovePlugin (ns = %s) y_acceleration_limit_min_ = %f", robot_namespace_.c_str(), y_acceleration_limit_min_);
    ROS_INFO("PlanarMovePlugin (ns = %s) rot_acceleration_limit_max_ = %f", robot_namespace_.c_str(), rot_acceleration_limit_max_);
    ROS_INFO("PlanarMovePlugin (ns = %s) rot_acceleration_limit_min_ = %f", robot_namespace_.c_str(), rot_acceleration_limit_min_);

    // joint_state_idel_sec
    joint_state_idel_sec_ = -1;
    if (sdf->HasElement("joint_state_idel_sec"))
      (sdf->GetElement("joint_state_idel_sec")->GetValue()->Get(joint_state_idel_sec_));
    last_cmd_subscribe_time_ = parent_->GetWorld()->SimTime();

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED("planar_move", "PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG_NAMED("planar_move", "OCPlugin (%s) has started",
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosPlanarMove::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosPlanarMove::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosPlanarMove::UpdateChild, this));
    
    rayShape_ = boost::dynamic_pointer_cast<physics::RayShape>(
         parent_->GetWorld()->Physics()->CreateShape("ray", physics::CollisionPtr()));

    this->last_time_ = parent_->GetWorld()->SimTime();

  }

  // Update the controller
  void GazeboRosPlanarMove::UpdateChild()
  {
#ifdef ENABLE_PROFILER
    IGN_PROFILE("GazeboRosPlanarMove::UpdateChild");
    IGN_PROFILE_BEGIN("fill ROS message");
#endif
    boost::mutex::scoped_lock scoped_lock(lock);
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent_->GetWorld()->SimTime();
#else
    common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
    if (cmd_timeout_>=0)
    {
      if ((ros::Time::now()-last_cmd_received_time_).toSec() > cmd_timeout_)
      {
        x_ = 0;
        y_ = 0;
        rot_ = 0;
      }
    }
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = parent_->WorldPose();
#else
    ignition::math::Pose3d pose = parent_->GetWorldPose().Ign();
#endif
    float yaw = pose.Rot().Yaw();
    float dt  = (current_time-this->last_time_).Double();

    ignition::math::Vector3d gravity(parent_->GetWorld()->Gravity());
    ignition::math::Vector3d linear_vel = ignition::math::Matrix4d(pose.Rot()).Inverse() * parent_->WorldLinearVel();
    ignition::math::Vector3d angular_vel = parent_->WorldAngularVel();

    // check for joint_state_idel_sec_
    if ( joint_state_idel_sec_ > 0 &&
         (current_time-this->last_cmd_subscribe_time_).Double() > joint_state_idel_sec_ ) {
      if (int((current_time-this->last_cmd_subscribe_time_).Double()*1000)%5000 < 1 ) {
        ROS_WARN("PlanarMovePlugin (ns = %s) did not received %s for %f sec",
                 robot_namespace_.c_str(), command_topic_.c_str(), joint_state_idel_sec_);
      }
      x_ = y_ = rot_ = 0;
    }
    linear_cmd_ = ignition::math::Vector3d (x_, y_, 0);
    angular_cmd_ = ignition::math::Vector3d(-recover_roll_velocity_p_gain_ * pose.Rot().Roll(),
                                 -recover_pitch_velocity_p_gain_* pose.Rot().Pitch(),
                                 rot_);
    // put on the ground
    double distBelow;
    std::string entityName;
    gazebo::physics::EntityPtr entityBelow;
    gazebo::physics::EntityPtr fromEntity = parent_;
    this->GetNearestEntityBelow(fromEntity, distBelow, entityName);
    if ( ( ! entityName.empty() ) && distBelow >  0.00001 ) {
      linear_cmd_[2] += recover_z_velocity_p_gain_ * gravity[2] * dt;
    }

    // accel limit
    ignition::math::Vector3d linear_acc  = (linear_cmd_  - last_linear_cmd_)/dt;
    ignition::math::Vector3d angular_acc = (angular_cmd_ - last_angular_cmd_)/dt;

    if ( linear_acc[0] > x_acceleration_limit_max_)  linear_cmd_[0] = last_linear_cmd_[0] + x_acceleration_limit_max_*dt;
    if ( linear_acc[0] < x_acceleration_limit_min_)  linear_cmd_[0] = last_linear_cmd_[0] + x_acceleration_limit_min_*dt;
    if ( linear_acc[1] > y_acceleration_limit_max_)  linear_cmd_[1] = last_linear_cmd_[1] + y_acceleration_limit_max_*dt;
    if ( linear_acc[1] < y_acceleration_limit_min_)  linear_cmd_[1] = last_linear_cmd_[1] + y_acceleration_limit_min_*dt;
    if ( angular_acc[2] > rot_acceleration_limit_max_) angular_cmd_[2] = last_angular_cmd_[2] + rot_acceleration_limit_max_*dt;
    if ( angular_acc[2] < rot_acceleration_limit_min_) angular_cmd_[2] = last_angular_cmd_[2] + rot_acceleration_limit_min_*dt;

    // velocity limit
    if ( linear_cmd_[0] > x_velocity_limit_max_ ) linear_cmd_[0] = x_velocity_limit_max_;
    if ( linear_cmd_[0] < x_velocity_limit_min_ ) linear_cmd_[0] = x_velocity_limit_min_;
    if ( linear_cmd_[1] > y_velocity_limit_max_ ) linear_cmd_[1] = y_velocity_limit_max_;
    if ( linear_cmd_[1] < y_velocity_limit_min_ ) linear_cmd_[1] = y_velocity_limit_min_;
    if ( angular_cmd_[2] > rot_velocity_limit_max_ ) angular_cmd_[2] = rot_velocity_limit_max_;
    if ( angular_cmd_[2] < rot_velocity_limit_min_ ) angular_cmd_[2] = rot_velocity_limit_min_;

    // compensate to horizental velocity
    ignition::math::Matrix3d pitch = ignition::math::Matrix3d();
    pitch.Axis(ignition::math::Vector3d(0,-1,0), pose.Rot().Pitch());

    if ( angular_cmd_[0] >  M_PI/2 ) angular_cmd_[0] =  M_PI/2;
    if ( angular_cmd_[0] < -M_PI/2 ) angular_cmd_[0] = -M_PI/2;
    if ( angular_cmd_[1] >  M_PI/2 ) angular_cmd_[1] =  M_PI/2;
    if ( angular_cmd_[1] < -M_PI/2 ) angular_cmd_[1] = -M_PI/2;
    
    ignition::math::Vector3d temp_ = pitch * linear_cmd_;
    parent_->SetLinearVel(ignition::math::Matrix4d(pose.Rot()) * temp_);
    parent_->SetAngularVel(ignition::math::Matrix4d(pose.Rot()) * angular_cmd_);


#ifdef ENABLE_PROFILER
    IGN_PROFILE_END();
#endif
    if (odometry_rate_ > 0.0) {
      double seconds_since_last_update =
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
#ifdef ENABLE_PROFILER
        IGN_PROFILE_BEGIN("publishOdometry");
#endif
        publishOdometry(seconds_since_last_update);
#ifdef ENABLE_PROFILER
        IGN_PROFILE_END();
#endif
        last_odom_publish_time_ = current_time;
      }
    }
#if GAZEBO_MAJOR_VERSION >= 8
    this->last_time_ = parent_->GetWorld()->SimTime();
#else
    this->last_time_ = parent_->GetWorld()->GetSimTime();
#endif
    this->last_linear_cmd_ = linear_cmd_;
    this->last_angular_cmd_ = angular_cmd_;
  }

  // Finalize the controller
  void GazeboRosPlanarMove::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosPlanarMove::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg)
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    last_cmd_received_time_ = ros::Time::now();
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
#if GAZEBO_MAJOR_VERSION >= 8
    last_cmd_subscribe_time_ = parent_->GetWorld()->SimTime();
#else
    last_cmd_subscribe_time_ = parent_->GetWorld()->GetSimTime();
#endif
  }

  void GazeboRosPlanarMove::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosPlanarMove::publishOdometry(double step_time)
  {

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->parent_->WorldPose();
#else
    ignition::math::Pose3d pose = this->parent_->GetWorldPose().Ign();
#endif

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3    vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame,
            base_footprint_frame));

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    linear.X() = (pose.Pos().X() - last_odom_pose_.Pos().X()) / step_time;
    linear.Y() = (pose.Pos().Y() - last_odom_pose_.Pos().Y()) / step_time;
    if (rot_ > M_PI / step_time)
    {
      // we cannot calculate the angular velocity correctly
      odom_.twist.twist.angular.z = this->angular_cmd_[2];
    }
    else
    {
      float last_yaw = last_odom_pose_.Rot().Yaw();
      float current_yaw = pose.Rot().Yaw();
      while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
      while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
      float angular_diff = current_yaw - last_yaw;
      odom_.twist.twist.angular.z = angular_diff / step_time;
    }
    last_odom_pose_ = pose;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_pub_.publish(odom_);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosPlanarMove)
}
