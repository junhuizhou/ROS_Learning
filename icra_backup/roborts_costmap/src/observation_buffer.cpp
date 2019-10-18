/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "observation_buffer.h"

using namespace std;
using namespace tf;

namespace roborts_costmap {
ObservationBuffer::ObservationBuffer(string topic_name, double observation_keep_time, double expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     double raytrace_range, TransformListener& tf, string global_frame,
                                     string sensor_frame, double tf_tolerance) :
    tf_(tf), observation_keep_time_(observation_keep_time), expected_update_rate_(expected_update_rate),
    last_updated_(ros::Time::now()), global_frame_(global_frame), sensor_frame_(sensor_frame), topic_name_(topic_name),
    min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range), tf_tolerance_(tf_tolerance)
{
  printf("Initializing ObservationBuffer...\n");
  ros::NodeHandle nh;

  // Get the name of the partner
  ifstream infile;
  string file = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
  infile.open(file.data());
  string s;
  string partner_name;
  while(getline(infile,s)) {
      if (s == "partner_name: \"robot1\"") {
        partner_name = "robot1";
        printf("My partner is robot 1\n");
        break;
      } else if (s == "partner_name: \"robot2\"") {
        partner_name = "robot2";
        printf("My partner is robot 2\n");
        break;
      }
  }
  infile.close();

  string partner_topic_sub = "/" + partner_name + "/partner_msg";
  partner_info_sub_ = nh.subscribe(partner_topic_sub, 1, &ObservationBuffer::PartnerCallback, this);
  printf("ObservationBuffer initilized\n");
}

void ObservationBuffer::PartnerCallback(const roborts_msgs::PartnerInformationConstPtr& partner_info) {
  partner_pose_ = partner_info->partner_pose;
  is_enemy_detected = partner_info->enemy_detected;
  if (is_enemy_detected) {
    enemy_poses_from_partner_.clear();
    for (int i = 0; i < partner_info->enemy_info.size(); i++)
      enemy_poses_from_partner_.push_back(partner_info->enemy_info[i].enemy_pos);
    // double x = enemy_pose_from_partner_.pose.position.x;
    // double y = enemy_pose_from_partner_.pose.position.y;
    // double z = enemy_pose_from_partner_.pose.position.z;
    // printf("enemy pose from my partner: (%lf, %lf, %lf)\n", x, y, z);
  }
}

ObservationBuffer::~ObservationBuffer()
{
}

bool ObservationBuffer::SetGlobalFrame(const std::string new_global_frame)
{
  ros::Time transform_time = ros::Time::now();
  std::string tf_error;

  if (!tf_.waitForTransform(new_global_frame, global_frame_, transform_time, ros::Duration(tf_tolerance_),
                            ros::Duration(0.01), &tf_error))
  {
    ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(),
              global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
    return false;
  }

  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    try
    {
      Observation& obs = *obs_it;

      geometry_msgs::PointStamped origin;
      origin.header.frame_id = global_frame_;
      origin.header.stamp = transform_time;
      origin.point = obs.origin_;

      // we need to transform the origin of the observation to the new global frame
      tf_.transformPoint(new_global_frame, origin, origin);
      obs.origin_ = origin.point;

      // we also need to transform the cloud of the observation to the new global frame
      pcl_ros::transformPointCloud(new_global_frame, *obs.cloud_, *obs.cloud_, tf_);
    }
    catch (TransformException& ex)
    {
      ROS_ERROR("TF Error attempting to transform an observation from %s to %s: %s", global_frame_.c_str(),
                new_global_frame.c_str(), ex.what());
      return false;
    }
  }

  // now we need to update our global_frame member
  global_frame_ = new_global_frame;
  return true;
}

void ObservationBuffer::BufferCloud(const sensor_msgs::PointCloud2& cloud)
{
  try
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    // Actually convert the PointCloud2 message into a type we can reason about
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

    // Constants
    double arc_length = 0.2;
    double r = 0.2;
    double angle_step = arc_length / r;
    const double h = 0.0;
    pcl::PointXYZ temp;
    temp.z = h;

    unsigned int old_size = pcl_cloud.points.size();

    if (!partner_pose_.header.frame_id.empty()) {
      // Transform partner_pose_ to partner_pose_tmp_
      geometry_msgs::PoseStamped partner_pose_tmp_;
      bool is_transform = true;
      tf_.waitForTransform(partner_pose_.header.frame_id, cloud.header.frame_id, ros::Time(0), ros::Duration(3.0));
      try {
        tf_.transformPose(cloud.header.frame_id, ros::Time(0), partner_pose_, partner_pose_.header.frame_id, partner_pose_tmp_);
      } catch (tf::ExtrapolationException &ex) {
        ROS_ERROR("Extrapolation Error looking up stamped point: %s", ex.what());
        is_transform = false;
      } catch (TransformException &tfe) {
        ROS_ERROR("TF Exception that should never happen from frame [%s] to [%s], %s", partner_pose_.header.frame_id.c_str(),
              cloud.header.frame_id.c_str(), tfe.what());
        is_transform = false;
      } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        is_transform = false;
      } catch (...) {
        ROS_ERROR("Unknown exception when transforming partner pose");
        is_transform = false;
      }
      if (is_transform) {
        // Add partner information into pcl_cloud
        for(double angle = 0; angle < 2*3.14; angle += angle_step) {
          temp.x = partner_pose_tmp_.pose.position.x + r * cos(angle);
          temp.y = partner_pose_tmp_.pose.position.y + r * sin(angle);
          pcl_cloud.push_back(temp);
        }
      }
    }

    arc_length = 0.135;
    r = 0.135;
    angle_step = arc_length / r;
    if (is_enemy_detected) {
      if (!enemy_poses_from_partner_.empty()) {
        for (int i = 0; i < enemy_poses_from_partner_.size(); i++) {
          // Transform enemy_pose_from_partner_ to enemy_pose_tmp_
					if (!enemy_poses_from_partner_[i].header.frame_id.empty()) {
						geometry_msgs::PoseStamped enemy_pose_tmp_;
            bool is_transform = true;
            tf_.waitForTransform(enemy_poses_from_partner_[i].header.frame_id, cloud.header.frame_id, ros::Time(0), ros::Duration(3.0));
            try {
              tf_.transformPose(cloud.header.frame_id, ros::Time(0), enemy_poses_from_partner_[i], enemy_poses_from_partner_[i].header.frame_id, enemy_pose_tmp_);
            } catch (tf::ExtrapolationException &ex) {
              ROS_ERROR("Extrapolation Error looking up stamped point: %s", ex.what());
              is_transform = false;
            } catch (TransformException &tfe) {
              ROS_ERROR("TF Exception that should never happen from frame [%s] to [%s], %s", enemy_poses_from_partner_[i].header.frame_id.c_str(),
                    cloud.header.frame_id.c_str(), tfe.what());
              is_transform = false;
            } catch (const std::exception &e) {
              std::cerr << e.what() << '\n';
              is_transform = false;
            } catch (...) {
              ROS_ERROR("Unknown exception when transforming enemy pose from partner");
              is_transform = false;
            }
            if (is_transform) {
              // Add partner information into pcl_cloud
              for(double angle = 0; angle < 2*3.14; angle += angle_step) {
                temp.x = enemy_pose_tmp_.pose.position.x + r * cos(angle);
                temp.y = enemy_pose_tmp_.pose.position.y + r * sin(angle);
                pcl_cloud.push_back(temp);
              }
            }
					}
				}
      }
    }

    unsigned int new_size = pcl_cloud.points.size();

    pts_num_added = new_size - old_size;

    BufferCloud(pcl_cloud);
  }
  catch (pcl::PCLException& ex)
  {
    ROS_ERROR("Failed to convert a message to a pcl type, dropping observation: %s", ex.what());
    return;
  }
}

void ObservationBuffer::BufferCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  Stamped < tf::Vector3 > global_origin;

  // create a new observation on the list to be populated
  observation_list_.push_front(Observation());
  clear_observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try
  {
    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    Stamped < tf::Vector3 > local_origin(tf::Vector3(0, 0, 0),
                                         pcl_conversions::fromPCL(cloud.header).stamp, origin_frame);
    tf_.waitForTransform(global_frame_, local_origin.frame_id_, local_origin.stamp_, ros::Duration(0.5));
    tf_.transformPoint(global_frame_, local_origin, global_origin);
    observation_list_.front().origin_.x = global_origin.getX();
    observation_list_.front().origin_.y = global_origin.getY();
    observation_list_.front().origin_.z = global_origin.getZ();
    clear_observation_list_.front().origin_.x = global_origin.getX();
    clear_observation_list_.front().origin_.y = global_origin.getY();
    clear_observation_list_.front().origin_.z = global_origin.getZ();

    // make sure to pass on the raytrace/obstacle range of the observation buffer to the observations
    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;
    clear_observation_list_.front().raytrace_range_ = raytrace_range_;
    clear_observation_list_.front().obstacle_range_ = obstacle_range_;

    pcl::PointCloud < pcl::PointXYZ > global_frame_cloud;

    // transform the point cloud
    pcl_ros::transformPointCloud(global_frame_, cloud, global_frame_cloud, tf_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // now we need to remove observations from the cloud that are below or above our height thresholds
    pcl::PointCloud < pcl::PointXYZ > &observation_cloud = *(observation_list_.front().cloud_);
    pcl::PointCloud < pcl::PointXYZ > &clear_observation_cloud = *(clear_observation_list_.front().cloud_);
    unsigned int cloud_size = global_frame_cloud.points.size();
    unsigned int clear_cloud_size = global_frame_cloud.points.size() - pts_num_added;
    observation_cloud.points.resize(cloud_size);
    clear_observation_cloud.points.resize(clear_cloud_size);
    unsigned int point_count = 0;
    unsigned int clear_point_count = 0;

    // copy over the points that are within our height bounds
    for (unsigned int i = 0; i < cloud_size; ++i)
    {
      if (global_frame_cloud.points[i].z <= max_obstacle_height_
          && global_frame_cloud.points[i].z >= min_obstacle_height_)
      {
        observation_cloud.points[point_count++] = global_frame_cloud.points[i];
        if (i < clear_cloud_size) { // this point comes from the laser
          clear_observation_cloud.points[clear_point_count++] = global_frame_cloud.points[i];
        }
      }
    }

    // resize the cloud for the number of legal points
    observation_cloud.points.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
    clear_observation_cloud.points.resize(clear_point_count);
    clear_observation_cloud.header.stamp = cloud.header.stamp;
    clear_observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  }
  catch (TransformException& ex)
  {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    clear_observation_list_.pop_front();
    ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
              cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = ros::Time::now();

  // we'll also remove any stale observations from the list
  PurgeStaleObservations();
  PurgeStaleClearObservations();

  // develop a clearing observation list, which should not
  // contains the partner and the enemy deteted by the partner.
  // clear_observation_list_.reserve(observation_list_.capacity);
  // std::copy(observation_list_.begin(), observation_list_.end(), clear_observation_list_.begin());
  // for(unsigned int i = 0; i < pts_num_added; i++) {
  //   clear_observation_list_.front().cloud_->points.pop();
  // }
}

// returns a copy of the observations
void ObservationBuffer::GetObservations(vector<Observation>& observations, bool is_clear)
{
  // first... let's make sure that we don't have any stale observations
  if (!is_clear) {
    PurgeStaleObservations();
  } else {
    PurgeStaleClearObservations();
  }

  // now we'll just copy the observations for the caller
  if (!is_clear) {
    list<Observation>::iterator obs_it;
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      observations.push_back(*obs_it);
    }
  } else {
    list<Observation>::iterator obs_it;
    for (obs_it = clear_observation_list_.begin(); obs_it != clear_observation_list_.end(); ++obs_it)
    {
      observations.push_back(*obs_it);
    }
  }
}

void ObservationBuffer::PurgeStaleObservations()
{
  if (!observation_list_.empty())
  {
    list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == ros::Duration(0.0))
    {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      Observation& obs = *obs_it;
      // check if the observation is out of date... and if it is, remove it and those that follow from the list
      ros::Duration time_diff = last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp;
      if ((last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp) > observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

void ObservationBuffer::PurgeStaleClearObservations()
{
  if (!clear_observation_list_.empty())
  {
    list<Observation>::iterator obs_it = clear_observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == ros::Duration(0.0))
    {
      clear_observation_list_.erase(++obs_it, clear_observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = clear_observation_list_.begin(); obs_it != clear_observation_list_.end(); ++obs_it)
    {
      Observation& obs = *obs_it;
      // check if the observation is out of date... and if it is, remove it and those that follow from the list
      ros::Duration time_diff = last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp;
      if ((last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp) > observation_keep_time_)
      {
        clear_observation_list_.erase(obs_it, clear_observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::IsCurrent() const
{
  if (expected_update_rate_ == ros::Duration(0.0))
    return true;

  bool current = (ros::Time::now() - last_updated_).toSec() <= expected_update_rate_.toSec();
  if (!current)
  {
    ROS_WARN(
        "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
        topic_name_.c_str(), (ros::Time::now() - last_updated_).toSec(), expected_update_rate_.toSec());
  }
  return current;
}

void ObservationBuffer::ResetLastUpdated()
{
  last_updated_ = ros::Time::now();
}

} //namespace roborts_costmap
