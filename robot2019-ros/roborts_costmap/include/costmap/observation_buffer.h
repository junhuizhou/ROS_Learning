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
#ifndef ROBORTS_COSTMAP_OBSERVATION_BUFFER_H
#define ROBORTS_COSTMAP_OBSERVATION_BUFFER_H

#include <vector>
#include <list>
#include <string>
#include <mutex>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include "observation.h"
#include "roborts_msgs/PartnerInformation.h"
#include "roborts_msgs/EnemyInfo.h"

namespace roborts_costmap {
/**
 * @class ObservationBuffer
 * @brief Takes in point clouds from sensors, transforms them to the desired frame, and stores them
 */
class ObservationBuffer {
 public:
  /**
   * @brief  Constructs an observation buffer
   * @param  topic_name The topic of the observations, used as an identifier for error and warning messages
   * @param  observation_keep_time Defines the persistence of observations in seconds, 0 means only keep the latest
   * @param  expected_update_rate How often this buffer is expected to be updated, 0 means there is no limit
   * @param  min_obstacle_height The minimum height of a hitpoint to be considered legal
   * @param  max_obstacle_height The minimum height of a hitpoint to be considered legal
   * @param  obstacle_range The range to which the sensor should be trusted for inserting obstacles
   * @param  raytrace_range The range to which the sensor should be trusted for raytracing to clear out space
   * @param  tf A reference to a TransformListener
   * @param  global_frame The frame to transform PointClouds into
   * @param  sensor_frame The frame of the origin of the sensor, can be left blank to be read from the messages
   * @param  tf_tolerance The amount of time to wait for a transform to be available when setting a new global frame
   */
  ObservationBuffer(std::string topic_name, double observation_keep_time, double expected_update_rate,
                    double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                    double raytrace_range, tf::TransformListener &tf, std::string global_frame,
                    std::string sensor_frame, double tf_tolerance, bool has_virtual_layer);

  void PartnerCallback(const roborts_msgs::PartnerInformationConstPtr& partner_info);

  /**
   * @brief  Destructor... cleans up
   */
  ~ObservationBuffer();

  /**
   * @brief Sets the global frame of an observation buffer. This will
   * transform all the currently cached observations to the new global
   * frame
   * @param new_global_frame The name of the new global frame.
   * @return True if the operation succeeds, false otherwise
   */
  bool SetGlobalFrame(const std::string new_global_frame);

  /**
   * @brief  Transforms a PointCloud to the global frame and buffers it
   * @param  cloud The cloud to be buffered
   */
  void BufferCloud(const sensor_msgs::PointCloud2 &cloud, bool is_virtual = false);

  /**
   * @brief  Transforms a PointCloud to the global frame and buffers it
   * <b>Note: The burden is on the user to make sure the transform is available... ie they should use a MessageNotifier</b>
   * @param  cloud The cloud to be buffered
   */
  void BufferCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud);

  /**
   * @brief  Pushes copies of all current observations onto the end of the vector passed in
   * @param  observations The vector to be filled
   */
  void GetObservations(std::vector<Observation> &observations, bool is_clear = false);

  /**
   * @brief  Check if the observation buffer is being update at its expected rate
   * @return True if it is being updated at the expected rate, false otherwise
   */
  bool IsCurrent() const;

  /**
   * @brief  Lock the observation buffer
   */
  inline void Lock() {
    lock_.lock();
  }

  /**
   * @brief  Lock the observation buffer
   */
  inline void Unlock() {
    lock_.unlock();
  }

  /**
   * @brief Reset last updated timestamp
   */
  void ResetLastUpdated();

 private:
  /**
   * @brief  Removes any stale observations from the buffer list
   */
  void PurgeStaleObservations();
  void PurgeStaleClearObservations();

  tf::TransformListener &tf_;
  const ros::Duration observation_keep_time_;
  const ros::Duration expected_update_rate_;
  ros::Time last_updated_;
  std::string global_frame_;
  std::string sensor_frame_;
  std::list<Observation> observation_list_;
  std::list<Observation> clear_observation_list_;
  unsigned int pts_num_added;
  std::string topic_name_;
  double min_obstacle_height_, max_obstacle_height_;
  std::recursive_mutex lock_;
  double obstacle_range_, raytrace_range_;
  double tf_tolerance_;
  bool has_virtual_layer_;
  ros::Subscriber partner_info_sub_;

public:
  geometry_msgs::PoseStamped partner_pose_;
  std::vector<geometry_msgs::PoseStamped> enemy_poses_from_partner_;
  bool is_enemy_detected;
};

}// namespace roborts_costmap
#endif  //ROBORTS_COSTMAP_OBSERVATION_BUFFER_H