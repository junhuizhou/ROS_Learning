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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "roborts_msgs/BackCameraAction.h"
#include <actionlib/client/terminal_state.h>

int main(int argc, char **argv) {
  //ros::init(argc, argv, "armor_detection_node_test_client");
    ros::init(argc, argv, "backcamera_client");

  // create the action client
  //actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> ac("armor_detection_node_action", true);
   actionlib::SimpleActionClient<roborts_msgs::BackCameraAction> ac("backcamera_node_action", true);
  
  //roborts_msgs::ArmorDetectionResult node_result;
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Start.");
  roborts_msgs::BackCameraGoal goal;

  char command = '0';

  while (command != '4') {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: start the action" << std::endl
              << "2: pause the action" << std::endl
              << "3: stop  the action" << std::endl
              << "4: exit the program" << std::endl
              << "5: start the tag detaction" << std::endl;

    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4'&& command != '5') {
      std::cout << "please inpugain!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

    switch (command) {
      //start thread.
      case '1':
        goal.command = 1;
        ROS_INFO("I am running the request");
        ac.sendGoal(goal);
        break;
        //pause thread.
      case '2':
        goal.command = 2;
        ROS_INFO("Action server will pause.");
        ac.sendGoal(goal);
        break;
        //stop thread.
      case '3':
        goal.command = 3;
        ROS_INFO("I am cancelling the request");
        ac.cancelGoal();
        break;
      case '5':
        goal.command = 5;
        ROS_INFO("running tag detection");
        ac.sendGoal(goal);
        break;
      default:
        break;
    }
  }
  return 0;
}
