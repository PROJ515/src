/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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
 *********************************************************************/

/* Author: Sachin Chitta */
#include <ctime>
#include <ros/ros.h>
#include<sensor_msgs/Joy.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
using namespace std;

float x_val = 0, y_val = 0, z_val = 0;
float roll = 0, pitch = 0, yaw = 0;
float scaler = 0.004, rot_scaler = 0.4;

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg ){

    if ( joy_msg->buttons[0] == 1){
    x_val = scaler * joy_msg->axes[0];
    y_val = scaler * joy_msg->axes[1];
    z_val = scaler * joy_msg->axes[2];

    if(scaler * x_val == abs(0.1)){
        x_val = 0;
    }
    if(scaler * y_val == abs(0.1)){
        y_val = 0;
    }
    if(scaler * y_val == abs(0.1)){
        y_val = 0;
    }
    }else{
        x_val = 0;
        y_val = 0;
        z_val = 0;
    }

    if (joy_msg->buttons[1] == 1){
        roll = rot_scaler * joy_msg->axes[3];
        pitch = rot_scaler * joy_msg->axes[4];
        yaw = rot_scaler * joy_msg->axes[5];

        if (rot_scaler * roll == abs(0.1)){
            roll = 0;
        }
        if (rot_scaler * pitch == abs(0.1)){
            pitch = 0;
        }
        if (rot_scaler * yaw == abs(0.1)){
            yaw = 0;
        }
    }else{
        roll = 0;
        pitch = 0;
        yaw = 0;
    }

}


static geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
    geometry_msgs::Quaternion q;
    double t0 = cos(yaw * 0.5);
    double t1 = sin(yaw * 0.5);
    double t2 = cos(roll * 0.5);
    double t3 = sin(roll * 0.5);
    double t4 = cos(pitch * 0.5);
    double t5 = sin(pitch * 0.5);
    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // joystick
  ros::Subscriber sub = node_handle.subscribe<sensor_msgs::Joy>("/spacenav/joy", 10, &joy_callback);


  /* This sleep is ONLY to allow Rviz to come up */
  //sleep(20.0);

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name
  // of the group you would like to control and plan for.
  ROS_INFO("0");
  moveit::planning_interface::MoveGroup group("right_arm");
  ROS_INFO("1");
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_INFO("2");
  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  ROS_INFO("3");
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // get the current position
  ROS_INFO("X Axis current position: %f", group.getCurrentPose().pose.position.x);
  //geometry_msgs::Pose current_pos;// = group.getCurrentPose();
  geometry_msgs::PoseStamped target_pose1;
  geometry_msgs::PoseStamped current_pos;

  current_pos.pose.position.x  = group.getCurrentPose().pose.position.x;
  current_pos.pose.position.y = group.getCurrentPose().pose.position.y;
  current_pos.pose.position.z = group.getCurrentPose().pose.position.z;

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  ros::Rate r(10); // 10 hz
  //for (int i = 0; i < 50; i++){
  moveit::planning_interface::MoveGroup::Plan my_plan;

  clock_t begin = clock();
  float step = 0.01;
  int counter = 0;
  ROS_INFO("Starting the loop");
  while(ros::ok()){

  //geometry_msgs::Quaternion q = createQuaternionFromRPY(roll, pitch, yaw);

  //transform.setRotation(q);
/*
  target_pose1.orientation.w = current_pos.orientation.w ; //+ q.w;
  target_pose1.orientation.x = current_pos.orientation.x ; // + q.x;
  target_pose1.orientation.y = current_pos.orientation.y ; //+ q.y;
  target_pose1.orientation.z = current_pos.orientation.z ; // + q.z;
*/
  current_pos.pose.position.x =  current_pos.pose.position.x + step; // x_val;
  //current_pos.position.y = current_pos.position.y; // + y_val;
  //current_pos.position.z = current_pos.position.z; // + z_val;
//  target_pose1.header.frame_id = "wrist_link";
  target_pose1.header.frame_id = "wrist";
  target_pose1.pose.position.x = current_pos.pose.position.x;
  target_pose1.pose.position.y = current_pos.pose.position.y;
  target_pose1.pose.position.z = current_pos.pose.position.z;


  group.setPoseTarget(target_pose1);
  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.


  //bool success = group.plan(my_plan);
  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  group.move();
  //ros::spinOnce();
    if (counter > 15){
      counter = 0;
      step = -step;
      clock_t end = clock();
      double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC * 100;
      std::cout << "delt time: " << elapsed_secs << std::endl;
      begin = clock();
    }
    ROS_INFO("counter: %d",  counter);
    counter ++;

    r.sleep();
  }
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);
// END_TUTORIAL

  ros::shutdown();
  return 0;
}
