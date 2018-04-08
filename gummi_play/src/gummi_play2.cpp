/* First attempt at moving the gummi arm through code */
/* Author: The Queenmeister */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Quaternion.h>


float dataX, dataY, dataZ, dataW;
bool flags = false;
void instruction_cb(const geometry_msgs::Quaternion::ConstPtr& msg){
	ROS_INFO("x: %d y: %d z: %d w: %d", msg->x, msg->y, msg->z, msg->w);
	dataX = msg->x;
	dataY = msg->y;
	dataZ = msg->z;
	dataW = msg->w;	
		flags = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	ros::Subscriber sub = node_handle.subscribe("cartesian_instructions", 1000, instruction_cb);
	// BEGIN_TUTORIAL
	//
	// Setup
	// ^^^^^
	//
	// MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
	// the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
	// are used interchangably.
	static const std::string PLANNING_GROUP = "right_arm";

	// The :move_group_interface:`MoveGroup` class can be easily
	// setup using just the name of the planning group you would like to control and plan for.
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	// We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to add and remove collision objects in our "virtual world" scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Raw pointers are frequently used to refer to the planning group for improved performance.
	const robot_state::JointModelGroup *joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	// Visualization
	// ^^^^^^^^^^^^^
	//
	// The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
	// and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
	// namespace rvt = rviz_visual_tools;
	// moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
	// visual_tools.deleteAllMarkers();

	// Remote control is an introspection tool that allows users to step through a high level script
	// via buttons and keyboard shortcuts in Rviz
	//  visual_tools.loadRemoteControl();

	// Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
	// Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	// text_pose.translation().z() = 1.75; // above head of PR2
	// visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

	// Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
	// visual_tools.trigger();

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	// Planning to a Pose goal
	// ^^^^^^^^^^^^^^^^^^^^^^^
	// We can plan a motion for this group to a desired pose for the
	// end-effector.

	/* 
	//Here is the current base_link to forearm_roll_u transform:
	At time 1522751255.779
	- Translation: [0.068, -0.184, -0.120]
	- Rotation: in Quaternion [-0.183, -0.116, 0.378, 0.900]
	in RPY (radian) [-0.432, -0.071, 0.811]
	in RPY (degree) [-24.734, -4.053, 46.492]
	 */

	ROS_INFO("Planning for home...");
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.243;
	target_pose1.position.y = -0.129;
	target_pose1.position.z = -0.162;
	move_group.setPoseTarget(target_pose1);
	/*
home: - Translation: [0.243, -0.129, -0.162]

- Translation: [0.242, -0.181, -0.050]

- Translation: [0.243, -0.130, -0.162]
- Rotation: in Quaternion [-0.002, 0.000, -0.002, 1.000]
in RPY (radian) [-0.003, -0.000, -0.005]
in RPY (degree) [-0.176, -0.000, -0.264]
	 */

	/*
	//current valid position:
	target_pose1.orientation.w = 0.9;
	target_pose1.position.x = 0.068;
	target_pose1.position.y = -0.184;
	target_pose1.position.z = -0.120;
	move_group.setPoseTarget(target_pose1);
	 */
	/*
	//This used to be here:
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.28;
	target_pose1.position.y = -0.7;
	target_pose1.position.z = 1.0;
	move_group.setPoseTarget(target_pose1);
	 */

	// Now, we call the planner to compute the plan and visualize it.
	// Note that we are just planning, not asking move_group
	// to actually move the robot.
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO("About to move home...");
	move_group.move();
	ROS_INFO("Moved!?");

	ros::Rate loop_rate(10);



	//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	// Visualizing plans
	// ^^^^^^^^^^^^^^^^^
	// We can also visualize the plan as a line with markers in Rviz.
	//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	//visual_tools.publishAxisLabeled(target_pose1, "pose1");
	//visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	//visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	//visual_tools.trigger();
	//visual_tools.prompt("next step");

	// Moving to a pose goal
	// ^^^^^^^^^^^^^^^^^^^^^
	//
	// Moving to a pose goal is similar to the step above
	// except we now use the move() function. Note that
	// the pose goal we had set earlier is still active
	// and so the robot will try to move to that goal. We will
	// not use that function in this tutorial since it is
	// a blocking function and requires a controller to be active
	// and report success on execution of a trajectory.

	/* Uncomment below line when working with a real robot */

	// Planning to a joint-space goal
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Let's set a joint space goal and move towards it.  This will replace the
	// pose target we set above.
	//
	// To start, we'll create an pointer that references the current robot's state.
	// RobotState is the object that contains all the current position/velocity/acceleration data.
	//  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	//
	// Next get the current set of joint values for the group.
	//  std::vector<double> joint_group_positions;
	//  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
	//  joint_group_positions[0] = -1.0;  // radians
	//  move_group.setJointValueTarget(joint_group_positions);

	//  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	//  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
	//   ROS_INFO("About to move...");
	//   move_group.move();
	//   ROS_INFO("Moved!?");
	// Visualize the plan in Rviz
	//visual_tools.deleteAllMarkers();
	//visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	//visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	//visual_tools.trigger();
	//visual_tools.prompt("next step");

	//Code removed here, see other file to add...
	while(ros::ok()){

		if (flags){
			geometry_msgs::Pose target_pose1;
			target_pose1.orientation.w = dataW;
			target_pose1.position.x = dataX;
			target_pose1.position.y = dataY;
			target_pose1.position.z = dataZ;
			move_group.setPoseTarget(target_pose1);

			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			ROS_INFO("About to move...");
			move_group.move();
			ROS_INFO("Moved!?");
			flags = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::shutdown();
	return 0;
}

