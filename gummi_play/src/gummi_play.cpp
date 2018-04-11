	/* First attempt at moving the gummi arm through code */
/* Author: The Queenmeister */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include<sensor_msgs/Joy.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen_conversions/eigen_msg.h>

float dataX, dataY, dataZ, dataW;
bool flags = false;
bool homeFlag = false;
bool joint_space_home_flag = false;
bool reach_flag = false;
void instruction_cb(const geometry_msgs::Quaternion::ConstPtr& msg){
	ROS_INFO("x: %f y: %f z: %f w: %f", msg->x, msg->y, msg->z, msg->w);
	dataX = msg->x;
	dataY = msg->y;
	dataZ = msg->z;
	dataW = msg->w;	
	flags = true;
	if (dataW == 2.0){
		homeFlag = true;
		flags = false;
	}
	if (dataW == 3.0){
		joint_space_home_flag = true;
		flags = false;
	}

	if (dataW == 4.0){
		reach_flag = true;
		flags = false;
	}
}



float scaler = 0.001;
void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg ){


	//Check to see whether parameter has been changed (non-cached, doesn't load master but takes longer to execute)
        float scalerCheck = scaler;
	if (ros::param::get("movement_scaler", scaler)){
		if (scalerCheck != scaler){
			ROS_INFO("Parameter \"Movement scaler\" set to: %f", scaler);
		}
	}
	
	//If 'start' has been pressed on the controller, accept input
	if (joy_msg->buttons[8] == 1){
		dataX = scaler*joy_msg->axes[0];
		dataZ = scaler*joy_msg->axes[1];
		dataY = scaler*joy_msg->axes[2];
		flags = true;
	} else {
		dataX = 0.0;
		dataY = 0.0;
		dataZ = 0.0;
		// flags = true;
	}

	//If 'A' has been pressed on the controller, go home (ee space)
	if ( joy_msg->buttons[0] == 1){
		homeFlag = true;
	}

	//If 'C' has been pressed on the controller, go home (joint space)
	if ( joy_msg->buttons[3] == 1){
		joint_space_home_flag = true;
	}
	//If 'something' has been pressed on the controller, go home (joint space)
	if ( joy_msg->buttons[2] == 1){
		reach_flag = true;
	}

}

std::vector<double> joint_space_home(void){
	std::vector<double> joint_group_positions;
	joint_group_positions.push_back(-0.542007);
	joint_group_positions.push_back(0.245437);
	joint_group_positions.push_back(-0.102265);
	joint_group_positions.push_back(-0.538427);
	joint_group_positions.push_back(-0.311909);
	joint_group_positions.push_back(0.0444854);
	joint_group_positions.push_back(0.0715858);
	return joint_group_positions;
}


geometry_msgs::Pose reach_pose(void){
	//- Translation: [0.470, -0.303, 0.113]
	geometry_msgs::Pose target_position;
	target_position.orientation.w = 1.0;
	target_position.position.x = 0.470;
	target_position.position.y = -0.303;
	target_position.position.z = 0.113;
	return(target_position);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Subscriber joy_sub = node_handle.subscribe<sensor_msgs::Joy>("/joy", 10, &joy_callback);



	ros::param::set("movement_scaler", 0.001);
	//n.setParam("acceptable_confidence", 0.6);
	/*if (n.getParamCached("acceptable_confidence", confidence_threshold)) {
	  ROS_INFO("Got param: %f", confidence_threshold);
	  }
	 */
	if (ros::param::getCached("movement_scaler", scaler)) {
		ROS_INFO("Parameter \"Movement scaler\" set to: %f", scaler);
	}



	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	ros::Subscriber sub = node_handle.subscribe("cartesian_instructions", 1000, instruction_cb);
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
	static const std::string PLANNING_GROUP = "right_arm";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup *joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


/*

 extra dummy link to your URDF.
[ INFO] [1523276274.297842866]: Ready to take commands for planning group right_arm.
-0.542007 0.245437 -0.102265 -0.538427 -0.311909 0.0444854 0.0715858 [ INFO] [1523276274.606088233]: Planning for home...
[ INFO] [1523276274.653048641]: About to move home...


*/

//get joint-space state
	  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	//
	// Next get the current set of joint values for the group.
	//  std::vector<double> joint_group_positions;
	//  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	//for (std::vector<double>::const_iterator i = joint_group_positions.begin(); i != joint_group_positions.end(); ++i){
	//	std::cout << *i << ' ';
	//}
//state got
	// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
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
	/*
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	move_group.setJointValueTarget(joint_space_home());
	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO("About to move...");
	move_group.move();
	ROS_INFO("Moved!?");*/
//-0.542007 0.245437 -0.102265 -0.538427 -0.311909 0.0444854 0.0715858 [ INFO] [1523276274.606088233]: Planning for home...

	//ROS_INFO("Planning for home...");
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.243;
	target_pose1.position.y = -0.129;
	target_pose1.position.z = -0.162;
	move_group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO("About to move home...");
	move_group.move();
	ROS_INFO("Moved!?");

	ros::Rate loop_rate(10);

	while(ros::ok()){

		if (flags){

			const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("wrist");
			geometry_msgs::Pose pose;
			tf::poseEigenToMsg(end_effector_state, pose);
			target_pose1.position.x += dataX;
			target_pose1.position.y += dataY;
			target_pose1.position.z += dataZ;

			move_group.setPoseTarget(target_pose1);

			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			ROS_INFO("About to move...");
			move_group.move();
			ROS_INFO("Moved!?");
			flags = false;
		} 
		if (homeFlag){
			target_pose1.orientation.w = 1.0;
			target_pose1.position.x = 0.243;
			target_pose1.position.y = -0.129;
			target_pose1.position.z = -0.162;
			move_group.setPoseTarget(target_pose1);

			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			ROS_INFO("About to move home...");
			move_group.move();
			ROS_INFO("Moved!?");
			homeFlag = false;
		}
		if (joint_space_home_flag){
			move_group.setJointValueTarget(joint_space_home());
			bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			ROS_INFO("About to move...");
			move_group.move();
			ROS_INFO("Moved!?");
			joint_space_home_flag = false;
		}
		if (reach_flag){
			move_group.setPoseTarget(reach_pose());

			moveit::planning_interface::MoveGroupInterface::Plan my_plan;

			bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			ROS_INFO("About to move home...");
			move_group.move();
			ROS_INFO("Moved!?");
			reach_flag = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::shutdown();
	return 0;
}





