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
}



float scaler = 0.001;
void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg ){

	if (ros::param::getCached("movement_scaler", scaler)) {
		ROS_INFO("Parameter \"Movement scaler\" set to: %f", scaler);
	}

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
	if ( joy_msg->buttons[0] == 1){
		homeFlag = true;
	}

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

	ROS_INFO("Planning for home...");
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

		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::shutdown();
	return 0;
}





