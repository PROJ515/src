#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int16.h"
#include <sstream>
#include "std_msgs/Int8MultiArray.h"
#include "princess_control/Int16Array.h"
#include "princess_control/FiducialMsg.h"
#include "princess_control/FiducialArray.h"


// Get current time in milliseconds from the Epoch (Unix)
// or the time the system started (Windows).



void fiducial_cb2(const princess_control::FiducialArray::ConstPtr& array){
	//Iterate through all IDs that have been found	
	ROS_INFO("Number of fiducials found: ", array->FiducialArray.size());
	for(int i = 0; i < array->FiducialArray.size(); i++){
		ROS_INFO("ID found: %d", array->FiducialArray[i].ID);
		ROS_INFO("Position x y z: %f %f %f. Orientation x y z: %f %f %f", array->FiducialArray[i].pose.position.x, array->FiducialArray[i].pose.position.y, array->FiducialArray[i].pose.position.z, array->FiducialArray[i].pose.orientation.x, array->FiducialArray[i].pose.orientation.y, array->FiducialArray[i].pose.orientation.z);

	}
}



void fiducial_cb(const princess_control::Int16Array::ConstPtr& array){
	//Iterate through all IDs that have been found
	for(int i = 0; i < array->IDs.size(); i++){
		ROS_INFO("ID found: %d", array->IDs[i]);
	}
}



int main(int argc, char **argv){
	ros::init(argc, argv, "headcam_object_saver");
	ros::NodeHandle nh;
	ros::Subscriber trigger_sub = nh.subscribe("fiducial_ids", 1000, fiducial_cb);
	ros::Publisher trigger_2_pub = nh.advertise<geometry_msgs::Quaternion>("trigger_2", 1000);
	ros::Rate rate(10); // 10 hz

	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}







