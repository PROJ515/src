#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int16.h"
#include <sstream>



// Get current time in milliseconds from the Epoch (Unix)
// or the time the system started (Windows).



bool triggerFlag = false;
void trigger_cb(const std_msgs::Int16::ConstPtr& trigger){
	triggerFlag = true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "time_test_node_a");
	ros::NodeHandle nh;
	ros::Subscriber trigger_sub = nh.subscribe("trigger", 1000, trigger_cb);
	ros::Publisher trigger_2_pub = nh.advertise<geometry_msgs::Quaternion>("trigger_2", 1000);

	while(ros::ok()){
		if (triggerFlag){
			ros::Time begin = ros::Time::now();
			/**
			 * This is a message object. You stuff it with data, and then publish it.
			 */
			//		std_msgs::String msg;

			//		std::stringstream ss;
			//ss << "hello world " << count;
			//		msg.data = ss.str();
			ros::Time t1 = ros::Time::now();




			ros::Time t2 = ros::Time::now();

			geometry_msgs::Quaternion msg;
			msg.x = begin.toSec();
			msg.y = t1.toSec();
			msg.z = t2.toSec();
			trigger_2_pub.publish(msg);

			ROS_INFO("Begin: %f, t1: %f, t2: %f", begin.toSec(), t1.toSec(), t2.toSec());
			ros::spinOnce();

			return 0;
		}
		ros::spinOnce();

	
	}
	return 0;
}

/*
   You can create a Time or Duration to a specific value as well, either floating-point seconds:
   Toggle line numbers

   ros::Time a_little_after_the_beginning(0.001);
   ros::Duration five_seconds(5.0);

   or through the two-integer constructor:
   Toggle line numbers

   ros::Time a_little_after_the_beginning(0, 1000000);
   ros::Duration five_seconds(5, 0);

   Converting Time and Duration Instances

   Time and Duration objects can also be turned into floating point seconds:
   Toggle line numbers

   double secs =ros::Time::now().toSec();

   ros::Duration d(0.5);
   secs = d.toSec();
 */




//use it
//msec_t current_time = time_ms();














