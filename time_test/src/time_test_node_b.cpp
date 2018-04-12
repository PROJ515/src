#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int16.h"
#include <sstream>

bool triggerFlag = false;
geometry_msgs::Quaternion received_times	;
ros::Time received_at_time;

void trigger2_cb(const geometry_msgs::Quaternion::ConstPtr& rec_times){
	received_at_time = ros::Time::now();
	received_times = *rec_times;
	triggerFlag = true;
}

//(*cirPtr) .radius = 10 ;
//=
//	cirPtr->radius = 10

int main(int argc, char **argv){
	ros::init(argc, argv, "time_test_node_b");
	ros::NodeHandle nh;
	ros::Subscriber trigger_2_sub = nh.subscribe("trigger_2", 1000, trigger2_cb);

	//ros::Rate loop_rate(10);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */

	while (ros::ok()){
		ros::spinOnce();

		if (triggerFlag){

			ros::Time t1 = ros::Time::now();
			float trigger_to_end = t1.toSec() - received_times.x;
//			float this_node_to_end = received_at_time.toSec() - received_times.x;
			ROS_INFO("Trigger to transmit (just before message leaves node one) duration: %f", received_times.z);
			ROS_INFO("Trigger to  now: %f", trigger_to_end);
			float message_latency = received_at_time.toSec() - received_times.z;
			ROS_INFO("Message latency: %f", message_latency);
			return 0;
		}


		//loop_rate.sleep();

	}





}


