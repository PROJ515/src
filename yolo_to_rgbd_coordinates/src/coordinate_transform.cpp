#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
//#include <darknet_ros/YoloObjectDetector.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>


typedef int64_t msec_t;

// Get current time in milliseconds from the Epoch (Unix)
// or the time the system started (Windows).
msec_t time_ms(void);



bool boxFlag = true;
bool depthFlag = false;
sensor_msgs::Image depth_image;
int depth_value;
float x_cent, y_cent;
int counter = 0;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void image_cb (const sensor_msgs::Image::ConstPtr& rc_data)
{
	ROS_INFO("depth image recieved %d", rc_data->data[5]);
	if (boxFlag && (counter > 50)){
		depth_value = rc_data->data[(y_cent * rc_data->width) + x_cent];
		ROS_INFO("BOXFLAG %d", depth_value);
		boxFlag = false;
		counter = 0;
	}
	counter++;
	//ROS_INFO("depth value of thing = %f", depth_v

	//depth_image.data = rc_data->data;
}

//sensor_msgs::Image get_image(void){
//}


//Global point cloud variable
pcl::PointCloud<pcl::PointXYZ> globalPC;

//Structure to hold xyz location
struct location {
	double x;
	double y;
	double z;
} ;

//function to convert 2D object location (in pixels) to 3D object location (in meters?)
location find_object(float x_centre, float y_centre){
	location Location;
	//The line below probably isn't right, but you get the idea
	int coord = (640 * y_centre) + x_centre;
	//int coord = int((480 * y_centre) + x_centre);
	//If we have a valid depth map...
	if (depthFlag){
		//ROS_INFO("3D location of thing: %f, %f, %f.", globalPC[coord].x, globalPC[coord].y, globalPC[coord].z);
		Location.x = globalPC[coord].x;
		Location.y = globalPC[coord].y;
		Location.z = globalPC[coord].z;
		depthFlag = false;
		return Location;
		//If we don't have a valid depth map...
	} else {
		//Keep trying for 40ms:
		msec_t current_time = time_ms();
		while (current_time > (time_ms() + 40)){
			if (depthFlag){
				Location.x = globalPC[coord].x;
				Location.y = globalPC[coord].y;
				Location.z = globalPC[coord].z;
				depthFlag = false;
				return Location;
			}
		}
		Location.x = 0;
		Location.y = 0;
		Location.z = 0;
		return Location;
	}
}

//Recieves point cloud, copies to global variable  
void points_cb2(const PointCloud::ConstPtr& msg){
	globalPC.points = msg->points;
	depthFlag = true;
}

void points_cb(const PointCloud::ConstPtr& msg)
{
	printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

	for (int iz = 0; iz < msg->points.size(); iz+= (msg->width + (msg->height/2))) {
		ROS_INFO("XYZ @ %d = %f, %f, %f.", iz, msg->points[iz].x, msg->points[iz].y, msg->points[iz].z);
	}
	//BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
	//  printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

double confidence_threshold;

std::string object_tf = "objectTF";
void twoD_coordinate_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{

	if (ros::param::getCached("acceptable_confidence", confidence_threshold)) {
		ROS_INFO("Confidence set to: %f", confidence_threshold);
	}

	int sizes = msg->boundingBoxes.size();

	ROS_INFO("number of found objects = [%d]", sizes);


	for(int i = 0; i < msg->boundingBoxes.size(); i++){
		x_cent = (msg->boundingBoxes[i].xmin + msg->boundingBoxes[i].xmax) /2;
		y_cent = (msg->boundingBoxes[i].ymin + msg->boundingBoxes[i].ymax) /2;
		ROS_INFO("Object \"%s \"found with %f confidence.", msg->boundingBoxes[i].Class.c_str(), msg->boundingBoxes[i].probability);
		ROS_INFO("Centre of \"%s \" can be found at pixel (%f, %f).", msg->boundingBoxes[i].Class.c_str(), x_cent, y_cent);
		if (msg->boundingBoxes[i].probability > confidence_threshold){
			location objectLocation = find_object(x_cent, y_cent);
			ROS_INFO("3D location of \"%s\": %f, %f, %f.", msg->boundingBoxes[i].Class.c_str(), objectLocation.x, objectLocation.y, objectLocation.z);
			ROS_INFO("TF SENT");
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			//transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
			if ((objectLocation.x != NULL) && (objectLocation.y != NULL) && (objectLocation.z != NULL)){
				transform.setOrigin( tf::Vector3(objectLocation.x, objectLocation.y, objectLocation.z) );
				tf::Quaternion q;
				q.setRPY(0.01, 0.01, 0.5);
				transform.setRotation(q);
				//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "testTF"));
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", msg->boundingBoxes[i].Class.c_str()));
			} else {
				ROS_WARNING("A Null value has been returned. The RGBD camera is probably looking up a point which is 'shadowed'. Need to fix this.");
			}
		} else {
			ROS_INFO("Confidence less than %f\% \, object discarded.", confidence_threshold*100);
			ROS_INFO("TF NOT SENT");
			// ROS_INFO("depth value at CofM = %f", depth_image[50]);//[y_cent]);
		}
	}

	boxFlag = true;




}


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "coordinate_transform");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;



	ros::param::set("acceptable_confidence", 0.6);
	//n.setParam("acceptable_confidence", 0.6);
	/*if (n.getParamCached("acceptable_confidence", confidence_threshold)) {
	  ROS_INFO("Got param: %f", confidence_threshold);
	  }
	 */
	if (ros::param::getCached("acceptable_confidence", confidence_threshold)) {
		ROS_INFO("Got param: %f", confidence_threshold);
	}
	//n.param("acceptable_confidence", confidence_threshold, 0.80);

	//ros::param::param("acceptable_confidence", confidence_threshold, 0.80);


	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::Subscriber yolo_sub = n.subscribe("/darknet_ros/bounding_boxes", 1, twoD_coordinate_callback);
	//ros::Subscriber depth_sub = n.subscribe("/camera/depth_registered/image_raw", 1, image_cb);
	//ros::Subscriber points_sub = n.subscribe("/camera/depth_registered/points", 1, points_cb);
	ros::Subscriber points_sub2 = n.subscribe("/camera/depth_registered/points", 1, points_cb2);


	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	return 0;
}

#if defined(__WIN32__)

#include <windows.h>

msec_t time_ms(void)
{
	return timeGetTime();
}

#else

#include <sys/time.h>

msec_t time_ms(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (msec_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

#endif






