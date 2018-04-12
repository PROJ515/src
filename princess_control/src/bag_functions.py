#!/usr/bin/env python
import rospy
import rosbag
import subprocess
import smach_ros
import time
import sys
#from headcam_node.msg import Int16Array
#from aruco_python.msg import Int16Array
#from aruco_python.msg import FiducialMsg
from princess_control.msg import FiducialMsg, FiducialArray

from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String, Int32, Float32, Bool
from geometry_msgs.msg import PoseStamped, Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult


location = PoseStamped()
def locationHolder(data):
    global location
    location = data



def bagger(data):
    print ("Saving:")
    print (data)
    print ("in rosbag")
    bag = rosbag.Bag('testzzz.bag', 'w')
    try:
        bag.write('/chatter', data)
#        bag.write('move_base_simple/goal', PoseStamped)
    finally:
        print ("bagger finished, closing bag..")
        bag.close()
        print ("passing data to unbagger_view")
        unbagger_view(data)





fileNumber = 0
def locationBagger():
    global fileNumber
    global location
    sFileNumber = str(fileNumber)
    bag = rosbag.Bag('location' + sFileNumber, 'w')
    print ("Saving location")
    print(location)
    print ("in bag file")
    print(bag)
    fileNumber += 1
    try:
        bag.write('move_base_simple/goal', location, location.header.stamp)
    finally:
        print("Location bagging finished, closing bag..")
        bag.close()
        print("Bag closed")


fileNumber = 0
def locationBagger():
    global fileNumber
    global location
    sFileNumber = str(fileNumber)
    bag = rosbag.Bag('location' + sFileNumber, 'w')
    print ("Saving location")
    print(location)
    print ("in bag file")
    print(bag)
    fileNumber += 1
    try:
        bag.write('move_base_simple/goal', location, location.header.stamp)
    finally:
        print("Location bagging finished, closing bag..")
        bag.close()
        print("Bag closed")


def start_chatter_viewer():
    subprocess.Popen("rostopic echo /chatter", shell=True)

def talker2(data):
    pub = rospy.Publisher('/chatter2_wassup', PoseStamped, queue_size=10)
    pub.publish(data)

def unbagger_view(data):
    bag = rosbag.Bag('testzzz.bag', 'r')
    print ("about to extract data from 'testzzz.bag'")
    start_chatter_viewer()
    bagData = bag.read_messages(topics=['/chatter'])
    for topic, msg, t in bag.read_messages(topics=['/chatter']):
        print (msg)
        #talker(bag)
        #talker(bagData)
        talker2(msg)
    bag.close()
    return msg

def unbagger():
    bag = rosbag.Bag('rplidar.bag', 'r')
    bagData = bag.read_messages(topics=['move_base_simple/goal'])
    for topic, msg, t in bag.read_messages(topics=['move_base_simple/goal']):
        print (msg)
        #talker(bag)
        #talker(bagData)
        talker(msg)
    bag.close()
    return msg


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)
    #rospy.Subscriber('geometry_msgs/PoseStamped', PoseStamped, talker)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, bagger)




def listener_tester():
    rospy.Subscriber('/chatter', String, bagger)

def goal_listener():
    #rospy.init_node('listener', anonymous=True)
    #rospy.Subscriber('geometry_msgs/PoseStamped', PoseStamped, talker)
    l = rospy.Subscriber('move_base/result', MoveBaseActionResult, goal_feed)
    return l




def clearBagCounter():
    global fileNumber
    fileNumber = 0






def set_goal():
    print ("Setting goal")
    try:
        fileAppend = oop.numberOut.get()
        print('location'+str(fileAppend))
        bag = rosbag.Bag('location'+str(fileAppend), 'r')
        # if oop.numberOut.get() == 0:
        #     bag = rosbag.Bag('goal1.bag', 'r')
        # elif oop.numberOut.get() == 1:
        #     bag = rosbag.Bag('goal1.bag', 'r')
        # elif oop.numberOut.get() == 2:
        #     bag = rosbag.Bag('goal2.bag', 'r')
        # elif oop.numberOut.get() ==3:
        #     bag = rosbag.Bag('goal3.bag', 'r')
    except:
        print("The bag file you have selected probably doesn't exist")
    bagData = bag.read_messages(topics=['move_base_simple/goal'])
    for topic, msg, t in bag.read_messages(topics=['move_base_simple/goal']):
        #print (msg)
        #talker(bag)
        #talker(bagData)
        talker(msg)
    bag.close()
    print ("Goal set")
    time.sleep(3)

def locationBagger2(location, ID):
    global fileNumber
    #global location
    sFileNumber = str(fileNumber)
    sID = str(ID)
    bag = rosbag.Bag('ID' + sID + 'location' + sFileNumber, 'w')
    print ("Saving location")
    print(location)
    print ("in bag file")
    print(bag)
    fileNumber += 1
    try:
        bag.write('move_base_simple/goal', location)
    finally:
        print("Location bagging finished, closing bag..")
        bag.close()
        print("Bag closed")


def to_pose_stamped(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.25

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose 

found_IDs = []
def fiducial_cb(data):
    for i in range (0, (len(data.FiducialArray))):
        print"ID", data.FiducialArray[i].ID, 'found'
#        print("is equal to", data.FiducialArray[i].ID)
#    	
        global found_IDs
        if (found_IDs.count(data.FiducialArray[i].ID) == 0):
            print"ID", data.FiducialArray[i].ID, 'has not been seen before, saving...'
            found_IDs.append(data.FiducialArray[i].ID)
            location = Pose()
            location.position = data.FiducialArray[i].pose.position
            location.orientation = data.FiducialArray[i].pose.orientation
#    
            locationBagger2(location, data.FiducialArray[i].ID)
        else:
            print"ID", data.FiducialArray[i].ID, "has already been found. Not saving."


def main():

    rospy.init_node('headcam_goal_saver', anonymous=True)
    rospy.Subscriber("fiducials", FiducialArray, fiducial_cb)
    #hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    #poses = {
    #    str(limb): PoseStamped(header=hdr, pose=Pose(position=pos, orientation=orient))}




    print "Running: listener_tester()"
#    listener_tester()
    listener()


	
    rospy.spin()

if __name__ == '__main__':
    main()

