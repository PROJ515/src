#!/usr/bin/env python

#Plan:
#Executive functions
#Start roscore, sensor drivers, rosserial
#start navigation stack
#feed first coordinate
#Wait until goal success
#Possibly disable navigation, possibly not
#Start interactive functions
####Kinect drivers
####Gesture stuff
####Voice api
####wait for voice command 
####close interactive functions
#Restart nav stack
#load objective (possibly voice dependant)
#wait for goal success
import sys
import roslib; roslib.load_manifest('smach_tutorials')
import sys
from mtTkinter import *
#import mttkinter as tk
#from Tkinter import messagebox as tm
#import messagebox as tm
import tkMessageBox as tm
import ttk
import rospy
import rosbag
import smach
import subprocess
#from sys import executable
#from subprocess import Popen, CREATE_NEW_CONSOLE
import smach_ros
import time
import threading
import actionlib

#actionlib_msgs / GoalStatusArray

#import utils
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String, Int32, Float32, Bool
from geometry_msgs.msg import PoseStamped, Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

mySettings = 1
#lidar = "rp" #choose between rp and hokuyo
lidar = "0" #choose between rp and hokuyo

#ui stuff
#import pygame, sys
#import pygame.locals()

#import msvcrt

       


class goal_success( object ):
    def __init__( self ):
        self.public_value = 'foo'

    def __call__( self ):
        return self.public_value





##setup switch case statements, C styley
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration
    
    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False


PIDen = True


def calculate(*args):
    try:
        value = float(feet.get())
        meters.set((0.3048 * value * 10000.0 + 0.5) / 10000.0)
    except ValueError:
        pass


def cmd_print(data):
    oop.cmd_vel.insert(END, data)
    oop.cmd_vel.insert(END, '\n')
    oop.cmd_vel.see("end")

def chatter_print(data):
    oop.chatterBox.insert(END, data)
    oop.chatterBox.insert(END, '\n')
    oop.chatterBox.see("end")

def v_left_print(data):
    oop.v_left_box.insert(END, data)
    oop.v_left_box.insert(END, '\n')
    oop.v_left_box.see("end")

def v_right_print(data):
    oop.v_right_box.insert(END, data)
    oop.v_right_box.insert(END, '\n')
    oop.v_right_box.see("end")



def restartSerial():
    oop.p.kill()
    oop.p = subprocess.Popen("rosrun rosserial_python serial_node.py /dev/ttyACM*", stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, shell=True)


def restartMovebase():
    print("Restarting move_base")
    oop.movebase_proc.kill()
    oop.movebase_proc = subprocess.Popen("roslaunch slambot_2dnav move_base.launch", stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE, shell=True)


def reader(f, buffer):
    while True:
        line = f.readline()
        if line:
            buffer.append(line)
        else:
            break


# def textFunc():
#    #p = subprocess.Popen("roslaunch rplidar_ros rplidar.launch",stdout=subprocess.PIPE,stderr=subprocess.PIPE, shell=True)
#    p = subprocess.Popen("rosrun rosserial_python serial_node.py /dev/ttyACM*",stdout=subprocess.PIPE,stderr=subprocess.PIPE, shell=True)
#    output, errors = p.communicate()
#    t.insert(END, output)




def textChatInit():
    p = subprocess.Popen("rostopic echo /chatter", stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    output, errors = p.communicate()
    t = Text(mainframe)  # , width=40, height=10)
    t.grid(column=0, row=3, sticky=(S, W))
    t.insert(END, output)


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.get_children(recursive=True):
        proc.kill()
    process.kill()


class PIDbars(object):
    def __init__(self):
        self.sliderName = sliderName
        self.boxName = boxName

    def bar(self):
        self = Scale(PIDframe, from_=0.0, to=20.0, orient=HORIZONTAL, resolution=0.01)
        self.config(length=200, label="Proportional gain", command=PIDpub, variable=lkp)
        self.grid(column=0, row=0)

    def pidBox(self):
        self = Entry(PIDframe)
        self.bind('<Return>', lambda e: setSlider(self.sliderName,
                                                  self.boxName))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.config(width=3)
        self.grid(column=1, row=0, sticky=(W))


class lPIDp(PIDbars):
    def bar(self):
        return lPIDp


def color_change(name):
    if PIDen:
        name.configure(bg="red")
        PIDen = False
    else:
        name.configure(bg="green")
        PIDen = True


def getVal(slider):
    return slider.get()  # will return the scale value


def setSlider(slider, box):
    slider.set(float(box.get()))  # get the entry value and set the slider to it
    box.delete(0, END)  # clear the entry box


def resetSlider(sl1, sl2, sl3, sl4, sl5, sl6):
    sl1.set(float(1.4))
    sl2.set(float(0.14))
    sl3.set(float(0.01))

    sl4.set(float(1.4))
    sl5.set(float(0.14))
    sl6.set(float(0.01))


def rvizLaunch():
    subprocess.Popen("killall -9 rviz", shell=True)
    time.sleep(1)
    rviz_proc = subprocess.Popen("rosrun rviz rviz", shell=True)


def lidarLaunch():
    subprocess.Popen("killall -9 rplidar.launch", shell=True)
    time.sleep(1)
    lidar_proc = subprocess.Popen("roslaunch rplidar_ros rplidar.launch", shell=True)


# def startRQTsteering():
#     try:
#         rqt_steering.kill()
#     except NameError:
#         var_exists = False
#     else:
#         var_exists = True
#
#     if var_exists == False:
#         rqt_steering = subprocess.Popen("rosrun rqt_robot_steering rqt_robot_steering", shell=True)
#     elif var_exists == True:
#         rqt_steering.kill()
#         rqt_steering = subprocess.Popen("rosrun rqt_robot_steering rqt_robot_steering", shell=True)
#
def startRQTsteering():
    try:
        rqt_steering.kill()
    except:
        rqt_steering = subprocess.Popen("rosrun rqt_robot_steering rqt_robot_steering", shell=True)


def talker(data):
    
    #pub = rospy.Publisher('forwardedGoals', PoseStamped, queue_size=10)
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    pub.publish(data)
    #file = open("testfile.txt","w") 
    #file.write("Hello") 
    #file.write(String(data)) 
    #file.write("   ") 
    #file.close()

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







    #
    # try:
    #     name_of_file = raw_input("Enter the name")
    #     IOjbect = open(name_of_file + ".txt", "w")
    #     IObject.write("Hey There")
    #     read_messages(topics=['move_base_simple/goal'])
    #     bag.write('/chatter', data)
    # # bag.write('move_base_simple/goal', PoseStamped)
    # finally:
    #     print ("bagger finished, closing bag..")
    #     bag.close()
    #     print ("passing data to unbagger_view")
    #     unbagger_view(data)


location = PoseStamped()
def locationHolder(data):
    global location
    location = data


class Employee:
    'stores count of waypoints'
    empCount = 0

    def __init__(self, name, salary):
        self.name = name
        self.salary = salary
        Employee.empCount += 1

    def displayCount(self):
        print "Total Employee %d" % Employee.empCount

    def displayEmployee(self):
        print "Name : ", self.name, ", Salary: ", self.salary


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

def talker2(data):
    pub = rospy.Publisher('/chatter2_wassup', String, queue_size=10)
    pub.publish(data)
 

def configure_slambot():
    subprocess.Popen("roslaunch slambot_2dnav slambot_configuration.launch", shell=True)
    #subprocess.Popen("roslaunch", "slambot_2dnav", "slambot_configuration.launch", shell=True)
    #subprocess.popen([sys.executable, 'script.py'], creationflags = subprocess.CREATE_NEW_CONSOLE)
    return

def configure_slambot_rp():
    subprocess.Popen("roslaunch rplidar_ros rplidar.launch", shell=True)
    #subprocess.Popen("roslaunch", "slambot_2dnav", "slambot_configuration.launch", shell=True)
    #subprocess.popen([sys.executable, 'script.py'], creationflags = subprocess.CREATE_NEW_CONSOLE)
    return

def start_roscore():
    subprocess.Popen("killall -9 roscore", shell=True)
    time.sleep(2)
    subprocess.Popen("roscore", shell=True)
    return

def start_rosserial():
    subprocess.Popen("rosrun rosserial_python serial_node.py /dev/ttyUSB0", shell=True)
    return

def start_move_base():
    subprocess.Popen("roslaunch slambot_2dnav move_base.launch", shell=True)
    return 

def start_rviz():
    subprocess.Popen("rosrun rviz rviz", shell=True)
    return 

def start_smachviewer():
    subprocess.Popen("rosrun smach_viewer smach_viewer.py", shell=True)
    return

            
def ui_init():
    pygame.init()
    BLACK = (0,0,0)
    WIDTH = 640
    HEIGHT = 480
    windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

    windowSurface.fill(BLACK)





    #rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(data)
     #   pub.publish(data)
     #   rate.sleep()

#def talker(foundData):
#    pub = rospy.Publisher('chatter', String, queue_size=10)
#    rospy.init_node('talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
#        rate.sleep()


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #rospy.loginfo("importantInformationGoesHere")

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




def waitkey(setKey):
    x = "0"
    while x != setKey: # ESC
        print("Please press", setKey, " to continue, or 'm' for a menu:")
        x=sys.stdin.read(1)[0]
        if x == "m":
            print("Options: ")
            print("Press 'g' to start listening for goal success")
#v = 'ten'
            x=sys.stdin.read(1)[0]
            for case in switch(x):
                #if case("g"):
                 #   goal_listener()
                if case(): # default, could also just omit condition or 'if True'
                    print "something else!"

# break was used here to look as much like the real thing as possible, but
# elif is generally just as good and more concise.



    #events = pygame.event.get()
    #for event in events:
    #    if event.key != pygame.K_$key:
    #         time.sleep(1)
    #         pass
    #    if event.type == QUIT:
    #         pygame.quit()
    #         sys.exit()
#
#    if msvcrt.kbhit():
#        key = msvcrt.getch()
#        print("Pressed key: ")
#        print(key)   # just to show the result
#        print("escape key: ")
#        print(setKey)
#        while (setkey != key):
#            time.sleep(1)



            


goalSuccess = 0
def goal_feed(result_data):
    #print "goal success!"
    global goalSuccess
    q = result_data
    goalSuccess = q.status_list[0].status

    #print p
    #q = p.split("text: ")
    #print("q.status_list[0].status")
    #print(q.status_list[0].status)

    #.goal_feed
    # print "q.status"
    # print q.status
    # print "q.__getattribute__('status')"
    # print q.__getattribute__('status')
    # print "q.__getattribute__('result')"
    # print q.__getattribute__('result')
    #global globalValue
    #globalValue = q.result
    #f = goal_success()
    #f.public_value = q.result
    #print f()
    #print p.status.status

def loopLocations():
    print("loop =")
    print(oop.looperEN.get())
    ## if data == 1:
    x798 = 1

def sequenceLocations():
    print("Sequence = ")
    print(oop.sequenceEN.get())
    x798 = 2

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

def check_goal():
    print ("Waiting for goal to be reached")
   # goal_feed()
    global goalSuccess
    if goalSuccess == 3:
        print("Goal success!")
        return 1
    else:
        print("Not there yet")
        return 0
    #compare with odometry
    #time.sleep(15)
    #return 1


flag = 0

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2','outcome6'],
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        while not rospy.is_shutdown():
            rospy.loginfo('Executing state FOO')
            time.sleep(0)
            if userdata.foo_counter_in < 3:
                userdata.foo_counter_out = userdata.foo_counter_in + 1
                return 'outcome1'
            else:
                userdata.foo_counter_out = 0
                if flag == 0:
                    return 'outcome6'
                else:
                    return 'outcome1'

#define state LaunchState
class LaunchState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome7'],
                             input_keys=['flag'])

    def execute(self, userdata):
        while not rospy.is_shutdown():
            rospy.loginfo('Launch state has been entered, this should only happen once!')
            print "Pre-flag set:"
            global flag
            print flag
            flag = 1
            print "Post-flag set:"
            print flag
            #start_rosserial()
            #time.sleep(2)
            #ui_init()
       #     goal_listener()
            if lidar == "rp":
                configure_slambot_rp()
            elif lidar == "hokuyo":
                configure_slambot()
            else:
                print("Please select a valid laser scanner (defined at top of code)")
            #time.sleep(2)
            #start_move_base()
            #time.sleep(2)
            #start_rviz()
            start_smachviewer()
            return 'outcome7'




goal = 0

class goalSet_state(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome0'],
                             input_keys=['flag'])

    def execute(self, userdata):
        while not rospy.is_shutdown():
            rospy.loginfo('goalSet_state entered, setting goal')
            set_goal()
            #time.sleep(1.5)
           # if check_goal() == 1:
            global goal
            #if oop.numberOut.get() == 0:
             #   return 'outcome0'
            #else:
            return 'outcome1'






class goalCheck_state(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1','stay'],
                             input_keys=['flag'])

    def execute(self, userdata):
        while not rospy.is_shutdown():
            rospy.loginfo('goalCheck_state entered, waiting for goal to be reached')
            global fileNumber
            if oop.numberOut.get() == 55:
                return 'stay'
            time.sleep(4)
            if check_goal() == 1:
                print("We're at the goal, success!")
                if oop.sequenceEN.get() == 1:
                    print("incrimenting...")
                    oop.numberOut.set(oop.numberOut.get()+1)
                    #if oop.looperEN == 1:
                    if oop.numberOut.get() == 2 :
                        oop.numberOut.set(0)
                        return 'outcome1'
                    else:
                        return 'outcome1'
                #waitkey("q");
                return 'outcome1'
            else:
                time.sleep(3)
                return 'stay'



stateKick = False 

#define state waiter
class waitState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome11', 'outcome1'],
                             input_keys=['result'])

    def execute(self, userdata):
        while not rospy.is_shutdown():
            rospy.loginfo('Waiting for goal success')
            time.sleep(3)
            #time.sleep(2)
            ##############################unbagger()
            #c = goal_listener()
            print "#########################################################################################"
            print "f()"
            print "Running: listener_tester()"
            listener_tester()
#            print globalValue
            global stateKick
            if stateKick == True:
                stateKick = False
                return 'outcome1'
            else:
                return 'outcome11'
            #global q
            # #global q.result
            # if globalValue == "goal success!":
            #     global globalValue
            #     globalValue = "empty"
            #     return 'outcome11'
            # else:
            #     return 'outcome1'

def leaveWaitState():
    global stateKick
    stateKick = True



'''
#define state goalWait
class goalWait(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome 1', 'outcome2'],
                             input_keys=['goalSuccess'])

    def execute(self, userdata):
        while not rospy.is_shutdown():
            rospy.loginfo('Waiting for navigation stack to reach goal coordinate')
            if goalSuccess
'''
# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in'])
        
    def execute(self, userdata):
        while not rospy.is_shutdown():
            rospy.loginfo('Executing state BAR')
            #time.sleep(1)
           # goal_listener()
            talker(unbagger())
            rospy.loginfo('Counter = %f'%userdata.bar_counter_in)        
            return 'outcome1'
        



class StdRedirector():
    def __init__(self, text_widget):
        self.text_space = text_widget

    def write(self, string):
        self.text_space.config(state=tkinter.NORMAL)
        self.text_space.insert("end", string)
        self.text_space.see("end")
        self.text_space.config(state=mtTkinter.DISABLED)

class Std_redirector(object):
    def __init__(self,widget):
        self.widget = widget

    def write(self,string):
        self.widget.insert(END,string)
        self.widget.see(END)



class OOP():
    def __init__(self):


        self.root = Tk()
        #self.root = tk.Tk()
        self.root.title("Dalek controller v1.65")

        subprocess.Popen("killall -9 roscore", shell=True)
        time.sleep(2)
        subprocess.Popen("roscore", shell=True)
        self.PIDpublisher = rospy.Publisher('/PID_tuneAll', Pose, queue_size=2)
        self.speedPublisher = rospy.Publisher('/speed_all', Pose, queue_size=2)
        self.motorENpub = rospy.Publisher('/controller_enable', Bool, queue_size=2)
        self.locationSubscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, locationHolder)

        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.createGUI()

        #sys.stdout = StdRedirector(self.mov_pane)
        sys.stdout = Std_redirector(self.t)
       # sys.stderr = StdRedirector(self.t)

    def controllerToggle(self):
        self.motorENpub.publish(self.motorEN.get())

    def _emergencyStop(self, dummy):
        haltController = BooleanVar()
        haltController.set(False)
        self.motorENpub.publish(haltController.get())
        self.motorEN.set(False)

    def speed_setup(self):
        self.speedFrame = ttk.Frame(self.mainframe)
        self.speedFrame.grid(column=2, row=0, sticky=(N, W))
        self.speedFrame.columnconfigure(0, weight=1)
        self.speedFrame.columnconfigure(1, weight=1, minsize=25)
        self.speedFrame.columnconfigure(2, weight=1)
        self.speedFrame.columnconfigure(3, weight=1, minsize=25)


        self.xLimit = Scale(self.speedFrame, from_=0.05, to=1.0, orient=HORIZONTAL, resolution=0.05)
        self.xLimit.config(length=200, label="Forwards", command=self._speedPub, variable=self.vXlim)
        self.xLimit.grid(column=0, row=0)

        self.xLen = Entry(self.speedFrame)
        self.xLen.bind('<Return>', lambda e: setSlider(self.xLimit, \
                                                       self.xLen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.xLen.config(width=3)
        self.xLen.grid(column=1, row=0, sticky=(W))

        self.NxLimit = Scale(self.speedFrame, from_=-0.05, to=-1.0, orient=HORIZONTAL, resolution=0.05)
        self.NxLimit.config(length=200, label="Reverse", command=self._speedPub, variable=self.NvXlim)
        self.NxLimit.grid(column=0, row=1)

        self.NxLen = Entry(self.speedFrame)
        self.NxLen.bind('<Return>', lambda e: setSlider(self.NxLimit, \
                                                        self.NxLen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.NxLen.config(width=3)
        self.NxLen.grid(column=1, row=1, sticky=(W))

        self.zLimit = Scale(self.speedFrame, from_=0.05, to=1.0, orient=HORIZONTAL, resolution=0.05)
        self.zLimit.config(length=200, label="Left turn", command=self._speedPub, variable=self.vZlim)
        self.zLimit.grid(column=2, row=0)

        self.zLen = Entry(self.speedFrame)
        self.zLen.bind('<Return>', lambda e: setSlider(self.zLimit, \
                                                       self.zLen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.zLen.config(width=3)
        self.zLen.grid(column=3, row=0, sticky=(W))

        self.NzLimit = Scale(self.speedFrame, from_=-0.05, to=-1.0, orient=HORIZONTAL, resolution=0.05)
        self.NzLimit.config(length=200, label="Right turn", command=self._speedPub, variable=self.NvZlim)
        self.NzLimit.grid(column=2, row=1)

        self.NzLen = Entry(self.speedFrame)
        self.NzLen.bind('<Return>', lambda e: setSlider(self.NzLimit, \
                                                        self.NzLen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.NzLen.config(width=3)
        self.NzLen.grid(column=3, row=1, sticky=(W))

        self.limitThreshold = Scale(self.speedFrame, from_=0.05, to=1.0, orient=HORIZONTAL, resolution=0.05)
        self.limitThreshold.config(length=200, label="Limit Threshold", command=self._speedPub,
                                   variable=self.limitThresh)
        self.limitThreshold.grid(column=0, row=3)

        self.limThreshEn = Entry(self.speedFrame)
        self.limThreshEn.bind('<Return>', lambda e: setSlider(self.limitThreshold, \
                                                              self.limThreshEn))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.limThreshEn.config(width=3)
        self.limThreshEn.grid(column=1, row=3, sticky=(W))

        self.v_left_box = Text(self.speedFrame, width=18)
        self.v_left_box.grid(column=0, row=4, sticky=(W))

        self.v_right_box = Text(self.speedFrame, width=18)
        self.v_right_box.grid(column=2, row=4, sticky=W)

        self.speedValues = self.speedPacket(0.5, -0.5, 0.75, -0.75, 0.25)

        self.vXlim.set(0.45)
        self.NvXlim.set(-0.45)
        self.vZlim.set(0.60)
        self.NvZlim.set(-0.60)
        self.limitThresh.set(0.25)


    def PID_setup(self):
        global PIDen
        PIDen = True

        self.PIDframe = ttk.Frame(self.mainframe)
        self.PIDframe.grid(column=1, row=0, sticky=(N, W))  # , W, E, S))
        self.PIDframe.columnconfigure(0, weight=1)
        self.PIDframe.columnconfigure(1, weight=1, minsize=25)
        self.PIDframe.columnconfigure(2, weight=1)
        self.PIDframe.columnconfigure(3, weight=1, minsize=25)


        self.lPscale = Scale(self.PIDframe, from_=0.0, to=20.0, orient=HORIZONTAL, resolution=0.01)
        self.lPscale.config(length=200, label="Proportional gain", command=self._PIDpub, variable=self.lkp)
        self.lPscale.grid(column=0, row=0)

        self.lIscale = Scale(self.PIDframe, from_=0.0, to=20.0, orient=HORIZONTAL,
                        resolution=0.01)  # , command=leftWheel.send(ki))
        self.lIscale.config(length=200, label="Integral gain", command=self._PIDpub, variable=self.lki)
        self.lIscale.grid(column=0, row=1)

        self.lDscale = Scale(self.PIDframe, from_=0.0, to=20.0, orient=HORIZONTAL, resolution=0.01)
        self.lDscale.config(length=200, label="Derivative gain", command=self._PIDpub, variable=self.lkd)
        self.lDscale.grid(column=0, row=2)

        self.lPen = Entry(self.PIDframe)
        self.lPen.bind('<Return>', lambda e: setSlider(self.lPscale,\
                                                       self.lPen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.lPen.config(width=3)
        self.lPen.grid(column=1, row=0, sticky=(W))

        self.lIen = Entry(self.PIDframe)
        self.lIen.bind('<Return>', lambda e: setSlider(self.lIscale,
                                                       self.lIen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.lIen.config(width=3)
        self.lIen.grid(column=1, row=1, sticky=(W))

        self.lDen = Entry(self.PIDframe)
        self.lDen.bind('<Return>', lambda e: setSlider(self.lDscale,
                                                       self.lDen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.lDen.config(width=3)
        self.lDen.grid(column=1, row=2, sticky=(W))

        self.rPscale = Scale(self.PIDframe, from_=0.0, to=20.0, orient=HORIZONTAL, resolution=0.01)
        self.rPscale.config(length=200, label="Proportional gain", command=self._PIDpub, variable=self.rkp)
        self.rPscale.grid(column=2, row=0)

        self.rIscale = Scale(self.PIDframe, from_=0.0, to=20.0, orient=HORIZONTAL,
                        resolution=0.01)  # , command=leftWheel.send(ki))
        self.rIscale.config(length=200, label="Integral gain", command=self._PIDpub, variable=self.rki)
        self.rIscale.grid(column=2, row=1)

        self.rDscale = Scale(self.PIDframe, from_=0.0, to=20.0, orient=HORIZONTAL, resolution=0.01)
        self.rDscale.config(length=200, label="Derivative gain", command=self._PIDpub, variable=self.rkd)
        self.rDscale.grid(column=2, row=2)

        self.rPen = Entry(self.PIDframe)
        self.rPen.bind('<Return>', lambda e: setSlider(self.rPscale,
                                                       self.rPen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.rPen.config(width=3)
        self.rPen.grid(column=3, row=0, sticky=(W))

        self.rIen = Entry(self.PIDframe)
        self.rIen.bind('<Return>', lambda e: setSlider(self.rIscale,
                                                       self.rIen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.rIen.config(width=3)
        self.rIen.grid(column=3, row=1, sticky=(W))

        self.rDen = Entry(self.PIDframe)
        self.rDen.bind('<Return>', lambda e: setSlider(self.rDscale,
                                                       self.rDen))  # hitting ENTER in the entry box will run setSlider() on the joint
        self.rDen.config(width=3)
        self.rDen.grid(column=3, row=2, sticky=(W))

        self.lPscale.set(1.4)
        self.lIscale.set(0.14)
        self.lDscale.set(0.01)

        self.rPscale.set(1.4)
        self.rIscale.set(0.14)
        self.rDscale.set(0.01)

        self.PID_enable = ttk.Checkbutton(self.PIDframe, text='Tick to enable PID', command=self._togglePID, variable=self.PIDstatus,
                                     onvalue=1, offvalue=0)
        self.PID_enable.grid(column=0, row=3)

        ttk.Button(self.PIDframe, text="Reset to default PID values",
                   command=lambda: resetSlider(self.lPscale, self.lIscale, self.lDscale, self.rPscale, self.rIscale, self.rDscale)).grid(column=2,
                                                                                                           row=4,
                                                                                                           sticky=(
                                                                                                           N, W))

        ttk.Button(self.PIDframe, text="RQT plot (PID graph)", command=self._rqt_plot).grid(column=0, row=4, sticky=(N, W))

    def _speedPub(self, data):
        self.speedValues.vXlim = self.vXlim.get()
        self.speedValues.NvXlim = self.NvXlim.get()
        self.speedValues.vZlim = self.vZlim.get()
        self.speedValues.NvZlim = self.NvZlim.get()
        self.speedValues.limitThresh = self.limitThresh.get()

        self.speedValues.send()

    def _PIDpub(self, data):
        # leftWheel = PIDpacket(1.4, 0.14, 0.01)
        #   print("gain: ")
        # leftWheel.ki = gain.get()
        #    print(ki.get())
        self.PIDvalues.lkp = self.lkp.get()
        self.PIDvalues.lki = self.lki.get()
        self.PIDvalues.lkd = self.lkd.get()

        self.PIDvalues.rkp = self.rkp.get()
        self.PIDvalues.rki = self.rki.get()
        self.PIDvalues.rkd = self.rkd.get()
        self.PIDvalues.send()

    def _rqt_plot(self):
        self.rqt_plot_proc = subprocess.Popen("rqt", shell=True)

    def _togglePID(self):
        self.PIDvalues.PIDenable = self.PIDstatus.get()
        self.PIDvalues.send()

  #  def _clearOdom(self):
  #      self.




    def _listener(self):
        rospy.Subscriber('/cmd_vel', Twist, cmd_print)
        rospy.Subscriber('/chatter', String, chatter_print)
        rospy.Subscriber('/v_left', Float32, v_left_print)
        rospy.Subscriber('/v_right', Float32, v_right_print)
       # rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_feed)
        rospy.Subscriber('/move_base/status', GoalStatusArray, goal_feed)

    def _buttonFrameSetup(self):
        self.buttonFrame = ttk.Frame(self.mainframe)
        self.buttonFrame.grid(column=0, row=0, sticky=(N, W))  # , W, E, S))
        self.buttonFrame.columnconfigure(0, weight=1, minsize=185)
        self.buttonFrame.columnconfigure(1, weight=1, minsize=120)
        self.buttonFrame.columnconfigure(2, weight=1, minsize=420)

        ttk.Button(self.buttonFrame, text="Start/Restart Rviz", command=rvizLaunch).grid(column=0, row=0,
                                                                                         sticky=(N, W))
        ttk.Button(self.buttonFrame, text="Start/Restart Lidar driver", command=lidarLaunch).grid(column=0, row=1,
                                                                                                  sticky=(N, W))
        ttk.Button(self.buttonFrame, text="Start/Restart Move_base", command=restartMovebase).grid(column=0, row=2,
                                                                                                   sticky=(N, W))
        ttk.Button(self.buttonFrame, text="Start manual robot steering", command=startRQTsteering).grid(column=0,
                                                                                                        row=3,
                                                                                                        sticky=( N, W))

        ttk.Button(self.buttonFrame, text="Leave wait-state", command=leaveWaitState).grid(column=1, row=0, sticky=(N, W))

        aLabel = ttk.Label(self.buttonFrame, text="Click below to execute location:")
        aLabel.grid(column=2, row=0, sticky=(W))

        #self.number = StringVar()
        number = StringVar()
        self.numberOut = IntVar()

        def clickMe():
            #action.configure(text="** I have been Clicked! **")
            action.configure(text='Location ' + number.get() + ' Selected')
            self.numberOut.set(int(number.get()))

        #aLabel.configure(foreground='red')  # 5
        # Adding a Button
        action = ttk.Button(self.buttonFrame, text="Click Me!", command=clickMe)
        action.grid(column=2, row=1, sticky=(W))


        ttk.Label(self.buttonFrame, text="Choose location:").grid(column=1, row=2, sticky=(W))

        numberChosen = ttk.Combobox(self.buttonFrame, width=12, textvariable=number, state='readonly')

        numberChosen['values'] = (1, 2, 3, 0, 55)
        numberChosen.grid(column=2, row=2, sticky=W)
        numberChosen.current(3)


    def _locationFrameSetup(self):
        self.locationFrame = ttk.Frame(self.buttonFrame)
        self.locationFrame.grid(column=1, row=3, columnspan=2, sticky=(N, W, S, E))  # , W, E, S))
        self.locationFrame.columnconfigure(0, weight=1, minsize=185)
        self.locationFrame.columnconfigure(1, weight=1, minsize=120)
        self.locationFrame.columnconfigure(2, weight=1, minsize=100)

        ttk.Button(self.locationFrame, text="Create bag", command=locationBagger).grid(column=0, row=0,
                                                                                         sticky=(N, W))
       # ttk.Button(self.locationFrame, text="yyyyyyyyyyyyyyyyyy", command=lidarLaunch).grid(column=0, row=1,
#                                                                                              sticky=(N, W))

        self.looperEN = BooleanVar()
        self.sequenceEN = BooleanVar()

        self.loop_enable = ttk.Checkbutton(self.locationFrame, text='Tick to loop', command=loopLocations, variable=self.looperEN,
                                     onvalue=1, offvalue=0)
        self.loop_enable.grid(column=0, row=3)


        self.sequence = ttk.Checkbutton(self.locationFrame, text='Tick to sequence', command=sequenceLocations, variable=self.sequenceEN,
                                     onvalue=1, offvalue=0)
        self.sequence.grid(column=0, row=2)

        ttk.Button(self.locationFrame, text = "Clear bag counter (overwrite)", command=clearBagCounter).grid(column=0, row=4, sticky = (N, W))
    # ttk.Button(
        self.motorEN = BooleanVar()

        self.motorToggle = ttk.Checkbutton(self.locationFrame, text="Enable/Disable motor controller", command=self.controllerToggle, variable=self.motorEN).grid(column=0, row=5)

    def _pubber(self):
        # pub = rospy.Publisher('forwardedGoals', PoseStamped, queue_size=10)
        #self.data = self.pubData.get()
        #    data = "hello"
        self.pub.publish(self.pubData.get())

    class speedPacket(object):
        def __init__(self, vXlim, NvXlim, vZlim, NvZlim, limitThresh):
            self.vXlim = vXlim
            self.vZlim = vZlim
            self.NvXlim = NvXlim
            self.NvZlim = NvZlim
            self.limitThresh = limitThresh


        def send(self):
            speed_all = Pose()
            speed_all.orientation.x = self.vXlim
            speed_all.orientation.y = self.NvXlim
            speed_all.orientation.z = self.vZlim
            speed_all.orientation.w = self.NvZlim

            speed_all.position.x = self.limitThresh

            oop.speedPublisher.publish(speed_all)

    class PIDpacket(object):

        def __init__(self, lkp, lki, lkd, rkp, rki, rkd, PIDenable):
            self.lkp = lkp
            self.lki = lki
            self.lkd = lkd

            self.rkp = rkp
            self.rki = rki
            self.rkd = rkd

            self.PIDenable = PIDenable

        def send(self):
            pid_all = Pose()
            pid_all.position.x = self.lkp
            pid_all.position.y = self.lki
            pid_all.position.z = self.lkd

            pid_all.orientation.x = self.rkp
            pid_all.orientation.y = self.rki
            pid_all.orientation.z = self.rkd

            pid_all.orientation.w = self.PIDenable

            oop.PIDpublisher.publish(pid_all)


    def createGUI(self):

        if (mySettings == 1):
            self.p = subprocess.Popen("rosrun rosserial_python serial_node.py /dev/ttyACM*", stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, shell=True)

            self.movebase_proc = subprocess.Popen("roslaunch slambot_2dnav move_base.launch", stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, shell=True)

       # rospy.init_node('tkinter_test_node', anonymous=True)

        self.PIDvalues = self.PIDpacket(1.4, 0.14, 0.01, 1.4, 0.14, 0.01, 1)
        self.speedValues = self.speedPacket(0.5, 0.5, 0.5, -0.5, 0.25)

        self.mainframe = ttk.Frame(self.root, padding="3 3 12 12")
        self.mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
        self.mainframe.columnconfigure(0, weight=1, minsize=25)
        self.mainframe.columnconfigure(1, weight=1, minsize=25)
        self.mainframe.columnconfigure(2, weight=1, minsize=25)
        self.mainframe.rowconfigure(0, weight=1, minsize=260)
        self.mainframe.rowconfigure(1, weight=1, minsize=200)
        self.mainframe.rowconfigure(2, weight=1)
        self.mainframe.rowconfigure(3, weight=1)

        self.lkp = DoubleVar()
        self.lki = DoubleVar()
        self.lkd = DoubleVar()

        self.rkp = DoubleVar()
        self.rki = DoubleVar()
        self.rkd = DoubleVar()

        self.vXlim = DoubleVar()
        self.NvXlim = DoubleVar()
        self.vZlim = DoubleVar()
        self.NvZlim = DoubleVar()
        self.limitThresh = DoubleVar()

        #PIDframe.rowconfigure(0, weight=1)
        #PIDframe.rowconfigure(1, weight=1)
        #PIDframe.rowconfigure(2, weight=1)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        #movebasePane()
        #self.mov_pane = Text(self.mainframe)#, width=40, height=10)
        #self.mov_pane.grid(column=2, row=1, sticky=(N, E, S, W))

        #serialPane()
        self.t = Text(self.mainframe)#, width=40, height=10)
        self.t.grid(column=0, row=1, rowspan=2, columnspan=2, sticky=(N, E, S, W))


        #ttk.Button(self.mainframe, text="Restart Rosserial", command=restartSerial).grid(column=0, row=1, sticky=(N, W))
        #ttk.Button(self.mainframe, text="Restart Move_base", command=restartMovebase).grid(column=1, row=1, sticky=(N, W))

        self.cmd_vel = Text(self.mainframe, width=18)
        self.cmd_vel.grid(column=2, row=1, rowspan=2, sticky=(N, W))

        self.chatterBox = Text(self.mainframe, width=18)
        self.chatterBox.grid(column=3, row=2, sticky=(N, W))



        self.PIDstatus = IntVar()
        self.PIDstatus.set(0)
        self.PID_setup()
        self.speed_setup()
        self._listener()
        self._buttonFrameSetup()
        self._locationFrameSetup()


        self.textpub_pane = ttk.Frame(self.mainframe, padding="3 3 12 12")
        self.textpub_pane.grid(column=3, row=1, sticky=(S, E, W))
        self.pubData = StringVar()
        self.pub_entry = ttk.Entry(self.textpub_pane, width=16, textvariable=self.pubData)
        self.pub_entry.grid(column=3, row=0, sticky=(S, E, N))
        ttk.Button(self.textpub_pane, text="Publish", command=self._pubber).grid(column=3, row=1, sticky=(S, E, W))




        for child in self.mainframe.winfo_children(): child.grid_configure(padx=5, pady=5)

        self.pub_entry.focus()
        self.root.bind('<Return>', self._pubber)
        self.root.bind('<Escape>', self._emergencyStop)

def _startSerial():
    print("Starting restartSerial")
    #global oop
    while True:
        line = oop.p.stdout.readline()
        oop.t.insert(END, line)
        oop.t.see("end")
        print line
        if not line: break

def _startMovebase():
    print("Starting move_base")
    global oop
    while True:
        line = oop.movebase_proc.stdout.readline()
        oop.mov_pane.insert(END, line)
        oop.mov_pane.see("end")
        if not line: break

#
# class LoginFrame(Frame):
#     def __init__(self, master):
#         #super(int).__init__(master)
#
#         self.loginFrame = ttk.Frame(oop.root, padding="3 3 12 12")
#         self.loginFrame.grid(column=0, row=0, sticky=(N, W, E, S))
#         self.loginFrame.columnconfigure(0, weight=1, minsize=25)
#
#         self.label_1 = Label(self, text="Username")
#         self.label_2 = Label(self, text="Password")
#
#         self.entry_1 = Entry(self)
#         self.entry_2 = Entry(self, show="*")
#
#         self.label_1.grid(row=0, sticky=E)
#         self.label_2.grid(row=1, sticky=E)
#         self.entry_1.grid(row=0, column=1)
#         self.entry_2.grid(row=1, column=1)
#
#         self.checkbox = Checkbutton(self, text="Keep me logged in")
#         self.checkbox.grid(columnspan=2)
#
#         self.logbtn = Button(self, text="Login", command = self._login_btn_clickked)
#         self.logbtn.grid(columnspan=2)
#
#         self.pack()

    def _login_btn_clickked(self):
        #print("Clicked")
        username = self.entry_1.get()
        password = self.entry_2.get()

        #print(username, password)

        if username == "john" and password == "password":
            tm.showinfo("Login info", "Welcome John")
        else:
            tm.showerror("Login error", "Incorrect username")


oop = OOP()


#lf = LoginFrame(oop.mainframe)

def redirector(inputStr):
    oop.mov_pane.insert(INSERT, inputStr)


def main():
    #Start roscore
    start_roscore()
   # sys.stdout.write = redirector  # whenever sys.stdout.write is called, redirector is called.

    #serialThread = threading.Thread(target=_startSerial)
    #serialThread.daemon = True
    #serialThread.start()
    #print("Starting serial thread")

    #movebaseThread = threading.Thread(target=_startMovebase)
    #movebaseThread.daemon = True
    #movebaseThread.start()
    #print("Starting move_base thread")


    rospy.init_node('smach_playing_with_python', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4',
                                            'outcome6':'LAUNCHSTATE'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})

        smach.StateMachine.add('LAUNCHSTATE', LaunchState(),
                               transitions={'outcome7':'WAITSTATE'},
                               remapping={'flag':'flag'})

        smach.StateMachine.add('WAITSTATE', waitState(),
                               transitions={'outcome11':'WAITSTATE',
                                            'outcome1':'goalSet_state'},
                               remapping={'result':'result'})

        smach.StateMachine.add('goalSet_state', goalSet_state(), 
                                transitions={'outcome1':'goalCheck_state',
                                             'outcome0':'goalSet_state'})

        smach.StateMachine.add('goalCheck_state', goalCheck_state(),
                                transitions={'outcome1':'goalSet_state',
                                              'stay':'goalCheck_state'})


    # Execute SMACH plan


    def executeSmach():
        outcome = sm.execute()

    smachThread = threading.Thread(target=executeSmach)
    smachThread.daemon = True
    smachThread.start()
    print("Executing smach")


    # Wait for ctrl-c to stop the application
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    oop.root.mainloop()
    sis.stop()
   # oop.root.destroy()


    #if __name__ == '__main__':
    #    try:
    #        talker()
    #    except rospy.ROSInterruptException:
    #        pass
    #----------------------------------------------------------------
    #if __name__ == '__main__':
    #    listener()


if __name__ == '__main__':
    main()

