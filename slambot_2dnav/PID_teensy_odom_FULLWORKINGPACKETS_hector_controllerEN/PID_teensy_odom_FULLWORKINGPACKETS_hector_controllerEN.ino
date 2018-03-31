  #include <NewPing.h>


//#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>  
#include <ros/time.h>
#include <tf/tf.h>
//#include <rosproxy_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include "dalekMotors.h"
#include "dalekSonics.h"
//#include "PID.h"
#include "PID_v1.h"
//#define HWSERIAL Serial1 // change for available pins  

//#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#define wheel_spacing 0.505
#define speedLimit 0.15
#define debug_leftEncoder


//#define doubleResolution

#define sonic2 19
NewPing sonar(A4, A4, 300);
NewPing sonar2(A5, A5, 300);

#define controllerConstant 6
#define controllerPulse 5


#define wheelSlots 128

#define controllerConstant 6
#define controllerPulse 5



ros::NodeHandle nh;
float tKp = 400;
float tKi = 4000;



char frameid[] = "/ultrasound";

nav_msgs::Odometry nav_msgs_odom;
//ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
ros::Publisher serial_odom("serial_odom", &nav_msgs_odom);
//  odom_pub.publish(odom);

tf::TransformBroadcaster odom_broadcaster;





//left and right wheel counter publishers
std_msgs::Int16 int_msg;
ros::Publisher rightWheel("rightWheel", &int_msg);
ros::Publisher leftWheel("leftWheel", &int_msg);
ros::Publisher iDebug("iDebug", &int_msg);

std_msgs::Float32 float32_msg;
ros::Publisher fDebug("fDebug", &float32_msg);
//publishers for v_left and v_right
ros::Publisher v_left_pub("v_left_pub", &float32_msg);
ros::Publisher v_right_pub("v_right_pub", &float32_msg);

ros::Publisher x_out_sub("x_out_sub", &float32_msg);
ros::Publisher z_out_sub("z_out_sub", &float32_msg);

//Left wheel PId input, output and setpoints
ros::Publisher lInputP("lInputP", &float32_msg);
ros::Publisher lOutputP("lOutputP", &float32_msg);
ros::Publisher lSetpointP("lSetpointP", &float32_msg);

//Right wheel PId input, output and setpoints
ros::Publisher rInputP("rInputP", &float32_msg);
ros::Publisher rOutputP("rOutputP", &float32_msg);
ros::Publisher rSetpointP("rSetpointP", &float32_msg);

//general chatter for debug
std_msgs::String str_msg_main;
ros::Publisher chatter("chatter", &str_msg_main);


//ultrasound publishers
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);
ros::Publisher pub_range2( "/ultrasound2", &range_msg);



float x1, z1;
void messageCb( const geometry_msgs::Twist& winner){
  x1 = winner.linear.x;
  z1 = winner.angular.z;
}


ros::Subscriber<geometry_msgs::Twist> cmd_vel_subscriber("cmd_vel", &messageCb);

unsigned long lTime, rTime = 0;

const byte leftInterruptPin = 0; //int 0, pin 2
const byte rightInterruptPin = 1; //int 1, pin 3
const byte leftStatusPin = 3; //d5
const byte rightStatusPin = 2;


volatile int lFlag = 0;
volatile int rFlag = 0;

float leftWheelDistance = 0.00;
float rightWheelDistance = 0.00;

long lLeftWheelDistance = 0L;
long lRightWheelDistance = 0L;

boolean  leftTriggered = true;
boolean  rightTriggered = true;

int leftWheelCounter = 0;
int rightWheelCounter = 0;



int newposition;
boolean calculated = false;

float v_left, v_right, old_v_left;

//[-25.624998, -25.624998, 0.000000]
//double x = 25.624998;
//double y = 25.624998;
double x = 0.0;
double y = 0.0;
double th = 0;

double vx = 0;
double vth = 0;

//ros::Time Llast_time, Rlast_time, current_time, last_time;
long lCurrent_time, lLast_time;
//long current_time, last_time;

float lKp = 1.4;
float lKi = 0.14;
float lKd = 0.01;//.5;

float rKp = 1.4;
float rKi = 0.14;
float rKd = 0.01;//.5;


double lSetpoint, lInput, lOutput;
double rSetpoint, rInput, rOutput;
double xout, zout, VL, VR;
double lError;

PID leftPID(&lInput, &lOutput, &lSetpoint,lKp,lKi,lKd, DIRECT);
PID rightPID(&rInput, &rOutput, &rSetpoint,rKp,rKi,rKd, DIRECT);

long timeout;

boolean PIDen = false;
long pubTime = 0;

/*void PIDtoggle(pidEnable){
 
 PIDen = PIDenable ? true : false;
 
 }
 */
double x1Lim = 0.55;
double x1LimNeg = -0.55;
double z1Lim = 0.65;
double z1LimNeg = -0.65;
double limitThreshold = 0.25;

geometry_msgs::Pose speed_all;
void speed_all_cb(const geometry_msgs::Pose& speed_all){
  x1Lim = speed_all.orientation.x;
  x1LimNeg = speed_all.orientation.y;
  z1Lim = speed_all.orientation.z;
  z1LimNeg = speed_all.orientation.w;

  limitThreshold = speed_all.position.x;
}


boolean clearOdom = true;
geometry_msgs::Pose pid_all;
void pid_all_cb(const geometry_msgs::Pose& pid_all){
  lKp = pid_all.position.x;
  lKi = pid_all.position.y;
  lKd = pid_all.position.z;

  rKp = pid_all.orientation.x;
  rKi = pid_all.orientation.y;
  rKd = pid_all.orientation.z;

  PIDen = boolean(pid_all.orientation.w);

  leftPID.SetTunings(lKp, lKi, lKd);
  rightPID.SetTunings(rKp, rKi, rKd);
}

void settings_cb(const geometry_msgs::Pose& settings){
  clearOdom = boolean(settings.position.x);
}

void controller_en_cb(const std_msgs::Bool& controller_en){
  if (controller_en.data){
    startController();
  } 
  else if (!controller_en.data){
    stopController();
  }
}


ros::Subscriber<geometry_msgs::Pose> PID_tune_sub("PID_tuneAll", pid_all_cb);
ros::Subscriber<geometry_msgs::Pose> speed_set_sub("speed_all", speed_all_cb);

ros::Subscriber<std_msgs::Bool> controller_en_sub("/controller_enable", controller_en_cb);



boolean debugFlag = true;

void startController(){
  stopMotors();
  delay(100);
  digitalWrite(controllerConstant, HIGH);
  delay(1500);
  /*delay(500);*/
   digitalWrite(controllerPulse, LOW);
   delay(300);
   digitalWrite(controllerPulse, HIGH);
   delay(800);
   digitalWrite(controllerPulse, LOW);
   delay(900);
   
  /* digitalWrite(controllerPulse, HIGH);
   delay(100);
   digitalWrite(controllerPulse, LOW);
   delay(900);*/
}

void stopController(){
  digitalWrite(controllerConstant, LOW);
}

void restartController(){
  stopController();
  delay(500);
  startController();
}

void controllerTest(){
  pinMode(controllerConstant, OUTPUT);
  pinMode(controllerPulse, OUTPUT);
  digitalWrite(controllerPulse, LOW);
  digitalWrite(controllerConstant, LOW);
  delay(500);
  digitalWrite(controllerPulse, HIGH);
  digitalWrite(controllerConstant, HIGH);  
  delay(500);
  digitalWrite(controllerPulse, LOW);
  digitalWrite(controllerConstant, LOW);
  delay(500);
  digitalWrite(controllerPulse, HIGH);
  digitalWrite(controllerConstant, HIGH);  
  delay(500);
}




void setup ()
{

  // HWSERIAL.begin(9600);

  //For motors
  stopMotors();
  nh.initNode();

  // broadcaster.init(nh);


  //  odom_broadcaster.init(nh);
  //  nh.advertise(odom_broadcaster);

  nh.subscribe(controller_en_sub);
  nh.subscribe(PID_tune_sub);
  nh.subscribe(cmd_vel_subscriber);
  nh.subscribe(speed_set_sub);


  //  nh.advertise(theta);
  //  nh.advertise(vtheta);
  nh.advertise(fDebug);
  nh.advertise(iDebug);

  nh.advertise(z_out_sub);
  nh.advertise(x_out_sub);

  nh.advertise(rightWheel);
  nh.advertise(leftWheel);

  nh.advertise(v_left_pub);
  nh.advertise(v_right_pub);

  nh.advertise(lInputP);
  nh.advertise(lOutputP);
  nh.advertise(lSetpointP);

  nh.advertise(rInputP);
  nh.advertise(rOutputP);
  nh.advertise(rSetpointP);

  //  nh.advertise(serial_odom);


  nh.advertise(chatter);




  //For sonar 
  nh.advertise(pub_range);
  nh.advertise(pub_range2);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.03;
  range_msg.max_range = 3.00;
  //  range_msg.max_range = 2.01;

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);  

  leftPID.SetOutputLimits(-1, 1);
  rightPID.SetOutputLimits(-1, 1);
  //stopMotors();

  //Arduino LED, set as output
  pinMode(13, OUTPUT);
  pinMode(2, INPUT);
  pinMode(5, INPUT);



  //Setup interrupts
  pinMode(leftInterruptPin, INPUT);
  pinMode(leftStatusPin, INPUT);
  pinMode(rightInterruptPin, INPUT);
  pinMode(rightStatusPin, INPUT);
  /*
#if defined(doubleResolution)
   attachInterrupt(leftInterruptPin, leftTrigger, CHANGE);
   attachInterrupt(rightInterruptPin, rightTrigger, CHANGE);
   #else
   */
  attachInterrupt(leftInterruptPin, leftTrigger, RISING);
  attachInterrupt(rightInterruptPin, rightTrigger, RISING);
  //#endif


  //current_time = micros();
  //last_time = micros();

  //  current_time = nh.now();
  //  last_time = current_time;
  lCurrent_time = millis()/1000;
  lLast_time = lCurrent_time;


  //  digitalWrite(controllerConstant, LOW);
  //  digitalWrite(controllerPulse, LOW);

  

  pinMode(controllerConstant, OUTPUT);
  pinMode(controllerPulse, OUTPUT);

  digitalWrite(controllerConstant, LOW);
  digitalWrite(controllerPulse, LOW);

  //  digitalWrite(controllerConstant, HIGH);
  //delay(1000);
  digitalWrite(controllerPulse, LOW);
  startController();

  // controllerTest();
  /*
  digitalWrite(controllerPulse, LOW);
   delay(100);
   digitalWrite(controllerPulse, HIGH);
   delay(900);
   digitalWrite(controllerPulse, LOW);
   delay(100);
   digitalWrite(controllerPulse, HIGH);
   delay(900);*/





  //deomSeed(734);

}  // end of setup




//This runs if an ISR is called, which doesn't exist. Prevents the micro resetting on a bad ISR

float oldposition, Roldposition, Rnewposition;
int newtime, Roldtime, Rnewtime;
int oldtime;
float vel;

long range_time, range_time2, range_time3;
//int currentTime;
int duration2, distance2;
boolean on = true;
boolean lCalc, rCalc = true;


float old_v_right;
float rnewposition, roldposition;
int rnewtime;
int roldtime;

double theta = 0;

int millCheck = 0;
int millCheck2 = 0;


//char base_link[] = "/base_link2";
//char odom[] = "/odom_serial";

boolean restartFlag = false;

void loop ()
{

  //[-25.624998, -25.624998, 0.000000]
  //double x = 25.624998;
  //double y = 25.624998;


  if ((millis() > 5000) && (restartFlag)){
    // stopController();
    //delay(2000);
    //startController();
    restartController();
    restartFlag = false;
  }


  if ((leftTriggered) && (millis() >= lTime)&& (micros() > 2000000)){

    lFlag == 1 ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);

    lFlag == -1 ? leftWheelCounter += 1 : leftWheelCounter -= 1;

    //convert pulses to millimeters
    lLeftWheelDistance = long(leftWheelCounter*1037L);
    leftWheelDistance = lLeftWheelDistance / wheelSlots;

    int_msg.data = leftWheelDistance;
    leftWheel.publish( &int_msg);


    old_v_left = v_left;
    newposition = leftWheelDistance;
    newtime = millis();
    v_left = (newposition-oldposition)/(newtime-oldtime);
    oldposition = newposition;
    oldtime = newtime;
    //    v_left = (v_left + old_v_left)/2;

    float32_msg.data = v_left;
    v_left_pub.publish( &float32_msg);


    lTime = millis() + 10;
    lCalc = calculated = true;
    leftTriggered = false;
  }


  if (lTime < millis() + 150){
    v_left = 0;
  }

  /*  if ((millis() > lTime +10) ){
   newposition = leftWheelDistance;
   newtime = millis();
   v_left = (newposition-oldposition)/(newtime-(lTime-1000));
   oldposition = newposition;
   oldtime = newtime;
   
   }
   */

  // digitalRead(rightInterruptPin) == 1 ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);

  if ((rightTriggered) && (millis() >= rTime) && (micros() > 2000000)){

    rFlag == -1 ? rightWheelCounter = rightWheelCounter + 1 : rightWheelCounter = rightWheelCounter - 1;

    //convert pulses to millimeters
    lRightWheelDistance = long(rightWheelCounter*1037L);
    rightWheelDistance = lRightWheelDistance / wheelSlots;

    //convert millimeters to meters
    //  rightWheelDistance = rightWheelDistance / 1000;



    int_msg.data = rightWheelDistance;
    rightWheel.publish( &int_msg);

    old_v_right = v_right;
    rnewposition = rightWheelDistance;
    rnewtime = millis();
    v_right = (rnewposition-roldposition)/(rnewtime-roldtime);
    roldposition = rnewposition;
    roldtime = rnewtime;
    //    v_right = (v_right + old_v_right)/2;



    float32_msg.data = v_right;
    v_right_pub.publish( &float32_msg);

    rTime = millis() + 10;
    rCalc = calculated = true;
    rightTriggered = false;
  }

  if (rTime < millis() + 150){
    v_right = 0;
  }

  /*  if ((millis() > rTime +2) ){
   newposition = rightWheelDistance;
   newtime = millis();
   v_right = (newposition-roldposition)/(newtime-(rTime-1000));
   roldposition = newposition;
   roldtime = newtime;
   
   }
   */



  if (calculated && (millis() > 2000)){
    //compute odometry in a typical way given the velocities of the robot
    //  tempTime = ;
    vx = (v_left + v_right)/2;
    vth = (v_right - v_left)/wheel_spacing;

    // current_time = nh.now();
    lCurrent_time = millis()/1000;

    //compute odometry in a typical way given the velocities of the robot
    //   double dt = (current_time.toSec() - last_time.toSec());
    double dt = lCurrent_time - lLast_time;

    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    if (clearOdom){
      v_left = 0;
      v_right = 0;
      vx = 0;
      vth = 0;
      delta_x = 0;
      rightWheelCounter = 0;
      leftWheelCounter = 0;
      delta_y = 0;
      delta_th = 0;
      x = 25.624998;
      y = 25.624998; 
      th = 0;
      clearOdom = false;
    }



    calculated = false;

  }

  //    float32_msg.data = x;
  //    fDebug.publish( &float32_msg);

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  //    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  /*  if (millis() > pubTime){
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(th);
   
   //first, we'll publish the transform over tf
   geometry_msgs::TransformStamped odom_trans;
   odom_trans.header.stamp = nh.now();
   odom_trans.header.frame_id = "odom";
   odom_trans.child_frame_id = "laser";
   
   odom_trans.transform.translation.x = x;
   odom_trans.transform.translation.y = y;
   odom_trans.transform.translation.z = 0.0;
   odom_trans.transform.rotation = odom_quat;
   
   //send the transform
   odom_broadcaster.sendTransform(odom_trans);
   
   
   
   
   //next, we'll publish the odometry message over ROS
   nav_msgs::Odometry odom;
   odom.header.stamp = nh.now();
   
   odom.header.frame_id = "odom";
   
   //set the position
   odom.pose.pose.position.x = x;
   odom.pose.pose.position.y = y;
   odom.pose.pose.position.z = 0.0;
   odom.pose.pose.orientation = odom_quat;
   
   //set the velocity
   odom.child_frame_id = "laser";
   odom.twist.twist.linear.x = vx;
   odom.twist.twist.linear.y = 0;
   odom.twist.twist.angular.z = vth;
   
   //publish the message
   // last_time = current_time;
   lLast_time = lCurrent_time;
   serial_odom.publish(&odom);
   //    pub_range.publish(&range_msg);
   
   
   //##############################################################################################################################
   //##############################################################################################################################
   //##############################################################################################################################
   pubTime = millis() + 50;
   }
   
   */

  if  (abs(x1) < 0.01){
    x1 = 0;
  }
  if (abs(z1) < 0.01){
    z1 = 0;
  }


  if ((x1 != 0) || (z1 != 0)) {

    //    HWSERIAL.print("x1 at start of loop: ");
    //    HWSERIAL.println(x1);
    /*
    if (PIDen){
     map(z1, -1, 100, analogRangeMin, analogRangeMax);
     */
    VL = x1 - ((z1*wheel_spacing)/2);
    VR = x1 + ((z1*wheel_spacing)/2);


    lInput = v_left;
    rInput = v_right;

    lSetpoint = VL;
    rSetpoint = VR;


    if (v_left > 0.15){
      lSetpoint = speedLimit;
    }

    if (v_right > 0.15){
      rSetpoint = speedLimit;
    }


    //   delay(10);


    float32_msg.data = lInput;
    lInputP.publish( &float32_msg);

    float32_msg.data = lSetpoint;
    lSetpointP.publish( &float32_msg);


    float32_msg.data = rInput;
    rInputP.publish( &float32_msg);

    float32_msg.data = rSetpoint;
    rSetpointP.publish( &float32_msg);

    if ((v_left > 0.08) || (v_right > 0.08)){
      leftPID.Compute(true);
      rightPID.Compute(true);
    } 
    else {
      leftPID.Compute(false);
      rightPID.Compute(false);
    }    




    float32_msg.data = rOutput;
    rOutputP.publish( &float32_msg);

    float32_msg.data = lOutput;
    lOutputP.publish( &float32_msg);


    xout = (rOutput + lOutput)/2;
    zout = (rOutput - lOutput)/2;

    //   HWSERIAL.print("x1 before PIDEN");
    //   HWSERIAL.println(x1);

    if (PIDen){
      //xz_updateStick(xout, zout);
    } 
    else {

      if ((abs(v_left) > limitThreshold) || (abs(v_right) < limitThreshold)){
        /*  if ((x1 == 0) && (z1 != 0)){
         z1 = z1 20;
         }*/
        if (x1 > x1Lim) x1 = x1Lim;

        if (x1 < x1LimNeg) x1 = x1LimNeg;

        if (z1 > z1Lim) z1 = z1Lim;

        if (z1 < z1LimNeg) z1 = z1LimNeg;

        /*
        if (x1 > 0.55){
         x1 = 0.55;
         } 
         else if (x1 < -0.50){
         x1 = -0.50;
         }
         if (z1 > 0.85){
         z1 = 0.85;
         } 
         else if (z1 < -0.85){
         z1 = -0.85;
         }
         */
      }      
      if (x1 < 0){
        x1 = -0.45;
      }

      xz_updateStick(x1, z1);
    }





    //Publish values going into and out of PID equation

    float32_msg.data = xout;
    x_out_sub.publish( &float32_msg);

    float32_msg.data = zout;
    z_out_sub.publish( &float32_msg);



    delay(10);
    timeout = millis();
  } 

  else {
    stopMotors();
  }

  millCheck2 = millis();

  int_msg.data = millCheck2 - millCheck;
  //  iDebug.publish( &int_msg);

  if ( millis() >= range_time ){
    // while(1){
    /*  range_msg.range = getRangeSRF(A4);
     range_msg.header.stamp = nh.now();
     pub_range.publish(&range_msg); 
     delay(10);
     
     range_msg.range = getRangeSRF(A5);
     range_msg.header.stamp = nh.now();
     pub_range2.publish(&range_msg);  
     delay(10);    
     range_time =  millis() + 200;
     nh.spinOnce();
     
     range_msg.range = sonar.ping_cm() / 100.0;
     range_msg.header.stamp = nh.now();
     pub_range.publish(&range_msg);
     */
    //  range_msg.range = getRangeSRF(A4);
    //range_msg.header.stamp = nh.now();
    //pub_range.publish(&range_msg); 

    range_msg.range = sonar2.ping_cm() / 100.0;
    range_msg.header.stamp = nh.now();
    pub_range2.publish(&range_msg);


    range_msg.range = sonar.ping_cm() / 100.0;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 250;

    //  nh.spinOnce();
    //      delay(1);

  }
  //    int tempPing = sonar.ping_cm();
  // float tempPing2 = sonar.ping_cm() / 100.0;
  /* if ((millis() >= range_time2 ) && (millis() > range_time - 150)){
   range_msg.range = getRangeSRF(A5);
   //        range_msg.range = getRangeSRF(A4);
   range_msg.header.stamp = nh.now();
   pub_range.publish(&range_msg);    
   range_time2 =  millis() + 200;
   }*/
  /*   range_msg.range = sonar.ping_cm() / 100.0;
   range_msg.header.stamp = nh.now();
   pub_range.publish(&range_msg);
   range_time2 = millis() + 100;
   */

  millCheck   = millis();

  /*
   if ( millis() >= range_time3 ) && (debugFlag)){
   lFlag = -1;
   leftTriggered = true;
   range_time3 =  millis() + 75;
   }
   
   
   if (( millis() >= range_time2 ) && (debugFlag)){
   rFlag = 1;
   debugFlag = false;
   rightTriggered = true;
   range_time2 =  millis() + 75;
   }
   */

  nh.spinOnce();
}  // end of loop

/*
#if defined(doubleResolution)
 void leftTrigger() {
 cli();
 
 if (digitalRead(leftInterruptPin) == 1){
 if (digitalRead(leftStatusPin) == 1){
 lFlag = 1;
 } 
 else if (digitalRead(leftStatusPin) == 0){
 lFlag = -1;
 }
 } 
 
 else if (digitalRead(leftInterruptPin) == 0){
 if (digitalRead(leftStatusPin) == 1){
 
 lFlag = -1;
 } 
 else if (digitalRead(leftStatusPin) == 0){
 lFlag = 1;
 } 
 }
 leftTriggered = true;
 sei();
 }
 
 
 
 
 void rightTrigger() {
 cli();
 
 if (digitalRead(rightInterruptPin) == 1){
 if (digitalRead(rightStatusPin) == 1){
 rFlag = 1;
 } 
 else if (digitalRead(rightStatusPin) == 0){
 rFlag = -1;
 }
 } 
 
 else if (digitalRead(rightInterruptPin) == 0){
 if (digitalRead(rightStatusPin) == 1){
 
 rFlag = -1;
 } 
 else if (digitalRead(rightStatusPin) == 0){
 rFlag = 1;
 } 
 }
 rightTriggered = true;
 sei();
 }
 
 #else 
 */
void leftTrigger() {
  cli();
  if (digitalRead(leftStatusPin) == 1){
    lFlag = 1;
  } 
  else if (digitalRead(leftStatusPin) == 0){
    lFlag = -1;
  }
  leftTriggered = true;
  sei();
}

void rightTrigger() {
  cli();
  if (digitalRead(rightStatusPin) == 1){
    rFlag = 1;
  } 
  else if (digitalRead(rightStatusPin) == 0){
    rFlag = -1;
  }

  rightTriggered = true;
  sei();
}
















