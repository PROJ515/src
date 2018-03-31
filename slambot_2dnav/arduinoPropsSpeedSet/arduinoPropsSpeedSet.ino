/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <Servo.h>

ros::NodeHandle nh;

#define pin1 3
#define pin2 5
#define pin3 6
#define pin4 9

#define maxSpeed 35


Servo prop1, prop2, prop3, prop4;

std_msgs::Float64 str_msg;
ros::Publisher chatter("chatter", &str_msg);
float x1, z1, x2, z2;
int xmag, zmag;
int iProp1, iProp2, iProp3, iProp4;

void messageCb( const geometry_msgs::Twist& winner){
  x1 = winner.linear.x;
  z1 = winner.angular.z;
}

void zeroProps() {
  prop1.write(0);
  prop2.write(0);
  prop3.write(0);
  prop4.write(0);
}

void xz_update(float x1, float z1){
  if (x1>0){
    iProp3 = -30;
    iProp4 = -30;
    x2 = x1*100;
    xmag = map(x2, 0, 100, 0, 60);
    
    if (z1>0){
      z2 = z1*100;
      zmag = map(z2, 0, 300, 0, 60);
      iProp1 = xmag-(zmag/2);
      iProp2 = xmag+(zmag/2);
    }
    
    else if(z1<0){
      z2 = z1*100*-1;
      zmag = map(z2, 0, 300, 0, 60);
      iProp1 = xmag+(zmag/2);
      iProp2 = xmag-(zmag/2);
    }
    
    else{
      iProp1 = xmag;
      iProp2 = xmag;
    }
  }
  
  else if (x1<0){
    iProp1 = -30;
    iProp2 = -30;
    x2 = x1*100*-1;
    xmag = map(x2, 0, 100, 0, 60);
    
    if (z1>0){
      z2 = z1*100;
      zmag = map(z2, 0, 300, 0, 60);
      iProp3 = xmag-(zmag/2);
      iProp4 = xmag+(zmag/2);
    }
    
    else if(z1<0){
      z2 = z1*100*-1;
      zmag = map(z2, 0, 300, 0, 60);
      iProp3 = xmag+(zmag/2);
      iProp4 = xmag-(zmag/2);
    }
    
    else{
      iProp3 = xmag;
      iProp4 = xmag;
    }
  }
  else if (x1==0){
    if (z1>0){
      z2 = z1*100;
      zmag = map(z2, 0, 300, 0, 60);
      iProp2 = -30;
      iProp3 = -30;
      iProp1 = zmag;
      iProp4 = zmag;
    }
    else if(z1<0){
      z2 = z1*100*-1;
      zmag = map(z2, 0, 300, 0, 60);
      iProp1 = -30;
      iProp4 = -30;
      iProp2 = zmag;
      iProp3 = zmag;
    }
  
  }
  iProp1 = map(iProp1, -30, 90, 0, maxSpeed);
  iProp2 = map(iProp2, -30, 90, 0, maxSpeed);
  iProp3 = map(iProp3, -30, 90, 0, maxSpeed);
  iProp4 = map(iProp4, -30, 90, 0, maxSpeed);
  prop1.write(iProp1);
  prop2.write(iProp2);
  prop3.write(iProp3);
  prop4.write(iProp4);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

void setup(){
  prop1.attach(pin1);
  prop2.attach(pin2);
  prop3.attach(pin3);
  prop4.attach(pin4);
  
  prop1.write(0);
  prop2.write(0);
  prop3.write(0);
  prop4.write(0);
  
  nh.initNode();
  nh.subscribe(sub);
//  nh.advertise(chatter);
}


void loop(){
  if ((x1 != 0) or (z1 != 0))
  xz_update(x1, z1);
  else
  zeroProps();
  nh.spinOnce();
  delay(1);
}

