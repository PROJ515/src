//dalekMotors.cpp
 
#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Int16.h>
//#include <PID_v1.h>



#define forwardPin 23
#define reversePin 21
#define rightPin 20
#define leftPin 22




//for full range of the joystick and to have the map centering at the correct point, use a min of 60 and a max of 200
/*
 *  Values at start of MEng project, should work but dont. Centre seems good.
 * #define analogRangeMin 68  
 * #define analogRangeMax 190
 */
//#define analogRangeMin 69  
//#define analogRangeMax 191
#define analogRangeMin 68  
#define analogRangeMax 190

//#define analogRangeMin 100
//#define analogRangeMax 190
//#define centre 136
// #define centre 134
#define centre 129
//#define debug_motors



#if defined(debug_motors)
    std_msgs::Int16 str_msg;
    ros::Publisher chatter_motors("chatter_motors", &str_msg); 
#endif 






void stopMotors(void){
  analogWrite(forwardPin, centre);
  analogWrite(reversePin, centre);
  analogWrite(leftPin, centre);
  analogWrite(rightPin, centre);
}



int xmag, zmag;
int iStickF, iStickB, iStickL, iStickR;

void xz_updateStick(float x1, float z1){

  xmag = x1*100    ;//*100;
  zmag = z1*100;
  
if (xmag > 100){
  xmag = 100;
} else if (xmag < -100){
  xmag = -100;
}

if (zmag > 100){
  zmag = 100;
} else if (zmag < -100){
  zmag = 100;
}

  if ((xmag == 0) && (zmag == 0)){
    stopMotors();
  } 
  else {

    iStickF = map(xmag, -100, 100, analogRangeMin, analogRangeMax);
    iStickB = map(xmag, -100, 100, analogRangeMax, analogRangeMin);
    iStickR = map(zmag, -100, 100, analogRangeMin, analogRangeMax);  
    iStickL = map(zmag, -100, 100, analogRangeMax, analogRangeMin);


    analogWrite(forwardPin, iStickF);
    analogWrite(reversePin, iStickB);
    analogWrite(leftPin, iStickL);
    analogWrite(rightPin, iStickR);
    
    
#if defined(debug_motors)
    str_msg.data = 1;
    chatter_motors.publish( &str_msg );


    str_msg.data = iStickF;
    chatter_motors.publish( &str_msg );


    str_msg.data = 2;
    chatter_motors.publish( &str_msg );


    str_msg.data = iStickB;
    chatter_motors.publish( &str_msg );

    str_msg.data = 3;
    chatter_motors.publish( &str_msg );


    str_msg.data = iStickL;
    chatter_motors.publish( &str_msg );


    str_msg.data = 4;
    chatter_motors.publish( &str_msg );


    str_msg.data = iStickR;
    chatter_motors.publish( &str_msg );
    
    #endif
    

  }
}
