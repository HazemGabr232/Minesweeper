#define BLUETOOTH_CTRL  0
#define JOYSTICK_CTRL   1

#include <HBridge.h>
HBridge hbridge(3, 5, 6, 9);//, 10, 11);

/* Control Mehtods */
char x ;

#if JOYSTICK_CTRL
#include <JoyControl.h>
JoyControl joystick(hbridge);
#endif
#include <ros.h>
#include <std_msgs/String.h>
ros::NodeHandle  nh;

void ROS_CALLBACK(const std_msgs::String &msg){
  x = msg.data[0] ;
  joystick.handle(x);  
}

ros::Subscriber<std_msgs::String> s("Joystick_",  &ROS_CALLBACK);

void setup() {
pinMode (4,OUTPUT);

  nh.initNode();
  nh.subscribe(s);
 
}


void loop() {
nh.spinOnce();
delay(1);
  
}


