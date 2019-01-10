
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
int marker_id=0;
float sonic_distance=300 ; //calibrate ultrasonic then change this 
long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
//ros::Time 
//long current_time, last_time;
ros::Time current_time;
ros::Time last_time;

double DistancePerCount = (3.14159265 * 0.062)/500;

//unsigned long current_time , last_time ;


double x=-8.5; 
double y=-9.5;
double th;
double v_left;//left motor speed
double v_right;//right motor speed
double vth;//angular velocity of robot
double deltaLeft;//no of ticks in left encoder since last update
double deltaRight;//no of ticks in right encoder since last update
double dt;
double delta_distance;//distance moved by robot since last update
double delta_th;//corresponging change in heading
double delta_x ;//corresponding change in x direction
double delta_y;//corresponding change in y direction
#define PI 3.14159265
#define TwoPI 6.28318531
/******************************************/

int encoderPin1 = 3;
int encoderPin2 = 2;
int encoderPin3 = 19;
int encoderPin4 = 18;


volatile int lastEncoded1 = 0;
volatile long encoderValue1 = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;

//long lastencoderValue1 = 0;

//long lastencoderValue2 = 0;


int lastMSB1 = 0;
int lastLSB1 = 0;
int lastMSB2 = 0;
int lastLSB2 = 0;







visualization_msgs::Marker marker;
ros::Publisher mark("mark", &marker);

#include <MetalDetector.h>	
MetalDetector lmetal(9), rmetal(10);

#include <Ultrasonic.h>
Ultrasonic    lsonic(/*echo_pin*/14,/*trig_pin*/15), rsonic(/*echo_pin*/16,/*trig_pin*/17);



ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;



char base_link[] = "/base_link";
char odom[] = "/odom";

void setup()
{

pinMode(encoderPin1, INPUT);
pinMode(encoderPin2, INPUT);
pinMode(encoderPin3, INPUT);
pinMode(encoderPin4, INPUT);

digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
digitalWrite(encoderPin3, HIGH); //turn pullup resistor on
digitalWrite(encoderPin4, HIGH); //turn pullup resistor on



//call updateEncoder() when any high/low changed seen
//on interrupt 0 (pin 2), or interrupt 1 (pin 3)
attachInterrupt(0, updateEncoder, CHANGE);
attachInterrupt(1, updateEncoder, CHANGE);
attachInterrupt(4, updateEncoder, CHANGE);
attachInterrupt(5, updateEncoder, CHANGE);


  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(mark);
  
}

void loop()
{  

  //nh.spinOnce();

 current_time = nh.now();

 deltaLeft = encoderValue1 -_PreviousLeftEncoderCounts;
 deltaRight = encoderValue2 - _PreviousRightEncoderCounts;
 
  dt = (current_time.toSec() - last_time.toSec());
 v_left = deltaLeft * DistancePerCount/dt;
 v_right = deltaRight * DistancePerCount/dt;
 delta_distance=0.5f*(double)(deltaLeft+deltaRight)*DistancePerCount;
 delta_th = (double)(deltaRight-deltaLeft)*DistancePerCount/0.36f; //Distance between the two wheels is 0.36m
 delta_x = delta_distance*(double)cos(th);
 delta_y = delta_distance*(double)sin(th);
 x += delta_x;
 y += delta_y;
 th += delta_th;
 if (th > PI)
 th -= TwoPI;
 else
 if ( th <= -PI)
 th += TwoPI;
 _PreviousLeftEncoderCounts = encoderValue1;
 _PreviousRightEncoderCounts = encoderValue2;

   
if (lmetal.detect()==0){
   if (lsonic.getDistance() < sonic_distance){
visualization_msgs::Marker marker;
 marker.header.frame_id = "left_metal";
 marker.header.stamp = ros::Time();
 marker.ns = "my_namespace";
 marker.id = marker_id;
 marker.type = visualization_msgs::Marker::CUBE; //if the mine above the ground will be shown as a blue cube on the suface
 marker.action = visualization_msgs::Marker::ADD;
 marker.pose.position.x = 0;
 marker.pose.position.y = 0;
 marker.pose.position.z = 0;
 marker.pose.orientation.x = 0.0;
 marker.pose.orientation.y = 0.0;
 marker.pose.orientation.z = 0.0;
 marker.pose.orientation.w = 1.0;
 marker.scale.x = 0.1;
 marker.scale.y = 0.1;
 marker.scale.z = 0.1;
 marker.color.a = 1.0; // Don't forget to set the alpha!
 marker.color.r = 0.0;
 marker.color.g = 0.0;
 marker.color.b = 1.0;
//only if using a MESH_RESOURCE marker type:
// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
 mark.publish( &marker );
marker_id++;
nh.spinOnce();
}   
   else{
 visualization_msgs::Marker marker;
 marker.header.frame_id = "left_metal";
 marker.header.stamp = ros::Time();
 marker.ns = "my_namespace";
 marker.id = marker_id;
 marker.type = visualization_msgs::Marker::CUBE; //if the mine under the ground will be shown as a green cube under the ground
 marker.action = visualization_msgs::Marker::ADD;
 marker.pose.position.x = 0;
 marker.pose.position.y = 0;
 marker.pose.position.z = -0.1;
 marker.pose.orientation.x = 0.0;
 marker.pose.orientation.y = 0.0;
 marker.pose.orientation.z = 0.0;
 marker.pose.orientation.w = 1.0;
 marker.scale.x = 0.1;
 marker.scale.y = 0.1;
 marker.scale.z = 0.1;
 marker.color.a = 1.0; // Don't forget to set the alpha!
 marker.color.r = 0.0;
 marker.color.g = 1.0;
 marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
 mark.publish( &marker );
marker_id++;
nh.spinOnce();
}
}

 if (rmetal.detect()==0){   //put another if for ultrasonic with another different marker 
    if (rsonic.getDistance() < sonic_distance){
visualization_msgs::Marker marker;
 marker.header.frame_id = "right_metal";
 marker.header.stamp = ros::Time();
 marker.ns = "my_namespace";
 marker.id = marker_id;
 marker.type = visualization_msgs::Marker::CUBE;  //if the mine above the ground will be shown as a blue cube on the surface
 marker.action = visualization_msgs::Marker::ADD;
 marker.pose.position.x = 0;
 marker.pose.position.y = 0;
 marker.pose.position.z = 0;
 marker.pose.orientation.x = 0.0;
 marker.pose.orientation.y = 0.0;
 marker.pose.orientation.z = 0.0;
 marker.pose.orientation.w = 1.0;
 marker.scale.x = 0.1;
 marker.scale.y = 0.1;
 marker.scale.z = 0.1;
 marker.color.a = 1.0; // Don't forget to set the alpha!
 marker.color.r = 0.0;
 marker.color.g = 0.0;
 marker.color.b = 1.0;
//only if using a MESH_RESOURCE marker type:
// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
 mark.publish( &marker );
marker_id++;
nh.spinOnce();
}
else{
 visualization_msgs::Marker marker;
 marker.header.frame_id = "right_metal";
 marker.header.stamp = ros::Time();
 marker.ns = "my_namespace";
 marker.id = marker_id;
 marker.type = visualization_msgs::Marker::CUBE; //if the mine under the ground will be shown as a green cube under the ground
 marker.action = visualization_msgs::Marker::ADD;
 marker.pose.position.x = 0;
 marker.pose.position.y = 0;
 marker.pose.position.z = -0.1;
 marker.pose.orientation.x = 0.0;
 marker.pose.orientation.y = 0.0;
 marker.pose.orientation.z = 0.0;
 marker.pose.orientation.w = 1.0;
 marker.scale.x = 0.1;
 marker.scale.y = 0.1;
 marker.scale.z = 0.1;
 marker.color.a = 1.0; // Don't forget to set the alpha!
 marker.color.r = 0.0;
 marker.color.g = 1.0;
 marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
 mark.publish( &marker );
marker_id++;
nh.spinOnce();
}
} 
    
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = "thunder";
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(th);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  
  nh.spinOnce();
  
  delay(10);
}




void updateEncoder(){
int MSB1 = digitalRead(encoderPin1); //MSB = most significant bit
int LSB1 = digitalRead(encoderPin2); //LSB = least significant bit
int MSB2 = digitalRead(encoderPin3); //MSB = most significant bit
int LSB2 = digitalRead(encoderPin4); //LSB = least significant bit

int encoded1 = (MSB1 << 1) |LSB1; //converting the 2 pin value to single number
int encoded2 = (MSB2 << 1) |LSB2;
int sum1 = (lastEncoded1 << 2) | encoded1; //adding it to the previous encoded value
int sum2 = (lastEncoded2 << 2) | encoded2;
if(sum1 == 0b1101 || sum1 == 0b0100 || sum1 == 0b0010 || sum1 == 0b1011) encoderValue1 ++; 
if(sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) encoderValue2 ++; 

if(sum1 == 0b1110 || sum1 == 0b0111 || sum1 == 0b0001 || sum1 == 0b1000) encoderValue1--; 
if(sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) encoderValue2 --; 
lastEncoded2 = encoded2; //store this value for next time 
lastEncoded1 = encoded1; //store this value for next time 
} 
