#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/Float32.h>

ros::NodeHandle nh;


Servo joint[5], gripper;

void receive_angles(const std_msgs::Float64MultiArray& msg)
{
  for(int k = 0; k < 5; k++)
    joint[k].write( msg.data[k] );
}

//void receive_angle(const std_msgs::Float32& msg)
//{
//  joint[0].write( msg.data );
//}

ros::Subscriber<std_msgs::Float64MultiArray> sub("angles", receive_angles );
//ros::Subscriber<std_msgs::Float32> sub("angle", receive_angle );

void setup() {
  // put your setup code here, to run once:
  joint[0].attach( 31 );
  joint[1].attach( 33 );
  joint[2].attach( 35 );
  joint[3].attach( 37 );
  joint[4].attach( 39 );
  gripper.attach( 41 );

  joint[0].write( 90 );
  joint[1].write( 90 );
  joint[2].write( 90 );
  joint[3].write( 90 );
  joint[4].write( 90 );
  gripper.write( 90 );

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
