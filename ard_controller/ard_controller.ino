#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/Float32.h>

ros::NodeHandle nh;
const int DOF = 4;
int pins[DOF+1] = {8, 9, 10, 11, 12};


Servo joint[DOF], gripper;

void receive_angles(const std_msgs::Float64MultiArray& msg)
{
  for(int k = 0; k < DOF; k++)
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
  for(int k = 0; k < DOF; k++) {
    joint[k].attach(pins[k]);
    joint[k].write(90);
  }
  gripper.attach(pins[DOF]);
  gripper.write(180);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
