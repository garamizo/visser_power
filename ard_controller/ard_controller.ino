#include <Servo.h>
//#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#define VISION_ROBOT

#ifdef RUNNER_ROBOT
const int DOF = 4;
int pins[DOF+1] = {8, 9, 10, 11, 12};
float offset[DOF] = {87, 95, 110, 87};
float dir[DOF] = {-1, -1, -1, 1};
#endif

#ifdef VISION_ROBOT
const int DOF = 5;
int pins[DOF+1] = {31, 33, 35, 37, 39, 41};
float offset[DOF] = {100, 90, 90, 70, 90};
float dir[DOF] = {-1, 1, -1, 1, -1};
#endif

ros::NodeHandle nh;

Servo joint[DOF], gripper;

void receive_angles(const sensor_msgs::JointState& msg)
{
  char success[] = {"success"};
  nh.loginfo(success);
  for(int k = 0; k < DOF; k++)
    joint[k].write(msg.position[k]*dir[k]*180.0/PI + offset[k]);
  //gripper.write(msg.data[DOF]);
  
}

void receive_angles2(const std_msgs::Float64MultiArray& msg)
{
  for(int k = 0; k < DOF; k++)
    joint[k].write(msg.data[k]*dir[k]*180.0/PI + offset[k]);
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", receive_angles);
ros::Subscriber<std_msgs::Float64MultiArray> sub2("joint_states2", receive_angles2);

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
  nh.subscribe(sub2);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
  nh.spinOnce();
}
