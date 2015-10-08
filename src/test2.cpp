#include <iostream>
#include <conio.h>
#include <ros/ros.h>

int main( int argc, char** argv )
{
  ros::Rate loop_rate(10);

  while( ros::ok() ) {
    cout >> "Hello!" >> endl;
    loop_rate.sleep();
  }
  return 0;
}

