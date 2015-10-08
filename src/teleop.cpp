#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <stdio.h>

geometry_msgs::Pose ef;
ros::Publisher pubT;
geometry_msgs::Transform msg;

void transform( const tf2_msgs::TFMessage tfm )
{
//  ROS_INFO("Pa!");
  ROS_INFO( "Size: %s", tfm.transforms[0].child_frame_id.c_str() );
  static tf::TransformBroadcaster br;

  static tf::Transform TEC( tf::Quaternion(-0.5, 0.5, -0.5, 0.5), tf::Vector3(-10e-2, 0, 15e-2) );
  static tf::Transform TEM( tf::Quaternion(0, 0, 0, 1), tf::Vector3(15e-2, 0, 0) );
  static tf::Transform TTF( tf::Quaternion(-0.5, 0.5, 0.5, 0.5), tf::Vector3(20e-2, 5e-2, 0) );
  static tf::StampedTransform TCT, TWE;
  tf::Transform TWEpp, TWF;

  if( strcmp( "EndEffector", tfm.transforms[0].child_frame_id.c_str() ) == 0 )
  {

    tf::transformStampedMsgToTF( tfm.transforms[0], TWE );
//    ROS_INFO("Inside!");

  }
  if( strcmp( "ar_marker_6", tfm.transforms[0].child_frame_id.c_str() ) == 0 )
  {
    
    tf::transformStampedMsgToTF( tfm.transforms[0], TCT );
 TWF = TWE * TEC * TCT * TTF;
  TWEpp = TWF * TEM.inverse();

  
  br.sendTransform( tf::StampedTransform(TWEpp, ros::Time::now(), "World", "EndEffectorPP") );
  br.sendTransform( tf::StampedTransform(TEC, ros::Time::now(), "EndEffector", "Camera") );
  br.sendTransform( tf::StampedTransform(TEM, ros::Time::now(), "EndEffector", "MaleConn") );
  br.sendTransform( tf::StampedTransform(TCT, ros::Time::now(), "Camera", "Tag") );
  br.sendTransform( tf::StampedTransform(TTF, ros::Time::now(), "Tag", "FemaleConn") );
  br.sendTransform( tf::StampedTransform(TWF, ros::Time::now(), "World", "FemaleConn2") );

  tf::transformTFToMsg( TWEpp, msg );
  }

 }

int main( int argc, char* argv[] )
{
  // create the node teleop
  ros::init( argc, argv, "teleop" );

  ros::NodeHandle n;
  pubT = n.advertise<geometry_msgs::Transform>("TWEt", 10);

  // subscrive to topic com, with maximum queue of 100
  ros::Subscriber tf_CT = n.subscribe( "tf", 10, transform );

  // what is this for?
  ros::Rate loop_rate(0.5);

  while( ros::ok() )
  {
    //chatter_pub.publish( msg );

    tf::Transform T( tf::Quaternion(0, 0, 0, 1), tf::Vector3(200e-3, 0, 200e-3) );

  tf::transformTFToMsg( T, msg );
    pubT.publish( msg );
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

