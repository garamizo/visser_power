#include <Servo.h>
#include "numericalTools.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>
#include <geometry_msgs/Transform.h>

#define DOF 5
#define DT_MS 50
#define MAX_SPEED 20e-2





float Q[DOF];
float S[] = { 300e-3, 0.0, 250e-3, 0.0, 0.0 };
Servo joint[DOF];
Servo gripper;

ros::NodeHandle nh;
tf::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped t;

geometry_msgs::Transform array2Transform( float* S, float q0 );

geometry_msgs::Transform array2Transform( float* S, float q0 ) {
  geometry_msgs::Transform T;

  T.translation.x = S[0];
  T.translation.y = S[1];
  T.translation.z = S[2];
  float r3 = q0, r2 = S[3], r1 = S[4];
  T.rotation.w = cos(r1/2)*cos(r2/2)*cos(r1/2) + sin(r1/2)*sin(r2/2)*sin(r3/2);
  T.rotation.x = sin(r1/2)*cos(r2/2)*cos(r3/2) - cos(r1/2)*sin(r2/2)*sin(r3/2);
  T.rotation.y = cos(r1/2)*sin(r2/2)*cos(r3/2) + sin(r1/2)*cos(r2/2)*sin(r3/2);
  T.rotation.z = cos(r1/2)*cos(r2/2)*sin(r3/2) - sin(r1/2)*sin(r2/2)*cos(r3/2);
  
  return T;
}

void Transform2array( float* S, geometry_msgs::Transform T ) {
  S[0] = T.translation.x;
  S[1] = T.translation.y;
  S[2] = T.translation.z;
  float q0 = T.rotation.w, q1 = T.rotation.x, q2 = T.rotation.y, q3 = T.rotation.z;
  S[3] = asin(2*(q0*q2-q3*q1));
  S[4] = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
}

void update_pose( const geometry_msgs::Transform& T){
  float S[5], S1[5];
  
  char msg[80];
  
  
  sprintf( msg, "Transform: %d %d %d %d %d %d %d", round(T.translation.x*1000), T.translation.y*1000, round(T.translation.z*1000), T.rotation.x*1000, T.rotation.y*1000, T.rotation.z*1000, round(T.rotation.w*1000) );
  nh.loginfo(msg);

  Transform2array( S1, T );
  sprintf( msg, "Array: %d %d %d %d %d", round(S1[0]*1000), round(S1[1]*1000), round(S1[2]*1000), round(S1[3]*180/PI), round(S1[4]*180/PI) );
  nh.loginfo(msg);
  
  DKFunc( Q, S );

  moveTo( Q, S, DOF );
}

ros::Subscriber<geometry_msgs::Transform> sub("TWEt", update_pose );


/*
void quat2euler321( float* e, geometry_msgs::Pose pose )
{
  float qw = pose.orientation.w, qx = pose.orientation.x, qy = pose.orientation.y, qz = pose.orientation.z;
  e[2] = atan2( 2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy) );
  e[1] = asin( 2*(qw*qy-qz*qx) );
  e[0] = atan2( 2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz) );
  
  vec3.x = e[0];
  vec3.y = e[1];
  vec3.z = e[2];
  pub.publish( &vec3 );
}

void update_pose( const geometry_msgs::Pose& pose )
{
  float e[3];
  quat2euler321( e, pose );
  float S[] = { pose.position.x, pose.position.y, pose.position.z, 0.0, 0.0 };
  moveTo( Q, S, DOF );
  nh.loginfo("here");
}
*/
void increment( float* Q, float* dQ ) {
  
  float dir[] = { -1, 1, -1, 1, -1 };
  float offset[] = { 100*PI/180, 90*PI/180, 90*PI/180, 70*PI/180, 90*PI/180 };
        
  const float TOL = 0.2;
  
  bool fail = false;
  for( int n = 0; n < 5; n++ ) {
    
    if( isnan( dQ[n] ) )
      fail = true;
  }
    
  if( fail ) {
    Serial.println( "NaN!" );
    for( int n = 0; n < 5; n++ )
      dQ[n] = random( -100, 100 ) / 100.0 * 5 * PI / 180;
  }
    
  for( int n = 0; n < 5; n++ ) {
    Q[n] = asin( sin( Q[n] + dQ[n] ) );
    joint[n].write( ( dir[n] * Q[n] + offset[n] ) * 180 / PI );
  }
}
  

void DKFunc( float* Q, float* S ) {
  
  float q1 = Q[0], 
        q2 = Q[1],
        q3 = Q[2],
        q4 = Q[3],
        q5 = Q[4];

  float (*F)[1] = (float(*)[1]) S;
  
  F[0][0] = cos(q1)*cos(3.141592653589793*(-1.0/2.0)+q2)*(2.9E1/2.0E2)+cos\
(3.141592653589793*(1.0/2.0)+q5)*sin(q1)*sin(q4)*(1.0/1.0E1)-cos(q1)*cos(q\
3)*sin(3.141592653589793*(-1.0/2.0)+q2)*(1.97E2/1.0E3)-cos(q1)*cos(3.14159\
2653589793*(-1.0/2.0)+q2)*sin(q3)*(1.97E2/1.0E3)-cos(q1)*cos(q3)*sin(3.141\
592653589793*(-1.0/2.0)+q2)*sin(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1\
)-cos(q1)*cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3)*sin(3.1415926535897\
93*(1.0/2.0)+q5)*(1.0/1.0E1)+cos(q1)*cos(q3)*cos(q4)*cos(3.141592653589793\
*(-1.0/2.0)+q2)*cos(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1)-cos(q1)*co\
s(q4)*cos(3.141592653589793*(1.0/2.0)+q5)*sin(q3)*sin(3.141592653589793*(-\
1.0/2.0)+q2)*(1.0/1.0E1);
  F[1][0] = cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q1)*(2.9E1/2.0E2)-cos\
(q3)*sin(q1)*sin(3.141592653589793*(-1.0/2.0)+q2)*(1.97E2/1.0E3)-cos(3.141\
592653589793*(-1.0/2.0)+q2)*sin(q1)*sin(q3)*(1.97E2/1.0E3)-cos(q1)*cos(3.1\
41592653589793*(1.0/2.0)+q5)*sin(q4)*(1.0/1.0E1)-cos(q3)*sin(q1)*sin(3.141\
592653589793*(-1.0/2.0)+q2)*sin(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1\
)-cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q1)*sin(q3)*sin(3.1415926535897\
93*(1.0/2.0)+q5)*(1.0/1.0E1)+cos(q3)*cos(q4)*cos(3.141592653589793*(-1.0/2\
.0)+q2)*cos(3.141592653589793*(1.0/2.0)+q5)*sin(q1)*(1.0/1.0E1)-cos(q4)*co\
s(3.141592653589793*(1.0/2.0)+q5)*sin(q1)*sin(q3)*sin(3.141592653589793*(-\
1.0/2.0)+q2)*(1.0/1.0E1);
  F[2][0] = sin(3.141592653589793*(-1.0/2.0)+q2)*(-2.9E1/2.0E2)-cos(q3)*co\
s(3.141592653589793*(-1.0/2.0)+q2)*(1.97E2/1.0E3)+sin(q3)*sin(3.1415926535\
89793*(-1.0/2.0)+q2)*(1.97E2/1.0E3)-cos(q3)*cos(3.141592653589793*(-1.0/2.\
0)+q2)*sin(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1)+sin(q3)*sin(3.14159\
2653589793*(-1.0/2.0)+q2)*sin(3.141592653589793*(1.0/2.0)+q5)*(1.0/1.0E1)-\
cos(q3)*cos(q4)*cos(3.141592653589793*(1.0/2.0)+q5)*sin(3.141592653589793*\
(-1.0/2.0)+q2)*(1.0/1.0E1)-cos(q4)*cos(3.141592653589793*(-1.0/2.0)+q2)*co\
s(3.141592653589793*(1.0/2.0)+q5)*sin(q3)*(1.0/1.0E1)+6.877E-2;
  F[3][0] = asin(cos(3.141592653589793*(1.0/2.0)+q5)*(sin(q4)*(cos(q3)*cos\
(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17-sin(q3)*sin(3.1415\
92653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+6.123233995736766E-17)+co\
s(q4)*(cos(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)+cos(3.141592653589793*\
(-1.0/2.0)+q2)*sin(q3)))-sin(3.141592653589793*(1.0/2.0)+q5)*(-cos(q3)*cos\
(3.141592653589793*(-1.0/2.0)+q2)+sin(q3)*sin(3.141592653589793*(-1.0/2.0)\
+q2)-cos(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.1232339957367\
66E-17-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+\
6.123233995736766E-17)*6.123233995736766E-17+sin(q4)*(cos(q3)*sin(3.141592\
653589793*(-1.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3))*6.1\
23233995736766E-17+3.749399456654644E-33));
  F[4][0] = -atan((cos(3.141592653589793*(1.0/2.0)+q5)*(-cos(q3)*cos(3.141\
592653589793*(-1.0/2.0)+q2)+sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)-c\
os(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17\
-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+6.1232\
33995736766E-17)*6.123233995736766E-17+sin(q4)*(cos(q3)*sin(3.141592653589\
793*(-1.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3))*6.1232339\
95736766E-17+3.749399456654644E-33)*6.123233995736766E-17-cos(q3)*cos(3.14\
1592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+sin(3.141592653589793*(\
1.0/2.0)+q5)*(sin(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.1232\
33995736766E-17-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.12323399573\
6766E-17+6.123233995736766E-17)+cos(q4)*(cos(q3)*sin(3.141592653589793*(-1\
.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3)))*6.1232339957367\
66E-17+sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+\
cos(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-1\
7-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+6.123\
233995736766E-17)-sin(q4)*(cos(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)+co\
s(3.141592653589793*(-1.0/2.0)+q2)*sin(q3))+2.295845021658468E-49)/(cos(3.\
141592653589793*(1.0/2.0)+q5)*(-cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q\
2)+sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)-cos(q4)*(cos(q3)*cos(3.141\
592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17-sin(q3)*sin(3.1415926535\
89793*(-1.0/2.0)+q2)*6.123233995736766E-17+6.123233995736766E-17)*6.123233\
995736766E-17+sin(q4)*(cos(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)+cos(3.\
141592653589793*(-1.0/2.0)+q2)*sin(q3))*6.123233995736766E-17+3.7493994566\
54644E-33)+cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*3.749399456654644E\
-33+sin(3.141592653589793*(1.0/2.0)+q5)*(sin(q4)*(cos(q3)*cos(3.1415926535\
89793*(-1.0/2.0)+q2)*6.123233995736766E-17-sin(q3)*sin(3.141592653589793*(\
-1.0/2.0)+q2)*6.123233995736766E-17+6.123233995736766E-17)+cos(q4)*(cos(q3\
)*sin(3.141592653589793*(-1.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2\
)*sin(q3)))-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*3.749399456654644\
E-33-cos(q4)*(cos(q3)*cos(3.141592653589793*(-1.0/2.0)+q2)*6.1232339957367\
66E-17-sin(q3)*sin(3.141592653589793*(-1.0/2.0)+q2)*6.123233995736766E-17+\
6.123233995736766E-17)*6.123233995736766E-17+sin(q4)*(cos(q3)*sin(3.141592\
653589793*(-1.0/2.0)+q2)+cos(3.141592653589793*(-1.0/2.0)+q2)*sin(q3))*6.1\
23233995736766E-17-1.405799628556214E-65));
}

float equivDisp( float* S0, float* S1 ) {
  float dS[] = { S1[0]-S0[0], S1[1]-S0[1], S1[2]-S0[2] };
  return( sqrt( dS[0]*dS[0] + dS[1]*dS[1] + dS[2]*dS[2] ) );
}

float diff( float* C, float* A, float* B, size_t len ) {
  float norm2 = 0;
  for( int n = 0; n < len; n++ ) {
    C[n] = A[n] - B[n];
    norm2 += C[n] * C[n];
  }
  return norm2;
}

void scale( float* C, float* A, float scale, size_t len ) {
  for( int n = 0; n < len; n++ )
    C[n] = A[n] * scale;
}

int moveTo( float* Q, float* St, int dof ) { // S[] = [x, y, z, --orientation--]
  
  // Calculate current position
  float Sn[dof], dS[dof], aux[dof];

  const float MAX_ERROR = pow(1e-2, 2);
  const unsigned long MAX_T = 500;
  long int T0 = millis();
  while( millis() - T0 < MAX_T ) {
    long int t0 = millis();
    
    // calculate dQ
    DKFunc( Q, Sn );
    float norm2 = diff( dS, St, Sn, dof );
    if( norm2 < MAX_ERROR )
      return 0;

//    Serial.println( norm2 * 1000 );
    
    float dQ[dof];
    float J[dof][dof];
    calcJacobian( &DKFunc, Q, dof, (float**) J );
    solveLS( (float**) J, (float**) dS, dof, (float**) aux );

    scale( dQ, aux, 0.1, dof );
    increment( Q, dQ );
  }
  return 1;
  //printMatrix( (float**) S0, dof, 1, "S = " );
}

void returnHome( int times ) {
  float S_HOME[] = { 15e-2, 0, 25e-2, 0 };
  
  moveTo( Q, S_HOME, 4 );
  for( int k = 0; k < times; k++ ) {
    float Sup[] = { S_HOME[0], S_HOME[1], S_HOME[2] + 5e-2, S_HOME[3] };
    moveTo( Q, Sup, 4 );
    float Sdown[] = { S_HOME[0], S_HOME[1], S_HOME[2] - 5e-2, S_HOME[3] };
    moveTo( Q, Sdown, 4 );
  }
}

void setup() {
  
  // put your setup code here, to run once:
  //Serial.begin( 115200 );
  
  joint[0].attach( 31 );
  joint[1].attach( 33 );
  joint[2].attach( 35 );
  joint[3].attach( 37 );
  joint[4].attach( 39 );
  gripper.attach( 41 );
  
  Q[0] = Q[1] = Q[2] = Q[3] = Q[4] = -0 * PI / 180;

  gripper.write( 170 );
  float S[] = { 300e-3, 0.0, 250e-3, 0.0, 0.0 };
  moveTo( Q, S, DOF );

  nh.initNode();
  broadcaster.init( nh );
  nh.subscribe(sub);
}

#define DT 200

int state = 0;
float S1[] = { 300e-3, 50e-3, 250e-3, 0.0, 0.0 };
float S2[] = { 300e-3, -50e-3, 250e-3, 0.0, 0.5 };
unsigned long int t0 = 0;
const unsigned long int dt = 3000;

void loop() {
  t.header.frame_id = "World";
  t.child_frame_id = "EndEffector";
  float S[5];
  DKFunc( Q, S );
  t.transform = array2Transform( S, Q[0] );
  t.header.stamp = nh.now();
  broadcaster.sendTransform( t );

  if( millis() - t0 > dt ) {
    t0 = millis();
    state = !state;
    //moveTo( Q, state ? S1 : S2, DOF );
  }
  
  nh.spinOnce();
  delay(10);
}

