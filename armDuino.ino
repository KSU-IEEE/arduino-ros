#include <pac-man-arm.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <behaviors/polar_coordinate.h>

/*
 * This will control the arm on the pac-man-bot 
 */

// CONSTANTS
/**********************************************************************/
#define ARM_BASE_HEIGHT 5
#define SENSOR_RADIUS 5

 // initializer variables: 
/*********************************************************************/
ros::NodeHandle nh;
std_msgs::Bool done;
std_msgs::Float64 distance;
float heading;


// init arm
/*********************************************************************/
pac_man_arm arm(SENSOR_RADIUS, ARM_BASE_HEIGHT);


// pubs 
/*********************************************************************/
ros::Publisher pub_done ("/arm/done", &done);
ros::Publisher pub_distance("/arm/distance", &distance);


// angle funcs
/*********************************************************************/
float global_to_rel(float theta) {
  float returner = theta;
  if (theta > heading) {
    returner = theta - heading;
  } else if (theta < heading) {
    returner = 360 - heading + theta;
  }

  return returner;
}


// callbacks
/*********************************************************************/
void heading_cb(const std_msgs::Float64 &val) {
  heading = int(val.data) % 360;
}

/*
 * All theta's will come in as an angle in relate to
 * the body of the bot, have to convert from Global
 * theta to relative theta
 */

void grabBlock_cb(const std_msgs::Float64& theta) {  
  float ang = global_to_rel(theta.data);
  arm.grabBlock(ang);

  done.data = true;
  pub_done.publish(&done);
}

void scan_cb(const behaviors::polar_coordinate& point) {
  float ang = global_to_rel(point.theta);
  distance.data = arm.point(ang, point.r);
  done.data =true;

  pub_distance.publish(&distance);
  pub_done.publish(&done);
}

void armReset_cb(const std_msgs::Bool& yes) {
  if (yes.data) {
    arm.setBaseAngle(0);
    arm.setArmAngle(0);
  }

  done.data = true;
  pub_done.publish(&done);
}

// subscribers
/*********************************************************************/
ros::Subscriber <std_msgs::Float64> sub_heading("heading", &heading_cb);
ros::Subscriber <std_msgs::Float64> sub_grabBlock("/arm/grabBlock", &grabBlock_cb);
ros::Subscriber <behaviors::polar_coordinate> sub_scan("/arm/distance", &scan_cb);
ros::Subscriber <std_msgs::Bool> sub_reset("/arm/reset", &armReset_cb);

// normal stuff
/*********************************************************************/
void setup() {
  // create publishers and subscribers
  nh.advertise( pub_done );
  nh.advertise(pub_distance);
  
  nh.subscribe(sub_heading);
  nh.subscribe(sub_grabBlock);
  nh.subscribe(sub_scan);
  nh.subscribe(sub_reset);

}

void loop() {
  // this one is all reactive, so everything happens in callbacks, leaving loop empty
  // on purpose

}
