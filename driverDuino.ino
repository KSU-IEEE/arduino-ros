#include <psuedo-tank.h>
#include <ros.h>
#include <behaviors/coordinate.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

/*
 * driverDuino.ino 
 * This file drives the wheels.....duh
 * 
 * It will accept inputs in the form of distances to move,
 * and turn commands, as well as read in current pose 
 * and heading
 */
 
// CONSTANTS
/**********************************************************************/


 // initializer variables: 
/*********************************************************************/
float heading, x, y;
std_msgs::Bool doneMove;
bool overUnder; // if True, checking coords until greater than target,
                // else it will check coords until less than target
bool check_x;   // determines whether or not to check x or y. If true
                // check x values, else y
float target_x, target_y;

ros::NodeHandle nh;


// init bot
/*********************************************************************/
psuedoTank bot (A1, A2, 13, A3, A4, 12, A2);


// pubs 
/*********************************************************************/
ros::Publisher pub_done("/bot/doneMove", &doneMove);

// callbacks
/*********************************************************************/
void move_cb(const std_msgs::Float32& val) {
  // moving in x or y
  if (int(heading) == 0) {
    // moving in y
    check_x = false;
    overUnder = true;
    target_y = y + val.data;
  } else if (int(heading) == 180) {
    check_x = false;
    overUnder = false;
    target_y = y - val.data;
  } else if (int(heading) == 90) {
    check_x = true;
    overUnder = true;
    target_x = x + val.data;
  } else {
    check_x = true;
    overUnder = false;
    target_x = x - val.data;
  }

  // send move command
  bot.forward();
  doneMove.data = false;

}

void turnLeft_cb(const std_msgs::Bool & val) {
  // send left command
  bot.left();
  doneMove.data = true;
  pub_done.publish( &doneMove);
}

void turnRight_cb(const std_msgs::Bool & val) {
  bot.right();
  doneMove.data = true;
  pub_done.publish( &doneMove);
}

void turn180_cb(const std_msgs::Bool & val) {
  bot.left();
  bot.left();
  doneMove.data = true;
  pub_done.publish( &doneMove);
}

void pose_cb(const behaviors::coordinate &loc) {
  x = loc.X;
  y = loc.Y;
}

void head_cb(const std_msgs::Float64 &head) {
  heading = int(head.data) % 360;
}

// subscribers
/*********************************************************************/
ros::Subscriber <std_msgs::Float32> sub_move("/bot/move", &move_cb);
ros::Subscriber <std_msgs::Bool> sub_turnLeft("/bot/turnLeft", &turnLeft_cb);
ros::Subscriber <std_msgs::Bool> sub_trunRight("/bot/turnRight", &turnRight_cb);
ros::Subscriber <std_msgs::Bool> sub_turn180("/bot/turn180", &turn180_cb);
ros::Subscriber <behaviors::coordinate> sub_pose("position", &pose_cb);
ros::Subscriber <std_msgs::Float64> sub_heading("heading", &head_cb);


// normal stuff
/*********************************************************************/
void setup() {
  nh.initNode();
  while (!nh.connected()) {
      nh.spinOnce();
  }

  // setup pubs and subs
  nh.advertise(pub_done);

  nh.subscribe(sub_move);
  nh.subscribe(sub_turnLeft);
  nh.subscribe(sub_trunRight);
  nh.subscribe(sub_turn180);
  nh.subscribe(sub_pose);
  nh.subscribe(sub_heading);

}

void loop() {
  // don't do anything unless set
  if(!doneMove.data) {
    if (overUnder ) {
      if (check_x)doneMove.data = (x >= target_x);  
      else doneMove.data = (y >= target_y);
    } else {
      if(check_x) doneMove.data = (x <= target_x);
      else doneMove.data = (y <= target_y);
    }

  if (doneMove.data) {
    // if passed, then send stop signals
    bot.brake();
    pub_done.publish( &doneMove );
  }
  nh.spinOnce();
}
}
