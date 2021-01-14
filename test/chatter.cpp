#include <ros.h>
#include <std_msgs/String.h>

#include <Arduino.h>

ros::NodeHandle nh;

// publisher messages
std_msgs::String str_msg;
std_msgs::String my_msg;

// define publishers
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher arduino_ros("/arduino/message", &my_msg);

// callback functions 
void tester_cb(const std_msgs::String& str) {
  my_msg.data = str.data;

  arduino_ros.publish( &my_msg );
}


// define subscribers
ros::Subscriber<std_msgs::String> tester_sub("tester1/message", &tester_cb);

char hello[13] = "hello world!";


void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(arduino_ros);
  nh.subscribe(tester_sub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}