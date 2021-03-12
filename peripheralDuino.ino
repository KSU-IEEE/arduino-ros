#include <NewPing.h> // for sonar sensor
#include <noise-recognition.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <behaviors/coordinate.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Char.h>
/*
 * peripheralDuino:
 * Starts by waiting for a start signal
 * Once it accepts, it will starting waiting for sound
 * Then it will send location data 
 */

// CONSTANTS
/*********************************************************************/
#define VAR 100
#define TARGET_FREQ 10000


// initializer variables: 
/*********************************************************************/
float heading;
behaviors::coordinate loc;
std_msgs::Char PP;
std_msgs::Bool start;
float prevDist[4]; // front back -- 0 = front left, 1 = front right, 2 = back left, 3 = back right 
enum FSM {WAIT, LISTEN, POSE};
int state;
bool isTurning = false;

ros::NodeHandle nh;


// pubs 
/*********************************************************************/
ros::Publisher pub_position("position", &loc);
ros::Publisher pub_ppLoc("PPLocation", &PP);
ros::Publisher pub_start("StartSignal", &start);


// sensors
/*********************************************************************/
// this duino will only have 6 of the peripheral sensors, the others are being passed over via ros
NewPing sensors [4] = {
  NewPing (2, 3), // front left
  NewPing (4, 5), // front right
  NewPing (6, 7), // back left
  NewPing (8, 9), // back right
};

noise_recognition::noiseRecognition nr(A0, VAR, TARGET_FREQ);

// callbacks
/*********************************************************************/
void heading_cb( const std_msgs::Float64& new_head) {
  heading = int(new_head.data) % 360;
}

void startListen_cb( const std_msgs::Bool& yes) {
  if (yes.data) {
    state = FSM::LISTEN;
  }
}

void turning_cb(const std_msgs::Bool& yes) {
  if (yes.data) {
    isTurning = true;
  }
}

void finished_cb(const std_msgs::Bool & yes) {
  if (yes.data && isTurning) {
    isTurning = false;

    // get new previous values
  for (int sensor = 0; sensor < 4; sensor++)
    prevDist[sensor] = sensors[sensor].ping_in(); 
  }
}


// subscribers
/*********************************************************************/
ros::Subscriber <std_msgs::Float64> sub_heading ("heading", &heading_cb);
ros::Subscriber <std_msgs::Bool> sub_startListen("StartListening", 1000);
ros::Subscriber <std_msgs::Bool> sub_turnLeft("/bot/turnLeft", &turning_cb);
ros::Subscriber <std_msgs::Bool> sub_turnRight("/bot/turnRight", &turning_cb);
ros::Subscriber <std_msgs::Bool> sub_turn180("/bot/turn180", &turning_cb);
ros::Subscriber <std_msgs::Bool> sub_doneMove("/bot/doneMove", &finished_cb);


// pose updates
/*********************************************************************/
// rounding to .5's 
float round_map(float num) {
  // get decimal to round
  float dec = abs(num) - abs(int(num));

  // handle rounding
  if (dec < .4) {
    dec = 0;
  } else if (dec < .8) {
    dec = .5;
  } else {
    dec = 1;
  }

  return (num > 0) ? (int(num) + dec) : (int(num) - dec);
}

// distance sensing functions
void update_pos() {
  if(!isTurning) {
  float currDist[4];
  float sum = 0;
  
  // check if moved 
  // first get current distances
  for (int sensor = 0; sensor < 4; sensor++){
    currDist[sensor] = sensors[sensor].ping_in();

    // add to sum after rounding
    if (sensor < 3){ // have to treat forward and backward distances seperately
      sum += round_map(currDist[sensor] - prevDist[sensor]);
    } else {
      sum +=round_map(currDist[sensor] + prevDist[sensor]);
    }
  }

  // average diff and round again
  float avg = round_map(sum/4);

  
  // updat pose values
  if (int(heading) == 0 || int(heading) == 180) {
    loc.Y += avg;
  } else {
    loc.X += avg;
  }

  if(avg != 0) {
    for(int i = 0; i < 4; i++) {
      prevDist[i] = currDist[i];
    }
  }

  // send pose 
  pub_position.publish( &loc );
  }
}


// noise recognition stuff
/*********************************************************************/
void do_noise_rec() {
  /*
   * If we get an input in this, then we assume the power pellets are in 
   * C, else they are in A
   */
  bool inC = false;
  float start = millis();
  int thirty_seconds_ms = 25000; // rounding this down to cut it early if needed
  bool flicker = true;
  while(millis() - start < thirty_seconds_ms) {
    inC = nr.listenFor(TARGET_FREQ);

    // flicker status light
    digitalWrite(10, (flicker) ? HIGH : LOW);
    flicker = !flicker;
    
    if(inC) break;
  }

  // set status lights
  digitalWrite(11, (inC) ? LOW : HIGH);
  digitalWrite(12, (inC) ? HIGH : LOW);
  
  PP.data = (inC) ? 'C' : 'A';
  pub_ppLoc.publish(&PP);
}


// normal stuff
/*********************************************************************/
void setup() {
  nh.initNode();
   while (!nh.connected()) {
        nh.spinOnce();
    }

  // setup fsm
  state = FSM::WAIT;

  // setup publishers and subscribers
  nh.advertise(pub_position);
  nh.advertise(pub_ppLoc);
  nh.advertise(pub_start);
  
  nh.subscribe(sub_heading);
  nh.subscribe(sub_startListen);
  nh.subscribe(sub_turnLeft);
  nh.subscribe(sub_turnRight);
  nh.subscribe(sub_turn180);
  nh.subscribe(sub_doneMove);

  // setup switch
  pinMode(13, INPUT);

  //setup LEDS
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  // get initial reads
  for (int sensor = 0; sensor < 4; sensor++){
    prevDist[sensor] = sensors[sensor].ping_in(); 
  }
}

void loop() {
  switch(state) {
    case (FSM::WAIT):  // break out of this in startListen_cb()
      int reading = digitalRead(13); // checkout the switch
      if (reading == HIGH) {
        start.data = true;
        pub_start.publish(&start);
      }
      break;
    case (FSM::LISTEN): // break out after a value is sent
      do_noise_rec();
      float val;
      nh.getParam("starting_x", &val, 1);
      loc.X = val;
      nh.getParam("starting_y", &val, 1);
      loc.Y = val;
      nh.getParam("startingHeading", &heading);
      break;
    case (FSM::POSE): // do this forever after the first two are called
      if (!isTurning) update_pos();
      break;
     nh.spinOnce();
  }

}
