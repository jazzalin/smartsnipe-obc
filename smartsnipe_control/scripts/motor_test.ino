/*
 * OBC Controller
 * Update OBC on peripheral status and events
*/
#include <ros.h>
#include <smartsnipe_msgs/ActuateDoor.h>
#include <smartsnipe_msgs/Shot.h>


// Door pins
int door_open[] = {11, 8, 6, 12};
int door_close[] = {10, 9, 7, 13};
int limit_open[] = {21, 29, 5, 17};
int limit_close[] = {26, 14, 31, 16};
int door_speed = 200;

// Receiver pins
int sensorPin[5] = {A11, A12, A13, A14, A15}; 
int sensorVal[5]; 
int sensorAvg[5];

int count = 0;
int min_count = 0;
int tempValue = 0;
int tempDiff = 0;
bool goal_detected = 0;
bool goal_scored = 0;
unsigned int goal_count[] = {0, 0, 0, 0};

ros::NodeHandle nh;
// Services
void doors_cb(const smartsnipe_msgs::ActuateDoor::Request & req, smartsnipe_msgs::ActuateDoor::Response & res);
ros::ServiceServer<smartsnipe_msgs::ActuateDoor::Request, smartsnipe_msgs::ActuateDoor::Response> doorServer("doors", &doors_cb);
// Topics
smartsnipe_msgs::Shot shot_msg;
ros::Publisher shotPub("shot_stats", &shot_msg);

volatile long door_count[] = {0, 0, 0, 0};
int state[] = {0, 0, 0, 0}; // by default, all doors start closed
// TODO: update motor tick specs
int N[] = {300, 300, 300, 300};

// Radargun
void radarScan();

int segment[4][7];//LCD segment data stored here

float measuredSpeed = 0.0, oldSpeed = 0.0;//converted data to speed
int min_val = 1024;
int max_val = 0;
bool reset = true;

void doors_cb(const smartsnipe_msgs::ActuateDoor::Request & req, smartsnipe_msgs::ActuateDoor::Response & res)
{
  if (req.reset)
  {
    for (int i = 0; i < 4; i++)
    {
      door_control(req.doors[i],i);
    }
  }
  else
  {
    for (int i = 0; i < 4; i++)
    {
      // Check if door is already in desired state
      if(req.doors[i] != state[i])
      {
        door_control(req.doors[i],i);
      }
    }
    res.success = true;
    res.message = "Door actuated";
  }
}

void door_control(int open, int i)
{
  long start = 0;
  start = door_count[i];
  bool switch_enabled = false;
  // Open door
  if (open)
  {
    analogWrite(door_open[i], door_speed);
    while(!switch_enabled)
    {
      if(!switch_enabled && digitalRead(limit_open[i]))
      {
        switch_enabled = true;
      }
    };
    digitalWrite(door_open[i], LOW);
  } else // close door
  {
    analogWrite(door_close[i], door_speed);
    while(!switch_enabled)
    {
      if(!switch_enabled && digitalRead(limit_close[i]))
      {
        switch_enabled = true;
      }
    };
    digitalWrite(door_close[i], LOW);
  }
  state[i] = open;
}

//##################################################

void setup()
{
  // ROS
  nh.initNode();
  nh.advertiseService(doorServer);
  nh.advertise(shotPub);
  shot_msg.door = (uint8_t *)malloc(sizeof(uint8_t)*4);
  shot_msg.door_length = 4;
  // Serial.begin(9600); //Testing only

  // Pins
  pinMode(door_open[0], OUTPUT);
  pinMode(door_close[0], OUTPUT);
  pinMode(door_open[1], OUTPUT);
  pinMode(door_close[1], OUTPUT);
  pinMode(door_open[2], OUTPUT);
  pinMode(door_close[2], OUTPUT);
  pinMode(door_open[3], OUTPUT);
  pinMode(door_close[3], OUTPUT);

  receiver_setup();
  
}

long publisher_timer;

void loop()
{
  // Scanning for shot speed and goal detection
  receiver_read();
  radarScan();
  if (millis() > publisher_timer)
  {
    shot_msg.stamp = nh.now();
    shot_msg.door[0] = goal_count[0];
    shot_msg.door[1] = goal_count[1];
    shot_msg.door[2] = goal_count[2];
    shot_msg.door[3] = goal_count[3];
    shot_msg.speed = measuredSpeed;

    if (goal_scored)
    {
      shot_msg.goal = true;
      goal_scored = false;
    } else
    {
      shot_msg.goal false;
    }
    shotPub.publish(&shot_msg);
    publisher_timer = millis() + 1000;
  }

  nh.spinOnce();
  delay(500);
}
