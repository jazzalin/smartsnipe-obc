/*
 * OBC Controller
 * Update OBC on peripheral status and events
*/
#include <ros.h>
#include <smartsnipe_msgs/ActuateDoor.h>
#include <smartsnipe_msgs/Shot.h>


// Door motor encoders parameters
#define ENC_D1_A 2
#define ENC_D1_B 3
#define ENC_D2_A 18
#define ENC_D2_B 4
#define ENC_D3_A 19
#define ENC_D3_B 5
#define ENC_D4_A 20
#define ENC_D4_B 6

// Door pins
int door_open[] = {12, 10, 8, 6};
int door_close[] = {13, 11, 9, 7};
int limit_open[] = {22, 24, 26, 28};
int limit_close[] = {23, 25, 27, 29};

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
int N[] = {400, 400, 400, 400};

// Radargun
void radarScan();

int segment[4][7];//LCD segment data stored here

float measuredSpeed = 0.0, oldSpeed = 0.0;//converted data to speed
int min_val = 1024;
int max_val = 0;

void doors_cb(const smartsnipe_msgs::ActuateDoor::Request & req, smartsnipe_msgs::ActuateDoor::Response & res)
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

void door_control(int open, int i)
{
  long start = 0;
  start = door_count[i];
  bool switch_enabled = false;
  // Open door
  if (open)
  {
    // digitalWrite(door_open[i], HIGH);
    analogWrite(door_open[i], 127);
    while(!switch_enabled && abs(door_count[i] - start) < N[i])
    {
      if(!switch_enabled && digitalRead(limit_open[i]))
      {
        switch_enabled = true;
      }
    };
    digitalWrite(door_open[i], LOW);
    // analogWrite(door_open[i], 127);
  } else // close door
  {
    analogWrite(door_close[i], 127);
    while(!switch_enabled && abs(door_count[i] - start) < N[i])
    {
      // if(!switch_enabled && digitalRead(limit_close[i]))
      // {
      //   switch_enabled = true;
      // }
    };
    digitalWrite(door_close[i], LOW);
  }
  state[i] = open;
}

void d1encoderEvent2x()
{
  if (digitalRead(ENC_D1_B) == 0) {
    if (digitalRead(ENC_D1_A) == 0) {
      // A fell, B is LOW
      door_count[0]--; // Moving reverse
    } else {
      // A rose, B is LOW
      door_count[0]++; // Moving reverse
    }
  } else {
    if (digitalRead(ENC_D1_A) == 0) {
      // A fell, B is HIGH
      door_count[0]++; // Moving reverse
    } else {
      // A rose, B is HIGH
      door_count[0]--; // Moving forward
    }
  }
}

void d2encoderEvent2x()
{
  if (digitalRead(ENC_D2_B) == 0) {
    if (digitalRead(ENC_D2_A) == 0) {
      door_count[1]--;
    } else {
      door_count[1]++;
    }
  } else {
    if (digitalRead(ENC_D2_A) == 0) {
      door_count[1]++;
    } else {
      door_count[1]--;
    }
  }
}

void d3encoderEvent2x()
{
  if (digitalRead(ENC_D3_B) == 0) {
    if (digitalRead(ENC_D3_A) == 0) {
      door_count[2]--;
    } else {
      door_count[2]++;
    }
  } else {
    if (digitalRead(ENC_D3_A) == 0) {
      door_count[2]++;
    } else {
      door_count[2]--;
    }
  }
}

void d4encoderEvent2x()
{
  if (digitalRead(ENC_D4_B) == 0) {
    if (digitalRead(ENC_D4_A) == 0) {
      door_count[3]--; 
    } else {
      door_count[3]++; 
    }
  } else {
    if (digitalRead(ENC_D4_A) == 0) {
      door_count[3]++; 
    } else {
      door_count[3]--; 
    }
  }
}


//##################################################

void setup()
{
  // ROS
  nh.initNode();
  nh.advertiseService(doorServer);
  nh.advertise(shotPub);
  // Serial.begin(9600); //Testing only

  // Pins
  pinMode(ENC_D1_A, INPUT);
  pinMode(ENC_D1_B, INPUT);
  pinMode(ENC_D2_A, INPUT);
  pinMode(ENC_D2_B, INPUT);
  pinMode(ENC_D3_A, INPUT);
  pinMode(ENC_D3_B, INPUT);
  pinMode(ENC_D4_A, INPUT);
  pinMode(ENC_D4_B, INPUT);

  pinMode(door_open[0], OUTPUT);
  pinMode(door_close[0], OUTPUT);
  pinMode(door_open[1], OUTPUT);
  pinMode(door_close[1], OUTPUT);
  pinMode(door_open[2], OUTPUT);
  pinMode(door_close[2], OUTPUT);
  pinMode(door_open[3], OUTPUT);
  pinMode(door_close[3], OUTPUT);

  // Init interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_D1_A), d1encoderEvent2x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D2_A), d2encoderEvent2x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D3_A), d3encoderEvent2x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D4_A), d4encoderEvent2x, CHANGE);
  
}

void loop()
{
  // example
  radarScan();
  shot_msg.stamp = nh.now();
  // TODO: replace shot_msg.door with door on which shot was made based on detection sensor reading (-1 otherwise)
  shot_msg.door = -1; 
  shot_msg.speed = measuredSpeed;
  shot_msg.goal = false;
  shotPub.publish(&shot_msg);
  // Serial.println(door_count[0]); //Testing only


  nh.spinOnce();
  delay(500);
}
