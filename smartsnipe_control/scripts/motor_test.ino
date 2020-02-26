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
#define ENC_D5_A 21
#define ENC_D5_B 7

// Door pins
int door_open[] = {22, 24, 26, 28, 30};
int door_close[] = {23, 25, 27, 29, 31};

ros::NodeHandle nh;
// Services
void doors_cb(const smartsnipe_msgs::ActuateDoor::Request & req, smartsnipe_msgs::ActuateDoor::Response & res);
ros::ServiceServer<smartsnipe_msgs::ActuateDoor::Request, smartsnipe_msgs::ActuateDoor::Response> doorServer("doors", &doors_cb);
// Topics
smartsnipe_msgs::Shot shot_msg;
ros::Publisher shotPub("shot_stats", &shot_msg);

volatile long door_count[] = {0, 0, 0, 0, 0};
int state[] = {0, 0, 0, 0, 0}; // by default, all doors start closed
// TODO: update motor tick specs
int N[] = {400, 400, 400, 800, 800};

void doors_cb(const smartsnipe_msgs::ActuateDoor::Request & req, smartsnipe_msgs::ActuateDoor::Response & res)
{
  for (int i = 0; i < 1; i++)
  {
    // Check if door is already in desired state
    // if(req.doors[i] != state[i])
    // {
    nh.loginfo("Received");
    door_control(req.doors[i],i);
// 
  }
  res.success = true;
  res.message = "Door actuated";
}

void door_control(int open, int i)
{
  long start = 0;
  start = door_count[i];
  // Open door
  if (open)
  {
    digitalWrite(door_open[i], HIGH);
    while(abs(door_count[i] - start) < N[i]) {};
    digitalWrite(door_open[i], LOW);
  } else // close door
  {
    digitalWrite(door_close[i], HIGH);
    while(abs(door_count[i] - start) < N[i]) {};
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

void d5encoderEvent2x()
{
  if (digitalRead(ENC_D5_B) == 0) {
    if (digitalRead(ENC_D5_A) == 0) {
      door_count[4]--;
    } else {
      door_count[4]++;
    }
  } else {
    if (digitalRead(ENC_D5_A) == 0) {
      door_count[4]++;
    } else {
      door_count[4]--;
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
  pinMode(ENC_D5_A, INPUT);
  pinMode(ENC_D5_B, INPUT);

  pinMode(door_open[0], OUTPUT);
  pinMode(door_close[0], OUTPUT);
  pinMode(door_open[1], OUTPUT);
  pinMode(door_close[1], OUTPUT);
  pinMode(door_open[2], OUTPUT);
  pinMode(door_close[2], OUTPUT);
  pinMode(door_open[3], OUTPUT);
  pinMode(door_close[3], OUTPUT);
  pinMode(door_open[4], OUTPUT);
  pinMode(door_close[4], OUTPUT);

  // Init interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_D1_A), d1encoderEvent2x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D2_A), d2encoderEvent2x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D3_A), d3encoderEvent2x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D4_A), d4encoderEvent2x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D5_A), d5encoderEvent2x, CHANGE);
  
}

void loop()
{
  // example
  shot_msg.stamp = nh.now();
  shot_msg.door = 1;
  shot_msg.speed = 100.0;
  shot_msg.goal = false;
  shotPub.publish(&shot_msg);
  // Serial.println(door_count[0]); //Testing only


  nh.spinOnce();
  delay(100);
}
