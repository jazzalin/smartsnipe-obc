
/*
 * Photodiode puck detection system
 */
void receiver_setup() {
  pinMode(8, OUTPUT);
  for (int i = 0; i<5; i++){
    pinMode(sensorPin[i], INPUT_PULLUP);
  }
//  Serial.begin(9600);
}

void receiver_read() {
  for(int j = 0; j < 1; j++){
    for (int i = 0; i<5; i++){
      sensorVal[i] = analogRead(sensorPin[i]);
      tempValue = sensorAvg[i];
      sensorAvg[i] = ((tempValue * 5) + sensorVal[i])/6;
//      Serial.println(tempValue);
//      if (i==4)
//        Serial.println(sensorAvg[i]);
      if (count > min_count){
        tempDiff = sensorAvg[i] - tempValue;
        //Serial.println(tempDiff);
        if (abs(tempDiff) > 100){
          goal_detected = 1;
          count = 0;
        }
      }
    }
    if (goal_detected){
       digitalWrite(8, HIGH);
      goal_scored = 1;
      count = 0;
      goal_count[j] = goal_count[j] + 1;
      goal_detected = 0;
      delay(100);
       digitalWrite(8, LOW);
      }
  
    count ++;
    if (count > 1000){
      count = min_count;
    }
  }
   digitalWrite(8, LOW);
}
