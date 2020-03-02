/* Arduino Radar Gun - UPDATED FOR RADAR BOARD
   Data Read Test Code - Arduino Leonardo
   http://www.kevindarrah.com/wiki/index.php?title=Arduino_Radar_Gun

   Determine speed displayed by radar fun by decoding the lines of its LCD display
*/

//TODO: configure analog thresholds correctly
void radarScan(){

  unsigned long scanTimeStart = millis();//timeout for scanning LCD

  while (millis() - scanTimeStart < 100) {//jump in and scan the LCD!
    for (int i = 7; i < 11; i++) { //sweep the common pins
      int val1 = analogRead(i);
//      Serial.println(val1);
      if (val1 < 100) {// LOW
        if (val1 < min_val) {
          min_val = val1;
        //   Serial.print("Min: ");
        //   Serial.println(min_val);  
        }
        for (int j = 0; j < 7; j++) {//sweep the segments after we find an enabled common
          int val2 = analogRead(j); 
          if (val2 > 500) {//HIGH
            if (val2 > max_val) {
              max_val = val2;
            //   Serial.print("Max: ");
            //   Serial.println(max_val);  
            }
            segment[i - 7][j] = 1;//set the segment to a 1
          }
        }
      }
    }
  }

  measuredSpeed = 0;//clear the speed, and we'll set it now based on the segment data

  /*
      ONES POSITION
  */

  //     3,5
  // 3,4     2,5
  //     2,4
  // 1,4     1,5
  //     0,5

  //Zero - Ones
  if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 0 && segment[1][4] == 1 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 0.0;
  }
  //One - Ones
  if (segment[3][4] == 0 && segment[3][5] == 0 && segment[2][5] == 1 && segment[2][4] == 0 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 0) {
    measuredSpeed = 1.0;
  }
  //Two - Ones
  if (segment[3][4] == 0 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 1 && segment[1][5] == 0 && segment[0][5] == 1) {
    measuredSpeed = 2.0;
  }
  //Three - Ones
  if (segment[3][4] == 0 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 3.0;
  }
  //Four - Ones
  if (segment[3][4] == 1 && segment[3][5] == 0 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 0) {
    measuredSpeed = 4.0;
  }
  //Five - Ones
  if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 0 && segment[2][4] == 1 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 5.0;
  }
  //Six - Ones
  if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 0 && segment[2][4] == 1 && segment[1][4] == 1 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 6.0;
  }
  //Seven - Ones
  if (segment[3][4] == 0 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 0 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 0) {
    measuredSpeed = 7.0;
  }
  //Eight - Ones
  if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 1 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 8.0;
  }
  //Nine - Ones
  if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 9.0;
  }


  /*
     TENS POSITION
  */
  //     3,3
  // 3,2     2,3
  //     2,2
  // 1,2     1,3
  //     0,3

  //One - Tens
  if (segment[3][2] == 0 && segment[2][2] == 0 && segment[2][3] == 1 && segment[3][3] == 0 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 0) {
    measuredSpeed = measuredSpeed + (1.0 * 10);
  }
  //Two - Tens
  if (segment[3][2] == 0 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 1 && segment[1][3] == 0 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (2.0 * 10);
  }
  //Three - Tens
  if (segment[3][2] == 0 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (3.0 * 10);
  }
  //Four - Tens
  if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 0 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 0) {
    measuredSpeed = measuredSpeed + (4.0 * 10);
  }
  //Five - Tens
  if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 0 && segment[3][3] == 1 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (5.0 * 10);
  }
  //Six - Tens
  if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 0 && segment[3][3] == 1 && segment[1][2] == 1 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (6.0 * 10);
  }
  //Seven - Tens
  if (segment[3][2] == 0 && segment[2][2] == 0 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 0) {
    measuredSpeed = measuredSpeed + (7.0 * 10);
  }
  //Eight - Tens
  if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 1 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (8.0 * 10);
  }
  //Nine - Tens
  if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (9.0 * 10);
  }

  /*
     HUNDREDS
  */
  //     3,1
  // 3,0     2,1
  //     2,0
  // 1,0     1,1
  //     0,1
  //One - Hundreds
  if (segment[3][1] == 0 && segment[3][0] == 0 && segment[2][1] == 1 && segment[2][0] == 0 && segment[1][0] == 0 && segment[1][1] == 1 && segment[0][1] == 0) {
    measuredSpeed = measuredSpeed + (1.0 * 100);
  }
  if (segment[3][1] == 1 && segment[3][0] == 0 && segment[2][1] == 1 && segment[2][0] == 1 && segment[1][0] == 1 && segment[1][1] == 0 && segment[0][1] == 1) {
    measuredSpeed = measuredSpeed + (2.0 * 100);
  }



  if (oldSpeed != measuredSpeed) {//ony print speed if it's new
    nh.logdebug(String(measuredSpeed).c_str());
  }
  oldSpeed = measuredSpeed;

//   for (int i = 0; i < 4; i++) {//for degugging and clearing segment data
//     Serial.print(i);
//     Serial.print(": ");
//     for (int j = 0; j < 7; j++) {
//       Serial.print(segment[i][j]);
//       Serial.print(",");
//       segment[i][j] = 0;
//     }
//     Serial.println("  ");
//   }
}