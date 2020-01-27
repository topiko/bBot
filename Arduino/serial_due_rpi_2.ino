/*
  Serial Event example

  When new serial data arrives, this sketch adds it to a String.
  When a newline is received, the loop prints the string and clears it.

  A good test for this is to try it with a GPS receiver that sends out
  NMEA 0183 sentences.

  NOTE: The serialEvent() feature is not available on the Leonardo, Micro, or
  other ATmega32U4 based boards.

  created 9 May 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/SerialEvent
*/

int inInt = 0;
unsigned char command[3] = {0, 0, 0};
int countSer = 0;
int orient = 1000;
boolean report = false;  // whether the string is complete

void setup() {
  // initialize serial:
  Serial3.begin(115200);
}

void loop() {
  // print the int when newInt
  if (countSer == 1) {
    command[0] = inInt;
  }
  else if (countSer == 2) {
    command[1] = inInt;
  }
  else if (countSer == 3) {
    command[2] = inInt;
    
    Serial3.write(orient >> 8);
    Serial3.write(orient & 0xff);
    countSer = 0;
  }

  // if (interrupt) : orient = update_accel_ 

  // set_motor_speeds(command)

  // if speeds changed: run_motors()
  
}


void serialEvent3() {
  inInt = (int)Serial3.read();
  report = true;
  countSer += 1;
}

// void accel_interrupt() {}  

  /*
  while (Serial3.available()) {
    // get the new byte:
    char inChar = (char)Serial3.read();
    
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  */
