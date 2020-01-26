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
boolean report = false;  // whether the string is complete

void setup() {
  // initialize serial:
  Serial3.begin(115200);
}

void loop() {
  // print the int when newInt
  if (report) {
    inInt += 1;
    Serial3.print(inInt);
    //Serial3.write(inInt);
    report = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent3() {
  inInt = (int)Serial3.read();
  report = true;
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
}
