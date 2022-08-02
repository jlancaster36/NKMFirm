/*
   BUTTONS
*/

typedef struct button
{
  int pin;
  int button;
} button_t;

button_t buttons[] = {
  {12, 1}, //Y
  {11, 2}, //B
  {10, 3}, //A
  {9, 4} //X,
  //  {A0, 14}, //DPAD_UP
  //  {A1, 15}, //DPAD_DOWN
  //  {A2, 16}, //DPAD_LEFT
  //  {A3, 17}, //DPAD_RIGHT
  //  {4, 5}, //LB
  //  {5, 6}, //LT
  //  {6, 7}, //RB
  //  {7, 8}, //RT
  //  {12, 13}, //MENU
  //  {A8, 18}, //MUTE
  //  {10, 11}, //VOL-
  //  {11, 12}, //VOL+
  //  {8, 9}, //SELECT
  //  {9, 10} //START
};

int buttons_size = sizeof(buttons) / sizeof(button_t);





/*
  Original code has been modified for Teensy 3.6
  Based on the JoystickMouseControl example code by Tom Igoe
  http://www.arduino.cc/en/Tutorial/JoystickMouseControl

  This example code is in the public domain.
  For Teensy, choose USB Type Mouse (or a type that includes Mouse) in the Tools menu.
  This sketch enables the Teensy to appear as a USB HID mouse controller, using a 2 axis
  analog joystick and buttons to control the mouse.
  Hardware:
  - 2-axis joystick connected to pins A0 and A1
  - pushbuttons connected to digital pins 0 and 1
  The mouse movement is always relative. This sketch reads two analog inputs
  that range from 0 to 1023 (or less on either end) and translates them into
  ranges of -6 to 6.
  The sketch assumes that the joystick resting values are around the middle of
  the range, but that they vary within a threshold.
*/

// set pin numbers for switch, joystick axes, and LED:
const int mouseEnable = 1;        // switch to turn on and off mouse control
const int mouseLeftButton = 0;    // input pin for the mouse pushButton
const int xAxis = A0;             // joystick X axis
const int yAxis = A1;             // joystick Y axis
const int xAxis2 = A2;             // joystick X axis
const int yAxis2 = A3;             // joystick Y axis
const int ledPin = 12;            // Mouse control LED

// parameters for reading the joystick:
int range = 12;               // output range of X or Y movement
int responseDelay = 5;        // response delay of the mouse, in ms
int threshold = range / 4;    // resting threshold
int center = range / 2;       // resting position value

bool mouseIsActive = false;   // whether or not to control the mouse
int lastSwitchState = LOW;    // previous switch state

void setup() {
  pinMode(mouseEnable, INPUT_PULLUP);           // the mouse enable toggle switch
  pinMode(mouseLeftButton, INPUT_PULLUP);       // the left mouse button pin
  pinMode(ledPin, OUTPUT);                      // the LED pin
  Serial.begin(9600);
  Mouse.begin();                                // take control of the mouse:
  Serial.println("USB HID Mouse Test.");        // debug message to confirm sketch is running

  //Digital pins buttons
  for (int i = 0; i < buttons_size; i++)
  {
    pinMode(buttons[i].pin, INPUT_PULLUP);
  }

  pinMode(11, INPUT_PULLUP);
}

//int buttonStat = 0;

void loop() {
  // read the mouse enable switch to control if mouse is active or disabled
  int switchState = digitalRead(mouseEnable);
  if (switchState != lastSwitchState) {
    if (switchState == HIGH) {
      mouseIsActive = !mouseIsActive;
      digitalWrite(ledPin, mouseIsActive);     // turn on LED to indicate mouse state:
    }
  }
  // save switch state for next comparison:
  lastSwitchState = switchState;

  // read and scale the two axes:
  int xReading = readAxis(xAxis);
  int yReading = readAxis(yAxis);

  // if the mouse control state is active, move the mouse:
  if (mouseIsActive) {
    Mouse.move(xReading, yReading, 0);         //  not using scroll wheel, just move X and Y
  }
  else {
    // Read inputs as gamepad if mouse not enabled

    Joystick.X(analogRead(xAxis)); //Left X
    Joystick.Y(analogRead(yAxis)); //Left Y
    Joystick.Z(analogRead(xAxis2)); //Right X
    Joystick.Zrotate(analogRead(yAxis2)); //Right Y

    //digital buttons
    //Digital pins buttons
    int i;
    for (i = 0; i < buttons_size; i++)
    {
      if (digitalRead(buttons[i].pin) == LOW)
      {
        Joystick.button(buttons[i].button, 1);
      }
      else
      {
        Joystick.button(buttons[i].button, 0);
      }
    }

//    int test_b = digitalRead(11);
//    if (buttonStat != test_b) {
//      buttonStat = test_b;
//     }
     
  }

  // read the left mouse button and assert or release a click if necessary:
  if (digitalRead(mouseLeftButton) == LOW) {   //  low means button is currently pressed
    if (!Mouse.isPressed(MOUSE_LEFT)) {        //  so assert the left mouse button if not already done
      Mouse.press(MOUSE_LEFT);
    }
  }
  // else the mouse button is currently high/not pressed:
  else {
    if (Mouse.isPressed(MOUSE_LEFT)) {         // if the left mouse button is currently pressed, release it:
      Mouse.release(MOUSE_LEFT);
    }
  }
  delay(responseDelay);
}

/*
  reads an axis (0 or 1 for x or y) and scales the analog input range to a range
  from 0 to <range>
  Analog Reading:    0 -----------------------------<Center>----------------------------- 1023
  Scaled Reading:    0 -----------------------------<Center>----------------------------- <range=12>

*/

int readAxis(int thisAxis) {
  // read the analog input:
  int reading = analogRead(thisAxis);

  // map the reading from the analog input range to the output range:
  reading = map(reading, 0, 1023, 0, range);

  // if the output reading is outside from the rest position threshold, use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
}
