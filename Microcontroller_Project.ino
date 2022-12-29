// ---------------------------------------------------------------------------
// Before attempting to use this sketch, please read the "Help with 15 Sensors Example Sketch":
// https://bitbucket.org/teckel12/arduino-new-ping/wiki/Help%20with%2015%20Sensors%20Example%20Sketch
//
// This example code was used to successfully communicate with 15 ultrasonic sensors. You can adjust
// the number of sensors in your project by changing SONAR_NUM and the number of NewPing objects in the
// "sonar" array. You also need to change the pins for each sensor for the NewPing objects. Each sensor
// is pinged at 33ms intervals. So, one cycle of all sensors takes 495ms (33 * 15 = 495ms). The results
// are sent to the "oneSensorCycle" function which currently just displays the distance data. Your project
// would normally process the sensor results in this function (for example, decide if a robot needs to
// turn and call the turn function). Keep in mind this example is event-driven. Your complete sketch needs
// to be written so there's no "delay" commands and the loop() cycles at faster than a 33ms rate. If other
// processes take longer than 33ms, you'll need to increase PING_INTERVAL so it doesn't get behind.
// ---------------------------------------------------------------------------
#include <NewPing.h>
#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>


#define SONAR_NUM     4 // Number of sensors.
#define MAX_DISTANCE 250 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

const int motorPinLeft  = 8;
const int motorPinCenter = 3;
const int motorPinRight   = 2;
const int motorPinBelow = 13;

int minLeftDistance   = 100;
int minCenterDistance = 100;
int minRightDistance  = 100;
int minBelowDistance  = 10;

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

static unsigned long timer = millis();

int waitTime = 3500;

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(6, 6, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(5, 5, MAX_DISTANCE),
  NewPing(4, 4, MAX_DISTANCE),
  NewPing(7, 7, MAX_DISTANCE),
};
SoftwareSerial mp3SoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini DFPlayer;

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!")); 
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void leftAlert(int distance) {
  
  if (millis() - timer > waitTime) {
    timer = millis();
    DFPlayer.playLargeFolder(01, distance+1);
  } 
  if (DFPlayer.available()) {
    printDetail(DFPlayer.readType(), DFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.

  }
}

void centerAlert(int distance) {

  if (millis() - timer > waitTime) {
    timer = millis();
    DFPlayer.playLargeFolder(02, distance+1);
  } 
  if (DFPlayer.available()) {
    printDetail(DFPlayer.readType(), DFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states
  }
}

void rightAlert(int distance) {
  
  if (millis() - timer > waitTime) {
    timer = millis();
    DFPlayer.playLargeFolder(03, distance+1);
  } 
  if (DFPlayer.available()) {
    printDetail(DFPlayer.readType(), DFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states
  }
}

void belowAlert(int distance) {
  
  if (millis() - timer > waitTime) {
    timer = millis();
    DFPlayer.playLargeFolder(04, 0001);
  } 
  if (DFPlayer.available()) {
    printDetail(DFPlayer.readType(), DFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states
  }
}

void holeAlert(int distance) {
  
  if (millis() - timer > waitTime) {
    timer = millis();
    DFPlayer.playLargeFolder(04, 0002);
  } 
  if (DFPlayer.available()) {
    printDetail(DFPlayer.readType(), DFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(motorPinLeft, OUTPUT);
  pinMode(motorPinCenter, OUTPUT);
  pinMode(motorPinRight, OUTPUT);
  pinMode(motorPinBelow, OUTPUT);
  mp3SoftwareSerial.begin(9600);
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!DFPlayer.begin(mp3SoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  DFPlayer.volume(10);  //Set volume value. From 0 to 30
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.

      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // Other code that *DOESN'T* analyze ping results can go here.
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()) { // This is how you check to see if the ping was received.
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM; //get the results in centimeter
  if(cm[0] < minLeftDistance && cm[0] > 1) {
    
    Serial.print("Obstacle ");
    Serial.print(cm[0]);
    Serial.println(" CM to the left");

    leftAlert(cm[0]);
    analogWrite(motorPinLeft, 512);
  }
  else {
    analogWrite(motorPinLeft, 0);
  }

  if(cm[1] < minCenterDistance && cm[1] > 1) {

    Serial.print("Obstacle ");
    Serial.print(cm[1]);
    Serial.println(" CM in front");

    centerAlert(cm[1]);
    digitalWrite(motorPinCenter, HIGH);
    }
  else {
    digitalWrite(motorPinCenter, LOW);
  }
  
  if(cm[2] < minRightDistance && cm[2] > 1) {

    Serial.print("Obstacle ");
    Serial.print(cm[2]);
    Serial.println(" CM to the right");

    rightAlert(cm[2]);
    analogWrite(motorPinRight, 512);
  }
  else {
    analogWrite(motorPinRight, 0);
  }
  
  if(cm[3] < minBelowDistance && cm[3] > 1) {

    Serial.print("Obstacle ");
    Serial.print(cm[3]);
    Serial.println(" watch out there is an obstacle below!! ");

    belowAlert(cm[3]);
    digitalWrite(motorPinBelow, HIGH);
  }
  else if (cm[3] > 60 ) {

    Serial.print("Obstacle ");
    Serial.print(cm[3]);
    Serial.println(" watch out there is a hole below!! ");

    holeAlert(cm[3]);
    digitalWrite(motorPinBelow, HIGH);
  }
  else {
    digitalWrite(motorPinBelow, LOW);
  }
}
}
