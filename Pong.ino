// CmdMessenger library available from https://github.com/dreamcat4/cmdmessenger
#include <CmdMessenger.h>

// Base64 library available from https://github.com/adamvr/arduino-base64
#include <Base64.h>

// Streaming4 library available from http://arduiniana.org/libraries/streaming/
#include <Streaming.h>

// Mustnt conflict / collide with our message payload data. Fine if we use base64 library ^^ above
char field_separator = ',';
char command_separator = '\n';

CmdMessenger cmdMessenger = CmdMessenger(Serial, field_separator, command_separator);

enum
{
  kCOMM_ERROR    = 000, // Lets Arduino report serial port comm error back to the PC (only works for some comm errors)
  kACK           = 001, // Arduino acknowledges cmd was received
  kARDUINO_READY = 002, // After opening the comm port, send this cmd 02 from PC to check arduino is ready
  kERR           = 003, // Arduino reports badly formatted cmd, or cmd not recognised

  // Our own messages:
  kKNOCK         = 004,
  kACCELEROMETER = 005,
  // Our messages ends
  
  kSEND_CMDS_END, // Mustnt delete this line
};

// Message buffer for talking to computer
char messageBuffer[100];

#define STATE_SILENT 0
#define STATE_RISE 1
#define STATE_FALL 2

#define TAKE_SAMPLES 30
#define STATE_CHANGE_TRESHOLD 20
#define LED_PIN 13
#define SENSOR_PIN A0

#define ACCELEROMETER_TRESHOLD 10
#define ACCELEROMETER_PIN_X A1
#define ACCELEROMETER_PIN_Y A2
#define ACCELEROMETER_PIN_Z A3

int ledState = LOW;

// Last peak value - will be our knock "velocity"
int peakValue = 0;

// Edge of signal from piezo will rise until it's peak and fall. The
int state = STATE_SILENT;

// We'll store number of TAKE_SAMPLES last samples in this array
int lastSensorValues[TAKE_SAMPLES];

// "Pointer" to latest sample
int lastSampleIndex = 0;

// Accelerometer value structure
typedef struct {
  int x;
  int y;
  int z;
} AccelerometerValue;

// Accelerometer values + previous values for comparison
AccelerometerValue accelerometer;
AccelerometerValue accelerometerOld;

void setup()
{
  pinMode(LED_PIN, OUTPUT); // declare the ledPin as as OUTPUT
  Serial.begin(115200);       // use the serial port
    
  // Clear the storage.. Just in case.
  for(int i = 0; i < TAKE_SAMPLES; i++) {
    lastSensorValues[i] = 0;
  }
}

void handleAccelerometer()
{
  int value = 0;
  AccelerometerValue diff;
  
  // Read accelerometer
  accelerometer.x = analogRead(ACCELEROMETER_PIN_X);
  accelerometer.y = analogRead(ACCELEROMETER_PIN_Y);
  accelerometer.z = analogRead(ACCELEROMETER_PIN_Z);
  
  // Calculate diff vector
  diff.x = accelerometer.x - accelerometerOld.x;
  diff.y = accelerometer.x - accelerometerOld.x;
  diff.z = accelerometer.z - accelerometerOld.z;
  value = sqrt(pow(diff.x, 2) + pow(diff.y, 2) + pow(diff.z, 2));
  
  // Send message only in case of 
  if(value > ACCELEROMETER_TRESHOLD)
  {
    accelerometerOld = accelerometer;
    sprintf(messageBuffer, "%d,%d,%d", accelerometer.x, accelerometer.y, accelerometer.z);
    cmdMessenger.sendCmd(kACCELEROMETER, messageBuffer);
  }
}

void handlePiezo()
{
  int lastState = state;
  int previousSampleIndex;
  int sensorReading = 0;
    
  int goneUp = 0;
  int goneDown = 0;
   
  // Store the new sensor value
  sensorReading = analogRead(SENSOR_PIN);
    
  // Do we have a change in sensor reading?
  if(sensorReading != lastSensorValues[lastSampleIndex]) {
    // Store the new value
    lastSampleIndex++;
    lastSampleIndex = lastSampleIndex > TAKE_SAMPLES ? 0 : lastSampleIndex;
    lastSensorValues[lastSampleIndex] = sensorReading;
    
    // Store the peak (if it's peak)
    peakValue = peakValue < sensorReading ? sensorReading : peakValue;
    
    // Check for state changes
    for(int i = 1; i < TAKE_SAMPLES; i++) {
      if(lastSensorValues[i-1] > lastSensorValues[i]) {
        goneUp++;
      } else {
        // We store only unique values so it must go up or down, there's no "stayedLevel"
        goneDown++;
      }
    }
    
    // See in what new state we are in
    if(goneUp > STATE_CHANGE_TRESHOLD) {
      state = STATE_RISE;
    } else if(goneDown > STATE_CHANGE_TRESHOLD) {
      state = STATE_FALL;
    }
    
    // Do we have a state change?
    if(lastState == STATE_RISE && state == STATE_FALL) {
      // Fire!
      sprintf(messageBuffer, "%d", peakValue);
      cmdMessenger.sendCmd(kKNOCK, messageBuffer);
      
      // Clean up, ie reset peak
      peakValue = 0;
      
      // Delay so we can ignore the vibration that follows (ie piezo flexes couple times)
      // 0.1s is enough for our application.. Might be a bit slow for drummer though.
      delay(30);
    }
  }
}

void loop()
{
    handlePiezo();
    handleAccelerometer();
}

