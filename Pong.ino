// CmdMessenger library available from https://github.com/dreamcat4/cmdmessenger
#include <CmdMessenger.h>

// Base64 library available from https://github.com/adamvr/arduino-base64
#include <Base64.h>

// Streaming4 library available from http://arduiniana.org/libraries/streaming/
#include <Streaming.h>

// Timer library from http://code.google.com/p/arduino-timerone/
#include <TimerOne.h>

// Mustnt conflict / collide with our message payload data. Fine if we use base64 library ^^ above
#define FIELD_SEPARATOR ','
#define COMMAND_SEPARATOR '\r\n'

CmdMessenger cmdMessenger = CmdMessenger(Serial, FIELD_SEPARATOR, COMMAND_SEPARATOR);

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

#define ACCELEROMETER_TRESHOLD 10
#define ACCELEROMETER_PIN_X A1
#define ACCELEROMETER_PIN_Y A2
#define ACCELEROMETER_PIN_Z A3

// Accelerometer value structure
typedef struct {
  int x;
  int y;
  int z;
} AccelerometerValue;

// Accelerometer values + previous values for comparison
AccelerometerValue accelerometer;
AccelerometerValue accelerometerOld;

// Piezo sensor settings and structures
#define PIEZO_PIN A0
#define PIEZO_TIMER_PERIOD 500
#define PIEZO_TAKE_SAMPLES 200
#define PIEZO_TRESHOLD 70
#define PIEZO_AFTER_EVENT_DELAY 150000

typedef struct {
  int currentIndex;
  int currentPeak;
  int currentPeakIndex;
  unsigned long currentPeakTime;
  unsigned long lastEventTime;
  int buffer[PIEZO_TAKE_SAMPLES];
} PiezoState;

PiezoState piezoState;

void setup()
{
  Serial.begin(115200); // use the serial port
  
  Timer1.initialize(PIEZO_TIMER_PERIOD);
  Timer1.disablePwm(1);
  Timer1.disablePwm(9);
  Timer1.disablePwm(2);
  Timer1.disablePwm(10);
  Timer1.attachInterrupt(handlePiezo);
  
  piezoStateReset(1);
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

// void piezoStateReset(PiezoState ps)
void piezoStateReset(int afterEvent)
{  
  piezoState.currentIndex = 0;
  piezoState.currentPeak = 0;
  piezoState.currentPeakIndex = 0;
  piezoState.currentPeakTime = micros();
  for(int i = 0; i < PIEZO_TAKE_SAMPLES; i++)
  {
    piezoState.buffer[i] = 0;
  }
  
  if(afterEvent)
  {
    piezoState.lastEventTime = micros();
  }
  else
  {
    piezoState.lastEventTime = 0;
  }
}

void handlePiezo()
{
  int currentValue = analogRead(PIEZO_PIN);
  unsigned long currentTime = micros();
  int currentIndex = (piezoState.currentIndex + 1) == PIEZO_TAKE_SAMPLES ? 0 : piezoState.currentIndex + 1;
  
  // Are we still in "idle" state after registering a knock on piezo?
  if((currentTime - piezoState.lastEventTime) < PIEZO_AFTER_EVENT_DELAY)
  {
    return;
  }
  
  // Do we have a peak value under currentIndex that will be "pushed out" from value buffer?
  if((piezoState.currentPeak > PIEZO_TRESHOLD) && (currentIndex == piezoState.currentPeakIndex))
  {
    sprintf(messageBuffer, "%d", piezoState.currentPeak);
    cmdMessenger.sendCmd(kKNOCK, messageBuffer);
    
    piezoStateReset(1);
    
    return;
  }
  
  // Otherwise just regular value "push in" and peak check
  piezoState.currentIndex = currentIndex;
  piezoState.buffer[piezoState.currentIndex] = currentValue;
  
  // Peak check
  if(piezoState.currentPeak < currentValue)
  {
    piezoState.currentPeak = currentValue;
    piezoState.currentPeakIndex = currentIndex;
    piezoState.currentPeakTime = micros();
  }
}

void loop()
{
    handleAccelerometer();
}

