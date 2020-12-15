#include "Dynamixel2Arduino.h"
#include <IRLibDecodeBase.h>  //We need both the coding and
#include <IRLib_P01_NEC.h>    //Lowest numbered protocol 1st
#include <IRLibCombo.h>       // After all protocols, include this
IRdecode myDecoder;
#include <IRLibRecv.h>

//Dynamixel lib:
using namespace ControlTableItem;
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
//

bool gripperclosed = false;
bool runnext = false;
double oldpos[6] = {0,180,180,180,270,50};
double pos_output[6] = {0,180, 180, 180, 270, 50};
int returnedValues[24];
double EMGSignal[2] = {0,0};
double YnValue[2] = {0,0};
double prevYnValue[2] = {0,0};
double weightFactor = 1.5/100.0;
int sum = 0;
bool valid_pkg = false;
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;
double emg_ch0, emg_ch1, emg_ch0_prev, emg_ch1_prev;
//IR:
  IRrecv myReceiver(3); //pin number for the receiver
  const byte interruptPin = 3;
  long unsigned int old_decodevalue;
  int motornumber = 1;
  Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//
//Timer
  unsigned long EMGsignal_Interval = 10000;
  const int Motor_Interval = 30000;
  unsigned long currentMillis = 0;
  unsigned long previousEMGsignalMillis = 0;
  unsigned long previousMotorMillis = 0;
  unsigned long previousMillis = 0;
//


void setup() {
  //Serial port setup:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  Serial2.begin(115200);
  //Pin mode setup:
  pinMode(2, OUTPUT);
  //Start value setup:
  emg_ch0_prev = 0;
  emg_ch1_prev = 0;
  //IR and interrupter setup:
  myReceiver.enableIRIn(); // Start the receiver
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, HIGH);
  //Motor Setup:
  for (int i = 1; i < 6; i++)
  {
      dxl.torqueOff(i);
      dxl.setOperatingMode(i, OP_POSITION);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, i, 100);
      dxl.writeControlTableItem(PROFILE_VELOCITY, i , 100);
      dxl.torqueOn(i);
      dxl.setGoalPosition(i, pos_output[i], UNIT_DEGREE);

  }
  dxl.setGoalPosition(4, 270, UNIT_DEGREE);
  dxl.setGoalPosition(5, 90, UNIT_DEGREE);
}

void loop() {
  get_emg();
  update_motor();
}

void get_emg()
{
  valid_pkg = false;
  sum = 0;
  currentMillis = micros();
  if (currentMillis - previousEMGsignalMillis >= EMGsignal_Interval)
  {
    if(Serial2.available() > 0){
      for (int i = 0; i <= 24; i++) {
        if(Serial2.available() > 0){
         returnedValues[i] = Serial2.read(); 
        }
      }
    }   
    for (int i = 0; i < 24; i++) {
      if (i > 2 && i < 24)
      {
        sum += returnedValues[i];
      }
    }
    EMGSignal[0] = word(returnedValues[19], returnedValues[20]);
    EMGSignal[1] = word(returnedValues[21], returnedValues[22]);
    for (int i = 0; i < 2; i++)
    {
      if(lowByte(sum) == 0xff)
      {
        YnValue[i] = weightFactor * EMGSignal[i] + (1.0- weightFactor) * YnValue[i];
        if (i == 0)
        {
          Serial.print(YnValue[i], DEC);
        }
        if (i == 1)
        {
          Serial.print("\t");
          Serial.println(YnValue[i], DEC);
        }
        valid_pkg = true;
      }
    }
    previousEMGsignalMillis += EMGsignal_Interval;
    //We make the package invalid by changing index 0 to 0xff:
    returnedValues[0]=0xff;
  }
}

void update_motor()
{
  currentMillis = micros();
  if (currentMillis - previousMotorMillis >= Motor_Interval)
  {
    //if (valid_pkg)
    //{
      emg_ch0 = YnValue[0];
      emg_ch1 = YnValue[1];
      // Move joint in positive direction:
      if(emg_ch0 > emg_ch1 && emg_ch0 > 300 && emg_ch0 != emg_ch0_prev)
      {
        if(motornumber == 3)
        {
          pos_output[motornumber] += 0.5;
          if(pos_output[motornumber] > 240)
          {
            pos_output[motornumber] = 240;
          }
        }
        if(motornumber == 2)
        {
          pos_output[motornumber] += 0.5;
          if(pos_output[motornumber] > 190)
          {
            pos_output[motornumber] = 190;
          }
        }
        if(motornumber == 1)
        {
          pos_output[motornumber] += 0.5;
          if(pos_output[motornumber] > 230)
          {
            pos_output[motornumber] = 230;
          }
        }
        if (motornumber <= 3 && oldpos[motornumber] != pos_output[motornumber])
        {
          dxl.setGoalPosition(motornumber, pos_output[motornumber], UNIT_DEGREE);
          oldpos[motornumber] = pos_output[motornumber];
        }
        else if(gripperclosed && motornumber == 4)
        {
          dxl.setGoalPosition(4, 270, UNIT_DEGREE);
          dxl.setGoalPosition(5, 90, UNIT_DEGREE);
          gripperclosed = false;
        }
        emg_ch0_prev = emg_ch0;
        emg_ch1_prev = emg_ch1;
      }
      //Move joint in negative direction:
      if(emg_ch1 > emg_ch0 && emg_ch1 > 300 && emg_ch1 != emg_ch1_prev)
      {
        if(motornumber == 3)
        {
          pos_output[motornumber] -= 0.5;
          if(pos_output[motornumber] < 120)
          {
            pos_output[motornumber] = 120;
          }
        }
        if(motornumber == 2)
        {
          pos_output[motornumber] -= 0.5;
          if(pos_output[motornumber] < 70)
          {
            pos_output[motornumber] = 70;
          }
        }
        if(motornumber == 1)
        {
          pos_output[motornumber] -= 0.5;
          if(pos_output[motornumber] < 0)
          {
            pos_output[motornumber] = 0;
          }
        }
        if (motornumber <= 3 && oldpos[motornumber] != pos_output[motornumber])
        {
          dxl.setGoalPosition(motornumber, pos_output[motornumber], UNIT_DEGREE);
          oldpos[motornumber] = pos_output[motornumber];
        }
        else if(!gripperclosed && motornumber ==4)
        {
          dxl.setGoalPosition(4, 210, UNIT_DEGREE);
          dxl.setGoalPosition(5, 165, UNIT_DEGREE);
          gripperclosed = true;
        }
        emg_ch1_prev = emg_ch1;
        emg_ch0_prev = emg_ch0;
      }
    //}
   previousMotorMillis += Motor_Interval; 
  }
}

void blink()
{
  oldpos[motornumber] = pos_output[motornumber];
  if (myReceiver.getResults())
  {
    myDecoder.decode();
    //storeCode();
    if (myDecoder.value != REPEAT_CODE && myDecoder.value != old_decodevalue)
    {
      old_decodevalue = myDecoder.value;
      switch(myDecoder.value)
      {
        case 0xFFA25D:
          motornumber = 1;
          break;
        case 0xFF629D:
          motornumber = 2;
          break;
        case 0xFFE21D:
          motornumber = 3;
          break;
        case 0xFF22DD:
          motornumber = 4;
          break;
        default:
          break;
      }
    }
    myReceiver.enableIRIn(); // Re-enable receiver
  }
  pos_output[motornumber] = oldpos[motornumber];
  emg_ch1_prev = 0; 
  emg_ch1 = 0;
  emg_ch0_prev = 0; 
  emg_ch0 = 0;
}
