#include "Filter.h"
#include "Dynamixel2Arduino.h"
#include <IRLibDecodeBase.h>  //We need both the coding and
#include <IRLib_P01_NEC.h>    //Lowest numbered protocol 1st
#include <IRLibCombo.h>       // After all protocols, include this
IRdecode myDecoder;
#include <IRLibRecv.h>

#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
double oldpos[5] = {180,180,180,270,50};
double pos_output[5] = {180, 180, 180, 270, 50};

bool gripperclosed = true;;
bool runnext = false;
int returnedValues[24];
double EMGSignal[2] = {0,0};
double YnValue[2] = {0,0};
double prevYnValue[2] = {0,0};
int counter = 0;
int medianEMG1 = 0;
int medianEMG2 = 0;
double prevValue = 0;
double weightFactor = 2/100.0;
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
//Timer:
  const int EMGsignal_Interval = 1000;
  const int Motor_Interval = 100;
  unsigned long currentMillis = 0;
  unsigned long previousEMGsignalMillis = 0;
  unsigned long previousMotorMillis = 0;
  unsigned long previousMillis = 0;
//
using namespace ControlTableItem;
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dxl.begin(57600);
  Serial2.begin(115200);
  pinMode(2, OUTPUT);
  myReceiver.enableIRIn(); // Start the receiver
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, HIGH);
  emg_ch0_prev = 0;
  emg_ch1_prev = 0;
  for (int i = 1; i < 6; i++)
  {
      dxl.torqueOff(i);
      dxl.setOperatingMode(i, OP_POSITION);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, i, 100);
      dxl.writeControlTableItem(PROFILE_VELOCITY, i , 100);
      dxl.torqueOn(i);
      dxl.setGoalPosition(i, pos_output[i-1], UNIT_DEGREE);

  }
  dxl.setGoalPosition(4, 270, UNIT_DEGREE);
  dxl.setGoalPosition(5, 90, UNIT_DEGREE);
}

void loop() {
  /*currentMillis = millis();
  if (currentMillis - previousEMGsignalMillis >= EMGsignal_Interval)
  {
    previousEMGsignalMillis += EMGsignal_Interval;
  }*/
  sum = 0;
  valid_pkg = false;
  get_emg();
  
}

void get_emg()
{
  while (valid_pkg == false)
  {
    Serial.println("Package:");
    if(Serial2.available()){
      if(Serial2.read() == 0x7E && Serial2.read() == 0x00 && Serial2.read() == 0x14 && Serial2.read() == 0x83 && Serial2.read()== 0x55 && Serial2.read()==0x66)
      {
        returnedValues[0] = 0x7E;
        returnedValues[1] = 0x00;
        returnedValues[2] = 0x14;
        returnedValues[3] = 0x83;
        returnedValues[4] = 0x55;
        returnedValues[5] = 0x66;
        //Serial.println("Size of start array = " + String(sizeof(returnedValues)/2));
        for (int i = 6; i <= sizeof(returnedValues)/2; i++) {
          returnedValues[i] = Serial2.read();
          //Serial.println(returnedValues[i], HEX);
        }    
      }
    }
      
    for (int i = 0; i < sizeof(returnedValues)/2; i++) {
      if (i > 2 && i < (sizeof(returnedValues)/2))
      {
        sum += returnedValues[i];
        //Serial.print("Nr " + String(i) + ":");
        //Serial.println(returnedValues[i], HEX);
      }
    }

    //Serial.println("Checksum is = " + String(lowByte(sum), HEX));
    EMGSignal[0] = word(returnedValues[19], returnedValues[20]);
    EMGSignal[1] = word(returnedValues[21], returnedValues[22]);
    for (int i = 0; i < 2; i++)
    {
      if(EMGSignal[i] < 1300 && lowByte(sum) == 0xff)
      {
        YnValue[i] = weightFactor * EMGSignal[i] + (1.0- weightFactor) * YnValue[i];
        valid_pkg = true;
        if (i == 0)
        {
          Serial.print(YnValue[i], DEC);
        }
        if (i == 1)
        {
          Serial.print("\t");
          Serial.println(YnValue[i], DEC);
        }
      }
    }
    for (int i = 0; i <= sizeof(returnedValues)/2; i++) {
      returnedValues[i] = 1;
    }
    if (valid_pkg)
    {
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
    } 
  }  
}

void update_motor()
{

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
      //Serial.println(myDecoder.value, HEX);
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
