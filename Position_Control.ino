#include "MegunoLink.h"
#include "Filter.h"
#include "Dynamixel2Arduino.h"
 
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
int returnedValues[24];
double EMGSignal[2] = {0,0};
double YnValue[2] = {0,0};
int counter = 0;
int medianEMG1 = 0;
int medianEMG2 = 0;
double prevValue = 0;
double weightFactor = 1.5/100.0;
int sum = 0;
bool valid_pkg = false;
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;
double emg_ch0, emg_ch1, emg_ch0_prev, emg_ch1_prev, pos_output;

//Timer
<<<<<<< Updated upstream
  const int EMGsignal_Interval = 10;
=======
  const int EMGsignal_Interval = 15;
>>>>>>> Stashed changes
  const int Motor_Interval = 100;
  unsigned long currentMillis = 0;
  unsigned long previousEMGsignalMillis = 0;
  unsigned long previousMotorMillis = 0;
//
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
ExponentialFilter<long> ADCFilter(1, 0);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dxl.begin(57600);
  Serial2.begin(115200);
  pinMode(2, OUTPUT);
  emg_ch0_prev = 0;
  emg_ch1_prev = 0;
  for (int i = 1; i < 4; i++)
  {
      dxl.torqueOff(i);
      dxl.setOperatingMode(i, OP_POSITION);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, i, 100);
      dxl.writeControlTableItem(PROFILE_VELOCITY, i , 100);
      dxl.torqueOn(i);
  }
}

void loop() {
  get_emg();
}

void get_emg()
{
  if (currentMillis - previousEMGsignalMillis >= EMGsignal_Interval)
  {
    sum = 0;
    valid_pkg = false;
    Serial.println("Package:");
    //delay(4);
    if(Serial2.available() > 0){
      // && Serial2.read() == 0x00 && Serial2.read() == 0x14 && Serial2.read() == 0x83 && Serial2.read()== 0x56 && Serial2.read()==0x78
      if(Serial2.read() == 0x7E && Serial2.read() == 0x00 && Serial2.read() == 0x14 && Serial2.read() == 0x83 && Serial2.read()== 0x56 && Serial2.read()==0x78)
      {
          returnedValues[0] = 0x7E;
          returnedValues[1] = 0x00;
          returnedValues[2] = 0x14;
          returnedValues[3] = 0x83;
          returnedValues[4] = 0x56;
          returnedValues[5] = 0x78;
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
      if(EMGSignal[i] > 40 && EMGSignal[i] < 1200 && lowByte(sum) == 0xff)
      {
        YnValue[i] = weightFactor * EMGSignal[i] + (1.0- weightFactor) * YnValue[i];
        valid_pkg = true;
        Serial.println(YnValue[1], DEC);
        Serial.print(",");
        Serial.println(YnValue[0], DEC);
        /*for (int i = 0; i <= sizeof(returnedValues)/2; i++) {
        returnedValues[i] = 0;
        }*/
      }
    }
<<<<<<< Updated upstream
    /*if (valid_pkg)
    {
      //Serial.print("RAW:");
      Serial.println(YnValue[1], DEC);
      Serial.print(",");
      Serial.println(YnValue[0], DEC);
      emg_ch0 = YnValue[0];
      emg_ch1 = YnValue[1];
      // Move joint in positive direction: 
      if(emg_ch0 > emg_ch1 && emg_ch0 > 99 && emg_ch0 > emg_ch0_prev)
      {
        pos_output =  map(emg_ch0, 100, 1000, 90, 260);
        dxl.setGoalPosition(motornumber, pos_output, UNIT_DEGREE);
        emg_ch0_prev = emg_ch0;
        emg_ch1_prev = emg_ch1;
      }
      if(emg_ch1 > emg_ch0 && emg_ch1 > 99 && emg_ch1 > emg_ch1_prev)
      {
        pos_output =  map(emg_ch1, 100, 1000, 260, 90);
        dxl.setGoalPosition(motornumber, pos_output, UNIT_DEGREE);
        emg_ch1_prev = emg_ch1;
        emg_ch0_prev = emg_ch0;
      }
    }*/
=======
      /*if (valid_pkg)
      {
        //Serial.print("RAW:");
        Serial.println(YnValue[1], DEC);
        Serial.print(",");
        Serial.println(YnValue[0], DEC);
        emg_ch0 = YnValue[0];
        emg_ch1 = YnValue[1];
        // Move joint in positive direction: 
        if(emg_ch0 > emg_ch1 && emg_ch0 > 99 && emg_ch0 > emg_ch0_prev)
        {
          pos_output =  map(emg_ch0, 100, 1000, 90, 260);
          dxl.setGoalPosition(motornumber, pos_output, UNIT_DEGREE);
          emg_ch0_prev = emg_ch0;
          emg_ch1_prev = emg_ch1;
        }
        if(emg_ch1 > emg_ch0 && emg_ch1 > 99 && emg_ch1 > emg_ch1_prev)
        {
          pos_output =  map(emg_ch1, 100, 1000, 260, 90);
          dxl.setGoalPosition(motornumber, pos_output, UNIT_DEGREE);
          emg_ch1_prev = emg_ch1;
          emg_ch0_prev = emg_ch0;
        }
      }*/
    previousEMGsignalMillis += EMGsignal_Interval;
>>>>>>> Stashed changes
  }
}
