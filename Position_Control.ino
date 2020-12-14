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
bool first_valid_pkg = false;
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;
double emg_ch0, emg_ch1, emg_ch0_prev, emg_ch1_prev, pos_output;

//Timer
  const int EMGsignal_Interval = 10000;
  const int Motor_Interval = 100;
  unsigned long currentMillis = 0;
  unsigned long previousEMGsignalMillis = 0;
  unsigned long previousMotorMillis = 0;
  unsigned long previousMillis = 0;
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
}

void loop() {
  currentMillis = micros();
  get_emg();
  Serial.println(currentMillis - previousMillis);
  previousMillis = currentMillis;
}

void get_emg()
{
  if (currentMillis - previousEMGsignalMillis >= EMGsignal_Interval)
  {
    sum = 0;
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
      if(lowByte(sum) == 0xff)
      {
        YnValue[i] = weightFactor * EMGSignal[i] + (1.0- weightFactor) * YnValue[i];
        first_valid_pkg = true;
        if (i == 0)
        {
          Serial.print(YnValue[i], DEC);
        }
        if (i == 1)
        {
          Serial.print("\t");
          Serial.println(YnValue[i], DEC);
        }
        for (int i = 0; i <= sizeof(returnedValues)/2; i++) {
        returnedValues[i] = 0;
        }
      }
    }
    previousEMGsignalMillis += EMGsignal_Interval;
  }
}
