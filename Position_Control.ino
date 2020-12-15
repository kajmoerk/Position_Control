#include "Dynamixel2Arduino.h"
#include <IRLibDecodeBase.h>  //We need both the coding and
#include <IRLib_P01_NEC.h>    //Lowest numbered protocol 1st
#include <IRLibCombo.h>       // After all protocols, include this
//Initialize IR object
IRdecode myDecoder;
#include <IRLibRecv.h>
#include "Variables.h"
//Dynamixel lib:
using namespace ControlTableItem;
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
//Setup IR receiver pin:
IRrecv myReceiver(3);
//Initialize Dynamixel object
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  //Serial port setup:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  Serial2.begin(115200);//Xbee serial port.
  //Pin mode setup:
  pinMode(2, OUTPUT);//Read/Write pin for RS485.
  //Start value setup:
  emg_ch0_prev = 0;
  emg_ch1_prev = 0;
  //IR and interrupter setup:
  myReceiver.enableIRIn(); // Start the receiver
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), IR_recv, HIGH);
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
