//Header file with all variables used:
//All variables are public.

//Xbee package handling
int returnedValues[24];
int sum = 0;
//Motor control
bool gripperclosed = false;
double oldpos[6] = {0,180,180,180,270,50};
double pos_output[6] = {0,180, 180, 180, 270, 50};
//EMG Filter
double EMGSignal[2] = {0,0};
double YnValue[2] = {0,0};
double prevYnValue[2] = {0,0};
double weightFactor = 1.5/100.0;
double emg_ch0, emg_ch1, emg_ch0_prev, emg_ch1_prev;
//Timer
  unsigned long EMGsignal_Interval = 10000; //In microseconds (1000 microsecond to 1 millisecond).
  const int Motor_Interval = 30000; //In micro seconds. 
  unsigned long currentMillis = 0;
  unsigned long previousEMGsignalMillis = 0;
  unsigned long previousMotorMillis = 0;
  unsigned long previousMillis = 0;
//IR
  const byte interruptPin = 3;
  long unsigned int old_decodevalue;
  int motornumber = 1;
  bool runnext = false;
//
