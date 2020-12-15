//Function for when the IR interupt happens. 
void IR_recv()
{
  oldpos[motornumber] = pos_output[motornumber];//Save current motors position
  if (myReceiver.getResults())//If we receive a IR signal
  {
    myDecoder.decode();//decode IR signal
    //storeCode();
    if (myDecoder.value != REPEAT_CODE && myDecoder.value != old_decodevalue)//Continue if its a new IR value.
    {
      old_decodevalue = myDecoder.value;//Update old IR value
      switch(myDecoder.value)//Change current motor depending on received IR signal.
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
  //Retrieve new motors old position.
  pos_output[motornumber] = oldpos[motornumber];
  //Reset both EMG channel's values. 
  emg_ch1_prev = 0; 
  emg_ch1 = 0;
  emg_ch0_prev = 0; 
  emg_ch0 = 0;
}
