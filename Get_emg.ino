void get_emg()
{
  sum = 0; // Reset sum.
  currentMillis = micros(); //Get current time.
  if (currentMillis - previousEMGsignalMillis >= EMGsignal_Interval)//Run code if we have waited 10 ms. 
  {
    if(Serial2.available() > 0){//Check if theres data in the buffer.
      for (int i = 0; i <= 24; i++) {
        if(Serial2.available() > 0){//Doublecheck buffer.
         returnedValues[i] = Serial2.read(); //Save Serial data. 
        }
      }
    }   
    for (int i = 0; i < 24; i++) { //Calculated sum (lsb of sum should be 0xff).
      if (i > 2 && i < 24)
      {
        sum += returnedValues[i];
      }
    }
    EMGSignal[0] = word(returnedValues[19], returnedValues[20]);//Combine channel_1 lsb and msb
    EMGSignal[1] = word(returnedValues[21], returnedValues[22]);//Combine channel_2 lsb and msb
    if(lowByte(sum) == 0xff)//Only if the package sum is valid.
    {
      for (int i = 0; i < 2; i++) // Run filtering for each channel
      {
        YnValue[i] = weightFactor * EMGSignal[i] + (1.0- weightFactor) * YnValue[i];
        if (i == 0)
        {
          //Serial.print(YnValue[i], DEC); //Print filtered channel_1 data.
        }
        if (i == 1)
        {
          //Serial.print("\t");
          //Serial.print(YnValue[i], DEC); //Print filtered channel_2 data.
        }
      }
      //Serial.print("\t");
      Serial.println(dxl.getPresentVelocity(motornumber, UNIT_RPM));
    }
    previousEMGsignalMillis += EMGsignal_Interval; //Update the time for next run.
    //We make the package invalid by changing index 0 to 0xff:
    returnedValues[0]=0xff;
  }
}
