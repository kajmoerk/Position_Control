//Function for updating motors with new position:
void update_motor()
{
  currentMillis = micros();//Get current time.
  if (currentMillis - previousMotorMillis >= Motor_Interval)//Run code if we have waited 30 ms. 
  {
    //Get EMG channel values.
    emg_ch0 = YnValue[0];
    emg_ch1 = YnValue[1];
    // Move joint in positive direction:
    if(emg_ch0 > emg_ch1 && emg_ch0 > 300 && emg_ch0 != emg_ch0_prev)
    {
      if(motornumber == 3)//Wrist joint.
      {
        pos_output[motornumber] += 0.5;//Increment.
        //Keep position below limits.
        if(pos_output[motornumber] > 240)
        {
          pos_output[motornumber] = 240;
        }
      }
      if(motornumber == 2)//Elbow joint.
      {
        pos_output[motornumber] += 0.5;//Increment
        //Keep position below limits.
        if(pos_output[motornumber] > 190)
        {
          pos_output[motornumber] = 190;
        }
      }
      if(motornumber == 1)//Shoulder joint.
      {
        pos_output[motornumber] += 0.5;//Increment.
        //Keep position below limits.
        if(pos_output[motornumber] > 230)
        {
          pos_output[motornumber] = 230;
        }
      }
      //Send new position to motor with motornumber ID.
      if (motornumber <= 3 && oldpos[motornumber] != pos_output[motornumber])
      {
        dxl.setGoalPosition(motornumber, pos_output[motornumber], UNIT_DEGREE);
        oldpos[motornumber] = pos_output[motornumber];//Save new oldpos.
      }
      else if(gripperclosed && motornumber == 4)//motornumber 4 is the gripper.
      {
        //Open the gripper.
        dxl.setGoalPosition(4, 270, UNIT_DEGREE);
        dxl.setGoalPosition(5, 90, UNIT_DEGREE);
        gripperclosed = false;
      }
      //Save previous EMG values.
      emg_ch0_prev = emg_ch0;
      emg_ch1_prev = emg_ch1;
    }
    //Move joint in negative direction:
    if(emg_ch1 > emg_ch0 && emg_ch1 > 300 && emg_ch1 != emg_ch1_prev)
    {
      if(motornumber == 3)//Wrist joint.
      {
        pos_output[motornumber] -= 0.5;//Decrement
        //Keep position below limits.
        if(pos_output[motornumber] < 120)
        {
          pos_output[motornumber] = 120;
        }
      }
      if(motornumber == 2)//Elbow joint.
      {
        pos_output[motornumber] -= 0.5;//Decrement
        //Keep position below limits.
        if(pos_output[motornumber] < 70)
        {
          pos_output[motornumber] = 70;
        }
      }
      if(motornumber == 1)//Shoulder joint.
      {
        pos_output[motornumber] -= 0.5;//Decrement
        //Keep position below limits.
        if(pos_output[motornumber] < 0)
        {
          pos_output[motornumber] = 0;
        }
      }
      //Send new position to motor with motornumber ID.
      if (motornumber <= 3 && oldpos[motornumber] != pos_output[motornumber])
      {
        dxl.setGoalPosition(motornumber, pos_output[motornumber], UNIT_DEGREE);
        oldpos[motornumber] = pos_output[motornumber];//Save new oldpos.
      }
      else if(!gripperclosed && motornumber ==4)//motornumber 4 is the gripper.
      {
        //Clode the gripper
        dxl.setGoalPosition(4, 210, UNIT_DEGREE);
        dxl.setGoalPosition(5, 165, UNIT_DEGREE);
        gripperclosed = true;
      }
      //Save previous EMG values.
      emg_ch1_prev = emg_ch1;
      emg_ch0_prev = emg_ch0;
    }
   previousMotorMillis += Motor_Interval; //Update the time for next run.
  }
}
