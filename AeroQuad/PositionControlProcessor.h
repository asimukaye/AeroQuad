/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 
 
  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 
 
  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef _AQ_POSITION_CONTROL_PROCESSOR_H_
#define _AQ_POSITION_CONTROL_PROCESSOR_H_


/* /// do not uncomment
  motorCommand[FRONT] = throttle - motorAxisCommandPitch  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR] =  throttle + motorAxisCommandPitch  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[RIGHT] = throttle - motorAxisCommandRoll   + (YAW_DIRECTION * motorAxisCommandYaw) + MotorOffset_Roll;
  motorCommand[LEFT] =  throttle + motorAxisCommandRoll   + (YAW_DIRECTION * motorAxisCommandYaw) - MotorOffset_Roll;
*/


#if defined Lidar2D
#define INVALID_THROTTLE_CORRECTION_HOKUYO -1000
void processHokuyoHold()
{
  if (HokuyoHoldState == ON) {
    int temphokuyoHoldThrottleCorrection_X = INVALID_THROTTLE_CORRECTION_HOKUYO;
    int temphokuyoHoldThrottleCorrection_Y = INVALID_THROTTLE_CORRECTION_HOKUYO;
    #if defined Hold_X  //roll
      int error_X = HokuyoPositionToHoldTarget_X - distance2D[plus_X];
      temphokuyoHoldThrottleCorrection_X = updatePID(HokuyoPositionToHoldTarget_X, distance2D[plus_X], &PID[GPSROLL_PID_IDX]);
      temphokuyoHoldThrottleCorrection_XGLOBAL = temphokuyoHoldThrottleCorrection_X;
      hokuyoHoldThrottleCorrection_X = temphokuyoHoldThrottleCorrection_X;
      if(((error_X<0)&&(RollVelocityDirection ==0)) || ((error_X>0)&&(RollVelocityDirection ==1))){
        hokuyoHoldThrottleCorrection_X = temphokuyoHoldThrottleCorrection_X/3;
      }
      hokuyoHoldThrottleCorrection_X = constrain(hokuyoHoldThrottleCorrection_X, minThrottleAdjust,maxThrottleAdjust);      
      if((-10<error_X)&&(error_X<10)){
        hokuyoHoldThrottleCorrection_X = 0;
      }      
    #endif
    if (temphokuyoHoldThrottleCorrection_X == INVALID_THROTTLE_CORRECTION_HOKUYO) {
        hokuyoHoldThrottleCorrection_X =0;
    }
    #if defined Hold_Y  //pitch
      int error_Y = HokuyoPositionToHoldTarget_Y - distance2D[plus_Y];
      temphokuyoHoldThrottleCorrection_Y = updatePID(HokuyoPositionToHoldTarget_Y, distance2D[plus_Y], &PID[GPSPITCH_PID_IDX]);
      temphokuyoHoldThrottleCorrection_YGLOBAL = temphokuyoHoldThrottleCorrection_Y;
      hokuyoHoldThrottleCorrection_Y = temphokuyoHoldThrottleCorrection_Y;
      if(((error_Y<0)&&(PitchVelocityDirection ==0)) || ((error_Y>0)&&(PitchVelocityDirection ==1))){
        hokuyoHoldThrottleCorrection_Y = temphokuyoHoldThrottleCorrection_Y/3;
      }      
      hokuyoHoldThrottleCorrection_Y = constrain(hokuyoHoldThrottleCorrection_Y, minThrottleAdjust,maxThrottleAdjust);
      if((-10<error_Y)&&(error_Y<10)){
        hokuyoHoldThrottleCorrection_Y = 0;
      }    
    #endif
    if (temphokuyoHoldThrottleCorrection_Y == INVALID_THROTTLE_CORRECTION_HOKUYO) {
        hokuyoHoldThrottleCorrection_Y =0;
    }	
  }
  else{
    hokuyoHoldThrottleCorrection_X   = 0;
    hokuyoHoldThrottleCorrection_Y  = 0;
  }
  
}

#endif

#endif // _AQ_Hokuyo_CONTROL_PROCESSOR_H_
