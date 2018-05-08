/*
  AeroQuad v3.0 - May 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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

#ifndef _VICON_DATA_
#define _VICON_DATA_

#ifndef VICON_SERIAL
#define VICON_SERIAL Serial2

struct viconPose{
    float x;
    float y;
    };
    
struct viconPose viconPose;

//char check;

void readValueSerial1(char *data, byte size) {
  byte index = 0;
  byte timeout = 0;
  data[0] = '\0';

  do {
    if (VICON_SERIAL.available() == 0) {
      delay(1);
      timeout++;
    } else {
      data[index] = VICON_SERIAL.read();
      timeout = 0;
      index++;
    }
  } while ((index == 0 || data[index-1] != ';') && (timeout < 10) && (index < size-1));

  data[index] = '\0';
}


// Used to read floating point values from the serial port
float readFloatSerial1() {
  char data[15] = "";

  readValueSerial1(data, sizeof(data));
  return atof(data);
}

void InitializeVicon(){
    VICON_SERIAL.begin(115200);
    }

// Correct this one
void ViconRead(){
    char check;

    if (VICON_SERIAL.available() == 0) {
      delay(1);
      }
    else{
          check = VICON_SERIAL.read();
        if (check == 'x'){
              viconPose.x = readFloatSerial1();
              }
        else if (check == 'y'){
              viconPose.y = readFloatSerial1();
                  }
          }
  
            }
        //~if (data == ','){
            //~viconPose.y = VICON_SERIAL.read()}
        //~else{
            //~viconPose.x = float(data)}
        //}}
#endif
#endif
