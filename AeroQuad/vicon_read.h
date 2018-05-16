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
    float vx;
    float vy;
    };
    
struct viconPose viconPose;


void InitializeVicon(){
    VICON_SERIAL.begin(115200);
    }

/*
void readValueSerial1(char *data, byte size) {
  byte index = 0;
  byte timeout = 0;
  data[0] = '\0';

  do {
    if (VICON_SERIAL.available() == 0) {
      delay(1);
      timeout++;
    }
     else {
      data[index] = VICON_SERIAL.read();
      timeout = 0;
      index++;
    }
  } while ((index == 0 || data[index-1] != ';') && (timeout < 10) && (index < size-1));

  data[index] = '\0';
}


// Used to read floating point values from the serial port
// Consider sending serial x and y of precise sizes instead of typedefs to avoid data loss
float readFloatSerial1() {
  char data[15] = "";
  readValueSerial1(data, sizeof(data));
  return atof(data);
}



void ViconRead(){
    char check;

    if (VICON_SERIAL.available() == 0) {
      viconPose.x = prevX;
      viconPose.y = prevY;
      tempintvariable = -1;
      //delay(1); //not sure of using this
      }
    else{
          check = VICON_SERIAL.read();
        if (check == 'x'){
              viconPose.x = readFloatSerial1();
              tempintvariable = 3;
              }
        else if (check == 'y'){
              viconPose.y = readFloatSerial1();
              tempintvariable = 6;
                  }
        else{
                tempintvariable = 2;
                      }
         viconPose.vx = (viconPose.x - prevX)*1000;
         viconPose.vy = (viconPose.y - prevY)*1000;
         prevX = viconPose.x;
         prevY = viconPose.y; 
                
          }
}
*/

void ViconRead2(){
  byte index = 0;
  byte timeout = 0;
  bool validdata = 0;
  char check = '\0';
  char data[10] = "";
  data[0] = '\0';

  
   if (VICON_SERIAL.available() == 0) {
      viconPose.x = prevX;
      viconPose.y = prevY;
      }
    else{
        //clear out first entry
        
        /*
      do { if (VICON_SERIAL.available() == 0) {
          delay(1);
          timeout++;
      }
        else {
        timeout = 0;
        check = VICON_SERIAL.read();
      }
          }while ((check !=  'x') && (timeout < 10));*/
          
       //consider second entry to be data   
      do {
        if (VICON_SERIAL.available() == 0) {
          delay(1); 
          timeout++;
          //miss++;
      }
        else {
        timeout = 0;
        //hits++;
        check = VICON_SERIAL.read();            
            
        if (check == 'x'){
            index = 0;
            validdata = 1;
            }         
        else if(check == 'y'){
            data[index] = '\0';
            //hits = index;
            validdata = 1;
            if (index < 2){
                viconPose.x = prevX;
                }
            else{
            viconPose.x = atof(data);} //adhoc solution to avoid data drops
            index = 0;}
            
        else if(check == ';'){
            data[index] = '\0';
            //miss = index;
            validdata = 0;
            viconPose.y = atof(data);}
                      
        else if (validdata) {
          data[index] = check;          //store as data
          index++;
        }        
      }
      }while ((index == 0 || check !=  ';') && (timeout < 10) && (index < sizeof(data)-1 ));
      //VICON_SERIAL.flush();
      //tempintvariable = timeout;
      //tempcharvariable = check;  
    }

   viconPose.vx = (viconPose.x - prevX)*10; //change this according to the POS_X_P gain values
   viconPose.vy = (viconPose.y - prevY)*10;
   prevX = viconPose.x;
   prevY = viconPose.y;     
   
}

#endif
#endif
