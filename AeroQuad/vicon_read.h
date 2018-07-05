
#ifndef _VICON_DATA_
#define _VICON_DATA_

#ifndef VICON_SERIAL
#define VICON_SERIAL Serial2
#define VELXY_SCALING 10.0 //This value should be roughly equal to the\
 loop frequency of pos hold
struct viconPose{
    float x;
    float y;
    float vx;
    float vy;
    float x_sp;
    float y_sp;
    };
    
struct viconPose viconPose;


void InitializeVicon(){
    VICON_SERIAL.begin(115200);
    }

void ViconWrite(){
    VICON_SERIAL.print("a");
    VICON_SERIAL.print(",");
    VICON_SERIAL.print(PositionHoldState);
    VICON_SERIAL.print(",");
    VICON_SERIAL.print(X_origin);
    VICON_SERIAL.print(",");
    VICON_SERIAL.println(Y_origin);
    }
    
void Vicon_IMU_Write(){
    VICON_SERIAL.print(1000*kinematicsAngle[XAXIS]);
    VICON_SERIAL.print(",");
    VICON_SERIAL.print(1000*kinematicsAngle[YAXIS]);
    VICON_SERIAL.print(",");
    VICON_SERIAL.print(1000*gyroHeading);
    VICON_SERIAL.print(",");
    VICON_SERIAL.print(100*gyroRate[XAXIS]);
    VICON_SERIAL.print(",");
    VICON_SERIAL.print(100*gyroRate[YAXIS]);
    VICON_SERIAL.print(",");
    VICON_SERIAL.print(100*gyroRate[ZAXIS]);
    VICON_SERIAL.print(",");
    VICON_SERIAL.print(100*filteredAccel[XAXIS]);
    VICON_SERIAL.print(",");
    VICON_SERIAL.print(100*filteredAccel[YAXIS]);
    VICON_SERIAL.print(",");
    VICON_SERIAL.println(100*filteredAccel[ZAXIS]);
    }

    
void ViconRead(){
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
    do {
        if (VICON_SERIAL.available() == 0) {
          delay(1); 
          timeout++;
            }
        else {
        timeout = 0;
        
        check = VICON_SERIAL.read();            
            if (check == 'p'){
                index = 0;
                validdata = 1;
                }
            
            else if(check == 'q'){
                data[index] = '\0';
                if (validdata){
                    viconPose.x_sp = atof(data);
                    }
                validdata = 1;
                index = 0;
                }
               
            else if(check == 'x'){
                data[index] = '\0';
                if (validdata){
                    viconPose.y_sp = atof(data);
                    }
                validdata = 1;
                index = 0;
                }
                         
            else if(check == 'y'){
                data[index] = '\0';
                if (validdata){
                    viconPose.x = atof(data);
                    }
                else{
                    viconPose.x += viconPose.vx;
                    } //adhoc solution to avoid data drops, quad should continue on its previous velocity
                validdata = 1;
                index = 0;
                }
                
            /*else if(check == 'u'){
                data[index] = '\0';
                if (validdata){
                    viconPose.y = atof(data);
                    }
                else{
                    viconPose.y += viconPose.vy;
                    } 
                validdata = 1;
                index = 0;
                }
            else if(check == 'v'){
                data[index] = '\0';
                if (validdata){
                    viconPose.vx = atof(data);
                    }
                validdata = 1;
                index = 0;
                }*/
                
            else if(check == ';'){
                data[index] = '\0';
                if (validdata){
                    viconPose.y = atof(data);
                    }
                else{
                    viconPose.y += viconPose.vy;
                    }
                validdata = 0;
                } 
                                      
            else if (validdata) {
              data[index] = check;          //store as data
              index++;
                }      
            }
        }while ((index == 0 || check !=  ';') && (timeout < 10) && (index < sizeof(data)-1 ));
        }

   //viconPose.vx = viconPose.vx*VEL_SCALING;
   //viconPose.vy = viconPose.vy*VEL_SCALING;
    viconPose.x = constrain(viconPose.x, -6.0, 6.0);
    viconPose.y = constrain(viconPose.y, -6.0, 6.0);

   viconPose.vx = (viconPose.x - prevX)*VELXY_SCALING;
   viconPose.vy = (viconPose.y - prevY)*VELXY_SCALING; //old vel scaling value w/o ekf  
   
      viconPose.vx = constrain(viconPose.vx, -5.0, 5.0);
      viconPose.vy = constrain(viconPose.vy, -5.0, 5.0);
    
#if defined FilterXY
   viconPose.vx = kal(viconPose.vx, x_est_val, x_est_err, 5.0, 10);
   viconPose.vy = kal(viconPose.vy, y_est_val, y_est_err, 5.0, 10);
#endif
   prevX = viconPose.x;
   prevY = viconPose.y;     
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

/*
void ViconRead3(){
  byte index = 0;
  byte timeout = 0;
  bool validdata = 0;
  char check = '\0';
  char data[10] = "";
  data[0] = '\0';

  
   if (VICON_SERIAL.available() == 0) {
      viconPose.x += viconPose.vx;
      viconPose.y += viconPose.vy;
      }
    else{
      do {
        if (VICON_SERIAL.available() == 0) {
          delay(1); 
          timeout++;
          miss++;
      }
        else {
        timeout = 0;
        hits++;
        check = VICON_SERIAL.read();            
            
        if (check == 'y'){
            index = 0;
            validdata = 1;
            }         
        else if(check == 'x'){
            data[index] = '\0';
            //hits = index;
            if (validdata){
                viconPose.y = atof(data);
                ready++;
                }
            else{
				viconPose.x += viconPose.vx;
            } //adhoc solution to avoid data drops, quad should continue on its previous velocity
            validdata = 1;
            index = 0;}
            
        else if(check == ';'){
            data[index] = '\0';
            //miss = index;
            if (validdata){
                viconPose.x = atof(data);
                readx++;
                }
            else{
				viconPose.y += viconPose.vy;
            } 
            validdata = 0;}
                      
        else if (validdata) {
          data[index] = check;          //store as data
          index++;
        }
        else if (!validdata){
			garbage++;}        
      }
      }while ((index == 0 || check !=  ';') && (timeout < 10) && (index < sizeof(data)-1 ));
      //VICON_SERIAL.flush();
      //tempintvariable = timeout;
      //tempcharvariable = check;  
    }

   viconPose.vx = (viconPose.x - prevX);
   viconPose.vy = (viconPose.y - prevY); 
   prevX = viconPose.x;
   prevY = viconPose.y;     
}*/
#endif
#endif
