// Hokuyo connected to beaglebone Black
#define Hokuyo_Serial Serial2

int boolean_to_decimal(bool Arr[]){
  int sum=0;
  for (int i = 0; i<18; i++){
    sum += Arr[i]*(1 << (17-i));
  }
  return sum;
}

void Init_Hokuyo(){
  Hokuyo_Serial.begin(115200);  	
}

void Hokuyo_Read(){
  if(Hokuyo_Serial.available()>=10){
    char check = Hokuyo_Serial.read();
    if(check == '+'){
      for(int i=0;i<9;i++){
      Hokuyo_str[i] = int(Hokuyo_Serial.read());
      Hokuyo_str[i] -=48;
      }
      for(int i=0;i<3;i++){  //3 data points
        for(int j=0;j<3;j++){  //each data point encrypted in 3 bytes
          for(int k=0;k<6;k++){
            bit_array[k + 6*j] = bitRead(Hokuyo_str[3*i+j],(5-k));  //one data point
          }
        }
        distance2D[i] = boolean_to_decimal(bit_array)/10;
      }      
    }
  }
  //output distance2D
  if(Hokuyo_Serial.available()>=10){
    Hokuyo_Read();
  }
}


float kal_hokuyoX(signed long distX){
  kX = ((est_errX+qX)/(meas_errX + est_errX+qX));
  est_valX = (1-kX)*est_valX + kX*distX;
  est_errX = (1-kX)*est_errX;
  return(est_valX);
   
}
float kal_hokuyoY(signed long distY){
  kY = ((est_errY+qY)/(meas_errY + est_errY+qY));
  est_valY = (1-kY)*est_valY + kY*distY;
  est_errY = (1-kY)*est_errY;
  return(est_valY);
   
}

