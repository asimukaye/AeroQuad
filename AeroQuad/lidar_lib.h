
#include "LIDARLite.h"
LIDARLite myLidarLite;

float prev_val;

int alt_read(){
  int temp = myLidarLite.distance();
  kal_filt = kal(temp);
  //Serial.print(temp);
 // Serial.print(" ");
 // Serial.println(kal_filt);
  //kal_filt = kal(myLidarLite.distance());
  //myLidarLite.distance(flase);
  return kal_filt;
}


float kal(signed long meas_val)
{
 /* if(meas_val != 0)
  {*/
    k = ((est_err+q)/(meas_err + est_err+q));
    est_val = (1-k)*est_val + k*meas_val;
    est_err = (1-k)*est_err;
   /* if(est_val != 0)
      prev_val = est_val;*/
    return(est_val);
/*  }
  else
    return(prev_val); */

}

