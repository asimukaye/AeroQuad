
#include "LIDARLite.h"
LIDARLite myLidarLite;
/*
float prev_val;
unsigned char i2caddress[2] = {0x64, 0x66};
// use STM pins below!
int pinsArray[2] = {50, 52};
*/
/*
float kal(float meas_val, float q, float meas_err)
{
    float k = ((z_est_err+q)/(meas_err + z_est_err+q));
    z_est_val = (1-k)*z_est_val + k*meas_val;
    z_est_err = (1-k)*z_est_err;
    return(z_est_val);}

float kal_vel(float meas_val, float q, float meas_err)
{
    float k = ((velz_est_err+q)/(meas_err + velz_est_err+q));
    velz_est_val = (1-k)*velz_est_val + k*meas_val;
    velz_est_err = (1-k)*velz_est_err;
    return(velz_est_val);}
    */

float kal(float meas_val, float& est_val, float& est_err , float q, float meas_err)
{
    float k = ((est_err+q)/(meas_err + est_err+q));
    est_val = (1-k)*est_val + k*meas_val;
    est_err = (1-k)*est_err;
    return(est_val);}

float alt_read(){
  int temp = myLidarLite.distance();
  //return kal(temp, 0.5, 10.0);
  return kal(temp, z_est_val, z_est_err, 0.5, 10.0);
}

float alt_vel_read(){
    float velz_raw = (estimatedAltitude - prevAltitude)*10 ; //scaled to decim/s
    prevAltitude = estimatedAltitude;
    //return kal_vel(velz_raw, 3.0, 10.0);
    //return kal(velz_raw, velz_est_val, velz_est_err, 0.5, 10.0); //was 3.0
    return velz_raw;
    }




