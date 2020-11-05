//Modified balancing robot code
//WIP
//22/10/2020

#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t GyY_offset;

const double Acc_rng = 32768/2;
const double Gyr_rng = 32768/250;
const double rad_to_deg = 360/6.2831853;
const int joy_X_PIN = A0;
const int joy_Y_PIN = A1;


unsigned long t_curr = 0;
unsigned long t_prev = 0;

float Theta_Y[] = {0.0, 0.0};
unsigned long t_delta[] = {1,2};

float target_angle = 1.25;
float stab_control = 0.0;
float advance_control = 0;
float diff_control = 0;
int mot = 0;


void setup(){

  //Start serial
  Serial.begin(9600);
  Serial.println("Serial port ready...");

  //Start I2C comms with MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("I2C ready...");


  //Set offsets
  GyY_offset = 100;

  //Read from the MPU once to set up initial conditions
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  Serial.println("MPU ready...");


  //Initialize the angles to the initial accel reading
  Theta_Y[0] = asin((double)AcX/Acc_rng) * rad_to_deg;
  Serial.print("Theta zero:");Serial.println(Theta_Y[0]);
  
  t_curr = millis();
  Serial.print("First millis:");Serial.println(t_curr);
  
  Serial.println("Theta and time initialized...");

}

void loop(){
  
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
//  Serial.println("MPU read");
  
  GyY -= GyY_offset;

//  Serial.print(t_curr);Serial.print(";;;");Serial.print(t_delta[0]);Serial.print(";");Serial.print(t_delta[1]);Serial.print(";");Serial.print(t_delta[2]);Serial.print(";");Serial.println(t_delta[3]);
  //shift time delta records
  for (int i = sizeof(t_delta)/sizeof(t_delta[0])-1; i > 0 ; i--){
    t_delta[i] = t_delta[i-1];
    }
    t_delta[0]=0;
//  Serial.println("Delta array shifted");
  
  t_delta[0] = millis() - t_curr;
  t_curr = millis();


  //shift theta records
  for (int i = sizeof(Theta_Y)/sizeof(Theta_Y[0])-1; i > 0 ; i--){
    Theta_Y[i] = Theta_Y[i-1];
    }
    Theta_Y[0]=0;
//  Serial.println("Theta array shifted");
  
  //correct the calculated angle with accelerometer data
  Theta_Y[0] = Theta_Y[1] + GyY * 0.001*t_delta[0] / Gyr_rng;
  if(abs(AcX)<10000){
    Theta_Y[0] = 0.99 * Theta_Y[0] + 0.01 * asin((double)AcX/Acc_rng) * rad_to_deg;

  }

//  advance_control = 0.02*map(analogRead(joy_X_PIN), 0, 1023, 1000, -1000);
//  diff_control = 0.1*map(analogRead(joy_Y_PIN), 0, 1023, 1000, -1000);
//  diff_control = controlCurve(diff_control, 2.0, 60.0, 0.0, 55, 0);
  
  //Call the PD control
  stab_control = PDControl(Theta_Y[0]-target_angle-advance_control, (Theta_Y[0]-Theta_Y[1])/(0.001*(float)t_delta[0]), 0.5, 0.01);
    
    
  //Call control curve function to scale the motor inputs
  mot = controlCurve(stab_control, 1.25, 45.0, 0.0, 200, 140);


  
  //Write to the motors
  //comment this out if working without proper wiring,
  //the driver is chink trash and can try to draw current through the arduino

  motorControl(-mot +diff_control, 3, 4, 5);    //motor inputs
  motorControl(mot -diff_control, 6, 7, 8);
  

  
}



void motorControl(int _w, int _enable, int _in1, int _in2){
  
  //Serial.print(AcX); Serial.print(","); Serial.print(mot); Serial.print(",");Serial.println(_w);
  
  
  if ( _w>=0 ) {
    digitalWrite(_in1,LOW);
    digitalWrite(_in2,HIGH);
  }
  else {
    digitalWrite(_in1,HIGH);
    digitalWrite(_in2,LOW);
  }
  analogWrite(_enable, abs(_w));
  
}

int controlCurve(float _input, float _deadzone, float _in_range_hi, float _in_range_lo,int _out_range_hi, int _out_range_lo){
  
  //Scale the  inputs to a response curve
  //The response curve is symmetrical, linear, and has a deadzone
    
  int result;
  
  if (abs(_input)<=_deadzone){      //deadzone//
    result = 0;
  }
  else if (_input > 0){
//    result=map(_input, _in_range_lo, _in_range_hi, _out_range_lo, _out_range_hi);
    result = ( (_input - _in_range_lo) / (_in_range_hi - _in_range_lo) ) * (_out_range_hi - _out_range_lo) + _out_range_lo;
  }
  else if (_input < 0){
//    result=map(_input, -_in_range_hi, -_in_range_lo, -_out_range_hi, -_out_range_lo);
    result = ( (_input + _in_range_lo) / (-_in_range_hi + _in_range_lo) ) * (-_out_range_hi + _out_range_lo) - _out_range_lo;
  }
  return result;
  
  }

float PDControl (float _error, float _error_derivative, float k_p, float k_d){

  //A simple PD control
  //Requires you to tell it everything

  float result;

  result = k_p * _error  + k_d * _error_derivative;

  return result;
  
  }
