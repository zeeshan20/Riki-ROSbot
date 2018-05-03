#ifndef RIKI_BASE_CONFIG_H
#define RIKI_BASE_CONFIG_H

#define DEBUG 1
// #ifndef IMU_DEVICE_CONFIG
// //#define IMU_DEVICE_CONFIG IMU_MPU6050
// #define IMU_DEVICE_CONFIG   IMU_GY85
// #endif


float K_P = 0.1; // P constant
float K_I = 0.2; // I constant
float K_D = 0.2; // D constant

//define your motors' specs here

const int MAX_RPM = 366; //motor's maximum RPM
const int COUNTS_PER_REV = 1632; //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
const float WHEEL_DIAMETER = 0.064; //wheel's diameter in meters

#define BASE_WIDTH 0.186 // width of the plate you are using

//ENCODER PINS
// left side encoders pins
#define MOTOR1_ENCODER_A 29 //front_A			
#define MOTOR1_ENCODER_B 31 //front_B			

// right side encoders pins
#define MOTOR2_ENCODER_A 28 //front_A			
#define MOTOR2_ENCODER_B 26 //front_B			

//don't change this if you followed the schematic diagram
//MOTOR PINS
//left side motor pins
#define MOTOR1_PWM 32  	       
#define MOTOR1_IN_A 20
#define MOTOR1_IN_B 33

//right side motor pins
#define MOTOR2_PWM 25	     
#define MOTOR2_IN_A 30	
#define MOTOR2_IN_B 27		

#endif
