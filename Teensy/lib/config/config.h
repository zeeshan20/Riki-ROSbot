#ifndef RIKI_BASE_CONFIG_H
#define RIKI_BASE_CONFIG_H

#define DEBUG 1
// #ifndef IMU_DEVICE_CONFIG
// #define IMU_DEVICE_CONFIG IMU_MPU6050
// #define IMU_DEVICE_CONFIG   IMU_GY85
// #endif


//*****  DEFINE MOTOR AND ENCODER PINS *****//
//*****  DEFINE BASE WIDTH AND ENCODER DATA *****//

float K_P = 0.1; 							// P constant
float K_I = 0.2; 							// I constant
float K_D = 0.2; 							// D constant

//***** Definition of Motor Specifications *****//

const int MAX_RPM = 366; 					//Motor's Maximum RPM
const int COUNTS_PER_REV = 1632; 			//Encoder Ticks per Revolution (Gear ratio * PPR * 4)
const float WHEEL_DIAMETER = 0.064; 		//wheel's diameter in meters

#define BASE_WIDTH 0.160 					// Base_width: Distance between two wheels centres

//***** ENCODER PINS *****//

// Left Wheel Encoders Pins
#define MOTOR1_ENCODER_A 29 			
#define MOTOR1_ENCODER_B 31 			

// Right Wheel Encoders Pins
#define MOTOR2_ENCODER_A 28 		
#define MOTOR2_ENCODER_B 26 		


//***** MOTOR PINS *****//

//Left Wheel Motor Pins
#define MOTOR1_PWM 32  	       				// PWM pin
#define MOTOR1_IN_A 20						// Motor +ve
#define MOTOR1_IN_B 33						// Motor -ve

//Right Wheel Motor Pins
#define MOTOR2_PWM 25	     				// PWM pin
#define MOTOR2_IN_A 30						// Motor +ve
#define MOTOR2_IN_B 27						// Motor -ve

#endif
