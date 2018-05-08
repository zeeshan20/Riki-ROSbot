#include <ArduinoHardware.h>



#include <Arduino.h>

#include <ros.h>
//header file for publishing "rpm"
#include <geometry_msgs/Vector3Stamped.h>
//header file for cmd_subscribing to "cmd_vel"
#include <geometry_msgs/Twist.h>
//header file for pid server
#include <riki_msgs/PID.h>
#include <riki_msgs/Velocities.h>
//header files for imu
#include <ros_arduino_msgs/RawImu.h>
//#include <geometry_msgs/Vector3.h>
#include <ros/time.h>

#include <Wire.h>

#include "config.h"
#include "Encoder.h"
#include "Motor.h"

#include "imu_configuration.h"

#define ENCODER_OPTIMIZE_INTERRUPTS

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 10 //hz
#define DEBUG_RATE 5

//left side motors
Motor motor1(MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B); //front

//right side motors
Motor motor2(MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); // front

//left side encoders
Encoder motor1_encoder(MOTOR1_ENCODER_A,MOTOR1_ENCODER_B); //front

//right side encoders
Encoder motor2_encoder(MOTOR2_ENCODER_A,MOTOR2_ENCODER_B); //front

float Motor::Kp = K_P;
float Motor::Kd = K_D;
float Motor::Ki = K_I;

int Motor::max_rpm = MAX_RPM;
int Motor::counts_per_rev = COUNTS_PER_REV;
float Motor::wheel_diameter = WHEEL_DIAMETER;

double required_angular_vel = 0;
double required_linear_vel = 0;
unsigned long previous_command_time = 0;
unsigned long previous_control_time = 0;
unsigned long publish_vel_time = 0;
unsigned long previous_imu_time = 0;
unsigned long previous_debug_time = 0;

bool is_first = true;

char buffer[50];

//callback function prototypes
void command_callback( const geometry_msgs::Twist& cmd_msg);
void pid_callback( const riki_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);

ros_arduino_msgs::RawImu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

riki_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup()
{
  nh.initNode();
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);
  while (!nh.connected()){
    nh.spinOnce();
  }
  nh.loginfo("RIKIBASE CONNECTED");

  Wire.begin();
  delay(5);
}

void loop(){
  //this block drives the robot based on defined rate
  if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
    do_kinematics();
    move_base();
    previous_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - previous_command_time) >= 400){
    stop_base();
  }

  //this block publishes velocity based on defined rate
  if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE)){
    publish_linear_velocity();
    publish_vel_time = millis();
  }

  //this block publishes the IMU data based on defined rate
  if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
    //sanity check if the IMU exits
    if (is_first){
      check_imu();
    } else{
      //publish the IMU data
      publish_imu();
    }
    previous_imu_time = millis();
  }

  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
  if(DEBUG){
    if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)) {
      print_debug();
      previous_debug_time = millis();
    }
  }
  //call all the callbacks waiting to be called
  nh.spinOnce();
}

void pid_callback( const riki_msgs::PID& pid)
{
  //callback function every time PID constants are received from lino_pid for tuning
  //this callback receives pid object where P,I, and D constants are stored
  Motor::Kp = pid.p;
  Motor::Kd = pid.d;
  Motor::Ki = pid.i;
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  required_linear_vel = cmd_msg.linear.x;
  required_angular_vel = cmd_msg.angular.z;

  previous_command_time = millis();
}

void do_kinematics()
{
  //convert m/s to m/min
  double linear_vel_mins = required_linear_vel * 60;
  //convert rad/s to rad/min
  double angular_vel_mins = required_angular_vel * 60;
  //calculate the wheel's circumference
  double circumference = PI * WHEEL_DIAMETER;
  //calculate the tangential velocity of the wheel if the robot's rotating where Vt = ω * radius
  double tangential_vel = angular_vel_mins * (BASE_WIDTH)/2; //correction made only base width to be divided by 2. 03-may-18

  //calculate and assign desired RPM for each motor
  //left side
  motor1.required_rpm = (linear_vel_mins / circumference) - (tangential_vel / circumference);
  //right side
  motor2.required_rpm = (linear_vel_mins / circumference) + (tangential_vel / circumference);
}

void move_base()
{
  //calculate each motor's rpm for pwm calculation and odometry
  motor1.calculate_rpm(motor1_encoder.read());
  motor2.calculate_rpm(motor2_encoder.read());

  motor1.spin(motor1.calculate_pwm());
  motor2.spin(motor2.calculate_pwm());
}

void stop_base()
{
  required_linear_vel = 0;
  required_angular_vel = 0;
}

void publish_linear_velocity()
{
  // this function publishes the linear speed of the robot

  //calculate the average RPM
  double average_rpm = (motor1.current_rpm + motor2.current_rpm) / 2; // RPM
  //convert revolutions per minute to revolutions per second
  double average_rps = average_rpm / 60; // RPS
  //calculate linear speed
  double linear_velocity = (average_rps * (WHEEL_DIAMETER * PI)); // m/s

  //fill in the object
  raw_vel_msg.linear_x = linear_velocity;
  raw_vel_msg.linear_y = 0.00;
  raw_vel_msg.angular_z = 0.00;

  //publish raw_vel_msg object to ROS
  raw_vel_pub.publish(&raw_vel_msg);
}

void check_imu()
{

    //this function checks if IMU is present
  raw_imu_msg.accelerometer = check_accelerometer();
  raw_imu_msg.gyroscope = check_gyroscope();
  raw_imu_msg.magnetometer = check_magnetometer();

  if (!raw_imu_msg.accelerometer){
    nh.logerror("Accelerometer NOT FOUND!");
  }

  if (!raw_imu_msg.gyroscope){
    nh.logerror("Gyroscope NOT FOUND!");
  }

  if (!raw_imu_msg.magnetometer){
    nh.logerror("Magnetometer NOT FOUND!");
  }


  is_first = false;
}

void publish_imu()
{
  //this function publishes raw IMU reading
  raw_imu_msg.header.stamp = nh.now();
  raw_imu_msg.header.frame_id = "imu_link";
    //measure accelerometer
    if (raw_imu_msg.accelerometer){
      measure_acceleration();
      raw_imu_msg.raw_linear_acceleration = raw_acceleration;
    }

    //measure gyroscope
    if (raw_imu_msg.gyroscope){
      measure_gyroscope();
      raw_imu_msg.raw_angular_velocity = raw_rotation;
    }

    //measure magnetometer
    if (raw_imu_msg.magnetometer){
      measure_magnetometer();
      raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
    }

  //publish raw_imu_msg object to ROS
  raw_imu_pub.publish(&raw_imu_msg);
}

void print_debug()
{
  sprintf (buffer, "Encoder Left: %ld", motor1_encoder.read());
  nh.loginfo(buffer);
  sprintf (buffer, "Encoder Right: %ld", motor2_encoder.read());
  nh.loginfo(buffer);
}
