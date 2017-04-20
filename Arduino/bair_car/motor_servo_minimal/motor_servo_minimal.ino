/*
Code written for Arduino Uno.

The purpose is to read PWM signals coming out of a radio receiver
and either relay them unchanged to ESC/servo or substitute signals from a host system.

The steering servo and ESC(motor) are controlled with PWM signals. These meaning of these signals
may vary with time, and across devices. To get uniform behavior, the user calibrates with each session.
The host system deals only in percent control signals that are assumed to be based on calibrated PWM
signals. Thus, 0 should always mean 'extreme left', 49 should mean 'straight ahead', and 99 'extreme right'
in the percent signals, whereas absolute values of the PWM can vary for various reasons.

24 April 2016
*/


/////////////////////
// All possible states (i.e. first numbers sent in tuples along serial)
//
// These are the possible states of the control system.
// States are reached by button presses or drive commands, except for error state.
//#define STATE_HUMAN_FULL_CONTROL_DATA            1
//#define STATE_HUMAN_FULL_CONTROL_NOT_DATA                          2
//#define STATE_CAFFE_CAFFE_STEER_HUMAN_MOTOR 3
//#define STATE_CAFFE_HUMAN_STEER_HUMAN_MOTOR 5
//#define STATE_CAFFE_CAFFE_STEER_CAFFE_MOTOR 6
//#define STATE_CAFFE_HUMAN_STEER_CAFFE_MOTOR 7
//#define STATE_LOCK_CALIBRATE                4
//#define STATE_ERROR                         -1
// Now for the sensors
//#define STATE_GPS                           "'gps'"
//#define STATE_GYRO                          "'gyro'"
//#define STATE_SONAR                         "'sonar'"
//#define STATE_ENCODER                       "'encoder'"
//
/////////////////////


/////////////////////
// Servo constants
//
// These define extreme min an max values that should never be broken.
#define SERVO_MAX   4000
#define MOTOR_MAX   SERVO_MAX
#define BUTTON_MAX  SERVO_MAX
#define SERVO_MIN   500
#define MOTOR_MIN   SERVO_MIN
#define BUTTON_MIN  SERVO_MIN
//
/////////////////////


/////////////////////
// Changed these values 1 Feb. 2017 to avoid falling into state 4 from state 6
// The hand-held radio controller has two buttons. Pressing the upper or lower
// allows for reaching separate PWM levels: ~ 1710, 1200, 1000, and 888
// These are used for different control states.
#define BUTTON_A 1900 //1988 // Human in full control of driving 
#define BUTTON_B 1700 // Lock state
#define BUTTON_C 1424  // Caffe steering, human on accelerator
#define BUTTON_D 870  // Calibration of steering and motor control ranges
#define BUTTON_DELTA 50 // range around button value that is considered in that value
//
/////////////////////



//#include "constants.h"
#include "PinChangeInterrupt.h" // Adafruit library
#include <Servo.h> // Arduino library

// These come from the radio receiver via three black-red-white ribbons.
#define PIN_SERVO_IN 11
#define PIN_MOTOR_IN 10
#define PIN_BUTTON_IN 12

// These go out to ESC (motor controller) and steer servo via black-red-white ribbons.
#define PIN_SERVO_OUT 9
#define PIN_MOTOR_OUT 8

// On-board LED, used to signal error state
#define PIN_LED_OUT 13
//
/////////////////////


// Below are variables that hold ongoing signal data. I try to initalize them to
// to sensible values, but they will immediately be reset in the running program.
//
// These volatile variables are set by interrupt service routines
// tied to the servo and motor input pins.

// These are three key values indicating current incoming signals.
// These are set in interrupt service routines.
volatile int button_pwm = 1210;
volatile int servo_pwm = 0;
volatile int motor_pwm = 0;
volatile int servo_write_pwm = 0;
volatile int motor_write_pwm = 0;
int servo_command_pwm = 0;
int motor_command_pwm = 0;
int motor_null_pwm = 1500;
int servo_null_pwm = 1400;
// These are used to interpret interrupt signals.
volatile unsigned long int button_prev_interrupt_time = 0;
volatile unsigned long int servo_prev_interrupt_time  = 0;
volatile unsigned long int motor_prev_interrupt_time  = 0;
//volatile unsigned long int state_transition_time_ms = 0;

int max_communication_delay = 100;

long unsigned int servo_command_time;
long unsigned int motor_command_time;

Servo servo;
Servo motor; 



volatile float rate_1 = 0.0; // for encoder
//
///////////////////

////////////////////////////////////////
//
void setup()
{
  // Establishing serial communication with host system. The best values for these parameters
  // is an open question. At 9600 baud rate, data can be missed.
  Serial.begin(115200);
  Serial.setTimeout(5);

  // Setting up three input pins
  pinMode(PIN_BUTTON_IN, INPUT_PULLUP);
  pinMode(PIN_SERVO_IN, INPUT_PULLUP);
  pinMode(PIN_MOTOR_IN, INPUT_PULLUP);

  // LED out
  pinMode(PIN_LED_OUT, OUTPUT);

  // Attach interrupt service routines to pins. A change in signal triggers interrupts.
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_BUTTON_IN),
    button_interrupt_service_routine, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SERVO_IN),
    servo_interrupt_service_routine, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_MOTOR_IN),
    motor_interrupt_service_routine, CHANGE);

  // Attach output pins to ESC (motor) and steering servo.
  servo.attach(PIN_SERVO_OUT); 
  motor.attach(PIN_MOTOR_OUT); 

  while(Serial.available() > 0) {
    char t = Serial.read();
  }
  
  while(servo_pwm==0 || motor_pwm==0) {
    delay(200);
  }
  servo_null_pwm = servo_pwm;
  motor_null_pwm = motor_pwm;

  encoder_setup();
}







void button_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - button_prev_interrupt_time;
  button_prev_interrupt_time = m;
  // Human in full control of driving
  if (dt>BUTTON_MIN && dt<BUTTON_MAX) {
    button_pwm = dt;
  }
}







void servo_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - servo_prev_interrupt_time;
  servo_prev_interrupt_time = m;
  if (dt>SERVO_MIN && dt<SERVO_MAX) {
    servo_pwm = dt;
    if(servo_command_pwm>0) {
      servo_write_pwm = servo_command_pwm;
      servo.writeMicroseconds(servo_write_pwm);
    } else if(servo_null_pwm>0) {
      servo_write_pwm = servo_null_pwm;
      servo.writeMicroseconds(servo_write_pwm);
    }
  } 
}





void motor_interrupt_service_routine(void) {
  volatile unsigned long int m = micros();
  volatile unsigned long int dt = m - motor_prev_interrupt_time;
  motor_prev_interrupt_time = m;
  if (dt>MOTOR_MIN && dt<MOTOR_MAX) {
    motor_pwm = dt;
    if(motor_command_pwm>0) {
      motor_write_pwm = motor_command_pwm;
      motor.writeMicroseconds(motor_write_pwm);
    } else if(motor_null_pwm>0) {
      motor_write_pwm = motor_null_pwm;
      motor.writeMicroseconds(motor_write_pwm);
    }
  }
}




void loop() {
  
  unsigned int A = Serial.parseInt();
  unsigned int B = Serial.parseInt();

   long unsigned int now = millis();
   
  if (A>500 && A<3000) {
    servo_command_pwm = A;
    servo_command_time = now;
  } else if (A>10000 && A<30000) {
    motor_command_pwm = A-10000;
    motor_command_time = now;
  }
  if (B>500 && B<3000) {
    servo_command_pwm = B;
    servo_command_time = now;
  } else if (B>10000 && B<30000) {
    motor_command_pwm = B-10000;
    motor_command_time = now;
  }
  
  if(now-servo_command_time > max_communication_delay || now-motor_command_time > max_communication_delay) {
    servo_command_pwm = 0;
    motor_command_pwm = 0;
  }
  
  encoder_loop();

  Serial.print("('mse',");
  Serial.print(button_pwm);
  Serial.print(",");
  Serial.print(servo_pwm);
  Serial.print(",");
  Serial.print(motor_pwm);
  Serial.print(",");
  /*
  Serial.print(servo_write_pwm);
  Serial.print(",");
  Serial.print(motor_write_pwm);
  Serial.print(",");
  */
  Serial.print(rate_1);
  Serial.println(")");

  delay(10);
}














////////////// ENCODER //////////////////
//PIN's definition
#include "RunningAverage.h"
#define encoder0PinA  2
#define encoder0PinB  3

RunningAverage enc_avg(10);

volatile int encoder0Pos = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;
volatile unsigned long int a = 0;
volatile unsigned long int b = 0;
volatile unsigned long int t1 = micros();
volatile unsigned long int t2 = 0;
volatile unsigned long int last_t2 = 0;
volatile unsigned long int dt = 0;


void encoder_setup() 
{
  //Serial.begin(9600);
  pinMode(encoder0PinA, INPUT);
  //turn on pullup resistor
  //digitalWrite(encoder0PinA, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  pinMode(encoder0PinB, INPUT); 
  //turn on pullup resistor
  //digitalWrite(encoder0PinB, HIGH); //ONLY FOR SOME ENCODER(MAGNETIC)!!!! 
  PastA = (boolean)digitalRead(encoder0PinA); //initial value of channel A;
  PastB = (boolean)digitalRead(encoder0PinB); //and channel B

//To speed up even more, you may define manually the ISRs
// encoder A channel on interrupt 0 (arduino's pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
// encoder B channel pin on interrupt 1 (arduino's pin 3)
  attachInterrupt(1, doEncoderB, CHANGE); 

  enc_avg.clear();
}

volatile unsigned long int doEncoderAdtSum = 1;

void encoder_loop()
{  
  dt = micros()-t1;
  if (doEncoderAdtSum > 0) {
    //enc_avg.addValue(1000.0*1000.0/16.0 * a / doEncoderAdtSum);
    enc_avg.addValue(1000.0*1000.0/12.0 * a / doEncoderAdtSum); //6 magnets
    rate_1 = enc_avg.getAverage();
    t1 = micros();
    a = 0;
    doEncoderAdtSum = 0;
  } else if (dt > 100000) {
    enc_avg.clear();
    rate_1 = 0;
    t1 = micros();
    a = 0;
    doEncoderAdtSum = 0;
  }
}

//you may easily modify the code  get quadrature..
//..but be sure this whouldn't let Arduino back! 
volatile float doEncoderAdt = 0.;
void doEncoderA()
{
  t2 = micros();
  a = a + 1;
  doEncoderAdtSum += t2 - last_t2; 
  //doEncoderAdt = float(t2 - last_t2);
  //enc_avg.addValue(62500. / doEncoderAdt);
  //rate_1 = enc_avg.getAverage();
  last_t2 = t2;
}

void doEncoderB()
{
     b += 1;
}
//
///////////////////

