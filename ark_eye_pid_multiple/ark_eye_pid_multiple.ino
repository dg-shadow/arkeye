/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <Arduino.h>
#include <ros.h>
#include <rosserial_arduino/Adc.h>



class ArkEyeServo
{
  private:
    PID *pid_;
    double setpoint_, input_, output_;
    double p_, i_, d_;
    double sensor_max_, sensor_min_;
    const int pwm_pin_, direction_pin_, input_pin_;
    boolean going;
    void read_average_input();
    int direction_;
  public:
    ArkEyeServo(int pwm_pin, int direction_pin, int input_pin, double sensor_min, double sensor_max, double p, double i, double d);
    void set_setpoint(double setpoint);
    void compute();
    void read_input();
   ~ArkEyeServo();
   double get_output();
   double get_input();
   double get_setpoint();
   void do_output();
   void start();
   void stop();
   int get_direction();
};

int ArkEyeServo::get_direction(void)
{
  return direction_;
}
ArkEyeServo::ArkEyeServo(int pwm_pin, int direction_pin, int input_pin, double sensor_min, double sensor_max, double p, double i, double d) : 
  sensor_max_(sensor_max), sensor_min_(sensor_min), p_(p), i_(i), d_(d), going(false), setpoint_(512), input_(512),
  pwm_pin_(pwm_pin), direction_pin_(direction_pin), input_pin_(input_pin), direction_(0), output_(0)
{
  pinMode(pwm_pin_, OUTPUT);
  pinMode(direction_pin_, OUTPUT);
  
  pinMode(input_pin_, INPUT);

  read_input();

  pid_ = new PID(&input_, &output_, &setpoint_, p_, i_, d_, DIRECT);  
}

void ArkEyeServo::start()
{
  pid_->SetMode(AUTOMATIC);
  pid_->SetOutputLimits(-254.0,254.0);
  
  going = true;
}

ArkEyeServo::~ArkEyeServo()
{
  delete pid_;
}
 
void ArkEyeServo::set_setpoint(double setpoint)
{
  setpoint_ = setpoint;
}
void ArkEyeServo::compute()
{
  if (!going) return;
  
  pid_->Compute();
}

void ArkEyeServo::do_output()
{ 
  if (!going) return;
  double error = (setpoint_ - input_);
  if (error < 0) error *= -1;
  if (error < 10) output_ = 0;

  
  if (output_ > 0) 
  {
    direction_  = 1;
    digitalWrite(direction_pin_, 1);
    analogWrite(pwm_pin_, output_);
  }
  else
  {
    direction_  = 0;
    digitalWrite(direction_pin_, 0);
    analogWrite(pwm_pin_, (-1*output_) );
  }
}

void ArkEyeServo::read_input()
{
  double  old_input = input_;
 
  input_ = 0.0;
  for (int x = 0; x < 4; x++) input_ += analogRead(input_pin_);
  input_ = input_/4.0;
 
 double  difference = input_ - old_input;
// if (difference > 50 || difference < -50) going = 0;

//  input_ = map(input_/4,sensor_min_, sensor_max_, 0, 1024);
  if (input_ < 0.0) input_ = 0.0;
}

double ArkEyeServo::get_input()
{
  return input_;
}

double ArkEyeServo::get_output()
{
  return output_;
}

double ArkEyeServo::get_setpoint()
{
  return setpoint_;
}


ros::NodeHandle ros_node_handle;

rosserial_arduino::Adc adc_msg;
ros::Publisher ros_publisher("adc", &adc_msg);

#define PITCH_PWM_PIN 5
#define PITCH_DIRECTION_PIN 4
#define PITCH_INPUT_PIN 0


#define YAW_PWM_PIN 3
#define YAW_DIRECTION_PIN 2
#define YAW_INPUT_PIN 1


#define PITCH_SENSOR_MIN 50
#define PITCH_SENSOR_MAX 950

//working for steps
#define PITCH_P .400  
#define PITCH_I .1
#define PITCH_D .11


#define YAW_P .400  
#define YAW_I .1
#define YAW_D .11



ArkEyeServo  pitch(PITCH_PWM_PIN, PITCH_DIRECTION_PIN, PITCH_INPUT_PIN, PITCH_SENSOR_MIN, PITCH_SENSOR_MAX, PITCH_P, PITCH_I, PITCH_D);;
ArkEyeServo  yaw(YAW_PWM_PIN, YAW_DIRECTION_PIN, YAW_INPUT_PIN, PITCH_SENSOR_MIN, PITCH_SENSOR_MAX, PITCH_P, PITCH_I, PITCH_D);

void setup()
{
  ros_node_handle.initNode();
  ros_node_handle.advertise(ros_publisher);


  pitch.start();
  yaw.start();
  delay(5000);
}


void loop()
{
  static int time = millis(), dirn = 1;
  static double setpoint = 512;
  int now = millis();

if (0) {
    if (now - time > 5000)
   {
      if (dirn)
      {
        setpoint = 850;
        dirn = 0;
      }
      else
      {
        setpoint = 100;
        dirn = 1;
      }
      
      time = millis();
   }
 }
  else if (1) {
    if (now - time > 100)
   {
      if (dirn)
      {
        if (setpoint < 950) setpoint = setpoint + 40;
        else dirn = 0;
      }
      else
      {
        if (setpoint > 150) setpoint = setpoint - 40;
        else dirn = 1;
      }
      time = millis();
   }
    
  }
  
  yaw.set_setpoint(setpoint);
  pitch.set_setpoint(setpoint);
  
  
  yaw.read_input();
  yaw.compute();
  
  pitch.read_input();
  pitch.compute();

  yaw.do_output();
  pitch.do_output();

  
  adc_msg.adc0 = pitch.get_input();
  adc_msg.adc1 = pitch.get_output() + 256;
  adc_msg.adc2 = pitch.get_setpoint();
  ros_publisher.publish(&adc_msg);
  ros_node_handle.spinOnce();
}
