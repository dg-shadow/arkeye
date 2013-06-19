/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

//#include <PID_v1.h>
#include <Arduino.h>
#include <ros.h>
#include <rosserial_arduino/Adc.h>



class ArkEyeServo
{
  private:
    //PID pid_;
    int last_time_;
    double setpoint_, input_, output_, last_input_, integral_;
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
  sensor_max_(sensor_max), sensor_min_(sensor_min), p_(p), i_(i), d_(d), going(false), input_(512),
  pwm_pin_(pwm_pin), direction_pin_(direction_pin), input_pin_(input_pin), direction_(0), output_(0)//, pid_(&input_, &output_, &setpoint_, p_, i_, d_, DIRECT)
{
  pinMode(pwm_pin_, OUTPUT);
  pinMode(direction_pin_, OUTPUT);
  
  pinMode(input_pin_, INPUT);

  read_input();
  last_input_ = input_;
  setpoint_ = input_;

//  pid_ = new PID(&input_, &output_, &setpoint_, p_, i_, d_, DIRECT);  
}

void ArkEyeServo::start()
{
  
  going = true;
}

ArkEyeServo::~ArkEyeServo()
{
}
 
void ArkEyeServo::set_setpoint(double setpoint)
{
  setpoint_ = setpoint;
}
void ArkEyeServo::compute()
{
  if (!going) return;
  int now = millis();
  if ((now - last_time_) < 100) return;
  
  double error = setpoint_ - input_;
  
//  if (error < 20 && error > -20) return;
  
  double imax = 50;
  
  integral_ = (integral_ + error * i_);
  if (integral_ > imax) integral_ = imax;
  else if (integral_ < -imax) integral_= -imax;
  
  double diff = input_ - last_input_;
//  if (diff < 0) diff *= -1;
  output_ = p_ * error  + integral_ - d_ * diff;
 
  if (output_ > 180.0) output_ = 180.0;
  else if (output_ < -180.0) output_ = -180.0;
  
  last_input_ = input_;
  last_time_ = millis();
}

void ArkEyeServo::do_output()
{ 
  if (!going) return;
  double error = (setpoint_ - input_);
  if (error < 0) error *= -1;
//if (error < 40) output_ = 0;

  
  if (output_ > 0) 
  {
    analogWrite(pwm_pin_, 0);
    direction_  = 1;
    digitalWrite(direction_pin_, 1);
    analogWrite(pwm_pin_, output_);
  }
  else
  {
    direction_  = 0;
    analogWrite(pwm_pin_, 0);
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

#define PITCH_PWM_PIN 3
#define PITCH_DIRECTION_PIN 2
#define PITCH_INPUT_PIN 0

#define LID_PWM_PIN 3
#define LID_DIRECTION_PIN 2
#define LID_INPUT_PIN 0


#define YAW_PWM_PIN 6
#define YAW_DIRECTION_PIN 7
#define YAW_INPUT_PIN 1

  
#define PITCH_SENSOR_MIN 50
#define PITCH_SENSOR_MAX 950

//working for steps
#define PITCH_P .71  
#define PITCH_I 0.01
#define PITCH_D .10


#define YAW_P .7  
#define YAW_I 0.01
#define YAW_D .081

#define LID_P .30
#define LID_I .006
#define LID_D 1.5



ArkEyeServo  pitch(PITCH_PWM_PIN, PITCH_DIRECTION_PIN, PITCH_INPUT_PIN, PITCH_SENSOR_MIN, PITCH_SENSOR_MAX, PITCH_P, PITCH_I, PITCH_D);;
ArkEyeServo  yaw(YAW_PWM_PIN, YAW_DIRECTION_PIN, YAW_INPUT_PIN, PITCH_SENSOR_MIN, PITCH_SENSOR_MAX, YAW_P, YAW_I, YAW_D);
ArkEyeServo  lid(LID_PWM_PIN, LID_DIRECTION_PIN, LID_INPUT_PIN, PITCH_SENSOR_MIN, PITCH_SENSOR_MAX, LID_P, LID_I, LID_D);



void setup()
{
  ros_node_handle.initNode();
  ros_node_handle.advertise(ros_publisher);


//  pitch.start();


//  yaw.start();
  lid.start();
  delay(2000);
}


void loop()
{
  static int time = millis(), dirn = 1, pause = 0;
  static double p_setpoint = 350, y_setpoint = 512, setpoint = 900;
  int now = millis();

  static int quadrant = 0;
  
    if (pause)
    {
       if (now - time > 1) 
       {
         pause --;
         time = millis();
       }
    }
    else if(1)    {
      if (now - time > 100)
     {
       if (dirn)
       {
         if (setpoint < 890) setpoint += 80;
         else
         {
           dirn = 0;
           pause = 200;
         }

       }
       else
       {
         if (setpoint > 150) setpoint -= 80;
         else 
         {
           dirn = 1;
           pause = 2000;
         }
       }
      time = millis();
   }
    
  }
/*
  yaw.set_setpoint(setpoint);
  yaw.read_input();
  yaw.compute(); 
  yaw.do_output();

  pitch.set_setpoint(setpoint);
  pitch.read_input();
  pitch.compute(); 
  pitch.do_output();
*/

//  pitch.set_setpoint(setpoint);

  lid.set_setpoint(setpoint);

  lid.read_input();
  lid.compute();
  lid.do_output();

  adc_msg.adc0 = lid.get_input();
  adc_msg.adc1 = lid.get_output() + 256;
  adc_msg.adc2 = lid.get_setpoint();
/*
  adc_msg.adc3 = yaw.get_input();
  adc_msg.adc4 = yaw.get_output() + 256;
  adc_msg.adc5 = yaw.get_setpoint();
*/
  ros_publisher.publish(&adc_msg);
  ros_node_handle.spinOnce();
}
