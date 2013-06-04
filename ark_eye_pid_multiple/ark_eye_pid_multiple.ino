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
    PID *pid_fwd_, *pid_rev_;
    double setpoint_, input_, output_fwd_, output_rev_;
    double p_, i_, d_;
    double sensor_max_, sensor_min_;
    const int fwd_pwm_pin_, rev_pwm_pin_, hbridge_enable_pin_, input_pin_;
    boolean going;
    void read_average_input();
  public:
    ArkEyeServo(int fwd_pwm_pin, int rev_pwm_pin, int h_bridge_enable_pin, int input_pin_, double sensor_min, double sensor_max, double p, double i, double d);
    void set_setpoint(double setpoint);
    void compute();
    void read_input();
   ~ArkEyeServo();
   double get_output_fwd();
   double get_input();
   double get_output_rev();
   void do_output();
   void start();
   void stop();
};

ArkEyeServo::ArkEyeServo(int fwd_pwm_pin, int rev_pwm_pin, int hbridge_enable_pin, int input_pin, double sensor_min, double sensor_max, double p, double i, double d) : 
  sensor_max_(sensor_max), sensor_min_(sensor_min), p_(p), i_(i), d_(d), going(false), setpoint_(512), input_(512),
  fwd_pwm_pin_(fwd_pwm_pin), rev_pwm_pin_(rev_pwm_pin), hbridge_enable_pin_(hbridge_enable_pin), input_pin_(input_pin)
{
  digitalWrite(hbridge_enable_pin_, 0);
  pinMode( hbridge_enable_pin_, OUTPUT);
  digitalWrite(hbridge_enable_pin_, 0);

  pinMode(fwd_pwm_pin_, OUTPUT);
  analogWrite(fwd_pwm_pin_, 0);

  pinMode(rev_pwm_pin_, OUTPUT);
  analogWrite(rev_pwm_pin_, 0);
  
  pinMode(input_pin_, INPUT);

  read_input();

  pid_fwd_ = new PID(&input_, &output_fwd_, &setpoint_, p_, i_, d_, DIRECT);
  pid_rev_ = new PID(&setpoint_, &output_rev_, &input_, p_, i_, d_, DIRECT);
  
}

void ArkEyeServo::start()
{
  pid_fwd_->SetMode(AUTOMATIC);
  pid_rev_->SetMode(AUTOMATIC);
  
  digitalWrite(hbridge_enable_pin_, 1);
  
  going = true;
}

ArkEyeServo::~ArkEyeServo()
{
  delete pid_fwd_;
  delete pid_rev_;
}

void ArkEyeServo::set_setpoint(double setpoint)
{
  setpoint_ = setpoint;
}
void ArkEyeServo::compute()
{
  if (!going) return;
  
  pid_fwd_->Compute();
  pid_rev_->Compute();
}

void ArkEyeServo::do_output()
{// might need to switch precedence of output depending on direction of movement.
  
  if (!going) return;
  
  if (output_fwd_)
  {
    analogWrite(rev_pwm_pin_, 0);
    analogWrite(fwd_pwm_pin_, output_fwd_);
  }
  else if (output_rev_)
  {
    analogWrite(fwd_pwm_pin_, 0);
    analogWrite(rev_pwm_pin_, output_rev_);
  }
  else
  {
    analogWrite(fwd_pwm_pin_, 0);
    analogWrite(rev_pwm_pin_, 0);
  }
}

void ArkEyeServo::read_input()
{
  input_ = map(analogRead(input_pin_), sensor_min_, sensor_max_, 0, 1024);
}

double ArkEyeServo::get_input()
{
  return input_;
}

double ArkEyeServo::get_output_fwd()
{
  return output_fwd_;
}

double ArkEyeServo::get_output_rev()
{
  return output_rev_;
}




ArkEyeServo * pitch;

ros::NodeHandle ros_node_handle;

rosserial_arduino::Adc adc_msg;
ros::Publisher ros_publisher("adc", &adc_msg);

#define PITCH_FWD_PWM_PIN 3
#define PITCH_REV_PWM_PIN 5
#define PITCH_HBRIDGE_PIN 4
#define PITCH_INPUT_PIN 0

#define PITCH_SENSOR_MIN 10
#define PITCH_SENSOR_MAX 1000

#define PITCH_P 0.4
#define PITCH_I 0.2
#define PITCH_D 0.1

void setup()
{
  ros_node_handle.initNode();
  ros_node_handle.advertise(p);


  pitch = new ArkEyeServo(PITCH_FWD_PWM_PIN, PITCH_REV_PWM_PIN, PITCH_HBRIDGE_PIN, PITCH_INPUT_PIN, PITCH_SENSOR_MIN, PITCH_SENSOR_MAX, PITCH_P, PITCH_I, PITCH_D);
  pitch->start();

}


void loop()
{

  pitch->read_input();
  pitch->compute();
  pitch->do_output();
  
  adc_msg.adc1 = pitch->get_input();
  adc_msg.adc2 = pitch->get_output_fwd();
  adc_msg.adc3 = pitch->get_output_rev();
  
  ros_publisher.publish(&adc_msg);
  ros_node_handle.spinOnce();
}
