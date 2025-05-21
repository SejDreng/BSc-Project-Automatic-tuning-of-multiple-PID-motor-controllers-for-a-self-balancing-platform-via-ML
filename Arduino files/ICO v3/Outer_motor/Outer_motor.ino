#include <Wire.h>   
#include <Adafruit_NeoPixel.h>
#include <PID_v1.h>
#define PIN 2
#define NUMPIXELS 3
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);                         

struct controlPacket{
  float typeID;
  float obsAngle;
  float reference;
  float kp;
  float ki;
  float kd;
  float error;
  float integral;
};

struct PIDsignalpacket{
  uint8_t typeID;
  float u;
};

controlPacket recPacket = {1, 0.0, 0.0, 0.0, 0.0, 0.0};  
PIDsignalpacket inputReference = {2, 0.0}; 
float rollFloat;     

double setpoint = 0;    
double input = 0;       
double output = 0;   
double error = 0;

double integral = 0;
double clamper_value = 0;
double prev_error = 0;
double prev_U = 0;

// INITIAL PID Gains (Is changed in the loop)
double Kp = 1, Ki = 0, Kd = 0;

int shiftDirection=0;

// Match with the master config
const float dt_calc = 0.03f;     // Time step for calculations (seconds)
const long loop_delay_ms = 30; // Run PID calculation every 100ms


void setup() 
{
  pinMode(11, OUTPUT);          // EN (IN1) - Enable motor
  pinMode(12, OUTPUT);          // PH (IN2) - Control direction       
  Wire.begin(1);                // Match wire address with according position as in master                            
  Wire.onReceive(recievePacket);    
  Serial.begin(115200);                  
}

void loop() 
{

  float derivative = (error-prev_error)/dt_calc;

  output = (Kp*error+Ki*integral+Kd*derivative);

  float max_change_per_cycle = 30.5f; // Tune this value
  float change = output - prev_U;
  if (change > max_change_per_cycle) {
    output = prev_U + max_change_per_cycle;
  } else if (change < -max_change_per_cycle) {
    output = prev_U - max_change_per_cycle;
  }
  prev_U = output; // Store for next iteration

  const float U_MAX = 100;
  if (output > U_MAX) {
    output = U_MAX;
  } else if (output< -U_MAX) {
    output = -U_MAX;
  }


  // Clamping is already done on the master, this is just added redundancy for the specific controller
  bool is_saturated = fabs(output) >= U_MAX;
  bool signs_match = (error * output > 0); 
                                            
  bool is_winding_up = signs_match; 
  bool is_error_growing = fabs(error) > fabs(prev_error);
  
  clamper_value = 1.0f;
  if ((is_saturated && is_winding_up && is_error_growing)) {
      clamper_value = 0.0f;
  }

  prev_error = error;
  applyMotorOutput(output);
  delay(loop_delay_ms);
}

void recievePacket(int pack)
{
  if (pack < 1) return; // Ensure at least 1 byte is received

  if (pack < sizeof(recPacket)) {
    while(Wire.available()){
      Wire.read();
    }  // Custom flush function to clear the remaining bytes
    return;
  }

  Wire.readBytes((byte*)&recPacket, sizeof(recPacket));
  // Now update PID parameters
  Kp = (double)recPacket.kp;
  Ki = (double)recPacket.ki;
  Kd = (double)recPacket.kd;
  setpoint = (double)recPacket.reference;
  input = (double)recPacket.obsAngle;
  error = (double)recPacket.error;
  integral = (double)recPacket.integral;

  // Discard any extra bytes in the buffer
  while (Wire.available()) {
    Wire.read(); // Discard remaining bytes
  }

  return;
}



void applyMotorOutput(double pidOutput) {
  
  int pwmValue = abs(pidOutput);

  if (pidOutput > 0) { 
    shiftDirection=1;
    digitalWrite(12, HIGH);  // Forward
    analogWrite(11, 255-pwmValue);  // Apply PWM to motor
  } else {
    shiftDirection=0;
    digitalWrite(12, LOW); // Reverse    
    analogWrite(11, pwmValue);  // Apply PWM to motor
  }
}


