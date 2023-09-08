
#include <Arduino.h>
#include <EEPROM.h>
#include "electricui.h"

#include "Wire.h" // For I2C

#include <SPI.h>

const int chipSelect = 10;

// stroke sensor calibration settings
float x_Stroke_Sensor = 177;

float r_Stroke_Sensor_Cal_Slope_Saved ; //= EEPROM.get(10, r_Stroke_Sensor_Cal_Slope_Saved);
float n_Stroke_Sensor_Cal_Offset_Saved ; //= EEPROM.get(20, n_Stroke_Sensor_Cal_Offset_Saved);

float r_Stroke_Sensor_Cal_Slope = 44.7;//r_Stroke_Sensor_Cal_Slope_Saved;   // mm/V
float n_Stroke_Sensor_Cal_Offset = -79;//n_Stroke_Sensor_Cal_Offset_Saved; // mm

uint8_t zero_stroke = 0;

// Test time limits
long t_Start = 0;
unsigned long t_Current;
uint16_t t_Test = 0;
long t_Test_Max = 360000000;
int t_Test_Hours_Max = t_Test_Max / 3600000;

// pin & relay settings
int potPin1 = A3; // The linear pot

unsigned long t_Blink;
uint16_t t_Blink_Speed = 1000;
uint16_t t_Blink_Speed_Long = 3000;

float setpoint = 150; // in mm
unsigned long lastSetpointUpdateTime = 0;
int previousSetpoint = 0;
float position = 100; // in mm
float total_wheel_travel = 0.0;
float previous_position = 0.0;
uint16_t sensor_value = 2.5; //counts
float sensor_voltage; //V

// PID parameters
float Kp = 0.2;  // Proportional gain
float Ki = 0.00; // Integral gain
float Kd = 0.6; // Derivative gain

// Initial values
float error = 0.0;          // Error
float previous_error = 0.0; // Previous error
float integral = 0.0;       // Integral
float derivative = 0.0;     // Derivative

// Control signal limits
float max_control_signal = 1.0;
float min_control_signal = 0.0;

int8_t control_signal = 0.5;

// Relay pin
int relayPin = 4;

void serial_write(uint8_t *data, uint16_t len)
{
  Serial.write(data, len); // output on the main serial port
}

// Instantiate the communication interface's management object
eui_interface_t serial_comms = EUI_INTERFACE(&serial_write);

// Electric UI manages variables referenced in this array
eui_message_t tracked_variables[] =
    {
        EUI_FLOAT("set_point", setpoint),
        EUI_FLOAT("x_position", position),
        EUI_FLOAT("n_Kp", Kp),
        EUI_FLOAT("n_Ki", Ki),
        EUI_FLOAT("n_Kd", Kd),
        EUI_FLOAT("n_err", error),
        EUI_FLOAT("n_prev_err", previous_error),
        EUI_FLOAT("n_intl", integral),
        EUI_FLOAT("n_der", derivative),
        EUI_INT8("n_cont", control_signal),
        EUI_FLOAT("n_str_cal", n_Stroke_Sensor_Cal_Offset),
        EUI_FLOAT("r_str_cal", r_Stroke_Sensor_Cal_Slope),
        EUI_UINT8("zero_stroke", zero_stroke),
        EUI_FLOAT("x_str_sen", x_Stroke_Sensor),
        EUI_INT16("t_Test_ms", t_Test),
        EUI_INT16("n_Sensor", sensor_value),
        EUI_FLOAT("v_Sensor", sensor_voltage),

};

// VERSION CONTROL:

// String N_Software_Version = "v1.0"; //INITIAL ISSUE, based on Spring_Rig_v3.0
// String N_Firmware_Version = "v2.5"; // adding stroke error to improve control. adding front end IO
// String N_Firmware_Version = "v2.6"; // adding back in LCD and calibration terms, removed buttons (all to be managed in FE now), sending calibration terms to and fron FE
//String N_Firmware_Version = "2.7.5"; // removing LCD elements to match hardware
String N_Firmware_Version = "2.8.0"; // added function to force setpoint to -200 if setpoint is static for 1sec. (less banging around!), calculating total wheel travel, and passing it to t_Test for now. resets on power cycle for now too.

void serial_rx_handler()
{
  // While we have data, we will pass those bytes to the ElectricUI parser
  while (Serial.available() > 0)
  {
    eui_parse(Serial.read(), &serial_comms); // Ingest a byte
  }
}

void Calibration()

{
  delay(1000);

  digitalWrite(relayPin, HIGH); // extend

  float V_Stroke_Sensor_Cal_min = (analogRead(potPin1) / 1023.0) * 5.0; // V

  delay(5000);

  digitalWrite(relayPin, LOW); // retract

  float V_Stroke_Sensor_Cal_max = (analogRead(potPin1) / 1023.0) * 5.0; // V

  r_Stroke_Sensor_Cal_Slope = x_Stroke_Sensor / (V_Stroke_Sensor_Cal_max - V_Stroke_Sensor_Cal_min);

  n_Stroke_Sensor_Cal_Offset = r_Stroke_Sensor_Cal_Slope * (0 - V_Stroke_Sensor_Cal_min) - 0;

  r_Stroke_Sensor_Cal_Slope_Saved = r_Stroke_Sensor_Cal_Slope;
  n_Stroke_Sensor_Cal_Offset_Saved = n_Stroke_Sensor_Cal_Offset;

  EEPROM.put(10, r_Stroke_Sensor_Cal_Slope_Saved);
  EEPROM.put(20, n_Stroke_Sensor_Cal_Offset_Saved);

  zero_stroke = 0;
  delay(1000);

  digitalWrite(relayPin, HIGH); // extend
}


void setup()
{
  // initialize the relay pin as an output andpPot pin as input:
  pinMode(relayPin, OUTPUT);
  pinMode(potPin1, INPUT);

  Serial.begin(115200);
  Wire.begin();

  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite (LED_BUILTIN, HIGH);

  // Provide the library with the interface we just setup
  eui_setup_interface(&serial_comms);

  // Provide the tracked variables to the library
  EUI_TRACK(tracked_variables);

  // Provide a identifier to make this board easy to find in the UI - need to come back o this and amke this work - think somethign is missing
  eui_setup_identifier("BumpRig", 5);

}

void loop()
{

  serial_rx_handler(); // check for new inbound data

  if (zero_stroke == 1)
  {
    Calibration();
  }
  
    // Check if one second has elapsed since the last setpoint update
  if (millis() - lastSetpointUpdateTime >= 500) {
    // Force 'setpoint' to -200 since no updates occurred in the last second
    setpoint = -200;

    // Update the lastUpdateTime to the current time
    lastSetpointUpdateTime = millis();
  }

   // Check if the setpoint has changed
  if (setpoint != previousSetpoint) {
    // Reset the lastUpdateTime when the setpoint changes
    lastSetpointUpdateTime = millis();
    // Update the previousSetpoint with the current setpoint value
    previousSetpoint = setpoint;
  }

  // read the sensor value
  sensor_value = analogRead(3);

  // convert sensor value to mm
  sensor_voltage = (sensor_value / 1023.0) * 5.0; // V
  position = (sensor_voltage * r_Stroke_Sensor_Cal_Slope) + n_Stroke_Sensor_Cal_Offset;

  // calculate error
  error = setpoint - position;

  // calculate integral term
  integral += error;

  // calculate derivative term
  derivative = error - previous_error;

  // calculate control signal
  control_signal = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // limit control signal
  if (control_signal > max_control_signal)
  {
    control_signal = max_control_signal;
  }
  else if (control_signal < min_control_signal)
  {
    control_signal = min_control_signal;
  }

  // control the relay
  if (control_signal > 0.0)
  {
    digitalWrite(relayPin, LOW); // retract
  }
  else
  {
    digitalWrite(relayPin, HIGH); // extend
  }

  // update previous error
  previous_error = error;

   // Calculate the absolute change in position and add it to the total distance traveled
  float delta_position = abs(position - previous_position);
  total_wheel_travel += delta_position;

  t_Test = total_wheel_travel;

  // Update previous position and previous error
  previous_position = position;
  previous_error = error;

  // wait a short time before repeating the loop
  delay(5);
}
