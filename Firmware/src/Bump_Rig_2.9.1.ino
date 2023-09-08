
#include <Arduino.h>
#include <EEPROM.h>
#include "electricui.h"
#include "Wire.h" // For I2C
#include <SPI.h>

const int chipSelect = 10;

// stroke sensor calibration settings
float x_Stroke_Sensor = 150;
float V_Stroke_Sensor_Cal_max = 0;
float V_Stroke_Sensor_Cal_min = 0;
float r_Stroke_Sensor_Cal_Slope = 45; // r_Stroke_Sensor_Cal_Slope_Saved;   // mm/V
float n_Stroke_Sensor_Cal_Offset = 0; // n_Stroke_Sensor_Cal_Offset_Saved; // mm

uint8_t zero_stroke = 0;

// pin & relay settings
int potPin1 = A3; // The linear pot

float setpoint = 150; // in mm
unsigned long lastSetpointUpdateTime = 0;
int previousSetpoint = 0;
float position = 100; // in mm
float x_total_wheel_travel = 0;
float n_Laps = 0;
float previous_position = 0;
uint16_t sensor_value = 2.5; // counts
float sensor_voltage;        // V

// filtering

const unsigned long samplingInterval = 100; // Set the sampling interval in milliseconds (e.g., 1000ms or 1 second)
unsigned long previousSamplingTime = 0;     // Variable to store the previous sampling time

const unsigned long resetInterval = 1000; // Set the sampling interval in milliseconds (e.g., 1000ms or 1 second)
unsigned long previousResetTime = 0;      // Variable to store the previous sampling time

// ave stroke parameters

float totalCycleDistance = 0.0;
int cycleCount = 0;
float minSensorValue = position;
float maxSensorValue = minSensorValue;
unsigned long cycleStartTime = 0;
unsigned long cycleDuration = 500;
float averageCycleDistance = 0;

// PID parameters
float Kp = 0.2;  // Proportional gain
float Ki = 0.00; // Integral gain
float Kd = 0.6;  // Derivative gain

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
        EUI_FLOAT("x_str_sen", x_Stroke_Sensor),
        EUI_FLOAT("x_travel", x_total_wheel_travel),
        EUI_FLOAT("Laps", n_Laps),
        EUI_FLOAT("averageStroke", averageCycleDistance),

};

// VERSION CONTROL:

// String N_Software_Version = "v1.0"; //INITIAL ISSUE, based on Spring_Rig_v3.0
// String N_Firmware_Version = "v2.5"; // adding stroke error to improve control. adding front end IO
// String N_Firmware_Version = "v2.6"; // adding back in LCD and calibration terms, removed buttons (all to be managed in FE now), sending calibration terms to and fron FE
// String N_Firmware_Version = "2.7.5"; // removing LCD elements to match hardware
// String N_Firmware_Version = "2.8.0"; // added function to force setpoint to -200 if setpoint is static for 1sec. (less banging around!), calculating total wheel travel, and passing it to t_Test for now. resets on power cycle for now too.
String N_Firmware_Version = "2.9.0"; // added function for test distnce, laps and average travel per cycle

void serial_rx_handler()
{
  // While we have data, we will pass those bytes to the ElectricUI parser
  while (Serial.available() > 0)
  {
    eui_parse(Serial.read(), &serial_comms); // Ingest a byte
  }
}

void setup()
{
  // initialize the relay pin as an output andpPot pin as input:
  pinMode(relayPin, OUTPUT);
  pinMode(potPin1, INPUT);

  Serial.begin(115200);
  Wire.begin();

  cycleStartTime = millis(); // Initialize the cycle start time

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

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

  // Check if one second has elapsed since the last setpoint update
  if (millis() - lastSetpointUpdateTime >= 500)
  {
    // Force 'setpoint' to -200 since no updates occurred in the last second
    setpoint = -200;

    // Update the lastUpdateTime to the current time
    lastSetpointUpdateTime = millis();
  }

  // Check if the setpoint has changed
  if (setpoint != previousSetpoint)
  {
    // Reset the lastUpdateTime when the setpoint changes
    lastSetpointUpdateTime = millis();
    // Update the previousSetpoint with the current setpoint value
    previousSetpoint = setpoint;
  }

  // read the sensor value
  sensor_value = analogRead(3);

  // convert sensor value to mm
  sensor_voltage = (sensor_value / 1023.0) * 3.3; // V
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

  // filtering

  // Check if it's time to take a new measurement
  if (millis() - previousSamplingTime >= samplingInterval)
  {
    // Reset the sampling time
    previousSamplingTime = millis();

    // Calculate the absolute change in position and add it to the total distance traveled
    float x_delta_position = fabs(position - previous_position);
    x_total_wheel_travel += x_delta_position;
    n_Laps = x_total_wheel_travel / 150000;
    previous_position = position;

    Serial.println(x_total_wheel_travel);
    Serial.println(n_Laps);
    Serial.println(position);
    Serial.println(averageCycleDistance);
    Serial.println(minSensorValue);
    Serial.println(maxSensorValue);
  }

  // calculate average stroke

  if (position < minSensorValue)
  {
    minSensorValue = position;
  }

  if (position > maxSensorValue)
  {
    maxSensorValue = position;
  }

  // Check if it's time to reset
  if (millis() - previousResetTime >= resetInterval)
  {
    // Reset the sampling time
    previousResetTime = millis();

    // calculate average over last sec

    averageCycleDistance = maxSensorValue - minSensorValue;

    // Reset minSensorValue and maxSensorValue for the next cycle
    minSensorValue = position;
    maxSensorValue = position;
  }

  // Update previous position and previous error

  previous_error = error;

  // wait a short time before repeating the loop
  delay(5);
}
