 
/*    humidifier_minsa
 
 An un-reviewed and experimental firmware for a medical humidifier
 specially designed and built to cover for the low supply caused by
 the COVID-19 pandemic. This is intended to be constantly monitored 
 and swiftly disposed after the pandemic ends. 
 DO NOT USE THIS FIRWMARE ON CRITICAL MEDICAL DEVICES.

 Copyright (C) 2020 Octavio Echeverria, Jahir Argote, Nicolas Tertusio

  
 GNU GPLv3 license
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.



 Contact: octavio280995@gmail.com

*/

#include <Arduino.h>
#include <SimpleDHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <ClickButton.h>
#include <math.h>
#include <gasboard7500E.h>
#include <DallasTemperature.h>
#include <OneWire.h> 

#ifdef O2SENSE_NEED_METADATA
bool has_sernum = false;
bool has_vernum = false;
#endif

HardwareSerial Serial3(PB11, PB10);

// Physical properties
#define B_VALUE                 3950
#define HUMIDITY_TEMP           95

// Pin definitions
#define DHTTYPE                 DHT22
#define DHTPIN                  PA4
#define WIND_SPEED_PIN          PA3
#define WIND_THERM_PIN          PA2
#define PIN_THERMISTOR          PA1
#define PLATE_RELAY_PIN         PA8
#define FAN_PIN                 PA10
#define HOSE_PIN                PA9
#define PIN_A                   PA7 
#define PIN_B                   PA6
#define BUTTON                  PA5
#define BUZZER_PIN              PA0
#define ONE_WIRE_BUS            PB8

// Behaviour characteristics
#define HOSE_CONVERSION_FACTOR  .01 // Converts pwm output to rms volts in output.
#define PLATE_HISTERESIS        5 // Plate deadzone size.
#define MIN_DELTA_T             2
#define LCD_ROWS         2     // Quantity of lcd rows.
#define LCD_COLUMNS      16    // Quantity of lcd columns.
#define LCD_SPACE_SYMBOL 0x20  //Space symbol from lcd ROM, see p.9 of GDM2004D datasheet.
// #define DEBUG

// Timming
#define FLOW_UPDATE_DELAY       10
#define TERMISTOR_UPDATE_DELAY  10
#define DHT_UPDATE_DELAY        2005
#define EXIT_HUMIDITY_ESTIMATION_DELAY   100
#define BANGBANG_CONTROL_DELAY  100
#define PID_FAN_CONTROL_DELAY   1000
#define EXECUTE_DELAY           10
#define ENCODER_UPDATE_DELAY    10
#define SCREEN_UPDATE_DELAY     500
#define BEEP_UPDATE_DELAY       10
#define BEEP_ONCE_DURATION      300
#define ALARM_UPDATE_DELAY      10
#define O2_UPDATE_DELAY         0
#define DS_UPDATE_DELAY         1000
#define HOSE_BB_UPDATE_DELAY    100
#define GET_VTEMP_DELAY         8000
#define PRINT_02_DELAY          4000

//PID Values
#define KP_PD_HUM               3.5
#define KD_PD_HUM               0
#define KP_PD_TEMP              8.5
#define KP_PD_TEMP_MOD          3
#define PERIODO                 2000
#define KP_FAN                  0.5
#define KD_FAN                  0
#define KI_FAN                  0
#define KP_SMC                  0.01
#define MAX_PWM_dot             500

//Cursor Locations
#define POSIBLE_POSITIONS       5
#define POSX1                   3
#define POSY1                   0
#define POSX2                   12
#define POSY2                   0
#define POSX3                   3
#define POSY3                   1
#define POSX4                   10
#define POSY4                   1
#define POSX5                   10
#define POSY5                   1

//Alarm critical values
#define MAX_PLATE_TEMP          150
#define DELTA_V                 91//101: mapped mode; 26: L/min mode


// Global Variables
uint32_t next_flow_update, next_termistor_update, next_dht_update, next_flow_estimation, next_bangbang_control, next_pidfan_control, next_pid_update, next_execute, next_encoder_update, next_screen_update, next_beep_update, next_alarm_update, next_o2_update, next_ds_update, next_hose_bb_update, get_vtemp_update;

// Structs

enum BeepType
{
  NO_BEEP,
  BEEP_ONCE,
  BEEP_TWICE = 3,
  BEEP_THRICE = 5,
  BEEP_CONTINUOS = 7
};

enum CursorPositions
{
  TOP_LEFT,
  TOP_RIGHT,
  DOWN_LEFT,
  DOWN_MIDDLE,
  DOWN_RIGHT
};

struct Beeps
{
  BeepType beep_type = NO_BEEP;
  uint16_t beep_priority = 0;
  uint16_t beep_id = 0;
  uint8_t beep_clock = 0;
};

struct StateVals
{
  float plate_temp = 0; // Current plat temperature. °C
  float vapor_temp = 0; // Current temperature at vapor chamber. °C 
  float after_hose_temp = 0;
  float est_temp = 0; // Estimated temperature after hose. °C
  float est_humidity = 0; // Estimated humidity after hose. RH%
  float vapor_abs_humidity = 0; // Current absolute humidity at vapor chamber. g/m3
  float est_abs_humidity = 0; // Estimated absolute humidity after hose. g/m3
  float current_airflow = 0; // Air volumetric flow rate. Lts/min
  float current_airspeed = 0; // Air speed. m/s
  uint16_t fan_pwm = 0; // Fan PWM duty cycle.
  float vapor_humidity = 0; // Current humidity at vapor chamber. RH%
  uint16_t hose_pwm = 0; // Hose power PWM duty cycle.
  float target_temp = 0; // Target temperature after hose. °C
  float target_humidity = 0; // Target relative humidity after hose. %RH
  float target_airflow = 0; // Target air flow. L/min
  float target_fio2 = 0;
  uint16_t initial_target_pwm = 0; //Target PWM for flow control
  bool pwr_state = 0; // Machine on or off.next_dht_update
  bool plate_relay_state = 0; // Plate on or off.
  bool plate_relay_cmd = 0;
  uint16_t o2_concentration = 0;
  uint16_t o2_flow = 0;
  uint16_t o2_temp = 0;
  uint16_t o2_test = 0;
  uint32_t clock = 0;
  char o2_buffer[64];
  Beeps current_beep;
  uint16_t adc_therm = 0;
  float them_resistance = 0;
  float duty_cycle = 0;
  float set_PWM = 0;
  float set_o2_flow = 0;
  bool hose_state = 0;
  float vapor_target_temp = 0;
  uint16_t adc_flow_t = 0;
  uint16_t adc_flow_v = 0;
  
  //Encoder Info Handlers
  uint32_t button_counter = 0; //Controls the cursor position
  bool is_main_menu = 1; //Controls Main menu state
  bool is_config_mode = 0; //Controls configuration mode state
  bool is_debug_mode = 0; //Controls debug mode
  //Alarm Flags
  bool is_alarm = 0;
  bool is_vapor_too_hot = 0;
  bool is_over_temp_flag = 0; // The flag is set whenever the relay is turned of because of the plate temp.
  bool is_out_of_water = 0; //Check plate temp to see if there is water left.
}state_vals;

struct TempTarget
{
  uint16_t target_temp = 31;
  uint16_t target_humidity=80;
  uint16_t target_v=20;
  uint16_t target_fio2 = 53;
  bool target_st;
  bool is_init_encoder_position;
  uint32_t encoder_position;
  char buffer[32];
  char range_temp[7]; //Previously 11
  char range_rh[21];
  char range_v[DELTA_V]; //Previously 41
  char range_st[4] = {0,1,0,1}; //Prev 0,0 1,1
  char range_fio2[46];
}target_vals;

struct Alarms
{
  bool  is_plate_too_hot;
  bool  is_ready_working;
  bool  is_setting_up;
  bool  is_in_stand_by;
  bool  is_dry;
  bool  has_unusual_flow;
  bool  has_sensor_fault;
  bool  error;
}les_alarms;

// Objects
SimpleDHT22 dht22(DHTPIN);
RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
ClickButton button1(BUTTON, LOW, CLICKBTN_PULLUP);
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

// Prototypes
void read_flow(StateVals *vals);
void read_thermistor(StateVals *vals);
void estimate_exit_humidity(StateVals *vals);
void control_PD_humidity(StateVals *vals);
void control_SMC_temp (StateVals *vals);
void control_PID_Fan(StateVals *vals);
void mapped_fan_control(StateVals *vals);
void execute(StateVals *vals);
void read_dht(StateVals *vals);
void read_encoder_button(StateVals *vals, TempTarget *target);
void write_main_menu(StateVals *vals, TempTarget *target);
void write_config_menu(StateVals *vals, TempTarget *target);
void write_debug_menu(StateVals *vals, TempTarget *target);
void screen_debug_manager(StateVals *vals);
float get_density(float temp);
void lcd_buffer_write(TempTarget *target);
void beep_manager(StateVals *vals);
void beep_creator(StateVals *vals, BeepType);
void encoderButtonISR();
void encoderISR();
byte i2c_scanner();
float arr_average(float *arr, uint16_t size);
float integral_control(float *i_control, uint16_t isize);
void manage_cursor(StateVals *vals);
void alarm_manager(StateVals *vals, Alarms *alarm);
void flow_to_PWM(StateVals *vals);
void read_o2(StateVals *vals, TempTarget *target);
void read_ds18b20(StateVals *vals);
void hose_bang_bang(StateVals *vals);
void get_target_temperature(StateVals *vals);

void setup() 
{
// // Make custom characters:
 byte Fi[] = { 
  B11110,
  B10000,
  B10001,
  B11100,
  B10001,
  B10001,
  B10001,
  B00000
};
 byte o2[] = {  
  B11100,
  B10100,
  B10100,
  B11111,
  B00001,
  B00111,
  B00100,
  B00111
};
byte Water[] = {
  B00100,
  B01110,
  B01110,
  B11101,
  B11101,
  B11011,
  B01110,
  B00000
};
byte Temperatura[] = {
  B00100,
  B01010,
  B01010,
  B01010,
  B01010,
  B10001,
  B10001,
  B01110
};
byte Grade[] = {
  B11100,
  B10100,
  B11100,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};
byte Bug[] = {
  B11111,
  B10101,
  B11111,
  B11111,
  B01110,
  B01010,
  B11011,
  B00000
};

  analogWriteFrequency(20000);
  TempTarget *target = &target_vals;
  Serial.begin(115200);
  Serial.println("Booting up!");
  pinMode(PLATE_RELAY_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  analogWrite(FAN_PIN, 0);
  pinMode(HOSE_PIN, OUTPUT);
  digitalWrite(HOSE_PIN,LOW);
  pinMode(DHTPIN, INPUT);
  pinMode(PIN_THERMISTOR, INPUT_ANALOG);
  pinMode(WIND_THERM_PIN, INPUT_ANALOG);
  pinMode(WIND_SPEED_PIN, INPUT_ANALOG);
  pinMode(BUZZER_PIN, OUTPUT);
  lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS);
    // Create a new characters:
  lcd.createChar(0, Temperatura);
  lcd.createChar(1, Water);
  lcd.createChar(2, Grade);
  lcd.createChar(3, Bug);
  lcd.createChar(4, Fi);
  lcd.createChar(5, o2);

  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), encoderISR, CHANGE);
  
  o2sens_init();
  Serial3.begin(9600,SERIAL_8N2);

  // Start up the library 
  sensors.begin();
  sensors.setWaitForConversion(false);

  //Boot up sequence
  analogWrite(FAN_PIN, 0);
  digitalWrite(PLATE_RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  sprintf(target->buffer, "Humidifier v2.00FABLAB-MINSA-UTP");
  lcd_buffer_write(&target_vals);
  delay(2000);
  digitalWrite(BUZZER_PIN, LOW);
  sprintf(target->buffer, "PROTOTIPO  ALPHAUSO EXPERIMENTAL");
  lcd_buffer_write(&target_vals);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  delay(300);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  delay(300);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
  next_screen_update = millis()+2000;
}

void loop() 
{
  StateVals *vals = &state_vals;
  Alarms *alarm = &les_alarms;
  static bool next_beep_overflow_flag = false, flow_estimate_overflow_flag = false, thermistor_update_overflow_flag = false, dht_update_overflow_flag = false, next_estimation_overflow_flag = false, next_bangbang_overflow_flag = false, next_pidfan_overflow_flag = false, pid_update_overflow_flag = false, next_execute_overflow_flag = false, next_encoder_overflow_flag = false, screen_update_overflow_flag = false, next_alarm_overflow_flag = false, o2_update_overflow_flag = false, ds_update_overflow_flag = false, hose_bb_overflow_flag = false, get_vtemp_overflow_flag = false;

  if (!next_beep_overflow_flag)
  {
    if(millis() > next_beep_update )
    {
      beep_manager(&state_vals);
      next_beep_update = millis() + BEEP_UPDATE_DELAY;
      if(next_beep_update < millis()) next_beep_overflow_flag = true;
    }
  }
  else if(millis() < next_beep_update) next_beep_overflow_flag = false;

  if (!next_alarm_overflow_flag)
  {
    if(millis() > next_alarm_update )
    {
      alarm_manager(&state_vals, &les_alarms);
      next_alarm_update = millis() + ALARM_UPDATE_DELAY;
      if(next_alarm_update < millis()) next_alarm_overflow_flag = true;
    }
  }
  else if(millis() < next_alarm_update) next_alarm_overflow_flag = false;

  if (!flow_estimate_overflow_flag)
  {
    if (millis() > next_flow_update) 
    {
      read_flow(&state_vals);
      next_flow_update = millis() + FLOW_UPDATE_DELAY;
      if(next_flow_update < millis()) flow_estimate_overflow_flag = true;
    }
  }
  else if(millis() < next_flow_update) flow_estimate_overflow_flag = false;

  if (!ds_update_overflow_flag)
    {
      if (millis() > next_ds_update) 
      {
        read_ds18b20(&state_vals);
        next_ds_update = millis() + DS_UPDATE_DELAY;
        if(next_ds_update < millis()) ds_update_overflow_flag = true;
      }
    }
    else if(millis() < next_ds_update) ds_update_overflow_flag = false;

  if (!get_vtemp_overflow_flag)
    {
      if (millis() > get_vtemp_update) 
      {
        get_target_temperature(&state_vals);
        get_vtemp_update = millis() + GET_VTEMP_DELAY;
        if(get_vtemp_update < millis()) get_vtemp_overflow_flag = true;
      }
    }
    else if(millis() < get_vtemp_update) get_vtemp_overflow_flag = false;

  if (!hose_bb_overflow_flag)
  {
    if (millis() > next_hose_bb_update) 
    {
      hose_bang_bang(&state_vals);
      next_hose_bb_update = millis() + HOSE_BB_UPDATE_DELAY;
      if(next_hose_bb_update < millis()) hose_bb_overflow_flag = true;
    }
  }
  else if(millis() < next_hose_bb_update) hose_bb_overflow_flag = false;

  if (!thermistor_update_overflow_flag)
  {
    if (millis() > next_termistor_update) 
    {
      read_thermistor(&state_vals);
      next_termistor_update = millis() + TERMISTOR_UPDATE_DELAY;
      if(next_termistor_update < millis()) thermistor_update_overflow_flag = true;
    }
  }
  else if(millis() < next_flow_update) thermistor_update_overflow_flag = false;

  if (!dht_update_overflow_flag)
  {
    if (millis() > next_dht_update) 
    {
      read_dht(&state_vals);
      next_dht_update = millis() + DHT_UPDATE_DELAY;
      if(next_dht_update < millis()) dht_update_overflow_flag = true;
    }
  }
  else if(millis() < next_dht_update) dht_update_overflow_flag = false;
  
  while (Serial3.available())
  {
    read_o2(&state_vals, &target_vals);
  }

  if (!next_estimation_overflow_flag)
  {
    if (millis() > next_flow_estimation) 
    {
      estimate_exit_humidity(&state_vals);
      next_flow_estimation = millis() + EXIT_HUMIDITY_ESTIMATION_DELAY;
      if(next_flow_estimation < millis()) next_estimation_overflow_flag = true;
    }
  }
  else if(millis() < next_flow_estimation) next_estimation_overflow_flag = false;

  if (!next_bangbang_overflow_flag)
  {
    if (millis() > next_bangbang_control) 
    {
      control_PD_humidity(&state_vals);
      control_SMC_temp(&state_vals);
      next_bangbang_control = millis() + BANGBANG_CONTROL_DELAY;
      if(next_bangbang_control < millis()) next_bangbang_overflow_flag = true;
    }
  }
  else if(millis() < next_bangbang_control) next_bangbang_overflow_flag = false;

  if (!next_pidfan_overflow_flag)
  {
    if (millis() > next_pidfan_control) 
    {
      control_PID_Fan(&state_vals);
      //mapped_fan_control(&state_vals);
      next_pidfan_control = millis() + PID_FAN_CONTROL_DELAY;
      if(next_pidfan_control < millis()) next_pidfan_overflow_flag = true;
    }
  }
  else if(millis() < next_pidfan_control) next_pidfan_overflow_flag = false;

  if (!next_execute_overflow_flag)
  {
    if (millis() > next_execute) 
    {
      execute(&state_vals);
      next_execute = millis() + EXECUTE_DELAY;
      if(next_execute < millis()) next_execute_overflow_flag = true;
    }
  }
  else if(millis() < next_execute) next_execute_overflow_flag = false;

  if (!next_encoder_overflow_flag)
  {
    if (millis() > next_encoder_update) 
    {
      read_encoder_button(&state_vals, &target_vals);
      next_encoder_update = millis() + ENCODER_UPDATE_DELAY;
      if(next_encoder_update < millis()) next_encoder_overflow_flag = true;
    }
  }
  else if(millis() < next_encoder_update) next_encoder_overflow_flag = false;

  if (!screen_update_overflow_flag)
  {
    manage_cursor(&state_vals);
    if (millis() > next_screen_update) 
    {  
      if(vals->is_main_menu)
      {
        write_main_menu(&state_vals, &target_vals);
      }
      else if (vals->is_config_mode)
      {
        write_config_menu(&state_vals, &target_vals);
      }
      else if (vals->is_debug_mode)
      {
        write_debug_menu(&state_vals, &target_vals);
      }
      next_screen_update = millis() + SCREEN_UPDATE_DELAY;
      if(next_screen_update < millis()) screen_update_overflow_flag = true;
    }
  }
  else if(millis() < next_screen_update) screen_update_overflow_flag = false;
}

void estimate_exit_humidity(StateVals *vals)
{
  #ifdef DEBUG
  Serial.println("Estimating flow...");
  #endif
  vals->vapor_abs_humidity = (6.112*exp((17.67*vals->vapor_temp)/(vals->vapor_temp + 243.5))*(vals->vapor_humidity)*2.1674)/(273.15+vals->vapor_temp); //[g/m³]
  float entry_density = get_density(vals->vapor_temp);
  float exit_density = get_density(vals->after_hose_temp);
  vals->est_abs_humidity = vals->vapor_abs_humidity * exit_density/entry_density; //water_mass_flow/exit_airflow;
  vals->est_humidity = ((273.15+vals->after_hose_temp)*vals->est_abs_humidity)/(6.112*exp((17.67*vals->after_hose_temp)/(vals->after_hose_temp + 243.5))*2.1674);
}

void control_PD_humidity(StateVals *vals)
{
  static float error_humidity_current;
  static float error_humidity_old, error_air_temp;
  float current_step = millis()%PERIODO;
  static float old_millis;

  vals->clock = current_step;
  //Read error values
  error_humidity_current = vals->target_humidity - vals->vapor_humidity;
  error_air_temp = vals->target_temp - vals->vapor_temp;

  //Write new Duty Cycle value
  vals->duty_cycle = (KP_PD_HUM * error_humidity_current + (KP_PD_TEMP+KP_PD_TEMP_MOD*vals->set_o2_flow/40) * error_air_temp);
  
  //Overwrite old error values
  error_humidity_old = error_humidity_current;
  old_millis = millis();

  if (vals->duty_cycle > 60)
  {
    vals->duty_cycle = 60;       
  }
  else if (vals->duty_cycle < 0)
  {
    vals->duty_cycle = 0;       
  }

  if (vals->is_over_temp_flag)
  {
    vals->duty_cycle = 0;
  }

  //TODO: change into a bool
  if (vals->est_humidity > vals->target_humidity)
  {
    vals->duty_cycle = 0;
  }
  //TODO: change to a P control of the vapor_target_temp
  if (vals->vapor_temp >= vals->vapor_target_temp)
  {
    vals->duty_cycle = 0;
  }

}

void control_SMC_temp (StateVals *vals)
{
  static float temp_old, old_millis,current_millis,pendiente,error_temp;
  float current_step = millis()%PERIODO;
  current_millis = millis();
  pendiente = (vals->plate_temp-temp_old)/(current_millis-old_millis);
  error_temp = vals->target_humidity+40 - vals->plate_temp;

  if(pendiente >= KP_SMC*error_temp)
  {
    vals->duty_cycle = 0;
  }
  old_millis = millis();
  temp_old = vals->plate_temp;

  if (vals->is_over_temp_flag)
  {
    vals->duty_cycle = 0;
  }
  //TODO: Make it a different function
  if ((current_step < ((vals->duty_cycle/100)*PERIODO)))
  {
    vals->plate_relay_cmd = true;
  }
  else
  {
    vals->plate_relay_cmd = false;
  }

  //TODO: Make it a different function
  if (vals->plate_temp > MAX_PLATE_TEMP)
  {
    vals->is_over_temp_flag = 1;
  }
  if (vals->plate_temp < MAX_PLATE_TEMP-15 && vals->is_over_temp_flag)
  {
    vals->is_over_temp_flag = 0;
  }
}

void control_PID_Fan(StateVals *vals)
{
  static float error_airflow_current;
  static float error_airflow_old;
  static float delta_error_airflow_current, integral_error_airflow_current;
  static int32_t  pwm_dot, pwm_const;
  static uint16_t old_millis, integral_num = 0;
  static float integral_array[180];
  static bool init = 0, pwm_bool = 0;
  static float delta_time  = (millis()-old_millis);

  //Read error values
  error_airflow_current = vals->target_airflow - vals->current_airflow;
  delta_error_airflow_current = (error_airflow_current - error_airflow_old)/(delta_time);
  
  //TODO Turn into a function - Integral limiter

  if (!init)
    {
      for(int i = 0; i<(sizeof(integral_array)/sizeof(integral_array[0])); i++)
      {
        integral_array[i] = 0.0;
      }
      init = true;
    }
  integral_array[integral_num++] = error_airflow_current*delta_time/1000;
  integral_error_airflow_current = integral_control(integral_array, sizeof(integral_array));
  

  if(integral_num > 180)
  {
    integral_num = 0;
  }

  //Calculate PWM dot
  pwm_dot = (KP_FAN * error_airflow_current + KI_FAN * integral_error_airflow_current +  KD_FAN * delta_error_airflow_current);
  
  //Write new PWM value    
  vals->fan_pwm = vals->fan_pwm + pwm_dot;

  
  if (error_airflow_current < 1.5 && error_airflow_current > -1.5 && !pwm_bool)
  {
    pwm_const = vals->fan_pwm;
    pwm_bool = 1;
  }
  else if (error_airflow_current > 1.5 || error_airflow_current < -1.5)
  {
    pwm_bool = 0;
  }
  
  if (pwm_bool)
  {
    vals->fan_pwm = pwm_const;
  }
  //Overwrite old error values>
  error_airflow_old = error_airflow_current;
  old_millis = millis();

  if (vals->fan_pwm > 255)
  {
    vals->fan_pwm = 255;       
  }
  else if (vals->fan_pwm < 0)
  {
    vals->fan_pwm = 0;       
  }
}

void mapped_fan_control(StateVals *vals)
{
  vals->fan_pwm = map(vals->target_airflow, 0, 100, 0, 256);
}

void execute(StateVals *vals)
{
  if(vals->pwr_state)
  {
    if(vals->vapor_temp > vals->vapor_target_temp)
    {
      vals->is_vapor_too_hot = 1;
    } 
    else
    {
      vals->is_vapor_too_hot = 0;
    }
    
    if(vals->plate_relay_cmd && !vals->is_vapor_too_hot)
    {
      digitalWrite(PLATE_RELAY_PIN, LOW);
      vals->plate_relay_state = true;
    }
    else
    {
      digitalWrite(PLATE_RELAY_PIN, HIGH);
      vals->plate_relay_state = false;
    }

    analogWrite(FAN_PIN,vals->fan_pwm);
    digitalWrite(HOSE_PIN, vals->hose_state);
  }
  else
  {
    analogWrite(FAN_PIN, 0);
    digitalWrite(HOSE_PIN, 0);
    digitalWrite(PLATE_RELAY_PIN, HIGH);
    vals->plate_relay_state = false;
  }
}

void read_dht(StateVals *vals)
{
  float humidity, temp;
  int err = SimpleDHTErrSuccess; 
  if ((err = dht22.read2(&temp, &humidity, NULL)) != SimpleDHTErrSuccess) 
  {
    return;
  }
  if(isnan(humidity)||isnan(temp))
    {
      vals->vapor_humidity = 0.0;
      vals->vapor_temp = 0.0;
      return;
    }
  vals->vapor_humidity = humidity;
  vals->vapor_temp = temp;
  return;
}

void read_thermistor(StateVals *vals)
{
  static uint8_t temp_num = 0;
  static float temp_array[256];
  static bool init = 0;
  if (!init)
    {
      for(int i = 0; i<(sizeof(temp_array)/sizeof(temp_array[0])); i++)
      {
        temp_array[i] = 25.0;
      }
      init = true;
    }
  uint16_t adc_read = analogRead(PIN_THERMISTOR);
  float resistance = 100000*(((float)adc_read)/(1024-(float)adc_read));
  // temp_array[temp_num] = (B_VALUE*(25+273.15) / ((25+273.15)*log(resistance/100000)+B_VALUE)) - 273.15;
  temp_array[temp_num] = (B_VALUE) / (log(resistance)+1.73544) - 273.15;
  temp_num++;
  vals->plate_temp = arr_average(temp_array, sizeof(temp_array)) - 7;
  vals->adc_therm = adc_read;
  vals->them_resistance = resistance;
}

void read_flow(StateVals *vals) 
{
  float TMP_Therm_ADunits = analogRead(WIND_THERM_PIN);
  float RV_Wind_ADunits = analogRead(WIND_SPEED_PIN);
  float x = RV_Wind_ADunits;
  float y = TMP_Therm_ADunits;
  static float x_prom,y_prom,sensor_airspeed;
  static uint8_t speed_num = 0;
  static float x_array[32];
  static float y_array[32];
  
  x_array[speed_num] = x;
  y_array[speed_num] = y;
  speed_num++; 

 
  if (speed_num > 31)
  {
    x_prom = arr_average(x_array, sizeof(x_array));
    y_prom = arr_average(y_array, sizeof(y_array));
    //sensor_airspeed =  -1.056891481e-5 *x_prom*x_prom*x_prom + 3.245687769e-5 *x_prom*x_prom * y_prom - 3.605536507e-5 *x_prom *y_prom*y_prom + 1.138111633e-5 *y_prom*y_prom*y_prom + 2.436108257e-3 *x_prom*x_prom - 1.102028777e-3 *x_prom*y_prom + 3.695040756e-3 *y_prom*y_prom - 1.163719965 *x_prom - 1.820651701 *y_prom + 580.7883239;
    sensor_airspeed = 1.694651444e-3 *y_prom*y_prom - 2.392395736e-3 *x_prom*y_prom + 1.974089026e-3 *x_prom*x_prom - 0.800739832 *y_prom - 7.872175317e-1 *x_prom + 496.3395487;
    speed_num = 0;

  }

  vals->adc_flow_v = arr_average(x_array, sizeof(x_array));
  vals->adc_flow_t = arr_average(y_array, sizeof(y_array));
  
  if (sensor_airspeed <  0)
  {
    sensor_airspeed = 0;
  }
  vals->current_airspeed = sensor_airspeed;
  if(vals->pwr_state)
  {
    vals->current_airflow = sensor_airspeed;
  }
  else
  {
    vals->current_airflow = 0;
  }
  
}

void read_o2(StateVals *vals, TempTarget *target)
{
  o2sens_feedUartByte(Serial3.read()); // give byte to the parser
  vals->o2_test = vals->o2_test + 1;
  if (o2sens_hasNewData()) // a complete packet has been processed
  {
    o2sens_clearNewData(); // clear the new packet flag
    vals->o2_test = vals->o2_test + 100 -12;
    vals->o2_concentration = o2sens_getConcentration16();
    vals->o2_flow = o2sens_getFlowRate16();
    vals->o2_temp = o2sens_getTemperature16(); 
  }
  vals->o2_test = vals->o2_test + 2;
}

void read_encoder_button(StateVals *vals, TempTarget *target)
{
  button1.Update();
  if(button1.clicks == 1 && (vals->is_config_mode || vals->is_debug_mode)) 
  {
    vals->button_counter++;
    if(vals->button_counter%POSIBLE_POSITIONS == 3 && vals->is_config_mode)
    {
      vals->button_counter++;
    }
    target->is_init_encoder_position = 1;
  }
  else if(button1.clicks == -1 )
  {
    //Screen state machine
    beep_creator(vals,BEEP_ONCE);
    if(vals->is_main_menu)
    {
      //Just to be safe.
      encoder.setPosition(10000);
      vals->is_config_mode = 1;
      vals->is_main_menu = 0;  
      vals->button_counter = 0;
      vals->is_debug_mode = 0;
    }
    else if(vals->is_config_mode)
    {
      vals->is_config_mode = 0;
      vals->is_main_menu = 1;
      vals->is_debug_mode = 0;
      vals->target_temp = target->target_temp;
      vals->target_humidity = target->target_humidity;
      vals->target_airflow = target->target_v;
      vals->pwr_state = target->target_st;
      vals->target_fio2 = target->target_fio2;
    }
    else if(vals->is_debug_mode)
    {
      vals->is_config_mode = 0;
      vals->is_main_menu = 1;
      vals->is_debug_mode = 0;
    }
  }
  else if(button1.clicks == 2 && vals->is_main_menu)
  {
    vals->is_config_mode = 0;
    vals->is_main_menu = 0;  
    vals->is_debug_mode = 1;
    vals->current_beep.beep_id++;
    vals->current_beep.beep_type=BEEP_CONTINUOS;
    vals->current_beep.beep_clock = vals->current_beep.beep_type;
  }
}

void manage_cursor(StateVals *vals)
{
  if(vals->is_config_mode)
  {
    switch(vals->button_counter%POSIBLE_POSITIONS)
    {
      case TOP_LEFT:
        lcd.setCursor(POSX1,POSY1);
        break;
      case TOP_RIGHT:
        lcd.setCursor(POSX2,POSY2);
        break;
      case DOWN_LEFT:
        lcd.setCursor(POSX3,POSY3);
        break;
      case DOWN_MIDDLE:
        lcd.setCursor(POSX4,POSY4);
        break;
      case DOWN_RIGHT:
        lcd.setCursor(POSX5,POSY5);     
        break;    
    }
    lcd.blink();
  }
  else
  {
    lcd.noBlink();
  } 
}

void write_config_menu(StateVals *vals, TempTarget *target)
{
  static uint32_t last_enc = 10000;
  static uint32_t value_encoder;
  //Rango de los cases
  static char cases[5] = {7,21,DELTA_V,46,4}; //3rd value prev 41isnan
  static char lcd_st[3][4] = {"OFF","ON "};

  //Define range of values for each variable
  for(int i=0;i<sizeof(target->range_temp);i++)
  {
    target->range_temp[i] = i+31;
  }
  for(int i=0;i<sizeof(target->range_rh);i++)
  {
    target->range_rh[i] = i+80;
  }
  for(int i=0;i<sizeof(target->range_v);i++)
  {
    target->range_v[i] = i+10;
  }
  for(int i=0;i<sizeof(target->range_fio2);i++)
  {
    target->range_fio2[i] = i+21;
  }
  
  if(target->is_init_encoder_position)
  {
    target->is_init_encoder_position = false; 
    switch (vals->button_counter%POSIBLE_POSITIONS)
    {
      case TOP_LEFT:
        for(int i=0;i<=sizeof(target->range_temp);i++)
        {
          if(target->target_temp==target->range_temp[i])
          {
            //Is the i value that matches the array position.
            value_encoder = i+sizeof(target->range_temp)*100;
            break;  
          }
        }
        break;
      case TOP_RIGHT:
        for(int i=0;i<=sizeof(target->range_rh);i++)
        {
          if(target->target_humidity==target->range_rh[i])
          {
            value_encoder = i+sizeof(target->range_rh)*100;
            break;  
          }
        }
        break;
      case DOWN_LEFT:
        for(int i=0;i<=sizeof(target->range_v);i++)
        {
          if(target->target_v==target->range_v[i])
          {
            value_encoder = i+sizeof(target->range_v)*100;
            break;  
          }
        }
        break;
      case DOWN_MIDDLE:
      for(int i=0;i<=sizeof(target->range_fio2);i++)
      {
        if(target->target_fio2==target->range_fio2[i])
        {
          value_encoder = i+sizeof(target->range_fio2)*100;
          break;  
        }
      }
      case DOWN_RIGHT:
      for(int i=0;i<=sizeof(target->range_st);i++)
      {
        if(target->target_st==target->range_st[i])
        {
          value_encoder = i+sizeof(target->range_st)*100;
          break;  
        }
      }
    }
    
  }
  
  //Change the target value using the encoder value as reference
  value_encoder = value_encoder + (int)((encoder.getPosition()/2-last_enc/2));
  target->encoder_position = (value_encoder)%(cases[vals->button_counter%POSIBLE_POSITIONS]);
  last_enc = encoder.getPosition();
  switch (vals->button_counter%POSIBLE_POSITIONS)
    {
    case TOP_LEFT:
      target->target_temp = target->range_temp[target->encoder_position];
      break;
    case TOP_RIGHT:
      target->target_humidity = target->range_rh[target->encoder_position];
      break;
    case DOWN_LEFT:
      target->target_v = target->range_v[target->encoder_position];
      break;
    case DOWN_MIDDLE:
      target->target_fio2 = target->range_fio2[target->encoder_position];
      break;
    case DOWN_RIGHT:
      target->target_st = target->range_st[target->encoder_position];     
      break;    
    }
  sprintf(target->buffer, "T:%2d%cC RH:%3d%%   %3dLPM   %c%c%c     ", target->target_temp,byte(2),target->target_humidity,target->target_v,lcd_st[target->target_st][0],lcd_st[target->target_st][1],lcd_st[target->target_st][2]);

  lcd_buffer_write(&target_vals);

}

void write_main_menu(StateVals *vals, TempTarget *target)
{
  //TODO: fix - this shouldn't be here
  float printed_est_humidity = vals->est_humidity;
  if(vals->est_humidity > 100)
  {
    printed_est_humidity = 100;
  }

  if(vals->pwr_state) 
  {
    sprintf(target->buffer, " %c%2d%cC   %c%3d%%  %2dLPM %c%c:%2d%% ON ", byte(0), (int)vals->after_hose_temp, byte(2), byte(1), (int)printed_est_humidity,(int)vals->current_airflow, byte(4), byte(5),(int)vals->o2_concentration/10);
  }
  else
  {
    sprintf(target->buffer, " %c%2d%cC   %c%3d%%  %2dLPM %c%c:%2d%% OFF", byte(0), (int)vals->after_hose_temp, byte(2), byte(1), (int)printed_est_humidity, (int)vals->current_airflow, byte(4), byte(5),(int)vals->o2_concentration/10);
  }
   
  lcd_buffer_write(&target_vals);
}

void write_debug_menu(StateVals *vals, TempTarget *target)
{
  switch (vals->button_counter%POSIBLE_POSITIONS)
      {
      case TOP_LEFT:
        sprintf(target->buffer, "%c CMD:%2d  %4d   Clk:%d  DC:%3d %c", byte(3),(int)vals->plate_relay_cmd,millis()%PERIODO, (int)vals->current_beep.beep_clock, (int)vals->duty_cycle,byte(3));
        break;
      case TOP_RIGHT:
        sprintf(target->buffer, "%c Flow:%2dL/min vTMP:%3d eRH:%3d%c",byte(3),(int)vals->current_airflow,(int)vals->vapor_target_temp,(int)vals->est_humidity,byte(3)); // overtempplat ,cuando el termistor fuerza que se apague
        break;
      case DOWN_LEFT:
        sprintf(target->buffer, "%c HOSE_TEMP:%3d PL_T:%2d OVR_T:%d",byte(3),(int)vals->after_hose_temp, (int)vals->plate_temp, (int)vals->is_over_temp_flag ); //adC TERMISTOR,  temp plato, velocidad,
        break;
      case DOWN_MIDDLE:
        sprintf(target->buffer, "%c VAPOR_TEM:%3d vRH%2d HOSE_ST:%d%c",byte(3),(int)vals->vapor_temp,(int)(vals->vapor_humidity),(int)vals->hose_state,byte(3));
        break;  
      case DOWN_RIGHT:
        sprintf(target->buffer, "%c ADC_T:%d ADC_V:%2d PWM:%3d %c",byte(3),(int)vals->adc_flow_t,(int)vals->adc_flow_v,(int)vals->fan_pwm,byte(3));
        break;   
      }
  lcd_buffer_write(&target_vals);
}

float get_density(float t)
{
  return (352.96/(273.15+t));
}

void encoderISR()
{
  encoder.readAB();
}

void encoderButtonISR()
{
  encoder.readPushButton(); 
}

void lcd_buffer_write(TempTarget *target)
{
  for (int i=0; i<(sizeof(target->buffer)/sizeof(char)); i++)
  {
    if(i<16) 
    {
      lcd.setCursor(i, 0);
      lcd.print(target->buffer[i]);
    }
    else 
    {
      lcd.setCursor(i-16,1);
      lcd.print(target->buffer[i]);
    }
  }
  return;
}

void beep_manager(StateVals *vals)
{
  Beeps current_beep = vals->current_beep;
  static uint16_t prev_beep_id;
  static uint32_t beep_once_timeout;

if((current_beep.beep_id != prev_beep_id) && millis()>beep_once_timeout)
  {
    digitalWrite(BUZZER_PIN, current_beep.beep_clock%2);
    if (current_beep.beep_clock > 0)
    {
      vals->current_beep.beep_clock--;
      beep_once_timeout = millis() + BEEP_ONCE_DURATION;
    }
    else
    {
      prev_beep_id = current_beep.beep_id;
    }
  }
}

void beep_creator(StateVals *vals, BeepType beep_num)
{
  vals->current_beep.beep_id++;
  vals->current_beep.beep_type=beep_num;
  vals->current_beep.beep_clock = vals->current_beep.beep_type;
}
float arr_average(float arr[256], uint16_t size)
{
  float sum = 0;
  float average = 0;
  int i = 0;
  for(i; i<(size/sizeof(float)); i++) 
  {
    sum = sum + arr[i];
  }
  average = sum / i;
  return (average);
}

float integral_control(float i_control[256], uint16_t isize)
{
  float sum = 0;
  int i = 0;
  for(i; i<(isize/sizeof(float)); i++)
  {
    sum = sum + i_control[i];
  }
  return (sum);
}

void alarm_manager(StateVals *vals, Alarms *alarm)
{
  if(vals->plate_temp>MAX_PLATE_TEMP)
  {
    alarm->is_plate_too_hot = 1;
  }
  if(1)
  {
    alarm->is_ready_working = 1;
  }
  if(1)
  {
    alarm->is_setting_up = 1;
  }
  if(1)
  {
    alarm->is_in_stand_by = 1;
  }
    if(vals->plate_temp>MAX_PLATE_TEMP+20)
  {
    alarm->is_dry = 1;
  }
  if(vals->current_airflow > 100)
  {
    alarm->has_unusual_flow = 1;
  }
    if(1)
  {
    alarm->has_sensor_fault = 1;
  }
  if(1)
  {
    alarm->error = 1;
  }
}

void read_ds18b20(StateVals *vals)
{
  sensors.requestTemperatures(); // Send the command to get temperature readings 
  vals->after_hose_temp = sensors.getTempCByIndex(0);
}

void hose_bang_bang(StateVals *vals)
{
  if(vals->after_hose_temp <= vals->target_temp)
  {
    vals->hose_state = 1;
  }
  else
  {
    vals->hose_state = 0;
  } 
}

void get_target_temperature(StateVals *vals)
{
  float target_vapor_temp, error_humidity, est_target_abs_humidity;
  //Calculate abs_humidity with target values
  float target_abs_humidity = (6.112*exp((17.67*vals->target_temp)/(vals->target_temp + 243.5))*(vals->target_humidity)*2.1674)/(273.15+vals->target_temp); //[g/m³]
  
  //Calculate target vapor temp
  for (int i=20;i<=38;i++)
  {
    target_vapor_temp = i;
    est_target_abs_humidity = (6.112*exp((17.67*target_vapor_temp)/(target_vapor_temp + 243.5))*(vals->vapor_humidity)*2.1674)/(273.15+target_vapor_temp); //[g/m³]
    //Calculate error humidity
    error_humidity = 100*(target_abs_humidity - est_target_abs_humidity)/target_abs_humidity;

    if(error_humidity <= 5 && error_humidity >= -5)
    {
      vals->vapor_target_temp = target_vapor_temp;
      return;
    }
  }
  //TODO: change to a better fix
  //Default value is 33°C because I say so
  vals->vapor_target_temp = 33;
}