 
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
#include <DHT.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <ClickButton.h>
#include <math.h>
//#include <Adafruit_SleepyDog.h>
#include <IWatchdog.h>

// Physical properties
#define DIAMETER                0.0177
#define REFERENCE_RESISTANCE    100000
#define NOMINAL_RESISTANCE      100000
#define NOMINAL_TEMPERATURE     23
// #define B_VALUE              870
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

// Behaviour characteristics
#define HOSE_CONVERSION_FACTOR  .01 // Converts pwm output to rms volts in output.
#define PLATE_HISTERESIS        5 // Plate deadzone size.
#define MIN_DELTA_T             2
#define LCD_ROWS         2     // Quantity of lcd rows.
#define LCD_COLUMNS      16    // Quantity of lcd columns.
#define LCD_SPACE_SYMBOL 0x20  //Space symbol from lcd ROM, see p.9 of GDM2004D datasheet.
// #define DEBUG

// Timming
#define FLOW_UPDATE_DELAY       100
#define TERMISTOR_UPDATE_DELAY  10
#define DHT_UPDATE_DELAY        2005
#define FLOW_ESTIMATION_DELAY   100
#define BANGBANG_CONTROL_DELAY  100
#define PID_FAN_CONTROL_DELAY   100
#define PID_UPDATE_DELAY        100
#define EXECUTE_DELAY           10
#define ENCODER_UPDATE_DELAY    10
#define SCREEN_UPDATE_DELAY     500
#define BEEP_UPDATE_DELAY       10
#define BEEP_ONCE_DURATION      300
#define ALARM_UPDATE_DELAY       10

//PID Values
#define KP_PD_HUM               3.5
#define KD_PD_HUM               0
#define PERIODO                 2000
#define KP_FAN                  0
#define KD_FAN                  0//35
#define KI_FAN                  0.045//0.000000000000000000000000000000000000000000000000000000000000000000000000000000000001
#define KP_SMC                  0.02
#define MAX_PWM_MODIFIER        500

//Cursor Locations
#define POSIBLE_POSITIONS       4
#define POSX1                   4
#define POSY1                   0
#define POSX2                   13
#define POSY2                   0
#define POSX3                   4
#define POSY3                   1
#define POSX4                   13
#define POSY4                   1

//Alarm critical values
#define MAX_PLATE_TEMP          95
#define MAX_V                   101 _//101: mapped mode; 41: L/min mode


// Globlas
const float zeroWindAdjustment =  .2;
uint32_t next_flow_update, next_termistor_update, next_dht_update, next_flow_estimation, next_bangbang_control, next_pidfan_control, next_pid_update, next_execute, next_encoder_update, next_screen_update, next_beep_update, next_alarm_update;
float kpa, kph;

// Structs

enum BeepType
{
  NO_BEEP,
  BEEP_ONCE,
  BEEP_TWICE,
  BEEP_TRRICE,
  BEEP_CONINUOS
};

enum CursorPositions
{
  TOP_LEFT,
  TOP_RIGHT,
  DOWN_LEFT,
  DOWN_RIGHT
};

struct Beeps
{
  BeepType beep_type = NO_BEEP;
  uint16_t beep_priority = 0;
  uint16_t beep_id = 0;
};

struct StateVals
{
  float plate_temp = 0; // Current plat temperature. °C
  float vapor_temp = 0; // Current temperature at vapor chamber. °C 
  float est_temp = 0; // Estimated temperature after hose. °C
  float est_humidity = 0; // Estimated humidity after hose. RH%
  float vapor_abs_humidity = 0; // Current absolute humidity at vapor chamber. g/m3
  float est_abs_humidity = 0; // Estimated absolute humidity after hose. g/m3
  float target_plate_temp = 0; // Target plate temperature. °C
  float current_airflow = 0; // Air volumetric flow rate. Lts/min
  float current_airspeed = 0; // Air speed. m/s
  uint16_t fan_pwm = 0; // Fan PWM duty cycle.
  float vapor_humidity = 0; // Current humidity at vapor chamber. RH%
  uint16_t hose_pwm = 0; // Hose power PWM duty cycle.
  float target_temp = 0; // Target temperature after hose. °C
  float target_humidity = 0; // Target relative humidity after hose. %RH
  float target_airflow = 0; // Target air flow. Lts/min
  uint16_t initial_target_pwm = 0; //Target PWM for flow control
  bool pwr_state = 0; // Machine on or off.next_dht_update
  bool plate_relay_state = 0; // Plate on or off.
  bool plate_relay_cmd = 0;
  uint32_t clock = 0;
  
  Beeps current_beep;
  uint16_t adc_therm = 0;
  float them_resistance = 0;
  float duty_cycle = 0;

  uint16_t adc_flow_t = 0;
  uint16_t adc_flow_v = 0;
  
  //Encoder Info Handlers
  uint32_t button_counter = 0; //Controls the cursor position
  bool is_main_menu = 1; //Controls Main menu state
  bool is_config_mode = 0; //Controls configuration mode state
  bool is_debug_mode = 0; //Controls debug mode
  //float fan_pwm = 0;
  //Alarm Flags
  bool is_alarm = 0;
  bool is_over_temp_flag = 0; // The flag is set whenever the relay is turned of because of the plate temp.
  bool is_out_of_water = 0; //Check plate temp to see if there is water left.
}state_vals;

struct TempTarget
{
  uint16_t target_temp;
  uint16_t target_humidity;
  uint16_t target_v;
  bool target_st;
  bool is_init_encoder_position;
  uint32_t encoder_position;
  char buffer[32];
  char range_temp[11];
  char range_rh[101];
  char range_v[41]; //Previously 41
  char range_st[4] = {0,0,1,1};


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
Thermistor* thermistor;
DHT dht(DHTPIN, DHTTYPE);
RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
ClickButton button1(BUTTON, LOW, CLICKBTN_PULLUP);


// Prototypes
void read_flow(StateVals *vals);
void read_thermistor(StateVals *vals);
void estimate_flow(StateVals *vals);
void control_PD_humidity(StateVals *vals);
void control_SMC_temp (StateVals *vals);
void control_PID_Fan(StateVals *vals);
void mapped_fan_control(StateVals *vals);
void update_pid(StateVals *vals);
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
void encoderButtonISR();
void encoderISR();
byte i2c_scanner();
float arr_average(float *arr, uint16_t size);
float integral_control(float *i_control, uint16_t isize);
void manage_cursor(StateVals *vals);
void read_flow_old(StateVals *vals);
void alarm_manager(StateVals *vals, Alarms *alarm);
void flow_to_PWM(StateVals *vals);
//void lcd_buffer_write_debug(char buffer [200],uint16_t buffer_size,uint16_t view_port_init);



void setup() 
{
// // Make custom characters:
 byte Heat1[] = {
  
  B11011,
  B10001,
  B10001,
  B00010,
  B00010,
  B00100,
  B10001,
  B11111
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
byte Skull[] = {
  B00000,
  B01110,
  B10101,
  B11011,
  B01110,
  B01110,
  B00000,
  B00000
};

  TempTarget *target = &target_vals;
  Serial.begin(115200);
  Serial.println("Booting up!");
  pinMode(PLATE_RELAY_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  analogWrite(FAN_PIN, 0);
  pinMode(HOSE_PIN, OUTPUT);
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

  encoder.begin();
  dht.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), encoderISR, CHANGE);
  
  //Boot up sequence
  digitalWrite(PLATE_RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  sprintf(target->buffer, "Humidifier v1.00FABLAB-MINSA-UTP");
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
  //IWatchdog.begin(4000000);
}

void loop() 
{
  StateVals *vals = &state_vals;
  Alarms *alarm = &les_alarms;
  static bool next_beep_overflow_flag = false, flow_estimate_overflow_flag = false, thermistor_update_overflow_flag = false, dht_update_overflow_flag = false, next_estimation_overflow_flag = false, next_bangbang_overflow_flag = false, next_pidfan_overflow_flag = false, pid_update_overflow_flag = false, next_execute_overflow_flag = false, next_encoder_overflow_flag = false, screen_update_overflow_flag = false, next_alarm_overflow_flag = false;

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
      //read_flow_old(&state_vals);
      next_flow_update = millis() + FLOW_UPDATE_DELAY;
      if(next_flow_update < millis()) flow_estimate_overflow_flag = true;
    }
  }
  else if(millis() < next_flow_update) flow_estimate_overflow_flag = false;

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
    if (millis() > next_dht_update) 
    {
      read_dht(&state_vals);
      next_dht_update = millis() + DHT_UPDATE_DELAY;
      if(next_dht_update < millis()) dht_update_overflow_flag = true;
    }
  else if(millis() < next_dht_update) dht_update_overflow_flag = false;

  if (!next_estimation_overflow_flag)
  {
    if (millis() > next_flow_estimation) 
    {
      estimate_flow(&state_vals);
      next_flow_estimation = millis() + FLOW_ESTIMATION_DELAY;
      if(next_flow_estimation < millis()) next_estimation_overflow_flag = true;
    }
  }
  else if(millis() < next_flow_estimation) next_estimation_overflow_flag = false;

  if (!next_bangbang_overflow_flag)
  {
    if (millis() > next_bangbang_control) 
    {
      control_PD_humidity(&state_vals);
      //control_SMC_temp(&state_vals);
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

  if (!pid_update_overflow_flag)
  {
    if (millis() > next_pid_update) 
    {
      update_pid(&state_vals);
      next_pid_update = millis() + PID_UPDATE_DELAY;
      if(next_pid_update < millis()) pid_update_overflow_flag = true;
    }
  }
  else if(millis() < next_pid_update) pid_update_overflow_flag = false;

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
//IWatchdog.reload();
}

void estimate_flow(StateVals *vals)
{
  #ifdef DEBUG
  Serial.println("Estimating flow...");
  #endif
  float delta_t = (-1.1306726802e-1*vals->current_airspeed*vals->current_airspeed - 7.364554637e-1*vals->hose_pwm*HOSE_CONVERSION_FACTOR*vals->current_airspeed + 3.197278912*vals->hose_pwm*vals->hose_pwm*HOSE_CONVERSION_FACTOR*HOSE_CONVERSION_FACTOR + 3.70769039 * vals->current_airspeed + 1.303135249*vals->hose_pwm*HOSE_CONVERSION_FACTOR - 5.9446);
  vals->vapor_abs_humidity = (6.112*exp((17.67*vals->vapor_temp)/(vals->vapor_temp + 243.5))*(vals->vapor_humidity/100)*2.1674)/(273.15+vals->vapor_temp);
  vals->est_temp = vals->vapor_temp + delta_t;
  float entry_density = get_density(vals->vapor_temp);
  float exit_density = get_density(vals->est_temp);
  float air_mass_flow = entry_density*vals->current_airflow*1000;
  float water_mass_flow = air_mass_flow*vals->vapor_abs_humidity;
  vals->est_abs_humidity = water_mass_flow/(air_mass_flow/exit_density);
  vals->est_humidity = (273.15+vals->vapor_temp*vals->est_abs_humidity)/(6.112*exp((17.67*vals->vapor_temp)/(vals->vapor_temp + 243.5))*2.1674);


  #ifdef DEBUG
  char serial_buff[100];
  sprintf(serial_buff, "DEBUG: DeltaT is estimated to be about %2.2f °C", delta_t);
  Serial.println(serial_buff);
  // serial_buff = "";
  sprintf(serial_buff, "DEBUG: AbsHumd at hose entry is estimated to be about %1.5f g/m3", vals->vapor_abs_humidity);
  Serial.println(serial_buff);
  // serial_buff = "";
  sprintf(serial_buff, "DEBUG: EntryDensity at hose entry is estimated to be about %1.5f kg/m3", entry_density);
  Serial.println(serial_buff);
  // serial_buff = "";
  sprintf(serial_buff, "DEBUG: ExitDensity at hose exit is estimated to be about %1.5f kg/m3", exit_density);
  Serial.println(serial_buff);
  // serial_buff = "";
  sprintf(serial_buff, "DEBUG: AirMassFlow is estimated to be about %1.5f kg/m3", air_mass_flow);
  Serial.println(serial_buff);
  // serial_buff = "";control_PD_humidity
  sprintf(serial_buff, "DEBUG: WaterMassFlow is estimated to be about %1.5f kg/m3", water_mass_flow);
  Serial.println(serial_buff);
  // serial_buff = "";
  sprintf(serial_buff, "DEBUG: AbsHumd at hose exit is estimated to be about %1.5f g/m3", vals->vapor_abs_humidity);
  Serial.println(serial_buff);
  // serial_buff = "";
  sprintf(serial_buff, "DEBUG: RH at hose exit is estimated to be about %1.5f g/m3", vals->vapor_abs_humidity);
  Serial.println(serial_buff);
  // serial_buff = "";
  #endif
}

void control_PD_humidity(StateVals *vals)
{
  static float error_humidity_current;
  static float error_humidity_old;
  static float delta_error_humidity_current;
  float current_step = millis()%PERIODO;
  static float old_millis;

  vals->clock = current_step;
  //Read error values
  error_humidity_current = vals->target_humidity - vals->vapor_humidity;
  delta_error_humidity_current = (error_humidity_current - error_humidity_old)/(millis()-old_millis);

  //Write new Duty Cycle value
  vals->duty_cycle = (KP_PD_HUM * error_humidity_current + KD_PD_HUM * delta_error_humidity_current);
  
  //Overwrite old error values
  error_humidity_old = error_humidity_current;
  old_millis = millis();

  if (vals->duty_cycle > 50)
  {
    vals->duty_cycle = 50;       
  }
  else if (vals->duty_cycle < 0)
  {
    vals->duty_cycle = 0;       
  }

  if (vals->is_over_temp_flag)
  {
    vals->duty_cycle = 0;
  }
  
  if ((current_step < ((vals->duty_cycle/100)*PERIODO)))
  {
    vals->plate_relay_cmd = true;
  }
  else
  {
    vals->plate_relay_cmd = false;
  }
  
  if (vals->plate_temp > MAX_PLATE_TEMP)
  {
    vals->is_over_temp_flag = 1;
  }
  if (vals->plate_temp < MAX_PLATE_TEMP-15 && vals->is_over_temp_flag)
  {
    vals->is_over_temp_flag = 0;
  }

}

void control_SMC_temp (StateVals *vals)
{
  static float temp_old, old_millis,current_millis,pendiente,error_temp;

  current_millis = millis();
  pendiente = (vals->plate_temp-temp_old)/(current_millis-old_millis);
  error_temp = HUMIDITY_TEMP - vals->plate_temp;

  if(pendiente < KP_SMC*error_temp)
  {
    vals->plate_relay_cmd = 1;
  }
  else
  {
    vals->plate_relay_cmd = 0;
  }
  old_millis = millis();

}

void control_PID_Fan(StateVals *vals)
{
  static float error_airflow_current;
  static float error_airflow_old;
  static float delta_error_airflow_current, integral_error_airflow_current;
  static uint32_t  pwm_modifier;

  
  static uint8_t old_millis, integral_num = 0;
  static float integral_array[100], old_target_flow;
  static bool init = 0;
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
  
  //Calculate PWM reference value and update error
  flow_to_PWM(&state_vals);
  old_target_flow = vals->target_airflow;

  if(integral_num > 100)
  {
    integral_num = 0;
  }

  //Calculate PWM modifier
  pwm_modifier = (KP_FAN * error_airflow_current + KI_FAN * integral_error_airflow_current +  KD_FAN * delta_error_airflow_current);
  
  // if (pwm_modifier > MAX_PWM_MODIFIER)
  // {
  //   pwm_modifier = MAX_PWM_MODIFIER;       
  // }
  // else if (pwm_modifier < -MAX_PWM_MODIFIER)
  // {
  //   pwm_modifier = MAX_PWM_MODIFIER;       
  // }

  //Write new PWM value    
  vals->fan_pwm = vals->initial_target_pwm + pwm_modifier + 35;
  
  //Overwrite old error values>
  error_airflow_old = error_airflow_current;
  old_millis = millis();
  //delta_error_humidity_old = delta_error_humidity_current;

  if (vals->fan_pwm > 256)
  {
    vals->fan_pwm = 256;       
  }
  else if (vals->fan_pwm < 0 || vals->target_airflow < 2)
  {
    vals->fan_pwm = 0;       
  }
  

}

void flow_to_PWM(StateVals *vals)
{ 
  float X = vals->target_airflow;
  uint16_t raw_pwm = 0.001063518307105 * X*X*X - 0.045439480234373 * X*X + 1.69342290412888 * X + 10.8045468316241;
  vals->initial_target_pwm = map(raw_pwm, 0, 100, 0, 256);
}

void mapped_fan_control(StateVals *vals)
{
  //Map target_flow (0-100%) to PWM[50,256] 

  vals->fan_pwm = map(vals->target_airflow, 0, 100, 0, 256);
 
  
}

void update_pid(StateVals *vals)
{
  #ifdef DEBUG
  Serial.println("Updating PID...");
  #endif
  float humidity_error = vals->target_humidity - vals->est_humidity;
  float airflow_error = vals->target_airflow - vals->current_airflow;

  float p_humidity_error = humidity_error * kph;
  float p_airflow_error = airflow_error * kpa;

  vals->target_plate_temp = p_humidity_error;
  //vals->fan_pwm = p_airflow_error;


  #ifdef DEBUG
  char serial_buff[100];
  sprintf(serial_buff, "DEBUG: Humidity error is %2.2f", humidity_error);
  Serial.println(serial_buff);
  // serial_buff = "";
  sprintf(serial_buff, "DEBUG: Airflow error is %2.2f", airflow_error);
  Serial.println(serial_buff);
  // serial_buff = "";
  sprintf(serial_buff, "DEBUG: Humidity P*Error is %2.2f", p_humidity_error);
  Serial.println(serial_buff);
  // serial_buff = "";
  sprintf(serial_buff, "DEBUG: Airflow P*Error %2.2f", p_airflow_error);
  Serial.println(serial_buff);
  // serial_buff = "";
  #endif
}

void execute(StateVals *vals)
{
  #ifdef DEBUG
  Serial.println("Executing...");
  #endif
  if(vals->pwr_state)
  {
    if(vals->vapor_temp >= vals->target_temp - MIN_DELTA_T && vals->is_over_temp_flag == 0) // Checks if vapor temp is over nearinhose_ping temp and activates overtempflag;
    {
      //vals->is_over_temp_flag = true;
    }
    else if (vals->vapor_temp < vals->target_temp - MIN_DELTA_T && vals->is_over_temp_flag) // Checks if vapor temp is lower than nearing temp and deactivate overtempflag
    {
      //vals->is_over_temp_flag = false;
    }

    // if(!vals->over_temp_flag && vals->plate_relay_cmd)
    if(vals->plate_relay_cmd)
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
    analogWrite(HOSE_PIN, 25/*vals->hose_pwm*/);
  }
  else
  {
    analogWrite(FAN_PIN, 0);
    analogWrite(HOSE_PIN, 0);
    digitalWrite(PLATE_RELAY_PIN, HIGH);
    vals->plate_relay_state = false;
  }
  
}

void read_dht(StateVals *vals)
{
  #ifdef DEBUG
  Serial.println("Reading DHT...");
  #endif
  float humidity, temp; 
  humidity = dht.readHumidity();
  temp = dht.readTemperature();
  Serial.println(humidity);
  Serial.println(temp);
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
  #ifdef DEBUG
  Serial.println("Reading thermistor...");
  #endif
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
  #ifdef DEBUG
  Serial.println("Reading flow...");
  #endif
  float TMP_Therm_ADunits = analogRead(WIND_THERM_PIN);
  float RV_Wind_ADunits = analogRead(WIND_SPEED_PIN);
  vals->adc_flow_t = TMP_Therm_ADunits;
  vals->adc_flow_v = RV_Wind_ADunits;
  float x = RV_Wind_ADunits;
  float y = TMP_Therm_ADunits;
  static float x_prom,y_prom,sensor_airspeed;


  static uint8_t speed_num = 0;
  static float x_array[10] {0,0,0,0,0,0,0,0,0,0};
  static float y_array[10] {0,0,0,0,0,0,0,0,0,0};
  
  x_array[speed_num] = x;
  y_array[speed_num] = y;
  speed_num++; 

 
  if (speed_num > 10)
  {
    x_prom = arr_average(x_array, sizeof(x_array));
    y_prom = arr_average(y_array, sizeof(y_array));
    sensor_airspeed =  1.133423908e-4f * x_prom*x_prom - 1.159148562e-4f * x_prom*y_prom -  7.96225819e-6f * y_prom*y_prom -  6.244728852e-2f * x_prom + 8.898163594e-2f * y_prom - 13.26006647;
    speed_num = 0;
  }

  
  
  if (sensor_airspeed <  0)
  {
    sensor_airspeed = 0;
  }
  vals->current_airspeed = sensor_airspeed;
  vals->current_airflow = vals->current_airspeed * ((3.1415/4) * pow(DIAMETER ,2)) * 60000;
  
}

void read_flow_old(StateVals *vals) 
{
  #ifdef DEBUG
  Serial.println("Reading flow...");
  #endif
  float TMP_Therm_ADunits = analogRead(WIND_THERM_PIN);
  float RV_Wind_ADunits = analogRead(WIND_SPEED_PIN);
  vals->adc_flow_t = TMP_Therm_ADunits;
  vals->adc_flow_v = RV_Wind_ADunits;
  float x = RV_Wind_ADunits;
  float y = TMP_Therm_ADunits;
  float sensor_airspeed;
  
  sensor_airspeed = 2.139572236e-5*(x*x)+2.16862434e-4*(x*y)-3.59876476e-4*(y*y)-1.678691211e-1*x+3.411792421e-1*y - 61.07186374; 

  if (sensor_airspeed < 0)
  {
    sensor_airspeed = 0;
  }

  static uint8_t speed_num = 0;
  static float speed_array[2];
  static bool is_init = 0;
  if (!is_init)
    {
      for(int i = 0; i<(sizeof(speed_array)/sizeof(speed_array[0])); i++)
      {
        speed_array[i] = 0;
      }
      is_init = true;
    }
  speed_array[speed_num] = sensor_airspeed;
  speed_num++; 

  if (speed_num > 2)
  {
    speed_num = 0;
  }

  vals->current_airspeed = arr_average(speed_array, sizeof(speed_array));
  vals->current_airflow = vals->current_airspeed * ((3.1415/4) * pow(DIAMETER ,2)) * 60000;

}

void read_encoder_button(StateVals *vals, TempTarget *target)
{
  button1.Update();
  if(button1.clicks == 1 && (vals->is_config_mode || vals->is_debug_mode)) 
  {
    vals->button_counter++;
    target->is_init_encoder_position = 1;
  }
  else if(button1.clicks == -1 )
  {
    //Screen state machine
    if(vals->is_main_menu)
    {
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
    }
    else if(vals->is_debug_mode)
    {
      vals->is_config_mode = 0;
      vals->is_main_menu = 1;
      vals->is_debug_mode = 0;
    }
    // vals->current_beep.beep_id = vals->current_beep.beep_id + 2;
    // vals->current_beep.beep_type=BEEP_TWICE;
    vals->current_beep.beep_id++;
    vals->current_beep.beep_type=BEEP_ONCE;
  }
  else if(button1.clicks == 2 && vals->is_main_menu)
  {
      vals->is_config_mode = 0;
      vals->is_main_menu = 0;  
      vals->is_debug_mode = 1;
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
      case DOWN_RIGHT:
        lcd.setCursor(POSX4,POSY4);     
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
  #ifdef DEBUG
  Serial.println("Updating LCD...");

  #endif
  
  static uint32_t value_encoder;
  
  //Rango de los cases
  static char cases[4] = {11,101,41,4}; //3rd value prev 41isnan

  static char lcd_st[3][4] = {"OFF","0N "};

  //Define range of values for each variable
  for(int i=0;i<sizeof(target->range_temp);i++)
  {
    target->range_temp[i] = i+28;
  }
  for(int i=0;i<sizeof(target->range_rh);i++)
  {
    target->range_rh[i] = i;
  }
  for(int i=0;i<sizeof(target->range_v);i++)
  {
    target->range_v[i] = i;
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
              value_encoder = i;
              break;  
            }
          }
          break;
        case TOP_RIGHT:
          for(int i=0;i<=sizeof(target->range_rh);i++)
          {
            if(target->target_humidity==target->range_rh[i])
            {
              value_encoder = i;
              break;  
            }
          }
          break;
        case DOWN_LEFT:
          for(int i=0;i<=sizeof(target->range_v);i++)
          {
            if(target->target_v==target->range_v[i])
            {
              value_encoder = i;
              break;  
            }
          }
          break;
        case DOWN_RIGHT:
        for(int i=0;i<=sizeof(target->range_st);i++)
        {
          if(target->target_st==target->range_st[i])
          {
            value_encoder = i;
            break;  
          }
        }
          break;   
        default:
          break;
      }
      encoder.setPosition((cases[vals->button_counter%POSIBLE_POSITIONS])*100+value_encoder);
    }
    //Handle Encoder in CONFIG MODE
    target->encoder_position = ((encoder.getPosition()+2000*cases[vals->button_counter%POSIBLE_POSITIONS]))%cases[vals->button_counter%POSIBLE_POSITIONS];

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
      case DOWN_RIGHT:
        target->target_st = target->range_st[target->encoder_position];     
        break;    
      }
    sprintf(target->buffer, " T:%2dC  RH:%3d%%  V:%2dL/min  %c%c%c ", target->target_temp,target->target_humidity,target->target_v,lcd_st[target->target_st][0],lcd_st[target->target_st][1],lcd_st[target->target_st][2]);

  lcd_buffer_write(&target_vals);

}

void write_main_menu(StateVals *vals, TempTarget *target)
{
  if(vals->pwr_state) 
  {
    sprintf(target->buffer, "%c%2d%cC   %c%3d%%   V:%2dL/min  ON   ", byte(0), (int)vals->vapor_temp, byte(2), byte(1), (int)vals->vapor_humidity, (int)vals->current_airflow);
  }
  else
  {
    sprintf(target->buffer, "%c%2d%cC   %c%3d%%   V:%2dL/min  OFF  ", byte(0), (int)vals->vapor_temp, byte(2), byte(1), (int)vals->vapor_humidity, (int)vals->current_airflow);
  } 
  lcd_buffer_write(&target_vals);
}

void write_debug_menu(StateVals *vals, TempTarget *target)
{

  switch (vals->button_counter%POSIBLE_POSITIONS)
      {
      case TOP_LEFT:
        sprintf(target->buffer, "%c CMD:%2d  %4d   Rst:%d  DC:%3d %c", byte(3),(int)vals->plate_relay_cmd,millis()%PERIODO, (int)vals->plate_relay_state, (int)vals->duty_cycle,byte(3));
        break;
      case TOP_RIGHT:
        sprintf(target->buffer, "%c Flow:%2dL/min rPWM:%3d PWM:%3d%c",byte(3),(int)vals->current_airflow,(int)vals->initial_target_pwm,(int)vals->fan_pwm,byte(3)); // overtempplat ,cuando el termistor fuerza que se apague
        break;
      case DOWN_LEFT:
        sprintf(target->buffer, "%c ADC_THERM:%3d PL_T:%2d OVR_T:%d",byte(3),(int)vals->adc_therm, (int)vals->plate_temp, (int)vals->is_over_temp_flag ); //adC TERMISTOR,  temp plato, velocidad,
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

void init_encoder_position()
{

}

void lcd_buffer_write(TempTarget *target)
{
  #ifdef DEBUG
  Serial.print("LCD: ");
  Serial.println(buffer);
  #endif
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
  if(current_beep.beep_type == NO_BEEP && current_beep.beep_id != prev_beep_id)
  {
    digitalWrite(BUZZER_PIN, LOW);
    prev_beep_id = current_beep.beep_id;
  }
  else if (current_beep.beep_type == BEEP_ONCE)
  {
    if(current_beep.beep_id != prev_beep_id)
    {
      digitalWrite(BUZZER_PIN, HIGH);
      beep_once_timeout = millis() + BEEP_ONCE_DURATION;
      prev_beep_id = current_beep.beep_id;
    }

    else if(millis()>beep_once_timeout)
    {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
  else if (current_beep.beep_type == BEEP_TWICE)
  {
    if((current_beep.beep_id != prev_beep_id) && BUZZER_PIN==0)
    {
      digitalWrite(BUZZER_PIN, HIGH);
      beep_once_timeout = millis() + BEEP_ONCE_DURATION;
      prev_beep_id++;
    }

    else if(millis()>beep_once_timeout)
    {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
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
