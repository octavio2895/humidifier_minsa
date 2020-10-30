 
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
 void sensirion_begin()
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.



 Contact: octavio280995@gmail.com

*/

#include <Arduino.h>
#include <SimpleDHT.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <ClickButton.h>
#include <math.h>
#include "gasboard7500E.h"
#include <HardwareSerial.h>
#include <DallasTemperature.h>
#include <OneWire.h> 

#ifdef O2SENSE_NEED_METADATA
bool has_sernum = false;
bool has_vernum = false;
#endif

HardwareSerial Serial3(PB11, PB10);

// #define OUTPUT_BUFFER_SIZE 32
// char output_buffer[OUTPUT_BUFFER_SIZE];

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
#define ONE_WIRE_BUS            PB8

// Behaviour characteristics
#define HOSE_CONVERSION_FACTOR  .01 // Converts pwm output to rms volts in output.
#define PLATE_HISTERESIS        5 // Plate deadzone size.
#define MIN_DELTA_T             2
#define LCD_ROWS                2     // Quantity of lcd rows.
#define LCD_COLUMNS             16    // Quantity of lcd columns.
#define LCD_SPACE_SYMBOL        0x20  //Space symbol from lcd ROM, see p.9 of GDM2004D datasheet.
// #define DEBUG

// Timming
#define FLOW_UPDATE_DELAY       20
#define TERMISTOR_UPDATE_DELAY  10
#define DHT_UPDATE_DELAY        2005
#define FLOW_ESTIMATION_DELAY   100
#define BANGBANG_CONTROL_DELAY  100
#define PID_FAN_CONTROL_DELAY   50
#define PID_UPDATE_DELAY        100
#define EXECUTE_DELAY           10
#define ENCODER_UPDATE_DELAY    10
#define SCREEN_UPDATE_DELAY     500
#define BEEP_UPDATE_DELAY       10
#define BEEP_ONCE_DURATION      300
#define ALARM_UPDATE_DELAY       10
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
#define KP_FAN                  100
#define KD_FAN                  0
#define KI_FAN                  0.04
#define KP_SMC                  0.01
#define MAX_PWM_MODIFIER        500

//Cursor Locations
#define POSIBLE_POSITIONS       5
#define POSX1                   3
#define POSY1                   0
#define POSX2                   12
#define POSY2                   0
#define POSX3                   1
#define POSY3                   1
#define POSX4                   10
#define POSY4                   1
#define POSX5                   14
#define POSY5                   1

//Alarm critical values
#define MAX_PLATE_TEMP          150
#define DELTA_V                 26//101: mapped mode; 41: L/min mode
#define CRC_POLYNOMIAL          (0x0131)


// Globlas
const float zeroWindAdjustment =  .2;
uint32_t next_flow_update, next_termistor_update, next_dht_update, next_flow_estimation, next_bangbang_control, next_pidfan_control, next_pid_update, next_execute, next_encoder_update, next_screen_update, next_beep_update, next_alarm_update, next_o2_update, next_ds_update, next_hose_bb_update, get_vtemp_update;
float kpa, kph;
byte print_command[] = {0x11,0x1,0x1,0xED};

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
  float target_plate_temp = 0; // Target plate temperature. °C
  float current_airflow = 0; // Air volumetric flow rate. Lts/min
  float current_airspeed = 0; // Air speed. m/s
  int16_t fan_pwm = 0; // Fan PWM duty cycle.
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
  bool is_possible_condition = 0;

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
Thermistor* thermistor;
SimpleDHT22 dht22(DHTPIN);
RotaryEncoder encoder(PIN_A, PIN_B, BUTTON);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
ClickButton button1(BUTTON, LOW, CLICKBTN_PULLUP);
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);


// Prototypes
void read_flow(StateVals *vals);
void read_thermistor(StateVals *vals);
void estimate_flow(StateVals *vals);
void control_PD_humidity(StateVals *vals);
void control_SMC_temp (StateVals *vals);
void control_PID_Fan(StateVals *vals);
void update_pid(StateVals *vals);
void execute(StateVals *vals);
void read_dht(StateVals *vals);
void read_encoder_button(StateVals *vals, TempTarget *target);
void write_main_menu(StateVals *vals, TempTarget *target);
void write_config_menu(StateVals *vals, TempTarget *target);
void write_debug_menu(StateVals *vals, TempTarget *target);
float get_density(float temp);
void lcd_buffer_write(TempTarget *target);
void beep_manager(StateVals *vals);
void beep_creator(StateVals *vals, BeepType);
void encoderButtonISR();
void encoderISR();
float arr_average(float *arr, uint16_t size);
float integral_control(float *i_control, uint16_t isize);
void manage_cursor(StateVals *vals);
void alarm_manager(StateVals *vals, Alarms *alarm);
void flow_to_PWM(StateVals *vals);
void read_o2(StateVals *vals, TempTarget *target);
void check_fio2_flow(StateVals *vals, TempTarget *target);
void read_ds18b20(StateVals *vals);
void hose_bang_bang(StateVals *vals);
void get_target_temperature(StateVals *vals);
void print_o2_screen(StateVals *vals, TempTarget *target);
void calculate_flow_sensirion(StateVals *vals);
void sensirion_restart();
void sensirion_begin();
uint8_t calcCRC(char buff[], int num);



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
  sensirion_begin();
  encoder.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), encoderISR, CHANGE);
  
  #ifdef O2SENSE_NEED_METADATA
  const uint8_t cmd_vernum[] = {O2SENSE_CMD_VERSIONNUMBER};
  const uint8_t cmd_sernum[] = {O2SENSE_CMD_SERIALNUMBER};
  #endif

  o2sens_init();
  Serial3.begin(9600,SERIAL_8N2);

  // Start up the library 
  sensors.begin();
  sensors.setWaitForConversion(false);

  //Boot up sequence
  analogWrite(FAN_PIN, 0);
  digitalWrite(PLATE_RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  sprintf(target->buffer, "Humidifier v1.00FABLAB-MINSA-UTP");
  lcd_buffer_write(&target_vals);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  sprintf(target->buffer, "PROTOTIPO  ALPHAUSO EXPERIMENTAL");
  lcd_buffer_write(&target_vals);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(150);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(150);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
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
      calculate_flow_sensirion(&state_vals);
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
}

void estimate_flow(StateVals *vals)
{
  #ifdef DEBUG
  Serial.println("Estimating flow...");
  #endif
  vals->vapor_abs_humidity = (6.112*exp((17.67*vals->vapor_temp)/(vals->vapor_temp + 243.5))*(vals->vapor_humidity)*2.1674)/(273.15+vals->vapor_temp); //[g/m³]
  float entry_density = get_density(vals->vapor_temp);
  float exit_density = get_density(vals->after_hose_temp);
  vals->est_abs_humidity = vals->vapor_abs_humidity * exit_density/entry_density; //water_mass_flow/exit_airflow;
  vals->est_humidity = ((273.15+vals->after_hose_temp)*vals->est_abs_humidity)/(6.112*exp((17.67*vals->after_hose_temp)/(vals->after_hose_temp + 243.5))*2.1674);

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
  static float error_humidity_old, error_air_temp;
  static float delta_error_humidity_current;
  float current_step = millis()%PERIODO;
  static float old_millis;

  vals->clock = current_step;
  //Read error values
  error_humidity_current = vals->target_humidity - vals->vapor_humidity;
  delta_error_humidity_current = (error_humidity_current - error_humidity_old)/(millis()-old_millis);
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

  //TODO: change into a bool alarm
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


    
  if(pendiente < KP_SMC*error_temp)
  {
    //vals->plate_relay_cmd = 1;
    //vals->duty_cycle = 30;
  }
  else
  {
    //vals->plate_relay_cmd = 0;
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
  static uint32_t  pwm_modifier, pwm_const;

  
  static uint16_t old_millis, integral_num = 0;
  static float integral_array[180], old_target_flow;
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
  
  //Calculate PWM reference value and update error
  flow_to_PWM(&state_vals);
  old_target_flow = vals->target_airflow;

  if(integral_num > 180)
  {
    integral_num = 0;
  }

  //Calculate PWM modifier
  pwm_modifier = (KP_FAN * error_airflow_current + KI_FAN * integral_error_airflow_current +  KD_FAN * delta_error_airflow_current);

  vals->fan_pwm = pwm_modifier;

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
  #ifdef DEBUG
  Serial.println("Reading DHT...");
  #endif
  float humidity, temp;
  int err = SimpleDHTErrSuccess; 
  if ((err = dht22.read2(&temp, &humidity, NULL)) != SimpleDHTErrSuccess) 
  {
    //Serial.print("Read DHT22 failed, err="); Serial.println(err);delay(2000);
    return;
  }
  // humidity = dht.readHumidity();
  // temp = dht.readTemperature();
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

 
  if (speed_num > 0)
  {
    x_prom = x;//arr_average(x_array, sizeof(x_array));
    y_prom = y;//arr_average(y_array, sizeof(y_array));
    //sensor_airspeed =  1.133423908e-4f * x_prom*x_prom - 1.159148562e-4f * x_prom*y_prom -  7.96225819e-6f * y_prom*y_prom -  6.244728852e-2f * x_prom + 8.898163594e-2f * y_prom - 13.26006647;
    //sensor_airspeed = 8.425983316e-4f  * x_prom*x_prom - 1.175031223e-3f * x_prom*y_prom + 4.294234517e-4f * y_prom*y_prom - 1.268141388e-1f * x_prom + 8.589904629e-2f * y_prom - 4.817979033f;
    //sensor_airspeed =  2.07278786e-7 *x_prom*x_prom*x_prom*x_prom - 1.258160808e-6 *x_prom*x_prom*x_prom*y_prom + 2.329122632e-6 *x_prom*x_prom*y_prom*y_prom - 1.684204298e-6 *x_prom* y_prom*y_prom*y_prom + 4.161528134e-7 *y_prom*y_prom*y_prom*y_prom + 1.865690411e-4 *x_prom*x_prom*x_prom - 2.397625704e-4 *x_prom*x_prom*y_prom - 1.05055706e-4 *x_prom*y_prom*y_prom + 1.293264445e-4 *y_prom*y_prom*y_prom - 1.107078016e-1 *x_prom*x_prom + 2.173253746e-1 *x_prom* y_prom - 7.790431822e-2 *y_prom*y_prom + 5.091561295 *x_prom - 17.07396366 *y_prom + 1779.975948;
    sensor_airspeed =  -1.056891481e-5 *x_prom*x_prom*x_prom + 3.245687769e-5 *x_prom*x_prom * y_prom - 3.605536507e-5 *x_prom *y_prom*y_prom + 1.138111633e-5 *y_prom*y_prom*y_prom + 2.436108257e-3 *x_prom*x_prom - 1.102028777e-3 *x_prom*y_prom + 3.695040756e-3 *y_prom*y_prom - 1.163719965 *x_prom - 1.820651701 *y_prom + 580.7883239;
    speed_num = 0;
  }

  
  
  if (sensor_airspeed <  0)
  {
    sensor_airspeed = 0;
  }
  vals->current_airspeed = sensor_airspeed;
  if(vals->pwr_state)
  {
    vals->current_airflow = sensor_airspeed;//vals->current_airspeed * ((3.1415/4) * pow(DIAMETER ,2)) * 60000;
  }
  else
  {
    vals->current_airflow = 0;
  }
  
}

void read_o2(StateVals *vals, TempTarget *target)
{
    // at least 1 byte from UART arrived
  if(vals->o2_test%100>19)
  {
    //o2sens_init();
    //vals->o2_test = 0;
    //o2sens_clearNewData(); // clear the new packet flag
  }
  //vals->o2_buffer = o2sens_getRawBuffer();

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
      vals->is_possible_condition = 0; // Set to 0 when using curve control mode
      check_fio2_flow(vals,target);
      
      if(vals->is_possible_condition)
      {
        vals->is_config_mode = 0;
        vals->is_main_menu = 1;
        vals->is_debug_mode = 0;
        vals->target_temp = target->target_temp;
        vals->target_humidity = target->target_humidity;
        vals->target_airflow = target->target_v;
        vals->pwr_state = target->target_st;
        vals->target_fio2 = target->target_fio2;
        if(vals->pwr_state) 
        {
          print_o2_screen(vals,target);
        }
      }
      else
      {
        beep_creator(vals,BEEP_THRICE);
      }
      

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
  #ifdef DEBUG
  Serial.println("Updating LCD...");

  #endif
  
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
    //Reference Value
    
    //Sets a encoder position so that it matchs the previous target value selected.
    //encoder.setPosition((cases[vals->button_counter%POSIBLE_POSITIONS])*100*value_encoder);

  }
  //Change the target value using the encoder value as referenc .
  
 
  value_encoder = value_encoder + (int)((encoder.getPosition()/2-last_enc/2));
  target->encoder_position = (value_encoder)%(cases[vals->button_counter%POSIBLE_POSITIONS]);
  //target->encoder_position = ((encoder.getPosition()/2/*+2000*cases[vals->button_counter%POSIBLE_POSITIONS]*/))%cases[vals->button_counter%POSIBLE_POSITIONS];
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
  sprintf(target->buffer, "T:%2d%cC RH:%3d%%  %2dLPM %c%c:%2d%% %c%c%c", target->target_temp,byte(2),target->target_humidity,target->target_v,byte(4), byte(5),target->target_fio2,lcd_st[target->target_st][0],lcd_st[target->target_st][1],lcd_st[target->target_st][2]);

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

void check_fio2_flow(StateVals *vals, TempTarget *target)
{
  //Family of curves
    //x = PWM; y = Flow; z = FiO2
  float y[9];
  float z[9];
  float x;

  
  for(int j=0;j<=8;j++)
  {
    for(int i=0;i<=100;i++)
    {
      x = (100-i);
      //O2 -> 0 LPM
      y[0] = 1.582528729e-4 *x*x*x - 3.774840906e-2 *x*x + 3.038936694 *x - 56.56627272;
      z[0] = 21;
      //O2 -> 5 LPM
      y[1] =  -4.378002164e-5 *x*x*x + 4.70444873e-3 *x*x + 2.225531197e-1 *x + 1.940022195;
      z[1] =  -1.355925417e-4 *x*x*x + 2.997287197e-2 *x*x - 2.217434699 *x + 84.24365604;
      //O2 -> 10 LPM
      y[2] = -4.231355855e-5 *x*x*x + 4.949013966e-3 *x*x + 1.670944547e-1 *x + 3.914332489;
      z[2] = -7.076545953e-5 *x*x*x + 1.714246045e-2 *x*x - 1.468597153 *x + 76.90164481;
      //O2 -> 15 LPM
      y[3] =  -4.932041583e-5 *x*x*x + 6.055963078e-3 *x*x + 9.825068594e-2 *x + 6.328023212;
      z[3] = 3.269801342e-3 *x*x - 6.756654765e-1 *x + 70.94783946;
      //O2 -> 20 LPM
      y[4] = -5.285194259e-5 *x*x*x + 7.2383787e-3 *x*x - 1.109175469e-2 *x + 9.589354885;
      z[4] = 3.089410837e-5 *x*x*x - 3.420035767e-3 *x*x - 2.139644891e-1 *x + 66.62772342;
      //O2 -> 25 LPM
      y[5] =  -6.510215051e-5 *x*x*x + 9.225284433e-3 *x*x - 1.154743871e-1 *x + 12.75113624;
      z[5] =  3.74394032e-5 *x*x*x - 5.478986346e-3 *x*x - 8.343020126e-3 *x + 65.02641395;
      //O2 -> 30 LPM
      y[6] = -7.056140714e-5 *x*x*x + 9.695200151e-3 *x*x - 1.200246868e-1 *x + 15.39647849;
      z[6] =  4.978629545e-5 *x*x*x - 7.90192228e-3 *x*x + 1.817315045e-1 *x + 61.9323571;
      //O2 -> 35 LPM
      y[7] = -5.91456245e-5 *x*x*x + 7.984263482e-3 *x*x - 6.686070826e-2 *x + 18.03295746;
      z[7] = 3.621884778e-5 *x*x*x - 5.712294935e-3 *x*x + 1.087590125e-1 *x + 62.22482041;
      //O2 -> 40 LPM
      y[8] = -6.476451903e-5 *x*x*x + 9.087696887e-3 *x*x - 1.324249364e-1 *x + 21.1368989;
      z[8] = 2.691775272e-5 *x*x*x - 4.215288196e-3 *x*x + 7.232841773e-2 *x + 61.56778182;

      if(target->target_v <= y[j] + 2 && target->target_v >= y[j] - 2)
      {
        if(target->target_fio2 <= z[j] + 2 && target->target_fio2 >= z[j] - 2)
        {
          vals->is_possible_condition = 1;
          vals->set_o2_flow = j*5;
          vals->set_PWM = x;
          return;
        }
      }
    } 
  }
}

void print_o2_screen(StateVals *vals, TempTarget *target)
{
  sprintf(target->buffer, "AJUSTAR VALVULA DE %c A %2d L/MIN  ",byte(5), (int)vals->set_o2_flow);
  lcd_buffer_write(&target_vals);
  next_screen_update = next_screen_update + PRINT_02_DELAY;
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
  //Default value is 35°C because I say so
  vals->vapor_target_temp = 33;

}


void calculate_flow_sensirion(StateVals *f) //TODO: Check for CRC error
{
  char c[3] = {0 ,0 ,0};
  int i = 0;
  uint16_t data = 0;

  Wire.requestFrom(64, 3);
  uint32_t timeout = millis() + 100;
  while(!Wire.available())
  {
    delay(1);
    if(millis()>timeout)
    {
      f->current_airflow = 66;
      return;
    }
  }
  if(Wire.available() > 3)
  {
    while(Wire.available())
    {
      Wire.read();
    }
  }
  while (Wire.available()) 
  {
    c[i++] = Wire.read(); //TODO: Blocking!
  }
  data = (c[0] << 8) | c[1];
  uint8_t crc = calcCRC(c, 2);
  if(crc != c[2])
  {
    f->current_airflow = 88;
    return;
  }
  f->current_airflow = ((double)(data-32768))/(120);
}

uint8_t calcCRC(char buff[], int num) 
{
    uint8_t crc = 0;
    for (uint8_t idx = 0; idx < num; idx++) 
    {
      crc ^= buff[idx];
      for (uint8_t loop = 8; loop > 0; --loop) 
      {
        if (crc & 0x80) crc = (crc << 1) ^ CRC_POLYNOMIAL;
        else crc = (crc << 1);
      }
    }
    return(crc);
}

void sensirion_begin()
{
  sensirion_restart();
  byte cmd[2] = {0x10, 0x00};
  Wire.begin();
  Wire.beginTransmission(64);
  Wire.write(cmd, 2);
  Wire.endTransmission();
  delay(200);
}

void sensirion_restart()
{
  byte cmd[2] = {0x20, 0x00};
  Wire.begin();
  Wire.beginTransmission(64);
  Wire.write(cmd, 2);
  Wire.endTransmission();
  delay(200); //TODO : Blocking!
}