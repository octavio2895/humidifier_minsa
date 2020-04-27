 
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
// Physical properties
#define DIAMETER                0.0177
#define REFERENCE_RESISTANCE    100000
#define NOMINAL_RESISTANCE      100000
#define NOMINAL_TEMPERATURE     23
// #define B_VALUE              870
#define B_VALUE                 3950


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
#define SCREEN_UPDATE_DELAY     10
#define BEEP_UPDATE_DELAY       10
#define BEEP_ONCE_DURATION      300

//PID Values
#define KP_BB                   60
#define KD_BB                   60
#define PERIODO                 2000
#define KP_FAN                  30
#define KD_FAN                  0//35
#define KI_FAN                  17

// Globlas
const float zeroWindAdjustment =  .2;
uint32_t next_flow_update, next_termistor_update, next_dht_update, next_flow_estimation, next_bangbang_control, next_pidfan_control, next_pid_update, next_execute, next_encoder_update, next_screen_update, next_beep_update;
float kpa, kph;

//Globals raras de Jahir
//char* background(uint16_t,uint16_t,uint16_t);
uint32_t next_jahir_screen_update = 0;
volatile uint16_t buttonCounter = 0;
static bool debug = 0;

// Structs

enum BeepType
{
  NO_BEEP,
  BEEP_ONCE,
  BEEP_TWICE,
  BEEP_TRRICE,
  BEEP_CONINUOS
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
  bool pwr_state = 0; // Machine on or off.next_dht_update
  bool plate_relay_state = 0; // Plate on or off.
  bool plate_relay_cmd = 0;
  bool over_temp_flag = 0; // The flag is set whenever the vapor temp is getting too close to output temp.
  Beeps current_beep;
  uint16_t adc_therm = 0;
  float them_resistance = 0;
  float duty_cycle = 0;
  float fan_duty_cycle = 0;
}state_vals;

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
void control_bangbang(StateVals *vals,uint32_t millis);
void control_PID_Fan(StateVals *vals,uint32_t millis);
void update_pid(StateVals *vals);
void execute(StateVals *vals);
void read_dht(StateVals *vals);
void read_encoder(StateVals *vals);
void screen_manager(StateVals *vals, uint32_t millis);
void screen_debug_manager(StateVals *vals, uint32_t millis);
float get_density(float temp);
void lcd_buffer_write(char buffer[32], uint16_t sizeof_buffer);
void beep_manager(StateVals *vals);
void encoderButtonISR();
void encoderISR();
byte i2c_scanner();
float arr_average(float *arr, uint16_t size);
float integral_control(float *i_control, uint16_t isize);
//void lcd_buffer_write_debug(char buffer [200],uint16_t buffer_size,uint16_t view_port_init);



void setup() 
{
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
  lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS);
  lcd_buffer_write("Humidifier v0.01FABLAB-MINSA-UTP", 32);
  encoder.begin();
  dht.begin();
  attachInterrupt(digitalPinToInterrupt(PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), encoderISR, CHANGE); 
  delay(2000);
 /* button1.Update();
  if(button1.clicks =! 0)
  {
    debug = 1;
  }*/
}

void loop() 
{
  if (millis() > next_beep_update)
  {
    beep_manager(&state_vals);
    next_beep_update = millis() + BEEP_UPDATE_DELAY;
  }
  if (millis() > next_flow_update) 
  {
    read_flow(&state_vals);
    next_flow_update = millis() + FLOW_UPDATE_DELAY;
  }
  if (millis() > next_termistor_update) 
  {
    read_thermistor(&state_vals);
    next_termistor_update = millis() + TERMISTOR_UPDATE_DELAY;
  }
  if (millis() > next_dht_update) 
  {
    read_dht(&state_vals);
    next_dht_update = millis() + DHT_UPDATE_DELAY;
  }
  if (millis() > next_flow_estimation) 
  {
    estimate_flow(&state_vals);
    next_flow_estimation = millis() + FLOW_ESTIMATION_DELAY;
  }
  if (millis() > next_bangbang_control) 
  {
    control_bangbang(&state_vals, millis());
    next_bangbang_control = millis() + BANGBANG_CONTROL_DELAY;
  }
  if (millis() > next_pidfan_control) 
  {
    control_PID_Fan(&state_vals, millis());
    next_pidfan_control = millis() + PID_FAN_CONTROL_DELAY;
  }
  if (millis() > next_pid_update) 
  {
    update_pid(&state_vals);
    next_pid_update = millis() + PID_UPDATE_DELAY;
  }
  if (millis() > next_execute) 
  {
    execute(&state_vals);
    next_execute = millis() + EXECUTE_DELAY;
  }
  if (millis() > next_encoder_update) 
  {
    read_encoder(&state_vals);
    next_encoder_update = millis() + ENCODER_UPDATE_DELAY;
  }
  if (millis() > next_screen_update) 
  {
    if(!debug)
    {
      screen_manager(&state_vals, millis());
    }
    else
    {
      screen_debug_manager(&state_vals, millis());
    }
    
    
    next_screen_update = millis() + SCREEN_UPDATE_DELAY;
  }
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
  // serial_buff = "";
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

void control_bangbang(StateVals *vals, uint32_t millis)
{
  static float error_humidity_current;
  static float error_humidity_old;
  static float delta_error_humidity_current;
  //volatile float delta_error_humidity_old;
  //volatile float targe_humidity = 80;//vals->target_humidity;
  static uint32_t current_step = millis%PERIODO,old_millis;

  
  //Enter logic if the temperature is going up
  //if (vals->plate_temp < (target_humidity-PLATE_HISTERESIS))
  //{
    //Read error values
    error_humidity_current = vals->target_humidity - vals->vapor_humidity;
    delta_error_humidity_current = (error_humidity_current - error_humidity_old)/(millis-old_millis);

    //Write new Duty Cycle value
    vals->duty_cycle = (KP_BB * error_humidity_current + KD_BB * delta_error_humidity_current);
    
    //Overwrite old error values
    error_humidity_old = error_humidity_current;
    old_millis = millis;
    //delta_error_humidity_old = delta_error_humidity_current;

     if (vals->duty_cycle > 100)
     {
        vals->duty_cycle = 100;       
     }
     else if (vals->duty_cycle < 0)
     {
        vals->duty_cycle = 0;       
     }
      #ifdef DEBUG
    Serial.println("BANGBANG...");
    #endif
    // if (vals->plate_temp < (vals->target_humidity-PLATE_HISTERESIS))vals->plate_relay_cmd = true; // Under lower range, activate.
    // else if (vals->plate_temp > (vals->plate_temp+PLATE_HISTERESIS))vals->plate_relay_cmd = false; // Over upper range, deactivate.
    

    /*if (vals->plate_temp < (target_humidity-PLATE_HISTERESIS))vals->plate_relay_cmd = true; // Under lower range, activate.
    else if (vals->plate_temp > (target_humidity+PLATE_HISTERESIS))vals->plate_relay_cmd = false; // Over upper range, deactivate.*/

    if (current_step < (vals->duty_cycle/100)*PERIODO)
    {
      vals->plate_relay_cmd = true;
    }
    else
    {
      vals->plate_relay_cmd = false;
      vals->plate_relay_state = false;
    }
    
    
    /*  if (vals->duty_cycle<1)
    {
      vals->plate_relay_cmd = false;
    }*/
  //}
  /*else*/ 
    if (vals->vapor_humidity > vals->target_humidity || vals->plate_temp > 130)
    {
      vals->plate_relay_cmd = false;
    }

}

void control_PID_Fan(StateVals *vals, uint32_t millis)
{
  static float error_airflow_current;
  static float error_airflow_old;
  static float delta_error_airflow_current, integral_error_airflow_current;
  static uint32_t old_millis;

  static float delta_time  = (millis()-old_millis);
  static uint8_t integral_num = 0;
  static float integral_array[256];
  static bool init = 0;

  //Enter logic if the temperature is going up
  //if (vals->plate_temp < (target_humidity-PLATE_HISTERESIS))
  //{
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
    integral_array[integral_num++] = error_airflow_current*delta_time;
    integral_error_airflow_current = integral_control(integral_array, sizeof(integral_array));
    
    //Write new Duty Cycle value    
    vals->fan_duty_cycle = (KP_FAN * error_airflow_current + KI_FAN * integral_error_airflow_current +  KD_FAN * delta_error_airflow_current);
    
    //Overwrite old error values
    error_airflow_old = error_airflow_current;
    old_millis = millis();
    //delta_error_humidity_old = delta_error_humidity_current;

     if (vals->fan_duty_cycle > 256)
     {
        vals->fan_duty_cycle = 256;       
     }
     else if (vals->fan_duty_cycle < 0)
     {
        vals->fan_duty_cycle = 0;       
     }
      #ifdef DEBUG
    Serial.println("BANGBANG...");
    #endif
    // if (vals->plate_temp < (vals->target_humidity-PLATE_HISTERESIS))vals->plate_relay_cmd = true; // Under lower range, activate.
    // else if (vals->plate_temp > (vals->plate_temp+PLATE_HISTERESIS))vals->plate_relay_cmd = false; // Over upper range, deactivate.
    

    /*if (vals->plate_temp < (target_humidity-PLATE_HISTERESIS))vals->plate_relay_cmd = true; // Under lower range, activate.
    else if (vals->plate_temp > (target_humidity+PLATE_HISTERESIS))vals->plate_relay_cmd = false; // Over upper range, deactivate.*/

 

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
  vals->fan_pwm = p_airflow_error;


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
    if(vals->vapor_temp >= vals->target_temp - MIN_DELTA_T && vals->over_temp_flag == 0) // Checks if vapor temp is over nearinhose_ping temp and activates overtempflag;
    {
      vals->over_temp_flag = true;
    }
    else if (vals->vapor_temp < vals->target_temp - MIN_DELTA_T && vals->over_temp_flag) // Checks if vapor temp is lower than nearing temp and deactivate overtempflag
    {
      vals->over_temp_flag = false;
    }

    // if(!vals->over_temp_flag && vals->plate_relay_cmd)
    if(vals->plate_relay_cmd)
    {
      digitalWrite(PLATE_RELAY_PIN, LOW);
      vals->plate_relay_state = true;
    }

    // if(vals->over_temp_flag || !vals->plate_relay_cmd)
    if(!vals->plate_relay_cmd)
    {
      digitalWrite(PLATE_RELAY_PIN, HIGH);
      vals->plate_relay_state = false;
    }

    analogWrite(FAN_PIN, vals->fan_duty_cycle);
    analogWrite(HOSE_PIN, vals->hose_pwm);
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
  vals->plate_temp = arr_average(temp_array, sizeof(temp_array));
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
  float x = RV_Wind_ADunits;
  float y = TMP_Therm_ADunits;
  float sensor_airspeed;
  /*// float RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);
  float RV_Wind_Volts = (RV_Wind_ADunits *  0.0032226563);
  float zeroWind_ADunits = -0.0006 * ((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;
  // float zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;
  float zeroWind_volts = (zeroWind_ADunits * 0.0032226563) - zeroWindAdjustment;
  float WindSpeed_mps =  pow(((RV_Wind_Volts - zeroWind_volts) / .2300) , 2.7265) / 21.97;*/
  sensor_airspeed = 2.139572236e-5*(x*x)+2.16862434e-4*(x*y)-3.59876476e-4*(y*y)-1.678691211e-1*x+3.411792421e-1*y - 61.07186374; 

  if (sensor_airspeed < 0)
  {
    sensor_airspeed = 0;
  }
  vals->current_airspeed = sensor_airspeed;
  vals->current_airflow = vals->current_airspeed * ((3.1415/4) * pow(DIAMETER ,2)) * 60000;
  
}

void read_encoder(StateVals *vals)
{
  return;
}


void screen_manager(StateVals *vals, uint32_t millis)
{
  #ifdef DEBUG
  Serial.println("Updating LCD...");

  #endif
  static bool menu;
  static char buffer[32];
  static const char mode1[4] = {'A','B','C','D'};
  
  static const char posx[4] = {4,13,4,13};
  static const char posy[4] = {0,0,1,1};

  static char range_temp[11];
  static char range_rh[101];
  static char range_v[41];
  static char range_st[4] = {0,0,1,1};
  
  //Rango de los cases
  static char cases[4] = {11,101,41,4};

  static uint16_t x,y,target_temp,target_humidity,target_v;
  static char lcd_st[3][4] = {"OFF","0N "};
  static bool aux,target_st;

  for(int i=0;i<sizeof(range_temp);i++)
  {
    range_temp[i] = i+28;
  }
  for(int i=0;i<sizeof(range_rh);i++)
  {
    range_rh[i] = i;
  }
  for(int i=0;i<sizeof(range_v);i++)
  {
    range_v[i] = i;
  }
  

  //See if a click happened
  button1.Update();
  if(button1.clicks == 1 && menu) 
  {
    buttonCounter++;
    aux=true;
  }

  //See if long click happened
  if(button1.clicks == -1 )
  {
    menu = !menu;
    if (menu)
    {
      vals->current_beep.beep_id++;
      vals->current_beep.beep_type=BEEP_ONCE;
      buttonCounter=0;
    }
    else
    {
      vals->current_beep.beep_id++;
      vals->current_beep.beep_type=BEEP_ONCE;
      vals->target_temp = target_temp;
      vals->target_humidity = target_humidity;
      vals->target_airflow = target_v;
      vals->pwr_state = target_st;
    }
  }

  //Print Cursor IN CONFIG MODE
  if (menu) //Menu Mode
  {
    lcd.setCursor(posx[x],posy[x]);
    lcd.blink();

    //Manage Cursor
    x = (buttonCounter)%4;

    if(aux)
    {
      aux=false; 
      switch (mode1[x])
      {
        case 'A':
          for(int i=0;i<=sizeof(range_temp);i++)
          {
            if(target_temp==range_temp[i])
            {
              y=i;
              break;  
            }
          }
          break;
        case 'B':
          for(int i=0;i<=sizeof(range_rh);i++)
          {
            if(target_humidity==range_rh[i])
            {
              y=i;
              break;  
            }
          }
          break;
        case 'C':
          for(int i=0;i<=sizeof(range_v);i++)
          {
            if(target_v==range_v[i])
            {
              y=i;
              break;  
            }
          }
          break;
        case 'D':
        for(int i=0;i<=sizeof(range_st);i++)
        {
          if(target_st==range_st[i])
          {
            y=i;
            break;  
          }
        }
          break;   
        default:
          break;
      }
      encoder.setPosition((cases[x])*100+y);
    }
    //Handle Encoder in CONFIG MODE
    y = ((encoder.getPosition()+2000*cases[x]))%cases[x];

    switch (mode1[x])
      {
      case 'A':
        target_temp = range_temp[y];
        break;
      case 'B':
        target_humidity = range_rh[y];
        break;
      case 'C':
        target_v = range_v[y];
        break;
      case 'D':
        target_st = range_st[y];     
        break;    
      default:
        break;
      }
    sprintf(buffer, " T:%2dC  RH:%3d%%  V:%2dL/min   %c%c%c ", target_temp,target_humidity,target_v,lcd_st[target_st][0],lcd_st[target_st][1],lcd_st[target_st][2]);
  }
  else //Background Mode
  {

    //Print ACTUAL Values
    //if(vals->pwr_state) sprintf(buffer, "T:%dC  RH:%3d%%  V:%2dL/min   ON  ", (int)vals->vapor_temp, (int)vals->vapor_humidity, (int)vals->current_airspeed);
    //else sprintf(buffer, "T:%dC  RH:%3d%%  V:%2dL/min   OFF ", (int)vals->vapor_temp, (int)vals->vapor_humidity, (int)vals->current_airspeed);
    
    //Print Thermal resistor values
    //if(vals->pwr_state) sprintf(buffer, "T:%dC  RH:%3d%%  V:%2dL/min   ON  ", (int)vals->vapor_temp, (int)vals->vapor_humidity, (int)vals->current_airspeed);
    /*else*/ sprintf(buffer, "Air_F:%dspd Therm:%dC  FPWM:%d St:%d ", (int)vals->current_airflow, (int)vals->current_airspeed, (int)vals->fan_duty_cycle,(int)vals->plate_relay_state);
    
  }
  if(millis>next_jahir_screen_update)
  {
    lcd_buffer_write(buffer, sizeof(buffer));
    next_jahir_screen_update = millis + 300;
  }
  return;
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

/*void screen_debug_manager(StateVals *vals, uint32_t millis)
{
  static char buffer[22];
  static const char mode1[4] = {'A','B','C','D'};
  static uint16_t y;// view_port_init;
  float plate_temp = 0; // Current plat temperature. °C
  float vapor_temp = 0; // Current temperature at vapor chamber. °C 
  float est_temp = 0; // Estimated temperature after hose. °C
  float est_humidity = 0; // Estimated humidity after hose. RH%
  float vapor_abs_humidity = 0; // Current absolute humidity at vapor chamber. g/m3
  float est_abs_humidity = 0; // Estimated absolute humidity after hose. g/m3
  float target_humidity = 0; // Target plate temperature. °C
  float current_airflow = 0; // Air volumetric flow rate. Lts/min
  float current_airspeed = 0; // Air speed. m/s
  uint16_t fan_pwm = 0; // Fan PWM duty cycle.
  float vapor_humidity = 0; // Current humidity at vapor chamber. RH%
  uint16_t hose_pwm = 0; // Hose power PWM duty cycle.
  float target_temp = 0; // Target temperature after hose. °C
  float target_humidity = 0; // Target relative humidity after hose. %RH
  float target_airflow = 0; // Target air flow. Lts/min
  bool pwr_state = 0; // Machine on or off.next_dht_update
  bool plate_relay_state = 0; // Plate on or off.
  bool plate_relay_cmd = 0;
  bool over_temp_flag = 0; // The flag is set whenever the vapor temp is getting too close to output temp.
  Beeps current_beep;
  uint16_t adc_therm = 0;
  float them_resistance = 0;

  //Handle Encoder in Debug MODE
  y = encoder.getPosition();

  if (y<0)
  {
    encoder.setPosition(0);
  }
  view_port_init = encoder.getPosition();

  sprintf(buffer, "T:%dC  RH:%3d%%  V:%2dL/min   ON  ", (int)vals->plate_temp, (int)vals->vapor_temp, (int)vals->est_temp,(int)vals->est_humidity, (int)vals->vapor_abs_humidity, (int)vals->,
                                                        (int)vals->plate_temp, (int)vals->vapor_temp, (int)vals->est_temp);


  if(millis>next_jahir_screen_update)
  {
    lcd_buffer_write_debug(buffer, sizeof(buffer),view_port_init);
    next_jahir_screen_update = millis + 300;
  }
  return;
  
}*/

void lcd_buffer_write(char buffer[32], uint16_t sizeof_buffer)
{
  #ifdef DEBUG
  Serial.print("LCD: ");
  Serial.println(buffer);
  #endif
  for (int i=0; i<(sizeof_buffer/sizeof(char)); i++)
  {
    if(i<16) 
    {
      lcd.setCursor(i, 0);
      lcd.print(buffer[i]);
    }
    else 
    {
      lcd.setCursor(i-16,1);
      lcd.print(buffer[i]);
    }
  }
  return;
}

/*void lcd_buffer_write_debug(char buffer[200], uint16_t sizeof_buffer,uint16_t view_port_init)
{
  #ifdef DEBUG
  Serial.print("LCD: ");
  Serial.println(buffer);
  #endif
  for (int i=0; i<(sizeof_buffer/sizeof(char)); i++)
  {
    if(i<16) 
    {
      lcd.setCursor(i, 0);
      lcd.print(buffer[i]);
    }
    else 
    {
      lcd.setCursor(i-16,1);
      lcd.print(buffer[i]);
    }
  }
  return;
}*/

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
  return (average-7);
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