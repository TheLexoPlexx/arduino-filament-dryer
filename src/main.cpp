#include <Arduino.h>
#include "../../../../.platformio/packages/framework-arduino-avr/variants/standard/pins_arduino.h"
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SHT31.h>
#include <PID_v1.h>

#define DEBOUNCE 5             // 0 is fine for most fans, crappy fans may require 10 or 20 to filter out noise
#define FANSTUCK_THRESHOLD 500 // if no interrupts were received for 500ms, consider the fan as stuck and report 0 RPM
#define FAN1_PWM 9
#define FAN2_PWM 10
#define FAN1_TACHO 2
#define FAN2_TACHO 3
#define MIN_FAN_SPD 0.5

#define HEATER_PIN 12

#define HX711_1_DaT 5
#define HX711_1_SCK 4
#define HX711_2_DaT 8
#define HX711_2_SCK 7

#define CLOCK_TIME 1000

#define TCA_ADDR 0x70
#define SHT_ADDR 0x44

HX711_ADC hx_1(HX711_1_DaT, HX711_1_SCK);
HX711_ADC hx_2(HX711_2_DaT, HX711_2_SCK);

SHT31 sht(SHT_ADDR, &Wire);

int time = 0;

const int calVal_eepromAdress_1 = 0;
const int calVal_eepromAdress_2 = 16;

double heater_setpoint, heater_input, heater_output;
double heater_p = 1;
double heater_i = 1;
double heater_d = 1;

PID heaterController(&heater_input, &heater_output, &heater_setpoint, heater_p, heater_i, heater_d, DIRECT);

double fan_setpoint, fan_input, fan_output;
double fan_p = 1;
double fan_i = 1;
double fan_d = 1;
double fan_duty;
double temp_diff;

PID fanController(&fan_input, &fan_output, &fan_setpoint, fan_p, fan_i, fan_d, REVERSE);

unsigned long volatile ts1a = 0, ts1b = 0, ts2a = 0, ts2b = 0;

void tachISR_1()
{
  unsigned long m = millis();
  if ((m - ts1b) > DEBOUNCE)
  {
    ts1a = ts1b;
    ts1b = m;
  }
}

void tachISR_2()
{
  unsigned long m = millis();
  if ((m - ts2b) > DEBOUNCE)
  {
    ts2a = ts2b;
    ts2b = m;
  }
}

unsigned long calcRPM(long a, long b)
{
  if (millis() - b < FANSTUCK_THRESHOLD && b != 0)
  {
    return (60000 / (b - a)) / 2;
  }
  else
    return 0;
}

void setPWM1A(float f)
{
  f = f < 0 ? 0 : f > 1 ? 1
                        : f;
  OCR1A = (uint16_t)(320 * f);
}

void setPWM1B(float f)
{
  f = f < 0 ? 0 : f > 1 ? 1
                        : f;
  OCR1B = (uint16_t)(320 * f);
}

void calibrate()
{
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false)
  {
    hx_1.update();
    hx_2.update();
    if (Serial.available() > 0)
    {
      if (Serial.available() > 0)
      {
        char inByte = Serial.read();
        if (inByte == 't')
        {
          Serial.print("Tare 1 in progress...");
          // Serial.print(hx_1.getTareStatus());
          hx_1.tare();
          Serial.print(" -> ");
          // Serial.print(hx_1.getTareStatus());
          Serial.println(" Done.");

          Serial.print("Tare 2 in progress...");
          // Serial.print(hx_2.getTareStatus());
          hx_2.tare();
          Serial.print(" -> ");
          // Serial.print(hx_2.getTareStatus());
          Serial.println(" Done.");
        }
      }
    }
    if (hx_1.getTareStatus() == true && hx_2.getTareStatus() == true)
    {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false)
  {
    hx_1.update();
    hx_2.update();
    if (Serial.available() > 0)
    {
      known_mass = Serial.parseFloat();
      if (known_mass != 0)
      {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  hx_1.refreshDataSet();
  hx_2.refreshDataSet();                                                // refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue_1 = hx_1.getNewCalibration(known_mass / 2); // get the new calibration value
  float newCalibrationValue_2 = hx_2.getNewCalibration(known_mass / 2); // get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue_1);
  Serial.print(", ");
  Serial.print(newCalibrationValue_2);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress_1);
  Serial.print(", ");
  Serial.print(calVal_eepromAdress_2);
  Serial.println("? y/n");

  _resume = false;
  while (_resume == false)
  {
    if (Serial.available() > 0)
    {
      char inByte = Serial.read();
      if (inByte == 'y')
      {
        EEPROM.put(calVal_eepromAdress_1, newCalibrationValue_1);
        EEPROM.put(calVal_eepromAdress_2, newCalibrationValue_1);
        Serial.print("Value ");
        Serial.print(newCalibrationValue_1);
        Serial.print(" saved to EEPROM address: ");
        Serial.print(calVal_eepromAdress_1);
        Serial.print(", ");
        Serial.println(calVal_eepromAdress_2);
        _resume = true;
      }
      else if (inByte == 'n')
      {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("***");
}

void setup_heater()
{
  pinMode(HEATER_PIN, OUTPUT);

  heater_input = 0;
  heater_setpoint = 90;

  heaterController.SetMode(AUTOMATIC);
}

void setup_fans()
{
  pinMode(FAN1_PWM, OUTPUT);
  pinMode(FAN2_PWM, OUTPUT);

  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << CS10) | (1 << WGM13);
  ICR1 = 320;
  OCR1A = 0;
  OCR1B = 0;

  pinMode(FAN1_TACHO, INPUT_PULLUP);
  pinMode(FAN2_TACHO, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FAN1_TACHO), tachISR_1, RISING);
  attachInterrupt(digitalPinToInterrupt(FAN2_TACHO), tachISR_2, RISING);

  fan_input = temp_diff;
  fan_setpoint = 0;

  fanController.SetMode(AUTOMATIC);
  fanController.SetOutputLimits(MIN_FAN_SPD * 255, 255);
}

void setup_scales()
{
  hx_1.begin();
  hx_2.begin();
  // LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step

  hx_1.start(stabilizingtime, _tare);
  hx_2.start(stabilizingtime, _tare);
  if (hx_1.getTareTimeoutFlag() || hx_1.getSignalTimeoutFlag())
  {
    Serial.println("[HX711_1] Timeout, check wiring and pin designations");
    while (1)
      ;
  }
  else
  {
    float cal = 0;
    EEPROM.get(calVal_eepromAdress_1, cal);
    Serial.print("[HX711_1] Calibration value loaded from EEPROM: ");
    Serial.println(cal);
    hx_1.setCalFactor(cal); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("[HX711_1] Startup complete");
  }
  while (!hx_1.update())
    ;

  if (hx_2.getTareTimeoutFlag() || hx_2.getSignalTimeoutFlag())
  {
    Serial.println("[HX711_2] Timeout, check wiring and pin designations");
    while (1)
      ;
  }
  else
  {
    float cal = 0;
    EEPROM.get(calVal_eepromAdress_1, cal);
    Serial.print("[HX711_1] Calibration value loaded from EEPROM: ");
    Serial.println(cal);
    hx_2.setCalFactor(cal); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("[HX711_2] Startup complete");
  }
  while (!hx_2.update())
    ;

  // TODO: Skips calibration on startup.
  // calibrate(); // start calibration procedure
}

void setTCAChannel(byte i)
{
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(9600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  setup_fans();

  // setup_scales();

  setup_heater();

  Wire.begin();
  Wire.setClock(100000);

  Serial.println("Setup complete");

  Serial.println("----");

  // Serial.println("time;scale;fan_pwm;rpm1;rpm2;temp1;hum2;temp2;hum2;temp3;hum3;temp4;hum4;tempdiff;avgtemp;heater;");
  Serial.println("time;fan_pwm;rpm1;rpm2;temp1;hum2;temp2;hum2;temp3;hum3;temp4;hum4;tempdiff;avgtemp;heater;");
}

void loop_fans()
{

  fanController.Compute();
  // fan_duty = MIN_FAN_SPD + fan_output / (255 / MIN_FAN_SPD);
  fan_duty = fan_output;

  setPWM1A(fan_duty);
  setPWM1B(fan_duty);

  // Serial.print("PWM: ");
  Serial.print(fan_duty);
  Serial.print(";");

  // Serial.print("RPM: ");
  Serial.print(calcRPM(ts1a, ts1b));
  Serial.print(";");
  Serial.print(calcRPM(ts2a, ts2b));
  Serial.print(";");
}

void loop_scales()
{
  static boolean newDataReady = 0;

  // check for new data/start next conversion:
  if (hx_1.update() || hx_2.update())
  {
    newDataReady = true;
  }

  // get smoothed value from the dataset:
  if (newDataReady)
  {
    float i_1 = hx_1.getData();
    float i_2 = hx_2.getData();
    float i_g = i_1 + i_2;
    Serial.print(i_g);
    Serial.print(";");

    newDataReady = 0;
  }

  // receive command from serial terminal
  if (Serial.available() > 0)
  {
    char inByte = Serial.read();
    if (inByte == 't')
    {
      hx_1.tareNoDelay(); // tare
      hx_2.tareNoDelay(); // tare
    }
    else if (inByte == 'r')
    {
      calibrate(); // calibrate
    }
  }

  // check if last tare operation is complete
  if (hx_1.getTareStatus() == true && hx_2.getTareStatus() == true)
  {
    Serial.println("Tare complete");
  }
}

void loop()
{

  time += 1;

  Serial.print(time);
  Serial.print(";");

  // loop_scales();

  loop_fans();

  // adjust here for the desired and connected temperature sensors

  float temp_max = 0;
  float temp_min = 255;
  float temp_total = 0;

  for (int i = 0; i <= 3; i++)
  {
    setTCAChannel(i);
    sht.begin();

    uint16_t stat = sht.readStatus();

    // 0: Hinten
    // 1: Unten
    // 2: Vorne
    // 3: Oben

    if (stat == 0xFFFF)
    {
      // Serial.println("");
    }
    else
    {
      if (
          sht.dataReady())
      {
        sht.read(false);

        float temp = sht.getTemperature();
        float hum = sht.getHumidity();

        if (temp > temp_max)
        {
          temp_max = temp;
        }

        if (temp < temp_min)
        {
          temp_min = temp;
        }

        temp_total += temp;

        Serial.print(temp);
        Serial.print(";");
        Serial.print(hum);
        Serial.print(";");
      }
      else
      {
        Serial.println("Data not ready");
      }
    }

    // sht.reset();
  }

  temp_diff = temp_max - temp_min;
  float avg_temp = temp_total / 4;
  Serial.print(temp_diff);
  Serial.print(";");
  Serial.print(avg_temp);
  Serial.print(";");

  heater_input = avg_temp * 0.5 + temp_max * 0.5;
  heaterController.Compute();
  analogWrite(HEATER_PIN, heater_output);
  Serial.print(heater_output);
  Serial.print(";");

  Serial.println("");

  if (temp_max > 95)
  {
    Serial.println("===== OVERTEMP ===== OVERTEMP ===== OVERTEMP ===== OVERTEMP =====");
    // shit pants and turn off heater
  }
  else
  {
    // resume normal operation
    // TODO
  }

  delay(CLOCK_TIME);
}