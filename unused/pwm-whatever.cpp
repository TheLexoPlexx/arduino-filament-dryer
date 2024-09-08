#include <Arduino.h>

unsigned long highTime; // integer for storing low time
unsigned long lowTime;  // integer for storing low time
float period;           // integer for storing period
float freq;             // intefer for storing frequency
float RPM;              // storing or calculating RPM

const byte FAN1_PWM = 9;
const byte FAN2_PWM = 10;
const byte FAN1_TACHO = 11;
const byte FAN2_TACHO = 12;

const word PWM_FREQ_HZ = 25000; // Adjust this value to adjust the frequency (Frequency in HZ!)  (Set currently to 25kHZ)
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);

const byte duty = 10;

void setPwmDuty(byte duty) // duty cycle is from 0 to 100
{
  OCR1A = (word)(duty * TCNT1_TOP) / 100;
  OCR1B = (word)(duty * TCNT1_TOP) / 100;
}

void setup()
{
  Serial.begin(9600);

  pinMode(FAN1_PWM, OUTPUT);
  pinMode(FAN2_PWM, OUTPUT);

  pinMode(FAN1_TACHO, INPUT);
  pinMode(FAN2_TACHO, INPUT);

  // Clear Timer1 control  and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  //  Set Timer1 configuration
  // COM1A(1:0) = 0b10   (Output A clear rising/set  falling)
  // COM1B(1:0) = 0b00   (Output B normal operation)
  // WGM(13:10)  = 0b1010 (Phase correct PWM)
  // ICNC1      = 0b0    (Input capture noise canceler  disabled)
  // ICES1      = 0b0    (Input capture edge select disabled)
  //  CS(12:10)  = 0b001  (Input clock select = clock/1)

  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
}

void loop()
{

  highTime = pulseIn(5, HIGH); // read high time
  lowTime = pulseIn(5, LOW);   // read low time
  period = highTime + lowTime; // Period = Ton + Toff
  freq = 1000000 / period;     // getting frequency with totalTime is in Micro seconds
  RPM = (freq / 2) * 60;       // we div by 2 since the fan tach outputs 2 pulses per revolution

  // Serial Print Data
  Serial.print("Frequency = ");
  Serial.print(freq);
  Serial.print(" Hz / ");
  Serial.print("RPM = ");
  Serial.println(RPM);
  Serial.println("");

  Serial.print("PWM (%) = ");
  Serial.println(duty);
  setPwmDuty(duty);
  delay(500);
}