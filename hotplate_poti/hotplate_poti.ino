// wemos d1 mini

//#include <SPI.h>
#include <Wire.h>

// MAX6675 library
#include "max6675.h"

// MegunoLink library
#include "CommandHandler.h" 

#define COOLDOWN_TIME 60 //60
#define PREHEAT_TIME 60
#define REFLOW_TIME 60

#define OLED_RESET 0  // GPIO0

float update_rate = 0.2; // temp average update rate

int thermoDO = 12;
int thermoCS = 13;
int thermoCLK = 14;
int MAX7219_DIN = 16;
int MAX7219_CS = 15;
int MAX7219_CLK = thermoCLK;

const int heater = 5;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
CommandHandler<> SerialCommandHandler;


void output(byte address, byte data)
{
  digitalWrite(MAX7219_CS, LOW);
  pinMode(MAX7219_DIN, OUTPUT);
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, address);
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, data);
  pinMode(MAX7219_DIN, INPUT);
  digitalWrite(MAX7219_CS, HIGH);
}

void displaysetup()
{
  output(0x0f, 0x00); //display test register - test mode off
  output(0x0c, 0x01); //shutdown register - normal operation
  output(0x0b, 0x07); //scan limit register - display digits 0 thru 7
  output(0x0a, 0x05); //intensity register - max brightness
  output(0x09, 0xff); //decode mode register - CodeB decode all digits
}

const int temp_preheat = 150;
const int temp_reflow = 240;


String state[] = {"OFF", "PREHEAT", "REFLOW", "COOLING"};
int state_now = 0;

unsigned long time_count = 0;

int offset = 0;

float temp_now, temp_avg, temp_last, error, lasterror;
float Kp=3.2, Ki=0.0, Kd=9.0; // PID constants.
int pwm;
int started_pwm_manual = 1;
int started_profile_manual = 1;

unsigned long sampleinterval = 200;
unsigned long lastsampletime = millis();
unsigned long lastdisplaytime = millis();
unsigned long lastpwmtime = millis();
unsigned long pwmperiod = 1000;
unsigned long pwmoff = 0;
unsigned long maxpwm = 256;
unsigned long plotperiod = 1000;
unsigned long lastprofiletime = millis();
long t_solder = millis();


void setup() {
  pinMode(heater, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(heater, LOW);
  digitalWrite(MAX7219_CS, HIGH);
  pinMode(MAX7219_CS, OUTPUT);
  pinMode(MAX7219_CLK, OUTPUT);
  displaysetup();

  Serial.begin(9600);
  SerialCommandHandler.AddCommand(F("interval"), cmd_setinterval);
  SerialCommandHandler.AddCommand(F("temp"), cmd_temp);
  SerialCommandHandler.AddCommand(F("run"), cmd_run);
  SerialCommandHandler.AddCommand(F("runprofile"), cmd_runprofile);
  SerialCommandHandler.AddCommand(F("pwm"), cmd_pwm);
  SerialCommandHandler.AddCommand(F("pwmperiod"), cmd_pwmperiod);
  SerialCommandHandler.SetDefaultHandler(Cmd_Unknown);
  temp_now = thermocouple.readCelsius();
  temp_avg = temp_now;
  temp_last = temp_now;
  delay(3250);
}


void loop() {
  SerialCommandHandler.Process();

  unsigned long millis_now = millis();

  if (millis_now > lastsampletime + sampleinterval || millis_now < lastsampletime) {
    //PrintScreen(state[state_now], temp_next, temp_now, time_count, perc);
    lastsampletime = millis_now;
    temp_now = thermocouple.readCelsius();
    temp_avg = temp_avg*(1.0-update_rate) + temp_now*update_rate;
  }

  if (millis_now > lastdisplaytime + 250 || millis_now < lastdisplaytime) {
    lastdisplaytime = millis_now;
    to_screen();
  }

  if (millis_now > lastprofiletime + plotperiod || millis_now < lastprofiletime) {
    lastprofiletime = millis_now;
    float temp_setpoint = temperature_profile(time_count++);

    // pid
    error = temp_setpoint - temp_avg;
    float pidout = (int)(error*Kp + (error - lasterror)*Kd);
    pwm = (int)pidout;
    if(pwm>255) pwm=255;
    if(pwm<0) pwm=0;

    lasterror=error;

    Serial.print(temp_now);
    Serial.print(" ");
    Serial.print(temp_avg);
    Serial.print(" ");
    Serial.print(temp_setpoint);
    Serial.print(" ");
    //Serial.print(error);
    //Serial.print(" ");
    Serial.print(pidout);
    Serial.print(" ");
    Serial.println(pwm);
  }


  if (millis_now > lastpwmtime + pwmperiod || millis_now < lastpwmtime) {
    lastpwmtime = millis_now;
    //if (started_pwm_manual) {
      digitalWrite(heater, HIGH);
      pwmoff = millis_now + pwmperiod*((float)pwm/maxpwm)-1;
    //}
  }
  if (millis_now > pwmoff && pwmoff>0) {
    pwmoff=0;
    digitalWrite(heater, LOW);
  }
  digitalWrite(LED_BUILTIN, !digitalRead(heater));

  if (started_pwm_manual==0 && started_profile_manual==0) digitalWrite(heater, LOW);

}

struct point {
  int s; // seconds
  int t; // temperature
};
struct point curve[] = { {0,0}, {90,150}, {180,150}, {240,200}, {360,0} };
//struct point curve[] = { {0,0}, {10,0}, {11,100}, {240,100}, {241,0} };

float temperature_profile(unsigned long _timecount) {

  const int array_size = sizeof(curve)/sizeof(curve[0]);

  // return 0 if _timecount outside curve points.
  if (_timecount < 0 || _timecount > curve[array_size-1].s)
    return 0;

  // find zone (index of profile_points)
  int zone=0;
  for( zone=0; zone < array_size-1; zone++) {
    if (_timecount< curve[zone+1].s ) {
      break;
    }
  }
  float s1 = curve[zone].s;
  float s2 = curve[zone+1].s;
  float t1 = curve[zone].t;
  float t2 = curve[zone+1].t;

  // calculate slope
  float slope = (t2-t1)/(s2-s1);

  // find t-axis intercept
  float b = t1-slope*s1;

  return slope*_timecount + b;
}

void regulate_temp(int temp, int should) {
  if (should <= temp - offset) {
    digitalWrite(heater, LOW);
  }
  else if (should > temp + offset) {
    digitalWrite(heater, HIGH);
  }
}

void to_screen() {
    char buffer[9]={'0'};

    char* buf_p = buffer;
    sprintf(buf_p, "%03d %03d ", (int)temp_now, (int)pwm);
    for(byte i=0; i<8; i++) {
      if(buffer[i]==0 || buffer[i]==' ') {
        output(8-i,0x0f);
      } else {
        //if(i==0 || i==4)
        //  output(8-i,byte(buffer[i])+0x80);
        //else
          output(8-i, buffer[i] );
      }
    }
}

void cmd_setinterval(CommandParameter &Parameters)
{
  sampleinterval = Parameters.NextParameterAsInteger(sampleinterval);
  Serial.print("sampleinterval set to ");
  Serial.println(sampleinterval);
}

void cmd_run(CommandParameter &Parameters)
{
  started_pwm_manual = Parameters.NextParameterAsInteger(started_pwm_manual);
  if (started_pwm_manual==0) {
    Serial.println("Stop!");
  } else {
    Serial.println("Start!");
  }
}

void cmd_runprofile(CommandParameter &Parameters)
{
  started_profile_manual = Parameters.NextParameterAsInteger(started_profile_manual);
  if (started_profile_manual==0) {
    Serial.println("Stop!");
  } else {
    time_count=0;
    Serial.println("Start!");
  }
}


void cmd_temp(CommandParameter &Parameters)
{
  Serial.print("temp_now is ");
  Serial.println(temp_now);
}

void cmd_pwm(CommandParameter &Parameters)
{
  pwm = Parameters.NextParameterAsInteger(pwm);
  Serial.print("pwm set to ");
  Serial.println(pwm);
}

void cmd_pwmperiod(CommandParameter &Parameters)
{
  pwmperiod = Parameters.NextParameterAsInteger(pwmperiod);
  Serial.print("pwmperiod set to ");
  Serial.println(pwm);
}

void Cmd_Unknown()
{
  Serial.println(F("I don't understand"));
}
