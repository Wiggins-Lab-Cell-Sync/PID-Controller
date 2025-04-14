#include <avr/pgmspace.h>

struct Controller{
  float P;
  float I;
  float D;

  float last_error;
  float last_temp;
  float sum_error;
  
  float M;
  float C;

  float (*output)(struct Controller*, float, float); // (this, temp, setpoint) -> output
};

float PID_OUT(struct Controller* self, float temp, float set_point){
  float error = set_point - temp;
  float bias_err = (error - 0.1 * self->last_error)/10;

  self->sum_error = self->sum_error + abs(error/1.5)* abs( error/1.5) * error/1.5 * exp(-error*error*3/1.5)*14; //* exp( error/abs(error) * bias_err );
  if(self->sum_error > 0){
    self->sum_error = min(200, self->sum_error);
  }else{
    self->sum_error = max(-200, self->sum_error);
  }
  
  float out_P = self->P * ( abs(error) * error + error * 5) + self->M * temp + self->C;
  float out_I = self->I * self->sum_error * exp(- abs( bias_err ));
  float out_D = self->D * bias_err * exp(-error*error/36);
  

  //float out = (self->P * error) + self->sum_error + (self->D * (error - self->last_error)*exp(-(error*error)/5));
  float out = out_P + out_I + out_D;

  // Serial.println(out_P);
  // Serial.println(out_I);
  // Serial.println(out_I);
  // Serial.println(bias_err);
  self->last_error = self->last_error*0.9 + error;
  self->last_temp = temp;
  return out;
}


float read_temp(){
  float v = analogRead(A0) * (5.11 / 1024.0);
  
  float res = ((5.11 - v) * 7400) / v; // solved for R1 in voltage divider.

  float TA = 22.6;//c
  float TB = 45.2;//c
  float RA = 10880;//ohm
  float RB = 4390;//ohm

  float B = log(RA / RB) * (1 / ((1 / TA) - (1 / TB)));

  float temp = 1 / ((log(res / RB) / B) + (1 / TB));
  return temp;
}

void printState(unsigned long TimeStamp, float CurrentTemp, float TargetTemp, float PIDOutput){
  Serial.print(TimeStamp);
  Serial.print(" ");
  Serial.print(CurrentTemp);
  Serial.print(" ");
  Serial.print(TargetTemp);
  Serial.print(" ");
  Serial.print( min(150, abs(PIDOutput)) * abs(PIDOutput)/PIDOutput);
  Serial.print(" ");
  if (PIDOutput > 0) {
    Serial.println("H");
  } else {
    Serial.println("C");
  }
}


struct Controller stabilizer = (struct Controller){
  .P = 1.5,
  .I = 0.1,
  .D = 300,

  .last_error = 0,
  .last_temp = 0,
  .sum_error = 0,

  // OUTPUT = M * (Temp) + C 
  // Numbers are obtained by fitting temperature vs output data
  .M = 2.161,
  .C = -45.59,

  .output = &PID_OUT
};



float set_point = 30;
float temp_sum = 0;

// LUT for temperture. Use matlab to generate the table. 
const PROGMEM double lookupTable[] = {
    30.00, 30.00, 30.01, 30.03, 30.05, 30.07, 30.11, 30.15, 30.19, 30.24,
    30.30, 30.36, 30.43, 30.50, 30.58, 30.66, 30.75, 30.84, 30.94, 31.05,
    31.16, 31.27, 31.39, 31.51, 31.64, 31.77, 31.91, 32.05, 32.20, 32.34,
    32.50, 32.65, 32.81, 32.97, 33.14, 33.31, 33.48, 33.65, 33.82, 34.00,
    34.18, 34.36, 34.55, 34.73, 34.92, 35.10, 35.29, 35.48, 35.67, 35.86,
    36.05, 36.24, 36.43, 36.61, 36.80, 36.99, 37.18, 37.36, 37.55, 37.73,
    37.91, 38.09, 38.26, 38.44, 38.61, 38.78, 38.95, 39.11, 39.27, 39.43,
    39.58, 39.73, 39.88, 40.02, 40.16, 40.29, 40.42, 40.55, 40.67, 40.79,
    40.90, 41.01, 41.11, 41.20, 41.30, 41.38, 41.46, 41.54, 41.61, 41.67,
    41.73, 41.79, 41.83, 41.87, 41.91, 41.94, 41.96, 41.98, 41.99, 42.00,
    42.00, 41.99, 41.98, 41.96, 41.94, 41.91, 41.87, 41.83, 41.79, 41.73,
    41.67, 41.61, 41.54, 41.46, 41.38, 41.30, 41.20, 41.11, 41.01, 40.90,
    40.79, 40.67, 40.55, 40.42, 40.29, 40.16, 40.02, 39.88, 39.73, 39.58,
    39.43, 39.27, 39.11, 38.95, 38.78, 38.61, 38.44, 38.26, 38.09, 37.91,
    37.73, 37.55, 37.36, 37.18, 36.99, 36.80, 36.61, 36.43, 36.24, 36.05,
    35.86, 35.67, 35.48, 35.29, 35.10, 34.92, 34.73, 34.55, 34.36, 34.18,
    34.00, 33.82, 33.65, 33.48, 33.31, 33.14, 32.97, 32.81, 32.65, 32.50,
    32.34, 32.20, 32.05, 31.91, 31.77, 31.64, 31.51, 31.39, 31.27, 31.16,
    31.05, 30.94, 30.84, 30.75, 30.66, 30.58, 30.50, 30.43, 30.36, 30.30,
    30.24, 30.19, 30.15, 30.11, 30.07, 30.05, 30.03, 30.01, 30.00, 30.00
};
const int dt = 3;  // Time interval between the LUT. In terms of seconds
const unsigned long period = dt*sizeof(lookupTable) / sizeof(lookupTable[0]); //In seconds
const unsigned long StartTime = 0;
// ----------

void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

//---------------------------------------------------------
unsigned long TimeStamp = 0; // time since start in seconds, used only for printing state
unsigned long prgm_count = 0;
unsigned long NextTime = 1000;

void loop() {
  TimeStamp = millis();

  if(TimeStamp > NextTime){
    on_second();
    NextTime = NextTime + 1000; // Update for the next read time
    prgm_count = 0;
  }
  
  on_centisecond();
  delay(1);
  prgm_count++;
}

void on_second(){
  float temp = temp_sum / prgm_count;
  temp_sum = 0;

  update_target();
  float output = stabilizer.output(&stabilizer, temp, set_point);
  digitalWrite(9, (output > 0) ? LOW : HIGH);
  analogWrite(10, min(150, abs(output)));


  printState(TimeStamp, temp, set_point, output);
  TimeStamp++;
}

//------
void update_target(){
  unsigned long TimeDiff = (TimeStamp)/1000;
  unsigned long rem = TimeDiff % period;
  unsigned int quo = rem / dt;
  set_point = pgm_read_float(&lookupTable[quo]);
}

//---------------------------------------------------------

void on_centisecond(){
  temp_sum += read_temp();
}

