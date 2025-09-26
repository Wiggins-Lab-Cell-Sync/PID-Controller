#include <avr/pgmspace.h>
#include <math.h>  // for isnan()

// ---------------- Host override state (NEW) ----------------
bool host_override = false;     // when true, host controls set_point
float host_setpoint = 30.0;     // last value received from host
// ----------------------------------------------------------

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

  self->sum_error = self->sum_error + abs(error/1.5)* abs( error/1.5) * error/1.5 * exp(-error*error*3/1.5)*14;
  if(self->sum_error > 0){
    self->sum_error = min(20, self->sum_error);
  }else{
    self->sum_error = max(-20, self->sum_error);
  }
  
  float out_P = self->P * ( abs(error) * error + error * 5) + self->M * temp + self->C;
  float out_I = self->I * self->sum_error * exp(- abs( bias_err ));
  float out_D = self->D * bias_err * exp(-error*error/36);
  float out = out_P + out_I + out_D;

  self->last_error = self->last_error*0.9 + error;
  self->last_temp = temp;
  return out;
}

float read_temp(){
  float v = analogRead(A0) * (5.13 / 1024.0);
  float res = ((5.13 - v) * 7400) / v; // solved for R1 in voltage divider.
  float TA = 24.;//c
  float TB = 45.5;//c
  float RA = 10260;//ohm
  float RB = 4240;//ohm

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
  Serial.print(PIDOutput);
  Serial.print(" ");
  if (PIDOutput > 0) {
    Serial.println("H");
  } else {
    Serial.println("C");
  }
}

struct Controller stabilizer = (struct Controller){
  .P = 2.0,
  .I = 0.1,
  .D = 300,

  .last_error = 0,
  .last_temp = 0,
  .sum_error = 0,

  // OUTPUT = M * (Temp) + C 
  // Numbers are obtained by fitting temperature vs output data
  .M = 2.31,
  .C = -46.35,

  .output = &PID_OUT
};

float set_point = 30;
float temp_sum = 0;
bool isSafe = true;
bool startCyc = false;
unsigned long startTime = 0;
unsigned int counter = 0;

// LUT for temperture. Use matlab to generate the table. 
const PROGMEM double lookupTable[] = {
    42.00, 42.00, 42.00, 42.00, 42.00, 42.00, 42.00, 42.00, 42.00, 42.00,  
    30.00, 30.00, 30.00, 30.00, 30.00, 30.00, 30.00, 30.00, 30.00, 30.00, 
};

const int dt = 60;  // Time interval between the LUT (seconds)
const unsigned long period = dt*sizeof(lookupTable) / sizeof(lookupTable[0]); // seconds

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(5);  // NEW: keep readStringUntil from blocking long
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

//---------------------------------------------------------
unsigned long TimeStamp = 0; // time since start in ms, used only for printing state
unsigned long prgm_count = 0;
unsigned long NextTime = 1000;

void loop() {
  TimeStamp = millis();

  if(TimeStamp > NextTime){
    on_second();
    NextTime = NextTime + 1000; // next second
    prgm_count = 0;
  }
  
  on_centisecond();
  delay(1);
  prgm_count++;
}

void on_second(){

  float temp = prgm_count > 0 ? (temp_sum / prgm_count) : temp_sum;
  temp_sum = 0;

  // --------- Read ALL complete lines from serial (non-blocking, latching) ---------
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line.equalsIgnoreCase("start")) {
      startCyc = true;
      startTime = millis();
    } else if (line.equalsIgnoreCase("end")) {
      startCyc = false;
      // keep host_override as-is; send "auto" to relinquish host control
    } else if (line.equalsIgnoreCase("auto")) {
      host_override = false;   // release to Arduino control (LUT/idle)
    } else {
      // numeric setpoint from host (e.g., "42" or "42.0")
      float maybe = line.toFloat();
      if (!isnan(maybe) && maybe > -50 && maybe < 150) {
        host_setpoint = maybe;
        host_override = true;  // latch host control until "auto" is received
      }
    }
  }

  // --------- Decide set_point once, AFTER serial parsing ---------
  if (host_override) {
    set_point = host_setpoint;           // Host wins; no flicker
  } else if (startCyc) {
    update_target();                     // LUT mode sets set_point
  } else {
    set_point = 30.0;                    // Idle fallback
  }

  // --------- PID and safety (unchanged) ---------
  float output = stabilizer.output(&stabilizer, temp, set_point);

  if (abs( output) > 80) {
    counter++;
  } else {
    counter = 0;
  }

  if (Serial.availableForWrite() < 35 || counter > 400){
    isSafe = false;
  }

  if (isSafe){
    if (output > 0) {
      output = min(150, output);
    } else {
      output = max(-150, output);
    }
  } else { // If something goes wrong, set the output to 0
    output = 0;
  }
  digitalWrite(9, (output > 0) ? LOW : HIGH);
  analogWrite(10, abs(output));
  printState(TimeStamp, temp, set_point, output);
  TimeStamp++;
}

//------
void update_target(){
  unsigned long TimeDiff = (TimeStamp-startTime)/1000;
  unsigned long rem = TimeDiff % period;
  unsigned int quo = rem / dt;
  set_point = pgm_read_float(&lookupTable[quo]);
}

//---------------------------------------------------------
void on_centisecond(){
  temp_sum += read_temp();
}