

struct Controller{
  float P;
  float I;
  float D;

  float last_error;
  float last_temp;
  float sum_error;
  
  float (*output)(struct Controller*, float, float); // (this, temp, setpoint) -> output
};

float PID_OUT(struct Controller* self, float temp, float set_point){
  float error = set_point - temp;

  self->sum_error = self->sum_error + self->I * error;
  if(self->sum_error > 0){
    self->sum_error = min(200, self->sum_error);
  }else{
    self->sum_error = max(-200, self->sum_error);
  }

  if(abs(error) > 5){
    self->sum_error = 0;
  }

  // if(self->last_error * error < 0 && abs(self->last_error - temp) > 0.2){ // opposite signs
  //   self->sum_error = 0;
  // }
  
  float out = (self->P * error) + self->sum_error + (self->D * (error - self->last_error));
  self->last_error = error;
  self->last_temp = temp;
  return out;
}


float read_temp(){
  float v = analogRead(A0) * (5 / 1024.0);
  
  float res = ((5 - v) * 7500) / v; // solved for R1 in voltage divider.

  float TA = 25;//c
  float TB = 35;//c
  float RA = 10000;//ohm
  float RB = 6522;//ohm

  float B = log(RA / RB) * (1 / ((1 / TA) - (1 / TB)));

  float temp = 1 / ((log(res / RB) / B) + (1 / TB));
  return temp;
}

void printState(int TimeStamp, float CurrentTemp, float TargetTemp, float PIDOutput){
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
  .P = 10,
  .I = 1,
  .D = 0,

  .last_error = 0,
  .last_temp = 0,
  .sum_error = 0,

  .output = &PID_OUT
};



float set_point = 42;
float temp_sum = 0;


//---------------------------------------------------------
int timeStamp = 0; // time since start in seconds, used only for printing state
void on_second(){
  float temp = temp_sum / 100;
  temp_sum = 0;

  while(Serial.available()){
    set_point = Serial.parseInt();
  }
  
  
  float output = stabilizer.output(&stabilizer, temp, set_point);

  digitalWrite(9, (output > 0) ? LOW : HIGH);
  analogWrite(10, min(150, abs(output)));

  printState(timeStamp, temp, set_point, output);
  timeStamp++;
}

//---------------------------------------------------------

void on_centisecond(){
  temp_sum += read_temp();
}

// ----------

int prgm_count = 0;
void loop() {
  if(prgm_count % 100 == 0){
    on_second();
    prgm_count = 0;
  }
  
  on_centisecond();
  delay(10);
  prgm_count++;
}


void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}
