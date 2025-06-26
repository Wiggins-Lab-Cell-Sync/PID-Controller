

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
    self->sum_error = min(20, self->sum_error);
  }else{
    self->sum_error = max(-20, self->sum_error);
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
  float v = analogRead(A0) * (5.13 / 1024.0);
  float res = ((5.13 - v) * 7400) / v; // solved for R1 in voltage divider.
  float TA = 24.;//c
  float TB = 45.5;//c
  float RA = 10260;//ohm
  float RB = 4240;//ohm
  // float RB = 4390;//ohm
  // float TA = 23.;//c
  // float TB = 42.;//c
  // float RA = 10910;//ohm
  // float RB = 4915;//ohm

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
  .P = 2.,
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
unsigned int counter = 0;
// ----------

void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

//---------------------------------------------------------
unsigned long TimeStamp = 0; // time since start in seconds, used only for printing state
unsigned long prgm_count = 0;
unsigned long NextTime = 1;

void loop() {
  TimeStamp = millis();

  if(TimeStamp > NextTime*1000){
    on_second();
    NextTime = NextTime + 1; // Update for the next read time
    prgm_count = 0;
  }
  
  on_centisecond();
  delay(1);
  prgm_count++;
}

void on_second(){
  float temp = temp_sum / prgm_count;
  temp_sum = 0;

  while(Serial.available()){
    set_point = Serial.parseInt();
    Serial.readStringUntil('\n');
  }
  
  
  float output = stabilizer.output(&stabilizer, temp, set_point);

  // If the output is continuously higher than 80 or lower than -80 than somethings wrong
  if (abs( output) > 80) {
    counter++;
  } else {
    counter = 0;
  }

  // If something goes wrong isSafe = false
  if (Serial.availableForWrite() < 35 || counter > 120){
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

//---------------------------------------------------------

void on_centisecond(){
  temp_sum += read_temp();
}



