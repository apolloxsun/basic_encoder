#define sensorPin 3
#define mdIN2 9
#define mdIN1 8
#define mdD2 6
#define mdD1 5
#define mdEN 7

int sensorData;
float speedVal;
int prevVal = 2;
int statusVal;
float revolveCnt = 0;
int counter = -1;
int currTime = 0;
int prevTime = -1;
float dt = 1;
int intCounter = 0;
float rpm = 0;
float rps;
int desiredVal;
float error;
int loopTime = 0;
int timeThreshold = 1000;
float priorError = 0;
float kp;
float ki;
float kd;
float priorIntegral;
float integral;

void setup() {
  pinMode(sensorPin, INPUT);
  pinMode(mdIN2, OUTPUT);
  pinMode(mdIN1, OUTPUT);
  pinMode(mdD2, OUTPUT);
  pinMode(mdD1, OUTPUT);
  pinMode(mdEN, OUTPUT);

  digitalWrite(mdEN, HIGH);
  digitalWrite(mdD1, LOW);
  digitalWrite(mdD2, HIGH);
  digitalWrite(mdIN1, HIGH);
  
  Serial.begin(115200);  
}

void loop(){
  loopTime = millis();
  //getting input
  if(Serial.available() > 0){ 
      desiredVal = Serial.parseInt();
      speedVal = map(desiredVal, 0, 280, 0, 255);
      analogWrite(mdIN1, speedVal);
      delay(100);

  }
  
  if(speedVal < 0){
    speedVal = map(desiredVal, 0, 280, 0, 155);
  }
  
  analogWrite(mdIN1, speedVal);
  
  //using kp to decrease error 
  error = desiredVal - rpm;
  kp = 0.0015;
  speedVal += (kp * error);

  ki = 0.01;
  integral = priorIntegral + error * ki;
  speedVal += ki*integral;
  
  kd = 0.00;
  speedVal += kd * (error - priorError) / dt;

  priorError = error;

  //counting revs and computing speed with measured interval 
  sensorData = digitalRead(sensorPin);
  statusVal = sensorData;  
  if(statusVal != prevVal){
    if(counter > 24){
      counter = 0;
      revolveCnt++;
    }
    if(intCounter >= 2){ 
      intCounter = 0;
      currTime = millis();
      dt = currTime - prevTime;
      prevTime = currTime;
    }
    counter++;
    intCounter++;
  }
  if(dt!=0){
  rps = (1000 / (dt*12));
  rpm = rps *60;
  }
  
  prevVal = statusVal;

  if(loopTime - prevTime > timeThreshold){
    rps = 0;
    rpm = 0;
  }
  //debugging
  //Serial.println(dt);
  //Serial.println(counter);
  /*Serial.print("Desired Speed: ");
  Serial.println(desiredVal);
  Serial.print("Error : ");
  Serial.println(error);
  Serial.println(speedVal);
  Serial.print("RPS : ");
  Serial.println(rps);
  Serial.print("RPM : ");*/
  Serial.println(rpm);
  //Serial.println(revolveCnt);
  //Serial.println("---");

}
