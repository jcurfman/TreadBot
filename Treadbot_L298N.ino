//Treadbot Code - Joshua Curfman
//L298N motor driver

//in1 and in2 for direction, motor1 for speed and enable
const int in1 = 7;
const int in2 = 8;
const int motor1 = 9;
const int in3 = 12;
const int in4 = 13;
const int motor2 = 11;

//setup analog stick and variables
const int xAxis = A1;
const int yAxis = A0;
int range = 24;
int center = range/2;
int threshold = range/6;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");
  
  //Initialize pins for L298N
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(motor1, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(motor2, OUTPUT);

  //set motors to not stop initially
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  //Test loop, needs substantial work
  int xReading, yReading;
  xReading=readAxis(xAxis);
  yReading=readAxis(yAxis);

  if(xReading==0 && yReading==0) {
    Serial.println("Stop");
    motorControl(1,0);
  }
  else if(xReading==0 && yReading!=0) {
    if(yReading>0) {
      Serial.println("Run forward");
      int newSpeed=map(yReading, 0, 12, 0, 200);
      motorControl(1, newSpeed);
    }
    else if(yReading<0) {
      Serial.println("Reverse");
      int newSpeed=map(abs(yReading), 0, 12, 0, 200);
      motorControl(3, newSpeed);
    }
  }
  else if(xReading!=0) {
    if(xReading>0) {
      Serial.println("Turn right");
      int newSpeed=map(xReading, 0, 12, 0, 128);
      motorControl(4, newSpeed);
    }
    else if(xReading<0) {
      Serial.println("Turn left");
      int newSpeed=map(abs(xReading), 0, 12, 0, 128);
      motorControl(2, newSpeed);
    }
  }
}

void motorControl(int directions, int motorSpeed) {
  //quick and dirty motor control function- pass direction and speed. 
  //directions-> 1=FORWARD, 2=LEFT, 3=BACKWARD, 4=RIGHT
  if(directions == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(motor1, motorSpeed);
    analogWrite(motor2, motorSpeed);
  }
  else if(directions == 2) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(motor1, motorSpeed);
    analogWrite(motor2, motorSpeed);
  }
  else if(directions == 3) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(motor1, motorSpeed);
    analogWrite(motor2, motorSpeed);
  }
  else if(directions == 4) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(motor1, motorSpeed);
    analogWrite(motor2, motorSpeed);
  }
}

int readAxis(int thisAxis) {
  //read the analog input, scale it, and ensure outside of center rest position
  int reading = analogRead(thisAxis);
  reading = map(reading, 0, 1023, 0, range);
  int distance = reading - center;
  if(abs(distance) < threshold) {
    distance = 0;
  }
  //return distance on this axis
  return distance;
}
