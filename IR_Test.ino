//IR TEST sketch, written by Joshua Curfman
//figuring out how best to use low-cost IR distance sensors

const int IRin1=A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  //Short distance IR sensor: 3V @ 10cm (4") -> 0.4V @ 80cm (32")
  //=29.988*pow(calcResult, -1.173); //curve fit based on datasheet, accurarcy may vary sensor to sensor
  //Medium distance IR sensor: 3V @ 20cm (8") -> 0.4V @ 150cm (60")
  //=65*pow(calcResult, -1.10); //curve fit based on datasheet
  float distance=readIRDistance(IRin1);
  float calcInches=distance*0.3937; //conversion from cm to inches
  Serial.print("Distance reading (in): ");
  Serial.println(calcInches);
}

float readIRDistance(int sensorPin) {
  //call to read in data, take an average of ten readings, and convert to distance in cm
  int numSamples=40;
  float storedValues[numSamples];
  float sum=0;
  for(int i=0; i<numSamples; i++) {
    //creates an array of readings to average
    int rawValue=analogRead(sensorPin);
    float rawVoltage=(rawValue/1024.0)*5;
    storedValues[i]=rawVoltage;
  }
  for(int i=0; i<numSamples; i++) {
    sum+=storedValues[i];
  }
  float calcResult=sum/numSamples;
  float calcDistance=65*pow(calcResult, -1.10);
  return calcDistance;
}

void oldTestLoop() {
  //THIS TESTING IS OLD, AND WAS USED TO FIGURE OUT HOW TO USE THESE SENSORS INITIALLY. RETAINED HERE FOR POSTERITY.
  //short distance IR sensor: 0.4V @ 80cm (32") -> 3V @ 10cm (4")
  //uncomment below this line. Sensor mostly working, further work to be done
  int maxVal=(3.0/5)*1024;
  int minVal=(0.4/5)*1024;
  int distAtMin=32;
  int distAtMax=4;
  //long distance IR sensor: 3V @ 100cm -> 1.4V @ 500cm
  //uncomment below this line. 
  //int maxVal=(3/5)*1024;
  //int minVal=(1.4/5)*1024;
  //int distAtMax=39/12;
  //int distAtMin=196/12;
  
  int rawDistance=analogRead(IRin1);
  float rawVoltage=(rawDistance/1024.0)*5.0;
  float calcDistance=29.988*pow(rawVoltage, -1.173); //cm, based off datasheet curve fit, may vary sensor to sensor.
  float calcInches=calcDistance*0.3937; //conversion from cm to inches

  Serial.print("IR 1: ");
  Serial.println(calcInches); 
  delay(500);
}
