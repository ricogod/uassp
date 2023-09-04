#include <Arduino.h>
#include <Encoder.h>
#define buttonCW 10
#define buttonCCW 11
#define buttonQuarterDegree 12
#define buttonHalfDegree 13

const int Enable = 9; // L298N ENA
const int motorPin1 = 7;   // L298N IN1
const int motorPin2 = 8;   // L298N IN2
const int encoderPinA = 3; // Encoder Channel A
const int encoderPinB = 2; // Encoder Channel B

Encoder myEnc(encoderPinA, encoderPinB);

long previousMillis = 0;
long currentMillis = 0;

volatile long currentEnc = 0;
volatile long previousEnc = 0;
volatile long prevPos = 0;
volatile long currentPos = 0;
volatile long rnEnc = 0;
int desiredEncoder;

int rot = 0;
float oldRotSpeed = 0;
  int lastStateA;
int currentStateA;
int encoderValue = 0;
 int motorSpeed = 0;
bool CW = true;

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(buttonCW, INPUT);
  pinMode(buttonCCW, INPUT);
  pinMode(buttonHalfDegree, INPUT);
  pinMode(buttonQuarterDegree, INPUT); 
  lastStateA = digitalRead(encoderPinA);
  Serial.begin(9600);
  Serial.print("Please press the button");
  // digitalWrite(motorPin1, LOW);
  // digitalWrite(motorPin2, HIGH);
}


float updateEncoder(void){
  currentEnc = myEnc.read();
  rnEnc=myEnc.read();
  const int readPerRev =3393; //encoder read per revolution

  float rotSpeed;
  const int interval = 1000;
  currentMillis = millis();

  if(currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
   
    rotSpeed= (float)((currentEnc - previousEnc)*60/(readPerRev));
     previousEnc = currentEnc;
    return rotSpeed;
  }
  delay (100);
}
void readEncoder(){
  currentStateA = digitalRead(encoderPinA);
  if (currentStateA != lastStateA){
    if (digitalRead(encoderPinB) != currentStateA){
      encoderValue++;
    }
    else{
      encoderValue--;
    }
  }
  lastStateA = currentStateA;

}
void changeDegree(){
  

  if (digitalRead(buttonQuarterDegree) == HIGH)
  {
   
    // desiredEncoder =currentEnc+848;
    if(CW){
      desiredEncoder =rnEnc+848;
    }
    else{
      desiredEncoder =rnEnc-848;
    }
    
    // if(currentEnc < desiredEncoder){
    //   motorSpeed = 128;
    // }
    Serial.print("45 deg ");
    delay(1000);
  }
  
  else if (digitalRead(buttonHalfDegree) == HIGH)
  {
    
    //  desiredEncoder = currentEnc+1697;
    if(CW){
      desiredEncoder = rnEnc+1697;
    }
    else{
      desiredEncoder =rnEnc-1697;
    }
    Serial.print("90 deg ");
    delay(1000);
  }
 
  else if (digitalRead(buttonCW) == HIGH)
  {
    
    motorSpeed = 0;
    Serial.print("CW ");
    CW = true;
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    delay(1000);
    
  }
  else if (digitalRead(buttonCCW) == HIGH)
  {
    
    motorSpeed = 0;
    Serial.print("CCW ");
    CW = false;
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    delay(1000);
  }

}

void loop() {

  
  analogWrite(Enable, motorSpeed);
   //initiate the valye to CW
  float newRotSpeed;
  newRotSpeed = updateEncoder();
  changeDegree();
  currentEnc=abs(currentEnc); //currentEnc always has positive value
  if(newRotSpeed != oldRotSpeed){
    Serial.print("Rotational Speed: ");
    Serial.print(newRotSpeed);
    Serial.println(" RPM");
    oldRotSpeed = newRotSpeed;
  }
  if(CW){
    if(rnEnc <= desiredEncoder){
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      motorSpeed = 128;
    }
    else{
      motorSpeed = 0;
    }
  }
  else if (!CW){
    
    if(rnEnc >= desiredEncoder){
      digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
      motorSpeed = 128;
    }
    else{
      motorSpeed = 0;
    }
  }
  
  // Serial.print("encoder:");
  // Serial.println(rnEnc);
  // Serial.print("desiredEncoder:");
  // Serial.println(desiredEncoder);  

}

