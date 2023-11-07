#include <util/atomic.h>

// define all encoder pins
#define ENCA1 15 // Yellow
#define ENCB1 14 // White

#define ENCA2 21 // Yellow
#define ENCB2 20 // White

#define ENCA3 23 // Yellow
#define ENCB3 22 // White

#define ENCA4 17 // Yellow
#define ENCB4 16 // White

//full spool = 13 pulley revolutions
//define all motor pins
int dir1 = 33;    // [LOW/HIGH] High=positive torque (wind-up the cable)
int dir2 = 31;    // [LOW/HIGH] HIGH=positive torque
int dir3 = 30;    // [LOW/HIGH] 
int dir4 = 32;    // [LOW/HIGH] 
int speed1 = 12;  // [0:255]    0=slow, 255=fast
int speed2 = 10;  // [0:255]    0=fast, 255=slow
int speed3 = 9;   // [0:255]    0=fast, 255=slow 
int speed4 = 11;  // [0:255]    0=slow, 255=fast


float testSpeed = 140;
float testSpeed2 = 200.0;
float testSpeed3 = 50.0;
int counter = 0;
int divy = 100;
int flipFlop = 0;

// Define encoder data struct
struct Encoder {
  volatile int pos; //ISR stable counter
  int EA;           //hall affect pin A
  int EB;           //hall affect pin B
  };

// define all encoders
Encoder enc1 = {0, ENCA1, ENCB1};
Encoder enc2 = {0, ENCA2, ENCB2};
Encoder enc3 = {0, ENCA3, ENCB3};
Encoder enc4 = {0, ENCA4, ENCB4};

void initEncoders(){
  pinMode(enc1.EA, INPUT);
  pinMode(enc1.EB, INPUT);
  pinMode(enc2.EA, INPUT);
  pinMode(enc2.EB, INPUT);
  pinMode(enc3.EA, INPUT);
  pinMode(enc3.EB, INPUT);
  pinMode(enc4.EA, INPUT);
  pinMode(enc4.EB, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(enc1.EA),ISR_readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(enc2.EA),ISR_readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(enc3.EA),ISR_readEncoder3,RISING);
  attachInterrupt(digitalPinToInterrupt(enc4.EA),ISR_readEncoder4,RISING);
}

// interrupt service routine for encoder updates - Note: you can not pass variables to ISR because it is hardware driven, 
//                                                       you must hard code the check for each encoder 
void ISR_readEncoder1(){
  if (digitalReadFast(enc1.EB)) { enc1.pos --; }   //hand selected ++ vs -- based on mechanical configuration of encoder, ++ = spooling up
  else { enc1.pos ++; }
  }
void ISR_readEncoder2(){
  if (digitalReadFast(enc2.EB)) { enc2.pos ++; }
  else { enc2.pos --; }
  }
void ISR_readEncoder3(){
  if (digitalReadFast(enc3.EB)) { enc3.pos --; }
  else { enc3.pos ++; }
  }
void ISR_readEncoder4(){
  if (digitalReadFast(enc4.EB)) { enc4.pos ++; }
  else { enc4.pos --; }
  }


//define this function to invert PWM for motor needs
void remapPWM(){};


void setup() {
  
  initEncoders();
  



  /*
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(dir4, OUTPUT);
  pinMode(speed1, OUTPUT);
  pinMode(speed2, OUTPUT);
  pinMode(speed3, OUTPUT);
  pinMode(speed4, OUTPUT);
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  digitalWrite(dir3, LOW);
  digitalWrite(dir4, LOW);

  */
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //counter ++;
  //Serial.println(counter);

  int position1 = 0; 
  int position2 = 0; 
  int position3 = 0; 
  int position4 = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    position1 = enc1.pos;
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    position2 = enc2.pos;
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    position3 = enc3.pos;
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    position4 = enc4.pos;
  }

  // Print the headers first
  //Serial.println("Var1\tVar2\tVar3\tVar4");

  // Then print the variables with a tab in between for alignment
  Serial.print(" ");
  Serial.print(position1);
  Serial.print("\t ");

  // Check if the number is negative to add an extra space for alignment
  if(position2 >= 0) Serial.print(" ");
  Serial.print(position2);
  Serial.print("\t ");

  if(position3 >= 0) Serial.print(" ");
  Serial.print(position3);
  Serial.print("\t ");

  if(position4 >= 0) Serial.print(" ");
  Serial.println(position4);
 
  /*
  if (counter<1000){
    //divy = divy*2;
    //counter = 0;
    if (flipFlop == 0){
      digitalWrite(dir1, HIGH);
      digitalWrite(dir2, HIGH);
      digitalWrite(dir3, HIGH);
      digitalWrite(dir4, HIGH);
      analogWrite(speed1, 40);    //0=slow, 255=fast      
      analogWrite(speed2, 100);   //0=fast, 255=slow
      analogWrite(speed3, 60);    //0=fast, 255=slow 
      analogWrite(speed4, 40);    //0=slow, 255=fast
      //Serial.print("M1: ccw");

      //flipFlop = 1;
    }
    else{
      digitalWrite(dir4, LOW);
      analogWrite(speed4, testSpeed);
      Serial.print("M1: cw");
      //flipFlop = 1;

    }
  }
  else if (1000 <= counter < 1300) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    digitalWrite(dir3, LOW);
    digitalWrite(dir4, LOW);
    analogWrite(speed1, testSpeed);
    analogWrite(speed2, testSpeed);
    analogWrite(speed3, testSpeed);
    analogWrite(speed4, testSpeed);
  }
  else {
    analogWrite(speed1, 0);
    analogWrite(speed2, 255);
    analogWrite(speed3, 255);
    analogWrite(speed4, 0);

  }
  */

}








//void readEnc(){
//  int r = digitalRead(EncB)
//}