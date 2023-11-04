#define ENCA 2 // Yellow
#define ENCB 3 // White

//full spool = 13 pulley revolutions
//Motor 1
int dir1 = 8;
int speed1 = 9;
//High=positive torque
//0=slow, 255=fast

//Motor 2
int dir2 = 7;
int speed2 = 6;
//HIGH=positive torque (wind up the cable)
//0=fast, 255=slow

//Motor3
int dir3 = 4;
int speed3 = 5;
//HIGH = positive torque
//0=fast, 255=slow 

//Motor4
int dir4 = 12;
int speed4 = 10;
//HIGH = positive torque
//0 = slow, 255 = fast

float testSpeed = 140;
float testSpeed2 = 200.0;
float testSpeed3 = 50.0;
int counter = 0;
int divy = 100;
int flipFlop = 0;

void setup() {
  // put your setup code here, to run once:
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


  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  counter ++;
  Serial.println(counter);

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
  

}

//void readEnc(){
//  int r = digitalRead(EncB)
//}
