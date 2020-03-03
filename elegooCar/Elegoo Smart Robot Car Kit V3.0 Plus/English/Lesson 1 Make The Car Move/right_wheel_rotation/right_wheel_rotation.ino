//www.elegoo.com

//     Right motor truth table
//Here are some handy tables to show the various modes of operation.
//  ENB         IN3             IN4         Description
//  LOW   Not Applicable   Not Applicable   Motor is off
//  HIGH        LOW             LOW         Motor is stopped (brakes)
//  HIGH        LOW             HIGH        Motor is on and turning forwards
//  HIGH        HIGH            LOW         Motor is on and turning backwards
//  HIGH        HIGH            HIGH        Motor is stopped (brakes)

// define IO pin
#define ENB 6
#define IN3 9
#define IN4 11
// define IO pin
#define ENA 5
#define IN1 7
#define IN2 8

//init the car
void setup() {
  pinMode(IN3, OUTPUT); //set IO pin mode OUTPUT
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  digitalWrite(ENB, HIGH);  //Enable right motor
  pinMode(IN1, OUTPUT);   //set IO pin mode OUTPUT
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);//Enable left motor
}

//mian loop
void loop() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); //Right wheel turning forwards
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);//Right wheel turning forwards
  delay(500);             //delay 500ms
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); //Right wheel stoped
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW); //Right wheel stoped
  delay(500);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);//Right wheel turning backwards
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); //Right wheel turning backwards
  delay(500);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH); //Right wheel stoped
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH); //Right wheel stoped
  delay(500);
}
