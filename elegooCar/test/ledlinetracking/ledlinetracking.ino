//www.elegoo.com

//Line Tracking IO define
#define LT_R !digitalRead(10)
#define LT_M !digitalRead(4)
#define LT_L !digitalRead(2)

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

#define carSpeed 150

void forward() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go forward!");
}

void back() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go back!");
}

void left() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go left!");
}

void right() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go right!");
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
}

void setup() {
  Serial.begin(9600);
  pinMode(10, INPUT);
  pinMode(4, INPUT);
  pinMode(2, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(11, OUTPUT);
}

void loop() {
  if (LT_M) {
    forward();
  }
  else if (LT_R) {
    right();
    Serial.println("Turning Right");
    while (LT_R);
  }
  else if (LT_L) {
    left();
    Serial.println("Turning Left");
    while (LT_L);
  }


  if (LT_L) {
    Serial.println("Blue LED ON");
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else if (LT_R) {
    Serial.println("Yellow LED ON");
    digitalWrite(11, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else {
    Serial.println("LED OFF");
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage
    digitalWrite(11, LOW);    // turn the LED off by making the voltage
  }
}
