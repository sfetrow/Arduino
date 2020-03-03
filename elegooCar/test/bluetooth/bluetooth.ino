#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define LED 13
#define blue 10
#define red 12
#define buzzer 3

int counter = 0;
bool blueLight = true;
bool moving = true;

unsigned char carSpeed = 250;
bool state = LOW;

void forward() {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Forward");
}

void back() {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Back");
}

void left() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Left");
}

void right() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right");
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
}

void stateChange() {
  state = !state;
  digitalWrite(LED, state);
  Serial.println("Light");
}

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(buzzer, OUTPUT);
  stop();
}

void loop() {
  if (Serial.available())
  {
    char getstr = Serial.read();
    switch (getstr) {
      case 'f': forward(); break;
      case 'b': back();   break;
      case 'l': left();   break;
      case 'r': right();  break;
      case 's': stop();   break;
      case 'a': stateChange(); break;
      default:  break;
    }
  }
  if (moving){
    if (counter < 100) {
      delay (1);
      if (blueLight == true) {
        digitalWrite(blue, HIGH);   // turn the LED on (HIGH is the voltage level)
        digitalWrite(red, LOW);
      } else {
        digitalWrite(blue, LOW);
        digitalWrite(red, HIGH);   // turn the LED on (HIGH is the voltage level)
      }
      counter ++;
    } else {
      blueLight = !blueLight;
      counter = 0;
    }
    if (blueLight) {
      tone(buzzer, 912);
    } else {
      tone(buzzer, 635);
    }
  }
}
