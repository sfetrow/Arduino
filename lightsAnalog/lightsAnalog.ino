#define blue 10
#define yellow 12
#define green 11
#define red 13

int dial = 0;
int speed = 50;
const int buttonPin = 9;

bool buttonState = 0;
bool lastState = 0;

void setup() {
  Serial.begin(9600);
  pinMode(blue, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(buttonPin, INPUT);
}

void loop() {
  buttonState = digitalRead(buttonPin);
  dial = analogRead(0);
  Serial.println(dial);
  if (dial < 256) {
    runLights(0);
  } else if (dial < 512) {
    runLights(1);
  }
  else if (dial < 768) {
    runLights(2);
  }
  else {
    runLights(3);
  }
}

void runLights(int c) {
  switch (c) {
    case 0:
      digitalWrite(blue, HIGH);
      delay(speed);
      digitalWrite(blue, LOW);
      Serial.println("blue");

      digitalWrite(yellow, HIGH);
      delay(speed);
      digitalWrite(yellow, LOW);
      Serial.println("yellow");

      digitalWrite(green, HIGH);
      delay(speed);
      digitalWrite(green, LOW);
      Serial.println("green");

      digitalWrite(red, HIGH);
      delay(speed);
      digitalWrite(red, LOW);
      Serial.println("red");
      break;

    case 1:
      digitalWrite(green, HIGH);
      delay(speed);
      digitalWrite(green, LOW);

      digitalWrite(yellow, HIGH);
      delay(speed);
      digitalWrite(yellow, LOW);

      digitalWrite(blue, HIGH);
      delay(speed);
      digitalWrite(blue, LOW);

      digitalWrite(red, HIGH);
      delay(speed);
      digitalWrite(red, LOW);
      break;

    case 2:
      digitalWrite(yellow, HIGH);
      delay(speed);
      digitalWrite(yellow, LOW);

      digitalWrite(red, HIGH);
      delay(speed);
      digitalWrite(red, LOW);

      digitalWrite(green, HIGH);
      delay(speed);
      digitalWrite(green, LOW);

      digitalWrite(blue, HIGH);
      delay(speed);
      digitalWrite(blue, LOW);
      break;

    case 3:
      break;
  }

}
