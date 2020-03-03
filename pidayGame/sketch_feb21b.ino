int blinkSpeed = 10;
int incomingByte = 0; // for incoming serial data
bool buttonState = false;
bool buttonLast = false;

int currentLED = 2;
bool LEDdirection = false;

void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT_PULLUP);
  //
  //  Serial.begin(9600); // Start a serial connection
  //  for (int i = 2; i <= 6; i++) {
  //    pinMode(i, OUTPUT);
  //  }
}
void loop() {
  for (int i = 2; i <= 6; i++) {
    digitalWrite(i, LOW);
  }

 x buttonState = digitalRead(7);
  if (!buttonState) {
    if (currentLED == 2) {
      for (int i = 2; i <= 6; i++) {
        digitalWrite(i, HIGH);
      }
    }
  } else {

  }

  digitalWrite(currentLED, HIGH);

  if (currentLED >= 6 || currentLED <= 2) {
    LEDdirection = !LEDdirection;
  }

  if (LEDdirection) {
    currentLED++;
  } else {
    currentLED--;
  }




  delay(500);


  //  if (Serial.available() > 0) {
  //    // read the incoming byte:
  //    incomingByte = Serial.read();
  //    // say what you got:
  //    Serial.print("I received: ");
  //    Serial.println(incomingByte, DEC);
  //  }
  //  for (int i = 2; i <= 6; i++) {
  //    digitalWrite(i, HIGH);
  //    delay(100);
  //    digitalWrite(i, LOW);
  //  }
}
