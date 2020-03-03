#define blue 13
#define red 12
#define buzzer 3

int counter = 0;
bool blueLight = true;

void setup() {
  // put your setup code here, to run once:
  pinMode(blue, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(buzzer, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
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
