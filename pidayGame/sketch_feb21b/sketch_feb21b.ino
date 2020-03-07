const byte MINLED = 2;
const byte MAXLED = 6;
const byte BUTTONPIN = 7;

bool buttonState = false;
bool buttonLast = false;
bool LEDdirection = false;

int counter = 0;
int currentLED = 0;
int LEDspeed = 100;

void setup() {
  //Start a serial connection
  Serial.begin(9600);

  // for loop to set LED pins to OUTPUTs
  for (byte i = MINLED; i <= MAXLED; i++) {
    pinMode(i, OUTPUT); //Special LED
  }

  // Set button pin to INPUT
  pinMode(BUTTONPIN, INPUT_PULLUP); //Button
}
void loop() {
//rollover
  if (currentLED > MAXLED) {
    LEDdirection = !LEDdirection;
    counter -= LEDspeed;
  } else if (currentLED < MINLED) {
    LEDdirection = !LEDdirection;
    counter += LEDspeed;
  }else{
    
  }

  //Divide Counter
  currentLED = counter / LEDspeed;
  currentLED += MINLED;
  
  //Turn off all LEDs
  for (byte i = MINLED; i <= MAXLED; i++) {
    digitalWrite(i, LOW);
  }
  digitalWrite(currentLED, HIGH);

  //Read Button
  buttonState = digitalRead(BUTTONPIN);

  //Soft Toggle
  if (buttonState != buttonLast) {
    if (!buttonState) {
      //If button is pressed
      if (currentLED == MINLED) {
        Serial.println("Target hit!");
      } else {
        Serial.println("Target missed!");
      }
    } else {
      //If button is released
    }
    //do the thing here
    buttonLast = buttonState;
  }
  delay(1);
}
