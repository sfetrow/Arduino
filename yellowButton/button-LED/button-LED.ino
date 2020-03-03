
//Constants won't change. 
const int buttonPin = 2;  // the number of the pushbutton pin
const int ledPin = 13;    // the number of the LED pin

// this varable will change
int buttonState = 0; // this varable will read what the button status is

void setup() {
//set's the LED pin as an output:
pinMode(ledPin,OUTPUT);
//set's the pushbutton pin as an input:
pinMode(buttonPin, INPUT);
}

void loop() {
  //reads the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

//checks is the puchbutton is pressed. If it is, the buttonState is HIGH:
    if (buttonState == HIGH){
    // turn LED on:
    digitalWrite(ledPin, HIGH);
   } else {
    //turn LED off:
    digitalWrite(ledPin, LOW);
  }
}
