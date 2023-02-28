#include <Joystick.h>

Joystick_ Joystick;
void setup() {
    // initialize button pins
    Joystick.begin();
    pinMode(1, INPUT_PULLUP); // UP BUTTON
    pinMode(2, INPUT_PULLUP); // RETRACT INTAKE
    pinMode(3, INPUT_PULLUP); // RELEASE INTAKE
    pinMode(4, INPUT_PULLUP); // STOW BUTTON
    pinMode(5, INPUT_PULLUP); // MIDDLE BUTTON
    pinMode(6, INPUT_PULLUP); // RIGHT BUTTON
    pinMode(7, INPUT_PULLUP); // CENTER BUTTON
    pinMode(8, INPUT_PULLUP); // LEFT BUTTON
    pinMode(9, INPUT_PULLUP); // DOWN BUTTON
    pinMode(10, INPUT_PULLUP); // PURPLE BUTTON
    pinMode(11, INPUT_PULLUP); // YELLOW BUTTON
    pinMode(12, INPUT_PULLUP); // BALANCE BUTTON
    pinMode(LED_BUILTIN, OUTPUT);
}

struct pin_value { // data type used to store the button state
    int pin;
    bool value; // value tracks wether the button is pressed, don't omit this
};

pin_value pin_v[] = {{1, false}, {2, false}, {3, false}, {4, false}, {5, false}, {6, false}, {7, false}, {8, false}, {9, false}, {10, false}, {11, false}, {12, false}};

void loop() {
    for(int i=0; i<=11; i++) {    
        bool reading = digitalRead(pin_v[i].pin); // false if the button is pressed
        if(!reading && !pin_v[i].value) { // if button is pressed and its not sending the button signal
            Joystick.pressButton(pin_v[i].pin-1);
            digitalWrite(LED_BUILTIN, HIGH);
            pin_v[i].value = true;
        } 
        else if(reading && pin_v[i].value) { // if button is not pressed but it is sending the button signal
            Joystick.releaseButton(pin_v[i].pin-1);
            digitalWrite(LED_BUILTIN, LOW);
            pin_v[i].value = false;
        }
    }
}