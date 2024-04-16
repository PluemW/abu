#include <Arduino.h>
#include <PWMServo.h>

PWMServo r1;
PWMServo r2;
PWMServo r3;
PWMServo r4;
int pos = 80;

void setup()
{
    pinMode(32,OUTPUT);
    // r1.attach(22);
    // r2.attach(23);
    // r3.attach(24);
    r4.attach(25);
}

void loop()
{
    digitalWrite(32,1);
    // r1.write(90);
    // r2.write(90);
    // r3.write(90);
    r4.write(90);
}