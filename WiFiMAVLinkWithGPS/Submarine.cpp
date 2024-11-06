#include "Submarine.hpp"

Submarine::Submarine(int power1Pin, int switch1APin, int switch1BPin, int power2Pin, int switch2APin, int switch2BPin)
{
    POWER_1 = power1Pin;
    SWITCH_1A = switch1APin;
    SWITCH_1B = switch1BPin;
    POWER_2 = power2Pin;
    SWITCH_2A = switch2APin;
    SWITCH_2B = switch2BPin;
}

void Submarine::setup()
{
    powerSetup();
}

void Submarine::powerOn(int num)
{
    digitalWrite(num, HIGH);
}

void Submarine::powerOff(int num)
{
    digitalWrite(num, LOW);
}

void Submarine::switchOn()
{
    digitalWrite(SWITCH_1A, HIGH);
    digitalWrite(SWITCH_1B, LOW);
}

void Submarine::switchOff()
{
    digitalWrite(SWITCH_1A, LOW);
    digitalWrite(SWITCH_1B, LOW);
}

void Submarine::switchOn2()
{
    digitalWrite(SWITCH_2A, HIGH);
    digitalWrite(SWITCH_2B, LOW);
}

void Submarine::switchOff2()
{
    digitalWrite(SWITCH_2A, LOW);
    digitalWrite(SWITCH_2B, LOW);
}

void Submarine::clawClose()
{
    switchOn();
    delay(100);
    powerOn(POWER_1);
    delay(400);
    powerOff(POWER_1);
    delay(100);
    switchOff();
}

void Submarine::up()
{
    switchOff();
    delay(100);
    powerOn(POWER_1);
    delay(900);
    powerOff(POWER_1);
    delay(100);
    switchOn();
}

void Submarine::clawOpen()
{
    switchOn2();
    delay(100);
    powerOn(POWER_2);
    delay(500);
    powerOff(POWER_2);
    delay(100);
    switchOff2();
}

void Submarine::down()
{
    switchOff2();
    delay(100);
    powerOn(POWER_2);
    delay(900);
    powerOff(POWER_2);
    delay(100);
    switchOn2();
}

void Submarine::powerSetup()
{
    digitalWrite(POWER_1, LOW);
    digitalWrite(SWITCH_1A, LOW);
    digitalWrite(SWITCH_1B, LOW);
    pinMode(POWER_1, OUTPUT);
    pinMode(SWITCH_1A, OUTPUT);
    pinMode(SWITCH_1B, OUTPUT);
    
    digitalWrite(POWER_2, LOW);
    digitalWrite(SWITCH_2A, LOW);
    digitalWrite(SWITCH_2B, LOW);
    pinMode(POWER_2, OUTPUT);
    pinMode(SWITCH_2A, OUTPUT);
    pinMode(SWITCH_2B, OUTPUT);
}
