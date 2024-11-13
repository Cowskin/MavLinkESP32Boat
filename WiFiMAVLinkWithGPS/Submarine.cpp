#include "Submarine.hpp"

Submarine::Submarine(int power1APin, int power1BPin, int switch1APin, int switch1BPin, int power2APin, int power2BPin, int switch2APin, int switch2BPin)
{
    POWER_1A = power1APin;
    POWER_1B = power1BPin;
    SWITCH_1A = switch1APin;
    SWITCH_1B = switch1BPin;
    POWER_2A = power2APin;
    POWER_2B = power2BPin;
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
    delay(50);
    powerOn(POWER_1A);
    powerOff(POWER_1B);
    delay(400);
    powerOff(POWER_1A);
    powerOff(POWER_1B);
    delay(50);
    switchOff();
}

void Submarine::up()
{
    switchOff();
    delay(50);
    powerOn(POWER_1A);
    powerOff(POWER_1B);
    delay(500);
    powerOff(POWER_1A);
    powerOff(POWER_1B);
    delay(50);
    switchOn();
}

void Submarine::clawOpen()
{
    switchOn2();
    delay(50);
    powerOn(POWER_2A);
    powerOff(POWER_2B);
    delay(400);
    powerOff(POWER_2A);
    powerOff(POWER_2B);
    delay(50);
    switchOff2();
}

void Submarine::down()
{
    switchOff2();
    delay(50);
    powerOn(POWER_2A);
    powerOff(POWER_2B);
    delay(500);
    powerOff(POWER_2A);
    powerOff(POWER_2B);
    delay(50);
    switchOn2();
}

void Submarine::powerSetup()
{
    pinMode(POWER_1A, OUTPUT);
    pinMode(POWER_1B, OUTPUT);
    pinMode(SWITCH_1A, OUTPUT);
    pinMode(SWITCH_1B, OUTPUT);
    digitalWrite(POWER_1A, LOW);
    digitalWrite(POWER_1B, LOW);
    digitalWrite(SWITCH_1A, LOW);
    digitalWrite(SWITCH_1B, LOW);
  
    pinMode(POWER_2A, OUTPUT);
    pinMode(POWER_2B, OUTPUT);
    pinMode(SWITCH_2A, OUTPUT);
    pinMode(SWITCH_2B, OUTPUT);
    digitalWrite(POWER_2A, LOW);
    digitalWrite(POWER_2B, LOW);
    digitalWrite(SWITCH_2A, LOW);
    digitalWrite(SWITCH_2B, LOW);
}
