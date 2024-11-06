#ifndef Submarine_h
#define Submarine_h

#include <Arduino.h>

class Submarine
{
public:
    Submarine(int power1Pin, int switch1APin, int switch1BPin, int power2Pin, int switch2APin, int switch2BPin);
    
    void setup();
    void clawClose();
    void clawOpen();
    void up();
    void down(); 

private:
    int POWER_1;
    int SWITCH_1A;
    int SWITCH_1B;
    int POWER_2;
    int SWITCH_2A;
    int SWITCH_2B;
    
    void powerSetup();

    void powerOn(int num);
    void powerOff(int num);
    void switchOn();
    void switchOff();
    void switchOn2();
    void switchOff2();
};

#endif
