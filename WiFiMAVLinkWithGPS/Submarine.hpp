#ifndef Submarine_h
#define Submarine_h

#include <Arduino.h>

class Submarine
{
public:
    Submarine(int power1APin, int power1BPin, int switch1APin, int switch1BPin, int power2APin, int power2BPin, int switch2APin, int switch2BPin);
    void setup();
    void clawClose();
    void clawOpen();
    void up();
    void down(); 

    void powerOn(int num);
    void powerOff(int num);
    void switchOn();
    void switchOff();
    void switchOn2();
    void switchOff2();

private:
    int POWER_1A;
    int POWER_1B;
    int SWITCH_1A;
    int SWITCH_1B;
    int POWER_2A;
    int POWER_2B;
    int SWITCH_2A;
    int SWITCH_2B;
    
    void powerSetup();

    
};

#endif
