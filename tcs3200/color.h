#include "mbed.h"

class ColorSensor{
    public:
    ColorSensor(PinName ss2, PinName ss3, PinName sout);
    DigitalOut s2;
    DigitalOut s3;
    InterruptIn _out;
    Ticker ts;
    void poll();
    int interrupted;
    int countR;
    int countG;
    int countB;
    int counter;
    int flag;
    int getRed();
    int getGreen();
    int getBlue();
    void getReading();
    void incCount();

    private:


    protected:


    };
