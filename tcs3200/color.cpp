#include "color.h"

    ColorSensor::ColorSensor(PinName ss2, PinName ss3, PinName sout):
    s2(ss2), s3(ss3), _out(sout)
    {
        s2.write(0);
        s3.write(0);
        interrupted = 12345;
        countR = counter;
        countG = counter;
        countB = counter;
        counter = 0;
        flag = 0;
        _out.mode(PullUp);
        _out.rise(this, &ColorSensor::incCount);
        ts.attach(this, &ColorSensor::getReading, .01);

    }


        /** Returns the Red intensity
        *@return the red intensity
        */
        int ColorSensor::getRed()
        {
            return countR;
        }

        /**Returns the blue intensity
        *@return the blue intensity
        */
        int ColorSensor::getBlue()
        {
            return countB;
        }

        /**Returns the Green intensity
        *
        *@return the green intensity
        */
        int ColorSensor::getGreen()
        {
            return countG;
        }

        /**Used in the PDM calculation
        */
        void ColorSensor::incCount()
        {
            counter++;
        }

        /**Cycles through the channels to get the reading from the R, G, and B channels
        */
        void ColorSensor::getReading()
        {

            flag++;
            if(flag == 1)
            {
                countR = counter;
                s2.write(1);
                s3.write(1);
            }
            else if(flag == 2)
            {
                countG = counter;
                s2.write(0);
                s3.write(1);
            }
            else if(flag ==3)
            {
                countB = counter;
                s2.write(0);
                s3.write(0);
            }
            else if(flag == 4)
            {
                flag = 0;
            }
            counter = 0;

        }

        /*
        * Wrapper for getting a reading
        */
        void ColorSensor::poll()
        {

            ColorSensor::getReading();

        }
