#ifndef Localization_h
#define Localization_h

#include <Arduino.h>
#include <Encoder.h>

class Localize
{
    private :
        
        Encoder *Enc_A,*Enc_B,*Enc_C,*Enc_D;
        uint8_t wheel_num;
        double pos_x = 0.0f,pos_y = 0.0f,head = 0.0f;
        float xcf = 1.00f,ycf = 1.00f,hcf = 1.00f;

        float cosd(float ceta)
        {
            return cos(ceta * M_PI / 180.0f);
        }
        float sind(float ceta)
        {
            return sin(ceta * M_PI / 180.0f);
        }
        template <typename T>int sgn(T val)
        {
            return (T(0) < val) - (val < T(0));
        }

    public :

        Localize(uint8_t pin_A1,uint8_t pin_B1,uint8_t pin_A2,uint8_t pin_B2,uint8_t pin_A3,uint8_t pin_B3) //Overloading Constructor for 3 omni wheels
        {
            wheel_num = 3;
            Encoder passer_Enc_A(pin_A1,pin_B1),passer_Enc_B(pin_A2,pin_B2),passer_Enc_C(pin_A3,pin_B3); // top right -> top left -> bottom
            Enc_A = &passer_Enc_A;
            Enc_B = &passer_Enc_B;
            Enc_C = &passer_Enc_C;
        }

        // Localize(uint8_t pin_A1,uint8_t pin_B1,uint8_t pin_A2,uint8_t pin_B2,uint8_t pin_A3,uint8_t pin_B3,uint8_t pin_A4,uint8_t pin_B4) //Overloading Constructor for 4 omni wheels or meccanum
        // {
        //     wheel_num = 4;
        //     Encoder passer_Enc_A(pin_A1,pin_B1),passer_Enc_B(pin_A2,pin_B2),passer_Enc_C(pin_A3,pin_B3),passer_Enc_D(pin_A4,pin_B4);
        //     Enc_A = &passer_Enc_A;
        //     Enc_B = &passer_Enc_B;
        //     Enc_C = &passer_Enc_C;
        //     Enc_D = &passer_Enc_D;
        // }

        void setCoef(float _xcf,float _ycf,float _hcf) //Set each coefficients for localization
        {
            xcf = _xcf;
            ycf = _ycf;
            hcf = _hcf;
        }

        void update() //Update positon of the robot
        {
            if(wheel_num == 3)
            {
                static int32_t last_odom_A = 0,last_odom_B = 0,last_odom_C = 0;

                const int32_t pulse_A = Enc_A->read();
                const int32_t pulse_B = Enc_B->read();
                const int32_t pulse_C = Enc_C->read();

                const int32_t odom_A = pulse_A - last_odom_A;
                const int32_t odom_B = pulse_B - last_odom_B;
                const int32_t odom_C = pulse_C - last_odom_C;

                last_odom_A = odom_A;
                last_odom_B = odom_B;
                last_odom_C = odom_C;

                const float dx = ((odom_A + odom_B) - (2 * odom_C)) * xcf;
                const float dy = (-odom_A + odom_B) * ycf;
                head = (odom_A + odom_B + odom_C) * hcf;
                head = fmod(fabs(head), 360.0f) * sgn(head);

                pos_x += (dx * cosd(head)) - (dy *  sind(head));
                pos_y += (dx *  sind(head)) + (dy *  cosd(head));
            }
        }

        double getX()
        {
            return pos_x;
        }
        double getY()
        {
            return pos_y;
        }
        double getHead()
        {
            return head;
        }
};

#endif