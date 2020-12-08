#ifndef Localization_h
#define Localization_h

#include <Arduino.h>
#include <Encoder.h>

class Localize
{
private:

    typedef struct PID
    {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
    };
    PID pid_tracking,pid_heading;

    Encoder *Enc_A = new Encoder(NULL,NULL), *Enc_B = new Encoder(NULL,NULL), *Enc_C = new Encoder(NULL,NULL), *Enc_D = new Encoder(NULL,NULL);
    uint8_t wheel_num;
    double pos_x = 0.0f, pos_y = 0.0f, head = 0.0f;
    float xcf = 1.00f, ycf = 1.00f, hcf = 1.00f;
    

    float cosd(float);
    float sind(float);

    template <typename T>
    int sgn(T);

public:
    Localize(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t); //Overloading Constructor for 3 omni wheels
    // Localize(uint8_t pin_A1,uint8_t pin_B1,uint8_t pin_A2,uint8_t pin_B2,uint8_t pin_A3,uint8_t pin_B3,uint8_t pin_A4,uint8_t pin_B4); //Overloading Constructor for 4 omni wheels or meccanum

    void setCoef(float, float, float); //Set each coefficients for localization
    void setPID_track(float,float,float);
    void setPID_head(float,float,float);
    void updatePos();                     //Update positon of the robot
    double getX();
    double getY();
    double getHead();

    // bool p2p_tracking(float, float, float, float, float);
};

#endif