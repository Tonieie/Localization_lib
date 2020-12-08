#include <Arduino.h>
#include <Localization.h>

Localize::Localize(uint8_t pin_A1, uint8_t pin_B1, uint8_t pin_A2, uint8_t pin_B2, uint8_t pin_A3, uint8_t pin_B3) //Overloading Constructor for 3 omni wheels
{
    wheel_num = 3;
    // top right -> top left -> bottom
    Enc_A = new Encoder(pin_A1, pin_B1);
    Enc_B = new Encoder(pin_A2, pin_B2);
    Enc_C = new Encoder(pin_A3, pin_B3);
}

// Localize(uint8_t pin_A1, uint8_t pin_B1, uint8_t pin_A2, uint8_t pin_B2, uint8_t pin_A3, uint8_t pin_B3, uint8_t pin_A4, uint8_t pin_B4) //Overloading Constructor for 4 omni wheels or meccanum
// {
//     wheel_num = 4;
//     Encoder passer_Enc_A(pin_A1, pin_B1), passer_Enc_B(pin_A2, pin_B2), passer_Enc_C(pin_A3, pin_B3), passer_Enc_D(pin_A4, pin_B4);
//     Enc_A = &passer_Enc_A;
//     Enc_B = &passer_Enc_B;
//     Enc_C = &passer_Enc_C;
//     Enc_D = &passer_Enc_D;
// }

float Localize::cosd(float ceta)
{
    return cos(ceta * M_PI / 180.0f);
}

float Localize::sind(float ceta)
{
    return sin(ceta * M_PI / 180.0f);
}

template <typename T>
int Localize::sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void Localize::setCoef(float xcf, float _cf, float hcf) //Set each coefficients for localization
{
    this->xcf = xcf;
    this->ycf = ycf;
    this->hcf = hcf;
}

void Localize::setPID_head(float kp, float ki, float kd)
{
    this->pid_heading.kp = kp;
    this->pid_heading.ki = ki;
    this->pid_heading.kd = kd;
}

void Localize::setPID_track(float kp, float ki, float kd)
{
    this->pid_tracking.kp = kp;
    this->pid_tracking.ki = ki;
    this->pid_tracking.kd = kd;
}

void Localize::updatePos() //Update positon of the robot
{
    if (wheel_num == 3)
    {

        static int32_t last_odom_A = 0, last_odom_B = 0, last_odom_C = 0;
        const int32_t pulse_A = Enc_A->read();
        const int32_t pulse_B = Enc_B->read();
        const int32_t pulse_C = Enc_C->read();
        // Serial.printf("pulse : %d %d %d\n",pulse_A,pulse_B,pulse_C);

        const int32_t odom_A = pulse_A - last_odom_A;
        const int32_t odom_B = pulse_B - last_odom_B;
        const int32_t odom_C = pulse_C - last_odom_C;
        // Serial.printf("odom : %d %d %d\n",odom_A,odom_B,odom_C);
        last_odom_A = pulse_A;
        last_odom_B = pulse_B;
        last_odom_C = pulse_C;

        const float dx = ((odom_A + odom_B) - (2 * odom_C)) * xcf;
        const float dy = (-odom_A + odom_B) * ycf;
        head = (pulse_A + pulse_B + pulse_C) * hcf;
        if (hcf != 1.00)
        {
            head = fmod(fabs(head), 360.0f) * sgn(head);
        }

        pos_x += (dx * cosd(head)) - (dy * sind(head));
        pos_y += (dx * sind(head)) + (dy * cosd(head));
        Serial.printf("%f %f %f\n", pos_x, pos_y, head);
    }
}

double Localize::getX()
{
    updatePos();
    return pos_x;
}

double Localize::getY()
{
    updatePos();
    return pos_y;
}

double Localize::getHead()
{
    updatePos();
    return head;
}

// bool Localize::p2p_tracking(float max_vel, float tx, float ty, float th, float tolerant)
// {
//     static uint8_t circle = 0;
//     static float last_dist = 0.0f,sum_dist = 0.0f,last_diff_head = 0.0f,sum_diff_head = 0.0f;

//     this->updatePos();
//     const float dx = tx - pos_x;
//     const float dy = ty - pos_y;
//     const float dist = sqrt((dx*dx) + (dy*dy));
//     const float dir = atan2(dy,dx) * 180.0f / M_PI;

//     const float compensate_dir = dir - head;

//     const float diff_head = th - head;
//     const float cmd_omega = (diff_head * pid_heading.kp) + (sum_diff_head * pid_heading.ki) + (last_diff_head * pid_heading.kd);
//     last_diff_head = diff_head;
//     sum_diff_head += diff_head;

//     const float cmd_vel = (dist * pid_tracking.kp) + (sum_dist * pid_tracking.ki) + (last_dist * pid_tracking.kd);
//     last_dist = dist;
//     sum_dist += dist;

// }