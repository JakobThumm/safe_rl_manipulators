#ifndef SYSTEM_H
#define SYSTEM_H

class System{
    private:
    public:
        double measurement_error_pos;
        double measurement_error_vel;
        double delay;

        System();

        System(double measurement_error_pos, double measurement_error_vel, double delay);

        ~System();
};
#endif