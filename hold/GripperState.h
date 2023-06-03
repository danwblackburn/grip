#ifndef GRIPPERSTATE_H
#define GRIPPERSTATE_H

class GripperState
{
    private:
        double sector_angle = 0;
        
        double x_finger = 0;
        double last_x_finger = 0;
        double v_finger = 0;

        double desired_force = 0;
        double torque_cmd = 0;


        static const double r_finger = 0.1651;  //  [m] or 6.5 in
        static const double r_pulley = 0.0095;  //  [m]
        static const double r_sector = 0.075;   //  [m]
        static const double gear_ratio = 5 / 3;

        void UpdateVelocity(double dt)
        {
            float velocity_estimate = (x_finger - last_x_finger) / dt;
            v_finger = 0.9 * v_finger + 0.1 * velocity_estimate;
        }

        void UpdateTorqueCmd()
        {
            torque_cmd = r_finger * r_pulley / (gear_ratio * r_sector) * desired_force;
        }

    public:
        GripperState() {}

        void UpdateState(double sector_angle, double dt)
        {
            this->sector_angle = sector_angle;
            last_x_finger = x_finger;
            x_finger = SectorAngleToFingerPos(sector_angle);
            UpdateVelocity(dt);
        }

        double GetFingerPos()
        {
            return x_finger;
        }

        double SectorAngleToFingerPos(double sector_angle)
        {
            return r_finger * r_pulley / (gear_ratio * r_sector) * sector_angle;
        }

        void SetDesiredForce(double force)
        {
            this->desired_force = force;
            UpdateTorqueCmd();
        }

        double GetTorqueCmd()
        {
            return torque_cmd;
        }
};

#endif
