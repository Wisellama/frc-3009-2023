#pragma once

#include <frc/XboxController.h>

class Controls {
    public:
    Controls(int xboxPort1, int xboxPort2, double deadband);
    ~Controls();

    double DriveForward();
    double DriveStrafe();
    double DriveRotate();
    
    bool ExtraWheels();
    bool ClawClamp();
    bool ArmExtend();
    bool ParkingBrake();

    double ArmRaise();
    double Wrist();

    bool DirectDriveArm();
    bool ReflectiveTapeMode();
    bool AprilTagMode();


    private:
    frc::XboxController *m_controller1;
    frc::XboxController *m_controller2;
    double m_deadband;

    double deadband(double value);
    double clampedDeadband(double value);
};