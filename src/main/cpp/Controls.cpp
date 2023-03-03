#include "Controls.h"

#include <algorithm>

#include <frc/XboxController.h>

Controls::Controls(int xboxPort1, int xboxPort2, double deadband) {
    m_controller1 = new frc::XboxController(xboxPort1);
    m_controller2 = new frc::XboxController(xboxPort2);
    m_deadband = deadband;
}

// This is the destructor. It's basically unnecessary due to the way we're coding on a Robot,
// but good practice to define how to cleanup your class if needed.
Controls::~Controls() {
    if (m_controller1 != nullptr) {
        delete m_controller1;
    }
    if (m_controller2 != nullptr) {
        delete m_controller2;
    }
}

// deadband lets you account for slop or a deadzone in the controller inputs.
// Anything below the deadband value will be ignored.
double Controls::deadband(double value) {
      if (std::abs(value) > m_deadband) {
        return value;
    } else {
        return 0.0;
    }
}

// clampedDeadband will apply the deadband() function to the value and then clamp the output between -1 and 1.
double Controls::clampedDeadband(double value) {
    double min = -1.0;
    double max = 1.0;
    return std::clamp(deadband(value), min, max);
}

double Controls::DriveForward() {
    frc::XboxController *c = m_controller1;

    double value = -1 * c->GetLeftY();

    if (!m_turbo) {
        value = clampedDeadband(value);
    }

    return value;
}

double Controls::DriveStrafe() {
    frc::XboxController *c = m_controller1;
    double strafeSpeed = 0.5;

    double value = 0.0;
    if (c->GetLeftBumper()) {
        value = -1 * strafeSpeed;
    } else if (c->GetRightBumper()) {
        value = strafeSpeed;
    }

    return clampedDeadband(value);
}

double Controls::DriveRotate() {
    frc::XboxController *c = m_controller1;

    double value = c->GetRightX();

    return clampedDeadband(value);
}

bool Controls::ExtraWheels() {
    frc::XboxController *c = m_controller1;
    return c->GetXButtonPressed();
}

bool Controls::ParkingBrake() {
    frc::XboxController *c = m_controller1;
    return c->GetYButtonPressed();
}

bool Controls::ClawClamp() {
    frc::XboxController *c = m_controller2;
    return c->GetAButtonPressed();
}

bool Controls::ArmExtend() {
    frc::XboxController *c = m_controller2;
    return c->GetYButtonPressed();
}

double Controls::ArmRaise() {
    frc::XboxController *c = m_controller2;

    double limit = 1.0;

    double leftTrigger = c->GetLeftTriggerAxis();
    double rightTrigger = c->GetRightTriggerAxis();

    leftTrigger = leftTrigger * limit;
    rightTrigger = rightTrigger * limit;

    double value = 0.0;

    if (rightTrigger > 0) {
        value = -1 * rightTrigger;
    } else if (leftTrigger > 0) {
        value = leftTrigger;
    }

    return clampedDeadband(value);
}

double Controls::Wrist() {
    frc::XboxController *c = m_controller2;

    double value = c->GetLeftY();
    return clampedDeadband(value);
}

bool Controls::ReflectiveTapeMode() {
    frc::XboxController *c = m_controller1;

    return c->GetBackButtonPressed();
}

bool Controls::AprilTagMode() {
    frc::XboxController *c = m_controller1;

    return c->GetStartButtonPressed();
}

bool Controls::DirectDriveArm() {
    frc::XboxController *c = m_controller2;

    return c->GetStartButtonPressed();
}

bool Controls::Turbo() {
    frc::XboxController *c = m_controller1;

    m_turbo = c->GetRightTriggerAxis() > m_deadband;
    
    return m_turbo;
}
bool Controls::ClawPressure() {
    frc::XboxController *c = m_controller2;

    return c->GetXButtonPressed();
}    
    
