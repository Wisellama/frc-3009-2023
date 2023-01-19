// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/XboxController.h>
// #include "frc/smartdashboard/Smartdashboard.h"
// #include "networktables/NetworkTable.h"
// #include "networktables/NetworkTableInstance.h"
// #include "networktables/NetworkTableEntry.h"
// #include "networktables/NetworkTableValue.h"
// #include "wpi/span.h"
#include "helpers.h"

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Invert the right side motors.
    m_frontRight.SetInverted(true);
    m_rearRight.SetInverted(true);
  }

  void TeleopPeriodic() override {
    /* Use the joystick X axis for forward movement, Y axis for lateral
     * movement, and Z axis for rotation.
     */

    // Scale down the joystick values
    double power = 0.5;

    // Fix joystick deadzone weirdness
    double threshold = 0.1;

    // Use the xbox bumpers for strafing
    //
    // I was trying to see if separating out the controls would make the cartesian drive happier, but not sure.
    // TODO We can switch this back to the X-axis down below.
    // I think there's just something physically wrong with our practice robot motors where some of them have more power than others.
    double x = 0.0;
    if (m_xbox.GetLeftBumper()) {
      x = 1.0;
    } else if (m_xbox.GetRightBumper()) {
      x = -1.0;
    }

    // I added this deadzone() function over in helpers.cpp so you have an example of how to put stuff in separate files and import them.

    //double x = deadzone(m_stick.GetX(), threshold);
    //double x = deadzone(m_xbox.GetLeftX(), threshold);

    double y = deadzone(m_xbox.GetLeftY(), threshold);
    double z = deadzone(m_xbox.GetRightX(), threshold);

    m_robotDrive.DriveCartesian(y * power * -1,
                                x * power * -1,
                                z * power);

    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    // double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    // double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    // double targetArea = table->GetNumber("ta",0.0);
    // double targetSkew = table->GetNumber("ts",0.0);
  }

 private:
  static constexpr int kFrontLeftChannel = 5;
  static constexpr int kRearLeftChannel = 4;
  static constexpr int kFrontRightChannel = 3;
  static constexpr int kRearRightChannel = 2;

  static constexpr int kJoystickChannel = 0;
  static constexpr int kXboxPort = 0;

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_frontLeft{kFrontLeftChannel};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rearLeft{kRearLeftChannel};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_frontRight{kFrontRightChannel};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rearRight{kRearRightChannel};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                  m_rearRight};

  frc::Joystick m_stick{kJoystickChannel};
  frc::XboxController m_xbox{kXboxPort};

  // TODO It looks like the roboRio has this ADIS16448 gyro built into it. We can try using it.
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html#adis16448
  //ADIS16448_IMU gyro;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
