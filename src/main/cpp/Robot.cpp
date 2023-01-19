// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/XboxController.h>
#include <frc/ADIS16448_IMU.h>
// #include "frc/smartdashboard/Smartdashboard.h"
// #include "networktables/NetworkTable.h"
// #include "networktables/NetworkTableInstance.h"
// #include "networktables/NetworkTableEntry.h"
// #include "networktables/NetworkTableValue.h"
// #include "wpi/span.h"

/**
 * This is a demo program showing how to use Mecanum control with the
 * MecanumDrive class.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Initialize the gyro/imu
    m_imu.Calibrate();

    // Initialize the motors
    m_frontRight.ConfigFactoryDefault();
    m_frontLeft.ConfigFactoryDefault();
    m_rearRight.ConfigFactoryDefault();
    m_rearLeft.ConfigFactoryDefault();

    // Invert the right side motors.
    m_frontRight.SetInverted(true);
    m_rearRight.SetInverted(true);

    // Limit the motor max speed
    double maxOutput = 0.5;
    m_robotDrive.SetMaxOutput(maxOutput);

    // The DriveCartesian class already has a deadzone adjustment thing.
    // It defaults to 0.02, we can bump up if needed.
    //m_robotDrive.SetDeadband(0.02);
  }

  void TeleopPeriodic() override {
    double x = m_xbox.GetLeftX();
    double y = m_xbox.GetLeftY() * -1;
    double z = m_xbox.GetRightX();
    units::degree_t a = m_imu.GetAngle();

    m_robotDrive.DriveCartesian(y, x, z, a);

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

  //static constexpr int kJoystickChannel = 0;
  static constexpr int kXboxPort = 0;

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_frontLeft{kFrontLeftChannel};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rearLeft{kRearLeftChannel};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_frontRight{kFrontRightChannel};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rearRight{kRearRightChannel};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                  m_rearRight};

  //frc::Joystick m_stick{kJoystickChannel};
  frc::XboxController m_xbox{kXboxPort};

  // https://wiki.analog.com/first/adis16448_imu_frc/cpp
  frc::ADIS16448_IMU m_imu{};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
