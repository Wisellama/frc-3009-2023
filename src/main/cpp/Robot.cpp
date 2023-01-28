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
#include <frc/TimedRobot.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/AnalogPotentiometer.h>

#include <photonlib/PhotonCamera.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <networktables/DoubleTopic.h>

#include <frc/filter/LinearFilter.h>

class Robot : public frc::TimedRobot {
public:
  nt::DoublePublisher accelerationX;
  nt::DoublePublisher accelerationY;
  nt::DoublePublisher accelerationZ;
  nt::DoublePublisher publishDistance;
  nt::DoublePublisher publishDistanceRaw;

  // Destructor
  ~Robot() noexcept override {};

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

    // robotCamera.SetLEDMode(photonlib::LEDMode::kBlink);

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("datatable");

    accelerationX = table->GetDoubleTopic("accelerationX").Publish();
    accelerationY = table->GetDoubleTopic("accelerationY").Publish();
    accelerationZ = table->GetDoubleTopic("accelerationZ").Publish();
    publishDistance = table->GetDoubleTopic("publishDistance").Publish();
    publishDistanceRaw = table->GetDoubleTopic("publishDistanceRaw").Publish();
  }

  // RobotPeriodic will run regardless of enabled/disabled
  void RobotPeriodic() override {
    accelerationX.Set(m_accelerationXFilter.Calculate(m_accelerometer.GetX()));
    accelerationY.Set(m_accelerationYFilter.Calculate(m_accelerometer.GetY()));
    accelerationZ.Set(m_accelerationZFilter.Calculate(m_accelerometer.GetZ()));

    double distance = m_ultrasonic.Get();
    publishDistanceRaw.Set(distance);

    distance = distance * kSonicScale; // convert to meters

    if (distance < kSonicLimitLower) {
      distance = -1.0; // Below lower limit of the sensor
    }

    publishDistance.Set(distance);

    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    // double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    // double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    // double targetArea = table->GetNumber("ta",0.0);
    // double targetSkew = table->GetNumber("ts",0.0);
  }

  void TeleopPeriodic() override {
    double x = m_xbox.GetLeftX();
    double y = m_xbox.GetLeftY() * -1;
    double z = m_xbox.GetRightX();
    units::degree_t a = m_imu.GetAngle();

    m_robotDrive.DriveCartesian(y, x, z, a);
  }

  void TeleopExit() override {
    // TODO our robot will leave Teleop mode after the game and we have 3 seconds before they score the results.
    // We should probably have this keep us still so that we can stay on the charging station.
    // Set the wheel speeds to zero and keep the stabilizer things deployed if we ever add those.
  }

  void AutonomousPeriodic() override {
    // TODO autonomous mode
    /*
    Most likely use the cameras and/or ultrasonic to place a cube or cone (3/4/6 points)
    and then either back up onto the charging station board and balance (8/12 points)
    or move around the charging station outside the community (3 points) to attempt to pickup another piece.

    For placing the game piece, the cameras can track the AprilTags and Reflective tape.
    They also give an estimated offset for distance and angle difference for us to line up.
    If we know the length of our arm, we can raise it up to the correct height and then line up with the cameras.

    For the charging station, we can probably also line up with one of the AprilTags to know where we are on the field.
    We can use the camera estimation and the field dimensions to know where to go.
    Once we're on the charging station, we can use the gyro/imu to detect if we need to rebalance.

    For leaving the community, we can use the AprilTags to help know where we are on the field
    and potentially the ultrasonic sensors to help avoid hitting the side walls or the charging station.
    The additional game pieces are set at a specific location, so we might be able to get pretty close using the AprilTags
    */
  }

private:
  static constexpr int kFrontLeftChannel = 5;
  static constexpr int kRearLeftChannel = 4;
  static constexpr int kFrontRightChannel = 3;
  static constexpr int kRearRightChannel = 2;

  //static constexpr int kJoystickChannel = 0;
  static constexpr int kXboxPort = 0;

  /*
  The MB1043 UltraSonic datasheet says that the minimum range is around 30cm, and I've tested this to be fairly accurate.
  It says the analog value is scaled by Vcc/1024 per 5 mm, where Vcc is 5v.
  Roughly supposed to receive a value of .293 for 300mm (min) and 4.885 for 5000mm (max)

  So to convert the value to millimeters, we have to multiply by 1024 * 5
  (their scaling factor was giving "per 5 mm", so to get 1 mm we gotta cancel that out too)

  https://www.maxbotix.com/ultrasonic_sensors/mb1043.htm
  */
  static constexpr double kSonicScale = 1024.0 * 5;
  static constexpr double kSonicLimitLower = 300.0;
  static constexpr int kSonicPort = 3;

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_frontLeft{kFrontLeftChannel};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rearLeft{kRearLeftChannel};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_frontRight{kFrontRightChannel};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rearRight{kRearRightChannel};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};

  frc::LinearFilter<double> m_accelerationXFilter = frc::LinearFilter<double>::MovingAverage(10);
  frc::LinearFilter<double> m_accelerationYFilter = frc::LinearFilter<double>::MovingAverage(10);
  frc::LinearFilter<double> m_accelerationZFilter = frc::LinearFilter<double>::MovingAverage(10);

  //frc::Joystick m_stick{kJoystickChannel};
  frc::XboxController m_xbox{kXboxPort};

  // https://wiki.analog.com/first/adis16448_imu_frc/cpp
  frc::ADIS16448_IMU m_imu{};

  frc::BuiltInAccelerometer m_accelerometer{};

  frc::AnalogPotentiometer m_ultrasonic{kSonicPort};

  // photonlib::PhotonCamera robotCamera{"photonvision"};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
