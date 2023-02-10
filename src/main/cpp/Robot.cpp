// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/XboxController.h>
#include <frc/ADIS16448_IMU.h>
#include <frc/TimedRobot.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/filter/LinearFilter.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/ArmFeedforward.h>

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

#include <rev/CANSparkMax.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "CameraAimer.h"

class Robot : public frc::TimedRobot {
public:
  nt::DoublePublisher accelerationX;
  nt::DoublePublisher accelerationY;
  nt::DoublePublisher accelerationZ;
  nt::DoublePublisher publishDistance;
  nt::DoublePublisher publishDistanceRaw;
  nt::DoublePublisher publishCompressorCurrent;

  // Destructor
  ~Robot() noexcept override {};

  void RobotInit() override {
    // Initialize the gyro/imu
    //m_imu.Calibrate();

    std::vector<rev::CANSparkMax*> sparkMotors = {
      &m_frontRight,
      &m_frontRightFollow,
      &m_frontRightFollow,
      &m_frontLeft,
      &m_frontLeftFollow,
      &m_rearRight,
      &m_rearRightFollow,
      &m_rearLeft,
      &m_rearLeftFollow,
      &m_armMotor,
    };

    // Set all motors into Brake mode so that they try to hold themselves in-position when no input is given.
    for (rev::CANSparkMax* motor : sparkMotors) {
      motor->RestoreFactoryDefaults();
      motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    m_wristMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    // Set secondary motor controllers to follow their pairs
    m_frontLeftFollow.Follow(m_frontLeft);
    m_frontRightFollow.Follow(m_frontRight);
    m_rearLeftFollow.Follow(m_rearLeft);
    m_rearRightFollow.Follow(m_rearRight);

    // Invert the right side motors.
    m_frontRight.SetInverted(true);
    m_frontRightFollow.SetInverted(true);
    m_rearRight.SetInverted(true);
    m_rearRightFollow.SetInverted(true);

    // Limit the motor max speed
    double maxOutput = 0.5;
    m_robotDrive.SetMaxOutput(maxOutput);
    m_robotDrive.SetDeadband(kDeadband);

    // Set default solenoid positions
    m_solenoidArm.Set(frc::DoubleSolenoid::kReverse);
    m_solenoidClaw.Set(frc::DoubleSolenoid::kReverse);
    m_solenoidWheels.Set(frc::DoubleSolenoid::kReverse);

    // The DriveCartesian class already has a deadzone adjustment thing.
    // It defaults to 0.02, we can bump up if needed.
    //m_robotDrive.SetDeadband(0.02);

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("datatable");

    accelerationX = table->GetDoubleTopic("accelerationX").Publish();
    accelerationY = table->GetDoubleTopic("accelerationY").Publish();
    accelerationZ = table->GetDoubleTopic("accelerationZ").Publish();
    publishDistance = table->GetDoubleTopic("publishDistance").Publish();
    publishDistanceRaw = table->GetDoubleTopic("publishDistanceRaw").Publish();
    publishCompressorCurrent = table->GetDoubleTopic("publishCompressorCurrent").Publish();
  }

  // RobotPeriodic will run regardless of enabled/disabled
  void RobotPeriodic() override {
    m_compressor.EnableDigital();

    //units::current::ampere_t compressorCurrent = m_compressor.GetCurrent();

    accelerationX.Set(m_accelerationXFilter.Calculate(m_accelerometer.GetX()));
    accelerationY.Set(m_accelerationYFilter.Calculate(m_accelerometer.GetY()));
    accelerationZ.Set(m_accelerationZFilter.Calculate(m_accelerometer.GetZ()));

    double distance = m_ultrasonic.Get();
    publishDistanceRaw.Set(distance);

    distance = distance * kSonicScale; // convert to meters

    if (distance < kSonicLimitLower) {
      distance = -1.0; // Below lower limit of the sensor, value isn't useful
    }

    publishDistance.Set(distance);

    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    // double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    // double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    // double targetArea = table->GetNumber("ta",0.0);
    // double targetSkew = table->GetNumber("ts",0.0);
  }

  void TeleopPeriodic() override {

    // Determine values for driving the robot
    double forward = 0.0;
    double sideways = 0.0;
    double rotate = 0.0;

    if (m_xbox.GetStartButton()) {
      // Drive the robot based on camera targeting
      AutoAimResult result = m_cameraAimer.AutoAim(-1);
      forward = result.GetForwardSpeed();
      rotate = result.GetRotationSpeed();
    } else {
      // Drive the robot based on controller input
      double x = m_xbox.GetLeftX();
      double y = m_xbox.GetLeftY() * -1;
      double z = m_xbox.GetRightX();
      //units::degree_t a = m_imu.GetAngle();

      forward = y;
      sideways = x;
      rotate = z;
    }

    m_robotDrive.DriveCartesian(forward, sideways, rotate);

    // Pnuematics control for arm extend and claw
    if (m_xbox.GetAButtonPressed()) {
      m_solenoidClaw.Toggle();
    } else if (m_xbox.GetYButtonPressed()) {
      m_solenoidArm.Toggle();
    } else if (m_xbox.GetXButtonPressed()) {
      m_solenoidWheels.Toggle();
    }

    // Move the arm using controller triggers
    double leftTrigger = m_xbox.GetLeftTriggerAxis();
    double rightTrigger = m_xbox.GetRightTriggerAxis();
    leftTrigger = std::clamp(leftTrigger, 0.0, 1.0);
    rightTrigger = std::clamp(rightTrigger, 0.0, 1.0);

    double armSpeed = 0.0;
    double maxArmOutput = 0.5;

    if (rightTrigger > 0.05) {
      armSpeed = -1 * rightTrigger;
    } else if (leftTrigger > 0.05) {
      armSpeed = leftTrigger;
    }

    // If you hit the back button (aka select button) it will try using the PIDController to maintain arm position.
    if (m_xbox.GetBackButton()) {
      double controllerValue = armSpeed;
      m_armGoal += controllerValue / 1000; // I'm tamping this down a lot so we don't plow through the ceiling.
      m_armRotation = m_armController.Calculate(m_armRotation, m_armGoal);
      armSpeed = m_armRotation;
    }

    armSpeed = std::clamp(armSpeed, -1 * maxArmOutput, maxArmOutput);
    m_armMotor.Set(armSpeed);

    double wristSpeed = 0.0;
    double maxWristOutput = 0.15;
    double currentPOV = m_xbox.GetPOV();

    if (currentPOV == 0) {
      wristSpeed = 1;
    } else if (currentPOV == 180) {
      wristSpeed = -1;
    };

    wristSpeed = std::clamp(wristSpeed, -1 * maxWristOutput, maxWristOutput);
    m_wristMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, wristSpeed);
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
  // CAN IDs for everything on the CAN network
  static constexpr int kFrontLeftChannel = 25;
  static constexpr int kFrontLeftFollowChannel = 5;
  static constexpr int kRearLeftChannel = 24;
  static constexpr int kRearLeftFollowChannel = 4;
  static constexpr int kFrontRightChannel = 23;
  static constexpr int kFrontRightFollowChannel = 3;
  static constexpr int kRearRightChannel = 22;
  static constexpr int kRearRightFollowChannel = 2;
  static constexpr int kArmChannel = 9;
  static constexpr int kWristChannel = 10;
  static constexpr int kPnuematics = 50;
  static constexpr int kPDP = 51;

  //static constexpr int kJoystickChannel = 0;
  static constexpr int kXboxPort = 0;
  //TODO setup accessories controller

  static constexpr double kDeadband = 0.1;

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

  // These SparkMax controllers handle the wheels and driving
  rev::CANSparkMax m_frontLeft{kFrontLeftChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontLeftFollow{kFrontLeftFollowChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRight{kFrontRightChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRightFollow{kFrontRightFollowChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeft{kRearLeftChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeftFollow{kRearLeftFollowChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRight{kRearRightChannel, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRightFollow{kRearRightFollowChannel, rev::CANSparkMax::MotorType::kBrushless};
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};


  // This SparkMax controls the arm up/down rotation
  rev::CANSparkMax m_armMotor{kArmChannel, rev::CANSparkMax::MotorType::kBrushless};

  // This Talon SRX controls the claw's wrist rotation
  ctre::phoenix::motorcontrol::can::TalonSRX m_wristMotor{kWristChannel};

  frc::LinearFilter<double> m_accelerationXFilter = frc::LinearFilter<double>::MovingAverage(10);
  frc::LinearFilter<double> m_accelerationYFilter = frc::LinearFilter<double>::MovingAverage(10);
  frc::LinearFilter<double> m_accelerationZFilter = frc::LinearFilter<double>::MovingAverage(10);

  //frc::Joystick m_stick{kJoystickChannel};
  frc::XboxController m_xbox{kXboxPort};

  // https://wiki.analog.com/first/adis16448_imu_frc/cpp
  //frc::ADIS16448_IMU m_imu{};

  frc::BuiltInAccelerometer m_accelerometer{};

  frc::AnalogPotentiometer m_ultrasonic{kSonicPort};

  CameraAimer m_cameraAimer{};

  frc::Compressor m_compressor{50, frc::PneumaticsModuleType::REVPH};
  frc::DoubleSolenoid m_solenoidArm{50, frc::PneumaticsModuleType::REVPH, 0, 1}; // Extend/retract the arm
  frc::DoubleSolenoid m_solenoidClaw{50, frc::PneumaticsModuleType::REVPH, 2, 3}; // Open/close the claw
  frc::DoubleSolenoid m_solenoidWheels{50, frc::PneumaticsModuleType::REVPH, 4, 5}; // Drop/raise the wheels

  // For smoother arm motion, we're using a PIDController that should allow the motor to keep the arm at a given level
  const double ARM_P = 4.0;
  const double ARM_D = 0.75;
  frc::PIDController m_armController {ARM_P, 0, ARM_D};
  double m_armRotation = 0.0;
  double m_armGoal = 0.0;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
