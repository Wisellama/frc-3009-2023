// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/drive/DifferentialDrive.h>
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
#include <rev/SparkMaxRelativeEncoder.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "CameraAimer.h"

class Robot : public frc::TimedRobot {
  nt::DoublePublisher accelerationX;
  nt::DoublePublisher accelerationY;
  nt::DoublePublisher accelerationZ;
  nt::DoublePublisher publishDistance;
  nt::DoublePublisher publishDistanceRaw;
  nt::DoublePublisher publishCompressorCurrent;

public:
  // Destructor
  ~Robot() noexcept override {};

  void RobotInit() override {
    // Initialize the gyro/imu
    //m_imu.Calibrate();

    // Set all motors into Brake mode so that they try to hold themselves in-position when no input is given.
    for (rev::CANSparkMax* motor : sparkMotors) {
      motor->RestoreFactoryDefaults();
      motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      motor->Set(0);
    }
    m_wristMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_wristMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, 0);
  

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
    m_robotDrive.SetDeadband(kDeadband); // I don't trust this to do what it says at all

    // Set default solenoid positions
    m_solenoidArm.Set(frc::DoubleSolenoid::kReverse);
    m_solenoidClaw.Set(frc::DoubleSolenoid::kForward);
    m_solenoidWheels.Set(frc::DoubleSolenoid::kReverse);

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto accelerometerTable = inst.GetTable("accelerometer");
    auto ultrasonicTable = inst.GetTable("accelerometer");
    auto table = inst.GetTable("table");

    accelerationX = accelerometerTable->GetDoubleTopic("accelerationX").Publish();
    accelerationY = accelerometerTable->GetDoubleTopic("accelerationY").Publish();
    accelerationZ = accelerometerTable->GetDoubleTopic("accelerationZ").Publish();
    publishDistance = ultrasonicTable->GetDoubleTopic("Distance").Publish();
    publishDistanceRaw = ultrasonicTable->GetDoubleTopic("DistanceRaw").Publish();
    publishCompressorCurrent = table->GetDoubleTopic("CompressorCurrent").Publish();

    for (auto motor : sparkMotors) {
      std::stringstream topicName;
      int id = motor->GetDeviceId();
      topicName << "MotorDebug" << id;
      auto motorTable = inst.GetTable(topicName.str());
      motorDebugPublishers[id] = motorTable->GetStringTopic(topicName.str()).Publish();
    }

    std::stringstream topicName;
    int id = m_wristMotor.GetDeviceID();
    topicName << "MotorDebug" << id;
    auto wristTable = inst.GetTable(topicName.str());
    motorDebugPublishers[id] = wristTable->GetStringTopic(topicName.str()).Publish();

    auto encoderTable = inst.GetTable("ArmEncoder");

    ArmEncoderPublisher = encoderTable->GetStringTopic("ArmEncoder").Publish();
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

    publishMotorDebugInfo();
    publishEncoderDebugInfo();

    AutoAimResult ignore = m_cameraAimer.AutoAimAprilTags(-1);
    AutoAimResult ignore2 = m_cameraAimer.AutoAimReflectiveTape();

    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    // double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    // double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    // double targetArea = table->GetNumber("ta",0.0);
    // double targetSkew = table->GetNumber("ts",0.0);
  }

  void TeleopInit() override {
    m_useAprilTagsPID = false;
    m_useReflectivePID = false;
    m_useArmPID = true; // arm motion is much easier to control with PID
    m_armGoal = m_armMotorEncoder.GetPosition();
    m_cameraAimer.enableDriverVisionMicrosoft();
    m_cameraAimer.enableDriverVisionLimelight();
  }

  // Teleop lasts 2 minutes 15 seconds
  void TeleopPeriodic() override {

    // Determine values for driving the robot
    double forward = 0.0;
    double sideways = 0.0;
    double rotate = 0.0;

    // Toggle AprilTag following mode
    if (m_xbox0.GetStartButton()) {
      toggleAprilTagMode();
    }

    // Toggle ReflectiveTape following mode
    if (m_xbox0.GetBackButton()) {
      toggleReflectiveTapeMode();
    }

    if (m_useAprilTagsPID) {
      // Drive the robot based on camera targeting
      AutoAimResult result = m_cameraAimer.AutoAimAprilTags(-1);
      forward = result.GetForwardSpeed();
      forward = forward * 0.01;
      rotate = result.GetRotationSpeed();
      rotate = rotate / 100.0;
    } else if (m_useReflectivePID) {
      AutoAimResult result = m_cameraAimer.AutoAimReflectiveTape();
      rotate = result.GetRotationSpeed();
      rotate = rotate / 100.0;
    } else {
      // Drive the robot based on controller input
      // x = m_xbox.GetLeftX();
      double y = m_xbox0.GetLeftY() * -1;
      double z = m_xbox0.GetRightX();
      //units::degree_t a = m_imu.GetAngle();

      //if (std::abs(x) > kDeadband) {
      //  sideways = x;
      //}
      if (std::abs(y) > kDeadband) {
        forward = y;
      }
      if (std::abs(z) > kDeadband) {
        rotate = z;
      }

      if (m_xbox0.GetLeftBumper()) {
        sideways = -0.5;
      } else if (m_xbox0.GetRightBumper()) {
        sideways = 0.5;
      }
    }

    // Toggle wheels down for extra grip
    if (m_xbox0.GetXButtonPressed()) {
      m_solenoidWheels.Toggle();
      m_wheelsdown = !m_wheelsdown;
    }
    if (m_wheelsdown){
      // Can't strafe when the wheels are down, tank drive only.
      sideways = 0;
    }

    m_robotDrive.DriveCartesian(forward, sideways, rotate);

    // Pnuematics control for arm extend and claw
    if (m_xbox1.GetAButtonPressed()) {
      m_solenoidClaw.Toggle();
    }
    
    if (m_xbox1.GetYButtonPressed()) {
      m_solenoidArm.Toggle();
    }
    

    // Move the arm using controller triggers
    double leftTrigger = m_xbox1.GetLeftTriggerAxis();
    double rightTrigger = m_xbox1.GetRightTriggerAxis();
    leftTrigger = std::clamp(leftTrigger, 0.0, 1.0);
    rightTrigger = std::clamp(rightTrigger, 0.0, 1.0);
    leftTrigger = leftTrigger*0.5;
    rightTrigger = rightTrigger*0.5;

    double armSpeed = 0.0;
    double maxArmOutput = 0.5;

    if (rightTrigger > kDeadband) {
      armSpeed = -1 * rightTrigger;
    } else if (leftTrigger > kDeadband) {
      armSpeed = leftTrigger;
    }

    // Toggle arm PID mode
    if (m_xbox1.GetStartButton()) {
      m_useArmPID = !m_useArmPID;

      if (m_useArmPID) {
        // reset goal to current position
        m_armGoal = m_armMotorEncoder.GetPosition();
      }
    }

    if (m_useArmPID) {
      if (rightTrigger > kDeadband) {
        m_armGoal -= rightTrigger * 0.5;
       } else if (leftTrigger > kDeadband) {
        m_armGoal += leftTrigger * 0.5;
       }
      
      armSpeed = std::clamp(m_armController.Calculate(m_armMotorEncoder.GetPosition(), m_armGoal), -1 * maxArmOutput, maxArmOutput);
      m_armMotor.Set(armSpeed);
    } else {
      armSpeed = std::clamp(armSpeed, -1 * maxArmOutput, maxArmOutput);
      armSpeed = armSpeed * 0.5;
      m_armMotor.Set(armSpeed);
    }



    double maxWristOutput = 0.5;
    double wristSpeed = m_xbox1.GetLeftY();


    wristSpeed = std::clamp(wristSpeed, -1 * maxWristOutput, maxWristOutput);
    m_wristMotor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, wristSpeed);
  }

  void TeleopExit() override {
    // TODO our robot will leave Teleop mode after the game and we have 3 seconds before they score the results.
    // We should probably have this keep us still so that we can stay on the charging station.
    // Set the wheel speeds to zero and keep the stabilizer things deployed if we ever add those.
  }

  void AutonomousInit() override {
    m_cameraAimer.disableDriverVisionMicrosoft();
    m_cameraAimer.disableDriverVisionLimelight();
  }

  // Auto lasts 15 seconds
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
  static constexpr int kXboxPort2 = 1;

  static constexpr double kDeadband = 0.15;

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
  frc::XboxController m_xbox0{kXboxPort};
  frc::XboxController m_xbox1{kXboxPort2};

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
  const double ARM_P = 0.1;
  const double ARM_D = 0.0;
  frc::PIDController m_armController {ARM_P, 0, ARM_D};
  double m_armRotation = 0.0;
  double m_armGoal = 0.0;
  bool m_useArmPID = true; // arm motion is much easier to control with PID
  bool m_useAprilTagsPID = false;
  bool m_useReflectivePID = false;
  bool m_wheelsdown = false;
 

  // A list of all the spark motors so we can conveniently loop through them.
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

    std::map<int, nt::StringPublisher> motorDebugPublishers;
    nt::StringPublisher ArmEncoderPublisher;

    void publishMotorDebugInfo() {
      for(auto motor : sparkMotors) {
        std::stringstream output;
        int id = motor->GetDeviceId();
        output << "ID: " << id << ", ";
        output << "Get: " << motor->Get() << ", ";
        output << "AppliedOutput: " << motor->GetAppliedOutput() << ", ";
        output << "BusVoltage: " << motor->GetBusVoltage() << ", ";
        output << "OutputCurrent: " << motor->GetOutputCurrent() << ", ";
        output << "\n";
        motorDebugPublishers[id].Set(output.str());
      }

      std::stringstream output;
      int id = m_wristMotor.GetDeviceID();
      output << "ID: " << id << ", ";
      output << "OutputVoltage: " << m_wristMotor.GetMotorOutputVoltage() << ", ";
      output << "BusVoltage: " << m_wristMotor.GetBusVoltage() << ", ";
      output << "SupplyCurrent: " << m_wristMotor.GetSupplyCurrent() << ", ";
      output << "OutputCurrent: " << m_wristMotor.GetOutputCurrent() << ", ";
      output << "\n";
      motorDebugPublishers[id].Set(output.str());
      std::stringstream topicName;
    }

    rev::SparkMaxRelativeEncoder m_armMotorEncoder = m_armMotor.GetEncoder();

    void publishEncoderDebugInfo() {
      std::stringstream output;
      output << "Position: " << m_armMotorEncoder.GetPosition() << ", ";
      output << "Velocity: " << m_armMotorEncoder.GetVelocity() << ", ";
      ArmEncoderPublisher.Set(output.str());
    }

    void toggleAprilTagMode() {
      m_useAprilTagsPID = !m_useAprilTagsPID;
      if (m_useAprilTagsPID) {
        // Disable ReflectiveTape mode
        m_useReflectivePID = false;
        // Disable Driver mode to enable pipeline processing
        m_cameraAimer.disableDriverVisionMicrosoft();
      } else {
        // Enable Driver mode to give us a smoother live feed while we're not looking for vision targets
        m_cameraAimer.enableDriverVisionMicrosoft();
      }
    }

    void toggleReflectiveTapeMode() {
      m_useReflectivePID = !m_useReflectivePID;

      if (m_useReflectivePID) {
        // Disable AprilTag mode
        m_useAprilTagsPID = false;
        // Disable Driver mode to enable pipeline processing
        m_cameraAimer.disableDriverVisionLimelight();
      } else {
        // Enable Driver mode to give us a smoother live feed while we're not looking for vision targets
        m_cameraAimer.enableDriverVisionLimelight();
      } 
    }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif