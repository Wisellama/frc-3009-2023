// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <ctime>
#include <iostream>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/ADIS16448_IMU.h>
#include <frc/TimedRobot.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/filter/LinearFilter.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/I2C.h>

#include <cameraserver/CameraServer.h>

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

#include <rev/CANSparkMax.h>

#include <ctre/phoenix/sensors/Pigeon2.h>

#include "CameraAimer.h"
#include "Controls.h"
#include "Arm.h"
#include "DigitMXPDisplay.h"
#include "Wrist.h"

class Robot : public frc::TimedRobot {
  nt::DoublePublisher accelerationX;
  nt::DoublePublisher accelerationY;
  nt::DoublePublisher accelerationZ;
  nt::DoublePublisher publishDistance;
  nt::DoublePublisher publishDistanceRaw;
  nt::DoublePublisher publishCompressorCurrent;
  nt::DoublePublisher publishAngle;

  nt::BooleanPublisher publishArmExtended;
  nt::BooleanPublisher publishClawClosed;
  nt::BooleanPublisher publishWheelsDown;
  nt::BooleanPublisher publishBrakeSet;
  nt::StringPublisher publishTimestamp;
  nt::StringPublisher publishRobotPose;

public:
  // Destructor
  ~Robot() noexcept override {};

  // This runs exactly once on robot power on
  void RobotInit() override {
    // Initialize the gyro/imu
    //m_imu.Calibrate();

    m_digitBoard.Test();

    // Start Arm-mounted camera connected directly to roborio
    frc::CameraServer::StartAutomaticCapture();

    // Set all motors into Brake mode so that they try to hold themselves in-position when no input is given.
    for (rev::CANSparkMax* motor : sparkMotors) {
      motor->RestoreFactoryDefaults();
      motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      motor->Set(0);
    }

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
    m_robotDrive.SetMaxOutput(kHalfSpeed);
    m_robotDrive.SetDeadband(kDeadband); // I don't trust this to do what it says at all

    // Set default solenoid positions
    retractArm();
    closeClaw();
    raiseWheels();
    resetEncoders();
    heavyClawPressure();

    // Reset the feedback controllers
    m_arm.ResetGoal();
    m_wrist.ResetGoal();

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto accelerometerTable = inst.GetTable("accelerometer");
    auto gyroTable = inst.GetTable("gyro");
    auto ultrasonicTable = inst.GetTable("ultrasonic");
    auto table = inst.GetTable("table");
    auto stateTable = inst.GetTable("robot_state");

    accelerationX = accelerometerTable->GetDoubleTopic("accelerationX").Publish();
    accelerationY = accelerometerTable->GetDoubleTopic("accelerationY").Publish();
    accelerationZ = accelerometerTable->GetDoubleTopic("accelerationZ").Publish();
    publishDistance = ultrasonicTable->GetDoubleTopic("Distance").Publish();
    publishDistanceRaw = ultrasonicTable->GetDoubleTopic("DistanceRaw").Publish();
    publishCompressorCurrent = table->GetDoubleTopic("CompressorCurrent").Publish();
    publishAngle = gyroTable->GetDoubleTopic("Angle").Publish();
    publishArmExtended = stateTable->GetBooleanTopic("ArmExtended").Publish();
    publishClawClosed = stateTable->GetBooleanTopic("ClawClosed").Publish();
    publishWheelsDown = stateTable->GetBooleanTopic("WheelsDown").Publish();
    publishBrakeSet = stateTable->GetBooleanTopic("BrakeSet").Publish();
    publishTimestamp = stateTable->GetStringTopic("Timestamp").Publish();
    publishRobotPose = stateTable->GetStringTopic("RobotPose").Publish();

    for (auto motor : sparkMotors) {
      std::stringstream topicName;
      int id = motor->GetDeviceId();
      topicName << "MotorDebug" << id;
      auto motorTable = inst.GetTable(topicName.str());
      motorDebugPublishers[id] = motorTable->GetStringTopic(topicName.str()).Publish();
    }

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

    //units::degree_t angle = m_imu.GetAngle();
    //publishAngle.Set(angle.value());

    distance = distance * kSonicScale; // convert to meters

    if (distance < kSonicLimitLower) {
      distance = -1.0; // Below lower limit of the sensor, value isn't useful
    }

    publishDistance.Set(distance);

    publishMotorDebugInfo();
    publishEncoderDebugInfo();

    publishRobotState();
    
    AutoAimResult ignore = m_cameraAimer.AutoAimAprilTags(-1);
    AutoAimResult ignore2 = m_cameraAimer.AutoAimReflectiveTape();

    // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    // double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    // double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    // double targetArea = table->GetNumber("ta",0.0);
    // double targetSkew = table->GetNumber("ts",0.0);
  }

  void TeleopInit() override {
    m_armDirectDrive = false;
    m_cameraAimer.enableDriverVisionMicrosoft();
    m_cameraAimer.enableDriverVisionLimelight();

    // Reset the feedback controllers
    m_arm.ResetGoal();
    m_wrist.ResetGoal();

    raiseWheels();
  }

  // Teleop lasts 2 minutes 15 seconds
  void TeleopPeriodic() override {

    // Determine values for driving the robot
    double forward = 0.0;
    double sideways = 0.0;
    double rotate = 0.0;

    // Toggle AprilTag following mode
    if (m_controls.AprilTagMode()) {
      m_cameraAimer.ToggleAprilTagMode();
    }

    if (m_controls.ReflectiveTapeMode()) {
      m_cameraAimer.ToggleReflectiveTapeMode();
    }

    if (m_cameraAimer.GetAprilTagMode()) {
      // Drive the robot based on camera targeting
      AutoAimResult result = m_cameraAimer.AutoAimAprilTags(-1);
      forward = result.GetForwardSpeed();
      forward = forward * 0.01;
      rotate = result.GetRotationSpeed();
      rotate = rotate / 100.0;

      auto pose = m_cameraAimer.EstimatePoseAprilTags(m_robotPose);
      if (pose.has_value()) {
        m_robotPose = pose.value().estimatedPose;
        //auto a = m_robotPose - m_robotPose;

      }

      // Just manually drive
      forward = m_controls.DriveForward();
    } else if (m_cameraAimer.GetReflectiveTapeMode()) {
      AutoAimResult result = m_cameraAimer.AutoAimReflectiveTape();
      rotate = result.GetRotationSpeed();
      rotate = rotate / 100.0;

      // Manually drive forward
      forward = m_controls.DriveForward();
    } else {
      // Drive the robot based on controller input
      sideways = m_controls.DriveStrafe();
      forward = m_controls.DriveForward();
      rotate = m_controls.DriveRotate();
      //units::degree_t a = m_imu.GetAngle();

      if (m_controls.Turbo()) {
        m_robotDrive.SetMaxOutput(kThreeQuartSpeed);
      } else {
        m_robotDrive.SetMaxOutput(kHalfSpeed);
      }
    }

    // Toggle wheels down for extra grip
    if (m_controls.ExtraWheels()) {
      toggleWheels();
    }

    if (m_wheelsDown){
      lowerWheels();

      // Can't strafe when the wheels are down, tank drive only.
      sideways = 0;

      // Can deploy parking brakes
      if (m_controls.ParkingBrake()) {
        toggleParkingBrake();
      }
    } else {
      raiseWheels();
    }

    m_robotDrive.DriveCartesian(forward, sideways, rotate);

    // Pnuematics control for arm extend and claw
    if (m_controls.ClawClamp()) {
      toggleClaw();
    }
    if (m_controls.ArmExtend()) {
      toggleArm();
    }
    if (m_controls.ClawPressure()) {
      toggleClawPressure();
    }
    

    // Move the arm using controller triggers
    double armInput = m_controls.ArmRaise();

    double armMove = 0.0;

    // Toggle arm PID mode
    if (m_controls.DirectDriveArm()) {
      m_armDirectDrive = !m_armDirectDrive;
      m_arm.ResetGoal();
    }

    if (m_armDirectDrive) {
      armMove = armInput * 0.4;
    } else {
      // The controller will give us values from -1 to 1, so move about 1 degree per cycle
      double encoderMove = m_arm.DegreesToEncoder(armInput);
      encoderMove = -1 * encoderMove;
      m_arm.MoveGoal(encoderMove);
      double move = m_arm.CalculateMove();
      armMove = move;
    }

    double maxArmOutput = kFullSpeed;
    armMove = std::clamp(armMove, -1 * maxArmOutput, maxArmOutput);
    m_armMotor.Set(armMove);

    double maxWristOutput = kHalfSpeed;
    double wristInput = m_controls.Wrist();

    double wristMove = 0.0;
    if (false) { // TODO if we want to try wrist stability with the feedback controller
      // Actually, I don't think this will work because the motor slips a lot which would screw up the encoder position.
      double wristEncoderMove = m_wrist.DegreesToEncoder(wristInput);
      m_wrist.MoveGoal(wristEncoderMove);

      wristMove = m_wrist.CalculateMove();
      wristMove = std::clamp(wristMove, -1 * maxWristOutput, maxWristOutput);
    } else {
      wristMove = wristInput;
    }

    m_wristMotor.Set(wristMove);

    if (m_controls.ToggleLimits()){
      m_arm.ToggleLimits();
    }
  }

  void TeleopExit() override {
    // TODO our robot will leave Teleop mode after the game and we have 3 seconds before they score the results.
    // We should probably have this keep us still so that we can stay on the charging station.
    // Set the wheel speeds to zero and keep the stabilizer things deployed if we ever add those.
  }

  void AutonomousInit() override {
    // Set the Microsoft camera to process AprilTags
    m_cameraAimer.disableDriverVisionMicrosoft();

    // Set the LimeLight camera to process AprilTags too,
    m_cameraAimer.SetAprilTagMode();

    // Alternatively, we could have it do reflective tape, or switch to it if needed
    //m_cameraAimer.SetReflectiveTapeMode();

    closeClaw();
    retractArm();
    releaseParkingBrake();
    lowerWheels(); // Vision tracking can only do tank drive mode anyway
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

    // Left/right start
    // Start robot facing the grid
    // Get in position to lift arm to top(?) shelf/pole
    // Get claw in position and release
    // Retract claw and drive out
    // Turn to face game piece and collect
    // Drive back to grid and face it
    // Line up to deposit 2nd piece (should be different piece type to start a link? or both cones)
    // Deposit 2nd piece
    // Line up with april tags to get onto charging station
    // Drive up charging station and balance
        // needs a way to disable this incase some other team is going to do this
        // if not charging station, drive out and pick up a 3rd piece if possible
    
    // Center start
    // Start robot facing the grid
    // Get in position to lift arm to top(?) shelf/pole
    // Get claw in position and release
    // drive out over charging station perhaps?
    // ???
    // balance if going to balance
  }

  void AutonomousExit() override {
    setParkingBrake();
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
  static constexpr int kPidgeonIMU = 52;

  static constexpr int kXboxPort1 = 0;
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
  static constexpr int kSonicPort = 2;

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

  // This SparkMax controls the claw's wrist rotation
  rev::CANSparkMax m_wristMotor{kWristChannel, rev::CANSparkMax::MotorType::kBrushless};
  
  frc::LinearFilter<double> m_accelerationXFilter = frc::LinearFilter<double>::MovingAverage(10);
  frc::LinearFilter<double> m_accelerationYFilter = frc::LinearFilter<double>::MovingAverage(10);
  frc::LinearFilter<double> m_accelerationZFilter = frc::LinearFilter<double>::MovingAverage(10);

  Controls m_controls {kXboxPort1, kXboxPort2, kDeadband};

  // https://wiki.analog.com/first/adis16448_imu_frc/cpp
  //frc::ADIS16448_IMU m_imu{};

  frc::BuiltInAccelerometer m_accelerometer{};

  frc::AnalogPotentiometer m_ultrasonic{kSonicPort};

  CameraAimer m_cameraAimer{};

  frc::Compressor m_compressor{50, frc::PneumaticsModuleType::REVPH};
  frc::DoubleSolenoid m_solenoidArm{50, frc::PneumaticsModuleType::REVPH, 0, 1}; // Extend/retract the arm
  frc::DoubleSolenoid m_solenoidClaw{50, frc::PneumaticsModuleType::REVPH, 2, 3}; // Open/close the claw
  frc::DoubleSolenoid m_solenoidWheels{50, frc::PneumaticsModuleType::REVPH, 4, 5}; // Drop/raise the wheels
  frc::DoubleSolenoid m_solenoidBrakes{50, frc::PneumaticsModuleType::REVPH, 6, 7}; // Enable/disable parking brake
  frc::DoubleSolenoid m_solenoidClawPressure{50, frc::PneumaticsModuleType::REVPH, 8, 9}; // Set the claw to high/low pressure mode

  bool m_armDirectDrive = false;
  bool m_wheelsDown = false;
  bool m_parkingBrake = false;
  bool m_clawClosed = true;
  bool m_clawPressureHigh = false;

  double kFullSpeed = 1.0;
  double kThreeQuartSpeed = 0.75;
  double kHalfSpeed = 0.5;

  frc::Pose3d m_robotPose {};

  DigitMXPDisplay m_digitBoard {};

  ctre::phoenix::sensors::Pigeon2 m_pigeon{kPidgeonIMU};

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
      &m_wristMotor,
    };

    std::map<int, nt::StringPublisher> motorDebugPublishers;
    nt::StringPublisher ArmEncoderPublisher;

    // https://www.revrobotics.com/rev-11-1271/
    rev::SparkMaxAlternateEncoder m_armMotorEncoder = m_armMotor.GetAlternateEncoder(
      rev::SparkMaxAlternateEncoder::Type::kQuadrature, 8192);
    rev::SparkMaxRelativeEncoder m_wristMotorEncoder = m_wristMotor.GetEncoder();

    Arm m_arm {&m_armMotorEncoder};
    Wrist m_wrist {&m_wristMotorEncoder};

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
    }

    void publishEncoderDebugInfo() {
      std::stringstream output;
      output << "Position: " << m_armMotorEncoder.GetPosition() << ", ";
      output << "Velocity: " << m_armMotorEncoder.GetVelocity() << ", ";
      ArmEncoderPublisher.Set(output.str());
    }

    void releaseParkingBrake() {
      m_solenoidBrakes.Set(frc::DoubleSolenoid::kReverse);
      m_parkingBrake = false;
    }

    void setParkingBrake() {
      m_solenoidBrakes.Set(frc::DoubleSolenoid::kForward);
      m_parkingBrake = true;
    }

    void toggleParkingBrake() {
      m_solenoidBrakes.Toggle();
      m_parkingBrake = !m_parkingBrake;
    }

    void lowerWheels() {
      m_solenoidWheels.Set(frc::DoubleSolenoid::kForward);
      m_wheelsDown = true;
    }

    void raiseWheels() {
      m_solenoidWheels.Set(frc::DoubleSolenoid::kReverse);
      m_wheelsDown = false;

      // Always release parking brake when raising the wheels
      releaseParkingBrake();
    }

    void toggleWheels() {
      m_solenoidWheels.Toggle();
      m_wheelsDown = !m_wheelsDown;

      if (!m_wheelsDown) {
        // Always release the parking brake if we raise the wheels
        releaseParkingBrake();
      }
    }

    void openClaw() {
      m_solenoidClaw.Set(frc::DoubleSolenoid::kReverse);
      m_clawClosed = false;
    }

    void closeClaw() {
      m_solenoidClaw.Set(frc::DoubleSolenoid::kForward);
      m_clawClosed = true;
    }

    void toggleClaw() {
      m_solenoidClaw.Toggle();
      m_clawClosed = !m_clawClosed;
    }

    void extendArm() {
      m_solenoidArm.Set(frc::DoubleSolenoid::kForward);
      m_arm.SetExtended();
    }

    void retractArm() {
      m_solenoidArm.Set(frc::DoubleSolenoid::kReverse);
      m_arm.SetRetracted();
    }

    void toggleArm() {
      m_solenoidArm.Toggle();
      m_arm.ToggleExtended();
    }

    void toggleClawPressure() {
      m_solenoidClawPressure.Toggle();
      m_clawPressureHigh = !m_clawPressureHigh;
    }

    void lightClawPressure() {
      m_solenoidClawPressure.Set(frc::DoubleSolenoid::kReverse);
      m_clawPressureHigh = false;
    }

    void heavyClawPressure() {
      m_solenoidClawPressure.Set(frc::DoubleSolenoid::kForward);
      m_clawPressureHigh = true;
    }

    void publishRobotState() {
    
      publishBrakeSet.Set(m_parkingBrake);
      publishWheelsDown.Set(m_wheelsDown);
      publishArmExtended.Set(m_arm.GetExtendedState());
      publishClawClosed.Set(m_clawClosed);

      std::stringstream poseStr;
      poseStr << "X: " << m_robotPose.X().value() << " Y: " << m_robotPose.Y().value() << " Z: " << m_robotPose.Z().value();
      publishRobotPose.Set(poseStr.str());

      frc::SmartDashboard::PutBoolean("ParkingBrake", m_parkingBrake);
      frc::SmartDashboard::PutBoolean("WheelsDown", m_wheelsDown);
      frc::SmartDashboard::PutBoolean("ArmExtended", m_arm.GetExtendedState());
      frc::SmartDashboard::PutBoolean("ClawClosed", m_clawClosed);
      frc::SmartDashboard::PutBoolean("ArmIgnoreLimits", m_arm.GetIgnoreLimits());
      frc::SmartDashboard::PutBoolean("HeavyClawPressure", m_clawPressureHigh);
      frc::SmartDashboard::PutString("RobotPose", poseStr.str());

      auto now = std::chrono::system_clock::now();
      std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
      publishTimestamp.Set(std::ctime(&nowTime));

      frc::SmartDashboard::PutNumber("ArmPositionEncoder", m_arm.GetEncoderPosition());
      frc::SmartDashboard::PutNumber("WristPositionEncoder", m_wrist.GetEncoderPosition());

      frc::SmartDashboard::PutBoolean("DigitDisplayA", m_digitBoard.GetButtonA());
      frc::SmartDashboard::PutBoolean("DigitDisplayB", m_digitBoard.GetButtonB());
      frc::SmartDashboard::PutNumber("DigitDisplayPot", m_digitBoard.GetPot());

      frc::SmartDashboard::PutNumber("GyroYaw", m_pigeon.GetYaw());
      frc::SmartDashboard::PutNumber("GyroPitch", m_pigeon.GetPitch());
      frc::SmartDashboard::PutNumber("GyroRoll", m_pigeon.GetRoll());
    }

    void resetEncoders() {
      m_armMotorEncoder.SetPosition(0);
      m_wristMotorEncoder.SetPosition(0);
    }


};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
