// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/TimedRobot.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/filter/LinearFilter.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/I2C.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/estimator/MecanumDrivePoseEstimator.h>

#include <cameraserver/CameraServer.h>

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

#include <rev/CANSparkMax.h>

#include <ctre/phoenix/sensors/WPI_Pigeon2.h>

#include "CameraAimer.h"
#include "Controls.h"
#include "Arm.h"
#include "DigitMXPDisplay.h"
#include "Wrist.h"
#include "GyroAimer.h"
#include "GyroLeveller.h"
#include "FeedbackController.h"
#include "AutoState.h"

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
    // An example of sleeping for 5 milliseconds (1000ms = 1 second)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    m_autoStates = new AutoState();

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

    // Initialize the drive pose estimator
    m_drivePoseEstimator = new frc::MecanumDrivePoseEstimator(
      m_driveKinematics,
      frc::Rotation2d{units::degree_t(m_pigeon.GetYaw())},
      m_driveWheelPositions,
      frc::Pose2d{});

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
    /*for (rev::CANSparkMax* motor : sparkMotors) {
      motor->Set(0);
    }*/

    m_compressor.EnableDigital();

    //m_digitBoard.Test();

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

    publishRobotState();
    
    AutoAimResult ignore = m_cameraAimer.AutoAimAprilTags(-1);
    AutoAimResult ignore2 = m_cameraAimer.AutoAimReflectiveTape();

    std::optional<photonlib::EstimatedRobotPose> cameraEstimate = m_cameraAimer.EstimatePoseAprilTags(m_robotPose);
    if (cameraEstimate.has_value()) {
      frc::Pose2d pose2d = Pose3dTo2d(cameraEstimate->estimatedPose);
      m_drivePoseEstimator->AddVisionMeasurement(pose2d, cameraEstimate->timestamp);
    }
    m_driveWheelPositions.frontLeft = m_frontLeftEncoder.GetPosition() * kDistancePerPulse;
    m_driveWheelPositions.frontRight = m_frontRightEncoder.GetPosition() * kDistancePerPulse;
    m_driveWheelPositions.rearLeft = m_rearLeftEncoder.GetPosition() * kDistancePerPulse;
    m_driveWheelPositions.rearRight = m_rearRightEncoder.GetPosition() * kDistancePerPulse;
    m_robotPose = m_drivePoseEstimator->Update(frc::Rotation2d{units::degree_t(m_pigeon.GetYaw())}, m_driveWheelPositions);

    frc::Pose2d zero {};
    frc::Pose2d goal {};
    frc::Transform2d diff = goal - m_robotPose;
    frc::SmartDashboard::PutString("PoseGoalDiff", PoseToStr(zero.TransformBy(diff)));
  }

  void TeleopInit() override {
    m_armDirectDrive = false;

    m_cameraAimer.SetAprilTagMode();
    //m_cameraAimer.enableDriverVisionMicrosoft();
    //m_cameraAimer.enableDriverVisionLimelight();

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

    if (true) {
      // Drive the robot based on controller input
      sideways = m_controls.DriveStrafe();
      forward = m_controls.DriveForward();
      
      if (m_controls.FaceGrid()) {
        // rotate = std::clamp(m_gyroAimer.CalculateToFaceStartingAngle(), -0.5, 0.5);
        // frc::SmartDashboard::PutNumber("FaceGridGyroAimer", rotate);
      } else {
        rotate = m_controls.DriveRotate();
      }

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

    armMove = armInput * 0.5;

    if (m_armHadInput && armInput == 0.0) {
      m_arm.ResetGoal();
      m_arm.SetGoal(m_arm.GetGoalWithOffset(0.005));
      m_armHadInput = false;
    } else if (armInput != 0.0) {
      m_arm.ResetGoal();
      m_armHadInput = true;
    }
 
    // Hold position
    if (!m_armHadInput) {
      double move = m_arm.CalculateMove();
      armMove = move;
    }

    double maxArmOutput = kHalfSpeed;
    armMove = std::clamp(armMove, -1 * maxArmOutput, maxArmOutput);
    m_armMotor.Set(armMove);
    if (m_armMotor.GetOutputCurrent() > 40) {
      m_armMotor.Set(armMove / 2);
    }

    double maxWristOutput = kHalfSpeed;
    double wristInput = m_controls.Wrist();

    double wristMove = wristInput;
    if (m_wristHadInput && wristInput == 0.0) {
      m_wrist.ResetGoal();
      m_wrist.SetGoal(m_wrist.GetGoalWithOffset(0.005));
      m_wristHadInput = false;
    } else if (wristInput != 0.0) {
      m_wrist.ResetGoal();
      m_wristHadInput = true;
    }
    if (!m_wristHadInput) {
      double wristPotMove = m_wrist.DegreesToPot(wristInput);
      m_wrist.MoveGoal(wristPotMove);

      wristMove = m_wrist.CalculateMove();
    }

    wristMove = std::clamp(wristMove, -1 * maxWristOutput, maxWristOutput);

    m_wristMotor.Set(wristMove);

    if (m_controls.ToggleLimits()){
      m_arm.ToggleLimits();
    }
  }

  void TeleopExit() override {
    // Our robot will leave Teleop mode after the game and we have 3 seconds before they score the results.
    // We should probably have this keep us still so that we can stay on the charging station.
    // Set the wheel speeds to zero and keep the stabilizer things deployed if we ever add those.
    setParkingBrake();
  }

  void AutonomousInit() override {
    if(m_digitBoard.GetPot() > DigitMXPDisplay::kPotCenter){
    // full counter clockwise is straight auto
    m_autoStates->Mode = AutoState::AutoModes::StraightBackup;

    } else{
      m_autoStates->Mode = AutoState::AutoModes::RampBackup;
      //full clockwise is ramp auto
    }

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
    m_gyroAimer.ResetGoal();
    m_gyroLeveller.ResetGoal();
    m_arm.ResetGoal();
    m_wrist.ResetGoal();
    m_robotDrive.SetMaxOutput(kHalfSpeed);

    m_autoStartingPosition = -1;

    // todo convert encoder position to feet/inches
    m_rearRightEncoder.SetPositionConversionFactor(2.4);
    m_autoStates->AutoStateFinish = false;
    m_autoStates->BackUpComplete = false;
    m_autoStates->BackupStraight = 0;
    m_autoStates->BackupStraightComplete = false;
    m_autoStates->Bounce = 0;
    m_autoStates->ContinueOntoPlatformComplete = false;
    m_autoStates->ExtendArm = 0;
    m_autoStates->ExtendArmComplete = false;
    m_autoStates->Level = 0;
    m_autoStates->LevelComplete = false;
    m_autoStates->LowerArmComplete = false;
    m_autoStates->LowerWristComplete = false;
    m_autoStates->OpenClaw = 0;
    m_autoStates->RetractArm = 0;
    m_autoStates->RaiseArmComplete = false;
    m_autoStates->OpenClawComplete = false;
    m_autoStates->RaiseWristComplete = false;
    m_autoStates->RetractArmComplete = false;
    m_autoForward = 0;
    m_autoRotate = 0;

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



   /*
    // find april tag to determine if on left/center/right

    // robot only needs to know its starting position once
    // <0: unknown   1: left   2: center   3: right
    if (m_autoStartingPosition < -1) {
      m_autoStartingPosition = m_cameraAimer.FindAutoStartFromAprilTags();

      // robot does not know where it is
      if (m_autoStartingPosition < -1) {
        HandleAutoDrive();
        return;
      }
    }

    // read the display thing with the buttons to determine if balancing or not


    bool isAutobalancing = false;
  
   
    // place game piece




    // drive out

    if (m_autoStartingPosition == 2) {
      // avoid going over the charge station if not autobalancing since might hit the other robot balancing
      if (isAutobalancing) {
        // TODO: investigate if it would be smart to have the robot drive around the charging station
        HandleAutoDrive();
        return;
      }


      // drive out over the balance station

    // driving out for left and right are both the same process
    } else {
      
    }

    // go balance
    if (isAutobalancing) {

    // if not balancing then grab another game piece
    } else {
      // turn around

    }
    */


    // Set all motors to 0 to avoid watchdog complaints
    for(auto motor : sparkMotors){
      motor->Set(0);
    }
    m_robotDrive.DriveCartesian(0, 0, 0);

    // Go through each AutonomousMode state and evaluate them one at a time.
    bool runAutonomousSequence = true;
    if (runAutonomousSequence){
      // Place a Cone
      if (!m_autoStates->RaiseArmComplete) {
        AutoStateRaiseArm();
      } else if (!m_autoStates->ExtendArmComplete) {
        AutoStateExtendArm();
      } else if (!m_autoStates->LowerWristComplete) {
        AutoStateLowerWrist();
      } else if (!m_autoStates->OpenClawComplete) {
        AutoStateOpenClaw();
      } else if (!m_autoStates->RaiseWristComplete) {
        AutoStateRaiseWrist();
      } else if (!m_autoStates->RetractArmComplete) {
        AutoStateRetractArm();
      }

      // After placing a Cone, determine which backup mode we're doing
      if (m_autoStates->RetractArmComplete) {
        if (m_autoStates->Mode == AutoState::AutoModes::StraightBackup) {
          // Backup and balance on the ramp
          if (!m_autoStates->BackupStraightComplete) {
            AutoStateBackupStraight();
          }
        } else if (m_autoStates->Mode == AutoState::AutoModes::RampBackup) {
          // Backup straight to leave the community
          if (!m_autoStates->BackUpComplete) {
            AutoStateBackUp();
          } else if (!m_autoStates->ContinueOntoPlatformComplete) { 
            AutoStateContinueOntoPlatform();
          }
        }
      }

      if (m_autoStates->AutoStateFinish) {
        AutoStateEnd();
      }
    }

    // testing
    //m_gyroAimer.SetGoal(15);

    // Update the arm based on new goals from the autonomous mode functions
    double armMove = m_arm.CalculateMove();
    double maxArmOutput = kQuarterSpeed;
    armMove = std::clamp(armMove, -1 * maxArmOutput, maxArmOutput);
    frc::SmartDashboard::PutNumber("arm move", armMove);
    m_armMotor.Set(armMove);

    // Update the wrist based on the goals too
    double wristMove = m_wrist.CalculateMove();
    frc::SmartDashboard::PutNumber("wrist move", wristMove);
    double maxWristOutput = kHalfSpeed;
    wristMove = std::clamp(wristMove, -1 * maxWristOutput, maxWristOutput);
    m_wristMotor.Set(wristMove);

    // Update the drive based on new goals
    m_autoRotate = 0;

    // keep the robot slow while testing to avoid catastrophe
    frc::SmartDashboard::PutNumber("AutoForward", m_autoForward);
    frc::SmartDashboard::PutNumber("AutoRotate", m_autoRotate);
    m_robotDrive.DriveCartesian(m_autoForward, 0, m_autoRotate);
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
  static constexpr int kPigeonIMU = 52;

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

  rev::SparkMaxRelativeEncoder m_frontLeftEncoder = m_frontLeft.GetEncoder();
  rev::SparkMaxRelativeEncoder m_frontRightEncoder = m_frontRight.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rearLeftEncoder = m_rearLeft.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rearRightEncoder = m_rearRight.GetEncoder();

  // Drivetrain kinematics for pose estimation
  units::meter_t kWheelDiameter = 6_in; // 6 inches (152.4mm)
  units::meter_t kWheelXSpacing = 23.25_in; // length-wise, forward/back wheel spacing, 23 3/16 inches (589mm)
  units::meter_t kWheelYSpacing = 22.5_in; // width-wise, left/right wheel spacing 22 1/2 inches (571.5mm)
  frc::Translation2d m_frontLeftWheel {kWheelXSpacing / 2, -1 * kWheelYSpacing / 2};
  frc::Translation2d m_frontRightWheel {kWheelXSpacing / 2, kWheelYSpacing / 2};
  frc::Translation2d m_rearLeftWheel {-1 * kWheelXSpacing / 2, -1 * kWheelYSpacing / 2};
  frc::Translation2d m_rearRightWheel {-1 * kWheelXSpacing / 2, kWheelYSpacing / 2};
  frc::MecanumDriveKinematics m_driveKinematics {m_frontLeftWheel, m_frontRightWheel, m_rearLeftWheel, m_rearRightWheel};
  frc::MecanumDriveWheelPositions m_driveWheelPositions {};
  frc::MecanumDrivePoseEstimator* m_drivePoseEstimator;

  double kEncoderResolution = 42;
  units::meter_t kDistancePerPulse = M_PI * kWheelDiameter / kEncoderResolution;

  // This SparkMax controls the arm up/down rotation
  rev::CANSparkMax m_armMotor{kArmChannel, rev::CANSparkMax::MotorType::kBrushless};

  // This SparkMax controls the claw's wrist rotation
  rev::CANSparkMax m_wristMotor{kWristChannel, rev::CANSparkMax::MotorType::kBrushless};
  
  frc::LinearFilter<double> m_accelerationXFilter = frc::LinearFilter<double>::MovingAverage(10);
  frc::LinearFilter<double> m_accelerationYFilter = frc::LinearFilter<double>::MovingAverage(10);
  frc::LinearFilter<double> m_accelerationZFilter = frc::LinearFilter<double>::MovingAverage(10);

  Controls m_controls {kXboxPort1, kXboxPort2, kDeadband};

  // https://wiki.analog.com/first/adis16448_imu_frc/cpp
  // frc::ADIS16448_IMU m_imu{};

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
  double kQuarterSpeed = 0.25;

  frc::Pose2d m_robotPose {};

  DigitMXPDisplay m_digitBoard {};

  ctre::phoenix::sensors::WPI_Pigeon2 m_pigeon{kPigeonIMU};

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
    Wrist m_wrist {};
    GyroAimer m_gyroAimer {&m_pigeon};
    GyroLeveller m_gyroLeveller {&m_pigeon};

    bool m_armHadInput = false;
    bool m_wristHadInput = false;
    double m_autoForward = 0.0;
    double m_autoRotate = 0.0;
    double m_highestLevel = 10;
    int m_autoStartingPosition = -1;

    AutoState* m_autoStates = nullptr;

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
      // Parking brake requires wheels to be down
      lowerWheels();
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


    void handleArmForInside() {
      retractArm();
      // close claw incase it's holding something
      closeClaw();
    }
    void handleArmForCollecting() {
      // move arm out far enough so it can pick things up
      // maybe let it move a little but clamp it? prevent driver from bringing the claw higher than it should be
    }

    // TODO get values for where the arm should be
    void handleArmForLowGoal() {
       closeClaw();
    }
    void handleArmForMiddleCone() {
       closeClaw();
    }
    void handleArmForMiddleCube() {
       closeClaw();
    }
    void handleArmForHighCone() {
       closeClaw();
    }
    void handleArmForHighCube() {
       closeClaw();
    }

    void handleReleasing() {
        openClaw();
    }



    void publishRobotState() {
    
      publishBrakeSet.Set(m_parkingBrake);
      publishWheelsDown.Set(m_wheelsDown);
      publishArmExtended.Set(m_arm.GetExtendedState());
      publishClawClosed.Set(m_clawClosed);

      std::stringstream poseStr;
      poseStr << "X: " << m_robotPose.X().value() << " Y: " << m_robotPose.Y().value();
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
      frc::SmartDashboard::PutNumber("WristPositionPot", m_wrist.GetPotPos());

      frc::SmartDashboard::PutBoolean("DigitDisplayA", m_digitBoard.GetButtonA());
      frc::SmartDashboard::PutBoolean("DigitDisplayB", m_digitBoard.GetButtonB());
      frc::SmartDashboard::PutNumber("DigitDisplayPot", m_digitBoard.GetPot());

      frc::SmartDashboard::PutNumber("GyroYaw", m_pigeon.GetYaw());
      frc::SmartDashboard::PutNumber("GyroPitch", m_pigeon.GetPitch());
      frc::SmartDashboard::PutNumber("GyroRoll", m_pigeon.GetRoll());
      frc::SmartDashboard::PutNumber("GyroHighestRoll", m_highestLevel);

      frc::SmartDashboard::PutNumber("WristGoal", m_wrist.GetGoal());
      frc::SmartDashboard::PutNumber("ArmGoal", m_arm.GetGoal());

      frc::SmartDashboard::PutBoolean("AutoRaiseArm", m_autoStates->RaiseArmComplete);
      frc::SmartDashboard::PutBoolean("AutoExtendArm", m_autoStates->ExtendArmComplete);
      frc::SmartDashboard::PutBoolean("AutoLowerWrist", m_autoStates->LowerWristComplete);
      frc::SmartDashboard::PutBoolean("AutoOpenClaw", m_autoStates->OpenClawComplete);
      frc::SmartDashboard::PutBoolean("AutoRaiseWrist", m_autoStates->RaiseWristComplete);
      frc::SmartDashboard::PutBoolean("AutoRetractArm", m_autoStates->RetractArmComplete);
      frc::SmartDashboard::PutBoolean("AutoBackUp", m_autoStates->BackUpComplete);
      frc::SmartDashboard::PutBoolean("AutoContinuePlatform", m_autoStates->ContinueOntoPlatformComplete);
      frc::SmartDashboard::PutBoolean("AutoLevel", m_autoStates->LevelComplete);
      frc::SmartDashboard::PutBoolean("AutoLowerArm", m_autoStates->LowerArmComplete);
      frc::SmartDashboard::PutBoolean("AutoBackupStraight", m_autoStates->BackupStraightComplete);
      frc::SmartDashboard::PutBoolean("AutoAutoFinish", m_autoStates->AutoStateFinish);
    }

    void resetEncoders() {
      m_armMotorEncoder.SetPosition(0);
      m_wristMotorEncoder.SetPosition(0);
    }

    void HandleAutoDrive() {
      // TODO: put auto drive stuff here  
    }

    // Raise the arm up all the way up to the top
    void AutoStateRaiseArm() {
      if (m_autoStates->RaiseArmComplete) {
        return;
      }

      m_arm.SetGoal(Arm::kEncoderUpperLimit);
      m_arm.SetGoal(m_arm.GetGoalWithOffset(0.005));
      m_wrist.SetGoal(Wrist::kPotUpperLimit);
      m_wrist.SetGoal(m_wrist.GetGoalWithOffset(0.005));
      
      // Check if it's done
      m_autoStates->RaiseArmComplete = m_arm.GetEncoderPosition() >= Arm::kEncoderUpperLimit - 0.1;
    }

    // Extend the arm out
    void AutoStateExtendArm() {
      if (m_autoStates->ExtendArmComplete) {
        return;
      }
      extendArm();
      m_autoStates->ExtendArm++;
      m_autoStates->ExtendArmComplete = m_autoStates->ExtendArm >= 50;
    }

    // Rotate the wrist down to level
    void AutoStateLowerWrist() {
      if (m_autoStates->LowerWristComplete) {
        return;
      }
      m_wrist.SetGoal(Wrist::kPotLevel);
      double offset = 0.2;
      m_wrist.SetGoal(m_wrist.GetGoalWithOffset(offset));
      m_autoStates->LowerWristComplete = m_wrist.GetPotPos() >= Wrist::kPotLevel - offset;
    }

    // Open the claw to drop the game piece
    void AutoStateOpenClaw() {
      if (m_autoStates->OpenClawComplete) {
        return;
      }
      openClaw();
      m_autoStates->OpenClaw++;
      m_autoStates->OpenClawComplete = m_autoStates->OpenClaw >= 80;
    }

     // Rotate the wrist back up
    void AutoStateRaiseWrist() {
      if (m_autoStates->RaiseWristComplete) {
        return;
      }
      m_wrist.SetGoal(Wrist::kPotUpperLimit);
      m_wrist.SetGoal(m_wrist.GetGoalWithOffset(0.005));
      m_autoStates->RaiseWristComplete = m_wrist.GetPotPos() < Wrist::kPotUpperLimit + 0.1;
    }

    // Retract the arm and close the claw
    void AutoStateRetractArm() {
      if (m_autoStates->RetractArmComplete) {
        return;
      }
      retractArm();
      closeClaw();
      m_autoStates->RetractArm++;
      m_autoStates->RetractArmComplete = m_autoStates->RetractArm >= 80;
    }

    // Move the robot backwards until we hit the platform and tip up a bit
    void AutoStateBackUp() {
      if (m_autoStates->BackUpComplete){
        return;
      }
      // Make sure the arm stays in and closed while we're lowering
      retractArm();
      closeClaw();

      m_autoForward = -0.5;

      m_autoStates->BackUpComplete = std::abs(m_gyroLeveller.GetPitch()) > 10;
    }

    // Continue onto the platform until it tips back downward
    void AutoStateContinueOntoPlatform() {
      if (m_autoStates->ContinueOntoPlatformComplete) {
        return;
      }
      retractArm();
      closeClaw();

      // Lower the arm
      double goal = Arm::kEncoderLowerLimit + 0.01;
      m_arm.SetGoal(goal);
      m_arm.SetGoal(m_arm.GetGoalWithOffset(0.005));
      m_wrist.SetGoal(Wrist::kPotUpperLimit);
      m_wrist.SetGoal(m_wrist.GetGoalWithOffset(0.05));
      //bool armIsDown = m_arm.GetEncoderPosition() <= goal + 0.1; // we don't care if it actually goes down

      double roll = m_gyroLeveller.GetPitch();

      if (std::abs(roll) > std::abs(m_highestLevel)) {
        m_highestLevel = roll;
      }

      bool robotTipping = std::abs(m_highestLevel) - std::abs(roll) > 5;

      m_autoStates->Bounce++;
      bool bounce = m_autoStates->Bounce >= 120;

      // Continue moving backwards
      m_autoForward = -0.4;

      if (bounce) {
        // but slow down after the initially wobbliness
        m_autoForward = -0.2;
      }

      m_autoStates->ContinueOntoPlatformComplete = robotTipping && bounce;

      if(m_autoStates->ContinueOntoPlatformComplete){
        // Stop moving forward
        m_autoForward = 0;
        m_autoRotate = 0;
        setParkingBrake();

        // This is a final autonomous state
        m_autoStates->AutoStateFinish = true;
      }
    }

    // Make the robot level on the platform
    /*
    void AutoStateLevel() {
      retractArm();
      closeClaw();

      m_gyroLeveller.SetGoal(0);

      m_autoForward = m_gyroLeveller.CalculateMove();
      m_autoForward *= 0.2;

       m_autoStates.Level++;
      bool timeout = m_autoStateCounters.Level >= 50;
      bool level = std::abs(m_pigeon.GetRoll()) < 3;
      m_autoState.LevelComplete = level && timeout;
    }
    */

    // Stop the robot when we're done with autonomous mode, ideally level on the platform
    void AutoStateEnd() {
      retractArm();
      closeClaw();
      setParkingBrake();
      m_arm.ResetGoal();
      m_wrist.ResetGoal();
      m_gyroLeveller.ResetGoal();
      m_gyroAimer.ResetGoal();
      m_autoForward = 0;
      m_autoRotate = 0;
    }

    // Lower the arm back inside the robot
    void AutoStateLowerArm() {
      if (m_autoStates->LowerArmComplete) {
        return;
      }
      // Make sure the arm stays in and closed while we're lowering
      retractArm();
      closeClaw();
      double goal = Arm::kEncoderLowerLimit + 0.1;
      m_arm.SetGoal(goal);
      m_autoStates->LowerArmComplete = m_arm.GetEncoderPosition() <= goal + 0.1;
    }

    void AutoStateBackupStraight() {
      if (m_autoStates->BackupStraightComplete) {
        return;
      }

      m_autoForward = -0.5;

      m_autoStates->BackupStraight++;
      bool time = m_autoStates->BackupStraight > 215; // This is tick timer, not a good thing to rely on but a backup in case everything else fails
  
      // todo check if position is greater than certain amount of feet

      m_autoStates->BackupStraightComplete = time;

      if (m_autoStates->BackupStraightComplete) {
        // This is a final autonomous state
        m_autoStates->AutoStateFinish = true;
      }
    }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
