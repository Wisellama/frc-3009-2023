#pragma once

#include <frc/controller/PIDController.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <photonlib/PhotonPoseEstimator.h>

#include <networktables/BooleanTopic.h>

#include "AutoAimResult.h"
#include "AprilTagPositionEnum.h"
#include "FeedbackController.h"

std::string PoseToStr(frc::Pose2d pos);
frc::Pose2d Pose3dTo2d(frc::Pose3d pose3d);

class CameraAimer {
  nt::BooleanPublisher m_publishFacingBlue;
  nt::BooleanPublisher m_publishFacingRed;
  nt::IntegerPublisher m_publishBestTargetId;
  nt::DoublePublisher m_publishReflectiveYaw;
  nt::DoublePublisher m_publishTargetYaw;
  nt::DoublePublisher m_publishRotation;
  nt::DoublePublisher m_publishForwardSpeed;
  nt::DoublePublisher m_publishRange;

  private:
  const std::string CAMERA_MICROSOFT = "Microsoft_LifeCam_HD-3000";
  const std::string CAMERA_LIMELIGHT = "OV5647";

  const int PIPELINE_APRILTAGS = 0;
  const int PIPELINE_REFLECTIVETAPE = 1;
  const int PIPELINE_COLORS = 2;

  const units::meter_t APRIL_TAG_HEIGHT = 42_cm;

  // How far from the target we want to be
  const units::meter_t ARM_LENGTH_FROM_CAMERA = 122_cm + 16_cm; // Camera is inset 16cm inside the robot, arm is almost full length outside the robot

  FeedbackController m_forwardController{};
  FeedbackController m_turnController{};

  photonlib::PhotonCamera m_cameraMicrosoft{CAMERA_MICROSOFT};
  // FRC uses the north-west-up coordinate system, where x = forward, y = horizontal, z = vertical
  // The translation amount is also measured from the center of the robot
  frc::Transform3d m_robotToCameraLimeLight{frc::Translation3d{24_cm, 14_cm, 82_cm}, frc::Rotation3d{0_deg, 0_deg, 0_deg}};
  frc::Transform3d m_robotToCameraMicrosoft{frc::Translation3d{25_cm, -19_cm, 82_cm}, frc::Rotation3d{0_deg, 10_deg, 0_deg}};
  frc::AprilTagFieldLayout m_fieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp);
  photonlib::PoseStrategy m_poseStrategy = photonlib::PoseStrategy::MULTI_TAG_PNP;

  photonlib::PhotonCamera m_cameraLimeLight{CAMERA_LIMELIGHT};

  photonlib::PhotonPoseEstimator m_poseEstimatorLimeLight{
    m_fieldLayout,
    m_poseStrategy, 
    photonlib::PhotonCamera{CAMERA_LIMELIGHT},
    m_robotToCameraLimeLight};

  photonlib::PhotonPoseEstimator m_poseEstimatorMicrosoft{
    m_fieldLayout,
    m_poseStrategy, 
    photonlib::PhotonCamera{CAMERA_MICROSOFT},
    m_robotToCameraMicrosoft};

  bool m_reflectiveTapeMode = false;
  bool m_aprilTagMode = false;
  bool m_colorMode = false;

  public:
  CameraAimer();
  ~CameraAimer() {};

  AutoAimResult AutoAimAprilTags(int targetId);
  std::optional<photonlib::EstimatedRobotPose> EstimatePoseAprilTags(frc::Pose2d previous);
  std::optional<photonlib::EstimatedRobotPose> EstimatePoseAprilTags(frc::Pose3d previous);
  AutoAimResult AutoAimReflectiveTape();

  void ToggleAprilTagMode();
  void ToggleReflectiveTapeMode();
  void SetAprilTagMode();
  void SetReflectiveTapeMode();
  bool GetAprilTagMode();
  bool GetReflectiveTapeMode();

  units::meter_t getDistanceFromGrid();

  void enableDriverVisionMicrosoft();
  void disableDriverVisionMicrosoft();

  void enableDriverVisionLimelight();
  void disableDriverVisionLimelight();

  void limeLightPipelineAprilTags();
  void limeLightPipelineReflectiveTape();
  void limeLightPipelineColors();
};