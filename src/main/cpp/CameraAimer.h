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

  const units::meter_t APRIL_TAG_HEIGHT = 36_cm;

  // How far from the target we want to be
  const units::meter_t ARM_LENGTH_FROM_CAMERA = 122_cm + 16_cm; // Camera is inset 16cm inside the robot, arm is almost full length outside the robot

  const double LINEAR_P = 0.05;
  const double LINEAR_D = 0.01;
  frc2::PIDController m_forwardController{LINEAR_P, 0.0, LINEAR_D};

  const double ANGULAR_P = 1.0;
  const double ANGULAR_I = 0.0;
  const double ANGULAR_D = 0.1;
  frc2::PIDController m_turnController{ANGULAR_P, ANGULAR_I, ANGULAR_D};
  frc2::PIDController m_turnControllerReflectiveTape{ANGULAR_P, ANGULAR_I, ANGULAR_D};

  photonlib::PhotonCamera m_cameraAprilTags{CAMERA_MICROSOFT};
  // FRC uses the north-west-up coordinate system, where x = forward, y = horizontal, z = vertical
  units::meter_t forward = 8_in;
  units::meter_t horizontal = -13_in;
  units::meter_t vertical = 25_in;
  frc::Translation3d m_translationAprilTags {forward, horizontal, vertical}; // Camera location on the robot, measured from the center
  frc::Rotation3d m_rotation {0_deg, -1_deg, 0_deg};
  frc::Transform3d m_robotToCameraAprilTags{m_translationAprilTags, m_rotation};
  frc::AprilTagFieldLayout m_fieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp);
  photonlib::PoseStrategy m_poseStrategy = photonlib::PoseStrategy::CLOSEST_TO_REFERENCE_POSE;

  photonlib::PhotonCamera m_cameraReflectiveTape{CAMERA_LIMELIGHT};
  frc::Translation3d m_translationReflectiveTape {forward, -1*horizontal, vertical};
  frc::Transform3d m_robotToCameraReflectiveTape{m_translationReflectiveTape, m_rotation};

  photonlib::PhotonPoseEstimator m_photonPoseEstimatorAprilTags{
    m_fieldLayout,
    m_poseStrategy, 
    photonlib::PhotonCamera{CAMERA_MICROSOFT},
    m_robotToCameraAprilTags};

  public:
  CameraAimer();
  ~CameraAimer() {};

  AutoAimResult AutoAimAprilTags(int targetId);
  std::optional<photonlib::EstimatedRobotPose> EstimatePoseAprilTags(frc::Pose3d previous);
  AutoAimResult AutoAimReflectiveTape();
  void enableDriverVisionMicrosoft();
  void disableDriverVisionMicrosoft();
  void enableDriverVisionLimelight();
  void disableDriverVisionLimelight();
};