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

  private:
  const std::string CAMERA_MICROSOFT = "Microsoft_LifeCam_HD-3000";
  const std::string CAMERA_LIMELIGHT = "OV5647";

  const units::meter_t TARGET_HEIGHT = 3_ft;

  // How far from the target we want to be
  const units::meter_t GOAL_RANGE_METERS = 3_ft;

  const double LINEAR_P = 0.1;
  const double LINEAR_D = 0.0;
  frc2::PIDController m_forwardController{LINEAR_P, 0.0, LINEAR_D};

  const double ANGULAR_P = 0.1;
  const double ANGULAR_D = 0.0;
  frc2::PIDController m_turnController{ANGULAR_P, 0.0, ANGULAR_D};

  const double REFLECTIVE_P = 0.1;
  const double REFLECTIVE_D = 0.0;
  frc2::PIDController m_turnControllerReflectiveTape{REFLECTIVE_P, 0.0, REFLECTIVE_D};

  photonlib::PhotonCamera m_cameraAprilTags{CAMERA_MICROSOFT};
  frc::Translation3d m_translation {24_in, 0.0_in, 24_in}; // Camera location on the robot
  frc::Rotation3d m_rotation {0_deg, 0_deg, 0_deg};
  frc::Transform3d m_robotToCamera{m_translation, m_rotation};
  frc::AprilTagFieldLayout m_fieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp);
  photonlib::PoseStrategy m_poseStrategy = photonlib::PoseStrategy::CLOSEST_TO_REFERENCE_POSE;

  photonlib::PhotonCamera m_cameraReflectiveTape{CAMERA_LIMELIGHT};

  photonlib::PhotonPoseEstimator m_photonPoseEstimator{
    m_fieldLayout,
    m_poseStrategy, 
    photonlib::PhotonCamera{CAMERA_MICROSOFT},
    m_robotToCamera};

  public:
  CameraAimer();
  ~CameraAimer() {};

  AutoAimResult AutoAimAprilTags(int targetId);
  std::optional<photonlib::EstimatedRobotPose> EstimatePose(frc::Pose3d previous);
  AutoAimResult AutoAimReflectiveTape();
};