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
  private:
  const units::meter_t TARGET_HEIGHT = 3_ft;

  // How far from the target we want to be
  const units::meter_t GOAL_RANGE_METERS = 3_ft;

  const double LINEAR_P = 0.1;
  const double LINEAR_D = 0.0;
  frc2::PIDController m_forwardController{LINEAR_P, 0.0, LINEAR_D};

  const double ANGULAR_P = 0.1;
  const double ANGULAR_D = 0.0;
  frc2::PIDController m_turnController{ANGULAR_P, 0.0, ANGULAR_D};

  photonlib::PhotonCamera m_camera{"photonvision"};
  frc::Translation3d m_translation {24_in, 0.0_in, 24_in}; // Camera location on the robot
  frc::Rotation3d m_rotation {0_deg, 0_deg, 0_deg};
  frc::Transform3d m_robotToCamera{m_translation, m_rotation};
  frc::AprilTagFieldLayout m_fieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp);
  photonlib::PoseStrategy m_poseStrategy = photonlib::PoseStrategy::CLOSEST_TO_REFERENCE_POSE;

  photonlib::PhotonPoseEstimator m_photonPoseEstimator{
    m_fieldLayout,
    m_poseStrategy, 
    photonlib::PhotonCamera{"photonvision"}, // this didn't work when I referenced m_camera for weird C++ reasons
    m_robotToCamera};

  nt::BooleanPublisher m_publishFacingBlue;
  nt::BooleanPublisher m_publishFacingRed;

  public:
  CameraAimer() {}; // Using all constants, so no constructor needed.
  ~CameraAimer() {};

  AutoAimResult AutoAim(int targetId);
  std::optional<photonlib::EstimatedRobotPose> EstimatePose(frc::Pose3d previous);
};