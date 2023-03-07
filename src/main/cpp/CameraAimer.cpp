#include <photonlib/PhotonUtils.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "CameraAimer.h"
#include "AutoAimResult.h"
#include "AprilTagPositionEnum.h"
#include "AprilTagHelpers.h"
#include "FeedbackController.h"

CameraAimer::CameraAimer() {
}

AutoAimResult CameraAimer::AutoAimAprilTags(int targetId) {
    // Based on this example:
    // https://github.com/PhotonVision/photonvision/blob/master/photonlib-cpp-examples/aimandrange/src/main/cpp/Robot.cpp

    // Use targetId -1 to specify "any" target. Then we'll use GetBestTarget().
    // Otherwise we'll only track for the exact target we're looking for.

    // Query the latest result from PhotonVision
    const auto& result = m_cameraMicrosoft.GetLatestResult();

    double forwardSpeed = 0.0;
    double rotationSpeed = 0.0;

    if (result.HasTargets()) {
      auto allTargets = result.GetTargets();
      auto bestTarget = result.GetBestTarget();
      int fiducialId = bestTarget.GetFiducialId(); // if we want to track a specific AprilTag, look for it in the results list

      std::vector<int> aprilTagsFound = getAprilTagIds(allTargets);

      bool facingBlue = lookingAtBlueCommunity(aprilTagsFound);
      bool facingRed = lookingAtRedCommunity(aprilTagsFound);

      frc::SmartDashboard::PutBoolean("FacingBlue", facingBlue);
      frc::SmartDashboard::PutBoolean("FacingRed", facingRed);
      frc::SmartDashboard::PutNumber("TargetId", fiducialId);

      // First calculate range for forward movement
      units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
          m_translationAprilTags.Z(), APRIL_TAG_HEIGHT, m_rotation.Y(),
          units::degree_t{result.GetBestTarget().GetPitch()});

      // Then set our goal to how far away we want to be
      m_forwardController.SetGoal(double(ARM_LENGTH_FROM_CAMERA));

      // Then use the goal and range with the feedback controller to determine how much we need to move.
      forwardSpeed = m_forwardController.CalculateMove(double(range));
      frc::SmartDashboard::PutNumber("AprilTagRange", double(range));
      frc::SmartDashboard::PutNumber("AprilTagForwardSpeed", forwardSpeed);

      // First get our yaw
      double yaw = result.GetBestTarget().GetYaw();

      // Then set our goal (of zero, we just want to line up with the target)
      m_turnController.SetGoal(0);

      // Then use the goal and yaw with the feedback controller to determine how much we need to move.
      rotationSpeed = m_turnController.CalculateMove(yaw);
      frc::SmartDashboard::PutNumber("RotationSpeed", rotationSpeed);
      frc::SmartDashboard::PutNumber("Yaw", yaw);
    }

    forwardSpeed = 0; // TODO Don't trust the values yet, just print them out first
    rotationSpeed = 0;
    AutoAimResult output {forwardSpeed, rotationSpeed};
    return output;
}

AutoAimResult CameraAimer::AutoAimReflectiveTape() {
  const auto& result = m_cameraLimeLight.GetLatestResult();
  double rotationSpeed = 0.0;

  if (result.HasTargets()) {
    double yaw = result.GetBestTarget().GetYaw();
    m_turnController.SetGoal(0);
    rotationSpeed = m_turnController.CalculateMove(yaw);
    frc::SmartDashboard::PutNumber("RotationSpeed", rotationSpeed);
    frc::SmartDashboard::PutNumber("Yaw", yaw);
  }

  // TODO don't trust the values yet, just print them out first
  rotationSpeed = 0;

  AutoAimResult output {0, rotationSpeed};
  return output;
}

std::optional<photonlib::EstimatedRobotPose> CameraAimer::EstimatePoseAprilTags(frc::Pose3d previous) {
  m_photonPoseEstimatorAprilTags.SetReferencePose(previous);
  return m_photonPoseEstimatorAprilTags.Update();
}

void CameraAimer::enableDriverVisionMicrosoft() {
  m_cameraMicrosoft.SetDriverMode(true);
}

void CameraAimer::disableDriverVisionMicrosoft() {
  m_cameraMicrosoft.SetDriverMode(false);
}

void CameraAimer::enableDriverVisionLimelight() {
  m_cameraLimeLight.SetDriverMode(true);
}

void CameraAimer::disableDriverVisionLimelight() {
  m_cameraLimeLight.SetDriverMode(false);
}

units::meter_t CameraAimer::getDistanceFromGrid() {
  const auto& result = m_cameraMicrosoft.GetLatestResult();

  if (!result.HasTargets()) {
    return -1_m;
  }

  std::vector<int> aprilTagsFound = getAprilTagIds(result.GetTargets());

  if (!lookingAtOwnCommunity(aprilTagsFound)) {
    return -1_m;
  }

  // units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
  //         CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH
  //         units::degree_t{result.GetBestTarget().GetPitch()});

  return -1_m;
}

void CameraAimer::limeLightPipelineAprilTags() {
  m_cameraLimeLight.SetPipelineIndex(PIPELINE_APRILTAGS);
}

void CameraAimer::limeLightPipelineReflectiveTape() {
  m_cameraLimeLight.SetPipelineIndex(PIPELINE_REFLECTIVETAPE);
}

void CameraAimer::limeLightPipelineColors() {
  m_cameraLimeLight.SetPipelineIndex(PIPELINE_COLORS);
}

void CameraAimer::ToggleAprilTagMode() {
  m_aprilTagMode = !m_aprilTagMode;
  if (m_aprilTagMode) {
    SetAprilTagMode();
  } else {
    // Enable Driver mode to give us a smoother live feed
    enableDriverVisionMicrosoft();
    enableDriverVisionLimelight();
  }
}

void CameraAimer::ToggleReflectiveTapeMode() {
  m_reflectiveTapeMode = !m_reflectiveTapeMode;

  if (m_reflectiveTapeMode) {
    SetReflectiveTapeMode();
  } else {
    // Enable Driver mode to give us a smoother live feed
    enableDriverVisionLimelight();
  }
}

// This only applies to the LimeLight
void CameraAimer::SetAprilTagMode() {
  // Disable other modes
  m_reflectiveTapeMode = false;
  m_colorMode = false;

  // Disable Driver mode to enable pipeline processing
  disableDriverVisionLimelight();

  // Set the LimeLight to AprilTag Mode
  m_cameraLimeLight.SetPipelineIndex(PIPELINE_APRILTAGS);
}

void CameraAimer::SetReflectiveTapeMode(){
  // Disable other modes
  m_aprilTagMode = false;
  m_colorMode = false;

  // Disable Driver mode to enable pipeline processing
  disableDriverVisionLimelight();

  // Set the LimeLight to Reflective Tape Mode
  m_cameraLimeLight.SetPipelineIndex(PIPELINE_REFLECTIVETAPE);
}

bool CameraAimer::GetAprilTagMode() {
  return m_aprilTagMode;
}

bool CameraAimer::GetReflectiveTapeMode() {
  return m_reflectiveTapeMode;
}