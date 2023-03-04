#include <photonlib/PhotonUtils.h>

#include "CameraAimer.h"
#include "AutoAimResult.h"
#include "AprilTagPositionEnum.h"
#include "AprilTagHelpers.h"

CameraAimer::CameraAimer() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("CameraAimer");
  m_publishFacingBlue = table->GetBooleanTopic("FacingBlue").Publish();
  m_publishFacingRed = table->GetBooleanTopic("FacingRed").Publish();
  m_publishBestTargetId = table->GetIntegerTopic("BestTargetId").Publish();
  m_publishTargetYaw = table->GetDoubleTopic("TargetYaw").Publish();
  m_publishReflectiveYaw = table->GetDoubleTopic("ReflectiveYaw").Publish();
  m_publishRotation = table->GetDoubleTopic("Rotation").Publish();
  m_publishForwardSpeed = table->GetDoubleTopic("ForwardSpeed").Publish();
  m_publishRange = table->GetDoubleTopic("Range").Publish();

  m_publishTargetYaw.Set(99.88);
  m_publishReflectiveYaw.Set(99.88);
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

      m_publishFacingBlue.Set(facingBlue);
      m_publishFacingRed.Set(facingRed);
      m_publishBestTargetId.Set(fiducialId);

      // First calculate range
      units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
          m_translationAprilTags.Z(), APRIL_TAG_HEIGHT, m_rotation.Y(),
          units::degree_t{result.GetBestTarget().GetPitch()});

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -1 * m_forwardController.Calculate(range.value(), ARM_LENGTH_FROM_CAMERA.value());
      m_publishRange.Set(range.value());
      m_publishForwardSpeed.Set(forwardSpeed);

      // Also calculate angular power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      m_publishTargetYaw.Set(result.GetBestTarget().GetYaw());
      rotationSpeed = -1 * m_turnController.Calculate(result.GetBestTarget().GetYaw(), 0);
      m_publishRotation.Set(rotationSpeed);
    }

    forwardSpeed = 0; // TODO Don't trust the values yet
    AutoAimResult output {forwardSpeed, rotationSpeed};
    return output;
}

AutoAimResult CameraAimer::AutoAimReflectiveTape() {
  const auto& result = m_cameraLimeLight.GetLatestResult();
  double rotationSpeed = 0.0;

  if (result.HasTargets()) {
    m_publishReflectiveYaw.Set(result.GetBestTarget().GetYaw());
    rotationSpeed = -1 * m_turnControllerReflectiveTape.Calculate(result.GetBestTarget().GetYaw(), 0);
  }

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