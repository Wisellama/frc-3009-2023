#include <photonlib/PhotonUtils.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "CameraAimer.h"
#include "AutoAimResult.h"
#include "AprilTagPositionEnum.h"
#include "AprilTagHelpers.h"
#include "FeedbackController.h"

CameraAimer::CameraAimer() {
  m_poseEstimatorLimeLight.SetMultiTagFallbackStrategy(photonlib::PoseStrategy::LOWEST_AMBIGUITY);
  m_poseEstimatorMicrosoft.SetMultiTagFallbackStrategy(photonlib::PoseStrategy::LOWEST_AMBIGUITY);
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
        m_robotToCameraLimeLight.Translation().Z(),
        APRIL_TAG_HEIGHT,
        m_robotToCameraLimeLight.Rotation().Y(),
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

std::optional<photonlib::EstimatedRobotPose> CameraAimer::EstimatePoseAprilTags(frc::Pose2d previous) {
  return EstimatePoseAprilTags(frc::Pose3d(previous));
}

std::optional<photonlib::EstimatedRobotPose> CameraAimer::EstimatePoseAprilTags(frc::Pose3d previous) {
  // Update our last pose with the new previous value
  m_poseEstimatorMicrosoft.SetLastPose(previous);
  m_poseEstimatorLimeLight.SetLastPose(previous);
  m_poseEstimatorLimeLight.SetReferencePose(previous);
  m_poseEstimatorMicrosoft.SetReferencePose(previous);

  auto llUpdate = m_poseEstimatorLimeLight.Update();
  auto mUpdate = m_poseEstimatorMicrosoft.Update();

  // If we only got 1 value, use that
  // Otherwise, create some sort of average between the 2
  auto compositePose = llUpdate;
  bool bothHadValues = llUpdate.has_value() && mUpdate.has_value();
  if (bothHadValues) {
    if (llUpdate.has_value()) {
      compositePose = llUpdate;
    } else if (mUpdate.has_value()) {
      compositePose = mUpdate;
    }
  } else {
    // We found an AprilTag with both cameras.
    frc::Transform3d diff = llUpdate->estimatedPose - mUpdate->estimatedPose;
    diff = diff / 2;
    compositePose->estimatedPose = compositePose->estimatedPose.TransformBy(diff);
    compositePose->timestamp = (llUpdate->timestamp + mUpdate->timestamp) / 2;
  }

  frc::SmartDashboard::PutString("PoseComposite", PoseToStr(Pose3dTo2d(compositePose->estimatedPose)));
  frc::SmartDashboard::PutString("PoseLL", PoseToStr(Pose3dTo2d(llUpdate->estimatedPose)));
  frc::SmartDashboard::PutString("PoseM", PoseToStr(Pose3dTo2d(mUpdate->estimatedPose)));

  return compositePose;
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
  m_aprilTagMode = true;
  m_reflectiveTapeMode = false;
  m_colorMode = false;

  // Disable Driver mode to enable pipeline processing
  disableDriverVisionLimelight();
  disableDriverVisionMicrosoft();

  // Set the LimeLight to AprilTag Mode
  m_cameraLimeLight.SetPipelineIndex(PIPELINE_APRILTAGS);
  m_cameraMicrosoft.SetPipelineIndex(PIPELINE_APRILTAGS);
}

void CameraAimer::SetReflectiveTapeMode(){
  // Disable other modes
  m_reflectiveTapeMode = true;
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

std::string PoseToStr(frc::Pose2d pos) {
  double x = double(pos.X());
  double y = double(pos.Y());

  // https://en.cppreference.com/w/cpp/io/c/fprintf
  int size = std::snprintf(nullptr, 0, "X: %f.2, Y: %f.2", x, y);
  std::vector<char> buf(size + 1);
  std::snprintf(&buf[0], buf.size(), "X: %f.2, Y: %f.2", x, y);

  std::string output = std::string(&buf[0]);

  return output;
}

frc::Pose2d Pose3dTo2d(frc::Pose3d pose3d) {
  frc::Rotation3d rotation3d = pose3d.Rotation();
  frc::Pose2d pose2d {frc::Translation2d{pose3d.X(), pose3d.Y()}, rotation3d.Z()};
  return pose2d;
}