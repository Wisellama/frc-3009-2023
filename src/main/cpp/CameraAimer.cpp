#include <photonlib/PhotonUtils.h>

#include "CameraAimer.h"
#include "AutoAimResult.h"
#include "AprilTagPositionEnum.h"
#include "AprilTagHelpers.h"

AutoAimResult CameraAimer::AutoAim(int targetId) {
    // Based on this example:
    // https://github.com/PhotonVision/photonvision/blob/master/photonlib-cpp-examples/aimandrange/src/main/cpp/Robot.cpp

    // Use targetId -1 to specify "any" target. Then we'll use GetBestTarget().
    // Otherwise we'll only track for the exact target we're looking for.

    // Query the latest result from PhotonVision
    const auto& result = m_camera.GetLatestResult();

    double forwardSpeed = 0.0;
    double rotationSpeed = 0.0;

    if (result.HasTargets()) {
      auto allTargets = result.GetTargets();
      auto bestTarget = result.GetBestTarget();
      //int fiducialId = bestTarget.GetFiducialId(); // if we want to track a specific AprilTag, look for it in the results list

      std::vector<int> aprilTagsFound = getAprilTagIds(allTargets);

      bool facingBlue = lookingAtBlueCommunity(aprilTagsFound);
      bool facingRed = lookingAtRedCommunity(aprilTagsFound);

      auto inst = nt::NetworkTableInstance::GetDefault();
      auto table = inst.GetTable("datatable");
      m_publishFacingBlue.Set(facingBlue);
      m_publishFacingRed.Set(facingRed);
      m_publishFacingBlue = table->GetBooleanTopic("publishFacingBlue").Publish();
      m_publishFacingRed = table->GetBooleanTopic("publishFacingRed").Publish();

      // First calculate range
      units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
          m_translation.Z(), TARGET_HEIGHT, m_rotation.Y(),
          units::degree_t{result.GetBestTarget().GetPitch()});

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_
      // range
      forwardSpeed = -1 * m_forwardController.Calculate(range.value(), GOAL_RANGE_METERS.value());

      // Also calculate angular power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -1 * m_turnController.Calculate(result.GetBestTarget().GetYaw(), 0);
    }

    AutoAimResult output {forwardSpeed, rotationSpeed};
    return output;
}

std::optional<photonlib::EstimatedRobotPose> CameraAimer::EstimatePose(frc::Pose3d previous) {
  m_photonPoseEstimator.SetReferencePose(previous);
  return m_photonPoseEstimator.Update();
}
