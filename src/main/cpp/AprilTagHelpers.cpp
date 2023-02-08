#include "AprilTagHelpers.h"
#include "AprilTagPositionEnum.h"

/*
These helper functions might not be useful if we just end up using their Pose Estimation stuff.
I just wanted to have something to debug with so we can tell which side of the field we're looking at.
*/

std::vector<AprilTagPosition> AprilTagsBlueCommunity {
    AprilTagPosition::BlueCommunityRight,
    AprilTagPosition::BlueCommunityCenter,
    AprilTagPosition::BlueCommunityRight
};

std::vector<AprilTagPosition> AprilTagsRedCommunity {
    AprilTagPosition::RedCommunityRight,
    AprilTagPosition::RedCommunityCenter,
    AprilTagPosition::RedCommunityRight
};

bool lookingAtGivenList(std::vector<AprilTagPosition> searchList, std::vector<int> idsDetected) {
  for (int id : idsDetected) {
    // https://stackoverflow.com/a/24139474
    if (std::find(searchList.begin(), searchList.end(), AprilTagPosition(id)) != searchList.end()) {
      return true;
    }
  }
  return false;
}

std::vector<int> getAprilTagIds(std::span<const photonlib::PhotonTrackedTarget> targets) {
  std::vector<int> output;
  for (auto target : targets) {
    output.push_back(target.GetFiducialId());
  }

  return output;
}

bool lookingAtBlueCommunity(std::vector<int> idsDetected) {
    return lookingAtGivenList(AprilTagsBlueCommunity, idsDetected);
}

bool lookingAtRedCommunity(std::vector<int> idsDetected) {
    return lookingAtGivenList(AprilTagsRedCommunity, idsDetected);
}

// Look for a specific AprilTag ID in the detected targets.
bool targetFound(int id, std::span<const photonlib::PhotonTrackedTarget> targets) {
  for (auto target : targets) {
    if (id == target.GetFiducialId()) {
      return true;
    }
  }
  return false;
}