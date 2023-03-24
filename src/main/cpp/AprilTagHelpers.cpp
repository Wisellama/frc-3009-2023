#include "AprilTagHelpers.h"
#include "AprilTagPositionEnum.h"

#include <frc/DriverStation.h>

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

bool listHasEntry(AprilTagPosition idSearchingFor, std::vector<int> idsDetected) {
  for (int id : idsDetected) {
    if (AprilTagPosition(id) == idSearchingFor) {
      return true;
    }
  }

  return false;
}

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

// Defaults to red alliance
bool lookingAtOwnCommunity(std::vector<int> idsDetected) {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    return lookingAtBlueCommunity(idsDetected);
  } else {
    return lookingAtRedCommunity(idsDetected);
  };
};

bool lookingAtOwnSubstation(std::vector<int> idsDetected) {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    return listHasEntry(AprilTagPosition::RedSubstation, idsDetected);
  } else {
    return listHasEntry(AprilTagPosition::BlueSubstation, idsDetected);
  };
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

int findAutoStart(int bestId) {
  const static AprilTagPosition bestTag = AprilTagPosition(bestId);

  if (bestTag == AprilTagPosition::BlueCommunityCenter || bestTag == AprilTagPosition::RedCommunityCenter) {
    return 2;
  }

  if (bestTag == AprilTagPosition::BlueCommunityLeft || bestTag == AprilTagPosition::RedCommunityLeft) {
    return 1;
  }

if (bestTag == AprilTagPosition::BlueCommunityRight || bestTag == AprilTagPosition::RedCommunityRight) {
    return 3;
  }

  return -1;
}