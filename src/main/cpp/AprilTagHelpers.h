#pragma once

#include <vector>
#include <span>
#include <photonlib/PhotonTrackedTarget.h>

#include "AprilTagPositionEnum.h"

// Search through a list of known ids to see if any detected ids match.
// If we find a match, then we know we are looking at the AprilTags in the searchList.
bool lookingAtGivenList(std::vector<AprilTagPosition> searchList, std::vector<int> idsDetected);

std::vector<int> getAprilTagIds(std::span<const photonlib::PhotonTrackedTarget> targets);

bool lookingAtBlueCommunity(std::vector<int> idsDetected);

bool lookingAtRedCommunity(std::vector<int> idsDetected);

bool lookingAtOwnCommunity(std::vector<int> idsDetected);

bool lookingAtOwnSubstation(std::vector<int> idsDetected);

// Look for a specific AprilTag ID in the detected targets.
bool targetFound(int id, std::span<const photonlib::PhotonTrackedTarget> targets);

// returns -1 for no target, 1 for left, 2 for center, 3 for right
int findAutoStart(int bestid);