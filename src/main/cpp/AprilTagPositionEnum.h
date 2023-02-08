#pragma once

// https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/TeamVersions/AprilTags-UserGuideandImages.pdf

/*
A reminder on the location of the various IDs on the field:
• Red Alliance Community (right to left) – IDs 1, 2, 3
• Blue Alliance Double Substation – ID 4
• Red Alliance Double Substation – ID 5
• Blue Alliance Community (right to left) – IDs 6, 7, 8
*/

enum class AprilTagPosition {
    RedCommunityRight = 1,
    RedCommunityCenter = 2,
    RedCommunityLeft = 3,

    BlueSubstation = 4,

    RedSubstation = 5,

    BlueCommunityRight = 6,
    BlueCommunityCenter = 7,
    BlueCommunityLeft = 8
};
