#pragma once
// Any AutonomousState that does not have a precise sensor
// to verify that the state is done can instead use a counter
// to simply wait for some amount of time and hope it completes.


struct AutoState {
    enum AutoModes { RampBackup, StraightBackup };

    AutoModes Mode = RampBackup; // change to StraightBackup to test other mode

    int ExtendArm = 0;
    int OpenClaw = 0;
    int RetractArm = 0;
    int Level = 0;
    int Bounce = 0;
    int BackupStraight = 0;

    bool RaiseArmComplete = false;
    bool ExtendArmComplete = false;
    bool LowerWristComplete = false;
    bool OpenClawComplete = false;
    bool RaiseWristComplete = false;
    bool RetractArmComplete = false;
    bool BackUpComplete = false;
    bool ContinueOntoPlatformComplete = false;
    bool LevelComplete = false;
    bool LowerArmComplete = false;
    bool BackupStraightComplete = false;

    bool AutoStateFinish = false;
};
