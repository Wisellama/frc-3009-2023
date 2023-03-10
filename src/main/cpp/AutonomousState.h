#pragma once
// Any AutonomousState that does not have a precise sensor
// to verify that the state is done can instead use a counter
// to simply wait for some amount of time and hope it completes.
struct AutoStateCounters {
    int ExtendArm = 0;
    int LowerWrist = 0;
    int OpenClaw = 0;
    int RaiseWrist = 0;
    int RetractArm = 0;
};
