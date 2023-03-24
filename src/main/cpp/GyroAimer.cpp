#include "GyroAimer.h"

#include <ctre/phoenix/sensors/Pigeon2.h>
#include <algorithm>
#include <math.h>

GyroAimer::GyroAimer(ctre::phoenix::sensors::Pigeon2 *pigeon) {
    m_pigeon = pigeon;
    m_startingYaw = m_pigeon->GetYaw();
}

double GyroAimer::GetYaw() {
    return m_pigeon->GetYaw();
}

double GyroAimer::CalculateMove() {
    double current = GetYaw() - m_startingYaw;
    // The gyro will return ever increasing values, so keep it within -180 to 180
    // https://stackoverflow.com/a/9138794
    current = std::fmod(current, kMaxDegrees);

    double move = m_feedbackController.CalculateMove(current);

    // Convert from degrees to motor movement
    move = move / kMaxDegrees;

    return move;
}

void GyroAimer::SetGoal(double goal) {
    // Keep any goal values within -180 to 180
    goal -= m_startingYaw;
    goal = std::fmod(goal, kMaxDegrees);
    m_feedbackController.SetGoal(goal);
}

void GyroAimer::SetGoalToOtherSide() {
    double newGoal = std::fmod(GetYaw() + 180, kMaxDegrees);
    m_feedbackController.SetGoal(newGoal);
}

void GyroAimer::ResetGoal() {
    SetGoal(GetYaw());
}

double GyroAimer::CalculateToFaceStartingAngle() {
    // imu calibrates and starts at -90
    // this will probably break if the gyro is ever calibrated again for some reason
    m_feedbackController.SetGoal(m_startingYaw);
    return CalculateMove();
}