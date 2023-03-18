#include "GyroLeveller.h"

#include <ctre/phoenix/sensors/Pigeon2.h>
#include <algorithm>
#include <math.h>

GyroLeveller::GyroLeveller(ctre::phoenix::sensors::Pigeon2 *pigeon) {
    m_pigeon = pigeon;
}

double GyroLeveller::GetPitch() {
    // TODO update to roll because the pigeon is backwards
    return m_pigeon->GetPitch();
}

double GyroLeveller::CalculateMove() {
    double current = GetPitch();
    // The gyro will return ever increasing values, so keep it within -180 to 180
    // https://stackoverflow.com/a/9138794
    current = std::fmod(current, kMaxDegrees);

    double move = m_feedbackController.CalculateMove(current);

    // Convert from degrees to motor movement
    move = move / kMaxDegrees;

    return move;
}

void GyroLeveller::SetGoal(double goal) {
    // Keep any goal values within -180 to 180
    goal = std::fmod(goal, kMaxDegrees);
    m_feedbackController.SetGoal(goal);
}

void GyroLeveller::ResetGoal() {
    SetGoal(GetPitch());
}
