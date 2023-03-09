#pragma once

class FeedbackController {
    public:
    FeedbackController();
    ~FeedbackController(){};

    void SetGoal(double position);
    void MoveGoal(double move);
    double GetGoal();
    void ClampGoal(double min, double max);
    double CalculateMove(double position);
    bool NearGoal(double position, double margin = 2);

    private:
    double m_goal = 0.0;
    double m_position = 0.0;
};