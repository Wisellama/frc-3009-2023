#pragma once

class AutoAimResult {
  private:
  double m_forwardSpeed;
  double m_rotationSpeed;

  public:
  AutoAimResult(double f, double r) {
    m_forwardSpeed = f;
    m_rotationSpeed = r;
  }

  ~AutoAimResult() {};

  AutoAimResult(const AutoAimResult& other) {
    m_forwardSpeed = other.m_forwardSpeed;
    m_rotationSpeed = other.m_rotationSpeed;
  }

  double GetForwardSpeed() { return m_forwardSpeed; }
  double GetRotationSpeed() { return m_rotationSpeed; }
};