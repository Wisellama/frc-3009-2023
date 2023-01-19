double deadzone(double value, double threshold) {
  if (value > threshold || value < -1*threshold) {
    return value;
  } else {
    return 0.0;
  }
}  