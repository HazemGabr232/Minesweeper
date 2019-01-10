#ifndef _HBRIDGE_H
#define _HBRIDGE_H

class HBridge {
private:
  int m1, m2, m3, m4, pwm1, pwm2;
  void setMotorsPWM(bool, bool, bool, bool, short, short);
  void setMotors(int, int, int, int);
  bool PWMEnabled = false;

public:
  HBridge(int, int, int, int, int, int);
  HBridge(int, int, int, int);

  void move(short, short);
  
  void forward();
  void backward();
  void left();
  void right();
  void stop();

  void enablePWM();
  void disablePWM();
};

#endif
