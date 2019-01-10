#ifndef _METAL_DETECTOR_H
#define _METAL_DETECTOR_H

class MetalDetector {
private:
  int pin;
public:
  MetalDetector(int);
  long detect();
};

#endif
