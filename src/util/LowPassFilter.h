#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <cmath>

namespace SimITL{

  class LowPassFilter{
  public:
    //constructors
    LowPassFilter():
    output(0),
    ePow(0){}


  LowPassFilter(float iCutOffFrequency, float iDeltaTime):
    output(0),
    ePow(1.0f - exp(-iDeltaTime * 2.0f * M_PI * iCutOffFrequency))
  {
  }

  float update(float input){
    return output += (input - output) * ePow;
  }

  float update(float input, float deltaTime, float cutoffFrequency){
    reconfigureFilter(deltaTime, cutoffFrequency); //Changes ePow accordingly.
    return output += (input - output) * ePow;
  }

  void reconfigureFilter(float deltaTime, float cutoffFrequency){
    ePow = 1.0f - exp(-deltaTime * 2.0f * M_PI * cutoffFrequency);
  }

  private:
    float output;
    float ePow;
  };

}

#endif //LOW_PASS_FILTER_H