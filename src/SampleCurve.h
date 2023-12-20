#ifndef SAMPLE_CURVE
#define SAMPLE_CURVE

#include <map>
#include <vector>
#include "vector_math.h"

using namespace vmath;

struct SamplePoint{
  float i = 0.0f;
  float v = 0.0f;
};

class SampleCurve{
public:
  //constructors
  SampleCurve(){}

  SampleCurve(const std::vector<SamplePoint>& samplePoints){
    mSamplePoints = samplePoints;
  }

  void resize(int count){
    mSamplePoints.resize(count);
  }

  void setSamplePoint(int index, float i, float value){
    mSamplePoints[index] = {i, value};
  }

  float sample(float i){
    if(mSamplePoints.size() < 2){
      return 0.0f;
    }

    // clamp to outer right value
    if(i >= mSamplePoints[mSamplePoints.size() - 1].i){
      return mSamplePoints[mSamplePoints.size() - 1].v;
    }

    // clamp to outer left value
    if(i <= mSamplePoints[0].i){
      return mSamplePoints[0].v;
    }

    for(int index = 1; index < mSamplePoints.size(); index++){
      if(mSamplePoints[index].i > i){  
        auto firstSample = mSamplePoints[index- 1];
        auto secondSample = mSamplePoints[index];

        float factor = (i - firstSample.i) / (secondSample.i - firstSample.i);

        return interpolate(firstSample.v, secondSample.v, factor );
      }
    }

    return 0.0f;
  }

private:  
  // index, SamplePoint
	std::vector<SamplePoint> mSamplePoints{};
};

#endif //SAMPLE_CURVE