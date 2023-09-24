#ifndef LERP_PASS_FLTR
#define LERP_PASS_FLTR

#include <cmath>

class LerpFilter{
public:
	//constructors
	LerpFilter():
	output(0.0f),
	factor(0.5f){}


LerpFilter(float factor):
	output(0.0f),
	factor(factor)
{
}

float update(float input){
	return output = input * (1.0f - factor) + output * factor;
}

float update(float input, float i){
  factor = i;
	return output = input * factor + output * (1.0f - factor);
}

private:
	float output;
	float factor;
};

#endif //LERP_PASS_FLTR