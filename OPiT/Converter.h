#ifndef __Converter_H_INCLUDED__   // if x.h hasn't been included yet...
#define __Converter_H_INCLUDED__
#include <opencv2/features2d.hpp>
#include <iostream>
class Converter
{
public:
	static float ImageToWorldX(float, float, float);
	static float ImageToWorldY(float, float, float);
	static float ImageToWorldF(float, float, float);

	static float WorldToImageX(float, float, float);
	static float WorldToImageY(float, float, float);
	static float WorldToImageF(float, float, float);
private:
	
};

#endif