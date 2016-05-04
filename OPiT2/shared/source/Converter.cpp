#include "Converter.h"
#include <iostream>

using namespace cv;
using namespace std;



/*
 *	if you know the camera's film (or digital sensor) has a width W in millimiters, 
		and the image width in pixels is w, you can convert the focal length f to world units 

		W = digital sensor width in mm
		w = image width in pixels
		F = focal length in world units (mm)
		f = focal length in pixels
 */
float Converter::ImageToWorldF(float f, float W, float w)
{
	return f * (W / w);
}

float Converter::ImageToWorldX(float x, float W, float w)
{
	return x * (W / w);
}

float Converter::ImageToWorldY(float y, float H, float h)
{
	return y * (H / h);
}





float Converter::WorldToImageF(float F, float W, float w)
{
	return F / (W / w);
}

float Converter::WorldToImageX(float X, float W, float w)
{
	return X / (W / w);
}

float Converter::WorldToImageY(float Y, float H, float h)
{
	return Y / (H / h);
}

/*
	Normalisation of 2D coords
	Requires input: Mat(3, 1, DataType<double>::type);
*/
Mat Converter::normalize2d(Mat coords)
{
	coords.at<double>(0, 0) = coords.at<double>(0, 0) / coords.at<double>(2, 0);
	coords.at<double>(1, 0) = coords.at<double>(1, 0) / coords.at<double>(2, 0);
	coords.at<double>(2, 0) = 1;

	return coords;
}

/*
Normalisation of 3D coords
Requires input: Mat(4, 1, DataType<double>::type);
*/
Mat Converter::normalize3d(Mat coords)
{
	coords.at<double>(0, 0) = coords.at<double>(0, 0) / coords.at<double>(3, 0);
	coords.at<double>(1, 0) = coords.at<double>(1, 0) / coords.at<double>(3, 0);
	coords.at<double>(2, 0) = coords.at<double>(2, 0) / coords.at<double>(3, 0);
	coords.at<double>(3, 0) = 1;

	return coords;
}



// Very useful helper function to get the type of a Mat.
string Converter::type2str(int type)
{
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}