#pragma once
#include "Image.h"
#include <opencv2/video/video.hpp>
#include <opencv2/video/background_segm.hpp>

class TransImage : public Image {
public:
	TransImage() {};
	TransImage(Mat, int, int);
	~TransImage() {};

	void MakeReverse();
	void DetectSkinColor();
	void Contours();
	Point HandCenter();
	int CountFinger(double);
};