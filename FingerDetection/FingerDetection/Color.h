#pragma once

#include "opencv2\core\core.hpp"

using namespace cv;

class Color // RGBQUAD equivalent
{
private:
	uchar red;
	uchar green;
	uchar blue;

public:
	Color(Vec3b);
	Color();
	~Color();
	uchar getRedColor();
	uchar getBlueColor();
	uchar getGreenColor();
	void setRedColor(uchar);
	void setBlueColor(uchar);
	void setGreenColor(uchar);
};

