#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\opencv.hpp>
#include <iostream>
#include "Color.h"
using namespace cv;
using namespace std;


class Image {
protected:
	Mat img;
	Mat SkinColorImg;
	Mat ContourImg;
	int width;
	int height;
	Point center;
	double radius;
	Image();

public:
	Image(int, int);
	~Image();
	int getWidth();
	int getHeight();
	Color getColorAtIndex(int, int);
	void setColorAtIndex(int, int, Color);
	void printLogDetection();
	void setPicture(Mat);
	Mat getPicture();

};