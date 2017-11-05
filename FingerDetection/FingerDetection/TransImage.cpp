#include "TransImage.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/background_segm.hpp>
#include "opencv2/opencv.hpp"
//#include "opencv2/bgsegm.hpp"
#include <iostream>
#include <sstream>
#include <queue>

using namespace cv;
float innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1);
TransImage::TransImage(Mat image, int inputWidth, int inputHeight) {
	this->img = image;
	this->SkinColorImg = Mat::zeros(image.size(), CV_8UC3);
	this->ContourImg = Mat::zeros(image.size(), CV_8UC3);
	this->width = inputWidth;
	this->height = inputHeight;
}

void TransImage::MakeReverse() {
	Mat reversed;
	flip(this->img, reversed, 1);
	this->setPicture(reversed);
}

void TransImage::DetectSkinColor() {
//	imshow("original", this->img);
	int elementSize = 3;

//	GaussianBlur(this->img, this->img, Size(9, 9), 2.0, 2.0);
//	cv::imshow("blur", this->img);

	// HSV를 이용한 Skin Detection
	Mat HSVSkin;
	int Hmin = 0, Hmax = 20;
	int Smin = 10, Smax = 150;
	int Vmin = 60, Vmax = 255;

	cvtColor(this->img, HSVSkin, CV_BGR2HSV);
	inRange(HSVSkin, Scalar(Hmin, Smin, Vmin), Scalar(Hmax, Smax, Vmax), HSVSkin);

	medianBlur(HSVSkin, HSVSkin, 5);


//	erode(HSVSkin, HSVSkin, Mat());
//	GaussianBlur(HSVSkin, HSVSkin, Size(9, 9), 2.0, 2.0);
	Mat elementHSV = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * elementSize + 1, 2 * elementSize + 1), cv::Point(elementSize, elementSize));
	dilate(HSVSkin, HSVSkin, elementHSV);

//	cv::imshow("HSVblur", HSVSkin);
	/*
	// RGB를 이용한 Skin Detection
	Mat BGRSkin;
	Color color;
	uchar b, g, r;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			color = this->getColorAtIndex(x, y);
			b = color.getBlueColor();
			g = color.getGreenColor();
			r = color.getRedColor();
			if (b > 20 && g > 40 && r > 95 && (max(max(b, g), r) - min(min(b, g), r)) > 15 && abs(r - g) > 15 && r > g && r > b) {
				this->setColorAtIndex(x, y, Color({ 255,255,255 }));
			}
			else {
				this->setColorAtIndex(x, y, Color({ 0,0,0 }));
			}
		}
	}
	*/
	

	// YCbCr를 이용한 Skin Detection
	Mat YCbCrSkin;
	int Ymin = 0, Ymax = 255;
	int CbMin = 133, CbMax = 173;
	int CrMin = 77, CrMax = 127;

	// BGR -> YCrCb로 컬러 공간 변환
	cvtColor(this->img, YCbCrSkin, CV_BGR2YCrCb);

	inRange(YCbCrSkin, Scalar(Ymin, CbMin, CrMin), Scalar(Ymax, CbMax, CrMax), YCbCrSkin);
//	cv::imshow("YCbCr", YCbCrSkin);

//	erode(YCbCrSkin, YCbCrSkin, Mat());
//	GaussianBlur(YCbCrSkin, YCbCrSkin, Size(9, 9), 2.0, 2.0);
	medianBlur(YCbCrSkin, YCbCrSkin, 5);
	Mat elementYCbCr = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * elementSize + 1, 2 * elementSize + 1), cv::Point(elementSize, elementSize));

	dilate(YCbCrSkin, YCbCrSkin, elementYCbCr);
//	cv::imshow("YCbCrblur", YCbCrSkin);


	bitwise_and(HSVSkin, YCbCrSkin, this->SkinColorImg);
//	imshow("WithBlur", this->img);
	
//	erode(this->img, this->img, Mat());
//	dilate(this->img, this->img, Mat());
//	imshow("erode&dilate", this->img);


	
}

// 외곽선 Contours추출
void TransImage::Contours() {

	Mat contours_output = Mat::zeros(this->SkinColorImg.size(), CV_8UC3);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(this->SkinColorImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));



	size_t largestContour = 0;
	for (size_t i = 0; i < contours.size(); i++) {
		if (contourArea(contours[i]) > contourArea(contours[largestContour])) {
			largestContour = i;
		}
	}
	drawContours(this->img, contours, largestContour, Scalar(255, 255, 255), 1, 8);
	drawContours(contours_output, contours, largestContour, Scalar(255, 255, 255),1,8);

	imshow("11", contours_output);

	

	queue<int> countfinger;
	int arr[6] = { 0 };
	bool start = false;
	int fingertips = 0;
	// convex hull
	if (contours.size() > 0) {
		int innerMin = 20, innerMax = 120, angleMin = 0, angleMax = 179, lengthMin = 10, lengthMax = 80;
		vector<vector<Point>>hull(1);
		convexHull(Mat(contours[largestContour]), hull[0], false);
		drawContours(contours_output, hull, 0, Scalar(0, 255, 0), 3);
		if (hull[0].size() > 2) {
			vector<int> hullIndexes;
			convexHull(Mat(contours[largestContour]), hullIndexes, true);
			vector<Vec4i> defects;
			convexityDefects(Mat(contours[largestContour]), hullIndexes, defects);
			Rect boundingBox = boundingRect(hull[0]);
			rectangle(contours_output, boundingBox, Scalar(255, 0, 0));
			Point center = Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
			vector<Point>fingerTips;


			
			for (size_t i = 0; i < defects.size(); i++) {
				Point p1 = contours[largestContour][defects[i][0]];
				Point p2 = contours[largestContour][defects[i][1]];
				Point p3 = contours[largestContour][defects[i][2]];
				double angle = atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;
				double inner_Angle = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
				double length = sqrt(std::pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2));
				if (angle > angleMin && angle<angleMax && inner_Angle>innerMin
					&& inner_Angle<innerMax && length > lengthMin / 100.0 * boundingBox.height
					&& length < lengthMax / 100.0 * boundingBox.height) {
					fingerTips.push_back(p1);
				}
			}
			for (size_t i = 0; i < fingerTips.size(); i++) {
				circle(contours_output, fingerTips[i], 9, Scalar(0, 255, 0), 2);
			}
			
			if (fingerTips.size() < 6) {
				if (countfinger.size() < 50) {
					countfinger.push(fingerTips.size());
					arr[fingerTips.size()]++;
					cout << "??" << countfinger.size()<<" " << fingerTips.size() << endl;
				}
				else {
					if (start == false) {
						int k = 0;
						int max = arr[0];
						for (k = 0; k < 6; k++) {
							if (max < arr[k])
								max = arr[k];
						}
						if (max * 2 > 90) {
							if (k == 5) {
								start = true;
								cout << "Start" << endl;
								cout<<"finger : " << fingerTips.size() << endl;
								fingertips = fingerTips.size();
							}
						}
						cout << max*2 << endl;
						countfinger.pop();
					}
					else {
						int k = 0;
						int max = arr[0];
						for (k = 0; k < 6; k++) {
							if (max < arr[k])
								max = arr[k];
						}
						if (max * 2 > 90 ) {
							if(fingertips != k)
								cout << "finger : " << fingerTips.size() << endl;
						}
						countfinger.pop();
					}
				}
			}
		}
	}


	imshow("22", contours_output);




	
}

// 손의 가운데점 찾기
Point TransImage::HandCenter() {
	
	Mat distanceTrans;
	this->ContourImg.convertTo(this->ContourImg, CV_8UC3);
	cvtColor(this->ContourImg, this->ContourImg, CV_BGR2GRAY);
	distanceTransform(this->ContourImg, distanceTrans, CV_DIST_L2, 5);

	int max[2];

	minMaxIdx(distanceTrans, NULL, &(this->radius), NULL, max, this->ContourImg);
	this->center = Point(max[1], max[0]);
	return center;
}

// 손가락 숫자 세기
int TransImage::CountFinger(double ratio) {
	Mat CircleImg(this->ContourImg.size(), CV_8UC3, Scalar(0, 0, 0));
	circle(CircleImg, this->center, (int)(this->radius*ratio), Scalar(0, 0, 255), 2);
	cvtColor(CircleImg, CircleImg, CV_BGR2GRAY);
	imshow("Img", CircleImg);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(CircleImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

//	findContours(CircleImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int countFinger = 0;
	if (contours.size() != 0) {
		for (int i = 1; i < contours[0].size(); i++) {
			Point p1 = contours[0][i - 1];
			Point p2 = contours[0][i];
			if (this->ContourImg.at<uchar>(p1.y, p1.x) == 0 && this->ContourImg.at<uchar>(p2.y, p2.x) > 1)
				countFinger++;
		}
	}
	else {
		return -1;
	}
	return countFinger - 1;
}

// theta = arcos(x. y/ |x||y|)
float innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1) {
	float distance1 = sqrt((px1 - cx1)*(px1 - cx1) + (py1 - cy1)*(py1 - cy1));
	float distance2 = sqrt((px2 - cx1)*(px2 - cx1) + (py2 - cy1)*(py2 - cy1));

	float Ax, Ay;
	float Bx, By;
	float Cx, Cy;

	Cx = cx1;
	Cy = cy1;

	if (distance1 < distance2) {
		Bx = px1;
		By = py1;
		Ax = px2;
		Ay = py2;
	}
	else {
		Bx = px2;
		By = py2;
		Ax = px1;
		Ay = py1;
	}

	float Q1 = Cx - Ax;
	float Q2 = Cy - Ay;
	float P1 = Bx - Ay;
	float P2 = By - Ay;

	float A = acos((P1*Q1 + P2*Q2) / (sqrt(P1*P1 + P2*P2)*sqrt(Q1*Q1 + Q2*Q2)));
	A = A * 180 / CV_PI;

	return A;
}