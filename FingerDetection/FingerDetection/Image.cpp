#include "Image.h"

Image::Image() {

}

Image::Image(int width, int height) {
	Mat image(height, width, CV_8UC3, Scalar(0, 0, 0));

	this->img = image;
	this->width = width;
	this->height = height;
}

Image::~Image() {

}

int Image::getWidth() {
	return this->width;
}

int Image::getHeight() {
	return this->height;
}

Color Image::getColorAtIndex(int x, int y) {
	if (x >= width || y >= height) {
		return Color();
	}
	else {
		Color color(this->img.at<Vec3b>(Point(x, y)));
		return color;
	}
}

void Image::setColorAtIndex(int x, int y, Color newColor) {
	Vec3b color = this->img.at<Vec3b>(Point(x, y));
	color[0] = newColor.getBlueColor();
	color[1] = newColor.getGreenColor();
	color[2] = newColor.getRedColor();
	this->img.at<Vec3b>(Point(x, y)) = color;
}

void Image::printLogDetection()
{
	std::cout << "=====================================================" << std::endl << std::endl;
	std::cout << "WIDTH = " << width << "   HEIGHT = " << height << std::endl << std::endl;;
	for (int y = 0; y < this->height; y++) {
		for (int x = 0; x < this->width; x++) {
			if (this->getColorAtIndex(x, y).getBlueColor() > 70) {
				std::cout << " ";
			}
			else {
				std::cout << "x";
			}
		}
		std::cout << std::endl;
	}
	std::cout << "=====================================================" << std::endl;
}

void Image::setPicture(Mat image) {
	this->img = image;
}

Mat Image::getPicture() {
	return this->img;
}