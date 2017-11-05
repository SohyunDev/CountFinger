#include "Color.h"

Color::Color(Vec3b color)
{
	this->blue = color[0];
	this->green = color[1];
	this->red = color[2];
}

Color::Color()
{
	this->blue = 255;
	this->green = 255;
	this->red = 255;
}


Color::~Color()
{
}

uchar Color::getRedColor()
{
	return this->red;
}

uchar Color::getBlueColor()
{
	return this->blue;
}

uchar Color::getGreenColor()
{
	return this->green;
}

void Color::setRedColor(uchar r) {
	this->red = r;
}

void Color::setBlueColor(uchar b) {
	this->blue = b;
}

void Color::setGreenColor(uchar g) {
	this->green = g;
}
