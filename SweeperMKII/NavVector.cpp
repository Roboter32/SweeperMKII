#include "NavVector.h"
#include <math.h>



NavVector::NavVector(float X, float Y)
{
	x = X;
	y = Y;
}

NavVector::NavVector(float DirX, float DirY, float Length)
{
	x = DirX;
	y = DirY;
	Rescale(Length);
}

void NavVector::Rescale(float Scale)
{
	x *= Scale;
	y *= Scale;
}

float NavVector::GetLength()
{
	return sqrtf((x * x) + (y * y));
}

void NavVector::SetLength(float Length)
{
	Rescale(Length / GetLength());
}

void NavVector::ScaleToX(float NewX)
{
	Rescale(NewX / x);
}

void NavVector::ScaleToY(float NewY)
{
	Rescale(NewY / y);
}

void NavVector::Reverse()
{
	x *= -1;
	y *= -1;
}

void NavVector::TurnLeft()
{
	float buff = x;
	x = y;
	y = buff;
	x *= -1;
}

void NavVector::TurnRight()
{
	float buff = x;
	x = y;
	y = buff;
	y *= -1;
}
