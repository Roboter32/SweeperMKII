#ifndef NavVector_h
#define NavVector_h

class NavVector
{
public:
	
	NavVector(float X, float Y);
	NavVector(float DirX, float DirY, float Length);
	float x, y;

	void Rescale(float Scale);
	float GetLength();
	void SetLength(float NewLength);
	void ScaleToX(float NewX);
	void ScaleToY(float NewY);
	void TurnLeft();
	void TurnRight();
	void Reverse();
private:

};

#endif
