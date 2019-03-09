#pragma once
#include "utils/Rotation2D.h"
#include "utils/Interpolable.h"

class Translation2D : public Interpolable<Translation2D>
{
public:
	Translation2D();
	Translation2D(double x, double y);
	Translation2D(const Translation2D &other);
	~Translation2D();
	
	void setX(double x);
	void setY(double y);
	
	double getX()const;
	double getY()const;

	double norm() const;
	Translation2D inverse() const;

	Translation2D translateBy(const Translation2D &other) const;
	Translation2D rotateBy(const Rotation2D &rotation)const;

	virtual Translation2D interpolate(const Translation2D &other, double x)const;
	Translation2D extrapolate(const Translation2D &other, double x)const;

private:
	double m_x;
	double m_y;
};
