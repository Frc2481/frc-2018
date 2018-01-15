#pragma once
#include "utils/Interpolable.h"

//#define M_PI 3.14159265359

class Rotation2D : public Interpolable<Rotation2D>
{
public:
	Rotation2D();
	Rotation2D(double x, double y, bool normalize);
	Rotation2D(const Rotation2D &other);
	~Rotation2D();

	Rotation2D& operator=(Rotation2D arg);

	static Rotation2D fromRadians(double angle_radians);
	static Rotation2D fromDegrees(double angle_degrees);
	Rotation2D rotateBy(const Rotation2D &other)const;
	Rotation2D inverse()const;

	void normalize();

	void setCos(double x);
	void setSin(double y);

	double getCos() const;
	double getSin() const;
	double getTan() const;
	double getRadians() const;
	double getDegrees() const;
		
	virtual Rotation2D interpolate(const Rotation2D &other, double x)const;
	Rotation2D extrapolate(const Rotation2D &other, double x)const;


private:
	double m_cos;
	double m_sin;
	const double kEpsilon = 1E-9;
};
