#pragma once
#include <utils/Rotation2D.h>
#include <utils/Translation2D.h>

#include "utils/Interpolable.h"

class RigidTransform2D : public Interpolable <RigidTransform2D>
{
public:
	class Delta {
	public:
		double m_dx;
		double m_dy;
		double m_dtheta;
		Delta(double dx, double dy, double dtheta);
	};
	RigidTransform2D();
	RigidTransform2D(const Translation2D &translation, const Rotation2D &rotation);
	RigidTransform2D(const RigidTransform2D &other);

	~RigidTransform2D();

	static RigidTransform2D fromTranslation(const Translation2D &translation);
	static RigidTransform2D fromRotation(const Rotation2D &rotation);
	static RigidTransform2D fromVelocity(Delta delta);

	RigidTransform2D transformBy(const RigidTransform2D &other);
	Translation2D getTranslation()const;
	Rotation2D getRotation()const;

	void setTranslation(const Translation2D &translation);
	void setRotation(const Rotation2D &rotation);

	RigidTransform2D inverse();

	virtual RigidTransform2D interpolate(const RigidTransform2D &other, double x) const ;

private:
	Translation2D m_translation;
	Rotation2D m_rotation;
};

