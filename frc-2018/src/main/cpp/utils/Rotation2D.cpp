#include <utils/Rotation2D.h>
#include <cmath>
#include <math.h>
#include <limits>

Rotation2D::Rotation2D() : m_cos(1), m_sin(0)
{}

Rotation2D::Rotation2D(double x, double y, bool normalized)
	: m_cos(x), m_sin(y) {
	if (normalized) {
		normalize();
	}
}

Rotation2D::Rotation2D(const Rotation2D &other)
	: m_cos(other.m_cos), m_sin(other.m_sin) {
}

Rotation2D::~Rotation2D()
{
}

Rotation2D& Rotation2D::operator=(Rotation2D other) {
	m_cos = other.m_cos;
	m_sin = other.m_sin;
	return *this;
}

Rotation2D Rotation2D::fromRadians(double angle_radians) {
	return Rotation2D(cos(angle_radians), sin(angle_radians), false);
}

Rotation2D Rotation2D::fromDegrees(double angle_degrees) {
	return fromRadians(angle_degrees * (M_PI / 180));
}

void Rotation2D::normalize() {
	double magnitude = hypot(m_cos, m_sin);
	if (magnitude > kEpsilon) {
		m_sin /= magnitude;
		m_cos /= magnitude;
	}
	else {
		m_sin = 0;
		m_cos = 1;
	}
}

void Rotation2D::setCos(double x) {
	m_cos = x;
}

void Rotation2D::setSin(double y) {
	m_sin = y;
}

double Rotation2D::getCos() const{
	return m_cos;
}

double Rotation2D::getSin() const{
	return m_sin;
}

double Rotation2D::getTan() const {
	if (fabs(m_cos) < kEpsilon) {
		if (fabs(m_sin) >= 0.0) {
			return std::numeric_limits<double>::infinity();
		}
		else {
			return -std::numeric_limits<double>::infinity();
		}
	}
	return m_sin / m_cos;
}

double Rotation2D::getRadians() const{
	return atan2(m_sin, m_cos);
}

double Rotation2D::getDegrees() const{
	return getRadians() * 180 / M_PI;
}

Rotation2D Rotation2D::rotateBy(const Rotation2D &other)const {
	return Rotation2D(m_cos * other.m_cos - m_sin * other.m_sin,
		m_cos * other.m_sin + m_sin * other.m_cos, true);
}

Rotation2D Rotation2D::inverse()const {
	return Rotation2D(m_cos, -m_sin, false);
}

Rotation2D Rotation2D::interpolate(const Rotation2D &other, double x)const
{
	if (x <= 0) {
		return Rotation2D(*this);
	}
	else if (x >= 1) {
		return Rotation2D(other);
	}
	double angle_diff = inverse().rotateBy(other).getRadians();
	return this->rotateBy(fromRadians(angle_diff * x));
}
