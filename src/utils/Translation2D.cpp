#include <utils/Translation2D.h>
#include <cmath>
Translation2D::Translation2D()
	: m_x(0), m_y(0) {
}

Translation2D::Translation2D(double x, double y)
	: m_x(x), m_y(y) {
}

Translation2D::Translation2D(const Translation2D &other)
	: m_x(other.m_x), m_y(other.m_y) {
}

Translation2D::~Translation2D()
{
}

void Translation2D::setX(double x)
{
	m_x = x;
}

void Translation2D::setY(double y)
{
	m_y = y;
}

double Translation2D::getX() const
{
	return m_x;
}

double Translation2D::getY() const
{
	return m_y;
}

double Translation2D::norm() const
{
	return sqrt(m_x * m_x + m_y * m_y);
}

Translation2D Translation2D::inverse() const
{
	return Translation2D(-m_x, -m_y);
}

Translation2D Translation2D::translateBy(const Translation2D &other) const
{
	return Translation2D(m_x + other.m_x, m_y + other.m_y);
}

Translation2D Translation2D::rotateBy(const Rotation2D &rotation) const
{
	return Translation2D(m_x * rotation.getCos() - m_y * rotation.getSin(), m_y * rotation.getCos() + m_x * rotation.getSin());
}

Translation2D Translation2D::interpolate(const Translation2D &other, double x) const {
	if (x <= 0) {
		return Translation2D(*this);
	}
	else if (x >= 1) {
		return Translation2D(other);
	}
	return extrapolate(other, x);
}

Translation2D Translation2D::extrapolate(const Translation2D &other, double x) const {
	return Translation2D(x * (other.m_x - m_x) + m_x, x * (other.m_y - m_y) + m_y);
}
