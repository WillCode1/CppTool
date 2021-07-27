#pragma once
struct TPose2D
{
	double  x;
	double  y;
	double  phi;

	TPose2D(const double& px = 0, const double& py = 0, const double& pp = 0)
	{
		x = px;
		y = py;
		phi = pp;
	}

	TPose2D& operator= (const double& value)
	{
		x = y = phi = value;
		return *this;
	}
	bool operator== (const TPose2D& p3f) const
	{
		return x == p3f.x && y == p3f.y && phi == p3f.phi;
	}
	TPose2D operator+ (const TPose2D& p3f) const
	{
		return TPose2D(x + p3f.x, y + p3f.y, phi + p3f.phi);
	}
	TPose2D operator- (const TPose2D& p3f) const
	{
		return TPose2D(x - p3f.x, y - p3f.y, phi - p3f.phi);
	}
	TPose2D operator* (const double& scale) const
	{
		return TPose2D(x * scale, y * scale, phi * scale);
	}
};
