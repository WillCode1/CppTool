#include "MathTool.h"
#include <cmath>

const double M_PI = 3.141592653589793284626433832795;


bool GeoAlgoTool::Equal(const double& a, const double& b, const double& dTol)
{
	return abs(a - b) < dTol;
}

bool GeoAlgoTool::LessEqualThan(const double& a, const double& b, const double& dTol)
{
	return a - b < dTol;
}

bool GeoAlgoTool::LessThan(const double& a, const double& b, const double& dTol)
{
	return LessEqualThan(a, b, dTol) && !Equal(a, b, dTol);
}

bool GeoAlgoTool::BiggerEqualThan(const double& a, const double& b, const double& dTol)
{
	return !LessThan(a, b, dTol);
}

bool GeoAlgoTool::BiggerThan(const double& a, const double& b, const double& dTol)
{
	return !LessEqualThan(a, b, dTol);
}

double GeoAlgoTool::Round(double decimals, int digits)
{
	if (!digits)
		return round(decimals);
	//return (decimals > 0.0) ? floor(decimals + 0.5) : ceil(decimals - 0.5);
	else if (digits > 0)
		return round(decimals * pow(10, digits)) / pow(10, digits);
	else
		return round(decimals / pow(10, -digits)) * pow(10, -digits);
}

double GeoAlgoTool::TransRadianToAngle(const double& dRadian)
{
	return (dRadian * 180 / M_PI);
}

double GeoAlgoTool::TransAngleToRadian(const double& dAngle)
{
	return (dAngle * M_PI / 180);
}

double GeoAlgoTool::NormalizeAngle(double & theta, double min, double max)
{
	if (theta >= min && theta < max)
		return theta;
	while (theta >= max)
		theta -= 2 * M_PI;
	while (theta < min)
		theta += 2 * M_PI;
	return theta;
}

double GeoAlgoTool::CalBulge(const double& dStartAngle, const double& dEndAngle)
{
	double dBulge = 0.0;
	double dAlfa = dEndAngle - dStartAngle;
	if (abs(dAlfa) < 1e-3)
	{
		dAlfa = 0.0;
	}
	//if (dAlfa < 0.0)//如果终点角度小于起点角度
	//{
	//	dAlfa = 2 * M_PI + dAlfa;
	//}
	dBulge = tan((dAlfa) / 4.0);

	return dBulge;
}

double GeoAlgoTool::TransBulgeToIncludeAngle(const double& dBulge)
{
	return atan(dBulge) * 4;
}
