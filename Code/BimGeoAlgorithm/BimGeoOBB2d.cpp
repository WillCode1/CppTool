#include "BimGeoOBB2d.h"

CBimGeoOBB2d::CBimGeoOBB2d()
{
	myHDims[0] = myHDims[1] = -1.0;
}

CBimGeoOBB2d::CBimGeoOBB2d(const gp_Pnt2d& theCenter, const gp_Dir2d& theXDirection, const gp_Dir2d& theYDirection, const double theHXSize, const double theHYSize)
{
	myCenter= theCenter.Coord();

	myAxes[0] = theXDirection;
	myAxes[1] = theYDirection;

	myHDims[0] = theHXSize;
	myHDims[1] = theHYSize;
}

void CBimGeoOBB2d::SetCenter(const gp_Pnt2d& theCenter)
{
	myCenter = theCenter.Coord();
}

void CBimGeoOBB2d::SetXComponent(const gp_Dir2d& theXDirection, const double theHXSize)
{
	myAxes[0] = theXDirection;
	myHDims[0] = theHXSize;
}

void CBimGeoOBB2d::SetYComponent(const gp_Dir2d& theYDirection, const double theHYSize)
{
	myAxes[1] = theYDirection;
	myHDims[1] = theHYSize;
}

const gp_Pnt2d& CBimGeoOBB2d::Center() const
{
	return myCenter;
}

const gp_Dir2d& CBimGeoOBB2d::XDirection() const
{
	return myAxes[0];
}

const gp_Dir2d& CBimGeoOBB2d::YDirection() const
{
	return myAxes[1];
}

const gp_Ax3 CBimGeoOBB2d::GetAx3() const
{
	gp_Dir xDir(XDirection().X(), XDirection().Y(), 0.);
	gp_Dir yDir(YDirection().X(), YDirection().Y(), 0.);
	gp_Dir normal = xDir.Crossed(yDir);
	return gp_Ax3(gp_Pnt(myCenter.X(), myCenter.Y(), 0.), normal, xDir);
}

/// @brief Returns the X Half-size Dimension of OBB
double CBimGeoOBB2d::XHSize() const
{
	return myHDims[0];
}

/// @brief Returns the Y Half-size Dimension of OBB
double CBimGeoOBB2d::YHSize() const
{
	return myHDims[1];
}

/// @brief Checks if the box is empty.
bool CBimGeoOBB2d::IsVoid() const
{
	return (myHDims[0] < 0.0) || (myHDims[1] < 0.0);
}

/// @brief Clears this box
void CBimGeoOBB2d::SetVoid()
{
	myHDims[0] = myHDims[1] = -1.0;
	myCenter = gp_Pnt2d();
	myAxes[0] = myAxes[1] = gp_Dir2d();
}

/// @brief Enlarges the box with the given value
/// @param theGapAdd 
void CBimGeoOBB2d::Enlarge(const double theGapAdd)
{
	const double aGap = Abs(theGapAdd);
	myHDims[0] += aGap;
	myHDims[1] += aGap;
}

/// @brief Returns square diagonal of this box
double CBimGeoOBB2d::SquareExtent() const
{
	return 4.0 * (myHDims[0] * myHDims[0] + myHDims[1] * myHDims[1]);
}
