#pragma once

#include <gp_Ax3.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_XYZ.hxx>
#include "BimGeoAlgorithmDef.h"

/// @brief The class describes the Oriented Bounding Box (OBB)
class EXPORTGEOALGOCLASS CBimGeoOBB2d
{
public:
	/// @brief Empty constructor
	/// @return 
	CBimGeoOBB2d();

	/// @brief Constructor taking all defining parameters
	CBimGeoOBB2d(const gp_Pnt2d& theCenter,
		const gp_Dir2d& theXDirection,
		const gp_Dir2d& theYDirection,
		const double theHXSize,
		const double theHYSize);

	/// @brief Sets the center of OBB
	/// @param theCenter 
	void SetCenter(const gp_Pnt2d& theCenter);

	/// @brief Sets the X component of OBB - direction and size
	/// @param theXDirection 
	/// @param theHXSize 
	void SetXComponent(const gp_Dir2d& theXDirection,
		const double theHXSize);

	/// @brief Sets the Y component of OBB - direction and size
	/// @param theYDirection 
	/// @param theHYSize 
	void SetYComponent(const gp_Dir2d& theYDirection, const double theHYSize);

	/// @brief Get Center
	/// @return 
	const gp_Pnt2d& Center() const;

	/// @brief Returns the X Direction of OBB
	/// @return 
	const gp_Dir2d& XDirection() const;

	/// @brief Returns the Y Direction of OBB
	/// @return 
	const gp_Dir2d& YDirection() const;

	/// @brief Returns ×ø±êÏµ
	const gp_Ax3 GetAx3() const;

	/// @brief Returns the X Half-size Dimension of OBB
	double XHSize() const;

	/// @brief Returns the Y Half-size Dimension of OBB
	double YHSize() const;

	/// @brief Checks if the box is empty.
	bool IsVoid() const;

	//! Clears this box
	void SetVoid();

	/// @brief Enlarges the box with the given value
	/// @param theGapAdd 
	void Enlarge(const double theGapAdd);

	/// @brief Returns the array of vertices in <this>.
	/// @brief The local coordinate of the vertex depending on the
	/// @brief index of the array are follow:
	/// @brief Index == 0: (-XHSize(), -YHSize(), 0)
	/// @brief Index == 1: ( XHSize(), -YHSize(), 0)
	/// @brief Index == 2: (-XHSize(),  YHSize(), 0)
	/// @brief Index == 3: ( XHSize(),  YHSize(), 0)
	//bool GetVertex(gp_Pnt2d theP[4]) const;

	//! Returns square diagonal of this box
	double SquareExtent() const;

private:

	/// @brief Center of the OBB
	gp_Pnt2d myCenter;

	/// @brief Directions of the box's axes (all vectors are already normalized)
	gp_Dir2d myAxes[2];

	/// @brief Half-size dimensions of the OBB
	double myHDims[2];
};

