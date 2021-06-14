#pragma once

#include "BimGeoAlgorithmDef.h"
#include "gp_Pnt.hxx"
#include "Bnd_Box.hxx"
#include <vector>
#include <list>

namespace BimGeo3DAlgorithmTool
{
	/// @brief 线与平面的交点
	EXPORTGEOALGOAPI void CalcIntersectCurveWithSurface(std::vector<gp_Pnt>& vecPt,
		const gp_Pnt& ptS, const gp_Pnt& ptE, const gp_Pnt& ptLocation, const gp_Dir& dir);

	/// @brief 点到面的垂足
	EXPORTGEOALGOAPI gp_Pnt CalcProjectOnSurface(const gp_Pnt& pt, const gp_Pnt& ptLocation, const gp_Dir& dir);
};

