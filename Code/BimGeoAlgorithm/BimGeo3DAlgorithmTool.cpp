#include "BimGeo3DAlgorithmTool.h"
#include "Geom_TrimmedCurve.hxx"
#include "GC_MakeSegment.hxx"
#include "GC_MakePlane.hxx"
#include "GeomAPI_IntCS.hxx"
#include "GeomAPI_ProjectPointOnSurf.hxx"

void BimGeo3DAlgorithmTool::CalcIntersectCurveWithSurface(std::vector<gp_Pnt>& vecPt, 
	const gp_Pnt& ptS, const gp_Pnt& ptE, const gp_Pnt& ptLocation, const gp_Dir& dir)
{
	GC_MakePlane plane(ptLocation, dir);
	GC_MakeSegment lineSeg(ptS, ptE);
	GeomAPI_IntCS Intersector(lineSeg.Value(), plane.Value());

	for (int i = 1; i <= Intersector.NbPoints(); ++i)
	{
		vecPt.emplace_back(Intersector.Point(i));
	}
}

gp_Pnt BimGeo3DAlgorithmTool::CalcProjectOnSurface(const gp_Pnt& pt, 
	const gp_Pnt& ptLocation, const gp_Dir& dir)
{
	GC_MakePlane plane(ptLocation, dir);
	GeomAPI_ProjectPointOnSurf Proj(pt, plane.Value());

	if (Proj.NbPoints() < 1)
	{
		return pt;
	}

	return Proj.Point(1);
}
