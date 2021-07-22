#include "BimGeoAlgorithmTool.h"
#include <algorithm>
#include <math.h>
#include <map>
#include "gp_Vec.hxx"
#include "gp_Ax1.hxx"
#include "gce_MakeCirc.hxx"
#include "GC_MakeSegment.hxx"
#include "GC_MakeCircle.hxx"
#include "GC_MakeArcOfCircle.hxx"
#include "GC_MakeLine.hxx"
#include "Geom_TrimmedCurve.hxx"
#include "GeomAPI_ProjectPointOnCurve.hxx"
#include "Geom2dAPI_ProjectPointOnCurve.hxx"
#include "Geom_TrimmedCurve.hxx"
#include "GCE2d_MakeSegment.hxx"
#include "GCE2d_MakeCircle.hxx"
#include "GCE2d_MakeArcOfCircle.hxx"
#include "GCE2d_MakeLine.hxx"
#include "Geom2d_TrimmedCurve.hxx"
#include "Geom2dAPI_InterCurveCurve.hxx"
#include "BimGeoPolylineOffset.h"
#include "BimSearchPolygonTool.h"
#include "QuadTreeNode.h"

bool BimGeoAlgorithmTool::equal(const double& a, const double& b, const double& dTol)
{
	if (abs(a - b) < dTol)
	{
		return true;
	}

	return false;
}

bool BimGeoAlgorithmTool::lessThan(const double& a, const double& b, const double& dTol)
{
	if (equal(a, b, dTol))
	{
		return false;
	}

	if (a - b < dTol)
	{
		return true;
	}

	return false;
}

bool BimGeoAlgorithmTool::lessEqualThan(const double& a, const double& b, const double& dTol)
{
	if (equal(a, b, dTol))
	{
		return true;
	}

	if (lessThan(a, b, dTol))
	{
		return true;
	}

	return false;
}

bool BimGeoAlgorithmTool::biggerThan(const double& a, const double& b, const double& dTol)
{
	return !lessEqualThan(a, b, dTol);
}

bool BimGeoAlgorithmTool::biggerEqualThan(const double& a, const double& b, const double& dTol)
{
	return !lessThan(a, b, dTol);
}

double BimGeoAlgorithmTool::round(double r)
{
	return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

bool BimGeoAlgorithmTool::IsArc(const double& dBulge, const double& dTol)
{
	if (abs(dBulge) <= dTol)
	{
		return false;
	}

	return true;
}

gp_Pnt BimGeoAlgorithmTool::GetCurveMidPoint(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge)
{
	if (IsArc(dBulge))
	{
		gp_Vec vec(ptStart, ptEnd);
		if (equal(vec.Magnitude(), 0.))
		{
			return ptStart;
		}

		double dDist = vec.Magnitude() * 0.5 * dBulge;
		vec.Normalize();

		// 顺时针旋转90°
		vec.Rotate(gp_Ax1(gp_Pnt(0., 0., 0.), gp_Dir(0., 0., -1.)), M_PI_2);
		vec *= dDist;

		gp_Pnt ptMid((ptStart.X() + ptEnd.X()) * 0.5 + vec.X(),
			(ptStart.Y() + ptEnd.Y()) * 0.5 + vec.Y(),
			(ptStart.Z() + ptEnd.Z()) * 0.5 + vec.Z());

		return ptMid;
	}

	gp_Pnt ptMid((ptStart.Coord() + ptEnd.Coord()) * 0.5);
	return ptMid;
}

gp_Pnt2d BimGeoAlgorithmTool::GetCurveMidPoint(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge)
{
	gp_Pnt ptMid = GetCurveMidPoint(gp_Pnt(ptStart.X(), ptStart.Y(), 0.), gp_Pnt(ptEnd.X(), ptEnd.Y(), 0.), dBulge);
	return gp_Pnt2d(ptMid.X(), ptMid.Y());
}

gp_Pnt2d BimGeoAlgorithmTool::GetArcMidPoint(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd,
	const gp_Pnt2d& ptCenter, const double& dRadius,
	const bool IsAnticlockwise, const double& dTol)
{
	if (ptStart.IsEqual(ptEnd, dTol))
	{
		return ptStart;
	}

	gp_Vec2d vec(ptStart, ptEnd);
	vec.Normalize();
	gp_Vec2d vecPerp(-vec.Y(), vec.X());
	if (IsAnticlockwise)
	{
		vecPerp *= -1;
	}

	gp_Pnt2d ptMid = ptCenter.Coord() + vecPerp.XY() * dRadius;

	return ptMid;
}

gp_Pnt2d BimGeoAlgorithmTool::GetArcMidPoint(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd,
	const gp_Pnt2d& ptRefStart, const gp_Pnt2d& ptRefEnd, const double& dRefBulge, const double& dTol)
{
	gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(ptRefStart, ptRefEnd, dRefBulge);
	GCE2d_MakeCircle arc(ptRefStart, ptMid, ptRefEnd);

	return GetArcMidPoint(ptStart, ptEnd, arc.Value()->Location(), arc.Value()->Radius(),
		arc.Value()->Circ2d().IsDirect(), dTol);
}

gp_Pnt BimGeoAlgorithmTool::GetArcMidPoint(const gp_Pnt& ptStart, const gp_Pnt& ptEnd,
	const gp_Pnt& ptRefStart, const gp_Pnt& ptRefEnd, const double& dRefBulge, const double& dTol)
{
	gp_Pnt2d ptMid = GetArcMidPoint(gp_Pnt2d(ptStart.X(), ptStart.Y()), gp_Pnt2d(ptEnd.X(), ptEnd.Y()),
		gp_Pnt2d(ptRefStart.X(), ptRefStart.Y()), gp_Pnt2d(ptRefEnd.X(), ptRefEnd.Y()), dRefBulge, dTol);

	return gp_Pnt(ptMid.X(), ptMid.Y(), 0.);
}

SGeoArcInfo BimGeoAlgorithmTool::GetArcInfo(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge)
{
	return GetArcInfo(gp_Pnt2d(ptStart.X(), ptStart.Y()), gp_Pnt2d(ptEnd.X(), ptEnd.Y()), dBulge);
}

SGeoArcInfo BimGeoAlgorithmTool::GetArcInfo(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge)
{
	if (ptStart.IsEqual(ptEnd, 0.0001))
	{

		return SGeoArcInfo();
	}

	gp_Pnt2d ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
	GCE2d_MakeCircle arc(ptStart, ptMid, ptEnd);

	SGeoArcInfo info;
	info.ptCenter = arc.Value()->Location();
	info.dRadius = arc.Value()->Radius();
	info.bIsAntiClockwise = arc.Value()->Circ2d().IsDirect();

	return info;
}

bool BimGeoAlgorithmTool::IsOn(const gp_Pnt& pt, const gp_Pnt& ptStart, const gp_Pnt& ptEnd,
	const double& dBulge, const double& dTol)
{
	if (ptStart.IsEqual(ptEnd, dTol))
	{
		return false;
	}

	if (ptStart.IsEqual(pt, dTol)
		|| ptEnd.IsEqual(pt, dTol))
	{
		return true;
	}

	gp_Pnt ptClose = GetClosePoint(pt, ptStart, ptEnd, dBulge, dTol);
	if (equal(pt.Distance(ptClose), 0., dTol))
	{
		return true;
	}

	return false;
}

bool BimGeoAlgorithmTool::IsOn(const gp_Pnt2d& pt, const gp_Pnt2d& ptStart,
	const gp_Pnt2d& ptEnd, const double& dBulge, const double& dTol)
{
	return IsOn(gp_Pnt(pt.X(), pt.Y(), 0.), gp_Pnt(ptStart.X(), ptStart.Y(), 0.),
		gp_Pnt(ptEnd.X(), ptEnd.Y(), 0.), dBulge, dTol);
}

bool BimGeoAlgorithmTool::GetProjectPt(gp_Pnt& ptProject, const gp_Pnt& pt, const gp_Pnt& ptStart,
	const gp_Pnt& ptEnd, const double& dBulge, const double& dTol)
{
	if (equal(dBulge, 0.))
	{// 直线
		if (ptStart.IsEqual(ptEnd, dTol))
		{
			return false;
		}

		GC_MakeLine line(ptStart, gp_Dir(gp_Vec(ptStart, ptEnd).XYZ()));
		GeomAPI_ProjectPointOnCurve Projector(pt, line.Value());
		if (Projector.NbPoints() < 1)
		{
			return false;
		}

		ptProject = Projector.NearestPoint();
	}
	else
	{// 弧线
		gp_Pnt ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
		GC_MakeCircle arc(ptStart, ptMid, ptEnd);
		if (pt.IsEqual(arc.Value()->Location(), dTol))
		{// 圆心
			ptProject = ptMid;
			return true;
		}

		GeomAPI_ProjectPointOnCurve Projector(pt, arc.Value());
		if (Projector.NbPoints() < 1)
		{
			return false;
		}

		ptProject = Projector.NearestPoint();
	}

	return true;
}

bool BimGeoAlgorithmTool::GetProjectPt(gp_Pnt2d& ptProject, const gp_Pnt2d& pt, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd,
	const double& dBulge, const double& dTol)
{
	gp_Pnt ptProject3d;
	bool bRet = GetProjectPt(ptProject3d, gp_Pnt(pt.X(), pt.Y(), 0.), gp_Pnt(ptStart.X(), ptStart.Y(), 0.),
		gp_Pnt(ptEnd.X(), ptEnd.Y(), 0.), dBulge, dTol);

	if (bRet)
	{
		ptProject.SetCoord(ptProject3d.X(), ptProject3d.Y());
	}

	return bRet;
}

gp_Pnt BimGeoAlgorithmTool::GetClosePoint(const gp_Pnt& pt, const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge, const double& dTol)
{
	if (ptStart.IsEqual(ptEnd, dTol))
	{
		return ptStart;
	}

	if (equal(dBulge, 0.))
	{// 直线
		GC_MakeSegment lineSeg(ptStart, ptEnd);
		if (lineSeg.Value().IsNull())
		{
			return ptStart;
		}

		GeomAPI_ProjectPointOnCurve Projector(pt, lineSeg.Value());
		if (Projector.NbPoints() < 1)
		{
			if (lessThan(pt.Distance(ptStart), pt.Distance(ptEnd)))
			{
				return ptStart;
			}
			else
			{
				return ptEnd;
			}
		}

		return Projector.NearestPoint();
	}
	else
	{// 弧线
		gp_Pnt ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
		GC_MakeArcOfCircle arc(ptStart, ptMid, ptEnd);
		if (arc.Value().IsNull())
		{
			return ptStart;
		}

		GeomAPI_ProjectPointOnCurve Projector(pt, arc.Value());
		if (Projector.NbPoints() < 1)
		{
			GC_MakeCircle arc(ptStart, ptMid, ptEnd);
			if (pt.IsEqual(arc.Value()->Location(), dTol))
			{// 圆心
				return ptMid;
			}

			if (lessThan(pt.Distance(ptStart), pt.Distance(ptEnd)))
			{
				return ptStart;
			}
			else
			{
				return ptEnd;
			}
		}

		int num = Projector.NbPoints();
		gp_Pnt ptClose = Projector.NearestPoint();
		double dMinDist = pt.Distance(ptClose);
		{
			double dDist = pt.Distance(ptStart);
			if (dDist < dMinDist)
			{
				ptClose = ptStart;
				dMinDist = dDist;
			}

			dDist = pt.Distance(ptEnd);
			if (dDist < dMinDist)
			{
				ptClose = ptEnd;
				dMinDist = dDist;
			}
		}

		return ptClose;
	}

	return gp_Pnt(0., 0., 0.);
}

gp_Pnt2d BimGeoAlgorithmTool::GetClosePoint(const gp_Pnt2d& pt, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge, const double& dTol)
{
	gp_Pnt ptClose = GetClosePoint(gp_Pnt(pt.X(), pt.Y(), 0.), gp_Pnt(ptStart.X(), ptStart.Y(), 0.),
		gp_Pnt(ptEnd.X(), ptEnd.Y(), 0.), dBulge, dTol);

	return gp_Pnt2d(ptClose.X(), ptClose.Y());
}

double BimGeoAlgorithmTool::GetParamOnCurve(const gp_Pnt& pt, const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge, const double& dTol)
{
	if (equal(dBulge, 0.))
	{// 直线
		GC_MakeSegment lineSeg(ptStart, ptEnd);
		GeomAPI_ProjectPointOnCurve Projector(pt, lineSeg.Value());
		if (Projector.NbPoints() < 1)
		{
			return -1.;
		}

		return Projector.Parameter(1);
	}
	else
	{// 弧线
		gp_Pnt ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
		GC_MakeCircle arc(ptStart, ptMid, ptEnd);
		GeomAPI_ProjectPointOnCurve Projector(pt, arc.Value());
		if (Projector.NbPoints() < 1)
		{
			return -1.;
		}

		return Projector.Parameter(1);
	}

	return -1.;
}

EXPORTGEOALGOAPI double BimGeoAlgorithmTool::GetParamOnCurve(const gp_Pnt2d & pt, const gp_Pnt2d & ptStart, const gp_Pnt2d & ptEnd, const double & dBulge, const double & dTol)
{
	return GetParamOnCurve(gp_Pnt(pt.X(), pt.Y(), 0.), gp_Pnt(ptStart.X(), ptStart.Y(), 0.),
		gp_Pnt(ptEnd.X(), ptEnd.Y(), 0.), dBulge, dTol);
}

gp_Pnt BimGeoAlgorithmTool::GetPtOnCurveByParam(const double& param, const gp_Pnt & ptStart, const gp_Pnt & ptEnd, const double & dBulge, const double & dTol)
{
	gp_Pnt pt(0., 0., 0.);
	if (equal(dBulge, 0.))
	{// 直线
		GC_MakeSegment lineSeg(ptStart, ptEnd);
		lineSeg.Value()->D0(param, pt);
	}
	else
	{// 弧线
		gp_Pnt ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
		GC_MakeCircle arc(ptStart, ptMid, ptEnd);
		arc.Value()->D0(param, pt);
	}

	return pt;
}

EXPORTGEOALGOAPI gp_Pnt2d BimGeoAlgorithmTool::GetPtOnCurveByParam(const double & param, const gp_Pnt2d & ptStart, const gp_Pnt2d & ptEnd, const double & dBulge, const double & dTol)
{
	gp_Pnt ptOnCurve = GetPtOnCurveByParam(param, gp_Pnt(ptStart.X(), ptStart.Y(), 0.),
		gp_Pnt(ptEnd.X(), ptEnd.Y(), 0.), dBulge, dTol);

	return gp_Pnt2d(ptOnCurve.X(), ptOnCurve.Y());
}

void BimGeoAlgorithmTool::CalcIntersect(std::vector<gp_Pnt>& vecIntersectPt, const gp_Pnt& ptS, const gp_Pnt& ptE,
	const double& dBulge, const gp_Pnt& ptSRef, const gp_Pnt& ptERef,
	const double& dBulgeRef, const double& dTol)
{
	std::vector<gp_Pnt2d> vecIntersectPt2d;
	CalcIntersect(vecIntersectPt2d, gp_Pnt2d(ptS.X(), ptS.Y()), gp_Pnt2d(ptE.X(), ptE.Y()), dBulge,
		gp_Pnt2d(ptSRef.X(), ptSRef.Y()), gp_Pnt2d(ptERef.X(), ptERef.Y()), dBulgeRef, dTol);

	for (int i = 0; i < vecIntersectPt2d.size(); ++i)
	{
		vecIntersectPt.emplace_back(gp_Pnt(vecIntersectPt2d[i].X(), vecIntersectPt2d[i].Y(), 0.));
	}
}

void BimGeoAlgorithmTool::CalcIntersect(std::vector<gp_Pnt2d>& vecIntersectPt, const gp_Pnt2d& ptS, const gp_Pnt2d& ptE,
	const double& dBulge, const gp_Pnt2d& ptSRef, const gp_Pnt2d& ptERef, const double& dBulgeRef, const double& dTol)
{
	if (ptS.IsEqual(ptE, dTol)
		|| ptSRef.IsEqual(ptERef, dTol))
	{
		return;
	}

	Handle(Geom2d_Curve) curve1;
	if (equal(dBulge, 0.))
	{// 线段
		GCE2d_MakeSegment lineSeg(ptS, ptE);
		curve1 = lineSeg.Value();
	}
	else
	{// 弧线
		gp_Pnt2d ptMid = GetCurveMidPoint(ptS, ptE, dBulge);
		GCE2d_MakeArcOfCircle arc(ptS, ptMid, ptE);

		curve1 = arc.Value();
	}

	Handle(Geom2d_Curve) curve2;
	if (equal(dBulgeRef, 0.))
	{// 线段
		GCE2d_MakeSegment lineSeg(ptSRef, ptERef);
		curve2 = lineSeg.Value();
	}
	else
	{// 弧线
		gp_Pnt2d ptMid = GetCurveMidPoint(ptSRef, ptERef, dBulgeRef);
		GCE2d_MakeArcOfCircle arc(ptSRef, ptMid, ptERef);

		curve2 = arc.Value();
	}

	if (curve1.IsNull() || curve2.IsNull())
	{
		return;
	}

	Geom2dAPI_InterCurveCurve Intersector(curve1, curve2, dTol);
	if (0 < Intersector.NbPoints())
	{
		for (int i = 1; i <= Intersector.NbPoints(); ++i)
		{
			vecIntersectPt.emplace_back(Intersector.Point(i));
		}
	}
	else if (0 < Intersector.NbSegments())
	{
		double dFirstParam = curve1->FirstParameter();
		double dLastParam = curve1->LastParameter();

		int len = Intersector.NbSegments();
		for (int i = 1; i <= Intersector.NbSegments(); ++i)
		{
			Handle(Geom2d_Curve) retCurve1, retCurve2;
			try
			{
				Intersector.Segment(i, retCurve1, retCurve2);
			}
			catch (...)
			{
				continue;
			}

			if (retCurve1.IsNull())
			{
				continue;
			}

			if (BimGeoAlgorithmTool::lessThan(dFirstParam, retCurve1->FirstParameter())
				&& BimGeoAlgorithmTool::lessThan(retCurve1->FirstParameter(), dLastParam))
			{
				gp_Pnt2d pt = curve1->Value(retCurve1->FirstParameter());
				vecIntersectPt.emplace_back(pt);
			}

			if (BimGeoAlgorithmTool::lessThan(dFirstParam, retCurve1->LastParameter())
				&& BimGeoAlgorithmTool::lessThan(retCurve1->LastParameter(), dLastParam))
			{
				gp_Pnt2d pt = curve1->Value(retCurve1->LastParameter());
				vecIntersectPt.emplace_back(pt);
			}
		}
	}


	{// 可能是平行
		if (BimGeoAlgorithmTool::IsOn(ptSRef, ptS, ptE, dBulge, dTol))
		{
			vecIntersectPt.emplace_back(ptSRef);
		}

		if (BimGeoAlgorithmTool::IsOn(ptERef, ptS, ptE, dBulge, dTol))
		{
			vecIntersectPt.emplace_back(ptERef);
		}

		if (!(ptSRef.IsEqual(ptS, dTol) || ptERef.IsEqual(ptS, dTol))
			&& BimGeoAlgorithmTool::IsOn(ptS, ptSRef, ptERef, dBulgeRef, dTol))
		{
			vecIntersectPt.emplace_back(ptS);
		}

		if (!(ptSRef.IsEqual(ptE, dTol) || ptERef.IsEqual(ptE, dTol))
			&& BimGeoAlgorithmTool::IsOn(ptE, ptSRef, ptERef, dBulgeRef, dTol))
		{
			vecIntersectPt.emplace_back(ptE);
		}
	}

	vecIntersectPt = SortByCurve(ptS, ptE, dBulge, vecIntersectPt, false);
}

void BimGeoAlgorithmTool::CalcLineIntersect(std::vector<gp_Pnt2d>& vecIntersectPt, const gp_Pnt2d& ptS,
	const gp_Dir2d& dir, const gp_Pnt2d& ptRefS, const gp_Dir2d& dirRef, const double& dTol)
{
	GCE2d_MakeLine line1(ptS, dir), line2(ptRefS, dirRef);
	Geom2dAPI_InterCurveCurve Intersector(line1.Value(), line2.Value(), dTol);
	int len = Intersector.NbPoints();
	for (int i = 1; i <= len; ++i)
	{
		gp_Pnt2d pt = Intersector.Point(i);
		vecIntersectPt.emplace_back(pt);
	}
}

void BimGeoAlgorithmTool::CalcIntersectWithExtend(std::vector<gp_Pnt2d>& vecIntersectPt, const gp_Pnt2d& ptS, const gp_Pnt2d& ptE,
	const double& dBulge, const gp_Pnt2d& ptSRef, const gp_Pnt2d& ptERef, const double& dBulgeRef, const double& dTol)
{
	Handle(Geom2d_Curve) curve1;
	if (equal(dBulge, 0.))
	{// 直线
		GCE2d_MakeLine line(ptS, ptE);
		curve1 = line.Value();
	}
	else
	{// 弧线
		gp_Pnt2d ptMid = GetCurveMidPoint(ptS, ptE, dBulge);
		GCE2d_MakeCircle arc(ptS, ptMid, ptE);
		curve1 = arc.Value();
	}

	Handle(Geom2d_Curve) curve2;
	if (equal(dBulgeRef, 0.))
	{// 直线
		GCE2d_MakeLine line(ptSRef, ptERef);
		curve2 = line.Value();
	}
	else
	{// 弧线
		gp_Pnt2d ptMid = GetCurveMidPoint(ptSRef, ptERef, dBulgeRef);
		GCE2d_MakeCircle arc(ptSRef, ptMid, ptERef);
		curve2 = arc.Value();
	}

	if (curve1.IsNull() || curve2.IsNull())
	{
		return;
	}

	Geom2dAPI_InterCurveCurve Intersector(curve1, curve2, dTol);
	int len = Intersector.NbPoints();
	for (int i = 1; i <= len; ++i)
	{
		vecIntersectPt.emplace_back(Intersector.Point(i));
	}
}

bool BimGeoAlgorithmTool::IsParallel(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge,
	const gp_Pnt& ptRefStart, const gp_Pnt& ptRefEnd, const double& dRefBulge, const double& dTol)
{
	if ((ptStart.IsEqual(ptEnd, dTol) && equal(dBulge, 0.))
		|| (ptRefStart.IsEqual(ptRefEnd, dTol) && equal(dRefBulge, 0.)))
	{
		return true;
	}

	if (equal(dBulge, 0.) && equal(dRefBulge, 0.))
	{// 直线
		gp_Vec vec1(ptStart, ptEnd), vec2(ptRefStart, ptRefEnd);
		vec1.Normalize();
		vec2.Normalize();
		return vec1.IsParallel(vec2, dTol);
	}
	else if (!equal(dBulge, 0.) && !equal(dRefBulge, 0.))
	{// 弧线
		gp_Pnt ptMid1 = GetCurveMidPoint(ptStart, ptEnd, dBulge);
		GCE2d_MakeCircle arc1(gp_Pnt2d(ptStart.X(), ptStart.Y()), gp_Pnt2d(ptMid1.X(), ptMid1.Y()),
			gp_Pnt2d(ptEnd.X(), ptEnd.Y()));

		gp_Pnt ptMid2 = GetCurveMidPoint(ptRefStart, ptRefEnd, dRefBulge);
		GCE2d_MakeCircle arc2(gp_Pnt2d(ptRefStart.X(), ptRefStart.Y()), gp_Pnt2d(ptMid2.X(), ptMid2.Y()),
			gp_Pnt2d(ptRefEnd.X(), ptRefEnd.Y()));

		if (arc1.Value()->Circ2d().Location().IsEqual(arc2.Value()->Circ2d().Location(), dTol))
		{// 同心圆
			Bnd_Box box1, box2;
			box1.Add(ptStart);
			box1.Add(ptEnd);
			box2.Add(ptRefStart);
			box2.Add(ptRefEnd);

			gp_Pnt ptMid1 = GetCurveMidPoint(ptStart, ptEnd, dBulge);
			gp_Pnt ptMid2 = GetCurveMidPoint(ptRefStart, ptRefEnd, dRefBulge);
			box1.Add(ptMid1);
			box2.Add(ptMid2);

			return !box1.IsOut(box2);
		}
	}

	return false;
}

bool BimGeoAlgorithmTool::IsParallel(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge,
	const gp_Pnt2d& ptRefStart, const gp_Pnt2d& ptRefEnd, const double& dRefBulge, const double& dTol)
{
	return IsParallel(gp_Pnt(ptStart.X(), ptStart.Y(), 0.), gp_Pnt(ptEnd.X(), ptEnd.Y(), 0.), dBulge,
		gp_Pnt(ptRefStart.X(), ptRefStart.Y(), 0.), gp_Pnt(ptRefEnd.X(), ptRefEnd.Y(), 0.), dRefBulge, dTol);
}

int BimGeoAlgorithmTool::IsLeftOfCurve(const gp_Pnt& pt, const gp_Pnt& ptStart, const gp_Pnt& ptEnd,
	const double& dBulge, const double& dTol)
{
	if (IsOn(pt, ptStart, ptEnd, dBulge, dTol))
	{
		return 3;
	}

	double dValue = 0.;
	if (IsArc(dBulge))
	{// 弧线
		gp_Pnt ptClose = GetClosePoint(pt, ptStart, ptEnd, dBulge);

		gp_Pnt ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
		GCE2d_MakeCircle arc(gp_Pnt2d(ptStart.X(), ptStart.Y()), gp_Pnt2d(ptMid.X(), ptMid.Y()),
			gp_Pnt2d(ptEnd.X(), ptEnd.Y()));

		gp_Pnt2d ptCenter = arc.Value()->Location();

		gp_Vec vec1(pt, ptClose);
		gp_Vec vec2(ptClose, gp_Pnt(ptCenter.X(), ptCenter.Y(), 0.));
		vec1.Normalize();
		vec2.Normalize();

		int nRet = 1;
		dValue = vec1.X() * vec2.Y() - vec1.Y() * vec2.X();
		if (equal(dValue, 0.))
		{
			double dDist = pt.Distance(gp_Pnt(ptCenter.X(), ptCenter.Y(), 0.));
			if (lessThan(dDist, arc.Value()->Radius()))
			{// 在圆内
				nRet = 0;
			}
		}
		else
		{
			vec1 = gp_Vec(pt, ptStart);
			vec2 = gp_Vec(ptStart, ptEnd);
			vec1.Normalize();
			vec2.Normalize();

			dValue = vec1.X() * vec2.Y() - vec1.Y() * vec2.X();

			if (lessThan(dValue, 0.))
			{
				return 1;
			}
			else if (lessThan(0., dValue))
			{
				return 0;
			}
		}

		if (!arc.Value()->Circ2d().IsDirect())
		{// 顺时针 取反
			nRet = nRet == 1 ? 0 : 1;
		}

		return nRet;
	}
	else
	{
		// 叉乘
		gp_Vec vec1(pt, ptStart), vec2(ptStart, ptEnd);
		vec1.Normalize();
		vec2.Normalize();

		dValue = vec1.X() * vec2.Y() - vec1.Y() * vec2.X();
	}

	if (lessThan(dValue, 0.))
	{
		return 1;
	}
	else if (lessThan(0., dValue))
	{
		return 0;
	}

	return 2;
}

int BimGeoAlgorithmTool::IsLeftOfCurve(const gp_Pnt2d& pt, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd,
	const double& dBulge, const double& dTol)
{
	return IsLeftOfCurve(gp_Pnt(pt.X(), pt.Y(), 0.), gp_Pnt(ptStart.X(), ptStart.Y(), 0.),
		gp_Pnt(ptEnd.X(), ptEnd.Y(), 0.), dBulge, dTol);
}

gp_Vec BimGeoAlgorithmTool::GetNormal(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge)
{
	gp_Vec vecNormal;
	if (equal(dBulge, 0.))
	{// 直线
		vecNormal = gp_Vec(ptStart, ptEnd);
		vecNormal.Rotate(gp_Ax1(), M_PI_2);
	}
	else
	{// 弧线
		gp_Pnt ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
		GCE2d_MakeCircle arc(gp_Pnt2d(ptStart.X(), ptStart.Y()), gp_Pnt2d(ptMid.X(), ptMid.Y()),
			gp_Pnt2d(ptEnd.X(), ptEnd.Y()));

		if (arc.Value()->Circ2d().IsDirect())
		{// 逆时针
			vecNormal = gp_Vec(0., 0., 1.);
		}
		else
		{// 顺时针
			vecNormal = gp_Vec(0., 0., -1.);
		}
	}

	return vecNormal;
}

void GetArcLimitePointFunc(std::vector<gp_Pnt2d>& points, SGeoArcInfo& arcInfo, const double& dStartAngle, const double& dEndAngle)
{
	if ((BimGeoAlgorithmTool::lessEqualThan(dStartAngle, 0.)
		&& BimGeoAlgorithmTool::lessEqualThan(0., dEndAngle))
		|| BimGeoAlgorithmTool::equal(dStartAngle, M_PI * 2.)
		|| BimGeoAlgorithmTool::equal(dEndAngle, M_PI * 2.))
	{// 第一象限
		gp_Pnt2d ptLim(arcInfo.ptCenter.Coord() + gp_Vec2d(arcInfo.dRadius, 0.).XY());
		points.emplace_back(ptLim);
	}

	if (BimGeoAlgorithmTool::lessEqualThan(dStartAngle, M_PI_2)
		&& BimGeoAlgorithmTool::lessEqualThan(M_PI_2, dEndAngle))
	{// 第二象限
		gp_Pnt2d ptLim(arcInfo.ptCenter.Coord() + gp_Vec2d(0., arcInfo.dRadius).XY());
		points.emplace_back(ptLim);
	}

	if (BimGeoAlgorithmTool::lessEqualThan(dStartAngle, M_PI)
		&& BimGeoAlgorithmTool::lessEqualThan(M_PI, dEndAngle))
	{// 第三象限
		gp_Pnt2d ptLim(arcInfo.ptCenter.Coord() + gp_Vec2d(-arcInfo.dRadius, 0.).XY());
		points.emplace_back(ptLim);
	}

	if (BimGeoAlgorithmTool::lessEqualThan(dStartAngle, M_PI * 1.5)
		&& BimGeoAlgorithmTool::lessEqualThan(M_PI * 1.5, dEndAngle))
	{// 第四象限
		gp_Pnt2d ptLim(arcInfo.ptCenter.Coord() + gp_Vec2d(0., -arcInfo.dRadius).XY());
		points.emplace_back(ptLim);
	}

	if (BimGeoAlgorithmTool::lessEqualThan(dStartAngle, M_PI * 2)
		&& BimGeoAlgorithmTool::lessEqualThan(M_PI * 2, dEndAngle))
	{// 第四象限
		gp_Pnt2d ptLim(arcInfo.ptCenter.Coord() + gp_Vec2d(arcInfo.dRadius, 0.).XY());
		points.emplace_back(ptLim);
	}
};

void BimGeoAlgorithmTool::GetCurveBoxPoints(std::vector<gp_Pnt2d>& points, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge)
{
	points.emplace_back(ptStart);
	points.emplace_back(ptEnd);

	if (IsArc(dBulge))
	{
		gp_Pnt2d ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
		points.emplace_back(ptMid);

		SGeoArcInfo arcInfo = GetArcInfo(ptStart, ptEnd, dBulge);
		gp_Vec2d vecStart(arcInfo.ptCenter, ptStart),
			vecEnd(arcInfo.ptCenter, ptEnd);

		if (equal(vecStart.Magnitude(), 0.)
			|| equal(vecEnd.Magnitude(), 0.))
		{
			return;
		}

		vecStart.Normalize();
		vecEnd.Normalize();

		double dAngleS = gp_Vec2d(1., 0.).Angle(vecStart);
		double dAngleE = gp_Vec2d(1., 0.).Angle(vecEnd);

		if (!arcInfo.bIsAntiClockwise)
		{// 顺时针
			double dTemp = dAngleS;
			dAngleS = dAngleE;
			dAngleE = dTemp;
		}

		if (lessEqualThan(dAngleS, 0.))
		{
			dAngleS += M_PI * 2;
		}

		if (lessEqualThan(dAngleE, 0.))
		{
			dAngleE += M_PI * 2;
		}

		if (lessThan(dAngleE, dAngleS))
		{
			GetArcLimitePointFunc(points, arcInfo, dAngleS, M_PI * 2);
			dAngleS = 0.;
		}

		GetArcLimitePointFunc(points, arcInfo, dAngleS, dAngleE);
	}
}

void BimGeoAlgorithmTool::GetCurveBoxPoints(std::vector<gp_Pnt>& points, const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge)
{
	std::vector<gp_Pnt2d> point2ds;
	GetCurveBoxPoints(point2ds, gp_Pnt2d(ptStart.X(), ptStart.Y()), gp_Pnt2d(ptEnd.X(), ptEnd.Y()), dBulge);

	for (int i = 0; i < point2ds.size(); ++i)
	{
		points.emplace_back(gp_Pnt(point2ds[i].X(), point2ds[i].Y(), 0.));
	}
}

Bnd_Box BimGeoAlgorithmTool::GetCurveBndBox(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge)
{
	std::vector<gp_Pnt> points;
	GetCurveBoxPoints(points, ptStart, ptEnd, dBulge);

	Bnd_Box box;
	for (int i = 0; i < points.size(); ++i)
	{
		box.Add(points[i]);
	}

	return box;
}

Bnd_Box2d BimGeoAlgorithmTool::GetCurveBndBox(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge)
{
	std::vector<gp_Pnt2d> points;
	GetCurveBoxPoints(points, ptStart, ptEnd, dBulge);

	Bnd_Box2d box;
	for (int i = 0; i < points.size(); ++i)
	{
		box.Add(points[i]);
	}

	return box;
}

double BimGeoAlgorithmTool::GetCurveLength(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge)
{
	return GetCurveLength(gp_Pnt2d(ptStart.X(), ptStart.Y()), gp_Pnt2d(ptEnd.X(), ptEnd.Y()), dBulge);
}

double BimGeoAlgorithmTool::GetCurveLength(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge)
{
	if (IsArc(dBulge))
	{// 弧线段
		gp_Pnt2d ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
		GCE2d_MakeCircle arc(ptStart, ptMid, ptEnd);

		gp_Pnt2d ptCenter = arc.Value()->Location();
		gp_Vec2d vecS(ptCenter, ptStart), vecE(ptCenter, ptEnd);
		vecS.Normalize();
		vecE.Normalize();

		double dAngle = vecS.Angle(vecE);
		if (!arc.Value()->Circ2d().IsDirect())
		{// 顺时针
			dAngle = -dAngle;
		}

		while (dAngle < 0)
		{
			dAngle += 2 * M_PI;
		}

		return dAngle * arc.Value()->Radius();
	}

	return ptStart.Distance(ptEnd);
}

void BimGeoAlgorithmTool::RemovePoint(std::list<SGeoCurveInfo>& curves, const double& dTol)
{
	std::list<SGeoCurveInfo>::iterator itBegin = curves.begin();
	while (itBegin != curves.end())
	{
		if (itBegin->ptS.IsEqual(itBegin->ptE, dTol))
		{
			itBegin = curves.erase(itBegin);
			continue;
		}

		++itBegin;
	}
}

void BimGeoAlgorithmTool::BreakCurves(std::list<SGeoCurveInfo>& curves, const double& dTol)
{
	if (curves.size() < 2)
	{
		return;
	}

	// 构建四叉树
	Bnd_Box2d box;
	for (std::list<SGeoCurveInfo>::iterator itCurve = curves.begin(); itCurve != curves.end(); ++itCurve)
	{
		Bnd_Box2d curveBox = GetCurveBndBox(itCurve->ptS, itCurve->ptE, itCurve->dBulge);
		box.Add(curveBox);
	}

	struct SQuadTreeCurveData
	{
		SQuadTreeCurveData() : dX(0.), dY(0.), dW(0.), dH(0.), pCurve(NULL), bIsErase(false) {}
		double dX;
		double dY;
		double dW;
		double dH;
		SGeoCurveInfo* pCurve;
		bool bIsErase;
	};

	double dXmin = 0., dYMin = 0., dXmax = 0., dYmax = 0.;
	box.Get(dXmin, dYMin, dXmax, dYmax);

	QuadTreeNode<SQuadTreeCurveData> quadTreeNode(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin,
		0, 10, QuadType::ROOT, NULL);

	std::list<SQuadTreeCurveData> quadTreeCurveDatas;
	for (std::list<SGeoCurveInfo>::iterator itBegin = curves.begin(); itBegin != curves.end(); ++itBegin)
	{
		Bnd_Box2d curveBox = GetCurveBndBox(itBegin->ptS, itBegin->ptE, itBegin->dBulge);
		curveBox.Get(dXmin, dYMin, dXmax, dYmax);

		SQuadTreeCurveData data;
		data.dX = dXmin;
		data.dY = dYMin;
		data.dW = dXmax - dXmin;
		data.dH = dYmax - dYMin;
		data.pCurve = &*itBegin;

		quadTreeCurveDatas.emplace_back(data);
		quadTreeNode.InsertObject(&*quadTreeCurveDatas.rbegin());
	}

	// 打断线段
	std::list<SGeoCurveInfo>::iterator itBegin = curves.begin();
	while (itBegin != curves.end())
	{
		Bnd_Box2d curveBox = GetCurveBndBox(itBegin->ptS, itBegin->ptE, itBegin->dBulge);
		curveBox.SetGap(dTol);
		curveBox.Get(dXmin, dYMin, dXmax, dYmax);

		std::list<SQuadTreeCurveData*> intersectCurveDatas = quadTreeNode.GetObjectsAt(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin);
		for (std::list<SQuadTreeCurveData*>::iterator itDataBegin = intersectCurveDatas.begin();
			itDataBegin != intersectCurveDatas.end(); ++itDataBegin)
		{
			SQuadTreeCurveData* pData = *itDataBegin;
			if (pData->bIsErase || pData->pCurve == &*itBegin)
			{
				continue;
			}

			std::vector<gp_Pnt2d> vecIntersectPt;
			CalcIntersect(vecIntersectPt, itBegin->ptS, itBegin->ptE, itBegin->dBulge,
				pData->pCurve->ptS, pData->pCurve->ptE, pData->pCurve->dBulge, dTol);

			if (0 < vecIntersectPt.size())
			{
				std::vector<SGeoCurveInfo> vecCurve = BreakCurveByPoint(*itBegin, vecIntersectPt, dTol);
				if (1 < vecCurve.size())
				{
					*itBegin = vecCurve[0];
					curveBox = GetCurveBndBox(itBegin->ptS, itBegin->ptE, itBegin->dBulge);
					curveBox.SetGap(dTol);

					for (int i = 1; i < vecCurve.size(); ++i)
					{// 新增
						curves.emplace_back(vecCurve[i]);

						Bnd_Box2d newBox = GetCurveBndBox(vecCurve[i].ptS, vecCurve[i].ptE, vecCurve[i].dBulge);
						newBox.Get(dXmin, dYMin, dXmax, dYmax);

						SQuadTreeCurveData data;
						data.dX = dXmin;
						data.dY = dYMin;
						data.dW = dXmax - dXmin;
						data.dH = dYmax - dYMin;
						data.pCurve = &*curves.rbegin();

						quadTreeCurveDatas.emplace_back(data);
						quadTreeNode.InsertObject(&*quadTreeCurveDatas.rbegin());
					}
				}

				vecCurve = BreakCurveByPoint(*pData->pCurve, vecIntersectPt, dTol);
				if (1 < vecCurve.size())
				{
					*pData->pCurve = vecCurve[0];
					for (int i = 1; i < vecCurve.size(); ++i)
					{// 新增
						curves.emplace_back(vecCurve[i]);

						SQuadTreeCurveData data;
						data.dX = dXmin;
						data.dY = dYMin;
						data.dW = dXmax - dXmin;
						data.dH = dYmax - dYMin;
						data.pCurve = &*curves.rbegin();

						quadTreeCurveDatas.emplace_back(data);
						quadTreeNode.InsertObject(&*quadTreeCurveDatas.rbegin());
					}
				}
			}
		}

		++itBegin;
	}
}

void BimGeoAlgorithmTool::BreakCurves(std::list<SGeoCurveInfo>& curves, const std::list<SGeoCurveInfo>& refCurves, const double & dTol)
{
	if (curves.size() < 2)
	{
		return;
	}

	// 构建四叉树
	Bnd_Box2d box;
	for (std::list<SGeoCurveInfo>::const_iterator itBegin = refCurves.begin();
		itBegin != refCurves.end(); ++itBegin)
	{
		Bnd_Box2d curveBox = GetCurveBndBox(itBegin->ptS, itBegin->ptE, itBegin->dBulge);
		box.Add(curveBox);
	}

	if (box.IsVoid() || box.IsWhole())
	{
		return;
	}

	struct SQuadTreeCurveData
	{
		double dX;
		double dY;
		double dW;
		double dH;
		const SGeoCurveInfo* pCurve;
	};

	double dXmin = 0., dYMin = 0., dXmax = 0., dYmax = 0.;
	box.Get(dXmin, dYMin, dXmax, dYmax);

	QuadTreeNode<SQuadTreeCurveData> quadTreeNode(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin,
		0, 10, QuadType::ROOT, NULL);

	std::list<SQuadTreeCurveData> quadTreeCurveDatas;
	for (std::list<SGeoCurveInfo>::const_iterator itBegin = refCurves.begin();
		itBegin != refCurves.end(); ++itBegin)
	{
		Bnd_Box2d curveBox = GetCurveBndBox(itBegin->ptS, itBegin->ptE, itBegin->dBulge);
		curveBox.Get(dXmin, dYMin, dXmax, dYmax);

		SQuadTreeCurveData data;
		data.dX = dXmin;
		data.dY = dYMin;
		data.dW = dXmax - dXmin;
		data.dH = dYmax - dYMin;
		data.pCurve = &*itBegin;

		quadTreeCurveDatas.emplace_back(data);
		quadTreeNode.InsertObject(&*quadTreeCurveDatas.rbegin());
	}

	// 打断线段
	std::list<SGeoCurveInfo>::iterator itBegin = curves.begin();
	while (itBegin != curves.end())
	{
		Bnd_Box2d bndBox = GetCurveBndBox(itBegin->ptS, itBegin->ptE, itBegin->dBulge);
		bndBox.SetGap(dTol);
		bndBox.Get(dXmin, dYMin, dXmax, dYmax);

		std::list<SQuadTreeCurveData*> intersectCurveDatas = quadTreeNode.GetObjectsAt(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin);
		for (std::list<SQuadTreeCurveData*>::iterator itDataBegin = intersectCurveDatas.begin();
			itDataBegin != intersectCurveDatas.end(); ++itDataBegin)
		{
			SQuadTreeCurveData* pData = *itDataBegin;
			std::vector<gp_Pnt2d> vecIntersectPt;
			CalcIntersect(vecIntersectPt, itBegin->ptS, itBegin->ptE, itBegin->dBulge,
				pData->pCurve->ptS, pData->pCurve->ptE, pData->pCurve->dBulge, dTol);

			if (0 < vecIntersectPt.size())
			{
				std::vector<SGeoCurveInfo> vecCurve = BreakCurveByPoint(*itBegin, vecIntersectPt, dTol);
				if (1 < vecCurve.size())
				{
					*itBegin = vecCurve[0];
					bndBox = GetCurveBndBox(itBegin->ptS, itBegin->ptE, itBegin->dBulge);
					bndBox.SetGap(dTol);

					for (int i = 1; i < vecCurve.size(); ++i)
					{
						curves.emplace_back(vecCurve[i]);
					}
				}
			}
		}

		++itBegin;
	}
}

std::vector<SGeoCurveInfo> BimGeoAlgorithmTool::BreakCurveByPoint(const SGeoCurveInfo& curve,
	const std::vector<gp_Pnt2d>& pts, const double& dTol, bool bIsSort)
{
	std::vector<SGeoCurveInfo> vecRet;
	SGeoCurveInfo currentCurve = curve;

	std::vector<gp_Pnt2d> vecSortPt;
	if (bIsSort)
	{
		vecSortPt = SortByCurve(curve, pts);
	}
	else
	{
		vecSortPt = pts;
	}

	for (int i = 0; i < vecSortPt.size(); ++i)
	{
		gp_Pnt2d ptClose = vecSortPt[i];
		if (currentCurve.ptS.IsEqual(ptClose, dTol)
			|| currentCurve.ptE.IsEqual(ptClose, dTol))
		{
			continue;
		}

		if (IsArc(curve.dBulge))
		{
			gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(curve.ptS, curve.ptE, curve.dBulge);
			GCE2d_MakeCircle arc(curve.ptS, ptMid, curve.ptE);

			gp_Pnt2d ptCenter = arc.Value()->Location();

			SGeoCurveInfo newCurve = currentCurve;
			newCurve.ptE = ptClose;
			newCurve.dBulge = CalcArcBulge(newCurve.ptS, newCurve.ptE, ptCenter, arc.Value()->Radius(),
				arc.Value()->Circ2d().IsDirect(), dTol);
			vecRet.emplace_back(newCurve);

			currentCurve.ptS = ptClose;
			currentCurve.ptE = curve.ptE;
			currentCurve.dBulge = CalcArcBulge(currentCurve.ptS, currentCurve.ptE, ptCenter, arc.Value()->Radius(),
				arc.Value()->Circ2d().IsDirect(), dTol);
		}
		else
		{
			SGeoCurveInfo newCurve = currentCurve;
			newCurve.ptE = ptClose;
			vecRet.emplace_back(newCurve);

			currentCurve.ptS = ptClose;
		}
	}

	vecRet.emplace_back(currentCurve);

	return vecRet;
}

double BimGeoAlgorithmTool::CalcArcBulge(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const gp_Pnt2d& ptCenter,
	const double& dRadius, const bool IsAnticlockwise, const double& dTol)
{
	if (ptS.IsEqual(ptE, dTol))
	{
		return 0.;
	}

	double dMidDist = ptS.Distance(ptE) * 0.5;
	gp_Pnt2d ptMid((ptS.Coord() + ptE.Coord()) * 0.5);

	gp_Pnt2d ptArcMid = GetArcMidPoint(ptS, ptE, ptCenter, dRadius, IsAnticlockwise);

	double dDist = ptArcMid.Distance(ptMid);
	double dBulge = dDist / dMidDist;
	if (!IsAnticlockwise)
	{
		dBulge = -dBulge;
	}

	return dBulge;
}

double BimGeoAlgorithmTool::CalcArcBulge(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const gp_Pnt2d& ptRefStart,
	const gp_Pnt2d& ptRefEnd, const double& dRefBulge, const double& dTol)
{
	gp_Pnt2d ptMid = GetCurveMidPoint(ptRefStart, ptRefEnd, dRefBulge);
	GCE2d_MakeCircle arc(ptRefStart, ptMid, ptRefEnd);
	if (arc.Value().IsNull())
	{
		return 0.;
	}

	return CalcArcBulge(ptS, ptE, arc.Value()->Location(), arc.Value()->Radius(),
		arc.Value()->Circ2d().IsDirect(), dTol);
}

double BimGeoAlgorithmTool::CalcArcBulge(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const gp_Pnt2d& ptMid)
{
	GCE2d_MakeCircle arc(ptS, ptMid, ptE);
	if (!arc.Value())
	{
		return 0.;
	}

	return CalcArcBulge(ptS, ptE, arc.Value()->Circ2d().Location(),
		arc.Value()->Circ2d().Radius(), arc.Value()->Circ2d().IsDirect());
}

std::vector<gp_Pnt2d> BimGeoAlgorithmTool::SortByCurve(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE,
	const double& dBulge, const std::vector<gp_Pnt2d>& vecIntersectPt, bool bIsCalcClosePt/* = true*/)
{
	std::vector<gp_Pnt2d> vecClosePt;
	if (vecIntersectPt.size() <= 1)
	{
		if (bIsCalcClosePt)
		{
			vecClosePt.emplace_back(BimGeoAlgorithmTool::GetClosePoint(vecIntersectPt[0], ptS, ptE, dBulge));
			return vecClosePt;
		}

		return vecIntersectPt;
	}

	for (int i = 0; i < vecIntersectPt.size(); ++i)
	{
		const gp_Pnt2d& pt = vecIntersectPt[i];
		if (bIsCalcClosePt)
		{
			vecClosePt.emplace_back(BimGeoAlgorithmTool::GetClosePoint(pt, ptS, ptE, dBulge));
		}
		else
		{
			vecClosePt.emplace_back(pt);
		}
	}

	std::map<double, gp_Pnt2d> mapDistPoint;
	if (IsArc(dBulge))
	{
		gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(ptS, ptE, dBulge);
		GCE2d_MakeCircle arc(ptS, ptMid, ptE);

		double dStartDist = 0.;
		{
			Geom2dAPI_ProjectPointOnCurve Projector(ptS, arc.Value());
			if (0 < Projector.NbPoints())
			{
				dStartDist = Projector.Parameter(1);
			}
		}

		for (int i = 0; i < vecClosePt.size(); ++i)
		{
			gp_Pnt2d& pt = vecClosePt[i];
			Geom2dAPI_ProjectPointOnCurve Projector(pt, arc.Value());
			if (Projector.NbPoints() < 1)
			{
				continue;
			}

			double dParam = Projector.Parameter(1) + dStartDist;
			if (lessEqualThan(M_PI * 2, dParam))
			{
				dParam -= M_PI * 2;
			}

			mapDistPoint[dParam] = pt;
		}
	}
	else
	{
		for (int i = 0; i < vecClosePt.size(); ++i)
		{
			mapDistPoint[ptS.Distance(vecClosePt[i])] = vecClosePt[i];
		}
	}

	double dPreDist = -1.;
	std::vector<gp_Pnt2d> vecPt;
	for (std::map<double, gp_Pnt2d>::iterator iter = mapDistPoint.begin();
		iter != mapDistPoint.end(); ++iter)
	{
		if (!BimGeoAlgorithmTool::equal(dPreDist, iter->first))
		{
			dPreDist = iter->first;
			vecPt.emplace_back(iter->second);
		}
	}

	return vecPt;
}

std::vector<gp_Pnt2d> BimGeoAlgorithmTool::SortByCurve(const SGeoCurveInfo& curve,
	const std::vector<gp_Pnt2d>& vecIntersectPt, bool bIsCalcClosePt)
{
	return SortByCurve(curve.ptS, curve.ptE, curve.dBulge, vecIntersectPt, bIsCalcClosePt);
}

std::vector<gp_Pnt> BimGeoAlgorithmTool::SortByCurve(const gp_Pnt& ptS, const gp_Pnt& ptE,
	const std::vector<gp_Pnt>& vecIntersectPt)
{
	if (vecIntersectPt.size() <= 1)
	{
		return vecIntersectPt;
	}

	std::map<double, gp_Pnt> mapDistPoint;
	for (int i = 0; i < vecIntersectPt.size(); ++i)
	{
		mapDistPoint[ptS.Distance(vecIntersectPt[i])] = vecIntersectPt[i];
	}

	std::vector<gp_Pnt> vecPt;
	for (std::map<double, gp_Pnt>::iterator iter = mapDistPoint.begin();
		iter != mapDistPoint.end(); ++iter)
	{
		vecPt.emplace_back(iter->second);
	}

	return vecPt;
}

void BimGeoAlgorithmTool::RemoveOverlap(std::list<SGeoCurveInfo>& curves, const double& dTol, const bool IsVectorCurve)
{
	if (curves.size() < 2)
	{
		return;
	}

	// 构建四叉树
	Bnd_Box2d box;
	for (std::list<SGeoCurveInfo>::iterator itCurve = curves.begin();
		itCurve != curves.end(); ++itCurve)
	{
		Bnd_Box2d curveBox = GetCurveBndBox(itCurve->ptS, itCurve->ptE, itCurve->dBulge);
		box.Add(curveBox);
	}

	struct SQuadTreeCurveData
	{
		SQuadTreeCurveData() : dX(0.), dY(0.), dW(0.), dH(0.), pCurve(NULL), bIsErase(false) {}
		double dX;
		double dY;
		double dW;
		double dH;
		SGeoCurveInfo* pCurve;
		bool bIsErase;
	};

	double dXmin = 0., dYMin = 0., dXmax = 0., dYmax = 0.;
	box.Get(dXmin, dYMin, dXmax, dYmax);

	QuadTreeNode<SQuadTreeCurveData> quadTreeNode(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin,
		0, 10, QuadType::ROOT, NULL);

	std::list<SQuadTreeCurveData> quadTreeCurveDatas;
	for (std::list<SGeoCurveInfo>::iterator itCurve = curves.begin();
		itCurve != curves.end(); ++itCurve)
	{
		Bnd_Box2d curveBox = GetCurveBndBox(itCurve->ptS, itCurve->ptE, itCurve->dBulge);
		curveBox.Get(dXmin, dYMin, dXmax, dYmax);

		SQuadTreeCurveData data;
		data.dX = dXmin;
		data.dY = dYMin;
		data.dW = dXmax - dXmin;
		data.dH = dYmax - dYMin;
		data.pCurve = &*itCurve;

		quadTreeCurveDatas.emplace_back(data);
		quadTreeNode.InsertObject(&*quadTreeCurveDatas.rbegin());
	}

	// 去除重复线
	std::list<SGeoCurveInfo>::iterator itBegin = curves.begin();
	while (itBegin != curves.end())
	{
		if (equal(itBegin->ptS.Distance(itBegin->ptE), 0.))
		{
			itBegin = curves.erase(itBegin);
			continue;
		}

		Bnd_Box2d bndBox = GetCurveBndBox(itBegin->ptS, itBegin->ptE, itBegin->dBulge);
		bndBox.SetGap(dTol);
		bndBox.Get(dXmin, dYMin, dXmax, dYmax);

		std::list<SQuadTreeCurveData*> intersectCurveDatas = quadTreeNode.GetObjectsAt(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin);
		for (std::list< SQuadTreeCurveData*>::iterator itData = intersectCurveDatas.begin();
			itData != intersectCurveDatas.end(); ++itData)
		{
			SQuadTreeCurveData* pData = *itData;
			if (pData->bIsErase)
			{
				continue;
			}

			if (pData->pCurve == &*itBegin)
			{
				continue;
			}

			if (IsArc(itBegin->dBulge) && IsArc(pData->pCurve->dBulge))
			{
				if (IsParallel(itBegin->ptS, itBegin->ptE, itBegin->dBulge,
					pData->pCurve->ptS, pData->pCurve->ptE, pData->pCurve->dBulge, dTol))
				{
					if (equal(itBegin->dBulge, pData->pCurve->dBulge, dTol)
						&& itBegin->ptS.IsEqual(pData->pCurve->ptS, dTol)
						&& itBegin->ptE.IsEqual(pData->pCurve->ptE, dTol))
					{
						pData->bIsErase = true;
						pData->pCurve->ptS = pData->pCurve->ptE;
						continue;
					}
					else if (!IsVectorCurve &&
						equal(-pData->pCurve->dBulge, pData->pCurve->dBulge, dTol)
						&& itBegin->ptS.IsEqual(pData->pCurve->ptE, dTol)
						&& itBegin->ptE.IsEqual(pData->pCurve->ptS, dTol))
					{
						pData->bIsErase = true;
						pData->pCurve->ptS = pData->pCurve->ptE;
						continue;
					}
				}
			}
			else if (!IsArc(itBegin->dBulge) && !IsArc(pData->pCurve->dBulge))
			{
				if ((itBegin->ptS.IsEqual(pData->pCurve->ptS, dTol)
					&& itBegin->ptE.IsEqual(pData->pCurve->ptE, dTol)))
				{
					pData->bIsErase = true;
					pData->pCurve->ptS = pData->pCurve->ptE;
					continue;
				}
				else if (!IsVectorCurve && (itBegin->ptS.IsEqual(pData->pCurve->ptE, dTol)
					&& itBegin->ptE.IsEqual(pData->pCurve->ptS, dTol)))
				{
					pData->bIsErase = true;
					pData->pCurve->ptS = pData->pCurve->ptE;
					continue;
				}
			}
			else if ((itBegin->ptS.IsEqual(pData->pCurve->ptS, dTol)
				&& itBegin->ptE.IsEqual(pData->pCurve->ptE, dTol))
				|| (!IsVectorCurve && (itBegin->ptS.IsEqual(pData->pCurve->ptE, dTol)
					&& itBegin->ptE.IsEqual(pData->pCurve->ptS, dTol))))
			{
				gp_Pnt2d ptMid = GetCurveMidPoint(pData->pCurve->ptS, pData->pCurve->ptE, pData->pCurve->dBulge);
				if (IsOn(ptMid, itBegin->ptS, itBegin->ptE, itBegin->dBulge, dTol))
				{
					pData->bIsErase = true;
					pData->pCurve->ptS = pData->pCurve->ptE;
					continue;
				}
			}
		}

		++itBegin;
	}

	RemovePoint(curves);
}

void BimGeoAlgorithmTool::RemoveOverlap(std::list<SGeoCurveInfo>& curves, const std::list<SGeoCurveInfo>& refCurves, const double & dTol, const bool IsVectorCurve)
{
	// 构建四叉树
	Bnd_Box2d box;
	for (std::list<SGeoCurveInfo>::const_iterator itCurve = refCurves.begin();
		itCurve != refCurves.end(); ++itCurve)
	{
		Bnd_Box2d curveBox = GetCurveBndBox(itCurve->ptS, itCurve->ptE, itCurve->dBulge);
		box.Add(curveBox);
	}

	if (box.IsVoid() || box.IsWhole())
	{
		return;
	}

	struct SQuadTreeCurveData
	{
		double dX;
		double dY;
		double dW;
		double dH;
		const SGeoCurveInfo* pCurve;
	};

	double dXmin = 0., dYMin = 0., dXmax = 0., dYmax = 0.;
	box.Get(dXmin, dYMin, dXmax, dYmax);

	QuadTreeNode<SQuadTreeCurveData> quadTreeNode(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin,
		0, 10, QuadType::ROOT, NULL);

	std::list<SQuadTreeCurveData> quadTreeCurveDatas;
	for (std::list<SGeoCurveInfo>::const_iterator itCurve = refCurves.begin();
		itCurve != refCurves.end(); ++itCurve)
	{
		Bnd_Box2d curveBox = GetCurveBndBox(itCurve->ptS, itCurve->ptE, itCurve->dBulge);
		curveBox.Get(dXmin, dYMin, dXmax, dYmax);

		SQuadTreeCurveData data;
		data.dX = dXmin;
		data.dY = dYMin;
		data.dW = dXmax - dXmin;
		data.dH = dYmax - dYMin;
		data.pCurve = &*itCurve;

		quadTreeCurveDatas.emplace_back(data);
		quadTreeNode.InsertObject(&*quadTreeCurveDatas.rbegin());
	}

	// 去除重复线
	std::list<SGeoCurveInfo>::iterator itBegin = curves.begin();
	while (itBegin != curves.end())
	{
		Bnd_Box2d bndBox = GetCurveBndBox(itBegin->ptS, itBegin->ptE, itBegin->dBulge);
		bndBox.SetGap(dTol);
		bndBox.Get(dXmin, dYMin, dXmax, dYmax);

		bool bIsContinue = false;
		std::list<SQuadTreeCurveData*> intersectCurveDatas = quadTreeNode.GetObjectsAt(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin);
		for (std::list< SQuadTreeCurveData*>::iterator itData = intersectCurveDatas.begin();
			itData != intersectCurveDatas.end(); ++itData)
		{
			SQuadTreeCurveData* pData = *itData;
			if (IsArc(itBegin->dBulge) && IsArc(pData->pCurve->dBulge))
			{
				if (IsParallel(itBegin->ptS, itBegin->ptE, itBegin->dBulge,
					pData->pCurve->ptS, pData->pCurve->ptE, pData->pCurve->dBulge, dTol))
				{
					if (equal(itBegin->dBulge, pData->pCurve->dBulge, dTol)
						&& itBegin->ptS.IsEqual(pData->pCurve->ptS, dTol)
						&& itBegin->ptE.IsEqual(pData->pCurve->ptE, dTol))
					{
						itBegin = curves.erase(itBegin);
						bIsContinue = true;
						break;
					}
					else if (!IsVectorCurve &&
						equal(-itBegin->dBulge, pData->pCurve->dBulge, dTol)
						&& itBegin->ptS.IsEqual(pData->pCurve->ptE, dTol)
						&& itBegin->ptE.IsEqual(pData->pCurve->ptS, dTol))
					{
						itBegin = curves.erase(itBegin);
						bIsContinue = true;
						break;
					}
				}
			}
			else if (!IsArc(itBegin->dBulge) && !IsArc(pData->pCurve->dBulge))
			{
				if ((itBegin->ptS.IsEqual(pData->pCurve->ptS, dTol)
					&& itBegin->ptE.IsEqual(pData->pCurve->ptE, dTol)))
				{
					itBegin = curves.erase(itBegin);
					bIsContinue = true;
					break;
				}
				else if (!IsVectorCurve && (itBegin->ptS.IsEqual(pData->pCurve->ptE, dTol)
					&& itBegin->ptE.IsEqual(pData->pCurve->ptS, dTol)))
				{
					itBegin = curves.erase(itBegin);
					bIsContinue = true;
					break;
				}
			}
		}

		if (bIsContinue)
		{
			continue;
		}

		++itBegin;
	}
}

std::list<SGeoCurveInfo> BimGeoAlgorithmTool::CalcTwoCurveOverlap(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd,
	const double& dBulge, const gp_Pnt2d& ptRefStart, const gp_Pnt2d& ptRefEnd, const double& dRefBulge,
	const double& dTol, const double& dAngleTol)
{
	std::list<SGeoCurveInfo> overlapCurves;
	if (IsArc(dBulge) != IsArc(dRefBulge)
		|| !IsParallel(ptStart, ptEnd, dBulge, ptRefStart, ptRefEnd, dRefBulge, dAngleTol))
	{
		return overlapCurves;
	}

	std::vector<gp_Pnt2d> vecPt;
	vecPt.emplace_back(ptStart);
	vecPt.emplace_back(ptEnd);
	vecPt.emplace_back(GetClosePoint(ptRefStart, ptStart, ptEnd, dBulge, dTol));
	vecPt.emplace_back(GetClosePoint(ptRefEnd, ptStart, ptEnd, dBulge, dTol));

	if (vecPt.size() < 2)
	{
		return overlapCurves;
	}

	std::vector<gp_Pnt2d> vecSortPt = SortByCurve(ptStart, ptEnd, dBulge, vecPt);
	for (int i = 1; i < vecSortPt.size(); ++i)
	{
		int j = i - 1;
		gp_Pnt2d ptMid((vecSortPt[j].Coord() + vecSortPt[i].Coord()) * 0.5);
		if (IsArc(dBulge))
		{
			ptMid = GetClosePoint(ptMid, ptStart, ptEnd, dBulge, dTol);
		}

		gp_Pnt2d ptClose = GetClosePoint(ptMid, ptRefStart, ptRefEnd, dRefBulge, dTol);
		if (ptClose.IsEqual(ptRefStart, dTol)
			|| ptClose.IsEqual(ptRefEnd, dTol))
		{
			continue;
		}

		SGeoCurveInfo curveInfo;
		curveInfo.ptS = vecSortPt[j];
		curveInfo.ptE = vecSortPt[i];
		curveInfo.dBulge = 0.;
		if (IsArc(dBulge))
		{
			curveInfo.dBulge = CalcArcBulge(curveInfo.ptS, curveInfo.ptE, ptStart, ptEnd, dBulge, dTol);
		}

		overlapCurves.emplace_back(curveInfo);
	}

	// 合并
	std::list<SGeoCurveInfo>::iterator itBegin = overlapCurves.begin();
	while (itBegin != overlapCurves.end())
	{
		std::list<SGeoCurveInfo>::iterator itNext = itBegin;
		if (++itNext == overlapCurves.end())
		{
			break;
		}

		if (itBegin->ptE.IsEqual(itNext->ptS, dTol))
		{
			itBegin->ptE = itNext->ptE;
			if (IsArc(dBulge))
			{
				itBegin->dBulge = CalcArcBulge(itBegin->ptS, itBegin->ptE, ptStart, ptEnd, dBulge, dTol);
			}

			overlapCurves.erase(itNext);

			continue;
		}

		++itBegin;
	}

	return overlapCurves;
}

std::list<SGeoCurveInfo> BimGeoAlgorithmTool::CalcTwoCurveOverlap(const SGeoCurveInfo& curve,
	const SGeoCurveInfo& curveRef, const double& dTol, const double& dAngleTol)
{
	return CalcTwoCurveOverlap(curve.ptS, curve.ptE, curve.dBulge, curveRef.ptS, curveRef.ptE, curveRef.dBulge, dTol, dAngleTol);
}

void BimGeoAlgorithmTool::ConnectEqualPoint(std::list<SGeoCurveInfo>& curves,
	const double& dTol, const bool bIsMerge, const bool bIsIsolated)
{
	if (curves.size() < 2)
	{
		return;
	}

	CEqualPointKey::s_tol = 0.0001;
	std::map<CEqualPointKey, int> mapPointCount;

	Bnd_Box2d box;
	for (std::list<SGeoCurveInfo>::iterator itCurve = curves.begin();
		itCurve != curves.end(); ++itCurve)
	{
		box.Add(itCurve->ptS);
		box.Add(itCurve->ptE);

		mapPointCount[itCurve->ptS]++;
		mapPointCount[itCurve->ptE]++;
	}

	if (box.IsVoid() || box.IsWhole())
	{
		return;
	}

	box.SetGap(dTol);

	if (mapPointCount.size() < 1)
	{
		return;
	}

	struct SRegularPointData
	{
		double dX;
		double dY;
		double dW;
		double dH;
		gp_Pnt2d* pPt;
		gp_Pnt2d* pOtherPt;
		SGeoCurveInfo* pCurve;
	};

	double dXmin = 0., dYMin = 0., dXmax = 0., dYmax = 0.;
	box.Get(dXmin, dYMin, dXmax, dYmax);

	QuadTreeNode<SRegularPointData> quadTreeNode(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin,
		0, 10, QuadType::ROOT, NULL);

	std::list<SRegularPointData> regularPointDatas;
	for (std::list<SGeoCurveInfo>::iterator itCurve = curves.begin();
		itCurve != curves.end(); ++itCurve)
	{
		SRegularPointData data;
		data.dX = itCurve->ptS.X();
		data.dY = itCurve->ptS.Y();
		data.dW = dTol;
		data.dH = dTol;
		data.pCurve = &*itCurve;
		data.pPt = &itCurve->ptS;
		data.pOtherPt = &itCurve->ptE;
		regularPointDatas.emplace_back(data);

		quadTreeNode.InsertObject(&*regularPointDatas.rbegin());

		data.dX = itCurve->ptE.X();
		data.dY = itCurve->ptE.Y();
		data.pPt = &itCurve->ptE;
		data.pOtherPt = &itCurve->ptS;
		regularPointDatas.emplace_back(data);

		quadTreeNode.InsertObject(&*regularPointDatas.rbegin());
	}

	// 规整点
	for (std::list<SRegularPointData>::iterator itData = regularPointDatas.begin();
		itData != regularPointDatas.end(); ++itData)
	{
		SRegularPointData& regularData = *itData;
		gp_Pnt2d* pPoint = regularData.pPt;
		if (bIsIsolated)
		{
			std::map<CEqualPointKey, int>::iterator itFind = mapPointCount.find(*pPoint);
			if (mapPointCount.end() != itFind && 1 < itFind->second)
			{
				continue;
			}
		}

		std::list<SRegularPointData*> retEqualPoints = quadTreeNode.GetObjectsAt(pPoint->X() - dTol, pPoint->Y() - dTol,
			dTol * 1.5, dTol * 1.5);

		if (bIsMerge)
		{
			for (std::list<SRegularPointData*>::iterator itRetData = retEqualPoints.begin();
				itRetData != retEqualPoints.end(); ++itRetData)
			{
				SRegularPointData* pData = *itRetData;
				if (BimGeoAlgorithmTool::equal(pPoint->X(), pData->dX, dTol)
					&& BimGeoAlgorithmTool::equal(pPoint->Y(), pData->dY, dTol))
				{
					*pPoint = *pData->pPt;
					mapPointCount[*pData->pPt]++;
				}
			}
		}
		else
		{
			double dMinDist = -1.;
			SRegularPointData* pCloseData = NULL;
			bool bIsIsolatedConnect = false;

			// 取最短的连接
			for (std::list<SRegularPointData*>::iterator itRetData = retEqualPoints.begin();
				itRetData != retEqualPoints.end(); ++itRetData)
			{
				SRegularPointData* pData = *itRetData;
				if (BimGeoAlgorithmTool::equal(pPoint->X(), pData->dX, dTol)
					&& BimGeoAlgorithmTool::equal(pPoint->Y(), pData->dY, dTol))
				{
					if ((!BimGeoAlgorithmTool::equal(pPoint->X(), pData->pPt->X())
						|| !BimGeoAlgorithmTool::equal(pPoint->Y(), pData->pPt->Y()))
						&& (!BimGeoAlgorithmTool::equal(regularData.pOtherPt->X(), pData->pPt->X())
							|| !BimGeoAlgorithmTool::equal(regularData.pOtherPt->Y(), pData->pPt->Y())))
					{// 过滤自身
						double dDist = pPoint->Distance(*pData->pPt);
						if (dMinDist < 0.
							|| dDist < dMinDist)
						{
							dMinDist = dDist;
							pCloseData = pData;
						}
					}
				}
			}

			if (0 < dMinDist && NULL != pCloseData)
			{// 在同一线段上不连接线
				if (bIsMerge)
				{
					*pPoint = *pCloseData->pPt;
					mapPointCount[*pCloseData->pPt]++;
				}
				else if (!BimGeoAlgorithmTool::equal(pPoint->X(), pCloseData->pPt->X())
					|| !BimGeoAlgorithmTool::equal(pPoint->Y(), pCloseData->pPt->Y()))
				{
					SGeoCurveInfo newCurve;
					newCurve.ptS = *pPoint;
					newCurve.ptE = *pCloseData->pPt;
					curves.emplace_back(newCurve);

					mapPointCount[*pCloseData->pPt]++;
				}
			}
		}
	}
}

void BimGeoAlgorithmTool::ArcSample(std::vector<gp_Pnt2d>& vecVertex, const gp_Pnt2d& ptStart,
	const gp_Pnt2d& ptEnd, const double& dBulge, const int nNum)
{
	if (nNum < 1)
	{
		return;
	}

	int nMaxNum = nNum;
	double dLength = GetCurveLength(ptStart, ptEnd, dBulge);
	if (dLength / nNum < 1.)
	{
		nMaxNum = (int)dLength + 1;
	}

	gp_Pnt2d ptMid = GetCurveMidPoint(ptStart, ptEnd, dBulge);
	GCE2d_MakeArcOfCircle arc(ptStart, ptMid, ptEnd);

	vecVertex.emplace_back(ptStart);

	double dStartParam = arc.Value()->FirstParameter();
	double dDeltaParam = (arc.Value()->LastParameter() - arc.Value()->FirstParameter()) / nMaxNum;
	for (int i = 1; i < nMaxNum - 1; ++i)
	{
		dStartParam += dDeltaParam;
		gp_Pnt2d pt;
		arc.Value()->D0(dStartParam, pt);

		vecVertex.emplace_back(pt);
	}

	if (dStartParam != arc.Value()->LastParameter())
	{
		vecVertex.emplace_back(ptEnd);
	}
}

std::list<SGeoRange> BimGeoAlgorithmTool::RangeUnion(
	const SGeoRange & rangeA, const SGeoRange& rangeB, const double& dTol)
{
	std::list<SGeoRange> retRanges;

	if (rangeA.dMax < rangeB.dMin)
	{
		if (BimGeoAlgorithmTool::lessThan(rangeA.dMax, rangeB.dMin - dTol))
		{
			//B           **********
			//A ********
			retRanges.emplace_back(rangeA);
			retRanges.emplace_back(rangeB);
		}
		else
		{
			//B           **********
			//A **********
			retRanges.emplace_back(SGeoRange(rangeA.dMin, rangeB.dMax));
		}
	}
	else if (rangeB.dMax < rangeA.dMin)
	{
		if (BimGeoAlgorithmTool::lessThan(rangeB.dMax, rangeA.dMin - dTol))
		{
			//B ********
			//A           **********
			retRanges.emplace_back(rangeA);
			retRanges.emplace_back(rangeB);
		}
		else
		{
			//B **********
			//A           **********
			retRanges.emplace_back(SGeoRange(rangeB.dMin, rangeA.dMax));
		}
	}
	else
	{
		//B ***************
		//A           **********

		//B           **********
		//A ***************

		double dMin = std::min(rangeA.dMin, rangeB.dMin);
		double dMax = std::max(rangeA.dMax, rangeB.dMax);

		retRanges.emplace_back(SGeoRange(dMin, dMax));
	}

	return retRanges;
}

std::list<SGeoRange> BimGeoAlgorithmTool::RangeUnion(const std::list<SGeoRange>& ranges, const double & dTol)
{
	std::vector<SGeoRange> midRanges;
	midRanges.insert(midRanges.end(), ranges.begin(), ranges.end());

	std::list<SGeoRange> retRanges;
	while (midRanges.size() > 0)
	{
		int nCurIndex = 0;

		bool isAddToResult = false;
		for (int nNexIndex = midRanges.size() - 1; nNexIndex > nCurIndex; nNexIndex--)
		{
			std::list<SGeoRange> tmpRanges = RangeUnion(midRanges[nCurIndex], midRanges[nNexIndex], dTol);
			if (tmpRanges.size() >= 2)
			{
				// 没能合并	继续尝试
				continue;
			}
			else
			{
				// 能合并
				isAddToResult = true;
				midRanges.erase(midRanges.begin() + nNexIndex);
				midRanges.erase(midRanges.begin() + nCurIndex);
				midRanges.insert(midRanges.end(), tmpRanges.begin(), tmpRanges.end());
				break;
			}
		}

		if (!isAddToResult)
		{
			retRanges.emplace_back(midRanges[nCurIndex]);
			midRanges.erase(midRanges.begin() + nCurIndex);
		}
	}

	return retRanges;
}

bool BimGeoAlgorithmTool::RangeIntersect(SGeoRange & interRange,
	const SGeoRange & rangeA, const SGeoRange & rangeB,
	const double & dTol)
{
	if (rangeA.dMin <= rangeB.dMax && rangeB.dMax <= rangeA.dMax)
	{
		// B_max in A

		if (rangeB.dMin < rangeA.dMin)
		{
			//B ********
			//A      **********
			interRange = SGeoRange(rangeA.dMin, rangeB.dMax);
		}
		else
		{
			//B     ********
			//A  **************
			interRange = rangeB;
		}
	}
	else if (rangeA.dMin <= rangeB.dMin && rangeB.dMin <= rangeA.dMax)
	{
		// B_min in A

		if (rangeA.dMax < rangeB.dMax)
		{
			//B     ****************
			//A  **************
			interRange = SGeoRange(rangeB.dMin, rangeA.dMax);
		}
		else
		{
			//B     ****************
			//A  *********************
			interRange = rangeB;
		}
	}
	else if (rangeB.dMin <= rangeA.dMin && rangeA.dMax <= rangeB.dMax)
	{
		//B  *********************
		//A     ****************
		interRange = rangeA;
	}
	else
	{
		return false;
	}

	// 移除小短线
	if (BimGeoAlgorithmTool::lessThan(interRange.dMax - interRange.dMin, dTol))
	{
		return false;
	}

	return true;
}

std::list<SGeoRange> BimGeoAlgorithmTool::RangeIntersect(const SGeoRange & rangeA, const std::list<SGeoRange>& rangeBs, const double & dTol)
{
	std::list<SGeoRange> tmpRangeBs = RangeUnion(rangeBs, dTol);

	std::list<SGeoRange> midRanges;

	SGeoRange tmpRange(0., 0.);
	for (std::list<SGeoRange>::const_iterator itTmpRangeB = tmpRangeBs.begin();
		itTmpRangeB != tmpRangeBs.end(); ++itTmpRangeB)
	{
		if (RangeIntersect(tmpRange, rangeA, *itTmpRangeB, dTol))
		{
			midRanges.emplace_back(tmpRange);
		}
	}

	std::list<SGeoRange> retRanges = RangeUnion(midRanges, dTol);

	return retRanges;
}

std::list<SGeoRange> BimGeoAlgorithmTool::RangeIntersect(const std::list<SGeoRange>& rangeAs, const std::list<SGeoRange>& rangeBs, const double & dTol)
{
	std::list<SGeoRange> tmpRangeAs = RangeUnion(rangeAs, dTol);
	std::list<SGeoRange> tmpRangeBs = RangeUnion(rangeBs, dTol);

	std::list<SGeoRange> midRanges;
	for (std::list<SGeoRange>::const_iterator itTmpRangeA = tmpRangeAs.begin();
		itTmpRangeA != tmpRangeAs.end(); ++itTmpRangeA)
	{
		std::list<SGeoRange> tmpRanges = RangeIntersect(*itTmpRangeA, tmpRangeBs, dTol);
		midRanges.insert(midRanges.end(), tmpRanges.begin(), tmpRanges.end());
	}

	std::list<SGeoRange> retRanges = RangeUnion(midRanges, dTol);

	return retRanges;
}

std::list<SGeoRange> BimGeoAlgorithmTool::RangeSubtract(
	const SGeoRange & rangeA, const SGeoRange& rangeB, const double& dTol)
{
	std::list<SGeoRange> retRanges;
	if (rangeA.dMin <= rangeB.dMax && rangeB.dMax <= rangeA.dMax)
	{
		// B_max in A

		if (rangeB.dMin < rangeA.dMin)
		{
			//B ********
			//A      **********
			retRanges.emplace_back(SGeoRange(rangeB.dMax, rangeA.dMax));
		}
		else
		{
			//B     ********
			//A  **************
			retRanges.emplace_back(SGeoRange(rangeA.dMin, rangeB.dMin));
			retRanges.emplace_back(SGeoRange(rangeB.dMax, rangeA.dMax));
		}
	}
	else if (rangeA.dMin <= rangeB.dMin && rangeB.dMin <= rangeA.dMax)
	{
		// B_min in A

		if (rangeA.dMax < rangeB.dMax)
		{
			//B     ****************
			//A  **************
			retRanges.emplace_back(SGeoRange(rangeA.dMin, rangeB.dMin));
		}
		else
		{
			//B     ****************
			//A  *********************
			retRanges.emplace_back(SGeoRange(rangeA.dMin, rangeB.dMin));
			retRanges.emplace_back(SGeoRange(rangeB.dMax, rangeA.dMax));
		}
	}
	else if (rangeB.dMin <= rangeA.dMin && rangeA.dMax <= rangeB.dMax)
	{
		//B  *********************
		//A     ****************
	}
	else
	{
		//B  ******
		//A          ***********

		//B          ***********
		//A  ******
		retRanges.emplace_back(rangeA);
	}

	return retRanges;
}

std::list<SGeoRange> BimGeoAlgorithmTool::RangeSubtract(const SGeoRange & rangeA, const std::list<SGeoRange>& rangeBs, const double & dTol)
{
	std::list<SGeoRange> midRanges;
	for (std::list<SGeoRange>::const_iterator itRangeB = rangeBs.begin();
		itRangeB != rangeBs.end(); ++itRangeB)
	{
		std::list<SGeoRange> tmpRanges = RangeSubtract(rangeA, *itRangeB, dTol);
		midRanges.insert(midRanges.end(), tmpRanges.begin(), tmpRanges.end());
	}

	std::list<SGeoRange> retRanges = RangeUnion(midRanges, dTol);

	return retRanges;
}

std::list<SGeoRange> BimGeoAlgorithmTool::RangeSubtract(const std::list<SGeoRange>& rangeAs, const std::list<SGeoRange>& rangeBs, const double & dTol)
{
	std::list<SGeoRange> tmpRangeAs = RangeUnion(rangeAs, dTol);
	std::list<SGeoRange> tmpRangeBs = RangeUnion(rangeBs, dTol);

	std::list<SGeoRange> midRanges;
	for (std::list<SGeoRange>::const_iterator itTmpRangeA = tmpRangeAs.begin();
		itTmpRangeA != tmpRangeAs.end(); ++itTmpRangeA)
	{
		std::list<SGeoRange> tmpRanges = RangeSubtract(*itTmpRangeA, tmpRangeBs, dTol);
		midRanges.insert(midRanges.end(), tmpRanges.begin(), tmpRanges.end());
	}

	std::list<SGeoRange> retRanges = RangeUnion(midRanges, dTol);

	return retRanges;
}

std::list<SGeoRange> BimGeoAlgorithmTool::RangeFilter(const std::list<SGeoRange>& ranges, const double & dTol)
{
	std::list<SGeoRange> retRanges = ranges;

	// 移除小短线
	retRanges.remove_if([&](const SGeoRange& range) -> bool
	{
		double length = range.dMax - range.dMin;
		return BimGeoAlgorithmTool::lessThan(length, dTol);
	});

	return retRanges;
}

bool BimGeoAlgorithmTool::IsOutWithOBB(const gp_Pnt2d& ptCenter, const double& dWidth, const double& dHeight,
	const double& dAngle, const gp_Pnt2d& ptRefCenter, const double& dRefWidth,
	const double& dRefHeight, const double& dRefAngle, const double& dTol)
{
	Bnd_Box box = OBBToAABB(ptCenter, dWidth, dHeight, dAngle);
	Bnd_Box boxRef = OBBToAABB(ptRefCenter, dRefWidth, dRefHeight, dRefAngle);
	box.SetGap(dTol);

	bool bRet = box.IsOut(boxRef);
	if (!bRet)
	{
		int nIndex = 0;
		if (!equal(dAngle, 0.))
		{
			nIndex = 1;
		}

		if (!equal(dRefAngle, 0.))
		{
			nIndex += 2;
		}

		switch (nIndex)
		{
		case 1:
		{
			box = OBBToAABB(gp_Pnt2d(0., 0.), dWidth, dHeight, 0.);
			gp_Pnt2d ptCalc = ptRefCenter.Coord() - ptCenter.Coord();
			gp_Trsf2d trsf;
			trsf.SetRotation(gp_Pnt2d(0., 0.), -dAngle);
			ptCalc.Transform(trsf);
			boxRef = OBBToAABB(ptCalc, dRefWidth, dRefHeight, dRefAngle - dAngle);
			box.SetGap(dTol);

			bRet = box.IsOut(boxRef);
		}
		break;
		case 2:
		{
			gp_Pnt2d ptCalc = ptCenter.Coord() - ptRefCenter.Coord();
			gp_Trsf2d trsf;
			trsf.SetRotation(gp_Pnt2d(0., 0.), -dRefAngle);
			ptCalc.Transform(trsf);
			box = OBBToAABB(ptCalc, dWidth, dHeight, dAngle - dRefAngle);
			boxRef = OBBToAABB(gp_Pnt2d(0., 0.), dRefWidth, dRefHeight, 0.);
			box.SetGap(dTol);

			bRet = box.IsOut(boxRef);
		}
		break;
		case 3:
		{
			box = OBBToAABB(gp_Pnt2d(0., 0.), dWidth, dHeight, 0.);

			gp_Pnt2d ptCalc = ptRefCenter.Coord() - ptCenter.Coord();
			gp_Trsf2d trsf;
			trsf.SetRotation(gp_Pnt2d(0., 0.), -dAngle);
			ptCalc.Transform(trsf);
			boxRef = OBBToAABB(ptCalc, dRefWidth, dRefHeight, dRefAngle - dAngle);
			box.SetGap(dTol);

			bRet = box.IsOut(boxRef);
			if (!bRet)
			{
				ptCalc = ptCenter.Coord() - ptRefCenter.Coord();
				trsf.SetRotation(gp_Pnt2d(0., 0.), -dRefAngle);
				ptCalc.Transform(trsf);
				box = OBBToAABB(ptCalc, dWidth, dHeight, dAngle - dRefAngle);
				boxRef = OBBToAABB(gp_Pnt2d(0., 0.), dRefWidth, dRefHeight, 0.);
				box.SetGap(dTol);

				bRet = box.IsOut(boxRef);
			}
		}
		break;
		default:
			break;
		}
	}

	return bRet;
}

Bnd_Box BimGeoAlgorithmTool::OBBToAABB(const gp_Pnt2d& ptCenter, const double& dWidth, const double& dHeight, const double& dAngle)
{
	gp_Vec2d vecX(dWidth * 0.5, 0.), vecY(0., dHeight * 0.5);
	Bnd_Box box;

	if (!equal(dAngle, 0.))
	{
		vecX.Rotate(dAngle);
		vecY.Rotate(dAngle);
	}

	gp_Pnt2d ptTemp = ptCenter.Coord() + vecX.XY() + vecY.XY();
	box.Add(gp_Pnt(ptTemp.X(), ptTemp.Y(), 0.));

	ptTemp = ptCenter.Coord() + vecX.XY() - vecY.XY();
	box.Add(gp_Pnt(ptTemp.X(), ptTemp.Y(), 0.));

	ptTemp = ptCenter.Coord() - vecX.XY() + vecY.XY();
	box.Add(gp_Pnt(ptTemp.X(), ptTemp.Y(), 0.));

	ptTemp = ptCenter.Coord() - vecX.XY() - vecY.XY();
	box.Add(gp_Pnt(ptTemp.X(), ptTemp.Y(), 0.));

	return box;
}
