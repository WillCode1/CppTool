#include "BimGeoPolylineTool.h"
#include "BimGeoPolylineOffset.h"
#include "BimGeoAlgorithmTool.h"
#include "GCE2d_MakeCircle.hxx"
#include "BimSearchPolygonTool.h"
#include "BimGeoTwoPolygonBoolOper.h"
#include <math.h>
#include "gp_Pnt2d.hxx"
#include "gp_Circ2d.hxx"
#include <vector>
#include "QuadTreeNode.h"
#include "BimGeoOBB2d.h"

#ifdef GEO_CXX11_IS_SUPPORTED // C++ 11 标准
#include "delaunay/CDT.h"
#endif

bool BimGeoPolylineTool::OffsetPolyline(std::list<CBimGeoPolyline>& polylines, const CBimGeoPolyline& orgPolyline,
	const double& dOffset, const double& dTol)
{
	CBimGeoPolylineOffset offsetPoly;
	bool bRet = offsetPoly.DoIt(orgPolyline, dOffset, dTol);
	polylines = offsetPoly.GetOffsetPolylines();
	return bRet;
}

bool BimGeoPolylineTool::OffsetPolyline(std::list<CBimGeoPolyline>& polylines, const CBimGeoPolyline& orgPolyline,
	const std::vector<double>& vecOffset, const double& dTol)
{
	CBimGeoPolylineOffset offsetPoly;
	bool bRet = offsetPoly.DoIt(orgPolyline, vecOffset, dTol);
	polylines = offsetPoly.GetOffsetPolylines();
	return bRet;
}

EPointWidthPolygonRel BimGeoPolylineTool::CalcPointWithPolygonRelation(const gp_Pnt2d& pt,
	const CBimGeoPolyline& polyline, const double& dTol)
{// 射线法
	Bnd_Box2d box = polyline.GetBndBox();
	if (box.IsVoid())
	{
		return EPointWidthPolygonRel::eOutPolygon;
	}

	box.SetGap(dTol);
	if (box.IsOut(pt))
	{
		return EPointWidthPolygonRel::eOutPolygon;
	}

	double dXmax = 0.;
	{
		double dXmin = 0., dYmin = 0., dYMax = 0.;
		box.Get(dXmin, dYmin, dXmax, dYMax);

		dXmax += 100.;
	}

	std::vector<int> vecIntersectSide;
	std::set<double> ySet;

	for (int i = 0; i < polyline.GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polyline.GetCurve(ptS, ptE, dBulge, i);

		if (BimGeoAlgorithmTool::IsOn(pt, ptS, ptE, dBulge, dTol))
		{
			return EPointWidthPolygonRel::eOnPolygon;
		}

		Bnd_Box2d curveBox = BimGeoAlgorithmTool::GetCurveBndBox(ptS, ptE, dBulge);
		double dXmaxRef = 0., dXminRef = 0., dYminRef = 0., dYMaxRef = 0.;
		{
			curveBox.Get(dXminRef, dYminRef, dXmaxRef, dYMaxRef);
		}

		if (BimGeoAlgorithmTool::lessEqualThan(pt.X(), dXmaxRef))
		{
			ySet.insert(ptS.Y());
			ySet.insert(ptE.Y());
			ySet.insert(dYminRef);
			ySet.insert(dYMaxRef);

			vecIntersectSide.emplace_back(i);
		}
	}

	if (ySet.size() < 2)
	{
		return EPointWidthPolygonRel::eOutPolygon;
	}

	std::set<double>::iterator itBegin = ySet.begin();
	double dBeginY = *itBegin, dEndY = *++itBegin;
	gp_Pnt2d ptMax(dXmax, (dBeginY + dEndY) * 0.5);

	if (dEndY - dBeginY < 1.)
	{
		while (++itBegin != ySet.end())
		{
			dBeginY = dEndY;
			dEndY = *itBegin;

			if (1. < dEndY - dBeginY)
			{
				ptMax.SetY((dEndY - dBeginY) * 0.5);
				break;
			}
			else
			{
				double dValue = (dEndY - dBeginY) * 0.5;
				if (ptMax.Y() < dValue)
				{
					ptMax.SetY(dValue);
				}
			}
		}
	}

	int nIntersectPtCount = 0.;
	for (int i = 0; i < vecIntersectSide.size(); ++i)
	{
		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polyline.GetCurve(ptS, ptE, dBulge, vecIntersectSide[i]);

		std::vector<gp_Pnt2d> vecIntersectPt;
		BimGeoAlgorithmTool::CalcIntersect(vecIntersectPt, ptS, ptE, dBulge, pt, ptMax, 0.);

		nIntersectPtCount += vecIntersectPt.size();
	}

	if (0 == nIntersectPtCount % 2)
	{
		return EPointWidthPolygonRel::eOutPolygon;
	}

	return EPointWidthPolygonRel::eInPolygon;
}

EPointWidthPolygonRel BimGeoPolylineTool::CalcPointWithPolygonRelation(const gp_Pnt& pt,
	const CBimGeoPolyline& polyline, const double& dTol)
{
	return  CalcPointWithPolygonRelation(gp_Pnt2d(pt.X(), pt.Y()), polyline, dTol);
}

EPolygonRel CalcTwoPolygonRelationImp(const CBimGeoPolyline* pCalcPloyline, const CBimGeoPolyline* pCalcRefPolyline, const double& dTol)
{
	int nOnPolySide = 0, nInPolySide = 0, nOutPolySide = 0;
	for (int i = 0; i < pCalcPloyline->GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		pCalcPloyline->GetCurve(ptS, ptE, dBulge, i);

		std::vector<gp_Pnt2d> intersectPts;
		BimGeoPolylineTool::GetIntersect(intersectPts, *pCalcRefPolyline, ptS, ptE, dBulge, dTol);

		intersectPts.emplace_back(ptS);
		intersectPts.emplace_back(ptE);
		if (2 < intersectPts.size())
		{
			intersectPts = BimGeoAlgorithmTool::SortByCurve(ptS, ptE, dBulge, intersectPts);
		}

		for (int i = 0; i < intersectPts.size(); ++i)
		{
			int j = i + 1;
			if (j == intersectPts.size())
			{
				break;
			}

			gp_Pnt2d ptMid((intersectPts[i].Coord() + intersectPts[j].Coord()) * 0.5);
			if (BimGeoAlgorithmTool::IsArc(dBulge))
			{
				ptMid = BimGeoAlgorithmTool::GetArcMidPoint(intersectPts[i], intersectPts[j],
					ptS, ptE, dBulge);
			}

			std::list<gp_Pnt2d*> pts;
			pts.emplace_back(&intersectPts[i]);
			pts.emplace_back(&intersectPts[j]);
			pts.emplace_back(&ptMid);

			for (std::list<gp_Pnt2d*>::iterator itPt = pts.begin(); itPt != pts.end(); ++itPt)
			{
				EPointWidthPolygonRel ePointRet = BimGeoPolylineTool::CalcPointWithPolygonRelation(**itPt, *pCalcRefPolyline, dTol);
				switch (ePointRet)
				{
					case EPointWidthPolygonRel::eOnPolygon:
						++nOnPolySide;
						break;
					case EPointWidthPolygonRel::eInPolygon:
						++nInPolySide;
						break;
					case EPointWidthPolygonRel::eOutPolygon:
						++nOutPolySide;
						break;
					default:
						break;
				}
			}
		}
	}

	EPolygonRel eRet = EPolygonRel::eOuter;
	switch (nInPolySide)
	{
		case 0:
		{
			switch (nOutPolySide)
			{
				case 0:
				{
					if (0 < nOnPolySide)
					{
						eRet = EPolygonRel::eOverlap;
					}
				}
				break;
				default:
				{
					if (0 < nOnPolySide)
					{
						eRet = EPolygonRel::eTagent;
					}
				}
				break;
			}
		}
		break;
		default:
		{
			if (0 < nOutPolySide)
			{
				eRet = EPolygonRel::eIntersect;
			}
			else
			{
				eRet = EPolygonRel::eAInB;
				if (0 < nOnPolySide)
				{
					eRet = EPolygonRel::eAInBWithTagent;
				}
			}
		}
		break;
	}

	return eRet;
};

EPolygonRel BimGeoPolylineTool::CalcTwoPolygonRelation(const CBimGeoPolyline& polylineA,
	const CBimGeoPolyline& polylineB, const double& dTol)
{
	Bnd_Box2d boxA = polylineA.GetBndBox();
	boxA.SetGap(dTol);

	Bnd_Box2d boxB = polylineB.GetBndBox();
	if (boxA.IsOut(boxB))
	{
		return EPolygonRel::eOuter;
	}

	EPolygonRel eRet = CalcTwoPolygonRelationImp(&polylineA, &polylineB, dTol);
	if (EPolygonRel::eTagent == eRet
		|| EPolygonRel::eOuter == eRet)
	{
		eRet = CalcTwoPolygonRelationImp(&polylineB, &polylineA, dTol);
		switch (eRet)
		{
			case EPolygonRel::eAInB:
				eRet = EPolygonRel::eBInA;
				break;
			case EPolygonRel::eBInA:
				eRet = EPolygonRel::eAInB;
				break;
			case EPolygonRel::eAInBWithTagent:
				eRet = EPolygonRel::eBInAWithTagent;
				break;
			case EPolygonRel::eBInAWithTagent:
				eRet = EPolygonRel::eAInBWithTagent;
				break;
			default:
				break;
		}
	}

	return eRet;
}

gp_Pnt2d BimGeoPolylineTool::GetClosePt(std::set<int>& indexSet, const gp_Pnt2d& pt, const CBimGeoPolyline& polyline, const double& dTol)
{
	return GetClosePtExt(indexSet, pt, polyline, std::set<int>(), dTol);
}

gp_Pnt2d BimGeoPolylineTool::GetClosePtExt(std::set<int>& indexSet, const gp_Pnt2d& pt,
	const CBimGeoPolyline& polyline, const std::set<int>& filterIndexSet, const double& dTol)
{
	gp_Pnt2d ptClose;
	double dDist = -1.;

	for (int i = 0; i < polyline.GetCurveNumber(); ++i)
	{
		if (filterIndexSet.end() != filterIndexSet.find(i))
		{
			continue;
		}

		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polyline.GetCurve(ptS, ptE, dBulge, i);

		gp_Pnt2d ptTemp = BimGeoAlgorithmTool::GetClosePoint(pt, ptS, ptE, dBulge, dTol);

		double dValue = pt.Distance(ptTemp);
		if (dDist < 0.
			|| BimGeoAlgorithmTool::lessEqualThan(dValue, dDist))
		{
			if (!(ptTemp.IsEqual(ptS, dTol) || ptTemp.IsEqual(ptE, dTol)))
			{// 顶点时
				indexSet.clear();
			}

			dDist = dValue;
			ptClose = ptTemp;
			indexSet.insert(i);
		}
	}

	return ptClose;
}

gp_Pnt2d BimGeoPolylineTool::GetCenterPt(const CBimGeoPolyline& polyline)
{
	CBimGeoPolyline calcPolyline = polyline;
	calcPolyline.Sample();
	int len = calcPolyline.GetVertexes().size();
	if (len < 3)
	{
		return gp_Pnt2d();
	}

	gp_Pnt2d pt1 = calcPolyline.GetVertexes()[0];
	gp_Pnt2d pt2 = calcPolyline.GetVertexes()[1];

	double dX = 0., dY = 0., dSumArea = 0.;
	for (int i = 2; i < len; ++i)
	{
		gp_Pnt2d pt3 = calcPolyline.GetVertexes()[i];
		double dArea = GetArea(pt1, pt2, pt3);
		if (0 == dArea)
		{
			continue;
		}

		dSumArea += dArea;

		dX += (pt1.X() + pt2.X() + pt3.X()) * dArea;
		dY += (pt1.Y() + pt2.Y() + pt3.Y()) * dArea;

		pt1 = pt2;
	}

	if (0 == dSumArea)
	{
		return gp_Pnt2d();
	}

	dSumArea *= 3;

	return gp_Pnt2d(dX / dSumArea, dY / dSumArea);
}

double BimGeoPolylineTool::GetArea(const gp_Pnt2d& pt1, const gp_Pnt2d& pt2, const gp_Pnt2d& pt3)
{
	gp_Vec2d vec1(pt1, pt2), vec2(pt2, pt3);
	double dArea = vec1.Crossed(vec2) * 0.5;

	return abs(dArea);
}

double BimGeoPolylineTool::GetArea(const gp_Pnt& pt1, const gp_Pnt& pt2, const gp_Pnt& pt3)
{
	gp_Vec vec1(pt1, pt2), vec2(pt1, pt3);
	double dLength = vec1.Magnitude();
	if (0. == dLength)
	{
		return 0.;
	}

	double dDot = vec1.Dot(vec2) / dLength;
	vec1.Normalize();

	gp_Pnt ptPerp(pt1.Coord() + vec1.XYZ() * dDot);
	double dHight = pt3.Distance(ptPerp);

	return abs(dHight * dLength * 0.5);
}

double BimGeoPolylineTool::GetArea(const gp_Pnt& pt1, const gp_Pnt& pt2,
	const gp_Pnt& pt3, const gp_Vec& normal)
{
	gp_Vec vec1(pt1, pt2), vec2(pt1, pt3);
	double dLength1 = vec1.Magnitude();
	double dLength2 = vec2.Magnitude();
	if (BimGeoAlgorithmTool::equal(0., dLength1)
		|| BimGeoAlgorithmTool::equal(0., dLength2))
	{
		return 0.;
	}

	double dAngle = vec1.AngleWithRef(vec2, normal);
	return dLength1 * dLength2 * sin(dAngle) * 0.5;
}

std::list<std::set<int>> BimGeoPolylineTool::GetParallelSide(const CBimGeoPolyline& polyline)
{
	std::list<std::set<int>> retParallelSides;

	std::set<int> useIndex;
	for (int i = 0; i < polyline.GetCurveNumber(); ++i)
	{
		if (useIndex.end() != useIndex.find(i))
		{
			continue;
		}

		useIndex.insert(i);

		std::set<int> parallelSideSet;
		parallelSideSet.insert(i);

		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polyline.GetCurve(ptS, ptE, dBulge, i);

		for (int j = i + 1; j < polyline.GetCurveNumber(); ++j)
		{
			if (useIndex.end() != useIndex.find(j))
			{
				continue;
			}

			gp_Pnt2d ptSRef, ptERef;
			double dBulgeRef = 0.;
			polyline.GetCurve(ptSRef, ptERef, dBulgeRef, j);

			if (BimGeoAlgorithmTool::IsParallel(ptS, ptE, 0., ptSRef, ptERef, 0.))
			{
				useIndex.insert(j);
				parallelSideSet.insert(j);
			}
		}

		if (1 < parallelSideSet.size())
		{
			retParallelSides.emplace_back(parallelSideSet);
		}
	}

	return retParallelSides;
}

bool BimGeoPolylineTool::IsHasIntersect(const CBimGeoPolyline& polyline, const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dBulge)
{
	Bnd_Box2d box = polyline.GetBndBox();
	Bnd_Box2d boxCurve = BimGeoAlgorithmTool::GetCurveBndBox(ptS, ptE, dBulge);
	if (box.IsVoid() || boxCurve.IsVoid() || box.IsOut(boxCurve))
	{
		return false;
	}

	for (int i = 0; i < polyline.GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptSRef, ptERef;
		double dBulgeRef = 0.;
		polyline.GetCurve(ptSRef, ptERef, dBulgeRef, i);

		Bnd_Box2d boxRef = BimGeoAlgorithmTool::GetCurveBndBox(ptSRef, ptERef, dBulgeRef);
		if (boxRef.IsVoid()
			|| boxCurve.IsOut(boxRef))
		{
			continue;
		}

		std::vector<gp_Pnt2d> vecIntersectPt;
		BimGeoAlgorithmTool::CalcIntersect(vecIntersectPt, ptS, ptE, dBulge, ptSRef, ptERef, dBulgeRef);
		if (0 < vecIntersectPt.size())
		{
			return true;
		}
	}

	return false;
}

void BimGeoPolylineTool::GetIntersect(std::vector<gp_Pnt2d>& intersectPts, const CBimGeoPolyline& polyline,
	const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dBulge, const double& dTol)
{
	Bnd_Box2d box = polyline.GetBndBox();
	Bnd_Box2d boxCurve = BimGeoAlgorithmTool::GetCurveBndBox(ptS, ptE, dBulge);
	if (box.IsVoid() || boxCurve.IsVoid())
	{
		return;
	}

	box.SetGap(dTol);
	if (box.IsOut(boxCurve))
	{
		return;
	}

	boxCurve.SetGap(dTol);
	for (int i = 0; i < polyline.GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptSRef, ptERef;
		double dBulgeRef = 0.;
		polyline.GetCurve(ptSRef, ptERef, dBulgeRef, i);

		Bnd_Box2d boxRef = BimGeoAlgorithmTool::GetCurveBndBox(ptSRef, ptERef, dBulgeRef);
		if (boxRef.IsVoid()
			|| boxCurve.IsOut(boxRef))
		{
			continue;
		}

		std::vector<gp_Pnt2d> vecIntersectPt;
		BimGeoAlgorithmTool::CalcIntersect(vecIntersectPt, ptS, ptE, dBulge, ptSRef, ptERef, dBulgeRef, dTol);
		for (int i = 0; i < vecIntersectPt.size(); ++i)
		{
			bool bIsAdd = true;
			for (int j = 0; j < intersectPts.size(); ++j)
			{
				if (intersectPts[j].IsEqual(vecIntersectPt[i], dTol))
				{
					bIsAdd = false;
					break;
				}
			}

			if (bIsAdd)
			{
				intersectPts.emplace_back(vecIntersectPt[i]);
			}
		}
	}
}

std::list<CBimGeoPolyline> BimGeoPolylineTool::SearchMaxPolygonWithBreakCurve(const std::list<SGeoCurveInfo>& curves,
	const double& dTol, const bool IsVectorCurve, const double& dAngleTol)
{
	CBimSearchPolygonTool tool;
	tool.DoIt(curves, dTol, ESearchType::eMaxPolygon, true, IsVectorCurve, dAngleTol);

	return tool.GetMaxPolygon();
}

std::list<CBimGeoPolyline> BimGeoPolylineTool::SearchMaxPolygonWithNoBreakCurve(const std::list<SGeoCurveInfo>& curves,
	const double& dTol, const bool IsVectorCurve, const double& dAngleTol)
{
	CBimSearchPolygonTool tool;
	tool.DoIt(curves, dTol, ESearchType::eMaxPolygon, false, IsVectorCurve, dAngleTol);

	return tool.GetMaxPolygon();
}

std::list<CBimGeoPolyline> BimGeoPolylineTool::SearchMinPolygonWithBreakCurve(const std::list<SGeoCurveInfo>& curves,
	const double& dTol, const bool IsVectorCurve, const double& dAngleTol)
{
	CBimSearchPolygonTool tool;
	tool.DoIt(curves, dTol, ESearchType::eMinPolygon, true, IsVectorCurve, dAngleTol);

	return tool.GetMinPolygon();
}

std::list<CBimGeoPolyline> BimGeoPolylineTool::SearchMinPolygonWithNoBreakCurve(const std::list<SGeoCurveInfo>& curves,
	const double& dTol, const bool IsVectorCurve, const double& dAngleTol)
{
	CBimSearchPolygonTool tool;
	tool.DoIt(curves, dTol, ESearchType::eMinPolygon, false, IsVectorCurve, dAngleTol);

	return tool.GetMinPolygon();
}

void BimGeoPolylineTool::MergePolylineSide(CBimGeoPolyline& polyline, const double& dTol)
{
	int nIndex = 0;
	while (nIndex < polyline.GetCurveNumber())
	{
		int nNextIndex = nIndex + 1;
		if (nNextIndex == polyline.GetCurveNumber())
		{
			if (!polyline.IsClose())
			{
				break;
			}

			nNextIndex = 0;
		}


		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polyline.GetCurve(ptS, ptE, dBulge, nIndex);

		gp_Pnt2d ptSRef, ptERef;
		double dBulgeRef = 0.;
		polyline.GetCurve(ptSRef, ptERef, dBulgeRef, nNextIndex);

		if (BimGeoAlgorithmTool::IsParallel(ptS, ptE, dBulge, ptSRef, ptERef, dBulgeRef, dTol)
			&& !ptS.IsEqual(ptERef, dTol))
		{
			if (BimGeoAlgorithmTool::IsArc(dBulge))
			{
				polyline.SetBugle(nIndex, BimGeoAlgorithmTool::CalcArcBulge(ptS, ptERef, ptS, ptE, dBulge));
			}

			polyline.RemoveVertex(nNextIndex);
			continue;
		}

		++nIndex;
	}
}

void BimGeoPolylineTool::RemovePoint(CBimGeoPolyline& polyline, const double& dTol)
{
	int nIndex = 0;
	while (nIndex < polyline.GetVertexes().size())
	{
		int nPreIndex = nIndex - 1;
		if (nPreIndex < 0)
		{
			if (polyline.IsClose())
			{
				nPreIndex = polyline.GetVertexes().size() - 1;
			}
			else
			{
				++nIndex;
				continue;
			}
		}

		if (polyline.GetVertexes()[nIndex].IsEqual(polyline.GetVertexes()[nPreIndex], dTol))
		{
			polyline.RemoveVertex(nIndex);
			continue;
		}

		++nIndex;
	}
}

bool BimGeoPolylineTool::IsSelfIntersect(const CBimGeoPolyline& polyline, const double& dTol)
{
	for (int i = 0; i < polyline.GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polyline.GetCurve(ptS, ptE, dBulge, i);

		Bnd_Box2d box = BimGeoAlgorithmTool::GetCurveBndBox(ptS, ptE, dBulge);
		box.SetGap(dTol);

		for (int j = i + 1; j < polyline.GetCurveNumber(); ++j)
		{
			gp_Pnt2d ptSRef, ptERef;
			double dBulgeRef = 0.;
			polyline.GetCurve(ptSRef, ptERef, dBulgeRef, j);

			Bnd_Box2d boxRef = BimGeoAlgorithmTool::GetCurveBndBox(ptS, ptE, dBulge);
			if (box.IsOut(boxRef))
			{
				continue;
			}

			std::vector<gp_Pnt2d> vecIntersectPt;
			BimGeoAlgorithmTool::CalcIntersect(vecIntersectPt, ptS, ptE, dBulge, ptSRef, ptERef, dBulgeRef, dTol);

			for (int i = 0; i < vecIntersectPt.size(); ++i)
			{
				if (!vecIntersectPt[i].IsEqual(ptS, dTol) && !vecIntersectPt[i].IsEqual(ptE, dTol))
				{
					return true;
				}
				else if (!vecIntersectPt[i].IsEqual(ptSRef, dTol) && !vecIntersectPt[i].IsEqual(ptERef, dTol))
				{
					return true;
				}
			}
		}
	}

	return false;
}

void BimGeoPolylineTool::InterceptPolyline(std::list<CBimGeoPolyline>& retPolies, const CBimGeoPolyline& polyline,
	const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dTol)
{
	if (ptS.IsEqual(ptE, dTol))
	{
		retPolies.emplace_back(polyline);
		return;
	}

	bool bStartUse = false, bEndUse = false;
	while (!bStartUse || !bEndUse)
	{
		CBimGeoPolyline retpolyline;
		for (int i = 0; i < polyline.GetCurveNumber(); ++i)
		{
			gp_Pnt2d ptSRef, ptERef;
			double dBulgeRef = 0.;
			polyline.GetCurve(ptSRef, ptERef, dBulgeRef, i);

			if (!bStartUse)
			{
				if (ptS.IsEqual(ptSRef, dTol))
				{
					retpolyline.AddVertex(ptS, dBulgeRef);

					bStartUse = true;
				}
				else if (ptS.IsEqual(ptERef, dTol))
				{
					bStartUse = true;
				}
				else if (BimGeoAlgorithmTool::IsOn(ptS, ptSRef, ptERef, dBulgeRef, dTol))
				{
					double dBulge = 0.;
					if (BimGeoAlgorithmTool::IsArc(dBulgeRef))
					{
						dBulge = BimGeoAlgorithmTool::CalcArcBulge(ptS, ptERef,
							ptSRef, ptERef, dBulgeRef, dTol);
					}

					retpolyline.AddVertex(ptS, dBulge);

					bStartUse = true;
				}

				if (bStartUse && 0 < retpolyline.GetVertexes().size())
				{
					if (BimGeoAlgorithmTool::IsOn(ptE, ptSRef, ptERef, dBulgeRef, dTol))
					{
						if (BimGeoAlgorithmTool::IsArc(dBulgeRef))
						{
							double dBulge = BimGeoAlgorithmTool::CalcArcBulge(*retpolyline.GetVertexes().rbegin(), ptE,
								ptSRef, ptERef, dBulgeRef, dTol);

							retpolyline.SetVertex(retpolyline.GetVertexes().size() - 1, *retpolyline.GetVertexes().rbegin(), dBulge);
						}

						retpolyline.AddVertex(ptE, 0.);

						bEndUse = true;
						break;
					}
				}
			}
			else
			{// 起点已经使用
				if (BimGeoAlgorithmTool::IsOn(ptE, ptSRef, ptERef, dBulgeRef, dTol))
				{
					double dBulge = 0.;
					if (BimGeoAlgorithmTool::IsArc(dBulgeRef))
					{
						dBulge = BimGeoAlgorithmTool::CalcArcBulge(ptS, ptERef,
							ptSRef, ptERef, dBulgeRef, dTol);
					}

					retpolyline.AddVertex(ptSRef, dBulge);

					retpolyline.AddVertex(ptE, 0.);

					bEndUse = true;
					break;
				}
				else
				{
					retpolyline.AddVertex(ptSRef, dBulgeRef);
				}
			}
		}

		if (0 < retpolyline.GetVertexes().size())
		{
			retPolies.emplace_back(retpolyline);
		}

		if (!bStartUse)
		{// 点不在多段线上
			break;
		}
	}
}

void BimGeoPolylineTool::TwoPolygonBooleanOperation(std::list<SBimGeoPolygonWithHole>& retPolygons,
	const CBimGeoPolyline* pPolylineA, const CBimGeoPolyline* pPolylineB,
	const EBooleanOperation eCalcType, const double& dTol)
{
	CBimGeoTwoPolygonBoolOper oper;
	oper.DoIt(pPolylineA, pPolylineB, eCalcType, dTol);

	retPolygons = oper.GetPolygons();
}

double BimGeoPolylineTool::GetPolygonArea(const CBimGeoPolyline& polygon)
{
	if (!polygon.IsClose())
	{
		return 0.;
	}

	double dArea = 0.;
	int len = (int)polygon.GetVertexes().size();
	if (2 < len)
	{
		for (int i = 0; i < len; ++i)
		{
			int j = i + 1;
			if (len == j)
			{
				j = 0;
			}

			dArea += polygon.GetVertexes()[i].X() * polygon.GetVertexes()[j].Y() -
				polygon.GetVertexes()[i].Y() * polygon.GetVertexes()[j].X();
		}

		dArea *= 0.5;
	}

	// 求圆弧的面积
	for (int i = 0; i < polygon.GetCurveNumber(); ++i)
	{
		if (!BimGeoAlgorithmTool::IsArc(polygon.GetBulges()[i]))
		{
			continue;
		}

		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polygon.GetCurve(ptS, ptE, dBulge, i);

		SGeoArcInfo arcInfo = BimGeoAlgorithmTool::GetArcInfo(ptS, ptE, dBulge);
		double dLength = BimGeoAlgorithmTool::GetCurveLength(ptS, ptE, dBulge);
		double dArcArea = dLength * arcInfo.dRadius * 0.5;

		double dTriangleArea = GetArea(arcInfo.ptCenter, ptS, ptE);
		dArcArea -= dTriangleArea;

		if (arcInfo.bIsAntiClockwise)
		{
			dArea += dArcArea;
		}
		else
		{
			dArea -= dArcArea;
		}
	}

	return dArea;
}

#ifdef GEO_CXX11_IS_SUPPORTED // C++ 11 标准
void GetPolygonEdgeImp(std::vector<CDT::Edge>& edges, const std::vector<CDT::V2d<double> >& vertices, const int nStartIndex,
	const CBimGeoPolyline& edgePolygon)
{
	int nCurIndex = nStartIndex;
	for (int i = 0; i < edgePolygon.GetCurveNumber(); ++i, ++nCurIndex)
	{
		int nNext = nCurIndex + 1;
		if (nNext == vertices.size())
		{
			nNext = nStartIndex;
		}

		edges.emplace_back(CDT::Edge(nCurIndex, nNext));
	}
}

void BimGeoPolylineTool::Triangle(std::list<SGeoTriangle>& triangles, const CBimGeoPolyline& polygon, const std::list<CBimGeoPolyline>& holes)
{
	using Triangulation = CDT::Triangulation<double>;

	Triangulation cdt = Triangulation(CDT::FindingClosestPoint::ClosestRandom);

	std::vector<CDT::V2d<double> > vertices;
	for (int i = 0; i < polygon.GetVertexes().size(); ++i)
	{
		CDT::V2d<double> pt;
		pt.x = polygon.GetVertexes()[i].X();
		pt.y = polygon.GetVertexes()[i].Y();

		vertices.emplace_back(pt);
	}

	int nStartIndex = 0;
	std::vector<CDT::Edge> edges;
	GetPolygonEdgeImp(edges, vertices, nStartIndex, polygon);
	nStartIndex = vertices.size();

	for (std::list<CBimGeoPolyline>::const_iterator itHole = holes.begin();
		itHole != holes.end(); ++itHole)
	{
		for (int i = 0; i < itHole->GetVertexes().size(); ++i)
		{
			CDT::V2d<double> pt;
			pt.x = itHole->GetVertexes()[i].X();
			pt.y = itHole->GetVertexes()[i].Y();
			vertices.emplace_back(pt);
		}

		GetPolygonEdgeImp(edges, vertices, nStartIndex, *itHole);

		nStartIndex = vertices.size();
	}

	cdt.insertVertices(vertices);
	cdt.insertEdges(edges);
	cdt.eraseOuterTriangles();

	if (cdt.triangles.size() < 1)
	{
		return;
	}

	// 构建四叉树
	std::list<SGeoTriangle> trianglesTemp;
	Bnd_Box2d box;
	for (int i = 0; i < cdt.triangles.size(); ++i)
	{
		CDT::Triangle& triangle = cdt.triangles[i];
		SGeoTriangle geoTriangle;
		geoTriangle.pt1 = gp_Pnt2d(vertices[triangle.vertices[0]].x, vertices[triangle.vertices[0]].y);
		geoTriangle.pt2 = gp_Pnt2d(vertices[triangle.vertices[1]].x, vertices[triangle.vertices[1]].y);
		geoTriangle.pt3 = gp_Pnt2d(vertices[triangle.vertices[2]].x, vertices[triangle.vertices[2]].y);

		double dArea = GetArea(geoTriangle.pt1, geoTriangle.pt2, geoTriangle.pt3);
		if (BimGeoAlgorithmTool::equal(dArea, 0.))
		{
			continue;
		}

		trianglesTemp.emplace_back(geoTriangle);

		box.Add(geoTriangle.pt1);
		box.Add(geoTriangle.pt2);
		box.Add(geoTriangle.pt3);
	}

	struct SQuadTreeTriangleData
	{
		double dX;
		double dY;
		double dW;
		double dH;
		SGeoTriangle* pTriangle = NULL;
		bool bIsErase = false;
	};

	double dXmin = 0., dYMin = 0., dXmax = 0., dYmax = 0.;
	box.Get(dXmin, dYMin, dXmax, dYmax);

	QuadTreeNode<SQuadTreeTriangleData> quadTreeNode(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin,
		0, 10, QuadType::ROOT, NULL);

	std::list<SQuadTreeTriangleData> quadTreeTriangleDatas;
	for (std::list<SGeoTriangle>::iterator itTriangle = trianglesTemp.begin(); itTriangle != trianglesTemp.end();
		++itTriangle)
	{
		SGeoTriangle& triangle = *itTriangle;
		Bnd_Box2d triangleBox;
		triangleBox.Add(triangle.pt1);
		triangleBox.Add(triangle.pt2);
		triangleBox.Add(triangle.pt3);
		triangleBox.Get(dXmin, dYMin, dXmax, dYmax);

		SQuadTreeTriangleData data;
		data.dX = dXmin;
		data.dY = dYMin;
		data.dW = dXmax - dXmin;
		data.dH = dYmax - dYMin;
		data.pTriangle = &triangle;

		quadTreeTriangleDatas.emplace_back(data);
		quadTreeNode.InsertObject(&*quadTreeTriangleDatas.rbegin());
	}

	// 判断三角形是否在洞内
	for (std::list<CBimGeoPolyline>::const_iterator itHole = holes.begin();
		itHole != holes.end(); ++itHole)
	{
		const CBimGeoPolyline& hole = *itHole;
		Bnd_Box2d holeBox = hole.GetBndBox();
		holeBox.SetGap(0.0001);
		holeBox.Get(dXmin, dYMin, dXmax, dYmax);

		std::list<SQuadTreeTriangleData*> intersectCurveDatas = quadTreeNode.GetObjectsAt(dXmin, dYMin, dXmax - dXmin, dYmax - dYMin);
		for (std::list<SQuadTreeTriangleData*>::const_iterator itData = intersectCurveDatas.begin();
			itData != intersectCurveDatas.end(); ++itData)
		{
			SQuadTreeTriangleData* pData = *itData;
			if (pData->bIsErase)
			{
				continue;
			}

			gp_Pnt2d pt((pData->pTriangle->pt1.X() + pData->pTriangle->pt2.X() + pData->pTriangle->pt3.X()) / 3,
				(pData->pTriangle->pt1.Y() + pData->pTriangle->pt2.Y() + pData->pTriangle->pt3.Y()) / 3);

			EPointWidthPolygonRel eRet = CalcPointWithPolygonRelation(pt, hole);
			if (EPointWidthPolygonRel::eInPolygon == eRet)
			{
				pData->bIsErase = true;
			}
		}
	}

	// 获取有效的三角形
	for (std::list<SQuadTreeTriangleData>::const_iterator itData = quadTreeTriangleDatas.begin();
		itData != quadTreeTriangleDatas.end(); ++itData)
	{
		if (itData->bIsErase)
		{
			continue;
		}

		triangles.emplace_back(*itData->pTriangle);
	}
}
#endif

gp_Pnt2d BimGeoPolylineTool::GetInnerPoint(const CBimGeoPolyline& polygon, const std::vector<const CBimGeoPolyline*>& holes)
{
	if (holes.size() < 1)
	{
		return GetCenterPt(polygon);
	}

	Bnd_Box2d box = polygon.GetBndBox();
	if (box.IsVoid())
	{
		return gp_Pnt2d();
	}

	double dXmin = 0., dXmax = 0., dYmin = 0., dYMax = 0.;
	{
		box.Get(dXmin, dYmin, dXmax, dYMax);

		dXmax += 100.;
		dXmin -= 100.;
	}

	std::set<double> ySet;
	for (int i = 0; i < polygon.GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polygon.GetCurve(ptS, ptE, dBulge, i);

		Bnd_Box2d curveBox = BimGeoAlgorithmTool::GetCurveBndBox(ptS, ptE, dBulge);
		double dXmaxRef = 0., dXminRef = 0., dYminRef = 0., dYMaxRef = 0.;
		{
			curveBox.Get(dXminRef, dYminRef, dXmaxRef, dYMaxRef);
		}

		ySet.insert(ptS.Y());
		ySet.insert(ptE.Y());
		ySet.insert(dYminRef);
		ySet.insert(dYMaxRef);
	}

	if (ySet.size() < 2)
	{
		return gp_Pnt2d();
	}

	std::set<double>::iterator itBegin = ySet.begin();
	double dBeginY = *itBegin, dEndY = *++itBegin;
	gp_Pnt2d ptMax(dXmax, (dBeginY + dEndY) * 0.5);

	if (dEndY - dBeginY < 1.)
	{
		while (++itBegin != ySet.end())
		{
			dBeginY = dEndY;
			dEndY = *itBegin;

			if (1. < dEndY - dBeginY)
			{
				ptMax.SetY((dEndY - dBeginY) * 0.5);
				break;
			}
			else
			{
				double dValue = (dEndY - dBeginY) * 0.5;
				if (ptMax.Y() < dValue)
				{
					ptMax.SetY(dValue);
				}
			}
		}
	}

	std::vector<gp_Pnt2d> intersectPts;
	gp_Pnt2d pt(dXmin, (dYmin + dYMax) * 0.5);
	GetIntersect(intersectPts, polygon, pt, ptMax, 0.);

	for (int i = 0; i < holes.size(); ++i)
	{
		GetIntersect(intersectPts, *holes[i], pt, ptMax, 0.);
	}

	std::vector<gp_Pnt2d> vecSortPt = BimGeoAlgorithmTool::SortByCurve(pt, ptMax, 0., intersectPts);
	if (vecSortPt.size() < 2)
	{
		return gp_Pnt2d();
	}

	if (2 == vecSortPt.size())
	{
		return gp_Pnt2d((vecSortPt[0].Coord() + vecSortPt[1].Coord()) * 0.5);
	}

	double dLength1 = vecSortPt[0].Distance(vecSortPt[1]);
	double dLength2 = vecSortPt[vecSortPt.size() - 1].Distance(vecSortPt[vecSortPt.size() - 2]);

	if (dLength1 < dLength2)
	{
		return gp_Pnt2d((vecSortPt[vecSortPt.size() - 1].Coord() + vecSortPt[vecSortPt.size() - 2].Coord()) * 0.5);
	}

	return gp_Pnt2d((vecSortPt[0].Coord() + vecSortPt[1].Coord()) * 0.5);
}

bool CalcInnerPoint(gp_Pnt2d& ptInner, const CBimGeoPolyline& polygon, const std::vector<const CBimGeoPolyline*>& holes,
	const gp_Pnt2d& ptS, const bool bIsXAxisSet, std::set<double>& axisSet, const double& dMax)
{
	std::set<double>::iterator itBegin = axisSet.begin();
	double dBeginAxis = *itBegin, dEndAxis = *++itBegin;
	gp_Pnt2d ptMax;
	if (bIsXAxisSet)
	{
		ptMax.SetCoord((dBeginAxis + dEndAxis) * 0.5, dMax);
	}
	else
	{
		ptMax.SetCoord(dMax, (dBeginAxis + dEndAxis) * 0.5);
	}

	if (dEndAxis - dBeginAxis < 1.)
	{
		while (++itBegin != axisSet.end())
		{
			dBeginAxis = dEndAxis;
			dEndAxis = *itBegin;

			if (1. < dEndAxis - dBeginAxis)
			{
				if (bIsXAxisSet)
				{
					ptMax.SetX((dEndAxis - dBeginAxis) * 0.5);
				}
				else
				{
					ptMax.SetY((dEndAxis - dBeginAxis) * 0.5);
				}

				break;
			}
			else
			{
				double dValue = (dEndAxis - dBeginAxis) * 0.5;
				if (bIsXAxisSet)
				{
					if (ptMax.X() < dValue)
					{
						ptMax.SetX(dValue);
					}
				}
				else
				{
					if (ptMax.Y() < dValue)
					{
						ptMax.SetY(dValue);
					}
				}
			}
		}
	}

	std::vector<gp_Pnt2d> intersectPts;
	BimGeoPolylineTool::GetIntersect(intersectPts, polygon, ptS, ptMax, 0.);

	for (int i = 0; i < holes.size(); ++i)
	{
		BimGeoPolylineTool::GetIntersect(intersectPts, *holes[i], ptS, ptMax, 0.);
	}

	std::vector<gp_Pnt2d> vecSortPt = BimGeoAlgorithmTool::SortByCurve(ptS, ptMax, 0., intersectPts);
	if (vecSortPt.size() < 2)
	{
		return false;
	}

	std::map<double, std::vector<gp_Pnt2d>> mapMidPoint;
	for (int i = 1; i < vecSortPt.size(); ++i)
	{
		gp_Pnt2d ptMid((vecSortPt[i].Coord() + vecSortPt[i - 1].Coord()) * 0.5);
		mapMidPoint[vecSortPt[i].Distance(vecSortPt[i - 1])].emplace_back(ptMid);
	}

	for (std::map<double, std::vector<gp_Pnt2d>>::reverse_iterator itBegin = mapMidPoint.rbegin();
		itBegin != mapMidPoint.rend(); ++itBegin)
	{
		for (int i = 0; i < itBegin->second.size(); ++i)
		{
			gp_Pnt2d& pt = itBegin->second[i];
			bool bIsValid = true;
			for (int j = 0; j < holes.size(); ++j)
			{
				const CBimGeoPolyline* pHole = holes[i];
				if (EPointWidthPolygonRel::eOutPolygon != BimGeoPolylineTool::CalcPointWithPolygonRelation(pt, *pHole))
				{
					bIsValid = false;
					break;
				}
			}

			if (bIsValid)
			{
				ptInner = pt;
				return true;
			}
		}
	}

	return false;
};

gp_Pnt2d BimGeoPolylineTool::GetInnerPointExt(const CBimGeoPolyline& polygon,
	const std::vector<const CBimGeoPolyline*>& holes, const bool bIsXAxisPrior)
{
	if (holes.size() < 1)
	{
		return GetCenterPt(polygon);
	}

	Bnd_Box2d box = polygon.GetBndBox();
	if (box.IsVoid())
	{
		return gp_Pnt2d();
	}

	double dXmin = 0., dXmax = 0., dYmin = 0., dYmax = 0.;
	{
		box.Get(dXmin, dYmin, dXmax, dYmax);

		dXmax += 100.;
		dXmin -= 100.;
		dYmin -= 100.;
		dYmax += 100.;
	}

	std::set<double> ySet, xSet;
	for (int i = 0; i < polygon.GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polygon.GetCurve(ptS, ptE, dBulge, i);

		Bnd_Box2d curveBox = BimGeoAlgorithmTool::GetCurveBndBox(ptS, ptE, dBulge);
		double dXmaxRef = 0., dXminRef = 0., dYminRef = 0., dYMaxRef = 0.;
		{
			curveBox.Get(dXminRef, dYminRef, dXmaxRef, dYMaxRef);
		}

		ySet.insert(ptS.Y());
		ySet.insert(ptE.Y());
		ySet.insert(dYminRef);
		ySet.insert(dYMaxRef);

		xSet.insert(ptS.X());
		xSet.insert(ptE.X());
		xSet.insert(dXminRef);
		xSet.insert(dXmaxRef);
	}

	bool bYAxisCanCalc = 2 < ySet.size();
	bool bXAxisCanCalc = 2 < xSet.size();

	gp_Pnt2d ptInner;
	int nCount = 2;
	bool bIsXAxisPriorTemp = bIsXAxisPrior;
	while (0 < nCount--)
	{
		if (bYAxisCanCalc && bIsXAxisPriorTemp)
		{
			gp_Pnt2d pt(dXmin, (dYmin + dYmax) * 0.5);
			if (CalcInnerPoint(ptInner, polygon, holes, pt, false, ySet, dXmax))
			{
				return ptInner;
			}
		}
		else if (bXAxisCanCalc && !bIsXAxisPriorTemp)
		{
			gp_Pnt2d pt((dXmin + dXmax) * 0.5, dYmin);
			if (CalcInnerPoint(ptInner, polygon, holes, pt, true, xSet, dYmax))
			{
				return ptInner;
			}
		}

		bIsXAxisPriorTemp = !bIsXAxisPriorTemp;
	}

	return GetCenterPt(polygon);
}

void GetPtsArcInterWithAxis(std::list<gp_Pnt2d>& retPoints,
	const gp_XY& xDirection, const gp_XY& yDirection,
	const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge, const double& dTol)
{
	SGeoArcInfo arcInfo = BimGeoAlgorithmTool::GetArcInfo(ptStart, ptEnd, dBulge);

	std::list<gp_Pnt2d> points(4);
	points.emplace_back(arcInfo.ptCenter.Coord().Added(xDirection * arcInfo.dRadius));
	points.emplace_back(arcInfo.ptCenter.Coord().Added(yDirection * arcInfo.dRadius));
	points.emplace_back(arcInfo.ptCenter.Coord().Subtracted(xDirection * arcInfo.dRadius));
	points.emplace_back(arcInfo.ptCenter.Coord().Subtracted(yDirection * arcInfo.dRadius));

	std::list<gp_Pnt2d>::iterator itPoint = points.begin();
	for (; itPoint != points.end(); itPoint++)
	{
		if (BimGeoAlgorithmTool::IsOn(*itPoint, ptStart, ptEnd, dBulge, dTol))
		{
			retPoints.emplace_back(*itPoint);
		}
	}
}

bool ConstructObbOnDirection(CBimGeoOBB2d& obb, const std::vector<SGeoCurveInfo>& curveList,
	const gp_Pnt2d& centerPoint, const gp_XY& xDirection, const gp_XY& yDirection, const double& maxLength, const double& dTol)
{
	gp_Pnt2d ptS_X_UnboundLine = centerPoint.Coord().Subtracted(xDirection * maxLength);
	gp_Pnt2d ptE_X_UnboundLine = centerPoint.Coord().Added(xDirection * maxLength);
	gp_Pnt2d ptS_Y_UnboundLine = centerPoint.Coord().Subtracted(yDirection * maxLength);
	gp_Pnt2d ptE_Y_UnboundLine = centerPoint.Coord().Added(yDirection * maxLength);
	std::list<double> xValues;
	std::list<double> yValues;

	const int curveCount = curveList.size();
	for (int i = 0; i < curveCount; i++)
	{
		const SGeoCurveInfo& curve = curveList[i];
		std::list<gp_Pnt2d> pointToProjectList;
		pointToProjectList.emplace_back(curve.ptS);
		pointToProjectList.emplace_back(curve.ptE);

		if (BimGeoAlgorithmTool::IsArc(curve.dBulge))
		{
			GetPtsArcInterWithAxis(pointToProjectList, xDirection, yDirection, curve.ptS, curve.ptE, curve.dBulge, dTol);
		}

		std::list<gp_Pnt2d>::iterator itPtToProject = pointToProjectList.begin();
		for (; itPtToProject != pointToProjectList.end(); itPtToProject++)
		{
			//x
			gp_Pnt2d xLineProPt = *itPtToProject;
			BimGeoAlgorithmTool::GetProjectPt(xLineProPt, *itPtToProject, ptS_X_UnboundLine, ptE_X_UnboundLine, 0);
			double dParamX = BimGeoAlgorithmTool::GetParamOnCurve(xLineProPt, ptS_X_UnboundLine, ptE_X_UnboundLine, 0.);
			if (dParamX > -1)
			{
				xValues.emplace_back(dParamX);
			}

			//y
			gp_Pnt2d yLineProPt = *itPtToProject;
			BimGeoAlgorithmTool::GetProjectPt(yLineProPt, *itPtToProject, ptS_Y_UnboundLine, ptE_Y_UnboundLine, 0);
			double dParamY = BimGeoAlgorithmTool::GetParamOnCurve(yLineProPt, ptS_Y_UnboundLine, ptE_Y_UnboundLine, 0.);
			if (dParamY > -1)
			{
				yValues.emplace_back(dParamY);
			}
		}
	}

	if (!xValues.empty() && !yValues.empty())
	{
		xValues.sort();
		yValues.sort();

		gp_Pnt2d minXPoint = BimGeoAlgorithmTool::GetPtOnCurveByParam(xValues.front(), ptS_X_UnboundLine, ptE_X_UnboundLine, 0.);
		gp_Pnt2d maxXPoint = BimGeoAlgorithmTool::GetPtOnCurveByParam(xValues.back(), ptS_X_UnboundLine, ptE_X_UnboundLine, 0.);
		gp_Pnt2d minYPoint = BimGeoAlgorithmTool::GetPtOnCurveByParam(yValues.front(), ptS_Y_UnboundLine, ptE_Y_UnboundLine, 0.);
		gp_Pnt2d maxYPoint = BimGeoAlgorithmTool::GetPtOnCurveByParam(yValues.back(), ptS_Y_UnboundLine, ptE_Y_UnboundLine, 0.);

		gp_Pnt2d xMidPoint = (minXPoint.Coord().Added(maxXPoint.Coord())) * 0.5;
		gp_Pnt2d yMidPoint = (minYPoint.Coord().Added(maxYPoint.Coord())) * 0.5;

		std::vector<gp_Pnt2d> intersectPoints;
		BimGeoAlgorithmTool::CalcLineIntersect(
			intersectPoints,
			gp_Pnt2d(xMidPoint.X(), xMidPoint.Y()),
			gp_Dir2d(yDirection.X(), yDirection.Y()),
			gp_Pnt2d(yMidPoint.X(), yMidPoint.Y()),
			gp_Dir2d(xDirection.X(), xDirection.Y()));

		if (intersectPoints.size() == 1)
		{
			obb.SetCenter(intersectPoints[0]);
			obb.SetXComponent(xDirection, maxXPoint.Distance(minXPoint) * 0.5);
			obb.SetYComponent(yDirection, maxYPoint.Distance(minYPoint) * 0.5);
			return true;
		}
	}

	return false;
}

SGeoCurveInfo GetPolygonCurveByIndex(const std::vector<gp_Pnt2d>& vecVertex, const std::vector<double>& vecBulge, const int& index)
{
	SGeoCurveInfo curve;
	curve.ptS = vecVertex[index];
	curve.ptE = vecVertex[(index + 1) % vecVertex.size()];
	curve.dBulge = vecBulge[index];
	return curve;
}

gp_Dir2d GetArcTangentDirInPoint(const SGeoArcInfo& arcInfo, const gp_Pnt2d& point)
{
	gp_Dir2d curDir;
	const gp_XY centerToPointDir = point.Coord().Subtracted(arcInfo.ptCenter.Coord());
	if (arcInfo.bIsAntiClockwise)
	{
		curDir = gp_Dir2d(-centerToPointDir.Y(), centerToPointDir.X());
	}
	else
	{
		curDir = gp_Dir2d(centerToPointDir.Y(), -centerToPointDir.X());
	}
	return curDir;
}

void CalculateArcOutPtTangePtOffsetToCenter(double& xOffset, double& yOffset, const SGeoArcInfo& arcInfo, const gp_Pnt2d& point)
{
	const double pointDistanceToCenter = point.Distance(arcInfo.ptCenter);
	xOffset = arcInfo.dRadius * arcInfo.dRadius / pointDistanceToCenter;
	yOffset = std::sqrt(arcInfo.dRadius * arcInfo.dRadius - xOffset * xOffset);
}

gp_Pnt2d GetCircleSameTangLineInterPt(const SGeoArcInfo& aArc, const SGeoArcInfo& bArc)
{
	const double centerDistance = aArc.ptCenter.Distance(bArc.ptCenter);
	gp_XY bigToSmallDir = bArc.ptCenter.Coord().Subtracted(aArc.ptCenter.Coord());
	bigToSmallDir.Normalize();

	gp_Pnt2d smallCenter = bArc.ptCenter;
	double bigRadius = aArc.dRadius;
	double smallRadius = bArc.dRadius;
	if (aArc.dRadius < bArc.dRadius)
	{
		bigToSmallDir *= -1;
		bigRadius = bArc.dRadius;
		smallRadius = aArc.dRadius;
		smallCenter = aArc.ptCenter;
	}
	const double outPointDistanceToSmallCircle = smallRadius * centerDistance / (bigRadius - smallRadius);
	return smallCenter.Coord().Added(bigToSmallDir * outPointDistanceToSmallCircle);
}

bool CalculateGeoPolygonConvexHull(CBimGeoPolyline& retConvexHull, const CBimGeoPolyline& polygon, const double& dTol)
{
	//检测负点 连接负点两侧的点 补缺
	//在含有弧线时 存在弧线内凹 也算负点
	//弧线外凸时可产生凹点 需要求直线与弧线的切线 求不到

	std::vector<gp_Pnt2d> vecVertex = polygon.GetVertexes();
	std::vector<double> vecBulge = polygon.GetBulges();

	//处理弧线内凹
	gp_Vec normal = polygon.Normal();
	if (normal.Angle(gp_Vec(0., 0., 1.)) > M_PI_2)
	{
		//认为是顺时针
		retConvexHull = polygon;
		retConvexHull.Reverse();
		vecVertex = retConvexHull.GetVertexes();
		vecBulge = retConvexHull.GetBulges();
		normal.Multiply(-1);
	}

	//抹掉向内凹的弧线
	for (int curIndex = 0; curIndex < vecVertex.size(); curIndex++)
	{
		SGeoCurveInfo curCurve = GetPolygonCurveByIndex(vecVertex, vecBulge, curIndex);
		bool curIsArc = BimGeoAlgorithmTool::IsArc(curCurve.dBulge);
		if (curIsArc && curCurve.dBulge < 0.)
		{
			//当前弧线内凹 抹掉当前线的弧度
			vecVertex[curIndex] = curCurve.ptS;
			vecBulge[curIndex] = 0.;
		}
	}
	double angleTol = 0.;
	bool isComplete = false;
	while (!isComplete)
	{
		isComplete = true;
		for (int curIndex = 0; curIndex < vecVertex.size(); curIndex++)
		{
			int nexIndex = (curIndex + 1) % vecVertex.size();

			SGeoCurveInfo curCurve = GetPolygonCurveByIndex(vecVertex, vecBulge, curIndex);
			SGeoCurveInfo nexCurve = GetPolygonCurveByIndex(vecVertex, vecBulge, nexIndex);

			bool curIsArc = BimGeoAlgorithmTool::IsArc(curCurve.dBulge);
			bool nexIsArc = BimGeoAlgorithmTool::IsArc(nexCurve.dBulge);
			if (!curIsArc && !nexIsArc)
			{
				gp_Dir2d curDir = curCurve.ptE.Coord().Subtracted(curCurve.ptS.Coord());
				gp_Dir2d nexDir = nexCurve.ptE.Coord().Subtracted(nexCurve.ptS.Coord());
				//两条线都是直线
				if (curDir.Crossed(nexDir) < angleTol)
				{
					//凹点 两条线合并
					vecVertex.erase(vecVertex.begin() + nexIndex);
					vecBulge.erase(vecBulge.begin() + nexIndex);

					vecVertex[curIndex] = curCurve.ptS;
					vecBulge[curIndex] = 0;

					isComplete = false;
				}
			}
			else if (curIsArc && !nexIsArc)
			{
				//当前弧线外凸
				SGeoArcInfo curArcInfo = BimGeoAlgorithmTool::GetArcInfo(curCurve.ptS, curCurve.ptE, curCurve.dBulge);

				gp_Dir2d curDir = GetArcTangentDirInPoint(curArcInfo, curCurve.ptE);

				gp_Dir2d nexDir = nexCurve.ptE.Coord().Subtracted(nexCurve.ptS.Coord());

				if (curDir.Crossed(nexDir) < angleTol)
				{
					//凹点 求解下一条线的终止点与当前弧线的切线点连接 没有切线则连接当前线起始点 抹掉当前线
					double xOffset;
					double yOffset;
					CalculateArcOutPtTangePtOffsetToCenter(xOffset, yOffset, curArcInfo, nexCurve.ptE);

					gp_XY centerToNexEndDir = nexCurve.ptE.Coord().Subtracted(curArcInfo.ptCenter.Coord());
					centerToNexEndDir.Normalize();
					gp_XY centerToTopPointDir(centerToNexEndDir.Y(), -centerToNexEndDir.X());

					gp_Pnt2d tangentPoint = curArcInfo.ptCenter.Coord()
						.Added(centerToNexEndDir * xOffset)
						.Added(centerToTopPointDir * yOffset);

					if (BimGeoAlgorithmTool::IsOn(tangentPoint, curCurve.ptS, curCurve.ptE, curCurve.dBulge))
					{
						//有切点
						double newBugle = BimGeoAlgorithmTool::CalcArcBulge(curCurve.ptS, tangentPoint, curCurve.ptS, curCurve.ptE, curCurve.dBulge);

						vecVertex[nexIndex] = tangentPoint;
						vecBulge[nexIndex] = 0.;

						vecVertex[curIndex] = curCurve.ptS;
						vecBulge[curIndex] = newBugle;
					}
					else
					{
						//无切点
						vecVertex.erase(vecVertex.begin() + curIndex);
						vecBulge.erase(vecBulge.begin() + curIndex);

						vecVertex[curIndex] = curCurve.ptS;
						vecBulge[curIndex] = 0;
					}
					isComplete = false;
				}
			}
			else if (!curIsArc && nexIsArc)
			{
				//当前弧线外凸
				SGeoArcInfo nexArcInfo = BimGeoAlgorithmTool::GetArcInfo(nexCurve.ptS, nexCurve.ptE, nexCurve.dBulge);

				gp_Dir2d nexDir = GetArcTangentDirInPoint(nexArcInfo, nexCurve.ptS);

				gp_Dir2d curDir = curCurve.ptE.Coord().Subtracted(curCurve.ptS.Coord());
				if (curDir.Crossed(nexDir) < angleTol)
				{
					//凹点 求解当前线的起点与下一条弧线的切线点连接 没有切线则连接下一条线终点 抹掉下一条线
					double xOffset;
					double yOffset;
					CalculateArcOutPtTangePtOffsetToCenter(xOffset, yOffset, nexArcInfo, curCurve.ptS);

					gp_XY centerToCurStaDir = curCurve.ptS.Coord().Subtracted(nexArcInfo.ptCenter.Coord());
					centerToCurStaDir.Normalize();
					gp_XY centerToTopPointDir(-centerToCurStaDir.Y(), centerToCurStaDir.X());

					gp_Pnt2d tangentPoint = nexArcInfo.ptCenter.Coord()
						.Added(centerToCurStaDir * xOffset)
						.Added(centerToTopPointDir * yOffset);

					if (BimGeoAlgorithmTool::IsOn(tangentPoint, nexCurve.ptS, nexCurve.ptE, nexCurve.dBulge))
					{
						//有切点
						double newBugle = BimGeoAlgorithmTool::CalcArcBulge(tangentPoint, nexCurve.ptE, nexCurve.ptS, nexCurve.ptE, nexCurve.dBulge);

						vecVertex[nexIndex] = tangentPoint;
						vecBulge[nexIndex] = newBugle;
					}
					else
					{
						//无切点
						vecVertex.erase(vecVertex.begin() + nexIndex);
						vecBulge.erase(vecBulge.begin() + nexIndex);
					}
					isComplete = false;
				}
			}
			else if (curIsArc && nexIsArc)
			{
				//先按交点的计算
				SGeoArcInfo curArcInfo = BimGeoAlgorithmTool::GetArcInfo(curCurve.ptS, curCurve.ptE, curCurve.dBulge);
				SGeoArcInfo nexArcInfo = BimGeoAlgorithmTool::GetArcInfo(nexCurve.ptS, nexCurve.ptE, nexCurve.dBulge);

				gp_Dir2d curDir = GetArcTangentDirInPoint(curArcInfo, curCurve.ptE);
				gp_Dir2d nexDir = GetArcTangentDirInPoint(nexArcInfo, nexCurve.ptS);

				if (curDir.Crossed(nexDir) < angleTol)
				{
					gp_XY curCenterToNexCenterDir = nexArcInfo.ptCenter.Coord().Subtracted(curArcInfo.ptCenter.Coord());
					gp_XY centerToTopPointDir(curCenterToNexCenterDir.Y(), -curCenterToNexCenterDir.X());
					centerToTopPointDir.Normalize();

					//关系为内凹
					//求解两个向外凸的弧线的公切线
					gp_Pnt2d tangentPoint0;	//curArc的切点
					gp_Pnt2d tangentPoint1;	//nexArc的切点
					if (std::abs(curArcInfo.dRadius - nexArcInfo.dRadius) < 1.)	//防止角度过小 无法求到两条公切线的交点
					{
						//以各自的顶部点为切点
						tangentPoint0 = curArcInfo.ptCenter.Coord().Added(centerToTopPointDir * curArcInfo.dRadius);
						tangentPoint1 = nexArcInfo.ptCenter.Coord().Added(centerToTopPointDir * nexArcInfo.dRadius);
					}
					else
					{
						//两圆外侧 公切线交点
						gp_Pnt2d outSidePoint = GetCircleSameTangLineInterPt(curArcInfo, nexArcInfo);

						double curXOffset;
						double curYOffset;
						CalculateArcOutPtTangePtOffsetToCenter(curXOffset, curYOffset, curArcInfo, outSidePoint);


						auto curCenterToOutSideDir = outSidePoint.Coord().Subtracted(curArcInfo.ptCenter.Coord());
						curCenterToOutSideDir.Normalize();
						gp_XY curCenterToTopPointDir(curCenterToOutSideDir.Y(), -curCenterToOutSideDir.X());

						tangentPoint0 = curArcInfo.ptCenter.Coord()
							.Added(curCenterToOutSideDir * curXOffset)
							.Added(curCenterToTopPointDir * curYOffset);

						double nexXOffset;
						double nexYOffset;
						CalculateArcOutPtTangePtOffsetToCenter(nexXOffset, nexYOffset, nexArcInfo, outSidePoint);

						auto nexCenterToOutSideDir = outSidePoint.Coord().Subtracted(nexArcInfo.ptCenter.Coord());
						nexCenterToOutSideDir.Normalize();
						gp_XY nexCenterToTopPointDir(nexCenterToOutSideDir.Y(), -nexCenterToOutSideDir.X());

						tangentPoint1 = nexArcInfo.ptCenter.Coord()
							.Added(nexCenterToOutSideDir * nexXOffset)
							.Added(nexCenterToTopPointDir * nexYOffset);
					}

					if (BimGeoAlgorithmTool::IsOn(tangentPoint0, curCurve.ptS, curCurve.ptE, curCurve.dBulge))
					{
						//有切点
						double newBugle = BimGeoAlgorithmTool::CalcArcBulge(curCurve.ptS, tangentPoint0, curCurve.ptS, curCurve.ptE, curCurve.dBulge);

						vecVertex.insert(vecVertex.begin() + nexIndex, tangentPoint0);
						vecBulge.insert(vecBulge.begin() + nexIndex, 0);
						nexIndex = (nexIndex + 1) % vecVertex.size();
						vecVertex[curIndex] = curCurve.ptS;
						vecBulge[curIndex] = newBugle;
					}
					else
					{
						//无切点
						vecVertex[curIndex] = curCurve.ptS;
						vecBulge[curIndex] = 0.;
					}

					if (BimGeoAlgorithmTool::IsOn(tangentPoint1, nexCurve.ptS, nexCurve.ptE, nexCurve.dBulge))
					{
						//有切点
						double newBugle = BimGeoAlgorithmTool::CalcArcBulge(tangentPoint1, nexCurve.ptE, nexCurve.ptS, nexCurve.ptE, nexCurve.dBulge);

						vecVertex[nexIndex] = tangentPoint1;
						vecBulge[nexIndex] = newBugle;
					}
					else
					{
						//无切点
						vecVertex.erase(vecVertex.begin() + nexIndex);
						vecBulge.erase(vecBulge.begin() + nexIndex);
					}
					isComplete = false;
				}
			}

			if (!isComplete)
			{
				for (int curVecIndex = vecVertex.size() - 1; curVecIndex >= 0; curVecIndex--)
				{
					int nexVecIndex = (curVecIndex - 1 + vecVertex.size()) % vecVertex.size();

					if (BimGeoAlgorithmTool::GetCurveLength(vecVertex[curVecIndex], vecVertex[nexVecIndex], vecBulge[curVecIndex]) <= dTol)
					{
						vecVertex.erase(vecVertex.begin() + curVecIndex);
						vecBulge.erase(vecBulge.begin() + curVecIndex);
					}
				}
				break;
			}
		}
	}
	if (vecVertex.size() > 2 && vecBulge.size() == vecVertex.size())
	{
		retConvexHull = CBimGeoPolyline(vecVertex, vecBulge, true);
		return true;
	}
	else
	{
		return false;
	}
}

void GetCurveListFromPolygon(std::vector<SGeoCurveInfo>& curveList, const CBimGeoPolyline& polygon)
{
	const int curveCount = polygon.GetCurveNumber();
	for (int i = 0; i < curveCount; i++)
	{
		SGeoCurveInfo curve;
		if (polygon.GetCurve(curve.ptS, curve.ptE, curve.dBulge, i))
		{
			curveList.emplace_back(curve);
		}
	}
}

void GetCurveListFromPolygon(std::list<SGeoCurveInfo>& curveList, const CBimGeoPolyline& polygon)
{
	const int curveCount = polygon.GetCurveNumber();
	for (int i = 0; i < curveCount; i++)
	{
		SGeoCurveInfo curve;
		if (polygon.GetCurve(curve.ptS, curve.ptE, curve.dBulge, i))
		{
			curveList.emplace_back(curve);
		}
	}
}

bool CalPtsConvexHull(CBimGeoPolyline& polygon, const std::list<gp_Pnt2d>& inputPts, const double& dTol)
{
	if (inputPts.size() <= 2)
	{
		return false;
	}

	std::vector<gp_Pnt2d> inputPtVec;
	inputPtVec.reserve(inputPts.size());
	inputPtVec.insert(inputPtVec.end(), inputPts.begin(), inputPts.end());

	std::list<int> indexList;
	int nIndexMinYPt = inputPtVec.size() - 1;
	indexList.emplace_front(nIndexMinYPt);
	for (int i = nIndexMinYPt - 1; i >= 0; i--)
	{
		if (inputPtVec[i].Y() < inputPtVec[nIndexMinYPt].Y())
		{
			nIndexMinYPt = i;
		}
		indexList.emplace_front(i);
	}
	indexList.erase(indexList.begin());

	gp_Pnt2d tmpPt = inputPtVec[0];
	inputPtVec[0] = inputPtVec[nIndexMinYPt];
	inputPtVec[nIndexMinYPt] = tmpPt;

	gp_Dir xDir(1., 0., 0.);
	gp_Vec2d tmpDir;
	std::map<int, double> angleMap;
	std::map<int, double> distanceMap;

	std::list<int>::iterator itIndex = indexList.begin();
	for (; itIndex != indexList.end(); )
	{
		tmpDir = inputPtVec[*itIndex].Coord().Subtracted(inputPtVec[0].Coord());
		double tmpDistance = tmpDir.Magnitude();

		if (tmpDistance <= dTol)
		{
			itIndex = indexList.erase(itIndex);
		}
		else
		{
			angleMap[*itIndex] = xDir.AngleWithRef(gp_Dir(tmpDir.X(), tmpDir.Y(), 0.), gp_Dir(0., 0., 1.));
			distanceMap[*itIndex] = tmpDistance;
			++itIndex;
		}
	}

	if (indexList.size() < 2)
	{
		return false;
	}

	indexList.sort([&](const int& nIndexA, const int& nIndexB) -> bool
	{
		if (angleMap[nIndexA] != angleMap[nIndexB])
		{
			return angleMap[nIndexA] < angleMap[nIndexB];
		}
		else
		{
			return distanceMap[nIndexA] < distanceMap[nIndexB];
		}
	});

	std::vector<int> hullIndexVec;
	hullIndexVec.reserve(inputPtVec.size());
	hullIndexVec.emplace_back(0);
	hullIndexVec.emplace_back(indexList.front());

	std::vector<int>::reverse_iterator itStackTop = hullIndexVec.rbegin();
	itIndex = indexList.begin();
	itIndex++;
	while (itIndex != indexList.end())
	{
		gp_Vec2d tmpDir = inputPtVec[*itIndex].Coord().Subtracted(inputPtVec[*itStackTop].Coord());
		if (tmpDir.Magnitude() < dTol)
		{
			//相同点 跳过
			++itIndex;
			continue;
		}
		if (hullIndexVec.size() == 1)
		{
			//集合只有一个点 加入
			hullIndexVec.emplace_back(*itIndex);
			itStackTop = hullIndexVec.rbegin();
			++itIndex;
			continue;
		}
		gp_Vec2d curDir = inputPtVec[*itStackTop].Coord().Subtracted(inputPtVec[*(itStackTop + 1)].Coord());
		gp_Vec2d nexDir = inputPtVec[*itIndex].Coord().Subtracted(inputPtVec[*(itStackTop + 1)].Coord());
		if (nexDir.Magnitude() < dTol)
		{
			//相同点 跳过
			++itIndex;
			continue;
		}

		double value0 = curDir.Crossed(nexDir);
		// 		double value1 = curDir.CrossMagnitude(nexDir);
		// 		double value2 = curDir.CrossSquareMagnitude(nexDir);
		// 		gp_Vec value3 = gp_Vec(curDir.X(), curDir.Y(), 0.).Crossed(gp_Vec(nexDir.X(), nexDir.Y(), 0.));
		if (value0 < 0)
		{
			hullIndexVec.pop_back();
			itStackTop = hullIndexVec.rbegin();
		}
		else
		{
			// 可用加入集合
			hullIndexVec.emplace_back(*itIndex);
			itStackTop = hullIndexVec.rbegin();
			++itIndex;
		}
	}

	if (hullIndexVec.size() > 2)
	{
		polygon.SetVertexes(std::vector<gp_Pnt2d>());
		polygon.SetBulges(std::vector<double>());
		for (std::vector<int>::iterator itHullIndex = hullIndexVec.begin();
			itHullIndex != hullIndexVec.end(); ++itHullIndex)
		{
			polygon.AddVertex(inputPtVec[*itHullIndex], 0.);
		}
		polygon.setClose(true);

		return true;
	}
	else
	{
		return false;
	}
}

bool BimGeoPolylineTool::CalculateCurvesConvexHull(CBimGeoPolyline& polygon,
	const std::list<SGeoCurveInfo>& inputCurves, const double& dTol)
{
	std::list<gp_Pnt2d> pts;
	for (std::list<SGeoCurveInfo>::const_iterator itCurve = inputCurves.begin();
		itCurve != inputCurves.end(); ++itCurve)
	{
		if (BimGeoAlgorithmTool::IsArc(itCurve->dBulge))
		{
			gp_Pnt2d midPt = BimGeoAlgorithmTool::GetCurveMidPoint(itCurve->ptS, itCurve->ptE, itCurve->dBulge);
			pts.emplace_back(midPt);
		}
		pts.emplace_back(itCurve->ptS);
		pts.emplace_back(itCurve->ptE);
	}

	if (CalPtsConvexHull(polygon, pts, dTol))
	{
		BimGeoPolylineTool::RemovePoint(polygon, dTol);
		return true;
	}
	return false;
}

bool BimGeoPolylineTool::CalculatePolygonMinAreaObb(CBimGeoOBB2d& minAreaObb, const CBimGeoPolyline& inputPolygon, const double& dTol)
{
	CBimGeoPolyline polygon = inputPolygon;
	if (!polygon.IsClose())
	{
		return false;
	}

	BimGeoPolylineTool::RemovePoint(polygon, dTol);
	if (polygon.GetVertexes().empty())
	{
		return false;
	}

	Bnd_Box2d box2d = polygon.GetBndBox();
	if (box2d.IsVoid())
	{
		return false;
	}

	const double maxLength = std::sqrt(box2d.SquareExtent());

	CBimGeoPolyline convexHull;
	CalculateGeoPolygonConvexHull(convexHull, polygon, dTol);

	std::vector<SGeoCurveInfo> curveList;
	GetCurveListFromPolygon(curveList, convexHull);

	bool isEmpty = true;
	CBimGeoOBB2d minObb;
	CBimGeoOBB2d tmpObb;

	SGeoCurveInfo tmpCurve;
	const int curveCount = convexHull.GetCurveNumber();
	for (int i = 0; i < curveCount; i++)
	{
		convexHull.GetCurve(tmpCurve.ptS, tmpCurve.ptE, tmpCurve.dBulge, i);
		gp_XY xDirection = tmpCurve.ptE.Coord().Subtracted(tmpCurve.ptS.Coord()).Normalized();
		gp_XY yDirection = gp_XY(-xDirection.Y(), xDirection.X());

		if (ConstructObbOnDirection(tmpObb, curveList, tmpCurve.ptS, xDirection, yDirection, maxLength, dTol))
		{
			if (isEmpty)
			{
				minObb = tmpObb;
				isEmpty = false;
			}
			else if (tmpObb.SquareExtent() < minObb.SquareExtent())
			{
				minObb = tmpObb;
			}
		}
	}

	if (!isEmpty)
	{
		minAreaObb = minObb;
		return true;
	}

	return false;
}