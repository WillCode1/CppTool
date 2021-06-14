#include "BimGeoTwoPolygonBoolOper.h"
#include "BimGeoPolylineTool.h"
#include "BimGeoAlgorithm/BimGeoAlgorithmTool.h"
#include <map>

CBimGeoTwoPolygonBoolOper::CBimGeoTwoPolygonBoolOper()
	: m_pPolygonA(NULL)
	, m_pPolygonB(NULL)
{
}

CBimGeoTwoPolygonBoolOper::~CBimGeoTwoPolygonBoolOper()
{
}

void CBimGeoTwoPolygonBoolOper::DoIt(const CBimGeoPolyline* pPolygonA, 
	const CBimGeoPolyline* pPolygonB, const EBooleanOperation eBooleanOper, const double& dTol)
{
	Clear();

	if (NULL == pPolygonA
		|| NULL == pPolygonB)
	{
		return;
	}

	m_pPolygonA = pPolygonA;
	m_pPolygonB = pPolygonB;
	m_eBooleanOper = eBooleanOper;
	m_dTol = dTol;

	CalcImp();
}

const std::list<SBimGeoPolygonWithHole>& CBimGeoTwoPolygonBoolOper::GetPolygons() const
{
	return m_retPolygon;
}

void CBimGeoTwoPolygonBoolOper::Clear()
{
	m_retPolygon.clear();
}

void CBimGeoTwoPolygonBoolOper::CalcImp()
{
	bool bIsReturn = false;
	EPolygonRel eRet = BimGeoPolylineTool::CalcTwoPolygonRelation(*m_pPolygonA, *m_pPolygonB, m_dTol);
	switch (eRet)
	{
	case EPolygonRel::eTagent:
	{
		switch (m_eBooleanOper)
		{
		case EBooleanOperation::ePolygonIntersect:
			bIsReturn = true;
			break;
		case EBooleanOperation::ePolygonSubTract:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonA;
			m_retPolygon.emplace_back(polygonWithHole);

			bIsReturn = true;
		}
		break;
		default:
			break;
		}
	}
		break;
	case EPolygonRel::eOuter:
	{
		switch (m_eBooleanOper)
		{
		case EBooleanOperation::ePolygonSubTract:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonA;
			m_retPolygon.emplace_back(polygonWithHole);
		}
		break;
		default:
			break;
		}

		bIsReturn = true;
	}
	break;
	case EPolygonRel::eAInB:
	case EPolygonRel::eAInBWithTagent:
	{
		switch (m_eBooleanOper)
		{
		case EBooleanOperation::ePolygonIntersect:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonA;
			m_retPolygon.emplace_back(polygonWithHole);
		}
		break;
		case EBooleanOperation::ePolygonUnion:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonB;
			m_retPolygon.emplace_back(polygonWithHole);
		}
		break;
		default:
			break;
		}

		bIsReturn = true;
	}
	break;
	case EPolygonRel::eBInA:
	{
		switch (m_eBooleanOper)
		{
		case EBooleanOperation::ePolygonIntersect:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonB;
			m_retPolygon.emplace_back(polygonWithHole);
		}
		break;
		case EBooleanOperation::ePolygonUnion:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonA;
			m_retPolygon.emplace_back(polygonWithHole);
		}
		break;
		case EBooleanOperation::ePolygonSubTract:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonA;
			polygonWithHole.holes.emplace_back(*m_pPolygonB);
			m_retPolygon.emplace_back(polygonWithHole);
		}
		break;
		default:
			break;
		}

		bIsReturn = true;
	}
	break;
	case EPolygonRel::eBInAWithTagent:
	{
		switch (m_eBooleanOper)
		{
		case EBooleanOperation::ePolygonIntersect:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonB;
			m_retPolygon.emplace_back(polygonWithHole);

			bIsReturn = true;
		}
		break;
		case EBooleanOperation::ePolygonUnion:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonA;
			m_retPolygon.emplace_back(polygonWithHole);

			bIsReturn = true;
		}
		break;
		default:
			break;
		}
	}
	break;
	case EPolygonRel::eOverlap:
	{
		switch (m_eBooleanOper)
		{
		case EBooleanOperation::ePolygonIntersect:
		case EBooleanOperation::ePolygonUnion:
		{
			SBimGeoPolygonWithHole polygonWithHole;
			polygonWithHole.orgPolygon = *m_pPolygonA;
			m_retPolygon.emplace_back(polygonWithHole);

			bIsReturn = true;
		}
		break;
		default:
			break;
		}

		bIsReturn = true;
	}
	break;
	default:
		break;
	}

	if (bIsReturn)
	{
		return;
	}

	CalcWithIntersectPolygon();

	if (EPolygonRel::eBInAWithTagent == eRet)
	{
		RemoveOverlapPolygon(m_pPolygonB);
	}
}

void CBimGeoTwoPolygonBoolOper::CalcWithIntersectPolygon()
{
	std::list<SGeoCurveInfo> curves;
	switch (m_eBooleanOper)
	{
	case EBooleanOperation::ePolygonSubTract:
	{
		CalcTwoPolygonBooleanOperData(curves, m_pPolygonA, m_pPolygonB, EBooleanOperation::ePolygonUnion);
		CalcTwoPolygonBooleanOperData(curves, m_pPolygonB, m_pPolygonA, EBooleanOperation::ePolygonIntersect);
	}
	break;
	default:
		CalcTwoPolygonBooleanOperData(curves, m_pPolygonA, m_pPolygonB, m_eBooleanOper);
		CalcTwoPolygonBooleanOperData(curves, m_pPolygonB, m_pPolygonA, m_eBooleanOper);
		break;
	}

	std::list<CBimGeoPolyline> polygons;
	switch (m_eBooleanOper)
	{
	case EBooleanOperation::ePolygonUnion:
		polygons = BimGeoPolylineTool::SearchMaxPolygonWithNoBreakCurve(curves, m_dTol);
		break;
	default:
		polygons = BimGeoPolylineTool::SearchMinPolygonWithNoBreakCurve(curves, m_dTol);
		break;
	}

	for (std::list<CBimGeoPolyline>::iterator itPolygon = polygons.begin();
	itPolygon != polygons.end(); ++itPolygon)
	{
		SBimGeoPolygonWithHole polygonWithHole;
		polygonWithHole.orgPolygon = *itPolygon;
		m_retPolygon.emplace_back(polygonWithHole);
	}
}

void CBimGeoTwoPolygonBoolOper::CalcTwoPolygonBooleanOperData(std::list<SGeoCurveInfo>& curves, const CBimGeoPolyline* pPolygonA, 
	const CBimGeoPolyline* pPolygonB, const EBooleanOperation eCalcType) const
{
	// 判断交点的进出性质
	int nStartPtWithPolyRel = 0;//	0 在外边 1 在多边形上 2 在多边形内
	std::list<gp_Pnt2d> points;// 交点集合
	std::map<gp_Pnt2d*, bool> mapPointAccess;//	出入点标识

	for (int i = 0; i < pPolygonA->GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;

		pPolygonA->GetCurve(ptS, ptE, dBulge, i);
		if (0 == i)
		{
			EPointWidthPolygonRel eRel = BimGeoPolylineTool::CalcPointWithPolygonRelation(ptS, *pPolygonB, m_dTol);
			switch (eRel)
			{
			case EPointWidthPolygonRel::eOnPolygon:
				nStartPtWithPolyRel = 1;
				break;
			case EPointWidthPolygonRel::eInPolygon:
				nStartPtWithPolyRel = 2;
				break;
			default:
				break;
			}
		}

		std::vector<gp_Pnt2d> intersectPts;
		BimGeoPolylineTool::GetIntersect(intersectPts, *pPolygonB, ptS, ptE, dBulge, m_dTol);

		if (1 <= intersectPts.size())
		{
			std::vector<gp_Pnt2d> sortPts = BimGeoAlgorithmTool::SortByCurve(ptS, ptE, dBulge, intersectPts);
			if (1 < sortPts.size())
			{
				for (int i = 0; i < sortPts.size(); i += 2)
				{
					if (sortPts[i].IsEqual(sortPts[i + 1], m_dTol))
					{
						continue;
					}

					points.emplace_back(sortPts[i]);

					gp_Pnt2d ptMid((sortPts[i].Coord() + sortPts[i + 1].Coord()) * 0.5);
					gp_Pnt2d ptClose = BimGeoAlgorithmTool::GetClosePoint(ptMid, ptS, ptE, dBulge, m_dTol);

					EPointWidthPolygonRel eRet = BimGeoPolylineTool::CalcPointWithPolygonRelation(ptClose, *pPolygonB, m_dTol);
					switch (eRet)
					{
					case EPointWidthPolygonRel::eOutPolygon:
						mapPointAccess[&*points.rbegin()] = false;
						break;
					case EPointWidthPolygonRel::eOnPolygon:
						mapPointAccess[&*points.rbegin()] = EBooleanOperation::ePolygonIntersect == eCalcType;
						break;
					case EPointWidthPolygonRel::eInPolygon:
						mapPointAccess[&*points.rbegin()] = true;
						break;
					default:
						break;
					}
				}
			}

			if (!sortPts.rbegin()->IsEqual(ptE, m_dTol))
			{
				points.emplace_back(*sortPts.rbegin());

				gp_Pnt2d ptMid((sortPts.rbegin()->Coord() + ptE.Coord()) * 0.5);
				gp_Pnt2d ptClose = BimGeoAlgorithmTool::GetClosePoint(ptMid, ptS, ptE, dBulge, m_dTol);

				EPointWidthPolygonRel eRet = BimGeoPolylineTool::CalcPointWithPolygonRelation(ptClose, *pPolygonB, m_dTol);
				switch (eRet)
				{
				case EPointWidthPolygonRel::eOutPolygon:
					mapPointAccess[&*points.rbegin()] = false;
					break;
				case EPointWidthPolygonRel::eOnPolygon:
					mapPointAccess[&*points.rbegin()] = EBooleanOperation::ePolygonIntersect == eCalcType;
					break;
				case EPointWidthPolygonRel::eInPolygon:
					mapPointAccess[&*points.rbegin()] = true;
					break;
				default:
					break;
				}
			}
			else
			{
				points.emplace_back(*sortPts.rbegin());
				mapPointAccess[&*points.rbegin()] = true;
			}
		}
	}

	// 获取数据
	bool bEntryPoint = true, bOutPoint = false;
	if (EBooleanOperation::ePolygonIntersect != eCalcType)
	{// 合并区域
		bEntryPoint = false;
		bOutPoint = true;
	}

	gp_Pnt2d ptS, ptE;
	bool bSetStart = false, bSetEnd = false;
	if (bEntryPoint && 0 != nStartPtWithPolyRel)
	{
		ptS = pPolygonA->GetVertexes()[0];
		bSetStart = true;
	}
	else if (!bEntryPoint && 0 == nStartPtWithPolyRel)
	{
		ptS = pPolygonA->GetVertexes()[0];
		bSetStart = true;
	}

	for (std::list<gp_Pnt2d>::iterator itPt = points.begin();
		itPt != points.end(); ++itPt)
	{
		gp_Pnt2d& pt = *itPt;
		std::map<gp_Pnt2d*, bool>::iterator itFind = mapPointAccess.find(&pt);
		if (mapPointAccess.end() == itFind)
		{
			continue;
		}

		if (!bSetStart)
		{
			if (bEntryPoint == itFind->second)
			{// 进
				ptS = pt;
				bSetStart = true;
			}
		}
		else
		{
			if (bOutPoint == itFind->second)
			{// 出
				ptE = pt;
				bSetEnd = true;
			}
		}

		if (bSetEnd)
		{
			bSetStart = false;
			bSetEnd = false;

			if (!ptS.IsEqual(ptE, m_dTol))
			{
				std::list<CBimGeoPolyline> retPolies;
				BimGeoPolylineTool::InterceptPolyline(retPolies, *pPolygonA, ptS, ptE, m_dTol);
				PolylineToCurves(curves, retPolies, points);
			}
		}
	}

	if (bSetStart && !bSetEnd)
	{
		ptE = pPolygonA->GetVertexes()[0];

		std::list<CBimGeoPolyline> retPolies;
		BimGeoPolylineTool::InterceptPolyline(retPolies, *pPolygonA, ptS, ptE, m_dTol);
		PolylineToCurves(curves, retPolies, points);
	}
}

void CBimGeoTwoPolygonBoolOper::PolylineToCurves(std::list<SGeoCurveInfo>& curves, 
	const std::list<CBimGeoPolyline>& polies, const std::list<gp_Pnt2d>& breakPoints) const
{
	for (std::list<CBimGeoPolyline>::const_iterator itPoly = polies.begin();
		itPoly != polies.end(); ++itPoly)
	{
		const CBimGeoPolyline& poly = *itPoly;
		for (int i = 0; i < poly.GetCurveNumber(); ++i)
		{
			SGeoCurveInfo curve;
			poly.GetCurve(curve.ptS, curve.ptE, curve.dBulge, i);

			BreakCurves(curves, curve, breakPoints);
		}
	}
}

void CBimGeoTwoPolygonBoolOper::BreakCurves(std::list<SGeoCurveInfo>& curves, 
	const SGeoCurveInfo& curve, const std::list<gp_Pnt2d>& breakPoints) const
{
	SGeoCurveInfo calcCurve = curve;
	for (std::list<gp_Pnt2d>::const_iterator itPt = breakPoints.begin();
		itPt != breakPoints.end(); ++itPt)
	{
		const gp_Pnt2d pt = *itPt;
		Bnd_Box2d bndBox = BimGeoAlgorithmTool::GetCurveBndBox(calcCurve.ptS, calcCurve.ptE, calcCurve.dBulge);
		if (bndBox.IsOut(pt))
		{
			continue;
		}

		if (pt.IsEqual(calcCurve.ptS, m_dTol)
			|| pt.IsEqual(calcCurve.ptE, m_dTol))
		{
			continue;
		}

		if (BimGeoAlgorithmTool::IsOn(pt, calcCurve.ptS, calcCurve.ptE, calcCurve.dBulge))
		{
			if (BimGeoAlgorithmTool::IsArc(curve.dBulge))
			{// 圆弧
				SGeoCurveInfo curve1;
				curve1.ptS = calcCurve.ptS;
				curve1.ptE = pt;
				curve1.dBulge = BimGeoAlgorithmTool::CalcArcBulge(curve1.ptS, curve1.ptE,
					curve.ptS, curve.ptE, curve.dBulge, m_dTol);

				calcCurve.ptS = pt;
				calcCurve.dBulge = BimGeoAlgorithmTool::CalcArcBulge(calcCurve.ptS, calcCurve.ptE,
					curve.ptS, curve.ptE, curve.dBulge, m_dTol);

				curves.emplace_back(curve1);
			}
			else
			{
				SGeoCurveInfo curve1;
				curve1.ptS = calcCurve.ptS;
				curve1.ptE = pt;
				curve1.dBulge = 0.;

				calcCurve.ptS = pt;

				curves.emplace_back(curve1);
			}
		}
	}

	if (!calcCurve.ptS.IsEqual(calcCurve.ptE, m_dTol))
	{
		curves.emplace_back(calcCurve);
	}
}

void CBimGeoTwoPolygonBoolOper::RemoveOverlapPolygon(const CBimGeoPolyline* pRefPolygon)
{
	std::list<SBimGeoPolygonWithHole>::iterator iter = m_retPolygon.begin();
	while (iter != m_retPolygon.end())
	{
		EPolygonRel eRet = BimGeoPolylineTool::CalcTwoPolygonRelation(iter->orgPolygon, *pRefPolygon, m_dTol);
		if (EPolygonRel::eOverlap == eRet)
		{
			iter = m_retPolygon.erase(iter);
			break;
		}

		++iter;
	}
}
