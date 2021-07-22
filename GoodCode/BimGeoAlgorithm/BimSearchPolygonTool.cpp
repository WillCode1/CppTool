#include "BimSearchPolygonTool.h"
#include "BimGeoAlgorithmTool.h"
#include "BimGeoPolylineTool.h"
#include "gp_Vec.hxx"
#include "GCE2d_MakeCircle.hxx"
#include "gp_Circ2d.hxx"

double CEqualPointKey::s_tol = 0.0001;
CEqualPointKey::CEqualPointKey(const gp_Pnt2d& pt)
	: m_pt(pt)
{
}

CEqualPointKey::~CEqualPointKey()
{
}

bool CEqualPointKey::operator<(const CEqualPointKey& rhs) const
{
	if (m_pt.IsEqual(rhs.m_pt, s_tol))
	{
		return false;
	}

	if (BimGeoAlgorithmTool::equal(m_pt.X(), rhs.m_pt.X(), s_tol))
	{
		if (BimGeoAlgorithmTool::lessThan(m_pt.Y(), rhs.m_pt.Y(), s_tol))
		{
			return false;
		}
	}
	else if (BimGeoAlgorithmTool::lessThan(m_pt.X(), rhs.m_pt.X(), s_tol))
	{
		return false;
	}

	return true;
}

const gp_Pnt2d& CEqualPointKey::GetPt() const
{
	return m_pt;
}

double CEqualAngleKey::s_tol = 0.0001;
CEqualAngleKey::CEqualAngleKey(const double& dAngle)
	: m_dAngle(dAngle)
{
}

CEqualAngleKey::~CEqualAngleKey()
{
}

bool CEqualAngleKey::operator<(const CEqualAngleKey& rhs) const
{
	return BimGeoAlgorithmTool::lessThan(m_dAngle, rhs.m_dAngle, s_tol);
}

const double& CEqualAngleKey::GetAngle() const
{
	return m_dAngle;
}

CEqualAngleKey::operator double() const
{
	return m_dAngle;
}

CBimSearchPolygonTool::CBimSearchPolygonTool()
	: m_eSearchType(ESearchType::eMinPolygon)
	, m_dTol(0.0001)
	, m_dAngleTol(0.01)
	, m_bIsVectorCurve(false)
{
}

CBimSearchPolygonTool::~CBimSearchPolygonTool()
{
}

void CBimSearchPolygonTool::DoIt(const std::list<SGeoCurveInfo>& curves, const double& dTol,
	const ESearchType eType, const bool IsBreak, const bool IsVectorCurve, const double& dAngleTol)
{
	if (curves.size() < 2)
	{
		return;
	}

	Clear();

	m_dTol = dTol;
	m_dAngleTol = dAngleTol;

	CEqualPointKey::s_tol = dTol;
	CEqualAngleKey::s_tol = m_dAngleTol;

	m_curves = curves;

	m_eSearchType = eType;

	m_bIsVectorCurve = IsVectorCurve;

	// 移除点
	BimGeoAlgorithmTool::RemovePoint(m_curves, m_dTol);
	if (IsBreak)
	{// 打断
		BimGeoAlgorithmTool::BreakCurves(m_curves, m_dTol);
		BimGeoAlgorithmTool::ConnectEqualPoint(m_curves, m_dTol, true);
	}

	BimGeoAlgorithmTool::RemovePoint(m_curves, m_dTol);

	// 去重
	BimGeoAlgorithmTool::RemoveOverlap(m_curves, m_dTol, IsVectorCurve);

	if (!IsVectorCurve)
	{// 无向
		RemoveIndependentCurve();

		int nCount = (int)m_curves.size();
		std::list<SGeoCurveInfo>::iterator itBegin = m_curves.begin();
		while (0 < nCount--)
		{
			SGeoCurveInfo info;
			info.ptS = itBegin->ptE;
			info.ptE = itBegin->ptS;

			if (BimGeoAlgorithmTool::IsArc(itBegin->dBulge))
			{// 弧线段
				info.dBulge = -itBegin->dBulge;
			}
			else
			{// 直线段
				info.dBulge = 0.;
			}

			m_curves.emplace_back(info);

			++itBegin;
		}
	}

	// 统计点与边的信息
	TotalPointWithCurveRel();

	// 搜索
	SearchPolygon();

	// 去除无效的封闭区域
	RemoveInValidPolygon();
}

const std::list<CBimGeoPolyline>& CBimSearchPolygonTool::GetMaxPolygon() const
{
	return m_maxPolygon;
}

const std::list<CBimGeoPolyline>& CBimSearchPolygonTool::GetMinPolygon() const
{
	return m_minPolygon;
}

void CBimSearchPolygonTool::Clear()
{
	m_curves.clear();
	m_maxPolygon.clear();
	m_minPolygon.clear();
	m_mapPointWithCurve.clear();
	m_mapBoundaryPoint.clear();
	m_mapBoundary.clear();
}

void CBimSearchPolygonTool::TotalPointWithCurveRel()
{
	for (std::list<SGeoCurveInfo>::iterator itBegin = m_curves.begin(); itBegin != m_curves.end(); ++itBegin)
	{
		m_mapPointWithCurve[itBegin->ptS].emplace_back(&*itBegin);
	}

	// 去除孤线和统计边界点
	POINTWITHCURVEMAP mapPointBoundary;
	POINTWITHCURVEMAP::iterator itBegin = m_mapPointWithCurve.begin();
	while (itBegin != m_mapPointWithCurve.end())
	{
		switch (itBegin->second.size())
		{
		case 1:
			if (!m_bIsVectorCurve)
			{
				itBegin = m_mapPointWithCurve.erase(itBegin);
			}
			else
			{
				++itBegin;
			}
			break;
		case 2:
		{
			if (ESearchType::eMaxPolygon == m_eSearchType)
			{
				m_mapBoundaryPoint[itBegin->first] = (int)itBegin->second.size();

				for (std::list<SGeoCurveInfo*>::iterator itCurve = itBegin->second.begin(); itCurve != itBegin->second.end(); ++itCurve)
				{
					mapPointBoundary[(*itCurve)->ptS].emplace_back(*itCurve);
				}

				++itBegin;
				break;
			}
		}
		default:
			m_mapBoundaryPoint[itBegin->first] = (int)itBegin->second.size();

			for (std::list<SGeoCurveInfo*>::iterator itCurve = itBegin->second.begin(); itCurve != itBegin->second.end(); ++itCurve)
			{
				mapPointBoundary[(*itCurve)->ptS].emplace_back(*itCurve);
			}

			++itBegin;
			break;
		}
	}

	for (itBegin = mapPointBoundary.begin(); itBegin != mapPointBoundary.end(); ++itBegin)
	{
		m_mapBoundary[itBegin->second.size()].insert(m_mapBoundary[itBegin->second.size()].end(),
			itBegin->second.begin(), itBegin->second.end());
	}
}

void CBimSearchPolygonTool::SearchPolygon()
{
	// 按点查找
	const gp_Pnt2d* pBoundaryPt = GetBoundaryPoint();
	while (NULL != pBoundaryPt)
	{
		POINTWITHCURVEMAP::iterator itFind = m_mapPointWithCurve.find(*pBoundaryPt);
		if (m_mapPointWithCurve.end() != itFind)
		{
			for (std::list<SGeoCurveInfo*>::iterator itCurve = itFind->second.begin(); itCurve != itFind->second.end(); ++itCurve)
			{
				if (NULL == *itCurve)
				{
					continue;
				}

				SearchPolygonImp(*itCurve);
			}
		}

		pBoundaryPt = GetBoundaryPoint();
	}

	// 按边查找
	/*auto pCurve = GetBoundary();
	while (NULL != pCurve)
	{
		SearchPolygonImp(pCurve);

		pCurve = GetBoundary();
	}*/
}

void CBimSearchPolygonTool::SearchPolygonImp(const SGeoCurveInfo* pCurve)
{
	CBimGeoPolyline polygon;
	polygon.AddVertex(pCurve->ptS, pCurve->dBulge);

	std::set<const SGeoCurveInfo*> useCurve;
	SearchPolygon(polygon, pCurve, useCurve);

	if (polygon.IsClose()
		&& IsValidPolyline(polygon)
		&& 1 < polygon.GetVertexes().size())
	{
		if (0 < polygon.Normal().Z())
		{// 逆时针
			if (ESearchType::eMinPolygon == m_eSearchType)
			{
				m_minPolygon.emplace_back(polygon);
				ModifyBoundary(polygon, useCurve);
			}
		}
		else
		{// 顺时针
			if (ESearchType::eMaxPolygon == m_eSearchType)
			{
				polygon.Reverse();
				m_maxPolygon.emplace_back(polygon);
				ModifyBoundary(polygon, useCurve);
			}
		}
	}
}

void CBimSearchPolygonTool::RemoveInValidPolygon()
{
	std::list<CBimGeoPolyline>::iterator itBeign = m_maxPolygon.begin();
	while (itBeign != m_maxPolygon.end())
	{
		Bnd_Box2d box = itBeign->GetBndBox();
		double dXmin = 0., dYmin = 0., dXmax = 0., dYmax = 0;
		box.Get(dXmin, dYmin, dXmax, dYmax);

		std::list<CBimGeoPolyline>::iterator itNext = itBeign;
		++itNext;

		bool bIsContinue = false;
		while (itNext != m_maxPolygon.end())
		{
			Bnd_Box2d nextBox = itNext->GetBndBox();
			double dOtherXmin = 0., dOtherYmin = 0., dOtherXmax = 0., dOtherYmax = 0;
			nextBox.Get(dOtherXmin, dOtherYmin, dOtherXmax, dOtherYmax);

			if (box.IsOut(nextBox))
			{// 相离
				++itNext;
				continue;
			}
			else if (BimGeoAlgorithmTool::lessEqualThan(dXmin, dOtherXmin)
				&& BimGeoAlgorithmTool::lessEqualThan(dYmin, dOtherYmin)
				&& BimGeoAlgorithmTool::lessEqualThan(dOtherXmax, dXmax)
				&& BimGeoAlgorithmTool::lessEqualThan(dOtherYmax, dYmax))
			{// box 包含 nextbox
				itNext = m_maxPolygon.erase(itNext);
				continue;
			}
			else if (BimGeoAlgorithmTool::lessEqualThan(dOtherXmin, dXmin)
				&& BimGeoAlgorithmTool::lessEqualThan(dOtherYmin, dYmin)
				&& BimGeoAlgorithmTool::lessEqualThan(dXmax, dOtherXmax)
				&& BimGeoAlgorithmTool::lessEqualThan(dYmax, dOtherYmax))
			{// nextbox 包含 box
				bIsContinue = true;
				itBeign = m_maxPolygon.erase(itBeign);
				break;
			}
			//else
			//{// 相交
			//	auto eRet = BimGeoPolylineTool::CalcTwoPolygonRelation(*itBeign, *itNext);
			//	if (EPolygonRel::eAInB == eRet
			//		|| EPolygonRel::eAInBWithTagent == eRet)
			//	{
			//		bIsContinue = true;
			//		itBeign = m_maxPolygon.erase(itBeign);
			//		break;
			//	}
			//	else if (EPolygonRel::eBInA == eRet
			//		|| EPolygonRel::eBInAWithTagent == eRet
			//		|| EPolygonRel::eOverlap == eRet)
			//	{
			//		itNext = m_maxPolygon.erase(itNext);
			//		continue;
			//	}
			//}

			++itNext;
		}

		if (bIsContinue)
		{
			continue;
		}

		++itBeign;
	}

	itBeign = m_minPolygon.begin();
	while (itBeign != m_minPolygon.end())
	{
		double dArea = BimGeoPolylineTool::GetPolygonArea(*itBeign);
		double dXmin = 0., dYmin = 0., dXmax = 0., dYmax = 0;
		{
			Bnd_Box2d box = itBeign->GetBndBox();
			box.Get(dXmin, dYmin, dXmax, dYmax);
		}

		std::list<CBimGeoPolyline>::iterator itNext = itBeign;
		++itNext;

		bool bIsContinue = false;
		while (itNext != m_minPolygon.end())
		{
			double dOtherXmin = 0., dOtherYmin = 0., dOtherXmax = 0., dOtherYmax = 0;
			{
				Bnd_Box2d box = itNext->GetBndBox();
				box.Get(dOtherXmin, dOtherYmin, dOtherXmax, dOtherYmax);
			}

			if (BimGeoAlgorithmTool::equal(dXmin, dOtherXmin)
				&& BimGeoAlgorithmTool::equal(dYmin, dOtherYmin)
				&& BimGeoAlgorithmTool::equal(dXmax, dOtherXmax)
				&& BimGeoAlgorithmTool::equal(dYmax, dOtherYmax))
			{
				double dOtherArea = BimGeoPolylineTool::GetPolygonArea(*itNext);
				if (BimGeoAlgorithmTool::equal(dArea, dOtherArea))
				{
					itNext = m_minPolygon.erase(itNext);
					continue;
				}
			}

			/*	auto eRet = BimGeoPolylineTool::CalcTwoPolygonRelation(*itBeign, *itNext);
				if (EPolygonRel::eOverlap == eRet)
				{
					itNext = m_minPolygon.erase(itNext);
					continue;
				}*/

			++itNext;
		}

		if (bIsContinue)
		{
			continue;
		}

		++itBeign;
	}

	// 去除共线
	MergeCollinear(m_maxPolygon);
	MergeCollinear(m_minPolygon);
}

const gp_Pnt2d* CBimSearchPolygonTool::GetBoundaryPoint()
{
	if (m_mapBoundaryPoint.size() < 1)
	{
		return NULL;
	}

	std::map<CEqualPointKey, int>::iterator itBoundaryPoint = m_mapBoundaryPoint.begin();
	std::map<CEqualPointKey, int>::iterator itBegin = itBoundaryPoint;
	while (itBegin != m_mapBoundaryPoint.end())
	{
		if (itBegin->second < 1)
		{
			if (itBoundaryPoint == itBegin)
			{
				itBoundaryPoint = m_mapBoundaryPoint.erase(itBegin);
				itBegin = itBoundaryPoint;
			}
			else
			{
				itBegin = m_mapBoundaryPoint.erase(itBegin);
			}

			continue;
		}

		if (itBoundaryPoint->second < itBegin->second)
		{
			itBoundaryPoint = itBegin;
		}

		++itBegin;
	}

	if (0 < itBoundaryPoint->second)
	{
		itBoundaryPoint->second = 0;
		return &itBoundaryPoint->first.GetPt();
	}

	return NULL;
}

const SGeoCurveInfo* CBimSearchPolygonTool::GetBoundary()
{
	MAPBOUNDARY::reverse_iterator itBegin = m_mapBoundary.rbegin();
	while (m_mapBoundary.rend() != itBegin)
	{
		for (std::list<SGeoCurveInfo*>::iterator itCurve = itBegin->second.begin(); itCurve != itBegin->second.end(); ++itCurve)
		{
			if (NULL != *itCurve)
			{
				SGeoCurveInfo* pRetCurve = *itCurve;
				*itCurve = NULL;
				return pRetCurve;
			}
		}

		++itBegin;
	}

	return NULL;
}

void CBimSearchPolygonTool::SearchPolygon(CBimGeoPolyline& polygon, const SGeoCurveInfo* pCurve, std::set<const SGeoCurveInfo*>& useCurve) const
{
	std::list<const SGeoCurveInfo*> polygonCurves;
	bool bFind = false;
	const SGeoCurveInfo* pStartCurve = pCurve;
	while (true)
	{
		if (NULL == pStartCurve)
		{
			break;
		}

		POINTWITHCURVEMAP::const_iterator itFind = m_mapPointWithCurve.find(pStartCurve->ptE);
		if (m_mapPointWithCurve.end() == itFind)
		{
			break;
		}

		std::map<CEqualAngleKey, std::list<const SGeoCurveInfo*>> mapCurveAngle;
		for (std::list<SGeoCurveInfo*>::const_iterator itCurve = itFind->second.begin(); itCurve != itFind->second.end(); ++itCurve)
		{
			if (NULL == *itCurve)
			{
				continue;
			}

			double dAngle = GetAngle(*itCurve);
			mapCurveAngle[dAngle].emplace_back(*itCurve);
		}

		SGeoCurveInfo info;
		info.ptS = pStartCurve->ptE;
		info.ptE = pStartCurve->ptS;
		info.dBulge = -pStartCurve->dBulge;
		double dAngle = GetAngle(&info);
		{
			bool bAdd = true;
			std::map<CEqualAngleKey, std::list<const SGeoCurveInfo*>>::iterator itFindReverse = mapCurveAngle.find(dAngle);
			if (mapCurveAngle.end() != itFindReverse)
			{
				for (std::list<const SGeoCurveInfo*>::iterator itCurve = itFindReverse->second.begin(); itCurve != itFindReverse->second.end(); ++itCurve)
				{
					const SGeoCurveInfo* pCurveInfo = *itCurve;
					if (info.ptS.IsEqual(pCurveInfo->ptS, m_dTol)
						&& info.ptE.IsEqual(pCurveInfo->ptE, m_dTol)
						&& BimGeoAlgorithmTool::equal(info.dBulge, pCurveInfo->dBulge))
					{
						bAdd = false;
						break;
					}
				}
			}

			if (bAdd)
			{
				mapCurveAngle[dAngle].emplace_back(&info);
			}
		}

		bool bIsReturn = false;
		switch (mapCurveAngle.size())
		{
		case 0:
			bIsReturn = true;
			break;
		case 1:
			if (mapCurveAngle.begin()->second.size() < 2)
			{// 只有自己的反向
				bIsReturn = true;
			}
			break;
		default:
			break;
		}

		if (bIsReturn)
		{
			break;
		}

		double dRetAngle = 0., dCurAngle = 0.;
		const SGeoCurveInfo* pRefCurve = pStartCurve;

		std::list<const SGeoCurveInfo*>* pCurves = NULL;
		std::map<CEqualAngleKey, std::list<const SGeoCurveInfo*>>::iterator itBegin = mapCurveAngle.begin();
		for (; mapCurveAngle.end() != itBegin; ++itBegin)
		{
			if (BimGeoAlgorithmTool::lessEqualThan(dAngle, itBegin->first, m_dAngleTol))
			{
				if (1 < itBegin->second.size())
				{
					const SGeoCurveInfo* pFirstCurve = NULL;
					double dAngle2 = 0.;// 类似于二阶导数
					const SGeoCurveInfo* pNextCurve = GetNextCurve(dRetAngle, dAngle2, pFirstCurve, itBegin->second, itBegin->first, dAngle, pRefCurve);
					if (IsReverse(pFirstCurve, pStartCurve) && 1 < mapCurveAngle.size())
					{
						if (NULL != pCurves)
						{// 取前一个
							break;
						}

						// 取最后一个
						std::list<const SGeoCurveInfo*> tempCurves = mapCurveAngle.rbegin()->second;
						double dRetAngleTemp = 0., dAngle2Temp = 0.;
						const SGeoCurveInfo* pNextCurveTemp = GetNextCurve(dRetAngleTemp, dAngle2Temp, pFirstCurve, tempCurves, mapCurveAngle.rbegin()->first, dAngle, pRefCurve);

						if (BimGeoAlgorithmTool::lessEqualThan(dRetAngle, mapCurveAngle.rbegin()->first)
							|| BimGeoAlgorithmTool::lessEqualThan(dRetAngle, dRetAngleTemp))
						{
							dCurAngle = mapCurveAngle.rbegin()->first;
							pCurves = &mapCurveAngle.rbegin()->second;
						}
						else
						{
							dCurAngle = itBegin->first;
							pCurves = &itBegin->second;
						}

						break;
					}
					else if (!IsReverse(pNextCurve, pStartCurve))
					{
						/*
						dRetAngle < dAngle2 && dAngle2 < dNextAngle 取前面一个
						*/

						if (NULL == pCurves
							|| BimGeoAlgorithmTool::lessEqualThan(dRetAngle, dAngle)
							|| BimGeoAlgorithmTool::lessEqualThan(dRetAngle, dAngle2))
						{
							dCurAngle = itBegin->first;
							pCurves = &itBegin->second;
						}

						/*
						dRetAngle < dAngle2 && dNextAngle < dAngle2	取小于dAngle2最近的一个
						*/
						if (BimGeoAlgorithmTool::IsArc(pRefCurve->dBulge))
						{// 取后面的数据
							std::map<CEqualAngleKey, std::list<const SGeoCurveInfo*>>::iterator itNext = ++itBegin;
							while (mapCurveAngle.end() != itNext)
							{
								if (!BimGeoAlgorithmTool::lessThan(itNext->first, dAngle2))
								{
									break;
								}

								dCurAngle = itNext->first;
								pCurves = &itNext->second;

								++itNext;
							}
						}

						break;
					}
				}

				if (NULL == pCurves)
				{
					dCurAngle = mapCurveAngle.rbegin()->first;
					pCurves = &mapCurveAngle.rbegin()->second;
				}

				break;
			}

			dCurAngle = itBegin->first;
			pCurves = &itBegin->second;
		}

		if (NULL == pCurves)
		{
			break;
		}

		const SGeoCurveInfo* pFirstCurve = NULL;
		double dAngle2 = 0.;// 类似于二阶导数
		pStartCurve = GetNextCurve(dRetAngle, dAngle2, pFirstCurve, *pCurves, dCurAngle, dAngle, pRefCurve);
		if (NULL == pStartCurve
			|| pStartCurve == &info
			|| useCurve.end() != useCurve.find(pStartCurve))
		{
			break;
		}

		if (1 < polygon.GetVertexes().size())
		{
			const std::vector<gp_Pnt2d>& vertexes = polygon.GetVertexes();
			if (vertexes[0].IsEqual(pStartCurve->ptS, m_dTol))
			{
				polygon.setClose(true);
				return;
			}
		}

		useCurve.insert(pStartCurve);
		polygonCurves.emplace_back(pStartCurve);

		if (0 < polygon.GetVertexes().size()
			&& polygon.GetVertexes().rbegin()->IsEqual(pStartCurve->ptE, m_dTol))
		{
			bFind = true;

			break;
		}
	}

	if (bFind)
	{
		for (std::list<const SGeoCurveInfo*>::iterator itCurve = polygonCurves.begin(); 
			itCurve != polygonCurves.end(); ++itCurve)
		{
			polygon.AddVertex((*itCurve)->ptS, (*itCurve)->dBulge);
		}

		polygon.setClose(true);
	}
}

double CBimSearchPolygonTool::GetAngle(const SGeoCurveInfo* pCurve) const
{
	gp_Vec2d vecNormal(1., 0.);
	if (BimGeoAlgorithmTool::IsArc(pCurve->dBulge, m_dTol))
	{
		gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(pCurve->ptS, pCurve->ptE, pCurve->dBulge);
		GCE2d_MakeCircle arc(pCurve->ptS, ptMid, pCurve->ptE);

		gp_Pnt2d ptCenter = arc.Value()->Location();
		gp_Vec2d vecCurve(ptCenter, pCurve->ptS);
		vecCurve.Normalize();

		vecNormal = gp_Vec2d(-vecCurve.Y(), vecCurve.X());
		if (!arc.Value()->Circ2d().IsDirect())
		{// 顺时针
			vecNormal = -vecNormal;
		}
	}
	else
	{
		vecNormal = gp_Vec2d(pCurve->ptS, pCurve->ptE);
		vecNormal.Normalize();
	}

	double d2PI = M_PI * 2;
	double dAngle = gp_Vec2d(1., 0.).Angle(vecNormal);
	while (dAngle < 0.)
	{
		dAngle += d2PI;
	}

	if (BimGeoAlgorithmTool::lessEqualThan(d2PI, dAngle))
	{
		dAngle -= d2PI;
	}

	return dAngle;
}

const SGeoCurveInfo* CBimSearchPolygonTool::GetNextCurve(double& dRetAngle, double& dCurAngle2, const SGeoCurveInfo*& pFirstCurve, const std::list<const SGeoCurveInfo*>& curves,
	const double& dCurAngle, const double& dRefAngle, const SGeoCurveInfo* pRefCurve) const
{
	dCurAngle2 = dRetAngle = dCurAngle;
	if (1 == curves.size())
	{
		return *curves.begin();
	}
	else if (0 < curves.size())
	{
		double dCalcAngle = dRefAngle;
		std::map<double, const SGeoCurveInfo*> mapCurveAngle;
		for (std::list<const SGeoCurveInfo*>::const_iterator itCurve = curves.begin();
			itCurve != curves.end(); ++itCurve)
		{
			const SGeoCurveInfo* pCurve  = *itCurve;
			if (BimGeoAlgorithmTool::IsArc(pCurve->dBulge, m_dTol))
			{
				gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(pCurve->ptS, pCurve->ptE, pCurve->dBulge);
				gp_Vec2d vecCurve(pCurve->ptS, ptMid);
				vecCurve.Normalize();

				double dAngle = gp_Vec2d(1., 0.).Angle(vecCurve);
				while (dAngle < 0.)
				{
					dAngle += M_PI * 2;
				}

				mapCurveAngle[dAngle] = pCurve;

				if (NULL != pRefCurve && IsReverse(pRefCurve, pCurve))
				{// 反向的线段
					dCurAngle2 = dAngle;
					dCalcAngle = dAngle;
				}
			}
			else
			{
				mapCurveAngle[dCurAngle] = pCurve;
			}
		}

		pFirstCurve = NULL;
		const SGeoCurveInfo* pCurve = NULL;
		for (std::map<double, const SGeoCurveInfo*>::iterator itData = mapCurveAngle.begin();
			itData != mapCurveAngle.end(); ++itData)
		{
			std::map<double, const SGeoCurveInfo*>::iterator::value_type& data = *itData;
			if (NULL == pFirstCurve)
			{
				pFirstCurve = data.second;
			}

			if (BimGeoAlgorithmTool::lessEqualThan(dCalcAngle, data.first, m_dAngleTol))
			{
				if (NULL == pCurve)
				{
					dRetAngle = mapCurveAngle.rbegin()->first;
					pCurve = mapCurveAngle.rbegin()->second;
				}
				else if ((BimGeoAlgorithmTool::equal(dCalcAngle, data.first))
					&& (NULL == pRefCurve || !IsReverse(data.second, pRefCurve)))
				{// 非反向的线段
					dRetAngle = mapCurveAngle.rbegin()->first;
					pCurve = mapCurveAngle.rbegin()->second;
				}

				break;
			}

			dRetAngle = data.first;
			pCurve = data.second;
		}

		return pCurve;
	}

	return NULL;
}

void CBimSearchPolygonTool::ModifyBoundary(const CBimGeoPolyline& polyline, const std::set<const SGeoCurveInfo*>& useCurve)
{
	for (int i = 0; i < polyline.GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptS, ptE;
		double dBulge = 0.;
		polyline.GetCurve(ptS, ptE, dBulge, i);

		std::map<CEqualPointKey, int>::iterator itFindPoint = m_mapBoundaryPoint.find(ptS);
		if (m_mapBoundaryPoint.end() != itFindPoint)
		{
			--itFindPoint->second;
		}

		POINTWITHCURVEMAP::iterator itFind = m_mapPointWithCurve.find(ptS);
		if (m_mapPointWithCurve.end() != itFind)
		{
			std::list<SGeoCurveInfo*>::iterator itBegin = itFind->second.begin();
			while (itFind->second.end() != itBegin)
			{
				if (useCurve.end() != useCurve.find(*itBegin))
				{
					*itBegin = NULL;
					break;
				}

				++itBegin;
			}
		}

		if (ESearchType::eMaxPolygon == m_eSearchType)
		{
			itFind = m_mapPointWithCurve.find(ptE);
			if (m_mapPointWithCurve.end() != itFind)
			{
				std::list<SGeoCurveInfo*>::iterator itBegin = itFind->second.begin();
				while (itFind->second.end() != itBegin)
				{
					if (NULL != *itBegin
						&& ptE.IsEqual((*itBegin)->ptS, m_dTol)
						&& BimGeoAlgorithmTool::equal(-dBulge, (*itBegin)->dBulge))
					{
						*itBegin = NULL;
						break;
					}

					++itBegin;
				}
			}
		}
	}
}

bool CBimSearchPolygonTool::IsValidPolyline(const CBimGeoPolyline& polyline) const
{
	if (polyline.GetVertexes().size() < 2)
	{
		return false;
	}
	else if (2 == polyline.GetVertexes().size())
	{
		if (!BimGeoAlgorithmTool::IsArc(polyline.GetBulges()[0], m_dTol)
			&& !BimGeoAlgorithmTool::IsArc(polyline.GetBulges()[1], m_dTol))
		{
			return false;
		}
	}

	std::map<CEqualPointKey, int> mapVertexCount;
	for (int i = 0; i < polyline.GetVertexes().size(); ++i)
	{
		mapVertexCount[polyline.GetVertexes()[i]]++;
		if (1 < mapVertexCount[polyline.GetVertexes()[i]])
		{
			return false;
		}
	}

	return true;
}

bool CBimSearchPolygonTool::IsReverse(const SGeoCurveInfo* pCurve, const SGeoCurveInfo* pRefCurve) const
{
	if (pCurve != pRefCurve
		&& pRefCurve->ptS.IsEqual(pCurve->ptE, m_dTol)
		&& pRefCurve->ptE.IsEqual(pCurve->ptS, m_dTol)
		&& BimGeoAlgorithmTool::equal(-pRefCurve->dBulge, pCurve->dBulge, m_dTol))
	{// 反向的线段
		return true;
	}

	return false;
}

void CBimSearchPolygonTool::RemoveIndependentCurve()
{
	std::map<CEqualPointKey, int> mapVertexCount;
	for (std::list<SGeoCurveInfo>::iterator itCurve = m_curves.begin();
		itCurve != m_curves.end(); ++itCurve)
	{
		mapVertexCount[itCurve->ptS]++;
		mapVertexCount[itCurve->ptE]++;
	}

	bool bEraseCurve = false;
	do
	{
		bEraseCurve = false;
		std::list<SGeoCurveInfo>::iterator itBegin = m_curves.begin();
		while (m_curves.end() != itBegin)
		{
			if (mapVertexCount[itBegin->ptS] < 2
				|| mapVertexCount[itBegin->ptE] < 2)
			{
				bEraseCurve = true;
				mapVertexCount[itBegin->ptS]--;
				mapVertexCount[itBegin->ptE]--;

				itBegin = m_curves.erase(itBegin);
				continue;
			}

			++itBegin;
		}
	} while (bEraseCurve);
}

void CBimSearchPolygonTool::MergeCollinear(std::list<CBimGeoPolyline>& polylines) const
{
	std::list<CBimGeoPolyline>::iterator itBegin = polylines.begin();
	while (polylines.end() != itBegin)
	{
		BimGeoPolylineTool::MergePolylineSide(*itBegin);

		if (itBegin->GetVertexes().size() < 1)
		{
			itBegin = polylines.erase(itBegin);
			continue;
		}

		++itBegin;
	}
}
