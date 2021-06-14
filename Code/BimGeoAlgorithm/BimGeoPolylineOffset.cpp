#include "BimGeoPolylineOffset.h"
#include "BimGeoAlgorithm/BimGeoAlgorithmTool.h"
#include "BimGeoAlgorithm/BimGeoPolylineTool.h"
#include "GCE2d_MakeCircle.hxx"
#include "gp_Circ2d.hxx"
#include "gp_Pnt2d.hxx"
#include "gp_Vec2d.hxx"
#include "gp_Dir2d.hxx"

CBimGeoPolylineOffset::CBimGeoPolylineOffset()
	: m_dTol(0.0001)
{
}

CBimGeoPolylineOffset::~CBimGeoPolylineOffset()
{
}

bool CBimGeoPolylineOffset::DoIt(const CBimGeoPolyline& polyline, const std::vector<double>& vecOffset, const double& dTol)
{
	Clear();

	if (polyline.GetVertexes().size() < 2)
	{
		return false;
	}

	m_dTol = dTol;

	return CalcOffsetImp(polyline, vecOffset);
}

bool CBimGeoPolylineOffset::DoIt(const CBimGeoPolyline& polyline, const double& dOffset, const double& dTol)
{
	Clear();

	if (BimGeoAlgorithmTool::equal(dOffset, 0., dTol))
	{
		return false;
	}

	if (polyline.GetVertexes().size() < 2)
	{
		return false;
	}

	m_dTol = dTol;

	std::vector<double> vecOffset;
	int nCount = polyline.GetVertexes().size();
	for (int i = 0; i < nCount; ++i)
	{
		vecOffset.emplace_back(dOffset);
	}

	return CalcOffsetImp(polyline, vecOffset);
}

const std::list<CBimGeoPolyline>& CBimGeoPolylineOffset::GetOffsetPolylines() const
{
	return m_retPolylines;
}

void CBimGeoPolylineOffset::Clear()
{
	m_retPolylines.clear();
	m_inValidVertex.clear();
}

bool CBimGeoPolylineOffset::CalcOffsetImp(const CBimGeoPolyline& polyline, const std::vector<double>& vecOffset)
{
	m_orgPolyline = polyline;
	m_vecOffset = vecOffset;

	m_orgPolyline = GetValidPolyline(m_vecOffset, m_orgPolyline);
	if (m_vecOffset.size() != m_orgPolyline.GetBulges().size()
		|| m_orgPolyline.GetVertexes().size() < 2)
	{
		return false;
	}

	BoundaryDegenerate();

	GeneratePolylines();

	return 0 < m_retPolylines.size();
}

CBimGeoPolyline CBimGeoPolylineOffset::GetValidPolyline(std::vector<double>& vecOffset, const CBimGeoPolyline& polyline) const
{
	/*	1、过滤顶点坐标闭合且闭合表示不为true的情况；
		2、过滤顶点坐标闭合且闭合表示为true的情况；									
	*/
	std::vector<gp_Pnt2d> vertexes = polyline.GetVertexes();
	std::vector<double> bulges = polyline.GetBulges();
	bool bIsClose = polyline.IsClose();

	if (vertexes.begin()->IsEqual(*vertexes.rbegin(), m_dTol))
	{
		bIsClose = true;
		vertexes.pop_back();
		bulges.pop_back();
		vecOffset.pop_back();
	}

	CBimGeoPolyline validPolyline(vertexes, bulges, bIsClose);
	return validPolyline;
}

void CBimGeoPolylineOffset::BoundaryDegenerate()
{
	const std::vector<gp_Pnt2d>& vecVertex = m_orgPolyline.GetVertexes();
	int nCount = (int)vecVertex.size();
	if (nCount < 2)
	{
		return;
	}

	if (!m_orgPolyline.IsClose())
	{
		if (BimGeoAlgorithmTool::IsArc(m_orgPolyline.GetBulges()[0], m_dTol))
		{
			m_vecOffsetVertex.emplace_back(gp_Vec2d(0., 0.));
			m_arcPointSet.insert(0);
		}
		else
		{
			gp_Vec2d vec(vecVertex[0], vecVertex[1]);
			vec.Normalize();

			vec *= m_vecOffset[0];
			vec.Rotate(M_PI_2);

			m_vecOffsetVertex.emplace_back(vec);
		}
	}
	else
	{
		CalcVertexOffsetVec(nCount - 1, 0, 1);
	}

	for (int i = 1; i < nCount; ++i)
	{
		int j = i + 1;
		if (j == nCount)
		{
			if (m_orgPolyline.IsClose())
			{
				j = 0;
			}
			else
			{
				break;
			}
		}

		CalcVertexOffsetVec(i - 1, i, j);
	}

	if (!m_orgPolyline.IsClose())
	{
		if (BimGeoAlgorithmTool::IsArc(m_orgPolyline.GetBulges()[nCount - 2], m_dTol))
		{
			m_vecOffsetVertex.emplace_back(gp_Vec2d(0., 0.));
			m_arcPointSet.insert(nCount - 1);
		}
		else
		{
			gp_Vec2d vec(vecVertex[nCount - 2], vecVertex[nCount - 1]);
			vec.Normalize();

			vec *= m_vecOffset[nCount - 2];
			vec.Rotate(M_PI_2);

			m_vecOffsetVertex.emplace_back(vec);
		}
	}

	// 去除退化的边
	RemoveDegenerateBoundary();
}

void CBimGeoPolylineOffset::RemoveDegenerateBoundary()
{
	const std::vector<gp_Pnt2d>& vecVertex = m_orgPolyline.GetVertexes();
	const std::vector<double>& vecBulge = m_orgPolyline.GetBulges();

	int nCount = (int)vecVertex.size();
	for (int i = 0; i < nCount; ++i)
	{
		int j = i + 1;
		if (j == nCount)
		{
			if (m_orgPolyline.IsClose())
			{
				j = 0;
			}
			else
			{
				break;
			}
		}

		if (m_arcPointSet.end() != m_arcPointSet.find(i)
			|| m_arcPointSet.end() != m_arcPointSet.find(j))
		{// 圆弧时先不考虑
			continue;
		}

		if (BimGeoAlgorithmTool::equal(m_vecOffset[i], 0., m_dTol)
			|| BimGeoAlgorithmTool::equal(m_vecOffsetVertex[i].Magnitude(), 0., m_dTol)
			|| BimGeoAlgorithmTool::equal(m_vecOffsetVertex[j].Magnitude(), 0., m_dTol))
		{// 不偏移
			continue;
		}

		gp_Dir2d dir(m_vecOffsetVertex[i].X(), m_vecOffsetVertex[i].Y()), dirRef(m_vecOffsetVertex[j].X(), m_vecOffsetVertex[j].Y());

		std::vector<gp_Pnt2d> vecIntersect;
		BimGeoAlgorithmTool::CalcLineIntersect(vecIntersect, vecVertex[i], dir, vecVertex[j], dirRef, m_dTol);
		if (0 < vecIntersect.size())
		{
			gp_Pnt ptIntersect(vecIntersect[0].X(), vecIntersect[0].Y(), 0.);
			gp_Pnt ptS(vecVertex[i].X(), vecVertex[i].Y(), 0.), ptE(vecVertex[j].X(), vecVertex[j].Y(), 0.);
			gp_Pnt ptProject;
			if (BimGeoAlgorithmTool::GetProjectPt(ptProject, ptIntersect, ptS, ptE, vecBulge[i], m_dTol))
			{
				double dMaxValue = ptProject.Distance(ptIntersect);
				bool bIsLeft = (0 == BimGeoAlgorithmTool::IsLeftOfCurve(ptIntersect, ptS, ptE, vecBulge[i], m_dTol));
				if (BimGeoAlgorithmTool::lessThan(m_vecOffset[i], 0))
				{// 小于0 外扩
					if (!bIsLeft
						&& BimGeoAlgorithmTool::lessEqualThan(dMaxValue, abs(m_vecOffset[i]), m_dTol))
					{
						m_inValidVertex.insert(i);
						m_inValidVertex.insert(j);
					}
				}
				else
				{// 大于0 内缩
					if (bIsLeft
						&& BimGeoAlgorithmTool::lessEqualThan(dMaxValue, abs(m_vecOffset[i]), m_dTol))
					{
						m_inValidVertex.insert(i);
						m_inValidVertex.insert(j);
					}
				}
			}
		}
	}
}

void CBimGeoPolylineOffset::CalcVertexOffsetVec(const int nPreIndex, const int nCurIndex, const int nNextIndex)
{
	gp_Pnt2d preVertex = m_orgPolyline.GetVertexes()[nPreIndex];
	gp_Pnt2d curVertex = m_orgPolyline.GetVertexes()[nCurIndex];
	gp_Pnt2d nextVertex = m_orgPolyline.GetVertexes()[nNextIndex];

	if (!BimGeoAlgorithmTool::IsArc(m_orgPolyline.GetBulges()[nCurIndex])
		&& !BimGeoAlgorithmTool::IsArc(m_orgPolyline.GetBulges()[nPreIndex]))
	{// 直线
		gp_Vec2d vec1(curVertex, preVertex), vec2(curVertex, nextVertex);
		if (BimGeoAlgorithmTool::equal(0., vec1.Magnitude())
			|| BimGeoAlgorithmTool::equal(0., vec2.Magnitude()))
		{
			m_vecOffsetVertex.emplace_back(gp_Vec2d(0., 0.));
			return;
		}

		vec1.Normalize();
		vec2.Normalize();

		gp_Vec2d vecNormal = vec1 * m_vecOffset[nCurIndex] + vec2 * m_vecOffset[nPreIndex];		//	偏移法向量
		
		double dAngle = (-vec1).Angle(vec2);
		if (dAngle < 0)
		{
			dAngle += M_PI * 2;
		}

		if (BimGeoAlgorithmTool::equal(0., dAngle, m_dTol))
		{// 共线
			vecNormal = gp_Vec2d(-vec2.Y(), vec2.X()) * m_vecOffset[nCurIndex];
		}

		if (BimGeoAlgorithmTool::lessThan(M_PI, dAngle))
		{// 优角反向
			vecNormal *= -1;
		}

		m_vecOffsetVertex.emplace_back(vecNormal);
	}
	else
	{// 圆弧
		m_vecOffsetVertex.emplace_back(gp_Vec2d(0., 0.));
		m_arcPointSet.insert(nCurIndex);
	}
}

void CBimGeoPolylineOffset::GeneratePolylines()
{
	int nCount = m_orgPolyline.GetVertexes().size();
	if (nCount < 2)
	{
		return;
	}

	bool bIsPreVertexValid = true;
	CBimGeoPolyline retPolyline;
	std::set<int> inValidSide;

	for (int i = 0; i < nCount; ++i)
	{
		bool bIsBreak = false;
		if (m_inValidVertex.end() != m_inValidVertex.find(i))
		{
			if (!m_orgPolyline.IsClose())
			{
				if (!bIsPreVertexValid)
				{
					continue;
				}

				bIsBreak = true;
			}
			else
			{
				continue;
			}
		}

		int nCurCount = retPolyline.GetVertexes().size();
		if (m_arcPointSet.end() != m_arcPointSet.find(i))
		{// 圆弧端点
			int nPreIndex = i - 1;
			if (nPreIndex < 0
				&& m_orgPolyline.IsClose())
			{
				nPreIndex = m_orgPolyline.GetVertexes().size() - 1;
			}

			OffsetArc(retPolyline, inValidSide, nPreIndex, i);

			if (nCurCount != retPolyline.GetVertexes().size())
			{
				bIsPreVertexValid = true;
			}
		}
		else
		{
			bIsPreVertexValid = true;

			gp_Pnt2d vertex = m_orgPolyline.GetVertexes()[i].Coord() + m_vecOffsetVertex[i].XY();
			retPolyline.AddVertex(vertex, 0.);
		}

		if (bIsBreak)
		{
			break;
		}
	}

	BimGeoPolylineTool::RemovePoint(retPolyline);

	retPolyline.setClose(m_orgPolyline.IsClose());
	if (1 < retPolyline.GetVertexes().size())
	{
		if (m_orgPolyline.IsClose())
		{
			gp_Vec vecNormalRet = retPolyline.Normal(), vecNormal = m_orgPolyline.Normal();
			bool bIsSameDirect = false;

			if (BimGeoAlgorithmTool::equal(vecNormalRet.Magnitude(), 0.)
				&& BimGeoAlgorithmTool::equal(vecNormal.Magnitude(), 0.))
			{
				bIsSameDirect = true;
			}
			else if (BimGeoAlgorithmTool::equal(vecNormalRet.Magnitude(), 0.)
				|| BimGeoAlgorithmTool::equal(vecNormal.Magnitude(), 0.))
			{
				bIsSameDirect = false;
			}
			else
			{
				double dAngle = retPolyline.Normal().AngleWithRef(m_orgPolyline.Normal(), gp_Vec(0., 0., 1.));
				if (BimGeoAlgorithmTool::lessEqualThan(0, dAngle, m_dTol)
					&& BimGeoAlgorithmTool::lessThan(dAngle, M_PI_2, m_dTol))
				{// 同向
					bIsSameDirect = true;
				}
			}

			if (bIsSameDirect)
			{
				CalcValidPolylines(retPolyline, inValidSide);
			}
		}
		else
		{
			CalcValidPolylines(retPolyline, inValidSide);
		}
	}
}

void CBimGeoPolylineOffset::CalcValidPolylines(const CBimGeoPolyline& polyline, const std::set<int>& inValidSide)
{
	if (0 < inValidSide.size()
		|| BimGeoPolylineTool::IsSelfIntersect(polyline, m_dTol))
	{
		bool IsClose = polyline.IsClose();

		std::list<SGeoCurveInfo> curves;
		for (int i = 0; i < polyline.GetCurveNumber(); ++i)
		{
			if (inValidSide.end() != inValidSide.find(i))
			{
				continue;
			}

			SGeoCurveInfo data;
			polyline.GetCurve(data.ptS, data.ptE, data.dBulge, i);

			curves.emplace_back(data);
		}

		std::list<CBimGeoPolyline> polygons = BimGeoPolylineTool::SearchMinPolygonWithBreakCurve(curves, m_dTol, true);
		if (0 < polygons.size())
		{
			m_retPolylines.insert(m_retPolylines.end(), polygons.begin(), polygons.end());
		}
	}
	else
	{
		m_retPolylines.emplace_back(polyline);
	}
}

void CBimGeoPolylineOffset::OffsetArc(CBimGeoPolyline& retPolyline, std::set<int>& inValidSide, const int nPreIndex, const int nIndex) const
{
	gp_Pnt2d ptPreS, ptPreE, ptS, ptE;
	double dPreBulge = 0., dBulge = 0.;

	if (nPreIndex < 0)
	{
		m_orgPolyline.GetCurve(ptS, ptE, dBulge, nIndex);

		bool bIsValid = false;
		SGeoCurveInfo curve = OffsetCurve(ptS, ptE, dBulge, m_vecOffset[nIndex], bIsValid);

		retPolyline.AddVertex(curve.ptS, curve.dBulge);

		return;
	}
	else if (nIndex == m_orgPolyline.GetCurveNumber())
	{
		m_orgPolyline.GetCurve(ptPreS, ptPreE, dPreBulge, nPreIndex);

		bool bIsPreValid = false;
		SGeoCurveInfo preCurve = OffsetCurve(ptPreS, ptPreE, dPreBulge, m_vecOffset[nPreIndex], bIsPreValid);

		if (BimGeoAlgorithmTool::IsArc(preCurve.dBulge))
		{// 圆弧
			gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(preCurve.ptS, preCurve.ptE, preCurve.dBulge);
			GCE2d_MakeCircle arc(*retPolyline.GetVertexes().rbegin(), ptMid, preCurve.ptE);

			double dBugle = BimGeoAlgorithmTool::CalcArcBulge(*retPolyline.GetVertexes().rbegin(),
				preCurve.ptE, arc.Value()->Location(), arc.Value()->Radius(), arc.Value()->Circ2d().IsDirect(), m_dTol);

			retPolyline.SetVertex(retPolyline.GetVertexes().size() - 1, *retPolyline.GetVertexes().rbegin(), dBugle);
		}

		retPolyline.AddVertex(preCurve.ptE, 0.);

		return;
	}

	m_orgPolyline.GetCurve(ptPreS, ptPreE, dPreBulge, nPreIndex);
	m_orgPolyline.GetCurve(ptS, ptE, dBulge, nIndex);

	bool bIsPreValid = false, bIsCurValid = false;
	SGeoCurveInfo preCurve = OffsetCurve(ptPreS, ptPreE, dPreBulge, m_vecOffset[nPreIndex], bIsPreValid);
	SGeoCurveInfo curve = OffsetCurve(ptS, ptE, dBulge, m_vecOffset[nIndex], bIsCurValid);

	std::vector<gp_Pnt2d> vecIntersectPt;
	BimGeoAlgorithmTool::CalcIntersect(vecIntersectPt, preCurve.ptS, preCurve.ptE, preCurve.dBulge,
		curve.ptS, curve.ptE, curve.dBulge, m_dTol);

	if (vecIntersectPt.size() < 1)
	{// 求延长线
		BimGeoAlgorithmTool::CalcIntersectWithExtend(vecIntersectPt, preCurve.ptS, preCurve.ptE, preCurve.dBulge,
			curve.ptS, curve.ptE, curve.dBulge, m_dTol);
	}

	switch (vecIntersectPt.size())
	{
	case 0:
	{// 补边
		bool bIsAnticlockwise = false;
		if (BimGeoAlgorithmTool::IsArc(preCurve.dBulge)
			&& 0 < retPolyline.GetVertexes().size())
		{// 圆弧
			gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(preCurve.ptS, preCurve.ptE, preCurve.dBulge);
			GCE2d_MakeCircle arc(*retPolyline.GetVertexes().rbegin(), ptMid, preCurve.ptE);
			
			double dBugle = 0.;
			if (!arc.Value().IsNull())
			{
				bIsAnticlockwise = arc.Value()->Circ2d().IsDirect();
				dBugle = BimGeoAlgorithmTool::CalcArcBulge(*retPolyline.GetVertexes().rbegin(),
					preCurve.ptE, arc.Value()->Location(), arc.Value()->Radius(), bIsAnticlockwise, m_dTol);
			}

			retPolyline.SetVertex(retPolyline.GetVertexes().size() - 1, *retPolyline.GetVertexes().rbegin(), dBugle);
		}

		{
			// 补圆弧边
			gp_Pnt2d ptCenter((preCurve.ptE.Coord() + curve.ptS.Coord()) * 0.5);
			double dBugle = BimGeoAlgorithmTool::CalcArcBulge(preCurve.ptE,
				curve.ptS, ptS, abs(m_vecOffset[nIndex]), m_vecOffset[nIndex] < 0, m_dTol);

			retPolyline.AddVertex(preCurve.ptE, dBugle);
		}

		if (BimGeoAlgorithmTool::IsArc(curve.dBulge))
		{// 圆弧
			double dBulge = curve.dBulge;
			if (nIndex == m_orgPolyline.GetVertexes().size() - 1
				&& 0 < retPolyline.GetVertexes().size())
			{// 闭合区域最后一个顶点
				gp_Pnt2d ptEnd = retPolyline.GetVertexes()[0];
				dBulge = BimGeoAlgorithmTool::CalcArcBulge(curve.ptS, ptEnd, curve.ptS,
					curve.ptE, curve.dBulge, m_dTol);
			}

			retPolyline.AddVertex(curve.ptS, dBulge);
		}
		else
		{// 直线段
			retPolyline.AddVertex(curve.ptS, 0.);
		}
	}
	break;
	default:
	{// 多个交点时取最近的
		std::map<double, gp_Pnt2d> mapIntersectPt;
		for (int i = 0; i < vecIntersectPt.size(); ++i)
		{
			mapIntersectPt[curve.ptS.Distance(vecIntersectPt[i])] = vecIntersectPt[i];
		}

		vecIntersectPt.clear();
		vecIntersectPt.emplace_back(mapIntersectPt.begin()->second);
	}
	case 1:
	{
		if (BimGeoAlgorithmTool::IsArc(preCurve.dBulge)
			&& 0 < retPolyline.GetVertexes().size())
		{// 圆弧
			gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(preCurve.ptS, preCurve.ptE, preCurve.dBulge);
			GCE2d_MakeCircle arc(*retPolyline.GetVertexes().rbegin(), ptMid, vecIntersectPt[0]);

			double dBulge = BimGeoAlgorithmTool::CalcArcBulge(*retPolyline.GetVertexes().rbegin(),
				vecIntersectPt[0], arc.Value()->Location(), arc.Value()->Radius(), arc.Value()->Circ2d().IsDirect(), m_dTol);

			retPolyline.SetVertex(retPolyline.GetVertexes().size() - 1, *retPolyline.GetVertexes().rbegin(), dBulge);
		}

		double dBulge = 0.;
		if (BimGeoAlgorithmTool::IsArc(curve.dBulge))
		{// 圆弧
			gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(curve.ptS, curve.ptE, curve.dBulge);
			GCE2d_MakeCircle arcCure(vecIntersectPt[0], ptMid, curve.ptE);
			if (!arcCure.Value().IsNull())
			{
				gp_Pnt2d ptEnd = curve.ptE;
				if (nIndex == m_orgPolyline.GetVertexes().size() - 1
					&& 0 < retPolyline.GetVertexes().size())
				{// 闭合区域最后一个顶点
					ptEnd = retPolyline.GetVertexes()[0];
				}

				dBulge = BimGeoAlgorithmTool::CalcArcBulge(vecIntersectPt[0], ptEnd,
					arcCure.Value()->Location(), arcCure.Value()->Radius(), arcCure.Value()->Circ2d().IsDirect(), m_dTol);
			}
		}

		retPolyline.AddVertex(vecIntersectPt[0], dBulge);
	}
	break;
	}

	if (!bIsCurValid)
	{
		inValidSide.insert(retPolyline.GetVertexes().size() - 1);
	}
}

SGeoCurveInfo CBimGeoPolylineOffset::OffsetCurve(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, 
	const double& dBulge, const double& dOffset, bool& bIsValid) const
{
	bIsValid = true;

	SGeoCurveInfo curve;
	if (BimGeoAlgorithmTool::equal(dOffset, 0.))
	{
		curve.ptS = ptS;
		curve.ptE = ptE;
		curve.dBulge = dBulge;

		return curve;
	}

	curve.dBulge = dBulge;
	if (BimGeoAlgorithmTool::IsArc(dBulge))
	{// 弧线段
		gp_Pnt2d ptMid = BimGeoAlgorithmTool::GetCurveMidPoint(ptS, ptE, dBulge);
		GCE2d_MakeCircle arc(ptS, ptMid, ptE);

		gp_Pnt2d ptCenter = arc.Value()->Location();
		double dRadius = arc.Value()->Radius();

		gp_Vec2d vecStart(ptS, ptCenter), vecEnd(ptE, ptCenter);
		vecStart.Normalize();
		vecEnd.Normalize();

		if (!arc.Value()->Circ2d().IsDirect())
		{// 顺时针
			vecStart *= -1.;
			vecEnd *= -1.;
		}

		curve.ptS = ptS.Coord() + vecStart.XY() * dOffset;
		curve.ptE = ptE.Coord() + vecEnd.XY() * dOffset;

		if (!arc.Value()->Circ2d().IsDirect())
		{// 顺时针 内缩
			if (BimGeoAlgorithmTool::lessThan(dOffset, -dRadius))
			{
				bIsValid = false;
			}
		}
		else
		{// 逆时针 内缩
			if (BimGeoAlgorithmTool::lessThan(dRadius, dOffset))
			{
				bIsValid = false;
			}
		}
	}
	else
	{// 线段
		gp_Vec2d vec(ptS, ptE);
		if (0 < vec.Magnitude())
		{
			vec.Normalize();

			gp_Vec2d vecPerp(-vec.Y(), vec.X());
			curve.ptS = ptS.Coord() + vecPerp.XY() * dOffset;
			curve.ptE = ptE.Coord() + vecPerp.XY() * dOffset;
		}
	}

	return curve;
}
