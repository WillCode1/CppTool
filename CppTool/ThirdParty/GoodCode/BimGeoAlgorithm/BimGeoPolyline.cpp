#include "BimGeoPolyline.h"
#include "BimGeoAlgorithmTool.h"
#include "BimGeoPolylineTool.h"
#include <algorithm>

CBimGeoPolyline::CBimGeoPolyline()
	: m_bIsClose(false)
{
}

CBimGeoPolyline::CBimGeoPolyline(const std::vector<gp_Pnt2d>& vecVertex, const std::vector<double>& vecBulge,
	const bool bIsClose)
	: m_vecVertex(vecVertex)
	, m_vecBulge(vecBulge)
	, m_bIsClose(bIsClose)
{
}

CBimGeoPolyline::~CBimGeoPolyline()
{
}

gp_Pnt2d CBimGeoPolyline::GetStartPoint() const
{
	if (0 < m_vecVertex.size())
	{
		return m_vecVertex[0];
	}

	return gp_Pnt2d(0., 0.);
}

gp_Pnt2d CBimGeoPolyline::GetEndPoint() const
{
	if (0 < m_vecVertex.size())
	{
		return m_vecVertex[m_vecVertex.size() - 1];
	}

	return gp_Pnt2d(0., 0.);
}

const std::vector<gp_Pnt2d>& CBimGeoPolyline::GetVertexes() const
{
	return m_vecVertex;
}

const std::vector<double>& CBimGeoPolyline::GetBulges() const
{
	return m_vecBulge;
}

void CBimGeoPolyline::SetVertexes(const std::vector<gp_Pnt2d>& vecVertex)
{
	m_vecVertex = vecVertex;
}

void CBimGeoPolyline::SetVertex(const int nIndex, const gp_Pnt2d& pt)
{
	if (0 <= nIndex && nIndex < m_vecBulge.size())
	{
		m_vecVertex[nIndex] = pt;
	}
}

void CBimGeoPolyline::SetBulges(const std::vector<double>& vecBulge)
{
	m_vecBulge = vecBulge;
}

void CBimGeoPolyline::SetBugle(const int nIndex, const double & dBulge)
{
	if (0 <= nIndex && nIndex < m_vecBulge.size())
	{
		m_vecBulge[nIndex] = dBulge;
	}
}

void CBimGeoPolyline::AddVertex(const gp_Pnt2d& pt, const double& dBulge)
{
	m_vecVertex.emplace_back(pt);
	m_vecBulge.emplace_back(dBulge);
}

void CBimGeoPolyline::SetVertex(const int nIndex, const gp_Pnt2d& pt, const double& dBulge)
{
	if (0 <= nIndex && nIndex < m_vecVertex.size())
	{
		m_vecVertex[nIndex] = pt;
		m_vecBulge[nIndex] = dBulge;
	}
}

bool CBimGeoPolyline::IsClose() const
{
	return m_bIsClose;
}

void CBimGeoPolyline::setClose(const bool bTrue)
{
	m_bIsClose = bTrue;
}

gp_Vec CBimGeoPolyline::Normal() const
{
	gp_Vec vecNormal;
	switch (m_vecVertex.size())
	{
	case 0:
	case 1:
		break;
	case 2:
	{
		if (!IsClose())
		{
			gp_Pnt ptS(m_vecVertex[0].X(), m_vecVertex[0].Y(), 0.), ptE(m_vecVertex[1].X(), m_vecVertex[1].Y(), 0.);
			vecNormal = BimGeoAlgorithmTool::GetNormal(ptS, ptE, m_vecBulge[0]);
			if (BimGeoAlgorithmTool::equal(vecNormal.Magnitude(), 0.))
			{
				return gp_Vec(0., 0., 0.);
			}

			vecNormal.Normalize();
		}
		else
		{
			gp_Pnt2d ptMid1((m_vecVertex[0].Coord() + m_vecVertex[1].Coord()) * 0.5);
			if (BimGeoAlgorithmTool::IsArc(m_vecBulge[0]))
			{
				ptMid1 = BimGeoAlgorithmTool::GetCurveMidPoint(m_vecVertex[0], m_vecVertex[1], m_vecBulge[0]);
			}

			gp_Pnt2d ptMid2((m_vecVertex[0].Coord() + m_vecVertex[1].Coord()) * 0.5);
			if (BimGeoAlgorithmTool::IsArc(m_vecBulge[1]))
			{
				ptMid2 = BimGeoAlgorithmTool::GetCurveMidPoint(m_vecVertex[1], m_vecVertex[0], m_vecBulge[1]);
			}

			gp_Vec2d vec1(m_vecVertex[0], ptMid1), vec2(ptMid1, ptMid2);
			if (BimGeoAlgorithmTool::equal(vec1.Magnitude(), 0.)
				|| BimGeoAlgorithmTool::equal(vec2.Magnitude(), 0.))
			{
				return gp_Vec(0., 0., 0.);
			}

			vec1.Normalize();
			vec2.Normalize();

			vecNormal = gp_Vec(0., 0., vec1.Crossed(vec2));
		}
	}
	break;
	default:
	{
		if (IsClose())
		{
			double dArea = BimGeoPolylineTool::GetPolygonArea(*this);
			if (BimGeoAlgorithmTool::lessThan(dArea, 0.))
			{
				return gp_Vec(0., 0., -1.);
			}

			return gp_Vec(0., 0., 1.);
		}
		else
		{
			gp_Pnt ptS(m_vecVertex[0].X(), m_vecVertex[0].Y(), 0.), ptE(m_vecVertex[1].X(), m_vecVertex[1].Y(), 0.);
			vecNormal = BimGeoAlgorithmTool::GetNormal(ptS, ptE, m_vecBulge[0]);
		}
	}
	break;
	}

	return vecNormal;
}

int CBimGeoPolyline::GetCurveNumber() const
{
	int len = (int)m_vecVertex.size();
	if (!m_bIsClose)
	{
		len -= 1;
	}

	return len;
}

bool CBimGeoPolyline::GetCurve(gp_Pnt2d& ptS, gp_Pnt2d& ptE, double& dBulge, const int nIndex) const
{
	if (nIndex < 0
		|| nIndex >= GetCurveNumber())
	{
		return false;
	}

	int i = nIndex;
	int j = i + 1;
	if (j == m_vecVertex.size())
	{
		j = 0;
	}

	ptS = m_vecVertex[i];
	ptE = m_vecVertex[j];
	dBulge = m_vecBulge[i];

	return true;
}

void CBimGeoPolyline::RemoveVertex(const int nIndex)
{
	if (0 <= nIndex
		&& nIndex < m_vecVertex.size())
	{
		m_vecVertex.erase(m_vecVertex.begin() + nIndex);
		m_vecBulge.erase(m_vecBulge.begin() + nIndex);
	}
}

Bnd_Box2d CBimGeoPolyline::GetBndBox() const
{
	Bnd_Box2d box;
	for (int i = 1; i < m_vecVertex.size(); ++i)
	{
		box.Add(m_vecVertex[i]);
		box.Add(m_vecVertex[i - 1]);

		if (BimGeoAlgorithmTool::IsArc(m_vecBulge[i - 1]))
		{// »¡Ïß
			Bnd_Box2d arcBox = BimGeoAlgorithmTool::GetCurveBndBox(m_vecVertex[i - 1],
				m_vecVertex[i], m_vecBulge[i - 1]);
			
			box.Add(arcBox);
		}
	}

	if (m_bIsClose)
	{
		box.Add(m_vecVertex[m_vecVertex.size() - 1]);
		box.Add(m_vecVertex[0]);

		if (BimGeoAlgorithmTool::IsArc(m_vecBulge[m_vecVertex.size() - 1]))
		{// »¡Ïß
			Bnd_Box2d arcBox = BimGeoAlgorithmTool::GetCurveBndBox(m_vecVertex[m_vecVertex.size() - 1],
				m_vecVertex[0], m_vecBulge[m_vecVertex.size() - 1]);

			box.Add(arcBox);
		}
	}

	return box;
}

void CBimGeoPolyline::Reverse()
{
	std::reverse(m_vecVertex.begin(), m_vecVertex.end());
	std::reverse(m_vecBulge.begin(), m_vecBulge.end());

	if (m_bIsClose)
	{
		m_vecVertex.insert(m_vecVertex.begin(), *m_vecVertex.rbegin());
		m_vecVertex.pop_back();
	}

	for (int i = 0; i < m_vecBulge.size(); ++i)
	{
		if (0. != m_vecBulge[i])
		{
			m_vecBulge[i] = -m_vecBulge[i];
		}
	}
}

void CBimGeoPolyline::Transform(const gp_Trsf2d& transMat)
{
	for (int i = 0; i < m_vecVertex.size(); ++i)
	{
		m_vecVertex[i].Transform(transMat);
	}
}

void CBimGeoPolyline::Sample(const int nNum)
{
	if (m_vecVertex.size() < 1)
	{
		return;
	}

	std::vector<gp_Pnt2d> vecNewVertex;
	vecNewVertex.emplace_back(m_vecVertex[0]);

	for (int i = 0; i < GetCurveNumber(); ++i)
	{
		gp_Pnt2d ptS, ptE; 
		double dBulge = 0.;
		GetCurve(ptS, ptE, dBulge, i);

		if (BimGeoAlgorithmTool::IsArc(dBulge))
		{
			std::vector<gp_Pnt2d> vecVertex;
			BimGeoAlgorithmTool::ArcSample(vecVertex, ptS, ptE, dBulge, nNum);

			for (int j = 1; j < vecVertex.size(); ++j)
			{
				vecNewVertex.emplace_back(vecVertex[j]);
			}
		}
		else
		{
			vecNewVertex.emplace_back(ptE);
		}
	}

	if (m_bIsClose)
	{
		vecNewVertex.pop_back();
	}

	m_vecVertex = vecNewVertex;
	m_vecBulge = std::vector<double>(m_vecVertex.size());
}

bool CBimGeoPolyline::operator == (const CBimGeoPolyline& rhs) const
{
	bool bRet = true;
	if (m_vecVertex.size() != rhs.m_vecVertex.size())
	{
		return false;
	}

	for (int i = 0; i < m_vecVertex.size(); ++i)
	{
		if (!BimGeoAlgorithmTool::equal(m_vecVertex[i].Distance(rhs.m_vecVertex[i]), 0.))
		{
			return false;
		}
	}

	bRet &= m_vecBulge == rhs.m_vecBulge;
	bRet &= m_bIsClose == rhs.m_bIsClose;

	return bRet;
}
