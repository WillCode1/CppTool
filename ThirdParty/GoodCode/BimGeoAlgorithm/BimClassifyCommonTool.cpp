#include "BimClassifyCommonTool.h"
#include "BimData/BimDbLeader.h"
#include "BimData/BimDbMText.h"
#include "BimData/BimDbText.h"
#include "BimGeoAlgorithm/BimGeoAlgorithmTool.h"
#include "BimData/BimDbLine.h"
#include "BimData/BimDbArc.h"
#include "BimData/BimDbPolyline.h"

gp_Pnt BimClassifyCommonTool::TransPt(const gp_Pnt2d& pt)
{
	return gp_Pnt(pt.X(), pt.Y(), 0);
}

gp_Pnt BimClassifyCommonTool::TransPt(const SBimPoint& pt)
{
	return gp_Pnt(pt.x, pt.y, pt.z);
}

gp_Pnt2d BimClassifyCommonTool::TransPt2d(const SBimPoint& pt)
{
	return gp_Pnt2d(pt.x, pt.y);
}

SBimPoint BimClassifyCommonTool::TransBimPt(const gp_Pnt& pt)
{
	return SBimPoint(pt.X(), pt.Y(), pt.Z());
}

SBimPoint BimClassifyCommonTool::TransBimPt(const gp_Pnt2d& pt)
{
	return SBimPoint(pt.X(), pt.Y(), 0);
}

Bnd_Box BimClassifyCommonTool::TransBndBox(const SBimBox& box)
{
	Bnd_Box bndBox;
	bndBox.Add(TransPt(box.ptMin));
	bndBox.Add(TransPt(box.ptMax));
	return bndBox;
}

Bnd_Box2d BimClassifyCommonTool::TransBndBox2d(const SBimBox& box)
{
	Bnd_Box2d bndBox;
	bndBox.Add(TransPt2d(box.ptMin));
	bndBox.Add(TransPt2d(box.ptMax));
	return bndBox;
}

SBimBox BimClassifyCommonTool::TransBimBox(const Bnd_Box& box)
{
	return SBimBox(TransBimPt(box.CornerMin()), TransBimPt(box.CornerMax()));
}

SBimBox BimClassifyCommonTool::TransBimBox(const Bnd_Box2d& box)
{
	double aXmin, aYmin, aXmax, aYmax;
	box.Get(aXmin, aYmin, aXmax, aYmax);
	SBimBox bndBox;
	bndBox.ptMin = SBimPoint(aXmin, aYmin, 0.);
	bndBox.ptMax = SBimPoint(aXmax, aYmax, 0.);
	return bndBox;
}

bool BimClassifyCommonTool::IsBoxIntersect(const SBimBox& box1, const SBimBox& box2, const double& dTol)
{
	Bnd_Box bndBox1 = BimClassifyCommonTool::TransBndBox(box1);
	Bnd_Box bndBox2 = BimClassifyCommonTool::TransBndBox(box2);
	if (bndBox1.IsOut(bndBox2))
	{
		return false;
	}
	return true;
}

void BimClassifyCommonTool::GetBoxCenter(const SBimBox& box, SBimPoint& ptCenter)
{
	ptCenter.x = (box.ptMin.x + box.ptMax.x) / 2.0;
	ptCenter.y = (box.ptMin.y + box.ptMax.y) / 2.0;
	ptCenter.z = (box.ptMin.z + box.ptMax.z) / 2.0;
}

bool BimClassifyCommonTool::IsPtInBox(const SBimPoint& pt, const SBimBox& box)
{
	gp_Pnt pntPt = BimClassifyCommonTool::TransPt(pt);
	Bnd_Box bndBox = BimClassifyCommonTool::TransBndBox(box);
	if (bndBox.IsVoid())
	{
		return false;
	}
	return !bndBox.IsOut(pntPt);
}

bool BimClassifyCommonTool::TransBimCurve(const SGeoCurveInfo& geoCurve, SBimCurve& curve)
{
	curve.ptStart = BimClassifyCommonTool::TransBimPt(geoCurve.ptS);
	curve.ptEnd = BimClassifyCommonTool::TransBimPt(geoCurve.ptE);
	curve.dBulge = geoCurve.dBulge;

	return true;
}

bool BimClassifyCommonTool::GetTextEntData(const IBimDbEntity* pEnt, SArchTextData& data)
{
	bool bGet = false;
	if (!pEnt)
	{
		return bGet;
	}

	IBimDbEntity* pUnConstEnt = const_cast<IBimDbEntity*>(pEnt);
	if (!pUnConstEnt)
	{
		return bGet;
	}

	EBimEntityType type;
	pEnt->GetType(type);
	switch (type)
	{
	case eText:
	{
		CBimDbText* pText = dynamic_cast<CBimDbText*>(pUnConstEnt);
		if (pText)
		{
			pText->GetPosition(data.ptInsert);
			pText->GetRotation(data.dRotate);
			pText->GetTextHeight(data.dHeight);
			pText->GetContent(data.strContent);
			pText->GetGeoExtents(data.extents);

			bGet = true;
		}
	}
	break;
	case eMText:
	{
		CBimDbMText* pMText = dynamic_cast<CBimDbMText*>(pUnConstEnt);
		if (pMText)
		{
			pMText->GetPosition(data.ptInsert);
			pMText->GetRotation(data.dRotate);
			pMText->GetTextHeight(data.dHeight);
			pMText->GetContent(data.strContent);
			pMText->GetGeoExtents(data.extents);
			bGet = true;
		}
	}
	break;
	case eLeader:
	{
		CBimDbLeader* pLeader = dynamic_cast<CBimDbLeader*>(pUnConstEnt);
		if (pLeader)
		{
			data.ptInsert = pLeader->VertexAt(0);

			std::wstring strContent;
			int nTextCount = pLeader->GetContentCount();
			for (int i = 0; i < nTextCount; ++i)
			{
				std::wstring tempContent;
				pLeader->GetContent(i, tempContent);
				strContent += tempContent;
			}
			data.strContent = strContent;

			pLeader->GetGeoExtents(data.extents);

			bGet = true;
		}
	}
	break;
	default:
		break;
	}

	return bGet;
}

bool BimClassifyCommonTool::GetCurveEntData(const IBimDbEntity* pEnt, 
	std::list<SArchCurveData>& curveList)
{
	if (!pEnt)
	{
		return false;
	}

	IBimDbEntity* pUnConstEnt = const_cast<IBimDbEntity*>(pEnt);
	if (!pUnConstEnt)
	{
		return false;
	}

	EBimEntityType type;
	pEnt->GetType(type);
	switch (type)
	{
	case EBimEntityType::eLine:
	{
		CBimDbLine* pLine = dynamic_cast<CBimDbLine*>(pUnConstEnt);
		if (!pLine)
		{
			return false;
		}

		SArchCurveData curveData;
		pLine->GetStartPt(curveData.ptStart);
		pLine->GetEndPt(curveData.ptEnd);
		curveData.dBulge = 0.;
		curveList.emplace_back(curveData);
	}
		break;
	case EBimEntityType::eArc:
	{
		CBimDbArc* pArc = dynamic_cast<CBimDbArc*>(pUnConstEnt);
		if (!pArc)
		{
			return false;
		}

		SArchCurveData curveData;
		pArc->GetStartPt(curveData.ptStart);
		pArc->GetEndPt(curveData.ptEnd);
		pArc->GetBulge(curveData.dBulge);
		curveList.emplace_back(curveData);
	}
	case EBimEntityType::ePolyline:
	{
		CBimDbPolyline* pPolyline = dynamic_cast<CBimDbPolyline*>(pUnConstEnt);
		if (!pPolyline)
		{
			return false;
		}

		int nCurveCount = pPolyline->GetCurveNumber();
		for (int nIndex = 0; nIndex < nCurveCount; ++nIndex)
		{
			SArchCurveData curveData;
			if (pPolyline->GetCurve(curveData.ptStart, curveData.ptEnd, curveData.dBulge, nIndex))
			{
				curveList.emplace_back(curveData);
			}
		}
	}

	default:
		break;
	}
	
	return true;
}

bool BimClassifyCommonTool::TransArchCurveToCurve(const SArchCurveData& archCurve, SBimCurve& curve)
{
	curve.ptStart = archCurve.ptStart;
	curve.ptEnd = archCurve.ptEnd;
	curve.dBulge = archCurve.dBulge;

	return true;
}

bool BimClassifyCommonTool::TransArchCurveToGeoCurve(const SArchCurveData& curveData, 
	SGeoCurveInfo& curveInfo)
{
	curveInfo.ptS = BimClassifyCommonTool::TransPt2d(curveData.ptStart);
	curveInfo.ptE = BimClassifyCommonTool::TransPt2d(curveData.ptEnd);
	curveInfo.dBulge = curveData.dBulge;
	if (BimGeoAlgorithmTool::equal(curveInfo.dBulge, 0))
	{
		curveInfo.dBulge = 0;
	}
	return true;
}

bool BimClassifyCommonTool::TransArchCurveToGeoCurve(const SArchCurveData& curveData, 
	SGeoCurveInfo3D& curveInfo)
{
	curveInfo.ptS = BimClassifyCommonTool::TransPt(curveData.ptStart);
	curveInfo.ptE = BimClassifyCommonTool::TransPt(curveData.ptEnd);
	curveInfo.dBulge = curveData.dBulge;
	if (BimGeoAlgorithmTool::equal(curveInfo.dBulge, 0))
	{
		curveInfo.dBulge = 0;
	}

	return true;
}
