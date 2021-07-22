#pragma once
#include "BimData/IBimDbEntity.h"
#include "BimClassifyEntityDef.h"
#include "BimGeoAlgorithm/BimGeoAlgorithmDef.h"
#include "Bnd_Box.hxx"
#include "BimData/BimDataDef.h"
#include "Bnd_Box2d.hxx"
#include <list>

namespace BimClassifyCommonTool
{
	/// @brief gp_Pnt2d转成gp_Pnt
	gp_Pnt TransPt(const gp_Pnt2d& pt);
	/// @brief SBimPoint转成gp_Pnt
	gp_Pnt TransPt(const SBimPoint& pt);
	/// @brief SBimPoint转成gp_Pnt2d
	gp_Pnt2d TransPt2d(const SBimPoint& pt);
	/// @brief gp_Pnt转成SBimPoint
	SBimPoint TransBimPt(const gp_Pnt& pt);
	/// @brief gp_Pnt转成SBimPoint
	SBimPoint TransBimPt(const gp_Pnt2d& pt);
	/// @brief SBimBox转成Bnd_Box
	Bnd_Box TransBndBox(const SBimBox& box);
	/// @brief SBimBox转成Bnd_Box2d
	Bnd_Box2d TransBndBox2d(const SBimBox& box);
	/// @brief Bnd_Box转成SBimBox
	SBimBox TransBimBox(const Bnd_Box& box);
	/// @brief Bnd_Box2d转成SBimBox
	SBimBox TransBimBox(const Bnd_Box2d& box);
	/// @brief 判断Box是否相交
	bool IsBoxIntersect(const SBimBox& box1, const SBimBox& box2, const double& dTol = 1e-3);
	/// @brief 获取box的中点
	void GetBoxCenter(const SBimBox& box, SBimPoint& ptCenter);
	/// @brief 点是否在box中
	bool IsPtInBox(const SBimPoint& pt, const SBimBox& box);
	/// @brief SGeoCurveInfo转成SBimCurve
	bool TransBimCurve(const SGeoCurveInfo& geoCurve, SBimCurve& curve);	
	/// @brief 转化ArchCurve数据
	bool TransArchCurveToCurve(const SArchCurveData& archCurve, SBimCurve& curve);
	/// @brief 转化ArchCurve数据
	bool TransArchCurveToGeoCurve(const SArchCurveData& curveData,
		SGeoCurveInfo& curveInfo);
	bool TransArchCurveToGeoCurve(const SArchCurveData& curveData,
		SGeoCurveInfo3D& curveInfo);
	/// @brief 获取文本对象数据
	bool GetTextEntData(const IBimDbEntity* pEnt, SArchTextData& data);
	/// @brief 获取线性对象数据
	bool GetCurveEntData(const IBimDbEntity* pEnt,
		std::list<SArchCurveData>& sideData);
};

