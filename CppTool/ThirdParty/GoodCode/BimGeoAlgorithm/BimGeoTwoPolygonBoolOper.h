#pragma once

#include "BimGeoPolyline.h"
#include <list>

/// @class CBimGeoTwoPolygonBoolOper
/// @brief 两个多边形布尔运算
/// @detail https://www.geeksforgeeks.org/weiler-atherton-polygon-clipping-algorithm/ weiler-atherton 算法

class CBimGeoTwoPolygonBoolOper
{
public:
	CBimGeoTwoPolygonBoolOper();
	~CBimGeoTwoPolygonBoolOper();
public:
	/// @brief 执行A布尔运算B
	void DoIt(const CBimGeoPolyline* pPolygonA, const CBimGeoPolyline* pPolygonB,
		const EBooleanOperation eBooleanOper, const double& dTol);

	/// @brief 获取计算结果
	const std::list<SBimGeoPolygonWithHole>& GetPolygons() const;
private:
	/// @brief 清空数据
	void Clear();

	/// @brief 计算
	void CalcImp();

	/// @brief 相交的多边形的布尔运算
	void CalcWithIntersectPolygon();

	/// @brief 计算两个多边形的布尔数据
	void CalcTwoPolygonBooleanOperData(std::list<SGeoCurveInfo>& curves, 
		const CBimGeoPolyline* pPolygonA, const CBimGeoPolyline* pPolygonB,
		const EBooleanOperation eCalcType) const;

	/// @brief 多段线转换成curve
	void PolylineToCurves(std::list<SGeoCurveInfo>& curves, const std::list<CBimGeoPolyline>& polies,
		const std::list<gp_Pnt2d>& breakPoints) const;

	/// @brief 打断线段
	void BreakCurves(std::list<SGeoCurveInfo>& curves, const SGeoCurveInfo& curve, const std::list<gp_Pnt2d>& breakPoints) const;

	/// @brief 移除无效的多边形
	void RemoveOverlapPolygon(const CBimGeoPolyline* pRefPolygon);
private:
	std::list<SBimGeoPolygonWithHole>		m_retPolygon;		//	计算结果
	const CBimGeoPolyline*					m_pPolygonA;		//	多边形A
	const CBimGeoPolyline*					m_pPolygonB;		//	多边形B
	double									m_dTol;				//	计算精度
	EBooleanOperation						m_eBooleanOper;		//	布尔计算类型
};

