#pragma once

#include "BimGeoAlgorithmDef.h"
#include "gp_Pnt.hxx"
#include "Bnd_Box.hxx"
#include "Bnd_Box2d.hxx"
#include <vector>
#include <list>
#include <set>

class CBimGeoPolyline;
struct SBimGeoPolygonWithHole;
class CBimGeoOBB2d;

namespace BimGeoPolylineTool
{
	/// @brief 等距偏移多边形
	/// @param [in ] polylines 偏移后的多段线
	/// @param [in ] orgPolyline 原始多段线(逆时针)
	/// @param [in ] dOffset 偏移距离 > 0 左偏 < 0 右偏
	/// @return true 或 false
	EXPORTGEOALGOAPI bool OffsetPolyline(std::list<CBimGeoPolyline>& polylines, const CBimGeoPolyline& orgPolyline,
		const double& dOffset, const double& dTol = 0.0001);

	/// @brief 非等距偏移多边形
	/// @param [in ] polylines 偏移后的多段线
	/// @param [in ] orgPolyline 原始多段线(逆时针)
	/// @param [in ] dOffset 偏移距离 > 0 左偏 < 0 右偏
	/// @return true 或 false
	EXPORTGEOALGOAPI bool OffsetPolyline(std::list<CBimGeoPolyline>& polylines, const CBimGeoPolyline& orgPolyline,
		const std::vector<double>& vecOffset, const double& dTol = 0.0001);

	/// @brief 计算点与多边形的关系
	EXPORTGEOALGOAPI EPointWidthPolygonRel CalcPointWithPolygonRelation(const gp_Pnt2d& pt, const CBimGeoPolyline& polyline, const double& dTol = 0.0001);
	EXPORTGEOALGOAPI EPointWidthPolygonRel CalcPointWithPolygonRelation(const gp_Pnt& pt, const CBimGeoPolyline& polyline, const double& dTol = 0.0001);

	/// @brief 计算两个多边形的关系
	EXPORTGEOALGOAPI EPolygonRel CalcTwoPolygonRelation(const CBimGeoPolyline& polylineA, const CBimGeoPolyline& polylineB, const double& dTol = 0.0001);

	/// @brief 计算点多段线的最近距离
	/// @param [out] indexSet 最近点所在边的下标 顶点时有两个
	/// @param [in ] pt 点
	/// @param [in ] polyline 多段线
	/// @param [in ] dTol 精度
	/// @return 最近点
	EXPORTGEOALGOAPI gp_Pnt2d GetClosePt(std::set<int>& indexSet, const gp_Pnt2d& pt, const CBimGeoPolyline& polyline, const double& dTol = 0.0001);

	/// @brief 计算点多段线的最近距离
	/// @param [out] indexSet 最近点所在边的下标 顶点时有两个
	/// @param [in ] pt 点
	/// @param [in ] polyline 多段线
	/// @param [in ] filterIndexSet 过滤的边
	/// @param [in ] dTol 精度
	/// @return 最近点
	EXPORTGEOALGOAPI gp_Pnt2d GetClosePtExt(std::set<int>& indexSet, const gp_Pnt2d& pt,
		const CBimGeoPolyline& polyline, const std::set<int>& filterIndexSet, const double& dTol = 0.0001);

	/// @brief 获取多边形的重心(弧线是放样后)
	/// @return 重心
	EXPORTGEOALGOAPI gp_Pnt2d GetCenterPt(const CBimGeoPolyline& polyline);

	/// @brief 计算三角形的面积
	EXPORTGEOALGOAPI double GetArea(const gp_Pnt2d& pt1, const gp_Pnt2d& pt2, const gp_Pnt2d& pt3);
	EXPORTGEOALGOAPI double GetArea(const gp_Pnt& pt1, const gp_Pnt& pt2, const gp_Pnt& pt3);
	
	/// @brief 计算三角形的面积
	/// @return 有正负
	EXPORTGEOALGOAPI double GetArea(const gp_Pnt& pt1, const gp_Pnt& pt2, const gp_Pnt& pt3,
		const gp_Vec& normal);

	/// @brief 获取平行边
	EXPORTGEOALGOAPI std::list<std::set<int>> GetParallelSide(const CBimGeoPolyline& polyline);

	/// @brief 是否相交
	EXPORTGEOALGOAPI bool IsHasIntersect(const CBimGeoPolyline& polyline, const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dBulge);

	/// @brief 获取交点
	EXPORTGEOALGOAPI void GetIntersect(std::vector<gp_Pnt2d>& intersectPts, const CBimGeoPolyline& polyline,
		const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dBulge, const double& dTol = 0.0001);

	/// @briref 搜索最大封闭区域
	/// @param [in ] curves 线段
	/// @param [in ] dTol 精度
	/// @param [in ] IsVectorCurve 是否是有向线段
	/// @param [in ] dAngleTol 角度精度
	/// @return 封闭区域
	EXPORTGEOALGOAPI std::list<CBimGeoPolyline> SearchMaxPolygonWithBreakCurve(const std::list<SGeoCurveInfo>& curves, const double& dTol = 0.0001, 
		const bool IsVectorCurve = false, const double& dAngleTol = 0.01);

	EXPORTGEOALGOAPI std::list<CBimGeoPolyline> SearchMaxPolygonWithNoBreakCurve(const std::list<SGeoCurveInfo>& curves, const double& dTol = 0.0001, 
		const bool IsVectorCurve = false, const double& dAngleTol = 0.01);

	/// @brief 搜索最小封闭区域
	EXPORTGEOALGOAPI std::list<CBimGeoPolyline> SearchMinPolygonWithBreakCurve(const std::list<SGeoCurveInfo>& curves, const double& dTol = 0.0001, 
		const bool IsVectorCurve = false, const double& dAngleTol = 0.01);

	EXPORTGEOALGOAPI std::list<CBimGeoPolyline> SearchMinPolygonWithNoBreakCurve(const std::list<SGeoCurveInfo>& curves, const double& dTol = 0.0001, 
		const bool IsVectorCurve = false, const double& dAngleTol = 0.01);

	/// @brief 合并多边形的共线边
	EXPORTGEOALGOAPI void MergePolylineSide(CBimGeoPolyline& polyline, const double& dTol = 0.0001);

	/// @brief 去除点
	EXPORTGEOALGOAPI void RemovePoint(CBimGeoPolyline& polyline, const double& dTol = 0.0001);

	/// @brief 多边形是否自交
	EXPORTGEOALGOAPI bool IsSelfIntersect(const CBimGeoPolyline& polyline, const double& dTol = 0.0001);

	/// @brief 根据起点终点截取多段线某一段
	EXPORTGEOALGOAPI void InterceptPolyline(std::list<CBimGeoPolyline>& retPolies, const CBimGeoPolyline& polyline,
		const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dTol = 0.0001);

	/// @brief 多边形的布尔运算
	EXPORTGEOALGOAPI void TwoPolygonBooleanOperation(std::list<SBimGeoPolygonWithHole>& retPolygons, const CBimGeoPolyline* pPolylineA,
		const CBimGeoPolyline* pPolylineB, const EBooleanOperation eCalcType, const double& dTol = 0.0001);

	/// @brief 计算多边形面积
	EXPORTGEOALGOAPI double GetPolygonArea(const CBimGeoPolyline& polygon);

#ifdef GEO_CXX11_IS_SUPPORTED // C++ 11 标准
	/// @brief 剖分三角形(不支持弧形)
	EXPORTGEOALGOAPI void Triangle(std::list<SGeoTriangle>& triangles, const CBimGeoPolyline& polygon, const std::list<CBimGeoPolyline>& holes);
#endif

	/// @brief 获取带洞多边形内部点(洞与多边形没有重叠边)
	EXPORTGEOALGOAPI gp_Pnt2d GetInnerPoint(const CBimGeoPolyline& polygon, const std::vector<const CBimGeoPolyline*>& holes);

	/// @brief 获取带洞多边形内部点(洞与多边形可以有重叠边)
	/// @param [in ] polygon 多边形
	/// @param [in ] holes 洞区域
	/// @param [in ] bIsXAxisPrior 是否X轴优先
	EXPORTGEOALGOAPI gp_Pnt2d GetInnerPointExt(const CBimGeoPolyline& polygon, 
		const std::vector<const CBimGeoPolyline*>& holes, const bool bIsXAxisPrior = true);

	/// @brief 计算线条凸包
	/// @param [out] polygon 凸包多多边形
	/// @param [in ] inputCurves 线
	/// @param [in ] dTol 精度
	EXPORTGEOALGOAPI bool CalculateCurvesConvexHull(CBimGeoPolyline& polygon,
		const std::list<SGeoCurveInfo>& inputCurves, const double& dTol = 0.0001);

	/// @brief 计算闭合多边形的最小矩形包围
	/// @param [out] minAreaObb 最小矩形外包
	/// @param [in ] polygon 多边形
	/// @param [in ] dTol 精度
	/// return true 成功; false 失败
	EXPORTGEOALGOAPI bool CalculatePolygonMinAreaObb(CBimGeoOBB2d& minAreaObb, const CBimGeoPolyline& polygon, const double& dTol = 0.0001);
};

