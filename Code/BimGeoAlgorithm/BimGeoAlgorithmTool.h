#pragma once

#include "BimGeoAlgorithmDef.h"
#include "gp_Pnt.hxx"
#include "Bnd_Box.hxx"
#include "Bnd_Box2d.hxx"
#include <vector>
#include <list>

namespace BimGeoAlgorithmTool
{
	/// @brief 是否等于
	EXPORTGEOALGOAPI bool equal(const double& a, const double& b, const double& dTol = 0.0001);

	/// @brief 是否小于
	EXPORTGEOALGOAPI bool lessThan(const double& a, const double& b, const double& dTol = 0.0001);

	/// @brief 是否小于等于
	EXPORTGEOALGOAPI bool lessEqualThan(const double& a, const double& b, const double& dTol = 0.0001);

	/// @brief 是否大于
	EXPORTGEOALGOAPI bool biggerThan(const double& a, const double& b, const double& dTol = 0.0001);

	/// @brief 是否大于等于
	EXPORTGEOALGOAPI bool biggerEqualThan(const double& a, const double& b, const double& dTol = 0.0001);

	/// @brief 四舍五入取整
	EXPORTGEOALGOAPI double round(double r);

	/// @brief 是否是圆弧
	EXPORTGEOALGOAPI bool IsArc(const double& dBulge, const double& dTol = 0.0001);

	/// @brief 获取线段的中心点
	EXPORTGEOALGOAPI gp_Pnt GetCurveMidPoint(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge);

	EXPORTGEOALGOAPI gp_Pnt2d GetCurveMidPoint(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge);

	/// @brief 获取圆弧某一段的中心点(必须与弧线段同向)
	EXPORTGEOALGOAPI gp_Pnt2d GetArcMidPoint(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd,
		const gp_Pnt2d& ptCenter, const double& dRadius, const bool IsAnticlockwise, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI gp_Pnt2d GetArcMidPoint(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd,
		const gp_Pnt2d& ptRefStart, const gp_Pnt2d& ptRefEnd, const double& dRefBulge, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI gp_Pnt GetArcMidPoint(const gp_Pnt& ptStart, const gp_Pnt& ptEnd,
		const gp_Pnt& ptRefStart, const gp_Pnt& ptRefEnd, const double& dRefBulge, const double& dTol = 0.0001);

	/// @brief 获取圆弧的信息
	EXPORTGEOALGOAPI SGeoArcInfo GetArcInfo(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge);

	EXPORTGEOALGOAPI SGeoArcInfo GetArcInfo(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge);

	/// @brief 判断点是否再线上
	EXPORTGEOALGOAPI bool IsOn(const gp_Pnt& pt, const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI bool IsOn(const gp_Pnt2d& pt, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	/// @brief 计算垂足
	/// @return 失败则没有垂足
	EXPORTGEOALGOAPI bool GetProjectPt(gp_Pnt& ptProject, const gp_Pnt& pt, const gp_Pnt& ptStart,
		const gp_Pnt& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI bool GetProjectPt(gp_Pnt2d& ptProject, const gp_Pnt2d& pt, const gp_Pnt2d& ptStart,
		const gp_Pnt2d& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	/// @brief 获取最近点
	/// @return 失败返回原点
	EXPORTGEOALGOAPI gp_Pnt GetClosePoint(const gp_Pnt& pt, const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI gp_Pnt2d GetClosePoint(const gp_Pnt2d& pt, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	/// @brief 获取线上的参数
	/// @return 失败返回-1 
	EXPORTGEOALGOAPI double GetParamOnCurve(const gp_Pnt& pt, const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI double GetParamOnCurve(const gp_Pnt2d& pt, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	/// @brief 通过参数获取线上的点
	/// @return 失败返回原点 
	EXPORTGEOALGOAPI gp_Pnt GetPtOnCurveByParam(const double& param, const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI gp_Pnt2d GetPtOnCurveByParam(const double& param, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge, const double& dTol = 0.0001);

	/// @brief 求交点(二维)
	EXPORTGEOALGOAPI void CalcIntersect(std::vector<gp_Pnt>& vecIntersectPt, const gp_Pnt& ptS, const gp_Pnt& ptE, const double& dBulge,
		const gp_Pnt& ptSRef, const gp_Pnt& ptERef, const double& dBulgeRef, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI void CalcIntersect(std::vector<gp_Pnt2d>& vecIntersectPt, const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dBulge,
		const gp_Pnt2d& ptSRef, const gp_Pnt2d& ptERef, const double& dBulgeRef, const double& dTol = 0.0001);

	/// @brief 求直线的交点
	EXPORTGEOALGOAPI void CalcLineIntersect(std::vector<gp_Pnt2d>& vecIntersectPt, const gp_Pnt2d& ptS, const gp_Dir2d& dir,
		const gp_Pnt2d& ptRefS, const gp_Dir2d& dirRef, const double& dTol = 0.0001);

	/// @brief 求包含延长线的交点
	EXPORTGEOALGOAPI void CalcIntersectWithExtend(std::vector<gp_Pnt2d>& vecIntersectPt, const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dBulge,
		const gp_Pnt2d& ptSRef, const gp_Pnt2d& ptERef, const double& dBulgeRef, const double& dTol = 0.0001);

	/// @brief 判断是否平行
	/// @param [in ] ptStart 线段1的起点
	/// @param [in ] ptEnd 线段1的终点
	/// @param [in ] dBulge 线段1的凸度
	/// @param [in ] ptRefStart 线段2的起点
	/// @param [in ] ptRefEnd 线段2的终点
	/// @param [in ] dRefBulge 线段2的凸度
	/// @param [in ] dTol 精度 约为1°
	EXPORTGEOALGOAPI bool IsParallel(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge,
		const gp_Pnt& ptRefStart, const gp_Pnt& ptRefEnd, const double& dRefBulge, const double& dTol = 0.017);

	EXPORTGEOALGOAPI bool IsParallel(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge,
		const gp_Pnt2d& ptRefStart, const gp_Pnt2d& ptRefEnd, const double& dRefBulge, const double& dTol = 0.017);

	/// @brief 是否在线的左边
	/// @param [in ] pt 点
	/// @param [in ] ptStart 线段起点
	/// @param [in ] ptEnd 线段终点
	/// @param [in ] dBulge 线段凸度
	/// @param [in ] dTol 线段起点
	/// @return 0 在线段左边 1 在线段右边 2 在延长线上 3 在线段上
	EXPORTGEOALGOAPI int IsLeftOfCurve(const gp_Pnt& pt, const gp_Pnt& ptStart, const gp_Pnt& ptEnd,
		const double& dBulge, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI int IsLeftOfCurve(const gp_Pnt2d& pt, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd,
		const double& dBulge, const double& dTol = 0.0001);

	/// @brief 求线段的法向量
	EXPORTGEOALGOAPI gp_Vec GetNormal(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge);

	/// @brief 获取线段外包点
	EXPORTGEOALGOAPI void GetCurveBoxPoints(std::vector<gp_Pnt2d>& points, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge);
	EXPORTGEOALGOAPI void GetCurveBoxPoints(std::vector<gp_Pnt>& points, const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge);

	/// @brief 获取线段的包围盒
	EXPORTGEOALGOAPI Bnd_Box GetCurveBndBox(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge);

	EXPORTGEOALGOAPI Bnd_Box2d GetCurveBndBox(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge);

	/// @brief 获取线段长度
	EXPORTGEOALGOAPI double GetCurveLength(const gp_Pnt& ptStart, const gp_Pnt& ptEnd, const double& dBulge);

	EXPORTGEOALGOAPI double GetCurveLength(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge);

	/// @brief 移除点
	EXPORTGEOALGOAPI void RemovePoint(std::list<SGeoCurveInfo>& curves, const double& dTol = 0.0001);

	/// @brief 打断
	EXPORTGEOALGOAPI void BreakCurves(std::list<SGeoCurveInfo>& curves, const double& dTol = 0.0001);

	/// @brief 按参考线打断
	EXPORTGEOALGOAPI void BreakCurves(std::list<SGeoCurveInfo>& curves, const std::list<SGeoCurveInfo>& refCurves, const double& dTol = 0.0001);

	/// @brief 根据点打断线段
	/// @param [in ] curve 需打断的线段
	/// @param [in ] pts 打断线段的点(点在线段上)
	/// @param [in ] dTol 精度
	/// @param [in ] bIsSort 是否排序点集合
	/// @return 打断后的子线段 空时则没有打断关系
	EXPORTGEOALGOAPI std::vector<SGeoCurveInfo> BreakCurveByPoint(const SGeoCurveInfo& curve,
		const std::vector<gp_Pnt2d>& pts, const double& dTol = 0.0001,
		bool bIsSort = true);

	/// @brief 计算弧线的凸度（起点终点是按圆的方向）
	/// @param [in ] ptS 圆上起点
	/// @param [in ] ptE 圆上终点
	/// @param [in ] ptCenter 圆心
	/// @param [in ] dRadius 半径
	/// @param [in ] IsAnticlockwise 是否时逆时针
	/// @param [in ] dTol 精度
	/// @return 凸度
	EXPORTGEOALGOAPI double CalcArcBulge(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const gp_Pnt2d& ptCenter,
		const double& dRadius, const bool IsAnticlockwise, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI double CalcArcBulge(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const gp_Pnt2d& ptRefStart,
		const gp_Pnt2d& ptRefEnd, const double& dRefBulge, const double& dTol = 0.0001);

	EXPORTGEOALGOAPI double CalcArcBulge(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const gp_Pnt2d& ptMid);

	/// @brief 根据线段排序线上的点
	/// @param [in ] ptS 起点
	/// @param [in ] ptE 终点
	/// @param [in ] dBulge 凸度
	/// @param [in ] vecIntersectPt 点集合
	/// @param [in ] bIsCalcClosePt 是否计算最近点
	EXPORTGEOALGOAPI std::vector<gp_Pnt2d> SortByCurve(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dBulge,
		const std::vector<gp_Pnt2d>& vecIntersectPt, bool bIsCalcClosePt = true);

	EXPORTGEOALGOAPI std::vector<gp_Pnt2d> SortByCurve(const SGeoCurveInfo& curve, const std::vector<gp_Pnt2d>& vecIntersectPt,
		bool bIsCalcClosePt = true);

	EXPORTGEOALGOAPI std::vector<gp_Pnt> SortByCurve(const gp_Pnt& ptS, const gp_Pnt& ptE,
		const std::vector<gp_Pnt>& vecIntersectPt);

	/// @brief 去除重合
	/// @param [in ] curves 线段集合
	/// @param [in ] dTol 计算精度
	/// @param [in ] IsVectorCurve 是否是矢量线段
	EXPORTGEOALGOAPI void RemoveOverlap(std::list<SGeoCurveInfo>& curves, const double& dTol = 0.0001, const bool IsVectorCurve = false);

	/// @brief 按参考线去除重合(不考虑自身重合和覆盖)(去重可能会产生小短线)
	EXPORTGEOALGOAPI void RemoveOverlap(std::list<SGeoCurveInfo>& curves, const std::list<SGeoCurveInfo>& refCurves,
		const double& dTol = 0.0001, const bool IsVectorCurve = false);

	/// @brief 求线段间的重叠区域
	EXPORTGEOALGOAPI std::list<SGeoCurveInfo> CalcTwoCurveOverlap(const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd, const double& dBulge,
		const gp_Pnt2d& ptRefStart, const gp_Pnt2d& ptRefEnd, const double& dRefBulge, const double& dTol = 0.0001, const double& dAngleTol = 0.0001);

	EXPORTGEOALGOAPI std::list<SGeoCurveInfo> CalcTwoCurveOverlap(const SGeoCurveInfo& curve, const SGeoCurveInfo& curveRef,
		const double& dTol = 0.0001, const double& dAngleTol = 0.0001);

	/// @brief 连接接相同的点
	/// @param [out] curves 线段
	/// @param [in ] dTol 精度
	/// @param [in ] bIsMerge 是否合并相同的点(false 新增一条线)
	/// @param [in ] bIsIsolated 是否只处理孤线段
	EXPORTGEOALGOAPI void ConnectEqualPoint(std::list<SGeoCurveInfo>& curves, const double& dTol = 0.0001,
		const bool bIsMerge = false, const bool bIsIsolated = false);

	/// @brief 放样弧线
	/// @param [out] vecVertex 放样后的顶点
	/// @param [in ] ptStart 起点
	/// @param [in ] ptEnd 终点
	/// @param [in ] dBulge 凸度
	/// @param [in ] nNum 放样个数
	EXPORTGEOALGOAPI void ArcSample(std::vector<gp_Pnt2d>& vecVertex, const gp_Pnt2d& ptStart, const gp_Pnt2d& ptEnd,
		const double& dBulge, const int nNum);

	/// @brief 并集 A ∪ B
	/// @param [in ] dTol 间距小于 dTol 会连接
	EXPORTGEOALGOAPI std::list<SGeoRange> RangeUnion(const SGeoRange & rangeA, const SGeoRange& rangeB, const double& dTol = 0.001);
	EXPORTGEOALGOAPI std::list<SGeoRange> RangeUnion(const std::list<SGeoRange>& ranges, const double & dTol = 0.001);

	/// @brief 交集 A ∩ B
	/// @param [in ] dTol 间距小于 dTol 会连接
	EXPORTGEOALGOAPI bool RangeIntersect(SGeoRange & interRange, const SGeoRange & rangeA, const SGeoRange & rangeB, const double & dTol = 0.001);
	EXPORTGEOALGOAPI std::list<SGeoRange> RangeIntersect(const SGeoRange & rangeA, const std::list<SGeoRange>& rangeBs, const double & dTol = 0.001);
	EXPORTGEOALGOAPI std::list<SGeoRange> RangeIntersect(const std::list<SGeoRange>& rangeAs, const std::list<SGeoRange>& rangeBs, const double & dTol = 0.001);

	/// @brief 差集 A - B
	/// @param [in ] dTol 间距小于 dTol 会连接
	EXPORTGEOALGOAPI std::list<SGeoRange> RangeSubtract(const SGeoRange & rangeA, const SGeoRange& rangeB, const double& dTol = 0.001);
	EXPORTGEOALGOAPI std::list<SGeoRange> RangeSubtract(const SGeoRange & rangeA, const std::list<SGeoRange>& rangeBs, const double & dTol = 0.001);
	EXPORTGEOALGOAPI std::list<SGeoRange> RangeSubtract(const std::list<SGeoRange>& rangeAs, const std::list<SGeoRange>& rangeBs, const double & dTol = 0.001);

	/// @brief 去短
	/// @param [in ] dTol 最短线的长度
	EXPORTGEOALGOAPI std::list<SGeoRange> RangeFilter(const std::list<SGeoRange>& ranges, const double & dTol = 0.001);

	/// @brief OBB包围盒是否相交
	/// @param [in ] ptCenter OBB的中心点
	/// @param [in ] dWidth OBB的宽度
	/// @param [in ] dHeight OBB的高度
	/// @param [in ] dAngle OBB的角度(逆时针与X轴的弧度)
	/// @param [in ] ptRefCenter 参考OBB的中心点
	/// @param [in ] dRefWidth 参考OBB的宽度
	/// @param [in ] dRefHeight 参考OBB的高度
	/// @param [in ] dRefAngle 参考OBB的角度(逆时针与X轴的弧度)
	/// @param [in ] dTol 计算精度
	EXPORTGEOALGOAPI bool IsOutWithOBB(const gp_Pnt2d& ptCenter, const double& dWidth, const double& dHeight, const double& dAngle,
		const gp_Pnt2d& ptRefCenter, const double& dRefWidth, const double& dRefHeight,
		const double& dRefAngle, const double& dTol = 0.0001);

	/// @brief OBB包围盒转换成AABB包围盒
	/// @param [in ] ptCenter OBB的中心点
	/// @param [in ] dWidth OBB的宽度
	/// @param [in ] dHeight OBB的高度
	/// @param [in ] dAngle OBB的角度(逆时针与X轴的弧度)
	EXPORTGEOALGOAPI Bnd_Box OBBToAABB(const gp_Pnt2d& ptCenter, const double& dWidth, const double& dHeight, const double& dAngle);
};

