#pragma once

#include "BimGeoPolyline.h"
#include <list>
#include <set>
#include <map>

/// @class CBimGeoPolylineOffset
/// @brief 多边形偏移算法(多边形须是逆时针)
/// @detail 多边形的偏移填充算法 https://www.jianshu.com/p/8c7e7c1afcb6

class CBimGeoPolylineOffset
{
public:
	CBimGeoPolylineOffset();
	~CBimGeoPolylineOffset();
public:
	/// @brief 非等距偏移
	/// @param [in ] polyline 偏移路径
	/// @param [in ] vecOffset 偏移距离（与路径的线段个数一致）
	/// @return 成功或失败
	bool DoIt(const CBimGeoPolyline& polyline, const std::vector<double>& vecOffset, const double& dTol);

	/// @brief 等距偏移
	/// @param [in ] polyline 偏移路径
	/// @param [in ] dOffset 偏移距离
	/// @return 成功或失败
	bool DoIt(const CBimGeoPolyline& polyline, const double& dOffset, const double& dTol);

	/// @brief 获取偏移后的多边形
	const std::list<CBimGeoPolyline>& GetOffsetPolylines() const;
private:
	/// @brief 清空数据
	void Clear();

	/// @brief 偏移
	bool CalcOffsetImp(const CBimGeoPolyline& polyline, const std::vector<double>& vecOffset);

	/// @brief 获取有效的多边形和偏移距离
	CBimGeoPolyline GetValidPolyline(std::vector<double>& vecOffset, const CBimGeoPolyline& polyline) const;

	/// @brief 边退化
	void BoundaryDegenerate();

	/// @brief 去除退化边
	void RemoveDegenerateBoundary();

	/// @brief 计算顶点的偏移向量
	void CalcVertexOffsetVec(const int nPreIndex, const int nCurIndex, const int nNextIndex);

	/// @brief 生成多边形
	void GeneratePolylines();

	/// @brief 计算有效封闭区域
	void CalcValidPolylines(const CBimGeoPolyline& polyline, const std::set<int>& inValidSide);

	/// @brief 偏移圆弧线段
	void OffsetArc(CBimGeoPolyline& retPolyline, std::set<int>& inValidSide, const int nPreIndex, const int nIndex) const;

	/// @brief 偏移线段
	SGeoCurveInfo OffsetCurve(const gp_Pnt2d& ptS, const gp_Pnt2d& ptE, const double& dBulge,
		const double& dOffset, bool& bIsValid) const;
private:
	CBimGeoPolyline						m_orgPolyline;			//	原始多边形
	std::vector<double>					m_vecOffset;			//	偏移距离
	std::list<CBimGeoPolyline>			m_retPolylines;			//	计算返回的数据
	double								m_dTol;					//	计算精度
	std::vector<gp_Vec2d>				m_vecOffsetVertex;		//	顶点偏移的向量
	std::set<int>						m_arcPointSet;			//	圆弧端点的集合
	std::set<int>						m_inValidVertex;		//	顶点退化
};

