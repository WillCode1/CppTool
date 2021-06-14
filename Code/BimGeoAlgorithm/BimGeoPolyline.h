#pragma once

#include "BimGeoAlgorithmDef.h"
#include <vector>
#include <list>
#include "gp_Pnt2d.hxx"
#include "Bnd_Box2d.hxx"
#include "gp_Trsf2d.hxx"

class EXPORTGEOALGOCLASS CBimGeoPolyline
{
public:
	CBimGeoPolyline();
	CBimGeoPolyline(const std::vector<gp_Pnt2d>& vecVertex, const std::vector<double>& vecBulge, const bool bIsClose);
	~CBimGeoPolyline();
public:
	/// @brief 获取起点
	gp_Pnt2d GetStartPoint() const;

	/// @brief 获取终点
	gp_Pnt2d GetEndPoint() const;

	/// @brief 获取顶点
	const std::vector<gp_Pnt2d>& GetVertexes() const;

	/// @brief 获取凸度
	const std::vector<double>& GetBulges() const;

	/// @brief 设置顶点
	void SetVertexes(const std::vector<gp_Pnt2d>& vecVertex);
	void SetVertex(const int nIndex, const gp_Pnt2d& pt);

	/// @brief 设置凸度
	void SetBulges(const std::vector<double>& vecBulge);
	void SetBugle(const int nIndex, const double& dBulge);

	/// @brief 添加顶点
	void AddVertex(const gp_Pnt2d& pt, const double& dBulge);

	/// @brief 设置指定点的坐标和凸度
	void SetVertex(const int nIndex, const gp_Pnt2d& pt, const double& dBulge);

	/// @brief 是否闭合
	bool IsClose() const;

	/// @brief 设置是否闭合
	void setClose(const bool bTrue);

	/// @brief 法向
	gp_Vec Normal() const;

	/// @brief 获取线段的个数
	int GetCurveNumber() const;

	/// @brief 获取指定线段
	/// @param [out] ptS 线段起点
	/// @param [out] ptE 线段终点
	/// @param [out] dBulge 线段凸度
	/// @param [in ] nIndex 下标 从0开始
	/// @retrun false 下标越界
	bool GetCurve(gp_Pnt2d& ptS, gp_Pnt2d& ptE, double& dBulge, const int nIndex) const;

	/// @brief 移除某个端点
	void RemoveVertex(const int nIndex);

	/// @brief 获取外包
	Bnd_Box2d GetBndBox() const;

	/// @brief 反向
	void Reverse();

	/// @brief 矩阵变换
	void Transform(const gp_Trsf2d& transMat);

	/// @brief 放样
	/// @param [in ] nNum 弧度放样的个数
	void Sample(const int nNum = 9);
public:
	bool operator == (const CBimGeoPolyline& rhs) const;
private:
	std::vector<gp_Pnt2d>		m_vecVertex;
	std::vector<double>			m_vecBulge;
	bool						m_bIsClose;
};

/// @brief 带动的多边形
struct SBimGeoPolygonWithHole
{
	CBimGeoPolyline					orgPolygon;
	std::list<CBimGeoPolyline>		holes;
};