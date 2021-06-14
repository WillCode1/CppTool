#pragma once

#include <list>
#include <map>
#include <set>
#include "gp_Pnt2d.hxx"
#include "BimGeoPolyline.h"

enum ESearchType
{
	eMaxPolygon	=	0,	//	最大封闭区域
	eMinPolygon,		//	最小封闭区域
};

/// @brief 两个相等的角度
class CEqualAngleKey
{
public:
	CEqualAngleKey(const double& dAngle);
	~CEqualAngleKey();
public:
	bool operator < (const CEqualAngleKey& rhs) const;

	const double& GetAngle() const;
	operator double() const;
public:
	static double s_tol;
private:
	double	m_dAngle;
};

/// @class CBimSearchPolygonTool
/// @brief 搜索多边形工具
/// @detail 基于夹角变化趋势的多边形自动搜索和生成算法 https://wenku.baidu.com/view/508fc5a45fbfc77da269b1c1.html

class CBimSearchPolygonTool
{
public:
	CBimSearchPolygonTool();
	~CBimSearchPolygonTool();
public:
	/// @brief 执行搜索
	/// @param [in ] curves 线段
	/// @param [in ] dTol 计算精度
	/// @param [in ] IsBreak 是否打断
	/// @param [in ] IsVectorCurve 是否是矢量线段
	void DoIt(const std::list<SGeoCurveInfo>& curves, const double& dTol, 
		const ESearchType eType, const bool IsBreak = true, const bool IsVectorCurve = false, const double& dAngleTol = 0.01);

	/// @brief 获取最大封闭区域
	const std::list<CBimGeoPolyline>& GetMaxPolygon() const;

	/// @brief 获取最小封闭区域
	const std::list<CBimGeoPolyline>& GetMinPolygon() const;
private:
	/// @brief 情况
	void Clear();

	/// @统计点与边的信息
	void TotalPointWithCurveRel();

	/// @brief 搜索封闭区域
	void SearchPolygon();

	/// @brief 搜索封闭区域
	void SearchPolygonImp(const SGeoCurveInfo* pCurve);

	/// @brief 去除无效的封闭区域
	void RemoveInValidPolygon();

	/// @brief 获取边界点
	const gp_Pnt2d* GetBoundaryPoint();

	/// @brief 获取边界
	const SGeoCurveInfo* GetBoundary();

	/// @brief 找封闭区域
	void SearchPolygon(CBimGeoPolyline& polygon, const SGeoCurveInfo* pCurve, std::set<const SGeoCurveInfo*>& useCurve) const;

	/// @brief 获取角度
	double GetAngle(const SGeoCurveInfo* pCurve) const;

	/// @brief 获取下一个线段
	const SGeoCurveInfo* GetNextCurve(double& dRetAngle, double& dCurAngle2, const SGeoCurveInfo*& pFirstCurve, const std::list<const SGeoCurveInfo*>& curves,
		const double& dCurAngle, const double& dRefAngle, const SGeoCurveInfo* pRefCurve) const;

	/// @brief 修改边界
	void ModifyBoundary(const CBimGeoPolyline& polyline, const std::set<const SGeoCurveInfo*>& useCurve);

	/// @brief 是否有效
	bool IsValidPolyline(const CBimGeoPolyline& polyline) const;

	/// @brief 是否反向
	bool IsReverse(const SGeoCurveInfo* pCurve, const SGeoCurveInfo* pRefCurve) const;

	/// @brief 去除孤线
	void RemoveIndependentCurve();

	/// @brief 合并共线
	void MergeCollinear(std::list<CBimGeoPolyline>& polylines) const;
private:
	typedef std::map<int, std::list<SGeoCurveInfo*>> MAPBOUNDARY;
	typedef std::map<CEqualPointKey, std::list<SGeoCurveInfo*>> POINTWITHCURVEMAP;

	std::list<SGeoCurveInfo>		m_curves;				//	线段的集合
	std::list<CBimGeoPolyline>		m_maxPolygon;			//	最大封闭区域
	std::list<CBimGeoPolyline>		m_minPolygon;			//	最小封闭读取
	POINTWITHCURVEMAP				m_mapPointWithCurve;	//	点与边的信息
	std::map<CEqualPointKey, int>	m_mapBoundaryPoint;		//	边界点
	MAPBOUNDARY						m_mapBoundary;			//	边界边
	double							m_dTol;					//	计算精度
	double							m_dAngleTol;			//	计算角度精度
	ESearchType						m_eSearchType;			//	查找封闭区域类型
	bool							m_bIsVectorCurve;		//	是否是矢量线段
};