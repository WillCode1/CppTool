#pragma once

#ifdef BIMGEOALGOR_MODULE
#define EXPORTGEOALGOCLASS __declspec(dllexport)
#define EXPORTGEOALGOAPI __declspec(dllexport)
#else
#define EXPORTGEOALGOCLASS __declspec(dllimport)
#define EXPORTGEOALGOAPI __declspec(dllimport)
#endif

// check if c++11 is supported
#if __cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1900)
#define GEO_CXX11_IS_SUPPORTED
#endif

#include "gp_Pnt2d.hxx"
#include "gp_Pnt.hxx"

/// @brief 多边形关系
enum EPointWidthPolygonRel
{
	eOnPolygon = 0,		//	在多边形上
	eInPolygon,			//	在多边形内
	eOutPolygon			//	在多边形外
};

/// @brief 多边形关系
enum EPolygonRel
{
	eIntersect = 0,			//	相交
	eTagent,				//	相切
	eOuter,					//	相离
	eAInB,					//	A在B内
	eAInBWithTagent,		//	A在B内并且有重叠
	eBInA,					//	B在A内
	eBInAWithTagent,		//	A在B内并且有重叠
	eOverlap				//	重叠
};

/// @brief 两个多边形的布尔运算
enum EBooleanOperation
{
	ePolygonIntersect = 0,			//	交集
	ePolygonUnion,					//	并集
	ePolygonSubTract				//	差集
};

/// @brief 线段信息
struct SGeoCurveInfo
{
	SGeoCurveInfo() : dBulge(0.) {}
	gp_Pnt2d		ptS;				//	起点
	gp_Pnt2d		ptE;				//	终点
	double			dBulge;				//	凸度
};

/// @brief 线段信息
struct SGeoCurveInfo3D
{
	SGeoCurveInfo3D() : dBulge(0.) {}
	gp_Pnt			ptS;				//	起点
	gp_Pnt			ptE;				//	终点
	double			dBulge;				//	凸度
};

/// @brief 圆弧信息
struct SGeoArcInfo
{
	gp_Pnt2d		ptCenter;				//	圆心
	double			dRadius;				//	半径
	bool			bIsAntiClockwise;		//	逆时针
};

/// @brief 三角形
struct SGeoTriangle
{
	gp_Pnt2d		pt1;
	gp_Pnt2d		pt2;
	gp_Pnt2d		pt3;
};

/// @brief 范围(一维区间)
struct SGeoRange
{
	SGeoRange()
	{

	}

	SGeoRange(const double& dmin, const double& dmax)
	{
		dMin = std::min(dmin, dmax);
		dMax = std::max(dmin, dmax);
	}
	double dMin;
	double dMax;
};

/// @brief 两个相等的点
class EXPORTGEOALGOCLASS CEqualPointKey
{
public:
	CEqualPointKey(const gp_Pnt2d& pt);
	~CEqualPointKey();
public:
	bool operator < (const CEqualPointKey& rhs) const;

	const gp_Pnt2d& GetPt() const;
public:
	static double s_tol;
private:
	gp_Pnt2d	m_pt;
};