// 3DMath.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <cmath>
#include <iostream>
#include "MathTool.h"

struct Point {
	Point(float _x, float _y) :x(_x), y(_y) {}
	float x, y;
};

// 顶点类
class Vertex3
{
public:
	Vertex3() {}
	Vertex3(float vx, float vy, float vz) :
		x(vx), y(vy), z(vz) {}
	~Vertex3() {}
	Vertex3 operator+(Vertex3 const& right) const
	{
		return Vertex3(x + right.x, y + right.y, z + right.z);
	}
	Vertex3 operator-(Vertex3 const& right) const
	{
		return Vertex3(x - right.x, y - right.y, z - right.z);
	}
	float operator*(Vertex3 const& right) const
	{
		return x * right.x + y * right.y + z * right.z;
	}
	friend Vertex3 operator*(float scalar, Vertex3 const& right);
public:
	float x, y, z;
};
// 标量乘法
Vertex3 operator*(float scalar, Vertex3 const& right)
{
	return Vertex3(scalar*right.x, scalar*right.y, scalar*right.z);
}
// 叉乘
Vertex3 crossPruduct(Vertex3 const& e1, Vertex3 const& e2)
{
	Vertex3 normal;
	normal.x = e1.y*e2.z - e2.y*e1.z;
	normal.y = e1.z*e2.x - e2.z*e1.x;
	normal.z = e1.x*e2.y - e2.x*e1.y;
	return normal;
}

// 射线类
class Ray
{
public:
	Ray() {}
	Ray(Vertex3 const& _Org, Vertex3 const& _Delta)
		: Org(_Org), Delta(_Delta) {
		length = sqrt(Delta.x*Delta.x + Delta.y*Delta.y + Delta.z*Delta.z);
		direction.x = GeoAlgoTool::Round(Delta.x / length, 6);
		direction.y = GeoAlgoTool::Round(Delta.y / length, 6);
		direction.z = GeoAlgoTool::Round(Delta.z / length, 6);
	}
	~Ray() {}

public:
	float length;		// 长度
	Vertex3 Org;		// 射线起点
	Vertex3 Delta;	// 射线长度和方向
	Vertex3 direction;	// 方向
};

// 射线与平面相交
bool rayPlaneIntersect(Ray const& ray, Vertex3 const& p0, Vertex3 const& p1, Vertex3 const& p2, Vertex3& intersection)
{
	// 计算顺时针的边向量
	Vertex3 e1 = p1 - p0;
	Vertex3 e2 = p2 - p1;
	// 计算表面法向量(无需正则化)
	Vertex3 normal = crossPruduct(e1, e2);
	// 计算点积，表示靠近三角形正面的程度
	const float dot = normal * ray.Delta;
	if (dot == 0.0f)	// 平行于三角面
		return false;
	else				// 相交于正面或反面
	{
		// 计算平面方程的d值,Ax+By+Cz=d
		float d = normal * p0;
		// 射线方程p(t) = Org + t*direction(t为射线运动距离)，带入平面方程
		float t = GeoAlgoTool::Round((d - normal * ray.Org) / (ray.direction*normal), 3);

		// 射线远离平面
		if (t < 0.0f)
			return false;
		// 射线不够长
		else if (t > ray.length)
			return false;
		// 射线与平面相交，计算出与平面的交点
		intersection.x = GeoAlgoTool::Round(ray.Org.x + t * ray.direction.x);
		intersection.y = GeoAlgoTool::Round(ray.Org.y + t * ray.direction.y);
		intersection.z = GeoAlgoTool::Round(ray.Org.z + t * ray.direction.z);
		return true;
	}
}
// 计算重心坐标
bool computeBarycentricCoords3d(Vertex3 const& p0, Vertex3 const& p1, Vertex3 const& p2, Vertex3 const& intersection, Vertex3& BarycentricCoords)
{
	// 计算顺时针的边向量
	const Vertex3 e1 = p1 - p0;
	const Vertex3 e2 = p2 - p1;
	// 计算表面法向量(无需正则化)
	const Vertex3 normal = crossPruduct(e1, e2);
	// 判断法向量中占优势的轴，选择投影平面
	float u1, u2, u3, u4;
	float v1, v2, v3, v4;
	if ((fabs(normal.x) >= fabs(normal.y)) && (fabs(normal.x) >= fabs(normal.z))) {
		// 抛弃x,向yz平面投影
		u1 = p0.y - p2.y;
		u2 = p1.y - p2.y;
		u3 = intersection.y - p0.y;
		u4 = intersection.y - p2.y;
		v1 = p0.z - p2.z;
		v2 = p1.z - p2.z;
		v3 = intersection.z - p0.z;
		v4 = intersection.z - p2.z;
	}
	else if (fabs(normal.y) >= fabs(normal.z)) {
		// 抛弃y,向xz平面投影
		u1 = p0.z - p2.z;
		u2 = p1.z - p2.z;
		u3 = intersection.z - p0.z;
		u4 = intersection.z - p2.z;
		v1 = p0.x - p2.x;
		v2 = p1.x - p2.x;
		v3 = intersection.x - p0.x;
		v4 = intersection.x - p2.x;
	}
	else {
		u1 = p0.x - p2.x;
		u2 = p1.x - p2.x;
		u3 = intersection.x - p0.x;
		u4 = intersection.x - p2.x;
		v1 = p0.y - p2.y;
		v2 = p1.y - p2.y;
		v3 = intersection.y - p0.y;
		v4 = intersection.y - p2.y;
	}
	// 计算分母，并判断是否合法
	const float denom = v1 * u2 - v2 * u1;
	if (denom == 0.0f)
		// 退化三角形--面积为0
		return false;
	// 计算重心坐标
	BarycentricCoords.x = GeoAlgoTool::Round((v4*u2 - v2 * u4) / denom, 3);
	BarycentricCoords.y = GeoAlgoTool::Round((v1*u3 - v3 * u1) / denom, 3);
	BarycentricCoords.z = GeoAlgoTool::Round(1.0f - BarycentricCoords.x - BarycentricCoords.y, 3);
	return true;
}
// 射线与三角形相交
bool rayTriangleIntersect(Ray const& ray, Vertex3 const& p0, Vertex3 const& p1, Vertex3 const& p2, Vertex3& intersection)
{
	// 判断是否与三角形所在平面相交，是就返回交点
	if (!rayPlaneIntersect(ray, p0, p1, p2, intersection))
		return false;
	// 求3D重心坐标
	Vertex3 BarycentricCoords;
	if (!computeBarycentricCoords3d(p0, p1, p2, intersection, BarycentricCoords))
		return false;
	// 判断交点是否在三角形内（通过重心坐标都为正）
	if (BarycentricCoords.x < 0 || BarycentricCoords.y < 0 || BarycentricCoords.z < 0)
		return false;
	return true;
}

// 判断点在平面凸四边形内
bool isPointInQuad(const Point& A, const Point& B, const Point& C, const Point& D, const Point& P)
{
	// 满足2D向量叉积，即AB*AP*sin，同号，则在各边的同侧
	// AB * AP = (b.x - a.x, b.y - a.y) * (p.x - a.x, p.y - a.y) = (b.x - a.x) * (p.y - a.y) + (b.y - a.y) * (p.x - a.x);
	// BC * BP = (c.x - b.x, c.y - b.y) * (p.x - b.x, p.y - b.y) = (c.x - b.x) * (p.y - b.y) + (c.y - b.y) * (p.x - b.x);
	const float a = (B.x - A.x)*(P.y - A.y) - (B.y - A.y)*(P.x - A.x);
	const float b = (C.x - B.x)*(P.y - B.y) - (C.y - B.y)*(P.x - B.x);
	const float c = (D.x - C.x)*(P.y - C.y) - (D.y - C.y)*(P.x - C.x);
	const float d = (A.x - D.x)*(P.y - D.y) - (A.y - D.y)*(P.x - D.x);

	return ((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0));
}

#ifdef OSG
#include <osg/Vec3d>
#include <osg/Vec4d>
#include <osg/BoundingBox>


// 计算3个平面交点
osg::Vec3d ComputePlanesIntersection(const osg::Vec4d& plane_one, const osg::Vec4d& plane_two, const osg::Vec4d& plane_three)
{
	const osg::Vec3d n1(plane_one.x(), plane_one.y(), plane_one.z());
	const osg::Vec3d n2(plane_two.x(), plane_two.y(), plane_two.z());
	const osg::Vec3d n3(plane_three.x(), plane_three.y(), plane_three.z());
	const double d1 = plane_one.w();
	const double d2 = plane_two.w();
	const double d3 = plane_three.w();

	const osg::Vec3d intersection = ((n3^n2)*d1 + (n1^n3)*d2 + (n2^n1)*d3) / ((n1^n2)*n3);

	std::cout << "n3^n2 = (" << (n3^n2).x() << "," << (n3^n2).y() << "," << (n3^n2).z() << ")\n";
	std::cout << "n1^n3 = (" << (n1^n3).x() << "," << (n1^n3).y() << "," << (n1^n3).z() << ")\n";
	std::cout << "n2^n1 = (" << (n2^n1).x() << "," << (n2^n1).y() << "," << (n2^n1).z() << ")\n";
	std::cout << "(n1^n2)*n3 = " << (n1^n2)*n3 << "\n";

	return  intersection;
}

// 射线与AABB相交
namespace {
	// 这里只判断与包围盒是否相交，不求交点个数，所以只考虑与各平面正面相交的情况
	bool RayAABBIntersects(const osg::Vec3& origin, const osg::Vec3& direction, const osg::BoundingBox& aabb)
	{
		osg::Vec3 IntersectionOnPlane; //射线与包围盒某面的交点
		osg::Vec3 min = aabb._min;
		osg::Vec3 max = aabb._max;

		float t;

		//分别判断射线与各维度面的相交情况

		//判断射线与包围盒x轴方向的面是否有交点
		if (direction.x() != 0.f) //若射线方向矢量的x轴分量为0，射线不可能经过包围盒朝x轴方向的两个面
		{
			//使用射线与平面相交的公式求交点
			// 这里只判断与包围盒是否相交，不求交点个数，所以只考虑与各平面正面相交的情况
			// 不考虑与背面相交的情况，排除 min.x < origin.x < max.x 的情况
			if (direction.x() > 0.f)//若射线沿x轴正方向偏移
				t = (min.x() - origin.x()) / direction.x();
			else  //射线沿x轴负方向偏移
				t = (max.x() - origin.x()) / direction.x();

			if (t >= 0.f) //t>=0时则射线与平面相交
			{
				IntersectionOnPlane = origin + direction * t;
				//判断交点是否在当前面内
				if (min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y() && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					return true;
				//是否在棱上，在; 分量在指定范围内，才会进入包围盒，否则只是蹭过包围盒
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					return true;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					return true;
				else if (IntersectionOnPlane.z() == min.z() && direction.z() > 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					return true;
				else if (IntersectionOnPlane.z() == max.z() && direction.z() < 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					return true;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && IntersectionOnPlane.z() == min.z() && direction.z() > 0.f)
					return true;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && IntersectionOnPlane.z() == min.z() && direction.z() > 0.f)
					return true;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && IntersectionOnPlane.z() == max.z() && direction.z() < 0.f)
					return true;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && IntersectionOnPlane.z() == max.z() && direction.z() < 0.f)
					return true;
			}
		}

		//若射线沿y轴方向有分量 判断是否与包围盒y轴方向有交点
		if (direction.y() != 0.f)
		{
			if (direction.y() > 0.f)
				t = (min.y() - origin.y()) / direction.y();
			else
				t = (max.y() - origin.y()) / direction.y();

			if (t >= 0.f)
			{
				IntersectionOnPlane = origin + direction * t;

				if (min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z() && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					return true;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() > 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					return true;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() < 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					return true;
				else if (IntersectionOnPlane.z() == min.z() && direction.z() > 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					return true;
				else if (IntersectionOnPlane.z() == max.z() && direction.z() < 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					return true;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() > 0.f && IntersectionOnPlane.z() == min.z() && direction.z() > 0.f)
					return true;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() < 0.f && IntersectionOnPlane.z() == min.z() && direction.z() > 0.f)
					return true;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() > 0.f && IntersectionOnPlane.z() == max.z() && direction.z() < 0.f)
					return true;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() < 0.f && IntersectionOnPlane.z() == max.z() && direction.z() < 0.f)
					return true;
			}
		}

		//若射线沿z轴方向有分量 判断是否与包围盒y轴方向有交点
		if (direction.z() != 0.f)
		{
			if (direction.z() > 0.f)
				t = (min.z() - origin.z()) / direction.z();
			else
				t = (max.z() - origin.z()) / direction.z();

			if (t >= 0.f)
			{
				IntersectionOnPlane = origin + direction * t;

				if (min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x() && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					return true;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					return true;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					return true;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() > 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					return true;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() < 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					return true;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && IntersectionOnPlane.x() == min.x() && direction.x() > 0.f)
					return true;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && IntersectionOnPlane.x() == min.x() && direction.x() > 0.f)
					return true;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && IntersectionOnPlane.x() == max.x() && direction.x() < 0.f)
					return true;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && IntersectionOnPlane.x() == max.x() && direction.x() < 0.f)
					return true;
			}
		}

		return false;
	}

	// 判断指向包围盒上顶点的视线，是否被包围盒遮挡
	// eye为起点，目标顶点为终点的射线
	// 其实一定会有交点的，只是可能在棱上
	// 若与包围盒上第一个正面相交（交点在棱或顶点也算），且满足t<1 <=> 视线被遮挡
	bool IsOutOfSight(const osg::Vec3& eye, const osg::Vec3& target, const osg::BoundingBox& aabb)
	{
		osg::Vec3 IntersectionOnPlane; //射线与包围盒某面的交点
		const osg::Vec3 distance = target - eye;
		const osg::Vec3 min = aabb._min;
		const osg::Vec3 max = aabb._max;

		// 可能在某度维度上与两个平面有交点，一个正面相交，一个背面
		float t = -1.f;	// 正面交点

		//分别判断射线与各维度面的相交情况

		//判断射线与包围盒x轴方向的面是否有交点
		if (distance.x() != 0.f) //若射线方向矢量的x轴分量为0，射线不可能经过包围盒朝x轴方向的两个面
		{
			//使用射线与平面相交的公式求交点
			if (distance.x() > 0.f)//若射线沿x轴正方向偏移
				t = (min.x() - eye.x()) / distance.x();
			else
				t = (max.x() - eye.x()) / distance.x();
			//正面交点范围0<=t<1，t=1时，交点就是目标点，且未被遮挡
			if (0.f <= t && t < 1.f)
			{
				IntersectionOnPlane = eye + distance * t;
				if (min.y() <= IntersectionOnPlane.y() && IntersectionOnPlane.y() <= max.y() && min.z() <= IntersectionOnPlane.z() && IntersectionOnPlane.z() <= max.z())
					return true;
			}
		}

		//若射线沿y轴方向有分量 判断是否与包围盒y轴方向有交点
		if (distance.y() != 0.f)
		{
			if (distance.y() > 0.f)
				t = (min.y() - eye.y()) / distance.y();
			else
				t = (max.y() - eye.y()) / distance.y();

			if (0.f <= t && t < 1.f)
			{
				IntersectionOnPlane = eye + distance * t;
				if (min.z() <= IntersectionOnPlane.z() && IntersectionOnPlane.z() <= max.z() && min.x() <= IntersectionOnPlane.x() && IntersectionOnPlane.x() <= max.x())
					return true;
			}
		}

		//若射线沿z轴方向有分量 判断是否与包围盒y轴方向有交点
		if (distance.z() != 0.f)
		{
			if (distance.z() > 0.f)
				t = (min.z() - eye.z()) / distance.z();
			else
				t = (max.z() - eye.z()) / distance.z();

			if (0.f <= t && t < 1.f)
			{
				IntersectionOnPlane = eye + distance * t;
				if (min.x() <= IntersectionOnPlane.x() && IntersectionOnPlane.x() <= max.x() && min.y() <= IntersectionOnPlane.y() && IntersectionOnPlane.y() <= max.y())
					return true;
			}
		}

		return false;
	}

	// 弃用，此函数返回射线与AABB交点个数（棱上、角上会重复，可以set排重）
	unsigned CountRayAABBIntersects(const osg::Vec3& origin, const osg::Vec3& direction, const osg::BoundingBox& aabb)
	{
		osg::Vec3 IntersectionOnPlane; //射线与包围盒某面的交点
		osg::Vec3 min = aabb._min;
		osg::Vec3 max = aabb._max;

		// 可能在某度维度上与两个平面有交点，一个正面相交，一个背面
		float t1 = -1.f;	// 正面交点
		float t2 = -1.f;	// 背面交点
		unsigned count = 0;

		//分别判断射线与各维度面的相交情况

		//判断射线与包围盒x轴方向的面是否有交点
		if (direction.x() != 0.f) //若射线方向矢量的x轴分量为0，射线不可能经过包围盒朝x轴方向的两个面
		{
			//使用射线与平面相交的公式求交点
			if (direction.x() > 0.f)//若射线沿x轴正方向偏移
			{
				if (min.x() >= origin.x())	//2个交点
				{
					t1 = (min.x() - origin.x()) / direction.x();
					t2 = (max.x() - origin.x()) / direction.x();
				}
				else if (min.x() < origin.x() && origin.x() < max.x())
				{
					t1 = -1.f;
					t2 = (max.x() - origin.x()) / direction.x();
				}
				else
					t1 = t2 = -1.f;
			}
			else
			{
				if (max.x() <= origin.x())
				{
					t1 = (max.x() - origin.x()) / direction.x();
					t2 = (min.x() - origin.x()) / direction.x();
				}
				else if (max.x() > origin.x() && origin.x() > min.x())
				{
					t1 = -1.f;
					t2 = (min.x() - origin.x()) / direction.x();
				}
				else
					t1 = t2 = -1.f;
			}

			if (t1 >= 0.f) //t>=0时则射线与平面相交
			{
				IntersectionOnPlane = origin + direction * t1;
				if (min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y() && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				//是否在棱上，在; 分量在指定范围内，才会进入包围盒，否则只是蹭过包围盒
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				else if (IntersectionOnPlane.z() == min.z() && direction.z() > 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.z() == max.z() && direction.z() < 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && IntersectionOnPlane.z() == min.z() && direction.z() > 0.f)
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && IntersectionOnPlane.z() == min.z() && direction.z() > 0.f)
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && IntersectionOnPlane.z() == max.z() && direction.z() < 0.f)
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && IntersectionOnPlane.z() == max.z() && direction.z() < 0.f)
					++count;
			}
			if (t2 >= 0.f)
			{
				IntersectionOnPlane = origin + direction * t2;
				if (min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y() && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				//是否在棱上，在; 判断起点是否在指定范围内，保证走出包围盒，而不是蹭过
				else if (IntersectionOnPlane.y() == min.y() && direction.y() < 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() > 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				else if (IntersectionOnPlane.z() == min.z() && direction.z() < 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.z() == max.z() && direction.z() > 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() < 0.f && IntersectionOnPlane.z() == min.z() && direction.z() < 0.f)
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() > 0.f && IntersectionOnPlane.z() == min.z() && direction.z() < 0.f)
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() < 0.f && IntersectionOnPlane.z() == max.z() && direction.z() > 0.f)
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() > 0.f && IntersectionOnPlane.z() == max.z() && direction.z() > 0.f)
					++count;
			}
		}

		//若射线沿y轴方向有分量 判断是否与包围盒y轴方向有交点
		if (direction.y() != 0.f)
		{
			if (direction.y() > 0.f)
			{
				if (min.y() >= origin.y())	//2个交点
				{
					t1 = (min.y() - origin.y()) / direction.y();
					t2 = (max.y() - origin.y()) / direction.y();
				}
				else if (min.y() < origin.y() && origin.y() < max.y())
				{
					t1 = -1.f;
					t2 = (max.y() - origin.y()) / direction.y();
				}
				else
					t1 = t2 = -1.f;
			}
			else
			{
				if (max.y() <= origin.y())
				{
					t1 = (max.y() - origin.y()) / direction.y();
					t2 = (min.y() - origin.y()) / direction.y();
				}
				else if (min.y() < origin.y() && origin.y() < max.y())
				{
					t1 = -1.f;
					t2 = (min.y() - origin.y()) / direction.y();
				}
				else
					t1 = t2 = -1.f;
			}

			if (t1 >= 0.f)
			{
				IntersectionOnPlane = origin + direction * t1;
				if (min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z() && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() > 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() < 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				else if (IntersectionOnPlane.z() == min.z() && direction.z() > 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.z() == max.z() && direction.z() < 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() > 0.f && IntersectionOnPlane.z() == min.z() && direction.z() > 0.f)
					++count;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() < 0.f && IntersectionOnPlane.z() == min.z() && direction.z() > 0.f)
					++count;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() > 0.f && IntersectionOnPlane.z() == max.z() && direction.z() < 0.f)
					++count;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() < 0.f && IntersectionOnPlane.z() == max.z() && direction.z() < 0.f)
					++count;
			}
			if (t2 >= 0.f)
			{
				IntersectionOnPlane = origin + direction * t2;
				if (min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z() && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() < 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() > 0.f && min.z() < IntersectionOnPlane.z() && IntersectionOnPlane.z() < max.z())
					++count;
				else if (IntersectionOnPlane.z() == min.z() && direction.z() < 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.z() == max.z() && direction.z() > 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() < 0.f && IntersectionOnPlane.z() == min.z() && direction.z() < 0.f)
					++count;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() > 0.f && IntersectionOnPlane.z() == min.z() && direction.z() < 0.f)
					++count;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() < 0.f && IntersectionOnPlane.z() == max.z() && direction.z() > 0.f)
					++count;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() > 0.f && IntersectionOnPlane.z() == max.z() && direction.z() > 0.f)
					++count;
			}
		}

		//若射线沿z轴方向有分量 判断是否与包围盒y轴方向有交点
		if (direction.z() != 0.f)
		{
			if (direction.z() > 0.f)
			{
				if (min.z() >= origin.z())	//2个交点
				{
					t1 = (min.z() - origin.z()) / direction.z();
					t2 = (max.z() - origin.z()) / direction.z();
				}
				else if (min.z() < origin.z() && origin.z() < max.z())
				{
					t1 = -1.f;
					t2 = (max.z() - origin.z()) / direction.z();
				}
				else
					t1 = t2 = -1.f;
			}
			else
			{
				if (max.z() <= origin.z())
				{
					t1 = (max.z() - origin.z()) / direction.z();
					t2 = (min.z() - origin.z()) / direction.z();
				}
				else if (min.z() < origin.z() && origin.z() < max.z())
				{
					t1 = -1.f;
					t2 = (min.z() - origin.z()) / direction.z();
				}
				else
					t1 = t2 = -1.f;
			}

			if (t1 >= 0.f)
			{
				IntersectionOnPlane = origin + direction * t1;
				if (min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x() && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() > 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() < 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && IntersectionOnPlane.x() == min.x() && direction.x() > 0.f)
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && IntersectionOnPlane.x() == min.x() && direction.x() > 0.f)
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() > 0.f && IntersectionOnPlane.x() == max.x() && direction.x() < 0.f)
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() < 0.f && IntersectionOnPlane.x() == max.x() && direction.x() < 0.f)
					++count;
			}
			if (t2 >= 0.f)
			{
				IntersectionOnPlane = origin + direction * t2;
				if (min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x() && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() < 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() > 0.f && min.x() < IntersectionOnPlane.x() && IntersectionOnPlane.x() < max.x())
					++count;
				else if (IntersectionOnPlane.x() == min.x() && direction.x() < 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.x() == max.x() && direction.x() > 0.f && min.y() < IntersectionOnPlane.y() && IntersectionOnPlane.y() < max.y())
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() < 0.f && IntersectionOnPlane.x() == min.x() && direction.x() < 0.f)
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() > 0.f && IntersectionOnPlane.x() == min.x() && direction.x() < 0.f)
					++count;
				else if (IntersectionOnPlane.y() == min.y() && direction.y() < 0.f && IntersectionOnPlane.x() == max.x() && direction.x() > 0.f)
					++count;
				else if (IntersectionOnPlane.y() == max.y() && direction.y() > 0.f && IntersectionOnPlane.x() == max.x() && direction.x() > 0.f)
					++count;
			}
		}

		return count;
	}
}

int main()
{
	auto res = isPointInQuad(Point(0.0, 0.0), Point(2.0, 1.0), Point(4.0, 1.0), Point(2.0, 0.0), Point(1.8, 0.5));
	std::cout << std::boolalpha << res << std::endl;
	std::cout << "--------------------------" << std::endl;

	osg::Vec4d one = osg::Vec4d(1, 0, 0, 1);
	osg::Vec4d two = osg::Vec4d(0, 1, 0, 1);
	osg::Vec4d three = osg::Vec4d(0, 0, 1, 1);

	osg::Vec3d intersection = ComputePlanesIntersection(one, two, three);

	std::cout << "intersection: (" << intersection.x() << "," << intersection.y() << "," << intersection.z() << ")\n";

	osg::Vec3 min(0., 0., 0.), max(2., 2., 2.), eye(-1, 0, 0);
	osg::BoundingBox aabb(min, max);
	std::cout << "--------------------------" << std::endl;

	std::cout << "Is intersected ?: " << RayAABBIntersects(min, max - min, aabb) << std::endl;
	std::cout << "Is out of sight: " << IsOutOfSight(eye, osg::Vec3(2, 0, 0) - eye, aabb) << std::endl;
	std::cout << "Count of intersections: " << CountRayAABBIntersects(min, max - min, aabb) << std::endl;

	return 0;
}
#endif // OSG

