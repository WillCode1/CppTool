// rayConeIntersect.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>
#include <math.h>
#include <string.h>

// 保留小数函数
float Round(float decimals, int digits = 6)
{
	if (!digits)
		return roundf(decimals);
	else if (digits > 0)
		return roundf(decimals * pow(10, digits)) / pow(10, digits);
	else
		return roundf(decimals / pow(10, -digits)) * pow(10, -digits);
}

// 圆锥参数
const int planeSum = 360;			// 圆锥侧面总数，也就是底面圆周上顶点总数
const float height = 10;			// 圆锥高
const float radius = 5;			// 半径
const float PI = 3.14159265358979323846;

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

Vertex3 operator*(float scalar, Vertex3 const& right)
{
	return Vertex3(scalar*right.x, scalar*right.y, scalar*right.z);
}

Vertex3 crossPruduct(Vertex3 const& e1, Vertex3 const& e2)
{
	Vertex3 normal;
	normal.x = e1.y*e2.z - e2.y*e1.z;
	normal.y = e1.z*e2.x - e2.z*e1.x;
	normal.z = e1.x*e2.y - e2.x*e1.y;
	return normal;
}

// 射线参数
#if 0
const Vertex3 Org = Vertex3(0, 0, 0);
const Vertex3 Delta = Vertex3(0, 10, 0);
#else
const Vertex3 Org = Vertex3(0, 0, 0);		// 射线起点
const Vertex3 Delta = Vertex3(5, 5, 5);		// 射线长度和方向
#endif // 0

class Cone {
public:
	Cone() {}
	Cone(int _planeSum, float _radius, float _height, Vertex3 _center = Vertex3(0, 0, 0))
		: planeSum(_planeSum), radius(_radius), height(_height), center(_center), tip(Vertex3(_center.x, _center.y + height, _center.z)) {}
	~Cone() {}

	// 写入圆锥顶点
	void printVertices(char* buf, FILE *p_file)
	{
		// 均匀切分圆心角，写入底面圆周坐标
		// 圆心角，步进
		float angle = 0, step = 2 * PI / planeSum;
		Vertex3 point(0, 0, 0);	// 底面圆周上的点

		for (size_t i = 1; i <= planeSum; i++)
			//while (angle >= 0 && angle < 2 * PI)
		{
			point.x = radius * cos(angle);
			point.z = radius * sin(angle);

			sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', point.x, 0.0, point.z);//(x,0.0,y)
			fwrite(buf, sizeof(char), strlen(buf), p_file);

			angle += step;
		}
		// 写入圆锥顶点，索引91
		sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', tip.x, tip.y, tip.z);
		fwrite(buf, sizeof(char), strlen(buf), p_file);
		// 写入底面圆心，索引92
		sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', center.x, center.y, center.z);
		fwrite(buf, sizeof(char), strlen(buf), p_file);
	}
	//写入顶点法线
	void printNormals(char* buf, FILE *p_file)
	{
		// 1.侧面顶点法线（逆时针，斜向下）
		// 设一个侧面中，顶点与底面两点分别构成两向量v1，v2
		Vertex3 v1, v2, normal;
		float angle = 0, step = 2 * PI / planeSum;
		for (size_t i = 1; i <= planeSum; i++)
			//while (angle >= 0 && angle < 2 * PI)
		{
			v1 = Vertex3(radius * cos(angle), -tip.y, radius * sin(angle));
			v2 = Vertex3(radius * cos(angle + step), -tip.y, radius * sin(angle + step));

			// v1 x v2 可获得顶点法向量normal
			normal = crossPruduct(v1, v2);

			// 向量标准化
			float modulus = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
			normal.x /= modulus;
			normal.y /= modulus;
			normal.z /= modulus;

			sprintf(buf, "%s %.6lf %.6lf %.6lf\n", "vn", normal.x, normal.y, normal.z);
			fwrite(buf, sizeof(char), strlen(buf), p_file);

			angle += step;
		}

		// 2.底面顶点法线（逆时针，向下）
		sprintf(buf, "%s %.6lf %.6lf %.6lf\n", "vn", 0.0, -1.0, 0.0);
		fwrite(buf, sizeof(char), strlen(buf), p_file);
	}
	// 画锥体表面
	void DrawFace(char* buf, FILE *p_file)
	{
		// 画锥体各个侧面，个数为planeSum
		int vCount = 1;		// 顶点索引
		int nCount = 1;		// 法线索引
		while (vCount <= planeSum)
		{
			// 顶点索引 = 侧面个数，贴图索引都 = 1
			sprintf(buf, "%c %d/%d/%d %d/%d/%d %d/%d/%d\n", 'f', vCount, 1, nCount, (vCount + 1 > planeSum ? (vCount + 1) % planeSum : vCount + 1), 1, nCount, planeSum + 1, 1, nCount);
			fwrite(buf, sizeof(char), strlen(buf), p_file);
			vCount++;
			nCount++;
		}
		// 画锥体底面
		for (vCount = 1; vCount <= planeSum; vCount++)
		{
			sprintf(buf, "%c %d/%d/%d %d/%d/%d %d/%d/%d\n", 'f', vCount, 1, nCount, (vCount + 1 > planeSum ? (vCount + 1) % planeSum : vCount + 1), 1, nCount, planeSum + 2, 1, nCount);
			fwrite(buf, sizeof(char), strlen(buf), p_file);
		}
	}

public:
	int planeSum;		// 圆锥侧面数，用于表示精度
	float radius;
	float height;
	Vertex3 center;
	Vertex3 tip;
};

class Ray
{
public:
	Ray() {}
	Ray(Vertex3 const& _Org, Vertex3 const& _Delta)
		: Org(_Org), Delta(_Delta) {
		length = sqrt(Delta.x*Delta.x + Delta.y*Delta.y + Delta.z*Delta.z);
		direction.x = Round(Delta.x / length, 6);
		direction.y = Round(Delta.y / length, 6);
		direction.z = Round(Delta.z / length, 6);
	}
	~Ray() {}

public:
	float length;		// 长度
	Vertex3 Org;		// 射线起点
	Vertex3 Delta;	// 射线长度和方向
	Vertex3 direction;	// 方向
};

bool rayPlaneIntersect(Ray const& ray, Vertex3 const& p0, Vertex3 const& p1, Vertex3 const& p2, Vertex3& intersection)
{
	// 计算顺时针的边向量
	Vertex3 e1 = p1 - p0;
	Vertex3 e2 = p2 - p1;
	// 计算表面法向量(无需正则化)
	Vertex3 normal = crossPruduct(e1, e2);
	// 计算点积，表示靠近三角形正面的程度
	float dot = normal * ray.Delta;
	if (dot == 0.0f)	// 平行于三角面
		return false;
	else				// 相交于正面或反面
	{
		// 计算平面方程的d值,Ax+By+Cz=d
		float d = normal * p0;
		// 射线方程p(t) = Org + t*direction(t为射线运动距离)，带入平面方程
		float t = Round((d - normal * ray.Org) / (ray.direction*normal), 3);

		// 射线远离平面
		if (t < 0.0f)
			return false;
		// 射线不够长
		else if (t > ray.length)
			return false;
		// 射线与平面相交，计算出与平面的交点
		intersection.x = Round(ray.Org.x + t * ray.direction.x);
		intersection.y = Round(ray.Org.y + t * ray.direction.y);
		intersection.z = Round(ray.Org.z + t * ray.direction.z);
		return true;
	}
}

bool computeBarycentricCoords3d(Vertex3 const& p0, Vertex3 const& p1, Vertex3 const& p2, Vertex3 const& intersection, Vertex3& BarycentricCoords)
{
	// 计算顺时针的边向量
	Vertex3 e1 = p1 - p0;
	Vertex3 e2 = p2 - p1;
	// 计算表面法向量(无需正则化)
	Vertex3 normal = crossPruduct(e1, e2);
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
	float denom = v1 * u2 - v2 * u1;
	if (denom == 0.0f)
		// 退化三角形--面积为0
		return false;
	// 计算重心坐标
	BarycentricCoords.x = Round((v4*u2 - v2 * u4) / denom, 3);
	BarycentricCoords.y = Round((v1*u3 - v3 * u1) / denom, 3);
	BarycentricCoords.z = Round(1.0f - BarycentricCoords.x - BarycentricCoords.y, 3);
	return true;
}

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

bool printIntersection(Cone const& cone, Ray const& ray, char* buf, FILE *p_file, int& Sum)
{
	bool flag = false;
	// 均匀切分圆心角，写入底面圆周坐标
	// 圆心角，步进
	float angle = 0, step = 2 * PI / cone.planeSum;
	Vertex3 p1, p2;	// 底面圆周上的点
	p1 = p2 = Vertex3(0, 0, 0);
	int cemian = 0, dimian = 0;		// 侧面、底面交点个数
	for (size_t i = 1; i <= cone.planeSum; i++)
		//while (angle >= 0 && angle < 2 * PI)
	{
		p1.x = Round(cone.radius * cos(angle));
		p1.z = Round(cone.radius * sin(angle));
		angle += step;
		// 最后一次计算p2取angle=0位置
//		p2.x = Round((i == cone.planeSum) ? cone.radius * cos(0) : cone.radius * cos(angle));
//		p2.z = Round((i == cone.planeSum) ? cone.radius * sin(0) : cone.radius * sin(angle));
		p2.x = Round(cone.radius * cos(angle));
		p2.z = Round(cone.radius * sin(angle));

		Vertex3 intersection;
		if (rayTriangleIntersect(ray, p1, p2, cone.tip, intersection)) {
			// 计算侧面可能存在的交点
			flag = true;
			printf("(%.6lf,%.6lf,%.6lf)\n", intersection.x, intersection.y, intersection.z);
			// 标记交点
			sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', intersection.x, intersection.y, intersection.z);
			fwrite(buf, sizeof(char), strlen(buf), p_file);
			sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', intersection.x + 0.1, intersection.y - 0.1, intersection.z + 0.1);
			fwrite(buf, sizeof(char), strlen(buf), p_file);
			sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', intersection.x - 0.1, intersection.y - 0.1, intersection.z + 0.1);
			fwrite(buf, sizeof(char), strlen(buf), p_file);
			cemian++;
		}
		if (rayTriangleIntersect(ray, p1, p2, cone.center, intersection)) {
			// 计算底面可能存在的交点
			flag = true;
			printf("(%.6lf,%.6lf,%.6lf)\n", intersection.x, intersection.y, intersection.z);
			// 标记交点
			sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', intersection.x, intersection.y, intersection.z);
			fwrite(buf, sizeof(char), strlen(buf), p_file);
			sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', intersection.x + 0.1, intersection.y - 0.1, intersection.z + 0.1);
			fwrite(buf, sizeof(char), strlen(buf), p_file);
			sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', intersection.x - 0.1, intersection.y - 0.1, intersection.z + 0.1);
			fwrite(buf, sizeof(char), strlen(buf), p_file);
			dimian++;
		}
	}
	printf("侧面交点：%d, 底面交点：%d\n", cemian, dimian);
	Sum = cemian + dimian;
	return flag;
}

int main()
{
	FILE *p_file = fopen("C:/Users/Administrator/Desktop/rayConeIntersect.obj", "w+");

	if (p_file) {
		Cone cone(planeSum, radius, height);
		Ray ray(Org, Delta);

		// 1.# Draw a cone.
		char buf[100] = { 0 };
		strcpy(buf, "# Draw a cone.\ng default\n");
		fwrite(buf, sizeof(char), strlen(buf), p_file);

		// 2.写入几何体顶点
		cone.printVertices(buf, p_file);

		// 写入射线始末点
		sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', Org.x, Org.y, Org.z);
		fwrite(buf, sizeof(char), strlen(buf), p_file);
		sprintf(buf, "%c %.6lf %.6lf %.6lf\n", 'v', Org.x + Delta.x, Org.y + Delta.y, Org.z + Delta.z);
		fwrite(buf, sizeof(char), strlen(buf), p_file);

		// 7.判断射线与圆锥是否有交点，有输出控制台、写入obj文件,返回交点总数
		int sum = 0;	// 交点总数
		if (!printIntersection(cone, ray, buf, p_file, sum))
			printf("无交点\n");

		// 3.写入贴图坐标
		sprintf(buf, "%s %.6lf %.6lf\n", "vt", 0.0, 0.0);//(0.0,0.0)
		fwrite(buf, sizeof(char), strlen(buf), p_file);

		// 4.写入顶点法线
		cone.printNormals(buf, p_file);

		// 5.材质、分组等配置
		strcpy(buf, "s off\nusemtl initialShadingGroup\ng pCone1\n");
		fwrite(buf, sizeof(char), strlen(buf), p_file);

		// 6.画锥体表面
		cone.DrawFace(buf, p_file);

		// 画射线
		sprintf(buf, "%c %d/%d/%d %d/%d/%d %d/%d/%d\n", 'f', planeSum + 3, 1, planeSum + 1, planeSum + 4, 1, planeSum + 1, planeSum + 3, 1, planeSum + 1);
		fwrite(buf, sizeof(char), strlen(buf), p_file);

		// 标记交点
		for (int index = planeSum + 5; index <= planeSum + 3 * sum + 2;) {
			sprintf(buf, "%c %d/%d/%d %d/%d/%d %d/%d/%d\n", 'f', index, 1, planeSum + 1, index + 1, 1, planeSum + 1, index + 2, 1, planeSum + 1);
			fwrite(buf, sizeof(char), strlen(buf), p_file);
			index += 3;
		}
		fclose(p_file);
		p_file = NULL;
	}

	std::cout << "Hello World!\n";
	return 0;
}

