// rayConeIntersect.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>
#include <math.h>
#include <string.h>

const int planeSum = 90;			// 圆锥侧面总数，也就是底面圆周上顶点总数
const double PI = 3.14159265358979323846;
const float height = 10;			// 圆锥高
const float radius = 5;			// 半径

class vector
{
public:
	vector();
	vector(double vx, double vy, double vz);
	~vector();

public:
	double x, y, z;
};

vector::vector(double vx, double vy, double vz) : x(vx), y(vy), z(vz) {}

vector::vector()
{
}

vector::~vector()
{
}

class Cone {
public:
	Cone() {}
	Cone(int _planeSum, double _radius, double _height, vector _center = vector(0, 0, 0)) :
		planeSum(_planeSum), radius(_radius), height(_height), center(_center), tip(vector(_center.x, _center.y + height, _center.z)) {}
	~Cone() {}

	// 写入圆锥顶点
	void printVertices(char* buf, FILE *p_file)
	{
		// 均匀切分圆心角，写入底面圆周坐标
		// 圆心角，步进
		double angle = 0, step = 2 * PI / planeSum;
		vector point(0, 0, 0);	// 底面圆周上的点

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
		vector v1, v2, normal;
		double angle = 0, step = 2 * PI / planeSum;
		for (size_t i = 1; i <= planeSum; i++)
			//while (angle >= 0 && angle < 2 * PI)
		{
			v1 = vector(radius * cos(angle), -tip.y, radius * sin(angle));
			v2 = vector(radius * cos(angle + step), -tip.y, radius * sin(angle + step));

			// v1 x v2 可获得顶点法向量normal
			normal.x = v1.y*v2.z - v2.y*v1.z;
			normal.y = v1.z*v2.x - v2.z*v1.x;
			normal.z = v1.x*v2.y - v2.x*v1.y;

			// 向量标准化
			double modulus = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
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
	double radius;
	double height;
	vector center;
	vector tip;
};

int main()
{
	FILE *p_file = fopen("C:/Users/Administrator/Desktop/cone.obj", "w+");

	if (p_file) {
		Cone cone(planeSum, radius, height);

		// 1.# Draw a cone.
		char buf[100] = { 0 };
		strcpy(buf, "# Draw a cone.\ng default\n");
		fwrite(buf, sizeof(char), strlen(buf), p_file);

		// 2.写入几何体顶点
		cone.printVertices(buf, p_file);

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

		fclose(p_file);
		p_file = NULL;
	}

	std::cout << "Hello World!\n";
	return 0;
}

