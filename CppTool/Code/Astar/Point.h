#pragma once
/*
	格点数据类
*/
struct Point
{
	int x, y;		//点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
	int F, G, H;	//F=G+H
	std::shared_ptr<Point> parent;	//parent的坐标

	Point() = default;
	Point(int _x, int _y)
		: x(_x), y(_y), F(0), G(0), H(0), parent(0) {}
};
