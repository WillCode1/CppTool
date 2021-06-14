#pragma once
/*
	地图数据类
*/
#include <vector>
#include <list>
#include "Point.h"

class MapData
{
public:
	MapData() = default;
	~MapData() = default;
	MapData(Point const& start, Point const& end);
	MapData(Point const& start, Point const& end, int Max);

	std::vector<std::vector<int>>& GetMap() { return map; }	//提供修改和访问地图的接口
	void RandomMap(int mapSize);			//随机生成指定大小地图
	void PrintMap() const;					//打印地图
private:
	Point m_start;
	Point m_end;
	std::vector<std::vector<int>> map;
};
