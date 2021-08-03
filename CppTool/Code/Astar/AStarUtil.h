#pragma once
/*
	A*算法工具类
*/
#include <vector>
#include <list>
#include <memory>
#include "MapData.h"
#include "Point.h"

const int kCost1 = 10; //直移一格消耗
const int kCost2 = 14; //斜移一格消耗

class Astar
{
public:
	static Astar& getInstance(void);
	void release(void);
	void InitAstar(std::vector<std::vector<int>> const& _maze) { maze = _maze; }
	std::list<std::shared_ptr<Point>> const& GetPath(Point const& start, Point const& end, std::vector<std::vector<int>>& map, bool isIgnoreCorner);

private:
	std::shared_ptr<Point> findPath(Point const& start, Point const& end, bool isIgnoreCorner);
	std::vector<std::shared_ptr<Point>> getSurroundPoints(const std::shared_ptr<Point> point, bool isIgnoreCorner);
	bool isCanReach(const std::shared_ptr<Point> point, const std::shared_ptr<Point> target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
	std::shared_ptr<Point> isInList(const std::list<std::shared_ptr<Point>> &list, const Point* point) const; //判断开启/关闭列表中是否包含某点
	std::shared_ptr<Point> getLeastFpoint() const; //从开启列表中返回F值最小的节点
	void MarkPath(std::vector<std::vector<int>>& _maze) const;
	
	//计算FGH值
	int calcG(const Point* curPoint, const Point* point) const
	{
		int extraG = (abs(point->x - curPoint->x) + abs(point->y - curPoint->y)) == 1 ? kCost1 : kCost2;
		int currentG = curPoint == NULL ? 0 : curPoint->G; //如果是初始节点，则其父节点是空  
		return currentG + extraG;
	}
	int calcH(const Point* point, const Point* end) const
	{
		//用简单的欧几里得距离计算H
		return sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y))*kCost1;
	}
	int calcF(const Point* point) const { return point->G + point->H; }
	
	//数据清理
	void Clear(bool isDown);

private:
	Astar() = default;
	//~Astar() = default;
	Astar(const Astar&);
	static Astar* s_instance;
	static int s_counter;

private:
	std::vector<std::vector<int>> maze;				//地图数据
	std::list<std::shared_ptr<Point>> openList;		//开启列表
	std::list<std::shared_ptr<Point>> closeList;	//关闭列表
	std::list<std::shared_ptr<Point>> path;			//返回路径
};