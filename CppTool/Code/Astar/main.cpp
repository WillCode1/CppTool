//随机初始化地图，寻路算法实现
#include <iostream>
#include "AStarUtil.h"
#include "MapData.h"

#if 0
#define	CUSTOM	//自定义地图
#else
#define	RANDOM	//随机地图
#endif // 0

#ifdef RANDOM
const int Max = 20;
#endif // RANDOM

int main()
{	
	using namespace std;
	//设置起始和结束点
	Point start(1, 1);
#ifdef RANDOM
	Point end(Max - 2, Max - 2);

	//随机初始化地图
	MapData maze(start, end, Max);
#elif defined(CUSTOM)
	Point end(6, 10);	//自定义地图终点

	MapData maze(start, end);
	//自定义初始化地图，用二维数组代表地图，1表示障碍物，0表示可通
	maze.GetMap() = {
		{ 1,1,1,1,1,1,1,1,1,1,1,1 },
		{ 1,0,0,1,1,0,1,0,0,0,0,1 },
		{ 1,0,0,1,1,0,0,0,0,0,0,1 },
		{ 1,0,0,0,0,0,1,0,0,1,1,1 },
		{ 1,1,1,0,0,0,0,0,1,1,0,1 },
		{ 1,1,0,1,0,0,0,0,0,0,0,1 },
		{ 1,0,1,0,0,0,0,1,0,0,0,1 },
		{ 1,1,1,1,1,1,1,1,1,1,1,1 }
	};
#endif // RANDOM/CUSTOM

	Astar& astar = Astar::getInstance();
	astar.InitAstar(maze.GetMap());

	//A*算法找寻路径，并标记在地图对象上
	list<std::shared_ptr<Point>> const& path = astar.GetPath(start, end, maze.GetMap(), false);

	//存在就打印路径
	if (!path.empty())
	{
/*		for (auto &p : path)
			cout << '(' << p->x << ',' << p->y << ')' << endl;*/
		cout << "最短路径：\n";
		maze.PrintMap();
	}else {
		cout << "找不到路径！\n";
		maze.PrintMap();
	}
	astar.release();

	return 0;
}