#include "AStarUtil.h"


Astar* Astar::s_instance = NULL;
int Astar::s_counter = 0;


Astar& Astar::getInstance(void)
{
	if (!s_instance)
		s_instance = new Astar();
	++s_counter;
	return *s_instance;
}
void Astar::release(void)
{
	if (s_counter && --s_counter == 0) {
		delete this;
		s_instance = NULL;
	}
}

void Astar::Clear(bool isDown)
{
	//仅保留地图数据
	if (isDown) {
		openList.clear();
		closeList.clear();
	}
	else {
		path.clear();
	}
}

std::shared_ptr<Point> Astar::getLeastFpoint() const
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();
		for (auto &point : openList)
			if (point->F < resPoint->F)
				resPoint = point;
		return resPoint;
	}
	return NULL;
}

void Astar::MarkPath(std::vector<std::vector<int>>& _maze) const
{
	for (auto &p : path)
		_maze[p->x][p->y] = 2;
}

std::shared_ptr<Point> Astar::findPath(Point const& start, Point const& end, bool isIgnoreCorner)
{
	openList.emplace_back(new Point(start.x, start.y));//置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint();	//找到F值最小的点  
		openList.remove(curPoint);			//从开启列表中删除
		closeList.push_back(curPoint);		//放到关闭列表
		//1,找到当前周围八个格中可以通过的格子  
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H  
			if (!isInList(openList, target.get()))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint.get(), target.get());
				target->H = calcH(target.get(), &end);
				target->F = calcF(target.get());

				openList.push_back(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F  
			else
			{
				int tempG = calcG(curPoint.get(), target.get());
				//从开启列表中找到对应的格子
				for (auto &point : openList)
				{
					if ((target->x == point->x) && (target->y == point->y))
					{
						target = point;
						break;
					}
				}
				if (tempG < target->G)
				{
					target->parent = curPoint;
					target->G = tempG;
					target->F = calcF(target.get());
				}
			}
			std::shared_ptr<Point> resPoint = isInList(openList, &end);
			if (resPoint)
				return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
		}
	}
	return NULL;
}

std::list<std::shared_ptr<Point>> const& Astar::GetPath(Point const& start, Point const& end, std::vector<std::vector<int>>& map, bool isIgnoreCorner)
{
	Clear(false);
	std::shared_ptr<Point> result = findPath(start, end, isIgnoreCorner);
	//返回路径，如果没找到路径，返回空链表  
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}
	MarkPath(map);
	Clear(true);
	return path;
}

std::shared_ptr<Point> Astar::isInList(const std::list<std::shared_ptr<Point>> &list, const Point* point) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标  
	for (auto p : list)
		if (p->x == point->x&&p->y == point->y)
			return p;
	return NULL;
}

bool Astar::isCanReach(const std::shared_ptr<Point> point, const std::shared_ptr<Point> target, bool isIgnoreCorner) const
{
	if (target->x<0 || target->x>maze.size() - 1
		|| target->y<0 || target->y>maze[0].size() - 1
		|| maze[target->x][target->y] == 1
		|| target->x == point->x&&target->y == point->y
		|| isInList(closeList, target.get())) //如果点与当前点重合、超出地图、是障碍物、或者在关闭列表中，返回false  
		return false;

	if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以  
		return true;
	else
	{
		//斜对角要判断是否绊住  
		if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
			return true;
		else
			return isIgnoreCorner;
	}
}

std::vector<std::shared_ptr<Point>> Astar::getSurroundPoints(const std::shared_ptr<Point> point, bool isIgnoreCorner)
{
	std::vector<std::shared_ptr<Point>> surroundList;

	for (int x = point->x - 1; x <= point->x + 1; x++)
		for (int y = point->y - 1; y <= point->y + 1; y++) {
			std::shared_ptr<Point> surroundPoint = std::make_shared<Point>(x, y);
			if (isCanReach(point, surroundPoint, isIgnoreCorner))
				surroundList.push_back(surroundPoint);
		}
	return surroundList;
}