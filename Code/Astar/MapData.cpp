#include <iostream>
#include <ctime>
#include "MapData.h"


MapData::MapData(Point const& start, Point const& end)
	: m_start(start), m_end(end) {}

MapData::MapData(Point const& start, Point const& end, int Max)
{
	m_start = start;
	m_end = end;
	RandomMap(Max);
}

void MapData::RandomMap(int mapSize)
{
	std::vector<std::vector<int>> tempMap;
	srand(time(NULL));
	for (size_t i = 0; i < mapSize; i++)
	{
		tempMap.push_back(std::vector<int>());
		for (size_t j = 0; j < mapSize; j++)
		{
			tempMap[i].push_back(0);
			if (i == m_start.x && j == m_start.y || i == m_end.x && j == m_end.y)
				continue;			//跳过起点、终点
			else if (!i || !j || i == mapSize - 1 || j == mapSize - 1)
				tempMap[i][j] = 1;	//边框
			else if (rand() % mapSize < mapSize / 3)
				tempMap[i][j] = 1;	//随机生成障碍
		}
	}
	map = tempMap;
}

void MapData::PrintMap() const
{
	for (size_t i = 0; i < map.size(); i++)
	{
		for (size_t j = 0; j < map[0].size(); j++)
		{
			if (i == m_start.x && j == m_start.y)
				std::cout << 'A';
			else if (i == m_end.x && j == m_end.y)
				std::cout << 'B';
			else if (map[i][j] == 1)
				std::cout << 'O';
			else if (map[i][j] == 0)
				std::cout << ' ';
			else
				std::cout << '*';
			std::cout << " ";
		}
		std::cout << std::endl;
	}
}
