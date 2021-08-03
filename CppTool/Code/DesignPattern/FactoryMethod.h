#pragma once
/*
	1.2、工厂方法模式
	定义一个创建对象的接口，其子类去具体现实这个接口以完成具体的创建工作。如果需要增加新的产品类，只需要扩展一个相应的工厂类即可。

	缺点：产品类数据较多时，需要实现大量的工厂类，这无疑增加了代码量。
*/
#include "Tank.h"
#include <iostream>
#include <string>
using namespace std;


//抽象工厂类，提供一个创建接口
class TankFactory
{
public:
	//提供创建产品实例的接口，返回抽象产品类
	virtual Tank* createTank() = 0;
};

//具体的创建工厂类，使用抽象工厂类提供的接口，去创建具体的产品实例
class Tank56Factory : public TankFactory
{
public:
	Tank* createTank() override
	{
		return new Tank56();
	}
};

//具体的创建工厂类，使用抽象工厂类提供的接口，去创建具体的产品实例
class Tank96Factory : public TankFactory
{
public:
	Tank* createTank() override
	{
		return new Tank96();
	}
};


int testFactoryMethod();
