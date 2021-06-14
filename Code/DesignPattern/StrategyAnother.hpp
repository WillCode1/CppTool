#pragma once
//使用函数指针实现策略模式
#include <iostream>
#include <functional>

void adcHurt()
{
	std::cout << "Adc Hurt" << std::endl;
}

void apcHurt()
{
	std::cout << "Apc Hurt" << std::endl;
}

//环境角色类， 使用传统的函数指针
class Soldier
{
public:
	typedef void(*Function)();
	Soldier(Function fun) : m_fun(fun)
	{
	}
	void attack()
	{
		m_fun();
	}
private:
	Function m_fun;
};

//环境角色类， 使用std::function<>
class Mage
{
public:
	typedef std::function<void()> Function;

	Mage(Function fun) : m_fun(fun)
	{
	}
	void attack()
	{
		m_fun();
	}
private:
	Function m_fun;
};

int test()
{
	Soldier* soldier = new Soldier(apcHurt);
	soldier->attack();
	delete soldier;
	soldier = nullptr;
	return 0;
}
