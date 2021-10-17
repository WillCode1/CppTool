#pragma once
/*
	模式定义
		状态模式（state），当一个对象的内在状态改变时允许改变其行为，这个对象看起来像是改变了其类。
	模式动机
		状态模式主要解决的是当控制一个对象状态转换的条件表达式过于复杂时的情况。把状态的判断逻辑转移到表示不同状态的一系列类当中，可以把复杂的判断逻辑简化。
		当一个对象的行为取决于它的状态，并且它必须在运行时根据状态改变它的行为时，就可以考虑使用状态模式了。
	https://www.cnblogs.com/wzxNote/p/13321934.html
 */
#include <string>
#include <iostream>
#include <memory>
using namespace std;
class People;

class State
{
public:
	State(const std::string& context) :m_Content(context) {}
	virtual ~State() {}
	virtual void Handle(People* people) = 0;

	const std::string& Content() const
	{
		return m_Content;
	}
protected:
	std::string m_Content;

private:
	int     m_Time;
};

class People
{
public:
	People(std::shared_ptr<State> state) :m_State(state) {}
	void ChangeState(std::shared_ptr<State> state)
	{
		m_State = state;
	}

	void Show()
	{
		std::cout << "来吧，展示!" << std::endl;
		std::cout << "当前状态:" << m_State->Content() << std::endl;
	}

	void SetTime(int time)
	{
		m_Now = time;
		m_State->Handle(this);
	}

	int Time()
	{
		return m_Now;
	}

private:
	int							m_Now;
	std::shared_ptr<State>		m_State;
};

class LifeState : public State
{
public:
	LifeState(const std::string& content = "生活状态") :State(content) {}
	void Handle(People *people) override;
};

class PlayState : public State
{
public:
	PlayState(const std::string& content = "玩乐状态") :State(content) {}
	void Handle(People *people) override;
};

class WorkState : public State
{
public:
	WorkState(const std::string& content = "工作状态") :State(content) {}
	void Handle(People *people) override;
};

int testState();

