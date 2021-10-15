/*
	备忘录模式：在不破坏封装性的前提下，捕获一个对象的内部状态，并在该对象之外保存这个状态。这样以后就可以将该对象恢复到原来保存的状态。
	备忘录模式中需要定义的角色类：
		1.Originator(发起人)：负责创建一个备忘录Memento，用以记录当前时刻自身的内部状态，并可使用备忘录恢复内部状态。Originator可以根据需要决定Memento存储自己的哪些内部状态。
		2.Memento(备忘录)：负责存储Originator对象的内部状态，并可以防止Originator以外的其他对象访问备忘录。
		  备忘录有两个接口：Caretaker只能看到备忘录的窄接口，他只能将备忘录传递给其他对象。Originator却可看到备忘录的宽接口，允许它访问返回到先前状态所需要的所有数据。
		3.Caretaker(管理者):负责备忘录Memento，不能对Memento的内容进行访问或者操作。
 */

 /*
 * 关键代码：Memento类、Originator类、Caretaker类；Originator类不与Memento类耦合，而是与Caretaker类耦合。
 */
#include <iostream>
#include <unordered_map>
using namespace std;

//需要保存的信息
typedef struct
{
	int grade;
	string arm;
	string corps;
}GameValue;

//Memento类
class Memento
{
public:
	Memento() {}
	Memento(const GameValue& value) :m_gameValue(value) {}
	const GameValue& getValue() const
	{
		return m_gameValue;
	}
private:
	GameValue m_gameValue;
};

//Originator类
class Game
{
public:
	Game(const GameValue& value) :m_gameValue(value)
	{}
	void addGrade()  //等级增加
	{
		m_gameValue.grade++;
	}
	void replaceArm(const string& arm)  //更换武器
	{
		m_gameValue.arm = arm;
	}
	void replaceCorps(const string& corps)  //更换工会
	{
		m_gameValue.corps = corps;
	}
	Memento saveValue()    //保存当前信息
	{
		Memento memento(m_gameValue);
		return memento;
	}
	void load(const Memento& memento) //载入信息
	{
		m_gameValue = memento.getValue();
	}
	void showValue()
	{
		cout << "Grade: " << m_gameValue.grade << endl;
		cout << "Arm  : " << m_gameValue.arm.data() << endl;
		cout << "Corps: " << m_gameValue.corps.data() << endl;
	}
private:
	GameValue m_gameValue;
};

//Caretaker类
class Caretake
{
public:
	void save(const unsigned int& index, const Memento& memento)  //保存信息
	{
		m_memento[index] = memento;
	}
	const Memento& load(const unsigned int& index)            //读已保存的信息
	{
		return m_memento[index];
	}
private:
	std::unordered_map<unsigned int, Memento> m_memento;
};

int testMemento()
{
	GameValue v1 = { 0, "Ak", "3K" };
	Game game(v1);    //初始值
	game.addGrade();
	game.showValue();
	cout << "----------" << endl;
	Caretake care;
	care.save(1, game.saveValue());  //保存当前值
	game.addGrade();          //修改当前值
	game.replaceArm("M16");
	game.replaceCorps("123");
	game.showValue();
	cout << "----------" << endl;
	game.load(care.load(1));   //恢复初始值
	game.showValue();
	return 0;
}
