/*
	原型模式：用原型实例指定创建对象的种类，并且通过拷贝这些原型创建新的对象。
		通俗的讲就是当需要创建一个新的实例化对象时，我们刚好有一个实例化对象，但是已经存在的实例化对象又不能直接使用。这种情况下拷贝一个现有的实例化对象来用，可能会更方便。
	
	以下情形可以考虑使用原型模式：
		当new一个对象，非常繁琐复杂时，可以使用原型模式来进行复制一个对象。比如创建对象时，构造函数的参数很多，而自己又不完全的知道每个参数的意义，
		就可以使用原型模式来创建一个新的对象，不必去理会创建的过程。

		当需要new一个新的对象，这个对象和现有的对象区别不大，我们就可以直接复制一个已有的对象，然后稍加修改。
		当需要一个对象副本时，比如需要提供对象的数据，同时又需要避免外部对数据对象进行修改，那就拷贝一个对象副本供外部使用。
 */


 /*
 * 关键代码：拷贝，return new className(*this);
 */
#include <iostream>

using namespace std;

//提供一个抽象克隆基类。
class Clone
{
public:
	virtual Clone* clone() = 0;
	virtual void show() = 0;
};

//具体的实现类
class Sheep :public Clone
{
public:
	Sheep(int id, string name) :Clone(),
		m_id(id), m_name(name)
	{
		cout << "Sheep() id address:" << &m_id << endl;
		cout << "Sheep() name address:" << &m_name << endl;
	}
	~Sheep()
	{
	}
	//关键代码拷贝构造函数
	Sheep(const Sheep& obj)
	{
		this->m_id = obj.m_id;
		this->m_name = obj.m_name;
		cout << "Sheep(const Sheep& obj) id address:" << &m_id << endl;
		cout << "Sheep(const Sheep& obj) name address:" << &m_name << endl;
	}
	//关键代码克隆函数，返回return new Sheep(*this)
	Clone* clone()
	{
		return new Sheep(*this);
	}
	void show()
	{
		cout << "id  :" << m_id << endl;
		cout << "name:" << m_name.data() << endl;
	}

private:
	int m_id;
	string m_name;
};

int testPrototype()
{
	Clone* s1 = new Sheep(1, "abs");
	s1->show();
	Clone* s2 = s1->clone();
	s2->show();

	delete s1;
	s1 = nullptr;
	delete s2;
	s2 = nullptr;
	return 0;
}
