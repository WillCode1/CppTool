/*
	模式定义
	访问者模式（Visitor），表示一个作用于某对象结构中的各个元素的操作。它使你可以在不改变各元素的类的前提下定义作用于这些元素的新操作。
	模式动机
	    访问者模式适用于数据结构相对稳定的系统
		它把数据结构和作用于数据结构之上的操作之间的耦合解脱开，使得操作集合可以相对自由的演化。
		访问者模式的目的是要把处理从数据结构分离出来。如果这样的系统有比较稳定的数据结构，又有已与变化的算法的话，使用访问者模式就是比较合适的，因为访问者模式使得算法操作的增加变得更容易。反之亦然。
	https://www.cnblogs.com/wzxNote/p/13278443.html
 */

#include <iostream>
#include <vector>
#include <list>
#include <string>

class Action;
class Person
{
public:
	Person(const std::string& type) : m_Type(type) {}
	virtual void Accept(Action* action) = 0;

	std::string Type()
	{
		return m_Type;
	}
protected:
	std::string m_Type;
};

class Man : public Person
{
public:
	Man() : Person("男人") {}
	virtual void Accept(Action *action);
};

class Woman : public Person
{
public:
	Woman() : Person("女人") {}
	virtual void Accept(Action *action);
};

class Action
{
public:
	Action(const std::string& type) : m_Type(type) {}
	virtual ~Action() {}
	virtual void PerformanceMan(Man* man) {};
	virtual void PerformanceWoman(Woman* woman) {};

protected:
	std::string     m_Type;
};

class Success : public Action
{
public:
	Success() :Action("成功") {}

	void PerformanceMan(Man* man) override
	{
		std::cout << man->Type() << this->m_Type << "时候的表现" << std::endl;
	}

	void PerformanceWoman(Woman* woman) override
	{
		std::cout << woman->Type() << this->m_Type << "时候的表现" << std::endl;
	}
};


class ObjectStruct
{
public:
	void Display(Action* action)
	{
		for (auto person : m_Person)
		{
			person->Accept(action);
		}
	}

	void Attach(Person *person)
	{
		m_Person.push_back(person);
	}

	void Detach(Person *person)
	{
		m_Person.remove(person);
	}

	std::list<Person*>       m_Person;
};


int testVisitor();
