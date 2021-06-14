#pragma once
/*
 * 关键代码：在目标类中增加一个ArrayList来存放观察者们。

   观察者模式：定义对象间的一种一对多的依赖关系，当一个对象的状态发生改变时，所有依赖于它的对象都要得到通知并自动更新。
   观察者模式从根本上讲必须包含两个角色：观察者和被观察对象。
   被观察对象自身应该包含一个容器来存放观察者对象，当被观察者自身发生改变时通知容器内所有的观察者对象自动更新。
   观察者对象可以注册到被观察者的中，完成注册后可以检测被观察者的变化，接收被观察者的通知。
   当然观察者也可以被注销掉，停止对被观察者的监控。
 */
#include <iostream>
#include <list>
#include <memory>
#include <string>
using namespace std;

class View;

//被观察者抽象类   数据模型
class DataModel
{
public:
	virtual ~DataModel() {}
	virtual void addView(View* view) = 0;
	virtual void removeView(View* view) = 0;
	virtual void notify() const = 0;
};

//观察者抽象类   视图
class View
{
public:
	virtual ~View() { cout << "~View()" << endl; }
	virtual void update() const = 0;
	virtual void setViewName(const string& name) = 0;
	virtual const string& name() const = 0;
};

//具体的被观察类， 整数模型
class IntDataModel :public DataModel
{
public:
	~IntDataModel() override
	{
		m_pViewList.clear();
	}

	void addView(View* view) override
	{
		//auto temp = make_shared<View>(view);
		shared_ptr<View> temp(view);
		if (find(m_pViewList.begin(), m_pViewList.end(), temp) == m_pViewList.end())
		{
			m_pViewList.push_front(temp);
		}
		else
		{
			cout << "View already exists" << endl;
		}
	}

	void removeView(View* view) override
	{
		for (auto iter = m_pViewList.begin(); iter != m_pViewList.end(); iter++)
		{
			if ((*iter).get() == view)
			{
				cout << "remove view" << (*iter)->name() << endl;
				m_pViewList.erase(iter);
				return;
			}
		}
	}

	void notify() const override
	{
		for (auto& view : m_pViewList)
		{
			view->update();
		}
	}

private:
	list<shared_ptr<View>> m_pViewList;
};

//具体的观察者类    表视图
class TableView : public View
{
public:
	TableView() : m_name("unknow") {}
	TableView(const string& name) : m_name(name) {}
	~TableView() { cout << "~TableView(): " << m_name.data() << endl; }

	void setViewName(const string& name) override
	{
		m_name = name;
	}

	const string& name() const override
	{
		return m_name;
	}

	void update() const override
	{
		cout << m_name.data() << " update" << endl;
	}

private:
	string m_name;
};

int testObserver();
