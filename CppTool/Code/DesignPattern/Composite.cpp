/*
	组合模式：将对象组合成树形结构以表示“部分-整体”的层次结构，组合模式使得客户端对单个对象和组合对象的使用具有一直性。
	既然讲到以树形结构表示“部分-整体”，那可以将组合模式想象成一根大树，将大树分成树枝和树叶两部分，树枝上可以再长树枝，也可以长树叶，树叶上则不能再长出别的东西。
	以下情况可以考虑使用组合模式：
		希望表示对象的部分-整体层次结构。
		希望客户端忽略组合对象与单个对象的不同，客户端将统一的使用组合结构中的所有对象。
 */

 /*
 * 关键代码：树枝内部组合该接口，并且含有内部属性list，里面放Component。
 */

#include <iostream>
#include <list>
#include <memory>

using namespace std;

//抽象类，提供组合和单个对象的一致接口
class Company
{
public:
	Company(const string& name) : m_name(name) {}
	virtual ~Company() { cout << "~Company()" << endl; }

	virtual void add(Company*) = 0;
	virtual void remove(const string&) = 0;
	virtual void display(int depth) = 0;

	virtual const string& name()
	{
		return m_name;
	}

protected:
	string m_name;
};

//具体的单个对象实现类，“树枝”类
class HeadCompany : public Company
{
public:
	HeadCompany(const string& name) : Company(name) {}
	virtual ~HeadCompany() { cout << "~HeadCompany()" << endl; }

	void add(Company* company) override
	{
		shared_ptr<Company> temp(company);
		m_companyList.push_back(temp);
	}

	void remove(const string& strName) override
	{
		list<shared_ptr<Company>>::iterator iter = m_companyList.begin();
		for (; iter != m_companyList.end(); iter++)
		{
			if ((*iter).get()->name() == strName)
			{
				//不应该在此处使用list<T>.erase(list<T>::iterator iter),会导致iter++错误，这里删除目标元素之后，必须return。
				m_companyList.erase(iter);
				return;
			}
		}
	}

	void display(int depth) override
	{
		for (int i = 0; i < depth; i++)
		{
			cout << "-";
		}
		cout << this->name().data() << endl;
		list<shared_ptr<Company>>::iterator iter = m_companyList.begin();
		for (; iter != m_companyList.end(); iter++)
		{
			(*iter).get()->display(depth + 1);
		}
	}

private:
	list<shared_ptr<Company>> m_companyList;
};

//具体的单个对象实现类，“树叶”类
class ResearchCompany : public Company
{
public:
	ResearchCompany(const string& name) : Company(name) {}
	virtual ~ResearchCompany() { cout << "~ResearchCompany()" << endl; }

	void add(Company*) override
	{
	}

	void remove(const string&) override
	{
	}

	void display(int depth) override
	{
		for (int i = 0; i < depth; i++)
		{
			cout << "-";
		}
		cout << m_name.data() << endl;
	}
};

//具体的单个对象实现类，“树叶”类
class SalesCompany : public Company
{
public:
	SalesCompany(const string& name) : Company(name) {}
	virtual ~SalesCompany() { cout << "~SalesCompany()" << endl; }

	void add(Company*) override
	{
	}

	void remove(const string&) override
	{
	}

	void display(int depth) override
	{
		for (int i = 0; i < depth; i++)
		{
			cout << "-";
		}
		cout << m_name.data() << endl;
	}
};

//具体的单个对象实现类，“树叶”类
class FinanceCompany : public Company
{
public:
	FinanceCompany(const string& name) : Company(name) {}
	virtual ~FinanceCompany() { cout << "~FinanceCompany()" << endl; }

	void add(Company*) override
	{
	}

	void remove(const string&) override
	{
	}

	void display(int depth) override
	{
		for (int i = 0; i < depth; i++)
		{
			cout << "-";
		}
		cout << m_name.data() << endl;
	}
};


int testComposite()
{
	HeadCompany* headRoot = new HeadCompany("Head Root Company");

	HeadCompany* childRoot1 = new HeadCompany("Child Company A");
	ResearchCompany* r1 = new ResearchCompany("Research Company A");
	SalesCompany* s1 = new SalesCompany("Sales Company A");
	SalesCompany* s2 = new SalesCompany("Sales Company B");
	FinanceCompany* f1 = new FinanceCompany("FinanceCompany A");

	childRoot1->add(r1);
	childRoot1->add(s1);
	childRoot1->add(s2);
	childRoot1->add(f1);

	HeadCompany* childRoot2 = new HeadCompany("Child Company B");
	ResearchCompany* r2 = new ResearchCompany("Research Company B");
	SalesCompany* s3 = new SalesCompany("Sales Company C");
	SalesCompany* s4 = new SalesCompany("Sales Company D");
	FinanceCompany* f2 = new FinanceCompany("FinanceCompany B");

	childRoot2->add(r2);
	childRoot2->add(s3);
	childRoot2->add(s4);
	childRoot2->add(f2);

	headRoot->add(childRoot1);
	headRoot->add(childRoot2);
	headRoot->display(1);

	cout << "\n***************\n" << endl;

	childRoot1->remove("Sales Company B");
	headRoot->display(1);

	cout << "\n***************\n" << endl;

	delete headRoot;
	headRoot = nullptr;

	return 0;
}
