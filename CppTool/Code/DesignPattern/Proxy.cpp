/*
	代理模式：为其它对象提供一种代理以控制这个对象的访问。在某些情况下，一个对象不适合或者不能直接引用另一个对象，而代理对象可以在客户端和目标对象之间起到中介作用。
	优点：
		职责清晰。真实的角色只负责实现实际的业务逻辑，不用关心其它非本职责的事务，通过后期的代理完成具体的任务。这样代码会简洁清晰。
		代理对象可以在客户端和目标对象之间起到中介的作用，这样就保护了目标对象。
		扩展性好。
 */
 /*
 * 关键代码：一个是真正的你要访问的对象(目标类)，一个是代理对象,真正对象与代理对象实现同一个接口,先访问代理类再访问真正要访问的对象。
 */
#include <iostream>
using namespace std;

class Girl
{
public:
	Girl(const string& name = "girl") :m_string(name) {}
	string getName()
	{
		return m_string;
	}
private:
	string m_string;
};

class Profession
{
public:
	virtual ~Profession() {}
	virtual void profess() = 0;
};

class YoungMan : public Profession
{
public:
	YoungMan(const Girl& girl) :m_girl(girl) {}
	void profess()
	{
		cout << "Young man love " << m_girl.getName().data() << endl;
	}

private:
	Girl m_girl;
};

class ManProxy : public Profession
{
public:
	ManProxy(const Girl& girl) :m_pMan(new YoungMan(girl)) {}
	~ManProxy()
	{
		delete m_pMan;
		m_pMan = nullptr;
	}
	void profess()
	{
		m_pMan->profess();
	}
private:
	YoungMan* m_pMan;
};

int testProxy(int argc, char *argv[])
{
	Girl girl("heihei");
	ManProxy* proxy = new ManProxy(girl);
	proxy->profess();

	delete proxy;
	proxy = nullptr;
	return 0;
}
