/*
	职责链模式：使多个对象都有机会处理请求，从而避免请求的发送者和接收者之前的耦合关系，将这些对象连成一条链，并沿着这条链传递请求，直到有一个对象处理它为止。
	职责链上的处理者负责处理请求，客户只需要将请求发送到职责链上即可，无需关心请求的处理细节和请求的传递，所有职责链将请求的发送者和请求的处理者解耦了。
 */

 /*
 * 关键代码：Handler内指明其上级，handleRequest()里判断是否合适，不合适则传递给上级。
 */
#include <iostream>
using namespace std;

enum RequestLevel
{
	Level_One = 0,
	Level_Two,
	Level_Three,
	Level_Num
};

//抽象处理者（Handler）角色，提供职责链的统一接口。
class Leader
{
public:
	Leader(Leader* leader) :m_leader(leader) {}
	virtual ~Leader() {}
	virtual void handleRequest(RequestLevel level) = 0;
protected:
	Leader* m_leader;
};

//具体处理者（Concrete Handler）角色
class Monitor :public Leader   //链扣1
{
public:
	Monitor(Leader* leader) :Leader(leader) {}
	void handleRequest(RequestLevel level)
	{
		if (level < Level_Two)
		{
			cout << "Mointor handle request : " << level << endl;
		}
		else
		{
			m_leader->handleRequest(level);
		}
	}
};

//具体处理者（Concrete Handler）角色
class Captain :public Leader    //链扣2
{
public:
	Captain(Leader* leader) :Leader(leader) {}
	void handleRequest(RequestLevel level)
	{
		if (level < Level_Three)
		{
			cout << "Captain handle request : " << level << endl;
		}
		else
		{
			m_leader->handleRequest(level);
		}
	}
};

//具体处理者（Concrete Handler）角色
class General :public Leader   //链扣3
{
public:
	General(Leader* leader) :Leader(leader) {}
	void handleRequest(RequestLevel level)
	{
		cout << "General handle request : " << level << endl;
	}
};

int testChainOfResponsibility()
{
	Leader* general = new General(nullptr);
	Leader* captain = new Captain(general);
	Leader* monitor = new Monitor(captain);
	monitor->handleRequest(Level_Two);

	delete monitor;
	monitor = nullptr;
	delete captain;
	captain = nullptr;
	delete general;
	general = nullptr;
	return 0;
}
