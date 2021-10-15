/*
	桥接模式：将抽象部分与实现部分分离，使它们都可以独立变换。
	以下情形考虑使用桥接模式：
		当一个对象有多个变化因素的时候，考虑依赖于抽象的实现，而不是具体的实现。
		当多个变化因素在多个对象间共享时，考虑将这部分变化的部分抽象出来再聚合/合成进来。
		当一个对象的多个变化因素可以动态变化的时候。

	优点：
		将实现抽离出来，再实现抽象，使得对象的具体实现依赖于抽象，满足了依赖倒转原则。
		更好的可扩展性。
		可动态的切换实现。桥接模式实现了抽象和实现的分离，在实现桥接模式时，就可以实现动态的选择具体的实现。
 */

 /*
 * 关键代码：将现实独立出来，抽象类依赖现实类。
 * 以下示例中，将各类App、各类手机独立开来，实现各种App和各种手机的自由桥接。
 */
#include <iostream>
using namespace std;

//抽象App类，提供接口
class App
{
public:
	virtual ~App() { cout << "~App()" << endl; }
	virtual void run() = 0;
};

//具体的App实现类
class GameApp :public App
{
public:
	void run()
	{
		cout << "GameApp Running" << endl;
	}
};

//具体的App实现类
class TranslateApp :public App
{
public:
	void run()
	{
		cout << "TranslateApp Running" << endl;
	}
};

//抽象手机类，提供接口
class MobilePhone
{
public:
	virtual ~MobilePhone() { cout << "~MobilePhone()" << endl; }
	virtual void appRun(App* app) = 0;  //实现App与手机的桥接
};

//具体的手机实现类
class XiaoMi :public MobilePhone
{
public:
	void appRun(App* app)
	{
		cout << "XiaoMi: ";
		app->run();
	}
};

//具体的手机实现类
class HuaWei :public MobilePhone
{
public:
	void appRun(App* app)
	{
		cout << "HuaWei: ";
		app->run();
	}
};

int testBridge()
{
	App* gameApp = new GameApp;
	App* translateApp = new TranslateApp;
	MobilePhone* mi = new XiaoMi;
	MobilePhone* hua = new HuaWei;
	mi->appRun(gameApp);
	mi->appRun(translateApp);
	hua->appRun(gameApp);
	hua->appRun(translateApp);

	delete hua;
	hua = nullptr;
	delete mi;
	mi = nullptr;
	delete gameApp;
	gameApp = nullptr;
	delete translateApp;
	translateApp = nullptr;

	return 0;
}
