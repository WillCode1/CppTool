// DesignPattern.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#define INHERIT
#ifdef INHERIT
#include "AdaptorInherit .h"
#else
#include "AdaptorComb.h"
#endif // INHERIT
#include "Template.h"
#include "Observer.h"
//#include "SimpleFactory.h"
#include "FactoryMethod.h"
#include "BlockingQueue.h"


int main()
{
	//testTemplate();
	//testObserver();
	cout << "====================" << endl;
	//testSimpleFactory();
	testFactoryMethod();
	cout << "====================" << endl;
	testConsume();
}
