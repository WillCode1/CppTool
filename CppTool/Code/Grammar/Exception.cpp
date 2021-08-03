#include "Exception.h"
#include <iostream>
using namespace std;
using namespace Exception;


// https://blog.csdn.net/chenlycly/article/details/45013817
/*
	1、对throw的说明
	C++函数后面后加关键字throw(something)限制，是对这个函数的异常安全性作出限制。
	举例及解释如下：
	void fun() throw() 表示fun不允许抛出任何异常，即fun是异常安全的。
	void fun() throw(...) 表示fun可以抛出任何形式的异常。
	void fun() throw(exceptionType) 表示fun只能抛出exceptionType类型的异常。
	比如：
	void GetTag() throw(int); 表示只抛出int类型异常
	void GetTag() throw(int，char); 表示抛出in，char类型异常
	void GetTag() throw(); 表示不会抛出任何类型异常
	void GetTag() throw(...); 表示抛出任何类型异常
	那么，void GetTag() throw(int); 表示只抛出int类型异常
	这句解释怎么理解呢？并不表示一定会抛出异常，但是一旦抛出异常只会抛出int类型。如果抛出非int类型异常，调用unexsetpion()函数，退出程序。
 */
double Devide(double x, double y) throw(ExceptionOperator)
{
	// 如果一个函数在执行过程中拋出的异常在本函数内就被 catch 块捕获并处理，那么该异常就不会拋给这个函数的调用者（也称为“上一层的函数”）；
	// 如果异常在本函数中没有被处理，则它就会被拋给上一层的函数。
	if (y == 0)
		throw ExceptionOperator("devided by zero");

	cout << "in Devide" << endl;
	return x / y;
}

int CountTax(int salary) throw(int)
{
	try {
		if (salary < 0)
			throw - 1;
		cout << "counting tax" << endl;
	}
	catch (int) {
		cout << "salary < 0" << endl;
	}
	cout << "tax counted" << endl;
	return salary * 0.15;
}

int testException()
{
	double f = 1.2;
	try {
		CountTax(-1);
		f = Devide(3, 0);
		cout << "end of try block" << endl;
	}
	catch (const ExceptionOperator& e) {
		cout << e.msg << endl;
	}
	cout << "f = " << f << endl;
	cout << "finished" << endl;
	return 0;
}
