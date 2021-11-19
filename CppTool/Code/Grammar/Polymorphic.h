#pragma once
#include <string>
#include <iostream>

class Father
{
public:
	virtual void test(std::string str = "father") { std::cout << str << " father test1!" << std::endl; }
	virtual void test2(short i) { std::cout << "father test2!" << std::endl; }

	std::string str = "father";
};

class Child: public Father
{
public:
	virtual void test(std::string str = "child") { std::cout << str << " child test1!" << std::endl; }
	virtual void test2(int i) { std::cout << "child test2!" << std::endl; }

	std::string str = "child";
};

void testPolymorphic()
{
	Father *f1 = new Child();
	f1->test();
	f1->test2(1);
	std::cout << f1->str << std::endl;

	Father *f2 = new Father();
	f2->test();
	f2->test2(1);
	std::cout << f2->str << std::endl;

	Child c1;
	c1.test();
	Father f3 = c1;
	f3.test();

	Child c2;
	Father& f4 = c2;
	f4.test();
}
