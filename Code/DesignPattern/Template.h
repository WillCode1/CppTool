/*
 * 模板模式：定义一个操作中的算法的骨架，而将一些步骤延迟到子类中。
   模板方法使得子类可以不改变一个算法的结构即可重定义该算法的某些特定步骤。
   当多个类有相同的方法，并且逻辑相同，只是细节上有差异时，可以考虑使用模板模式。
   具体的实现上可以将相同的核心算法设计为模板方法，具体的实现细节有子类实现。

   缺点:每一个不同的实现都需要一个子类来实现，导致类的个数增加，使得系统更加庞大。
   以生产电脑为例，电脑生产的过程都是一样的，只是一些装配的器件可能不同而已。
*/

#include <iostream>
using namespace std;

class Computer
{
public:
	void product()
	{
		installCpu();
		installRam();
		installGraphicsCard();
	}

protected:
	virtual void installCpu() = 0;
	virtual void installRam() = 0;
	virtual void installGraphicsCard() = 0;
};

class ComputerA : public Computer
{
protected:
	void installCpu() override
	{
		cout << "ComputerA install Inter Core i5" << endl;
	}

	void installRam() override
	{
		cout << "ComputerA install 2G Ram" << endl;
	}

	void installGraphicsCard() override
	{
		cout << "ComputerA install Gtx940 GraphicsCard" << endl;
	}
};

class ComputerB : public Computer
{
protected:
	void installCpu() override
	{
		cout << "ComputerB install Inter Core i7" << endl;
	}

	void installRam() override
	{
		cout << "ComputerB install 4G Ram" << endl;
	}

	void installGraphicsCard() override
	{
		cout << "ComputerB install Gtx960 GraphicsCard" << endl;
	}
};

int testTemplate();
