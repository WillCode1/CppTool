/*
	装饰模式：动态地给一个对象添加一些额外的功能，它是通过创建一个包装对象，也就是装饰来包裹真实的对象。新增加功能来说，装饰器模式比生产子类更加灵活。
	以下情形考虑使用装饰模式：
		需要扩展一个类的功能，或给一个类添加附加职责。
		需要动态的给一个对象添加功能，这些功能可以再动态的撤销。
		需要增加由一些基本功能的排列组合而产生的非常大量的功能，从而使继承关系变的不现实。
		当不能采用生成子类的方法进行扩充时。一种情况是，可能有大量独立的扩展，为支持每一种组合将产生大量的子类，使得子类数目呈爆炸性增长。
		另一种情况可能是因为类定义被隐藏，或类定义不能用于生成子类。
 */
 /*
 * 关键代码：1、Component 类充当抽象角色，不应该具体实现。 2、修饰类引用和继承 Component 类，具体扩展类重写父类方法。
 */
#include <iostream>
using namespace std;

//抽象构件（Component）角色：给出一个抽象接口，以规范准备接收附加责任的对象。
class Component
{
public:
	virtual ~Component() {}
	virtual void configuration() = 0;
};

//具体构件（Concrete Component）角色：定义一个将要接收附加责任的类。
class Car : public Component
{
public:
	void configuration() override
	{
		cout << "A Car" << endl;
	}
};

//装饰（Decorator）角色：持有一个构件（Component）对象的实例，并实现一个与抽象构件接口一致的接口。
class DecorateCar : public Component
{
public:
	DecorateCar(Component* car) : m_pCar(car) {}

	void configuration() override
	{
		m_pCar->configuration();
	}

private:
	Component* m_pCar;
};

//具体装饰（Concrete Decorator）角色：负责给构件对象添加上附加的责任。
class DecorateLED : public DecorateCar
{
public:
	DecorateLED(Component* car) : DecorateCar(car) {}

	void configuration() override
	{
		DecorateCar::configuration();
		addLED();
	}

private:
	void addLED()
	{
		cout << "Install LED" << endl;
	}

};

//具体装饰（Concrete Decorator）角色：负责给构件对象添加上附加的责任。
class DecoratePC : public DecorateCar
{
public:
	DecoratePC(Component* car) : DecorateCar(car) {}

	void configuration() override
	{
		DecorateCar::configuration();
		addPC();
	}

private:
	void addPC()
	{
		cout << "Install PC" << endl;
	}
};

//具体装饰（Concrete Decorator）角色：负责给构件对象添加上附加的责任。
class DecorateEPB : public DecorateCar
{
public:
	DecorateEPB(Component* car) : DecorateCar(car) {}

	void configuration() override
	{
		DecorateCar::configuration();
		addEPB();
	}

private:
	void addEPB()
	{
		cout << "Install Electrical Park Brake" << endl;
	}
};

int testDecorator()
{
	Car* car = new Car;
	DecorateLED* ledCar = new DecorateLED(car);
	DecoratePC* pcCar = new DecoratePC(ledCar);
	DecorateEPB* epbCar = new DecorateEPB(pcCar);

	epbCar->configuration();

	delete epbCar;
	epbCar = nullptr;

	delete pcCar;
	pcCar = nullptr;

	delete ledCar;
	ledCar = nullptr;

	delete car;
	car = nullptr;

	return 0;
}
