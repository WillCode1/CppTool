#include "FactoryMethod.h"

int testFactoryMethod()
{
	TankFactory* factory56 = new Tank56Factory();
	Tank* tank56 = factory56->createTank();
	tank56->type();

	TankFactory* factory96 = new Tank96Factory();
	Tank* tank96 = factory96->createTank();
	tank96->type();

	delete tank96;
	tank96 = nullptr;
	delete factory96;
	factory96 = nullptr;

	delete tank56;
	tank56 = nullptr;
	delete factory56;
	factory56 = nullptr;

	return 0;
}
