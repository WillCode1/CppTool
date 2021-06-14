#include "Strategy.h"

int testStrategy()
{
	Archer<ApcHurt>* arc = new Archer<ApcHurt>;
	arc->attack();

	delete arc;
	arc = nullptr;

	return 0;
}
