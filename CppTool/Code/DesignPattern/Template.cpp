#include "Template.h"

int testTemplate()
{
	ComputerB* c1 = new ComputerB();
	c1->product();

	delete c1;
	c1 = nullptr;

	return 0;
}
