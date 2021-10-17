#include "Visitor.h"

void Woman::Accept(Action * action)
{
	action->PerformanceWoman(this);
}

void Man::Accept(Action * action)
{
	action->PerformanceMan(this);
}

int testVisitor()
{
	ObjectStruct obj;

	Man* man = new Man();
	obj.Attach(man);

	Woman*woman = new Woman();
	obj.Attach(woman);

	Success* success = new Success();
	obj.Display(success);

	//Failed* failed = new Failed();
	//obj.Display(failed);

	//Married* married = new Married();
	//obj.Display(married);

	//BreakUp* breakUp = new BreakUp();
	//obj.Display(breakUp);

	return 0;
}
