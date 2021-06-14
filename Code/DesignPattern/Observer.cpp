#include "Observer.h"

int testObserver()
{
	/*
	* 这里需要补充说明的是在此示例代码中，View一旦被注册到DataModel类之后，DataModel解析时会自动解析掉
	* 内部容器中存储的View对象，因此注册后的View对象不需要在手动去delete，再去delete View对象会出错。
	*/

	View* v1 = new TableView("TableView1");
	View* v2 = new TableView("TableView2");
	View* v3 = new TableView("TableView3");
	View* v4 = new TableView("TableView4");

	IntDataModel* model = new IntDataModel;
	model->addView(v1);
	model->addView(v2);
	model->addView(v3);
	model->addView(v4);

	model->notify();

	cout << "-------------\n" << endl;

	model->removeView(v1);

	model->notify();

	delete model;
	model = nullptr;

	return 0;
}
