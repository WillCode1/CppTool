//析构函数的调用时机
#include <iostream>
using namespace std;

class A{
public:
    ~A(void){cout << "A::~A(void)"<<endl;}
};
int main(void)
{
    do{
        A a1;
        A* pa2 = new A;
        cout << "手动释放堆区变量,调用析构函数" << endl;
        delete pa2;     //delete-->A::~A()
        cout << "作用域终止运算符,释放栈区变量,调用析构函数" << endl;
    }while(0);          //"}"-->A::~A()
    cout << "Process end." << endl;
    return 0;
}
