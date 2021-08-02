//调用拷贝构造函数的时机
//g++ -fno-elide-constructors,去掉编译优化
#include <iostream>
using namespace std;

class A{
public:
    A(void){}
    A(const A& that){
        cout << "A::A(const A&)" << endl;
    }
};
void foo(A a){}
A bar(void){
    A a;
    cout << "bar::" << &a << endl;
    return a;   //拷贝构造,会优化
}

int main(void)
{
    A a1;
    A a2 = a1;  //拷贝构造
    foo(a1);    //拷贝构造
    A a3 = bar();//拷贝构造,会优化
    cout << "a3:" << &a3 << endl;
    return 0;
}
