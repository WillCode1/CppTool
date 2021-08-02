//常对象只能调用常函数
//常对象包括常引用和常指针
#include <iostream>
using namespace std;

class A{
public:
    //void func1(const A* this)
    void func1(void)const{
        cout << "常函数" << endl;
    }
    //void func2(A* this)
    void func2(void){
        cout << "非常函数" << endl;
    }
};
int main(void)
{
    A b;
    b.func1();      //非->常
    b.func2();      //非->非
    const A b2 = b;
    b2.func1();     //常->常
    //b2.func2();   //常->非,error
    
    const A* p = &b;//p常指针
    p->func1();     //常->常
    //p->func2();   //常->非,error

    const A& r = b; //r常引用
    r.func1();      //常->常
    //r.func2();    //常->非,error
    return 0;
}

