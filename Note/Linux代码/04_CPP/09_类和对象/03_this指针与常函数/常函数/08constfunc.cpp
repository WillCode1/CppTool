//相同签名成员函数的常版本和非常版本可以重载
#include <iostream>
using namespace std;

class A{
public:
    //void func(const A*){}
    void func(void)const{
        cout << "func函数常版本" << endl;
    }
    //void func(A*){}
    void func(void){
        cout << "func函数非常版本" << endl;
    }
};
int main(void)
{
    A a;
    const A& ra = a;
    a.func();   //非常对象匹配非常版本
    ra.func();  //常对象匹配常版本
    return 0;	
}
