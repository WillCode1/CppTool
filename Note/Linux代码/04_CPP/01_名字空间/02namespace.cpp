//名字空间声明
#include <iostream>
using namespace std;

namespace ns1{
    void func(void){
        cout << "ns1::func" << endl;
    }
}
namespace ns2{
    void func(void){
        cout << "ns2::func" << endl;
    }
}
int main(void)
{
    //名字空间声明(等价于定义局部变量)
    using ns1::func;
    func();
    //名字空间指令(相当于全局变量可见)
    using namespace ns2;
    func();     //ns1::func,局部优先
    ns2::func();//ns2::func
    return 0;
}
