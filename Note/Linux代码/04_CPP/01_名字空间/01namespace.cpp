//名字空间指令
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
    //1.常用方法,使用作用域限定符"::"
    ns1::func();
    //2.名字空间指令(相当于全局变量可见)
    using namespace ns1;
    func();
    using namespace ns2;
    //func();   //error
    ns2::func();
    return 0;
}
