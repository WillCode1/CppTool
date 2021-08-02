//无名名字空间
#include <iostream>
using namespace std;

namespace ns1{
    int num = 10;
}

//将会放入无名名字空间
//无需指令声明,默认可见
int num = 20;

int main(void)
{
    cout << num << endl;//20
    //名字空间声明
    using ns1::num;
    cout << num << endl;//10
    cout << ::num << endl;//20
    return 0;
}
