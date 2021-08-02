//引用,即别名
#include <iostream>
using namespace std;

int main(void)
{
    int a = 123;
    int& b = a; //b引用a
    int& c = b;
    b++;
    cout << "a=" << a << endl;
    cout << "c=" << c << endl;

    int d = 321;
    b = d;      //将d赋值给a
    cout << "b=" << b << endl;//321
    cout << "c=" << c << endl;//321
    return 0;
}
