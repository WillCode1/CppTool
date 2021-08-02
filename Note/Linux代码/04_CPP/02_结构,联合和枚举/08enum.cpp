//定义枚举时,可以省略enum关键字
//C++中的枚举是独立的数据类型,不能用整型数赋值

#include <iostream>
using namespace std;

int main(void)
{
    enum Color{RED,GREEN,BLUE=10,YELLOW};
    Color c = RED;      //标C里也可以这么赋值
    cout << c << endl;  //0
    //c = 10;           //error
    c = BLUE;
    cout << c << endl;  //10
    return 0;
}
