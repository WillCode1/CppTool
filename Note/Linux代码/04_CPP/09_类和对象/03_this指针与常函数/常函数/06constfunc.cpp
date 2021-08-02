//常函数和mutable关键字
#include <iostream>
using namespace std;

class A{
public:
    A(int data = 0):m_data(data){}
    //声明为常函数，避免意外修改其成员变量
    void print(void)const{
        cout << m_data/*++*/ << endl;
    }
    void show(void)const{
        cout << m_data++ << endl;
        //cout << const_cast<A*>(this) -> m_data++ << endl;
    }
private:
    //mutable修饰的成员变量会在使用时去除常属性
    mutable int m_data;
};
int main(void)
{
    A a(100);
    a.show();
    a.show();
    return 0;
}

