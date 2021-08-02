//单例模式：饿汉式
#include <iostream>
using namespace std;
class A{
public:
    //3)提供该对象的访问接口
    static A& getInstance(void){
        return s_instance;
    }
    void print(void){
        cout << m_data << endl;
    }
private:
    //1)私有化所有的构造函数
    A(int data = 0):m_data(data){}
    A(const A&);
    int m_data;
    //2)饿汉模式,进程启动就创建的唯一对象
    static A s_instance;
};
A A::s_instance(1234);

int main(void)
{
    A& a1 = A::getInstance();
    A& a2 = A::getInstance();
    cout << &a1 << ' ' << &a2 << endl;
    a1.print();
    a2.print();
    return 0;
}

