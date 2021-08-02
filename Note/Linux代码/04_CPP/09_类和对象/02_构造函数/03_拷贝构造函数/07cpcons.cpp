//拷贝构造函数
#include <iostream>
using namespace std;

class A{
public:
    A(int data = 0){
        cout << "A::A(void)" << endl;
        m_data = data;
    }
    //拷贝构造函数，参数必须是当前 类 类型的常引用
    A(const A& that){
        cout << "A::A(const A&)" << endl;
        m_data = that.m_data;
    }
    int m_data;
};
int main(void)
{
    A a1(100);	
    //A a2(a1);
    A a2 = a1;  //和上面写法完全等价	
    cout << a1.m_data << endl;//100
    cout << a2.m_data << endl;//100
    return 0;
}


