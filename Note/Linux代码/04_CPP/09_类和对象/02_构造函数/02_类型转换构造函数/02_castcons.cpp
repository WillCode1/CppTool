//类型转换构造函数
//类类型的对象转换成别的对象
#include <iostream>
using namespace std;

class A{
public:
    A(int length=0, int width=0){
        m_length = length;
        m_width = width;
    }
    void print(void){
        cout << "面积：" << m_length*m_width << endl;
    }
//protected:
    int m_length;
    int m_width;
};
class B:public A{
public:
    B(int length=0, int width=0, int high=0):
        A(length,width),m_high(high){}
    //类型转换构造函数
    explicit B(A const& a){
        m_length = a.m_length;
        m_width = a.m_width;
        //一调用构造函数,清空对象内数据
        m_high = 1;
    }
    void print(void){
        cout << "体积：" << m_length*m_width*m_high << endl;
    }
private:
    int m_high;
};

int main(void){
    A a(3,4);
    a.print();
    //显示类型转换
    B b;
    b = B(a);
    b.print();
    return 0;
}
