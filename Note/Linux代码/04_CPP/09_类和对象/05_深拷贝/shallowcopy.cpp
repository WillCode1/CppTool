//编译器提供的拷贝构造与拷贝赋值代码(浅拷贝)
#include <iostream>
using namespace std;

class Integer{
public:
    Integer(int data = 0):m_data(new int(data)){
    }
    //缺省的拷贝构造函数
    Integer(const Integer& that):
        m_data(that.m_data){}
    //缺省的拷贝赋值操作符函数
    Integer& operator=(const Integer& that){
        m_data = that.m_data;
        return *this;
    }
    ~Integer(void){
        delete m_data;
    }
    int get(void)const{
        return *m_data;
    }
private:
    int* m_data;
};
int main(void)
{
    Integer i(100);
    cout << i.get() << endl;//100
    Integer i2(i);  //拷贝构造
    cout << i2.get() << endl;//100
    Integer i3;
    i3 = i2;        //拷贝赋值
    cout << i2.get() << endl;//100
    return 0;
}

