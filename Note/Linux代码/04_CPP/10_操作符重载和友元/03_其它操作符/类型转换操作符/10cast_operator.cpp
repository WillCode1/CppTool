//类型转换操作符重载函数
#include <iostream>
using namespace std;

class Integer{
public:
    //int-->Integer
    explicit Integer(int data = 0):m_data(data){}
    //Integer-->int
    explicit operator int(void)const{
        cout << "类型转换操作符重载函数" << endl;
        return m_data;
    }
private:
    int m_data;
};
int main(void)
{
    Integer i1;
    i1 = (Integer)100;

    cout << int(i1) << endl;//100
    int i2 = int(i1);
    cout << i2 << endl;//100
    return 0;
}

