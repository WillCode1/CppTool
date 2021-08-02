//取相反数'-'/二进制取反'~'
#include <iostream>
using namespace std;

class Integer{
public:
    Integer(int i = 0):m_i(i){}
    void print(void)const{
        cout << m_i << endl;
    }
    //自定义取相反数操作符
    const Integer operator-(void)const{
        return Integer(-m_i);
    }
    //自定义~为乘方效果
    friend const Integer operator~(const Integer&i){
        return Integer(i.m_i * i.m_i);
    }
private:
    int m_i;
};
int main(void)
{
    Integer i(100);
    Integer j = -i;
    j.print();//-100
    j = ~i;
    j.print();//10000
    return 0;
}

