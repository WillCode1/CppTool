//插入'<<',提取'>>'运算符重载
#include <iostream>
using namespace std;

class Complex{
public:
    Complex(int r,int i):m_r(r),m_i(i){}
private:
    int m_r;//复数实部
    int m_i;//复数虚部
    friend ostream& operator<<(ostream& os,const Complex& c){
        return os << c.m_r << '+' << c.m_i << 'i';
    }
    friend istream& operator>>(istream& is,Complex& c){
        return is >> c.m_r >> c.m_i;
    }
};
int main(void)
{
    Complex c1(1,2);
    Complex c2(3,4);
    cout << c1 << endl;//operator<<(cout,c1)
    cout << c1 << ',' << c2 << endl;
    cout << "请输入两个复数:" ;
    cin >> c1 >> c2;
    cout << c1 << ',' << c2 << endl;
    return 0;
}

