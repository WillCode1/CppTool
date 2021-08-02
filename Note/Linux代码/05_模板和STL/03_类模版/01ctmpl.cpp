//类模版演示
#include <iostream>
using namespace std;
template<typename T>
class Comparator{
public:
    Comparator(T x, T y):m_x(x),m_y(y){}
    T max(void)const{
        return m_x < m_y ? m_y : m_x;
    }
    T min(void)const{
        //这里不用'>',避免实例化的类需要多重载一个操作符
        return m_x < m_y ? m_x : m_y;
    }
private:
    T m_x, m_y;
};
class Integer{
public:
    Integer(int arg):m_var(arg){}
    friend ostream& operator<<(ostream& os, Integer const& i){
        return os << i.m_var;
    } 
    bool operator<(Integer const& i)const{
        return m_var < i.m_var;
    }
private:
    int m_var;
};
int main(void){
    int a = 123, b = 456;
    Comparator<int> ci(a, b);
    cout << ci.max() << ' ' << ci.min() << endl;

    double c = 1.3, d = 4.6;
    Comparator<double> cd(c, d);
    cout << cd.max() << ' ' << cd.min() << endl;
    
    string e = "hello", f = "world";
    Comparator<string> cs(e, f);
    cout << cs.max() << ' ' << cs.min() << endl;
    
    Comparator<Integer> cn(a, b);
    cout << cn.max() << ' ' << cn.min() << endl;
    return 0;
}
