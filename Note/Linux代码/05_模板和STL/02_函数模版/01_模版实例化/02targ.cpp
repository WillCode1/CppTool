//实例化模板的类型实参,必须满足具体函数的实现要求
#include <iostream>
using namespace std;
template<typename T>
T add(T x, T y){
    return x + y;
}
class Integer{
public:
    Integer(int arg):m_var(arg){}
    Integer const operator+(Integer const& i)const{
        return m_var + i.m_var;
    }
    friend ostream& operator<<(ostream& os, Integer const& i){
        return os << i.m_var;
    }
private:
    int m_var;
};
int main(void){
    cout << add<int>(123, 456) << endl;
    cout << add<double>(1.3, 4.6) << endl;
    cout << add<string>("Hello, ", "World !") << endl;
    //基本类型无法重载
//  cout << add<char const*>("Hello, ", "World !") << endl;
    cout << add<Integer>(123, 456) << endl;
    return 0;
}
