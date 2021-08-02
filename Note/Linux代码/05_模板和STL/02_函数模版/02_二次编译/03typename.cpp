//二次编译
#include <iostream>
using namespace std;
class A{
public:
    typedef unsigned int uint;
    class B{};
};
template<typename T>
void foo(void){
    //告诉编译器,二次编译时再识别类型
    typename T::uint u;
    typename T::B b;
}

int main(void){
    A::uint u;
    A::B b;
    foo<A>();
    return 0;
}
