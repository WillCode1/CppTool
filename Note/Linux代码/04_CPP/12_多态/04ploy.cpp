//虚析构函数
#include <iostream>
using namespace std;

class Base{
public:
    Base(void){
        cout << "Base::Base()" << endl;
        m_data = new int;
    }
    virtual ~Base(void){//虚析构函数
        cout << "Base::~Base()" << endl;
        delete m_data;
        m_data = NULL;
    }
    int* m_data;
};
class Derived:public Base{
public:
    Derived(void){
        cout << "Derived::Derived()" << endl;
        m_data = new int;
    }
    ~Derived(void){
        cout << "Derived::~Derived()" << endl;
        delete m_data;
        m_data = NULL;
    }
    int* m_data;
};
int main(void)
{
    Base* pb = new Derived;
    //...
    delete pb;
    return 0;
}

