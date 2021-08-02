//智能指针
#include <iostream>
using namespace std;

class A{
public:
    A(void){cout << "A::A()" << endl;}
    ~A(void){cout << "A::~A()" << endl;}
};
//智能指针类
class PA{
public:
    PA(A* pa = NULL):m_pa(pa){}
    ~PA(void){
        if(m_pa)    
            delete m_pa;
    }
private:
    A* m_pa;
};
int main(void)
{
    //A* pa = new A;
    PA pa(new A);   //智能指针
    try{
        //...
        //分配4G大小的内存,error
        char* pstr = new char[0xffffffff];
        //...
        //delete pa;  //执行不到
    }
    catch(...){
        cout << "程序发生异常" << endl;
        return -1;
    }
    return 0;
}

