//new,对象创建操作符
//delete,对象销毁操作符
#include <iostream>
#include <cstdlib>
using namespace std;
class A{
public:
    A(void){cout << "A::A()" << endl;}
    ~A(void){cout << "A::~A()" << endl;}
    static void* operator new(size_t size){
        cout << "A::new" << endl;
        void* pv = malloc(size);
        return pv;
    }
    static void operator delete(void* pv){
        cout << "A::delete" << endl;
        free(pv);
    }
};
int main(void)
{
    //1)A* pa = (A*)A::operator new(sizeof(A));
    //2)pa->A::A()
    A *pa = new A;
    //1)pa->~A::A()
    //2)A::operator delete(pa)
    delete pa;
    return 0;
}


