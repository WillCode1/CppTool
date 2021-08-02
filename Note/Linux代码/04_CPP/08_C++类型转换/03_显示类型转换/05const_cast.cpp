//常量类型转换(去常转换)
#include <iostream>
using namespace std;

int main(void)
{
    const volatile int ci = 100;
    //int* pci = &ci;   //不允许有普通指针指向常量
    int* pci = const_cast<int*>(&ci);
    *pci = 200;
    cout << "ci=" << ci << endl;//200
    cout << "*pci=" << *pci << endl;
    cout << "&ci:" << (void*)&ci << endl;
    cout << "pci:" << pci << endl;
    return 0;
}
