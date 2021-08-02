//新的运算符new/new[]/delete/delete[]
#include <iostream>
using namespace std;

int main(void){
    //int* p = (int*)malloc(sizeof(int));
    int* p = new int;
    *p = 100;
    cout << *p << endl;
    delete p;
    p = NULL;
    
    //分配内存同时初始化
    p = new int(200);
    cout << *p << endl;//200
    (*p)++;
    cout << *p << endl;//201
    delete p;
    p = NULL;

    //new数组同时初始化,C++11中支持
    //int* pa = new int[10]{0,1,2,3,4,5,6,7,8,9};
    int* pa = new int[10];
    for(int i = 0; i < 10; i++){
        pa[i] = i;
        cout << pa[i] << ' ';
    }
    cout << endl;
    delete[] pa;
    pa = NULL;
    return 0;
}
