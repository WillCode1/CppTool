//const关键字的使用
//定义指向常量的指针
#include <iostream>
using namespace std;

int main(){
    //定义的常量必须被初始化,否则后面不可修改
    const int i = 100;
    cout << i << endl;

    //多了一个可以修改定义常量的途径
    //int* p1 = &i;     //error
    
    //解决办法
    //1.定义常量指针指向常量
    const int* p1 = &i;
    cout << *p1 << endl;    //100

    //2.去常转换
    int* p2 = const_cast<int*>(&i);
    *p2 = 200;
    cout << i << endl;      //100,volatile
    cout << *p2 << endl;    //200
    return 0;
}
