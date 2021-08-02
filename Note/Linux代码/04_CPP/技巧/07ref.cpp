//多采用二进制的位运算
//引用做形参
#include <iostream>
using namespace std;
void swap1(int* x,int* y){
    *x = *x ^ *y;
    *y = *x ^ *y;  
    *x = *x ^ *y;
}
//引用不用分配内存
void swap2(int& x,int& y){
    x = x ^ y;
    y = x ^ y;  
    x = x ^ y;
}
int main(void)
{
    int x = 100, y = 200;
    //swap1(&x,&y);
    swap2(x,y);
    cout << "x=" << x << endl;
    cout << "y=" << y << endl;
    return 0;	
}
