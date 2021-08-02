//º¯Êý²Ù×÷·û'()'
#include <iostream>
using namespace std;

class Square{
public:
    double operator()(double x)const{
        return x*x;
    }
    int operator()(int a,int b,int c=9){
        return a + b - c;
    }
};
int main(void)
{
    Square square;
    //square.operator()(13.)
    cout << square(13.) << endl;//169
    cout << square(10,20,30) << endl;//0
    cout << square(10,20) << endl;//21
    return 0;
}

