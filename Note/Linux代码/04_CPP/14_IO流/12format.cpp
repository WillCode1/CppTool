//格式化I/O
#include <iomanip>//带参数的留控制符
#include <iostream>
#include <cmath>
using namespace std;

int main(void)
{
    cout << sqrt(200) << endl;
    //设置精度
    cout << setprecision(10) << sqrt(200) << endl;
    //科学计数法显示
    cout << scientific << sqrt(200) << endl;
    cout << "[";
    cout << setw(10) << showpos << left << 1234;
    cout << "]" << endl;
    return 0;
}

