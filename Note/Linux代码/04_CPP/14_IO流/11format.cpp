//格式化I/O
#include <cmath>
#include <iostream>
using namespace std;

int main(void){
    cout << sqrt(200) << endl;  //默认保留6位有效数字
    cout.precision(10);         //设置保留10位有效数字
    cout << sqrt(200) << endl;

    cout.setf(ios::scientific); //科学计数法
    cout << sqrt(200) << endl;

    cout << "[";
    cout.width(10);             //设置域宽10个字符
    cout.setf(ios::internal);   //内插对齐,数据位右对齐,符号位做对齐
    cout.setf(ios::showpos);    //显示符号位
    cout << 1234;
    cout << "]" << endl;
    return 0;	
}

