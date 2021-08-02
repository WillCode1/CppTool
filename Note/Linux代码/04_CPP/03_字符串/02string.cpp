//字符串的运算
#include <iostream>
#include <cstring>
using namespace std;

int main(void)
{
    //赋值/拷贝(strcpy)
    string s1 = "hello", s2 = "world";
    s1 = s2;
    cout << "s1 = " << s1 << endl;  //world
    
    //拼接(同下)
    string s3 = "Hans ", s4 = "Zimmer";
    string s5 = s3 + s4;
    cout << "s5 = " << s3 + s4 << endl;//Hans Zimmer
    
    //拼接赋值(strcat)
    s1 += s2;
    cout << "s1 = " << s1 << endl;    //helloworld
    
    //关系比较(strcmp)
    cout << boolalpha << (s1 == s2) << endl;//false
    cout << (s1 > s2) << endl;  //true
    
    //下标访问
    s1[20] = 'A';    //不检查越界修改
    cout << s1[20] << endl;
    cout << s1[0] << s1[1] << s1[2] << endl;   //wor
    return 0;
}
