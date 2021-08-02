//获取字符串大小
//字符串对象的操作函数
#include <iostream>
#include <cstring>
using namespace std;

int main(void)
{
    string s1 = "hello";
    //获取字符串长度
    cout << strlen(s1.c_str()) << endl;
    cout << s1.length() << endl;
    cout << s1.size() << endl;
    
    //字符串的交换
    string s2 = "abcdefg";
    swap(s1,s2);
    cout << "s1= " << s1 << endl;    //abcdefg
    cout << "s2= " << s2 << endl;    //hello
    //清空字符串
    s1.clear();
    cout << "s1= " << s1 << endl;
    //判断字符串是否为空
    cout << boolalpha << s1.empty() << endl;//true
    return 0;
}
