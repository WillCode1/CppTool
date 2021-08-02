//定义结构体时,可以省略struct关键字
//并且结构体可以包含成员函数
#include <iostream>
using namespace std;

//声明结构体
struct Student{
    char name[100];
    int age;
    void who(void){
        cout << "我叫" << name << ",今年" << age << "岁。" << endl;
    }
};

int main(void)
{
    //定义结构体
    Student s = {"张飞",25};
    s.who();
    return 0;
}
