//引用结构体
#include <iostream>
using namespace std;

struct Student{
    char name[100];
    int age;
};

void printStudent(const Student& s){
    cout << s.name << ',' << s.age/*++*/ << endl;
}
int main(void)
{
    const Student s = {"悟空",26};
    printStudent(s);
    return 0;
}
