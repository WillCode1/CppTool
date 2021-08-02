//在类的外部定义成员函数，必须用作用域限定操作符
//说明属于哪个类
#include "stu.h"
//类的实现
Student::Student(const string& name){
    m_name = name;
}
void Student::who(void){
    cout << m_name << endl;
}
