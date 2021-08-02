//this指针作为函数的实参，实现对象之间的交互(了解)
#include <iostream>
using namespace std;

class Student;//短视声明
class Teacher{
public:
    void educate(Student* s);
    void reply(const string& answer){
        m_answer = answer;
    }	
private:
    string m_answer;
};
class Student{
public:
    void ask(const string& question,Teacher* t){
        cout << "问题：" << question << endl;
        t->reply("不知道");
    }
};
void Teacher::educate(Student* s){
    s->ask("什么是this指针?",this);
    cout << "学生回答:" << m_answer << endl;
}
int main(void)
{
    Teacher jianggl;
    Student limu;
    jianggl.educate(&limu);
    return 0;
}


