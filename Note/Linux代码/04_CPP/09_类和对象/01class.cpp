//类的初识
#include <iostream>
using namespace std;
/*struct*/
class Student{
public:
    void eat(const string& food){
        cout << "我吃" << food << endl;
    }
    void sleep(int time){
        cout << "我睡了" << time << "小时" << endl;
    }
    void learn(const string& course){
        cout << "我学" << course << endl;
    }
    void who(void){
        cout << "我叫" << m_name << ",今年" << 
            m_age << "岁，学号是"<< m_no << endl;
    }
    /*私有属性的成员不能访问，但是可以提供公有的接口
    函数来间接访问，可以对非法数据加以限定，或者控
    制业务逻辑的合理性*/
    void setName(const string& name){
        if(name == "二"){
            cout << "你才" << name << endl;
        }
        else{
            m_name = name;
        }
    }
    void setAge(int age){
        if(age < 0){
            cout << "无效年龄" << endl;
        }
        else{
            m_age = age;
        }
    }
    void setNo(int no){
        if(no < 1000){
            cout << "无效学号" << endl;
        }
        else{
            m_no = no;
        }
    }
private:
    string m_name;
    int m_age;
    int m_no;
};
int main(void)
{
    //对象实例化
    Student s;
    
    s.setName("张飞");
    s.setName("二");
    s.setAge(25);
    s.setAge(-10);
    s.setNo(10086);
    s.setNo(250);
    
    s.who();
    s.eat("牛肉");
    s.sleep(8);
    s.learn("C++编程");
    return 0;
}
