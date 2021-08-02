//继承,向上向下造型
#include <iostream>
using namespace std;
//人类
class Human{
public:
    Human(const string& name,int age):
        m_name(name),m_age(age),m_id(100){}
    void eat(const string& food)const{
        cout << "我在吃" << food << endl;	
    }
    void sleep(int time)const{
        cout << "我睡了" << time << "小时." << endl;
    }
protected://保护类型成员可以在类的内部和子类中访问
    string m_name;
    int m_age;
    //私有成员子类无法直接访问，但可以公有或保护的接口函数来间接访问
    const int& get_id(void)const{
        return m_id;
    }
private: 
    int m_id;
};
//学生类，继承人类
class Student:public Human{
public:
    //通过初始化表显示指名基类子对象的初始化方式
    Student(const string& name,int age,int no):
        Human(name,age),m_no(no){}
    void who(void)const{
        cout << "我叫" << m_name << ",今年" << 
            m_age << "岁，学号是" << m_no << 
            "，身份证号" << get_id() << endl;
    }
    void learn(const string& course)const{
        cout << "我在学" << course << endl;
    }
private:
    int m_no;
};
//教师类,继承人类
class Teacher:public Human{
public:
    Teacher(const string& name,int age,double salary):
        Human(name,age),m_salary(salary){}
    void who(void)const{
        cout << "我叫" << m_name << "今年," <<
            m_age << "岁,每个月的薪水为:" <<
            m_salary << endl;
    }
    void teach(const string& course)const{
        cout << "我在教" << course << endl;
    }
private:
    double m_salary;
};
int main(void)
{
    Student s("悟空",30,10011);
    s.who();
    s.eat("水果");
    s.sleep(8);
    s.learn("念经");

    Teacher t("三藏",35,800.5);
    t.who();
    t.eat("面条");
    t.sleep(6);
    t.teach("佛法");

    //向上造型，安全，可以隐式转换
    //ph：指向子类对象的基类指针
    Human* ph = &s;
    ph->eat("桃子");
    ph->sleep(10);
    //ph->who();

    //安全的向下造型，必须显示完成
    Student* ps = static_cast<Student*>(ph);
    ps->who();

    //危险的向下造型
    Human h("张飞",28);
    ps = static_cast<Student*>(&h);
    ps->who();
    return 0;
}


