#include <iostream>
using namespace std;

//普通员工
class Employee{
public:
    Employee(const string& name,int id,int grade):
        m_name(name),m_id(id),m_grade(grade){}
    //打印员工信息
    void printInfo(void){
        printBasic();//公有信息
        printExtra();//特有信息
    }
    //计算员工工资
    void paySalary(void){
        cout << "工资为：" << calBasic()+calMerit() << endl;
    }
protected:
    double m_attend;//出勤率
private:
    //计算基本工资
    double calBasic(void){
        cout << "输入出勤天数:";
        int attend;
        cin >> attend;
        m_attend = attend/23.0;
        m_basic = s_grades[m_grade-1]*m_attend;
        return m_basic;
    }
    //计算绩效工资
    virtual double calMerit(void){
        return m_basic / 2;
    }
    //打印员工的基本信息
    void printBasic(void){
        cout << "姓名:" << m_name << "，职级:" <<
            m_grade << ",工号:" << m_id << endl;
    }
    //打印员工的特有信息
    virtual void printExtra(void){
        cout << "职位：普通员工" << endl;
    }
    string m_name;  //姓名
    int m_id;       //工号
    int m_grade;    //职级
    double m_basic; //基本工资
    static double s_grades[6];//职级薪资表
};
double Employee::s_grades[] = {3000,3500,5000,6000,8000,10000};

//技术员
class Technician:virtual public Employee{
public:
    Technician(const string& name,int id,int grade,double allow):
        Employee(name,id,grade),m_allow(allow){}
protected:
    void printExtra(void){
        cout << "职位:技术人员" << endl;
        cout << "研发津贴:" << m_allow << endl;
    }
    double calMerit(void){
        cout << "输入进度因数:";
        double factor;
        cin >> factor;
        //小时数*出勤率*进度因数*研发津贴
        return 8*23*m_attend*factor*m_allow;
    }
private:
    double m_allow;//研发津贴
};
//经理
class Manager:virtual public Employee{
public:
    Manager(const string& name,int id,int grade,double bouns):
        Employee(name,id,grade),m_bouns(bouns){}
protected:
    void printExtra(void){
        cout << "职位：经理" << endl;
        cout << "绩效奖金:" << m_bouns << endl;
    }
    double calMerit(void){
        cout << "输入绩效因数:" ;
        double factor;
        cin >> factor;
        //绩效奖金*绩效因数*出勤率
        return m_bouns*factor*m_attend;
    }
private:
    double m_bouns;//绩效奖金
};
//技术主管
class TechMngr:public Technician,public Manager{
public:
    TechMngr(const string& name,int id,int grade,double allow,double bouns):
        Technician(name,id,grade,allow),Manager(name,id,grade,bouns),
            Employee(name,id,grade){}
private:
    void printExtra(void){
        Technician::printExtra();
        Manager::printExtra();
    }
    double calMerit(void){
        //技术员绩效+经理绩效/2
        return (Technician::calMerit()+Manager::calMerit())/2;
    }
};

int main(void)
{
    Employee emp("张三",10016,2);
    emp.printInfo();
    emp.paySalary();
    cout << "=================" << endl;
    Technician tech("孔明",10017,4,50);
    tech.printInfo();
    tech.paySalary();
    cout << "=================" << endl;
    Manager mngr("刘备",10011,5,5000);
    mngr.printInfo();
    mngr.paySalary();
    cout << "=================" << endl;
    TechMngr techmngr("关羽",10086,5,50,5000);
    techmngr.printInfo();
    techmngr.paySalary();
    return 0;
}
