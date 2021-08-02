//多继承,向上造型
#include <iostream>
using namespace std;

class Phone{//电话类
public:
    Phone(const string& number):m_number(number){}
    void call(const string& number){
        cout << m_number << "打给" << number <<endl;
    }
private:
    string m_number;
};
class Player{//播放器类
public:
    Player(const string& media):m_media(media){}
    void play(const string& music){
        cout << m_media << "播放器播放" << music << endl;
    }
private:
    string m_media;
};
class Computer{//计算机类
public:
    Computer(const string& os):m_os(os){}
    void run(const string& app){
        cout << "在" << m_os << "系统上运行" << app << endl;
    }
private:
    string m_os;
};
//智能手机:多重继承
class SmartPhone:public Phone,public Player,public Computer{
public:		
    SmartPhone(const string& number,const string& media,const string& os):
        Phone(number),Player(media),Computer(os){}
};
int main(void)
{
    SmartPhone sp("18710053085","MP4","Android");
    sp.call("0101-62132018");
    sp.play("最炫小苹果.mp3");
    sp.run("Angry Bird");

    SmartPhone* p1 = &sp;
    //编译器将自动做指针的偏移
    Phone* p2 = p1;     //p1+0
    Player* p3 = p1;    //p1+sizeof(Phone)
    Computer* p4 = p1;  //p1+sizoeof(Phone)+sizoef(Player)
    cout << "p1=" << p1 << endl;
    cout << "p2=" << p2 << endl;
    cout << "p3=" << p3 << endl;
    cout << "p4=" << p4 << endl;
    return 0;
}

