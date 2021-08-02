//练习：实现一个电子钟,外加计时器功能
#include <iostream>
#include <ctime>
#include <cstdio>
using namespace std;

class Clock{
public:
    //缺参默认为0，表现为计时器；此外是电子钟
    Clock(bool timer = false):
        m_hour(0),m_min(0),m_sec(0){
        if(timer){
            time_t t = time(NULL);
            tm* local = localtime(&t);
            m_hour = local->tm_hour;
            m_min = local->tm_min;
            m_sec = local->tm_sec;
        }
    }
    void run(void){
        while(1){
            show();
            tick();
        }
    }
private:
    void tick(void){
        sleep(1);
        if(++m_sec == 60){
            m_sec = 0;
            if(++m_min == 60){
                m_min = 0;
                if(++m_hour == 24){
                    m_hour = 0;
                }
            }
        }
    }
    void show(void){
        printf("\r%02d:%02d:%02d",m_hour,m_min,m_sec);
        fflush(stdout);
    }
private:
    int m_hour;
    int m_min;
    int m_sec;
};

int main(){
    //Clock c = Clock(3123);
    Clock c = Clock();
    c.run();
    return 0;
}
