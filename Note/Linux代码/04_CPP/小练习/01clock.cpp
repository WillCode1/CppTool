//练习：实现一个电子钟
#include <iostream>
#include <ctime>
#include <cstdio>
using namespace std;

class Clock{
public:
    Clock(time_t t){
        tm* local = localtime(&t);
        m_hour = local->tm_hour;
        m_min = local->tm_min;
        m_sec = local->tm_sec;
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
    Clock c = Clock(time(NULL));
    c.run();
    return 0;
}
