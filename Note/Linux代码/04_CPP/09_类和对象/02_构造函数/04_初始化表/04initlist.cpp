//成员变量的初始化顺序由其在类中声明顺序决定，而与初始化表的顺序无关
#include <iostream>
#include <cstring>
using namespace std;

class Dummy{	
public:
    Dummy(const char* psz):
        //m_str(psz),m_len(m_str.length()){}
        
        //尽量避免成员变量在初始化过程中的互相依赖
        m_str(psz?psz:""),m_len(psz?strlen(psz):0){}
    size_t m_len;
    string m_str;
};
int main(void)
{
    //Dummy d("abcdefg");
    Dummy d(NULL);
    cout << d.m_str << ',' << d.m_len << endl;
    return 0;	
}
