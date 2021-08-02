//字符串I/O流
#include <iostream>
#include <sstream>
using namespace std;

int main(void)
{
    int i = 100;
    double d = 1.23;
    char c = 'A';
    char s[20] = "bye C++";
    //sprintf()
#if 0 
    char buf[100];
    sprintf(buf,"%d %g %c %s",i,d,c,s);
    printf("%s",buf);
#endif
    ostringstream oss;
    oss << i << ' ' << d << ' ' << c << ' ' << s; 
    cout << oss.str() << endl;  //转换成字符串对象输出
    cout << "========================" << endl;
    //sscanf()
    istringstream iss;
    iss.str("123 3.14 B helloworld");
    iss >> i >> d >> c >> s;
    cout << i << ' ' << d << ' ' << c << ' ' << s;
    cout << endl;
    return 0;
}

