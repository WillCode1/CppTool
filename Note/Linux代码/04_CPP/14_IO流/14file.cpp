//文件I/O流
#include <iostream>
#include <fstream>
using namespace std;

int main(void)
{
    //fprintf()
    ofstream ofs("file.txt");
    ofs << 1234 << ' ' << 56.78 << ' ' << "hello" << endl;
    ofs.close();
    cout << "========================" << endl;
    //fscanf()
    ifstream ifs("file.txt");
    int i;
    double d;
    string s;
    ifs >> i >> d >> s;
    cout << i << ' ' << d << ' ' << s << endl;
    ifs.close();
    return 0;
}
