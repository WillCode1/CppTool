//二进制文件I/O流
#include <iostream>
#include <fstream>
using namespace std;

int main(void)
{
    //write()
    ofstream ofs("over.txt");   //创建对象(打开文件)
    char wbuf[]="jiangguiliang94@sina.com";
    ofs.write(wbuf,sizeof(wbuf));
    ofs.close();
    
    //read()
    ifstream ifs("over.txt");   //创建对象(打开文件)
    char rbuf[100]={0};
    ifs.read(rbuf,sizeof(rbuf));
    cout << rbuf << endl;
    ifs.close();
    return 0;
}

