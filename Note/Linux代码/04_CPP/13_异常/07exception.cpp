//建议从标准异常类中派生自己的异常类,以构成多态
#include <iostream>
#include <cstdio>
using namespace std;
//继承标准异常类
class FileError:public exception{
public:
    const char* what(void)const throw() {
        return "std::File Error";
    }
};
void func(void){
    if(!fopen("none.txt","r")){
        throw FileError();
    }
}
int main(void)
{
    try{
        func();
        char* p = new char[0xffffffff];
    }
    catch(exception& ex){
        cout << ex.what() << endl;
        return -1;
    }
    return 0;
}

