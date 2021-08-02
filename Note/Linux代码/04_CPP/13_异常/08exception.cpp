//异常说明
#include <iostream>
using namespace std;

class FileError{};
class MemoryError{};

void foo(void)throw(FileError,int,MemoryError);
int main(void)
{
    try{
        foo();
    }
    catch(FileError& ex){
        cout << "文件操作异常处理..." << endl;
        return -1;
    }
    catch(MemoryError& ex){
        cout << "内存操作异常处理..." << endl;
        return -1;
    }
    catch(...){
        cout << "其它操作异常处理..." << endl;
        return -1;
    }
    return 0;
}
void foo(void)throw(int,FileError,MemoryError){
    throw MemoryError();
    throw FileError();
    throw -1;
}

