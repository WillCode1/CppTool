//标准输入输出流
#include <iostream>
#include <cstdio>

int main(void){
    //<< std::endl为打印'\n'
    std::cout << "hello world!" << std::endl;
    printf("hello world!\n");
    write(1, "hello world!\n", 13);
    
    int a, b;
    //从标准输入流提取数据放入变量a,b
    std::cin >> a >> b;
    //向标准输出流插入数据a,b
    std::cout << a << ',' << b << std::endl;
}
