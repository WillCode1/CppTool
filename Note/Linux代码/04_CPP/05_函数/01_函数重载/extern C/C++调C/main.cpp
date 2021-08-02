//C++µ÷C
#include <iostream>
extern void add(int, int);
extern void sub(int, int);
extern "C"{
    #include "stdio.h"
}

int main(void){
    add(3, 5);
    sub(5, 3);
    return 0;
}
