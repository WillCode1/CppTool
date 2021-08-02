//Cµ÷C++
//gcc -lstdc++
#include <stdio.h>
extern void add(int, int);
extern void sub(int, int);

int main(void){
    add(3, 5);  //_Z3addii(3, 5);
    sub(5, 3);  //_Z3subii(5, 3);
    return 0;
}
