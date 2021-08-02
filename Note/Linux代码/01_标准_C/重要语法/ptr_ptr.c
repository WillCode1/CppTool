//指针的值传递和址传递
#include <stdio.h>

int main(void){
    int a = 10, b = 20;
    int *p1 = NULL, *p2 = NULL;
    printf("a = %d, b = %d\n", a, b);
   
    //值传递
    p1 = &a;
    p2 = p1;    //p2指向a
    printf("*p1 = %d, *p2 = %d\n", *p1, *p2);
    p2 = &b;    //p2指向b
    printf("*p1 = %d, *p2 = %d\n", *p1, *p2);
    
    //址传递
    int **pp = NULL;
    pp = &p1;   //pp指向p1
    *pp = &b;   //p1指向b
    printf("*p1 = %d\n", *p1);
    return 0;
}
