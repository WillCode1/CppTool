/*
   结构体类型和结构体指针演示
   */
#include <stdio.h>
/*struct person {
    int age;
    float height;
    char name[10];
};
typedef struct person sperson;*///起别名的语句
typedef struct /*person*/ {
    int age;
    float height;
    char name[10];
} sperson;

int main() {
    //struct person prsn;  //结构体变量声明语句
    sperson prsn1 = {20, 1.74f, "abc"};
    sperson prsn2 = {0};
    sperson *p_person = NULL;  //结构体指针
    printf("请输入年龄：");
    scanf("%d", &(prsn1.age));
    printf("请输入身高：");
    scanf("%g", &(prsn1.height));
    scanf("%*[^\n]");
    scanf("%*c");
    printf("请输入姓名：");
    fgets(prsn1.name, 10, stdin);
    printf("年龄是%d\n", prsn1.age);
    printf("身高是%g\n", prsn1.height);
    printf("姓名是%s\n", prsn1.name);
    prsn2 = prsn1;
    printf("年龄是%d\n", prsn2.age);
    printf("身高是%g\n", prsn2.height);
    printf("姓名是%s\n", prsn2.name);
    p_person = &prsn1;
    printf("年龄是%d\n", p_person->age);
    printf("身高是%g\n", p_person->height);
    printf("姓名是%s\n", p_person->name);
    return 0;
}
