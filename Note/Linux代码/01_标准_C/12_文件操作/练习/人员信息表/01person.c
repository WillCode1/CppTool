/* 文件练习
 * 编写程序从键盘得到多个人员信息并记录到二进制文件里
 * 每个人员信息包括整数类型的id,float类型的工资和姓名
 * 要求文件的内容可以扩展
 */
#include<stdio.h>
#include<string.h>

typedef struct {
    int id;
    float salary;
    char name[10];
}person;

int main(){
    int choice = 0;
    person p = {0};
    FILE *p_file = fopen("person.bin", "ab");
    if(p_file){
        while(1){
            printf("请输入员工id：");
            scanf("%d" ,&(p.id));
            printf("请输入员工工资：");
            scanf("%g" ,&(p.salary));
            scanf("%*[^\n]");
            scanf("%*c");
            printf("请输入员工姓名：");
            //scanf("%s", p.name);
            fgets(p.name, 10, stdin);
            if(strlen(p.name) == 9 && p.name[8] != '\n'){
                scanf("%*[^\n]");
                scanf("%*c");
            }
            fwrite(&p, sizeof(person), 1, p_file);
            printf("是否还有员工信息输入？有1，没有0：");
            scanf("%d", &choice);
            if(choice == 0){
                break;
            }
        }
        fclose(p_file);
        p_file = NULL;
    }
    return 0;
}
