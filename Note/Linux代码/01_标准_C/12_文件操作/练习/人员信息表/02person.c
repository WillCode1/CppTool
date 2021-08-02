/* 文件练习
 * 编写程序从文件里获得所有人员信息并显示在屏幕上
 */
#include<stdio.h>

typedef struct {
    int id;
    float salary;
    char name[10];
}person;

int main(){
    int size = 0;
    person p = {0};
    FILE *p_file = fopen("person.bin", "rb");
    if(p_file){
        while(1){
            size = fread(&p, sizeof(person), 1, p_file);
            if(!size){
                break;
            }
            printf("员工id：%d\n", p.id);
            printf("员工工资：%g\n", p.salary);
            printf("员工姓名：%s\n", p.name);
        }
        fclose(p_file);
        p_file = NULL;
    }
    return 0;
}
