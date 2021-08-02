/* 位置指针练习，
 * 编写程序从person.bin文件中获得所有人的id并显示在屏幕上
 * 只能拿id，用位置指针跳过
 */
#include<stdio.h>

typedef struct {
    int id;
    float salary;
    char name[10];
}person;

int main(){
    int id = 0, size = 0;
    FILE *p_file = fopen("person.bin", "rb");
    if(p_file){
        while(1){
            size = fread(&id, sizeof(int), 1, p_file);
            if(!size){
                break;
            }
            printf("id:%d\n", id);
            fseek(p_file, sizeof(person) - sizeof(int), SEEK_CUR);
        }
        fclose(p_file);
        p_file = NULL;
    }
    return 0;
}
