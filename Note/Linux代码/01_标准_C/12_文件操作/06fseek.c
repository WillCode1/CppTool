/*
    文件位置指针演示
    ftell,rewind,fseek函数
    */
#include<stdio.h>

int main(){
    char ch = 0;
    FILE *p_file = fopen("abc.txt", "rb");
    if(p_file){
        //rewind(p_file);
        fseek(p_file, 2, SEEK_SET);
        printf("%ld\n", ftell(p_file));
        fread(&ch, sizeof(char), 1, p_file);
        printf("%c\n", ch);
        //rewind(p_file);
        fseek(p_file, 4, SEEK_CUR);
        printf("%ld\n", ftell(p_file));
        fread(&ch, sizeof(char), 1, p_file);
        printf("%c\n", ch);
        //rewind(p_file);
        fseek(p_file, -3, SEEK_END);
        printf("%ld\n", ftell(p_file));
        fread(&ch, sizeof(char), 1, p_file);
        printf("%c\n", ch);
        fclose(p_file);
        p_file = NULL;
    }
    return 0;
}
