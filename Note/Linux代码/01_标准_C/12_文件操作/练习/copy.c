/* 文件拷贝练习，编写程序实现文件拷贝功能
 * ./a.out abc.txt def.txt
 */
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv){
    int r_size = 0, w_size = 0, tmp = 0;
    char buf[100] = {0};
    if(argc < 3){
        printf("请输入文件路径\n");
        return 1;
    }
    else if(!strcmp(*(argv + 1), *(argv + 2))){
        printf("新文件路径不要与原文件相同\n");
        return 2;
    }
    FILE *p_src = fopen(*(argv + 1), "rb");
    if(!p_src){
        printf("原始文件打开失败\n");
        return 3;
    }
    FILE *p_dest = fopen(*(argv + 2), "wb");
    if(!p_dest){
        printf("新文件创建失败\n");
        fclose(p_src);
        p_src = NULL;
        return 4;
    }
    while((r_size = fread(buf, sizeof(char), 100, p_src)) > 0){
        //确保缓存里内容全部写出后，再执行下一次
        tmp = r_size;
        while((w_size = fwrite(buf, sizeof(char), tmp, p_dest)) > 0){
            tmp += w_size;
            r_size -= w_size;
            if(!r_size) break;
        }
    }
    fclose(p_dest);
    p_dest = NULL;
    fclose(p_src);
    p_src = NULL;
    return 0;
}
