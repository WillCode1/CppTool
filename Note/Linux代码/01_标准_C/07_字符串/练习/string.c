/*统计一个字符串在另一个字符串中出现的次数
 */
#include<stdio.h>
#include<string.h>

int main(){
    char *p_str = "abcdefaabcdefabfedsabcf";
    char buf[10] = {0};
    int cnt = 0;
    printf("请输入要找的字符串：");
    scanf("%s", buf);
    while(1){
        p_str = strstr(p_str,buf);
        if(p_str == 0){
            break;
        }
        p_str++;
        cnt++;
    }
    printf("字符串一共出现了%d次\n", cnt);
    return 0;
}
