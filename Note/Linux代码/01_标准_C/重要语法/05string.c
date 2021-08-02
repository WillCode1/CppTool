/*单个字符，与两种字符串表示方法的比较
 */
#include <stdio.h>

int main(){
    char ch = 'c';
    //char str = "abc"; //错误写法，把字符串赋值给单个字符
    char *p_str = "abc";
    char str[] = "abc";
    printf("%c\n", ch);
    printf("%s\n", p_str);
    printf("%s\n", str);
    return 0;
}
