/* 模拟登陆练习
 * 编写用户输入用户名和密码，把用户输入的内容和正确内容做对比
 * 正确的用户名是admin
 * 正确的密码是123456
 * 一共给用户三次机会，最后要给出登陆结果
 */
#include<stdio.h>
#include<string.h>

int main(){
    char buf[10] = {0};
    int cnt = 1;
    for(cnt = 1; cnt <= 3; cnt++){
        printf("请输入用户名：");
        fgets(buf, 10, stdin);
        if(strlen(buf) == 9 && buf[8] != '\n'){
            scanf("%*[^\n]");
            scanf("%*c");
        }
        if(strcmp(buf, "admin\n")){
            printf("用户名出错！\n");
            continue;
        }
        printf("请输入密码：");
        fgets(buf, 10, stdin);
        if(strlen(buf) == 9 && buf[8] != '\n'){
            scanf("%*[^\n]");
            scanf("%*c");
        }
        if(strcmp(buf, "123456\n")){
            printf("密码错误！\n");
            continue;
        }
        break;
    }
    if(cnt <= 3){
        printf("登陆成功！\n");
    }
    else{
        printf("登陆失败\n");
    }
    return 0;
}
