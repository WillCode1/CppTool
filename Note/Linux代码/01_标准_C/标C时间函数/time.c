//时间函数使用
//time,ctime,gmtime,localtime,asctime
#include <stdio.h>
#include <time.h>

int main(){
    time_t tm;
    time(&tm);  //当前时间存入tm
    printf("%ld\n",time(0));
    //把时间转化为字符串
    printf("%s",ctime(&tm));

    //把当前时间转化为格林威治时间结构体
    struct tm *p_tm = gmtime(&tm);
    //把时间结构体转化为字符串
    printf("%s", asctime(p_tm));
    //把当前时间转化为所在时区时间结构体
    p_tm = localtime(&tm);
    printf("%s", asctime(p_tm));
    return 0;
}
