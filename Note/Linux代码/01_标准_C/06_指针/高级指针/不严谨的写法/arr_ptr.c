/*
 * 指针数组,数组指针
 * 可以用gdb调试(gcc -g)
 * set args可以设置参数
 */
#include <stdio.h>

int main(int argc, char** argv){
    //二级指针
    if(argc == 1){
        printf("请输入主函数的参数！\n");
        return 0;
    }
    printf("%c\n", *(*(argv+1)+1));

    //字符指针数组
    char* strs[] = {"abc", "def", "ghi", "jkl"};
    printf("%c\n", *(*(strs+2)+2));
    
    //二维数组
    int arr[2][3] = {1, 2, 3, 4, 5, 6};
    printf("%d\n", *(*(arr+1)+1));
    
    //数组指针
    //一维数组指针指向一维数组
    int (*p_arr)[3] = NULL;
    p_arr = arr;            //数组指针真身
    //int (*p_arr)[3] = arr;

    //同一个地址的不同访问方式
    printf("%d\n", *(*(p_arr+1)+1));
    printf("%d\n", *(*(p_arr+2)-1));
    printf("%d\n", *(*p_arr+5));

    //一维数组指针指向数组首元素
    int brr[] = {5,4,3,2,1};
    p_arr = brr;
    //指向数组的第二个元素
    printf("%d %d\n", *(*p_arr+1), (*p_arr)[1]);
    //指向的第二个数组的元素和首地址
    printf("%d %d %d %p\n", **(p_arr+1), *p_arr[1], *(*(p_arr+1)-1), p_arr[1]);
    return 0;
}
