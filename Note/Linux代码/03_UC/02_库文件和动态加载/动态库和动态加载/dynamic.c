/* 动态加载相关函数
 * dlopen,dlclose,dlerror,dlsym
 * 编译时加上-ldl
 */
#include <stdio.h>
#include <dlfcn.h>
typedef int (*math_t)(int, int);

int main(){
    int x = 3, y = 5;
    //加载动态库
    void *handle = dlopen("libtmath.so", RTLD_NOW);
    if(!handle){
        printf("%s\n", dlerror());
        return 1;
    }
    //加载动态库里的函数到内存
    void *p = dlsym(handle, "mul");
    if(!p){
        printf("%s\n", dlerror());
        return 2;
    }
    //将p强制转换成函数指针
    int (*f)(int,int) = (int(*)(int,int))p;
    printf("%d*%d=%d\n", x, y, f(x,y));
    //将动态库的引用计数减一
    dlclose(handle);
    return 0;
}
