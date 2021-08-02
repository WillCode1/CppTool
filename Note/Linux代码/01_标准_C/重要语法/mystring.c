//mystrlen,mystrcat,mystrcpy
#include <stdio.h>

int mystrlen(const char *p_src){
    int cnt = 0;
    while(*p_src != '\0'){
        cnt++;
        p_src++;
    }
    return cnt;
}

int mystrcpy(char *p_src, const char *p_dest){
    while(*p_dest != '\0'){
        *p_src = *p_dest;
        p_src++;
        p_dest++;
    }
    *p_src = '\0';
    return 0;
}

int mystrcat(char *p_src, const char *p_dest){
    p_src += mystrlen(p_src);
    while(*p_dest != '\0'){
        *p_src = *p_dest;
        p_src++;
        p_dest++;
    }
    *p_src = '\0';
    return 0;
}

int main(){
    char buf[20] = "Zimmer";
    mystrcat(buf, ",hello!");
    printf("%d\n", mystrlen(buf));
    printf("%s\n", buf);
    mystrcpy(buf, "thank you!");
    printf("%s\n", buf);
    return 0;
}
