//编写程序实现命令ls -l
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include <pwd.h>
#include <grp.h>
#include <dirent.h>

int mystat(const char*, struct stat*);

int main(int argc, char *argv[]){
    struct stat st;
    //1.后面跟有文件名
    if(argc > 1){
        mystat(argv[1], &st);
        return 0;
    }
    //2.不加文件名
    //遍历目录
    DIR *dir = opendir("./");
    if(NULL == dir){
        perror("opendir");
        return -1;
    }
    struct dirent *ent;
    while(ent = readdir(dir)){
        //跳过'.''..'
        if(!strcmp(ent->d_name,".")||!strcmp(ent->d_name,"..")){
            continue;
        }
        mystat(ent->d_name, &st);
    }
    closedir(dir);
    return 0;
}
//显示文件元数据
int mystat(const char *path, struct stat *p_st){
    char f_type;
    int res = 0;
    res = stat(path, p_st);
    if(-1 == res){
        perror("stat");
        return -1;
    }
    //1.文件权限
    //文件类型
    switch(p_st->st_mode & S_IFMT){
        case S_IFSOCK:  f_type = 's';   break;
        case S_IFLNK:   f_type = 'l';   break;
        case S_IFREG:   f_type = '-';   break;
        case S_IFBLK:   f_type = 'b';   break;
        case S_IFDIR:   f_type = 'd';   break;
        case S_IFCHR:   f_type = 'c';   break;
        case S_IFIFO:   f_type = 'f';   break;
    }
    printf("%c", f_type);
    //属主,属组,其他权限
    char user[3] = {'-','-','-'};
    if(p_st->st_mode & S_IRUSR)    user[0] = 'r';
    if(p_st->st_mode & S_IWUSR)    user[1] = 'w';
    if(p_st->st_mode & S_IXUSR)    user[2] = 'x';
    char group[3] = {'-','-','-'};
    if(p_st->st_mode & S_IRGRP)    group[0] = 'r';
    if(p_st->st_mode & S_IWGRP)    group[1] = 'w';
    if(p_st->st_mode & S_IXGRP)    group[2] = 'x';
    char other[3] = {'-','-','-'};
    if(p_st->st_mode & S_IROTH)    other[0] = 'r';
    if(p_st->st_mode & S_IWOTH)    other[1] = 'w';
    if(p_st->st_mode & S_IXOTH)    other[2] = 'x';
    printf("%c%c%c%c%c%c%c%c%c",user[0],user[1],user[2],group[0],group[1],group[2],other[0],other[1],other[2]);
    //2.硬链接数
    printf(" %d ", p_st->st_nlink);
    //3.属主名
    struct passwd *owner = getpwuid(p_st->st_uid);
    printf("%s ", owner->pw_name);
    //4.属组名
    struct group *gp = getgrgid(p_st->st_gid);
    printf("%s ", gp->gr_name);
    //5.文件大小
    printf("%ld ", p_st->st_size);
    //6.最后访问时间
    struct tm *tm = localtime(&(p_st->st_mtime));
    printf(" %d月 %d %d:%d ", tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min);
    //7.文件名
    printf("%s\n", path);
    return 0;
}
