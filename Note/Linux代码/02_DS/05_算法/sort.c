//各种排序算法的使用
#include <stdio.h>

//冒泡排序函数
void bubble(int arr[],int len)
{
    //外层控制比较的轮数
    int i = 0,j = 0;
    for(i = 1; i < len; i++)
    {
        //内层循环控制针对当前轮数的下标访问
        for(j = 0; j < len-i; j++)
        {
            //左边元素 大于 右边 交换
            if(arr[j] > arr[j+1])
            {
                int temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}

//插入排序算法的实现
void insert(int arr[],int len)
{
    //1.从第二个元素起，依次取出每个元素
    int i = 0,j = 0;
    for(i = 1; i < len; i++)
    {
        //保存取出的元素
        int temp = arr[i];
        //2.使用取出的元素和左边有序元素比较，如果左边元素大于取出元素，则左边元素右移
        for(j = i; arr[j-1]>temp && j>=1; j--)
        {
            arr[j] = arr[j-1];
        }
        //3.直到左边元素小于取出元素，或者左边不再有元素，则将取出元素插入
        //防止自己给自己赋值,优化
        if(i != j)
        {
            arr[j] = temp;
        }
    }

}

//选择排序算法的实现
void choose(int arr[],int len)
{
    //1.从第一个元素起依次取出，记录下标
    int i = 0,j = 0;
    for(i = 0; i < len-1; i++)
    {
        int min = i;
        //2.使用记录的最小值与后续元素比较，如果后续元素小于记录的最小值，则重新记录下标
        for(j = i+1; j < len; j++)
        {
            if(arr[j] < arr[min])
            {
                    min = j;
            }
        }
        //3.直到记录的最小值与后续元素比较完毕，则交换记录的最小值和最开始假定的最小值
        //一开始假定的最小值确实是这组数中最小的
        if(i != min)
        {
            int temp = arr[i];
            arr[i] = arr[min];
            arr[min] = temp;
        }
    }
}

//快速排序算法的实现
void quick(int arr[],int left,int right)
{
    //1.根据左边元素和右边元素下标计算中间元素的下标
    int p = (left+right)/2;
    //2.选择中间元素作为基准值，保存
    int pivot = arr[p];
    //3.分别使用左右两边的元素与基准值进行比较
    int i = 0,j = 0;
    for(i = left,j = right; i < j; )
    {
        //首先使用左边元素与基准值比较,如果左边元素小于基准值,则比较下一个元素
        while(arr[i] < pivot && i < p)
        {
            i++;
        }
        //如果左边元素大于基准值,则将左边的元素赋值给p指向的位置,p指向数据来源的位置
        if(i < p)
        {
            arr[p] = arr[i];
            p = i;
        }
        //右边元素方法同上
        while(arr[j] >= pivot && j > p)
        {
            j--;
        }
        if(j > p)
        {
            arr[p] = arr[j];
            p = j;
        }
    }
    //当i 和 j 相等时,将基准值放到重合的位置上
    arr[p] = pivot;
    //分别对左边元素和右边元素进行递归排序
    if(p - left > 1)
    {
        quick(arr,left,p-1);
    }
    if(right - p > 1)
    {
        quick(arr,p+1,right);
    }
}

int main(void)
{
    int arr[9] = {20,8,25,3,15,10,30,5,22};
    //调用排序算法进行排序
    //bubble(arr,9);
    //insert(arr,9);
    //choose(arr,9);
    quick(arr,0,8);
    //打印排序后的结果
    int i = 0;
    printf("排序后的结果是：");
    for(i = 0; i < 9; i++)
    {
        printf("%d ",arr[i]);
    }
    printf("\n");
    return 0;
}

