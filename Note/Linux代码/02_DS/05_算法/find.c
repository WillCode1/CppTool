//二分查找算法的实现
#include <stdio.h>

int find(int arr[],int left,int right,int data)
{
    // :8,24 > 表示向右缩进
    if(left <= right)
    {
        //1.计算中间元素的下标
        int p = (left+right)/2; 
        //2.使用中间元素和目标元素进行比较，如果相等，则直接返回下标
        if(arr[p] == data)
        {
            return p;
        }
        //3.如果中间元素小于目标元素，则去中间元素的右侧继续查找
        else if(arr[p] < data)
        {
            return find(arr,p+1,right,data);
        }
        //4.如果中间元素大于目标元素,则去中间元素的左侧继续查找
        else
        {
            return find(arr,left,p-1,data);
        }
    }
    //5.比较完毕，查找失败
    return -1;

}

int main(void)
{
    int arr[9] = {11,22,33,44,55,66,77,88,99};
    //调用查找函数查找元素33,返回下标
    printf("元素33所在的下标是：%d\n",find(arr,0,8,33)); // 2
    printf("元素60所在的下标是：%d\n",find(arr,0,8,60)); // -1
    return 0;
}
