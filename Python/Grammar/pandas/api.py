import numpy as np
import pandas as pd

'''
DataFrame.where（cond, other=nan, inplace=False, axis=None, level=无，try_cast=否，发生错误时触发）
inplace：布尔类型，默认为False。是否对数据执行就地操作
axis：如果需要，则设置对齐轴，默认无。
level：对齐级别，默认无
Try_cast：布尔型，默认False 尝试将结果强制转换回输入类型（如果可能的话），
raise_on_error：布尔类型，默认为True 是否提高无效数据类型(例如。试图在弦上到哪里去)
另见DataFrame.mask()

注意事项
where方法是一个if-then习语的应用。对于调用DataFrame中的每个元素，如果cond为True，则使用元素；否则使用DataFrameother中的对应元素。
DataFrame.where()的签名与numpy.where()的签名不一致。粗略的df1.where（m，df2）相当于np.where（m，df1，df2）。
有关更多详细信息和示例，请参阅索引编制中的where文档。
'''

'''where方法是一个if-then习语的应用。对于调用DataFrame中的每个元素，如果cond为True，则使用元素；否则使用DataFrameother中的对应元素。'''
s = pd.Series(range(5))
# print(s)
print(s.where(s > 2))

df = pd.DataFrame(np.arange(10).reshape(-1, 2), columns=['A', 'B'])
print(df)
m = df % 3 == 0
print(df.where(m, -df))
