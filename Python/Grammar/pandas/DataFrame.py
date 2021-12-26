import pandas as pd
import numpy as np

data = {'name': ['Joe', 'Mike', 'Jack', 'Rose', 'David', 'Marry', 'Wansi', 'Sidy', 'Jason', 'Even'],
        'age': [25, 32, 18, np.nan, 15, 20, 41, np.nan, 37, 32],
        'gender': [1, 0, 1, 1, 0, 1, 0, 0, 1, 0],
        'isMarried': ['yes', 'yes', 'no', 'yes', 'no', 'no', 'no', 'yes', 'no', 'no']}

labels = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j']
df = pd.DataFrame(data, index=labels)
# print(df)

# =============================
# 1）行（列）选取（单维度选取）：df[]。这种情况一次只能选取行或者列，即一次选取中，只能为行或者列设置筛选条件（只能为一个维度设置筛选条件）。
# a）整数索引切片：前闭后开
print(df[0:2])

# b）标签索引切片：前闭后闭
print(df['a':'b'])

# c）布尔数组
# print(df[[True, True, True, False, False, False, False, False, False, False]])
# print(df[[each > 30 for each in df['age']]])
# 注意：像上面这种通过多个布尔条件判断的情况，多个条件最好（一定）用括号括起来，否则非常容易出错。
print(df[(df['age'] > 30) & (df['isMarried'] == 'no')])

# 2)列选取
print(df['name'])
print(df[['name', 'age']])
print(df[lambda df: df.columns[0]])

# =============================
# 2）区域选取（多维选取）：df.loc[]，df.iloc[]，df.ix[]。这种方式可以同时为多个维度设置筛选条件。
'''
3 区域选取
　　区域选取可以从多个维度（行和列）对数据进行筛选，可以通过df.loc[]，df.iloc[]，df.ix[]三种方法实现。采用df.loc[]，df.iloc[]，df.ix[]这三种方法进行数据选取时，方括号内必须有两个参数，第一个参数是对行的筛选条件，第二个参数是对列的筛选条件，两个参数用逗号隔开。df.loc[]，df.iloc[]，df.ix[]的区别如下：
　　df.loc[]只能使用标签索引，不能使用整数索引，通过便签索引切边进行筛选时，前闭后闭。
　　df.iloc[]只能使用整数索引，不能使用标签索引，通过整数索引切边进行筛选时，前闭后开。；
　　df.ix[]既可以使用标签索引，也可以使用整数索引。
　　下面分别通过实例演示这三种方法。
'''
# 选取索引为‘a’的行：
print(df.loc['a', :])
# 选取索引为‘a’或‘b’或‘c’的行
print(df.loc[['a', 'b', 'c'], "age"])
# 输出所有人的姓名和年龄（选取name和age列）
print(df.loc[:, ['name', 'age', 'isMarried']])

# =============================
# 3）单元格选取（点选取）：df.at[]，df.iat[]。准确定位一个单元格。
'''
4 单元格选取
　　单元格选取包括df.at[]和df.iat[]两种方法。
    df.at[]和df.iat[]使用时必须输入两个参数，即行索引和列索引，其中df.at[]只能使用标签索引，df.iat[]只能使用整数索引。
    df.at[]和df.iat[]选取的都是单个单元格（单行单列），所以返回值都为基本数据类型。
'''
print(df.at['b', 'name'])
print(df.iat[1, 0])

'''
5 拓展与总结
　　1）选取某一整行（多个整行）或某一整列（多个整列）数据时，可以用df[]、df.loc[]、df.iloc[]，此时df[]的方法书写要简单一些。
　　2）进行区域选取时，如果只能用标签索引，则使用df.loc[]或df.ix[]，如果只能用整数索引，则用df.iloc[]或df.ix[]。
        不过我看到有资料说，不建议使用df.ix[],因为df.loc[]和df.iloc[]更精确（有吗？我没理解精确在哪，望告知）。
　　3）如果选取单元格，则df.at[]、df.iat[]、df.loc[]、df.iloc[]都可以，不过要注意参数。　　
　　4）选取数据时，返回值存在以下情况：

如果返回值包括单行多列或多行单列时，返回值为Series对象；
如果返回值包括多行多列时，返回值为DataFrame对象；
如果返回值仅为一个单元格（单行单列）时，返回值为基本数据类型，例如str，int等。
　　5）df[]的方式只能选取行和列数据，不能精确到单元格，所以df[]的返回值一定DataFrame或Series对象。
　　6）当使用DataFrame的默认索引（整数索引）时，整数索引即为标签索引。例如，使用上面的data实例化一个DataFrame对象：
'''
