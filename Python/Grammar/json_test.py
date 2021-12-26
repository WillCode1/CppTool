#!/usr/bin/env python3
# -*- coding: utf-8 -*-

""" python转json方法 """

__author__ = 'Wei Di'

import json

test_dict = {'name': 'luotianshuai', 'age': 18}
test_dict = {'d': dict(name='Hello'), 'str': 'hello', 'arr': [1, 2, "a"], "f": True, "None": None}
print('未dumps前类型为:', type(test_dict))

# dumps 将数据通过特殊的形式转换为所有程序语言都识别的字符串
json_str = json.dumps(test_dict, indent=4)
print('dumps后的类型为:', type(json_str))

# loads 将字符串通过特殊的形式转为python数据类型
new_dic = json.loads(json_str)
print('重新loads加载为数据类型:', type(new_dic))
print('*' * 50)

# dump 将数据通过特殊的形式转换为所有语言都识别的字符串并写入文件
with open("sample.json", 'w') as openfile:
    json.dump(test_dict, openfile, indent=4)
#    json.dump(new_dic, openfile, indent=4)

# load 从文件读取字符串并转换为python的数据类型
with open("sample.json", 'rb') as loadfile:
    load_dic = json.load(loadfile)
    print('load 并赋值给load_dic后的数据类型:', type(load_dic))
