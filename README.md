# CppTool
C++工具包



# Boost安装

一、 下载boost

boost_1_51_0.zip 下载并解压到C盘根文件夹

二、编译boost
1、生成生命行程序
　　执行bootstrap.bat
2、编译
　　执行b2.exe,完成后显示：
The Boost C++ Libraries were successfully built!
The following directory should be added to compiler include paths:
    C:/boost_1_51_0
The following directory should be added to linker library paths:
    C:\boost_1_51_0\stage\lib

三、使用boost
1、创建一个win32 console
2、引用bootst
　　C/C++ -> Additional Include Directories: C:\boost_1_51_0
　　Linker-> Additional Library Directories: C:\boost_1_51_0\stage\lib
　　Linker->Input->Additional Dependencies :libboost_signals-vc110-mt-gd-1_51.lib;libboost_regex-vc110-mt-gd-1_51.lib;
3、Code如下：

```c++
#include "stdafx.h"
#include <boost/regex.hpp>
#include <boost/signals.hpp>
#include <boost/lambda/lambda.hpp>

#include <iostream>
#include <cassert>

struct print_sum {
  void operator()(int x, int y) const { std::cout << x+y << std::endl; }
};

struct print_product {

  void operator()(int x, int y) const { std::cout << x*y << std::endl; }

};

int _tmain(int argc, _TCHAR* argv[])
{
    boost::signal2<void, int, int, boost::last_value<void>, std::string> sig;
    sig.connect(print_sum());
    sig.connect(print_product());
    sig(3, 5);
    std::string line;    
    boost::regex pat( "^Subject: (Re: |Aw: )*(.*)" );   
    while (std::cin)
    {        
        std::getline(std::cin, line);        
        boost::smatch matches;       
        if (boost::regex_match(line, matches, pat)) 
            std::cout << matches[2] << std::endl;    
    }
    return 0;
}
```

示例程序在vs2012下通过，输出：

```c++
8
15
```

