#!/bin/sh

# author: yueyin.zhou
# initial built date:2019-11-21

scriptpath=$(cd $(dirname $0); pwd)
exe_build() {
echo "<<<<<<build directory is: $1>>>>>>"
mkdir -p $1 
cd $1 
rm -rf *
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4 || exit "$$?";
cd $scriptpath/../ 
rm -rf $1 
}

echo -e "\033[42;37m **********************Start compile********************** \033[0m"
echo `date`

cd ..
#build exe
exe_build build

cp config/stereo_calib.json ../../bin/x86_64-linux/
cp -r ./data ../../bin/x86_64-linux/

echo -e "\033[42;37m **********************Compile finished!********************** \033[0m"
echo `date`
