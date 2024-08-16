#!/bin/bash

# 当接收到SIGINT信号时，结束脚本执行
trap "echo 'Script terminated by user'; exit" SIGINT
export time=$(date +%Y-%m-%d-%H-%M-%S)
source /home/tdt/.zshrc

# 获取脚本所在的绝对路径
SCRIPT_PATH=$(dirname $(realpath $0))
cd $SCRIPT_PATH/..
echo $SCRIPT_PATH

# 设置程序路径
PROGRAM="./src/tdt_vision/launch/radar.launch.py"

source ./install/setup.zsh
echo "to be launched"
sleep 5
while true
do
    # 检查程序是否正在运行
    if pgrep -f $PROGRAM > /dev/null
    then
        echo "$PROGRAM is running~"
    else
        echo "$PROGRAM launch failed!"

        # 启动程序
        ros2 launch $PROGRAM
    fi

    # 等待一段时间再次检查
    sleep 1
done
