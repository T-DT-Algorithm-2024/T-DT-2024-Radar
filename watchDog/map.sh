#!/bin/bash

# 当接收到SIGINT信号时，结束脚本执行
trap "echo 'Script terminated by user'; exit" SIGINT

# 获取脚本所在的绝对路径
SCRIPT_PATH=$(dirname $(realpath $0))
cd $SCRIPT_PATH/..

source ./install/setup.bash

# 设置程序路径
PROGRAM="ros2 run debug_map debug_map"

while true
do
    # 检查程序是否正在运行
    if pgrep -f "$PROGRAM" > /dev/null
    then
        echo "$PROGRAM is running~"
    else
        echo "$PROGRAM launch failed!"

        # 启动程序
        $PROGRAM
    fi

    # 等待一段时间再次检查
    sleep 1
done
