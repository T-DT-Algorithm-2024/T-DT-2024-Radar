int8 self_color #0是蓝 2是红
int16 match_time # 比赛时间，若尚未开始则发送比赛开始倒计时时间的负数，若比赛结束发送-100 若未连接至裁判系统发送-200
uint8[16] robot_hp #参考0x0003
uint8[6] marks
uint8 ultimate
uint32 eventtype