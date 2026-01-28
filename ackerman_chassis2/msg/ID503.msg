# ackerman_chassis2/msg/ID503.msg
# 0x503底盘反馈报文完整解析
# 周期: 10ms, 波特率: 500k, 标准帧, 底层->上位机

# Byte0
int32 actual_speed              # 实际车速,单位:km/h

# Byte1
bool actual_epb_sts            # 实际电磁刹状态,0:关闭 1:开启
bool actual_back_sts           # 实际倒车灯状态,0:关闭 1:开启
