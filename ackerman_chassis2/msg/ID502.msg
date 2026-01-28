# ackerman_chassis2/msg/ID502.msg
# 0x502底盘反馈报文完整解析
# 周期: 10ms, 波特率: 500k, 标准帧, 底层->上位机

# Byte0: 状态与灯光
uint8 actual_status   # Bit0-1: 实际车辆状态 (0:人工 1:遥控器 2:自动驾驶 3:紧急刹车)
uint8 actual_gear               # Bit2-3: 实际车辆档位 (1:D 2:N 3:R)
string actual_gear_str          # 档位信息 (1:D 2:N 3:R)
bool actual_left_light          # Bit4: 实际左转向灯 (0:关 1:开)
bool actual_right_light         # Bit5: 实际右转向灯 (0:关 1:开)
bool actual_head_light          # Bit6: 实际近光灯   (0:关 1:开)
bool actual_brake_light         # Bit7: 实际刹车灯   (0:关 1:开)

# Byte1-2: 电驱转速
int16 actual_moto_spd           # 实际电驱转速 (rpm) = (Byte1 << 8) | Byte2

# Byte3-4: 方向盘角度
int16 actual_steering_angle     # 实际方向盘角度 (度) = ((Byte3 << 8) | Byte4) - 1024

# Byte5: 刹车压力
uint8 actual_brake_pressure     # 实际刹车压力 (bar) 0~80

# Byte6: 通信超时状态
bool can0_pc501_timeout         # Bit0: CAN0接收PC0x501报文超时 (0:正常 1:超时)
bool can0_rmc301_timeout        # Bit1: CAN0接收RMC0x301报文超时 (0:正常 1:超时)
bool can1_eps18f_timeout        # Bit2: CAN1接收EPS_0x18f报文超时 (0:正常 1:超时)
bool can1_ebs142_timeout        # Bit3: CAN1接收EBS_0x142报文超时 (0:正常 1:超时)
bool can2_motoa10f8109a_timeout # Bit4: CAN2接收MotoA0x10F8109A报文超时 (0:正常 1:超时)

# Byte7: 故障状态
bool steer_error                # Bit0: 转向故障 (0:正常 1:故障)
bool brake_error                # Bit1: 制动故障 (0:正常 1:故障)
bool moto_error                 # Bit2: 驱动故障 (0:正常 1:故障)
