#!/usr/bin/env python3
# This python script file is not a part of this function package, and is to test whether the UART serial is functioning well.
# To run this script please type "python3 gray_sensor_test.py" in your console.
# Last edited by Chrono Black at 20260804.

import sys
import signal
import os
import time
import serial
import serial.tools.list_ports

def signal_handler(signal, frame):
    print("\nExiting...")
    sys.exit(0)

def serialTest():
    print("=== 可用串口设备列表 ===")
    os.system('ls /dev/tty[a-zA-Z]*')
    
    # 1. 获取串口设备名
    uart_dev = input("\n请输入要测试的串口设备名 (例如 /dev/ttyS1): ")

    # 2. 获取波特率
    baudrate = input("请输入波特率 (默认115200): ")
    if not baudrate:
        baudrate = 115200
    
    try:
        # 3. 打开串口
        # 注意：对于二进制数据，timeout设置短一些，比如0.5秒，避免接收时长时间阻塞
        ser = serial.Serial(uart_dev, int(baudrate), timeout=0.5)
        print(f"\n✅ 成功打开串口: {ser.name}")
    except Exception as e:
        print(f"❌ 打开串口失败: {e}")
        return 1

    print("🚀 开始测试！按 CTRL+C 退出")
    print("-" * 30)

    try:
        while True:
            # --- 发送阶段 ---
            # 4. 构造并发送启动指令
            # 直接使用 bytes 类型，不要用字符串的 encode，这是处理二进制数据最正确的方式
            cmd_to_send = bytes([0x57, 0x01]) 
            write_num = ser.write(cmd_to_send)
            print(f"📤 已发送 {write_num} 字节: {cmd_to_send.hex()}")

            # --- 接收阶段 ---
            # 5. 读取传感器返回的数据
            # 根据协议，一帧数据至少有 6 个字节 (0x57, 0x75, Data0, Data1?, 0x02/0x03)
            # 我们先尝试读取 10 个字节，确保能覆盖所有情况
            received_data = ser.read(10)
            
            if received_data:
                print(f"📥 收到 {len(received_data)} 字节: {received_data.hex()}")
                
                # 6. (可选) 简单解析数据
                if len(received_data) >= 3 and received_data[0] == 0x75 and received_data[2] == 0x02:
                    print("   ✅ 帧头正确！")
                    # 这里可以继续解析 Data0, Data1 和帧尾
                else:
                    print("   ⚠️ 数据格式可能不正确")
            else:
                print("📥 未收到数据 (超时)")

            print("-" * 30)
            time.sleep(1) # 每秒发送一次指令

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\n串口已关闭。")
    return 0

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    if serialTest() != 0:
        print("❌ 串口测试失败！")
    else:
        print("✅ 串口测试成功！")