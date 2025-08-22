# send_fixed.py
import serial
import time
import sys
import os

# --- 配置参数 ---
FRAME_WIDTH = 256
FRAME_HEIGHT = 160
# 每帧数据的大小 (字节)
FRAME_SIZE_BYTES = int(FRAME_WIDTH * FRAME_HEIGHT / 2)
TARGET_FPS = 24 # 目标发送帧率

# --- 函数定义 ---

def find_serial_port():
    """自动查找可用的串口并让用户选择"""
    try:
        import serial.tools.list_ports
    except ImportError:
        print("错误: 未找到 'pyserial' 库。请运行 'pip install pyserial' 进行安装。")
        sys.exit(1)
        
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("错误：未找到任何串口设备。请检查STM32是否已连接并被识别为COM口。")
        sys.exit(1)
        
    print("可用的串口:")
    for i, port in enumerate(ports):
        print(f"  {i}: {port.device} - {port.description}")
    
    while True:
        try:
            choice = int(input("请选择要使用的串口编号: "))
            if 0 <= choice < len(ports):
                return ports[choice].device
            else:
                print("无效的选择，请重试。")
        except (ValueError, IndexError):
            print("无效的输入，请输入数字编号。")

# --- 主程序 ---

if __name__ == "__main__":
    # 1. 获取视频文件路径
    video_path = "./lagtrain.bin"
    if not (os.path.exists(video_path) and os.path.isfile(video_path)):
        print(f"错误：文件 '{video_path}' 不存在。")
        sys.exit(1)

    # 2. 查找并选择串口
    com_port = find_serial_port()

    try:
        # 3. 打开串口
        print(f"正在打开串口 {com_port}...")
        # 对于USB VCP，波特率通常是虚拟的，但设置一个高值是个好习惯。
        # 去掉读超时，因为我们不再读取任何数据。
        ser = serial.Serial(com_port, baudrate=12000000, timeout=None)
        print("串口已打开。")
    except serial.SerialException as e:
        print(f"打开串口失败: {e}")
        sys.exit(1)
        
    # 4. 打开视频文件
    try:
        video_file = open(video_path, 'rb')
    except IOError as e:
        print(f"错误：无法打开视频文件 '{video_path}': {e}")
        ser.close()
        sys.exit(1)
        
    # 计算每帧的延时
    frame_delay = 1.0 / TARGET_FPS
    
    print(f"\n准备就绪，将以 {TARGET_FPS} FPS 发送视频帧...")
    print("按 Ctrl+C 停止。")
    
    try:
        frame_count = 0
        total_bytes_sent = 0
        start_time = time.time()
        
        while True:
            frame_start_time = time.time()
            
            # 从.bin文件中读取一帧数据
            frame_data = video_file.read(FRAME_SIZE_BYTES)
            
            # 如果视频播放完毕，则从头开始
            if len(frame_data) < FRAME_SIZE_BYTES:
                print("\n视频播放完毕，从头开始循环。")
                video_file.seek(0)
                frame_data = video_file.read(FRAME_SIZE_BYTES)
                if len(frame_data) < FRAME_SIZE_BYTES:
                    print("错误：文件太小，无法读取完整一帧。")
                    break
            
            # --- 核心修改：直接发送，不再等待ACK ---
            # USB VCP有自己的流控机制，可以防止数据丢失。
            # 如果STM32来不及处理，ser.write会自然地阻塞。
            bytes_written = ser.write(frame_data)
            
            frame_count += 1
            total_bytes_sent += bytes_written

            # 控制帧率
            elapsed_time = time.time() - frame_start_time
            sleep_time = frame_delay - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # 打印进度
            current_run_time = time.time() - start_time
            if current_run_time > 0:
                 avg_speed = (total_bytes_sent / 1024) / current_run_time
            else:
                 avg_speed = float('inf')

            actual_fps = 1.0 / (time.time() - frame_start_time)
            print(f"已发送: {frame_count} 帧 | 速率: {actual_fps:.2f} FPS | 平均速度: {avg_speed:.2f} KB/s", end='\r')

    except KeyboardInterrupt:
        print("\n程序已停止。")
    except Exception as e:
        print(f"\n发生错误: {e}")
    finally:
        # 清理
        video_file.close()
        ser.close()
        print("\n串口已关闭。")