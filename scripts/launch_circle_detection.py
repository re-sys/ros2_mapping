#!/usr/bin/env python3

import subprocess
import sys
import time
import signal
import os

def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    print("\n正在关闭所有进程...")
    sys.exit(0)

def main():
    """主函数"""
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    print("启动点云圆形检测系统...")
    print("按Ctrl+C停止所有进程")
    
    try:
        # 启动点云处理器节点
        print("启动点云处理器节点...")
        processor_process = subprocess.Popen([
            sys.executable, 
            os.path.join(os.path.dirname(__file__), 'pointcloud_processor.py')
        ])
        
        # 等待一下确保处理器节点启动
        time.sleep(2)
        
        # 启动图像显示节点
        print("启动图像显示节点...")
        display_process = subprocess.Popen([
            sys.executable, 
            os.path.join(os.path.dirname(__file__), 'image_display_node.py')
        ])
        
        print("所有节点已启动")
        print("等待点云数据...")
        
        # 等待进程结束
        while True:
            # 检查进程是否还在运行
            if processor_process.poll() is not None:
                print("点云处理器节点已退出")
                break
                
            if display_process.poll() is not None:
                print("图像显示节点已退出")
                break
                
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭进程...")
    except Exception as e:
        print(f"启动过程中出错: {e}")
    finally:
        # 清理进程
        try:
            if 'processor_process' in locals():
                processor_process.terminate()
                processor_process.wait(timeout=5)
        except:
            pass
            
        try:
            if 'display_process' in locals():
                display_process.terminate()
                display_process.wait(timeout=5)
        except:
            pass
            
        print("所有进程已关闭")

if __name__ == '__main__':
    main() 