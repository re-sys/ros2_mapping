def RPM(x):
    return 450*x+571

# 用户输入
def main():
    print("RPM计算器")
    print("公式: RPM = 450 * x + 571")
    print("-" * 30)
    
    while True:
        try:
            # 获取用户输入
            user_input = input("请输入x值 (输入'q'退出): ")
            
            # 检查是否退出
            if user_input.lower() == 'q':
                print("程序退出")
                break
            
            # 转换为浮点数
            x = float(user_input)
            
            # 计算RPM
            rpm = RPM(x)
            
            # 输出结果
            print(f"x = {x}")
            print(f"RPM = 450 * {x} + 571 = {rpm:.2f}")
            print("-" * 30)
            
        except ValueError:
            print("输入错误！请输入一个有效的数字或'q'退出")
            print("-" * 30)
        except KeyboardInterrupt:
            print("\n程序被中断")
            break

if __name__ == "__main__":
    main()
