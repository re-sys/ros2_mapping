#到篮板距离和转速的函数：
# 计算 RPM 的函数
def calculate_rpm(x):
    RPM = 436.262883 * x + 834.703062
    return RPM

# 用户输入
x_input = float(input("请输入 x 的值："))
rpm_result = calculate_rpm(x_input)

# 输出结果
print(f"对应的 RPM 是：{rpm_result:.2f}")