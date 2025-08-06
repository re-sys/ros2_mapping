#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

# 数据点
points = [
    (6.6, 3590),
    (6.2, 3390),
    (5.8, 3190),
    (5.4, 3010),
    (5.0, 2830),
    (4.6, 2650),
    (4.2, 2470),
    (3.8, 2290),
    (3.4, 2110),
    (3.0, 1930)
]

# 分离x和y坐标
x_coords = [point[0] for point in points]
y_coords = [point[1] for point in points]

# 线性拟合
slope, intercept, r_value, p_value, std_err = stats.linregress(x_coords, y_coords)

# 创建拟合线的点
x_line = np.linspace(min(x_coords), max(x_coords), 100)
y_line = slope * x_line + intercept

# 创建图形
plt.figure(figsize=(10, 8))

# 绘制数据点
plt.scatter(x_coords, y_coords, color='red', s=100, label='数据点', zorder=5)

# 标注每个点
for i, (x, y) in enumerate(points):
    plt.annotate(f'点{i+1}\n({x}, {y})', 
                xy=(x, y), 
                xytext=(10, 10),
                textcoords='offset points',
                ha='left',
                va='bottom',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7),
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))

# 绘制拟合直线
plt.plot(x_line, y_line, color='blue', linewidth=2, label=f'拟合直线: y = {slope:.2f}x + {intercept:.2f}')

# 设置图形属性
plt.xlabel('X坐标', fontsize=12)
plt.ylabel('Y坐标', fontsize=12)
plt.title('6个数据点的线性拟合', fontsize=14)
plt.grid(True, alpha=0.3)
plt.legend()

# 显示拟合参数
plt.text(0.05, 0.95, f'斜率: {slope:.2f}\n截距: {intercept:.2f}\n相关系数: {r_value:.4f}', 
         transform=plt.gca().transAxes, 
         verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# 调整布局
plt.tight_layout()

# 保存图片
plt.savefig('points_and_fit_line.png', dpi=300, bbox_inches='tight')
print(f"图片已保存为 'points_and_fit_line.png'")

# 显示图形
plt.show()

# 打印拟合结果
print(f"\n拟合结果:")
print(f"斜率: {slope:.4f}")
print(f"截距: {intercept:.4f}")
print(f"相关系数: {r_value:.4f}")
print(f"方程: y = {slope:.4f}x + {intercept:.4f}")

# 计算预测值
print(f"\n预测值:")
for x in x_coords:
    y_pred = slope * x + intercept
    print(f"x={x}, y预测={y_pred:.2f}") 