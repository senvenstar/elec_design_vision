import matplotlib.pyplot as plt
import numpy as np

# 请替换为你的实际文件路径
filename = 'data.txt'

# 存储时间和数据
times = []
x_data, vx_data = [], []
y_data, vy_data = [], []
z_data, vz_data = [], []

with open(filename, 'r') as file:
    lines = file.readlines()

i = 0
while i < len(lines):
    line = lines[i].strip()
    if '<MSG>:' in line:
        try:
            # 解析时间：格式 00:03:225
            time_str = line.split('<MSG>:')[0].strip()
            min_sec_msec = time_str.split(':')
            minutes = int(min_sec_msec[0])
            seconds = int(min_sec_msec[1])
            milliseconds = int(min_sec_msec[2])
            total_seconds = minutes * 60 + seconds + milliseconds / 1000.0

            # 读取接下来的 6 行数据
            x = float(lines[i+1].strip())
            vx = float(lines[i+2].strip())
            y = float(lines[i+3].strip())
            vy = float(lines[i+4].strip())
            z = float(lines[i+5].strip())
            vz = float(lines[i+6].strip())

            # 存入列表
            times.append(total_seconds)
            x_data.append(x)
            vx_data.append(vx)
            y_data.append(y)
            vy_data.append(vy)
            z_data.append(z)
            vz_data.append(vz)

            i += 7
        except Exception as e:
            print(f"解析第 {i+1} 行时出错: {e}")
            i += 1
    else:
        i += 1

# 检查是否读取到数据
if len(times) == 0:
    print("❌ 未读取到有效数据，请检查文件格式。")
else:
    print(f"✅ 成功读取 {len(times)} 组数据。")

    # 创建 3 个子图
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))
    fig.suptitle("Position and Velocity (Zero-Aligned)", fontsize=16)

    def align_y_axes(ax1, data1, color1, label1, ax2, data2, color2, label2):
        """对齐两个 y 轴，使 0 对齐，且单位高度对应相同数值变化"""
        # 计算两个数据的绝对最大值
        max1 = np.max(np.abs(data1))
        max2 = np.max(np.abs(data2))
        if max1 == 0: max1 = 1
        if max2 == 0: max2 = 1

        # 设定一个“视觉比例因子”，让两轴在图上占据相同高度
        scale = max1 / max2

        # 设置 ax2 的 ylim，使其与 ax1 的缩放一致（0 对齐）
        ax1.set_ylim(-max1, max1)
        ax2.set_ylim(-max2 * scale, max2 * scale)

        # 绘制数据
        ax1.plot(times, data1, color=color1, marker='o', markersize=2, linewidth=1, label=label1)
        ax2.plot(times, data2, color=color2, marker='s', markersize=2, linewidth=1, label=label2)

        # 设置标签
        ax1.set_ylabel(label1, color=color1)
        ax2.set_ylabel(label2, color=color2)

        # 合并图例
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    # 第一张图：x 和 vx
    ax1 = axs[0]
    ax1_vx = ax1.twinx()
    align_y_axes(ax1, x_data, 'red', 'x', ax1_vx, vx_data, 'blue', '$v_x$')

    # 第二张图：y 和 vy
    ax2 = axs[1]
    ax2_vy = ax2.twinx()
    align_y_axes(ax2, y_data, 'green', 'y', ax2_vy, vy_data, 'orange', '$v_y$')

    # 第三张图：z 和 vz
    ax3 = axs[2]
    ax3_vz = ax3.twinx()
    align_y_axes(ax3, z_data, 'purple', 'z', ax3_vz, vz_data, 'cyan', '$v_z$')

    # 统一设置 x 标签
    for ax in axs:
        ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()