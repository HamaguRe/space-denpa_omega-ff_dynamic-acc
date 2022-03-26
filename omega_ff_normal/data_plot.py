# グラフ作成

import csv
import matplotlib.pyplot as plt

# ---------- CSVファイルからデータ読み込み ----------- #
# 時刻
t = []
# オイラー角の真値
ypr = [[], [], []]
# オイラー角の推定値
ypr_hat = [[], [], []]
# 角速度バイアスの真値
gyr_bias = [[], [], []]
# 角速度バイアスの推定値
gyr_bias_hat = [[], [], []]
# 四元数の真値
q = [[], [], [], []]
# 四元数の推定値
q_hat = [[], [], [], []]
# 加速度外乱
a_dr = [[], [], []]

# CSVからデータを読み出して配列に追加
with open('./result.csv') as f:
    reader = csv.reader(f)
    for row in reader:
        nums = [float(v) for v in row]  # 文字列から浮動小数点数に変換

        # 時刻
        t.append(nums[0])
        # オイラー角
        for i in range(3):
            ypr[i].append(nums[i+1])
            ypr_hat[i].append(nums[i+4])
        # ジャイロバイアス
        for i in range(3):
            gyr_bias[i].append(nums[i+7])
            gyr_bias_hat[i].append(nums[i+10])
        # 四元数
        for i in range(4):
            q[i].append(nums[i+13])
            q_hat[i].append(nums[i+17])
        # 加速度外乱
        for i in range(3):
            a_dr[i].append(nums[i+21])


# ----------------- グラフ描画の準備 ------------------ #
# Figureを追加
fig1 = plt.figure(figsize = (13, 7))
plt.suptitle('Omega Feedback Filter (Normal)',fontsize=20)

ax1 = fig1.add_subplot(331, ylabel='X axis (Roll)', title='Euler angles [rad]')
ax2 = fig1.add_subplot(334, ylabel='Y axis (Pitch)')
ax3 = fig1.add_subplot(337, ylabel='Z axis (Yaw)', xlabel='time [s]')
ax4 = fig1.add_subplot(332, title='Bias of Angular velocity [rad/s]')
ax5 = fig1.add_subplot(335)
ax6 = fig1.add_subplot(338, xlabel='time [s]')
ax7 = fig1.add_subplot(333, title='Acceleration disturbance [m/s^2]')
ax8 = fig1.add_subplot(336)
ax9 = fig1.add_subplot(339, xlabel='time [s]')

ax1.plot(t, ypr[2],     label="True", color="black")
ax1.plot(t, ypr_hat[2], label="Estimated", color="red", linestyle = "--")
ax2.plot(t, ypr[1],     label="True", color="black")
ax2.plot(t, ypr_hat[1], label="Estimated", color="red", linestyle = "--")
ax3.plot(t, ypr[0],     label="True", color="black")
ax3.plot(t, ypr_hat[0], label="Estimated", color="red", linestyle = "--")
ax4.plot(t, gyr_bias[0],     label="True", color="black")
ax4.plot(t, gyr_bias_hat[0], label="Estimated", color="red", linestyle = "--")
ax5.plot(t, gyr_bias[1],     label="True", color="black")
ax5.plot(t, gyr_bias_hat[1], label="Estimated", color="red", linestyle = "--")
ax6.plot(t, gyr_bias[2],     label="True", color="black")
ax6.plot(t, gyr_bias_hat[2], label="Estimated", color="red", linestyle = "--")
ax7.plot(t, a_dr[0], label="True", color="black")
ax8.plot(t, a_dr[1], label="True", color="black")
ax9.plot(t, a_dr[2], label="True", color="black")
ax1.legend()
ax4.legend()
ax7.legend()
# 表示範囲 x軸
ax1.set_xlim(0, t[-1])
ax2.set_xlim(0, t[-1])
ax3.set_xlim(0, t[-1])
ax4.set_xlim(0, t[-1])
ax5.set_xlim(0, t[-1])
ax6.set_xlim(0, t[-1])
ax7.set_xlim(0, t[-1])
ax8.set_xlim(0, t[-1])
ax9.set_xlim(0, t[-1])
# 表示範囲 y軸
euler_range = 3.5
bias_range = 0.08
ax1.set_ylim(-euler_range, euler_range)
ax2.set_ylim(-euler_range, euler_range)
ax3.set_ylim(-euler_range, euler_range)
ax4.set_ylim(-bias_range, bias_range)
ax5.set_ylim(-bias_range, bias_range)
ax6.set_ylim(-bias_range, bias_range)

# 四元数だけ別ウィンドウで表示
fig2 = plt.figure(figsize = (7, 7))
plt.suptitle('Quaternion',fontsize=20)
ax1 = fig2.add_subplot(411, ylabel='q0')
ax2 = fig2.add_subplot(412, ylabel='q1')
ax3 = fig2.add_subplot(413, ylabel='q2')
ax4 = fig2.add_subplot(414, xlabel='time [s]', ylabel='q3')
ax1.plot(t, q[0],    label="True", color="black")
ax1.plot(t, q_hat[0], label="Estimated", color="red", linestyle = "--")
ax2.plot(t, q[1],    label="True", color="black")
ax2.plot(t, q_hat[1], label="Estimated", color="red", linestyle = "--")
ax3.plot(t, q[2],    label="True", color="black")
ax3.plot(t, q_hat[2], label="Estimated", color="red", linestyle = "--")
ax4.plot(t, q[3],    label="True", color="black")
ax4.plot(t, q_hat[3], label="Estimated", color="red", linestyle = "--")
ax1.legend()
ax1.set_ylim(-1.2, 1.2)
ax2.set_ylim(-1.2, 1.2)
ax3.set_ylim(-1.2, 1.2)
ax4.set_ylim(-1.2, 1.2)

plt.show()