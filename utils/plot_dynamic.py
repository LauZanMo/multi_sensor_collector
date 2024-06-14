import numpy as np
import matplotlib.pyplot as plt

datasets_path = "/home/ubuntu/dataset/whu/event_dataset"
prefix = "walk_fast0"
dataset_names = ["walk_fast01", "walk_fast02", "walk_fast03"]
fig_name = "walk_fast01-03_imu_mag"
output_path = "/home/ubuntu/视频/evins实验"

mag_stamps = []
gyro_mags = []
acc_mags = []
for name in dataset_names:
    imu_data = np.loadtxt(datasets_path + "/" + name + "/" + name + "_IMU.txt")
    mag_stamps.append(imu_data[:, 0] - imu_data[0, 0])
    gyro_mag = np.linalg.norm(np.rad2deg(imu_data[:, 1:4]), axis=1)
    gyro_mags.append(gyro_mag)
    acc_mag = np.abs(np.linalg.norm(imu_data[:, 4:7], axis=1) - 9.8)
    acc_mags.append(acc_mag)

plt.figure()
plt.subplot(211)
for i in range(len(dataset_names)):
    plt.plot(mag_stamps[i], gyro_mags[i], label=prefix + str(i + 1))
plt.legend(loc="upper right")
plt.grid()
plt.xlabel("时间 (s)")
plt.ylabel("角速度幅值 (deg/s)")

plt.subplot(212)
for i in range(len(dataset_names)):
    plt.plot(mag_stamps[i], acc_mags[i], label=prefix + str(i + 1))
plt.legend(loc="upper right")
plt.grid()
plt.xlabel("时间 (s)")
plt.ylabel("(比力-重力)幅值 (m/s^2)")

plt.tight_layout()
plt.savefig(output_path + "/" + fig_name + ".png")
plt.show()
