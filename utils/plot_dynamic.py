import numpy as np
import matplotlib.pyplot as plt

datasets_path = "/home/ubuntu/dataset/whu/20240109"
dataset_names = ["box01", "box02", "box03"]
fig_name = "imu_mag_box1-3"


mag_stamps = []
gyro_mags = []
acc_mags = []
for name in dataset_names:
    imu_data = np.loadtxt(datasets_path + "/" + name + "/" + name + "_IMU.txt")
    mag_stamps.append(imu_data[:, 0] - imu_data[0, 0])
    gyro_mag = np.linalg.norm(np.rad2deg(imu_data[:, 1:4]), axis=1)
    gyro_mags.append(gyro_mag)
    acc_mag = np.linalg.norm(imu_data[:, 4:7], axis=1) - 9.8
    acc_mags.append(acc_mag)


plt.figure()
plt.subplot(211)
for i in range(len(dataset_names)):
    plt.plot(mag_stamps[i], gyro_mags[i], label=dataset_names[i])
plt.legend()
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Angular Rate Magnitude (deg/s)")


plt.subplot(212)
for i in range(len(dataset_names)):
    plt.plot(mag_stamps[i], acc_mags[i], label=dataset_names[i])
plt.legend()
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Acceleration Magnitude (m/s^2)")


plt.tight_layout()
plt.savefig(datasets_path + "/output/" + fig_name + ".png")
plt.show()
