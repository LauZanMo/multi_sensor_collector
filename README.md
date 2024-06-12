# multi_sensor_collector

## 依赖

- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.15
- [glog](https://github.com/google/glog)
- [Eigen](https://gitlab.com/libeigen/eigen)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [Intel Threaded Building Blocks (TBB)](https://github.com/oneapi-src/oneTBB)
- [HDF5](https://www.hdfgroup.org/downloads/hdf5/)
- [OpenCV](https://github.com/opencv/opencv) >= 4.2.0
- [OpenEB3](https://github.com/prophesee-ai/openeb)>=3.1.0

在Ubuntu20.04上，可以使用以下命令安装依赖：

```bash
sudo apt-get install apt-utils build-essential software-properties-common wget unzip curl git cmake libgoogle-glog-dev googletest libgtest-dev libboost-all-dev libusb-1.0-0-dev libeigen3-dev libyaml-cpp-dev libtbb-dev libopencv-dev libhdf5-dev libglew-dev libglfw3-dev libcanberra-gtk-module ffmpeg
```

额外的，OpenEB需要源码编译：

```bash
git clone https://github.com/prophesee-ai/openeb.git -b 3.1.2
cd openeb && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCOMPILE_PYTHON3_BINDINGS=OFF ..
make -j $(($(nproc) - 1))
sudo make install
sudo ldconfig
```

如果需要将hdf5转为rosbag，还需要安装额外的依赖：

- [ROS](https://ros.org/)
- [dvs_msgs](https://github.com/uzh-rpg/rpg_dvs_ros)

其中，dvs_msgs是rpg_dvs_ros下的一个子包，单独拷贝出来进行编译即可。

## 编译

如果仅想采集数据，可以直接按下述进行编译，如果还需要将hdf5文件转为rosbag，还需要在`cmake ..`指令中加入`-DBUILD_UTILS=ON`选项：

```bash
git clone https://github.com/LauZanMo/multi_sensor_collector
cd multi_sensor_collector && mkdir build && cd build
cmake ..
make -j $(($(nproc) - 1))
```

## 执行

可执行文件放在项目根目录的bin文件夹下，其中multi_sensor_collector为采集程序，采集的配置文件为config/suite.yaml，以下对参数进行介绍：

```yaml
log_path: "/home/ubuntu/dataset/output" # log输出路径

synchronizer:
    label: "SYNC" # 标签，没啥用
    port: "/dev/ttyUSB1" # INS-Probe同步串口的端口号
    baudrate: 115200 # 串口波特率
    timeout: 500 # 串口连接等待时间，单位为ms

sensors:
    -   sensor:
        model: "INS-Probe" # 传感器模型，会加载对应插件
        label: "IMU" # 标签，在HDF5文件中，该传感器的所有数据都会放这个路径下
        port: "/dev/ttyUSB0" # INS-Probe数据串口的端口号
        baudrate: 115200 # 串口波特率
        timeout: 500 # 串口连接等待时间，单位为ms
        rate: 200 # IMU频率，接收IMU数据等于这个量才会进行写入
    -   sensor:
        model: "EVK4-HD" # 传感器模型，会加载对应插件
        label: "CAM" # 标签，在HDF5文件中，该传感器的所有数据都会放这个路径下
        bias_file: "../config/evk4_hd.bias" # 事件相机的配置文件，参数需要调整在metavision_studio中操作（需要额外安装）
        rate: 20000000 # 事件流控制频率
        sync_rate: 10.0 # INS-Probe同步频率，该频率要在INS-Probe中配置
        acc_size: 20000 # 可视化的累积事件的数量，太多了会卡

data_writer:
    file_name: "/home/ubuntu/dataset/whu/20240109/test.h5" # 输出文件
```

h52bag为转换程序，转换的配置文件为config/h52bag.yaml，以下对参数进行介绍：

```yaml
log_path: "/home/ubuntu/dataset/output" # log输出路径

# 注意：这里的路径为批量处理数据集的总路径，且数据集名需要和文件夹名一致
# 举例：${dataset_path}----box01--box01.h5
#                      |--box02--box02.h5
#                      |--walk01--walk01.h5
#                      |--walk02--walk02.h5
datasets_path: "/home/ubuntu/dataset/whu/20240109"

bag_writer:
    sensors:
        -   sensor:
            label: "IMU" # 标签，在HDF5文件中，该传感器的所有数据都会放这个路径下
            topic: "/dvs/imu" # 写入rosbag中的话题
            type: "imu" # 写入信息的类型
            params: [] # 参数，暂无
        -   sensor:
            label: "CAM" # 标签，在HDF5文件中，该传感器的所有数据都会放这个路径下
            topic: "/dvs/events" # 写入rosbag中的话题
            type: "event" # 写入信息的类型
            params: ["1280", "720", "100.0"] # 参数，分别为分辨率宽、高和该信息发布的频率
```
