# Livox雷达驱动

## 安装livox雷达驱动

```terminal
# 安装依赖包CMAKE
sudo apt install cmake

# 下载Livox-SDK文件
git clone https://github.com/Livox-SDK/Livox-SDK2.git

cd Livox-SDK2
mkdir build && cmake ..
make
sudo make install
```

## 3D地图查看

```terminal
sudo apt install pcl-tools
```

