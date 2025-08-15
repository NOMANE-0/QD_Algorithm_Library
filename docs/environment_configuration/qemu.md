# QEMU

一个另类替代交叉编译的方法

QEMU 是一款免费开源的模拟器。我们将使用它来模拟 arm 架构，以便在 x86 电脑上编译出 arm 架构的程序

```bash
sudo apt install qemu-user-static
docker run --rm --privileged multiarch/qemu-user-static:register --reset
docker pull --platform arm64 osrf/ros:humble-desktop
docker run -it --platform arm64 \
    -v  ~/armHumble:/home \
    --network host \
    -v /usr/bin/qemu-aarch64-static:/usr/bin/qemu-aarch64-static \
    --name armHumble \
    arm64v8/ros:humble /bin/bash
```
