# openpose ros2

ubuntu22.04 + docker + cuda11.7.1 + cudnn8.5.0.96 + ros2

## prerequisites

## troubleshooting

```text
https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1567#issuecomment-619345906
```

```sh
xhost + local:
```

```sh
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext
```

```sh
# BUILD_PYTHON はなぜか GUI では ON にならない
cmake -DBUILD_PYTHON=ON ..
export PYTHONPATH="/root/workspace/openpose/build/python:$PYTHONPATH"
```

```sh
./build/examples/openpose/openpose.bin --video examples/media/video.avi --net_resolution "-1x256"
```

qt でエラーが出る場合

```sh
sudo apt install libxkbcommon-x11-0
```

```sh
sudo apt-get install libcanberra-gtk*
```
