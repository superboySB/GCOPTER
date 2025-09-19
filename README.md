# 学习笔记
## 配置
```sh
docker build -f docker/simulation.dockerfile -t gcopter_image:v0 --network=host --progress=plain .

docker run --name gcopter-sim -itd --privileged --gpus all --network=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /home/dzp/projects/GCOPTER:/workspace/ROS/src/GCOPTER
    -e DISPLAY=$DISPLAY \
    -e LOCAL_USER_ID="$(id -u)" \
    gcopter_image:v0 /bin/bash

docker exec -it gcopter-sim /bin/bash

cpufreq-set -g performance 
```