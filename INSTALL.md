# Yanu simulator with isaac sim

## Summary
*This section is to be filled out by the project manager based on the [summary document](https://docs.google.com/spreadsheets/d/12xi2yOMm-X5PEecgyRe3WEurcSaN9A5z4DHgBacQT6M).*

| Company | [Yanu](https://yanu.ai/) |
| :--- | :--- |
| Project Manager | [Project Manager](https://profile.link) |
| Project Team | [Kristjan Laht](https://github.com/KingBoomie); [Houman Masnavi](https://www.linkedin.com/in/houman-masnavi/?originalSubdomain=ee) |
| Challenge Tackled | 3d model optimization;  |
| Technology Used | nvidia isaac robot simulator |
| Lessons Learned | link explains well how sim and real are two very very different environments https://cfvod.kaltura.com//api_v3//index.php//service//attachment_attachmentAsset//action//serve//attachmentAssetId//1_0c9c81g7//ks//djJ8MjkzNTc3MXxEgkLwxL1prkl_vPyZp8uOVHRh4OkIyYSQh5bkF9LPXFHSVaogGkeoPYMNmtz8zYTxjMwFzeQRvofwuPSAJeCYNarZmBMSc0anFEnczRLLZqU7YWjx11mchibaSnYLoWpXwkvekMDI6kuLzZxNBEmsuBRfnlFpWSctMWo8_Xg-pTeLdbEe23LxDHXnwAi806BdanyAptcmZFr5ykD6PUPu |
| Result Published |  |
| Target Group |  |
| Diagrams/Photos |  |
| Video |  |


## Installation

1. Follow the instructions [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) to 
get the isaac container. 

2. Get an interactive shell in the container:
```docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    nvcr.io/nvidia/isaac-sim:2022.2.1
```

3. Start isaac sim with native livestream mode:
```
./runheadless.native.sh
```

4. git clone this repo to `~/docker/isaac-sim/documents/Kit/apps/Isaac-Sim/scripts`:
```bash
cd ~/docker/isaac-sim/documents/Kit/apps/Isaac-Sim/scripts
git clone https://github.com/KingBoomie/yanu-twin
cd yanu-twin
```

5. start roscore `source /opt/ros/noetic/setup.zsh && roscore`

6. in the container shell, run the script with the command:
```bash
source /opt/ros/noetic/setup.zsh && source ~/dev/yanu-workspace/devel/setup.zsh
./python.sh /root/Documents/Kit/apps/Isaac-Sim/scripts/yanu-twin/import_robot.py
```

7. in shells 3,4 and 5, start the yanu nodes

```bash
source /opt/ros/noetic/setup.zsh && source ~/dev/yanu-workspace/devel/setup.zsh
roslaunch yanu_3_moveit_config demo.launch
```


```bash
source /opt/ros/noetic/setup.zsh && source ~/dev/yanu-workspace/devel/setup.zsh
rosrun robobar_engine yanu_docker_server
```

```bash
source /opt/ros/noetic/setup.zsh && source ~/dev/yanu-workspace/devel/setup.zsh
rosrun robobar_engine yanu_client
```

