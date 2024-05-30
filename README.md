# APMP
This is a repository of the online localization module of:

**[ICRA 24] Accurate Prior-centric Monocular Positioning with Offline LiDAR Fusion**

- A hierarchical visual tracking of [SuperPoint](https://github.com/magicleap/SuperPointPretrainedNetwork) features.
- Visual map is compatible with [COLMAP](https://colmap.github.io/) format.

Contributers: [Jinhao He *](https://github.com/David-Willo/), [Huaiyang Huang *](https://github.com/hyhuang1995), [Shuyang Zhang](https://github.com/ShuyangUni), [Jianhao Jiao](https://github.com/gogojjh), _et al._

{*} for equal contribution

**CODE release coming soon.**

---
![ICRA24-Presentation](https://github.com/David-Willo/APMP/assets/14790278/d88a7654-1303-466a-aded-194527709156)

---
### example result (may need some time to load the gif):
![icra24-gif1-crop](https://github.com/David-Willo/APMP/assets/14790278/33d64966-26fc-427f-9b4f-4b4810b05ec9)


# installation

1. prerequisites

ubuntu 20.04, ros-noetic, cuda-11.7, cudnn-8.5

2. clone repo

under a catkin workspace
```
mkdir src
cd src
git clone https://github.com/David-Willo/APMP.git --recursive

```

3. install
under catkin workspace

```
catkin init
catkin config --extend /opt/ros/noetic
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```
catkin build sophus
catkin build xslam_visual_localization
```


## tests

```
catkin test xslam_visual_features
catkin test xslam_visual_localization
```

## known issues:
1. build error due to protobuf version mismatch.

update the CMakeList.txt of protobuf_catkin
```
set(USE_SYSTEM_PROTOBUF "OFF") to compile with given version
```
## Getting Start:
TODO
