# APMP
This is a repository of the online localization module of:

**[ICRA 24] Accurate Prior-centric Monocular Positioning with Offline LiDAR Fusion**

- A hierarchical visual tracking of [SuperPoint](https://github.com/magicleap/SuperPointPretrainedNetwork) features.
- Visual map is compatible with [COLMAP](https://colmap.github.io/) format.

Contributers: [Jinhao He *](https://github.com/David-Willo/), [Huaiyang Huang *](https://github.com/hyhuang1995), [Shuyang Zhang](https://github.com/ShuyangUni), [Jianhao Jiao](https://github.com/gogojjh), _et al._

{*} for equal contribution


---
![ICRA24-Presentation](https://github.com/David-Willo/APMP/assets/14790278/d88a7654-1303-466a-aded-194527709156)

---
### example result (may need some time to load the gif):
![icra24-gif1-crop](https://github.com/David-Willo/APMP/assets/14790278/33d64966-26fc-427f-9b4f-4b4810b05ec9)


# Build the project

1. prerequisites

ubuntu 20.04, ros-noetic, cuda-11.7, cudnn-8.5 


2. clone repo

under a catkin workspace
```
mkdir src
cd src
git clone https://github.com/David-Willo/APMP.git --recursive

```
due to lfs size limit, download and unzip extra dependencies
- `dependencies`: 
    - `torch_catkin`: catkin wrapper for libtorch and torch_tensorrt [repo](https://github.com/David-Willo/torch_catkin)
    - `xslam_test_data`: model files and sample data [download and extract](https://drive.google.com/drive/folders/10zBkkRtqMTM4WOV0tfBpXaTPjkfRnFPy?usp=sharing)

3. build

under catkin workspace

```
catkin init
catkin config --extend /opt/ros/noetic
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

4. prepare data and run the script

Download [demo data](https://drive.google.com/file/d/1wfFz8Xjewd19Kv7yhrvV2TbMhVaxDo8Q/view?usp=sharing) and extract,

update path in runner script (xslam/script/run_apmp.sh) and execute,

the map database is compatible with the colmap sparse reconstruction format, 

feature extraction and database generation can follow [hloc demo](https://github.com/cvg/Hierarchical-Localization/blob/master/demo.ipynb)


## known issues:
1. build error due to protobuf version mismatch.

update the CMakeList.txt of protobuf_catkin
```
set(USE_SYSTEM_PROTOBUF "OFF") to compile with given version
```
### Acknowledgements
Our implementation refers to [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2), [hloc](https://github.com/cvg/Hierarchical-Localization), [SuperPointPretrainedNetwork](https://github.com/magicleap/SuperPointPretrainedNetwork), [maplab](https://github.com/ethz-asl/maplab)
We really appreciate these open source projects!
