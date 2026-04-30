# bbs3d_ros2

ROS 2 wrapper for [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) — full-search 3D global localization with branch-and-bound on point cloud maps.

> **Status:** WIP. See [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) for the build-out roadmap and current step.

## Prerequisites
- Ubuntu 22.04
- ROS 2 humble
- CUDA 12.0+
- Eigen 3.4+ (auto-fetched via submodule recursion)

## Install

```bash
# 1. Clone recursively into your colcon workspace
cd ~/colcon_ws/src
git clone --recursive https://github.com/abudori3939/bbs3d_ros2.git
cd bbs3d_ros2
[ ! -f 3d_bbs/COLCON_IGNORE ] && touch 3d_bbs/COLCON_IGNORE  # only needed if you forgot --recursive flags

# 2. Build & install upstream 3D-BBS into /usr/local (one-time, must repeat after any submodule update)
cd 3d_bbs && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install

# 3. Build the ROS 2 wrapper
cd ~/colcon_ws
colcon build --packages-select bbs3d_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> **Important:** After `git submodule update` (or any change inside `3d_bbs/`), re-run step 2. Skipping this leaves stale headers / `libgpu_bbs3d.so` on your system that may not match the wrapper's expectations.

## Run

```bash
ros2 launch bbs3d_ros2 bbs3d_rviz2.launch.py config_file:=/path/to/your/config.yaml
```

(Detailed run instructions and topic / service contract land with Step 5 — see the plan.)

## License
MIT. Built on top of [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) (MIT). Please cite the original paper if you use this in research:

```
@inproceedings{aoki20243dbbs,
  title={3D-BBS: Global Localization for 3D Point Cloud Scan Matching Using Branch-and-Bound Algorithm},
  author={Koki Aoki and Kenji Koide and Shuji Oishi and Masashi Yokozuka and Atsuhiko Banno and Junichi Meguro},
  booktitle={IEEE International Conference on Robotics and Automation},
  year={2024},
  organization={IEEE}
}
```
