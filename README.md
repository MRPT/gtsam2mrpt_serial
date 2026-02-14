[![Ubuntu CI](https://github.com/MRPT/gtsam2mrpt_serial/actions/workflows/cmake.yml/badge.svg)](https://github.com/MRPT/gtsam2mrpt_serial/actions/workflows/cmake.yml)

| Distro | Build dev | Build releases | Stable version |
| ---    | ---       | ---            | ---         |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__gtsam2mrpt_serial__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__gtsam2mrpt_serial__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__gtsam2mrpt_serial__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__gtsam2mrpt_serial__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__gtsam2mrpt_serial__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__gtsam2mrpt_serial__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/gtsam2mrpt_serial)](https://index.ros.org/?search_packages=true&pkgs=gtsam2mrpt_serial) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__gtsam2mrpt_serial__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__gtsam2mrpt_serial__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__gtsam2mrpt_serial__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__gtsam2mrpt_serial__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__gtsam2mrpt_serial__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__gtsam2mrpt_serial__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/gtsam2mrpt_serial)](https://index.ros.org/?search_packages=true&pkgs=gtsam2mrpt_serial) |
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__gtsam2mrpt_serial__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__gtsam2mrpt_serial__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__gtsam2mrpt_serial__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__gtsam2mrpt_serial__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__gtsam2mrpt_serial__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__gtsam2mrpt_serial__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/gtsam2mrpt_serial)](https://index.ros.org/?search_packages=true&pkgs=gtsam2mrpt_serial) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__gtsam2mrpt_serial__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__gtsam2mrpt_serial__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uN64__gtsam2mrpt_serial__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__gtsam2mrpt_serial__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__gtsam2mrpt_serial__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__gtsam2mrpt_serial__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/gtsam2mrpt_serial)](https://index.ros.org/?search_packages=true&pkgs=gtsam2mrpt_serial) |

# gtsam2mrpt_serial
A C++ library offering a GTSAM ⇆ mrpt-serialization bridge.

This library offers efficient, binary serialization of GTSAM data structures, including whole `NonLinearFactorGraphs` and `Values`.
Storage format has the advantages of [mrpt-serialization](https://docs.mrpt.org/reference/latest/group_mrpt_serialization_grp.html):
- Fast and efficient (binary storage by default).
- Binary files are **portable** between machine architectures, endianness, word size, and operative system (unlike Boost).
- Support for **versioning**, so future versions will be able to read old files.
- Automatic support for `.gz` and `.zstd` compression via [mrpt::io::CCompressedOutputStream](https://docs.mrpt.org/reference/latest/class_mrpt_io_CCompressedOutputStream.html).
- GTSAM objects can be serialized to/from any stream defined in [mrpt-io](https://docs.mrpt.org/reference/latest/group_mrpt_io_grp.html), like TCP sockets, files, or pipes.

C++17 is required since that is the minimum C++ standard required by MRPT. 

## Dependencies

- MRPT: Install with `sudo apt install libmrpt-dev` (on Ubuntu 22.04 or newer) or from their ROS 2 packages, or otherwise [see install instructions](https://docs.mrpt.org/reference/latest/download-mrpt.html).
- GTSAM: Build from sources or install from [the PPA](https://gtsam.org/get_started/).

## Examples of use. 

See [gtsam2mrpt_serial/tests/main.cpp](gtsam2mrpt_serial/tests/main.cpp).

## Performance

Simple performance profiling shows an average time cost of roughly **50% the native Boost binary serialization mechanism** for mid-sized graphs.

The following quantitative analysis was done with [this code](gtsam2mrpt_serial/tests/main.cpp), on an Ubuntu 20.04 (Boost 1.71, MRPT 2.4.4) and a Intel(R) Core(TM) i7-6700HQ CPU @ 2.60GHz CPU. Note that the time scale is **logarithmic**.

![profiling-ser](docs/profiling-ser.png)

![profiling-deser](docs/profiling-deser.png)
