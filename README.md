# 3d_reconstruction

This is a graduation project.

The goal of this project is reconstruct a 3d point cloud from RGBD data captured by Intel RealSense D455i camera.

## Requirements

Operating System: Ubuntu 22.04 and macOS 13 are tested. Windows is not supported, use WSL2 instead.

CUDA support only works on Linux or WSL2 with NVIDIA GPU and Clang 14 or G++ 12 compiler. You need to install NVIDIA
driver and CUDA toolkit 12 first.

Libraries:

- librealsense2
- PCL
- glfw3
- spdlog
- lyra
- fast_gicp
- OpenMP

Compiler:

- G++ 12
- Clang 14
- Apple Clang 14

For macOS, you need also install Xcode Command Line Tools from Apple and QT5 using Homebrew.

## Compile

For Ubuntu 22.04:

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=clang ..
make
```

Change `clang` to `g++-12` if you want to use G++ 12.

Add `-DBUILD_WITH_CUDA=ON` to enable CUDA support.

For macOS 13:

```bash
mkdir build
cd build
export Qt5_DIR=/opt/homebrew/Cellar/qt@5/{QT_VERSION}
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

Replace `{QT_VERSION}` with your installed version of QT5.

## Run

```bash
./3d_reconstruction --help
```

# License

This project is licensed under the GPL 3.0 License - see the [LICENSE](LICENSE) file for details
