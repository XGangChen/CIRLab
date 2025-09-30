# Kinect-V2
This repository tries to run the Xbox KinectV2 using [libfreenect2](https://github.com/OpenKinect/libfreenect2), and using [pylibfreenect2](https://github.com/r9y9/pylibfreenect2) to transfer the environment from CPP to Python. Also, uses [kinect2_ros2](https://github.com/krepa098/kinect2_ros2) to bridge KinectV2 to ROS2 Humble.

* Many issues would occur while setting up the system. Using GPT or any other AI tools to solve it is a good choice.

## [libfreenect2](https://github.com/OpenKinect/libfreenect2)
### Environment Setup
* Create a conda environment for pylibfreenect2.
  ```
  conda create -n kinectx python=3.9
  conda activate kinectx
  ```
* Download libfreenect2 source
  ```
  git clone https://github.com/OpenKinect/libfreenect2.git
  cd libfreenect2
  ```
* Install build tools
  ```
  sudo apt-get install build-essential cmake pkg-config
  ```
* Install CUDA toolkit from Nvidia's official website (make sure the toolkit version fits your driver version)
  ```
  # Mine is 12.8
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
  sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
  wget https://developer.download.nvidia.com/compute/cuda/12.8.0/local_installers/cuda-repo-ubuntu2204-12-8-local_12.8.0-570.86.10-1_amd64.deb
  sudo dpkg -i cuda-repo-ubuntu2204-12-8-local_12.8.0-570.86.10-1_amd64.deb
  sudo cp /var/cuda-repo-ubuntu2204-12-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
  sudo apt-get update
  sudo apt-get -y install cuda-toolkit-12-8
  ```
* Install CUDA samples
  ```
  git clone https://github.com/NVIDIA/cuda-samples.git
  ```
* Install the development package for TurboJPEG and the OpenGL viewer.
  ```
  sudo apt update
  sudo apt install libturbojpeg0-dev
  sudo apt install libglfw3-dev libglew-dev
  ```
* Fresh build pointing to 12.8 Samples
  ```
  cd ~/libfreenect2
  sudo mkdir build && cd build
  SAMPLES_INC="$HOME/cuda-samples/Common"
  
  cmake "$HOME/libfreenect2" \
    -DENABLE_CUDA=ON \
    -DCUDA_ARCH_BIN=89 -DCUDA_ARCH_PTX=89 \
    -DCUDA_NVCC_FLAGS="-Wno-deprecated-gpu-targets;-I$SAMPLES_INC" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_INSTALL_PREFIX=/usr/local
  ```
* Compile and install
  ```
  make -j$(nproc)
  sudo make install
  sudo ldconfig

  # 2) Confirm headers are where many tools expect them
  ls /usr/local/include/libfreenect2/config.h
  ```
* Confirm whether Linux even “sees” the Kinect on the USB bus by `lsusb`. You should see something like `Bus 001 Device 007: ID 045e:02ad Microsoft Corp. Xbox NUI Kinect Sensor`. If there isn't exist, replug it and try again.
* Copy the sample rules straight from the libfreenect2 repo. (The file name would be different; you should confirm by the same path.)
  ```
  sudo cp ~/libfreenect2/platform/linux/udev/90-kinect2.rules \
        /etc/udev/rules.d/90-kinect2.rules
  ```
* Reload and re-trigger udev so it picks up the new file
  ```
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  groups # Make sure your user is in plugdev
  ```
* Finally, try Protonect.
  ```
  ~/libfreenect2/build/bin/Protonect

  # Try to run with GPUs
  ~/libfreenect2/build/bin/Protonect cuda
  ```

## Python Environment Transfer ([pylibfreenect2](https://github.com/r9y9/pylibfreenect2))
* Build deps (Cython needed; older setuptools is safer for legacy setup.py)
  ```
  pip install -U "pip<24.2" wheel "setuptools<70" "Cython<3"

  # Install a prebuilt NumPy wheel compatible with Py3.9
  pip install "numpy==1.23.5"

  conda install -c conda-forge libgcc-ng libstdcxx-ng
  conda install opencv
  
  # Environment hints so the build can find the C/C++ bits
  export LIBFREENECT2_INSTALL_PREFIX="$CONDA_PREFIX"
  export PKG_CONFIG_PATH="$CONDA_PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH}"
  export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH}"
  ```
* Using Pylibfreenect2 [pylibfreeenect2](https://github.com/r9y9/pylibfreenect2 )
```
# from wherever you keep your projects…
git clone https://github.com/r9y9/pylibfreenect2.git
cd pylibfreenect2

# If your real headers are in /usr/local, bridge them into the conda prefix:
sudo mkdir -p "$CONDA_PREFIX/include/libfreenect2"
sudo rsync -a /usr/local/include/libfreenect2/ "$CONDA_PREFIX/include/libfreenect2/"

# Make sure libs are visible at runtime for Python
sudo ln -sf /usr/local/lib/libfreenect2.so* "$CONDA_PREFIX/lib/" 2>/dev/null || true
echo "$CONDA_PREFIX/lib" | sudo tee /etc/ld.so.conf.d/conda-libfreenect2.conf >/dev/null
sudo ldconfig

python -m pip install --no-build-isolation -e .
```
Then you can run the example script:
```
python examples/multiframe_listener.py
```

## [Kinect2 to ROS2](https://github.com/krepa098/kinect2_ros2)
* You have to install ROS2 Humble first.
* Clone the report into your `/ros_ws/src` and build. (If you don't have it, make one.)
  ```
  mkdir -p ~/ros_ws/src
  cd ~/ros_ws
  # Build the workspace
  colcon build --symlink-install

  cd ~/ros_ws/src
  git clone https://github.com/krepa098/kinect2_ros2.git
  ```
* Install system-wide to /usr/local
  ```
  cd ~/libfreenect2/build
  sudo make install
  sudo ldconfig

  # Verify Protonect installed copy:
  ldd /usr/local/bin/Protonect | grep -i freenect2
  # expect: /usr/local/lib/libfreenect2.so.0.2
  ```
* Install the image transport plugins (Humble)
  ```
  sudo apt update
  sudo apt install -y \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-theora-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-depth-image-proc \
    ros-humble-image-pipeline \
    ros-humble-image-transport-plugins
  ```
* Do this in a fresh shell
  ```
  # 0) No conda
  conda deactivate 2>/dev/null || true
  conda deactivate 2>/dev/null || true
  
  # 1) Prefer /usr/local when finding/linking
  export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
  export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  
  # (optional but helpful)
  export LDFLAGS="-L/usr/local/lib -Wl,-rpath,/usr/local/lib"
  export CPPFLAGS="-I/usr/local/include"
  
  # 2) Base ROS 2
  source /opt/ros/humble/setup.bash
  source ~/ros_ws/install/setup.bash

  
  # 3) Clean rebuild
  cd ~/ros_ws
  rm -rf build install log
  colcon build --symlink-install --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
  ```
  
* Source the workspace
  ```
  # To avoid sourcing it every time you open the terminal, add this line to your ~/.bashrc
  echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```
* Run the launch file
  ```
  # sanity check:
  ros2 pkg prefix depth_image_proc   # should print /opt/ros/humble or similar

  # launch
  ros2 launch kinect2_bridge kinect2_bridge_launch.yaml

  # View color image
  ros2 run rqt_image_view rqt_image_view
  ```

