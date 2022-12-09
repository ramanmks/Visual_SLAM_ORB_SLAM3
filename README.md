### Visual_SLAM_ORB_SALM3
This repository contains all the necessary information and codes required to compile and successfully test run ORB_SLAM3 on an freshly installed system running Ubuntu 20.04

Tip #1:

We use Ubuntu 20.04, henceforth Pangolin, Opencv , and Eigen3 should be installed. Among them, Pangolin and Eigen3 can be easily installed by directly pulling the repository from the Git. Also, it is advised to run Ubuntu as a standalone OS and not using a Virtual Machine. I have experienced a lot of problems on running VM.

As per my experience, Macs with Apple Silicon Processors cannot compile and run ORB_SLAM3 successfully. I've tried using Parallels with a M1 Macbook pro and the translation between the X86 and ARM causes a lot of processes to fail. Hence, it is advised to run this on a X86 System.

I'm commenting about some errors that one may come across while building ORB_SLAM3. Prior knowlege about this may prove to be benificial and would simplify the debugging process as well.

### Error1:

```
In file included from /usr/local/include/pangolin/utils/signal_slot.h:3,
                 from /usr/local/include/pangolin/windowing/window.h:35,
                 from /usr/local/include/pangolin/display/display.h:34,
                 from /usr/local/include/pangolin/pangolin.h:38,
                 from /home/a616708946/slambook/ch5/code/disparity.cpp:8:
/usr/local/include/sigslot/signal.hpp:109:79: error: ‘decay_t’ is not a member of ‘std’; did you mean ‘decay’?
  109 | constexpr bool is_weak_ptr_compatible_v = detail::is_weak_ptr_compatible<std::decay_t<P>>::value;
      |                                                                               ^~~~~~~
      |                                                                               decay
/usr/local/include/sigslot/signal.hpp:109:79: error: ‘decay_t’ is not a member of ‘std’; did you mean ‘decay’?
  109 | constexpr bool is_weak_ptr_compatible_v = detail::is_weak_ptr_compatible<std::decay_t<P>>::value;
      |                                                                               ^~~~~~~
      |                                                                               decay
/usr/local/include/sigslot/signal.hpp:109:87: error: template argument 1 is invalid
  109 | constexpr bool is_weak_ptr_compatible_v = detail::is_weak_ptr_compatible<std::decay_t<P>>::value;
```


### **Update Cmakelists.txt from -std=c++11 to -std=c++14**  

```  
  CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
```

### Error 2:

### One might encounter error due to the version of Eigen.

```
 ../core/base_edge.h: 33:10: fatal error: Eigen/Core: No such file or directory 
#include <Eigen/Core>
```

### For this, Replace all of #include <Eigen/(any packages)> to #include <eigen3/Eigen/(any packages)>

Like this:

```
#include <Eigen/Core>
to
#include <eigen3/Eigen/Core>
```

### Now we can move on to install the ORB_SLAM3 on a Freshly installed Ubuntu 20.04 System:

### Install all the dependencies using the commands below:

```
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev
sudo apt-get install libglew-dev libboost-all-dev libssl-dev
sudo apt install libeigen3-dev
```
### Install OpenCV version 3.2.0

```
cd ~
mkdir Dev && cd Dev
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.2.0
```

### Put the following code at the top of the header file for the file:

```
gedit ./modules/videoio/src/cap_ffmpeg_impl.hpp
#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
#define AVFMT_RAWPICTURE 0x0020
```

Now, save and close the file and then proceed to make and build OpenCV

```
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j 3
sudo make install
```

### Install Pangolin

```
cd ~/Dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
mkdir build 
cd build 
cmake .. -D CMAKE_BUILD_TYPE=Release 
make -j 3 
sudo make install
```

### Now coming to the ORB_SLAM3 installation:

```
cd ~/Dev
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git 
cd ORB_SLAM3
```
We need to change the header file gedit ./include/LoopClosing.h at line 51 from

```
Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
```
to

```
Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose; 
```

In order to make this comiple. Now, we can comiple ORB-SLAM3 and it dependencies as DBoW2 and g2o.


Now Simply just run (if you encounter compiler, try to run the this shell script 2 or 3 more time. It works for me.)

```
./build.sh
```

 In addition, when compiling, I encountered the following error, which puzzled me. We actually need to add the following codes in error causing files. Add it in [directory]/ORB_SLAM3/include/Cameramodels/Kannalabrandt8.h 

 make[2]: *** [CMakeFiles/ORB_SLAM3.dir/src/LocalMapping.cc.o] Error 1

 ![image](https://user-images.githubusercontent.com/100104834/206598623-ce0b314d-1c54-4be3-97c9-0aa77f24e07a.png)

![image](https://user-images.githubusercontent.com/100104834/206598760-8afbef6d-82dd-4ad7-bbd0-ff0d8683a2e2.png)

### Don't forget to change the opencv version when compiling ./build.sh

![image](https://user-images.githubusercontent.com/100104834/206598929-54d79359-5208-4b51-9f62-e55c919ee0fa.png)

*** I have referenced most of the errors that I encountered while building ORB_SLAM3; for other errors pertaining to this, feel free to do a simple search on the interenet. Most of the solutions to the errors are available in Git and CSDN Repositories.


### Testing with Datasets:

1. Download the data set from : https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets#downloads

2. Modify the datasets : Path: ORB_SLAM3/dataset/MH_##/mav0(Example: after MH01 is decompressed, the file name is mav0, create a new MH01 folder in the dataset folder, put mav0 in it, do not modify the file name directly!!!)

3. Double-click to open the script "euroc_examples.sh" in the ORB-SLAM3 source code, and find the instructions containing MH01

![image](https://user-images.githubusercontent.com/100104834/206599776-a2ca2b17-62bc-49e8-848c-678c928d70e3.png)


### Installing ROS

For Ubuntu Install ROS Noetic from the Official ROS Website; follow the instructions and copy paste the commands in terminal to install ROS

http://wiki.ros.org/noetic/Installation/Ubuntu

### Using USB_CAMERA for capture in real time.

1. Create a Catkin Workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

Compile the Workspace:

```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

Download the camera source code from : https://github.com/ros-drivers/usb_cam

Unzip the package and put it in the above src

![image](https://user-images.githubusercontent.com/100104834/206600915-4a5838fc-bb16-4bf1-afaa-556869efe3c1.png)

### To launch camera using ROS

```
//Open a Fresh terminal and invoke roscore
roscore
// In another terminal
cd catkin_ws
catkin_make
roslaunch usb_cam usb_cam-test.launch
```

 Camera parameters can be changed in usb_cam-test.launch

 ![image](https://user-images.githubusercontent.com/100104834/206601312-d55afada-266c-41e6-82f9-ab854cda6809.png)

![image](https://user-images.githubusercontent.com/100104834/206601397-16a504e8-558a-4829-ba6c-9482b13d197b.png)

Open another window and enter rqt_graph to view the message, and see that the usb_cam node has published a message to /usb_cam/image_raw, image_view has subscribed to the message, and the content of the message is the image frame, which is displayed by the image_view

![image](https://user-images.githubusercontent.com/100104834/206601486-ec2ea3a8-4909-4a1b-a4c9-6a17bc9cc391.png)


### Calibration of the Monocular Camera

For this project, I used a monocular camera for the capture. The camera needs to be calibrated in order to effectively communicate with the ORB_SLAM3 to produce tangible results during the capture

```
sudo apt-get install ros-noetic-camera-calibration
roslaunch usb_cam usb_cam-test.launch
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/usb_cam/image_raw camera:=/usb_cam
```
The cameracalibrator.py calibration program requires the following input parameters.
1) size: calibrate the number of internal corners of the checkerboard, how many rows are there in the chessboard used here, and each row has an internal corner.
2) square: This parameter corresponds to the side length of each checkerboard, in meters.
3) image and camera: set the topic of images released by the camera.

Move the board according to x (left and right), y (up and down), size (front and back), skew (tilt), etc., until the progress bars of x, y, size, and skew all turn into green positions.

![image](https://user-images.githubusercontent.com/100104834/206602007-93f9f285-233c-49e3-9040-682f4f77d210.png)

Press the CALIBRATE button when all the tabs are green and wait for the calibration to end.

After finishing, click SAVE, and then COMMIT, there will be a calibration result yaml file address after the terminal. 

After opening, modify it according to the format of Asus.yaml, name it xx (custom name).yaml, copy it to the /home/xxx/src/ORB_SLAM3/Examples/ROS/ORB_SLAM3 directory, and the calibration is completed.

![image](https://user-images.githubusercontent.com/100104834/206602228-632f299d-0374-46bd-92bc-b0fd002872f1.png)

Note: The default rosrun is to call /usr/bin/python, but my Ubuntu20.04 does not have that file, only python2, python3, python3.8, this time I found the cameracalibrator.py file, under normal circumstances, the location should be / opt/ros/noetic/lib/camera_calibration/cameracalibrator.py, after opening and editing with sudo vim cameracalibrator.py, change the first line to:

![image](https://user-images.githubusercontent.com/100104834/206602308-749532ca-9280-465f-8c06-74d7071792a3.png)

Now, we can use ROS to start the USB_CAM node and after the successful installation of all the other components we can run the ORB_SLAM3 in real time.

```
roscore
roslaunch usb_cam usb_cam-test.launch
rosrun ORB_SLAM3 Mono /home//Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/linux/Dev/ORB_SLAM3/xxx.yaml
```

If it does not work, add ORB_SALM3 as a node:

```
vim ~/.bashrc
source /opt/ros/noetic/setup.bash
source /home/linux/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/xxx/Dev/ORB_SLAM3/Examples/ROS
source ~/.bashrc
sudo vim /opt/ros/noetic/setup.sh
source /home/linux/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/xxx/Dev/ORB_SLAM3/Examples/ROS
source /opt/ros/noetic/setup.sh
```
![image](https://user-images.githubusercontent.com/100104834/206610319-5d05abdc-833c-4b50-8a09-f4204e71b077.png)

Explanation: When compiling the usb camera node, the source code was put into the src of the workspace and then compiled. Now there is a ROS node in SLAM3. After compiling ./build_ros.sh, it should be directly used , and if you want to use the usb camera, you need to change the following files and recompile ./build_ros.sh


![image](https://user-images.githubusercontent.com/100104834/206610398-1fbe963f-a19e-481b-bfa1-9e172278644e.png)
