# Object-Tracking
## This object tracking library is optimized by using Eigen3 and openMP
___
&emsp;&emsp;*Author: Haibo Wang*<br>
&emsp;&emsp;*Email: dasuda2015@163.com*
___

&emsp;&emsp;*这是一个开源的物体追踪库,开发语言为C++,复现了经典和主流的物体追踪算法及涉及到的算法(比如RANSAC,最小二乘法),当然这些都是Eigen3及openMP优化过的.*

## 实验平台:
___
*CPU: intel i7 6700K*<br>*Memory:8Gx2 DDR4*
___
## 性能展示:
___
**RANSAC**: 600样本点 500次迭代 耗时(10次平均):*100us *<br>
**Kalman Filter**: 二维坐标追踪,系统状态变量[x,y,dx,dy],预测+更新,平均耗时:*8us*
___
## 一.工欲善其事 必先利其器——相机标定<br>
___
&emsp;&emsp;使用opencv自带的标定工具确定相机的内参,棋盘图片已给出，傻瓜式操作,了解一下
___
### Requirments:<br>
		opencv: >=3.3.0 or >=3.0.0
		python: >=2.7
		python-opencv:
### Quick Start:<br>
#### 1.get checkerboard images using your camera
		python getChecker.py # press 's' to save image
#### 2.get images list file
		./create_imagelist imagelist.yaml *.jpg
#### 3.get camera's internal reference
		./calibration -w=7 -h=7 imagelist.yaml
#### 4.Attention! you can also run calibration_camera.sh instead of running 2 and 3
		sudo sh calibration_camera.sh
&emsp;&emsp;get out_camera_data.yml ,and it's your camera's internal reference file
#### 5.you can run ./demo get corrected images
		./demo