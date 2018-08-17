# Object-Tracking
### This object tracking library is optimized by using Eigen3 and openMP
___
*Author: Haibo Wang*<br>
*Email: dasuda2015@163.com*
___

这是一个开源的物体追踪库,开发语言为C++,复现了经典和主流的物体追踪算法及涉及到的算法(比如RANSAC,最小二乘法),当然这些都是Eigen3及openMP优化过的.

### 实验平台:
___
*CPU: intel i7 6700K*<br>*Memory:8Gx2 DDR4*
___
### 性能展示:
___
**RANSAC**: 600样本点 500次迭代 耗时(10次平均):*100us *<br>
**Kalman Filter**: 二维坐标追踪,系统状态变量[x,y,dx,dy],预测+更新,平均耗时:*8us*
___
