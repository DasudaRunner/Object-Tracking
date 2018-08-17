# Object-Tracking
### This object tracking library is optimized by using Eigen3 and openMP
___
*Author: Haibo Wang*
*Email: dasuda2015@163.com*
___

这是一个开源的物体追踪库,开发语言为C++,复现了经典和主流的物体追踪算法及涉及到的算法(比如RANSAC,最小二乘法),当然这些都是Eigen3及openMP优化过的.

实验平台:
<<<<<<< HEAD
*CPU: intel i7 6700K*
=======
*CPU: intel i7 6700K,*
>>>>>>> 63b60d9ec12a3b37bd68b20901b2682d08699a1a
*Memory:8Gx2 DDR4*

### 性能展示:
___
<<<<<<< HEAD
**RANSAC**: 600样本点 500次迭代 耗时(10次平均):*100us *
**Kalman Filter**: 二维坐标追踪,系统状态变量[x,y,dx,dy],预测+更新,平均耗时:*8us*
___
=======
RANSAC: 600样本点 500次迭代 时间(10次平均):100us 
___
>>>>>>> 63b60d9ec12a3b37bd68b20901b2682d08699a1a
