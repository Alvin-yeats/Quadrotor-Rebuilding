# Quadrotor-Rebuilding
Dissatisfied with the code same as shit , so determind of rebuilding my own code.

介绍：一个简单的四轴主控程序
功能介绍：
	保证四轴飞行时姿态角的稳定
	抗干扰能力
	可拓展：航拍，手势控制，跟随模式，路径规划
  
注：造成四轴姿态角量程不满±90°的原因是因为滤波问题，拟从滤波着手，在保证输出数据稳定的情况下，
选用适当的滤波算法对陀螺仪、加速计数据进行滤波。
