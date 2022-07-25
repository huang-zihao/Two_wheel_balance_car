一、动力学分析
1.对两个轮子和车体受力分析（直接参考论文
2.建立动力学微分方程（直接参考论文


二、器材的使用
最重要的两个器材，MPU5060，电机驱动模块

#MPU5060（姿态检测：3轴加速度传感器+3轴角速度传感器
1.mpu6050是采用的I2C通信，需要将开发板BOOT1和BOOT0置于0
2.需要进行滤波操作，在z轴加速度上误差特别大

![image](https://user-images.githubusercontent.com/109507018/180629533-ad665100-ab85-4ac0-877a-4f38ea61dc0e.png)

可以通过简单均值滤波处理一下，即将开始读取的一部分数据进行平均，再于获取函数中减去（注意在z轴加速度需要补回，因为一开始是打算z轴朝上进行稳定值得读取）


#AB向无刷直流电机
AB向编码器：

![image](https://user-images.githubusercontent.com/109507018/180652436-93de5264-e845-45ea-97a9-416221dedffb.png)

无刷直流驱动方式：
直接pwm单向输入即可
但是需要驱动、稳压


三、算法
#控制算法（模糊、神经网络
#卡尔曼、互补滤波器


四、代码、参数整理调节
