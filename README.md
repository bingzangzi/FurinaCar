适用于2024年的全国机器人大赛全地形小车项目，使用Arduino编程实现

furinacar.ino:初始化和主循环

trailing.h:寻迹和任务模块，包含寻迹、障碍翻越、颜色识别、执行倒弹珠任务等内容

gyro.h:陀螺仪模块，读取串口中来自陀螺仪的数据并处理

Timer2ServoPwm：使用Arduino的Timer2实现对伺服电机的PWM控制，来源：https://github.com/atp798/Timer2ServoPwm

