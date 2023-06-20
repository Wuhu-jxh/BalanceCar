//
// Created by 神奇bug在哪里 on 6/20/23.
//

#ifndef BALANCECAR_SETTINGS_H
#define BALANCECAR_SETTINGS_H

/************MPU6050参数*************/
#define SMPLRT_DIV   0x19  // 采样率分频，典型值：0x07(125Hz) */
#define CONFIG       0x1A  // 低通滤波频率，典型值：0x06(5Hz) */
#define GYRO_CONFIG  0x1B  // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) */
#define ACCEL_CONFIG 0x1C  // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) */

#define ACCEL_XOUT_H 0x43  // 存储最近的X轴、Y轴、Z轴加速度感应器的测量值 */
#define ACCEL_XOUT_L 0x44
#define ACCEL_YOUT_H 0x45
#define ACCEL_YOUT_L 0x46
#define ACCEL_ZOUT_H 0x47
#define ACCEL_ZOUT_L 0x48

#define TEMP_OUT_H   0x41  // 存储的最近温度传感器的测量值 */
#define TEMP_OUT_L   0x42

#define GYRO_XOUT_H  0x3B  // 存储最近的X轴、Y轴、Z轴陀螺仪感应器的测量值 */
#define GYRO_XOUT_L  0x3C
#define GYRO_YOUT_H  0x3D
#define GYRO_YOUT_L  0x3E
#define GYRO_ZOUT_H  0x3F
#define GYRO_ZOUT_L  0x40

#define PWR_MGMT_1   0x6B   // 电源管理，典型值：0x00(正常启用) */
#define	PWR_MGMT_2		0x6C	//电源管理
#define WHO_AM_I     0x75 	// IIC地址寄存器(默认数值0x68，只读) */
#define MPU6050_ADDR 0xD0	// MPU6050手册上的地址，这里也可以使用serch函数去搜索

/************PID环_速度环参数*************/
#define SPEED_PID_KP 0.5f
#define SPEED_PID_KI 0.0f
//速度环不应有KD
#define GLOBAL_PID_LIMIT 1000 //PID积分抗饱和
/************PID环_角度环(转向环)参数*************/
#define ANGLE_PID_KP 0.5f
//角度环不应有KI
//角度环不应有KD

/************PID环_位置环(直立环)参数*************/
#define POSITION_PID_KP 0.5f
//位置环不应有KI
#define POSITION_PID_KD 0.0f
#define POSITION_PID_TARGET 0.0f

/************PID_核心参数*************/
#define CORE_PID_SPEED_EN 1 //速度环使能
#define CORE_PID_ANGLE_EN 1 //角度环使能
#define CORE_PID_POSITION_EN 1 //位置环使能
/**
 * @brief PID滤波模式
 * @param 0 -- 不滤波
 * @param 1 -- 低通滤波
 * @param 2 -- 卡尔曼滤波
 * @param 3 -- 一阶滞后滤波
 * @param 4 -- 均值滤波
 */
#define CORE_PID_FILTER_MODE 0 //滤波模式

/**
 * @attention 特别说明
 * @p 一阶滞后滤波：
 * 优点是实现简单，计算量小，不需要额外的存储空间；
 * 缺点是滤波效果不够理想，不能有效地分离出有用的信号成分，而且会引入相位延迟，影响系统的响应速度。
 * @p 低通滤波:
 * 优点是滤波效果较好，可以根据信号的频率特性选择合适的截止频率，保留有用的信号成分；
 * 缺点是实现复杂，计算量大，需要额外的存储空间，而且也会引入相位延迟，影响系统的响应速度。
 * @p 卡尔曼滤波:
 * 优点是可以利用前一时刻的状态和可能的测量值来得到当前时刻下的状态的最优估计
 * 缺点是资源消耗大，需要知道系统模型和噪声模型的准确信息
 * @p 均值滤波:
 * 优点是实现简单，计算量小，不需要额外的存储空间；
 * 缺点是滤波效果不够理想，不能有效地分离出有用的信号成分，而且会引入相位延迟，影响系统的响应速度。
 */
#if CORE_PID_FILTER_MODE == 1
#define CORE_PID_FILTER_MODE 3 //一阶状态下这两个算法的代码一致，可以复用
#elif CORE_PID_FILTER_MODE == 2
#define CORE_PID_FILTER_KALMAN_Q 0.02f //卡尔曼滤波Q
#define CORE_PID_FILTER_KALMAN_R 0.5f //卡尔曼滤波R
#elif CORE_PID_FILTER_MODE == 3
#define CORE_PID_FILTER_LAG 0.5f //一阶滞后滤波系数
#elif CORE_PID_FILTER_MODE == 4
#define CORE_PID_FILTER_MEAN 0.5f //均值滤波系数
#endif

#endif //BALANCECAR_SETTINGS_H
