//
// Created by mulai on 2023/6/18.
//

#include <math.h>
#include "MPU6050.h"
#include "i2c.h"

#define I2C_Channel hi2c2 //定义当前使用的I2C的通道

_MPU6050_DATA MPU6050_Data;
static int16_t MPU6050Addr = 0xD0;//定义初始的MPU6050的从机地址，如果固定且已知就可直接用定义而不用再去调用I2C_Serch函数


/***********************************************************
*@fuction	:I2C_Read  &&  I2C_Write
*@brief	:I2C的底层收发函数重构
*@param	:DevAddr：I2C从机地址，MemAddr：目标寄存器，oData  or  iData：需要发送或接收的数据
*@return:返回收发的结果
*@author:JCML
*@date	:2023-06-18
***********************************************************/
int8_t I2C_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData, uint8_t Datalen)
{
  return HAL_I2C_Mem_Read(&I2C_Channel,DevAddr,MemAddr,1,oData,Datalen,1000);
}

int8_t I2C_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData,uint8_t Datalen)
{
  return HAL_I2C_Mem_Write(&I2C_Channel,DevAddr,MemAddr,1,iData,Datalen,1000);
}

/***********************************************************
*@fuction	:I2C_Serch
*@brief	:查找I2C总线上的从机地址
*@param	:无
*@return:从机的地址
*@author:JCML
*@date	:2023-06-18
***********************************************************/
int16_t I2C_Serch(void)
{
  for(uint8_t i = 1; i < 255; i++)
  {
    if(HAL_I2C_IsDeviceReady(&I2C_Channel, i, 1, 1000) == HAL_OK)
    {
      MPU6050Addr = i;
      return i;
    }
  }
  return 0xD1;
}
/***********************************************************
*@fuction	:MPU6050_Init
*@brief	:MPU6050的初始化
*@param	:MPU6050的地址
*@return:是否成功
*@author:JCML
*@date	:2023-06-18
***********************************************************/
int8_t MPU6050_Init(int16_t Addr)
{
  uint8_t check;
  I2C_Read(Addr,WHO_AM_I,&check, 1);//读取MPU6050地址寄存器，以确认通信是否成功
  if(check == 0x68) // 确认设备的地址寄存器是否符合
  {
    check = 0x01;// 电源管理1，典型值：0x00(正常启用) */
    I2C_Write(Addr,PWR_MGMT_1,&check,1); 	    // 唤醒
    check = 0x00;// 电源管理2，典型值：0x00(正常启用) */
    I2C_Write(Addr,PWR_MGMT_2,&check,1); 	    // 唤醒
    check = 0x07;// 采样率分频，典型值：0x07(125Hz) */
    I2C_Write(Addr,SMPLRT_DIV,&check,1);	    // 125Hz的速率
    check = 0x06;
    I2C_Write(Addr,CONFIG,&check,1);
    check = 0x18;// 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) */
    I2C_Write(Addr,GYRO_CONFIG,&check,1);		// 陀螺配置
     check = 0x18;// 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) */
    I2C_Write(Addr,ACCEL_CONFIG,&check,1);	 	// 加速度配置

    return 0;
  }
  return -1;
}
/***********************************************************
*@fuction	:MPU6050_Read_Accel
*@brief	:MPU6050读取三个方向的加速度
*@param	:无
*@return:无
*@author:JCML
*@date	:2023-06-18
***********************************************************/
void MPU6050_Read_Accel(void)
{
  uint8_t Read_Buf[6];

  // 寄存器依次是加速度X高 - 加速度X低 - 加速度Y高位 - 加速度Y低位 - 加速度Z高位 - 加速度度Z低位
  I2C_Read(MPU6050Addr, ACCEL_XOUT_H, Read_Buf,6);//读取并转存至Buff中

  MPU6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);//转化其高八位以及低八位并进行转换
  MPU6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
  MPU6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);

//  MPU6050_Data.Accel_X = MPU6050_Data.Accel_X / 16384.0f;
//  MPU6050_Data.Accel_Y = MPU6050_Data.Accel_Y / 16384.0f;
//  MPU6050_Data.Accel_Z = MPU6050_Data.Accel_Z / 16384.0f;

}
/***********************************************************
*@fuction	:MPU6050_Read_Gyro
*@brief	:MPU6050读取三个方向的角度
*@param	:无
*@return:无
*@author:JCML
*@date	:2023-06-18
***********************************************************/
void MPU6050_Read_Gyro(void)
{
  uint8_t Read_Buf[6];

  // 寄存器依次是角度X高 - 角度X低 - 角度Y高位 - 角度Y低位 - 角度Z高位 - 角度Z低位
  I2C_Read(MPU6050Addr, GYRO_XOUT_H, Read_Buf,6);;

  MPU6050_Data.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
  MPU6050_Data.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
  MPU6050_Data.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);

//  MPU6050_Data.Gyro_X = MPU6050_Data.Gyro_X / 131.0f;
//  MPU6050_Data.Gyro_Y = MPU6050_Data.Gyro_Y / 131.0f;
//  MPU6050_Data.Gyro_Z = MPU6050_Data.Gyro_Z / 131.0f;

}
/***********************************************************
*@fuction	:MPU6050_Read_Temp
*@brief	:MPU6050读取温度
*@param	:无
*@return:无
*@author:JCML
*@date	:2023-06-18
***********************************************************/
void MPU6050_Read_Temp(void)
{
  uint8_t Read_Buf[2];

  I2C_Read(MPU6050Addr, TEMP_OUT_H, Read_Buf,2);;

  MPU6050_Data.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);

  MPU6050_Data.Temp = 36.53f + ((float )MPU6050_Data.Temp / 340.0f);//转换计算
}



void MPU6050_Read_DMP(void)
{
    //从MPU6050读取DMP解算数据
    uint8_t Read_Buf[14];
    I2C_Read(MPU6050Addr,0x3B,Read_Buf,14);
    MPU6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
    MPU6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
    MPU6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
    MPU6050_Data.Temp = (int16_t)(Read_Buf[6] << 8 | Read_Buf[7]);
    MPU6050_Data.Gyro_X = (int16_t)(Read_Buf[8] << 8 | Read_Buf[9]);
    MPU6050_Data.Gyro_Y = (int16_t)(Read_Buf[10] << 8 | Read_Buf[11]);
    MPU6050_Data.Gyro_Z = (int16_t)(Read_Buf[12] << 8 | Read_Buf[13]);
    //计算DMP
    MPU6050_Data.Accel_X = MPU6050_Data.Accel_X / 16384.0f;
    MPU6050_Data.Accel_Y = MPU6050_Data.Accel_Y / 16384.0f;
    MPU6050_Data.Accel_Z = MPU6050_Data.Accel_Z / 16384.0f;
    MPU6050_Data.Gyro_X = MPU6050_Data.Gyro_X / 131.0f;
    MPU6050_Data.Gyro_Y = MPU6050_Data.Gyro_Y / 131.0f;
    MPU6050_Data.Gyro_Z = MPU6050_Data.Gyro_Z / 131.0f;
    MPU6050_Data.Temp = 36.53f + ((float )MPU6050_Data.Temp / 340.0f);
    //计算俯仰角和横滚角
    MPU6050_Data.DMP_Pitch = asinf(-2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_Z + 2 * MPU6050_Data.Accel_Y * MPU6050_Data.Accel_Y) * 57.3;
    MPU6050_Data.DMP_Roll = atan2f(2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_Y + 2 * MPU6050_Data.Accel_Z * MPU6050_Data.Accel_Y, -2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_X - 2 * MPU6050_Data.Accel_Y * MPU6050_Data.Accel_Y + 1) * 57.3;
    //计算航向角
    MPU6050_Data.DMP_Yaw = atan2f(2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_Y + 2 * MPU6050_Data.Accel_Z * MPU6050_Data.Accel_Y, -2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_X - 2 * MPU6050_Data.Accel_Y * MPU6050_Data.Accel_Y + 1) * 57.3;
    //计算角速度
    MPU6050_Data.Gyro_X = MPU6050_Data.Gyro_X / 131.0f;
    MPU6050_Data.Gyro_Y = MPU6050_Data.Gyro_Y / 131.0f;
    MPU6050_Data.Gyro_Z = MPU6050_Data.Gyro_Z / 131.0f;
//    //计算角速度积分
//    MPU6050_Data.Gyro_X_Int = MPU6050_Data.Gyro_X_Int + MPU6050_Data.Gyro_X * 0.01;
//    MPU6050_Data.Gyro_Y_Int = MPU6050_Data.Gyro_Y_Int + MPU6050_Data.Gyro_Y * 0.01;
//    MPU6050_Data.Gyro_Z_Int = MPU6050_Data.Gyro_Z_Int + MPU6050_Data.Gyro_Z * 0.01;
//    //计算角速度积分修正
//    MPU6050_Data.Gyro_X_Int = MPU6050_Data.Gyro_X_Int + 0.0001 * MPU6050_Data.Roll;
//    MPU6050_Data.Gyro_Y_Int = MPU6050_Data.Gyro_Y_Int - 0.0001 * MPU6050_Data.Pitch;
//    MPU6050_Data.Gyro_Z_Int = MPU6050_Data.Gyro_Z_Int - 0.0001 * MPU6050_Data.Yaw;
//


}
