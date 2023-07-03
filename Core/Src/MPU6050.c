//
// Created by mulai on 2023/6/18.
//

#include <math.h>
#include "MPU6050.h"
#include "i2c.h"

#define I2C_Channel hi2c2 //���嵱ǰʹ�õ�I2C��ͨ��

_MPU6050_DATA MPU6050_Data;
static int16_t MPU6050Addr = 0xD0;//�����ʼ��MPU6050�Ĵӻ���ַ������̶�����֪�Ϳ�ֱ���ö����������ȥ����I2C_Serch����


/***********************************************************
*@fuction	:I2C_Read  &&  I2C_Write
*@brief	:I2C�ĵײ��շ������ع�
*@param	:DevAddr��I2C�ӻ���ַ��MemAddr��Ŀ��Ĵ�����oData  or  iData����Ҫ���ͻ���յ�����
*@return:�����շ��Ľ��
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
*@brief	:����I2C�����ϵĴӻ���ַ
*@param	:��
*@return:�ӻ��ĵ�ַ
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
*@brief	:MPU6050�ĳ�ʼ��
*@param	:MPU6050�ĵ�ַ
*@return:�Ƿ�ɹ�
*@author:JCML
*@date	:2023-06-18
***********************************************************/
int8_t MPU6050_Init(int16_t Addr)
{
  uint8_t check;
  I2C_Read(Addr,WHO_AM_I,&check, 1);//��ȡMPU6050��ַ�Ĵ�������ȷ��ͨ���Ƿ�ɹ�
  if(check == 0x68) // ȷ���豸�ĵ�ַ�Ĵ����Ƿ����
  {
    check = 0x01;// ��Դ����1������ֵ��0x00(��������) */
    I2C_Write(Addr,PWR_MGMT_1,&check,1); 	    // ����
    check = 0x00;// ��Դ����2������ֵ��0x00(��������) */
    I2C_Write(Addr,PWR_MGMT_2,&check,1); 	    // ����
    check = 0x07;// �����ʷ�Ƶ������ֵ��0x07(125Hz) */
    I2C_Write(Addr,SMPLRT_DIV,&check,1);	    // 125Hz������
    check = 0x06;
    I2C_Write(Addr,CONFIG,&check,1);
    check = 0x18;// �������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s) */
    I2C_Write(Addr,GYRO_CONFIG,&check,1);		// ��������
     check = 0x18;// ���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz) */
    I2C_Write(Addr,ACCEL_CONFIG,&check,1);	 	// ���ٶ�����

    return 0;
  }
  return -1;
}
/***********************************************************
*@fuction	:MPU6050_Read_Accel
*@brief	:MPU6050��ȡ��������ļ��ٶ�
*@param	:��
*@return:��
*@author:JCML
*@date	:2023-06-18
***********************************************************/
void MPU6050_Read_Accel(void)
{
  uint8_t Read_Buf[6];

  // �Ĵ��������Ǽ��ٶ�X�� - ���ٶ�X�� - ���ٶ�Y��λ - ���ٶ�Y��λ - ���ٶ�Z��λ - ���ٶȶ�Z��λ
  I2C_Read(MPU6050Addr, ACCEL_XOUT_H, Read_Buf,6);//��ȡ��ת����Buff��

  MPU6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);//ת����߰�λ�Լ��Ͱ�λ������ת��
  MPU6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
  MPU6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);

//  MPU6050_Data.Accel_X = MPU6050_Data.Accel_X / 16384.0f;
//  MPU6050_Data.Accel_Y = MPU6050_Data.Accel_Y / 16384.0f;
//  MPU6050_Data.Accel_Z = MPU6050_Data.Accel_Z / 16384.0f;

}
/***********************************************************
*@fuction	:MPU6050_Read_Gyro
*@brief	:MPU6050��ȡ��������ĽǶ�
*@param	:��
*@return:��
*@author:JCML
*@date	:2023-06-18
***********************************************************/
void MPU6050_Read_Gyro(void)
{
  uint8_t Read_Buf[6];

  // �Ĵ��������ǽǶ�X�� - �Ƕ�X�� - �Ƕ�Y��λ - �Ƕ�Y��λ - �Ƕ�Z��λ - �Ƕ�Z��λ
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
*@brief	:MPU6050��ȡ�¶�
*@param	:��
*@return:��
*@author:JCML
*@date	:2023-06-18
***********************************************************/
void MPU6050_Read_Temp(void)
{
  uint8_t Read_Buf[2];

  I2C_Read(MPU6050Addr, TEMP_OUT_H, Read_Buf,2);;

  MPU6050_Data.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);

  MPU6050_Data.Temp = 36.53f + ((float )MPU6050_Data.Temp / 340.0f);//ת������
}



void MPU6050_Read_DMP(void)
{
    //��MPU6050��ȡDMP��������
    uint8_t Read_Buf[14];
    I2C_Read(MPU6050Addr,0x3B,Read_Buf,14);
    MPU6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
    MPU6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
    MPU6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
    MPU6050_Data.Temp = (int16_t)(Read_Buf[6] << 8 | Read_Buf[7]);
    MPU6050_Data.Gyro_X = (int16_t)(Read_Buf[8] << 8 | Read_Buf[9]);
    MPU6050_Data.Gyro_Y = (int16_t)(Read_Buf[10] << 8 | Read_Buf[11]);
    MPU6050_Data.Gyro_Z = (int16_t)(Read_Buf[12] << 8 | Read_Buf[13]);
    //����DMP
    MPU6050_Data.Accel_X = MPU6050_Data.Accel_X / 16384.0f;
    MPU6050_Data.Accel_Y = MPU6050_Data.Accel_Y / 16384.0f;
    MPU6050_Data.Accel_Z = MPU6050_Data.Accel_Z / 16384.0f;
    MPU6050_Data.Gyro_X = MPU6050_Data.Gyro_X / 131.0f;
    MPU6050_Data.Gyro_Y = MPU6050_Data.Gyro_Y / 131.0f;
    MPU6050_Data.Gyro_Z = MPU6050_Data.Gyro_Z / 131.0f;
    MPU6050_Data.Temp = 36.53f + ((float )MPU6050_Data.Temp / 340.0f);
    //���㸩���Ǻͺ����
    MPU6050_Data.DMP_Pitch = asinf(-2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_Z + 2 * MPU6050_Data.Accel_Y * MPU6050_Data.Accel_Y) * 57.3;
    MPU6050_Data.DMP_Roll = atan2f(2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_Y + 2 * MPU6050_Data.Accel_Z * MPU6050_Data.Accel_Y, -2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_X - 2 * MPU6050_Data.Accel_Y * MPU6050_Data.Accel_Y + 1) * 57.3;
    //���㺽���
    MPU6050_Data.DMP_Yaw = atan2f(2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_Y + 2 * MPU6050_Data.Accel_Z * MPU6050_Data.Accel_Y, -2 * MPU6050_Data.Accel_X * MPU6050_Data.Accel_X - 2 * MPU6050_Data.Accel_Y * MPU6050_Data.Accel_Y + 1) * 57.3;
    //������ٶ�
    MPU6050_Data.Gyro_X = MPU6050_Data.Gyro_X / 131.0f;
    MPU6050_Data.Gyro_Y = MPU6050_Data.Gyro_Y / 131.0f;
    MPU6050_Data.Gyro_Z = MPU6050_Data.Gyro_Z / 131.0f;
//    //������ٶȻ���
//    MPU6050_Data.Gyro_X_Int = MPU6050_Data.Gyro_X_Int + MPU6050_Data.Gyro_X * 0.01;
//    MPU6050_Data.Gyro_Y_Int = MPU6050_Data.Gyro_Y_Int + MPU6050_Data.Gyro_Y * 0.01;
//    MPU6050_Data.Gyro_Z_Int = MPU6050_Data.Gyro_Z_Int + MPU6050_Data.Gyro_Z * 0.01;
//    //������ٶȻ�������
//    MPU6050_Data.Gyro_X_Int = MPU6050_Data.Gyro_X_Int + 0.0001 * MPU6050_Data.Roll;
//    MPU6050_Data.Gyro_Y_Int = MPU6050_Data.Gyro_Y_Int - 0.0001 * MPU6050_Data.Pitch;
//    MPU6050_Data.Gyro_Z_Int = MPU6050_Data.Gyro_Z_Int - 0.0001 * MPU6050_Data.Yaw;
//


}
