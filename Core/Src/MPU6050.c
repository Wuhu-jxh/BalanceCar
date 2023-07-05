//
// Created by mulai on 2023/6/18.
//

#include <math.h>
#include "MPU6050.h"
#include "i2c.h"
#include "Serial.h"
#include "settings.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu.h"
#include "invensense.h"

#define I2C_Channel hi2c2 //���嵱ǰʹ�õ�I2C��ͨ��

_MPU6050_DATA MPU6050_Data;
static int16_t MPU6050Addr = 0xD0;//�����ʼ��MPU6050�Ĵӻ���ַ������̶�����֪�Ϳ�ֱ���ö����������ȥ����I2C_Serch����


static struct platform_data_s gyro_pdata = {
        .orientation = { 1, 0, 0,
                         0, 1, 0,
                         0, 0, 1}
};
int set_int_enable(unsigned char enable);





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
int8_t I2C_Readn(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                 unsigned char *data_ptr)
{
    return HAL_I2C_Mem_Read(&I2C_Channel,slave_addr,reg_addr,sizeof(reg_addr),data_ptr,len,1000);
}
int8_t I2C_Writen(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                  unsigned char *data_ptr){
    return HAL_I2C_Mem_Write(&I2C_Channel,slave_addr,reg_addr,sizeof(reg_addr),data_ptr,len,1000);
}
int8_t I2C_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData,uint8_t Datalen)
{
  return HAL_I2C_Mem_Write(&I2C_Channel,DevAddr,MemAddr,1,iData,Datalen,1000);
}
int8_t I2C_Writei(uint16_t DevAddr, uint16_t MemAddr, uint8_t iData,uint8_t Datalen)
{
    return HAL_I2C_Mem_Write(&I2C_Channel,DevAddr,MemAddr,1,&iData,Datalen,1000);
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

void MPU6050_DMP_init()
{
     mpu_init(NULL); //�����ƺ�������Ҫʲô����
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);//���ô�����
        mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//����FIFO
        mpu_set_sample_rate(DEFAULT_MPU_HZ);//���ò�����
        dmp_load_motion_driver_firmware();//����DMP�̼�
        dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));//���������Ƿ���
        dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                           DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                           DMP_FEATURE_GYRO_CAL);//����DMP����
        dmp_set_fifo_rate(DEFAULT_MPU_HZ);//����FIFO����
        mpu_set_dmp_state(1);//ʹ��DMP
        dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);//����DMP�ж�ģʽ
        set_int_enable(1);//ʹ��MPU6050�ж�

}

void MPU6050_Read_DMP(void)
{
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    float q0,q1,q2,q3;
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
    int q30 = 1073741824;
    q0 = quat[0] / q30;
    q1 = quat[1] / q30;
    q2 = quat[2] / q30;
    q3 = quat[3] / q30;
    MPU6050_Data.Quat_W = q0;
    MPU6050_Data.Quat_X = q1;
    MPU6050_Data.Quat_Y = q2;
    MPU6050_Data.Quat_Z = q3;
    MPU6050_Data.Gyro_X = gyro[0];
    MPU6050_Data.Gyro_Y = gyro[1];
    MPU6050_Data.Gyro_Z = gyro[2];
    MPU6050_Data.Accel_X = accel[0];
    MPU6050_Data.Accel_Y = accel[1];
    MPU6050_Data.Accel_Z = accel[2];
    MPU6050_Data.Time = sensor_timestamp;
    MPU6050_Data.Sensors = sensors;
    MPU6050_Data.More = more;
    //��ȡ��̬��
    MPU6050_Data.DMP_Roll = atan2(2 * q0 * q1 + q2 * q3, q0 * q0 + q3 * q3 - q1 * q1 - q2 * q2) * 57.3;
    MPU6050_Data.DMP_Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
    MPU6050_Data.DMP_Yaw = atan2(2 * q1 * q2 + q0 * q3, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
}
