#include "tca9535.h"

int TCA9535_WrByte(uint8_t dev, uint8_t index, uint8_t data){
    int status;
    int a = 5;
    uint8_t *buffer = &a;//初始化指针

    buffer[0] = index;
    buffer[1] = data;

    status = I2C2Write(dev, buffer,(uint8_t)2);
    return status;
}

int TCA9535_RdByte(uint8_t dev, uint8_t index, uint8_t *data){
    int status;
    int b = 6;
    uint8_t *buffer = &b;//初始化指针
    buffer[0] = index;//8位寄存器地址
    status = I2C2Write(dev, buffer, (uint8_t)1);//向I2C总线发送设备地址及8位寄存器地址
    if( !status ){
        status = I2C2Read(dev, buffer,1);//读取该寄存器中的值，并将其返回到buffer中
        if( !status ){
            *data = (uint8_t)buffer[0];//将buffer中的值传给data
        }
    }

    return status;
}

//I2C底层通讯函数选择
/* VL6180X底层驱动函数：写 */
int I2C2Write(uint8_t addr, uint8_t *buff, uint8_t len)
{
  int status, i;
  I2C_Start2();//开启I2C总线
  I2C_SendByte2(addr | 0);//发送8位设备地址及LSB的写方向位，实现写功能
  if (I2C_WaitAck2())//如果从机没有应答
  {
    I2C_Stop2();//关闭I2C总线
    return 1;//返回报错标志
  }
  for (i = 0; i < len; i++)
  {
    I2C_SendByte2(buff[i]);//发送buff中的各字节
    if (i == len - 1)//在发送最后一个字节的数据时
    {
      if (I2C_WaitAck2())//等待从机应答
      {
        I2C_Stop2();//关闭I2C总线
        status = 1;//返回报错标志
      }
    }
    else
    {
      status = I2C_WaitAck2();//返回报错标志
    }
  }
  I2C_Stop2();//关闭I2C总线
  return status;
}

/* VL6180X底层驱动函数：读 */
int I2C2Read(uint8_t addr, uint8_t *buff, uint8_t len)
{
  int status, i;
  I2C_Start2();//开启I2C总线
  I2C_SendByte2(addr | 1);//发送8位设备地址及LSB的读方向位，实现读功能
  I2C_WaitAck2();//等待从机应答
  for (i = 0; i < len; i++)
  {
    if (i == len - 1)//在读取最后一个字节的数据时
    {
      buff[i] = I2C_ReadByte2(0);//读取数据，将其保存于buff[i]，并向从机发送非应答信号，终止数据读取
    }
    else
    {
      buff[i] = I2C_ReadByte2(1);//读取数据，将其保存于buff[i]，并向从机发送应答信号，继续读取数据
    }
  }
  I2C_Stop2();//关闭I2C总线
  return status = 0;
}
