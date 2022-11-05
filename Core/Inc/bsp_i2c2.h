#ifndef __I2C_EEPROM_H__
#define	__I2C_EEPROM_H__

#include "main.h"
#include "stm32f1xx_hal.h"
#include <inttypes.h>

#define I2C_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C_GPIO_PORT                       EXP_SCL_GPIO_Port //待修改
#define I2C_SCL_PIN                         EXP_SCL_Pin //待修改
#define I2C_SDA_PIN                         EXP_SDA_Pin //待修改

#define I2C_SCL_HIGH()                      HAL_GPIO_WritePin(I2C_GPIO_PORT,I2C_SCL_PIN,GPIO_PIN_SET)    // ����ߵ�ƽ
#define I2C_SCL_LOW()                       HAL_GPIO_WritePin(I2C_GPIO_PORT,I2C_SCL_PIN,GPIO_PIN_RESET)  // ����͵�ƽ
#define I2C_SDA_HIGH()                      HAL_GPIO_WritePin(I2C_GPIO_PORT,I2C_SDA_PIN,GPIO_PIN_SET)    // ����ߵ�ƽ
#define I2C_SDA_LOW()                       HAL_GPIO_WritePin(I2C_GPIO_PORT,I2C_SDA_PIN,GPIO_PIN_RESET)  // ����͵�ƽ
#define I2C_SDA_READ()                      HAL_GPIO_ReadPin(I2C_GPIO_PORT,I2C_SDA_PIN)



void    I2C_Start2(void);
void    I2C_Stop2(void);
void    I2C_SendByte2(uint8_t _ucByte);
uint8_t I2C_ReadByte2(uint8_t ack);
uint8_t I2C_WaitAck2(void);
void    I2C_Ack2(void);
void    I2C_NAck2(void);

#endif /* __I2C_EEPROM_H__ */

