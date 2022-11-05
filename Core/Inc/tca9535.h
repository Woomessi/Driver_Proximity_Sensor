#include <vl6180x_types.h>

#define EXPANDER_ADDRESS 0x40//已左移一位
#define  TCA9535_INPUT_PORT0_REG        0x00
#define  TCA9535_INPUT_PORT1_REG        0x01
#define  TCA9535_OUTPUT_PORT0_REG       0x02
#define  TCA9535_OUTPUT_PORT1_REG       0x03
#define  TCA9535_INVERSION_PORT0_REG    0x04
#define  TCA9535_INVERSION_PORT1_REG    0x05
#define  TCA9535_CONFIG_PORT0_REG       0x06
#define  TCA9535_CONFIG_PORT1_REG       0x07
#define  TCA9535_CONFIG_VAL0      0xE0
#define  TCA9535_CONFIG_VAL1      0xFB

int TCA9535_WrByte(uint8_t dev, uint8_t index, uint8_t data);

int TCA9535_RdByte(uint8_t dev, uint8_t index, uint8_t *data);

int I2C2Write(uint8_t addr, uint8_t *buff, uint8_t len);

int I2C2Read(uint8_t addr, uint8_t *buff, uint8_t len);
