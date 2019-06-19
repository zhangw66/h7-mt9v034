#include "i2c_gpio.h"
void delay_us_poll_systick(uint32_t us)
{
	uint32_t need_ticks = 0, expend_ticks = 0;
	uint32_t told, tnow = 0;
	need_ticks = us * 480;
	//__disable_irq();
	told = SysTick->VAL;
	while (1)
	{
		tnow = SysTick->VAL;
		if (tnow != told)
		{
			if (tnow < told)
				expend_ticks += told - tnow;
			else
				expend_ticks += SysTick->LOAD - tnow + told;
			told = tnow;
			if (expend_ticks >= need_ticks)
				break;
		}
	};
	//__enable_irq();
}
void i2c_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
#if 0
    GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_Initure.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);
    IIC_SDA = 1;
    IIC_SCL = 1;
	#else
	/* USER CODE END I2C3_MspInit 0 */

	__HAL_RCC_GPIOF_CLK_ENABLE();
	GPIO_InitStruct.Pin = I2C_GPIO_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(I2C_GPIO_SDA_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = I2C_GPIO_SCL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(I2C_GPIO_SCL_PORT, &GPIO_InitStruct);
#endif
}

static void i2c_master_gen_start(void)
{
	SDA_OUT();
	IIC_SDA(1);
	IIC_SCL(1);
	i2c_delay(4);
 	IIC_SDA(0);//START:when CLK is high,DATA change form high to low
	i2c_delay(4);
	IIC_SCL(0);
}

static void i2c_master_gen_stop(void)
{
	SDA_OUT();
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	i2c_delay(4);
	IIC_SCL(1);
	i2c_delay(4);
	IIC_SDA(1);
}
/*
	non-zero is failure
*/
static int8_t i2c_master_wait_ack(void)
{
	uint8_t err_times = 0;
	SDA_IN();
	IIC_SDA(1);
	i2c_delay(1);
	IIC_SCL(1);
	i2c_delay(1);
	while(READ_SDA) {
		err_times++;
		if(err_times > 250) {
			i2c_master_gen_stop();
			return -1;
		}
	}
	IIC_SCL(0);
	return 0;
}
static void i2c_master_gen_ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	i2c_delay(2);
	IIC_SCL(1);
	i2c_delay(2);
	IIC_SCL(0);
}
static void i2c_master_gen_nack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	i2c_delay(2);
	IIC_SCL(1);
	i2c_delay(2);
	IIC_SCL(0);
}
static void i2c_master_send_byte(uint8_t data)
{
    uint8_t i;
	SDA_OUT();
    IIC_SCL(0);
    for(i = 0; i < 8; i++)
    {
        IIC_SDA((data & 0x80) >> 7);
        data <<= 1;
		i2c_delay(2);
		IIC_SCL(1);
		i2c_delay(2);
		IIC_SCL(0);
		i2c_delay(2);
    }
}
static uint8_t i2c_master_read_byte(uint8_t end_level)
{
	uint8_t i,recv=0;
	SDA_IN();
    for(i = 0; i < 8; i++) {
        IIC_SCL(0);
        i2c_delay(2);
		IIC_SCL(1);
        recv <<= 1;
        if(READ_SDA)
			recv++;
		i2c_delay(1);
    }
    if(end_level)
        i2c_master_gen_nack();
    else
        i2c_master_gen_ack();
    return recv;
}
HAL_StatusTypeDef i2c_master_transmit(uint16_t dev_addr,uint8_t *pdata, uint16_t size)
{
	uint16_t i = 0;
	i2c_master_gen_start();
	i2c_master_send_byte(dev_addr | 0x00);    //slaveaddr
	if (i2c_master_wait_ack())
		return HAL_TIMEOUT;
	for (i = 0; i < size; i++) {
		i2c_master_send_byte(pdata[i]);
		if (i2c_master_wait_ack())
			return HAL_TIMEOUT;
	}
	i2c_master_gen_stop();
	return HAL_OK;
}
HAL_StatusTypeDef i2c_master_receive(uint16_t dev_addr,uint8_t *pdata, uint16_t size)
{
	uint16_t i = 0;
	i2c_master_gen_start();
	i2c_master_send_byte(dev_addr | 0x01);    //slaveaddr,mode:read
	if (i2c_master_wait_ack())
		return HAL_TIMEOUT;
	for (i = 0; i < size - 1; i++) {
		pdata[i] = i2c_master_read_byte(I2C_MASTER_READ_WTTH_ACK);     //ack
	}
	pdata[i] = i2c_master_read_byte(I2C_MASTER_READ_WITH_NACK);     		//noack
	i2c_master_gen_stop();
	return HAL_OK;
}

HAL_StatusTypeDef i2c_master_read_mem(uint16_t dev_addr,uint16_t mem_addr,uint8_t *pdata, uint16_t size)
{
	uint16_t i = 0;
	i2c_master_gen_start();
	i2c_master_send_byte(dev_addr | 0x00);    //slaveaddr,mode:write
	if (i2c_master_wait_ack())
		return HAL_TIMEOUT;
	i2c_master_send_byte(mem_addr);    //memery address we should read from
	if (i2c_master_wait_ack())
		return HAL_TIMEOUT;
	i2c_master_gen_start();
	i2c_master_send_byte(dev_addr | 0x01);    //slaveaddr,mode:write
	if (i2c_master_wait_ack())
		return HAL_TIMEOUT;
	for (i = 0; i < size - 1; i++) {
		pdata[i] = i2c_master_read_byte(I2C_MASTER_READ_WTTH_ACK);     //ack
	}
	pdata[i] = i2c_master_read_byte(I2C_MASTER_READ_WITH_NACK);     		//noack
	i2c_master_gen_stop();
	return HAL_OK;
}
