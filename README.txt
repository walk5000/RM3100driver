
说明：
1、六个文件中RM3100.c与RM3100.h提供API调用，用户无需修改；
2、smm_demo_iic.c中提供I2C接口单次采样示例，cmm_demo_iic.c中提供I2C接口周期采样示例；
3、smm_demo_spi.c中提供SPI接口单次采样示例，cmm_demo_spi.c中提供SPI接口周期采样示例；
4、I2C接口示例中读取DRDY引脚判断数据状态；SPI接口示例中读取STATUS寄存器判断数据状态；
5、用户需根据自己的MCU完成I2C读写、SPI读写及定时器调用等功能；
