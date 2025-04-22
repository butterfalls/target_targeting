#include "main.h"
#include "OLED_Font.h"
#include "i2c.h"

// 定义OLED的I2C地址
#define OLED_ADDRESS 0x78

/*引脚配置*/
// 删除软件I2C的引脚定义
// #define SCL_Pin GPIO_Pin_6
// #define SCL_GPIO_Port GPIOB
// #define SDA_Pin GPIO_Pin_7
// #define SDA_GPIO_Port GPIOB
// #define OLED_W_SCL(x) HAL_GPIO_WritePin(GPIOB, SCL_Pin, (GPIO_PinState)(x))
// #define OLED_W_SDA(x) HAL_GPIO_WritePin(GPIOB, SDA_Pin, (GPIO_PinState)(x))

/*引脚初始化*/
void OLED_I2C_Init(void)
{
	// 使用硬件I2C1，不需要手动初始化GPIO
	// 在CubeMX中已经配置好了I2C1
}

/**
 * @brief  I2C开始
 * @param  无
 * @retval 无
 */
void OLED_I2C_Start(void)
{
	// 使用硬件I2C，不需要手动控制开始信号
}

/**
 * @brief  I2C停止
 * @param  无
 * @retval 无
 */
void OLED_I2C_Stop(void)
{
	// 使用硬件I2C，不需要手动控制停止信号
}

/**
 * @brief  I2C发送一个字节
 * @param  Byte 要发送的一个字节
 * @retval 无
 */
void OLED_I2C_SendByte(uint8_t Byte)
{
	HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, &Byte, 1, 100);
}

/**
 * @brief  OLED写命令
 * @param  Command 要写入的命令
 * @retval 无
 */
void OLED_WriteCommand(uint8_t Command)
{
	uint8_t buf[2] = {0x00, Command}; // 0x00表示写命令
	HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, buf, 2, 100);
}

/**
 * @brief  OLED写数据
 * @param  Data 要写入的数据
 * @retval 无
 */
void OLED_WriteData(uint8_t Data)
{
	uint8_t buf[2] = {0x40, Data}; // 0x40表示写数据
	HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, buf, 2, 100);
}

/**
 * @brief  OLED设置光标位置
 * @param  Y 以左上为原点，向下方向的坐标，范围：0~7
 * @param  X 以左上为原点，向右方向的坐标，范围：0~127
 * @retval 无
 */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);				 //设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4)); //设置X位置4位
	OLED_WriteCommand(0x00 | (X & 0x0F));		 //设置X位置4位
}

/**
 * @brief  OLED清屏
 * @param  无
 * @retval 无
 */
void OLED_Clear(void)
{
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for (i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/**
 * @brief  OLED部分清屏
 * @param  Line 行位，范围：1~4
 * @param  start 列开始位，范围：1~16
 * @param  end 列开始位，范围：1~16
 * @retval 无
 */
void OLED_Clear_Part(uint8_t Line, uint8_t start, uint8_t end)
{
	uint8_t i, Column;
	for (Column = start; Column <= end; Column++)
	{
		OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8); //设置光标位置在上半部
		for (i = 0; i < 8; i++)
		{
			OLED_WriteData(0x00); //显示上半部分内
		}
		OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8); //设置光标位置在下半部
		for (i = 0; i < 8; i++)
		{
			OLED_WriteData(0x00); //显示下半部分内
		}
	}
}

/**
 * @brief  OLED显示一个字
 * @param  Line 行位，范围：1~4
 * @param  Column 列位，范围：1~16
 * @param  Char 要显示的一个字，范围：ASCII见字符
 * @retval 无
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8); //设置光标位置在上半部
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]); //显示上半部分内
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8); //设置光标位置在下半部
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]); //显示下半部分内
	}
}

/**
 * @brief  OLED显示字符串
 * @param  Line 起始行位，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  String 要显示的字符串，范围：ASCII见字符
 * @retval 无
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/**
  * @brief  OLED显示一个汉字
  * @param  Line 行位，范围：1~4
  * @param  Column 列位，范围：1~16
  * @param  Chinese 要显示的汉字在字库数组中的位置
  * @retval 无
  */
void OLED_ShowWord(uint8_t Line, uint8_t Column, uint8_t Chinese)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F16x16[Chinese][i]);	
	}
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8 + 8);
	for (i = 1; i < 8; i++)
	{
		OLED_WriteData(OLED_F16x16[Chinese][i+8]);	
	}
	OLED_SetCursor((Line - 1) * 2 +1, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F16x16[Chinese][i+16]);
	}
	OLED_SetCursor((Line - 1) * 2 +1, (Column - 1) * 8 + 8);
	for (i = 1; i < 8; i++)
	{
		OLED_WriteData(OLED_F16x16[Chinese][i+16+8]);
	}
}

/**
  * @brief  OLED显示一串中文字
  * @param  Line 行位，范围：1~4
  * @param  Column 列位，范围：1~16
  * @param  Chinese[] 要显示的汉字在字库数组中的位置，数组里放每个字的位置
  * @param	Length 要显示中文的长度，范围：1~8
  * @retval 无
  */
void OLED_ShowChinese(uint8_t Line, uint8_t Column, uint8_t *Chinese,uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowWord(Line, Column + i*2,Chinese[i]);
	}
}


/**
 * @brief  OLED次方函数
 * @retval 返回值等于X的Y次方
 */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
 * @brief  OLED显示数字（十进制，无小数）
 * @param  Line 起始行位，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~4294967295
 * @param  Length 要显示数字的长度，范围：1~10
 * @retval 无
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
 * @brief  OLED显示数字（十进制，带符号数）
 * @param  Line 起始行位，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：-2147483648~2147483647
 * @param  Length 要显示数字的长度，范围：1~10
 * @retval 无
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
 * @brief  OLED显示数字（十六进制，无小数）
 * @param  Line 起始行位，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
 * @param  Length 要显示数字的长度，范围：1~8
 * @retval 无
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
 * @brief  OLED显示数字（二进制，无小数）
 * @param  Line 起始行位，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
 * @param  Length 要显示数字的长度，范围：1~16
 * @retval 无
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
 * @brief  OLED初始化
 * @param  无
 * @retval 无
 */
void OLED_Init(void)
{
	// 增加上电延时
	HAL_Delay(100);  // 使用HAL_Delay替代for循环延时

	// 初始化I2C
	OLED_I2C_Init();

	// 等待OLED稳定
	HAL_Delay(100);

	OLED_WriteCommand(0xAE); //关闭显示

	OLED_WriteCommand(0xD5); //设置显示时钟分比/震荡器频率
	OLED_WriteCommand(0x80);

	OLED_WriteCommand(0xA8); //设置多路复用
	OLED_WriteCommand(0x3F);

	OLED_WriteCommand(0xD3); //设置显示偏移
	OLED_WriteCommand(0x00);

	OLED_WriteCommand(0x40); //设置显示开始

	OLED_WriteCommand(0xA1); //设置左右方向0xA1正常 0xA0左右反置

	OLED_WriteCommand(0xC8); //设置上下方向0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA); //设置COM引脚件配
	OLED_WriteCommand(0x12);

	OLED_WriteCommand(0x81); //设置对比度控
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9); //设置预充电周
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB); //设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4); //设置整个显示打开/关闭

	OLED_WriteCommand(0xA6); //设置正常/倒转显示

	OLED_WriteCommand(0x8D); //设置充电
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF); //开显示

	OLED_Clear(); // OLED清屏
	
	// 等待显示稳定
	HAL_Delay(100);
}
