#ifndef __SPERRE_GPIO_H__
#define __SPERRE_GPIO_H__

/* Enet */
#define ENET_CRS_DV 		IMX_GPIO_NR(1, 25)
#define ENET_RXD1			IMX_GPIO_NR(1, 26)

#define LCD_PPEN			IMX_GPIO_NR(1, 0)
#define BL_EN				IMX_GPIO_NR(4, 15)
#define GPIO_Debug_LED		IMX_GPIO_NR(1, 2)
#define RTC_IRQ				IMX_GPIO_NR(1, 4)
#define IO_INT				IMX_GPIO_NR(1, 5)
#define MMC_RST				IMX_GPIO_NR(1, 7)
#define IO_RESET			IMX_GPIO_NR(1, 8)
#define TOUCH_RES			IMX_GPIO_NR(7, 12)
#define TOUCH_IRQ			IMX_GPIO_NR(1, 1)
#define PMIC_INT_B			IMX_GPIO_NR(7, 13)
#define RS485_PW			IMX_GPIO_NR(4, 5)
#define ADS1248_RESET		IMX_GPIO_NR(1, 9)
#define ADS1248_START		IMX_GPIO_NR(1, 16)
#define SPI_NOR_WP			IMX_GPIO_NR(4, 14)

#endif
