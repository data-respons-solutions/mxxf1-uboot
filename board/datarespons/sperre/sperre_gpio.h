#ifndef __SPERRE_GPIO_H__
#define __SPERRE_GPIO_H__

/* Enet */
#define ENET_CRS_DV 		IMX_GPIO_NR(1, 25)	/* Enet Phy reset */
#define ENET_RXD1			IMX_GPIO_NR(1, 26)

#define LCD_PPEN			IMX_GPIO_NR(1, 0)	/* Output active low */
#define BL_EN				IMX_GPIO_NR(4, 15)	/* Output active low */
#define GPIO_Debug_LED		IMX_GPIO_NR(1, 2)	/* Output active high */
#define RTC_IRQ				IMX_GPIO_NR(1, 4)	/* Output active high */
#define IO_INT				IMX_GPIO_NR(1, 5)	/* Input high */
#define MMC_RST				IMX_GPIO_NR(1, 7)	/* Output active high */
#define IO_RESET			IMX_GPIO_NR(1, 8)	/* Output active high */
#define TOUCH_RES			IMX_GPIO_NR(7, 12)	/* Output active high */
#define TOUCH_IRQ			IMX_GPIO_NR(1, 1)	/* Output active high */
#define PMIC_INT_B			IMX_GPIO_NR(7, 13)	/* Output active high */
#define RS485_PW			IMX_GPIO_NR(4, 5)	/* Output active high */
#define ADS1248_RESET		IMX_GPIO_NR(1, 9)	/* Output active high */
#define ADS1248_START		IMX_GPIO_NR(1, 16)	/* Output active high */
#define SPI_NOR_WP			IMX_GPIO_NR(4, 14)	/* Output active high */

#endif
