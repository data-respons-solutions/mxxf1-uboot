#ifndef __SPERRE_GPIO_H__
#define __SPERRE_GPIO_H__

/* Enet */
#define GPIO_ENET_CRS_DV 		IMX_GPIO_NR(1, 25)	/* Enet Phy reset */
#define GPIO_ENET_RXD1			IMX_GPIO_NR(1, 26)

#define GPIO_LCD_PPEN			IMX_GPIO_NR(1, 0)	/* LCD Power Enable, Output active low */
#define GPIO_BL_EN				IMX_GPIO_NR(4, 15)	/* BL - LCD back light, Output active low */
#define GPIO_Debug_LED			IMX_GPIO_NR(1, 2)	/* Output active high */
#define GPIO_RTC_IRQ			IMX_GPIO_NR(1, 4)	/* Output active high */
#define GPIO_IO_INT				IMX_GPIO_NR(1, 5)	/* Input high */
#define GPIO_MMC_RST			IMX_GPIO_NR(1, 7)	/* Output active high */
#define GPIO_IO_RESET			IMX_GPIO_NR(1, 8)	/* Output active high */
#define GPIO_TOUCH_RES			IMX_GPIO_NR(7, 12)	/* Output active high */
#define GPIO_TOUCH_IRQ			IMX_GPIO_NR(1, 1)	/* Output active high */
#define GPIO_PMIC_INT_B			IMX_GPIO_NR(7, 13)	/* Output active high */
#define GPIO_RS485_PW			IMX_GPIO_NR(4, 5)	/* Output active high */
#define GPIO_ADS1248_RESET		IMX_GPIO_NR(1, 9)	/* Output active high */
#define GPIO_ADS1248_START		IMX_GPIO_NR(1, 16)	/* Output active high */
#define GPIO_SPI_NOR_WP			IMX_GPIO_NR(4, 14)	/* Output active high */

#define GPIO_USB_OTG_PWR		IMX_GPIO_NR(3, 22)	/* Output active low - EIM_D22__USB_OTG_PWR */
#define GPIO_USB_H1_PWR			IMX_GPIO_NR(3, 31)	/* Output active low - EIM_D31__GPIO3_IO31 */

#endif
