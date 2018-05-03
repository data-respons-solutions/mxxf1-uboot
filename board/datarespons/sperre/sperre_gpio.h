#ifndef __SPERRE_GPIO_H__
#define __SPERRE_GPIO_H__

/* Enet */
#define GPIO_ENET_PHY_RESET 	IMX_GPIO_NR(6, 14)	/* Enet Phy reset - Output active low*/
#define GPIO_ENET_INT			IMX_GPIO_NR(1, 9)	/* Enet Phy interrupt - input high */

#define GPIO_LCD_PPEN			IMX_GPIO_NR(1, 0)	/* LCD Power Enable, Output active low */
#define GPIO_BL_EN				IMX_GPIO_NR(4, 15)	/* BL - LCD back light, Output active low */
#define GPIO_BL_PWM				IMX_GPIO_NR(1, 21)	/* BL - PWM back light, Output active low */
#define GPIO_Debug_LED			IMX_GPIO_NR(1, 2)	/* Output active high */
#define GPIO_RTC_IRQ			IMX_GPIO_NR(1, 4)	/* Output active low */
#define GPIO_IO_INT				IMX_GPIO_NR(1, 5)	/* Input high */
#define GPIO_MMC_RST			IMX_GPIO_NR(1, 7)	/* Output active low */
#define GPIO_IO_RESET			IMX_GPIO_NR(1, 8)	/* Output active high */
#define GPIO_TOUCH_RES			IMX_GPIO_NR(7, 12)	/* Output active low */
#define GPIO_TOUCH_IRQ			IMX_GPIO_NR(1, 1)	/* Output active low */
#define GPIO_PMIC_INT_B			IMX_GPIO_NR(7, 13)	/* Output active high */
#define GPIO_RS485_PW			IMX_GPIO_NR(4, 5)	/* Output active high */
//#define GPIO_ADS1248_RESET		IMX_GPIO_NR(1, 9)	/* Output active high */
#define GPIO_ADS1248_START		IMX_GPIO_NR(1, 16)	/* Output active high */
#define GPIO_SPI_NOR_WP			IMX_GPIO_NR(4, 14)	/* Output active high */

#define SPI_CS_GPIO				IMX_GPIO_NR(4, 24)	/* Output active low - SPI_NOR_CS */

#define GPIO_USB_OTG_PWR_EN		IMX_GPIO_NR(3, 22)	/* Output active low - EIM_D22__USB_OTG_PWR */
#define GPIO_USB_H1_PWR_EN		IMX_GPIO_NR(3, 31)	/* Output active low - EIM_D31__GPIO3_IO31 */

#endif
