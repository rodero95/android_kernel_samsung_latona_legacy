/* linux/arch/arm/mach-omap2/wl127x_wifi.c
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>
#include <linux/wl12xx.h>

#include "wl127x_wifi.h"
#include "mux.h"

void config_wlan_mux(void)
{
	/* WLAN PW_EN and IRQ */
	omap_mux_init_gpio(WL127X_WIFI_PMENA_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(WL127X_WIFI_IRQ_GPIO, OMAP_PIN_INPUT);

	/* MMC3 */
	omap_mux_init_signal("etk_clk.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcspi1_cs1.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_d4.sdmmc3_dat0", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_d5.sdmmc3_dat1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_d6.sdmmc3_dat2", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_d3.sdmmc3_dat3", OMAP_PIN_INPUT_PULLUP);
}

static struct wl12xx_platform_data wl127x_wlan_data __initdata = {
	.irq = -1, /* OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),*/
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
};

static void wl127x_wifi_init(void)
{
	int ret;

	pr_info("%s: start\n", __func__);
	ret = gpio_request(WL127X_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			WL127X_WIFI_IRQ_GPIO);
	}
	ret = gpio_request(WL127X_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			WL127X_WIFI_PMENA_GPIO);
		gpio_free(WL127X_WIFI_IRQ_GPIO);
	}
	gpio_direction_input(WL127X_WIFI_IRQ_GPIO);
	gpio_direction_output(WL127X_WIFI_PMENA_GPIO, 0);
	wl127x_wlan_data.irq = OMAP_GPIO_IRQ(WL127X_WIFI_IRQ_GPIO);
	
	if (wl12xx_set_platform_data(&wl127x_wlan_data))
	{
		pr_err("Error setting wl12xx data\n");
	}
	printk("Wifi init done\n");
}


device_initcall(wl127x_wifi_init);