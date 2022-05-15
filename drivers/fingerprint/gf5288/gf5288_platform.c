/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf5288_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf5288_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	const char *fp_project_name;
	
	/*get vdd_en resource*/
	gf_dev->vdd_en_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_vdd_en", 0);
	if (!gpio_is_valid(gf_dev->vdd_en_gpio)) {
		pr_info("VDD_EN GPIO is invalid.\n");
		return ERROR;
	}
	
	rc = gpio_request(gf_dev->vdd_en_gpio, "goodix_vdd_en");
	if (rc) {
		pr_info("Failed to request VDD_EN GPIO. rc = %d\n", rc);
		return ERROR;
	}
	/*rc = gpio_direction_output(gf_dev->vdd_en_gpio, 1);
    if (rc){
		pr_info ("bio_fp_error can not set vreg\n");
		return ERROR;
    }*/

	/*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_info ("RESET GPIO is invalid.\n");
		return ERROR;
	}

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_info("Failed to request RESET GPIO. rc = %d\n", rc);
		return ERROR;
	}

    gpio_direction_output(gf_dev->reset_gpio, 0);

	/*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_irq", 0);
	pr_info("gf::irq_gpio:%d\n", gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_info("IRQ GPIO is invalid.\n");
		return ERROR;
	}
	
	printk("gf_dev->irq_gpio = %d\n", gf_dev->irq_gpio);
	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_info("Failed to request IRQ GPIO. rc = %d\n", rc);
		return ERROR;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	rc = of_property_read_string(gf_dev->spi->dev.of_node, "vivo,project-name", &fp_project_name);
	if (rc) {
		printk("%s:vivo,project-name property do not find\n", __func__);
		fp_project_name = "default";
	}
	printk("%s:vivo,project-name = %s\n", __func__, fp_project_name);
	if (!strncmp(fp_project_name, "PD1730C", 7)) {
		gf_dev->vreg = regulator_get(&gf_dev->spi->dev, "vcc_spi");
		if (IS_ERR(gf_dev->vreg)) {
			printk("gf5288 unable to get vcc_spi\n");
		} else {
			if (regulator_count_voltages(gf_dev->vreg) > 0) {
			rc = regulator_set_voltage(gf_dev->vreg, 3000000, 3000000);
				if (rc) {
					printk("gf5288 unable to set voltage on vcc_spi");
				}
			}
			regulator_set_load(gf_dev->vreg, 20000);
			rc = regulator_enable(gf_dev->vreg);
			if (rc) {
				printk("gf5288 error enabling vdd_ana %d\n", rc);
				regulator_put(gf_dev->vreg);
				gf_dev->vreg = NULL;
			}
			printk("gf5288 set voltage on vcc_spi for goodix fingerprint");
		}
	}

	return 0;
}

void gf5288_cleanup(struct gf_dev *gf_dev)
{
	pr_info ("[info] %s\n", __func__);
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf5288_power_on(struct gf_dev *gf_dev)
{
    int rc = 0;
	
	rc = gpio_direction_output(gf_dev->vdd_en_gpio, 1);
    if (rc) {
		pr_info ("gf5288 power on fail.\n");
		return ERROR;
    }
	pr_info("gf:%s, exit\n", __func__);
	
    return rc;
}

int gf5288_power_off(struct gf_dev *gf_dev)
{
    int rc = 0;
	
	rc = gpio_direction_output(gf_dev->vdd_en_gpio, 0);
    if (rc) {
		pr_info ("gf5288 power off fail.\n");
		return ERROR;
    }
	pr_info("gf:%s, exit\n", __func__);
	
    return rc;
}

int gf5288_hw_get_power_state(struct gf_dev *gf_dev)
{

	int retval = 0;
	retval = gpio_get_value(gf_dev->vdd_en_gpio);
	pr_info("gf:%s, retval=%d\n", __func__, retval);
	return retval;
}

int gf5288_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return ERROR;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(10);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf5288_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return ERROR;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

