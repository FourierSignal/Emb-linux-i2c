#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>


//static int bbbgpio_alert_remove(struct platform_device *pdev);

struct bbbgpio_alert_platform_data {
        unsigned int alert_gpio;
        unsigned can_sleep;
        unsigned active_low;
        unsigned default_state;   /* default_state should be one of LEDS_GPIO_DEFSTATE_(ON|OFF|KEEP) */
        const char *name;
};

struct bbbgpio_alert_platform_data *pdata=NULL;

static irqreturn_t bbbgpio_alert_ISR(int irq, void *dev_id)
{
	printk("bbbgpio_alert_ISR: irq=%d\n",irq);
		/*
		printk("Interrupt on ON_LED mod\n");
		*/
		return IRQ_HANDLED;

		return IRQ_NONE;
}


static const struct of_device_id of_bbbgpio_alert_match[] = {
        { .compatible = "bbbgpio_alert", },
        {},
};


static int bbbgpio_alert_probe(struct platform_device *pdev)
{

	struct device_node *np=NULL, *child=NULL;
        struct pinctrl *pinctrl=NULL;
        unsigned int count=0;
	int  ret = 0;

	printk(KERN_ALERT "In bbbgpio_alert_probe: \n");
{
	const char *devname;
 	devname = dev_name(&(pdev->dev));
	printk(KERN_ALERT "devname=%s\n",devname);
	printk(KERN_ALERT "devname=%s\n",kobject_name(&( (pdev->dev.kobj))) );
}
        pinctrl = devm_pinctrl_get_select_default(&pdev->dev);		
        if (IS_ERR(pinctrl))
        {
          dev_warn(&pdev->dev,"pins are not configured from the driver\n");
	  printk(KERN_ALERT "pinctrl=%ld\n",PTR_ERR(pinctrl));
        }

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct bbbgpio_alert_platform_data),GFP_KERNEL);
	if (!pdata)
        return(-ENOMEM);

	np = pdev->dev.of_node;
        count = of_get_child_count(np);
        if (!count)
                return (-ENODEV);
	printk("child-count=%d\n",count);
    for_each_child_of_node(np, child){
        pdata->alert_gpio = of_get_gpio(child, 0);
	printk("gpio-%d \n",pdata->alert_gpio);

	pdata->name = of_get_property(child, "label", NULL) ? : child->name;
        printk("pdata->name=%s\n",pdata->name);

	ret = gpio_request_one( pdata->alert_gpio,GPIOF_IN,pdata->name);  		
//	ret = gpio_request_one( pdata->alert_gpio,GPIOF_IN,"gpio_int");  		
        if(ret < 0)
           return ret;
	printk(KERN_ALERT "bbbgpio_alert:gpio_request_one\n");
}

{
	ret = request_irq(gpio_to_irq(pdata->alert_gpio),bbbgpio_alert_ISR,IRQF_TRIGGER_FALLING , "bbbgpio_alert",NULL);
//	ret = request_irq(gpio_to_irq(pdata->alert_gpio),bbbgpio_alert_ISR,IRQF_TRIGGER_RISING , "bbbgpio_alert",NULL);
	//ret = request_irq(gpio_to_irq(pdata->alert_gpio),bbbgpio_alert_ISR,IRQF_TRIGGER_LOW , "bbbgpio_alert",NULL);
	if(ret != 0)
		printk("\n Request_irq Failed %d\n",ret);
	printk("irq_no=%d\n",gpio_to_irq(pdata->alert_gpio) );
};
	gpio_export(pdata->alert_gpio,0);
	platform_set_drvdata(pdev, pdata);
   
	printk(KERN_ALERT "bbbgpio_alert:probe \n");
return 0;
}


static int bbbgpio_alert_remove(struct platform_device *pdev)
{

	gpio_unexport(pdata->alert_gpio);
	free_irq(gpio_to_irq(pdata->alert_gpio),NULL);
	gpio_free(pdata->alert_gpio );

printk("bbbgpio_alert_remove: done\n");
         return 0;
}


static struct platform_driver bbbgpio_alert_driver = {
        .probe          = bbbgpio_alert_probe,
        .remove         = bbbgpio_alert_remove,
        .driver         = {
                .name   = "bbbgpio_alert_custom",
                .owner  = THIS_MODULE,
                .of_match_table = of_match_ptr(of_bbbgpio_alert_match),
        },
};

static int __init bbbgpio_alert_init(void)
{
	platform_driver_register(&bbbgpio_alert_driver);
	printk(KERN_ALERT "bbbgpio_alert Drv Registered\n");
return 0; 
}

static void  bbbgpio_alert_exit(void)
{
	platform_driver_unregister(&bbbgpio_alert_driver);
	printk(KERN_ALERT "bbb_gpio_alert Drv exit\n");

}

module_init(bbbgpio_alert_init);
module_exit(bbbgpio_alert_exit);
MODULE_LICENSE("GPL");
