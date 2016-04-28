
/*
PATH=$PATH:/home/jaguar/Documents/Emb_linux/em_lin/practice/Linux_srcARM/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/

sudo make ARCH=arm  CROSS_COMPILE=/home/jaguar/Documents/Emb_linux/em_lin/practice/Linux_srcARM/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-   -C /root/beagleboneb_emb_linux_2015/emb_linux_kernel1/bb-kernel/KERNEL  M=`pwd` modules

*/


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/device.h>

/* Vdd info */
#define MCP3021_VDD_MAX    	5500
#define MCP3021_VDD_MIN    	2700
#define MCP3021_VDD_REF    	3300

/* output format */
#define MCP3021_SAR_SHIFT    2
#define MCP3021_SAR_MASK    0x3ff

#define MCP3021_OUTPUT_RES    10    /* 10-bit resolution */
#define MCP3021_OUTPUT_SCALE    4

#define MCP3221_SAR_SHIFT    0
#define MCP3221_SAR_MASK    0xfff
#define MCP3221_OUTPUT_RES    12	/* 12-bit resolution */
#define MCP3221_OUTPUT_SCALE    1

enum chips {
    mcp3021,
    mcp3221
};


struct mcp3221_data {
    struct device *hwmon_dev;
    u32 vdd;    /* device power supply */
    u16 sar_shift;
    u16 sar_mask;
    u8 output_res;
    u8 output_scale;
};

static int mcp3221_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    //int err;
    struct mcp3221_data *data = NULL;

        dump_stack();

        //a check like below will be based on our client device hw requirements and 
        //what we need from master/ adapter device controller 
        //
        //who provides this system API ? i2c core - how does the i2c core know the 
        //features of adapter/master device controller ??
        //
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    data = devm_kzalloc(&client->dev, sizeof(struct mcp3221_data),
                GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    i2c_set_clientdata(client, data);

       //based on our id's device specific information, 
       //we initialize hw features in our private data   

    switch (id->driver_data) {
    case mcp3021:
        data->sar_shift = MCP3021_SAR_SHIFT;
        data->sar_mask = MCP3021_SAR_MASK;
        data->output_res = MCP3021_OUTPUT_RES;
        data->output_scale = MCP3021_OUTPUT_SCALE;
        break;

    case mcp3221:
        data->sar_shift = MCP3221_SAR_SHIFT;
        data->sar_mask = MCP3221_SAR_MASK;
        data->output_res = MCP3221_OUTPUT_RES;
        data->output_scale = MCP3221_OUTPUT_SCALE;
        break;
    }
        //this approach is legacy approach 
    if (client->dev.platform_data) {
        data->vdd = *(u32 *)client->dev.platform_data;
        if (data->vdd > MCP3021_VDD_MAX || data->vdd < MCP3021_VDD_MIN)
            return -EINVAL;
    } else
        data->vdd = MCP3021_VDD_REF;


    //assuming that this is the interface that we will be using for 
    //device access, attribute's show will be accessed to actually 
    //access the device 
    // err = sysfs_create_file(&client->dev.kobj, &dev_attr_in0_input.attr);
    // if (err)
    // return err;

/*    data->hwmon_dev = hwmon_device_register(&client->dev);
    if (IS_ERR(data->hwmon_dev)) {
        err = PTR_ERR(data->hwmon_dev);
        goto exit_remove;
    }
*/
    return 0;

//exit_remove:
 //   sysfs_remove_file(&client->dev.kobj, &dev_attr_in0_input.attr);
 //   return err;
}




static int mcp3221_remove(struct i2c_client *client)
{
    struct mcp3221_data *data = i2c_get_clientdata(client);

//    hwmon_device_unregister(data->hwmon_dev);
//    sysfs_remove_file(&client->dev.kobj, &dev_attr_in0_input.attr);

    return 0;
}





static const struct i2c_device_id mcp3221_id[] = {
    { "mcp3021", mcp3021 },
    { "mcp3221", mcp3221 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mcp3221_id);


static struct i2c_driver mcp3221_driver = {
    .driver = {
        .name = "mcp3221_ADCdrv",
    },
    .probe = mcp3221_probe,
    .remove = mcp3221_remove,
    .id_table = mcp3221_id,
};



int adc_init(void)
{
  int ret;
 ret = i2c_add_driver(&mcp3221_driver);
 return ret;

}


void adc_exit(void)
{
     i2c_del_driver(&mcp3221_driver);

}


module_init(adc_init); 
module_exit(adc_exit);

//module_i2c_driver(mcp3221_driver);
MODULE_DESCRIPTION("MCP3221 driver");
MODULE_LICENSE("GPL");
