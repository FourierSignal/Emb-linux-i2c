/*
 * TC1321.c - driver for Microchip TC1321 .
 *
 *
 * This driver export the value of Digital value of DAC to sysfs.
 * Through the sysfs interface, lm-sensors tool
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/device.h>
#include<asm/ioctl.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

/* Vdd info */
#define TC1321_VDD_MAX        5500
#define TC1321_VDD_MIN        2700
#define TC1321_VDD_REF        3300

#define DATA_REG_SELECT  0X00
#define CONFIG_REG_SELECT  0X01
#define STDBY 0x01
#define NORMAL 0x00


#define TC1321_DRIVER_TYPE               'M'
#define TC1321_IOC_MODE            _IOW(TC1321_DRIVER_TYPE, 1,unsigned char )
#define TC1321_IOC_WRDATA          _IOW(TC1321_DRIVER_TYPE, 2, unsigned short )
#define TC1321_IOC_RDDATA          _IOR(TC1321_DRIVER_TYPE, 3, unsigned short )
#define TC1321_IOC_MAX 3 

static int N_devices = 1;
module_param(N_devices, int, S_IRUGO);

static dev_t dac_first;

static struct class *dac_class;

enum chips {
    tc1321,
};

/*
 * Client data (each client gets its own)
 */
struct tc1321_data {
    struct device *hwmon_dev;
    u32 vdd;    /* device power supply */
    u16 sar_shift;
    u16 sar_mask;
    u8 output_res;
    struct cdev c_dev;
    dev_t devicenum;
    unsigned int major;
    unsigned int minor;
    const char device_name[30];
struct mutex dac_lock;
struct i2c_client *clientp;
};



static int tc1321_ConfigWrite(struct i2c_client *client,char mode)
{

// struct tc1321_data *data = i2c_get_clientdata(client);
printk(" tc1321_ConfigWrite:\n");
unsigned char i2c_data[2];
int ret=0;
i2c_data[0]=CONFIG_REG_SELECT;
i2c_data[1]= mode;
printk("i2c_data -before send = %x %x \n",i2c_data[0],i2c_data[1]);
ret = i2c_master_send(client, i2c_data,2 );
        if (ret !=  1) {
                dev_err(&client->dev, "i2c write data cmd failed\n");
                return ret < 0 ? ret : -EIO;
        }

return 0;
}





static int tc1321_DataWrite(struct i2c_client *client,void __user *argp)
{

  //struct tc1321_data *data = i2c_get_clientdata(client);
  unsigned char i2c_data[3];
int ret;

printk(" tc1321_DataWrite:\n");

i2c_data[0]= DATA_REG_SELECT;
memcpy(i2c_data+1,argp,2);
printk("i2c_data -before send = %x %x %x\n",i2c_data[0],i2c_data[1],i2c_data[2]);
ret = i2c_master_send(client, i2c_data,3 );
        if (ret != 3) {
                dev_err(&client->dev, "i2c write data cmd failed\n");
                return ret < 0 ? ret : -EIO;
        }

return 0;
}


static int tc1321_DataRead(struct i2c_client *client,void __user *argp)
{
  // struct tc1321_data *data = i2c_get_clientdata(client);
    unsigned char i2c_data[3];
int ret;
printk(" tc1321_DataRead:\n");

i2c_data[0]= DATA_REG_SELECT;
printk("i2c_data -before send = %x %x %x\n",i2c_data[0],i2c_data[1],i2c_data[2]);
ret = i2c_master_send(client, i2c_data,1 );
        if (ret != 1) {
                dev_err(&client->dev, "i2c read data cmd failed\n");
                return ret < 0 ? ret : -EIO;
        }

ret = i2c_master_recv(client, i2c_data+1,2);
if (ret < 0)
   return ret;
if (ret != 2)
   return -EIO;

printk("i2c_data -After Rxve = %x %x %x\n",i2c_data[0],i2c_data[1],i2c_data[2]);
//argp= i2c_data+1;
memcpy(argp,i2c_data+1,2);
return 0;
}


static ssize_t send_input_to_dac(struct device *dev, struct device_attribute *attr,
        char *buf)
{
int ret=0;
    struct i2c_client *client = to_i2c_client(dev);
//    struct tc1321_data *data = i2c_get_clientdata(client);

printk(" send_input_to_dac:\n");
ret=tc1321_DataWrite(client,buf);
    if (ret < 0)
        return ret;
return 0;

}
static ssize_t read_dac_data_reg(struct device *dev, struct device_attribute *attr,
        char *buf)
{
int ret=0;
    struct i2c_client *client = to_i2c_client(dev);
//    struct tc1321_data *data = i2c_get_clientdata(client);

printk(" read_dac_data_reg:\n");
ret= tc1321_DataRead(client,buf);
    if (ret < 0)
        return ret;
return 0;

}
//create attribute for device 
static DEVICE_ATTR(dac_input, S_IWUGO, NULL, send_input_to_dac);
static DEVICE_ATTR(dac_data_reg, S_IRUGO, NULL, read_dac_data_reg);
static int dac_tc1321_open(struct inode *inode, struct file *file)
{

    struct tc1321_data *data;
    int ret=0;
    //printk(" \n");
    printk(" dac_tc1321_open: process %i (%s)\n",current->pid, current->comm);
    //printk(" struct inode *inode=%p  struct file *file=%p\n",inode,file);
    data = container_of(inode->i_cdev,struct tc1321_data, c_dev);
    //printk(" priv dac object = %p  w.r.t cdev of opened device\n",data);
    file->private_data =(void *)data;
    //printk(" keep device object in devicefile's private_data\n");
return ret;

}
static int dac_tc1321_release(struct inode *inode, struct file *file)
{
    struct tc1321_data *data=NULL;
    //printk("\n\n");
    printk(" dac_tc1321_release: process %i (%s)\n",current->pid, current->comm);
    data=(struct tc1321_data *)file->private_data;
    printk("dac priv obj=%p\n",data);
    file->private_data=NULL;
    return 0;
}


//static long dac_tc1321_ioctl(unsigned int tc1321_cmd, unsigned short arg)
static long dac_tc1321_ioctl(struct file *file,unsigned int tc1321_cmd, unsigned long arg)
{

        char  __user *argp = (void __user *)arg;

        struct tc1321_data *data =     file->private_data;
        int ret=0;

        printk(" tc1321_cmd=%x\n",tc1321_cmd);
//        printk("*argp *argp+1 *argp+2 %x %x %x\n",*argp,*(argp+1),*(argp+2));
        if (_IOC_TYPE(tc1321_cmd) != TC1321_DRIVER_TYPE)
        return -ENOTTY;

        if (_IOC_NR(tc1321_cmd) > TC1321_IOC_MAX)
        return -ENOTTY;

        printk("dac_tc1321_ioctl: process %i (%s)\n",current->pid, current->comm);
//      printk("struct file *file=%p\n",file);
//      printk("device object ptr =%p available in private_data of opened device file\n",data);
        //Note -Locking taken care by -i2c Framework

        switch(tc1321_cmd)
        {
        case TC1321_IOC_MODE:
                ret=tc1321_ConfigWrite(data->clientp,*argp );
                printk("TC1321_IOC_STDBY");break; 
        case TC1321_IOC_WRDATA:
                ret= tc1321_DataWrite(data->clientp , argp);                                
                printk("TC1321_IOC_WRDATA");
                printk(": argp argp+1 => %d %d\n",*argp,*(argp+1));
                break; 
        case TC1321_IOC_RDDATA:
                ret= tc1321_DataRead(data->clientp , argp);
                printk("TC1321_IOC_RDDATA");
                printk(": argp argp+1 => %d %d\n",*argp,*(argp+1));
                break; 
        default :
                printk("Unknown ioctl-cmd\n");
                ret=-ENOTTY;
        break;
        }
        return ret;
}

static struct file_operations dac_tc1321_fops = {
    .open     = dac_tc1321_open,
    .release  = dac_tc1321_release,
    .unlocked_ioctl   = dac_tc1321_ioctl,
    .owner    = THIS_MODULE,
};


static int tc1321_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    int err=0;
    struct tc1321_data *data = NULL;
        int ret;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    data = devm_kzalloc(&client->dev, sizeof(struct tc1321_data),
                GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    i2c_set_clientdata(client, data);
/*
    data->sar_shift = MCP3021_SAR_SHIFT;
    data->sar_mask = MCP3021_SAR_MASK;
    data->output_res = MCP3021_OUTPUT_RES;

    if (dev_get_platdata(&client->dev))
    {
        data->vdd = *(u32 *)dev_get_platdata(&client->dev);
        if (data->vdd > MCP3021_VDD_MAX || data->vdd < MCP3021_VDD_MIN)
            return -EINVAL;
    }
    else
    {
        data->vdd = MCP3021_VDD_REF;
    }
*/
//create file entry for the device attribute under sysfs directory of device kobject.
    err = sysfs_create_file(&client->dev.kobj, &dev_attr_dac_input.attr);
    if (err)
        return err;

    err = sysfs_create_file(&client->dev.kobj, &dev_attr_dac_data_reg.attr);
    if (err)
        return err;

    data->hwmon_dev = hwmon_device_register(&client->dev);
    if (IS_ERR(data->hwmon_dev)) {
        err = PTR_ERR(data->hwmon_dev);
        goto exit_remove;
    }

    data->devicenum= dac_first;
    data->minor= MINOR(dac_first);
    data->major= MAJOR(dac_first);
    dac_first++;

   cdev_init(&(data->c_dev),&dac_tc1321_fops);
    //printk("  cdev Initialised with dac_tc1321_fops\n");
    if( (ret=cdev_add(&(data->c_dev),data->devicenum,1)) < 0 )
    {
        //printk(KERN_ALERT "(cdev)tc1321_dac  not added to the system\n");
        goto err1;
    }
    sprintf(data->device_name, "dac%d",data->minor);
    //printk(" (cdev)tc1321_dac added to system\n");
    //sprintf(devname, "dac%d",data->minor);
    //if (device_create(dac_class, NULL,data->devicenum, NULL, devname) == NULL)
    if (device_create(dac_class, NULL,data->devicenum, NULL,data->device_name) == NULL)
    {
        //printk(" device create: failed \n");
        goto err2;
    }
    data->clientp= client;
    return 0;
err2: 
    cdev_del(&(data->c_dev));
    //printk(" dac%d removed from system\n",MINOR(data->devicenum));
err1:
    hwmon_device_unregister(data->hwmon_dev);
exit_remove:
    sysfs_remove_file(&client->dev.kobj, &dev_attr_dac_input.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_dac_data_reg.attr);
    return err;
}

static int tc1321_remove(struct i2c_client *client)
{
    struct tc1321_data *data = i2c_get_clientdata(client);

    hwmon_device_unregister(data->hwmon_dev);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_dac_input.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_dac_data_reg.attr);
    cdev_del(&(data->c_dev));
    device_destroy(dac_class,data->devicenum);

    return 0;
}

static const struct i2c_device_id tc1321_id[] = {
    { "tc1321", tc1321 },
    { }
};
MODULE_DEVICE_TABLE(i2c, tc1321_id);

static struct i2c_driver tc1321_driver = {
    .driver = {
        .name = "tc1321-drv",
    },
    .probe = tc1321_probe,
    .remove = tc1321_remove,
    .id_table = tc1321_id,
};


static int __init tc1321_dac_drv_init(void)
{
    int retval;
    if((retval= alloc_chrdev_region(&dac_first, 0, N_devices, "tc1321")))
    {
    //printk(" unable to Allocate device Numbers\n");
    return -EBUSY;
    }
    //printk(" Allocated device Numbers:MajorNum=%d Minor=%d\n",MAJOR(dac_first),MINOR(dac_first) );
    dac_class = class_create(THIS_MODULE, "dac_devices");
    if (IS_ERR(dac_class)) {
    retval = PTR_ERR(dac_class);
    //printk(" class_create Failed\n");
    goto err1;
    }
    //printk(" class created for the dac\n");
    //printk(" Registering platform_driver.....\n");
    //printk(" i2c subsystem Allocates i2c_client structure\n");
    //printk(" and calls probe for  all driver compatible nodes in DT\n");
    i2c_add_driver(&tc1321_driver);
    //printk(" tc1321_driver registered\n");
printk("ioctl-cmd = %x\n",TC1321_IOC_MODE);
printk("ioctl-cmd = %x\n",TC1321_IOC_WRDATA);
printk("ioctl-cmd = %x\n",TC1321_IOC_RDDATA);

//dac_tc1321_ioctl(TC1321_IOC_RDDATA,digital_data);

    return 0;
err1:
    unregister_chrdev_region(dac_first,N_devices);
    //printk(" free the Allocated device Numbers\n");
    //printk(KERN_ERR "tc1321_driver: cannot register device. error=%d\n",retval);
    //printk(" free the Allocated device Numbers\n");
    //printk(KERN_ERR "tc1321_driver: cannot register device. error=%d\n",retval);
    return retval;
}

static void __exit  tc1321_dac_drv_exit(void)
{
    //printk("\n\n");
    //printk(KERN_ALERT " tc1321_dac_drv_exit\n");
    i2c_del_driver(&tc1321_driver);
    //printk(" unregister platform_driver\n");
    class_destroy(dac_class);
    //printk(" class for the uart_obj_device_device : destroyed\n");
    unregister_chrdev_region(dac_first,N_devices);
    //printk(" free the Allocated device Numbers\n");   
    //printk(KERN_INFO " tc1321_dac_drv: unloading.\n");
}
module_init(tc1321_dac_drv_init);
module_exit(tc1321_dac_drv_exit);

MODULE_DESCRIPTION("Microchip TC1321 driver");
MODULE_LICENSE("GPL");
