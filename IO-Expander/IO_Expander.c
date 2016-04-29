/*
PATH=$PATH:/home/jaguar/Documents/Emb_linux/em_lin/practice/Linux_srcARM/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/

sudo make ARCH=arm  CROSS_COMPILE=/home/jaguar/Documents/Emb_linux/em_lin/practice/Linux_srcARM/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-   -C /root/beagleboneb_emb_linux_2015/emb_linux_kernel1/bb-kernel/KERNEL  M=`pwd` modules

cp IO_Expander.ko    /home/jaguar/Documents/Emb_linux/em_lin/practice/rootfs_3/rfs_bb_static_1.17.4/usr/lib/modules/
*/


/*
 * MCP23008.c - driver for Microchip MCP23008 .
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
#define MCP23008_VDD_MAX        5500
#define MCP23008_VDD_MIN        2700
#define MCP23008_VDD_REF        3300

#define IODIR     0x00
#define IPOL      0x01 
#define GPINTEN   0X02
#define DEFVAL    0x03 
#define INTCON    0x04 
#define IOCON     0x05
#define GPPU      0x06
#define INTF      0x07
#define INTCAP    0x08
#define GPIO      0x09
#define OLAT      0x0A


#define MCP23008_DRIVER_TYPE               'M'
#define MCP23008_IODIR_WRDATA          _IOW(MCP23008_DRIVER_TYPE, 1, unsigned short )
#define MCP23008_IODIR_RDDATA          _IOR(MCP23008_DRIVER_TYPE, 2, unsigned short )

#define MCP23008_IPOL_WRDATA          _IOW(MCP23008_DRIVER_TYPE, 3, unsigned short )
#define MCP23008_IPOL_RDDATA          _IOR(MCP23008_DRIVER_TYPE, 4, unsigned short )

#define MCP23008_GPINTEN_WRDATA          _IOW(MCP23008_DRIVER_TYPE,5, unsigned short )
#define MCP23008_GPINTEN_RDDATA          _IOR(MCP23008_DRIVER_TYPE,6, unsigned short )

#define MCP23008_DEFVAL_WRDATA          _IOW(MCP23008_DRIVER_TYPE,7, unsigned short )
#define MCP23008_DEFVAL_RDDATA          _IOR(MCP23008_DRIVER_TYPE,8, unsigned short )

#define MCP23008_IOC_MAX 8 

static int N_devices = 1;
module_param(N_devices, int, S_IRUGO);

static dev_t ioexpander_first;

static struct class *ioexpander_class;

enum chips {
    mcp23008,
};

/*
 * Client data (each client gets its own)
 */
struct mcp23008_data {
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




static int mcp23008_DataWrite(struct i2c_client *client,int reg_sel,void __user *argp)
{

  //struct mcp23008_data *data = i2c_get_clientdata(client);
  unsigned char i2c_data[2];
int ret;

printk(" mcp23008_DataWrite:\n");
// |S|IO-expAddr|W|Reg-offset|Data8-in|P|
i2c_data[0]= reg_sel;
memcpy(i2c_data+1,argp,1);
printk("i2c_data -before send = %x %x \n",i2c_data[0],i2c_data[1]);
ret = i2c_master_send(client, i2c_data,2 );
        if (ret != 3) {
                dev_err(&client->dev, "i2c write data cmd failed\n");
                return ret < 0 ? ret : -EIO;
        }
return 0;
}


static int mcp23008_DataRead(struct i2c_client *client,int reg_sel,void __user *argp)
{
  // struct mcp23008_data *data = i2c_get_clientdata(client);
    unsigned char i2c_data[2];
int ret;
printk(" mcp23008_DataRead:\n");
// |S|io-expAddr|W|SR|reg-offset|R|Data8-out|P|
i2c_data[0]= reg_sel;
printk("i2c_data -before send = %x %x \n",i2c_data[0],i2c_data[1]);
ret = i2c_master_send(client, i2c_data,1 );
        if (ret != 1) {
                dev_err(&client->dev, "i2c read data cmd failed\n");
                return ret < 0 ? ret : -EIO;
        }

ret = i2c_master_recv(client, i2c_data+1,1);
if (ret < 0)
   return ret;
if (ret != 2)
   return -EIO;

printk("i2c_data -After Rxve = %x %x \n",i2c_data[0],i2c_data[1]);
//argp= i2c_data+1;
memcpy(argp,i2c_data+1,1);
return 0;
}

static ssize_t i2cWrite_to_ioExp(struct device *dev, struct device_attribute *attr,
        char *buf)
{
int ret=0;
    struct i2c_client *client = to_i2c_client(dev);
//    struct mcp23008_data *data = i2c_get_clientdata(client);

int reg_sel = 0x00;
printk(" i2cWrite_to_ioExp:\n");
ret=mcp23008_DataWrite(client,reg_sel,buf);
    if (ret < 0)
        return ret;
return 0;

}

static ssize_t i2cRead_from_ioExp(struct device *dev, struct device_attribute *attr,
        char *buf)
{
int ret=0;
    struct i2c_client *client = to_i2c_client(dev);
//    struct mcp23008_data *data = i2c_get_clientdata(client);
int reg_sel = 0x00;
printk(" i2cRead_from_ioExp:\n");
ret= mcp23008_DataRead(client,reg_sel,buf);
    if (ret < 0)
        return ret;
return 0;

}


//create attribute for device 
static DEVICE_ATTR(reg_write, S_IWUGO,NULL, i2cWrite_to_ioExp );
static DEVICE_ATTR(reg_read,S_IRUGO, i2cRead_from_ioExp,NULL);

static struct attribute *mcp23008_attrlist[] = {
      &dev_attr_reg_write.attr,
      &dev_attr_reg_read.attr,
};


static struct attribute_group  mcp23008_attr_group = {
	.attrs = mcp23008_attrlist,
};



static int ioexp_mcp23008_open(struct inode *inode, struct file *file)
{

    struct mcp23008_data *data;
    int ret=0;
    //printk(" \n");
    printk(" ioexp_mcp23008_open: process %i (%s)\n",current->pid, current->comm);
    //printk(" struct inode *inode=%p  struct file *file=%p\n",inode,file);
    data = container_of(inode->i_cdev,struct mcp23008_data, c_dev);
    //printk(" priv dac object = %p  w.r.t cdev of opened device\n",data);
    file->private_data =(void *)data;
    //printk(" keep device object in devicefile's private_data\n");
return ret;

}
static int ioexp_mcp23008_release(struct inode *inode, struct file *file)
{
    struct mcp23008_data *data=NULL;
    //printk("\n\n");
    printk(" ioexp_mcp23008_release: process %i (%s)\n",current->pid, current->comm);
    data=(struct mcp23008_data *)file->private_data;
    printk("dac priv obj=%p\n",data);
    file->private_data=NULL;
    return 0;
}


//static long ioexp_mcp23008_ioctl(unsigned int mcp23008_cmd, unsigned short arg)
static long ioexp_mcp23008_ioctl(struct file *file,unsigned int mcp23008_cmd, unsigned long arg)
{

        char  __user *argp = (void __user *)arg;

        struct mcp23008_data *data =     file->private_data;
        int ret=0;

        printk(" mcp23008_cmd=%x\n",mcp23008_cmd);
//        printk("*argp *argp+1 *argp+2 %x %x %x\n",*argp,*(argp+1),*(argp+2));
        if (_IOC_TYPE(mcp23008_cmd) != MCP23008_DRIVER_TYPE)
        return -ENOTTY;

        if (_IOC_NR(mcp23008_cmd) > MCP23008_IOC_MAX)
        return -ENOTTY;

        printk("ioexp_mcp23008_ioctl: process %i (%s)\n",current->pid, current->comm);
//      printk("struct file *file=%p\n",file);
//      printk("device object ptr =%p available in private_data of opened device file\n",data);
        //Note -Locking taken care by -i2c Framework

        switch(mcp23008_cmd)
        {
        case MCP23008_IODIR_WRDATA:
                ret= mcp23008_DataWrite(data->clientp,IODIR,argp);                                
                printk("MCP23008_IOC_WRDATA");
                printk(": argp argp+1 => %d %d\n",*argp,*(argp+1));
                break; 
        case MCP23008_IODIR_RDDATA:
                ret= mcp23008_DataRead(data->clientp,IODIR,argp);
                printk("MCP23008_IOC_RDDATA");
                printk(": argp argp+1 => %d %d\n",*argp,*(argp+1));
                break; 
        case MCP23008_IPOL_WRDATA:
                ret= mcp23008_DataWrite(data->clientp,IPOL,argp);                  
                printk("MCP23008_IPOL_WRDATA");
                printk(": argp argp+1 => %d %d\n",*argp,*(argp+1));
                break; 
        case MCP23008_IPOL_RDDATA:
                ret= mcp23008_DataRead(data->clientp,IPOL,argp);
                printk("MCP23008_IPOL_RDDATA");
                printk(": argp argp+1 => %d %d\n",*argp,*(argp+1));
                break;

        default :
                printk("Unknown ioctl-cmd\n");
                ret=-ENOTTY;
        break;
        }
        return ret;
}

static struct file_operations ioexp_mcp23008_fops = {
    .open     = ioexp_mcp23008_open,
    .release  = ioexp_mcp23008_release,
    .unlocked_ioctl   = ioexp_mcp23008_ioctl,
    .owner    = THIS_MODULE,
};


static int mcp23008_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    int err=0;
    struct mcp23008_data *data = NULL;
        int ret;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    data = devm_kzalloc(&client->dev, sizeof(struct mcp23008_data),
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
    ret = sysfs_create_group(&client->dev.kobj,&mcp23008_attr_group);
    if (ret)
        return -1;


    data->hwmon_dev = hwmon_device_register(&client->dev);
    if (IS_ERR(data->hwmon_dev)) {
        err = PTR_ERR(data->hwmon_dev);
        goto exit_remove;
    }

    data->devicenum= ioexpander_first;
    data->minor= MINOR(ioexpander_first);
    data->major= MAJOR(ioexpander_first);
    ioexpander_first++;

   cdev_init(&(data->c_dev),&ioexp_mcp23008_fops);
    //printk("  cdev Initialised with ioexp_mcp23008_fops\n");
    if( (ret=cdev_add(&(data->c_dev),data->devicenum,1)) < 0 )
    {
        //printk(KERN_ALERT "(cdev)mcp23008_dac  not added to the system\n");
        goto err1;
    }
    sprintf(data->device_name,"IOexpander%d",data->minor);
    //printk(" (cdev)mcp23008_dac added to system\n");
    //sprintf(devname, "dac%d",data->minor);
    //if (device_create(ioexpander_class, NULL,data->devicenum, NULL, devname) == NULL)
    if (device_create(ioexpander_class, NULL,data->devicenum, NULL,data->device_name) == NULL)
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
      sysfs_remove_group(&client->dev.kobj,&mcp23008_attr_group);
//    sysfs_remove_file(&client->dev.kobj, &dev_attr_reg_write.attr);
 //   sysfs_remove_file(&client->dev.kobj, &dev_attr_reg_read.attr);
    return err;
}

static int mcp23008_remove(struct i2c_client *client)
{
    struct mcp23008_data *data = i2c_get_clientdata(client);

    hwmon_device_unregister(data->hwmon_dev);
    sysfs_remove_group(&client->dev.kobj,&mcp23008_attr_group);
//    sysfs_remove_file(&client->dev.kobj, &dev_attr_reg_write.attr);
//    sysfs_remove_file(&client->dev.kobj, &dev_attr_reg_read.attr);
    cdev_del(&(data->c_dev));
    device_destroy(ioexpander_class,data->devicenum);

    return 0;
}

static const struct i2c_device_id mcp23008_id[] = {
    { "mcp23008", mcp23008 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mcp23008_id);

static struct i2c_driver mcp23008_drv = {
    .driver = {
        .name = "mcp23008-drv",
    },
    .probe = mcp23008_probe,
    .remove = mcp23008_remove,
    .id_table = mcp23008_id,
};


static int __init mcp23008_IOExpander_drv_init(void)
{
    int retval;
    if((retval= alloc_chrdev_region(&ioexpander_first, 0, N_devices, "mcp23008")))
    {
    //printk(" unable to Allocate device Numbers\n");
    return -EBUSY;
    }
    //printk(" Allocated device Numbers:MajorNum=%d Minor=%d\n",MAJOR(ioexpander_first),MINOR(ioexpander_first) );
    ioexpander_class = class_create(THIS_MODULE, "IO-Expander");
    if (IS_ERR(ioexpander_class)) {
    retval = PTR_ERR(ioexpander_class);
    //printk(" class_create Failed\n");
    goto err1;
    }
    //printk(" class created for the dac\n");
    //printk(" Registering platform_driver.....\n");
    //printk(" i2c subsystem Allocates i2c_client structure\n");
    //printk(" and calls probe for  all driver compatible nodes in DT\n");
    i2c_add_driver(&mcp23008_drv);
    //printk(" mcp23008 registered\n");

//ioexp_mcp23008_ioctl(MCP23008_IOC_RDDATA,digital_data);

    return 0;
err1:
    unregister_chrdev_region(ioexpander_first,N_devices);
    //printk(" free the Allocated device Numbers\n");
    //printk(KERN_ERR "mcp23008: cannot register device. error=%d\n",retval);
    //printk(" free the Allocated device Numbers\n");
    //printk(KERN_ERR "mcp23008: cannot register device. error=%d\n",retval);
    return retval;
}

static void __exit  mcp23008_IOExpander_drv_exit(void)
{
    //printk("\n\n");
    //printk(KERN_ALERT " mcp23008_IOExpander_drv_exit\n");
    i2c_del_driver(&mcp23008_drv);
    //printk(" unregister platform_driver\n");
    class_destroy(ioexpander_class);
    //printk(" class for the uart_obj_device_device : destroyed\n");
    unregister_chrdev_region(ioexpander_first,N_devices);
    //printk(" free the Allocated device Numbers\n");   
    //printk(KERN_INFO " mcp23008_IOExpander_drv: unloading.\n");
}
module_init(mcp23008_IOExpander_drv_init);
module_exit(mcp23008_IOExpander_drv_exit);

MODULE_DESCRIPTION("Microchip MCP23008 driver");
MODULE_LICENSE("GPL");
