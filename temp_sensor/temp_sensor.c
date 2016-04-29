/*
 * mcp9800.c - driver for Microchip mcp9800 .
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
/*
#define mcp9800_VDD_MAX        5500
#define mcp9800_VDD_MIN        2700
#define mcp9800_VDD_REF        3300
*/

#define TA_REG_SELECT  0X00
#define CONFIG_REG_SELECT  0X01
#define THYST_REG_SELECT  0X02
#define TSET_REG_SELECT  0X03
/*
#define STDBY 0x01
#define NORMAL 0x00
*/

/*
#define mcp9800_DRIVER_TYPE               'M'
#define mcp9800_IOC_MODE            _IOW(mcp9800_DRIVER_TYPE, 1,unsigned char )
#define mcp9800_IOC_WRDATA          _IOW(mcp9800_DRIVER_TYPE, 2, unsigned short )
#define mcp9800_IOC_RDDATA          _IOR(mcp9800_DRIVER_TYPE, 3, unsigned short )
#define mcp9800_IOC_MAX 3 
*/
static int N_devices = 1;
module_param(N_devices, int, S_IRUGO);

static dev_t temp_first;

static struct class *temp_class;

enum chips {
    mcp9800,
};

/*
 * Client data (each client gets its own)
 */
struct mcp9800_data {
    struct device *hwmon_dev;
    u32 vdd;    /* device power supply */
    u16 sar_shift;
    u16 sar_mask;
    u8 output_res;
    struct cdev c_dev;
    dev_t devicenum;
    unsigned int major;
    unsigned int minor;
    char device_name[30];
struct mutex temp_lock;
struct i2c_client *clientp;
};


/*
static int mcp9800_ConfigWrite(struct i2c_client *client,char mode)
{

// struct mcp9800_data *data = i2c_get_clientdata(client);
unsigned char i2c_data[2];
printk(" mcp9800_ConfigWrite:\n");
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
*/



static int mcp9800_DataWrite(struct i2c_client *client,void __user *argp)
{

char *arg1p = (char *)argp;
  //struct mcp9800_data *data = i2c_get_clientdata(client);
char *i2c_data = (char *)argp;
//unsigned char i2c_data[3];
int ret;

printk(" mcp9800_DataWrite:\n");
printk("argp=%p\n",argp);
printk("*arg1p=%x *(arg1p+1)=%x  *(arg1p+2)=%x \n", *arg1p,*(arg1p+1),*(arg1p+2) );
memcpy(i2c_data,argp,3);
printk("i2c_data -before send = %x %x %x\n",i2c_data[0],i2c_data[1],i2c_data[2]);
ret = i2c_master_send(client, i2c_data,3 );
        if (ret != 3) {
                dev_err(&client->dev, "i2c write data cmd failed\n");
                return ret < 0 ? ret : -EIO;
        }

return 0;
}


static int mcp9800_DataRead(struct i2c_client *client,void __user *argp)
{
  // struct mcp9800_data *data = i2c_get_clientdata(client);
char *arg1p = (char *)argp;
char *i2c_data = (char *)argp;
//unsigned char i2c_data[3];
int ret;
printk(" mcp9800_DataRead:\n");
printk("argp=%p\n",argp);
printk("*arg1p=%x *(arg1p+1)=%x  *(arg1p+2)=%x \n", *arg1p,*(arg1p+1),*(arg1p+2) );
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
//memcpy(argp,i2c_data+1,2);
//printk("*arg1p=%d *(arg1p+1)=%d  *(arg1p+2)=%d \n", *arg1p,*(arg1p+1),*(arg1p+2) );
return 0;
}

/*
static ssize_t send_input_to_temp(struct device *dev, struct device_attribute *attr,
        char *buf)
{
int ret=0;
    struct i2c_client *client = to_i2c_client(dev);
//    struct mcp9800_data *data = i2c_get_clientdata(client);

printk(" send_input_to_temp:\n");
ret=mcp9800_DataWrite(client,buf);
    if (ret < 0)
        return ret;
return 0;

}
*/

static ssize_t write_config_reg(struct device *dev, struct device_attribute *attr,
        char *buf,size_t count)
{
int ret=0;
char  val;
//int val=0;
struct i2c_client *client = to_i2c_client(dev);
//    struct mcp9800_data *data = i2c_get_clientdata(client);
dump_stack();
printk(" write_config_data_reg:\n");
printk("buf=%p count=%d \n", buf,count );
printk("*buf=%x,*(buf+1)=%x,*(buf+2)=%x %x %x %x \n",*buf,*(buf+1), *(buf+2),*(buf+3),*(buf+4),*(buf+5) );
//val= atoi(buf);
  ret = kstrtos8(buf,0, &val);
  if (ret)
   return ret;
printk("val=%d %x\n",val,val);
buf[0]= CONFIG_REG_SELECT;
buf[1]= val;
buf[2]= 0;
printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
ret= mcp9800_DataWrite(client,buf);
    if (ret < 0)
        return ret;
//printk("\nbuf=%s\n",buf);  
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
return count; 
}


static ssize_t write_tset_reg(struct device *dev, struct device_attribute *attr,
        char *buf,size_t count)
{
int ret=0;
char  val;
//int val=0;
    struct i2c_client *client = to_i2c_client(dev);
//    struct mcp9800_data *data = i2c_get_clientdata(client);
dump_stack();
printk(" write_temp_data_reg:\n");
printk("buf=%p count=%d \n", buf,count );
printk("*buf=%x,*(buf+1)=%x,*(buf+2)=%x %x %x %x \n",*buf,*(buf+1), *(buf+2),*(buf+3),*(buf+4),*(buf+5) );
//val= atoi(buf);
  ret = kstrtos8(buf,0, &val);
  if (ret)
   return ret;
printk("val=%d %x\n",val,val);
buf[0]= TSET_REG_SELECT;
buf[1]= val;
buf[2]= 0;
printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
ret= mcp9800_DataWrite(client,buf);
    if (ret < 0)
        return ret;
//printk("\nbuf=%s\n",buf);  
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
return count; 
}


static ssize_t write_hyst_reg(struct device *dev, struct device_attribute *attr,
        char *buf,size_t count)
{
int ret=0;
char  val;
//int val=0;
    struct i2c_client *client = to_i2c_client(dev);
//    struct mcp9800_data *data = i2c_get_clientdata(client);
dump_stack();
printk(" write_hyst_data_reg:\n");
printk("buf=%p count=%d \n", buf,count );
printk("*buf=%x,*(buf+1)=%x,*(buf+2)=%x %x %x %x \n",*buf,*(buf+1), *(buf+2),*(buf+3),*(buf+4),*(buf+5) );
//val= atoi(buf);
  ret = kstrtos8(buf,0, &val);
  if (ret)
   return ret;
printk("val=%d %x\n",val,val);
buf[0]= THYST_REG_SELECT;
buf[1]= val;
buf[2]= 0;
printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
ret= mcp9800_DataWrite(client,buf);
    if (ret < 0)
        return ret;
//printk("\nbuf=%s\n",buf);  
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
return count; 
}









static ssize_t read_tset_reg(struct device *dev, struct device_attribute *attr,
        char *buf)
{
int ret=0;
char i2c_data[3];
struct i2c_client *client = to_i2c_client(dev);
//    struct mcp9800_data *data = i2c_get_clientdata(client);
dump_stack();
printk(" read_temp_data_reg:\n");
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
i2c_data[0]= TSET_REG_SELECT;
ret= mcp9800_DataRead(client,i2c_data);
    if (ret < 0)
        return ret;
//printk("\nbuf=%s\n",buf);  
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
printk("val=%d %x \n",i2c_data[1],i2c_data[1]);

if(i2c_data[1]&0x80)
{
i2c_data[1]=(~i2c_data[1])+1;
sprintf(buf,"Tset = -%d\n",i2c_data[1]);
}
else
{
sprintf(buf,"Tset = %d\n",i2c_data[1]);
}
printk("\n%s\n",buf);  
{
//char val = -0;
char val = 0x80;
char sal ;
printk("val=%d\n",val);
val = 0x50;
printk("val=%d\n",val);
val = -25;
printk("val=%x\n",val);
sal=~val;
printk("~val=%x\n",sal);
printk("~val+1=%x\n",sal+1);
//printk("~val=%d",val+1);


}

return ret;
}

static ssize_t read_temp_reg(struct device *dev, struct device_attribute *attr,
        char *buf)
{
int ret=0;
    struct i2c_client *client = to_i2c_client(dev);
char i2c_data[3];
//    struct mcp9800_data *data = i2c_get_clientdata(client);
dump_stack();
printk(" read_temp_data_reg:\n");
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
i2c_data[0]= TA_REG_SELECT;
ret= mcp9800_DataRead(client,i2c_data);
    if (ret < 0)
        return ret;
ret=sprintf(buf,"Ta = %d\n",i2c_data[1]);
printk("\n%s\n",buf);  
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
return ret;
}

static ssize_t read_config_reg(struct device *dev, struct device_attribute *attr,
        char *buf)
{
int ret=0;
    struct i2c_client *client = to_i2c_client(dev);
char i2c_data[3];
//    struct mcp9800_data *data = i2c_get_clientdata(client);
dump_stack();
printk(" read_temp_data_reg:\n");
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
i2c_data[0]= CONFIG_REG_SELECT;
ret= mcp9800_DataRead(client,i2c_data);
    if (ret < 0)
        return ret;
ret=sprintf(buf,"CONFIG = %x\n",i2c_data[1]);
printk("\n%s\n",buf);  
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
return ret;
}

static ssize_t read_hyst_reg(struct device *dev, struct device_attribute *attr,
        char *buf)
{
int ret=0;
    struct i2c_client *client = to_i2c_client(dev);
char i2c_data[3];
//    struct mcp9800_data *data = i2c_get_clientdata(client);
dump_stack();
printk(" read_temp_data_reg:\n");
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
i2c_data[0]= THYST_REG_SELECT;
ret= mcp9800_DataRead(client,i2c_data);
    if (ret < 0)
        return ret;
ret=sprintf(buf,"hyst = %d\n",i2c_data[1]);
printk("\n%s\n",buf);  
//printk("bufP=%p *buf=%x,*(buf+1)=%x,*(buf+2)=%x \n",buf, *buf,*(buf+1), *(buf+2) );
return ret;
}

//create attribute for device 

static DEVICE_ATTR(temp_reg, S_IRUGO,read_temp_reg,NULL);
static DEVICE_ATTR(config_reg, S_IRUGO|S_IWUGO,read_config_reg,write_config_reg);
static DEVICE_ATTR(hyst_reg, S_IRUGO|S_IWUGO,read_hyst_reg,write_hyst_reg);
static DEVICE_ATTR(tset_reg, S_IRUGO|S_IWUGO,read_tset_reg,write_tset_reg);

static int temp_mcp9800_open(struct inode *inode, struct file *file)
{

    struct mcp9800_data *data;
    int ret=0;
    //printk(" \n");
    printk(" temp_mcp9800_open: process %i (%s)\n",current->pid, current->comm);
    //printk(" struct inode *inode=%p  struct file *file=%p\n",inode,file);
    data = container_of(inode->i_cdev,struct mcp9800_data, c_dev);
    //printk(" priv temp object = %p  w.r.t cdev of opened device\n",data);
    file->private_data =(void *)data;
    //printk(" keep device object in devicefile's private_data\n");
return ret;
}

static int temp_mcp9800_release(struct inode *inode, struct file *file)
{
    struct mcp9800_data *data=NULL;
    //printk("\n\n");
    printk(" temp_mcp9800_release: process %i (%s)\n",current->pid, current->comm);
    data=(struct mcp9800_data *)file->private_data;
    printk("temp priv obj=%p\n",data);
    file->private_data=NULL;
    return 0;
}


//static long temp_mcp9800_ioctl(unsigned int mcp9800_cmd, unsigned short arg)
static long temp_mcp9800_ioctl(struct file *file,unsigned int mcp9800_cmd, unsigned long arg)
{

//        char  __user *argp = (void __user *)arg;

//        struct mcp9800_data *data =     file->private_data;
        int ret=0;
/*
        printk(" mcp9800_cmd=%x\n",mcp9800_cmd);
//        printk("*argp *argp+1 *argp+2 %x %x %x\n",*argp,*(argp+1),*(argp+2));
        if (_IOC_TYPE(mcp9800_cmd) != mcp9800_DRIVER_TYPE)
        return -ENOTTY;

        if (_IOC_NR(mcp9800_cmd) > mcp9800_IOC_MAX)
        return -ENOTTY;

        printk("temp_mcp9800_ioctl: process %i (%s)\n",current->pid, current->comm);
//      printk("struct file *file=%p\n",file);
//      printk("device object ptr =%p available in private_data of opened device file\n",data);
        //Note -Locking taken care by -i2c Framework

        switch(mcp9800_cmd)
        {
        case mcp9800_IOC_MODE:
                ret=mcp9800_ConfigWrite(data->clientp,*argp );
                printk("mcp9800_IOC_STDBY");break; 
        case mcp9800_IOC_WRDATA:
                ret= mcp9800_DataWrite(data->clientp , argp);                                
                printk("mcp9800_IOC_WRDATA");
                printk(": argp argp+1 => %d %d\n",*argp,*(argp+1));
                break; 
        case mcp9800_IOC_RDDATA:
                ret= mcp9800_DataRead(data->clientp , argp);
                printk("mcp9800_IOC_RDDATA");
                printk(": argp argp+1 => %d %d\n",*argp,*(argp+1));
                break; 
        default :
                printk("Unknown ioctl-cmd\n");
                ret=-ENOTTY;
        break;
        }
*/        return ret;
}


static struct file_operations temp_mcp9800_fops = {
    .open     = temp_mcp9800_open,
    .release  = temp_mcp9800_release,
    .unlocked_ioctl   = temp_mcp9800_ioctl,
    .owner    = THIS_MODULE,
};


static int mcp9800_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    int err=0;
    struct mcp9800_data *data = NULL;
        int ret;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENODEV;

    data = devm_kzalloc(&client->dev, sizeof(struct mcp9800_data),
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


    err = sysfs_create_file(&client->dev.kobj, &dev_attr_tset_reg.attr);
    if (err)
        return err;

    err = sysfs_create_file(&client->dev.kobj, &dev_attr_temp_reg.attr);
    if (err)
        return err;
    err = sysfs_create_file(&client->dev.kobj, &dev_attr_config_reg.attr);
    if (err)
        return err;

    err = sysfs_create_file(&client->dev.kobj, &dev_attr_hyst_reg.attr);
    if (err)
        return err;



/*
    data->hwmon_dev = hwmon_device_register(&client->dev);
    if (IS_ERR(data->hwmon_dev)) {
        err = PTR_ERR(data->hwmon_dev);
        goto exit_remove;
    }
*/
    data->devicenum= temp_first;
    data->minor= MINOR(temp_first);
    data->major= MAJOR(temp_first);
    temp_first++;

   cdev_init(&(data->c_dev),&temp_mcp9800_fops);
    //printk("  cdev Initialised with temp_mcp9800_fops\n");
    if( (ret=cdev_add(&(data->c_dev),data->devicenum,1)) < 0 )
    {
        //printk(KERN_ALERT "(cdev)mcp9800  not added to the system\n");
        goto err1;
    }
    sprintf(data->device_name, "temp%d",data->minor);
    //printk(" (cdev)mcp9800 added to system\n");
    //sprintf(devname, "temp%d",data->minor);
    //if (device_create(temp_class, NULL,data->devicenum, NULL, devname) == NULL)
    if (device_create(temp_class, NULL,data->devicenum, NULL,data->device_name) == NULL)
    {
        //printk(" device create: failed \n");
        goto err2;
    }
    data->clientp= client;
    return 0;
err2: 
    cdev_del(&(data->c_dev));
    //printk(" temp%d removed from system\n",MINOR(data->devicenum));
err1:
//    hwmon_device_unregister(data->hwmon_dev);
//exit_remove:
//    sysfs_remove_file(&client->dev.kobj, &dev_attr_temp_input.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_tset_reg.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_temp_reg.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_config_reg.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_hyst_reg.attr);
    return err;
}

static int mcp9800_remove(struct i2c_client *client)
{
    struct mcp9800_data *data = i2c_get_clientdata(client);

//   hwmon_device_unregister(data->hwmon_dev);
//   sysfs_remove_file(&client->dev.kobj, &dev_attr_temp_input.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_temp_reg.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_tset_reg.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_config_reg.attr);
    sysfs_remove_file(&client->dev.kobj, &dev_attr_hyst_reg.attr);
    cdev_del(&(data->c_dev));
    device_destroy(temp_class,data->devicenum);

    return 0;
}

static const struct i2c_device_id mcp9800_id[] = {
    { "mcp980x", mcp9800 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mcp9800_id);

static struct i2c_driver mcp9800_driver = {
    .driver = {
        .name = "mcp9800-drv",
    },
    .probe = mcp9800_probe,
    .remove = mcp9800_remove,
    .id_table = mcp9800_id,
};


static int __init mcp9800_drv_init(void)
{
    int retval;
    if((retval= alloc_chrdev_region(&temp_first, 0, N_devices, "mcp9800")))
    {
    //printk(" unable to Allocate device Numbers\n");
    return -EBUSY;
    }
    //printk(" Allocated device Numbers:MajorNum=%d Minor=%d\n",MAJOR(temp_first),MINOR(temp_first) );
    temp_class = class_create(THIS_MODULE, "temp_devices");
    if (IS_ERR(temp_class)) {
    retval = PTR_ERR(temp_class);
    //printk(" class_create Failed\n");
    goto err1;
    }
    //printk(" class created for the temp\n");
    //printk(" Registering platform_driver.....\n");
    //printk(" i2c subsystem Allocates i2c_client structure\n");
    //printk(" and calls probe for  all driver compatible nodes in DT\n");
    i2c_add_driver(&mcp9800_driver);
    //printk(" mcp9800_driver registered\n");
/*
printk("ioctl-cmd = %x\n",mcp9800_IOC_MODE);
printk("ioctl-cmd = %x\n",mcp9800_IOC_WRDATA);
printk("ioctl-cmd = %x\n",mcp9800_IOC_RDDATA);
*/

//dac_mcp9800_ioctl(mcp9800_IOC_RDDATA,digital_data);

    return 0;
err1:
    unregister_chrdev_region(temp_first,N_devices);
    //printk(" free the Allocated device Numbers\n");
    //printk(KERN_ERR "mcp9800_driver: cannot register device. error=%d\n",retval);
    //printk(" free the Allocated device Numbers\n");
    //printk(KERN_ERR "mcp9800_driver: cannot register device. error=%d\n",retval);
    return retval;
}

static void __exit  mcp9800_drv_exit(void)
{
    //printk("\n\n");
    //printk(KERN_ALERT " mcp9800_drv_exit\n");
    i2c_del_driver(&mcp9800_driver);
    //printk(" unregister platform_driver\n");
    class_destroy(temp_class);
    //printk(" class for the uart_obj_device_device : destroyed\n");
    unregister_chrdev_region(temp_first,N_devices);
    //printk(" free the Allocated device Numbers\n");   
    //printk(KERN_INFO " mcp9800_drv: unloading.\n");
}
module_init(mcp9800_drv_init);
module_exit(mcp9800_drv_exit);

MODULE_DESCRIPTION("Microchip mcp9800 driver");
MODULE_LICENSE("GPL");
