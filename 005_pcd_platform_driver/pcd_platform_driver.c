#include<linux/module.h>
#include<linux/fs.h>
#include<linux/cdev.h>
#include<linux/device.h>
#include<linux/kdev_t.h>
#include<linux/uaccess.h>
#include<linux/err.h>
#include<linux/platform_device.h>
#include<linux/mod_devicetable.h>
#include<linux/slab.h>
#include<linux/of.h>
#include "platform.h"


#undef pr_fmt
#define pr_fmt(fmt) "%s: " fmt,__func__

#define MAX_DEVICES 10

/* For extracing driver info */
struct device_config
{
    int config_item1 ; int config_item2;
};

enum pcdev_name
{
    PCDEVA1X,PCDEVB1X,PCDEVC1X,PCDEVD1X
};

struct device_config pcdev_config[] =
{
    [PCDEVA1X] = {.config_item1 = 60, .config_item2 = 30},
    [PCDEVB1X] = {.config_item1 = 50, .config_item2 = 25},
    [PCDEVC1X] = {.config_item1 = 30, .config_item2 = 15},
    [PCDEVD1X] = {.config_item1 = 10, .config_item2 = 5}

};

/*Device private data structure */
struct pcdev_private_data
{
    struct pcdev_platform_data pdata;
    char *buffer;
    dev_t dev_num;
    struct cdev cdev;
};

/*Driver private data structure */
struct pcdrv_private_data
{
    int total_devices;
    dev_t device_num_base;
    struct class *class_pcd;
    struct device *device_pcd;
};


struct pcdrv_private_data pcdrv_data;


loff_t pcd_lseek(struct file *filp , loff_t offset , int whence)
{
    return -EINVAL;
}

ssize_t pcd_read (struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{ return -ENOMEM; }

ssize_t pcd_write(struct file *filp, const char __user *buff, size_t count , loff_t *f_pos)
{ 
    return 0; 
}

int check_permission(int dev_perm,int acc_mode)
{
    if(dev_perm == RDWR) return 0;
    if((dev_perm ==RDONLY ) && (acc_mode & FMODE_READ) && !(acc_mode & FMODE_WRITE))
    return 0;
    if((dev_perm ==WRONLY ) && (acc_mode & FMODE_WRITE) && !(acc_mode & FMODE_READ))
        return 0;
    
    return -EPERM;
}

int pcd_open(struct inode *inode , struct file *filp)
{
    return 0;
}

int pcd_release(struct inode *inode, struct file *filp )
{
    pr_info("close was successfull \n");
    /*put the destruction of the device*/
    return 0;
}


struct file_operations pcd_fops = {
    .open = pcd_open,
    .write = pcd_write,
    .read = pcd_read,
    .llseek= pcd_lseek,
    .release = pcd_release,
    .owner = THIS_MODULE
};

int pcd_platform_driver_probe(struct platform_device *pdev)
{
    pr_info("device_detected\n");
#if 0
    int ret;
    struct pcdev_private_data *dev_data;
    struct pcdev_platform_data  *pdata;
    pr_info("A device is detected \n");
    /*1. Get the platform data */
    pdata = (struct pcdev_platform_data*)dev_get_platdata(&pdev->dev); // or pdev->dev.platform_data;  
    if(!pdata)
    {
        pr_info("No plataform data available");
        ret = -EINVAL;
        pr_info("Device probe failed \n");
        return ret;
    }
    
    /*2. Dynamically allocate the memory for the device private data */
    dev_data = devm_kzalloc(&pdev->dev,sizeof(*dev_data),GFP_KERNEL);
    if(!dev_data)
    {
        pr_err("Cannot allocate memory\n");
        ret = -ENOMEM;
        pr_info("Device probe failed \n");
        return ret;
    }

    /* Save the device  private data pointer in platform device strucutre for using in remove function*/
    dev_set_drvdata(&pdev->dev,dev_data); // ordev_data;

    dev_data->pdata.size = pdata->size;
    dev_data->pdata.perm = pdata->perm;
    dev_data->pdata.serial_number = pdata->serial_number;
        
    pr_info("Device serial number = %s\n", dev_data->pdata.serial_number);
    pr_info("Device size = %zu\n", dev_data->pdata.size);
    pr_info("Device permission = %zu\n", dev_data->pdata.perm);

    pr_info("Config_item 1 = %d\n",pcdev_config[pdev->id_entry->driver_data].config_item1 );
    pr_info("Config_item 2 = %d\n",pcdev_config[pdev->id_entry->driver_data].config_item2 );
    /*
    pr_info("Printing from source \n");
    pr_info("Device serial number extra = %s\n",pdev->dev->serial_number);
    pr_info("Device size extra = %s\n", pdev->dev->size);
    pr_info("Device permission extra = %s\n", pdev->dev->perm);
    */
    /*3. Dynamically allocate memory for the device buffer using size information from the platform data */
    dev_data->buffer = devm_kzalloc(&pdev->dev,dev_data->pdata.size,GFP_KERNEL);
    if(!dev_data->buffer)
    {
        pr_err("Cannot allocate memory\n");
        ret = -ENOMEM;
        return ret;
    }

    /*4 Get the device number*/
    dev_data->dev_num = pcdrv_data.device_num_base + pdev->id;

    /*5. Do cdev and cdev_init  */
    cdev_init(&dev_data->cdev,&pcd_fops);
    dev_data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&dev_data->cdev,dev_data->dev_num,1);
    if (ret  < 0 )
    {                                                                              
        pr_err("cdev add failed\n");                                                
        return ret;
    }                                                                              
    
    /*6. Create device file for the detected platform device for /dev/pdev-x*/
    pcdrv_data.device_pcd = device_create(pcdrv_data.class_pcd,NULL,dev_data->dev_num,NULL,"pcdev-%d",pdev->id);
    if(IS_ERR(pcdrv_data.device_pcd))
    {
        pr_err("Device create failed \n");
        ret = PTR_ERR(pcdrv_data.device_pcd);
        cdev_del(&dev_data->cdev);
        return ret;
    }
    
    pcdrv_data.total_devices++;

    pr_info("A device probe successfull \n");
#endif
    return 0;

/*7.Error handling */

}

int pcd_platform_driver_remove (struct platform_device *pdev)
{
#if 0
    struct pcdev_private_data *dev_data= dev_get_drvdata(&pdev->dev);
    /*1.Remove a device that was created with device_create()  */
    device_destroy(pcdrv_data.class_pcd,dev_data->dev_num );
    /*2.Remove a cdev entry from the system*/
    cdev_del(&dev_data->cdev);
    /*Not needed now when we used devm_kzalloc
    //3.Free the memory held by the device
    kfree(dev_data->buffer);
    kfree(dev_data);
    */

    pcdrv_data.total_devices--;

#endif
    pr_info("A device is removed \n");
    return 0;
}

/* for matching the version from platform device to load the driver */

struct platform_device_id  pcdev_ids[] =
{
     { .name = "pcdev-A1x", .driver_data = PCDEVA1X},
     { .name = "pcdev-B1x", .driver_data = PCDEVB1X},
     { .name = "pcdev-C1x", .driver_data = PCDEVC1X},
     { .name = "pcdev-D1x", .driver_data = PCDEVD1X},
     {}
};


struct of_device_id org_pcdev_dt_match[] ={
    {.compatible ="pcdev-A1x", .data = (void*)PCDEVA1X},
    {.compatible ="pcdev-B1x", .data = (void*)PCDEVB1X},
    {.compatible ="pcdev-B1x", .data = (void*)PCDEVC1X},
    {.compatible ="pcdev-D1x", .data = (void*)PCDEVD1X},
    {}
};
struct platform_driver pcd_platform_driver = {
    .probe = pcd_platform_driver_probe,
    .remove = pcd_platform_driver_remove,
    .id_table = pcdev_ids,
    .driver = {
        .name = "pseudo-char-device",
        .of_match_table = org_pcdev_dt_match

    }
};
static int __init pcd_platform_driver_init(void)
{
    int ret;
    /*1. Dynamically allocate a device for MAX_DEVICES */
    ret = alloc_chrdev_region(&pcdrv_data.device_num_base,0,MAX_DEVICES,"pcdevs");
    if (ret < 0 ) 
    {
        pr_err("Alloc chrdev failed \n");
        return ret;
    }
    /*2.Create device class under /sys/class */
    pcdrv_data.class_pcd = class_create(THIS_MODULE,"pcd_class");
    if ( IS_ERR(pcdrv_data.class_pcd) )
    {
        pr_err("Class createion failed \n");
        ret = PTR_ERR(pcdrv_data.class_pcd);
        unregister_chrdev_region(pcdrv_data.device_num_base,MAX_DEVICES);
        return ret;

    }
    /*3. Register platform driver */
    platform_driver_register(&pcd_platform_driver);

    pr_info("pcd platform driver loaded \n");
    return 0;

}

//should execute the statment opposite of init chronologically
static void __exit pcd_platform_driver_cleanup(void)
{
    /*1. Unregister platform driver */
    platform_driver_unregister(&pcd_platform_driver);
    /*2. destory class */
    class_destroy(pcdrv_data.class_pcd);
    /*3. Unregister the deivce number for MAX_DEVICES */
    unregister_chrdev_region(pcdrv_data.device_num_base,MAX_DEVICES);

    pr_info("pcd platform driver unloaded \n");
}

module_init(pcd_platform_driver_init);
module_exit(pcd_platform_driver_cleanup);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Omkar");
MODULE_DESCRIPTION("A pseudo character driver which handles N platform device");
