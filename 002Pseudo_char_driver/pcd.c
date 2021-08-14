#include<linux/module.h>
#include<linux/fs.h>
#include<linux/cdev.h>
#include<linux/device.h>
#include<linux/kdev_t.h>
#include<linux/uaccess.h>
#include<linux/err.h>

#undef pr_fmt
#define pr_fmt(fmt) "%s: " fmt,__func__

#define DEV_MEM_SIZE 512
/*Pesudo device number */
char deviceBuffer[DEV_MEM_SIZE]; // This is our device, a memory segment

loff_t pcd_lseek(struct file *filp , loff_t offset , int whence)
{
    loff_t temp;
    pr_info("lseek requested \n");
    pr_info("current value of file position %lld ", filp->f_pos);
    switch(whence)
    {
        case SEEK_SET:
            if((offset >DEV_MEM_SIZE) || (offset<0) )
                return -EINVAL;
         filp->f_pos=offset;
         break;
        case SEEK_CUR:
        temp = filp->f_pos + offset;
        if((temp > DEV_MEM_SIZE ) || (temp < 0 ) ) 
            return -EINVAL;
         filp->f_pos =temp;
         break;
        case SEEK_END:
        temp = DEV_MEM_SIZE + offset;
        if((temp > DEV_MEM_SIZE ) || (temp < 0 ) ) 
            return -EINVAL;
         filp->f_pos = temp;
         break;
        default:
            return -EINVAL;
    }
    
    pr_info("New value of file position %lld ", filp->f_pos);
    return filp->f_pos;

}
ssize_t pcd_read (struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{
    pr_info("read requested for %zu byte \n",count);
    pr_info("Current file position %lld", *f_pos);
    /*Adjust the count  */
    if ((*f_pos + count) > DEV_MEM_SIZE)
        count = DEV_MEM_SIZE - *f_pos;

    /*copy to user */
    if( copy_to_user(buff,&deviceBuffer[*f_pos],count))
    {
        return -EFAULT;
    }
    
    /* Update current file position */
    *f_pos += count;

    pr_info("Number of bytes sucessfully read %zu", count);
    pr_info("Updated file position %lld", *f_pos);
    
    /*Return number of byte read successfully */
    return count;
}

ssize_t pcd_write(struct file *filp, const char __user *buff, size_t count , loff_t *f_pos)
{
    pr_info("write requested for %zu byte \n",count);
    pr_info("Current file position %lld", *f_pos);
    /*Adjust the count  */
    if ((*f_pos + count) > DEV_MEM_SIZE)
        count = DEV_MEM_SIZE - *f_pos;

    if(!count)
    {
        pr_err("No space left on the device");
        return -ENOMEM;
    }
    /*copy from  user */
    if( copy_from_user(&deviceBuffer[*f_pos],buff,count))
        return -EFAULT;
    
    /* Update current file position */
    *f_pos += count;

    pr_info("Number of bytes sucessfully written %zu", count);
    pr_info("Updated file position %lld", *f_pos);
    
    /*Return number of byte written successfully */
    return count;
}

int pcd_open(struct inode *inode , struct file *filp)
{
    pr_info("open was successfull \n");
    /*put the initilization of the device*/
    return 0;
}

int pcd_release(struct inode *inode, struct file *filp )
{
    pr_info("close was successfull \n");
    /*put the destruction of the device*/
    return 0;
}


/* Hold device number*/
dev_t deviceNumber;

struct cdev pcd_cdev;
struct file_operations pcd_fops = {
    .open = pcd_open,
    .write = pcd_write,
    .read = pcd_read,
    .llseek= pcd_lseek,
    .release = pcd_release,
    .owner = THIS_MODULE
};

struct class *class_pcd;
struct device *device_pcd; 

static int __init pcd_driver_init(void)
{
    int ret ;
    /*1. Allocating device number */
    ret = alloc_chrdev_region(&deviceNumber,0,1,"pcd_devices");
    if (ret < 0 )
    {
        pr_err("Chardev allocation failed\n");
        goto out;
    }
    pr_info("Device number <major>:<minor>= %d:%d\n",MAJOR(deviceNumber),MINOR(deviceNumber));
    
    /*2. Intializationof the device*/
    cdev_init(&pcd_cdev,&pcd_fops); //Initializtion
    pr_info("Called cdev_init");

    /*3. Registering the device with VFS*/
    ret = cdev_add(&pcd_cdev,deviceNumber,1); //putting in VFS system
    if (ret  < 0 )
    {
        pr_err("Chardev failed\n");
        goto unreg_chrdev;
    }
    pcd_cdev.owner = THIS_MODULE;
    pr_info("Callged pcd_dev ");
    
    /*4. create device class under /sys/class/ */
    class_pcd = class_create(THIS_MODULE,"pcd_class");
    if(IS_ERR(class_pcd))
    {
        pr_err("Class creation failed\n");
        ret = PTR_ERR(class_pcd);
        goto cdev_del;
    }
    pr_info("Called class_create");
       
    /*5. populate the sysfs with device information */
    device_pcd = device_create(class_pcd,NULL,deviceNumber,NULL,"pcd");
    if(IS_ERR(device_pcd))
    {
        pr_err("Device creation failed\n");
        ret = PTR_ERR(device_pcd);
        goto class_del;
    }
    pr_info("Called device_create");
    pr_info("Module init was successfull");



    return 0;

class_del:
    class_destroy(class_pcd);
cdev_del:
    cdev_del(&pcd_cdev);
unreg_chrdev:
    unregister_chrdev_region(deviceNumber,1);
out: 
    return ret;
}

//should execute the statment opposite of init chronologically
static void __exit pcd_driver_cleanup(void)
{
    device_destroy(class_pcd,deviceNumber);
    class_destroy(class_pcd);
    cdev_del(&pcd_cdev);
    unregister_chrdev_region(deviceNumber,1);
    pr_info("Module unloaded\n");
}

MODULE_LICENSE("GPL");
module_init(pcd_driver_init);
module_exit(pcd_driver_cleanup);
