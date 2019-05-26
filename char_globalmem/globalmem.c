//a simple char device driver:globalmem
//copyroght (c) 2019 Edwards
//License under GPLv3 or later

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>


#define GLOBALMEM_SIZE 	0x1000 	//4k
#define MEM_CLEAR		0x1		//1
#define GLOBALMEM_MAJOR	230

static int globalmem_major = GLOBALMEM_MAJOR;
module_param(globalmem_major, int, S_IRUGO);//被加载时可以传递给他的值，
											//并成为模块内的全局变量。

struct globalmem_dev{
	struct cdev cdev;
	unsigned char mem[GLOBALMEM_SIZE];
};										

struct globalmem_dev* globalmem_devp;

static int globalmem_open(struct inode* inode, struct file* filp)
{
	filp->private_data = globalmem_devp;
	return 0;	
}

static int globalmem_release(struct inode* inode, struct file* filp)
{
	return 0;
}
	
static long globalmem_ioctl(struct file* filp, unsigned int cmd, 
							unsigned long arg)
{
	struct globalmem_dev* dev = filp->private_data;//指向私有数据指针
	switch(arg)
	{
		case MEM_CLEAR:
			memset(dev->mem, 0, GLOBALMEM_SIZE);
			printk(KERN_INFO "globalmem is set to zero. \n");
			break;
		default:
			return -EINVAL;
				
	}

	return 0;
}

static ssize_t globalmem_read(struct file* filp,
							  char __user* buf,
							  size_t size,
							  loff_t* ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct globalmem_dev* dev = filp->private_data;//指向私有数据指针
	
	if(p >= GLOBALMEM_SIZE)
		return 0;
	if(count > GLOBALMEM_SIZE -p)
		count = GLOBALMEM_SIZE -p;
	
	if(copy_to_user(buf, dev->mem +p, count)){
		
		return -EFAULT;
	}else{
		*ppos += count;
		ret = count;
		
		printk(KERN_INFO "read %u byte(s) from %lu \n", count, p);
	}
								  
	return ret;								  				  
}

static ssize_t globalmem_write(struct file* filp,
								const char __user* buf,
								size_t size,
								loff_t* ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct globalmem_dev* dev = filp->private_data;
	
	if(p >= GLOBALMEM_SIZE)
		return 0;
	if(count > GLOBALMEM_SIZE - p)
		count = GLOBALMEM_SIZE - p;
	
	if(copy_from_user(dev->mem + p, buf, count))
		ret = -EFAULT;
	else{
		*ppos += count;
		ret = count;
		
		printk(KERN_INFO "written %u byte(s) from %lu  \n", count, p);
	}
	
	return ret;
}

static loff_t globalmem_llseek(struct file* filp, loff_t offset, int orig)
{
	loff_t ret =0;
	switch(orig){
		
		case 0://SEEK_SET
			if(offset < 0){
				ret = -EINVAL;
				break;
			}
			if((unsigned int)offset > GLOBALMEM_SIZE){
				ret = -EINVAL;
				break;
			}
			filp->f_pos = (unsigned int)offset;
			ret = filp->f_pos;
			break;
		case 1://SEEK_CUR
			if((filp->f_pos+offset) > GLOBALMEM_SIZE){
				ret = -EINVAL;
				break;
			}
			if((filp->f_pos+offset) < 0){
				ret = -EINVAL;
				break;
			}
			filp->f_pos += offset;
			ret = filp->f_pos;
			break;
		case 2://SEEK_END
			if(offset > 0){
				ret = -EINVAL;
				break;
			}
			if((filp->f_pos+offset) < 0){
				ret = -EINVAL;
				break;
			}
			filp->f_pos = (GLOBALMEM_SIZE+offset);
			ret = filp->f_pos;
			break;
		
		default:
			ret = -EINVAL;
		break;
		
		
	}

	return ret;
	
}


static const struct file_operations globalmem_fops = {
	
	.owner 	= THIS_MODULE,
	.llseek = globalmem_llseek,
	.read 	= globalmem_read,
	.write	= globalmem_write,
	.unlocked_ioctl = globalmem_ioctl,
	.open	= globalmem_open,
	.release = globalmem_release,
};

static void globalmem_setup_cdev(struct globalmem_dev* dev, int index)
{
	int err, devno = MKDEV(globalmem_major, index);
	
	cdev_init(&dev->cdev, &globalmem_fops);//初始化字符设备操作接口
	
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);//注册设备
	if(err)
		printk(KERN_INFO "Error %d adding globalmem%d \n", err, index);
}						

static int __init globalmem_init(void)
{
	int ret;
	//MKDEV： 是用来将主设备号和次设备号，转换成一个主次设备号的。(设备号)
	dev_t devno = MKDEV(globalmem_major, 0);
	
	//First :要分配的设备编号范围的初始值, 这组连续设备号的起始设备号, 相当于register_chrdev（）中主设备号
	//Count:连续编号范围.   是这组设备号的大小（也是次设备号的个数）
	//Name:编号相关联的设备名称. (/proc/devices); 本组设备的驱动名称
	if(globalmem_major)
		ret = register_chrdev_region(devno, 1, "globalmem");//静态注册
	else{
		ret = alloc_chrdev_region(&devno, 0, 1, "globalmem");//动态注册
		globalmem_major = MAJOR(devno);//从设备号中提取主设备号
	}
	
	if(ret < 0 )
		return ret;
	//kmalloc() 申请的内存位于物理内存映射区域，而且在物理上也是连续的，它们与真实的物理地址只有一个固定的偏移，
	//因为存在较简单的转换关系，所以对申请的内存大小有限制，不能超过128KB
	//kzalloc() 函数与 kmalloc() 非常相似，参数及返回值是一样的，可以说是前者是后者的一个变种，
	//因为 kzalloc() 实际上只是额外附加了 __GFP_ZERO 标志。所以它除了申请内核内存外，还会对申请到的内存内容清零。
	globalmem_devp = kzalloc(sizeof(struct globalmem_dev), GFP_KERNEL);
	if(!globalmem_devp){
		ret = -ENOMEM;
		goto fail_malloc;
	}
	globalmem_setup_cdev(globalmem_devp, 0);
	
	printk(KERN_INFO "init char globalmem okay.\n");
	return 0;
	
	fail_malloc:
	unregister_chrdev_region(devno, 1);
	return ret;
}
module_init(globalmem_init);

static void __exit globalmem_exit(void)
{
	cdev_del(&globalmem_devp->cdev);
	kfree(globalmem_devp);
	unregister_chrdev_region(MKDEV(globalmem_major, 0), 1);
	printk(KERN_INFO "exit char globalmem okay.\n");
}
module_exit(globalmem_exit);

MODULE_AUTHOR("Edwards");
MODULE_LICENSE("GPL v3");