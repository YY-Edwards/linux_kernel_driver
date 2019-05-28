//a simple char device driver:globalfifo
//copyroght (c) 2019 Edwards
//License under GPLv3 or later

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>


#define GLOBALFIFO_SIZE 	0x1000 	//4k
#define MEM_CLEAR		0x1		//1
#define GLOBALFIFO_MAJOR	230
#define DEVICE_NUM		10

static int globalfifo_major = GLOBALFIFO_MAJOR;
module_param(globalfifo_major, int, S_IRUGO);//被加载时可以传递给他的值，
											//并成为模块内的全局变量。

struct globalfifo_dev{
	struct cdev cdev;
	unsigned int current_len;
	unsigned char mem[GLOBALFIFO_SIZE];
	struct mutex mutex;
	wait_queue_head_t	r_wait;
	wait_queue_head_t	w_wait;
	
};										

struct globalfifo_dev* globalfifo_devp;

static int globalfifo_open(struct inode* inode, struct file* filp)
{
	//filp->private_data = globalfifo_devp;
	//通过结构体成员的指针
	//找到对应结构体的指针。
	struct globalfifo_dev* devp = container_of(inode->i_cdev,
												struct globalfifo_dev, cdev);
	filp->private_data = devp;											
	return 0;	
}

static int globalfifo_release(struct inode* inode, struct file* filp)
{
	return 0;
}
	
static long globalfifo_ioctl(struct file* filp, unsigned int cmd, 
							unsigned long arg)
{
	struct globalfifo_dev* dev = filp->private_data;//指向私有数据指针
	switch(arg)
	{
		case MEM_CLEAR:
			mutex_lock(&dev->mutex);
			memset(dev->mem, 0, GLOBALFIFO_SIZE);
			mutex_unlock(&dev->mutex);
			printk(KERN_INFO "globalfifo is set to zero. \n");
			break;
		default:
			return -EINVAL;
				
	}

	return 0;
}

static ssize_t globalfifo_read(struct file* filp,
							  char __user* buf,
							  size_t size,
							  loff_t* ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct globalfifo_dev* dev = filp->private_data;//指向私有数据指针
	DECLARE_WAITQUEUE(wait, current);//初始化等待队列元素
	
	mutex_lock(&dev->mutex);
	add_wait_queue(&dev->r_wait, &wait);//添加wait到队列
	
	while(dev->current_len == 0){//有效长度为0
	
		if(filp->f_flags & O_NONBLOCK){//如果配置为非阻塞则立即返回结果
			ret = -EAGAIN;
			goto out;
		}
		__set_current_state(TASK_INTERUPTIBLE);//可被信号唤醒
		mutex_unlock(&dev->mutex);//睡眠前，先释放锁。
		
		schedule();//切换
		if(signal_pending(current){//醒来后第一时间判断是否是因为信号而被唤醒
			ret = -ERESTARTSYS;
			goto out2;
		
		}
		
		mutex_lock(&dev->mutex);//再次上锁	
	}
	
	
	// if(p >= GLOBALFIFO_SIZE)
		// return 0;
	// if(count > GLOBALFIFO_SIZE -p)
		// count = GLOBALFIFO_SIZE -p;
	
	// mutex_lock(&dev->mutex);
	
	if(count > dev->current_len)
		count = dev->current_len;
	
	if(copy_to_user(buf, dev->mem, count)){	
		ret =  -EFAULT;
		goto out;
	}else{
		//将已读数据移除，即自动指针偏移
		memcpy(dev->mem, dev->mem + count, dev->current_len - count);
		dev->current_len -= count;	
		printk(KERN_INFO "read %u byte(s),current_len:%d \n", count, 
				dev->current_len);
		
		wake_up_interruptible(&dev->w_wait);//允许写
		ret = count;
	}
	
	out:
		mutex_unlock(&dev->mutex);
	
	out2:
		remove_wait_queue(&dev->w_wait, &wait);//移除wait
		set_current_state(TASK_RUNNING);//设置状态
	
	return ret;								  				  
}

static ssize_t globalfifo_write(struct file* filp,
								const char __user* buf,
								size_t size,
								loff_t* ppos)
{
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct globalfifo_dev* dev = filp->private_data;
	DECLARE_WAITQUEUE(wait, current);//初始化等待队列元素
	
	mutex_lock(&dev->mutex);
	add_wait_queue(&dev->w_wait, &wait);//添加wait到写队列
	
	
	
	
	while(dev->current_len == GLOBALFIFO_SIZE){//如果已满
		if(filp->f_flags & O_NONBLOCK){//且非阻塞
			ret = -EAGAIN;
			goto out;
		}
		//设置状态
		__set_current_state(TASK_INTERUPTIBLE);
		mutex_unlock(&dev->mutex);
		schedule();//睡眠前，先释放锁。
		if(signal_pending(current)){
			ret = -ERESTARTSYS;
			goto out2;
		}
		
		mutex_lock(&dev->mutex);
	}
	
	if(count > GLOBALFIFO_SIZE - dev->current_len)
		count = GLOBALFIFO_SIZE -dev->current_len;
	
	
	if(copy_from_user(dev->mem + dev->current_len, buf, count)){	
		ret =  -EFAULT;
		goto out;
	}else{
		//将已读数据移除，即自动指针偏移
		dev->current_len += count;	
		printk(KERN_INFO "written %u byte(s),current_len:%d \n", count, 
				dev->current_len);
		
		wake_up_interruptible(&dev->r_wait);//允许读
		ret = count;
	}
	
	
	out:
		mutex_unlock(&dev->mutex);
	
	out2:
		remove_wait_queue(&dev->w_wait, &wait);//移除wait
		set_current_state(TASK_RUNNING);//设置状态
	
	return ret;	
	
	// if(p >= GLOBALFIFO_SIZE)
		// return 0;
	// if(count > GLOBALFIFO_SIZE - p)
		// count = GLOBALFIFO_SIZE - p;	
	// mutex_lock(&dev->mutex);
	// if(copy_from_user(dev->mem + p, buf, count))
		// ret = -EFAULT;
	// else{
		// *ppos += count;
		// ret = count;
		
		// printk(KERN_INFO "written %u byte(s) from %lu  \n", count, p);
	// }
	// mutex_unlock(&dev->mutex);
	// return ret;
}

static loff_t globalfifo_llseek(struct file* filp, loff_t offset, int orig)
{
	loff_t ret =0;
	switch(orig){
		
		case 0://SEEK_SET
			if(offset < 0){
				ret = -EINVAL;
				break;
			}
			if((unsigned int)offset > GLOBALFIFO_SIZE){
				ret = -EINVAL;
				break;
			}
			filp->f_pos = (unsigned int)offset;
			ret = filp->f_pos;
			break;
		case 1://SEEK_CUR
			if((filp->f_pos+offset) > GLOBALFIFO_SIZE){
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
			filp->f_pos = (GLOBALFIFO_SIZE+offset);
			ret = filp->f_pos;
			break;
		
		default:
			ret = -EINVAL;
		break;
		
		
	}

	return ret;
	
}


static const struct file_operations globalfifo_fops = {
	
	.owner 	= THIS_MODULE,
	.llseek = globalfifo_llseek,
	.read 	= globalfifo_read,
	.write	= globalfifo_write,
	.unlocked_ioctl = globalfifo_ioctl,
	.open	= globalfifo_open,
	.release = globalfifo_release,
};

static void globalfifo_setup_cdev(struct globalfifo_dev* dev, int index)
{
	int err, devno = MKDEV(globalfifo_major, index);
	
	cdev_init(&dev->cdev, &globalfifo_fops);//初始化字符设备操作接口
	
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);//注册设备
	if(err)
		printk(KERN_INFO "Error %d adding globalfifo%d \n", err, index);
}						

static int __init globalfifo_init(void)
{
	int ret;
	//int i;
	//MKDEV： 是用来将主设备号和次设备号，转换成一个主次设备号的。(设备号)
	dev_t devno = MKDEV(globalfifo_major, 0);
	
	//First :要分配的设备编号范围的初始值, 这组连续设备号的起始设备号, 相当于register_chrdev（）中主设备号
	//Count:连续编号范围.   是这组设备号的大小（也是次设备号的个数）
	//Name:编号相关联的设备名称. (/proc/devices); 本组设备的驱动名称
	if(globalfifo_major)
		ret = register_chrdev_region(devno, 1, "globalfifo");//静态注册
		//ret = register_chrdev_region(devno, DEVICE_NUM, "globalfifo");//静态注册
	else{
		ret = alloc_chrdev_region(&devno, 0, 1, "globalfifo");//动态注册
		//ret = alloc_chrdev_region(&devno, 0, DEVICE_NUM, "globalfifo");//动态注册
		globalfifo_major = MAJOR(devno);//从设备号中提取主设备号
	}
	
	if(ret < 0 )
		return ret;
	//kmalloc() 申请的内存位于物理内存映射区域，而且在物理上也是连续的，它们与真实的物理地址只有一个固定的偏移，
	//因为存在较简单的转换关系，所以对申请的内存大小有限制，不能超过128KB
	//kzalloc() 函数与 kmalloc() 非常相似，参数及返回值是一样的，可以说是前者是后者的一个变种，
	//因为 kzalloc() 实际上只是额外附加了 __GFP_ZERO 标志。所以它除了申请内核内存外，还会对申请到的内存内容清零。
	globalfifo_devp = kzalloc(sizeof(struct globalfifo_dev), GFP_KERNEL);
	//globalfifo_devp = kzalloc(sizeof(struct globalfifo_dev)*DEVICE_NUM, GFP_KERNEL);
	if(!globalfifo_devp){
		ret = -ENOMEM;
		goto fail_malloc;
	}
	
	globalfifo_setup_cdev(globalfifo_devp, 0);
	mutex_init(&globalfifo_devp->mutex);
	//初始化等待队列头部
	init_waitqueue_head(&globalfifo_devp->r_wait);
	init_waitqueue_head(&globalfifo_devp->w_wait);
	// for(i =0; i<DEVICE_NUM; i++)
		// globalfifo_setup_cdev(globalfifo_devp+i, i);
	
	printk(KERN_INFO "init char globalfifo okay.\n");
	return 0;
	
	fail_malloc:
	unregister_chrdev_region(devno, 1);
	//unregister_chrdev_region(devno, DEVICE_NUM);
	return ret;
}
module_init(globalfifo_init);

static void __exit globalfifo_exit(void)
{
	//int i;
	cdev_del(&globalfifo_devp->cdev);
	// for(i =0; i<DEVICE_NUM; i++)
		// cdev_del(&(globalfifo_devp +i)->cdev);

	kfree(globalfifo_devp);
	unregister_chrdev_region(MKDEV(globalfifo_major, 0), 1);
	//unregister_chrdev_region(MKDEV(globalfifo_major, 0), DEVICE_NUM);
	printk(KERN_INFO "exit char globalfifo okay.\n");
}
module_exit(globalfifo_exit);

MODULE_AUTHOR("Edwards");
MODULE_LICENSE("GPL v3");