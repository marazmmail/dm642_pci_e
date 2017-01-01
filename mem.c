//#include <linux/config.h> /* Fix for modern kernels */ 
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>

//#include <asm/system.h>		/* cli(), *_flags */ /* Fix for modern kernels */ 
#include <asm/uaccess.h>	/* copy_*_user */
#include <asm/io.h>

#include "dm642_pci.h"

extern int dm642_tasks_per_dev;
extern struct file_operations task_fops;
static int dm642_open (struct inode *inode, struct file *filp)
{
	unsigned int minor,id;
	struct dm642_dev *dev; /* device information */
	dev = container_of(inode->i_cdev, struct dm642_dev, cdev);
	minor = iminor(inode);
	TRACE("open dm642_dev: %p minor = %d", dev, minor);
	filp->private_data = dev; /* for other methods */
	id = minor / (1+dm642_tasks_per_dev) * (1+dm642_tasks_per_dev);
	if( id != minor)
	{
		filp->f_op = &task_fops;
		if (filp->f_op->open)
			return filp->f_op->open(inode,filp);
	}
	return 0;
}

static int dm642_release (struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t dm642_read (struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct dm642_dev *dev = filp->private_data;
	u32 remain = count; 
	u32 toread = 0;
	u8 *tmp_buf = (u8 *)buf;
	u8 *dsp_addr;	
	u32 dspp_page_index;
	u32 dspp_page_offset; 
 
	/*
	* CAUTION: use copy_to_user access the dsp memory space directly, 
	* this can work on x86 architecture. on some architectures 
	* the memory mapped IO stuff needs to be accessed differently.
	* Also is the PCI interface access 8-bit bytes, 16-bit halfwords,
	* 24-bit words, or 32-bit words is copy_to_user implemation relative.
	* 
	* The proper way of getting at I/O memory is via a set fo functions
	* (defined via <asm/io.h>) provided for that purpose(ldd3 page 250.).
	* ioread8/16/32, memcpy_fromio/toio etc.   
	*/
	if( down_interruptible(&dev->hw_access_sem) )
		return -ERESTARTSYS;	
	while(remain)
	{
		dspp_page_offset = *f_pos & (DSPP_PAGE_SIZE - 1);
		if( dspp_page_offset + remain > DSPP_PAGE_SIZE)
			toread = DSPP_PAGE_SIZE - dspp_page_offset;
		else 
			toread = remain;
 		dsp_addr = dev->dspmem_base + dspp_page_offset;
        	dspp_page_index = (*f_pos & DSPP_PAGE_MASK)>>DSPP_PAGE_SHIFT;
		outl(dspp_page_index,dev->io_base+DSPP);
		toread -= copy_to_user(tmp_buf, dsp_addr, toread);
		if(!toread)
		{
			up(&dev->hw_access_sem);
			if( (count - remain) == 0 )	
				return -EFAULT;
			else
				return (count - remain);
		}
		*f_pos += toread;
		remain -= toread;
		tmp_buf += toread;
    	}	
	up(&dev->hw_access_sem);
     	return count;   
}

static ssize_t dm642_write (struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct dm642_dev *dev = filp->private_data;
	u32 remain = count; 
	u32 towrite = 0;
	u8 *tmp_buf = (u8 *)buf;
	u8 *dsp_addr;	
	u32 dspp_page_index;
	u32 dspp_page_offset; 
	
	if( down_interruptible(&dev->hw_access_sem) )
		return -ERESTARTSYS;	
	while(remain)
	{
		dspp_page_offset = *f_pos & (DSPP_PAGE_SIZE - 1);
		if( dspp_page_offset + remain > DSPP_PAGE_SIZE)
			towrite = DSPP_PAGE_SIZE - dspp_page_offset;
		else 
			towrite = remain;
 		dsp_addr = dev->dspmem_base + dspp_page_offset;
        	dspp_page_index = (*f_pos & DSPP_PAGE_MASK)>>DSPP_PAGE_SHIFT;
		outl(dspp_page_index,dev->io_base+DSPP);
		towrite -= copy_from_user(dsp_addr, tmp_buf, towrite);
		if(!towrite)
		{
			up(&dev->hw_access_sem);
			if( (count - remain) == 0 )	
				return -EFAULT;
			else
				return (count - remain);
		}
		*f_pos += towrite;
		remain -= towrite;
		tmp_buf += towrite;
    	}	
	up(&dev->hw_access_sem);
     	return count;   
}

extern int dm642_reset_dsp (struct dm642_dev *dev);
extern void dm642_unreset_dsp(struct dm642_dev *dev);
//static int dm642_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg) /* Fix for modern kernels */ 
static long dm642_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dm642_dev *dev = filp->private_data;
	//struct inode *inode = file_inode(filp); /* Fix for modern kernels */ /* inode not used */
	int err = 0;
	long retval = 0;
	struct emif emif;
   	struct regval regval; 
	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != DM642_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > DM642_IOC_MAXNR) return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;
	switch(cmd) {
		case DM642_IOC_WARMRESET:
			dm642_reset_dsp (dev);
			break;
	  	case DM642_IOC_RUN:
			dm642_unreset_dsp (dev);
			break;
		case DM642_IOCS_INITEMIF:
			if( copy_from_user(&emif, (void __user *)arg, 
					sizeof(struct emif)) )
				return -EFAULT;
			TRACE("emfi.GBLCTL: %08x", emif.GBLCTL);
			memcpy_toio(dev->mmr_base, &emif, sizeof(struct emif));
			break;		
		case DM642_IOCS_REGVAL:
			if( copy_from_user(&regval, (void __user *)arg,
					sizeof(struct regval)) )
				return -EFAULT;
			if( regval.reg < MMR_BASE ) 
				return -EINVAL;
			regval.reg -= MMR_BASE;
			if(regval.reg > MMR_SIZE)
				return -EINVAL;
			iowrite32(regval.val, dev->mmr_base+regval.reg); 
			break;
		case DM642_IOCG_REGVAL:
			if( copy_from_user(&regval, (void __user *)arg,
					sizeof(struct regval)) )
				return -EFAULT;
			if( regval.reg < MMR_BASE ) 
				return -EINVAL;
			regval.reg -= MMR_BASE;
			if(regval.reg > MMR_SIZE)
				return -EINVAL;
			regval.val = ioread32(dev->mmr_base+regval.reg);
			if( copy_to_user((void __user *)arg, &regval,  
					sizeof(struct regval)) )
				return -EFAULT;
			break;
		case DM642_IOCH_REGVAL:
			if( arg < MMR_BASE ) 
				return -EINVAL;
			arg -= MMR_BASE;
			if(arg > MMR_SIZE)
				return -EINVAL;
			retval = ioread32(dev->mmr_base+arg);
			TRACE("ioread32: %08lx: %08x",arg, retval); 
			return retval;
	  	default:  /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}
	return retval;
}

static loff_t dm642_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	switch(whence) {
	case 0: /* SEEK_SET */
		newpos = off;
		break;
	case 1: /* SEEK_CUR */
		newpos = filp->f_pos + off;
		break;
	case 2: /* SEEK_END */
	default: /* can't happen */
		return -EINVAL;
	}
	if (newpos<0) return -EINVAL;
	TRACE("newpos: %x",newpos);
	filp->f_pos = newpos;
	return newpos;
}

struct file_operations dm642_fops = {
	.owner	 = THIS_MODULE,
	.open	 = dm642_open,
	.release = dm642_release,
	.llseek  = dm642_llseek,
	.read	 = dm642_read,
	.write	 = dm642_write,
	//.ioctl	 = dm642_ioctl, /* Fix for modern kernels */
	.unlocked_ioctl = dm642_ioctl,
};

extern int dm642_major;
int dm642_setup_cdev(struct dm642_dev *dev, int index)
{
	int err, devno = MKDEV(dm642_major, index);
    printk(KERN_NOTICE "Adding dm642_dev%d", index); /* Print if init */
	cdev_init(&dev->cdev, &dm642_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &dm642_fops;
	err = cdev_add (&dev->cdev, devno, 1+dm642_tasks_per_dev);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding dm642_dev%d", err, index);
	return err;
}

void dm642_unsetup_cdev(struct dm642_dev *dev)
{
	cdev_del(&dev->cdev);
}

