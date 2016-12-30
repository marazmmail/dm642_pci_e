//#include <linux/config.h> /* No more config.h in kernels > 2.6 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
//#include <linux/fcntl.h> /*Helps fix O_ACCMODE*/
//#include <linux/sched.h> /*Helps fix TASK_UNINTERRUPTIBLE */
#include <linux/fs.h> /*Helps fix the struct intializer */
#include <asm/irq_vectors.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include "dm642_pci.h"

#define IFS DRV_NAME ": "

int dm642_major = DM642_MAJOR;
int dm642_devs = DM642_DEVS;	
int dm642_tasks_per_dev = DM642_TASKS_PER_DEV;
int task_iopackets = TASK_IOPACKETS; 
int iopacket_size = IOPACKET_SIZE;
int pci_dsp_handshake_addr = PCI_DSP_HANDSHAKE_ADDR; 

module_param(dm642_major, int, 0);
module_param(dm642_devs, int, 0);
module_param(dm642_tasks_per_dev, int, 0);
module_param(task_iopackets, int, 0);
module_param(iopacket_size, int, 0);
module_param(pci_dsp_handshake_addr, int, 0);

int g_handshake_page_offset;
int g_handshake_page_index;
int g_devs_num = 0;
struct dm642_dev **g_devs = NULL;
//static spinlock_t g_devs_spinlock = SPIN_LOCK_UNLOCKED; /*  */
static DEFINE_SPINLOCK(g_devs_spinlock) ;


static struct pci_device_id ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TMS320DM642), },
	{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TMS320C6205), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

static const char *resource_flag_to_str(unsigned long flags)
{
	if(flags & IORESOURCE_IO)
		return "IO";
	if(flags & IORESOURCE_MEM)
	{
		if(flags & IORESOURCE_PREFETCH)
			return "Prefetchable memory";
		else
			return "Non-Prefetchable memory";
	}
	return "Unknown region";
}

static void dm642_cleanup_dev(struct dm642_dev *dev)
{
	if(dev == NULL)
		return;
	if(dev->dspmem_base)
		iounmap(dev->dspmem_base);
	if(dev->mmr_base);
		iounmap(dev->mmr_base);	
	pci_release_regions (dev->pci_dev);
	pci_disable_device(dev->pci_dev);
	kfree(dev);	
}

static int dm642_init_board(struct pci_dev *pdev, struct dm642_dev **dev_out)
{
	struct dm642_dev *dev;
	int rc = -ENODEV;
	unsigned long start,len,flags;

	*dev_out = NULL;
	dev = kmalloc(sizeof(struct dm642_dev), GFP_KERNEL);
	if (!dev) {
		printk (KERN_ERR IFS "%s: Unable to alloc new net device\n", pci_name(pdev));
		return -ENOMEM;
	}	
	TRACE("new dm642_dev: %p", dev);
	memset(dev, 0, sizeof(struct dm642_dev));
	dev->pci_dev = pdev;
	
	rc = pci_enable_device (pdev);
	if (rc)
		goto err_out;
	rc = pci_request_regions (pdev, DRV_NAME);
	if (rc)
		goto err_out;
	/* enable PCI bus-mastering */
	pci_set_master (pdev);
	start = pci_resource_start (pdev, 0);
	len = pci_resource_len (pdev, 0);
	flags = pci_resource_flags (pdev, 0);
	if( (len != DSPP_PAGE_SIZE) || !(flags&IORESOURCE_PREFETCH) )
		goto err_out;
	dev->dspmem_base = ioremap(start, len);
	if(dev->dspmem_base == NULL)
	{
		rc = -EIO;
		goto err_out;	
	}
	printk(KERN_INFO IFS "%s: Base0: %lx, len: %lx. - %s MAP to: %p\n",
		pci_name(pdev),	
		start,
		len,
		resource_flag_to_str(flags),
		dev->dspmem_base
		);
	
	start = pci_resource_start (pdev, 1);
	len = pci_resource_len (pdev, 1);
	flags = pci_resource_flags (pdev, 1);
	if( !(flags&IORESOURCE_MEM) )
		goto err_out;
	dev->mmr_base = ioremap(start, len);
	if(dev->mmr_base == NULL)
	{
		rc = -EIO;
		goto err_out;
	}
	if(pdev->device == PCI_DEVICE_ID_TMS320DM642)
	{ 
		dev->mmr_offset1 = dev->mmr_base + C64X_MMR_OFFSET1; 
		dev->mmio_offset = dev->mmr_base + C64X_MMIO_OFFSET;
	}else if(pdev->device == PCI_DEVICE_ID_TMS320C6205){
		dev->mmr_offset1 = dev->mmr_base + C62X_MMR_OFFSET1; 
		dev->mmio_offset = dev->mmr_base + C62X_MMIO_OFFSET;
	}else{
		printk(KERN_ERR "%s.%d should not run here",__FUNCTION__,__LINE__);
		dev->mmr_offset1 = dev->mmr_base;
		dev->mmio_offset = dev->mmr_base;
	}
	printk(KERN_INFO IFS "%s: Base1: %lx, len: %lx. - %s MAP to: %p\n",
		pci_name(pdev),	
		start,
		len,
		resource_flag_to_str(flags),
		dev->mmr_base
		);
	start = pci_resource_start (pdev, 2);
	len = pci_resource_len (pdev, 2);
	flags = pci_resource_flags (pdev, 2);
	if( !(flags & IORESOURCE_IO) )
		goto err_out;
	dev->io_base = start;
	printk(KERN_INFO IFS "%s: Base2: %lx, len: %lx. - %s\n",
		pci_name(pdev),	
		start,
		len,
		resource_flag_to_str(flags)
		);
	printk(KERN_INFO IFS "irq = %d\n",pdev->irq);
	*dev_out = dev;
	return 0;	
err_out:
	dm642_cleanup_dev(dev);
	return rc;	
}

void host_enable_int(struct dm642_dev *dev);
extern irqreturn_t dm642_interrupt(int irq, void *dev_id, struct pt_regs *regs);
extern int dm642_setup_cdev(struct dm642_dev *dev, int index);
static int probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct dm642_dev *dev;
	int ret;
	int i;
	u8 revision;
	TRACE();

	pci_read_config_byte(pdev, PCI_REVISION_ID, &revision);
	printk(KERN_INFO IFS "pci dev %s (id %04x:%04x rev %02x) detected.\n",
		       pci_name(pdev), pdev->vendor, pdev->device, revision);
	
	ret = dm642_init_board(pdev, &dev);
	if(ret < 0)
		return ret;
	pci_set_drvdata (pdev, dev);

	dev->printkBuf = (char *)__get_free_pages(GFP_KERNEL, get_order(iopacket_size));
	if(dev->printkBuf == NULL)
	{
		dm642_cleanup_dev(dev);
		return -ENOMEM;
	}
	memset(dev->printkBuf, 0, iopacket_size);
	
	//init_MUTEX(&dev->hw_access_sem); /*  init_MUTEX{_LOCKED}() was initially implemented as a semaphore.  */
	//init_MUTEX(&dev->task_access_sem); /*  init_MUTEX{_LOCKED}() was initially implemented as a semaphore.  sema_init(&(lptr->device.sem), 1); */
	sema_init(&dev->hw_access_sem,1);
	sema_init(&dev->task_access_sem,1);

	INIT_LIST_HEAD(&dev->task_list);
	spin_lock_init(&dev->task_list_spinlock);

	spin_lock(&g_devs_spinlock);
	for(i=0; i<dm642_devs; i++)
	{
		if(g_devs[i] == NULL)
			break; 	
	}
	dev->index = i;
	g_devs[i] = dev;
	g_devs_num++;
	spin_unlock(&g_devs_spinlock);

	
	ret = request_irq (dev->pci_dev->irq, dm642_interrupt, IRQF_SHARED, "dm642_pci", dev);
	if (ret){
		printk(KERN_ERR "request_irq %d failed!",dev->pci_dev->irq);
	}

	i *= (1+dm642_tasks_per_dev);
	dm642_setup_cdev(dev, i);	
	host_enable_int(dev);
	return 0;
}

extern void dm642_unsetup_cdev(struct dm642_dev *dev);
static void remove(struct pci_dev *pdev)
{
	struct dm642_dev *dev = pci_get_drvdata (pdev);
	TRACE();
	/* clean up any allocated resources and stuff here.
	 * like call release_region();
	 */
	spin_lock(&g_devs_spinlock);
	g_devs_num--;
	g_devs[dev->index] = NULL;
	spin_unlock(&g_devs_spinlock);

	free_irq(dev->pci_dev->irq, dev);
	dm642_unsetup_cdev(dev);
	if(dev->printkBuf)
		free_pages((unsigned long)dev->printkBuf, 
				get_order(iopacket_size));
	dm642_cleanup_dev(dev);
}

static struct pci_driver pci_driver = {
	.name = DRV_NAME,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};

int alloc_devs(void)
{
	int result;
	dev_t dev = MKDEV(dm642_major, 0);
	int devcount = dm642_devs*(1+dm642_tasks_per_dev);
	
	if (dm642_major)
	{
		result = register_chrdev_region(dev, 
			devcount, 
			DRV_NAME);
	}else {
		result = alloc_chrdev_region(&dev, 0, 
				devcount,
				DRV_NAME);
		dm642_major = MAJOR(dev);
	}
	if (result < 0)
		return result;
	printk(KERN_INFO"alloc_chrdev_region: major = %d count = %d\n",
		dm642_major, devcount);
	g_devs = kmalloc( sizeof(struct dm642_dev *)*dm642_devs, GFP_KERNEL);
	if(g_devs == NULL)
	{
		unregister_chrdev_region(MKDEV (dm642_major, 0), devcount);
		return -ENOMEM;
	}
	memset(g_devs, 0, sizeof(struct dm642_dev *)); 
	return 0; /* succeed */
}

void free_devs(void)
{
	int devcount = dm642_devs*(1+dm642_tasks_per_dev);
	unregister_chrdev_region(MKDEV (dm642_major, 0), devcount);
}

extern int dm642_create_proc( void );
extern int update_pci_dsp_handshake_addr(u32 addr);
static int __init pci_dm642_init(void)
{
	int ret;
	TRACE();
	update_pci_dsp_handshake_addr(pci_dsp_handshake_addr);
	ret = alloc_devs();
	if(ret < 0)
		return ret;
	dm642_create_proc();
	return pci_register_driver(&pci_driver);
}

extern void delete_task(struct dm642_dev *dev, struct dm642_task *task);
extern void dm642_remove_proc( void );
static void __exit pci_dm642_exit(void)
{
	struct dm642_task *task;
	int i;
	TRACE();
	dm642_remove_proc();
	free_devs();
	for(i=0; i<g_devs_num; i++)
	{
		struct dm642_dev *dev = g_devs[i];
		list_for_each_entry(task, &dev->task_list, list)
		{
			delete_task(dev,task);
		}
	}
	pci_unregister_driver(&pci_driver);
	/* must be here */
	kfree(g_devs);
}

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("yliqiang@gmail.com");

module_init(pci_dm642_init);
module_exit(pci_dm642_exit);

