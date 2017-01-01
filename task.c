//#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/sched.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/delay.h>	/* udelay */
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <asm/io.h>

//#include <linux/videodev.h> /* yum install libv4l-devel  ln -s /usr/include/libv4l1-videodev.h /lib/modules/3.10.0-514.2.2.el7.x86_64.debug/build/include/linux/videodev.h  */
/* Commented out for stdint.h:9:26: error: no include path in which to search for stdint.h*/

#include <linux/videodev2.h> /* Fix for error: ‘VIDIOC_REQBUFS’ undeclared (first use in this function) , V4L2_MEMORY_MMAP .... */

#include "dm642_pci.h"

extern int task_iopackets; 
extern int iopacket_size;

static int alloc_iopackets(struct dm642_task *task, int mode )
{
	struct iopacket *p;
	int i;
	unsigned long addr;
	int size;

	for(i=0; i<task_iopackets; i++)
	{
		p = kmalloc(sizeof(struct iopacket),GFP_KERNEL);
		if(p == NULL)
		{
			printk(KERN_ERR "kmalloc iopacket failed!");
			return -ENOMEM;
		}	
		memset(p, 0, sizeof(struct iopacket));
		p->data = (void *)__get_free_pages(GFP_KERNEL, 
					get_order(iopacket_size)
					);
		if(p->data == NULL)
		{	
			kfree(p);
			printk(KERN_ERR "kmalloc iopacket->data failed!");
			return -ENOMEM;
		}
		memset(p->data, 0, iopacket_size);
		size = iopacket_size;
		addr = (unsigned long) p->data;
		while (size > 0) {
			SetPageReserved(virt_to_page((void *)addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
		if(mode == O_WRONLY)
		{
			if(task->write_ary == NULL)
			{
				task->write_ary = kmalloc(
						sizeof(void *) * task_iopackets,
						GFP_KERNEL);
				if(task->write_ary)
					memset(task->write_ary, 0, 
						task_iopackets *sizeof(void *));
			}
		 	if(task->write_ary)
				task->write_ary[i] = p;	
			p->index = i;
			list_add_tail(&p->list, &task->write_f_queue);
		}else if(mode == O_RDONLY)
		{
			if(task->read_ary == NULL)
			{
				task->read_ary = kmalloc(
						sizeof(void *) * task_iopackets,
						GFP_KERNEL);
				if(task->read_ary)
					memset(task->read_ary, 0, 
						task_iopackets *sizeof(void *));
			}
		 	if(task->read_ary)
				task->read_ary[i] = p;
			p->index = i;	
			list_add_tail(&p->list, &task->read_f_queue);
		}else
			printk(KERN_ERR "%s.%d: should not reach here!",__FUNCTION__,__LINE__);	
			
	}
	return 0;	
}

static int task_alloc_iopackets(struct dm642_task *task, int flags )
{
	int ret = 0;
	TRACE();
	if ( (flags & O_ACCMODE) == O_WRONLY){
		if(atomic_inc_and_test(&task->write_allocated))
		{
			TRACE("O_WRONLY");
			ret = alloc_iopackets(task, O_WRONLY);
			if(ret < 0)
				atomic_dec(&task->write_allocated);
		}
	}else if ( (flags & O_ACCMODE) == O_RDONLY){
		if(atomic_inc_and_test(&task->read_allocated))
		{
			TRACE("O_RDONLY");
			ret = alloc_iopackets(task, O_RDONLY);
			if(ret < 0)
				atomic_dec(&task->read_allocated);
		}
	}else if ( (flags & O_ACCMODE) == O_RDWR){
		if(atomic_inc_and_test(&task->write_allocated))
		{
			TRACE("O_RDWR WRONLY");
			ret = alloc_iopackets(task, O_WRONLY);
			if(ret < 0)
				atomic_dec(&task->write_allocated);
		}
		if(atomic_inc_and_test(&task->read_allocated))
		{
			TRACE("O_RDWR RDONLY");
			ret = alloc_iopackets(task, O_RDONLY);
			if(ret < 0)
				atomic_dec(&task->read_allocated);
		}
	}
	return ret;
}

static void task_reset(struct dm642_task *task)
{
	struct list_head *cur,*next;
	TRACE();
	task->read_seq = 0;
	task->write_seq = 0;	
	if(atomic_read(&task->write_allocated) >= 0)
	{
		TRACE("reset write_f_queue");
		list_for_each_safe(cur, next, &task->write_r_queue)
		{
			list_del(cur);
			list_add_tail(cur, &task->write_f_queue);
		}	
		
	}
	if(atomic_read(&task->read_allocated) >= 0)
	{
		TRACE("reset read_f_queue");
		list_for_each_safe(cur, next, &task->read_r_queue)
		{
			list_del(cur);
			list_add_tail(cur, &task->read_f_queue);
		}
		list_for_each_safe(cur, next, &task->read_c_queue)
		{
			list_del(cur);
			list_add_tail(cur, &task->read_f_queue);
		}
	}
}

void delete_task(struct dm642_dev *dev, struct dm642_task *task)
{
	struct iopacket *p;
	unsigned long addr;
	int size;
	
	TRACE();
	task_reset(task);
	list_for_each_entry(p, &task->read_f_queue, list)
	{
		if(p->data)
		{
			size = iopacket_size;
			addr = (unsigned long) p->data;
			while (size > 0) {
				ClearPageReserved(virt_to_page((void *)addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			free_pages((unsigned long)p->data, get_order(iopacket_size));
		}
		if(p)
			kfree(p);
	}
	if(task->read_ary)
		kfree(task->read_ary);	
	INIT_LIST_HEAD(&task->read_f_queue);
	list_for_each_entry(p, &task->write_f_queue, list)
	{
		if(p->data)
		{
			size = iopacket_size;
			addr = (unsigned long) p->data;
			while (size > 0) {
				ClearPageReserved(virt_to_page((void *)addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			free_pages((unsigned long)p->data, get_order(iopacket_size));
		}
		if(p)
			kfree(p);
	}	
	INIT_LIST_HEAD(&task->write_f_queue);
	if(task->write_ary)
		kfree(task->write_ary);
	kfree(task);
}

static int task_init(struct dm642_dev *dev, struct dm642_task **task_out)
{
	struct dm642_task *task;
	unsigned long flags;
	
	TRACE();	
	*task_out = NULL;
	task = kmalloc(sizeof(struct dm642_task),GFP_KERNEL);
	if(task == NULL)
	{	
		printk(KERN_ERR "kmalloc dm642_task failed!");
		return -ENOMEM;
	}
	memset(task, 0, sizeof(struct dm642_task));

	init_waitqueue_head(&task->readq);
	init_waitqueue_head(&task->writeq);

	task->read_seq = 0;
	task->write_seq = 0;	

	INIT_LIST_HEAD(&task->read_f_queue);
	INIT_LIST_HEAD(&task->read_r_queue);
	INIT_LIST_HEAD(&task->read_c_queue);
	INIT_LIST_HEAD(&task->write_f_queue);
	INIT_LIST_HEAD(&task->write_r_queue);
	spin_lock_init(&task->read_queue_spinlock);
	spin_lock_init(&task->write_queue_spinlock);
 
	atomic_set(&task->read_allocated,-1);
	atomic_set(&task->write_allocated,-1);
	task->access_count = 0;
	task->dev = dev;
	
	spin_lock_irqsave(&dev->task_list_spinlock, flags);
	list_add(&task->list, &dev->task_list);
	spin_unlock_irqrestore(&dev->task_list_spinlock, flags);
	*task_out = task;
	return 0;
}

static struct dm642_task *find_task(struct dm642_dev *dev, u32 id)
{
	struct dm642_task *task;
	unsigned long flags;
	
	spin_lock_irqsave(&dev->task_list_spinlock, flags);
	list_for_each_entry(task, &dev->task_list, list){
		if(task->id == id)
		{
			spin_unlock_irqrestore(&dev->task_list_spinlock, flags);
			return task;
		}
	}
	spin_unlock_irqrestore(&dev->task_list_spinlock, flags);
	return NULL;
}
 
extern int request_io(struct dm642_task *task, struct iopacket *p);
extern int update_pci_dsp_handshake_addr(u32 addr);
extern int pci_dsp_handshake_addr;
static int task_open (struct inode *inode, struct file *filp)
{
	unsigned int minor;
	struct dm642_dev *dev; /* device information */
	struct dm642_task *task;
	int ret = 0;
	
	TRACE("");
	dev = container_of(inode->i_cdev, struct dm642_dev, cdev);
	if( down_interruptible(&dev->task_access_sem) )
		return -ERESTARTSYS;
	minor = iminor(inode);
	task = find_task(dev, minor);
	TRACE("task = %p", task);
	if(task == NULL){
		ret = task_init(dev, &task);	
		if(ret < 0)
			goto out;
		task->id = minor;
	}
	if( (task->access_count == 0) )
	{
		u32 pcima = ioread32(dev->mmr_offset1 + PCIMA);
		if(pcima == (REQUEST_MAGIC & 0xfffffffc))
		{
			u32 dspma = ioread32(dev->mmr_offset1 + DSPMA);
			update_pci_dsp_handshake_addr(dspma);
			iowrite32(ACK_MAGIC, dev->mmr_offset1 + PCIMA);
		}
		if(pci_dsp_handshake_addr == -1)
		{
			printk(KERN_ERR"should load dsp side app first!\n");
			up(&dev->task_access_sem);
			return -ENODEV;
		} 
		struct iopacket p;
		if(dev->printkBuf)
		{
			p.state = READ_REQUEST;
			p.rlen = iopacket_size;
			p.data = dev->printkBuf;
			p.seq = 0xdeb9;
			//dirty way 
			task->id = 0;
			request_io(task, &p);
			task->id = minor;
		}
		p.state = OPEN;
		request_io(task, &p);
		
	}	
	ret = task_alloc_iopackets(task, filp->f_flags);
	if(ret < 0)
		goto out;
	filp->private_data = task;
	task->access_count++;
out:
	up(&dev->task_access_sem);
	return ret;
}

static int task_release (struct inode *inode, struct file *filp)
{
	struct dm642_dev *dev = container_of(inode->i_cdev, struct dm642_dev, cdev);
	struct dm642_task *task = filp->private_data;

	TRACE();
	if( down_interruptible(&dev->task_access_sem) )
		return -ERESTARTSYS;	
	if(--task->access_count == 0)
	{
#ifdef RELEASE_RESOURCE_AFTER_CLOSE
		spin_lock_irqsave(&dev->task_list_spinlock, flags);
		list_del(&task->list);
		spin_unlock_irqrestore(&dev->task_list_spinlock, flags);
		delete_task(dev,task);	
#else
		task_reset(task);
#endif
		struct iopacket p;
		p.state = CLOSE;
		request_io(task, &p);
	}
	up(&dev->task_access_sem);
	return 0;
}

static ssize_t try_to_complete_read(struct dm642_task *task, char __user *buf, size_t count)
{
	unsigned long flags;
	struct iopacket *p = NULL;
	struct list_head *next;
	ssize_t ret = 0;
	int toread = 0;
	cycles_t t1;
	cycles_t t2;	
	do{
		spin_lock_irqsave(&task->read_queue_spinlock, flags);
		if(list_empty(&task->read_c_queue)) {
        	    spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
                	TRACE("%s reading: going to sleep\n", current->comm);
			if (wait_event_interruptible(task->readq, 
				!list_empty(&task->read_c_queue)))
				return -ERESTARTSYS; 
	                spin_lock_irqsave(&task->read_queue_spinlock, flags);
		}
		while(!list_empty(&task->read_c_queue) && count)
		{
			next = (&task->read_c_queue)->next;
			list_del_init(next);	
			spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
			p = list_entry(
				next,
				struct iopacket,
				list);
			if(count > p->clen)
			{
				count -= p->clen;
				toread = p->clen;
			}else{
				toread = count;
				count = 0;
			}
			//TRACE("p->data: %s",(char *)p->data);
			if(ret == 0)
				task->r_cycles = p->r_cycles;
			t1 = get_cycles();
			if( copy_to_user(buf,p->data,toread) )
			{
				if(ret)
					return ret;
				else	
					return -EFAULT;
			}
			t2 = get_cycles();
			DPRINTK("copy_to_user: %d time: %lld\n",toread,t2-t1);
			buf += toread;
			ret += toread;
			spin_lock_irqsave(&task->read_queue_spinlock, flags);
			list_add_tail(next, &task->read_f_queue);
		}
		TRACE("count = %d",count);
		spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
	}while(count);
	if(p)
	{
		task->c_cycles = p->c_cycles;
		DPRINTK("read consume time: %lld\n", task->c_cycles - task->r_cycles);
	}
	TRACE("ret = %x",ret);
	return ret;
}

static int request_read(struct dm642_task *task, size_t count, int block)
{
	unsigned long flags;
	struct iopacket *p;
	struct list_head *next;
	do{
		if(block)
		{        
			spin_lock_irqsave(&task->read_queue_spinlock, flags);
	        	while (list_empty(&task->read_f_queue)) {
        	        	/* release the lock */
                		spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
                		TRACE("%s reading: going to sleep\n", current->comm);
	                	if (wait_event_interruptible(task->readq, !list_empty(&task->read_f_queue)))
        	                	return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
                		spin_lock_irqsave(&task->read_queue_spinlock, flags);
        		}
		}else{
			spin_lock_irqsave(&task->read_queue_spinlock, flags);
			if( list_empty(&task->read_f_queue))
			{
				spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
				return -EBUSY;
			}
		}
	        next = (&task->read_f_queue)->next;
        	list_del_init(next);
	        spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
        	p = list_entry(next,
                	struct iopacket,
	                list);
		if(count == 0)
		{
			p->rlen = iopacket_size;
			p->endblock = 1;
		}else if(count > iopacket_size)
		{
			p->rlen = iopacket_size;
			count -= iopacket_size;
			p->endblock = 0;
		}else{
        		p->rlen = count;
			p->endblock = 1;
			count = 0;
		}
		p->seq = task->read_seq++;
        	p->state = READ_REQUEST;
	
		spin_lock_irqsave(&task->read_queue_spinlock, flags);
        	list_add_tail(next, &task->read_r_queue);
		spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
       		TRACE("request_io"); 
		request_io(task,p);
	}while(count > 0);
	return 0;	
}

static ssize_t task_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct dm642_task *task = filp->private_data;
	int ret;

	TRACE("");
//	ret = try_to_complete_read(task, buf, count);
//	if(ret)
//		return ret;
  //      if (filp->f_flags & O_NONBLOCK)
    //    	return -EAGAIN;
	if(count > task_iopackets*iopacket_size)
		count = task_iopackets*iopacket_size;
	request_read(task, count, 0);
	ret = try_to_complete_read(task,buf,count);
	//request_read(task, 0, 0);
	return ret;
}

ssize_t task_write(struct file *filp, const char __user *buf, size_t count,
		loff_t *f_pos)
{
	struct dm642_task *task = filp->private_data;
	struct iopacket *p = NULL;
	struct list_head *next = NULL;
	unsigned long flags;
	/* only for test */
	TRACE("");
//#define TEST_READ
#ifdef TEST_READ
	spin_lock_irqsave(&task->read_queue_spinlock, flags);
	if(!list_empty(&task->read_r_queue))
	{
		next = (&task->read_r_queue)->next;
		list_del_init(next);
		list_add_tail(next,&task->read_c_queue);
	}
	spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
	if(next){
		p = list_entry(next,struct iopacket,list);
		count = min(count,p->rlen);
		if( copy_from_user(p->data, buf, count) )
			return -EFAULT;
		p->clen = count;
		TRACE("complte_len = %d",p->clen);
		wake_up_interruptible(&task->readq); 
	}
#else
        spin_lock_irqsave(&task->write_queue_spinlock, flags);
        while (list_empty(&task->write_f_queue)) {
                /* release the lock */
                spin_unlock_irqrestore(&task->write_queue_spinlock, flags);
                if (filp->f_flags & O_NONBLOCK)
                        return -EAGAIN;
                TRACE("%s write: going to sleep\n", current->comm);
                if (wait_event_interruptible(task->writeq, !list_empty(&task->write_f_queue)))
                        return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
                spin_lock_irqsave(&task->write_queue_spinlock, flags);
        }
        next = (&task->write_f_queue)->next;
        list_del_init(next);
        list_add_tail(next, &task->write_r_queue);
        spin_unlock_irqrestore(&task->write_queue_spinlock, flags);
        p = list_entry(next,
                struct iopacket,
                list);
        count = min(count, (size_t)iopacket_size);
	if( copy_from_user(p->data, buf, count) )
        	return -EFAULT;
	p->rlen = count;
	p->seq = task->write_seq++;
        p->state = WRITE_REQUEST;
        request_io(task,p);
#endif
	return count;
}

static int ioctl_usercopy(struct inode *inode, struct file *file, /* .ioctl to .unlocked_ioctl for task_ioctl */
//static int ioctl_usercopy(struct file *file,
	       unsigned int cmd, unsigned long arg,
	       //int (*func)(struct inode *inode, struct file *file, /* .ioctl to .unlocked_ioctl for task_ioctl */
		   int (*func)(struct inode *inode, struct file *file,
			   unsigned int cmd, void *arg))
{
	char	sbuf[128];
	void    *mbuf = NULL;
	void	*parg = NULL;
	int	err  = -EINVAL;

	/*  Copy arguments into temp kernel buffer  */
	switch (_IOC_DIR(cmd)) {
	case _IOC_NONE:
		parg = NULL;
		break;
	case _IOC_READ:
	case _IOC_WRITE:
	case (_IOC_WRITE | _IOC_READ):
		if (_IOC_SIZE(cmd) <= sizeof(sbuf)) {
			parg = sbuf;
		} else {
			/* too big to allocate from stack */
			mbuf = kmalloc(_IOC_SIZE(cmd),GFP_KERNEL);
			if (NULL == mbuf)
				return -ENOMEM;
			parg = mbuf;
		}
		
		err = -EFAULT;
		if (_IOC_DIR(cmd) & _IOC_WRITE)
			if (copy_from_user(parg, (void __user *)arg, _IOC_SIZE(cmd)))
				goto out;
		break;
	}

	/* call driver */
	err = func(inode, file, cmd, parg); /* .ioctl to .unlocked_ioctl for task_ioctl */
	// err = func(file, cmd, parg);
	if (err == -ENOIOCTLCMD)
		err = -EINVAL;
	if (err < 0)
		goto out;

	/*  Copy results into user buffer  */
	switch (_IOC_DIR(cmd))
	{
	case _IOC_READ:
	case (_IOC_WRITE | _IOC_READ):
		if (copy_to_user((void __user *)arg, parg, _IOC_SIZE(cmd)))
			err = -EFAULT;
		break;
	}

out:
	if (mbuf)
		kfree(mbuf);
	return err;
}

static int task_do_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, void *arg)
{
	struct dm642_task *task = filp->private_data;
	int retval = 0;
	switch(cmd) {
		case DM642_TASK_IOC_1:
			break;
		case VIDIOC_REQBUFS:
		{
                	struct v4l2_requestbuffers *req = arg;
			
                	if (req->memory != V4L2_MEMORY_MMAP)
                        	return -EINVAL;
                	if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
			    req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT )
                        	return -EINVAL;
			req->count = task_iopackets;
			break;	 
		}
        	case VIDIOC_QUERYBUF: 
		{
                	struct v4l2_buffer *buf = arg;
                	int index;
			enum v4l2_buf_type type;
			struct iopacket *iop = NULL;
			
			index = buf->index;
			type = buf->type;
                	if (index < 0 || index >= task_iopackets)
                        	return -EINVAL;
	                if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE && type != V4L2_BUF_TYPE_VIDEO_OUTPUT )
        	                return -EINVAL;
                	memset(buf, 0, sizeof(struct v4l2_buffer));
			if(type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			{
				if(task->read_ary)
					iop = task->read_ary[index]; 	 
			}else if(type == V4L2_BUF_TYPE_VIDEO_OUTPUT){
				if(task->write_ary)
					iop = task->write_ary[index];
			}
			if(iop)
			{
                		buf->type = type;
		                buf->index = iop->index;
        		        buf->bytesused = 0;
                		buf->flags = V4L2_BUF_FLAG_MAPPED;
				if(iop->state == READ_REQUEST || iop->state == WRITE_REQUEST ){ 	
        	        		buf->flags |= V4L2_BUF_FLAG_QUEUED;
        		        	buf->bytesused = iop->rlen;
				}else if(iop->state == READ_COMPLETE || iop->state == WRITE_COMPLETE){	
		                        buf->flags |= V4L2_BUF_FLAG_DONE;
					buf->bytesused = iop->clen;
				}
        		        //buf.timestamp = ;
                		buf->sequence = iop->seq;
		                buf->memory = V4L2_MEMORY_MMAP;
		                buf->m.offset = index * iopacket_size;
				if(type == V4L2_BUF_TYPE_VIDEO_OUTPUT){
					buf->m.offset += task_iopackets*iopacket_size;
				}
        		        buf->length = iopacket_size;
			}
			break;
		}
		case VIDIOC_QBUF: 
		{
                	struct v4l2_buffer *buf = arg;
                	int index;
			enum v4l2_buf_type type;
			struct iopacket *iop = NULL;
			unsigned long flags;
			
			index = buf->index;
			type = buf->type;
                	if (buf->memory != V4L2_MEMORY_MMAP)
                        	return -EINVAL;
	                if (index < 0 || index >= task_iopackets)
        	                return -EINVAL;
	                if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			{
				if(task->read_ary == NULL)
					return -EINVAL;
				iop = task->read_ary[index];
				if(iop)
				{
					assert(iop->state != READ_REQUEST);
					TRACE("CAPTURE iop->state: %d",iop->state);
					//if(iop->state == READ_REQUEST)
					//	return -EINVAL;
					spin_lock_irqsave(&task->read_queue_spinlock, flags);
					list_del_init(&iop->list);
	        			list_add_tail(&iop->list, &task->read_r_queue);
                			spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
					iop->rlen = iopacket_size;
					iop->seq = task->read_seq++;
					iop->state = READ_REQUEST;
        				request_io(task,iop);
        	        		buf->flags |= V4L2_BUF_FLAG_QUEUED;
		                	buf->flags &= ~V4L2_BUF_FLAG_DONE;
				}
			}else if(type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
			{
				TRACE("buf->bytesused: %d",buf->bytesused);
				if(buf->bytesused == 0 || buf->bytesused > iopacket_size)
					return -EINVAL;
				if(task->write_ary == NULL)
					return -EINVAL;
				iop = task->write_ary[index];
				if(iop)
				{
					assert(iop->state != WRITE_REQUEST);
					TRACE("OUTPUT iop->state: %d",iop->state);
					//if(iop->state == WRITE_REQUEST)
					//	return -EINVAL;
					spin_lock_irqsave(&task->write_queue_spinlock, flags);
					list_del_init(&iop->list);
        				list_add_tail(&iop->list, &task->write_r_queue);
                			spin_unlock_irqrestore(&task->write_queue_spinlock, flags);
					iop->rlen = buf->bytesused;
					iop->seq = task->write_seq++;
					iop->state = WRITE_REQUEST;
        				request_io(task,iop);
        	        		buf->flags |= V4L2_BUF_FLAG_QUEUED;
		                	buf->flags &= ~V4L2_BUF_FLAG_DONE;
				}
			}else
        	                return -EINVAL;
	                break;
        	}
		case VIDIOC_DQBUF: 
		{
                	struct v4l2_buffer *buf = arg;
			enum v4l2_buf_type type;
			struct iopacket *iop = NULL;
			unsigned long flags;
			struct list_head *next;
			
	                if (buf->memory != V4L2_MEMORY_MMAP)
        	                return -EINVAL;
			type = buf->type;
	                if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			{
				spin_lock_irqsave(&task->read_queue_spinlock, flags);
		        	while (list_empty(&task->read_c_queue)) {
                			spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
                			if (filp->f_flags & O_NONBLOCK)
		                        	return -EAGAIN;
	                		if (wait_event_interruptible(task->readq,!list_empty(&task->read_c_queue)))
                        			return -ERESTARTSYS;
	                		spin_lock_irqsave(&task->read_queue_spinlock, flags);
        			}
				next = (&task->read_c_queue)->next;
				list_del_init(next);
				spin_unlock_irqrestore(&task->read_queue_spinlock, flags);
				iop = list_entry(next,struct iopacket,list);
				buf->flags |= V4L2_BUF_FLAG_DONE;
			}else if(type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
			{
        			spin_lock_irqsave(&task->write_queue_spinlock, flags);
			        while (list_empty(&task->write_f_queue)) {
                			spin_unlock_irqrestore(&task->write_queue_spinlock, flags);
			                if (filp->f_flags & O_NONBLOCK)
                        			return -EAGAIN;
                			if (wait_event_interruptible(task->writeq, !list_empty(&task->write_f_queue)))
			                        return -ERESTARTSYS;
			                spin_lock_irqsave(&task->write_queue_spinlock, flags);
				}
        			next = (&task->write_f_queue)->next;
			        list_del_init(next);
			        spin_unlock_irqrestore(&task->write_queue_spinlock, flags);
			        iop = list_entry(next,struct iopacket,list);
			}else
				return -EINVAL;
			if(iop)
			{
				buf->index = iop->index;	
				buf->bytesused = iop->clen;
		                //buf->timestamp = ;
                		buf->sequence = iop->seq;
		                buf->memory = V4L2_MEMORY_MMAP;
				buf->length = iopacket_size;
			}
			break;
		}
	  	default:  /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}
	return retval;
}

//static int task_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg) /* .ioctl to .unlocked_ioctl for task_ioctl */
static long task_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	
	struct inode *inode = file_inode(filp); /* new >3.19 metod for getting inode*/
	return ioctl_usercopy(inode, filp, cmd, arg, task_do_ioctl);
}

static unsigned int task_poll(struct file *filp, poll_table *wait)
{
	return POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM;
}


extern int task_mmap(struct file *filp, struct vm_area_struct *vma);

struct file_operations task_fops = {
	.owner	 = THIS_MODULE,
	.open	 = task_open,
	.release = task_release,
	.read	 = task_read,
	.write	 = task_write,
	//.ioctl	 = task_ioctl, /* .ioctl to .unlocked_ioctl for task_ioctl */
	.unlocked_ioctl = task_ioctl,
	.mmap	 = task_mmap,
	.poll	 = task_poll,	
};

extern int get_complete_stuff(struct dm642_dev *dev, struct pci_dsp_handshake *hs);
irqreturn_t dm642_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct dm642_dev *dev = (struct dm642_dev *)dev_id;
	struct dm642_task *task = NULL;
	struct pci_dsp_handshake hs;
	struct list_head *next;
	struct iopacket *p;

	if(get_complete_stuff(dev,&hs))
		return IRQ_NONE;
	TRACE("ack: %08x",hs.request);
	TRACE("type: %d, id: %d, len: %d",hs.type, hs.task_id, hs.len);
	TRACE("seq: %08x", hs.seq);
	TRACE("addr: %08x", hs.addr);
	/* list_for_each_entry dirty the "task" pointer */
	//list_for_each_entry(task, &dev->task_list, list)
	list_for_each(next, &dev->task_list)
	{
		task = list_entry(next, struct dm642_task, list);
		if(task->id == hs.task_id)
			break;
		task = NULL;
	}
	TRACE("task=%p",task);
	if(task){
		if(hs.type == READ_COMPLETE){
			if(!list_empty(&task->read_r_queue)){
				next = (&task->read_r_queue)->next;	
				p = list_entry(next, struct iopacket, list);
				assert(p->seq == hs.seq);
				assert(p->rlen >= hs.len);
				p->clen = hs.len;
				p->state = hs.type;
				p->c_cycles = get_cycles();
				do_gettimeofday(&p->c_tv);
			        DPRINTK("read spend time(us): %06u CPU cycles: %06u\n",
				     (int)(p->c_tv.tv_usec-p->r_tv.tv_usec),
				     (int)(p->c_cycles-p->r_cycles));	
				list_del(next);
				list_add_tail(next, &task->read_c_queue);
				TRACE("p->endblock: %d",p->endblock);
				if(p->endblock)
					wake_up_interruptible(&task->readq); 
			}else{
				printk(KERN_ERR "%s.%d: should not reach here!",__FUNCTION__,__LINE__);	
			}
		}else if(hs.type == WRITE_COMPLETE){
			if(!list_empty(&task->write_r_queue)){
				next = (&task->write_r_queue)->next;	
				p = list_entry(next, struct iopacket, list);
				assert(p->seq == hs.seq);
				assert(p->rlen == hs.len);
				p->state = hs.type;
				p->c_cycles = get_cycles();
				do_gettimeofday(&p->c_tv);
			        DPRINTK("write spend time(us): %06u CPU cycles: %06u\n",
				     (int)(p->c_tv.tv_usec-p->r_tv.tv_usec),
				     (int)(p->c_cycles-p->r_cycles));	
				list_del(next);
				list_add_tail(next, &task->write_f_queue);
				wake_up_interruptible(&task->writeq); 
			}else{
				printk(KERN_ERR "%s.%d: should not reach here!",__FUNCTION__,__LINE__);	
			}
		}else{
			printk(KERN_ERR "%s.%d: should not reach here!",__FUNCTION__,__LINE__);	
		}
	}else{
		if(hs.task_id == 0 && hs.type == READ_COMPLETE)
		{
			if(dev->printkBuf)
			{
				assert(hs.len <= iopacket_size);
				dev->printkBuf[hs.len] = 0;
				printk(dev->printkBuf);
			}
		}else
			printk(KERN_ERR "%s.%d: should not reach here!",__FUNCTION__,__LINE__);	
	}
		
        return IRQ_HANDLED;
}