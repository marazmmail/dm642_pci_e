#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <asm/io.h>
#include "dm642_pci.h"

extern int g_devs_num;
extern struct dm642_dev **g_devs;

static void *
dm642_seq_start (struct seq_file *s, loff_t * pos)
{
	if (*pos >= g_devs_num)
    		return NULL;		/* No more to read */
  	return g_devs[*pos];
}

static void *
dm642_seq_next (struct seq_file *s, void *v, loff_t * pos)
{
  	(*pos)++;
  	if (*pos >= g_devs_num)
    		return NULL;
  	return g_devs[*pos];
}

static void
dm642_seq_stop (struct seq_file *s, void *v)
{
}

static void dump_list(struct seq_file *s, struct list_head *list)
{
	struct list_head *next;
	struct iopacket *p;
	seq_printf(s, "%p",list);	
	for(next = list->next; next != list; next = next->next)
	{
		p = list_entry(next, struct iopacket, list);
		seq_printf(s, " -> %p(%p)",next, p->data); 
	}
	seq_printf(s, "\n");	
}

static int
dm642_seq_show (struct seq_file *s, void *v)
{
  	struct dm642_dev *dev = (struct dm642_dev *) v;
	struct dm642_task *task;
  	/* hw_acces_sem don't need if don't access memory space */

  	//if (down_interruptible(&dev->hw_access_sem))
  	//      return -ERESTARTSYS;
  	seq_printf (s, "dm642_%i:\n", dev->index);
	seq_printf (s, "spins: %d\n", dev->spins);
  	seq_printf (s, "HSR: %08x HDCR: %08x DSPP: %08x\n",
	      inl (dev->io_base + HSR),
	      inl (dev->io_base + HDCR), inl (dev->io_base + DSPP));
	seq_printf (s, "RSTSRC: %08x PCIIS: %08x PCIIEN: %08x\n",
		ioread32(dev->mmr_offset1 + RSTSRC),
		ioread32(dev->mmr_offset1 + PCIIS),
		ioread32(dev->mmr_offset1 + PCIIEN));
	seq_printf (s, "DSPMA: %08x PCIMA: %08x PCIMC: %08x\n",
		ioread32(dev->mmr_offset1 + DSPMA),
		ioread32(dev->mmr_offset1 + PCIMA),
		ioread32(dev->mmr_offset1 + PCIMC));
	seq_printf (s, "CDSPA: %08x CPCIA: %08x CCNT: %08x\n",
		ioread32(dev->mmr_offset1 + CDSPA),
		ioread32(dev->mmr_offset1 + CPCIA),
		ioread32(dev->mmr_offset1 + CCNT));
	list_for_each_entry(task, &dev->task_list, list)
	{
		seq_printf(s, "task: %d\n",task->id);
		seq_printf(s, "read free queue:\n");
		dump_list(s, &task->read_f_queue);
		seq_printf(s, "read request queue:\n");
		dump_list(s, &task->read_r_queue);
		seq_printf(s, "read complete queue:\n");
		dump_list(s, &task->read_c_queue);
		seq_printf(s, "write free queue:\n");
		dump_list(s, &task->write_f_queue);
		seq_printf(s, "write request queue:\n");
		dump_list(s, &task->write_r_queue);
	}
  	//up(&dev->hw_access_sem);
  	return 0;
}

static struct seq_operations dm642_seq_ops = {
  	.start = dm642_seq_start,
  	.next = dm642_seq_next,
  	.stop = dm642_seq_stop,
  	.show = dm642_seq_show
};


static int
dm642_open (struct inode *inode, struct file *file)
{
  	return seq_open (file, &dm642_seq_ops);
};

static struct file_operations dm642_proc_ops = {
  	.owner = THIS_MODULE,
  	.open = dm642_open,
  	.read = seq_read,
  	.llseek = seq_lseek,
  	.release = seq_release
};

int
dm642_create_proc (void)
{
  	//struct proc_dir_entry *entry;
	//entry = create_proc_entry (DRV_NAME, 0, NULL);
  	proc_create (DRV_NAME, 0, NULL, &dm642_proc_ops);
  	
	//if (entry)
    //		entry->proc_fops = &dm642_proc_ops;
  	return 0;
}

void
dm642_remove_proc (void)
{
  	remove_proc_entry (DRV_NAME, NULL);
}

