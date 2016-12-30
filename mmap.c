//#include <linux/config.h>
#include <linux/module.h>

#include <linux/fs.h>  		/* Maybe ? Fix: mmap.c:33:32: error: dereferencing pointer to incomplete type ‘struct file’   */
#include <linux/mm.h>		/* everything */
#include <linux/errno.h>	/* error codes */
#include <asm/pgtable.h>

#include "dm642_pci.h"		/* local definitions */

extern int task_iopackets; 
extern int iopacket_size;
void dm642_vma_open(struct vm_area_struct *vma)
{
	struct dm642_task *task = vma->vm_private_data;

	task->vmas++;
}

void dm642_vma_close(struct vm_area_struct *vma)
{
	struct dm642_task *task = vma->vm_private_data;

	task->vmas--;
}

struct vm_operations_struct dm642_vm_ops = {
	.open =     dm642_vma_open,
	.close =    dm642_vma_close,
};

int task_mmap(struct file *filp, struct vm_area_struct *vma)
{
	// struct dm642_task *task = filp->private_data; /* mmap.c:33:32: error: dereferencing pointer to incomplete type ‘struct file’ */
	struct dm642_task *task = filp->private_data;
	
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long pfn, pos;
	int index = offset / iopacket_size;

	if (size > iopacket_size)
		return -EINVAL;
	if (index > task_iopackets*2)
		return -EINVAL;
	if(index < task_iopackets) //read buf
	{
		struct iopacket *p;
		if(task->read_ary == NULL)
			return -ENOMEM;
		p = task->read_ary[index];
		assert(p);
		if(p == NULL)
			return -ENOMEM;
		pos = (unsigned long)p->data;
	}else{
		struct iopacket *p;
		if(task->write_ary == NULL)
			return -ENOMEM;
		index -= task_iopackets;
		p = task->write_ary[index];
		assert(p);
		if(p == NULL)
			return -ENOMEM;
		pos = (unsigned long)p->data;
	}
#undef virt_to_pfn
#define virt_to_pfn(kaddr)	(__pa(kaddr) >> PAGE_SHIFT)
	pfn = virt_to_pfn((void *)pos);
	if (remap_pfn_range(vma, start, pfn, size, PAGE_SHARED)) 
		return -EAGAIN;
	vma->vm_ops = &dm642_vm_ops;
	vma->vm_flags &= ~VM_IO;	/* not I/O memory */
	//vma->vm_flags |= VM_RESERVED;	/* avoid to swap out this VMA */  /* mmap.c:72:19: error: ‘VM_RESERVED’ undeclared (first use in this function) */
	vma->vm_flags |= (VM_IO | VM_LOCKED | (VM_DONTEXPAND | VM_DONTDUMP));
	vma->vm_private_data = (void *) task;
	dm642_vma_open(vma);
	
	return 0;
}

