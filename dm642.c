//#include <linux/config.h>

#include <linux/fs.h>		/* Maybe ? Fix: dm642.c:132:5: error: implicit declaration of function ‘up’  */
#include <linux/sched.h> 	/* Maybe ? Fix: wait.h:429:9: error: ‘TASK_INTERRUPTIBLE’ undeclared (first use in this function  */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/atomic.h>

#include "dm642_pci.h"

extern int pci_dsp_handshake_addr;
extern int g_handshake_page_offset;
extern int g_handshake_page_index;

int update_pci_dsp_handshake_addr(u32 addr)
{
	pci_dsp_handshake_addr = addr;
 	g_handshake_page_offset = pci_dsp_handshake_addr & (DSPP_PAGE_SIZE - 1);
        g_handshake_page_index = (pci_dsp_handshake_addr & DSPP_PAGE_MASK)>>DSPP_PAGE_SHIFT;
	return 0;
}

int dm642_reset_dsp (struct dm642_dev *dev)
{
	u32 boot;
	if (!(boot = inl(dev->io_base+HDCR) & PCIBOOT))
	{
		printk(KERN_ERR "Can not boot from PCI port, something maybe wrong!\n");
		return 1;
	}
	outl(boot|WARMRESET,dev->io_base+HDCR);
	mdelay(20);
	
	return 0;
}

void dm642_unreset_dsp(struct dm642_dev *dev)
{
	TRACE();
	outl(inl(dev->io_base+HDCR)|DSPINT,dev->io_base+HDCR);
}

void host_to_dsp_int(struct dm642_dev *dev)
{
	outl(inl(dev->io_base+HDCR)|DSPINT,dev->io_base+HDCR);
}

void dsp_to_host_int(struct dm642_dev *dev)
{
	iowrite32(0x08,dev->mmr_offset1+RSTSRC);
}

void host_enable_int(struct dm642_dev *dev)
{
	u32 regVal;

        regVal = inl(dev->io_base+HSR);
	regVal = ((regVal & ~INTAM) & 0x1f);
        outl(regVal,dev->io_base+HSR);
}

void host_disable_int(struct dm642_dev *dev)
{
	u32 regVal;
	
	regVal = inl(dev->io_base+HSR);
	regVal = ((regVal | INTAM) & 0x1f);			
	outl(regVal,dev->io_base+HSR);
}

static inline void host_clear_int(struct dm642_dev *dev)
{
	u32 regVal;
	
	regVal = inl(dev->io_base+HSR);
	regVal = ((regVal | INTSRC) & 0x1f);			
	outl(regVal,dev->io_base+HSR);
	
}

static inline int check_pci_int(struct dm642_dev *dev)
{
	u32 regVal;

	regVal = inl(dev->io_base+HSR);
	if ((regVal & INTAVAL) == INTAVAL) {
        	return 1;
	}
	return 0;
}

void dm642_dma_write(struct dm642_dev *dev,u32 pci_bus_addr, u32 dsp_bus_addr,u16 count)
{
	u32 pcimc;
	iowrite32(dsp_bus_addr,dev->mmr_offset1+DSPMA);
	iowrite32(pci_bus_addr,dev->mmr_offset1+PCIMA);
 	pcimc = (count << CNT_SHIFT)|START_READP;
	wmb();
	iowrite32(pcimc,dev->mmr_offset1+PCIMC);
}

void dm642_dma_read(struct dm642_dev *dev,u32 pci_bus_addr, u32 dsp_bus_addr, u16 count)
{
	u32 pcimc;
	iowrite32(dsp_bus_addr,dev->mmr_offset1+DSPMA);
        iowrite32(pci_bus_addr,dev->mmr_offset1+PCIMA);
	pcimc = (count << CNT_SHIFT)|START_WRITE;
	wmb();
	iowrite32(pcimc,dev->mmr_offset1+PCIMC);
}

#define SPIN_COUNT 10
int request_io(struct dm642_task *task, struct iopacket *p)
{
	struct dm642_dev *dev = task->dev;
	u32 regVal;
	u32 spin_count = 0;
	wait_queue_head_t wait;
	
	init_waitqueue_head (&wait);
	if( down_interruptible(&dev->hw_access_sem) )
		return -ERESTARTSYS;	
	do{
		regVal = ioread32(dev->dspmem_base+g_handshake_page_offset);
		rmb();
		if(spin_count++ == SPIN_COUNT)
		{	dev->spins++;
			spin_count = 0;
			if(wait_event_interruptible_timeout(wait, 0, HZ/50))
			{
					
				up(&dev->hw_access_sem);
                        	return -ERESTARTSYS;
			}
		}
	}while(regVal != ACK_MAGIC);
	regVal = (p->state << 28) | (task->id << 16) | p->rlen; 
	/* FIXME the pci_dsp_handshare_addr should not cross 4M boundary */
        outl(g_handshake_page_index,dev->io_base+DSPP);
	iowrite32(regVal, dev->dspmem_base + g_handshake_page_offset+4);
	iowrite32(p->seq, dev->dspmem_base + g_handshake_page_offset+8);
	TRACE("type: %d ,__pa(p->data): %lx, seq: %d",
		p->state, __pa(p->data), p->seq);
	iowrite32(virt_to_bus(p->data), dev->dspmem_base+g_handshake_page_offset+12);
	iowrite32(REQUEST_MAGIC, dev->dspmem_base+g_handshake_page_offset);
	wmb();
	p->r_cycles = get_cycles(); 
	do_gettimeofday(&p->r_tv); 
	host_to_dsp_int(dev);
	up(&dev->hw_access_sem);
	return 0;
}

int get_complete_stuff(struct dm642_dev *dev, struct pci_dsp_handshake *hs)
{
	u32 *dsp_ack = (u32 *)hs;
	u32 regVal;
	u32 dspp_index_old;

	regVal = inl(dev->io_base+HSR);
	/* check the int */
	if ((regVal & INTAVAL) == INTAVAL) 
	{
		/* clear the int */
		regVal = ((regVal | INTSRC) & 0x1f);			
		outl(regVal,dev->io_base+HSR);
		/* read the dsp ack stuff */
		dspp_index_old = inl(dev->io_base+DSPP);//save the dspp index
        	outl(g_handshake_page_index,dev->io_base+DSPP);
		*dsp_ack++ = ioread32(dev->dspmem_base 
					+ g_handshake_page_offset+16); 
		*dsp_ack++ = ioread32(dev->dspmem_base 
					+ g_handshake_page_offset+20); 
		*dsp_ack++ = ioread32(dev->dspmem_base 
					+ g_handshake_page_offset+24); 
		*dsp_ack++ = ioread32(dev->dspmem_base 
					+ g_handshake_page_offset+28);
		/* ack we have read */
		iowrite32(ACK_MAGIC, dev->dspmem_base + 
					g_handshake_page_offset+16); 
        	outl(dspp_index_old,dev->io_base+DSPP);//restore the dspp index
		return 0;
	}
        return 1;
}

