#ifndef DM642_PCI_H
#define DM642_PCI_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/time.h>
#include <linux/timex.h>
#define DM642_MAJOR		0	
#define DM642_DEVS		4
#define DM642_TASKS_PER_DEV 	255
#define TASK_IOPACKETS	 	3
#define IOPACKET_SIZE		32*1024
#define PCI_DSP_HANDSHAKE_ADDR	-1	//0 for get from dsp app 

#define DRV_NAME "dm642_pci"

#define DM642_DEBUG 
//#define DM642_TRACE
//#define DM642_NDEBUG

#ifdef DM642_DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#ifdef DM642_TRACE
#define TRACE(fmt,args...)	printk("%s.%d: "fmt"\n", __FUNCTION__, __LINE__, ##args)
#else
#define TRACE(fmt,args...)
#endif

#ifdef DM642_NDEBUG
#define assert(expr) do {} while (0)
#else
#define assert(expr) \
        if(unlikely(!(expr))) {                                 \
        printk(KERN_ERR "Assertion failed! %s,%s,%s,line=%d\n", \
        #expr,__FILE__,__FUNCTION__,__LINE__);                  \
        }
#endif

#define PCI_DEVICE_ID_TMS320DM642	0x9065
#define PCI_DEVICE_ID_TMS320C6205       0xA106     

struct dm642_dev {
	void *dspmem_base;	/* Base 0: 4M-byte prefetchable maps to all of DSP memory 
				   with the DSP Page register. Prefetch reads have all bytes valid. */
	void *mmr_base;		/* Base 1: 8M-byte non-prefetchable maps to DSP's memory-mapped registers.
				   Non-prefetch supports byte enables. */
	void *mmr_offset1;
	void *mmio_offset;
	u32 io_base;		/* Base 2: 16-byte I/O contains I/O registers for the PCI host */
	int index;
	struct list_head task_list;
	spinlock_t task_list_spinlock;
	
	//struct semaphore hw_access_sem;     /* mutual exclusion semaphore     */ /* dm642_pci.h:60:19: error: field ‘hw_access_sem’ has incomplete type */
	//struct semaphore task_access_sem; /* dm642_pci.h:61:19: error: field ‘task_access_sem’ has incomplete type */
	struct semaphore hw_access_sem;     /* mutual exclusion semaphore     */
	struct semaphore task_access_sem;
	char *printkBuf;
	u32 spins;	
	struct pci_dev *pci_dev;
	struct cdev cdev;
};

struct dm642_task {
	struct list_head list;
	u32 id;
	struct dm642_dev *dev;
        wait_queue_head_t readq, writeq;       /* read and write queues */
	u32 read_seq;
	u32 write_seq;
	atomic_t read_allocated;
	atomic_t write_allocated;
	struct list_head read_f_queue;	/* free */
	struct list_head write_f_queue;	 
	struct list_head read_r_queue;	/* request */
	struct list_head read_c_queue;	/* complete */
	struct list_head write_r_queue;
	spinlock_t read_queue_spinlock; 
	spinlock_t write_queue_spinlock;
	struct iopacket **read_ary;
	struct iopacket **write_ary; 
/* performance test */
	cycles_t r_cycles;
	cycles_t c_cycles;
	u32 access_count;
	u32 vmas;
};

enum PacketState{
	READ_REQUEST = 1,
	READ_COMPLETE = 2,
	WRITE_REQUEST = 3,
	WRITE_COMPLETE = 4,
	OPEN = 5,
	CLOSE = 6,
};

struct iopacket{
	struct list_head list;
	u32 index;
	u32 seq;
	u32 state; 		
	u32 rlen;
	u32 clen;
	u32 endblock;	
	void *data;
/* performance test */
	cycles_t r_cycles;
	cycles_t c_cycles;
	struct timeval r_tv;
	struct timeval c_tv;	
};

/*
the protocal to exchange info. to dsp 
32 31 30 29 28 .... 16 15... 0 
request/ack magic
type        task_id    len	     
seq/data  (type related)
address/? (type related)
*/
#define REQUEST_MAGIC 	0xBABEF00D
#define ACK_MAGIC	0xFEEDBEEF
struct pci_dsp_handshake {
	u32	request;
	u32	len:16,
		task_id:12,
		type:4;
	u32	seq;
	u32 	addr;    	
};
/* dsp hardware stuff */
/*
  PCI I/O Registers Accessed via I/O Space (Base 2 Memory)
*/
enum PCI_IORegisters {
	HSR = 0x00,
	HDCR = 0x04,
	DSPP = 0x08,
};

/* Host Status Register (HSR) */
enum HSRBits {
	INTSRC = 0x01,	
	INTAVAL = 0x02, /*HR (host read only*/
	INTAM = 0x04,
	CFGERR = 0x08, /*HR*/
	EEREAD = 0x10, /*HR*/
};

/* Host-to-DSP Control Register (HDCR) */
enum HDCRBits { 
	WARMRESET = 0x01,
	DSPINT = 0x02,
	PCIBOOT = 0x04,	/*HR*/
};

/* DSP Page Register (DSPP) */
enum DSPPBits {
	DSPP_PAGE_SHIFT = 22,
	DSPP_PAGE_SIZE = (1UL << DSPP_PAGE_SHIFT),
 	DSPP_PAGE_MASK = (~(DSPP_PAGE_SIZE-1)),
 	MAP = (1UL << 10), /*HR*/	
};

/*
PCI Memory-Mapped Peripheral Registers
The PCI base 1 register on the DSP is configured for an 8M-byte non-prefetchable
region. This memory is mapped into the DSP memory-mapped registers 0x01800000.
*/
#define MMR_BASE	0x01800000
#define MMR_SIZE	0x800000
#define C62X_MMR_OFFSET1	(0x01A40000 - MMR_BASE)
#define C62X_MMR_OFFSET2	(0x01A80000 - MMR_BASE)
#define C62X_MMIO_OFFSET	(0x01A7FFF0 - MMR_BASE)
#define C64X_MMR_OFFSET1	(0x01C00000 - MMR_BASE)
#define C64X_MMR_OFFSET2	(0x01C20000 - MMR_BASE)
#define C64X_MMIO_OFFSET	(0x01C1FFF0 - MMR_BASE)

enum PCI_MMPRegisters {
	/* OFFSET1 */
	RSTSRC	= 0x00, 	
	PMDCSR	= 0x04,
	PCIIS	= 0x08,
	PCIIEN	= 0x0c,
	DSPMA	= 0x10,
	PCIMA	= 0x14,
	PCIMC	= 0x18,
	CDSPA	= 0x1c,
	CPCIA	= 0x20,
	CCNT	= 0x24,
	HALT	= 0x28,
	/* OFFSET2 */
	RRADD	= 0x00,
	EEDAT	= 0x04,
	EECTL	= 0x08,
};
enum PCIMCBits {
	CNT_SHIFT = 16,
	FLUSH_CURRENT = 0,
	START_WRITE = 1,
	START_READP = 2,
	START_READNP = 3,
};

#else
typedef unsigned long u32;
#endif
struct emif {
	u32 GBLCTL;
	u32 CECTL1;
	u32 CECTL0;
	u32 Reserved;
	u32 CECTL2;
	u32 CECTL3;
	u32 SDCTL;
	u32 SDTIM;
	u32 SDEXT;
};

struct regval {
	u32 reg;
	u32 val;
};

#define DM642_IOC_MAGIC  'Y'
#define DM642_IOC_WARMRESET	_IO(DM642_IOC_MAGIC, 0)
#define DM642_IOC_RUN		_IO(DM642_IOC_MAGIC, 1)
/*
 * S means "Set" through a ptr,
 * T means "Tell" directly
 * G means "Get" (to a pointed var)
 * Q means "Query", response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */
#define DM642_IOCS_INITEMIF	_IOW(DM642_IOC_MAGIC, 2, struct emif)
#define DM642_IOCS_REGVAL   	_IOW(DM642_IOC_MAGIC, 3, struct regval)
#define DM642_IOCG_REGVAL   	_IOWR(DM642_IOC_MAGIC,4, struct regval)
#define DM642_IOCH_REGVAL	_IO(DM642_IOC_MAGIC,  5)

#define DM642_IOC_MAXNR 10


#define DM642_TASK_IOC_MAGIC  'T'
#define DM642_TASK_IOC_1	_IO(DM642_TASK_IOC_MAGIC, 0)
#define DM642_TASK_IOC_2	_IO(DM642_TASK_IOC_MAGIC, 1)

#define DM642_TASK_IOC_MAXNR 10
#endif

