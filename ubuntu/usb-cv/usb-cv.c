/******************************************************************************
 *
 *  Copyright 2017
 *  Broadcom Limited
 *  1 Yishun Avenue 7, Singapore 768923, Singapore
 *
 *****************************************************************************/

#include <linux/string.h>
#include <linux/mm.h>
#include "usb-cv.h"
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/uaccess.h>


volatile int int_urb_status = -1;
volatile int tx_bulk_in_sig = 0;
volatile int rx_bulk_in_sig = 0;
volatile int ush_is_in_sbl = 0;
volatile int reply_received_ush_is_in_sbl = 0;

int packet_for_sbi_in_sbl = 0;

struct write_buffer  {    
	unsigned int	cmd_length;     //Use int for both 32 and 64 bit kernel
	unsigned char	cmd_buffer[1];
};

typedef unsigned int cv_transport_type;
typedef unsigned short cv_command_id;

typedef struct td_cv_host_control_get_request {
	cv_transport_type		transportType;	/* CV_TRANS_TYPE_HOST_CONTROL */
	uint32_t				transportLen;	/* length of entire transport block in bytes */
	cv_command_id			commandID;		/* CV_HOST_CONTROL_GET_REQUEST */
} /*__attribute__((__packed__))*/ cv_host_control_get_request;

/* table of devices that work with this driver */
static struct usb_device_id cv_dev_table [] = {
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_0000) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5800) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5801) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5802) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5804) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5805) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5821) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5822) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5823) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5824) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5825) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5826) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5830) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5831) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5832) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5833) },
	{ USB_DEVICE(USB_CV_VENDOR_ID, USB_CV_PID_5834) },
	{ }					/* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, cv_dev_table);

static DEFINE_MUTEX(cv_dev_usb_mutex);

static struct usb_driver cv_dev_driver;


void* KMalloc(size_t size, int flags)
{
	void *pMem;
	pMem = kmalloc(size, flags);
	DBG_PRINT(INFO_DBG, "kmalloc: 0x%x (0x%x) bytes at: %p\n", (int)size, flags, pMem);
	return pMem;
}


void KFree(void* pMem)
{
	DBG_PRINT(INFO_DBG, "kfree: %p\n", pMem);
	kfree(pMem);
}



int txCmdOk(struct usb_cv *dev)
{
	unsigned long start;
	unsigned long cur;
	unsigned long deltaTime;
	int txstatus;

	DBG_PRINT(INFO_DBG, "TX cmd in progress\n");
	start = jiffies;
	while (!tx_bulk_in_sig) {
		txstatus = (dev->bulk_out_urb)->status;		

		if(!packet_for_sbi_in_sbl) msleep(2);

		cur = jiffies;
		deltaTime = cur - start;

		if (deltaTime > MAX_WAIT_TX_TIME*HZ) {
			DBG_PRINT(ERR_DBG, "TX timeout, waited %d seconds\n", (unsigned int)(deltaTime/HZ));
			return 0;
		}
	}

	// check if status error
	txstatus = dev->tx_status;
	if (txstatus) {
		DBG_PRINT(ERR_DBG, "tx error 0x%x\n", txstatus);
		return 0;
	}

	DBG_PRINT(INFO_DBG, "TX cmd done, urb->status: 0x%x\n", txstatus);

	// tx ok
	return 1;
}


void waitForResponse(struct usb_cv *dev)
{
	unsigned long start;
	unsigned long cur;
	unsigned long deltaTime;

	DBG_PRINT(INFO_DBG, "Waiting for RX response in progress\n");
	start = jiffies;
	while (!rx_bulk_in_sig) {		

		if(!packet_for_sbi_in_sbl) msleep(10);

		cur = jiffies;
		deltaTime = cur - start;

		if (deltaTime > MAX_WAIT_RX_TIME*HZ) {
			DBG_PRINT(ERR_DBG, "RX timeout, waited %d seconds\n", (unsigned int)(deltaTime/HZ));
			break;
		}
	}
}


void displayDump(int outLen, unsigned char *buf)
{
	char tmp[4] = "";
	char tmpl[4] = "";
	int i = 0, j = 0;
	int k = 0;
	char content[DISPLEN*3+1], display[DISPLEN+1];
	char contentl[DISPLEN*3+1];//, displayl[DISPLEN+1];

	if((NULL==buf)||(0==outLen)) {
		DBG_PRINT(ERR_DBG, "Wrong input to the functions check at %d, in %s\n", __LINE__-2, __FILE__);
	}

	content[0] = display[0] = display[DISPLEN] = '\0';
	contentl[0] = '\0';

	// check length
	if (outLen>0x100) {
		DBG_PRINT(INFO_DBG,"Buffer Length: 0x%x, only printing 0x100\n", outLen);
		outLen=0x100;
	}

	do{
		sprintf(tmp, "%02x ", buf[i]);
		sprintf(tmpl, "%02x ", buf[i]);
		strcat(content, tmp);

		/* printk adds new line, don't print now */
		/* DBG_PRINT(ERR_DBG,tmp); */
		display[i%DISPLEN] = '.';

		if((buf[i]>31)&&(buf[i]<127))
			display[i%DISPLEN] = buf[i];

		i++;
		k++;

		if(k%8 == 0) {
			/* Print this way due to new line added by printk */
			DBG_PRINT(INFO_DBG,"%s\t\t%s",content,display);
			/*
			DBG_PRINT(INFO_DBG,"%s",display);
			DBG_PRINT(INFO_DBG,"\n");
			*/
		}

		if (0 == (i%DISPLEN)) {
			display[DISPLEN] = '\0';
			content[0] = display[0] = '\0';
			continue;
		}

		if (i == outLen) {
			display[i%DISPLEN] = '\0';
			for(j = (i%DISPLEN); j < DISPLEN; j++) {
				/* Each byte takes 3 spaces */
				strcat(contentl,  "   ");
			}

			/* Print this way due to new line added by printk */
			DBG_PRINT(INFO_DBG,"%s%s\t\t%s",content, contentl, display);
			/*
			DBG_PRINT(ERR_DBG,"\t%s",contentl);
			DBG_PRINT(ERR_DBG,"\t%s\n",display);
			*/
		}

	} while(i<outLen);

	DBG_PRINT(INFO_DBG,"\n");
}


static void cv_dev_delete(struct kref *kref)
{
	struct usb_cv *dev = to_cv_dev(kref);

	/* free memory allocated for interrupt buffer */

	if(dev->udev != NULL)
		usb_put_dev(dev->udev);

	if(dev != NULL)
		KFree (dev);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
static void cv_dev_read_interrupt_callback (struct urb *urb, struct pt_regs *regs)
#else
static void cv_dev_read_interrupt_callback (struct urb *urb)
#endif

{
	struct usb_cv *dev;
	dev = (struct usb_cv *)urb->context;
	int_urb_status = urb->status;

	DBG_PRINT(INFO_DBG, "In cv_dev_read_interrupt_callback: %d, actual_length: 0x%x\n", urb->status, urb->actual_length);

	switch (urb->status) {
		case 0:                 /* success */
			break;
		case -ECONNRESET:       /* unlink */
		case -ENOENT:
		case -ESHUTDOWN:
		//case -EPROTO:           /* This happens before disconnecting (USH rebooting), still submitting here */ 
			return;
		/* -EPIPE:  should clear the halt */
		default:                /* error */
			DBG_PRINT(ERR_DBG, "resubmitting the urb in interrupt_callback :%d\n",
						urb->status);
	}

	int_urb_status = usb_submit_urb (urb, GFP_ATOMIC);
	if (int_urb_status) 
		DBG_PRINT(ERR_DBG,"can't resubmit urb in read interrupt callback\n");

	return;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
static void cv_dev_read_bulk_callback (struct urb *urb, struct pt_regs *regs)
#else
static void cv_dev_read_bulk_callback (struct urb *urb)
#endif
{
	struct usb_cv *dev = urb->context;
	int status, len;
	int submit_urb = 0;

	DBG_PRINT(INFO_DBG, "CV, In cv_dev_read_bulk_callback()\n");

	if(ush_is_in_sbl && reply_received_ush_is_in_sbl)
	{
		DBG_PRINT(INFO_DBG, "Reply in SBL has been received, don't process in cv_dev_read_bulk_callback\n");
		return;
	}
	
	status = urb->status;

	if(status == -EPROTO)
	{
		/* This happens before disconnecting (USH rebooting), don't destroy the data received previously */
		DBG_PRINT(INFO_DBG, "EPROTO, don't process in cv_dev_read_bulk_callback\n");
		return;
	}

	dev->actual_length = urb->actual_length;
	dev->rx_status = urb->status;

	DBG_PRINT(INFO_DBG, "In cv_dev_read_bulk_callback: %d, actual_length: 0x%x\n", status, urb->actual_length);
	len = urb->actual_length;

	switch (status) {
		case 0:                 /* success */
			break;
		case -ECONNRESET:       /* unlink */
		case -ENOENT:
		case -ESHUTDOWN:
			memset(dev->bulk_in_buffer, '\0', IN_BUFFER_SIZE);
			rx_bulk_in_sig = 1;
			return;
		/* -EPIPE:  should clear the halt */
		default:                /* error */
			memset(dev->bulk_in_buffer, '\0', IN_BUFFER_SIZE);
			DBG_PRINT(ERR_DBG, "resubmitting the urb in read bulk callback :%d\n", status);
	}

	if(ush_is_in_sbl)
	{
		// Don't submit if reply has been received when in SBL mode
		if(!reply_received_ush_is_in_sbl)
		{
			submit_urb = 1;			
			reply_received_ush_is_in_sbl = 1;
		}
	}
	else
	{
		submit_urb = 1;	
	}
    
	if(submit_urb)
	{
		status = usb_submit_urb(urb, GFP_ATOMIC);
		if (status)
			DBG_PRINT(ERR_DBG,"can't resubmit read urb \n");
	}

	rx_bulk_in_sig = 1;
}


static int cv_dev_open(struct inode *inode, struct file *file)
{
	struct usb_cv *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;
	int_urb_status = -1;

	DBG_PRINT(INFO_DBG,"CV, In cv_dev_open()\n");

	mutex_lock(&cv_dev_usb_mutex);
	
	subminor = iminor(inode);

	interface = usb_find_interface(&cv_dev_driver, subminor);

	if (!interface) {
		DBG_PRINT(ERR_DBG,"%s - error, can't find device for minor %d\n", __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);

	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	spin_lock(&dev->cv_lock);
	
	/* increment our usage count for the device */
	kref_get(&dev->kref);

	spin_unlock(&dev->cv_lock);

	/* save our object in the file's private structure */
	file->private_data = dev;

exit:
	mutex_unlock(&cv_dev_usb_mutex);
	return retval;
}


static int cv_dev_release(struct inode *inode, struct file *file)
{
	struct usb_cv *dev;

	DBG_PRINT(INFO_DBG, "CV, In cv_dev_release()\n");

	dev = (struct usb_cv *)file->private_data;

	if (dev == NULL) {
		return -ENODEV;
	}

	spin_lock(&dev->cv_lock);

	/* decrement the count on our device */
	kref_put(&dev->kref, cv_dev_delete);

	spin_unlock(&dev->cv_lock);

	return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
static void cv_dev_write_bulk_callback(struct urb *urb, struct pt_regs *regs)
#else
static void cv_dev_write_bulk_callback(struct urb *urb)
#endif
{
	struct usb_cv *dev = urb->context;
	int status;

	DBG_PRINT(INFO_DBG, "CV, In cv_dev_write_bulk_callback()\n");

	int_urb_status = -1;

	status = urb->status;

	DBG_PRINT(INFO_DBG, "In cv_dev_write_bulk_callback, status: %d, actual_length: %d\n", status, urb->actual_length);

	/* sync/async unlink faults aren't errors */
	 if (status) {
		if (status == -ENOENT ||
			status == -ECONNRESET ||
			status == -ESHUTDOWN) {
			goto exit;
		} else {
			DBG_PRINT(ERR_DBG, "nonzero status received: %d\n", status);
		}
	}

exit:

	dev->tx_status = status;

	if(dev->bulk_out_buffer != NULL) {
		KFree(dev->bulk_out_buffer);
		dev->bulk_out_buffer=NULL;
	}

	if(dev->bulk_out_urb != NULL) 
		usb_free_urb(dev->bulk_out_urb);

	// signal that the tx has completed
	tx_bulk_in_sig = 1;
}


static void cv_dev_submit_cmd_tasklet(unsigned long data)
{
	struct usb_cv *dev;	
	int retval;	 

	DBG_PRINT(INFO_DBG, "CV, In cv_dev_submit_cmd_tasklet()\n");

	dev = (struct usb_cv *)data;	

	spin_lock(&dev->cv_lock); 	

	/* create a urb, and a buffer for it, and copy the data to the urb */
	dev->bulk_out_urb = usb_alloc_urb(0, GFP_ATOMIC);

	if (!dev->bulk_out_urb) {
		retval = -ENOMEM;
		spin_unlock(&dev->cv_lock);
		return;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(dev->bulk_out_urb,
					dev->udev,
					usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
					dev->bulk_out_buffer,
					dev->write_count,
					cv_dev_write_bulk_callback,
					dev);

	/* send the data out the bulk port */
	retval = usb_submit_urb(dev->bulk_out_urb, GFP_ATOMIC);
	if (retval) {
		DBG_PRINT(ERR_DBG,"%s - failed submitting write urb, error %d\n", __FUNCTION__, retval);
	}

	spin_unlock(&dev->cv_lock);
}


static ssize_t  cv_dev_submit_cmd(struct file *file, struct usb_cv *dev, const char __user  *user_buffer)
{
	int retval=0;
	int  count;
	void __user *argp = (void __user *)user_buffer;
	struct write_buffer *wbuffer = NULL;
	struct write_buffer *rbuffer = (struct write_buffer *)argp;	
	int curr_buf_size;
	unsigned char *curr_buf_ptr;
	bool last_packet_for_sbi_in_sbl = 0;

	DBG_PRINT(INFO_DBG, "CV, In cv_dev_submit_cmd()\n");

	spin_lock(&dev->cv_lock);

	// Get length at beginning of the user_buffer
	if (copy_from_user(&count, argp, sizeof(int))) {
		DBG_PRINT(ERR_DBG,"Copy length failed in %s\n",__FUNCTION__);
		retval = -EFAULT;
		goto error;
	}

	//FW Upgrade packets for SBI in SBL
	if(count & FW_UPGRADE_IN_SBL_MASK) 
	{
		//Clear the flag
		count &= ~FW_UPGRADE_IN_SBL_MASK;
		packet_for_sbi_in_sbl = 1;
	}
	else
	{
		packet_for_sbi_in_sbl = 0;
	}

	if(count & FW_UPGRADE_IN_SBL_LAST_PKT_MASK) 
	{
		DBG_PRINT(INFO_DBG,"Last pkt received in SBL\n");

		//Clear the flag
		count &= ~FW_UPGRADE_IN_SBL_LAST_PKT_MASK;
		last_packet_for_sbi_in_sbl = 1;
	}
	else
	{
		last_packet_for_sbi_in_sbl = 0;
	}

	DBG_PRINT(INFO_DBG,"Received write request of %d bytes\n",(int)count);

	if (count > OUT_BUFFER_SIZE_LIMIT) 
	{
		DBG_PRINT(ERR_DBG,"Write request length %d too long in %s\n", count, __FUNCTION__);
		retval = -EFAULT;
		goto error;
	}

	//wbuffer = (struct write_buffer *)KMalloc(sizeof(struct write_buffer) + OUT_BUFFER_SIZE - sizeof(unsigned char),GFP_ATOMIC);
	wbuffer = (struct write_buffer *)KMalloc(sizeof(struct write_buffer) + count - sizeof(unsigned char),GFP_ATOMIC);
	if(wbuffer == NULL) {
		DBG_PRINT(ERR_DBG,"Failed to KMalloc memory in %s\n",__FUNCTION__);
		retval = -ENOMEM;
		goto error;
	}

	if (copy_from_user(wbuffer, argp, sizeof(struct write_buffer) + count - sizeof(unsigned char))) {
		DBG_PRINT(ERR_DBG,"Copy failed in %s\n",__FUNCTION__);
		retval = -EFAULT;
		KFree(wbuffer);
		goto error;
	}

	DBG_PRINT(INFO_DBG,"In %s\n",__FUNCTION__);

	
	if(packet_for_sbi_in_sbl)
	{
		wbuffer->cmd_length &= ~FW_UPGRADE_IN_SBL_MASK;
	}

	if(last_packet_for_sbi_in_sbl)
	{
		wbuffer->cmd_length &= ~FW_UPGRADE_IN_SBL_LAST_PKT_MASK;
	}

	curr_buf_size = wbuffer->cmd_length;
	curr_buf_ptr = wbuffer->cmd_buffer;

	/* Increase alloc size to avoid errors when size is too small here */
	dev->bulk_out_buffer = (char *)KMalloc(curr_buf_size < 512 ? 512 : curr_buf_size,GFP_ATOMIC);     
	dev->write_count = curr_buf_size;

	if (!dev->bulk_out_buffer) {
		DBG_PRINT(ERR_DBG,"Failed to KMalloc memory in %s\n",__FUNCTION__);
		retval = -ENOMEM;
		KFree(wbuffer);
		goto error;
	}

	/* verify that we actually have some data to write */
	if (curr_buf_size == 0) {
		DBG_PRINT(ERR_DBG,"The Length of data to be written is zero\n");
		KFree(wbuffer);
		goto error;
	}

	DBG_PRINT(INFO_DBG,"Current write request of %d bytes\n", curr_buf_size);

	memcpy(dev->bulk_out_buffer, curr_buf_ptr, curr_buf_size);

	DBG_PRINT(INFO_DBG, "Conforming write of \n");
	displayDump(curr_buf_size, dev->bulk_out_buffer);

	if(file->f_flags & O_NONBLOCK) {

		spin_unlock(&dev->cv_lock);
		tasklet_init(&dev->cv_dev_tl, cv_dev_submit_cmd_tasklet, (unsigned long)dev);
		tasklet_schedule(&dev->cv_dev_tl); 
		tasklet_kill(&dev->cv_dev_tl); 

		msleep(200);

		retval = dev->actual_length;
		if (copy_to_user((void __user *)rbuffer->cmd_buffer, dev->bulk_in_buffer,dev->actual_length)) {
			DBG_PRINT(ERR_DBG,"Copy to user failed in %s\n", __FUNCTION__);
			retval = -EFAULT;
		}

		DBG_PRINT(INFO_DBG, "Date read from device \n");
		displayDump(dev->actual_length, dev->bulk_in_buffer);
		goto error;

	} else {
	
		/* create a urb, and a buffer for it, and copy the data to the urb */
		dev->bulk_out_urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!dev->bulk_out_urb) {
			DBG_PRINT(ERR_DBG,"Failed to allocate urb in %s\n", __FUNCTION__);
			retval = -ENOMEM;
			goto error;
		}

		/* initialize the urb properly */
		usb_fill_bulk_urb(dev->bulk_out_urb,
						dev->udev,
						usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
						dev->bulk_out_buffer,
						dev->write_count,
						cv_dev_write_bulk_callback,
						dev);

		// initialize bulk in parameters
		memset(dev->bulk_in_buffer, '\0', IN_BUFFER_SIZE);
		dev->actual_length = 0;
		tx_bulk_in_sig = 0;		// reset tx signal
		rx_bulk_in_sig = 0;		// reset rx signal

		/* send the data out the bulk port */
		retval = usb_submit_urb(dev->bulk_out_urb, GFP_ATOMIC);
		if (retval) {
			DBG_PRINT(ERR_DBG,"%s - failed submitting write urb, error %d\n", __FUNCTION__, retval);
			goto error;
		}

		DBG_PRINT(INFO_DBG, "URB Submitted OK, waiting for tx complete\n");
		if (!txCmdOk(dev)) {
			DBG_PRINT(ERR_DBG,"Tx failed\n");
			retval = -EFAULT;
			goto error;
		}

		if(packet_for_sbi_in_sbl && !last_packet_for_sbi_in_sbl)
		{
			DBG_PRINT(INFO_DBG,"SBI load in SBL, no response from USH expected\n");
			retval = 0;
			goto error;			
		}
	}

	if(wbuffer != NULL)
		KFree(wbuffer);

	if(!(file->f_flags & O_NONBLOCK))
	{
		DBG_PRINT(INFO_DBG, "Transmitted command, waiting for rx\n");
		waitForResponse(dev);
		if (dev->rx_status != 0) {
			DBG_PRINT(ERR_DBG,"rx error: 0x%x\n", dev->rx_status);
			retval = -EFAULT;
			goto error;
		}

		DBG_PRINT(INFO_DBG, "Completed the Bulk read request with %d bytes\n", dev->actual_length);
		if (dev->actual_length == 0) {
			DBG_PRINT(ERR_DBG,"failed to rx data\n");
			retval = -EFAULT;
			goto error;
		}

		if (copy_to_user((void __user *)rbuffer->cmd_buffer, dev->bulk_in_buffer, dev->actual_length))  {
			DBG_PRINT(ERR_DBG,"Copy to user failed in %s\n", __FUNCTION__);
			retval = -EFAULT;
			goto error;
		}

		DBG_PRINT(INFO_DBG, "Data read from device \n");
		displayDump(dev->actual_length, dev->bulk_in_buffer);
        
		retval = dev->actual_length;
	}

error:

	if (dev->bulk_out_buffer != NULL) {
		KFree(dev->bulk_out_buffer);
		dev->bulk_out_buffer=NULL;
	}

	spin_unlock(&dev->cv_lock);
	return retval;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int  cv_dev_ioctl(struct inode *inode, struct file *file, unsigned int code, unsigned long value) 
#else
/* Changed from static int to static long to get rid of compilation errors on Ubuntu 16.04 */
static long  cv_dev_ioctl(struct file *file, unsigned int code, unsigned long value) 
#endif
{
	struct usb_cv *dev;
	struct usb_interface *iface;
	struct usb_device *udev;
	int retval = 0;

	/*
	* extract the type and number bitfields, and don't decode
	* wrong cmds: return ENOTTY before verify_area()
	*/
    
	DBG_PRINT(INFO_DBG,"CV, In cv_dev_ioctl()\n");

	if(_IOC_TYPE(code) != CV_IOC_MAGIC) return -ENOTTY;
	if(_IOC_NR(code) > CV_IOC_MAXNR) return -ENOTTY; 

	dev = (struct usb_cv *)file->private_data;
	iface = dev->interface;
	udev = interface_to_usbdev (iface);

	switch(code) {

		case CV_SUBMIT_CMD:
			DBG_PRINT(INFO_DBG,"Received CV_SUBMIT_COMMAND\n");
			if(dev->bulk_in_buffer)
			{
				memset(dev->bulk_in_buffer, '\0', IN_BUFFER_SIZE);	
				retval = cv_dev_submit_cmd(file, dev, (void __user *)value);
			}
			else
			{
				DBG_PRINT(ERR_DBG,"dev->bulk_in_buffer is NULL\n");
				retval = -EFAULT;
			}
			break;

		case CV_GET_COMMAND_STATUS:
			DBG_PRINT(INFO_DBG,"Received CV_GET_COMMAND_STATUS\n");
			if(dev->bulk_in_buffer)
			{
				memset(dev->bulk_in_buffer, '\0', IN_BUFFER_SIZE);	
				retval = cv_dev_submit_cmd(file, dev, (void __user *)value);
			}
			else
			{
				DBG_PRINT(ERR_DBG,"dev->bulk_in_buffer is NULL\n");
				retval = -EFAULT;
			}
			break;

		default :
			DBG_PRINT(ERR_DBG,"Unknown IOCTL request received\n");
			break;
	}

	return retval;
}

	
static struct file_operations cv_dev_fops = {
	.owner =	THIS_MODULE,
	.open =		cv_dev_open,
	//.write =	cv_dev_write,
	.release =	cv_dev_release,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	.ioctl = 	cv_dev_ioctl,
#else
	/* compat_ioctl is for 32 bit user space applications. */
	/* .unlocked_ioctl = 	cv_dev_ioctl, */
	.compat_ioctl = 	cv_dev_ioctl,
#endif
};


/* 
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with devfs and the driver core
 */

static struct usb_class_driver cv_dev_class = {
	.name = DEVICE_NAME"%d",
	.fops = &cv_dev_fops,
	//.mode = S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
	.minor_base = USB_CV_MINOR_BASE,
};


static int cv_dev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_cv *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_device_descriptor descriptor;
	int i;
	int retval = -ENOMEM;
	static int probe_on = 0;
	int pipe;

	probe_on++;

	DBG_PRINT(INFO_DBG, "Interface Minor :%d\n",interface->minor);
	DBG_PRINT(INFO_DBG, "BRCM CV_USB Device Driver First Cut \n");
	/* Comment out to get built with Ubuntu 16.04 */
	/* DBG_PRINT(INFO_DBG, "Built is %s %s\n",__DATE__,__TIME__); */

	/* allocate memory for our device state and initialize it */
	dev = (struct usb_cv *)KMalloc(sizeof(struct usb_cv), GFP_ATOMIC);
	if (dev == NULL) {
		goto error;
	}

	memset(dev, 0x00, sizeof (*dev));
	kref_init(&dev->kref);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));  
	dev->interface = interface;
	descriptor = dev->udev->descriptor;

	/* Validate the VENDOR ID & PRODUCT ID */
	if ( (descriptor.idVendor != USB_CV_VENDOR_ID) || 
		((descriptor.idProduct != USB_CV_PID_0000) &&
		(descriptor.idProduct != USB_CV_PID_5800) &&
		(descriptor.idProduct != USB_CV_PID_5801) &&
		(descriptor.idProduct != USB_CV_PID_5802) &&
		(descriptor.idProduct != USB_CV_PID_5804) &&
		(descriptor.idProduct != USB_CV_PID_5805) &&
		(descriptor.idProduct != USB_CV_PID_5821) &&
		(descriptor.idProduct != USB_CV_PID_5822) &&
		(descriptor.idProduct != USB_CV_PID_5823) &&
		(descriptor.idProduct != USB_CV_PID_5824) &&
		(descriptor.idProduct != USB_CV_PID_5825) &&
		(descriptor.idProduct != USB_CV_PID_5826) &&
		(descriptor.idProduct != USB_CV_PID_5830) &&
		(descriptor.idProduct != USB_CV_PID_5831) &&
		(descriptor.idProduct != USB_CV_PID_5832) &&
		(descriptor.idProduct != USB_CV_PID_5833) &&
		(descriptor.idProduct != USB_CV_PID_5834)) )
	{
		/* This is not an error case, so returning ZERO */
		/* We need to free the memory allocated as this device is
			no supported by this driver */  
		KFree(dev);
		DBG_PRINT(INFO_DBG, "Probe on CV Driver for VENDOR:%X\tPRODUCT:%X\n",
				descriptor.idVendor,descriptor.idProduct);
		return 0;
	}

	if(descriptor.idProduct == USB_CV_PID_5830) 
	{
		ush_is_in_sbl = 1;
		reply_received_ush_is_in_sbl = 0;
	}
	else
	{
		ush_is_in_sbl = 0;
	}

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	iface_desc = interface->cur_altsetting;

	DBG_PRINT(ERR_DBG, " bLength = 0x%02x \n bDescriptorType = 0x%02x \n bcdUSB = 0x%02x \n bDeviceClass = 0x%02x \n bDeviceProtocol = 0x%02x \n bMaxPacketSize = 0x%02x \n idVendor = 0x%02x \n idProduct = 0x%02x \n bcdDevice = 0x%02x \n iManufacturer = 0x%02x \n iProduct = 0x%02x \n iSerialNumber = 0x%02x \n bNumConfigurations = 0x%02x \n", 
			descriptor.bLength, descriptor.bDescriptorType, descriptor.bcdUSB, descriptor.bDeviceClass,
			descriptor.bDeviceProtocol, descriptor.bMaxPacketSize0, descriptor.idVendor, descriptor.idProduct,
			descriptor.bcdDevice, descriptor.iManufacturer, descriptor.iProduct, descriptor.iSerialNumber,
			descriptor.bNumConfigurations);

	if( (descriptor.bDeviceClass) || (descriptor.bDeviceSubClass) || (descriptor.bDeviceProtocol)) {
		DBG_PRINT(ERR_DBG, "Non Zero values of Class/SubClass/Protocol. Not a Composite device \n");
	}

	DBG_PRINT(INFO_DBG, " bNumEndpoints = %d\n", iface_desc->desc.bNumEndpoints);

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in_endpointAddr &&
			(endpoint->bEndpointAddress & USB_DIR_IN) &&
			((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK))
		{
			/* we found a bulk in endpoint */
			DBG_PRINT(INFO_DBG, " found a bulk in endpoint %d\n", i);
			dev->bulk_in_size = endpoint->wMaxPacketSize;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
		}

		/* Changed USB_DIR_IN to USB_DIR_OUT */
		if (!dev->bulk_out_endpointAddr &&
			!(endpoint->bEndpointAddress & USB_DIR_OUT) &&
			((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK))
		{
			/* we found a bulk out endpoint */
			DBG_PRINT(INFO_DBG, " found a bulk out endpoint %d\n", i);
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_out_size = endpoint->wMaxPacketSize;	
		}

		if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) &&
			((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT))
		{
			DBG_PRINT(INFO_DBG, " found a interrupt endpoint %d\n", i);
			dev->interrupt_in_endpoint = endpoint;
			dev->interrupt_in_size = endpoint->wMaxPacketSize;
		}
	}

	if (probe_on%2 == 1) {

		if ( !(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
			DBG_PRINT(ERR_DBG,"Could not find both bulk-in and bulk-out endpoints\n");
			goto error;
		}

		if( !dev->interrupt_in_endpoint ) {
			DBG_PRINT(ERR_DBG,"Could not find interrupt_in_endpoint \n");
			goto error;
		}

		dev->interrupt_in_urb = usb_alloc_urb(0, GFP_ATOMIC);

		if (!dev->interrupt_in_urb) {
			retval = -ENOMEM;
			goto error;
		}

		dev->interrupt_in_buffer = (unsigned char *)KMalloc(dev->interrupt_in_size, GFP_ATOMIC);
		if (!dev->interrupt_in_buffer) {
			retval = -ENOMEM;
			usb_free_urb(dev->interrupt_in_urb);
			goto error;
		}

		memset(dev->interrupt_in_buffer, '\0',8);	

		pipe = usb_rcvintpipe(dev->udev, dev->interrupt_in_endpoint->bEndpointAddress);

		usb_fill_int_urb(dev->interrupt_in_urb, dev->udev,pipe,
		dev->interrupt_in_buffer, 8,
		cv_dev_read_interrupt_callback, dev,
		dev->interrupt_in_endpoint->bInterval);

		retval = usb_submit_urb(dev->interrupt_in_urb, GFP_ATOMIC);
		if (retval) {
			retval = -ENOMEM;
			DBG_PRINT(ERR_DBG,"%s - failed submitting interrupt urb, error %d\n", __FUNCTION__, retval);
			goto error;
		}

		dev->bulk_in_urb = usb_alloc_urb(0, GFP_ATOMIC);

		if (!dev->bulk_in_urb) {
			retval = -ENOMEM;
			DBG_PRINT(ERR_DBG,"%s - failed to alloc_urb %d\n", __FUNCTION__, retval);
			goto error;
		}

		dev->bulk_in_buffer = (unsigned char *)KMalloc(IN_BUFFER_SIZE, GFP_ATOMIC);

		if(!dev->bulk_in_buffer) {
			retval = -ENOMEM;
			DBG_PRINT(ERR_DBG,"%s - failed to allocate memory %d\n", __FUNCTION__, retval);
			usb_free_urb(dev->bulk_in_urb);
			goto error;
		}

		memset(dev->bulk_in_buffer, '\0', IN_BUFFER_SIZE);

		usb_fill_bulk_urb(dev->bulk_in_urb,
					dev->udev,
					usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
					dev->bulk_in_buffer,
					IN_BUFFER_SIZE,
					cv_dev_read_bulk_callback,
					dev);

		retval = usb_submit_urb(dev->bulk_in_urb, GFP_ATOMIC);
		if (retval) {
			DBG_PRINT(ERR_DBG,"%s - failed submitting bulk urb, error %d\n", __FUNCTION__, retval);
			goto error;
		}

		spin_lock_init(&dev->cv_lock);

	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &cv_dev_class);

	if (retval) {
		/* something prevented us from registering this driver */
		DBG_PRINT(ERR_DBG,"Not able to get a minor for this device.\n");
		usb_set_intfdata(interface, NULL);
		goto error;
	}
	/* let the user know what node this device is now attached to */
	DBG_PRINT(ERR_DBG,"USB Credential Vault device now attached to USBCv-%d\n", interface->minor);
	return 0;

error:
	if (dev)
		kref_put(&dev->kref, cv_dev_delete);
	return retval;
}


static void cv_dev_disconnect(struct usb_interface *interface)
{
	struct usb_cv *dev;
	int minor = interface->minor;

	if(interface != NULL) {
	
		/* prevent cv_dev_open() from racing cv_dev_disconnect() */
		mutex_lock(&cv_dev_usb_mutex);

		dev = usb_get_intfdata(interface);

		if(dev!=NULL) {

			if(dev->interrupt_in_urb != NULL)
				usb_kill_urb(dev->interrupt_in_urb);

			if(dev->interrupt_in_buffer != NULL)
				KFree(dev->interrupt_in_buffer);

			if(dev->interrupt_in_urb != NULL)
				usb_free_urb(dev->interrupt_in_urb);

			if(dev->bulk_in_urb != NULL) 
				usb_kill_urb(dev->bulk_in_urb);

			if(dev->bulk_in_buffer != NULL)
				KFree(dev->bulk_in_buffer);

			if(dev->bulk_in_urb != NULL)
				usb_free_urb(dev->bulk_in_urb);

		}

		usb_set_intfdata(interface, NULL);
		/* give back our minor */
		usb_deregister_dev(interface, &cv_dev_class);
		mutex_unlock(&cv_dev_usb_mutex);

		/* decrement our usage count */
		kref_put(&dev->kref, cv_dev_delete);
		DBG_PRINT(ERR_DBG,"USB Credential Vault #%d now disconnected\n", minor);
	}

}


static struct usb_driver cv_dev_driver = {
	.name = "Credential Vault",
	.id_table = cv_dev_table,
	.probe = cv_dev_probe,
	.disconnect = cv_dev_disconnect,
};


static int __init usb_cv_dev_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&cv_dev_driver);
	if (result)
		DBG_PRINT(ERR_DBG,"usb_register failed. Error number %d\n", result);

	return result;
}


static void __exit usb_cv_dev_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&cv_dev_driver);
}

module_init (usb_cv_dev_init);
module_exit (usb_cv_dev_exit);

module_param(interval, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(interval, "Overrides interrupt interval");
MODULE_LICENSE("GPL");
