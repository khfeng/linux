/******************************************************************************
 *
 *  Copyright 2017
 *  Broadcom Limited
 *  1 Yishun Avenue 7, Singapore 768923, Singapore
 *
 *****************************************************************************/
#ifndef _USB_CV_H_
#define _USB_CV_H_

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/usb.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/fs.h>


/* Define these values to match your devices */
#define USB_CV_VENDOR_ID		0x0a5c

#define USB_CV_PID_0000			0x0000

#define USB_CV_PID_5800         0x5800
#define USB_CV_PID_5801         0x5801
#define USB_CV_PID_5802         0x5802
#define USB_CV_PID_5804         0x5804
#define USB_CV_PID_5805         0x5805
#define USB_CV_PID_5821         0x5821
#define USB_CV_PID_5822         0x5822
#define USB_CV_PID_5823         0x5823
#define USB_CV_PID_5824         0x5824
#define USB_CV_PID_5825         0x5825
#define USB_CV_PID_5826         0x5826
#define USB_CV_PID_5830         0x5830
#define USB_CV_PID_5831         0x5831
#define USB_CV_PID_5832         0x5832
#define USB_CV_PID_5833         0x5833
#define USB_CV_PID_5834         0x5834
#endif

#define CMD_BUF_SIZE	16*sizeof(unsigned char) 

#define CV_IOC_MAGIC 	'k'

#define CV_GET_LAST_COMMAND_STATUS		_IOR(CV_IOC_MAGIC,1,int)

#define CV_GET_LATEST_COMMAND_STATUS	_IOR(CV_IOC_MAGIC,2,int)

#define CV_GET_COMMAND_STATUS			_IOR(CV_IOC_MAGIC,3,int)

#define CV_GET_CONFIG_DESCRIPTOR		_IOR(CV_IOC_MAGIC,4,int)

#define CV_SUBMIT_CMD					_IOW(CV_IOC_MAGIC,5,int)

#define CV_BURN_FW						_IOWR(CV_IOC_MAGIC,6,int)

#define CV_HOST_CTL						_IOWR(CV_IOC_MAGIC,7,int)


#define CV_IOC_MAXNR	7


#define GET_STATUS			0

#define GET_DESCRIPTOR		0x06

#define GET_CONFIGURATION	0x08


#define VENDOR_WRITE_REQUEST_TYPE	0x42   // 0100 0010

#define VENDOR_READ_REQUEST_TYPE	0xC2   // 1100 0010


#define ERR_DBG			0

#define INIT_DBG		1

#define INFO_DBG		2

#define TX_DBG			3

#define INTR_DBG		4

typedef unsigned short cv_command_id;

#define FW_UPGRADE_IN_SBL_MASK 0x80000000
#define FW_UPGRADE_IN_SBL_LAST_PKT_MASK 0x40000000

/* Get a minor range for your devices from the usb maintainer */
#define USB_CV_MINOR_BASE	192
#define BUFFER_SIZE			4096
#define IN_BUFFER_SIZE		99000

#define OUT_BUFFER_SIZE		4096
#define OUT_BUFFER_SIZE_LIMIT 524288 /* 512 KB */

#define DISPLEN				0x08


#define to_cv_dev(d) container_of(d, struct usb_cv, kref)

#define DEVICE_NAME "usb/cv"

#define MAX_WAIT_TX_TIME	10
#define MAX_WAIT_RX_TIME	10

#ifdef __KERNEL__
/* Structure to hold all of our device specific stuff */
struct usb_cv {
	spinlock_t						cv_lock;
	struct usb_device*				udev;                   /* the usb device for this device */
	struct usb_interface*			interface;              /* the interface for this device */

	size_t							bulk_in_size;           /* the size of the receive buffer */

	size_t							bulk_out_size;          /* the size of the maximum transmit buffer */

	size_t							interrupt_in_size;	/* the size of the interrupt buffer */

	__u8							bulk_in_endpointAddr;   /* the address of the bulk in endpoint */

	__u8							bulk_out_endpointAddr;  /* the address of the bulk out endpoint */



	struct usb_endpoint_descriptor*	interrupt_in_endpoint;


	struct urb*						interrupt_in_urb;

	struct urb*						bulk_in_urb;

	struct urb*						bulk_out_urb;

	struct urb*						descriptor_in_urb;



	unsigned char*					interrupt_in_buffer;

	unsigned char*					bulk_out_buffer;

	unsigned char*					bulk_in_buffer;

	unsigned char*					descriptor_in_buffer;

	unsigned char*					cmd_in_buffer;

	unsigned char*					out_buffer;

	unsigned char*					in_buffer;



	struct usb_ctrlrequest*			cr_desc;

	struct usb_ctrlrequest*			cr_submit_cmd;

	struct usb_ctrlrequest*			cr_get_cmd;



	int								probe_done;

	int								actual_length;	

	struct tasklet_struct			cv_dev_tl;

	int								write_count;

	struct kref						kref;

	int								tx_status;

	int								rx_status;

};
#endif

/* Global variable that defines the present debug level of the driver. */
static int debug_level = ERR_DBG;
static int interval = 1;
#define DBG_PRINT(dbg_level, args...)  if(!(debug_level<dbg_level)) printk(args)

#endif /*  _USB_CV_H_ */
