/*
 *
 *  Realtek Bluetooth USB driver
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/errno.h>
#include <linux/usb.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>

#include <linux/version.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/suspend.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>


/***********************************
** Realtek - For rtk_btusb driver **
***********************************/
#define BTUSB_RPM		0* USB_RPM 	//	1 SS enable; 0 SS disable

//when os suspend, module is still powered,usb is not powered,
//this may set to 1 ,and must comply with special patch code
#define CONFIG_RESET_RESUME		0

#define URB_CANCELING_DELAY_MS	10  	 // Added by Realtek
#define PRINT_CMD_EVENT			0
#define PRINT_ACL_DATA			0

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 33)
#define HDEV_BUS		hdev->bus
#define USB_RPM			1
#else
#define HDEV_BUS		hdev->type
#define USB_RPM			0
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38)
#define NUM_REASSEMBLY 3
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
#define GET_DRV_DATA(x)		hci_get_drvdata(x)
#else
#define GET_DRV_DATA(x)		x->driver_data
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
#define SCO_NUM    hdev->conn_hash.sco_num
#else
#define SCO_NUM     hci_conn_num(hdev, SCO_LINK)
#endif

static int patch_add(struct usb_interface* intf);
static void patch_remove(struct usb_interface* intf);
static int download_patch(struct usb_interface* intf);
static int set_btoff(struct usb_interface* intf);
static void print_event(struct sk_buff *skb);
static void print_command(struct sk_buff *skb);
static void print_acl (struct sk_buff *skb,int dataOut);

#define BTUSB_MAX_ISOC_FRAMES	10
#define BTUSB_INTR_RUNNING		0
#define BTUSB_BULK_RUNNING		1
#define BTUSB_ISOC_RUNNING		2
#define BTUSB_SUSPENDING		3
#define BTUSB_DID_ISO_RESUME	4
/*******************************/
// Added by Realtek
#define BTUSB_NEXT_RX_URB_SUBMITTING		5
/*******************************/

struct btusb_data {
	struct hci_dev       *hdev;
	struct usb_device    *udev;
	struct usb_interface *intf;
	struct usb_interface *isoc;

	spinlock_t lock;

	unsigned long flags;

	struct work_struct work;
	struct work_struct waker;

	struct usb_anchor tx_anchor;
	struct usb_anchor intr_anchor;
	struct usb_anchor bulk_anchor;
	struct usb_anchor isoc_anchor;
	struct usb_anchor deferred;
	int tx_in_flight;
	spinlock_t txlock;

	struct usb_endpoint_descriptor *intr_ep;
	struct usb_endpoint_descriptor *bulk_tx_ep;
	struct usb_endpoint_descriptor *bulk_rx_ep;
	struct usb_endpoint_descriptor *isoc_tx_ep;
	struct usb_endpoint_descriptor *isoc_rx_ep;

	__u8 cmdreq_type;

	unsigned int sco_num;
	int isoc_altsetting;
	int suspend_count;
};


//----------
#define HCI_CMD_READ_BD_ADDR 0x1009
#define HCI_VENDOR_CHANGE_BDRATE 0xfc17
#define HCI_VENDOR_READ_RTK_ROM_VERISION 0xfc6d
#define HCI_VENDOR_READ_LMP_VERISION 0x1001

#define ROM_LMP_8723a               0x1200
#define ROM_LMP_8723b               0x8723
#define ROM_LMP_8821a               0X8821
#define ROM_LMP_8761a               0X8761

//signature: Realtech
const uint8_t  RTK_EPATCH_SIGNATURE[8]={0x52,0x65,0x61,0x6C,0x74,0x65,0x63,
0x68};
//Extension Section IGNATURE:0x77FD0451
const uint8_t Extension_Section_SIGNATURE[4]={0x51,0x04,0xFD,0x77};
uint16_t project_id[]=
{
	ROM_LMP_8723a,
	ROM_LMP_8723b,
	ROM_LMP_8821a,
	ROM_LMP_8761a
};
struct rtk_eversion_evt {
	uint8_t status;
	uint8_t version;
}__attribute__ ((packed));

struct rtk_epatch_entry{
	uint16_t chipID;
	uint16_t patch_length;
	uint32_t start_offset;
} __attribute__ ((packed));

struct rtk_epatch{
    	uint8_t signature[8];
	uint32_t fm_version;
	uint16_t number_of_total_patch;
	struct rtk_epatch_entry entry[0];
} __attribute__ ((packed));

struct rtk_extension_entry{
	uint8_t opcode;
	uint8_t length;
	uint8_t *data;
} __attribute__ ((packed));


