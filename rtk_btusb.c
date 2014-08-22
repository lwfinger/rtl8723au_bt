/*
 *
 *  Realtek Bluetooth USB driver
 *
 *  Copyright (C) 2012-2015  Edward Bian <edward_bian@realsil.com.cn>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/usb.h>
#include <linux/version.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#define VERSION "0.8"

static struct usb_driver btusb_driver;
#if 1
#define RTKBT_DBG(fmt, arg...) printk(KERN_INFO "rtk_btusb: " fmt "\n" , ## arg)
#else
#define RTKBT_DBG(fmt, arg...)
#endif


/*******************************/
#include <linux/version.h>
#include <linux/pm_runtime.h>
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

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 4, 0)
#define GET_DRV_DATA(x)		hci_get_drvdata(x)
#else
#define GET_DRV_DATA(x)		x->driver_data
#endif




#define BTUSB_RPM		0* USB_RPM	//	1 SS enable; 0 SS disable
#define LOAD_CONFIG		0
#define URB_CANCELING_DELAY_MS	10   // Added by Realtek

static int patch_add(struct usb_interface* intf);
static void patch_remove(struct usb_interface* intf);
static int download_patch(struct usb_interface* intf);
static int set_btoff(struct usb_interface* intf);


static struct usb_device_id btusb_table[] = {
	{ .match_flags = USB_DEVICE_ID_MATCH_VENDOR |
					 USB_DEVICE_ID_MATCH_INT_INFO,
	  .idVendor = 0x0bda,
	  .bInterfaceClass = 0xe0,
	  .bInterfaceSubClass = 0x01,
	  .bInterfaceProtocol = 0x01 },
	{ }
};
/*******************************/


MODULE_DEVICE_TABLE(usb, btusb_table);


#define BTUSB_MAX_ISOC_FRAMES	10
#define BTUSB_INTR_RUNNING		0
#define BTUSB_BULK_RUNNING		1
#define BTUSB_ISOC_RUNNING		2
#define BTUSB_SUSPENDING		3
#define BTUSB_DID_ISO_RESUME	4

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

static int inc_tx(struct btusb_data *data)
{
	unsigned long flags;
	int rv;

	spin_lock_irqsave(&data->txlock, flags);
	rv = test_bit(BTUSB_SUSPENDING, &data->flags);
	if (!rv)
		data->tx_in_flight++;
	spin_unlock_irqrestore(&data->txlock, flags);

	return rv;
}

static void btusb_intr_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;

		if (hci_recv_fragment(hdev, HCI_EVENT_PKT,
						urb->transfer_buffer,
						urb->actual_length) < 0) {
			BT_ERR("%s corrupted event packet", hdev->name);
			hdev->stat.err_rx++;
		}
	}

	if (!test_bit(BTUSB_INTR_RUNNING, &data->flags))
		return;

	usb_mark_last_busy(data->udev);
	usb_anchor_urb(urb, &data->intr_anchor);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("btusb_intr_complete %s urb %p failed to resubmit (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	//	dev = urb->dev;
	//	RTKBT_DBG("dev->state = %d ",dev->state);
	}
}

static int btusb_submit_intr_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	BT_DBG("%s", hdev->name);

	if (!data->intr_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(data->intr_ep->wMaxPacketSize);

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvintpipe(data->udev, data->intr_ep->bEndpointAddress);

	usb_fill_int_urb(urb, data->udev, pipe, buf, size,
						btusb_intr_complete, hdev,
						data->intr_ep->bInterval);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &data->intr_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		BT_ERR("btusb_submit_intr_urb %s urb %p submission failed (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_bulk_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;
	//struct usb_device    *dev ;
	BT_DBG("%s urb %p status %d count %d", hdev->name,
					urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;

		if (hci_recv_fragment(hdev, HCI_ACLDATA_PKT,
						urb->transfer_buffer,
						urb->actual_length) < 0) {
			BT_ERR("%s corrupted ACL packet", hdev->name);
			hdev->stat.err_rx++;
		}
	}

	if (!test_bit(BTUSB_BULK_RUNNING, &data->flags))
		return;

	usb_anchor_urb(urb, &data->bulk_anchor);
	usb_mark_last_busy(data->udev);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("btusb_bulk_complete %s urb %p failed to resubmit (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	//	dev = urb->dev;
	//	RTKBT_DBG("dev->state = %d ",dev->state);
	}
}

static int btusb_submit_bulk_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size = HCI_MAX_FRAME_SIZE;

	BT_DBG("%s", hdev->name);

	if (!data->bulk_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvbulkpipe(data->udev, data->bulk_rx_ep->bEndpointAddress);

	usb_fill_bulk_urb(urb, data->udev, pipe,
					buf, size, btusb_bulk_complete, hdev);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_mark_last_busy(data->udev);
	usb_anchor_urb(urb, &data->bulk_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		BT_ERR("btusb_submit_bulk_urb %s urb %p submission failed (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_isoc_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int i, err;

	BT_DBG("%s urb %p status %d count %d", hdev->name,
					urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		for (i = 0; i < urb->number_of_packets; i++) {
			unsigned int offset = urb->iso_frame_desc[i].offset;
			unsigned int length = urb->iso_frame_desc[i].actual_length;

			if (urb->iso_frame_desc[i].status)
				continue;

			hdev->stat.byte_rx += length;

			if (hci_recv_fragment(hdev, HCI_SCODATA_PKT,
						urb->transfer_buffer + offset,
								length) < 0) {
				BT_ERR("%s corrupted SCO packet", hdev->name);
				hdev->stat.err_rx++;
			}
		}
	}

	if (!test_bit(BTUSB_ISOC_RUNNING, &data->flags))
		return;

	usb_anchor_urb(urb, &data->isoc_anchor);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("btusb_isoc_complete %s urb %p failed to resubmit (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static inline void __fill_isoc_descriptor(struct urb *urb, int len, int mtu)
{
	int i, offset = 0;

	BT_DBG("len %d mtu %d", len, mtu);

	for (i = 0; i < BTUSB_MAX_ISOC_FRAMES && len >= mtu;
					i++, offset += mtu, len -= mtu) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = mtu;
	}

	if (len && i < BTUSB_MAX_ISOC_FRAMES) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = len;
		i++;
	}

	urb->number_of_packets = i;
}

static int btusb_submit_isoc_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	BT_DBG("%s", hdev->name);

	if (!data->isoc_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize) *
						BTUSB_MAX_ISOC_FRAMES;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvisocpipe(data->udev, data->isoc_rx_ep->bEndpointAddress);

	urb->dev      = data->udev;
	urb->pipe     = pipe;
	urb->context  = hdev;
	urb->complete = btusb_isoc_complete;
	urb->interval = data->isoc_rx_ep->bInterval;

	urb->transfer_flags  = URB_FREE_BUFFER | URB_ISO_ASAP;
	urb->transfer_buffer = buf;
	urb->transfer_buffer_length = size;

	__fill_isoc_descriptor(urb, size,
			le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize));

	usb_anchor_urb(urb, &data->isoc_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		BT_ERR("btusb_submit_isoc_urb %s urb %p submission failed (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void btusb_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *) skb->dev;
	struct btusb_data *data = GET_DRV_DATA(hdev);

//	RTKBT_DBG("btusb_tx_complete %s urb %p status %d count %d", hdev->name,
//					urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (!urb->status)
		hdev->stat.byte_tx += urb->transfer_buffer_length;
	else
		hdev->stat.err_tx++;

done:
	spin_lock(&data->txlock);
	data->tx_in_flight--;
	spin_unlock(&data->txlock);

	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static void btusb_isoc_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *) skb->dev;

	BT_DBG("btusb_isoc_tx_complete %s urb %p status %d count %d", hdev->name,
					urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (!urb->status)
		hdev->stat.byte_tx += urb->transfer_buffer_length;
	else
		hdev->stat.err_tx++;

done:
	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static int btusb_open(struct hci_dev *hdev)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int err;

	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		return err;

	data->intf->needs_remote_wakeup = 1;
	RTKBT_DBG("%s start pm_usage_cnt(0x%x)",__FUNCTION__,atomic_read(&(data->intf ->pm_usage_cnt)));

	/*******************************/
	if (0 == atomic_read(&hdev->promisc)) {
		RTKBT_DBG("btusb_open hdev->promisc ==0");
		err = -1;
                //goto failed;
	}
	err = download_patch(data->intf);
	if (err < 0) goto failed;
	/*******************************/

	if (test_and_set_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (test_and_set_bit(BTUSB_INTR_RUNNING, &data->flags))
		goto done;

	err = btusb_submit_intr_urb(hdev, GFP_KERNEL);
	if (err < 0)
		goto failed;

	err = btusb_submit_bulk_urb(hdev, GFP_KERNEL);
	if (err < 0) {
		mdelay(URB_CANCELING_DELAY_MS);      // Added by Realtek
		usb_kill_anchored_urbs(&data->intr_anchor);
		goto failed;
	}

	set_bit(BTUSB_BULK_RUNNING, &data->flags);
	btusb_submit_bulk_urb(hdev, GFP_KERNEL);

done:
	usb_autopm_put_interface(data->intf);
	RTKBT_DBG("%s end  pm_usage_cnt(0x%x)",__FUNCTION__,atomic_read(&(data->intf ->pm_usage_cnt)));

	return 0;

failed:
	clear_bit(BTUSB_INTR_RUNNING, &data->flags);
	clear_bit(HCI_RUNNING, &hdev->flags);
	usb_autopm_put_interface(data->intf);
	RTKBT_DBG("%s failed  pm_usage_cnt(0x%x)",__FUNCTION__,atomic_read(&(data->intf ->pm_usage_cnt)));
	return err;
}

static void btusb_stop_traffic(struct btusb_data *data)
{
	mdelay(URB_CANCELING_DELAY_MS);    // Added by Realtek
	usb_kill_anchored_urbs(&data->intr_anchor);
	usb_kill_anchored_urbs(&data->bulk_anchor);
	usb_kill_anchored_urbs(&data->isoc_anchor);
}

static int btusb_close(struct hci_dev *hdev)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	int i,err;

	if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags))
		return 0;

	RTKBT_DBG("btusb_close");
	/*******************************/
	for (i = 0; i < NUM_REASSEMBLY; i++) {
		if(hdev->reassembly[i]) {
			kfree_skb(hdev->reassembly[i]);
			hdev->reassembly[i] = NULL;
			RTKBT_DBG("%s free ressembly i=%d",__FUNCTION__,i);
		}
	}
	/*******************************/
	cancel_work_sync(&data->work);
	cancel_work_sync(&data->waker);

	clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
	clear_bit(BTUSB_BULK_RUNNING, &data->flags);
	clear_bit(BTUSB_INTR_RUNNING, &data->flags);

	btusb_stop_traffic(data);
	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		goto failed;

	data->intf->needs_remote_wakeup = 0;
	usb_autopm_put_interface(data->intf);

failed:
	mdelay(URB_CANCELING_DELAY_MS);     // Added by Realtek
	usb_scuttle_anchored_urbs(&data->deferred);
	return 0;
}

static int btusb_flush(struct hci_dev *hdev)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);

	RTKBT_DBG("%s add delay ",__FUNCTION__);
	mdelay(URB_CANCELING_DELAY_MS);     // Added by Realtek
	usb_kill_anchored_urbs(&data->tx_anchor);

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
static int btusb_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
#else
static int btusb_send_frame(struct sk_buff *skb)
{
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
#endif
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct usb_ctrlrequest *dr;
	struct urb *urb;
	unsigned int pipe;
	unsigned char *aligned_data;
	int err;

	BT_DBG("%s", hdev->name);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	skb->dev = (void *) hdev;
#endif

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return -EBUSY;

	/* Some platforms such as ARM require dword alignment for the skb->data
	 * To ensure that, allocate a buffer and copy the data
	 */
	aligned_data = kmalloc(max_t(unsigned int, 256, skb->len), GFP_ATOMIC);
	if (!aligned_data)
		return -ENOMEM;
	memcpy(aligned_data, skb->data, skb->len);

	switch (bt_cb(skb)->pkt_type) {
	case HCI_COMMAND_PKT:
		urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!urb) {
			err = -ENOMEM;
			goto exit;
		}
		dr = kmalloc(sizeof(*dr), GFP_ATOMIC);
		if (!dr) {
			err = -ENOMEM;
			goto done;
		}

		dr->bRequestType = data->cmdreq_type;
		dr->bRequest     = 0;
		dr->wIndex       = 0;
		dr->wValue       = 0;
		dr->wLength      = __cpu_to_le16(skb->len);

		pipe = usb_sndctrlpipe(data->udev, 0x00);

		usb_fill_control_urb(urb, data->udev, pipe, (void *) dr,
				     aligned_data, skb->len, btusb_tx_complete, skb);

		hdev->stat.cmd_tx++;
		break;

	case HCI_ACLDATA_PKT:
		if (!data->bulk_tx_ep) {
			err = -ENODEV;
			goto exit;
		}

		urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!urb) {
			err = -ENOMEM;
			goto exit;
		}
		pipe = usb_sndbulkpipe(data->udev,
					data->bulk_tx_ep->bEndpointAddress);

		usb_fill_bulk_urb(urb, data->udev, pipe,
				  aligned_data, skb->len, btusb_tx_complete, skb);

		hdev->stat.acl_tx++;
		break;

	case HCI_SCODATA_PKT:
		if (!data->isoc_tx_ep || hdev->conn_hash.sco_num < 1) {
			err = -ENODEV;
			goto exit;
		}
		urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, GFP_ATOMIC);
		if (!urb) {
			err = -ENOMEM;
			goto exit;
		}

		pipe = usb_sndisocpipe(data->udev,
					data->isoc_tx_ep->bEndpointAddress);

		usb_fill_int_urb(urb, data->udev, pipe,
				 aligned_data, skb->len, btusb_isoc_tx_complete,
				 skb, data->isoc_tx_ep->bInterval);

		urb->transfer_flags  = URB_ISO_ASAP;

		__fill_isoc_descriptor(urb, skb->len,
				le16_to_cpu(data->isoc_tx_ep->wMaxPacketSize));

		hdev->stat.sco_tx++;
		goto skip_waking;

	default:
		err = -EILSEQ;
		goto exit;
	}

	err = inc_tx(data);
	if (err) {
		usb_anchor_urb(urb, &data->deferred);
		schedule_work(&data->waker);
		err = 0;
		goto done;
	}

skip_waking:
	usb_anchor_urb(urb, &data->tx_anchor);
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		BT_ERR("btusb_send_frame %s urb %p submission failed", hdev->name, urb);
		kfree(urb->setup_packet);
		usb_unanchor_urb(urb);
	} else {
		usb_mark_last_busy(data->udev);
	}
	usb_free_urb(urb);

done:
exit:
	kfree(aligned_data);
	return err;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
static void btusb_destruct(struct hci_dev *hdev)
{
	RTKBT_DBG("btusb_destruct %s", hdev->name);
	hci_free_dev(hdev);
}
#endif



static void btusb_notify(struct hci_dev *hdev, unsigned int evt)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);

	BT_DBG("%s evt %d", hdev->name, evt);
	RTKBT_DBG("btusb_notify : %s evt %d", hdev->name, evt);

	if (hdev->conn_hash.sco_num != data->sco_num) {
		data->sco_num = hdev->conn_hash.sco_num;
		schedule_work(&data->work);
	}
}

static inline int __set_isoc_interface(struct hci_dev *hdev, int altsetting)
{
	struct btusb_data *data = GET_DRV_DATA(hdev);
	struct usb_interface *intf = data->isoc;
	struct usb_endpoint_descriptor *ep_desc;
	int i, err;

	if (!data->isoc)
		return -ENODEV;

	err = usb_set_interface(data->udev, 1, altsetting);
	if (err < 0) {
		BT_ERR("%s setting interface failed (%d)", hdev->name, -err);
		return err;
	}

	data->isoc_altsetting = altsetting;

	data->isoc_tx_ep = NULL;
	data->isoc_rx_ep = NULL;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!data->isoc_tx_ep && usb_endpoint_is_isoc_out(ep_desc)) {
			data->isoc_tx_ep = ep_desc;
			continue;
		}

		if (!data->isoc_rx_ep && usb_endpoint_is_isoc_in(ep_desc)) {
			data->isoc_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->isoc_tx_ep || !data->isoc_rx_ep) {
		BT_ERR("%s invalid SCO descriptors", hdev->name);
		return -ENODEV;
	}

	return 0;
}

static void btusb_work(struct work_struct *work)
{
	struct btusb_data *data = container_of(work, struct btusb_data, work);
	struct hci_dev *hdev = data->hdev;
	int err;

	if (hdev->conn_hash.sco_num > 0) {
		if (!test_bit(BTUSB_DID_ISO_RESUME, &data->flags)) {
			err = usb_autopm_get_interface(data->isoc ? data->isoc : data->intf);
			if (err < 0) {
				clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
				mdelay(URB_CANCELING_DELAY_MS);    // Added by Realtek
				usb_kill_anchored_urbs(&data->isoc_anchor);
				return;
			}

			set_bit(BTUSB_DID_ISO_RESUME, &data->flags);
		}
		if (data->isoc_altsetting != 2) {
			clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
			mdelay(URB_CANCELING_DELAY_MS);    // Added by Realtek
			usb_kill_anchored_urbs(&data->isoc_anchor);

			if (__set_isoc_interface(hdev, 2) < 0)
				return;
		}

		if (!test_and_set_bit(BTUSB_ISOC_RUNNING, &data->flags)) {
			if (btusb_submit_isoc_urb(hdev, GFP_KERNEL) < 0)
				clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
			else
				btusb_submit_isoc_urb(hdev, GFP_KERNEL);
		}
	} else {
		clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
		mdelay(URB_CANCELING_DELAY_MS);      // Added by Realtek
		usb_kill_anchored_urbs(&data->isoc_anchor);

		__set_isoc_interface(hdev, 0);
		if (test_and_clear_bit(BTUSB_DID_ISO_RESUME, &data->flags))
			usb_autopm_put_interface(data->isoc ? data->isoc : data->intf);
	}
}

static void btusb_waker(struct work_struct *work)
{
	struct btusb_data *data = container_of(work, struct btusb_data, waker);
	int err;

	err = usb_autopm_get_interface(data->intf);
	RTKBT_DBG("%s start  pm_usage_cnt(0x%x)",__FUNCTION__,atomic_read(&(data->intf ->pm_usage_cnt)));
	if (err < 0)
		return;

	usb_autopm_put_interface(data->intf);
	RTKBT_DBG("%s end  pm_usage_cnt(0x%x)",__FUNCTION__,atomic_read(&(data->intf ->pm_usage_cnt)));
}

static int btusb_probe(struct usb_interface *intf,
				const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *ep_desc;
	struct btusb_data *data;
	struct hci_dev *hdev;
	int i, err,flag1,flag2;
	struct usb_device *udev;
	udev = interface_to_usbdev(intf);

	/* interface numbers are hardcoded in the spec */
	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return -ENODEV;

	/*******************************/
	flag1=device_can_wakeup(&udev->dev);
	flag2=device_may_wakeup(&udev->dev);
	RTKBT_DBG("btusb_probe 1==========can_wakeup=%x	 flag2=%x",flag1,flag2);
	//device_wakeup_enable(&udev->dev);
	device_wakeup_disable(&udev->dev);
	flag1=device_can_wakeup(&udev->dev);
	flag2=device_may_wakeup(&udev->dev);
	RTKBT_DBG("wakeup_disable==========can_wakeup=%x	 flag2=%x",flag1,flag2);
	err = patch_add(intf);
	if (err < 0) return -1;
	/*******************************/

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;


	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!data->intr_ep && usb_endpoint_is_int_in(ep_desc)) {
			data->intr_ep = ep_desc;
			continue;
		}

		if (!data->bulk_tx_ep && usb_endpoint_is_bulk_out(ep_desc)) {
			data->bulk_tx_ep = ep_desc;
			continue;
		}

		if (!data->bulk_rx_ep && usb_endpoint_is_bulk_in(ep_desc)) {
			data->bulk_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->intr_ep || !data->bulk_tx_ep || !data->bulk_rx_ep) {
		kfree(data);
		return -ENODEV;
	}

	data->cmdreq_type = USB_TYPE_CLASS;

	data->udev = interface_to_usbdev(intf);
	data->intf = intf;

	spin_lock_init(&data->lock);

	INIT_WORK(&data->work, btusb_work);
	INIT_WORK(&data->waker, btusb_waker);
	spin_lock_init(&data->txlock);

	init_usb_anchor(&data->tx_anchor);
	init_usb_anchor(&data->intr_anchor);
	init_usb_anchor(&data->bulk_anchor);
	init_usb_anchor(&data->isoc_anchor);
	init_usb_anchor(&data->deferred);

	hdev = hci_alloc_dev();
	if (!hdev) {
		kfree(data);
		return -ENOMEM;
	}

	HDEV_BUS = HCI_USB;

	data->hdev = hdev;

	SET_HCIDEV_DEV(hdev, &intf->dev);

	hdev->open     = btusb_open;
	hdev->close    = btusb_close;
	hdev->flush    = btusb_flush;
	hdev->send     = btusb_send_frame;
	hdev->notify   = btusb_notify;


#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 4, 0)
	hci_set_drvdata(hdev, data);
#else
	hdev->driver_data = data;
	hdev->destruct = btusb_destruct;
	hdev->owner = THIS_MODULE;
#endif



	/* Interface numbers are hardcoded in the specification */
	data->isoc = usb_ifnum_to_if(data->udev, 1);

	if (data->isoc) {
		err = usb_driver_claim_interface(&btusb_driver,
							data->isoc, data);
		if (err < 0) {
			hci_free_dev(hdev);
			kfree(data);
			return err;
		}
	}

	err = hci_register_dev(hdev);
	if (err < 0) {
		hci_free_dev(hdev);
		kfree(data);
		return err;
	}

	usb_set_intfdata(intf, data);

	return 0;
}

static void btusb_disconnect(struct usb_interface *intf)
{
	struct btusb_data *data = usb_get_intfdata(intf);
	struct hci_dev *hdev;
	struct usb_device *udev;
	udev = interface_to_usbdev(intf);

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return;

	if (!data)
		return;

	RTKBT_DBG("btusb_disconnect");
	/*******************************/
	patch_remove(intf);
	/*******************************/

	hdev = data->hdev;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
	__hci_dev_hold(hdev);
#endif

	usb_set_intfdata(data->intf, NULL);

	if (data->isoc)
		usb_set_intfdata(data->isoc, NULL);

	hci_unregister_dev(hdev);

	if (intf == data->isoc)
		usb_driver_release_interface(&btusb_driver, data->intf);
	else if (data->isoc)
		usb_driver_release_interface(&btusb_driver, data->isoc);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
	__hci_dev_put(hdev);
#endif

	hci_free_dev(hdev);
	kfree(data);
}

#ifdef CONFIG_PM
static int btusb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct btusb_data *data = usb_get_intfdata(intf);

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return 0;

	/*******************************/
	RTKBT_DBG("btusb_suspend message.event=0x%x,data->suspend_count=%d",message.event,data->suspend_count);
	if (!test_bit(HCI_RUNNING, &data->hdev->flags)) {
		RTKBT_DBG("btusb_suspend-----bt is off");
		set_btoff(data->intf);
	}
	/*******************************/

	if (data->suspend_count++)
		return 0;

	spin_lock_irq(&data->txlock);
	if (!((message.event & PM_EVENT_AUTO) && data->tx_in_flight)) {
		set_bit(BTUSB_SUSPENDING, &data->flags);
		spin_unlock_irq(&data->txlock);
	} else {
		spin_unlock_irq(&data->txlock);
		data->suspend_count--;
		return -EBUSY;
	}

	cancel_work_sync(&data->work);

	btusb_stop_traffic(data);
	mdelay(URB_CANCELING_DELAY_MS);      // Added by Realtek
	usb_kill_anchored_urbs(&data->tx_anchor);

	return 0;
}

static void play_deferred(struct btusb_data *data)
{
	struct urb *urb;
	int err;

	while ((urb = usb_get_from_anchor(&data->deferred))) {

	       /************************************/
		usb_anchor_urb(urb, &data->tx_anchor);
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err < 0) {
			BT_ERR("play_deferred urb %p submission failed",  urb);
			kfree(urb->setup_packet);
			usb_unanchor_urb(urb);
		} else {
			usb_mark_last_busy(data->udev);
		}
		usb_free_urb(urb);
		/************************************/
		data->tx_in_flight++;
	}
	mdelay(URB_CANCELING_DELAY_MS);     // Added by Realtek
	usb_scuttle_anchored_urbs(&data->deferred);
}

static int btusb_resume(struct usb_interface *intf)
{
	struct btusb_data *data = usb_get_intfdata(intf);
	struct hci_dev *hdev = data->hdev;
	int err = 0;

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return 0;


	/*******************************/
	RTKBT_DBG("btusb_resume data->suspend_count=%d",data->suspend_count);

	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		RTKBT_DBG("btusb_resume-----bt is off,download patch");
		download_patch(intf);
	}
	else
	        RTKBT_DBG("btusb_resume,----bt is on");
	/*******************************/
	if (--data->suspend_count)
		return 0;

	if (test_bit(BTUSB_INTR_RUNNING, &data->flags)) {
		err = btusb_submit_intr_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_INTR_RUNNING, &data->flags);
			goto failed;
		}
	}

	if (test_bit(BTUSB_BULK_RUNNING, &data->flags)) {
		err = btusb_submit_bulk_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_BULK_RUNNING, &data->flags);
			goto failed;
		}

		btusb_submit_bulk_urb(hdev, GFP_NOIO);
	}

	if (test_bit(BTUSB_ISOC_RUNNING, &data->flags)) {
		if (btusb_submit_isoc_urb(hdev, GFP_NOIO) < 0)
			clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
		else
			btusb_submit_isoc_urb(hdev, GFP_NOIO);
	}

	spin_lock_irq(&data->txlock);
	play_deferred(data);
	clear_bit(BTUSB_SUSPENDING, &data->flags);
	spin_unlock_irq(&data->txlock);
	schedule_work(&data->work);

	return 0;

failed:
	mdelay(URB_CANCELING_DELAY_MS);      // Added by Realtek
	usb_scuttle_anchored_urbs(&data->deferred);
//done:
	spin_lock_irq(&data->txlock);
	clear_bit(BTUSB_SUSPENDING, &data->flags);
	spin_unlock_irq(&data->txlock);

	return err;
}
#endif

static struct usb_driver btusb_driver = {
	.name		= "rtk_btusb",
	.probe		= btusb_probe,
	.disconnect	= btusb_disconnect,
#ifdef CONFIG_PM
	.suspend	= btusb_suspend,
	.resume		= btusb_resume,
#endif
	.id_table	= btusb_table,
	.supports_autosuspend = 1,
};

static int __init btusb_init(void)
{
	RTKBT_DBG("Realtek Bluetooth USB driver ver %s", VERSION);

	return usb_register(&btusb_driver);
}

static void __exit btusb_exit(void)
{
	usb_deregister(&btusb_driver);
}

module_init(btusb_init);
module_exit(btusb_exit);

MODULE_AUTHOR("Edward Bian <edward_bian@realsil.com.cn>");
MODULE_DESCRIPTION("Realtek Bluetooth USB driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");



/*******************************
**    Reasil patch code
********************************/


#include <linux/firmware.h>
#include <linux/suspend.h>
#include <net/bluetooth/hci.h>


#define CMD_CMP_EVT		0x0e
#define PKT_LEN			300
#define MSG_TO			1000
#define PATCH_SEG_MAX	252
#define DATA_END		0x80
#define DOWNLOAD_OPCODE	0xfc20
#define BTOFF_OPCODE	0xfc28
#define TRUE			1
#define FALSE			0
#define CMD_HDR_LEN		sizeof(struct hci_command_hdr)
#define EVT_HDR_LEN		sizeof(struct hci_event_hdr)
#define CMD_CMP_LEN		sizeof(struct hci_ev_cmd_complete)


enum rtk_endpoit {
	CTRL_EP = 0,
	INTR_EP = 1,
	BULK_EP = 2,
	ISOC_EP = 3
};

typedef struct {
	uint16_t	prod_id;
	uint16_t	lmp_sub;
	char		*patch_name;
	char		*config_name;
	uint8_t		*fw_cache;
	int			fw_len;
} patch_info;

typedef struct {
	struct list_head		list_node;
	struct usb_interface	*intf;
	struct usb_device		*udev;
	struct notifier_block	pm_notifier;
	patch_info				*patch_entry;
} dev_data;

typedef struct {
	dev_data	*dev_entry;
	int			pipe_in, pipe_out;
	uint8_t		send_pkt[PKT_LEN];
	uint8_t		rcv_pkt[PKT_LEN];
	struct hci_command_hdr		*cmd_hdr;
	struct hci_event_hdr		*evt_hdr;
	struct hci_ev_cmd_complete	*cmd_cmp;
	uint8_t		*req_para, *rsp_para;
	uint8_t		*fw_data;
	int			pkt_len, fw_len;
} xchange_data;

typedef struct {
	uint8_t index;
	uint8_t data[PATCH_SEG_MAX];
} __attribute__((packed)) download_cp;

typedef struct {
	uint8_t status;
	uint8_t index;
} __attribute__((packed)) download_rp;


static dev_data* dev_data_find(struct usb_interface* intf);
static patch_info* get_patch_entry(struct usb_device* udev);
static int rtkbt_pm_notify(struct notifier_block* notifier, ulong pm_event, void* unused);
static int load_firmware(dev_data* dev_entry, uint8_t** buff);
static void init_xdata(xchange_data* xdata, dev_data* dev_entry);
static int check_fw_version(xchange_data* xdata);
static int get_firmware(xchange_data* xdata);
static int download_data(xchange_data* xdata);
static int send_hci_cmd(xchange_data* xdata);
static int rcv_hci_evt(xchange_data* xdata);


static patch_info patch_table[] = {
	{0, 0x1200, "rtk_bt/rtk8723a.bin", "rtk8723_bt_config", NULL, 0}
};

static LIST_HEAD(dev_data_list);


int patch_add(struct usb_interface* intf)
{
	dev_data	*dev_entry;
	struct usb_device *udev;

	RTKBT_DBG("patch_add");
	dev_entry = dev_data_find(intf);
	if (NULL != dev_entry)
		return -1;

	udev = interface_to_usbdev(intf);
#if BTUSB_RPM
	RTKBT_DBG("auto suspend is enabled");
	usb_enable_autosuspend(udev);
	pm_runtime_set_autosuspend_delay(&(udev->dev),2000);
#endif

	dev_entry = kzalloc(sizeof(dev_data), GFP_KERNEL);
	dev_entry->intf = intf;
	dev_entry->udev = udev;
	dev_entry->pm_notifier.notifier_call = rtkbt_pm_notify;
	dev_entry->patch_entry = get_patch_entry(udev);
	list_add(&dev_entry->list_node, &dev_data_list);
	register_pm_notifier(&dev_entry->pm_notifier);

	return 0;
}

void patch_remove(struct usb_interface* intf)
{
	dev_data *dev_entry;
	struct usb_device *udev;

	udev = interface_to_usbdev(intf);
#if BTUSB_RPM
	usb_disable_autosuspend(udev);
#endif

	dev_entry = dev_data_find(intf);
	if (NULL == dev_entry)
		return;

	RTKBT_DBG("patch_remove");
	list_del(&dev_entry->list_node);
	unregister_pm_notifier(&dev_entry->pm_notifier);
	kfree(dev_entry);
}

int download_patch(struct usb_interface* intf)
{
	dev_data		*dev_entry;
	xchange_data	xdata;
	uint8_t			*fw_buf;
	int				ret_val;

	RTKBT_DBG("download_patch start");
	dev_entry = dev_data_find(intf);
	if (NULL == dev_entry) {
		ret_val = -1;
		RTKBT_DBG("NULL == dev_entry");
		goto patch_end;
	}

	init_xdata(&xdata, dev_entry);
	ret_val = check_fw_version(&xdata);
	if (ret_val != 0)
		goto patch_end;

	ret_val = get_firmware(&xdata);
	if (ret_val < 0) {
		RTKBT_DBG("get_firmware failed!");
		goto patch_end;
	}
	fw_buf = xdata.fw_data;

	ret_val = download_data(&xdata);
	if (ret_val < 0) {
		RTKBT_DBG("download_data failed!");
		goto patch_fail;
	}

	ret_val = check_fw_version(&xdata);
	if (ret_val <= 0) {
		ret_val = -1;
		goto patch_fail;
	}

	ret_val = 0;
patch_fail:
	kfree(fw_buf);
patch_end:
	RTKBT_DBG("Rtk patch end %d", ret_val);
	return ret_val;
}

int set_btoff(struct usb_interface* intf)
{
	dev_data		*dev_entry;
	xchange_data	xdata;
	int				ret_val;

	RTKBT_DBG("set_btoff");
	dev_entry = dev_data_find(intf);
	if (NULL == dev_entry)
		return -1;

	init_xdata(&xdata, dev_entry);
	xdata.cmd_hdr->opcode = cpu_to_le16(BTOFF_OPCODE);
	xdata.cmd_hdr->plen = 1;
	xdata.pkt_len = CMD_HDR_LEN + 1;
	xdata.send_pkt[CMD_HDR_LEN] = 1;

	ret_val = send_hci_cmd(&xdata);
	if (ret_val < 0)
		return ret_val;

	ret_val = rcv_hci_evt(&xdata);
	if (ret_val < 0)
		return ret_val;

	RTKBT_DBG("set_btoff done");
	return 0;
}

dev_data* dev_data_find(struct usb_interface* intf)
{
	dev_data *dev_entry;

	list_for_each_entry(dev_entry, &dev_data_list, list_node) {
		if (dev_entry->intf == intf)
			return dev_entry;
	}

	return NULL;
}

patch_info* get_patch_entry(struct usb_device* udev)
{
	patch_info	*patch_entry;
	uint16_t	pid;

	patch_entry = patch_table;
	pid = le16_to_cpu(udev->descriptor.idProduct);
	while (pid != patch_entry->prod_id) {
		if (0 == patch_entry->prod_id) break;
		patch_entry++;
	}

	return patch_entry;
}

int rtkbt_pm_notify(
	struct notifier_block* notifier,
	ulong	pm_event,
	void*	unused)
{
	dev_data	*dev_entry;
	patch_info	*patch_entry;
	struct usb_device *udev;

	dev_entry = container_of(notifier, dev_data, pm_notifier);
	patch_entry = dev_entry->patch_entry;
	udev = dev_entry->udev;
	RTKBT_DBG("rtkbt_pm_notify pm_event =%ld",pm_event);
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		patch_entry->fw_len = load_firmware(dev_entry, &patch_entry->fw_cache);
		if (patch_entry->fw_len <= 0) {
			RTKBT_DBG("rtkbt_pm_notify return NOTIFY_BAD");
			return NOTIFY_BAD;
		}

		if (!device_may_wakeup(&udev->dev)) {
			dev_entry->intf->needs_binding = 1;
			RTKBT_DBG("remote wakeup not support, set intf->needs_binding = 1");
		}
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		if (patch_entry->fw_len > 0) {
			kfree(patch_entry->fw_cache);
			patch_entry->fw_cache = NULL;
			patch_entry->fw_len = 0;
		}
#if BTUSB_RPM
		usb_disable_autosuspend(udev);
		usb_enable_autosuspend(udev);
		pm_runtime_set_autosuspend_delay(&(udev->dev),2000);
#endif
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

int load_firmware(dev_data* dev_entry, uint8_t** buff)
{
	const struct firmware	*fw;
	struct usb_device		*udev;
	patch_info	*patch_entry;
	char		*fw_name;
	int		fw_len = 0, ret_val;

	RTKBT_DBG("load_firmware");
	udev = dev_entry->udev;
	patch_entry = dev_entry->patch_entry;
	fw_name = patch_entry->patch_name;
	ret_val = request_firmware(&fw, fw_name, &udev->dev);
	if (ret_val < 0) goto fw_fail;

	*buff = kzalloc(fw->size, GFP_KERNEL);
	if (NULL == *buff) goto alloc_fail;
	memcpy(*buff, fw->data, fw->size);
	fw_len = fw->size;

#if LOAD_CONFIG
	release_firmware(fw);
	fw_name = patch_entry->config_name;
	ret_val = request_firmware(&fw, fw_name, &udev->dev);
	if (ret_val < 0) {
		fw_len = 0;
		kfree(*buff);
		*buff = NULL;
		goto fw_fail;
	}

	*buff = krealloc(*buff, fw_len + fw->size, GFP_KERNEL);
	if (NULL == *buff) {
		fw_len = 0;
		goto alloc_fail;
	}
	memcpy(*buff + fw_len, fw->data, fw->size);
	fw_len += fw->size;
#endif

alloc_fail:
	release_firmware(fw);
fw_fail:
	return fw_len;
}

void init_xdata(
	xchange_data*	xdata,
	dev_data*		dev_entry)
{
	memset(xdata, 0, sizeof(xchange_data));
	xdata->dev_entry = dev_entry;
	xdata->pipe_in = usb_rcvintpipe(dev_entry->udev, INTR_EP);
	xdata->pipe_out = usb_sndctrlpipe(dev_entry->udev, CTRL_EP);
	xdata->cmd_hdr = (struct hci_command_hdr*)(xdata->send_pkt);
	xdata->evt_hdr = (struct hci_event_hdr*)(xdata->rcv_pkt);
	xdata->cmd_cmp = (struct hci_ev_cmd_complete*)(xdata->rcv_pkt + EVT_HDR_LEN);
	xdata->req_para = xdata->send_pkt + CMD_HDR_LEN;
	xdata->rsp_para = xdata->rcv_pkt + EVT_HDR_LEN + CMD_CMP_LEN;
}

int check_fw_version(xchange_data* xdata)
{
	struct hci_rp_read_local_version *read_ver_rsp;
	patch_info	*patch_entry;
	int			ret_val;

	xdata->cmd_hdr->opcode = cpu_to_le16(HCI_OP_READ_LOCAL_VERSION);
	xdata->cmd_hdr->plen = 0;
	xdata->pkt_len = CMD_HDR_LEN;

	ret_val = send_hci_cmd(xdata);
	if (ret_val < 0) {
		goto version_end;
	}

	ret_val = rcv_hci_evt(xdata);
	if (ret_val < 0) {
		goto version_end;
	}

	patch_entry = xdata->dev_entry->patch_entry;
	read_ver_rsp = (struct hci_rp_read_local_version*)(xdata->rsp_para);
	RTKBT_DBG("check_fw_version : read_ver_rsp->lmp_subver = 0x%x",read_ver_rsp->lmp_subver);
	if (patch_entry->lmp_sub != read_ver_rsp->lmp_subver)
		return 1;

	ret_val = 0;
version_end:
	return ret_val;
}

int get_firmware(xchange_data* xdata)
{
	dev_data	*dev_entry;
	patch_info	*patch_entry;

	dev_entry = xdata->dev_entry;
	patch_entry = dev_entry->patch_entry;
	if (patch_entry->fw_len > 0) {
		xdata->fw_data = kzalloc(patch_entry->fw_len, GFP_KERNEL);
		if (NULL == xdata->fw_data) return -ENOMEM;
		memcpy(xdata->fw_data, patch_entry->fw_cache, patch_entry->fw_len);
		xdata->fw_len = patch_entry->fw_len;
	} else {
		xdata->fw_len = load_firmware(dev_entry, &xdata->fw_data);
		if (xdata->fw_len <= 0) return -1;
	}

	return 0;
}

int download_data(xchange_data* xdata)
{
	download_cp *cmd_para;
	download_rp *evt_para;
	uint8_t		*pcur;
	int			pkt_len, frag_num, frag_len;
	int			i, ret_val;

	cmd_para = (download_cp*)xdata->req_para;
	evt_para = (download_rp*)xdata->rsp_para;
	pcur = xdata->fw_data;
	pkt_len = CMD_HDR_LEN + sizeof(download_cp);
	frag_num = xdata->fw_len / PATCH_SEG_MAX + 1;
	frag_len = PATCH_SEG_MAX;

	for (i = 0; i < frag_num; i++) {
		cmd_para->index = i;
		if (i == (frag_num - 1)) {
			cmd_para->index |= DATA_END;
			frag_len = xdata->fw_len % PATCH_SEG_MAX;
			pkt_len -= (PATCH_SEG_MAX - frag_len);
		}
		xdata->cmd_hdr->opcode = cpu_to_le16(DOWNLOAD_OPCODE);
		xdata->cmd_hdr->plen = sizeof(uint8_t) + frag_len;
		xdata->pkt_len = pkt_len;
		memcpy(cmd_para->data, pcur, frag_len);

		ret_val = send_hci_cmd(xdata);
		if (ret_val < 0)
			return ret_val;

		ret_val = rcv_hci_evt(xdata);
		if (ret_val < 0)
			return ret_val;
		if (0 != evt_para->status)
			return -1;

		pcur += PATCH_SEG_MAX;
	}

	return xdata->fw_len;
}

int send_hci_cmd(xchange_data* xdata)
{
	int ret_val;

	ret_val = usb_control_msg(
		xdata->dev_entry->udev, xdata->pipe_out,
		0, USB_TYPE_CLASS, 0, 0,
		(void*)(xdata->send_pkt),
		xdata->pkt_len, MSG_TO);

	return ret_val;
}

int rcv_hci_evt(xchange_data* xdata)
{
	int ret_len, ret_val;
	int i;   // Added by Realtek

	while (1) {
		// **************************** Modifed by Realtek (begin)
		for(i = 0; i < 5; i++) {   // Try to send USB interrupt message 5 times.
			ret_val = usb_interrupt_msg(xdata->dev_entry->udev,
						    xdata->pipe_in,
						    (void*)(xdata->rcv_pkt),
						    PKT_LEN,
						    &ret_len, MSG_TO);
			if(ret_val >= 0)
				break;
		}
		// **************************** Modifed by Realtek (end)

		if (ret_val < 0)
			return ret_val;


		if (CMD_CMP_EVT == xdata->evt_hdr->evt) {
			if (xdata->cmd_hdr->opcode == xdata->cmd_cmp->opcode)
				return ret_len;
		}
	}
}
