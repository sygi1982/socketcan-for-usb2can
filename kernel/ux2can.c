/*
 * ux2can.c - universal driver for devices which are accessible only from user space
 *
 * This file is derived from linux/drivers/net/can/ux2can.c
 *
 * ux2can.c Authors : sygi <sygi@canbus.pl>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307. You can also get it
 * at http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <socketcan-users@lists.berlios.de>
 *
 */

#include <linux/version.h>
#include <linux/module.h>

#include <asm/system.h>
#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/in.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/if_slip.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include <socketcan/can.h>
#include <socketcan/can/core.h>
#include <socketcan/can/dev.h>

#include <linux/platform_device.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
#error This driver does not support Kernel versions < 2.6.22
#endif

#define DRIVER_NAME	"ux2can"

#define CONFIG_CAN_DEBUG

#ifdef CONFIG_CAN_DEBUG
#define DBG(args...)  (printk(KERN_ERR DRIVER_NAME": %s ", __func__), printk(args"\n"))
#else
#define DBG(args...)
#endif

#define ERR(args...)  (printk(KERN_ERR DRIVER_NAME":"), printk(args"\n"))

struct ux2can {
	struct can_priv		can;	/* must be the first member! */

	struct net_device	*net;
	struct mutex		lock;
	struct platform_device  *pdev;
	atomic_t		registred;

	struct list_head	tx_list;
	spinlock_t		tx_lock;
};

struct ux2can_transfer {
	struct can_frame	frame;
	struct list_head	entry;
};

static int ux2can_netopen(struct net_device *net)
{
	struct ux2can *dev = netdev_priv(net);
	int err;

	/* check or determine and set bittime */
	err = open_candev(net);
	if (err)
		goto out;

	INIT_LIST_HEAD(&dev->tx_list);
	spin_lock_init(&dev->tx_lock);
	dev->can.state = CAN_STATE_ERROR_ACTIVE;

	/* start queuing */
	netif_start_queue(net);
	atomic_set(&dev->registred, 1);

out:
	return err;
}

static int ux2can_netstop(struct net_device *net)
{
	struct ux2can *dev = netdev_priv(net);
	struct ux2can_transfer *xfer, *tmp;

	atomic_set(&dev->registred, 0);

	netif_stop_queue(net);
	dev->can.state = CAN_STATE_STOPPED;

	// clean tx_list
	list_for_each_entry_safe(xfer, tmp, &dev->tx_list, entry) {
		list_del(&xfer->entry);
		kfree(xfer);
	}

	close_candev(net);

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
static int ux2can_net_start_xmit(struct sk_buff *skb, struct net_device *net)
#else
static netdev_tx_t ux2can_net_start_xmit(struct sk_buff *skb, struct net_device *net)
#endif
{
	struct ux2can *dev = netdev_priv(net);
	struct can_frame *frame;
	struct ux2can_transfer *xfer;

	frame = (struct can_frame *)skb->data;
	dev->net->trans_start = jiffies;

	/* Allocate driver data */
	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	if (!xfer) {
		dev->net->stats.tx_dropped++;
		return NETDEV_TX_BUSY;
	}

	INIT_LIST_HEAD(&xfer->entry);
	memcpy(&xfer->frame, frame, sizeof(*frame));
	spin_lock(&dev->tx_lock);
	list_add(&xfer->entry, &dev->tx_list);
	spin_unlock(&dev->tx_lock);

	return NETDEV_TX_OK;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
static const struct net_device_ops ux2can_netdev_ops = {
	.ndo_open = ux2can_netopen,
	.ndo_stop = ux2can_netstop,
	.ndo_start_xmit = ux2can_net_start_xmit,
};
#endif

static inline int is_registred(struct ux2can *dev)
{
	return atomic_read(&dev->registred);
}

static void hexstring2array(char *hexs, int slen, unsigned char *tab)
{
	int i, j = 0;
	unsigned char val;

	for (i = 0; i < slen; i++) {
		val = hexs[j] > '9' ? hexs[j] - 0x37 : hexs[j] - 0x30;
		tab[i] = val << 4;
		val = hexs[j+1] > '9' ? hexs[j+1] - 0x37 : hexs[j+1] - 0x30;
		tab[i] |= val & 0x0F;
		j += 2;
	}
}

/*
 * Expected input: <hex pdu>
 * Example: "echo 12345678081234567812345678 > fifo" sends hex pdu
 */
static ssize_t ux2can_write_fifo(struct device *this,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	unsigned int id;
	unsigned char in[40];
	char int_buf[40];
	char *token;
	char *val;
	struct sk_buff *skb;
	struct can_frame *frame;

	struct ux2can *dev = (struct ux2can *) this->platform_data;

	strncpy((char *) &int_buf, buf, sizeof(int_buf));
	token = (char *) &int_buf;

	/* Parse message */
	val = strsep(&token, " ");
	dev_info(dev, "RX: %s -> %d", val, strlen(val));

	hexstring2array(val, strlen(val), in);

	if (!is_registred(dev)) {
		return count;
	}

	skb = alloc_can_skb(dev->net, &frame);
	if (!skb) {
		dev->net->stats.rx_dropped++;
		return count;
	}

	id = in[0] << 24;
	id |= in[1] << 16;
	id |= in[2] << 8;
	id |= in[3];

	frame->can_id = id;
	frame->can_dlc = in[4];
	memcpy(&frame->data[0], &in[5], in[4]);
	dev->net->stats.rx_packets++;
	dev->net->stats.rx_bytes += frame->can_dlc;
	netif_rx(skb);

	return count;
}

static void array2hexstring(unsigned char *tab, int tablen, char *hexs)
{
	char ref[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	int i;

	for (i = 0; i < tablen; i++) {
		hexs[i*2] = ref[((tab[i] >> 4) & 0xF)];
		hexs[(i*2) + 1] = ref[(tab[i] & 0xF)];
	}
}

/*
 * Expected output: <hex pdu>
 * Example: "cat fifo" -> 12345678081234567812345678 reads hex pdu
 */
static ssize_t ux2can_read_fifo(struct device *this,
			      struct device_attribute *attr,
			      char *buf)
{
	struct ux2can *dev = (struct ux2can *) this->platform_data;
	struct ux2can_transfer *xfer;
	unsigned int id;
	unsigned char dlc;
	unsigned char data[8];
	char out[17];
	int count = 0;

	if (!is_registred(dev))
		return 0;

	// read data from list - this what was xmited
	spin_lock(&dev->tx_lock);
	if (list_empty(&dev->tx_list)) {
		spin_unlock(&dev->tx_lock);
		return 0;
	} else {
		xfer = list_first_entry(&dev->tx_list, struct ux2can_transfer, entry);
	}

	id = xfer->frame.can_id;
	dlc = xfer->frame.can_dlc;
	memcpy(&data[0], &xfer->frame.data[0], dlc);

	list_del(&xfer->entry);
	kfree(xfer);

	spin_unlock(&dev->tx_lock);

	array2hexstring(data, dlc, out);

	out[dlc * 2] = 0; // terminate string
	dev_info(dev, "TX: %x %d %s -> %d", id, dlc, out, strlen(out));
	dev->net->stats.tx_packets++;
	dev->net->stats.tx_bytes += dlc;
	netif_wake_queue(dev->net);

	count = sprintf(buf, "%08X%02X%s", id, dlc, out);

	buf[strlen(buf)] = '\0';

	return count + 1;
}

static DEVICE_ATTR(fifo, S_IRUGO | S_IWUGO, ux2can_read_fifo, ux2can_write_fifo);

/*
 * Does nothing
 */
static ssize_t ux2can_write_conf(struct device *this,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	char *token;
	char *val;
	char int_buf[5];
	int rate;

	struct ux2can *dev = (struct ux2can *) this->platform_data;

	strncpy((char *) &int_buf, buf, sizeof(int_buf));
	token = (char *) &int_buf;

	/* Parse message */
	val = strsep(&token, " ");
	dev_info(dev, "%s", val);

	hexstring2array(val, strlen(val), &rate);

	if (!is_registred(dev))
		return count;

	dev->can.bittiming.bitrate = rate;

	return count;
}

static DEVICE_ATTR(conf, S_IWUGO, NULL, ux2can_write_conf);

/*
 * Expected input: <hex pdu>
 * Example: "echo 1 > status" updates status
 */
static ssize_t ux2can_write_status(struct device *this,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	char *token;
	char *val;
	char int_buf[5];
	unsigned char new_status = 0;

	struct ux2can *dev = (struct ux2can *) this->platform_data;

	strncpy((char *) &int_buf, buf, sizeof(int_buf));
	token = (char *) &int_buf;

	/* Parse message */
	val = strsep(&token, " ");
	dev_info(dev, "%s", val);

	hexstring2array(val, strlen(val), &new_status);

	if (!is_registred(dev))
		return count;

	/* Update can state statistics */
	switch (dev->can.state) {
	case CAN_STATE_BUS_OFF:
		can_bus_off(dev->net);
		break;
	case CAN_STATE_ERROR_ACTIVE:
		if (new_status >= CAN_STATE_ERROR_WARNING &&
		    new_status <= CAN_STATE_BUS_OFF)
			dev->can.can_stats.error_warning++;
	case CAN_STATE_ERROR_WARNING:	/* fallthrough */
		if (new_status >= CAN_STATE_ERROR_PASSIVE &&
		    new_status <= CAN_STATE_BUS_OFF)
			dev->can.can_stats.error_passive++;
		break;
	default:
		break;
	}
	dev->can.state = new_status;

	return count;
}

static DEVICE_ATTR(status, S_IWUGO, NULL, ux2can_write_status);

static struct can_bittiming_const ux2can_bittiming_const = {
	// dummy values
	.name		= "ux2can",
	.tseg1_min	= 4,
	.tseg1_max	= 16,
	.tseg2_min	= 2,
	.tseg2_max	= 8,
	.sjw_max	= 4,
	.brp_min	= 2,
	.brp_max	= 128,
	.brp_inc	= 1,
};

static struct can_bittiming ux2can_bittiming = {
	// dummy values
	.bitrate	= 1000,
	.tq		= 2000,
};

int __init ux2can_probe(struct platform_device *pdev)
{
	struct ux2can *dev;
	struct net_device *net;
	int retval = 0;

	dev_info(&pdev->dev, "Probing (pdev = 0x%X)...\n",
		(u32) pdev);

	// Allocate can/net device
	net = alloc_candev(sizeof(struct ux2can), 1);
	if (!net) {
		retval = -ENOMEM;
		goto error;
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
	net->netdev_ops = &ux2can_netdev_ops;
#else
	net->open = ux2can_netopen;
	net->stop = ux2can_netstop;
	net->hard_start_xmit = ux2can_net_start_xmit;
#endif
	dev = netdev_priv(net);
	dev->net = net;
	dev->pdev = pdev;
	dev->can.bittiming_const = &ux2can_bittiming_const;
	dev->can.bittiming = ux2can_bittiming;
	dev_set_drvdata(&pdev->dev, (struct net_device *)net);
	// store platform data to get access from write/read functions
	pdev->dev.platform_data = dev;
	mutex_init(&dev->lock);
	atomic_set(&dev->registred, 0);
	SET_NETDEV_DEV(net, &pdev->dev);

	retval = register_candev(net);
	if (retval) {
		retval = -ENODEV;
		goto error_free;
	}

	if (device_create_file(&pdev->dev, &dev_attr_fifo) != 0)
		dev_warn(&(pdev->dev),
			 "Unable to create sysfs entry for fifo");

	if (device_create_file(&pdev->dev, &dev_attr_conf) != 0)
		dev_warn(&(pdev->dev),
			 "Unable to create sysfs entry for conf");

	if (device_create_file(&pdev->dev, &dev_attr_status) != 0)
		dev_warn(&(pdev->dev),
			 "Unable to create sysfs entry for status");

	dev_info(&pdev->dev, "driver loaded\n");

	DBG("ok");
	return 0;
error_free:
	free_candev(net);
	kfree (dev);
error:
	DBG("error");
	return retval;
}

static int __devexit ux2can_remove(struct platform_device *pdev)
{
	struct net_device *net = NULL;

	net = dev_get_drvdata(&pdev->dev);
	unregister_candev(net);
	free_candev(net);

	dev_info(&pdev->dev, "driver unloaded\n");

	return 0;
}

static struct platform_driver ux2can_driver = {
	.probe = ux2can_probe,
	.remove = __devexit_p(ux2can_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

// Virtual CAN device -> ?? is this the best way to store it
static struct platform_device *virtual = NULL;

static int __init ux2can_init(void)
{
	// Virtual ux2can device
	virtual = platform_device_register_simple("ux2can", 1, NULL, 0);
	DBG("virtual device registered");

	return platform_driver_register(&ux2can_driver);
}

module_init(ux2can_init);

static void __exit ux2can_exit(void)
{
	platform_driver_unregister(&ux2can_driver);
	DBG("virtual device unregistered");

	if (virtual)
		platform_device_del(virtual);
	DBG("virtual device removed");

	virtual = NULL;
}

module_exit(ux2can_exit);

MODULE_DESCRIPTION(DRIVER_NAME" interface");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("<sygi@canbus.pl>");
