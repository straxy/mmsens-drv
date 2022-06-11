/******************************************************************************
 *
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; version 2 of the License.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *****************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#define DEVICE_FILE_NAME	"mmsens"
#define DRIVER_NAME			"mmsensdrv"

#define MMSENS_CTRL_OFFSET		(0x0u)
#define MMSENS_STATUS_OFFSET	(0x4u)
#define MMSENS_DATA_OFFSET		(0x8u)

#define CTRL_EN_MASK	(0x1u)
#define CTRL_IEN_MASK	(0x2u)
#define CTRL_FREQ_MASK	(0x4u)

#define STATUS_IFG_MASK	(0x2u)

#define DATA_MASK		(0xFFFFu)

#define FREQ_NORMAL_MASK	(0x0u)
#define FREQ_FAST_MASK		(0x4u)

#define FREQ_LEN	(10)

/**
 * struct mmsens - mmsens device private data structure
 * @base_addr:	base address of the device
 * @irq:	interrupt for the device
 * @dev:	struct device pointer
 * @parent: parent pointer
 * @cdev:	struct cdev
 * @devt:	dev_t member
 */
struct mmsens {
	void __iomem *base_addr;
	int irq;
	struct device *dev;
	struct device *parent;
	struct cdev cdev;
	dev_t devt;
};

/* global so it can be destroyed when module is removed */
static struct class* mmsens_class;

/**
 * Open file operation for mmsens.
 */
static int mmsens_open(struct inode *inode, struct file *filp) {
	struct mmsens *dev;

	/* store mmsens pointer for read */
	dev = container_of(inode->i_cdev, struct mmsens, cdev);
	filp->private_data = dev;

	return 0;
}

/**
 * Release file operation for mmsens.
 */
static int mmsens_release(struct inode *inode, struct file *filp) {
	return 0;
}

/**
 * Read file operation for mmsens.
 * Reads DATA register and puts raw value that is read into @p buf variable.
 */
static ssize_t mmsens_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
	struct mmsens *dev;
	/* buffer */
	static char buffer[10];
	/* Number of bytes written to the buffer */
	ssize_t bytes_read = 0;
	/* data storage */
	int data_reg;

	if (*f_pos) {
		*f_pos = 0;
		return 0;
	}

	dev = filp->private_data;

	/* read data from DATA register */
	data_reg = ioread32(dev->base_addr + MMSENS_DATA_OFFSET);

	/* pack it into buffer */
	sprintf(buffer, "%d\n", data_reg);
	bytes_read = strlen(buffer);

	/* copy_to_user */
	if (copy_to_user(buf, buffer, bytes_read)) {
		return -EFAULT;
	}
	*f_pos += bytes_read;

	/* return number of bytes read */
	return bytes_read;
}

/**
 * Write file operation is not supported.
 */
static ssize_t mmsens_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	pr_alert("not implemented\n");
	return -EINVAL;
}

static struct file_operations mmsensdev_fops = {
	.owner = THIS_MODULE,
	.open = mmsens_open,
	.release = mmsens_release,
	.read = mmsens_read,
	.write = mmsens_write,
};

/* SYSFS attributes */

/**
 * Interrupt status getter.
 */
static ssize_t interrupt_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct mmsens *dev = dev_get_drvdata(child);

	u32 interrupt = ioread32(dev->base_addr + MMSENS_STATUS_OFFSET);
	interrupt &= STATUS_IFG_MASK;

	return sprintf(buf, "%d\n", !!interrupt);
}
static DEVICE_ATTR_RO(interrupt);

/**
 * Enable interrupt getter.
 */
static ssize_t enable_interrupt_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct mmsens *dev = dev_get_drvdata(child);

	u32 enable_interrupt = ioread32(dev->base_addr + MMSENS_CTRL_OFFSET);
	enable_interrupt &= CTRL_IEN_MASK;

	return sprintf(buf, "%d\n", !!enable_interrupt);
}

/**
 * Enable interrupt setter.
 */
static ssize_t enable_interrupt_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmsens *dev = dev_get_drvdata(child);

	u32 ctrl = ioread32(dev->base_addr + MMSENS_CTRL_OFFSET);

	int enable_interrupt;
	sscanf(buf, "%d", &enable_interrupt);

	if (!enable_interrupt) {
		ctrl &= ~CTRL_IEN_MASK;
	} else {
		ctrl |= CTRL_IEN_MASK;
	}

	iowrite32(ctrl, dev->base_addr + MMSENS_CTRL_OFFSET);

	return count;
}
static DEVICE_ATTR_RW(enable_interrupt);

/**
 * Enable device getter.
 */
static ssize_t enable_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct mmsens *dev = dev_get_drvdata(child);

	u32 enable = ioread32(dev->base_addr + MMSENS_CTRL_OFFSET);
	enable &= CTRL_EN_MASK;

	return sprintf(buf, "%d\n", !!enable);
}

/**
 * Enable device setter.
 */
static ssize_t enable_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmsens *dev = dev_get_drvdata(child);

	u32 ctrl = ioread32(dev->base_addr + MMSENS_CTRL_OFFSET);

	int enable;
	sscanf(buf, "%d", &enable);

	if (!enable) {
		ctrl &= ~CTRL_EN_MASK;
	} else {
		ctrl |= CTRL_EN_MASK;
	}

	iowrite32(ctrl, dev->base_addr + MMSENS_CTRL_OFFSET);

	return count;
}
static DEVICE_ATTR_RW(enable);

/**
 * Current sampling frequency getter.
 */
static ssize_t frequency_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct mmsens *dev = dev_get_drvdata(child);

	u32 frequency = ioread32(dev->base_addr + MMSENS_CTRL_OFFSET);
	frequency &= CTRL_FREQ_MASK;

	if (frequency == FREQ_NORMAL_MASK) {
		return sprintf(buf, "normal\n");
	} else {
		return sprintf(buf, "fast\n");
	}
}

/**
 * Current sampling frequency setter.
 */
static ssize_t frequency_store(struct device *child, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmsens *dev = dev_get_drvdata(child);
	char str_freq[FREQ_LEN];

	u32 ctrl = ioread32(dev->base_addr + MMSENS_CTRL_OFFSET);

	sscanf(buf, "%10s", str_freq);

	if (!strncasecmp(str_freq, "normal", FREQ_LEN)) {
		ctrl &= ~CTRL_FREQ_MASK;
	} else if (!strncasecmp(str_freq, "fast", FREQ_LEN)) {
		ctrl |= CTRL_FREQ_MASK;
	}

	iowrite32(ctrl, dev->base_addr + MMSENS_CTRL_OFFSET);

	return count;
}
static DEVICE_ATTR_RW(frequency);

/**
 * Display available frequency settings.
 */
static ssize_t available_frequencies_show(struct device *child, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "normal fast\n");
}
static DEVICE_ATTR_RO(available_frequencies);

/**
 * Formatted data getter.
 * Data should be 4-digit BCD number.
 */
static ssize_t data_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct mmsens *dev = dev_get_drvdata(child);

	u32 data = ioread32(dev->base_addr + MMSENS_DATA_OFFSET);
	data &= DATA_MASK;

	return sprintf(buf, "%04X\n", data);
}
static DEVICE_ATTR_RO(data);

static struct attribute *mmsens_attrs[] = {
	&dev_attr_interrupt.attr,
	&dev_attr_enable_interrupt.attr,
	&dev_attr_enable.attr,
	&dev_attr_frequency.attr,
	&dev_attr_available_frequencies.attr,
	&dev_attr_data.attr,
	NULL,
};

ATTRIBUTE_GROUPS(mmsens);

/**
 * Interrupt handler.
 */
static irqreturn_t mmsens_isr(int irq, void *data)
{
	struct mmsens *dev = data;

	pr_info("Interrupt received\n");

	iowrite32(0, dev->base_addr + MMSENS_STATUS_OFFSET);

	sysfs_notify(&dev->dev->kobj, NULL, "interrupt");

	return IRQ_HANDLED;
}

/**
 * Device tree match table.
 */
static const struct of_device_id mmsens_of_match[] = {
	{ .compatible = "mistra,mmsens", },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, mmsens_of_match);

/**
 * Initialize driver.
 * Configure character device and class.
 */
static int mmsensdev_setup(struct device *parent)
{
	int ret;
	dev_t devt;
	struct mmsens *dev;

	dev = dev_get_drvdata(parent);

	ret = alloc_chrdev_region(&devt, 0, 1, DEVICE_FILE_NAME);
	if (ret < 0) {
		dev_err(parent, "failed to alloc chrdev region\n");
		goto fail_alloc_chrdev_region;
	}
	dev->devt = devt;

	cdev_init(&dev->cdev, &mmsensdev_fops);
	ret = cdev_add(&dev->cdev, devt, 1);
	if (ret < 0) {
		dev_err(parent, "failed to add cdev\n");
		goto fail_add_cdev;
	}

	mmsens_class = class_create(THIS_MODULE, "mmsens");
	if (!mmsens_class) {
		ret = -EEXIST;
		dev_err(parent, "failed to create class\n");
		goto fail_create_class;
	}

	dev->dev = device_create_with_groups(mmsens_class, parent, devt, dev, mmsens_groups, "%s%d", DEVICE_FILE_NAME, MINOR(devt));
	if (IS_ERR(dev->dev)) {
		dev->dev = NULL;
		ret = -EINVAL;
		dev_err(parent, "failed to create device\n");
		goto fail_create_device;
	}

	return 0;

fail_create_device:
	class_destroy(mmsens_class);
fail_create_class:
	cdev_del(&dev->cdev);
fail_add_cdev:
	unregister_chrdev_region(devt, 1);
fail_alloc_chrdev_region:
	return ret;
}

/**
 * Driver probe function.
 * Configure interrupt and set up driver.
 */
static int mmsens_probe(struct platform_device *pdev)
{
	int ret;
	struct mmsens *dev;
	struct resource *res;
	const struct of_device_id *match;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->parent = &pdev->dev;

	match = of_match_node(mmsens_of_match, pdev->dev.of_node);
	if (!match) {
		dev_err(&pdev->dev, "of_match_node() failed\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->base_addr))
		return PTR_ERR(dev->base_addr);

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "invalid IRQ\n");
		return dev->irq;
	}
	ret = devm_request_irq(&pdev->dev, dev->irq, mmsens_isr, 0, dev_name(&pdev->dev), dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to request IRQ\n");
		return ret;
	}

	/* initialize device */
	ret = mmsensdev_setup(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create device\n");
		return ret;
	}

	return 0;
}

/**
 * Driver remove.
 */
static int mmsens_remove(struct platform_device *pdev)
{
	struct mmsens *dev;

	dev = dev_get_drvdata(&pdev->dev);

	device_destroy(mmsens_class, dev->devt);
	class_destroy(mmsens_class);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->devt, 1);

	return 0;
}

static struct platform_driver mmsens_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mmsens_of_match,
	},
	.probe = mmsens_probe,
	.remove = mmsens_remove,
};

/**
 * Module init.
 */
static int __init mmsens_init(void)
{
	pr_alert("Hello, world!\n");
	return platform_driver_register(&mmsens_driver);
}

/**
 * Module exit.
 */
static void __exit mmsens_exit(void)
{
	pr_alert("Goodbye, world!\n");
	return platform_driver_unregister(&mmsens_driver);
}

module_init(mmsens_init);
module_exit(mmsens_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Memory-mapped sensor Driver and Device");
MODULE_AUTHOR("Strahinja Jankovic");
