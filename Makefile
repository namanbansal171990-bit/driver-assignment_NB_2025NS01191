obj-m += adc16_dma.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
// SPDX-License-Identifier: GPL-2.0
/*
 * adc16_dma.c - Linux Character + DMA Driver for 16-bit 4-Channel High-Speed ADC
 * Target: ARM Cortex-A53 SoC (e.g., NXP i.MX8M Plus) over SPI + DMA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/ioctl.h>

#define DRIVER_NAME         "adc16_dma"
#define ADC_RING_SIZE       (64 * 1024)   /* 64 KB coherent DMA ring buffer */
#define ADC_SAMPLE_BYTES    8             /* 4 channels x 16-bit (2 bytes) */

/* IOCTL Definitions */
#define ADC_IOC_MAGIC       'A'
#define ADC_IOC_START       _IO(ADC_IOC_MAGIC, 1)
#define ADC_IOC_STOP        _IO(ADC_IOC_MAGIC, 2)
#define ADC_IOC_GET_STATUS  _IOR(ADC_IOC_MAGIC, 3, u32)

struct adc_dev {
    struct spi_device *spi;
    struct cdev cdev;
    struct device *chrdev;
    dev_t devt;

    /* Hardware resources */
    struct gpio_desc *drdy_gpio;
    int drdy_irq;
    struct dma_chan *rx_dma;

    /* Coherent DMA Ring Buffer */
    void *dma_buf;
    dma_addr_t dma_handle;
    size_t buf_size;

    /* Ring indices & synchronization */
    spinlock_t lock;
    size_t head;            /* Written by DMA / ISR */
    size_t tail;            /* Read by user space */
    bool running;
    bool overflow;
    wait_queue_head_t read_queue;
};

static struct class *adc_class;
static dev_t adc_base_devt;

/* Helper: calculate unread bytes in circular buffer */
static inline size_t adc_ring_available(struct adc_dev *adc)
{
    return (adc->head - adc->tail) & (adc->buf_size - 1);
}

/* 1. DMA Completion Callback (Bottom-Half / Tasklet Context) */
static void adc_dma_complete_cb(void *param)
{
    struct adc_dev *adc = param;
    unsigned long flags;
    size_t next_head;

    spin_lock_irqsave(&adc->lock, flags);

    next_head = (adc->head + ADC_SAMPLE_BYTES) & (adc->buf_size - 1);
    if (next_head == adc->tail) {
        /* Buffer full: flag overrun and drop oldest/newest boundary */
        adc->overflow = true;
    } else {
        adc->head = next_head;
    }

    spin_unlock_irqrestore(&adc->lock, flags);

    /* Wake up any sleeping read() or poll() callers */
    wake_up_interruptible(&adc->read_queue);
}

/* 2. Top-Half DRDY GPIO Interrupt Handler */
static irqreturn_t adc_drdy_isr(int irq, void *dev_id)
{
    struct adc_dev *adc = dev_id;
    struct dma_async_tx_descriptor *desc;
    dma_addr_t dst_addr;

    if (!adc->running)
        return IRQ_NONE;

    dst_addr = adc->dma_handle + adc->head;

    /* Prepare single-frame DMA transfer from SPI RX FIFO to coherent ring */
    desc = dmaengine_prep_slave_single(adc->rx_dma, dst_addr,
                                       ADC_SAMPLE_BYTES,
                                       DMA_DEV_TO_MEM,
                                       DMA_PREP_INTERRUPT);
    if (!desc)
        return IRQ_HANDLED;

    desc->callback = adc_dma_complete_cb;
    desc->callback_param = adc;

    dmaengine_submit(desc);
    dma_async_issue_pending(adc->rx_dma);

    return IRQ_HANDLED;
}

/* 3. Character File Operations */
static int adc_open(struct inode *inode, struct file *filp)
{
    struct adc_dev *adc = container_of(inode->i_cdev, struct adc_dev, cdev);
    filp->private_data = adc;
    return 0;
}

static ssize_t adc_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos)
{
    struct adc_dev *adc = filp->private_data;
    unsigned long flags;
    size_t avail, chunk;
    int ret;

    if (count < ADC_SAMPLE_BYTES)
        return -EINVAL;

    /* Block until samples arrive unless O_NONBLOCK is set */
    if (adc_ring_available(adc) == 0) {
        if (filp->f_flags & O_NONBLOCK)
            return -EAGAIN;

        ret = wait_event_interruptible(adc->read_queue,
                                       adc_ring_available(adc) > 0 || !adc->running);
        if (ret)
            return ret;
    }

    spin_lock_irqsave(&adc->lock, flags);
    if (adc->overflow) {
        adc->overflow = false;
        spin_unlock_irqrestore(&adc->lock, flags);
        return -EOVERFLOW;
    }

    avail = adc_ring_available(adc);
    chunk = min(count, avail);
    /* Clamp to end of physical ring before wrap-around */
    chunk = min(chunk, adc->buf_size - adc->tail);
    spin_unlock_irqrestore(&adc->lock, flags);

    if (copy_to_user(buf, (u8 *)adc->dma_buf + adc->tail, chunk))
        return -EFAULT;

    spin_lock_irqsave(&adc->lock, flags);
    adc->tail = (adc->tail + chunk) & (adc->buf_size - 1);
    spin_unlock_irqrestore(&adc->lock, flags);

    return chunk;
}

static __poll_t adc_poll(struct file *filp, poll_table *wait)
{
    struct adc_dev *adc = filp->private_data;
    __poll_t mask = 0;

    poll_wait(filp, &adc->read_queue, wait);

    if (adc_ring_available(adc) > 0)
        mask |= EPOLLIN | EPOLLRDNORM;
    if (adc->overflow)
        mask |= EPOLLERR;

    return mask;
}

/* Zero-copy mapping of coherent DMA buffer into user space */
static int adc_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct adc_dev *adc = filp->private_data;
    size_t req_size = vma->vm_end - vma->vm_start;

    if (req_size > adc->buf_size)
        return -EINVAL;

    /* Enforce read-only mapping so user space cannot corrupt DMA memory */
    if (vma->vm_flags & VM_WRITE)
        return -EPERM;

    return dma_mmap_coherent(&adc->spi->dev, vma,
                             adc->dma_buf, adc->dma_handle, req_size);
}

static long adc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct adc_dev *adc = filp->private_data;
    unsigned long flags;
    u32 status;

    switch (cmd) {
    case ADC_IOC_START:
        spin_lock_irqsave(&adc->lock, flags);
        adc->head = 0;
        adc->tail = 0;
        adc->overflow = false;
        adc->running = true;
        spin_unlock_irqrestore(&adc->lock, flags);
        enable_irq(adc->drdy_irq);
        return 0;

    case ADC_IOC_STOP:
        disable_irq(adc->drdy_irq);
        dmaengine_terminate_sync(adc->rx_dma);
        spin_lock_irqsave(&adc->lock, flags);
        adc->running = false;
        spin_unlock_irqrestore(&adc->lock, flags);
        wake_up_interruptible(&adc->read_queue);
        return 0;

    case ADC_IOC_GET_STATUS:
        status = (adc->running ? 0x1 : 0x0) | (adc->overflow ? 0x2 : 0x0);
        if (copy_to_user((u32 __user *)arg, &status, sizeof(status)))
            return -EFAULT;
        return 0;

    default:
        return -ENOTTY;
    }
}

static const struct file_operations adc_fops = {
    .owner          = THIS_MODULE,
    .open           = adc_open,
    .read           = adc_read,
    .poll           = adc_poll,
    .mmap           = adc_mmap,
    .unlocked_ioctl = adc_ioctl,
};

/* 4. SPI Platform Probe & Remove */
static int adc_spi_probe(struct spi_device *spi)
{
    struct adc_dev *adc;
    int ret;

    adc = devm_kzalloc(&spi->dev, sizeof(*adc), GFP_KERNEL);
    if (!adc)
        return -ENOMEM;

    adc->spi = spi;
    adc->buf_size = ADC_RING_SIZE;
    spin_lock_init(&adc->lock);
    init_waitqueue_head(&adc->read_queue);
    spi_set_drvdata(spi, adc);

    /* Configure 64-bit DMA mask and allocate coherent ring buffer */
    ret = dma_set_mask_and_coherent(&spi->dev, DMA_BIT_MASK(64));
    if (ret)
        return ret;

    adc->dma_buf = dma_alloc_coherent(&spi->dev, adc->buf_size,
                                      &adc->dma_handle, GFP_KERNEL);
    if (!adc->dma_buf)
        return -ENOMEM;

    /* Request RX DMA channel from Device Tree ("dmas" property) */
    adc->rx_dma = dma_request_chan(&spi->dev, "rx");
    if (IS_ERR(adc->rx_dma)) {
        ret = PTR_ERR(adc->rx_dma);
        goto err_free_dma;
    }

    /* Request DRDY GPIO & bind falling-edge IRQ (disabled until START ioctl) */
    adc->drdy_gpio = devm_gpiod_get(&spi->dev, "drdy", GPIOD_IN);
    if (IS_ERR(adc->drdy_gpio)) {
        ret = PTR_ERR(adc->drdy_gpio);
        goto err_release_chan;
    }

    adc->drdy_irq = gpiod_to_irq(adc->drdy_gpio);
    ret = devm_request_irq(&spi->dev, adc->drdy_irq, adc_drdy_isr,
                           IRQF_TRIGGER_FALLING | IRQF_NO_AUTOEN,
                           DRIVER_NAME, adc);
    if (ret)
        goto err_release_chan;

    /* Register character device node (/dev/adc0) */
    adc->devt = MKDEV(MAJOR(adc_base_devt), 0);
    cdev_init(&adc->cdev, &adc_fops);
    adc->cdev.owner = THIS_MODULE;

    ret = cdev_add(&adc->cdev, adc->devt, 1);
    if (ret)
        goto err_release_chan;

    adc->chrdev = device_create(adc_class, &spi->dev, adc->devt, adc, "adc0");
    if (IS_ERR(adc->chrdev)) {
        ret = PTR_ERR(adc->chrdev);
        goto err_del_cdev;
    }

    dev_info(&spi->dev, "adc16_dma probed: 64KB DMA ring at %pad\n", &adc->dma_handle);
    return 0;

err_del_cdev:
    cdev_del(&adc->cdev);
err_release_chan:
    dma_release_channel(adc->rx_dma);
err_free_dma:
    dma_free_coherent(&spi->dev, adc->buf_size, adc->dma_buf, adc->dma_handle);
    return ret;
}

static void adc_spi_remove(struct spi_device *spi)
{
    struct adc_dev *adc = spi_get_drvdata(spi);

    if (adc->running)
        disable_irq(adc->drdy_irq);

    dmaengine_terminate_sync(adc->rx_dma);
    device_destroy(adc_class, adc->devt);
    cdev_del(&adc->cdev);
    dma_release_channel(adc->rx_dma);
    dma_free_coherent(&spi->dev, adc->buf_size, adc->dma_buf, adc->dma_handle);
}

static const struct of_device_id adc_of_match[] = {
    { .compatible = "vendor,adc16-dma" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adc_of_match);

static struct spi_driver adc_spi_driver = {
    .driver = {
        .name           = DRIVER_NAME,
        .of_match_table = adc_of_match,
    },
