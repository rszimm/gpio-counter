#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/delay.h>

// NOTE: This module only does rising edge counting.  If you need falling edge or both edges,
// you can modify the IRQF_TRIGGER_RISING flag in request_irq() to IRQF_TRIGGER_FALLING or
// IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING respectively.  Making this configurable is a likely
// future enhancement.

#undef pr_fmt
/* Preceed all log messages with the module name */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

/* Maximum number of monitoring channels */
#define MAX_CHANNELS 10

static int gpio_pins[MAX_CHANNELS];  /* GPIO pin will be set via module parameter */
static int num_gpio_pins = 0; /* Number of GPIO pins to monitor, default 0 */
static unsigned int debounce_time_us = 0;  /* Configurable debounce time.  0=no debounce filtering*/
static unsigned long debounce_jiffies = 0; /* Computed from debounce_time_us */

/* Module parameter for GPIO pin (read-only) */
module_param_array(gpio_pins, int, &num_gpio_pins, 0444);
MODULE_PARM_DESC(gpio_pins, "Comma-separated list of GPIO pin numbers (up to " __stringify(MAX_CHANNELS) "). Read-only.");

/* Module parameter for debounce time, 0 = no debounce (read-only) */
module_param(debounce_time_us, uint, 0444);
MODULE_PARM_DESC(debounce_time_us, "Debounce time in microseconds (0 = no debounce)");

/* ===== Per-channel state ===== */
struct gc_channel {
    atomic_t        pulse_count;
    int             gpio_pin;
    int             gpio_irq;
    unsigned long   last_interrupt_time; /* for debounce */
    struct kobject *kobj;                 /* /sys/kernel/gpio-counter/<idx> */
    struct mutex    lock;                 /* serialize sysfs R/W (future-safe) */
    spinlock_t      irq_lock;             /* for interrupt context */
};

static struct gc_channel channels[MAX_CHANNELS];


/* 
 * Interrupt handler
 *
 * The counter is updated with an atomic operation so that no spinlock is needed 
 * in the IRQ context.  However, the debounce logic needs a lock to be fully
 * correct if multiple interrupts arrive very close together on a multi-core machine.
 */
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    struct gc_channel *ch = dev_id;

    /* Debounce logic using configurable microsecond threshold */
    if (debounce_jiffies > 0) {
        unsigned long flags, interrupt_time;
        spin_lock_irqsave(&ch->irq_lock, flags);
        interrupt_time = jiffies;
        
        if (time_before(interrupt_time, ch->last_interrupt_time + debounce_jiffies)) {
            spin_unlock_irqrestore(&ch->irq_lock, flags);
            return IRQ_HANDLED;
        }
        ch->last_interrupt_time = interrupt_time;
        spin_unlock_irqrestore(&ch->irq_lock, flags);
    }

    /* Atomically increment the pulse count */
    atomic_inc(&ch->pulse_count);

    return IRQ_HANDLED;
}

/* sysfs: per-channel pulse_count attribute */
struct gc_attr {
    struct kobj_attribute attr;
    int index; 
};

/*
 * Sysfs show function (read)
 *
 * We use a mutex to serialize access so that if additional operations are added later,
 * the sysfs routines do not interleave unexpectedly.
 */
static ssize_t pulse_count_show(struct kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
    struct gc_attr *ga = container_of(attr, struct gc_attr, attr);
    struct gc_channel *ch = &channels[ga->index];
    unsigned int count;

    mutex_lock(&ch->lock);
    count = atomic_read(&ch->pulse_count);
    mutex_unlock(&ch->lock);

    return sysfs_emit(buf, "%u\n", count);
}

/*
 * Sysfs store function (write)
 *
 * Writing a 0 will reset the counter; any other value is ignored.
 * The operation is serialized by the mutex.
 */
static ssize_t pulse_count_store(struct kobject *kobj,
                                 struct kobj_attribute *attr, const char *buf, size_t count)
{
    struct gc_attr *ga = container_of(attr, struct gc_attr, attr);
    struct gc_channel *ch = &channels[ga->index];
    unsigned int val;

    if (kstrtouint(buf, 10, &val) < 0) {
        pr_err("Invalid input\n");
        return -EINVAL;
    }

    mutex_lock(&ch->lock);
    if (val == 0) {
        atomic_set(&ch->pulse_count, 0);
        pr_info("Pulse count reset to 0\n");
    } else {
        pr_info("Write ignored. To reset, write 0\n");
    }
    mutex_unlock(&ch->lock);

    return count;
}

/*
 * Sysfs show function (read) for gpio_pin attribute
 */
static ssize_t gpio_pin_show(struct kobject *kobj,
                             struct kobj_attribute *attr, char *buf)
{
    struct gc_attr *ga = container_of(attr, struct gc_attr, attr);
    struct gc_channel *ch = &channels[ga->index];
    return sysfs_emit(buf, "%d\n", ch->gpio_pin);
}

/* Sysfs attribute group */
static struct kobject *root_kobj;
static struct gc_attr pulse_attrs[MAX_CHANNELS];
static struct gc_attr gpio_attrs[MAX_CHANNELS];
static struct attribute *chan_attrs[MAX_CHANNELS][3]; /* Just two attributes/files [pulse_count, gpio_pin, NULL] */
static struct attribute_group chan_groups[MAX_CHANNELS];

/* Module initialization */
static int __init gpio_counter_init(void)
{
    int i, ret;

    if (num_gpio_pins <= 0) {
        pr_err("No GPIO pins specified. Example: insmod gpio-counter.ko gpio_pins=423,424\n");
        return -EINVAL;
    }
    if (num_gpio_pins > MAX_CHANNELS) {
        pr_err("Too many GPIO pins (%d). Max is %d.\n", num_gpio_pins, MAX_CHANNELS);
        return -EINVAL;
    }

    /* Validate GPIO pins (usually it's someone trying to use old GPIO numbers) */
    for (i = 0; i < num_gpio_pins; i++) {
        if (!gpio_is_valid(gpio_pins[i])) {
            pr_err("Invalid GPIO pin number at index %d: %d\n", i, gpio_pins[i]);
            pr_err("cat /sys/kernel/debug/gpio to inspect valid kernel GPIO numbers\n");
            return -EINVAL;
        }
    }

    /* Create /sys/kernel/gpio-counter root */
    root_kobj = kobject_create_and_add("gpio-counter", kernel_kobj);
    if (!root_kobj) {
        pr_err("Failed to create root kobject\n");
        return -ENOMEM;
    }

    /* Initialize channels and sysfs per-channel subtree */
    for (i = 0; i < num_gpio_pins; i++) {
        struct gc_channel *ch = &channels[i];
        char name[8]; /* "0".."9" */

        ch->gpio_pin = gpio_pins[i];
        ch->gpio_irq = -1;
        ch->last_interrupt_time = 0;
        mutex_init(&ch->lock);
        spin_lock_init(&ch->irq_lock);
        atomic_set(&ch->pulse_count, 0);

        /* Request and configure GPIO */
        ret = gpio_request(ch->gpio_pin, "gpio-counter");
        if (ret) {
            pr_err("GPIO request failed for pin %d (idx %d): %d\n",
                   ch->gpio_pin, i, ret);
            goto err_unwind;
        }

        ret = gpio_direction_input(ch->gpio_pin);
        if (ret) {
            pr_err("Failed to set GPIO %d as input: %d\n", ch->gpio_pin, ret);
            goto err_unwind;
        }

        /* IRQ setup */
        ch->gpio_irq = gpio_to_irq(ch->gpio_pin);
        if (ch->gpio_irq < 0) {
            ret = ch->gpio_irq;
            pr_err("gpio_to_irq failed for GPIO %d: %d\n", ch->gpio_pin, ret);
            goto err_unwind;
        }

        ret = request_irq(ch->gpio_irq, gpio_irq_handler,
                          IRQF_TRIGGER_RISING, "gpio-counter", ch);
        if (ret) {
            pr_err("request_irq failed for GPIO %d (irq %d): %d\n",
                   ch->gpio_pin, ch->gpio_irq, ret);
            goto err_unwind;
        }

        /* Create /sys/kernel/gpio-counter/<i>/ */
        snprintf(name, sizeof(name), "%d", i);
        ch->kobj = kobject_create_and_add(name, root_kobj);
        if (!ch->kobj) {
            pr_err("Failed to create kobject for channel %d\n", i);
            ret = -ENOMEM;
            goto err_unwind;
        }

        /* Prepare per-channel pulse_count instance */
        pulse_attrs[i].index = i;
        sysfs_attr_init(&pulse_attrs[i].attr.attr);
        pulse_attrs[i].attr.attr.name = "pulse_count";
        pulse_attrs[i].attr.attr.mode = 0644;
        pulse_attrs[i].attr.show  = pulse_count_show;
        pulse_attrs[i].attr.store = pulse_count_store;

        /* Prepare per-channel gpio_pin instance */
        gpio_attrs[i].index = i;
        sysfs_attr_init(&gpio_attrs[i].attr.attr);
        gpio_attrs[i].attr.attr.name = "gpio_pin";
        gpio_attrs[i].attr.attr.mode = 0444;
        gpio_attrs[i].attr.show  = gpio_pin_show;
        gpio_attrs[i].attr.store = NULL;

        /* Build a tiny attribute group per channel */
        chan_attrs[i][0] = &pulse_attrs[i].attr.attr;
        chan_attrs[i][1] = &gpio_attrs[i].attr.attr;
        chan_attrs[i][2] = NULL;
        chan_groups[i].attrs = chan_attrs[i];

        ret = sysfs_create_group(ch->kobj, &chan_groups[i]);
        if (ret) {
            pr_err("Failed to create sysfs group for channel %d: %d\n", i, ret);
            goto err_unwind;
        }

        pr_info("Channel %d online: GPIO %d -> /sys/kernel/gpio-counter/%d/pulse_count\n",
                i, ch->gpio_pin, i);
    }
    debounce_jiffies = usecs_to_jiffies(debounce_time_us);
    pr_info("Loaded %d channel(s). Debounce: %u us\n", num_gpio_pins, debounce_time_us);
    return 0;
err_unwind:
    /* Clean up anything we successfully created up to i (inclusive). */
    while (--i >= 0) {
        struct gc_channel *ch = &channels[i];

        if (ch->kobj) {
            sysfs_remove_group(ch->kobj, &chan_groups[i]);
            kobject_put(ch->kobj);
            ch->kobj = NULL;
        }
        if (ch->gpio_irq >= 0) {
            free_irq(ch->gpio_irq, ch);
            ch->gpio_irq = -1;
        }
        if (gpio_is_valid(ch->gpio_pin))
            gpio_free(ch->gpio_pin);
    }

    if (root_kobj) {
        kobject_put(root_kobj);
        root_kobj = NULL;
    }
    return ret;
}

/* Module cleanup */
static void __exit gpio_counter_exit(void)
{
    int i;

    for (i = 0; i < num_gpio_pins; i++) {
        struct gc_channel *ch = &channels[i];

        if (ch->kobj) {
            sysfs_remove_group(ch->kobj, &chan_groups[i]);
            kobject_put(ch->kobj);
            ch->kobj = NULL;
        }
        if (ch->gpio_irq >= 0) {
            free_irq(ch->gpio_irq, ch);
            ch->gpio_irq = -1;
        }
        if (gpio_is_valid(ch->gpio_pin))
            gpio_free(ch->gpio_pin);
    }

    if (root_kobj) {
        kobject_put(root_kobj);
        root_kobj = NULL;
    }

    pr_info("Pulse counter module unloaded\n");
}

module_init(gpio_counter_init);
module_exit(gpio_counter_exit);

MODULE_ALIAS("gpio-counter");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Richard Zimmerman <rszimm@gmail.com>");
MODULE_DESCRIPTION("A thread-safe GPIO pulse counter kernel module with configurable GPIO pins");
MODULE_VERSION("0.2");
