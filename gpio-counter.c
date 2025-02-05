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

#undef pr_fmt
/* Preceed all log messages with the module name */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

/* Use an atomic counter instead of a plain unsigned int */
static atomic_t pulse_count;
static int gpio_irq;
static int gpio_pin;  /* GPIO pin will be set via module parameter */
static unsigned int debounce_time_us = 0;  /* Configurable debounce time.  0=no debounce filtering*/

/* Mutex to serialize sysfs access to pulse_count */
static DEFINE_MUTEX(pulse_count_mutex);

/* Module parameter for GPIO pin (read-only) */
module_param(gpio_pin, int, 0444);
MODULE_PARM_DESC(gpio_pin, "GPIO pin number for pulse counting (read-only)");

/* Module parameter for debounce time, 0 = no debounce (read-only) */
module_param(debounce_time_us, uint, 0444);
MODULE_PARM_DESC(debounce_time_us, "Debounce time in microseconds (0 = no debounce)");

/* 
 * Interrupt handler
 *
 * The counter is updated with an atomic operation so that no spinlock is needed 
 * in the IRQ context.
 */
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = jiffies;

    /* Debounce logic using configurable microsecond threshold */
    if (debounce_time_us > 0) {
        unsigned long debounce_jiffies = usecs_to_jiffies(debounce_time_us);
        
        if (debounce_jiffies > 0 && 
            interrupt_time - last_interrupt_time < debounce_jiffies) {
            return IRQ_HANDLED;
        }
        last_interrupt_time = interrupt_time;
    }

    /* Atomically increment the pulse count */
    atomic_inc(&pulse_count);

    return IRQ_HANDLED;
}

/*
 * Sysfs show function (read)
 *
 * We use a mutex to serialize access so that if additional operations are added later,
 * the sysfs routines do not interleave unexpectedly.
 */
static ssize_t pulse_count_show(struct kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
    unsigned int count;

    mutex_lock(&pulse_count_mutex);
    count = atomic_read(&pulse_count);
    mutex_unlock(&pulse_count_mutex);

    return sprintf(buf, "%u\n", count);
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
    unsigned int val;

    if (kstrtouint(buf, 10, &val) < 0) {
        pr_err("Invalid input\n");
        return -EINVAL;
    }

    mutex_lock(&pulse_count_mutex);
    if (val == 0) {
        atomic_set(&pulse_count, 0);
        pr_info("Pulse count reset to 0\n");
    } else {
        pr_info("Write ignored. To reset, write 0\n");
    }
    mutex_unlock(&pulse_count_mutex);

    return count;
}

/* Define the sysfs attribute for pulse_count */
static struct kobj_attribute pulse_count_attr =
    __ATTR(pulse_count, 0644, pulse_count_show, pulse_count_store);

static struct attribute *attrs[] = {
    &pulse_count_attr.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

static struct kobject *pulse_kobj;

/* Module initialization */
static int __init gpio_counter_init(void)
{
    int ret;
    /* Validate GPIO pin */
    if (gpio_pin <= 100) {
        pr_err("Invalid GPIO pin number: %d\n", gpio_pin);
        pr_err("cat /sys/kernel/debug/gpio for kernel GPIO number");
        return -EINVAL;
    }

    /* Initialize the atomic counter to 0 */
    atomic_set(&pulse_count, 0);

    /* Request GPIO */
    ret = gpio_request(gpio_pin, "gpio-counter");
    if (ret) {
        pr_err("Failed to request GPIO %d: error %d\n", gpio_pin, ret);
        return ret;
    }

    /* Set GPIO as input */
    ret = gpio_direction_input(gpio_pin);
    if (ret) {
        pr_err("Failed to set GPIO %d as input: error %d\n", gpio_pin, ret);
        goto cleanup_gpio;
    }

    /* Request interrupt on the rising edge */
    gpio_irq = gpio_to_irq(gpio_pin);
    ret = request_irq(gpio_irq, gpio_irq_handler, IRQF_TRIGGER_RISING,
                      "gpio-counter", NULL);
    if (ret) {
        pr_err("Failed to request IRQ for GPIO %d: error %d\n", gpio_pin, ret);
        goto cleanup_gpio;
    }

    /* Create sysfs entry under /sys/kernel/ */
    pulse_kobj = kobject_create_and_add("gpio-counter", kernel_kobj);
    if (!pulse_kobj) {
        pr_err("Failed to create sysfs kobject\n");
        ret = -ENOMEM;
        goto cleanup_irq;
    }

    ret = sysfs_create_group(pulse_kobj, &attr_group);
    if (ret) {
        pr_err("Failed to create sysfs group: error %d\n", ret);
        goto cleanup_kobject;
    }

    pr_info("Pulse counter module loaded with GPIO pin %d\n", gpio_pin);
    return 0;

cleanup_kobject:
    kobject_put(pulse_kobj);
cleanup_irq:
    free_irq(gpio_irq, NULL);
cleanup_gpio:
    gpio_free(gpio_pin);
    return ret;
}

/* Module cleanup */
static void __exit gpio_counter_exit(void)
{
    sysfs_remove_group(pulse_kobj, &attr_group);
    kobject_put(pulse_kobj);
    free_irq(gpio_irq, NULL);
    gpio_free(gpio_pin);
    pr_info("Pulse counter module unloaded\n");
}

module_init(gpio_counter_init);
module_exit(gpio_counter_exit);

MODULE_ALIAS("gpio-counter");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Richard Zimmerman <rszimm@gmail.com>");
MODULE_DESCRIPTION("A thread-safe GPIO pulse counter kernel module with configurable GPIO pin");
MODULE_VERSION("0.1");
