// SPDX-License-Identifier: GPL-2.0

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/resource.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/*
 * There are 3 YU GPIO blocks:
 * gpio[0]: HOST_GPIO0->HOST_GPIO31
 * gpio[1]: HOST_GPIO32->HOST_GPIO63
 * gpio[2]: HOST_GPIO64->HOST_GPIO69
 */
#define MLXBF2_GPIO_MAX_PINS_PER_BLOCK 32

/*
 * arm_gpio_lock register:
 * bit[31]	lock status: active if set
 * bit[15:0]	set lock
 * The lock is enabled only if 0xd42f is written to this field
 */
#define YU_ARM_GPIO_LOCK_ADDR		0x2801088
#define YU_ARM_GPIO_LOCK_SIZE		0x8
#define YU_LOCK_ACTIVE_BIT(val)		(val >> 31)
#define YU_ARM_GPIO_LOCK_ACQUIRE	0xd42f
#define YU_ARM_GPIO_LOCK_RELEASE	0x0

/*
 * gpio[x] block registers and their offset
 */
#define YU_GPIO_DATAIN			0x04
#define YU_GPIO_MODE1			0x08
#define YU_GPIO_MODE0			0x0c
#define YU_GPIO_DATASET			0x14
#define YU_GPIO_DATACLEAR		0x18
#define YU_GPIO_MODE1_CLEAR		0x50
#define YU_GPIO_MODE0_SET		0x54
#define YU_GPIO_MODE0_CLEAR		0x58

struct mlxbf2_gpio_context_save_regs {
	u32 gpio_mode0;
	u32 gpio_mode1;
};

/* BlueField-2 gpio block context structure. */
struct mlxbf2_gpio_context {
	struct gpio_chip gc;

	/* YU GPIO blocks address */
	void __iomem *gpio_io;

	struct mlxbf2_gpio_context_save_regs *csave_regs;
};

/* BlueField-2 gpio shared structure. */
struct mlxbf2_gpio_param {
	void __iomem *io;
	struct resource *res;
	struct mutex *lock;
};

static struct resource yu_arm_gpio_lock_res =
	DEFINE_RES_MEM_NAMED(YU_ARM_GPIO_LOCK_ADDR, YU_ARM_GPIO_LOCK_SIZE, "YU_ARM_GPIO_LOCK");

static DEFINE_MUTEX(yu_arm_gpio_lock_mutex);

static struct mlxbf2_gpio_param yu_arm_gpio_lock_param = {
	.res = &yu_arm_gpio_lock_res,
	.lock = &yu_arm_gpio_lock_mutex,
};

/* Request memory region and map yu_arm_gpio_lock resource */
static int mlxbf2_gpio_get_lock_res(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	resource_size_t size;
	int ret = 0;

	mutex_lock(yu_arm_gpio_lock_param.lock);

	/* Check if the memory map already exists */
	if (yu_arm_gpio_lock_param.io)
		goto exit;

	res = yu_arm_gpio_lock_param.res;
	size = resource_size(res);

	if (!devm_request_mem_region(dev, res->start, size, res->name)) {
		ret = -EFAULT;
		goto exit;
	}

	yu_arm_gpio_lock_param.io = devm_ioremap(dev, res->start, size);
	if (!yu_arm_gpio_lock_param.io)
		ret = -ENOMEM;

exit:
	mutex_unlock(yu_arm_gpio_lock_param.lock);

	return ret;
}

/*
 * Acquire the YU arm_gpio_lock to be able to change the direction
 * mode. If the lock_active bit is already set, return an error.
 */
static int mlxbf2_gpio_lock_acquire(struct mlxbf2_gpio_context *gs)
{
	u32 arm_gpio_lock_val;

	mutex_lock(yu_arm_gpio_lock_param.lock);
	raw_spin_lock(&gs->gc.bgpio_lock);

	arm_gpio_lock_val = pete_readl("drivers/gpio/gpio-mlxbf2.c:125", yu_arm_gpio_lock_param.io);

	/*
	 * When lock active bit[31] is set, ModeX is write enabled
	 */
	if (YU_LOCK_ACTIVE_BIT(arm_gpio_lock_val)) {
		raw_spin_unlock(&gs->gc.bgpio_lock);
		mutex_unlock(yu_arm_gpio_lock_param.lock);
		return -EINVAL;
	}

	pete_writel("drivers/gpio/gpio-mlxbf2.c:136", YU_ARM_GPIO_LOCK_ACQUIRE, yu_arm_gpio_lock_param.io);

	return 0;
}

/*
 * Release the YU arm_gpio_lock after changing the direction mode.
 */
static void mlxbf2_gpio_lock_release(struct mlxbf2_gpio_context *gs)
	__releases(&gs->gc.bgpio_lock)
	__releases(yu_arm_gpio_lock_param.lock)
{
	pete_writel("drivers/gpio/gpio-mlxbf2.c:148", YU_ARM_GPIO_LOCK_RELEASE, yu_arm_gpio_lock_param.io);
	raw_spin_unlock(&gs->gc.bgpio_lock);
	mutex_unlock(yu_arm_gpio_lock_param.lock);
}

/*
 * mode0 and mode1 are both locked by the gpio_lock field.
 *
 * Together, mode0 and mode1 define the gpio Mode dependeing also
 * on Reg_DataOut.
 *
 * {mode1,mode0}:{Reg_DataOut=0,Reg_DataOut=1}->{DataOut=0,DataOut=1}
 *
 * {0,0}:Reg_DataOut{0,1}->{Z,Z} Input PAD
 * {0,1}:Reg_DataOut{0,1}->{0,1} Full drive Output PAD
 * {1,0}:Reg_DataOut{0,1}->{0,Z} 0-set PAD to low, 1-float
 * {1,1}:Reg_DataOut{0,1}->{Z,1} 0-float, 1-set PAD to high
 */

/*
 * Set input direction:
 * {mode1,mode0} = {0,0}
 */
static int mlxbf2_gpio_direction_input(struct gpio_chip *chip,
				       unsigned int offset)
{
	struct mlxbf2_gpio_context *gs = gpiochip_get_data(chip);
	int ret;

	/*
	 * Although the arm_gpio_lock was set in the probe function, check again
	 * if it is still enabled to be able to write to the ModeX registers.
	 */
	ret = mlxbf2_gpio_lock_acquire(gs);
	if (ret < 0)
		return ret;

	pete_writel("drivers/gpio/gpio-mlxbf2.c:185", BIT(offset), gs->gpio_io + YU_GPIO_MODE0_CLEAR);
	pete_writel("drivers/gpio/gpio-mlxbf2.c:186", BIT(offset), gs->gpio_io + YU_GPIO_MODE1_CLEAR);

	mlxbf2_gpio_lock_release(gs);

	return ret;
}

/*
 * Set output direction:
 * {mode1,mode0} = {0,1}
 */
static int mlxbf2_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset,
					int value)
{
	struct mlxbf2_gpio_context *gs = gpiochip_get_data(chip);
	int ret = 0;

	/*
	 * Although the arm_gpio_lock was set in the probe function,
	 * check again it is still enabled to be able to write to the
	 * ModeX registers.
	 */
	ret = mlxbf2_gpio_lock_acquire(gs);
	if (ret < 0)
		return ret;

	pete_writel("drivers/gpio/gpio-mlxbf2.c:213", BIT(offset), gs->gpio_io + YU_GPIO_MODE1_CLEAR);
	pete_writel("drivers/gpio/gpio-mlxbf2.c:214", BIT(offset), gs->gpio_io + YU_GPIO_MODE0_SET);

	mlxbf2_gpio_lock_release(gs);

	return ret;
}

/* BlueField-2 GPIO driver initialization routine. */
static int
mlxbf2_gpio_probe(struct platform_device *pdev)
{
	struct mlxbf2_gpio_context *gs;
	struct device *dev = &pdev->dev;
	struct gpio_chip *gc;
	unsigned int npins;
	int ret;

	gs = devm_kzalloc(dev, sizeof(*gs), GFP_KERNEL);
	if (!gs)
		return -ENOMEM;

	/* YU GPIO block address */
	gs->gpio_io = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gs->gpio_io))
		return PTR_ERR(gs->gpio_io);

	ret = mlxbf2_gpio_get_lock_res(pdev);
	if (ret) {
		dev_err(dev, "Failed to get yu_arm_gpio_lock resource\n");
		return ret;
	}

	if (device_property_read_u32(dev, "npins", &npins))
		npins = MLXBF2_GPIO_MAX_PINS_PER_BLOCK;

	gc = &gs->gc;

	ret = bgpio_init(gc, dev, 4,
			gs->gpio_io + YU_GPIO_DATAIN,
			gs->gpio_io + YU_GPIO_DATASET,
			gs->gpio_io + YU_GPIO_DATACLEAR,
			NULL,
			NULL,
			0);

	if (ret) {
		dev_err(dev, "bgpio_init failed\n");
		return ret;
	}

	gc->direction_input = mlxbf2_gpio_direction_input;
	gc->direction_output = mlxbf2_gpio_direction_output;
	gc->ngpio = npins;
	gc->owner = THIS_MODULE;

	platform_set_drvdata(pdev, gs);

	ret = devm_gpiochip_add_data(dev, &gs->gc, gs);
	if (ret) {
		dev_err(dev, "Failed adding memory mapped gpiochip\n");
		return ret;
	}

	return 0;
}

static int __maybe_unused mlxbf2_gpio_suspend(struct device *dev)
{
	struct mlxbf2_gpio_context *gs = dev_get_drvdata(dev);

	gs->csave_regs->gpio_mode0 = pete_readl("drivers/gpio/gpio-mlxbf2.c:284", gs->gpio_io +
		YU_GPIO_MODE0);
	gs->csave_regs->gpio_mode1 = pete_readl("drivers/gpio/gpio-mlxbf2.c:286", gs->gpio_io +
		YU_GPIO_MODE1);

	return 0;
}

static int __maybe_unused mlxbf2_gpio_resume(struct device *dev)
{
	struct mlxbf2_gpio_context *gs = dev_get_drvdata(dev);

	pete_writel("drivers/gpio/gpio-mlxbf2.c:296", gs->csave_regs->gpio_mode0, gs->gpio_io +
		YU_GPIO_MODE0);
	pete_writel("drivers/gpio/gpio-mlxbf2.c:298", gs->csave_regs->gpio_mode1, gs->gpio_io +
		YU_GPIO_MODE1);

	return 0;
}
static SIMPLE_DEV_PM_OPS(mlxbf2_pm_ops, mlxbf2_gpio_suspend, mlxbf2_gpio_resume);

static const struct acpi_device_id __maybe_unused mlxbf2_gpio_acpi_match[] = {
	{ "MLNXBF22", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, mlxbf2_gpio_acpi_match);

static struct platform_driver mlxbf2_gpio_driver = {
	.driver = {
		.name = "mlxbf2_gpio",
		.acpi_match_table = mlxbf2_gpio_acpi_match,
		.pm = &mlxbf2_pm_ops,
	},
	.probe    = mlxbf2_gpio_probe,
};

module_platform_driver(mlxbf2_gpio_driver);

MODULE_DESCRIPTION("Mellanox BlueField-2 GPIO Driver");
MODULE_AUTHOR("Mellanox Technologies");
MODULE_LICENSE("GPL v2");
