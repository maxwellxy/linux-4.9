/*
 * Copyright (c) 2017 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2017 Vadim Pasternak <vadimp@mellanox.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_data/mlxreg.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

/* Offset of event and mask registers from status register. */
#define MLXREG_HOTPLUG_EVENT_OFF	1
#define MLXREG_HOTPLUG_MASK_OFF		2
#define MLXREG_HOTPLUG_AGGR_MASK_OFF	1

/* ASIC health parameters. */
#define MLXREG_HOTPLUG_HEALTH_MASK	0x02
#define MLXREG_HOTPLUG_RST_CNTR		3

#define MLXREG_HOTPLUG_PROP_OKAY	"okay"
#define MLXREG_HOTPLUG_PROP_DISABLED	"disabled"
#define MLXREG_HOTPLUG_PROP_STATUS	"status"

#define MLXREG_HOTPLUG_ATTRS_MAX	24

/**
 * struct mlxreg_hotplug_priv_data - platform private data:
 * @irq: platform device interrupt number;
 * @pdev: platform device;
 * @plat: platform data;
 * @dwork: delayed work template;
 * @lock: spin lock;
 * @hwmon: hwmon device;
 * @mlxreg_hotplug_attr: sysfs attributes array;
 * @mlxreg_hotplug_dev_attr: sysfs sensor device attribute array;
 * @group: sysfs attribute group;
 * @groups: list of sysfs attribute group for hwmon registration;
 * @cell: location of top aggregation interrupt register;
 * @mask: top aggregation interrupt common mask;
 * @aggr_cache: last value of aggregation register status;
 */
struct mlxreg_hotplug_priv_data {
	int irq;
	struct device *dev;
	struct platform_device *pdev;
	struct mlxreg_hotplug_platform_data *plat;
	struct regmap *regmap;
	struct delayed_work dwork_irq;
	struct delayed_work dwork;
	spinlock_t lock; /* sync with interrupt */
	struct device *hwmon;
	struct attribute *mlxreg_hotplug_attr[MLXREG_HOTPLUG_ATTRS_MAX + 1];
	struct sensor_device_attribute_2
			mlxreg_hotplug_dev_attr[MLXREG_HOTPLUG_ATTRS_MAX];
	struct attribute_group group;
	const struct attribute_group *groups[2];
	u32 cell;
	u32 mask;
	u32 aggr_cache;
	bool after_probe;
};

#if defined(CONFIG_OF_DYNAMIC) && !defined(CONFIG_COMPILE_TEST)
/**
 * struct mlxreg_hotplug_device_en - Open Firmware property for enabling device
 *
 * @name - property name;
 * @value - property value string;
 * @length - length of proprty value string;
 *
 * The structure is used for the devices, which require some dynamic
 * selection operation allowing access to them.
 */
static struct property mlxreg_hotplug_device_en = {
	.name = MLXREG_HOTPLUG_PROP_STATUS,
	.value = MLXREG_HOTPLUG_PROP_OKAY,
	.length = sizeof(MLXREG_HOTPLUG_PROP_OKAY),
};

/**
 * struct mlxreg_hotplug_device_dis - Open Firmware property for disabling
 * device
 *
 * @name - property name;
 * @value - property value string;
 * @length - length of proprty value string;
 *
 * The structure is used for the devices, which require some dynamic
 * selection operation disallowing access to them.
 */
static struct property mlxreg_hotplug_device_dis = {
	.name = MLXREG_HOTPLUG_PROP_STATUS,
	.value = MLXREG_HOTPLUG_PROP_DISABLED,
	.length = sizeof(MLXREG_HOTPLUG_PROP_DISABLED),
};

static int mlxreg_hotplug_of_device_create(struct mlxreg_core_data *data)
{
	return of_update_property(data->np, &mlxreg_hotplug_device_en);
}

static void mlxreg_hotplug_of_device_destroy(struct mlxreg_core_data *data)
{
	of_update_property(data->np, &mlxreg_hotplug_device_dis);
	of_node_clear_flag(data->np, OF_POPULATED);
}
#else
static int mlxreg_hotplug_of_device_create(struct mlxreg_core_data *data)
{
	return 0;
}

static void mlxreg_hotplug_of_device_destroy(struct mlxreg_core_data *data)
{
}
#endif

static int mlxreg_hotplug_device_create(struct mlxreg_core_data *data)
{
	data->hpdev.adapter = i2c_get_adapter(data->hpdev.nr);
	if (!data->hpdev.adapter)
		return -EFAULT;

	data->hpdev.client = i2c_new_device(data->hpdev.adapter,
					    data->hpdev.brdinfo);
	if (!data->hpdev.client) {
		i2c_put_adapter(data->hpdev.adapter);
		data->hpdev.adapter = NULL;
		return -EFAULT;
	}

	return 0;
}

static void mlxreg_hotplug_device_destroy(struct mlxreg_core_data *data)
{
	if (data->hpdev.client) {
		i2c_unregister_device(data->hpdev.client);
		data->hpdev.client = NULL;
	}

	if (data->hpdev.adapter) {
		i2c_put_adapter(data->hpdev.adapter);
		data->hpdev.adapter = NULL;
	}
}

static int mlxreg_hotplug_dev_enable(struct mlxreg_core_data *data)
{
	int err;

	/* Enable and create device. */
	if (data->np)
		err = mlxreg_hotplug_of_device_create(data);
	else
		err = mlxreg_hotplug_device_create(data);

	return err;
}

static void mlxreg_hotplug_dev_disable(struct mlxreg_core_data *data)
{
	/* Disable and unregister platform device. */
	if (data->np)
		mlxreg_hotplug_of_device_destroy(data);
	else
		mlxreg_hotplug_device_destroy(data);
}

static ssize_t mlxreg_hotplug_attr_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct mlxreg_hotplug_priv_data *priv = dev_get_drvdata(dev);
	struct mlxreg_core_hotplug_platform_data *pdata;
	int index = to_sensor_dev_attr_2(attr)->index;
	int nr = to_sensor_dev_attr_2(attr)->nr;
	struct mlxreg_core_item *item;
	struct mlxreg_core_data *data;
	u32 regval;
	int ret;

	pdata = dev_get_platdata(&priv->pdev->dev);
	item = pdata->items + nr;
	data = item->data + index;

	ret = regmap_read(priv->regmap, data->reg, &regval);
	if (ret)
		return ret;

	if (item->health) {
		regval &= data->mask;
	} else {
		/* Bit = 0 : functional if item->inversed is true. */
		if (item->inversed)
			regval = !(regval & data->mask);
		else
			regval = !!(regval & data->mask);
	}

	return sprintf(buf, "%u\n", regval);
}

static int
mlxreg_hotplug_attr_init(struct mlxreg_hotplug_priv_data *priv)
{
	struct mlxreg_core_hotplug_platform_data *pdata;
	struct mlxreg_core_item *item;
	struct mlxreg_core_data *data;
	int num_attrs = 0, id = 0, i, j;

	pdata = dev_get_platdata(&priv->pdev->dev);
	item = pdata->items;

	for (i = 0; i < pdata->counter; i++)
		num_attrs += (item + i)->count;

	priv->group.attrs = devm_kzalloc(&priv->pdev->dev, num_attrs *
					 sizeof(struct attribute *),
					 GFP_KERNEL);
	if (!priv->group.attrs)
		return -ENOMEM;

	for (i = 0; i < pdata->counter; i++, item++) {
		data = item->data;
		for (j = 0; j < item->count; j++, data++, id++) {
			priv->mlxreg_hotplug_attr[id] =
			&priv->mlxreg_hotplug_dev_attr[id].dev_attr.attr;
			priv->mlxreg_hotplug_attr[id]->name =
				devm_kasprintf(&priv->pdev->dev, GFP_KERNEL,
					       data->label);

			if (!priv->mlxreg_hotplug_attr[id]->name) {
				dev_err(priv->dev, "Memory allocation failed for attr %d.\n",
					id);
				return -ENOMEM;
			}

			priv->mlxreg_hotplug_dev_attr[id].dev_attr.attr.mode =
									0444;
			priv->mlxreg_hotplug_dev_attr[id].dev_attr.show =
						mlxreg_hotplug_attr_show;
			priv->mlxreg_hotplug_dev_attr[id].nr = i;
			priv->mlxreg_hotplug_dev_attr[id].index = j;
			sysfs_attr_init(
			&priv->mlxreg_hotplug_dev_attr[id].dev_attr.attr);
		}
	}

	priv->group.attrs = priv->mlxreg_hotplug_attr;
	priv->groups[0] = &priv->group;
	priv->groups[1] = NULL;

	return 0;
}

static void
mlxreg_hotplug_work_helper(struct mlxreg_hotplug_priv_data *priv,
			   struct mlxreg_core_item *item)
{
	struct mlxreg_core_data *data;
	u32 asserted, regval, bit;
	int ret;

	/*
	 * Validate if item related to received signal type is valid.
	 * It should never happen, excepted the situation when some
	 * piece of hardware is broken. In such situation just produce
	 * error message and return. Caller must continue to handle the
	 * signals from other devices if any.
	 */
	if (unlikely(!item)) {
		dev_err(priv->dev, "False signal: at offset:mask 0x%02x:0x%02x.\n",
			item->reg, item->mask);

		return;
	}

	/* Mask event. */
	ret = regmap_write(priv->regmap, item->reg + MLXREG_HOTPLUG_MASK_OFF,
			   0);
	if (ret)
		goto access_error;

	/* Read status. */
	ret = regmap_read(priv->regmap, item->reg, &regval);
	if (ret)
		goto access_error;

	/* Set asserted bits and save last status. */
	regval &= item->mask;
	asserted = item->cache ^ regval;
	item->cache = regval;

	for_each_set_bit(bit, (unsigned long *)&asserted, 8) {
		data = item->data + bit;
		if (regval & BIT(bit)) {
			if (item->inversed)
				mlxreg_hotplug_dev_disable(data);
			else
				mlxreg_hotplug_dev_enable(data);
		} else {
			if (item->inversed)
				mlxreg_hotplug_dev_enable(data);
			else
				mlxreg_hotplug_dev_disable(data);
		}
	}

	/* Acknowledge event. */
	ret = regmap_write(priv->regmap, item->reg + MLXREG_HOTPLUG_EVENT_OFF,
			   0);
	if (ret)
		goto access_error;

	/* Unmask event. */
	ret = regmap_write(priv->regmap, item->reg + MLXREG_HOTPLUG_MASK_OFF,
			   item->mask);
	if (ret)
		goto access_error;

	return;

access_error:
	dev_err(priv->dev, "Failed to complete workqueue.\n");
}

static void
mlxreg_hotplug_health_work_helper(struct mlxreg_hotplug_priv_data *priv,
				  struct mlxreg_core_item *item)
{
	struct mlxreg_core_data *data = item->data;
	u32 regval;
	int i, ret;

	for (i = 0; i < item->count; i++, data++) {
		/* Mask event. */
		ret = regmap_write(priv->regmap, data->reg +
				   MLXREG_HOTPLUG_MASK_OFF, 0);
		if (ret)
			goto access_error;

		/* Read status. */
		ret = regmap_read(priv->regmap, data->reg, &regval);
		if (ret)
			goto access_error;

		regval &= data->mask;
		item->cache = regval;
		if (regval == MLXREG_HOTPLUG_HEALTH_MASK) {
			if ((data->health_cntr++ == MLXREG_HOTPLUG_RST_CNTR) ||
			    !priv->after_probe) {
				mlxreg_hotplug_dev_enable(data);
				data->attached = true;
			}
		} else {
			if (data->attached) {
				mlxreg_hotplug_dev_disable(data);
				data->attached = false;
				data->health_cntr = 0;
			}
		}

		/* Acknowledge event. */
		ret = regmap_write(priv->regmap, data->reg +
				   MLXREG_HOTPLUG_EVENT_OFF, 0);
		if (ret)
			goto access_error;

		/* Unmask event. */
		ret = regmap_write(priv->regmap, data->reg +
				   MLXREG_HOTPLUG_MASK_OFF, data->mask);
		if (ret)
			goto access_error;
	}

	return;

access_error:
	dev_err(priv->dev, "Failed to complete workqueue.\n");
}

/*
 * mlxreg_hotplug_work_handler - performs traversing of device interrupt
 * registers according to the below hierarchy schema:
 *
 *				Aggregation registers (status/mask)
 * PSU registers:		*---*
 * *-----------------*		|   |
 * |status/event/mask|----->    | * |
 * *-----------------*		|   |
 * Power registers:		|   |
 * *-----------------*		|   |
 * |status/event/mask|----->    | * |
 * *-----------------*		|   |
 * FAN registers:		|   |--> CPU
 * *-----------------*		|   |
 * |status/event/mask|----->    | * |
 * *-----------------*		|   |
 * ASIC registers:		|   |
 * *-----------------*		|   |
 * |status/event/mask|----->    | * |
 * *-----------------*		|   |
 *				*---*
 *
 * In case some system changed are detected: FAN in/out, PSU in/out, power
 * cable attached/detached, ASIC helath good/bad, relevant device is created
 * or destroyed.
 */
static void mlxreg_hotplug_work_handler(struct work_struct *work)
{
	struct mlxreg_hotplug_priv_data *priv = container_of(work,
			struct mlxreg_hotplug_priv_data, dwork_irq.work);
	struct mlxreg_core_hotplug_platform_data *pdata;
	struct mlxreg_core_item *item;
	unsigned long flags;
	u32 regval, aggr_asserted;
	int i;
	int ret;

	pdata = dev_get_platdata(&priv->pdev->dev);
	item = pdata->items;
	/* Mask aggregation event. */
	ret = regmap_write(priv->regmap, pdata->cell +
			   MLXREG_HOTPLUG_AGGR_MASK_OFF, 0);
	if (ret < 0)
		goto access_error;

	/* Read aggregation status. */
	ret = regmap_read(priv->regmap, pdata->cell, &regval);
	if (ret)
		goto access_error;

	regval &= pdata->mask;
	aggr_asserted = priv->aggr_cache ^ regval;
	priv->aggr_cache = regval;

	/* Handle topology and health configuration changes. */
	for (i = 0; i < pdata->counter; i++, item++) {
		if (aggr_asserted & item->aggr_mask) {
			if (item->health)
				mlxreg_hotplug_health_work_helper(priv, item);
			else
				mlxreg_hotplug_work_helper(priv, item);
		}
	}

	if (aggr_asserted) {
		spin_lock_irqsave(&priv->lock, flags);

		/*
		 * It is possible, that some signals have been inserted, while
		 * interrupt has been masked by mlxreg_hotplug_work_handler.
		 * In this case such signals will be missed. In order to handle
		 * these signals delayed work is canceled and work task
		 * re-scheduled for immediate execution. It allows to handle
		 * missed signals, if any. In other case work handler just
		 * validates that no new signals have been received during
		 * masking.
		 */
		cancel_delayed_work(&priv->dwork_irq);
		schedule_delayed_work(&priv->dwork_irq, 0);

		spin_unlock_irqrestore(&priv->lock, flags);

		return;
	}

	/* Unmask aggregation event (no need acknowledge). */
	ret = regmap_write(priv->regmap, pdata->cell +
			   MLXREG_HOTPLUG_AGGR_MASK_OFF, pdata->mask);
	if (ret)
		goto access_error;

	return;

access_error:
	dev_err(priv->dev, "Failed to complete workqueue.\n");
}

static int mlxreg_hotplug_set_irq(struct mlxreg_hotplug_priv_data *priv)
{
	struct mlxreg_core_hotplug_platform_data *pdata;
	struct mlxreg_core_item *item;
	int i;
	int ret;

	pdata = dev_get_platdata(&priv->pdev->dev);
	item = pdata->items;

	for (i = 0; i < pdata->counter; i++, item++) {
		/* Clear group presense event. */
		ret = regmap_write(priv->regmap, item->reg +
				   MLXREG_HOTPLUG_EVENT_OFF, 0);
		if (ret)
			goto access_error;

		/* Set group initial status as mask and unmask group event. */
		if (item->inversed) {
			item->cache = item->mask;
			ret = regmap_write(priv->regmap, item->reg +
					   MLXREG_HOTPLUG_MASK_OFF,
					   item->mask);
			if (ret)
				goto access_error;
		}
	}

	/* Keep aggregation initial status as zero and unmask events. */
	ret = regmap_write(priv->regmap, pdata->cell +
			   MLXREG_HOTPLUG_AGGR_MASK_OFF, pdata->mask);
	if (ret)
		goto access_error;

	/* Invoke work handler for initializing hot plug devices setting. */
	mlxreg_hotplug_work_handler(&priv->dwork_irq.work);

	enable_irq(priv->irq);

	return 0;

access_error:
	dev_err(priv->dev, "Failed to set interrupts.\n");

	enable_irq(priv->irq);

	return ret;
}

static void mlxreg_hotplug_unset_irq(struct mlxreg_hotplug_priv_data *priv)
{
	struct mlxreg_core_hotplug_platform_data *pdata;
	struct mlxreg_core_item *item;
	struct mlxreg_core_data *data;
	int count, i, j;

	pdata = dev_get_platdata(&priv->pdev->dev);
	item = pdata->items;
	disable_irq(priv->irq);
	cancel_delayed_work_sync(&priv->dwork_irq);

	/* Mask aggregation event. */
	regmap_write(priv->regmap, pdata->cell + MLXREG_HOTPLUG_AGGR_MASK_OFF,
		     0);

	/* Clear topology configurations. */
	for (i = 0; i < pdata->counter; i++, item++) {
		data = item->data;
		/* Mask group presense event. */
		regmap_write(priv->regmap, data->reg + MLXREG_HOTPLUG_MASK_OFF,
			     0);
		/* Clear group presense event. */
		regmap_write(priv->regmap, data->reg +
			     MLXREG_HOTPLUG_EVENT_OFF, 0);

		/* Remove all the attached devices in group. */
		count = item->count;
		for (j = 0; j < count; j++, data++)
			mlxreg_hotplug_dev_disable(data);
	}
}

static irqreturn_t mlxreg_hotplug_irq_handler(int irq, void *dev)
{
	struct mlxreg_hotplug_priv_data *priv =
				(struct mlxreg_hotplug_priv_data *)dev;

	/* Schedule work task for immediate execution.*/
	schedule_delayed_work(&priv->dwork_irq, 0);

	return IRQ_HANDLED;
}

static int mlxreg_hotplug_probe(struct platform_device *pdev)
{
	struct mlxreg_core_hotplug_platform_data *pdata;
	struct mlxreg_hotplug_priv_data *priv;
	int err;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "Failed to get platform data.\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (pdata->irq) {
		priv->irq = pdata->irq;
	} else {
		priv->irq = platform_get_irq(pdev, 0);
		if (priv->irq < 0) {
			dev_err(&pdev->dev, "Failed to get platform irq: %d\n",
				priv->irq);
			return priv->irq;
		}
	}

	priv->regmap = pdata->regmap;
	priv->dev = pdev->dev.parent;
	priv->pdev = pdev;

	err = devm_request_irq(&pdev->dev, priv->irq,
			       mlxreg_hotplug_irq_handler, IRQF_TRIGGER_FALLING
			       | IRQF_SHARED, "mlxreg-hotplug", priv);
	if (err) {
		dev_err(&pdev->dev, "Failed to request irq: %d\n", err);
		return err;
	}

	disable_irq(priv->irq);
	spin_lock_init(&priv->lock);
	INIT_DELAYED_WORK(&priv->dwork_irq, mlxreg_hotplug_work_handler);
	/* Perform initial interrupts setup. */
	mlxreg_hotplug_set_irq(priv);

	priv->after_probe = true;
	dev_set_drvdata(&pdev->dev, priv);

	err = mlxreg_hotplug_attr_init(priv);
	if (err) {
		dev_err(&pdev->dev, "Failed to allocate attributes: %d\n",
			err);
		return err;
	}

	priv->hwmon = devm_hwmon_device_register_with_groups(&pdev->dev,
					"mlxreg_hotplug", priv, priv->groups);
	if (IS_ERR(priv->hwmon)) {
		dev_err(&pdev->dev, "Failed to register hwmon device %ld\n",
			PTR_ERR(priv->hwmon));
		return PTR_ERR(priv->hwmon);
	}

	return 0;
}

static int mlxreg_hotplug_remove(struct platform_device *pdev)
{
	struct mlxreg_hotplug_priv_data *priv = dev_get_drvdata(&pdev->dev);

	/* Clean interrupts setup. */
	mlxreg_hotplug_unset_irq(priv);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id mlxreg_hotplug_dt_match[] = {
	{ .compatible = "mellanox,mlxreg-hotplug" },
	{ },
};
MODULE_DEVICE_TABLE(of, mlxreg_hotplug_dt_match);
#endif

static struct platform_driver mlxreg_hotplug_driver = {
	.driver = {
	    .name = "mlxreg-hotplug",
	    .of_match_table = of_match_ptr(mlxreg_hotplug_dt_match),
	},
	.probe = mlxreg_hotplug_probe,
	.remove = mlxreg_hotplug_remove,
};

module_platform_driver(mlxreg_hotplug_driver);

MODULE_AUTHOR("Vadim Pasternak <vadimp@mellanox.com>");
MODULE_DESCRIPTION("Mellanox regmap hotplug platform driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:mlxreg-hotplug");
