// SPDX-License-Identifier: GPL-2.0+
// Copyright 2019 IBM Corp.
#include <linux/sched/mm.h>
#include "trace.h"
#include "ocxl_internal.h"

int ocxl_global_mmio_read32(struct ocxl_afu *afu, size_t offset,
				enum ocxl_endian endian, u32 *val)
{
	if (offset > afu->config.global_mmio_size - 4)
		return -EINVAL;

#ifdef __BIG_ENDIAN__
	if (endian == OCXL_HOST_ENDIAN)
		endian = OCXL_BIG_ENDIAN;
#endif

	switch (endian) {
	case OCXL_BIG_ENDIAN:
		*val = readl_be((char *)afu->global_mmio_ptr + offset);
		break;

	default:
		*val = pete_readl("drivers/misc/ocxl/mmio.c:24", (char *)afu->global_mmio_ptr + offset);
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ocxl_global_mmio_read32);

int ocxl_global_mmio_read64(struct ocxl_afu *afu, size_t offset,
				enum ocxl_endian endian, u64 *val)
{
	if (offset > afu->config.global_mmio_size - 8)
		return -EINVAL;

#ifdef __BIG_ENDIAN__
	if (endian == OCXL_HOST_ENDIAN)
		endian = OCXL_BIG_ENDIAN;
#endif

	switch (endian) {
	case OCXL_BIG_ENDIAN:
		*val = readq_be((char *)afu->global_mmio_ptr + offset);
		break;

	default:
		*val = pete_readq("drivers/misc/ocxl/mmio.c:49", (char *)afu->global_mmio_ptr + offset);
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ocxl_global_mmio_read64);

int ocxl_global_mmio_write32(struct ocxl_afu *afu, size_t offset,
				enum ocxl_endian endian, u32 val)
{
	if (offset > afu->config.global_mmio_size - 4)
		return -EINVAL;

#ifdef __BIG_ENDIAN__
	if (endian == OCXL_HOST_ENDIAN)
		endian = OCXL_BIG_ENDIAN;
#endif

	switch (endian) {
	case OCXL_BIG_ENDIAN:
		writel_be(val, (char *)afu->global_mmio_ptr + offset);
		break;

	default:
		pete_writel("drivers/misc/ocxl/mmio.c:74", val, (char *)afu->global_mmio_ptr + offset);
		break;
	}


	return 0;
}
EXPORT_SYMBOL_GPL(ocxl_global_mmio_write32);

int ocxl_global_mmio_write64(struct ocxl_afu *afu, size_t offset,
				enum ocxl_endian endian, u64 val)
{
	if (offset > afu->config.global_mmio_size - 8)
		return -EINVAL;

#ifdef __BIG_ENDIAN__
	if (endian == OCXL_HOST_ENDIAN)
		endian = OCXL_BIG_ENDIAN;
#endif

	switch (endian) {
	case OCXL_BIG_ENDIAN:
		writeq_be(val, (char *)afu->global_mmio_ptr + offset);
		break;

	default:
		pete_writeq("drivers/misc/ocxl/mmio.c:100", val, (char *)afu->global_mmio_ptr + offset);
		break;
	}


	return 0;
}
EXPORT_SYMBOL_GPL(ocxl_global_mmio_write64);

int ocxl_global_mmio_set32(struct ocxl_afu *afu, size_t offset,
				enum ocxl_endian endian, u32 mask)
{
	u32 tmp;

	if (offset > afu->config.global_mmio_size - 4)
		return -EINVAL;

#ifdef __BIG_ENDIAN__
	if (endian == OCXL_HOST_ENDIAN)
		endian = OCXL_BIG_ENDIAN;
#endif

	switch (endian) {
	case OCXL_BIG_ENDIAN:
		tmp = readl_be((char *)afu->global_mmio_ptr + offset);
		tmp |= mask;
		writel_be(tmp, (char *)afu->global_mmio_ptr + offset);
		break;

	default:
		tmp = pete_readl("drivers/misc/ocxl/mmio.c:130", (char *)afu->global_mmio_ptr + offset);
		tmp |= mask;
		pete_writel("drivers/misc/ocxl/mmio.c:132", tmp, (char *)afu->global_mmio_ptr + offset);
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ocxl_global_mmio_set32);

int ocxl_global_mmio_set64(struct ocxl_afu *afu, size_t offset,
				enum ocxl_endian endian, u64 mask)
{
	u64 tmp;

	if (offset > afu->config.global_mmio_size - 8)
		return -EINVAL;

#ifdef __BIG_ENDIAN__
	if (endian == OCXL_HOST_ENDIAN)
		endian = OCXL_BIG_ENDIAN;
#endif

	switch (endian) {
	case OCXL_BIG_ENDIAN:
		tmp = readq_be((char *)afu->global_mmio_ptr + offset);
		tmp |= mask;
		writeq_be(tmp, (char *)afu->global_mmio_ptr + offset);
		break;

	default:
		tmp = pete_readq("drivers/misc/ocxl/mmio.c:161", (char *)afu->global_mmio_ptr + offset);
		tmp |= mask;
		pete_writeq("drivers/misc/ocxl/mmio.c:163", tmp, (char *)afu->global_mmio_ptr + offset);
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ocxl_global_mmio_set64);

int ocxl_global_mmio_clear32(struct ocxl_afu *afu, size_t offset,
				enum ocxl_endian endian, u32 mask)
{
	u32 tmp;

	if (offset > afu->config.global_mmio_size - 4)
		return -EINVAL;

#ifdef __BIG_ENDIAN__
	if (endian == OCXL_HOST_ENDIAN)
		endian = OCXL_BIG_ENDIAN;
#endif

	switch (endian) {
	case OCXL_BIG_ENDIAN:
		tmp = readl_be((char *)afu->global_mmio_ptr + offset);
		tmp &= ~mask;
		writel_be(tmp, (char *)afu->global_mmio_ptr + offset);
		break;

	default:
		tmp = pete_readl("drivers/misc/ocxl/mmio.c:192", (char *)afu->global_mmio_ptr + offset);
		tmp &= ~mask;
		pete_writel("drivers/misc/ocxl/mmio.c:194", tmp, (char *)afu->global_mmio_ptr + offset);
		break;
	}


	return 0;
}
EXPORT_SYMBOL_GPL(ocxl_global_mmio_clear32);

int ocxl_global_mmio_clear64(struct ocxl_afu *afu, size_t offset,
				enum ocxl_endian endian, u64 mask)
{
	u64 tmp;

	if (offset > afu->config.global_mmio_size - 8)
		return -EINVAL;

#ifdef __BIG_ENDIAN__
	if (endian == OCXL_HOST_ENDIAN)
		endian = OCXL_BIG_ENDIAN;
#endif

	switch (endian) {
	case OCXL_BIG_ENDIAN:
		tmp = readq_be((char *)afu->global_mmio_ptr + offset);
		tmp &= ~mask;
		writeq_be(tmp, (char *)afu->global_mmio_ptr + offset);
		break;

	default:
		tmp = pete_readq("drivers/misc/ocxl/mmio.c:224", (char *)afu->global_mmio_ptr + offset);
		tmp &= ~mask;
		pete_writeq("drivers/misc/ocxl/mmio.c:226", tmp, (char *)afu->global_mmio_ptr + offset);
		break;
	}

	pete_writeq("drivers/misc/ocxl/mmio.c:230", tmp, (char *)afu->global_mmio_ptr + offset);

	return 0;
}
EXPORT_SYMBOL_GPL(ocxl_global_mmio_clear64);
