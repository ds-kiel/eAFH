/*
 * Copyright (c) 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Common part of DMA drivers for some NXP SoC.
 */
#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <drivers/dma.h>
#include <fsl_dma.h>
#include <fsl_inputmux.h>
#include <logging/log.h>

#define DT_DRV_COMPAT nxp_lpc_dma

LOG_MODULE_REGISTER(dma_mcux_lpc, CONFIG_DMA_LOG_LEVEL);

struct dma_mcux_lpc_config {
	DMA_Type *base;
	uint32_t num_of_channels;
	void (*irq_config_func)(const struct device *dev);
};

struct call_back {
	dma_transfer_config_t *transferConfig;
	dma_handle_t dma_handle;
	const struct device *dev;
	void *user_data;
	dma_callback_t dma_callback;
	enum dma_channel_direction dir;
	bool busy;
	uint32_t channel_index;
};

struct dma_mcux_lpc_dma_data {
	struct call_back *data_cb;
	uint32_t *channel_index;
	uint32_t num_channels_used;
};

#define DEV_CFG(dev) \
	((const struct dma_mcux_lpc_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct dma_mcux_lpc_dma_data *)dev->data)
#define DEV_BASE(dev) ((DMA_Type *)DEV_CFG(dev)->base)

#define DEV_CHANNEL_DATA(dev, ch)                                              \
		((struct call_back *)(&(DEV_DATA(dev)->data_cb[ch])))

#define DEV_DMA_HANDLE(dev, ch)                                               \
		((dma_handle_t *)(&(DEV_CHANNEL_DATA(dev, ch)->dma_handle)))

static void nxp_lpc_dma_callback(dma_handle_t *handle, void *param,
			      bool transferDone, uint32_t intmode)
{
	int ret = 1;
	struct call_back *data = (struct call_back *)param;
	uint32_t channel = handle->channel;

	if (transferDone) {
		data->busy = false;
		ret = 0;
	}

	if (intmode == kDMA_IntError) {
		DMA_AbortTransfer(handle);
	}

	data->dma_callback(data->dev, data->user_data, channel, ret);
}

/* Handles DMA interrupts and dispatches to the individual channel */
static void dma_mcux_lpc_irq_handler(const struct device *dev)
{
	DMA_IRQHandle(DEV_BASE(dev));
/*
 * Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store
 * immediate overlapping exception return operation might vector
 * to incorrect interrupt
 */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
	__DSB();
#endif
}

/* Configure a channel */
static int dma_mcux_lpc_configure(const struct device *dev, uint32_t channel,
				  struct dma_config *config)
{
	dma_handle_t *p_handle;
	struct call_back *data;
	struct dma_mcux_lpc_dma_data *dma_data = DEV_DATA(dev);
	struct dma_block_config *block_config = config->head_block;
	dma_transfer_type_t transfer_type;
	uint32_t virtual_channel;
	uint32_t total_dma_channels;

	if (NULL == dev || NULL == config) {
		return -EINVAL;
	}

	/* Check if have a free slot to store DMA channel data */
	if (dma_data->num_channels_used > DEV_CFG(dev)->num_of_channels) {
		LOG_ERR("out of DMA channel %d", channel);
		return -EINVAL;
	}

#if defined FSL_FEATURE_DMA_NUMBER_OF_CHANNELS
	total_dma_channels = FSL_FEATURE_DMA_NUMBER_OF_CHANNELS;
#else
	total_dma_channels = FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(DEV_BASE(dev));
#endif

	/* Check if the dma channel number is valid */
	if (channel >= total_dma_channels) {
		LOG_ERR("invalid DMA channel number %d", channel);
		return -EINVAL;
	}

	if (config->source_data_size != 4U &&
		config->source_data_size != 2U &&
		config->source_data_size != 1U) {
		LOG_ERR("Source unit size error, %d", config->source_data_size);
		return -EINVAL;
	}

	if (config->dest_data_size != 4U &&
		config->dest_data_size != 2U &&
		config->dest_data_size != 1U) {
		LOG_ERR("Dest unit size error, %d", config->dest_data_size);
		return -EINVAL;
	}

	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
		transfer_type = kDMA_MemoryToMemory;
		break;
	case MEMORY_TO_PERIPHERAL:
		transfer_type = kDMA_MemoryToPeripheral;
		break;
	case PERIPHERAL_TO_MEMORY:
		transfer_type = kDMA_PeripheralToMemory;
		break;
	default:
		LOG_ERR("not support transfer direction");
		return -EINVAL;
	}

	/* If needed, allocate a slot to store dma channel data */
	if (dma_data->channel_index[channel] == -1) {
		dma_data->channel_index[channel] = dma_data->num_channels_used;
		dma_data->num_channels_used++;
	}

	/* Get the slot number that has the dma channel data */
	virtual_channel = dma_data->channel_index[channel];
	/* dma channel data */
	p_handle = DEV_DMA_HANDLE(dev, virtual_channel);
	data = DEV_CHANNEL_DATA(dev, virtual_channel);

	data->dir = config->channel_direction;

	if (data->busy) {
		DMA_AbortTransfer(p_handle);
	}
	DMA_CreateHandle(p_handle, DEV_BASE(dev), channel);
	DMA_SetCallback(p_handle, nxp_lpc_dma_callback, (void *)data);

	LOG_DBG("channel is %d", p_handle->channel);

	if (config->source_chaining_en && config->dest_chaining_en) {
		LOG_DBG("link dma out 0 to channel %d", config->linked_channel);
		/* Link DMA_OTRIG 0 to channel */
		INPUTMUX_AttachSignal(INPUTMUX, 0, config->linked_channel);
	}

	/* Allocate the transfer config structures if needed */
	if (data->transferConfig == NULL) {
		data->transferConfig = k_malloc(CONFIG_DMA_LINK_QUEUE_SIZE *
						sizeof(dma_transfer_config_t));
		if (!data->transferConfig) {
			LOG_ERR("HEAP_MEM_POOL_SIZE is too small");
			return -ENOMEM;
		}
	}

	if (block_config->source_gather_en || block_config->dest_scatter_en) {
		if (config->block_count > CONFIG_DMA_LINK_QUEUE_SIZE) {
			LOG_ERR("please config DMA_LINK_QUEUE_SIZE as %d",
				config->block_count);
			return -EINVAL;
		}
		int i = 0;
		dma_transfer_config_t *next_transfer;

		while (block_config != NULL) {
			/* Check if this last element in the chain */
			if (block_config->next_block == NULL) {
				next_transfer = NULL;
			} else {
				next_transfer = &(data->transferConfig[i + 1]);
			}

			DMA_PrepareTransfer(
				&(data->transferConfig[i]),
				(void *)block_config->source_address,
				(void *)block_config->dest_address,
				config->dest_data_size,
				block_config->block_size, transfer_type,
				(void *)next_transfer);
			block_config = block_config->next_block;
			i++;
		}

	} else {
		/* block_count shall be 1 */
		DMA_PrepareTransfer(
			&(data->transferConfig[0]),
			(void *)block_config->source_address,
			(void *)block_config->dest_address,
			config->dest_data_size,
			block_config->block_size, transfer_type,
			NULL);
	}
	DMA_SubmitTransfer(p_handle, &(data->transferConfig[0]));

	data->busy = false;
	if (config->dma_callback) {
		LOG_DBG("INSTALL call back on channel %d", channel);
		data->user_data = config->user_data;
		data->dma_callback = config->dma_callback;
		data->dev = dev;
	}

	return 0;
}

static int dma_mcux_lpc_start(const struct device *dev, uint32_t channel)
{
	uint32_t virtual_channel = DEV_DATA(dev)->channel_index[channel];
	struct call_back *data = DEV_CHANNEL_DATA(dev, virtual_channel);

	LOG_DBG("START TRANSFER");
	LOG_DBG("DMA CTRL 0x%x", DEV_BASE(dev)->CTRL);
	data->busy = true;
	DMA_StartTransfer(DEV_DMA_HANDLE(dev, virtual_channel));
	return 0;
}

static int dma_mcux_lpc_stop(const struct device *dev, uint32_t channel)
{
	uint32_t virtual_channel = DEV_DATA(dev)->channel_index[channel];
	struct call_back *data = DEV_CHANNEL_DATA(dev, virtual_channel);

	if (!data->busy) {
		return 0;
	}
	DMA_AbortTransfer(DEV_DMA_HANDLE(dev, virtual_channel));
	data->busy = false;
	return 0;
}

static int dma_mcux_lpc_reload(const struct device *dev, uint32_t channel,
			       uint32_t src, uint32_t dst, size_t size)
{
	uint32_t virtual_channel = DEV_DATA(dev)->channel_index[channel];
	struct call_back *data = DEV_CHANNEL_DATA(dev, virtual_channel);

	if (data->busy) {
		DMA_AbortTransfer(DEV_DMA_HANDLE(dev, virtual_channel));
	}
	return 0;
}

static int dma_mcux_lpc_get_status(const struct device *dev, uint32_t channel,
				   struct dma_status *status)
{
	uint32_t virtual_channel = DEV_DATA(dev)->channel_index[channel];
	struct call_back *data = DEV_CHANNEL_DATA(dev, virtual_channel);

	if (data->busy) {
		status->busy = true;
		status->pending_length = DMA_GetRemainingBytes(DEV_BASE(dev), channel);
	} else {
		status->busy = false;
		status->pending_length = 0;
	}
	status->dir = data->dir;
	LOG_DBG("DMA CR 0x%x", DEV_BASE(dev)->CTRL);
	LOG_DBG("DMA INT 0x%x", DEV_BASE(dev)->INTSTAT);

	return 0;
}

static int dma_mcux_lpc_init(const struct device *dev)
{
	struct dma_mcux_lpc_dma_data *data = DEV_DATA(dev);
	int size_channel_data;
	int total_dma_channels;

	/* Array to store DMA channel data */
	size_channel_data =
		sizeof(struct call_back) * DEV_CFG(dev)->num_of_channels;
	data->data_cb = k_malloc(size_channel_data);
	if (!data->data_cb) {
		LOG_ERR("HEAP_MEM_POOL_SIZE is too small");
		return -ENOMEM;
	}

	for (int i = 0; i < DEV_CFG(dev)->num_of_channels; i++) {
		data->data_cb[i].transferConfig = NULL;
	}

#if defined FSL_FEATURE_DMA_NUMBER_OF_CHANNELS
	total_dma_channels = FSL_FEATURE_DMA_NUMBER_OF_CHANNELS;
#else
	total_dma_channels = FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(DEV_BASE(dev));
#endif
	/*
	 * This array is used to hold the index associated with the array
	 * holding channel data
	 */
	data->channel_index = k_malloc(sizeof(uint32_t) * total_dma_channels);
	if (!data->channel_index) {
		LOG_ERR("HEAP_MEM_POOL_SIZE is too small");
		return -ENOMEM;
	}

	/*
	 * Initialize to -1 to indicate dma channel does not have a slot
	 * assigned to store dma channel data
	 */
	for (int i = 0; i < total_dma_channels; i++) {
		data->channel_index[i] = -1;
	}

	data->num_channels_used = 0;


	DMA_Init(DEV_BASE(dev));
	INPUTMUX_Init(INPUTMUX);

	return 0;
}

static const struct dma_driver_api dma_mcux_lpc_api = {
	.config = dma_mcux_lpc_configure,
	.start = dma_mcux_lpc_start,
	.stop = dma_mcux_lpc_stop,
	.reload = dma_mcux_lpc_reload,
	.get_status = dma_mcux_lpc_get_status,
};

#define DMA_MCUX_LPC_CONFIG_FUNC(n)				\
	static void dma_mcux_lpc_config_func_##n(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    dma_mcux_lpc_irq_handler, DEVICE_GET(dma_mcux_lpc_##n), 0);\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}
#define DMA_MCUX_LPC_IRQ_CFG_FUNC_INIT(n)				\
	.irq_config_func = dma_mcux_lpc_config_func_##n
#define DMA_MCUX_LPC_INIT_CFG(n)					\
	DMA_MCUX_LPC_DECLARE_CFG(n,				\
				       DMA_MCUX_LPC_IRQ_CFG_FUNC_INIT(n))

#define DMA_MCUX_LPC_DECLARE_CFG(n, IRQ_FUNC_INIT)		\
static const struct dma_mcux_lpc_config dma_##n##_config = {	\
	.base = (DMA_Type *)DT_INST_REG_ADDR(n),			\
	.num_of_channels = DT_INST_PROP(n, dma_channels),			\
	IRQ_FUNC_INIT							\
}

#define DMA_INIT(n) \
									\
	static const struct dma_mcux_lpc_config dma_##n##_config;\
									\
	static struct dma_mcux_lpc_dma_data dma_data_##n = {	\
		.data_cb = NULL,					\
	};								\
									\
	DEVICE_AND_API_INIT(dma_mcux_lpc_##n, DT_INST_LABEL(n),	\
			    &dma_mcux_lpc_init,				\
			    &dma_data_##n, &dma_##n##_config,\
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,\
			    &dma_mcux_lpc_api);			\
									\
	DMA_MCUX_LPC_CONFIG_FUNC(n)				\
									\
	DMA_MCUX_LPC_INIT_CFG(n);

DT_INST_FOREACH_STATUS_OKAY(DMA_INIT)
