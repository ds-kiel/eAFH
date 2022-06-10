/*
 * Copyright (c) 2020 Siddharth Chandrasekaran <siddharth@embedjournal.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <kernel.h>
#include <init.h>
#include <device.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <logging/log.h>

#include "osdp_common.h"

LOG_MODULE_REGISTER(osdp, CONFIG_OSDP_LOG_LEVEL);

struct osdp_device {
	struct ring_buf rx_buf;
	struct ring_buf tx_buf;
#ifdef CONFIG_OSDP_MODE_PD
	int rx_event_data;
	struct k_fifo rx_event_fifo;
#endif
	uint8_t rx_fbuf[CONFIG_OSDP_UART_BUFFER_LENGTH];
	uint8_t tx_fbuf[CONFIG_OSDP_UART_BUFFER_LENGTH];
	struct uart_config dev_config;
	const struct device *dev;
};

static struct osdp osdp_ctx;
static struct osdp_cp osdp_cp_ctx;
static struct osdp_pd osdp_pd_ctx[CONFIG_OSDP_NUM_CONNECTED_PD];
static struct osdp_device osdp_device;
static struct k_thread osdp_refresh_thread;
static K_THREAD_STACK_DEFINE(osdp_thread_stack, CONFIG_OSDP_THREAD_STACK_SIZE);

static void osdp_uart_isr(const struct device *dev, void *user_data)
{
	uint8_t buf[64];
	size_t read, wrote, len;
	struct osdp_device *p = user_data;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {

		if (uart_irq_rx_ready(dev)) {
			read = uart_fifo_read(dev, buf, sizeof(buf));
			if (read) {
				wrote = ring_buf_put(&p->rx_buf, buf, read);
				if (wrote < read) {
					LOG_ERR("RX: Drop %u", read - wrote);
				}
			}
		}

		if (uart_irq_tx_ready(dev)) {
			len = ring_buf_get(&p->tx_buf, buf, 1);
			if (!len) {
				uart_irq_tx_disable(dev);
			} else {
				wrote = uart_fifo_fill(dev, buf, 1);
			}
		}
	}
#ifdef CONFIG_OSDP_MODE_PD
	/* wake osdp_refresh thread */
	k_fifo_put(&p->rx_event_fifo, &p->rx_event_data);
#endif
}

static int osdp_uart_receive(void *data, uint8_t *buf, int len)
{
	struct osdp_device *p = data;

	return (int)ring_buf_get(&p->rx_buf, buf, len);
}

static int osdp_uart_send(void *data, uint8_t *buf, int len)
{
	int sent = 0;
	struct osdp_device *p = data;

	sent = (int)ring_buf_put(&p->tx_buf, buf, len);
	uart_irq_tx_enable(p->dev);
	return sent;
}

static void osdp_uart_flush(void *data)
{
	struct osdp_device *p = data;

	ring_buf_reset(&p->tx_buf);
	ring_buf_reset(&p->rx_buf);
}

struct osdp *osdp_get_ctx()
{
	return &osdp_ctx;
}

static struct osdp *osdp_build_ctx(struct osdp_channel *channel)
{
	int i;
	struct osdp *ctx;
	struct osdp_pd *pd;
	int pd_adddres[CONFIG_OSDP_NUM_CONNECTED_PD] = {0};

#ifdef CONFIG_OSDP_MODE_PD
	pd_adddres[0] = CONFIG_OSDP_PD_ADDRESS;
#else
	if (osdp_extract_address(pd_adddres)) {
		return NULL;
	}
	return NULL;
#endif
	ctx = &osdp_ctx;
	ctx->cp = &osdp_cp_ctx;
	ctx->cp->__parent = ctx;
	ctx->cp->num_pd = CONFIG_OSDP_NUM_CONNECTED_PD;
	ctx->pd = &osdp_pd_ctx[0];
	SET_CURRENT_PD(ctx, 0);

	for (i = 0; i < CONFIG_OSDP_NUM_CONNECTED_PD; i++) {
		pd = TO_PD(ctx, i);
		pd->offset = i;
		pd->seq_number = -1;
		pd->__parent = ctx;
		pd->address = pd_adddres[i];
		pd->baud_rate = CONFIG_OSDP_UART_BAUD_RATE;
		memcpy(&pd->channel, channel, sizeof(struct osdp_channel));
		k_mem_slab_init(&pd->cmd.slab,
				pd->cmd.slab_buf, sizeof(struct osdp_cmd),
				CONFIG_OSDP_PD_COMMAND_QUEUE_SIZE);
	}
	return ctx;
}

void osdp_refresh(void *arg1, void *arg2, void *arg3)
{
	struct osdp *ctx = osdp_get_ctx();

	while (1) {
#ifdef CONFIG_OSDP_MODE_PD
		k_fifo_get(&osdp_device.rx_event_fifo, K_FOREVER);
#else
		k_msleep(50);
#endif
		osdp_update(ctx);
	}
}

static int osdp_init(const struct device *arg)
{
	ARG_UNUSED(arg);
	uint8_t c;
	struct osdp *ctx;
	struct osdp_device *p = &osdp_device;
	struct osdp_channel channel = {
		.send = osdp_uart_send,
		.recv = osdp_uart_receive,
		.flush = osdp_uart_flush,
		.data = &osdp_device,
	};

#ifdef CONFIG_OSDP_MODE_PD
	k_fifo_init(&p->rx_event_fifo);
#endif

	ring_buf_init(&p->rx_buf, sizeof(p->rx_fbuf), p->rx_fbuf);
	ring_buf_init(&p->tx_buf, sizeof(p->tx_fbuf), p->tx_fbuf);

	/* init OSDP uart device */
	p->dev = device_get_binding(CONFIG_OSDP_UART_DEV_NAME);
	if (p->dev == NULL) {
		LOG_ERR("Failed to get UART dev binding");
		k_panic();
	}

	/* configure uart device to 8N1 */
	p->dev_config.baudrate = CONFIG_OSDP_UART_BAUD_RATE;
	p->dev_config.data_bits = UART_CFG_DATA_BITS_8;
	p->dev_config.parity = UART_CFG_PARITY_NONE;
	p->dev_config.stop_bits = UART_CFG_STOP_BITS_1;
	p->dev_config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	uart_configure(p->dev, &p->dev_config);

	uart_irq_rx_disable(p->dev);
	uart_irq_tx_disable(p->dev);
	uart_irq_callback_user_data_set(p->dev, osdp_uart_isr, p);

	/* Drain UART fifo */
	while (uart_irq_rx_ready(p->dev)) {
		uart_fifo_read(p->dev, &c, 1);
	}

	/* Both TX and RX are interrupt driven */
	uart_irq_rx_enable(p->dev);

	/* setup OSDP */
	ctx = osdp_build_ctx(&channel);
	if (osdp_setup(ctx)) {
		LOG_ERR("Failed to setup OSDP device!");
		k_panic();
	}

	LOG_INF("OSDP init okay!");

	/* kick off refresh thread */
	k_thread_create(&osdp_refresh_thread, osdp_thread_stack,
			CONFIG_OSDP_THREAD_STACK_SIZE, osdp_refresh,
			NULL, NULL, NULL, K_PRIO_COOP(2), 0, K_NO_WAIT);
	return 0;
}

SYS_INIT(osdp_init, POST_KERNEL, 10);
