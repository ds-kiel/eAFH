/*
 * Copyright Runtime.io 2018. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Shell transport for the mcumgr SMP protocol.
 */

#include <string.h>
#include <zephyr.h>
#include <init.h>
#include "net/buf.h"
#include "mgmt/mgmt.h"
#include "mgmt/mcumgr/serial.h"
#include "mgmt/mcumgr/buf.h"
#include "mgmt/mcumgr/smp.h"
#include "mgmt/mcumgr/smp_shell.h"
#include "drivers/uart.h"
#include "syscalls/uart.h"
#include "shell/shell.h"
#include "shell/shell_uart.h"

static struct zephyr_smp_transport smp_shell_transport;

static struct mcumgr_serial_rx_ctxt smp_shell_rx_ctxt;

/** SMP mcumgr frame fragments. */
enum smp_shell_esc_mcumgr {
	ESC_MCUMGR_PKT_1,
	ESC_MCUMGR_PKT_2,
	ESC_MCUMGR_FRAG_1,
	ESC_MCUMGR_FRAG_2,
};

/** These states indicate whether an mcumgr frame is being received. */
enum smp_shell_mcumgr_state {
	SMP_SHELL_MCUMGR_STATE_NONE,
	SMP_SHELL_MCUMGR_STATE_HEADER,
	SMP_SHELL_MCUMGR_STATE_PAYLOAD
};

static int read_mcumgr_byte(struct smp_shell_data *data, uint8_t byte)
{
	bool frag_1;
	bool frag_2;
	bool pkt_1;
	bool pkt_2;

	pkt_1 = atomic_test_bit(&data->esc_state, ESC_MCUMGR_PKT_1);
	pkt_2 = atomic_test_bit(&data->esc_state, ESC_MCUMGR_PKT_2);
	frag_1 = atomic_test_bit(&data->esc_state, ESC_MCUMGR_FRAG_1);
	frag_2 = atomic_test_bit(&data->esc_state, ESC_MCUMGR_FRAG_2);

	if (pkt_2 || frag_2) {
		/* Already fully framed. */
		return SMP_SHELL_MCUMGR_STATE_PAYLOAD;
	}

	if (pkt_1) {
		if (byte == MCUMGR_SERIAL_HDR_PKT_2) {
			/* Final framing byte received. */
			atomic_set_bit(&data->esc_state, ESC_MCUMGR_PKT_2);
			return SMP_SHELL_MCUMGR_STATE_PAYLOAD;
		}
	} else if (frag_1) {
		if (byte == MCUMGR_SERIAL_HDR_FRAG_2) {
			/* Final framing byte received. */
			atomic_set_bit(&data->esc_state, ESC_MCUMGR_FRAG_2);
			return SMP_SHELL_MCUMGR_STATE_PAYLOAD;
		}
	} else {
		if (byte == MCUMGR_SERIAL_HDR_PKT_1) {
			/* First framing byte received. */
			atomic_set_bit(&data->esc_state, ESC_MCUMGR_PKT_1);
			return SMP_SHELL_MCUMGR_STATE_HEADER;
		} else if (byte == MCUMGR_SERIAL_HDR_FRAG_1) {
			/* First framing byte received. */
			atomic_set_bit(&data->esc_state, ESC_MCUMGR_FRAG_1);
			return SMP_SHELL_MCUMGR_STATE_HEADER;
		}
	}

	/* Non-mcumgr byte received. */
	return SMP_SHELL_MCUMGR_STATE_NONE;
}

size_t smp_shell_rx_bytes(struct smp_shell_data *data, const uint8_t *bytes,
			  size_t size)
{
	size_t consumed = 0;		/* Number of bytes consumed by SMP */

	/* Process all bytes that are accepted as SMP commands. */
	while (size != consumed) {
		uint8_t byte = bytes[consumed];
		int mcumgr_state = read_mcumgr_byte(data, byte);

		if (mcumgr_state == SMP_SHELL_MCUMGR_STATE_NONE) {
			break;
		}

		if (data->cur < sizeof(data->mcumgr_buff) - 1) {
			data->mcumgr_buff[data->cur] = byte;
			++data->cur;
		}

		/* Newline in payload means complete frame */
		if (mcumgr_state == SMP_SHELL_MCUMGR_STATE_PAYLOAD &&
		    byte == '\n') {
			data->mcumgr_buff[data->cur] = '\0';
			data->cmd_rdy = true;
			atomic_clear_bit(&data->esc_state, ESC_MCUMGR_PKT_1);
			atomic_clear_bit(&data->esc_state, ESC_MCUMGR_PKT_2);
			atomic_clear_bit(&data->esc_state, ESC_MCUMGR_FRAG_1);
			atomic_clear_bit(&data->esc_state, ESC_MCUMGR_FRAG_2);
			data->cur = 0U;
		}

		++consumed;
	}

	return consumed;
}

void smp_shell_process(struct smp_shell_data *data)
{
	if (data->cmd_rdy) {
		data->cmd_rdy = false;
		struct net_buf *nb;
		int line_len;

		/* Strip the trailing newline. */
		line_len = strlen(data->mcumgr_buff) - 1;

		nb = mcumgr_serial_process_frag(&smp_shell_rx_ctxt,
						data->mcumgr_buff,
						line_len);
		if (nb != NULL) {
			zephyr_smp_rx_req(&smp_shell_transport, nb);
		}
	}
}

static uint16_t smp_shell_get_mtu(const struct net_buf *nb)
{
	return CONFIG_MCUMGR_SMP_SHELL_MTU;
}

static int smp_shell_tx_raw(const void *data, int len, void *arg)
{
	const struct shell *const sh = shell_backend_uart_get_ptr();
	const struct shell_uart *const su = sh->iface->ctx;
	const struct shell_uart_ctrl_blk *const scb = su->ctrl_blk;
	const uint8_t *out = data;

	while ((out != NULL) && (len != 0)) {
		uart_poll_out(scb->dev, *out);
		++out;
		--len;
	}

	return 0;
}

static int smp_shell_tx_pkt(struct zephyr_smp_transport *zst,
			    struct net_buf *nb)
{
	int rc;

	rc = mcumgr_serial_tx_pkt(nb->data, nb->len, smp_shell_tx_raw, NULL);
	mcumgr_buf_free(nb);

	return rc;
}

int smp_shell_init(void)
{
	zephyr_smp_transport_init(&smp_shell_transport, smp_shell_tx_pkt,
				  smp_shell_get_mtu, NULL, NULL);

	return 0;
}
