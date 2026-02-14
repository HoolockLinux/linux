// SPDX-License-Identifier: GPL-2.0
//
// Apple SoC SPI device driver
//
// Copyright The Asahi Linux Contributors
//
// Based on spi-sifive.c, Copyright 2018 SiFive, Inc.

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>

#define APPLE_SPI_CTRL			0x000
#define APPLE_SPI_CTRL_RUN		BIT(0)
#define APPLE_SPI_CTRL_TX_RESET		BIT(2)
#define APPLE_SPI_CTRL_RX_RESET		BIT(3)

#define APPLE_SPI_CFG			0x004
#define APPLE_S5L_SPI_CFG_AUTO_TX	BIT(0)
#define APPLE_SPI_CFG_CPHA		BIT(1)
#define APPLE_SPI_CFG_CPOL		BIT(2)
#define APPLE_SPI_CFG_MODE		GENMASK(6, 5)
#define APPLE_SPI_CFG_MODE_POLLED	0
#define APPLE_SPI_CFG_MODE_IRQ		1
#define APPLE_SPI_CFG_MODE_DMA		2
#define APPLE_S5L_SPI_CFG_ROLESEL	BIT(3)
#define APPLE_S5L_SPI_CFG_CLOCK_ENABLE	BIT(4)
#define APPLE_SPI_CFG_IE_RXCOMPLETE	BIT(7)
#define APPLE_SPI_CFG_IE_TXRXTHRESH	BIT(8)
#define APPLE_S5L_SPI_CFG_IE_WR_COLLISION	BIT(10)
#define APPLE_S5L_SPI_CFG_IE_HUNG		BIT(11)
#define APPLE_S5L_SPI_CFG_IE_RXL_TIMEOUT	BIT(12)
#define APPLE_S5L_SPI_CFG_MSB_FIRST	BIT(13)
#define APPLE_SPI_CFG_LSB_FIRST		BIT(13)
#define APPLE_S5L_SPI_CFG_CLOCK_SELECT	BIT(14)
#define APPLE_SPI_CFG_WORD_SIZE		GENMASK(16, 15)
#define APPLE_SPI_CFG_WORD_SIZE_8B	0
#define APPLE_SPI_CFG_WORD_SIZE_16B	1
#define APPLE_SPI_CFG_WORD_SIZE_32B	2
#define APPLE_SPI_CFG_FIFO_THRESH	GENMASK(18, 17)
#define APPLE_SPI_CFG_FIFO_THRESH_8B	0
#define APPLE_SPI_CFG_FIFO_THRESH_4B	1
#define APPLE_SPI_CFG_FIFO_THRESH_1B	2
#define APPLE_S5L_SPI_CFG_FIFO_THRESH_4B	0
#define APPLE_S5L_SPI_CFG_FIFO_THRESH_2B	1
#define APPLE_S5L_SPI_CFG_FIFO_THRESH_1B	2
#define APPLE_SPI_CFG_IE_TXCOMPLETE	BIT(21)

#define APPLE_SPI_STATUS			0x008
#define APPLE_SPI_STATUS_RXCOMPLETE		BIT(0)
#define APPLE_SPI_STATUS_TXRXTHRESH		BIT(1)
#define APPLE_SPI_STATUS_TXCOMPLETE		BIT(2)
#define APPLE_S5L_SPI_STATUS_WR_COLLISION	BIT(3)
#define APPLE_S5L_SPI_STATUS_HUNG		BIT(4)
#define APPLE_S5L_SPI_STATUS_RXL_TIMEOUT	BIT(5)
#define APPLE_S5L_SPI_STATUS_TXFIFO		GENMASK(10, 6)
#define APPLE_S5L_SPI_STATUS_RXFIFO		GENMASK(15, 11)
#define APPLE_S5L_SPI_STATUS_TXCOMPLETE		BIT(22)

#define APPLE_SPI_PIN			0x00c
#define APPLE_SPI_PIN_KEEP_MOSI		BIT(0)
#define APPLE_SPI_PIN_CS		BIT(1)

#define APPLE_SPI_TXDATA		0x010
#define APPLE_SPI_RXDATA		0x020
#define APPLE_SPI_CLKDIV		0x030
#define APPLE_SPI_CLKDIV_MAX		0x7ff
#define APPLE_SPI_RXCNT			0x034
#define APPLE_SPI_WORD_DELAY		0x038
#define APPLE_SPI_TXCNT			0x04c

#define APPLE_SPI_FIFOSTAT		0x10c
#define APPLE_SPI_FIFOSTAT_TXFULL	BIT(4)
#define APPLE_SPI_FIFOSTAT_LEVEL_TX	GENMASK(15, 8)
#define APPLE_SPI_FIFOSTAT_RXEMPTY	BIT(20)
#define APPLE_SPI_FIFOSTAT_LEVEL_RX	GENMASK(31, 24)

#define APPLE_SPI_IE_XFER		0x130
#define APPLE_SPI_IF_XFER		0x134
#define APPLE_SPI_XFER_RXCOMPLETE	BIT(0)
#define APPLE_SPI_XFER_TXCOMPLETE	BIT(1)

#define APPLE_SPI_IE_FIFO		0x138
#define APPLE_SPI_IF_FIFO		0x13c
#define APPLE_SPI_FIFO_RXTHRESH		BIT(4)
#define APPLE_SPI_FIFO_TXTHRESH		BIT(5)
#define APPLE_SPI_FIFO_RXFULL		BIT(8)
#define APPLE_SPI_FIFO_TXEMPTY		BIT(9)
#define APPLE_SPI_FIFO_RXUNDERRUN	BIT(16)
#define APPLE_SPI_FIFO_TXOVERFLOW	BIT(17)

#define APPLE_SPI_SHIFTCFG		0x150
#define APPLE_SPI_SHIFTCFG_CLK_ENABLE	BIT(0)
#define APPLE_SPI_SHIFTCFG_CS_ENABLE	BIT(1)
#define APPLE_SPI_SHIFTCFG_AND_CLK_DATA	BIT(8)
#define APPLE_SPI_SHIFTCFG_CS_AS_DATA	BIT(9)
#define APPLE_SPI_SHIFTCFG_TX_ENABLE	BIT(10)
#define APPLE_SPI_SHIFTCFG_RX_ENABLE	BIT(11)
#define APPLE_SPI_SHIFTCFG_BITS		GENMASK(21, 16)
#define APPLE_SPI_SHIFTCFG_OVERRIDE_CS	BIT(24)

#define APPLE_SPI_PINCFG		0x154
#define APPLE_SPI_PINCFG_KEEP_CLK	BIT(0)
#define APPLE_SPI_PINCFG_KEEP_CS	BIT(1)
#define APPLE_SPI_PINCFG_KEEP_MOSI	BIT(2)
#define APPLE_SPI_PINCFG_CLK_IDLE_VAL	BIT(8)
#define APPLE_SPI_PINCFG_CS_IDLE_VAL	BIT(9)
#define APPLE_SPI_PINCFG_MOSI_IDLE_VAL	BIT(10)

#define APPLE_SPI_DELAY_PRE		0x160
#define APPLE_SPI_DELAY_POST		0x168
#define APPLE_SPI_DELAY_ENABLE		BIT(0)
#define APPLE_SPI_DELAY_NO_INTERBYTE	BIT(1)
#define APPLE_SPI_DELAY_SET_SCK		BIT(4)
#define APPLE_SPI_DELAY_SET_MOSI	BIT(6)
#define APPLE_SPI_DELAY_SCK_VAL		BIT(8)
#define APPLE_SPI_DELAY_MOSI_VAL	BIT(12)

#define APPLE_S5L_SPI_CFG_IE_FLAGS (APPLE_SPI_CFG_IE_RXCOMPLETE | \
		APPLE_SPI_CFG_IE_TXCOMPLETE | APPLE_SPI_CFG_IE_TXRXTHRESH)

#define APPLE_SPI_FIFO_DEPTH		16

/*
 * The slowest refclock available is 24MHz, the highest divider is 0x7ff,
 * the largest word size is 32 bits, the FIFO depth is 16, the maximum
 * intra-word delay is 0xffff refclocks. So the maximum time a transfer
 * cycle can take is:
 *
 * (0x7ff * 32 + 0xffff) * 16 / 24e6 Hz ~= 87ms
 *
 * Double it and round it up to 200ms for good measure.
 */
#define APPLE_SPI_TIMEOUT_MS		200

enum apple_spi_type {
	APPLE_SPI_TYPE_S5L,
	APPLE_SPI_TYPE_MC,
};

struct apple_spi {
	struct device *dev;
	void __iomem          *regs;        /* MMIO register address */
	struct clk            *clk;         /* bus clock */
	struct completion     done;         /* wake-up from interrupt */
	int irq;
	spinlock_t lock;
	u32 status_bit;
	enum apple_spi_type   type;
};

static inline void reg_write(struct apple_spi *spi, int offset, u32 value)
{
	writel_relaxed(value, spi->regs + offset);
}

static inline u32 reg_read(struct apple_spi *spi, int offset)
{
	return readl_relaxed(spi->regs + offset);
}

static inline void reg_mask(struct apple_spi *spi, int offset, u32 clear, u32 set)
{
	u32 val = reg_read(spi, offset);

	val &= ~clear;
	val |= set;
	reg_write(spi, offset, val);
}

static void apple_s5l_spi_init(struct apple_spi *spi)
{
	/* Set CS high (inactive) and disable override and auto-CS */
	reg_write(spi, APPLE_SPI_PIN, APPLE_SPI_PIN_CS);

	/* Reset FIFOs */
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_RX_RESET | APPLE_SPI_CTRL_TX_RESET);

	/* Configure defaults, Disable IRQs */
	reg_write(spi, APPLE_SPI_CFG,
		  FIELD_PREP(APPLE_SPI_CFG_FIFO_THRESH, APPLE_S5L_SPI_CFG_FIFO_THRESH_1B) |
		  FIELD_PREP(APPLE_SPI_CFG_MODE, APPLE_SPI_CFG_MODE_IRQ) |
		  FIELD_PREP(APPLE_SPI_CFG_WORD_SIZE, APPLE_SPI_CFG_WORD_SIZE_8B) |
		  APPLE_S5L_SPI_CFG_CLOCK_SELECT | APPLE_S5L_SPI_CFG_ROLESEL |
		  APPLE_S5L_SPI_CFG_CLOCK_ENABLE);

	/* Reset existing IRQs */
	reg_write(spi, APPLE_SPI_STATUS, reg_read(spi, APPLE_SPI_STATUS));

	/* Disable delays */
	reg_write(spi, APPLE_SPI_WORD_DELAY, 0);
}

static void apple_spi_init(struct apple_spi *spi)
{
	/* Set CS high (inactive) and disable override and auto-CS */
	reg_write(spi, APPLE_SPI_PIN, APPLE_SPI_PIN_CS);
	reg_mask(spi, APPLE_SPI_SHIFTCFG, APPLE_SPI_SHIFTCFG_OVERRIDE_CS, 0);
	reg_mask(spi, APPLE_SPI_PINCFG, APPLE_SPI_PINCFG_CS_IDLE_VAL, APPLE_SPI_PINCFG_KEEP_CS);

	/* Reset FIFOs */
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_RX_RESET | APPLE_SPI_CTRL_TX_RESET);

	/* Configure defaults */
	reg_write(spi, APPLE_SPI_CFG,
		  FIELD_PREP(APPLE_SPI_CFG_FIFO_THRESH, APPLE_SPI_CFG_FIFO_THRESH_8B) |
		  FIELD_PREP(APPLE_SPI_CFG_MODE, APPLE_SPI_CFG_MODE_IRQ) |
		  FIELD_PREP(APPLE_SPI_CFG_WORD_SIZE, APPLE_SPI_CFG_WORD_SIZE_8B));

	/* Disable IRQs */
	reg_write(spi, APPLE_SPI_IE_FIFO, 0);
	reg_write(spi, APPLE_SPI_IE_XFER, 0);

	/* Disable delays */
	reg_write(spi, APPLE_SPI_DELAY_PRE, 0);
	reg_write(spi, APPLE_SPI_DELAY_POST, 0);
}

static int apple_spi_prepare_message(struct spi_controller *ctlr, struct spi_message *msg)
{
	struct apple_spi *spi = spi_controller_get_devdata(ctlr);
	struct spi_device *device = msg->spi;

	u32 cfg = ((device->mode & SPI_CPHA ? APPLE_SPI_CFG_CPHA : 0) |
		   (device->mode & SPI_CPOL ? APPLE_SPI_CFG_CPOL : 0));

	if (spi->type == APPLE_SPI_TYPE_S5L)
		cfg |= (device->mode & SPI_LSB_FIRST ? APPLE_SPI_CFG_LSB_FIRST : 0);
	else
		cfg |= (device->mode & SPI_LSB_FIRST ? 0 : APPLE_S5L_SPI_CFG_MSB_FIRST);

	/*
	 * Update core config. This works because APPLE_SPI_CFG_LSB_FIRST and
	 * APPLE_S5L_SPI_CFG_MSB_FIRST is the same bit but with reversed
	 * meaning.
	 */
	reg_mask(spi, APPLE_SPI_CFG,
		 APPLE_SPI_CFG_CPHA | APPLE_SPI_CFG_CPOL | APPLE_SPI_CFG_LSB_FIRST, cfg);

	return 0;
}

static void apple_spi_set_cs(struct spi_device *device, bool is_high)
{
	struct apple_spi *spi = spi_controller_get_devdata(device->controller);

	reg_mask(spi, APPLE_SPI_PIN, APPLE_SPI_PIN_CS, is_high ? APPLE_SPI_PIN_CS : 0);
}

static bool apple_spi_prep_transfer(struct apple_spi *spi, struct spi_transfer *t)
{
	u32 cr, fifo_threshold;

	/* Calculate and program the clock rate */
	cr = DIV_ROUND_UP(clk_get_rate(spi->clk), t->speed_hz);
	reg_write(spi, APPLE_SPI_CLKDIV, min_t(u32, cr, APPLE_SPI_CLKDIV_MAX));

	/* Update bits per word */
	if (spi->type == APPLE_SPI_TYPE_S5L)
		reg_mask(spi, APPLE_SPI_CFG, APPLE_SPI_CFG_WORD_SIZE,
			 FIELD_PREP(APPLE_SPI_CFG_WORD_SIZE, t->bits_per_word >> 4));
	else
		reg_mask(spi, APPLE_SPI_SHIFTCFG, APPLE_SPI_SHIFTCFG_BITS,
			 FIELD_PREP(APPLE_SPI_SHIFTCFG_BITS, t->bits_per_word));

	/* We will want to poll if the time we need to wait is
	 * less than the context switching time.
	 * Let's call that threshold 5us. The operation will take:
	 *    bits_per_word * fifo_threshold / hz <= 5 * 10^-6
	 *    200000 * bits_per_word * fifo_threshold <= hz
	 */
	fifo_threshold = APPLE_SPI_FIFO_DEPTH / 2;
	return (200000 * t->bits_per_word * fifo_threshold) <= t->speed_hz;
}

static irqreturn_t apple_s5l_spi_irq(int irq, void *dev_id)
{
	struct apple_spi *spi = dev_id;
	irqreturn_t retval = IRQ_NONE;
	unsigned long flags;

	u32 status = reg_read(spi, APPLE_SPI_STATUS);
	reg_write(spi, APPLE_SPI_STATUS, status); /* Ack all IRQs */

	if (status & APPLE_S5L_SPI_STATUS_WR_COLLISION) {
		dev_err(spi->dev, "[IRQ] Data collision Error\n");
		retval = IRQ_HANDLED;
	}

	if (status & APPLE_S5L_SPI_STATUS_HUNG) {
		//dev_info(spi->dev, "[IRQ] SPI Hang up");
		retval = IRQ_HANDLED;
	}

	if (status & APPLE_S5L_SPI_STATUS_RXL_TIMEOUT) {
		//dev_info_ratelimited(spi->dev, "[IRQ] SPI Rx Data Leftover timeout\n");
		retval = IRQ_HANDLED;
	}

	spin_lock_irqsave(&spi->lock, flags);

	if (status & spi->status_bit) {
		/* Disable interrupts until next transfer */
		reg_mask(spi, APPLE_SPI_CFG, APPLE_S5L_SPI_CFG_IE_FLAGS, 0);
		spi->status_bit = status;
		complete(&spi->done);
		dev_info(spi->dev, "[IRQ] SPI Complete! 0x%x\n", status);
	}

	spin_unlock_irqrestore(&spi->lock, flags);

	if (retval == IRQ_NONE) {
		dev_err(spi->dev, "[IRQ] unhandled irq status: 0x%x\n", status);
	}

	return retval;
}

static irqreturn_t apple_spi_irq(int irq, void *dev_id)
{
	struct apple_spi *spi = dev_id;
	u32 fifo = reg_read(spi, APPLE_SPI_IF_FIFO) & reg_read(spi, APPLE_SPI_IE_FIFO);
	u32 xfer = reg_read(spi, APPLE_SPI_IF_XFER) & reg_read(spi, APPLE_SPI_IE_XFER);

	if (fifo || xfer) {
		/* Disable interrupts until next transfer */
		reg_write(spi, APPLE_SPI_IE_XFER, 0);
		reg_write(spi, APPLE_SPI_IE_FIFO, 0);
		complete(&spi->done);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

// out_status: what got handled
static int apple_s5l_spi_wait(struct apple_spi *spi, u32 status_bit, u32 *out_status, int poll)
{
	int ret = 0;
	if (poll) {
		/*
		 * Disable IRQ at irqchip level since there is no way to disable
		 * individual condition from firing IRQ without also disabling the
		 * indication in the status register.
		 */
		disable_irq(spi->irq);
		dev_info(spi->dev, "[POLL] Now waiting for 0x%x\n", status_bit);

		unsigned long timeout = jiffies + APPLE_SPI_TIMEOUT_MS * HZ / 1000;
		u32 status;

		reg_mask(spi, APPLE_SPI_CFG, APPLE_SPI_CFG_MODE | APPLE_S5L_SPI_CFG_IE_FLAGS,
			      APPLE_S5L_SPI_CFG_IE_FLAGS | FIELD_PREP(APPLE_SPI_CFG_MODE, APPLE_SPI_CFG_MODE_POLLED));

		do {
			status = reg_read(spi, APPLE_SPI_STATUS);
			reg_write(spi, APPLE_SPI_STATUS, status);
			*out_status |= status;

			if (status & APPLE_S5L_SPI_STATUS_WR_COLLISION) {
				//dev_err_ratelimited(spi->dev, "[POLL] Data collision Error\n");
			}
			if (status & APPLE_S5L_SPI_STATUS_HUNG) {
				//dev_info_ratelimited(spi->dev, "[POLL] SPI Hang Up\n");
			}
			if (status & APPLE_S5L_SPI_STATUS_RXL_TIMEOUT) {
				//dev_info_ratelimited(spi->dev, "[POLL] SPI Rx Data Leftover Timeout\n");
			}

			if (time_after(jiffies, timeout)) {
				ret = -ETIMEDOUT;
				break;
			}
		} while (!(status & status_bit));

		enable_irq(spi->irq);
		//dev_info(spi->dev, "[POLL] SPI Wait completed with ret: %d\n", ret);
	} else {
		unsigned long flags;

		dev_info(spi->dev, "[IRQ] Now waiting for 0x%x\n", status_bit);

		spin_lock_irqsave(&spi->lock, flags);
		spi->status_bit = status_bit;
		spin_unlock_irqrestore(&spi->lock, flags);

		reg_mask(spi, APPLE_SPI_CFG, APPLE_SPI_CFG_MODE | APPLE_S5L_SPI_CFG_IE_FLAGS,
					     FIELD_PREP(APPLE_SPI_CFG_MODE, APPLE_SPI_CFG_MODE_IRQ)
					     | APPLE_S5L_SPI_CFG_IE_FLAGS);
		reinit_completion(&spi->done);

		if (!wait_for_completion_timeout(&spi->done,
						 msecs_to_jiffies(APPLE_SPI_TIMEOUT_MS)))
			ret = -ETIMEDOUT;

		/* IRQ handler set spi->status_bit status when handling IRQ */
		*out_status |= spi->status_bit;
		spi->status_bit = 0;
	}

	return ret;
}

static int apple_spi_wait(struct apple_spi *spi, u32 fifo_bit, u32 xfer_bit, int poll)
{
	int ret = 0;

	if (poll) {
		u32 fifo, xfer;
		unsigned long timeout = jiffies + APPLE_SPI_TIMEOUT_MS * HZ / 1000;

		do {
			fifo = reg_read(spi, APPLE_SPI_IF_FIFO);
			xfer = reg_read(spi, APPLE_SPI_IF_XFER);
			if (time_after(jiffies, timeout)) {
				ret = -ETIMEDOUT;
				break;
			}
		} while (!((fifo & fifo_bit) || (xfer & xfer_bit)));
	} else {
		reinit_completion(&spi->done);
		reg_write(spi, APPLE_SPI_IE_XFER, xfer_bit);
		reg_write(spi, APPLE_SPI_IE_FIFO, fifo_bit);

		if (!wait_for_completion_timeout(&spi->done,
						 msecs_to_jiffies(APPLE_SPI_TIMEOUT_MS)))
			ret = -ETIMEDOUT;

		reg_write(spi, APPLE_SPI_IE_XFER, 0);
		reg_write(spi, APPLE_SPI_IE_FIFO, 0);
	}

	return ret;
}

static void apple_spi_tx(struct apple_spi *spi, const void **tx_ptr, u32 *left,
			 unsigned int bytes_per_word)
{
	u32 inuse, words, wrote;

	if (!*tx_ptr)
		return;

	if (spi->type == APPLE_SPI_TYPE_S5L)
		inuse = FIELD_GET(APPLE_S5L_SPI_STATUS_TXFIFO, reg_read(spi, APPLE_SPI_STATUS));
	else
		inuse = FIELD_GET(APPLE_SPI_FIFOSTAT_LEVEL_TX, reg_read(spi, APPLE_SPI_FIFOSTAT));
	words = wrote = min_t(u32, *left, APPLE_SPI_FIFO_DEPTH - inuse);

	if (!words)
		return;

	*left -= words;

	switch (bytes_per_word) {
	case 1: {
		const u8 *p = *tx_ptr;

		while (words--)
			reg_write(spi, APPLE_SPI_TXDATA, *p++);
		break;
	}
	case 2: {
		const u16 *p = *tx_ptr;

		while (words--)
			reg_write(spi, APPLE_SPI_TXDATA, *p++);
		break;
	}
	case 4: {
		const u32 *p = *tx_ptr;

		while (words--)
			reg_write(spi, APPLE_SPI_TXDATA, *p++);
		break;
	}
	default:
		WARN_ON(1);
	}

	*tx_ptr = ((u8 *)*tx_ptr) + bytes_per_word * wrote;
}

static void apple_spi_rx(struct apple_spi *spi, void **rx_ptr, u32 *left,
			 unsigned int bytes_per_word)
{
	u32 words, read;

	if (!*rx_ptr)
		return;

	if (spi->type == APPLE_SPI_TYPE_S5L)
		words = FIELD_GET(APPLE_S5L_SPI_STATUS_RXFIFO, reg_read(spi, APPLE_SPI_STATUS));
	else
		words = read = FIELD_GET(APPLE_SPI_FIFOSTAT_LEVEL_RX, reg_read(spi, APPLE_SPI_FIFOSTAT));
	WARN_ON(words > *left);

	if (!words)
		return;

	*left -= min_t(u32, *left, words);

	switch (bytes_per_word) {
	case 1: {
		u8 *p = *rx_ptr;

		while (words--)
			*p++ = reg_read(spi, APPLE_SPI_RXDATA);
		break;
	}
	case 2: {
		u16 *p = *rx_ptr;

		while (words--)
			*p++ = reg_read(spi, APPLE_SPI_RXDATA);
		break;
	}
	case 4: {
		u32 *p = *rx_ptr;

		while (words--)
			*p++ = reg_read(spi, APPLE_SPI_RXDATA);
		break;
	}
	default:
		WARN_ON(1);
	}

	*rx_ptr = ((u8 *)*rx_ptr) + bytes_per_word * read;
}

static int apple_spi_transfer_one(struct spi_controller *ctlr, struct spi_device *device,
				  struct spi_transfer *t)
{
	struct apple_spi *spi = spi_controller_get_devdata(ctlr);
	bool poll = apple_spi_prep_transfer(spi, t);
	const void *tx_ptr = t->tx_buf;
	void *rx_ptr = t->rx_buf;
	unsigned int bytes_per_word;
	u32 words, remaining_tx, remaining_rx;
	u32 xfer_flags = 0;
	u32 fifo_flags, out_status;
	int retries = 100;
	int ret = 0;

	if (t->bits_per_word > 16)
		bytes_per_word = 4;
	else if (t->bits_per_word > 8)
		bytes_per_word = 2;
	else
		bytes_per_word = 1;

	words = t->len / bytes_per_word;
	remaining_tx = tx_ptr ? words : 0;
	remaining_rx = rx_ptr ? words : 0;
	dev_info(&ctlr->dev, "remaining_tx: %d remaining_rx: %d\n", remaining_tx, remaining_rx);

	/* Reset FIFOs */
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_RX_RESET | APPLE_SPI_CTRL_TX_RESET);

	if (spi->type == APPLE_SPI_TYPE_S5L) {
		/* Clear IRQ flags */
		reg_write(spi, APPLE_SPI_STATUS, reg_read(spi, APPLE_SPI_STATUS));

		/* Determine transfer completion flags we wait for */
		//if (tx_ptr)
		//	xfer_flags |= APPLE_S5L_SPI_STATUS_TXCOMPLETE;
		if (rx_ptr) {
			if (poll)
				xfer_flags |= APPLE_SPI_STATUS_RXCOMPLETE;
			reg_mask(spi, APPLE_SPI_CFG, APPLE_S5L_SPI_CFG_AUTO_TX, APPLE_S5L_SPI_CFG_AUTO_TX);
		}

		if (tx_ptr || !poll)
			xfer_flags = APPLE_SPI_STATUS_TXRXTHRESH;

		dev_info(&ctlr->dev, "setting xfer_flags 0x%x tx: %d rx: %d", xfer_flags, !!tx_ptr, !!rx_ptr);
	} else {
		/* Clear IRQ flags */
		reg_write(spi, APPLE_SPI_IF_XFER, ~0);
		reg_write(spi, APPLE_SPI_IF_FIFO, ~0);

		/* Determine transfer completion flags we wait for */
		if (tx_ptr)
			xfer_flags |= APPLE_SPI_XFER_TXCOMPLETE;
		if (rx_ptr)
			xfer_flags |= APPLE_SPI_XFER_RXCOMPLETE;
	}

	/* Set transfer length */
	reg_write(spi, APPLE_SPI_TXCNT, remaining_tx);
	reg_write(spi, APPLE_SPI_RXCNT, remaining_rx);

	/* Prime transmit FIFO */
	apple_spi_tx(spi, &tx_ptr, &remaining_tx, bytes_per_word);

	/* Start transfer */
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_RUN);

	/* TX again since a few words get popped off immediately */
	apple_spi_tx(spi, &tx_ptr, &remaining_tx, bytes_per_word);

	while (xfer_flags) {
		fifo_flags = 0;
		out_status = 0;

		if (remaining_tx)
			fifo_flags |= APPLE_SPI_FIFO_TXTHRESH;
		if (remaining_rx)
			fifo_flags |= APPLE_SPI_FIFO_RXTHRESH;

		/* Wait for anything to happen */
		if (spi->type == APPLE_SPI_TYPE_S5L)
			ret = apple_s5l_spi_wait(spi, xfer_flags, &out_status, poll);
		else
			ret = apple_spi_wait(spi, fifo_flags, xfer_flags, poll);

		if (ret) {
			dev_err(&ctlr->dev, "transfer timed out (remaining %d tx, %d rx) poll %d status 0x%x out_status %x\n",
				remaining_tx, remaining_rx, poll, reg_read(spi, APPLE_SPI_STATUS), out_status);
			goto err;
		}


		/* Stop waiting on transfer halves once they complete */
		if (spi->type == APPLE_SPI_TYPE_MC)
			xfer_flags &= ~reg_read(spi, APPLE_SPI_IF_XFER);
		else if (spi->type == APPLE_SPI_TYPE_S5L)
			xfer_flags &= ~out_status;

		/* Transmit and receive everything we can */
		apple_spi_tx(spi, &tx_ptr, &remaining_tx, bytes_per_word);
		apple_spi_rx(spi, &rx_ptr, &remaining_rx, bytes_per_word);
	}

	/*
	 * Sometimes the transfer completes before the last word is in the RX FIFO.
	 * Normally one retry is all it takes to get the last word out.
	 */
	while (remaining_rx && retries--)
		apple_spi_rx(spi, &rx_ptr, &remaining_rx, bytes_per_word);

	if (remaining_tx)
		dev_err(&ctlr->dev, "transfer completed with %d words left to transmit\n",
			remaining_tx);
	if (remaining_rx)
		dev_err(&ctlr->dev, "transfer completed with %d words left to receive\n",
			remaining_rx);


err:
	if (spi->type == APPLE_SPI_TYPE_MC) {
		fifo_flags = reg_read(spi, APPLE_SPI_IF_FIFO);
		WARN_ON(fifo_flags & APPLE_SPI_FIFO_TXOVERFLOW);
		WARN_ON(fifo_flags & APPLE_SPI_FIFO_RXUNDERRUN);
	} else
		reg_mask(spi, APPLE_SPI_CFG, APPLE_S5L_SPI_CFG_AUTO_TX, 0);

	/* Stop transfer */
	reg_write(spi, APPLE_SPI_CTRL, 0);

	return ret;
}

bool defered = false;

static int apple_spi_probe(struct platform_device *pdev)
{
	struct apple_spi *spi;
	int ret, irq;
	struct spi_controller *ctlr;

	if (!defered) {
		defered = true;
		return -EPROBE_DEFER;
	}

	ctlr = devm_spi_alloc_host(&pdev->dev, sizeof(struct apple_spi));
	if (!ctlr)
		return -ENOMEM;

	spi = spi_controller_get_devdata(ctlr);
	spi->dev = &pdev->dev;
	init_completion(&spi->done);

	spi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spi->regs))
		return PTR_ERR(spi->regs);

	spi->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(spi->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(spi->clk),
				     "Unable to find or enable bus clock\n");

	spi->type = (enum apple_spi_type)of_device_get_match_data(&pdev->dev);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	spi->irq = irq;
	spin_lock_init(&spi->lock);

	if (spi->type == APPLE_SPI_TYPE_S5L)
		ret = devm_request_irq(&pdev->dev, irq, apple_s5l_spi_irq, 0,
			       dev_name(&pdev->dev), spi);
	else
		ret = devm_request_irq(&pdev->dev, irq, apple_spi_irq, 0,
			       dev_name(&pdev->dev), spi);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Unable to bind to interrupt\n");

	ctlr->dev.of_node = pdev->dev.of_node;
	ctlr->bus_num = pdev->id;
	ctlr->num_chipselect = 1;
	ctlr->mode_bits = SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST;
	ctlr->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 32);
	ctlr->prepare_message = apple_spi_prepare_message;
	ctlr->set_cs = apple_spi_set_cs;
	ctlr->transfer_one = apple_spi_transfer_one;
	ctlr->use_gpio_descriptors = true;
	ctlr->auto_runtime_pm = true;

	pm_runtime_set_active(&pdev->dev);
	ret = devm_pm_runtime_enable(&pdev->dev);
	if (ret < 0)
		return ret;

	if (spi->type == APPLE_SPI_TYPE_S5L)
		apple_s5l_spi_init(spi);
	else
		apple_spi_init(spi);

	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "devm_spi_register_controller failed\n");

	return 0;
}

static const struct of_device_id apple_spi_of_match[] = {
	{ .compatible = "apple,t8103-spi", .data = (void*)APPLE_SPI_TYPE_MC },
	{ .compatible = "apple,s5l8960x-spi", .data = (void*)APPLE_SPI_TYPE_S5L },
	{ .compatible = "apple,spi", .data = (void*)APPLE_SPI_TYPE_MC },
	{}
};
MODULE_DEVICE_TABLE(of, apple_spi_of_match);

static struct platform_driver apple_spi_driver = {
	.probe = apple_spi_probe,
	.driver = {
		.name = "apple-spi",
		.of_match_table = apple_spi_of_match,
	},
};
module_platform_driver(apple_spi_driver);

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("Apple SoC SPI driver");
MODULE_LICENSE("GPL");
