/*
 * Copyright (c) 2019 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_qmspi

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_xec, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t INTERFACE_MODE                     :2;    // bits[1:0]
		uint32_t TX_TRANSFER_ENABLE                 :2;    // bits[3:2]
		uint32_t TX_DMA_ENABLE                      :2;    // bits[5:4]
		uint32_t RX_TRANSFER_ENABLE                 :1;    // bits[6]
		uint32_t RX_DMA_ENABLE                      :2;    // bits[8:7]
		uint32_t CLOSE_TRANFSER_ENABLE              :1;    // bits[9]
		uint32_t TRANSFER_UNITS                     :2;    // bits[11:10]
		uint32_t DESCRIPTION_BUFFER_NEXT_POINTER    :4;    // bits[15:12]
		uint32_t DESCRIPTION_BUFFER_LAST            :1;    // bits[16]
		uint32_t TRANSFER_LENGTH                    :15;   // bits[31:17]
	} bitfld;
}QmspiDescriptionBuffer_0_Register; //0x30


typedef union {
	uint32_t VALUE;
	struct {
		uint32_t DELAY_CS_ON_TO_CLOCK_START    :4;   // bits[3:0]
		uint32_t                               :4;   // bits[7:4]
		uint32_t DELAY_CLK_STOP_TO_CS_OFF      :4;   // bits[11:8]
		uint32_t                               :4;   // bits[15:12]
		uint32_t DELAY_LAST_DATA_HOLD          :4;   // bits[19:16]
		uint32_t                               :4;   // bits[23:20] Reserved
		uint32_t DELAY_CS_OFF_TO_CS_ON         :8;   // bits[31:24]
	} bitfld;
}QmspiChipSelectTimingRegister; //0x28

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t RECEIVE_BUFFER      :32;   // bits[31:0]
	} bitfld;
}QmspiReceiveBufferRegister; //0x24

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t RECEIVE_BUFFER_TTRANSMIT_BUFFERIGGER      :32;   // bits[31:0]
	} bitfld;
}QmspiTransmitBufferRegister; //0x20

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t TRANSMIT_BUFFER_TRIGGER     :16;    // bits[15:0]
		uint32_t RECEIVE_BUFFER_TRIGGER      :16;   // bits[31:16]
	} bitfld;
}QmspiBufferCountTriggerRegister; //0x1C

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t TRANSFER_COMPLETE_ENABLE          :1;    // bits[0]
		uint32_t DMA_COMPLETE_ENABLE               :1;    // bits[1]
		uint32_t TRANSMIT_BUFFER_ERROR_ENABLE      :1;    // bits[2]
		uint32_t RECEIVE_BUFFER_ERROR_ENABLE       :1;    // bits[3]
		uint32_t PROGRAMMING_ERROR_ENABLE          :1;    // bits[4]
		uint32_t                                   :3;    // bits[7:5] Reserved
		uint32_t TRANSMIT_BUFFER_FULL_ENABLE       :1;    // bits[8]
		uint32_t TRANSMIT_BUFFER_EMPTY_ENABLE      :1;    // bits[9]
		uint32_t TRANSMIT_BUFFER_REQUEST_ENABLE    :1;    // bits[10]
		uint32_t                                   :1;    // bits[11]
		uint32_t RECEIVE_BUFFER_FULL_ENABLE        :1;    // bits[12]
		uint32_t RECEIVE_BUFFER_EMPTY_ENABLE       :1;    // bits[13]
		uint32_t RECEIVE_BUFFER_REQUEST_ENABLE     :1;    // bits[14]
		uint32_t                                   :17;   // bits[31:15] Reserved
	} bitfld;
}QmspiInterruptEnableRegister; //0x18

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t TRANSMIT_BUFFER_COUNT      :1;     // bits[15:0]		
		uint32_t RECEIVE_BUFFER_COUNT       :30;    // bits[31:16]
	} bitfld;
}QmspiBufferCountStatusRegister; //0x14

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t TRANSFER_COMPLETE          :1;    // bits[0]
		uint32_t DMA_COMPLETE               :1;    // bits[1]
		uint32_t TRANSMIT_BUFFER_ERROR      :1;    // bits[2]
		uint32_t RECEIVE_BUFFER_ERROR       :1;    // bits[3]
		uint32_t PROGRAMMING_ERROR          :1;    // bits[4]
		uint32_t                            :3;    // bits[7:5]
		uint32_t TRANSMIT_BUFFER_FULL       :1;    // bits[8]
		uint32_t TRANSMIT_BUFFER_EMPTY      :1;    // bits[9]
		uint32_t TRANSMIT_BUFFER_REQUEST    :1;    // bits[10]
		uint32_t TRANSMIT_BUFFER_STALL      :1;    // bits[11]
		uint32_t RECEIVE_BUFFER_FULL        :1;    // bits[12]
		uint32_t RECEIVE_BUFFER_EMPTY       :1;    // bits[13]
		uint32_t RECEIVE_BUFFER_REQUEST     :1;    // bits[14]
		uint32_t RECEIVE_BUFFER_STALL       :1;    // bits[15]
		uint32_t TRANSFER_ACTIVE            :1;    // bits[16]
		uint32_t                            :7;    // bits[23:17]
		uint32_t CURRENT_DESCRIPTION_BUFFER :4;    // bits[27:24]
		uint32_t                            :4;    // bits[31:28] Reserved
	} bitfld;
}QmspiStatusRegister; //0x10

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t WRITE_PROTECT_OUT_VALUE     :1;    // bits[0]
		uint32_t WRITE_PROTECT_OUT_ENABLE    :1;    // bits[1]
		uint32_t HOLD_OUT_VALUE              :1;    // bits[2]
		uint32_t HOLD_OUT_ENABLE             :1;    // bits[3]
		uint32_t PULLDOWN_ON_NOT_SELECTED    :1;    // bits[4]
		uint32_t PULLUP_ON_NOT_SELECTED      :1;    // bits[5]
		uint32_t PULLDOWN_ON_NOT_DRIVEN      :1;    // bits[6]
		uint32_t PULLUP_ON_NOT_DRIVEN        :1;    // bits[1]
		uint32_t                             :24;   // bits[31:8]
	} bitfld;
}QmspiInterfaceControlRegister; //0x0C

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t START                  :1;     // bits[0]
		uint32_t STOP                   :1;     // bits[1]
		uint32_t CLEAR_DATA_BUFFER      :1;     // bits[2]		
		uint32_t                        :30;    // bits[31:3] Reserved
	} bitfld;
}QmspiExecuteRegister; //0x08

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t INTERFACE_MODE                :2;    // bits[1:0]
		uint32_t TX_TRANSFER_ENABLE            :2;    // bits[3:2]
		uint32_t TX_DMA_ENABLE                 :2;    // bits[5:4]
		uint32_t RX_TRANSFER_ENABLE            :1;    // bits[6]
		uint32_t RX_DMA_ENABLE                 :2;    // bits[8:7]
		uint32_t CLOSE_TRANSFER_ENABLE         :1;    // bits[9]
		uint32_t CHIP_TRANSFER_UNITSSELECT     :2;    // bits[11:10]
		uint32_t DESCRIPTION_BUFFER_POINTER    :4;    // bits[15:12]
		uint32_t DESCRIPTION_BUFFER_ENABLE     :1;    // bits[16]
		uint32_t TRANSFER_LENGTH               :15;   // bits[31:17]
	} bitfld;
}QmspiControlRegister; //0x04

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t ACTIVATE       :1;     // bits[0]
		uint32_t SOFT_RESET     :1;     // bits[1]		
		uint32_t SAF_DMA_MODE   :1;     // bits[2]
		uint32_t                :5;     // bits[7:3] Reserved
		uint32_t CPOL           :1;     // bits[8]
		uint32_t CHPA_MOSI      :1;     // bits[9]
		uint32_t CHPA_MISO      :1;     // bits[10]
		uint32_t                :1;     // bits[11] Reserved
		uint32_t CHIP_SELECT    :2;     // bits[13:12]
		uint32_t                :2;     // bits[15:14] Reserved
		uint32_t CLOCK_DIVIDE   :9;     // bits[24:16]
		uint32_t                :7;     // bits[31:25] Reserved
	} bitfld;
}QmspiModeRegister; //0x00


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Device constant configuration parameters */
struct spi_qmspi_config {
	QMSPI_Type *regs;
	uint32_t cs_timing;
	uint8_t girq;
	uint8_t girq_pos;
	uint8_t girq_nvic_aggr;
	uint8_t girq_nvic_direct;
	uint8_t irq_pri;
	uint8_t chip_sel;
	uint8_t width;	/* 1(single), 2(dual), 4(quad) */
	uint8_t unused;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
};

/* Device run time data */
struct spi_qmspi_data {
	struct spi_context ctx;
};

static inline uint32_t descr_rd(QMSPI_Type *regs, uint32_t did)
{
	printk("[spidri][descr_rd] func entry\n");
	uintptr_t raddr = (uintptr_t)regs + MCHP_QMSPI_DESC0_OFS +
			  ((did & MCHP_QMSPI_C_NEXT_DESCR_MASK0) << 2);

	return REG32(raddr);
}

static inline void descr_wr(QMSPI_Type *regs, uint32_t did, uint32_t val)
{
	printk("[spidri][descr_wr] func entry\n");
	uintptr_t raddr = (uintptr_t)regs + MCHP_QMSPI_DESC0_OFS +
			  ((did & MCHP_QMSPI_C_NEXT_DESCR_MASK0) << 2);

	REG32(raddr) = val;
}

static inline void ryan_txb_wr8(QMSPI_Type *regs, uint8_t data8)
{
	printk("[spidri][ryan_txb_wr8] func entry\n");
	uint8_t *qmspi_TransmitBufferReg = ((volatile uint8_t *)(uintptr_t)(&regs->TX_FIFO));
	printk("[spidri][ryan_txb_wr8] qmspi_TransmitBufferReg base address = 0x%X\n", qmspi_TransmitBufferReg);
	QmspiTransmitBufferRegister QmspiTransmitBufferReg;
 	QmspiTransmitBufferReg.bitfld.RECEIVE_BUFFER_TTRANSMIT_BUFFERIGGER = data8;
	printk("[spidri][ryan_txb_wr8] QmspiTransmitBufferReg.bitfld.RECEIVE_BUFFER_TTRANSMIT_BUFFERIGGER = 0x%X\n", QmspiTransmitBufferReg.bitfld.RECEIVE_BUFFER_TTRANSMIT_BUFFERIGGER);
	printk("[spidri][ryan_txb_wr8] QmspiTransmitBufferReg.VALUE = 0x%X\n", QmspiTransmitBufferReg.VALUE);
	*qmspi_TransmitBufferReg = QmspiTransmitBufferReg.VALUE;
}

static inline void txb_wr8(QMSPI_Type *regs, uint8_t data8)
{
	printk("[spidri][txb_wr8] func entry\n");
	REG8(&regs->TX_FIFO) = data8;
}

static inline uint8_t ryan_rxb_rd8(QMSPI_Type *regs)
{
	printk("[spidri][ryan_rxb_rd8] func entry\n");
	uint8_t *qmspi_ReceiveBufferReg = ((volatile uint8_t *)(uintptr_t)(&regs->RX_FIFO));
	printk("[spidri][ryan_rxb_rd8] qmspi_ReceiveBufferReg base address = 0x%X\n", qmspi_ReceiveBufferReg);
	QmspiReceiveBufferRegister QmspiReceiveBufferReg;
	QmspiReceiveBufferReg.bitfld.RECEIVE_BUFFER = *qmspi_ReceiveBufferReg;
	printk("[spidri][ryan_rxb_rd8] QmspiReceiveBufferReg.bitfld.RECEIVE_BUFFER = 0x%X\n", QmspiReceiveBufferReg.bitfld.RECEIVE_BUFFER);
	printk("[spidri][ryan_rxb_rd8] QmspiReceiveBufferReg.VALUE = 0x%X\n", QmspiReceiveBufferReg.VALUE);
	
	return QmspiReceiveBufferReg.VALUE;
}

static inline uint8_t rxb_rd8(QMSPI_Type *regs)
{
	printk("[spidri][rxb_rd8] func entry\n");
	return REG8(&regs->RX_FIFO);
}

/*
 * Program QMSPI frequency.
 * MEC1501 base frequency is 48MHz. QMSPI frequency divider field in the
 * mode register is defined as: 0=maximum divider of 256. Values 1 through
 * 255 divide 48MHz by that value.
 */
static void qmspi_set_frequency(QMSPI_Type *regs, uint32_t freq_hz)
{
	printk("[spidri][qmspi_set_frequency] func entry\n");
	uint32_t div, qmode;
	printk("[spidri][qmspi_set_frequency] freq_hz = %d\n", freq_hz);

	if (freq_hz == 0) {
		div = 0; /* max divider = 256 */
	} else {
		div = MCHP_QMSPI_INPUT_CLOCK_FREQ_HZ / freq_hz;
		printk("[spidri][qmspi_set_frequency] div = %d\n", div);
		if (div == 0) {
			div = 1; /* max freq. divider = 1 */
		} else if (div > 0xffu) {
			div = 0u; /* max divider = 256 */
		}
	}

	qmode = regs->MODE & ~(MCHP_QMSPI_M_FDIV_MASK);
	qmode |= (div << MCHP_QMSPI_M_FDIV_POS) & MCHP_QMSPI_M_FDIV_MASK;
	printk("[spidri][qmspi_set_frequency] qmode = 0x%X\n", qmode);
	regs->MODE = qmode;
}

/*
 * SPI signalling mode: CPOL and CPHA
 * CPOL = 0 is clock idles low, 1 is clock idle high
 * CPHA = 0 Transmitter changes data on trailing of preceding clock cycle.
 *          Receiver samples data on leading edge of clock cycle.
 *        1 Transmitter changes data on leading edge of current clock cycle.
 *          Receiver samples data on the trailing edge of clock cycle.
 * SPI Mode nomenclature:
 * Mode CPOL CPHA
 *  0     0    0
 *  1     0    1
 *  2     1    0
 *  3     1    1
 * MEC1501 has three controls, CPOL, CPHA for output and CPHA for input.
 * SPI frequency < 48MHz
 *	Mode 0: CPOL=0 CHPA=0 (CHPA_MISO=0 and CHPA_MOSI=0)
 *	Mode 3: CPOL=1 CHPA=1 (CHPA_MISO=1 and CHPA_MOSI=1)
 * Data sheet recommends when QMSPI set at max. SPI frequency (48MHz).
 * SPI frequency == 48MHz sample and change data on same edge.
 *  Mode 0: CPOL=0 CHPA=0 (CHPA_MISO=1 and CHPA_MOSI=0)
 *  Mode 3: CPOL=1 CHPA=1 (CHPA_MISO=0 and CHPA_MOSI=1)
 */

const uint8_t smode_tbl[4] = {
	0x00u, 0x06u, 0x01u, 0x07u
};

const uint8_t smode48_tbl[4] = {
	0x04u, 0x02u, 0x05u, 0x03u
};

static void qmspi_set_signalling_mode(QMSPI_Type *regs, uint32_t smode)
{
	printk("[spidri][qmspi_set_signalling_mode] func entry\n");
	const uint8_t *ptbl;
	uint32_t m;

	ptbl = smode_tbl;
	if (((regs->MODE >> MCHP_QMSPI_M_FDIV_POS) &
	    MCHP_QMSPI_M_FDIV_MASK0) == 1) {
		ptbl = smode48_tbl;
	}

	m = (uint32_t)ptbl[smode & 0x03];
	regs->MODE = (regs->MODE & ~(MCHP_QMSPI_M_SIG_MASK))
		     | (m << MCHP_QMSPI_M_SIG_POS);
}

/*
 * QMSPI HW support single, dual, and quad.
 * Return QMSPI Control/Descriptor register encoded value.
 */
static uint32_t qmspi_config_get_lines(const struct spi_config *config)
{
	printk("[spidri][qmspi_config_get_lines] func entry\n");

	printk("[spidri][qmspi_config_get_lines] config->operation & SPI_LINES_MASK = 0x%X\n", config->operation & SPI_LINES_MASK);
	printk("[spidri][qmspi_config_get_lines]  DT_INST_PROP(0, lines) = 0x%X\n",  DT_INST_PROP(0, lines));

#ifdef CONFIG_SPI_EXTENDED_MODES
	uint32_t qlines;

	switch (config->operation & SPI_LINES_MASK) {
	case SPI_LINES_SINGLE:
		qlines = MCHP_QMSPI_C_IFM_4X;
		printk("[spidri][qmspi_config_get_lines] MCHP_QMSPI_C_IFM_1X\n");
		break;
#if DT_INST_PROP(0, lines) > 1
	case SPI_LINES_DUAL:
		qlines = MCHP_QMSPI_C_IFM_2X;
		printk("[spidri][qmspi_config_get_lines] MCHP_QMSPI_C_IFM_2X\n");
		break;
#endif
#if DT_INST_PROP(0, lines) > 2
	case SPI_LINES_QUAD:
		qlines = MCHP_QMSPI_C_IFM_4X;
		printk("[spidri][qmspi_config_get_lines] MCHP_QMSPI_C_IFM_4X\n");
		break;
#endif
	default:
		qlines = 0xffu;
		printk("[spidri][qmspi_config_get_lines] default qlines = 0xffu\n");
	}

	return qlines;
#else
	printk("[spidri][qmspi_config_get_lines] return MCHP_QMSPI_C_IFM_1X\n");
	return MCHP_QMSPI_C_IFM_1X;
#endif
}


static int ryan_qmspi_configure(const struct device *dev,
			   const struct spi_config *config)
{
	printk("[spidri][ryan_qmspi_configure] func entry\n");
	const struct spi_qmspi_config *cfg = dev->config;
	struct spi_qmspi_data *data = dev->data;
	QMSPI_Type *regs = cfg->regs;
	uint32_t smode;

	printk("[spidri][ryan_qmspi_configure] QMSPI Mode reg base address = 0x%X\n", &regs->MODE);
	printk("[spidri][ryan_qmspi_configure] QMSPI Control reg base address = 0x%X\n", &regs->CTRL);
	printk("[spidri][ryan_qmspi_configure] QMSPI Execute reg base address = 0x%X\n", &regs->EXE);
	printk("[spidri][ryan_qmspi_configure] QMSPI Interface Control reg base address = 0x%X\n", &regs->IFCTRL);
	printk("[spidri][ryan_qmspi_configure] QMSPI Buffer Count Status reg base address = 0x%X\n", &regs->BCNT_STS);
	printk("[spidri][ryan_qmspi_configure] QMSPI Interrupt Enable reg base address = 0x%X\n", &regs->IEN);
	printk("[spidri][ryan_qmspi_configure] QMSPI Buffer Count Trigger reg base address = 0x%X\n", &regs->BCNT_TRIG);
	printk("[spidri][ryan_qmspi_configure] QMSPI Transmit Buffer reg base address = 0x%X\n", &regs->TX_FIFO);
	printk("[spidri][ryan_qmspi_configure] QMSPI Receive Buffer reg base address = 0x%X\n", &regs->RX_FIFO);
	printk("[spidri][ryan_qmspi_configure] QMSPI Chip Select Timing reg base address = 0x%X\n", &regs->CSTM);

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		return -ENOTSUP;
	}

	if (config->operation & (SPI_TRANSFER_LSB | SPI_OP_MODE_SLAVE
				 | SPI_MODE_LOOP)) {
		return -ENOTSUP;
	}

	smode = qmspi_config_get_lines(config);
	printk("[spidri][ryan_qmspi_configure] qmspi_config_get_lines() = 0x%X\n", smode);
	if (smode == 0xff) {
		return -ENOTSUP;
	}

	uint32_t *qmspi_ControlReg = ((volatile uint32_t *)(uintptr_t)(&regs->CTRL));
	printk("[spidri][ryan_qmspi_configure] qmspi_ControlReg base address = 0x%X\n", qmspi_ControlReg);
	QmspiControlRegister QmspiControlReg;
 	QmspiControlReg.bitfld.INTERFACE_MODE = smode;
	printk("[spidri][ryan_qmspi_configure] QmspiControlReg.bitfld.INTERFACE_MODE = 0x%X\n", QmspiControlReg.bitfld.INTERFACE_MODE);
	printk("[spidri][ryan_qmspi_configure] QmspiControlReg.VALUE = 0x%X\n", QmspiControlReg.VALUE);
	*qmspi_ControlReg = QmspiControlReg.VALUE;

	/* Use the requested or next highest possible frequency */
	qmspi_set_frequency(regs, config->frequency);

	smode = 0;
	if ((config->operation & SPI_MODE_CPHA) != 0U) {
		smode |= (1ul << 0);
	}

	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		smode |= (1ul << 1);
	}

	qmspi_set_signalling_mode(regs, smode);

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		return -ENOTSUP;
	}

	uint32_t *qmspi_ModeReg = ((volatile uint32_t *)(uintptr_t)(&regs->MODE));
	printk("[spidri][ryan_qmspi_configure] qmspi_ModeReg base address = 0x%X\n", qmspi_ModeReg);
	QmspiModeRegister QmspiModeReg;
 	QmspiModeReg.VALUE = *qmspi_ModeReg;
	printk("[spidri][ryan_qmspi_configure] QmspiModeReg.VALUE = 0x%X\n", QmspiModeReg.VALUE);

	/* chip select */
#if DT_INST_PROP(0, chip_select) == 0
	QmspiModeReg.bitfld.CHIP_SELECT = 0;
#else
	QmspiModeReg.bitfld.CHIP_SELECT = 1;
#endif
	printk("[spidri][ryan_qmspi_configure] QmspiModeReg.VALUE = 0x%X\n", QmspiModeReg.VALUE);
	*qmspi_ModeReg = QmspiModeReg.VALUE;

	/* chip select timing */
	uint32_t *qmspi_ChipSelectTimingRegister = ((volatile uint32_t *)(uintptr_t)(&regs->CSTM));
	printk("[spidri][ryan_qmspi_configure] qmspi_ChipSelectTimingRegister base address = 0x%X\n", qmspi_ChipSelectTimingRegister);
	QmspiChipSelectTimingRegister QmspiChipSelectTimingReg;
 	QmspiChipSelectTimingReg.VALUE = cfg->cs_timing;
	printk("[spidri][ryan_qmspi_configure] QmspiChipSelectTimingReg.VALUE = 0x%X\n", QmspiChipSelectTimingReg.VALUE);
	*qmspi_ChipSelectTimingRegister = QmspiChipSelectTimingReg.VALUE;

	data->ctx.config = config;

	QmspiModeReg.bitfld.ACTIVATE = 1;
	*qmspi_ModeReg = QmspiModeReg.VALUE;

	return 0;
}
/*
 * Configure QMSPI.
 * NOTE: QMSPI can control two chip selects. At this time we use CS0# only.
 */
static int qmspi_configure(const struct device *dev,
			   const struct spi_config *config)
{
	printk("[spidri][qmspi_configure] func entry\n");
	const struct spi_qmspi_config *cfg = dev->config;
	struct spi_qmspi_data *data = dev->data;
	QMSPI_Type *regs = cfg->regs;
	uint32_t smode;

	printk("[spidri][qmspi_configure] QMSPI Mode reg base address = 0x%X\n", &regs->MODE);
	printk("[spidri][qmspi_configure] QMSPI Control reg base address = 0x%X\n", &regs->CTRL);
	printk("[spidri][qmspi_configure] QMSPI Execute reg base address = 0x%X\n", &regs->EXE);
	printk("[spidri][qmspi_configure] QMSPI Interface Control reg base address = 0x%X\n", &regs->IFCTRL);
	printk("[spidri][qmspi_configure] QMSPI Buffer Count Status reg base address = 0x%X\n", &regs->BCNT_STS);
	printk("[spidri][qmspi_configure] QMSPI Interrupt Enable reg base address = 0x%X\n", &regs->IEN);
	printk("[spidri][qmspi_configure] QMSPI Buffer Count Trigger reg base address = 0x%X\n", &regs->BCNT_TRIG);
	printk("[spidri][qmspi_configure] QMSPI Transmit Buffer reg base address = 0x%X\n", &regs->TX_FIFO);
	printk("[spidri][qmspi_configure] QMSPI Receive Buffer reg base address = 0x%X\n", &regs->RX_FIFO);
	printk("[spidri][qmspi_configure] QMSPI Chip Select Timing reg base address = 0x%X\n", &regs->CSTM);

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		return -ENOTSUP;
	}

	if (config->operation & (SPI_TRANSFER_LSB | SPI_OP_MODE_SLAVE
				 | SPI_MODE_LOOP)) {
		return -ENOTSUP;
	}

	smode = qmspi_config_get_lines(config);
	printk("[spidri][qmspi_configure] qmspi_config_get_lines() = 0x%X\n", smode);
	if (smode == 0xff) {
		return -ENOTSUP;
	}

	regs->CTRL = smode;

	/* Use the requested or next highest possible frequency */
	qmspi_set_frequency(regs, config->frequency);

	smode = 0;
	if ((config->operation & SPI_MODE_CPHA) != 0U) {
		smode |= (1ul << 0);
	}

	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		smode |= (1ul << 1);
	}

	qmspi_set_signalling_mode(regs, smode);

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		return -ENOTSUP;
	}

	/* chip select */
	smode = regs->MODE & ~(MCHP_QMSPI_M_CS_MASK);
#if DT_INST_PROP(0, chip_select) == 0
	smode |= MCHP_QMSPI_M_CS0;
#else
	smode |= MCHP_QMSPI_M_CS1;
#endif
	regs->MODE = smode;

	/* chip select timing */
	regs->CSTM = cfg->cs_timing;

	data->ctx.config = config;

	regs->MODE |= MCHP_QMSPI_M_ACTIVATE;

	return 0;
}

/*
 * Transmit dummy clocks - QMSPI will generate requested number of
 * SPI clocks with I/O pins tri-stated.
 * Single mode: 1 bit per clock -> IFM field = 00b. Max 0x7fff clocks
 * Dual mode: 2 bits per clock  -> IFM field = 01b. Max 0x3fff clocks
 * Quad mode: 4 bits per clock  -> IFM field = 1xb. Max 0x1fff clocks
 * QMSPI unit size set to bits.
 */
static int qmspi_tx_dummy_clocks(QMSPI_Type *regs, uint32_t nclocks)
{
	printk("[spidri][qmspi_tx_dummy_clocks] func entry\n");
	uint32_t descr, ifm, qstatus;

	ifm = regs->CTRL & MCHP_QMSPI_C_IFM_MASK;
	descr = ifm | MCHP_QMSPI_C_TX_DIS | MCHP_QMSPI_C_XFR_UNITS_BITS
		| MCHP_QMSPI_C_DESCR_LAST | MCHP_QMSPI_C_DESCR0;

	if (ifm & 0x01) {
		nclocks <<= 1;
	} else if (ifm & 0x02) {
		nclocks <<= 2;
	}
	descr |= (nclocks << MCHP_QMSPI_C_XFR_NUNITS_POS);

	descr_wr(regs, 0, descr);

	regs->CTRL |= MCHP_QMSPI_C_DESCR_EN;
	regs->IEN = 0;

	//regs->IEN = MCHP_QMSPI_IEN_XFR_DONE; //ryan
	printk("[spidri][qmspi_tx_dummy_clocks] regs->IEN = 0x%X\n", regs->IEN);

	regs->STS = 0xfffffffful;

	regs->EXE = MCHP_QMSPI_EXE_START;
	do {
		qstatus = regs->STS;
		printk("[spidri][qmspi_tx_dummy_clocks] qstatus = 0x%X\n", qstatus);

		if (qstatus & MCHP_QMSPI_STS_PROG_ERR) {
			return -EIO;
		}
	} while ((qstatus & MCHP_QMSPI_STS_DONE) == 0);

	return 0;
}

/*
 * Return unit size power of 2 given number of bytes to transfer.
 */
static uint32_t qlen_shift(uint32_t len)
{
	printk("[spidri][qlen_shift] func entry\n");
	uint32_t ushift;

	/* is len a multiple of 4 or 16? */
	if ((len & 0x0F) == 0) {
		ushift = 4;
	} else if ((len & 0x03) == 0) {
		ushift = 2;
	} else {
		ushift = 0;
	}

	return ushift;
}

/*
 * Return QMSPI unit size of the number of units field in QMSPI
 * control/descriptor register.
 * Input: power of 2 unit size 4, 2, or 0(default) corresponding
 * to 16, 4, or 1 byte units.
 */
static uint32_t get_qunits(uint32_t qshift)
{
	printk("[spidri][get_qunits] func entry\n");
	if (qshift == 4) {
		return MCHP_QMSPI_C_XFR_UNITS_16;
	} else if (qshift == 2) {
		return MCHP_QMSPI_C_XFR_UNITS_4;
	} else {
		return MCHP_QMSPI_C_XFR_UNITS_1;
	}
}

/*
 * Allocate(build) one or more descriptors.
 * QMSPI contains 16 32-bit descriptor registers used as a linked
 * list of operations. Using only 32-bits there are limitations.
 * Each descriptor is limited to 0x7FFF units where unit size can
 * be 1, 4, or 16 bytes. A descriptor can perform transmit or receive
 * but not both simultaneously. Order of descriptor processing is specified
 * by the first descriptor field of the control register, the next descriptor
 * fields in each descriptor, and the descriptors last flag.
 */
static int qmspi_descr_alloc(QMSPI_Type *regs, const struct spi_buf *txb,
			     int didx, bool is_tx)
{
	printk("[spidri][qmspi_descr_alloc] func entry\n");
	uint32_t descr, qshift, n, nu;
	int dn;

	if (didx >= MCHP_QMSPI_MAX_DESCR) {
		return -EAGAIN;
	}

	if (txb->len == 0) {
		return didx; /* nothing to do */
	}

	/* b[1:0] IFM and b[3:2] transmit mode */
	descr = (regs->CTRL & MCHP_QMSPI_C_IFM_MASK);
	if (is_tx) {
		descr |= MCHP_QMSPI_C_TX_DATA;
	} else {
		descr |= MCHP_QMSPI_C_RX_EN;
	}

	/* b[11:10] unit size 1, 4, or 16 bytes */
	qshift = qlen_shift(txb->len);
	nu = txb->len >> qshift;
	descr |= get_qunits(qshift);

	do {
		descr &= 0x0FFFul;

		dn = didx + 1;
		/* b[15:12] next descriptor pointer */
		descr |= ((dn & MCHP_QMSPI_C_NEXT_DESCR_MASK0) <<
			  MCHP_QMSPI_C_NEXT_DESCR_POS);

		n = nu;
		if (n > MCHP_QMSPI_C_MAX_UNITS) {
			n = MCHP_QMSPI_C_MAX_UNITS;
		}

		descr |= (n << MCHP_QMSPI_C_XFR_NUNITS_POS);
		descr_wr(regs, didx, descr);

		if (dn < MCHP_QMSPI_MAX_DESCR) {
			didx++;
		} else {
			return -EAGAIN;
		}

		nu -= n;
	} while (nu);

	return dn;
}

static int ryan_qmspi_tx(QMSPI_Type *regs, const struct spi_buf *tx_buf,
		    bool close)
{
	printk("[spidri][ryan_qmspi_tx] func entry\n");
	const uint8_t *p = tx_buf->buf;
	size_t tlen = tx_buf->len;
	uint32_t descr;
	int didx;

	if (tlen == 0) {
		return 0;
	}

	/* Buffer pointer is NULL and number of bytes != 0 ? */
	if (p == NULL) {
		return qmspi_tx_dummy_clocks(regs, tlen);
	}

	didx = qmspi_descr_alloc(regs, tx_buf, 0, true);
	if (didx < 0) {
		return didx;
	}

	/* didx points to last allocated descriptor + 1 */
	__ASSERT(didx > 0, "QMSPI descriptor index=%d expected > 0\n", didx);
	didx--;

	descr = descr_rd(regs, didx) | MCHP_QMSPI_C_DESCR_LAST;
	if (close) {
		descr |= MCHP_QMSPI_C_CLOSE;
	}
	descr_wr(regs, didx, descr);

	uint32_t *qmspi_ControlReg = ((volatile uint32_t *)(uintptr_t)(&regs->CTRL));
	printk("[spidri][ryan_qmspi_tx] qmspi_ControlReg base address = 0x%X\n", qmspi_ControlReg);
	QmspiControlRegister QmspiControlReg;
	QmspiControlReg.VALUE = *qmspi_ControlReg;
	QmspiControlReg.bitfld.DESCRIPTION_BUFFER_ENABLE = 1;
	QmspiControlReg.bitfld.DESCRIPTION_BUFFER_POINTER = 1;
	printk("[spidri][ryan_qmspi_tx] QmspiControlReg.VALUE = 0x%X\n", QmspiControlReg.VALUE);
	*qmspi_ControlReg = QmspiControlReg.VALUE;

	uint32_t *qmspi_InterruptEnableReg = ((volatile uint32_t *)(uintptr_t)(&regs->IEN));
	printk("[spidri][ryan_qmspi_tx] qmspi_InterruptEnableReg base address = 0x%X\n", qmspi_InterruptEnableReg);
	QmspiInterruptEnableRegister QmspiInterruptEnableReg;
	QmspiInterruptEnableReg.VALUE = 0;
	//QmspiInterruptEnableReg.bitfld.TRANSFER_COMPLETE_ENABLE = 1 //ryan interrupt
	printk("[spidri][ryan_qmspi_tx] QmspiInterruptEnableReg.VALUE = 0x%X\n", QmspiInterruptEnableReg.VALUE);
	*qmspi_InterruptEnableReg = QmspiInterruptEnableReg.VALUE;

	uint32_t *qmspi_StatusRegister = ((volatile uint32_t *)(uintptr_t)(&regs->STS));
	printk("[spidri][ryan_qmspi_tx] qmspi_StatusRegister base address = 0x%X\n", qmspi_StatusRegister);
	QmspiStatusRegister QmspiStatusReg;
	QmspiStatusReg.VALUE = 0xfffffffful;
	printk("[spidri][ryan_qmspi_tx] QmspiStatusReg.VALUE = 0x%X\n", QmspiStatusReg.VALUE);
	*qmspi_StatusRegister = QmspiStatusReg.VALUE;

	/* preload TX_FIFO */
	while (tlen) {
		tlen--;
		ryan_txb_wr8(regs, *p); //txb_wr8(regs, *p);
		p++;

		QmspiStatusReg.VALUE = *qmspi_StatusRegister;
		if(QmspiStatusReg.bitfld.TRANSMIT_BUFFER_FULL == 1)
		{
			printk("[spidri][ryan_qmspi_tx] QmspiStatusReg.bitfld.TRANSMIT_BUFFER_FULL\n");
			break;
		}
	}

	uint32_t *qmspi_ExecuteRegister = ((volatile uint32_t *)(uintptr_t)(&regs->EXE));
	printk("[spidri][ryan_qmspi_tx] qmspi_ExecuteRegister base address = 0x%X\n", qmspi_ExecuteRegister);
	QmspiExecuteRegister QmspiExecuteReg;
	QmspiExecuteReg.bitfld.START = 1;
	printk("[spidri][ryan_qmspi_tx] QmspiExecuteReg.VALUE = 0x%X\n", QmspiExecuteReg.VALUE);
	*qmspi_ExecuteRegister = QmspiExecuteReg.VALUE;

	QmspiStatusReg.VALUE = *qmspi_StatusRegister;
	if (QmspiStatusReg.bitfld.PROGRAMMING_ERROR == 1) {
		printk("[spidri][ryan_qmspi_tx] QmspiStatusReg.bitfld.PROGRAMMING_ERROR\n");
		return -EIO;
	}

	while (tlen) {

		QmspiStatusReg.VALUE = *qmspi_StatusRegister;
		while (QmspiStatusReg.bitfld.TRANSMIT_BUFFER_FULL == 1) {
		}

		ryan_txb_wr8(regs, *p); //txb_wr8(regs, *p);
		p++;
		tlen--;
	}

	/* Wait for TX FIFO to drain and last byte to be clocked out */
	for (;;) {
		QmspiStatusReg.VALUE = *qmspi_StatusRegister;
		if (QmspiStatusReg.bitfld.TRANSFER_COMPLETE == 1) {
			printk("[spidri][ryan_qmspi_tx] MCHP_QMSPI_STS_DONE\n");
			break;
		}
	}

	return 0;
}

static int qmspi_tx(QMSPI_Type *regs, const struct spi_buf *tx_buf,
		    bool close)
{
	printk("[spidri][qmspi_tx] func entry\n");
	const uint8_t *p = tx_buf->buf;
	size_t tlen = tx_buf->len;
	uint32_t descr;
	int didx;

	if (tlen == 0) {
		return 0;
	}

	/* Buffer pointer is NULL and number of bytes != 0 ? */
	if (p == NULL) {
		return qmspi_tx_dummy_clocks(regs, tlen);
	}

	didx = qmspi_descr_alloc(regs, tx_buf, 0, true);
	if (didx < 0) {
		return didx;
	}

	/* didx points to last allocated descriptor + 1 */
	__ASSERT(didx > 0, "QMSPI descriptor index=%d expected > 0\n", didx);
	didx--;

	descr = descr_rd(regs, didx) | MCHP_QMSPI_C_DESCR_LAST;
	if (close) {
		descr |= MCHP_QMSPI_C_CLOSE;
	}
	descr_wr(regs, didx, descr);

	regs->CTRL = (regs->CTRL & MCHP_QMSPI_C_IFM_MASK) |
		     MCHP_QMSPI_C_DESCR_EN | MCHP_QMSPI_C_DESCR0;
	regs->IEN = 0;
	//regs->IEN = MCHP_QMSPI_IEN_XFR_DONE; //ryan interrupt
	printk("[spidri][qmspi_tx] regs->IEN = 0x%X\n", regs->IEN);
	regs->STS = 0xfffffffful;

	/* preload TX_FIFO */
	while (tlen) {
		tlen--;
		ryan_txb_wr8(regs, *p); //txb_wr8(regs, *p);
		p++;

		if (regs->STS & MCHP_QMSPI_STS_TXBF_RO) {
			break;
		}
	}

	regs->EXE = MCHP_QMSPI_EXE_START;

	if (regs->STS & MCHP_QMSPI_STS_PROG_ERR) {
		return -EIO;
	}

	while (tlen) {

		while (regs->STS & MCHP_QMSPI_STS_TXBF_RO) {
		}

		ryan_txb_wr8(regs, *p); //txb_wr8(regs, *p);
		p++;
		tlen--;
	}

	/* Wait for TX FIFO to drain and last byte to be clocked out */
	for (;;) {
		if (regs->STS & MCHP_QMSPI_STS_DONE) {
			printk("[spidri][qmspi_tx] MCHP_QMSPI_STS_DONE\n");
			break;
		}
	}

	return 0;
}

static int ryan_qmspi_rx(QMSPI_Type *regs, const struct spi_buf *rx_buf,
		    bool close)
{
	printk("[spidri][ryan_qmspi_rx] func entry\n");
	uint8_t *p = rx_buf->buf;
	size_t rlen = rx_buf->len;
	uint32_t descr;
	int didx;
	uint8_t data_byte;

	if (rlen == 0) {
		return 0;
	}

	didx = qmspi_descr_alloc(regs, rx_buf, 0, false);
	printk("[spidri][ryan_qmspi_rx] qmspi_descr_alloc() = 0x%X\n", didx);
	if (didx < 0) {
		return didx;
	}

	/* didx points to last allocated descriptor + 1 */
	__ASSERT_NO_MSG(didx > 0);
	didx--;

	descr = descr_rd(regs, didx) | MCHP_QMSPI_C_DESCR_LAST;
	printk("[spidri][ryan_qmspi_rx] descr = 0x%X\n", descr);
	if (close) {
		descr |= MCHP_QMSPI_C_CLOSE;
	}
	descr_wr(regs, didx, descr);

	uint32_t *qmspi_ControlReg = ((volatile uint32_t *)(uintptr_t)(&regs->CTRL));
	printk("[spidri][ryan_qmspi_rx] qmspi_ControlReg base address = 0x%X\n", qmspi_ControlReg);
	QmspiControlRegister QmspiControlReg;
	QmspiControlReg.VALUE = *qmspi_ControlReg;
	QmspiControlReg.bitfld.DESCRIPTION_BUFFER_ENABLE = 1;
	QmspiControlReg.bitfld.DESCRIPTION_BUFFER_POINTER = 1;
	printk("[spidri][ryan_qmspi_rx] QmspiControlReg.VALUE = 0x%X\n", QmspiControlReg.VALUE);
	*qmspi_ControlReg = QmspiControlReg.VALUE;

	uint32_t *qmspi_InterruptEnableReg = ((volatile uint32_t *)(uintptr_t)(&regs->IEN));
	printk("[spidri][ryan_qmspi_rx] qmspi_InterruptEnableReg base address = 0x%X\n", qmspi_InterruptEnableReg);
	QmspiInterruptEnableRegister QmspiInterruptEnableReg;
	QmspiInterruptEnableReg.VALUE = 0;
	printk("[spidri][ryan_qmspi_rx] QmspiInterruptEnableReg.VALUE = 0x%X\n", QmspiInterruptEnableReg.VALUE);
	*qmspi_InterruptEnableReg = QmspiInterruptEnableReg.VALUE;

	uint32_t *qmspi_StatusRegister = ((volatile uint32_t *)(uintptr_t)(&regs->STS));
	printk("[spidri][ryan_qmspi_rx] qmspi_StatusRegister base address = 0x%X\n", qmspi_StatusRegister);
	QmspiStatusRegister QmspiStatusReg;
	QmspiStatusReg.VALUE = 0xfffffffful;
	printk("[spidri][ryan_qmspi_rx] QmspiStatusReg.VALUE = 0x%X\n", QmspiStatusReg.VALUE);
	*qmspi_StatusRegister = QmspiStatusReg.VALUE;

	/*
	 * Trigger read based on the descriptor(s) programmed above.
	 * QMSPI will generate clocks until the RX FIFO is filled.
	 * More clocks will be generated as we pull bytes from the RX FIFO.
	 * QMSPI Programming error will be triggered after start if
	 * descriptors were programmed options that cannot be enabled
	 * simultaneously.
	 */
	uint32_t *qmspi_ExecuteRegister = ((volatile uint32_t *)(uintptr_t)(&regs->EXE));
	printk("[spidri][ryan_qmspi_rx] qmspi_ExecuteRegister base address = 0x%X\n", qmspi_ExecuteRegister);
	QmspiExecuteRegister QmspiExecuteReg;
	QmspiExecuteReg.bitfld.START = 1;
	printk("[spidri][ryan_qmspi_rx] QmspiExecuteReg.VALUE = 0x%X\n", QmspiExecuteReg.VALUE);
	*qmspi_ExecuteRegister = QmspiExecuteReg.VALUE;

	QmspiStatusReg.VALUE = *qmspi_StatusRegister;
	if (QmspiStatusReg.bitfld.PROGRAMMING_ERROR == 1) {
		printk("[spidri][ryan_qmspi_rx] QmspiStatusReg.bitfld.PROGRAMMING_ERROR\n");
		return -EIO;
	}

	while (rlen) {
		QmspiStatusReg.VALUE = *qmspi_StatusRegister;
		if (QmspiStatusReg.bitfld.RECEIVE_BUFFER_EMPTY == 1) {
			data_byte = ryan_rxb_rd8(regs); //rxb_rd8(regs);
			if (p != NULL) {
				*p++ = data_byte;
			}
			rlen--;
		}
	}

	return 0;
}


static int qmspi_rx(QMSPI_Type *regs, const struct spi_buf *rx_buf,
		    bool close)
{
	printk("[spidri][qmspi_rx] func entry\n");
	uint8_t *p = rx_buf->buf;
	size_t rlen = rx_buf->len;
	uint32_t descr;
	int didx;
	uint8_t data_byte;

	if (rlen == 0) {
		return 0;
	}

	didx = qmspi_descr_alloc(regs, rx_buf, 0, false);
	printk("[spidri][qmspi_transceive] qmspi_descr_alloc() = 0x%X\n", didx);
	if (didx < 0) {
		return didx;
	}

	/* didx points to last allocated descriptor + 1 */
	__ASSERT_NO_MSG(didx > 0);
	didx--;

	descr = descr_rd(regs, didx) | MCHP_QMSPI_C_DESCR_LAST;
	printk("[spidri][qmspi_transceive] descr = 0x%X\n", descr);
	if (close) {
		descr |= MCHP_QMSPI_C_CLOSE;
	}
	descr_wr(regs, didx, descr);

	regs->CTRL = (regs->CTRL & MCHP_QMSPI_C_IFM_MASK)
		     | MCHP_QMSPI_C_DESCR_EN | MCHP_QMSPI_C_DESCR0;
	regs->IEN = 0;
	printk("[spidri][qmspi_rx] regs->IEN = 0x%X\n", regs->IEN);

	regs->STS = 0xfffffffful;

	/*
	 * Trigger read based on the descriptor(s) programmed above.
	 * QMSPI will generate clocks until the RX FIFO is filled.
	 * More clocks will be generated as we pull bytes from the RX FIFO.
	 * QMSPI Programming error will be triggered after start if
	 * descriptors were programmed options that cannot be enabled
	 * simultaneously.
	 */
	regs->EXE = MCHP_QMSPI_EXE_START;
	if (regs->STS & MCHP_QMSPI_STS_PROG_ERR) {
		return -EIO;
	}

	while (rlen) {
		if (!(regs->STS & MCHP_QMSPI_STS_RXBE_RO)) {
			data_byte = ryan_rxb_rd8(regs); //rxb_rd8(regs);
			if (p != NULL) {
				*p++ = data_byte;
			}
			rlen--;
		}
	}

	return 0;
}

static int ryan_qmspi_transceive(const struct device *dev,
			    const struct spi_config *config,
			    const struct spi_buf_set *tx_bufs,
			    const struct spi_buf_set *rx_bufs)
{
	printk("[spidri][ryan_qmspi_transceive] func entry\n");
	const struct spi_qmspi_config *cfg = dev->config;
	struct spi_qmspi_data *data = dev->data;
	QMSPI_Type *regs = cfg->regs;
	const struct spi_buf *ptx;
	const struct spi_buf *prx;
	size_t nb;
	uint32_t descr, last_didx;
	int err;

	printk("[spidri][ryan_qmspi_transceive] QMSPI Mode reg base address = 0x%X\n", &regs->MODE);
	printk("[spidri][ryan_qmspi_transceive] QMSPI Control reg base address = 0x%X\n", &regs->CTRL);
	printk("[spidri][ryan_qmspi_transceive] QMSPI Execute reg base address = 0x%X\n", &regs->EXE);
	printk("[spidri][ryan_qmspi_transceive] QMSPI Interface Control reg base address = 0x%X\n", &regs->IFCTRL);
	printk("[spidri][ryan_qmspi_transceive] QMSPI Buffer Count Status reg base address = 0x%X\n", &regs->BCNT_STS);
	printk("[spidri][ryan_qmspi_transceive] QMSPI Interrupt Enable reg base address = 0x%X\n", &regs->IEN);
	printk("[spidri][ryan_qmspi_transceive] QMSPI Buffer Count Trigger reg base address = 0x%X\n", &regs->BCNT_TRIG);
	printk("[spidri][ryan_qmspi_transceive] QMSPI Transmit Buffer reg base address = 0x%X\n", &regs->TX_FIFO);
	printk("[spidri][ryan_qmspi_transceive] QMSPI Receive Buffer reg base address = 0x%X\n", &regs->RX_FIFO);
	printk("[spidri][ryan_qmspi_transceive] QMSPI Chip Select Timing reg base address = 0x%X\n", &regs->CSTM);

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	err = ryan_qmspi_configure(dev, config); //qmspi_configure(dev, config);
	printk("[spidri][ryan_qmspi_transceive] qmspi_configure() = 0x%X\n", err);
	if (err != 0) {
		goto done;
	}

	spi_context_cs_control(&data->ctx, true);

	if (tx_bufs != NULL) {
		ptx = tx_bufs->buffers;
		nb = tx_bufs->count;
		while (nb--) {
			err = qmspi_tx(regs, ptx, false); //ryan_qmspi_tx(regs, ptx, false);
			printk("[spidri][ryan_qmspi_transceive] ryan_qmspi_tx() = 0x%X\n", err);
			if (err != 0) {
				goto done;
			}
			ptx++;
		}
	}

	if (rx_bufs != NULL) {
		prx = rx_bufs->buffers;
		nb = rx_bufs->count;
		while (nb--) {
			err = qmspi_rx(regs, prx, false); //ryan_qmspi_rx(regs, prx, false);
			printk("[spidri][ryan_qmspi_transceive] ryan_qmspi_rx() = 0x%X\n", err);
			if (err != 0) {
				goto done;
			}
			prx++;
		}
	}

	uint32_t *qmspi_StatusRegister = ((volatile uint32_t *)(uintptr_t)(&regs->STS));
	printk("[spidri][ryan_qmspi_transceive] qmspi_StatusRegister base address = 0x%X\n", qmspi_StatusRegister);
	QmspiStatusRegister QmspiStatusReg;
	QmspiStatusReg.VALUE = *qmspi_StatusRegister;
	printk("[spidri][ryan_qmspi_transceive] QmspiStatusReg.VALUE = 0x%X\n", QmspiStatusReg.VALUE);
	
	/*
	 * If caller doesn't need CS# held asserted then find the last
	 * descriptor, set its close flag, and set stop.
	 */
	if (!(config->operation & SPI_HOLD_ON_CS)) {
		/* Get last descriptor from status register */
		last_didx = (regs->STS >> MCHP_QMSPI_C_NEXT_DESCR_POS)
			    & MCHP_QMSPI_C_NEXT_DESCR_MASK0;
		descr = descr_rd(regs, last_didx) | MCHP_QMSPI_C_CLOSE;
		descr_wr(regs, last_didx, descr);

		uint32_t *qmspi_ExecuteRegister = ((volatile uint32_t *)(uintptr_t)(&regs->EXE));
		printk("[spidri][ryan_qmspi_transceive] qmspi_ExecuteRegister base address = 0x%X\n", qmspi_ExecuteRegister);
		QmspiExecuteRegister QmspiExecuteReg;
		QmspiExecuteReg.bitfld.STOP = 1;
		printk("[spidri][ryan_qmspi_transceive] QmspiExecuteReg.VALUE = 0x%X\n", QmspiExecuteReg.VALUE);
		*qmspi_ExecuteRegister = QmspiExecuteReg.VALUE;
	}

	spi_context_cs_control(&data->ctx, false);

done:
	spi_context_release(&data->ctx, err);
	return err;
}

static int qmspi_transceive(const struct device *dev,
			    const struct spi_config *config,
			    const struct spi_buf_set *tx_bufs,
			    const struct spi_buf_set *rx_bufs)
{
	printk("[spidri][qmspi_transceive] func entry\n");
	const struct spi_qmspi_config *cfg = dev->config;
	struct spi_qmspi_data *data = dev->data;
	QMSPI_Type *regs = cfg->regs;
	const struct spi_buf *ptx;
	const struct spi_buf *prx;
	size_t nb;
	uint32_t descr, last_didx;
	int err;

	printk("[spidri][qmspi_transceive] QMSPI Mode reg base address = 0x%X\n", &regs->MODE);
	printk("[spidri][qmspi_transceive] QMSPI Control reg base address = 0x%X\n", &regs->CTRL);
	printk("[spidri][qmspi_transceive] QMSPI Execute reg base address = 0x%X\n", &regs->EXE);
	printk("[spidri][qmspi_transceive] QMSPI Interface Control reg base address = 0x%X\n", &regs->IFCTRL);
	printk("[spidri][qmspi_transceive] QMSPI Buffer Count Status reg base address = 0x%X\n", &regs->BCNT_STS);
	printk("[spidri][qmspi_transceive] QMSPI Interrupt Enable reg base address = 0x%X\n", &regs->IEN);
	printk("[spidri][qmspi_transceive] QMSPI Buffer Count Trigger reg base address = 0x%X\n", &regs->BCNT_TRIG);
	printk("[spidri][qmspi_transceive] QMSPI Transmit Buffer reg base address = 0x%X\n", &regs->TX_FIFO);
	printk("[spidri][qmspi_transceive] QMSPI Receive Buffer reg base address = 0x%X\n", &regs->RX_FIFO);
	printk("[spidri][qmspi_transceive] QMSPI Chip Select Timing reg base address = 0x%X\n", &regs->CSTM);

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	err = ryan_qmspi_configure(dev, config); //qmspi_configure(dev, config);
	printk("[spidri][qmspi_transceive] qmspi_configure() = 0x%X\n", err);
	if (err != 0) {
		goto done;
	}

	spi_context_cs_control(&data->ctx, true);

	if (tx_bufs != NULL) {
		ptx = tx_bufs->buffers;
		nb = tx_bufs->count;
		while (nb--) {
			err = qmspi_tx(regs, ptx, false);
			printk("[spidri][qmspi_transceive] qmspi_tx() = 0x%X\n", err);
			if (err != 0) {
				goto done;
			}
			ptx++;
		}
	}

	if (rx_bufs != NULL) {
		prx = rx_bufs->buffers;
		nb = rx_bufs->count;
		while (nb--) {
			err = qmspi_rx(regs, prx, false);
			printk("[spidri][qmspi_transceive] qmspi_rx() = 0x%X\n", err);
			if (err != 0) {
				goto done;
			}
			prx++;
		}
	}

	/*
	 * If caller doesn't need CS# held asserted then find the last
	 * descriptor, set its close flag, and set stop.
	 */
	if (!(config->operation & SPI_HOLD_ON_CS)) {
		/* Get last descriptor from status register */
		last_didx = (regs->STS >> MCHP_QMSPI_C_NEXT_DESCR_POS)
			    & MCHP_QMSPI_C_NEXT_DESCR_MASK0;
		descr = descr_rd(regs, last_didx) | MCHP_QMSPI_C_CLOSE;
		descr_wr(regs, last_didx, descr);
		regs->EXE = MCHP_QMSPI_EXE_STOP;
	}

	spi_context_cs_control(&data->ctx, false);

done:
	spi_context_release(&data->ctx, err);
	return err;
}

static int qmspi_transceive_sync(const struct device *dev,
				 const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	printk("[spidri][qmspi_transceive_sync] func entry\n");
	return qmspi_transceive(dev, config, tx_bufs, rx_bufs); //ryan_qmspi_transceive(dev, config, tx_bufs, rx_bufs); //
}

#ifdef CONFIG_SPI_ASYNC
static int qmspi_transceive_async(const struct device *dev,
				  const struct spi_config *config,
				  const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs,
				  struct k_poll_signal *async)
{
	return -ENOTSUP;
}
#endif



static int ryan_qmspi_release(const struct device *dev,
			 const struct spi_config *config)
{
	printk("[spidri][ryan_qmspi_release] func entry\n");
	struct spi_qmspi_data *data = dev->data;
	const struct spi_qmspi_config *cfg = dev->config;
	QMSPI_Type *regs = cfg->regs;

	printk("[spidri][ryan_qmspi_release] QMSPI Mode reg base address = 0x%X\n", &regs->MODE);
	printk("[spidri][ryan_qmspi_release] QMSPI Control reg base address = 0x%X\n", &regs->CTRL);
	printk("[spidri][ryan_qmspi_release] QMSPI Execute reg base address = 0x%X\n", &regs->EXE);
	printk("[spidri][ryan_qmspi_release] QMSPI Interface Control reg base address = 0x%X\n", &regs->IFCTRL);
	printk("[spidri][ryan_qmspi_release] QMSPI Buffer Count Status reg base address = 0x%X\n", &regs->BCNT_STS);
	printk("[spidri][ryan_qmspi_release] QMSPI Interrupt Enable reg base address = 0x%X\n", &regs->IEN);
	printk("[spidri][ryan_qmspi_release] QMSPI Buffer Count Trigger reg base address = 0x%X\n", &regs->BCNT_TRIG);
	printk("[spidri][ryan_qmspi_release] QMSPI Transmit Buffer reg base address = 0x%X\n", &regs->TX_FIFO);
	printk("[spidri][ryan_qmspi_release] QMSPI Receive Buffer reg base address = 0x%X\n", &regs->RX_FIFO);
	printk("[spidri][ryan_qmspi_release] QMSPI Chip Select Timing reg base address = 0x%X\n", &regs->CSTM);

	/* Force CS# to de-assert on next unit boundary */
	uint32_t *qmspi_ExecuteReg = ((volatile uint32_t *)(uintptr_t)(&regs->EXE));
	printk("[spidri][ryan_qmspi_release] qmspi_ExecuteReg base address = 0x%X\n", qmspi_ExecuteReg);
	QmspiExecuteRegister QmspiExecuteReg;
	QmspiExecuteReg.VALUE = *qmspi_ExecuteReg;
	printk("[spidri][ryan_qmspi_release] QmspiExecuteReg.VALUE = 0x%X\n", QmspiExecuteReg.VALUE);
	QmspiExecuteReg.bitfld.STOP = 1;
	*qmspi_ExecuteReg = QmspiExecuteReg.VALUE;
	printk("[spidri][ryan_qmspi_release] *qmspi_ExecuteReg  = 0x%X\n", *qmspi_ExecuteReg );

	uint32_t *qspi_StatusReg = ((volatile uint32_t *)(uintptr_t)(&regs->STS));
	printk("[spidri][ryan_qmspi_release] qspi_StatusReg base address = 0x%X\n", qspi_StatusReg);
	QmspiStatusRegister QmspiStatusReg;
	QmspiStatusReg.VALUE = *qspi_StatusReg;

	while (1) {
		QmspiStatusReg.VALUE = *qspi_StatusReg;
		if(QmspiStatusReg.bitfld.TRANSFER_ACTIVE == 0)
		{
			printk("[spidri][ryan_qmspi_release]QmspiStatusReg.bitfld.TRANSFER_ACTIVE = 0x%X\n", QmspiStatusReg.bitfld.TRANSFER_ACTIVE);
			break;
		}
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int qmspi_release(const struct device *dev,
			 const struct spi_config *config)
{
	printk("[spidri][qmspi_release] func entry\n");
	struct spi_qmspi_data *data = dev->data;
	const struct spi_qmspi_config *cfg = dev->config;
	QMSPI_Type *regs = cfg->regs;

	printk("[spidri][qmspi_release] QMSPI Mode reg base address = 0x%X\n", &regs->MODE);
	printk("[spidri][qmspi_release] QMSPI Control reg base address = 0x%X\n", &regs->CTRL);
	printk("[spidri][qmspi_release] QMSPI Execute reg base address = 0x%X\n", &regs->EXE);
	printk("[spidri][qmspi_release] QMSPI Interface Control reg base address = 0x%X\n", &regs->IFCTRL);
	printk("[spidri][qmspi_release] QMSPI Buffer Count Status reg base address = 0x%X\n", &regs->BCNT_STS);
	printk("[spidri][qmspi_release] QMSPI Interrupt Enable reg base address = 0x%X\n", &regs->IEN);
	printk("[spidri][qmspi_release] QMSPI Buffer Count Trigger reg base address = 0x%X\n", &regs->BCNT_TRIG);
	printk("[spidri][qmspi_release] QMSPI Transmit Buffer reg base address = 0x%X\n", &regs->TX_FIFO);
	printk("[spidri][qmspi_release] QMSPI Receive Buffer reg base address = 0x%X\n", &regs->RX_FIFO);
	printk("[spidri][qmspi_release] QMSPI Chip Select Timing reg base address = 0x%X\n", &regs->CSTM);

	/* Force CS# to de-assert on next unit boundary */
	regs->EXE = MCHP_QMSPI_EXE_STOP;

	while (regs->STS & MCHP_QMSPI_STS_ACTIVE_RO) {
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static void ryan_spi_xec_isr(const struct device *dev)
{
	const struct spi_qmspi_config *cfg = dev->config;
	printk("[spidri][ryan_spi_xec_isr] func entry\n");

	MCHP_GIRQ_CLR_EN(cfg->girq, cfg->girq_pos);
	MCHP_GIRQ_SRC_CLR(cfg->girq, cfg->girq_pos);
	MCHP_GIRQ_BLK_CLREN(cfg->girq);
}

static int ryan_qmspi_init(const struct device *dev)
{
	printk("[spidri][ryan_qmspi_init] func entry\n");
	int err;
	const struct spi_qmspi_config *cfg = dev->config;
	struct spi_qmspi_data *data = dev->data;
	QMSPI_Type *regs = cfg->regs;
	int ret;

	printk("[spidri][ryan_qmspi_init] DT_INST_REG_ADDR(0) = 0x%X\n", (QMSPI_Type *)DT_INST_REG_ADDR(0));
	printk("[spidri][ryan_qmspi_init] QMSPI Mode reg base address = 0x%X\n", &regs->MODE);
	printk("[spidri][ryan_qmspi_init] QMSPI Control reg base address = 0x%X\n", &regs->CTRL);
	printk("[spidri][ryan_qmspi_init] QMSPI Execute reg base address = 0x%X\n", &regs->EXE);
	printk("[spidri][ryan_qmspi_init] QMSPI Interface Control reg base address = 0x%X\n", &regs->IFCTRL);
	printk("[spidri][ryan_qmspi_init] QMSPI Buffer Count Status reg base address = 0x%X\n", &regs->BCNT_STS);
	printk("[spidri][ryan_qmspi_init] QMSPI Interrupt Enable reg base address = 0x%X\n", &regs->IEN);
	printk("[spidri][ryan_qmspi_init] QMSPI Buffer Count Trigger reg base address = 0x%X\n", &regs->BCNT_TRIG);
	printk("[spidri][ryan_qmspi_init] QMSPI Transmit Buffer reg base address = 0x%X\n", &regs->TX_FIFO);
	printk("[spidri][ryan_qmspi_init] QMSPI Receive Buffer reg base address = 0x%X\n", &regs->RX_FIFO);
	printk("[spidri][ryan_qmspi_init] QMSPI Chip Select Timing reg base address = 0x%X\n", &regs->CSTM);
	printk("[spidri][ryan_qmspi_init] &regs->RSVD1 = 0x%X\n", &regs->RSVD1);
	printk("[spidri][ryan_qmspi_init] &regs->DESCR = 0x%X\n", &regs->DESCR);

	printk("[spidri][ryan_qmspi_init] cfg->cs_timing = 0x%X\n", cfg->cs_timing);
	printk("[spidri][ryan_qmspi_init] cfg->girq = 0x%X\n", cfg->girq);
	printk("[spidri][ryan_qmspi_init] cfg->girq_pos = 0x%X\n", cfg->girq_pos);
	printk("[spidri][ryan_qmspi_init] cfg->girq_nvic_aggr = 0x%X\n", cfg->girq_nvic_aggr);
	printk("[spidri][ryan_qmspi_init] cfg->girq_nvic_direct = 0x%X\n", cfg->girq_nvic_direct);
	printk("[spidri][ryan_qmspi_init] cfg->irq_pri = 0x%X\n", cfg->irq_pri);
	printk("[spidri][ryan_qmspi_init] cfg->chip_sel = 0x%X\n", cfg->chip_sel);
	printk("[spidri][ryan_qmspi_init] cfg->width = 0x%X\n", cfg->width);
	printk("[spidri][ryan_qmspi_init] cfg->unused = 0x%X\n", cfg->unused);

	printk("[spidri][ryan_qmspi_init] cfg->pcfg->states->pins = 0x%X\n", cfg->pcfg->states->pins);
	printk("[spidri][ryan_qmspi_init] cfg->pcfg->states->pin_cnt = 0x%X\n", cfg->pcfg->states->pin_cnt);
	printk("[spidri][ryan_qmspi_init] cfg->pcfg->states->id = 0x%X\n", cfg->pcfg->states->id);
	printk("[spidri][ryan_qmspi_init] cfg->pcfg->state_cnt = 0x%X\n", cfg->pcfg->state_cnt);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("QSPI pinctrl setup failed (%d)", ret);
		return ret;
	}

	mchp_pcr_periph_slp_ctrl(PCR_QMSPI, MCHP_PCR_SLEEP_DIS);

	uint32_t *qmspi_ModeReg = ((volatile uint32_t *)(uintptr_t)(&regs->MODE));
	printk("[spidri][ryan_qmspi_init] SpiQmspiModeReg base address = 0x%X\n", qmspi_ModeReg);
	QmspiModeRegister QmspiModeReg;
	QmspiModeReg.VALUE = *qmspi_ModeReg;
	printk("[spidri][ryan_qmspi_init] QmspiModeReg.VALUE = 0x%X\n", QmspiModeReg.VALUE);
	QmspiModeReg.bitfld.SOFT_RESET = 1;
	*qmspi_ModeReg = QmspiModeReg.VALUE;
	printk("[spidri][ryan_qmspi_init] *qmspi_ModeReg  = 0x%X\n", *qmspi_ModeReg );
	//printk("[spidri][ryan_qmspi_init] regs->MODE = 0x%X\n", regs->MODE);

	printk("[spidri][ryan_qmspi_init] DT_INST_IRQN(0) = 0x%X\n", DT_INST_IRQN(0));
	printk("[spidri][ryan_qmspi_init] DT_INST_IRQ(0, priority) = 0x%X\n", DT_INST_IRQ(0, priority));
	printk("[spidri][ryan_qmspi_init] DEVICE_DT_INST_GET(0) = 0x%X\n", DEVICE_DT_INST_GET(0));

	printk("[spidri][ryan_qmspi_init] cfg->girq = 0x%X\n", cfg->girq);
	printk("[spidri][ryan_qmspi_init] cfg->girq_pos = 0x%X\n", cfg->girq_pos);

	cfg->irq_config_func(); //ryan interrupt

	MCHP_GIRQ_CLR_EN(cfg->girq, cfg->girq_pos);
	MCHP_GIRQ_SRC_CLR(cfg->girq, cfg->girq_pos);

	MCHP_GIRQ_BLK_CLREN(cfg->girq);
	NVIC_ClearPendingIRQ(cfg->girq_nvic_direct);

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	MCHP_GIRQ_SRC(cfg->girq) = BIT(cfg->girq_pos); //ryan interrupt
	MCHP_GIRQ_ENSET(cfg->girq) = BIT(cfg->girq_pos); //ryan interrupt

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

/*
 * Initialize QMSPI controller.
 * Disable sleep control.
 * Disable and clear interrupt status.
 * Initialize SPI context.
 * QMSPI will be configured and enabled when the transceive API is called.
 */
static int qmspi_init(const struct device *dev)
{
	printk("[spidri][qmspi_init] func entry\n");
	int err;
	const struct spi_qmspi_config *cfg = dev->config;
	struct spi_qmspi_data *data = dev->data;
	QMSPI_Type *regs = cfg->regs;
	int ret;

	printk("[spidri][qmspi_init] DT_INST_REG_ADDR(0) = 0x%X\n", (QMSPI_Type *)DT_INST_REG_ADDR(0));
	printk("[spidri][qmspi_init] QMSPI Mode reg base address = 0x%X\n", &regs->MODE);
	printk("[spidri][qmspi_init] QMSPI Control reg base address = 0x%X\n", &regs->CTRL);
	printk("[spidri][qmspi_init] QMSPI Execute reg base address = 0x%X\n", &regs->EXE);
	printk("[spidri][qmspi_init] QMSPI Status base address = 0x%X\n", &regs->STS);
	printk("[spidri][qmspi_init] QMSPI Interface Control reg base address = 0x%X\n", &regs->IFCTRL);
	printk("[spidri][qmspi_init] QMSPI Buffer Count Status reg base address = 0x%X\n", &regs->BCNT_STS);
	printk("[spidri][qmspi_init] QMSPI Interrupt Enable reg base address = 0x%X\n", &regs->IEN);
	printk("[spidri][qmspi_init] QMSPI Buffer Count Trigger reg base address = 0x%X\n", &regs->BCNT_TRIG);
	printk("[spidri][qmspi_init] QMSPI Transmit Buffer reg base address = 0x%X\n", &regs->TX_FIFO);
	printk("[spidri][qmspi_init] QMSPI Receive Buffer reg base address = 0x%X\n", &regs->RX_FIFO);
	printk("[spidri][qmspi_init] QMSPI Chip Select Timing reg base address = 0x%X\n", &regs->CSTM);
	printk("[spidri][qmspi_init] &regs->RSVD1 = 0x%X\n", &regs->RSVD1);
	printk("[spidri][qmspi_init] &regs->DESCR = 0x%X\n", &regs->DESCR);

	printk("[spidri][qmspi_init] cfg->cs_timing = 0x%X\n", cfg->cs_timing);
	printk("[spidri][qmspi_init] cfg->girq = 0x%X\n", cfg->girq);
	printk("[spidri][qmspi_init] cfg->girq_pos = 0x%X\n", cfg->girq_pos);
	printk("[spidri][qmspi_init] cfg->girq_nvic_aggr = 0x%X\n", cfg->girq_nvic_aggr);
	printk("[spidri][qmspi_init] cfg->girq_nvic_direct = 0x%X\n", cfg->girq_nvic_direct);
	printk("[spidri][qmspi_init] cfg->irq_pri = 0x%X\n", cfg->irq_pri);
	printk("[spidri][qmspi_init] cfg->chip_sel = 0x%X\n", cfg->chip_sel);
	printk("[spidri][qmspi_init] cfg->width = 0x%X\n", cfg->width);
	printk("[spidri][qmspi_init] cfg->unused = 0x%X\n", cfg->unused);

	printk("[spidri][qmspi_init] cfg->pcfg->states->pins = 0x%X\n", cfg->pcfg->states->pins);
	printk("[spidri][qmspi_init] cfg->pcfg->states->pin_cnt = 0x%X\n", cfg->pcfg->states->pin_cnt);
	printk("[spidri][qmspi_init] cfg->pcfg->states->id = 0x%X\n", cfg->pcfg->states->id);
	printk("[spidri][qmspi_init] cfg->pcfg->state_cnt = 0x%X\n", cfg->pcfg->state_cnt);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("QSPI pinctrl setup failed (%d)", ret);
		return ret;
	}

	mchp_pcr_periph_slp_ctrl(PCR_QMSPI, MCHP_PCR_SLEEP_DIS);

	regs->MODE = MCHP_QMSPI_M_SRST;
	//printk("[spidri][qmspi_init] regs->MODE = 0x%X\n", regs->MODE);

	printk("[spidri][qmspi_init] DT_INST_IRQN(0) = 0x%X\n", DT_INST_IRQN(0));
	printk("[spidri][qmspi_init] DT_INST_IRQ(0, priority) = 0x%X\n", DT_INST_IRQ(0, priority));
	printk("[spidri][qmspi_init] DEVICE_DT_INST_GET(0) = 0x%X\n", DEVICE_DT_INST_GET(0));

	printk("[spidri][qmspi_init] cfg->girq = 0x%X\n", cfg->girq);
	printk("[spidri][qmspi_init] cfg->girq_pos = 0x%X\n", cfg->girq_pos);

	cfg->irq_config_func(); //ryan interrupt

	MCHP_GIRQ_CLR_EN(cfg->girq, cfg->girq_pos);
	MCHP_GIRQ_SRC_CLR(cfg->girq, cfg->girq_pos);

	MCHP_GIRQ_BLK_CLREN(cfg->girq);
	NVIC_ClearPendingIRQ(cfg->girq_nvic_direct);

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	MCHP_GIRQ_SRC(cfg->girq) = BIT(cfg->girq_pos); //ryan interrupt
	MCHP_GIRQ_ENSET(cfg->girq) = BIT(cfg->girq_pos); //ryan interrupt

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_qmspi_driver_api = {
	.transceive = qmspi_transceive_sync,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = qmspi_transceive_async,
#endif
	.release = ryan_qmspi_release, //qmspi_release,
};


#define XEC_QMSPI_CS_TIMING_VAL(a, b, c, d) (((a) & 0xFu) \
					     | (((b) & 0xFu) << 8) \
					     | (((c) & 0xFu) << 16) \
					     | (((d) & 0xFu) << 24))


#define XEC_QMSPI_0_CS_TIMING XEC_QMSPI_CS_TIMING_VAL(			\
				DT_INST_PROP(0, dcsckon),		\
				DT_INST_PROP(0, dckcsoff),		\
				DT_INST_PROP(0, dldh),			\
				DT_INST_PROP(0, dcsda))

#if DT_NODE_HAS_STATUS(DT_INST(0, microchip_xec_qmspi), okay)

PINCTRL_DT_INST_DEFINE(0);

static void ryan_spi_xec_irq_config(void);

static const struct spi_qmspi_config spi_qmspi_0_config = {
	.regs = (QMSPI_Type *)DT_INST_REG_ADDR(0),
	.cs_timing = XEC_QMSPI_0_CS_TIMING,
	.girq = MCHP_QMSPI_GIRQ_NUM,
	.girq_pos = MCHP_QMSPI_GIRQ_POS,
	.girq_nvic_direct = MCHP_QMSPI_GIRQ_NVIC_DIRECT,
	.irq_pri = DT_INST_IRQ(0, priority),
	.chip_sel = DT_INST_PROP(0, chip_select),
	.width = DT_INST_PROP(0, lines),
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.irq_config_func = ryan_spi_xec_irq_config,
};

static void ryan_spi_xec_irq_config(void)		\
{                                               \
	IRQ_CONNECT(DT_INST_IRQN(0),				\
		    DT_INST_IRQ(0, priority),			\
		    ryan_spi_xec_isr,					\
		    DEVICE_DT_INST_GET(0), 0);			\
	irq_enable(DT_INST_IRQN(0));				\
}

static struct spi_qmspi_data spi_qmspi_0_dev_data = {
	SPI_CONTEXT_INIT_LOCK(spi_qmspi_0_dev_data, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_qmspi_0_dev_data, ctx),
	SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(0), ctx)
};

DEVICE_DT_INST_DEFINE(0,
		    &ryan_qmspi_init, NULL, &spi_qmspi_0_dev_data,
		    &spi_qmspi_0_config, POST_KERNEL,
		    CONFIG_SPI_INIT_PRIORITY, &spi_qmspi_driver_api);			

#endif /* DT_NODE_HAS_STATUS(DT_INST(0, microchip_xec_qmspi), okay) */
