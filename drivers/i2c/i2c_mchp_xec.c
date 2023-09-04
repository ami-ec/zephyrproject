/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_xec_i2c

#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_mchp, CONFIG_I2C_LOG_LEVEL);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define	BIT0	0x01		// 0000,0001
#define	BIT1	0x02		// 0000,0010
#define	BIT2	0x04		// 0000,0100
#define	BIT3	0x08		// 0000,1000
#define	BIT4	0x10		// 0001,0000
#define	BIT5	0x20		// 0010,0000
#define	BIT6	0x40		// 0100,0000
#define	BIT7	0x80		// 1000,0000
#define	BIT8	0x0100		// 0000,0001,0000,0000
#define	BIT9	0x0200		// 0000,0010,0000,0000
#define	BIT10	0x0400		// 0000,0100,0000,0000
#define	BIT11	0x0800		// 0000,1000,0000,0000
#define	BIT12	0x1000		// 0001,0000,0000,0000
#define	BIT13	0x2000		// 0010,0000,0000,0000
#define	BIT14	0x4000		// 0100,0000,0000,0000
#define	BIT15	0x8000		// 1000,0000,0000,0000
#define	BIT16	0x00010000	// 0000,0000,0000,0001,0000,0000,0000,0000
#define	BIT17	0x00020000	// 0000,0000,0000,0010,0000,0000,0000,0000
#define	BIT18	0x00040000	// 0000,0000,0000,0100,0000,0000,0000,0000
#define	BIT19	0x00080000	// 0000,0000,0000,1000,0000,0000,0000,0000
#define	BIT20	0x00100000	// 0000,0000,0001,0000,0000,0000,0000,0000
#define	BIT21	0x00200000	// 0000,0000,0010,0000,0000,0000,0000,0000
#define	BIT22	0x00400000	// 0000,0000,0100,0000,0000,0000,0000,0000
#define	BIT23	0x00800000	// 0000,0000,1000,0000,0000,0000,0000,0000
#define	BIT24	0x01000000	// 0000,0001,0000,0000,0000,0000,0000,0000
#define	BIT25	0x02000000	// 0000,0010,0000,0000,0000,0000,0000,0000
#define	BIT26	0x04000000	// 0000,0100,0000,0000,0000,0000,0000,0000
#define	BIT27	0x08000000	// 0000,1000,0000,0000,0000,0000,0000,0000
#define	BIT28	0x10000000	// 0001,0000,0000,0000,0000,0000,0000,0000
#define	BIT29	0x20000000	// 0010,0000,0000,0000,0000,0000,0000,0000
#define	BIT30	0x40000000	// 0100,0000,0000,0000,0000,0000,0000,0000
#define	BIT31	0x80000000	// 1000,0000,0000,0000,0000,0000,0000,0000

#define RYAN_I2C_SMB0_BASE_ADDR		(*(	volatile uint32_t	*)	0x40004000 )
#define RYAN_I2C_SMB1_BASE_ADDR		(*(	volatile uint32_t	*)	0x40004400 )
#define RYAN_I2C_SMB2_BASE_ADDR		(*(	volatile uint32_t	*)	0x40004800 )
#define RYAN_I2C_SMB3_BASE_ADDR		(*(	volatile uint32_t	*)	0x40004C00 )
#define RYAN_I2C_SMB4_BASE_ADDR		(*(	volatile uint32_t	*)	0x40005000 )

#define RYAN_I2C_CONTROL_REGISTER_OFFSET                         0x00
#define RYAN_I2C_STATUS_REGISTER_OFFSET                          0x00
#define RYAN_I2C_OWN_ADDRESS_REGISTER_OFFSET                     0x04
#define RYAN_I2C_DATA_REGISTER_OFFSET                            0x08
#define RYAN_I2C_REPEATED_START_HOLD_TIME_REGISTER_OFFSET        0x18
#define RYAN_I2C_COMPLETION_REGISTER_OFFSET                      0x20
#define RYAN_I2C_CONFIGURATION_REGISTER_OFFSET                   0x28
#define RYAN_I2C_BUS_CLOCK_REGISTER_OFFSET                       0x2C
#define RYAN_I2C_BUS_BLOCK_ID_REGISTER_OFFSET                    0x30 
#define RYAN_I2C_BUS_REVISION_REGISTER_OFFSET                    0x34 
#define RYAN_I2C_BUS_BIT_BANG_CONTROL_REGISTER_OFFSET            0x38
#define RYAN_I2C_DATA_TIMING_REGISTER_OFFSET                     0x40
#define RYAN_I2C_TIME_OUT_SCALING_REGISTER_OFFSET                0x44
#define RYAN_I2C_WAKE_STATUS_REGISTER_OFFSET                     0x60
#define RYAN_I2C_WAKE_ENABLE_REGISTER_OFFSET                     0x64
#define RYAN_I2C_SLAVE_ADDRESS_REGISTER_OFFSET                   0x6C
#define RYAN_I2C_PROMISCUOUS_INTERRUPT_REGISTER_OFFSET           0x70
#define RYAN_I2C_PROMISCUOUS_INTERRUPT_ENABLE_REGISTER_OFFSET    0x74
#define RYAN_I2C_PROMISCUOUS_CONTROL_REGISTER_OFFSET             0x78

#define RYAN_I2C_CHANNEL 0

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t PROMISCUOUS_ACK    :1;     // bits[0] Promiscuous Ack. 0=NAK the address byte, 1=ACK the address byte
		uint32_t                    :31;    // bits[31:1] Reserved.
	} bitfld;
}i2c_PromiscuousControlRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t PROMISCUOUS_ADDRESS_INTERRUPT_ENABLE    :1;     // bits[0] Promiscuous Address Interrupt Enable. Enables Slave Address.
		uint32_t                                         :31;    // bits[31:1] Reserved.
	} bitfld;
}i2c_PromiscuousInterruptEnableRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t PROMISCUOUS_ADDRESS_INTERRUPT    :1;     // bits[0] Functional only in Promiscuous Mode.
		uint32_t                                  :31;    // bits[31:1] Reserved.
	} bitfld;
}i2c_PromiscuousInterruptRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t SLAVE_ADDRESS    :8;     // bits[7:0] This field is the Slave Address and Direction bit from the 1st byte of a Slave transfer.
		uint32_t                  :24;    // bits[31:8] Reserved.
	} bitfld;
}i2c_SlaveAddressRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t START_DETECT_INT_EN    :1;     // bits[0] Enable Start Bit Detection Interrupt. The Start Bit Detection Interrupt is wake-capable.
		uint32_t                        :31;    // bits[31:1] Reserved.
	} bitfld;
}i2c_WakeEnableRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t START_BIT_DETECTION    :1;     // bits[0] This bit is set to '1' when a START bit is detected while the controller is enabled.
		uint32_t                        :31;    // bits[31:1] Reserved.
	} bitfld;
}i2c_WakeStatusRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t                 :24;    // bits[23:0] Test.
		uint32_t BUS_IDLE_MIN    :8;     // bits[31:8] 
	} bitfld;
}i2c_TimeOutScalingRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t DATA_HOLD           :8;    // bits[7:0] 
		uint32_t RESTART_SETUP       :8;    // bits[15:8] 
		uint32_t STOP_SETUP          :8;    // bits[23:16] 
		uint32_t FIRST_START_HOLD    :8;    // bits[31:24] 
	} bitfld;
}i2c_DataTimingRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t BBEN      :1;     // bits[0] Bit-Bang Mode Enable.
		uint32_t CLDIR     :1;     // bits[1] Bit-Bang Mode Clock direction.
		uint32_t DADIR     :1;     // bits[2] Bit-Bang Mode Data direction.
		uint32_t BBCLK     :1;     // bits[3] Bit-Bang Mode Clock.state.
		uint32_t BBDAT     :1;     // bits[4] Bit-Bang Mode Data.state.
		uint32_t BBCLKI    :1;     // bits[5] Bit-Bang Clock In.
		uint32_t BBDATI    :1;     // bits[6] Bit-Bang Data In.
		uint32_t           :25;    // bits[31:7] Reserved.
	} bitfld;
}i2c_BitBangControlRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t    REVISION    :8;     // bits[7:0] Block Revision Number.
		uint32_t                :24;    // bits[31:8] Reserved.
	} bitfld;
}i2c_RevisionRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t ID    :8;     // bits[7:0] Block ID.
		uint32_t       :24;    // bits[31:8] Reserved.
	} bitfld;
}i2c_BlockIdRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t LOW_PERIOD     :1;     // bits[7:0] 
		uint32_t HIGH_PERIOD    :1;     // bits[15:8] 
		uint32_t                :16;    // bits[31:16] Reserved.
	} bitfld;
}i2c_BusClockRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t                 :2;     // bits[1:0] Reserved.
		uint32_t DTEN            :1;     // bits[2] This bit enables Device Time-Out checking.
		uint32_t MCEN            :1;     // bits[3] This bit enables Master Cumulative Time-Out checking.
		uint32_t SCEN            :1;     // bits[4] This bit enables Slave Cumulative Time-Out checking.
		uint32_t BIDEN           :1;     // bits[5] This bit enables Bus Idle Time-Out checking.
		uint32_t TIMERR          :1;     // bits[6] Time Out Error Detected.
		uint32_t                 :1;     // bits[7] Reserved.
		uint32_t DTO             :1;     // bits[8] Device Time-Out status.
		uint32_t MCTO            :1;     // bits[9] Master Cumulative Time-Out status.
		uint32_t SCTO            :1;     // bits[10] Slave Cumulative Time-Out status.
		uint32_t CHDL            :1;     // bits[11] Clock High Data Low time-out status. This status is asserted if the SCL remains high while SDA remains low.
		uint32_t CHDH            :1;     // bits[12] Clock High Data High time-out status.This is the bus idle time-out status.
		uint32_t BER             :1;     // bits[13] Bus Error Status.
		uint32_t LAB             :1;     // bits[14] Lost Arbitration Status.
		uint32_t                 :1;     // bits[15] Reserved.
		uint32_t SNAKR           :1;     // bits[16] Slave NACK sent while Receiving.
		uint32_t STR             :1;     // bits[17] Slave Transmit/Receive.
		uint32_t                 :1;     // bits[18] Reserved.
		uint32_t                 :1;     // bits[19] Test.
		uint32_t REPEAT_READ     :1;     // bits[20] 
		uint32_t REPEAT_WRITE    :1;     // bits[21] 
		uint32_t                 :2;     // bits[23:22] Reserved.
		uint32_t MNAKX           :1;     // bits[24] Master received a NACK while Transmitting.
		uint32_t MTR             :1;     // bits[25] Master Transmit/Receive. This bit reports the phase of the Master State Machine when it asserts MDONE.
		uint32_t                 :3;     // bits[28:26] Reserved.
		uint32_t IDLE            :1;     // bits[29] 
		uint32_t MDONE           :1;     // bits[30] 1=Master transaction completed, 0=Master transaction not completed
		uint32_t SDONE           :1;     // bits[31] Slave Done. 1=Slave transaction completed, 0=Slave transaction not completed
	} bitfld;
}i2c_CompletionRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t RPT_START_HOLD_TIME    :8;     // bits[7:0] This is the value of the timing requirement tHd:Sta in the I2C specification for a repeated START bit.
		uint32_t                        :24;    // bits[31:8] Reserved.
	} bitfld;
}i2c_PepeatedStartHoldTimeRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t DATA    :8;     // bits[7:0] This register holds the data that are either shifted out to or shifted in from the I2C port.
		uint32_t         :24;    // bits[31:8] Reserved.
	} bitfld;
}i2c_DataRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t NBB        :1;     // bits[0] The Bus Busy bit indicates when the bus is in use. A zero indicates that the bus is busy and access is not possible.
		uint32_t LAB        :1;     // bits[1] LAB (Lost Arbitration) is set when, in multi-master operation, arbitration is lost to another master on the bus.
		uint32_t AAS        :1;     // bits[2] AAS (Addressed As Slave) is valid only when PIN is asserted ('0').
		uint32_t LRB_AD0    :1;     // bits[3] LBR/AB0 ("Last Received Bit" or "Address 0") is valid only while the PIN bit is asserted ('0').
		uint32_t BER        :1;     // bits[4] When BER (Bus Error) is asserted, a misplaced START or STOP condition or Bus Time-outs have been detected.
		uint32_t STS        :1;     // bits[5] This bit is asserted (‘1’) when an externally generated STOP condition is detected. It is used only in slave receiver mode.
		uint32_t            :1;     // bits[6] Test.
		uint32_t PIN        :1;     // bits[7] The operation of PIN (Pending Interrupt).
		uint32_t            :24;    // bits[31:8] Reserved.
	} bitfld;
}i2c_StatusRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t  PORT_SEL   :  4;               //!< [0..3] determine one of 16 bus ports apply to SDAT/SCLK               
		uint32_t  TCEN       :  1;               //!< [4..4] 1: Bus Time-Outs are enabled                                   
		uint32_t  SLOW_CLOCK :  1;               //!< [5..5] 1: Bus Clock multiplied by 4, thus frequency/4                 
		uint32_t             :  1;
		uint32_t  PECEN      :  1;               //!< [7..7] 1: Hardware PEC Support is enabled                             
		uint32_t  FEN        :  1;               //!< [8..8] 1: Digital Filter is enabled. 0: bypassed.                     
		uint32_t  RESET      :  1;               //!< [9..9] 1: initialized to power-on default state.                      
		uint32_t  ENAB       :  1;               //!< [10..10] 1: normal operation, 0: lowest power                         
		uint32_t  DSA        :  1;               //!< [11..11] 0: Slave Address I2C Compatibility, 1: SMBus                 
		uint32_t  FAIR       :  1;               //!< [12..12] 1: MCTP Fairness protocol is in effect.                      
		uint32_t             :  1;
		uint32_t  GC_DIS     :  1;               //!< [14..14] General Call address 0: enabled, 1: disabled                 
		uint32_t             :  1;
		uint32_t  FLUSH_SXBUF:  1;               //!< [16..16] 1: Slave Transmit Buffer to be marked empty.                 
		uint32_t  FLUSH_SRBUF:  1;               //!< [17..17] 1: Slave Receive Buffer to be marked empty.                  
		uint32_t  FLUSH_MXBUF:  1;               //!< [18..18] 1: Master Transmit Buffer to be marked empty.                
		uint32_t  FLUSH_MRBUF:  1;               //!< [19..19] 1: Master Receive Buffer to be marked empty.                 
		uint32_t             :  8;
		uint32_t  EN_AAS     :  1;               //!< [28..28] 0: Disable AAS Interrupt, 1: Enable                          
		uint32_t  ENIDI      :  1;               //!< [29..29] 1: Idle interrupt is enabled. 0: disabled.                   
		uint32_t  ENMI       :  1;               //!< [30..30] 1: Master Done interrupt is enabled. 0: disabled             
		uint32_t  ENSI       :  1;               //!< [31..31] 1: Slave Done interrupt is enabled. 0: disabled              
	} bitfld;
}i2c_ConfigurationRegister;

typedef union {
	uint32_t VALUE;
	struct {
		uint32_t ACK    :1;     // bits[0] Acknowledge.
		uint32_t STO    :1;     // bits[1] Transmit STOP.
		uint32_t STA    :1;     // bits[2] Transmit START.
		uint32_t ENI    :1;     // bits[3] The Enable Interrupt bit controls the Interrupts.
		uint32_t        :2;     // bits[5:4] Reserved.
		uint32_t ESO    :1;     // bits[6] The Enable Serial Output bit enables and disables the serial data output.
		uint32_t PIN    :1;     // bits[7] The Pending Interrupt Not bit serves as a software reset function.
		uint32_t        :24;    // bits[31:8] Reserved.
	} bitfld;
}i2c_ControlRegister;

struct i2c_register {
	i2c_ControlRegister *ControlRegister;
	i2c_StatusRegister *StatusRegister;
	i2c_DataRegister *DataRegister;
	i2c_PepeatedStartHoldTimeRegister *PepeatedStartHoldTimeRegister;
	i2c_CompletionRegister *CompletionRegister;
	i2c_ConfigurationRegister *ConfigurationRegister;
	i2c_BusClockRegister *BusClockRegister;
	i2c_BlockIdRegister *BlockIdRegister;
	i2c_RevisionRegister *RevisionRegister;
	i2c_BitBangControlRegister *BitBangControlRegister;
	i2c_DataTimingRegister *DataTimingRegister;
	i2c_TimeOutScalingRegister *TimeOutScalingRegister;
	i2c_WakeStatusRegister *WakeStatusRegister;
	i2c_WakeEnableRegister *WakeEnableRegister;
	i2c_SlaveAddressRegister *SlaveAddressRegister;
	i2c_PromiscuousInterruptRegister *PromiscuousInterruptRegister;
	i2c_PromiscuousInterruptEnableRegister *PromiscuousInterruptEnableRegister;
	i2c_PromiscuousControlRegister *PromiscuousControlRegister;
};

/*
#define		PIN						BIT7		// Operation of the Pending Interrupt
#define		STS						BIT5		// Slave receiver
#define		BER						BIT4		// Bus Error
#define		AD0						BIT3		// Address 0
#define		LRB						BIT3		// Last Received
#define		AAS						BIT2		// Addressed As Slave
#define		LAB						BIT1		// Lost Arbitration
#define		NBB						BIT0		// Bus Busy

#define		PIN						BIT7		// ACK
#define		ESO						BIT6		// Enable Serial Output
#define		ENI						BIT3		// Enable Interrupt
#define		STA						BIT2		// Transmit START+address
#define		STO						BIT1		// Transmit STOP go to SLV/REC mode
#define		ACK						BIT0		// ACK
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SPEED_100KHZ_BUS    0
#define SPEED_400KHZ_BUS    1
#define SPEED_1MHZ_BUS      2

#define EC_OWN_I2C_ADDR		0x7F
#define RESET_WAIT_US		20
#define BUS_IDLE_US_DFLT	5

/* I2C timeout is 10 ms (WAIT_INTERVAL * WAIT_COUNT) */
#define WAIT_INTERVAL		50
#define WAIT_COUNT		200

/* Line High Timeout is 2.5 ms (WAIT_LINE_HIGH_USEC * WAIT_LINE_HIGH_COUNT) */
#define WAIT_LINE_HIGH_USEC	25
#define WAIT_LINE_HIGH_COUNT	100

/* I2C Read/Write bit pos */
#define I2C_READ_WRITE_POS  0

struct xec_speed_cfg {
	uint32_t bus_clk;
	uint32_t data_timing;
	uint32_t start_hold_time;
	uint32_t config;
	uint32_t timeout_scale;
};

struct i2c_xec_config {
	uint32_t port_sel;
	uint32_t base_addr;
	uint8_t girq_id;
	uint8_t girq_bit;
	struct gpio_dt_spec sda_gpio;
	struct gpio_dt_spec scl_gpio;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
};


struct i2c_xec_data {
	uint32_t pending_stop;
	uint32_t error_seen;
	uint32_t timeout_seen;
	uint32_t previously_in_read;
	uint32_t speed_id;
	struct i2c_target_config *slave_cfg;
	bool slave_attached;
	bool slave_read;

	struct k_sem device_sync_sem;
	//struct i2c_register *I2cRegister;
};

/* Recommended programming values based on 16MHz
 * i2c_baud_clk_period/bus_clk_period - 2 = (low_period + hi_period)
 * bus_clk_reg (16MHz/100KHz -2) = 0x4F + 0x4F
 *             (16MHz/400KHz -2) = 0x0F + 0x17
 *             (16MHz/1MHz -2) = 0x05 + 0x09
 */
static const struct xec_speed_cfg xec_cfg_params[] = {
	[SPEED_100KHZ_BUS] = {
		.bus_clk            = 0x00004F4F,
		.data_timing        = 0x0C4D5006,
		.start_hold_time    = 0x0000004D,
		.config             = 0x01FC01ED,
		.timeout_scale      = 0x4B9CC2C7,
	},
	[SPEED_400KHZ_BUS] = {
		.bus_clk            = 0x00000F17,
		.data_timing        = 0x040A0A06,
		.start_hold_time    = 0x0000000A,
		.config             = 0x01000050,
		.timeout_scale      = 0x159CC2C7,
	},
	[SPEED_1MHZ_BUS] = {
		.bus_clk            = 0x00000509,
		.data_timing        = 0x04060601,
		.start_hold_time    = 0x00000006,
		.config             = 0x10000050,
		.timeout_scale      = 0x089CC2C7,
	},
};

/*
static void ryan_i2c_xec_bus_isr(const struct device *dev);

struct ryan_i2c_isr {
	uint32_t girq_bit;
	void (*the_isr)(const struct device *dev);
};

const struct ryan_i2c_isr ryan_bus_isr[] = {
	{MCHP_I2C_SMB0_GIRQ_VAL, ryan_i2c_xec_bus_isr},
};

static uint8_t bus_isr_cnt = sizeof(ryan_bus_isr) / sizeof(struct ryan_i2c_isr);

static void ryan_i2c_xec_bus_isr(const struct device *dev)
{
	const struct i2c_xec_config *config = dev->config;
	uint32_t girq_result;

	girq_result = MCHP_GIRQ_RESULT(config->girq_id);

	for (int i = 0; i < bus_isr_cnt; i++) {
		struct ryan_i2c_isr entry = ryan_bus_isr[i];

		if (girq_result & entry.girq_bit) {
			if (entry.the_isr != NULL) {
				entry.the_isr(dev);
			}
		}
	}

	REG32(MCHP_GIRQ_SRC_ADDR(config->girq_id)) = girq_result;
}
*/

static void ryan_i2c_xec_reset_config(const struct device *dev)
{
	printk("[i2cdri][ryan_i2c_xec_reset_config] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	struct i2c_xec_data *data =
		(struct i2c_xec_data *const) (dev->data);

	uint32_t ba = config->base_addr;
	printk("[i2cdri][ryan_i2c_xec_reset_config] ba = 0x%X\n", ba);

	uint32_t *I2cConfigurationReg =  (volatile uint32_t *)((uintptr_t)(ba) + (uintptr_t)(RYAN_I2C_CONFIGURATION_REGISTER_OFFSET));
	uint32_t *I2cControlReg = ((volatile uint32_t *)(uintptr_t)(ba));

	// Assert RESET and clr others 
	i2c_ConfigurationRegister I2c_ConfigurationReg;
	I2c_ConfigurationReg.VALUE = *I2cConfigurationReg;
	printk("[i2cdri][ryan_i2c_xec_reset_config](01) I2c_ConfigurationReg.VALUE = 0x%X\n", I2c_ConfigurationReg.VALUE);
	I2c_ConfigurationReg.bitfld.RESET = 0x01;
	*I2cConfigurationReg = I2c_ConfigurationReg.VALUE;
	printk("[i2cdri][ryan_i2c_xec_reset_config](02) *I2cConfigurationReg = 0x%X\n", *I2cConfigurationReg);
	
	k_busy_wait(RESET_WAIT_US);

	// Bus reset
	*I2cConfigurationReg = 0;
	I2c_ConfigurationReg.VALUE = *I2cConfigurationReg;
	printk("[i2cdri][ryan_i2c_xec_reset_config](03) *I2cConfigurationReg = 0x%X\n", *I2cConfigurationReg);

	// Write 0x80. i.e Assert PIN bit, ESO = 0 and Interrupts
	// disabled (ENI)
	//
	i2c_ControlRegister I2c_ControlReg;
	I2c_ControlReg.VALUE = *I2cControlReg;
	printk("[i2cdri][ryan_i2c_xec_reset_config](04) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);
	I2c_ControlReg.bitfld.PIN = 0x01;
	*I2cControlReg = I2c_ControlReg.VALUE;
	printk("[i2cdri][ryan_i2c_xec_reset_config](05) *I2cControlReg = 0x%X\n", *I2cControlReg);

	printk("[i2cdri][ryan_i2c_xec_reset_config](06) config->port_sel = 0x%X\n", config->port_sel);

	// Enable controller and I2C filters 
	I2c_ConfigurationReg.VALUE = *I2cConfigurationReg;
	printk("[i2cdri][ryan_i2c_xec_reset_config](07) I2c_ConfigurationReg.VALUE = 0x%X\n", I2c_ConfigurationReg.VALUE);
	I2c_ConfigurationReg.bitfld.GC_DIS = 0x01;
	I2c_ConfigurationReg.bitfld.ENAB = 0x01;
	I2c_ConfigurationReg.bitfld.FEN = 0x01;
	I2c_ConfigurationReg.bitfld.ENMI = 0x01; //1: Master Done interrupt is enabled. 0: disabled 
	//I2c_ConfigurationReg.bitfld.ENIDI = 0x01; //1: Idle interrupt is enabled.
	I2c_ConfigurationReg.bitfld.PORT_SEL = config->port_sel;
	*I2cConfigurationReg = I2c_ConfigurationReg.VALUE;
	printk("[i2cdri][ryan_i2c_xec_reset_config](08) *I2cConfigurationReg = 0x%X\n", *I2cConfigurationReg);


	// Configure bus clock register, Data Timing register,
	// Repeated Start Hold Time register,
	// and Timeout Scaling register
	//
	MCHP_I2C_SMB_BUS_CLK(ba) = xec_cfg_params[data->speed_id].bus_clk;
	MCHP_I2C_SMB_DATA_TM(ba) = xec_cfg_params[data->speed_id].data_timing;
	MCHP_I2C_SMB_RSHT(ba) =
		xec_cfg_params[data->speed_id].start_hold_time;
	MCHP_I2C_SMB_TMTSC(ba) = xec_cfg_params[data->speed_id].timeout_scale;

	I2c_ControlReg.VALUE = *I2cControlReg;
	printk("[i2cdri][ryan_i2c_xec_reset_config](09) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);
	I2c_ControlReg.bitfld.PIN = 0x01;
	I2c_ControlReg.bitfld.ESO = 0x01;
	I2c_ControlReg.bitfld.ACK = 0x01;
	I2c_ControlReg.bitfld.ENI = 0x01; //Enable Interrupt
	*I2cControlReg = I2c_ControlReg.VALUE;
	printk("[i2cdri][ryan_i2c_xec_reset_config](10) *I2cControlReg = 0x%X\n", *I2cControlReg);

	uint32_t *I2cCompletionReg =  (volatile uint32_t *)((uintptr_t)(ba) + (uintptr_t)(RYAN_I2C_COMPLETION_REGISTER_OFFSET));
	i2c_CompletionRegister I2c_CompletionReg;
	I2c_CompletionReg.VALUE = *I2cCompletionReg;
	printk("[i2cdri][ryan_i2c_xec_reset_config](11) I2c_CompletionReg.VALUE = 0x%X\n", I2c_CompletionReg.VALUE);
	I2c_CompletionReg.bitfld.MDONE = 0x01;
	//I2c_CompletionReg.bitfld.IDLE = 0x01; //1: Idle interrupt is enabled.
	*I2cCompletionReg = I2c_CompletionReg.VALUE;
	printk("[i2cdri][ryan_i2c_xec_reset_config](12) *I2cCompletionReg = 0x%X\n", *I2cCompletionReg);

	k_busy_wait(RESET_WAIT_US);
}


static void i2c_xec_reset_config(const struct device *dev)
{
	printk("[i2cdri][i2c_xec_reset_config] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	struct i2c_xec_data *data =
		(struct i2c_xec_data *const) (dev->data);

	uint32_t ba = config->base_addr;

	printk("[i2cdri][i2c_xec_reset_config] ba = 0x%X\n", ba);

	printk("[i2cdri][i2c_xec_reset_config] MCHP_I2C_SMB_CFG(ba) = 0x%X\n", *(volatile uint32_t *)((uintptr_t)(ba) + (uintptr_t)(RYAN_I2C_CONFIGURATION_REGISTER_OFFSET)));
	// Assert RESET and clr others 
	MCHP_I2C_SMB_CFG(ba) = MCHP_I2C_SMB_CFG_RESET;
	printk("[i2cdri][i2c_xec_reset_config] MCHP_I2C_SMB_CFG(ba) RESET = 0x%X\n", *(volatile uint32_t *)((uintptr_t)(ba) + (uintptr_t)(RYAN_I2C_CONFIGURATION_REGISTER_OFFSET)));

	k_busy_wait(RESET_WAIT_US);

	// Bus reset 
	MCHP_I2C_SMB_CFG(ba) = 0;

	// Write 0x80. i.e Assert PIN bit, ESO = 0 and Interrupts
	// disabled (ENI)
	//
	MCHP_I2C_SMB_CTRL_WO(ba) = MCHP_I2C_SMB_CTRL_PIN;

	printk("[i2cdri][i2c_xec_reset_config] config->port_sel = 0x%X\n", config->port_sel);

	// Enable controller and I2C filters 
	MCHP_I2C_SMB_CFG(ba) = MCHP_I2C_SMB_CFG_GC_EN |
				MCHP_I2C_SMB_CFG_ENAB |
				MCHP_I2C_SMB_CFG_FEN |
				(config->port_sel &
				 MCHP_I2C_SMB_CFG_PORT_SEL_MASK);

	// Configure bus clock register, Data Timing register,
	// Repeated Start Hold Time register,
	// and Timeout Scaling register
	//
	MCHP_I2C_SMB_BUS_CLK(ba) = xec_cfg_params[data->speed_id].bus_clk;
	MCHP_I2C_SMB_DATA_TM(ba) = xec_cfg_params[data->speed_id].data_timing;
	MCHP_I2C_SMB_RSHT(ba) =
		xec_cfg_params[data->speed_id].start_hold_time;
	MCHP_I2C_SMB_TMTSC(ba) = xec_cfg_params[data->speed_id].timeout_scale;

	MCHP_I2C_SMB_CTRL_WO(ba) = MCHP_I2C_SMB_CTRL_PIN |
				   MCHP_I2C_SMB_CTRL_ESO |
				   MCHP_I2C_SMB_CTRL_ACK;

	k_busy_wait(RESET_WAIT_US);
}

static int xec_spin_yield(int *counter)
{
	printk("[i2cdri][xec_spin_yield] func entry\n");

	*counter = *counter + 1;
    printk("[i2cdri][xec_spin_yield] xec_spin_yield=0x%x\n", *counter);

	if (*counter > WAIT_COUNT) {
		return -ETIMEDOUT;
	}

	k_busy_wait(WAIT_INTERVAL);

	return 0;
}

static void cleanup_registers(uint32_t ba)
{
	printk("[i2cdri][cleanup_registers] func entry\n");

	uint32_t cfg = MCHP_I2C_SMB_CFG(ba);

	cfg |= MCHP_I2C_SMB_CFG_FLUSH_MXBUF_WO;
	MCHP_I2C_SMB_CFG(ba) = cfg;
	cfg &= ~MCHP_I2C_SMB_CFG_FLUSH_MXBUF_WO;

	cfg |= MCHP_I2C_SMB_CFG_FLUSH_MRBUF_WO;
	MCHP_I2C_SMB_CFG(ba) = cfg;
	cfg &= ~MCHP_I2C_SMB_CFG_FLUSH_MRBUF_WO;

	cfg |= MCHP_I2C_SMB_CFG_FLUSH_SXBUF_WO;
	MCHP_I2C_SMB_CFG(ba) = cfg;
	cfg &= ~MCHP_I2C_SMB_CFG_FLUSH_SXBUF_WO;

	cfg |= MCHP_I2C_SMB_CFG_FLUSH_SRBUF_WO;
	MCHP_I2C_SMB_CFG(ba) = cfg;
	cfg &= ~MCHP_I2C_SMB_CFG_FLUSH_SRBUF_WO;
}

#ifdef CONFIG_I2C_TARGET
static void restart_slave(uint32_t ba)
{
	MCHP_I2C_SMB_CTRL_WO(ba) = MCHP_I2C_SMB_CTRL_PIN |
				   MCHP_I2C_SMB_CTRL_ESO |
				   MCHP_I2C_SMB_CTRL_ACK |
				   MCHP_I2C_SMB_CTRL_ENI;
}
#endif

static void recover_from_error(const struct device *dev)
{
	printk("[i2cdri][recover_from_error] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	uint32_t ba = config->base_addr;

	printk("[i2cdri][recover_from_error] ba = 0x%X\n", ba);

	cleanup_registers(ba);
	ryan_i2c_xec_reset_config(dev); //i2c_xec_reset_config(dev);
}

static int wait_bus_free(const struct device *dev)
{
	printk("[i2cdri][wait_bus_free] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	int ret;
	int counter = 0;
	uint32_t ba = config->base_addr;

	printk("[i2cdri][wait_bus_free] ba = 0x%X\n", ba);

	while (!(MCHP_I2C_SMB_STS_RO(ba) & MCHP_I2C_SMB_STS_NBB)) {
		ret = xec_spin_yield(&counter);

		if (ret < 0) {
			return ret;
		}
	}

	/* Check for bus error */
	if (MCHP_I2C_SMB_STS_RO(ba) & MCHP_I2C_SMB_STS_BER) {
		recover_from_error(dev);
		return -EBUSY;
	}

	return 0;
}

/*
 * Wait with timeout for I2C controller to finish transmit/receive of one
 * byte(address or data).
 * When transmit/receive operation is started the I2C PIN status is 1. Upon
 * normal completion I2C PIN status asserts(0).
 * We loop checking I2C status for the following events:
 * Bus Error:
 *      Reset controller and return -EBUSY
 * Lost Arbitration:
 *      Return -EPERM. We lost bus to another controller. No reset.
 * PIN == 0: I2C Status LRB is valid and contains ACK/NACK data on 9th clock.
 *      ACK return 0 (success)
 *      NACK Issue STOP, wait for bus minimum idle time, return -EIO.
 * Timeout:
 *      Reset controller and return -ETIMEDOUT
 *
 * NOTE: After generating a STOP the controller will not generate a START until
 * Bus Minimum Idle time has expired.
 */
static int wait_completion(const struct device *dev)
{
    printk("[i2cdri][wait_completion] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	int ret;
	int counter = 0;
	uint32_t ba = config->base_addr;
	
	printk("[i2cdri][wait_completion] ba = 0x%X\n", ba);

	while (1) {
		uint8_t status = MCHP_I2C_SMB_STS_RO(ba);
		printk("[i2cdri][wait_completion] status = 0x%X\n", status);

		/* Is bus error ? */
		if (status & MCHP_I2C_SMB_STS_BER) {
			recover_from_error(dev);
			return -EBUSY;
		}

		/* Is Lost arbitration ? */
		status = MCHP_I2C_SMB_STS_RO(ba);
		if (status & MCHP_I2C_SMB_STS_LAB) {
			recover_from_error(dev);
			return -EPERM;
		}

		status = MCHP_I2C_SMB_STS_RO(ba);
		/* PIN -> 0 indicates I2C is done */
		if (!(status & MCHP_I2C_SMB_STS_PIN)) {
			/* PIN == 0. LRB contains state of 9th bit */
			if (status & MCHP_I2C_SMB_STS_LRB_AD0) { /* NACK? */
				/* Send STOP */
				MCHP_I2C_SMB_CTRL_WO(ba) =
						MCHP_I2C_SMB_CTRL_PIN |
						MCHP_I2C_SMB_CTRL_ESO |
						MCHP_I2C_SMB_CTRL_STO |
						MCHP_I2C_SMB_CTRL_ACK;
				k_busy_wait(BUS_IDLE_US_DFLT);
				return -EIO;
			}
			break; /* success: ACK */
		}

		ret = xec_spin_yield(&counter);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

/*
 * Call GPIO driver to read state of pins.
 * Return boolean true if both lines HIGH else return boolean false
 */
static bool check_lines_high(const struct device *dev)
{
	printk("[i2cdri][check_lines_high] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const)(dev->config);
	gpio_port_value_t sda = 0, scl = 0;

	if (gpio_port_get_raw(config->sda_gpio.port, &sda)) {
		LOG_ERR("gpio_port_get_raw for %s SDA failed", dev->name);
		return false;
	}

	/* both pins could be on same GPIO group */
	if (config->sda_gpio.port == config->scl_gpio.port) {
		scl = sda;
	} else {
		if (gpio_port_get_raw(config->scl_gpio.port, &scl)) {
			LOG_ERR("gpio_port_get_raw for %s SCL failed",
				dev->name);
			return false;
		}
	}

	return (sda & BIT(config->sda_gpio.pin)) && (scl & BIT(config->scl_gpio.pin));

}

static int i2c_xec_configure(const struct device *dev,
			     uint32_t dev_config_raw)
{
	printk("[i2cdri][i2c_xec_configure] func entry\n");

	struct i2c_xec_data *data =
		(struct i2c_xec_data *const) (dev->data);

	if (!(dev_config_raw & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP;
	}

	if (dev_config_raw & I2C_ADDR_10_BITS) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		data->speed_id = SPEED_100KHZ_BUS;
		break;
	case I2C_SPEED_FAST:
		data->speed_id = SPEED_400KHZ_BUS;
		break;
	case I2C_SPEED_FAST_PLUS:
		data->speed_id = SPEED_1MHZ_BUS;
		break;
	default:
		return -EINVAL;
	}

	ryan_i2c_xec_reset_config(dev); //i2c_xec_reset_config(dev);

	return 0;
}

static int ryan_i2c_xec_poll_write(const struct device *dev, struct i2c_msg msg,
			      uint16_t addr)
{
	printk("[i2cdri][ryan_i2c_xec_poll_write] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	struct i2c_xec_data *data =
		(struct i2c_xec_data *const) (dev->data);
	uint32_t ba = config->base_addr;
	uint8_t i2c_timer = 0, byte;
	int ret;

	uint32_t *I2cControlReg = ((volatile uint32_t *)(uintptr_t)(ba));
	uint32_t *I2cDataReg =  (volatile uint32_t *)((uintptr_t)(ba) + (uintptr_t)(RYAN_I2C_DATA_REGISTER_OFFSET));
	i2c_ControlRegister I2c_ControlReg;
	i2c_DataRegister I2c_DataReg;
	I2c_ControlReg.VALUE = *I2cControlReg;
	I2c_DataReg.VALUE = *I2cDataReg;

	printk("[i2cdri][ryan_i2c_xec_poll_write] base address = 0x%X\n", ba);

	if (data->timeout_seen == 1) {
		/* Wait to see if the slave has released the CLK */
		ret = wait_completion(dev);
		printk("[i2cdri][ryan_i2c_xec_poll_write](01) released CLK wait_completion = 0x%X\n",ret);

		if (ret) {
			data->timeout_seen = 1;
			LOG_ERR("%s: %s wait_completion failure %d\n",
				__func__, dev->name, ret);

			printk("[i2cdri][ryan_i2c_xec_poll_write](02) %s: %s wait_completion failure %d\n",__func__, dev->name, ret);	
			return ret;
		}
		data->timeout_seen = 0;

		/* If we are here, it means the slave has finally released
		 * the CLK. The master needs to end that transaction
		 * gracefully by sending a STOP on the bus.
		 */
		LOG_DBG("%s: %s Force Stop", __func__, dev->name);

		I2c_ControlReg.VALUE = *I2cControlReg;
		printk("[i2cdri][ryan_i2c_xec_poll_write](03) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);
		I2c_ControlReg.bitfld.PIN = 0x01;
		I2c_ControlReg.bitfld.ESO = 0x01;
		I2c_ControlReg.bitfld.STO = 0x01;
		I2c_ControlReg.bitfld.ACK = 0x01;
		*I2cControlReg = I2c_ControlReg.VALUE;		
		printk("[i2cdri][ryan_i2c_xec_poll_write](04) *I2cControlReg = 0x%X\n", *I2cControlReg);

		k_busy_wait(BUS_IDLE_US_DFLT);
		data->pending_stop = 0;

		/* If the timeout had occurred while the master was reading
		 * something from the slave, that read needs to be completed
		 * to clear the bus.
		 */
		if (data->previously_in_read == 1) {
			data->previously_in_read = 0;

			I2c_DataReg.VALUE = *I2cDataReg;
			byte = I2c_DataReg.VALUE;
			printk("[i2cdri][ryan_i2c_xec_poll_write](05) *I2cDataReg = 0x%X\n", byte);
		}

		printk("[i2cdri][ryan_i2c_xec_poll_write](06) -EBUSY\n");
		return -EBUSY;
	}

	if ((data->pending_stop == 0) || (data->error_seen == 1)) {
		/* Wait till clock and data lines are HIGH */
		while (check_lines_high(dev) == false) {
			if (i2c_timer >= WAIT_LINE_HIGH_COUNT) {
				LOG_DBG("%s: %s not high",
					__func__, dev->name);
				data->error_seen = 1;

				printk("[i2cdri][ryan_i2c_xec_poll_write](07) %s: %s not high\n",__func__, dev->name);
				return -EBUSY;
			}
			k_busy_wait(WAIT_LINE_HIGH_USEC);
			i2c_timer++;
		}

		if (data->error_seen) {
			LOG_DBG("%s: Recovering %s previously in error",
				__func__, dev->name);

			printk("[i2cdri][ryan_i2c_xec_poll_write](08) %s: Recovering %s previously in error\n",__func__, dev->name);	
			data->error_seen = 0;
			recover_from_error(dev);
		}

		/* Wait until bus is free */
		ret = wait_bus_free(dev);
		if (ret) {
			data->error_seen = 1;
			LOG_DBG("%s: %s wait_bus_free failure %d",
				__func__, dev->name, ret);

			printk("[i2cdri][ryan_i2c_xec_poll_write](09) %s: %s wait_bus_free failure %d\n",__func__, dev->name, ret);	
			return ret;
		}

		/* Send slave address */
		I2c_DataReg.bitfld.DATA = (addr & ~BIT(0));
		*I2cDataReg = I2c_DataReg.VALUE;
		printk("[i2cdri][ryan_i2c_xec_poll_write](10) *I2cDataReg = 0x%X\n", *I2cDataReg);

		/* Send start and ack bits */
		I2c_ControlReg.VALUE = *I2cControlReg;
		printk("[i2cdri][ryan_i2c_xec_poll_write](11) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);
		I2c_ControlReg.bitfld.PIN = 0x01;
		I2c_ControlReg.bitfld.ESO = 0x01;
		I2c_ControlReg.bitfld.STA = 0x01;
		//I2c_ControlReg.bitfld.STO = 0;
		I2c_ControlReg.bitfld.ACK = 0x01;
		*I2cControlReg = I2c_ControlReg.VALUE;
		printk("[i2cdri][ryan_i2c_xec_poll_write](12) *I2cControlReg = 0x%X\n", *I2cControlReg);	

		ret = wait_completion(dev);
		printk("[i2cdri][ryan_i2c_xec_poll_write](13) Send start and ack bits wait_completion = 0x%X\n",ret);

		switch (ret) {
		case 0:	/* Success */
			break;

		case -EIO:
			LOG_WRN("%s: No Addr ACK from Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][ryan_i2c_xec_poll_write](14) %s: No Addr ACK from Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);	
			return ret;

		default:
			data->error_seen = 1;
			LOG_ERR("%s: %s wait_comp error %d for addr send",
				__func__, dev->name, ret);

			printk("[i2cdri][ryan_i2c_xec_poll_write](15) %s: %s wait_comp error %d for addr send\n",__func__, dev->name, ret);		
			return ret;
		}
	}

	/* Send bytes */
	for (int i = 0U; i < msg.len; i++) {
		I2c_DataReg.bitfld.DATA = msg.buf[i];
		*I2cDataReg = I2c_DataReg.VALUE;
		printk("[i2cdri][ryan_i2c_xec_poll_write](16) *I2cDataReg = 0x%X\n", *I2cDataReg);

		ret = wait_completion(dev);
		printk("[i2cdri][ryan_i2c_xec_poll_write](17) Send bytes wait_completion = 0x%X\n",ret);

		switch (ret) {
		case 0:	/* Success */
			break;

		case -EIO:
			LOG_ERR("%s: No Data ACK from Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][ryan_i2c_xec_poll_write](18) %s: No Data ACK from Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);	
			return ret;

		case -ETIMEDOUT:
			data->timeout_seen = 1;
			LOG_ERR("%s: Clk stretch Timeout - Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][ryan_i2c_xec_poll_write](19) %s: Clk stretch Timeout - Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);	
			return ret;

		default:
			data->error_seen = 1;
			LOG_ERR("%s: %s wait_completion error %d for data send",
				__func__, dev->name, ret);

			printk("[i2cdri][ryan_i2c_xec_poll_write](20) %s: %s wait_completion error %d for data send\n",__func__, dev->name, ret);	
			return ret;
		}
	}

	/* Handle stop bit for last byte to write */
	if (msg.flags & I2C_MSG_STOP) {
		/* Send stop and ack bits */
		I2c_ControlReg.VALUE = *I2cControlReg;
		printk("[i2cdri][ryan_i2c_xec_poll_write](21) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);
		I2c_ControlReg.bitfld.PIN = 0x01;
		I2c_ControlReg.bitfld.ESO = 0x01;
		//I2c_ControlReg.bitfld.STA = 0;
		I2c_ControlReg.bitfld.STO = 0x01;
		I2c_ControlReg.bitfld.ACK = 0x01;
		*I2cControlReg = I2c_ControlReg.VALUE;	
		printk("[i2cdri][ryan_i2c_xec_poll_write](22) *I2cControlReg = 0x%X\n", *I2cControlReg);

		data->pending_stop = 0;
	} else {
		data->pending_stop = 1;
	}

	return 0;
}

static int i2c_xec_poll_write(const struct device *dev, struct i2c_msg msg,
			      uint16_t addr)
{
	printk("[i2cdri][i2c_xec_poll_write] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	struct i2c_xec_data *data =
		(struct i2c_xec_data *const) (dev->data);
	uint32_t ba = config->base_addr;
	uint8_t i2c_timer = 0, byte;
	int ret;

	printk("[i2cdri][i2c_xec_poll_write] base address = 0x%X\n", ba);

	if (data->timeout_seen == 1) {
		// Wait to see if the slave has released the CLK
		ret = wait_completion(dev);
		printk("[i2cdri][i2c_xec_poll_write](01) released CLK wait_completion = 0x%X\n",ret);

		if (ret) {
			data->timeout_seen = 1;
			LOG_ERR("%s: %s wait_completion failure %d\n",
				__func__, dev->name, ret);

			printk("[i2cdri][i2c_xec_poll_write](02) %s: %s wait_completion failure %d\n",__func__, dev->name, ret);	
			return ret;
		}
		data->timeout_seen = 0;

		// If we are here, it means the slave has finally released
		// the CLK. The master needs to end that transaction
		// gracefully by sending a STOP on the bus.
		//
		LOG_DBG("%s: %s Force Stop", __func__, dev->name);

		printk("[i2cdri][i2c_xec_poll_write](03) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));
		MCHP_I2C_SMB_CTRL_WO(ba) =
					MCHP_I2C_SMB_CTRL_PIN |
					MCHP_I2C_SMB_CTRL_ESO |
					MCHP_I2C_SMB_CTRL_STO |
					MCHP_I2C_SMB_CTRL_ACK;
		printk("[i2cdri][i2c_xec_poll_write](04) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));

		k_busy_wait(BUS_IDLE_US_DFLT);
		data->pending_stop = 0;

		// If the timeout had occurred while the master was reading
		// something from the slave, that read needs to be completed
		// to clear the bus.
		//
		if (data->previously_in_read == 1) {
			data->previously_in_read = 0;
			byte = MCHP_I2C_SMB_DATA(ba);
			printk("[i2cdri][i2c_xec_poll_write](05) MCHP_I2C_SMB_DATA(ba) = 0x%X\n", byte);
		}

		printk("[i2cdri][i2c_xec_poll_write](06) -EBUSY\n");
		return -EBUSY;
	}

	if ((data->pending_stop == 0) || (data->error_seen == 1)) {
		// Wait till clock and data lines are HIGH 
		while (check_lines_high(dev) == false) {
			if (i2c_timer >= WAIT_LINE_HIGH_COUNT) {
				LOG_DBG("%s: %s not high",
					__func__, dev->name);
				data->error_seen = 1;

				printk("[i2cdri][i2c_xec_poll_write](07) %s: %s not high\n",__func__, dev->name);
				return -EBUSY;
			}
			k_busy_wait(WAIT_LINE_HIGH_USEC);
			i2c_timer++;
		}

		if (data->error_seen) {
			LOG_DBG("%s: Recovering %s previously in error",
				__func__, dev->name);

			printk("[i2cdri][i2c_xec_poll_write](08) %s: Recovering %s previously in error\n",__func__, dev->name);	
			data->error_seen = 0;
			recover_from_error(dev);
		}

		// Wait until bus is free
		ret = wait_bus_free(dev);
		if (ret) {
			data->error_seen = 1;
			LOG_DBG("%s: %s wait_bus_free failure %d",
				__func__, dev->name, ret);

			printk("[i2cdri][i2c_xec_poll_write](09) %s: %s wait_bus_free failure %d\n",__func__, dev->name, ret);		
			return ret;
		}

		// Send slave address
		printk("[i2cdri][i2c_xec_poll_write](10) MCHP_I2C_SMB_DATA(ba) = 0x%X\n", MCHP_I2C_SMB_DATA(ba));
		MCHP_I2C_SMB_DATA(ba) = (addr & ~BIT(0));
		printk("[i2cdri][i2c_xec_poll_write](11) MCHP_I2C_SMB_DATA(ba) = 0x%X\n", MCHP_I2C_SMB_DATA(ba));

		// Send start and ack bits
		printk("[i2cdri][i2c_xec_poll_write](12) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));
		MCHP_I2C_SMB_CTRL_WO(ba) = MCHP_I2C_SMB_CTRL_PIN |
				MCHP_I2C_SMB_CTRL_ESO | MCHP_I2C_SMB_CTRL_STA |
				MCHP_I2C_SMB_CTRL_ACK;
		printk("[i2cdri][i2c_xec_poll_write](13) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));

		ret = wait_completion(dev);
		printk("[i2cdri][i2c_xec_poll_write](14) Send start and ack bits wait_completion = 0x%X\n",ret);

		switch (ret) {
		case 0:	// Success
			break;

		case -EIO:
			LOG_WRN("%s: No Addr ACK from Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][i2c_xec_poll_write](15) %s: No Addr ACK from Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);		
			return ret;

		default:
			data->error_seen = 1;
			LOG_ERR("%s: %s wait_comp error %d for addr send",
				__func__, dev->name, ret);

			printk("[i2cdri][i2c_xec_poll_write](16) %s: %s wait_comp error %d for addr send\n",__func__, dev->name, ret);	
			return ret;
		}
	}

	// Send bytes
	for (int i = 0U; i < msg.len; i++) {
		MCHP_I2C_SMB_DATA(ba) = msg.buf[i];
		ret = wait_completion(dev);
		printk("[i2cdri][i2c_xec_poll_write](17) Send bytes wait_completion = 0x%X\n",ret);

		switch (ret) {
		case 0:	// Success 
			break;

		case -EIO:
			LOG_ERR("%s: No Data ACK from Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][i2c_xec_poll_write](18) %s: No Data ACK from Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);
			return ret;

		case -ETIMEDOUT:
			data->timeout_seen = 1;
			LOG_ERR("%s: Clk stretch Timeout - Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][i2c_xec_poll_write](19) %s: Clk stretch Timeout - Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);	
			return ret;

		default:
			data->error_seen = 1;
			LOG_ERR("%s: %s wait_completion error %d for data send",
				__func__, dev->name, ret);

			printk("[i2cdri][i2c_xec_poll_write](20) %s: %s wait_completion error %d for data send\n",__func__, dev->name, ret);		
			return ret;
		}
	}

	// Handle stop bit for last byte to write 
	if (msg.flags & I2C_MSG_STOP) {
		// Send stop and ack bits 
		printk("[i2cdri][i2c_xec_poll_write](21) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));
		MCHP_I2C_SMB_CTRL_WO(ba) =
				MCHP_I2C_SMB_CTRL_PIN |
				MCHP_I2C_SMB_CTRL_ESO |
				MCHP_I2C_SMB_CTRL_STO |
				MCHP_I2C_SMB_CTRL_ACK;
		printk("[i2cdri][i2c_xec_poll_write](22) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));

		data->pending_stop = 0;
	} else {
		data->pending_stop = 1;
	}

	return 0;
}


static int ryan_i2c_xec_poll_read(const struct device *dev, struct i2c_msg msg,
			     uint16_t addr)
{
    printk("[i2cdri][ryan_i2c_xec_poll_read] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	struct i2c_xec_data *data =
		(struct i2c_xec_data *const) (dev->data);
	uint32_t ba = config->base_addr;
	uint8_t byte, ctrl, i2c_timer = 0;
	int ret;	

	uint32_t *I2cControlReg = ((volatile uint32_t *)(uintptr_t)(ba));
	uint32_t *I2cDataReg =  (volatile uint32_t *)((uintptr_t)(ba) + (uintptr_t)(RYAN_I2C_DATA_REGISTER_OFFSET));
	i2c_ControlRegister I2c_ControlReg;
	i2c_DataRegister I2c_DataReg;
	I2c_ControlReg.VALUE = *I2cControlReg;
	I2c_DataReg.VALUE = *I2cDataReg;

	printk("[i2cdri][ryan_i2c_xec_poll_read] base address = 0x%X\n", ba);

	if (data->timeout_seen == 1) {
		/* Wait to see if the slave has released the CLK */
		ret = wait_completion(dev);
		printk("[i2cdri][ryan_i2c_xec_poll_read](01) released CLK wait_completion = 0x%X\n",ret);

		if (ret) {
			data->timeout_seen = 1;
			LOG_ERR("%s: %s wait_completion failure %d\n",
				__func__, dev->name, ret);

			printk("[i2cdri][ryan_i2c_xec_poll_read](02) %s: %s wait_completion failure %d\n",__func__, dev->name, ret);		
			return ret;
		}
		data->timeout_seen = 0;

		/* If we are here, it means the slave has finally released
		 * the CLK. The master needs to end that transaction
		 * gracefully by sending a STOP on the bus.
		 */
		LOG_DBG("%s: %s Force Stop", __func__, dev->name);

		I2c_ControlReg.VALUE = *I2cControlReg;
		printk("[i2cdri][ryan_i2c_xec_poll_read](03) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);
		I2c_ControlReg.bitfld.PIN = 0x01;
		I2c_ControlReg.bitfld.ESO = 0x01;
		I2c_ControlReg.bitfld.STO = 0x01;
		I2c_ControlReg.bitfld.ACK = 0x01;
		*I2cControlReg = I2c_ControlReg.VALUE;
		printk("[i2cdri][ryan_i2c_xec_poll_read](04) *I2cControlReg = 0x%X\n", *I2cControlReg);

		k_busy_wait(BUS_IDLE_US_DFLT);

		printk("[i2cdri][ryan_i2c_xec_poll_read](05) -EBUSY\n");
		return -EBUSY;
	}

	if (!(msg.flags & I2C_MSG_RESTART) || (data->error_seen == 1)) {
		/* Wait till clock and data lines are HIGH */
		while (check_lines_high(dev) == false) {
			if (i2c_timer >= WAIT_LINE_HIGH_COUNT) {
				LOG_DBG("%s: %s not high",
					__func__, dev->name);
				data->error_seen = 1;

				printk("[i2cdri][ryan_i2c_xec_poll_read](06) %s: %s not high\n",__func__, dev->name);
				return -EBUSY;
			}
			k_busy_wait(WAIT_LINE_HIGH_USEC);
			i2c_timer++;
		}

		if (data->error_seen) {
			LOG_DBG("%s: Recovering %s previously in error",
				__func__, dev->name);

			printk("[i2cdri][ryan_i2c_xec_poll_read](07) %s: Recovering %s previously in error\n",__func__, dev->name);	
			data->error_seen = 0;
			recover_from_error(dev);
		}

		/* Wait until bus is free */
		ret = wait_bus_free(dev);
		if (ret) {
			data->error_seen = 1;
			LOG_DBG("%s: %s wait_bus_free failure %d",
				__func__, dev->name, ret);

			printk("[i2cdri][ryan_i2c_xec_poll_read](08) %s: %s wait_bus_free failure %d\n",__func__, dev->name, ret);			
			return ret;
		}
	}

	/* MCHP I2C spec recommends that for repeated start to write to control
	 * register before writing to data register
	 */
	I2c_ControlReg.VALUE = *I2cControlReg;
	printk("[i2cdri][ryan_i2c_xec_poll_read](09) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);
	I2c_ControlReg.bitfld.ESO = 0x01;
	I2c_ControlReg.bitfld.STA = 0x01;
	I2c_ControlReg.bitfld.ACK = 0x01;
	*I2cControlReg = I2c_ControlReg.VALUE;	
	printk("[i2cdri][ryan_i2c_xec_poll_read](10) *I2cControlReg = 0x%X\n", *I2cControlReg);

	/* Send slave address */
	I2c_DataReg.VALUE = (addr | BIT(0));
	printk("[i2cdri][ryan_i2c_xec_poll_read](11) I2c_DataReg.VALUE = 0x%X\n", I2c_DataReg.VALUE);
	*I2cDataReg = I2c_DataReg.VALUE;
	printk("[i2cdri][ryan_i2c_xec_poll_read](12) *I2cDataReg = 0x%X\n", *I2cDataReg);

	ret = wait_completion(dev);
	printk("[i2cdri][ryan_i2c_xec_poll_read](13) Send start and ack bits wait_completion = 0x%X\n",ret);
	
	switch (ret) {
	case 0:	/* Success */
		break;

	case -EIO:
		data->error_seen = 1;
		LOG_WRN("%s: No Addr ACK from Slave 0x%x on %s",
			__func__, addr >> 1, dev->name);

		printk("[i2cdri][ryan_i2c_xec_poll_read](14) %s: No Addr ACK from Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);	
		return ret;

	case -ETIMEDOUT:
		data->previously_in_read = 1;
		data->timeout_seen = 1;
		LOG_ERR("%s: Clk stretch Timeout - Slave 0x%x on %s",
			__func__, addr >> 1, dev->name);

		printk("[i2cdri][ryan_i2c_xec_poll_read](15) %s: Clk stretch Timeout - Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);		
		return ret;

	default:
		data->error_seen = 1;
		LOG_ERR("%s: %s wait_completion error %d for address send",
			__func__, dev->name, ret);

		printk("[i2cdri][ryan_i2c_xec_poll_read](16) %s: %s wait_completion error %d for address send\n",__func__, dev->name, ret);	
		return ret;
	}

	if (msg.len == 1) {
		/* Send NACK for last transaction */
		I2c_ControlReg.VALUE = *I2cControlReg;
		printk("[i2cdri][ryan_i2c_xec_poll_read](17) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);
		I2c_ControlReg.bitfld.ESO = 0x01;
		I2c_ControlReg.bitfld.ENI = 0x01; //Enable Interrupt
		*I2cControlReg = I2c_ControlReg.VALUE;
		printk("[i2cdri][ryan_i2c_xec_poll_read](18) *I2cControlReg = 0x%X\n", *I2cControlReg );
	}

	/* Read dummy byte */
	I2c_DataReg.VALUE = *I2cDataReg;
	byte = I2c_DataReg.VALUE;
	printk("[i2cdri][ryan_i2c_xec_poll_read](19) *I2cDataReg = 0x%X\n", byte);

	for (int i = 0U; i < msg.len; i++) {
		ret = wait_completion(dev);
		printk("[i2cdri][ryan_i2c_xec_poll_read](20) Read dummy byte wait_completion = 0x%X\n",ret);

		switch (ret) {
		case 0:	/* Success */
			break;

		case -EIO:
			LOG_ERR("%s: No Data ACK from Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][ryan_i2c_xec_poll_read](21) %s: No Data ACK from Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);		
			return ret;

		case -ETIMEDOUT:
			data->previously_in_read = 1;
			data->timeout_seen = 1;
			LOG_ERR("%s: Clk stretch Timeout - Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][ryan_i2c_xec_poll_read](22) %s: Clk stretch Timeout - Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);			
			return ret;

		default:
			data->error_seen = 1;
			LOG_ERR("%s: %s wait_completion error %d for data send",
				__func__, dev->name, ret);

			printk("[i2cdri][ryan_i2c_xec_poll_read](23) %s: %s wait_completion error %d for data send\n",__func__, dev->name, ret);	
			return ret;
		}

		if (i == (msg.len - 1)) {
			if (msg.flags & I2C_MSG_STOP) {
				/* Send stop and ack bits */
				I2c_ControlReg.VALUE = *I2cControlReg;
				printk("[i2cdri][ryan_i2c_xec_poll_read](24) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);	
				I2c_ControlReg.bitfld.PIN = 0x01;
				I2c_ControlReg.bitfld.ESO = 0x01;
				//I2c_ControlReg.bitfld.STA = 0;
				I2c_ControlReg.bitfld.STO = 0x01;
				I2c_ControlReg.bitfld.ACK = 0x01;
				*I2cControlReg = I2c_ControlReg.VALUE;
				printk("[i2cdri][ryan_i2c_xec_poll_read](25) *I2cControlReg = 0x%X\n", *I2cControlReg);	
				data->pending_stop = 0;
			}
		} else if (i == (msg.len - 2)) {
			/* Send NACK for last transaction */
			I2c_ControlReg.VALUE = *I2cControlReg;
			printk("[i2cdri][ryan_i2c_xec_poll_read](26) I2c_ControlReg.VALUE = 0x%X\n", I2c_ControlReg.VALUE);
			I2c_ControlReg.bitfld.PIN = 0x01;
			*I2cControlReg = I2c_ControlReg.VALUE;
			printk("[i2cdri][ryan_i2c_xec_poll_read](27) *I2cControlReg = 0x%X\n", *I2cControlReg);
		}

		I2c_DataReg.VALUE = *I2cDataReg;
		msg.buf[i] = I2c_DataReg.VALUE;
		printk("[i2cdri][ryan_i2c_xec_poll_read](28) *I2cDataRe = 0x%X\n", msg.buf[i]);
	}

	return 0;
}

static int i2c_xec_poll_read(const struct device *dev, struct i2c_msg msg,
			     uint16_t addr)
{
    printk("[i2cdri][i2c_xec_poll_read] func entry\n");

	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	struct i2c_xec_data *data =
		(struct i2c_xec_data *const) (dev->data);
	uint32_t ba = config->base_addr;
	uint8_t byte, ctrl, i2c_timer = 0;
	int ret;

	printk("[i2cdri][i2c_xec_poll_read] base address = 0x%X\n", ba);

	if (data->timeout_seen == 1) {
		// Wait to see if the slave has released the CLK 
		ret = wait_completion(dev);
		printk("[i2cdri][i2c_xec_poll_read](01) released CLK wait_completion = 0x%X\n",ret);

		if (ret) {
			data->timeout_seen = 1;
			LOG_ERR("%s: %s wait_completion failure %d\n",
				__func__, dev->name, ret);

			printk("[i2cdri][i2c_xec_poll_read](02) %s: %s wait_completion failure %d\n",__func__, dev->name, ret);	
			return ret;
		}
		data->timeout_seen = 0;

		// If we are here, it means the slave has finally released
		// the CLK. The master needs to end that transaction
		// gracefully by sending a STOP on the bus.
		//
		LOG_DBG("%s: %s Force Stop", __func__, dev->name);

		printk("[i2cdri][i2c_xec_poll_read](03) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));
		MCHP_I2C_SMB_CTRL_WO(ba) =
					MCHP_I2C_SMB_CTRL_PIN |
					MCHP_I2C_SMB_CTRL_ESO |
					MCHP_I2C_SMB_CTRL_STO |
					MCHP_I2C_SMB_CTRL_ACK;
		printk("[i2cdri][i2c_xec_poll_read](04) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));

		k_busy_wait(BUS_IDLE_US_DFLT);

		printk("[i2cdri][i2c_xec_poll_read](05)-EBUSY\n");
		return -EBUSY;
	}

	if (!(msg.flags & I2C_MSG_RESTART) || (data->error_seen == 1)) {
		// Wait till clock and data lines are HIGH
		while (check_lines_high(dev) == false) {
			if (i2c_timer >= WAIT_LINE_HIGH_COUNT) {
				LOG_DBG("%s: %s not high",
					__func__, dev->name);
				data->error_seen = 1;

				printk("[i2cdri][i2c_xec_poll_read](06) %s: %s not high\n",__func__, dev->name);	
				return -EBUSY;
			}
			k_busy_wait(WAIT_LINE_HIGH_USEC);
			i2c_timer++;
		}

		if (data->error_seen) {
			LOG_DBG("%s: Recovering %s previously in error",
				__func__, dev->name);

			printk("[i2cdri][i2c_xec_poll_read](07) %s: Recovering %s previously in error\n",__func__, dev->name);		
			data->error_seen = 0;
			recover_from_error(dev);
		}

		// Wait until bus is free
		ret = wait_bus_free(dev);
		if (ret) {
			data->error_seen = 1;
			LOG_DBG("%s: %s wait_bus_free failure %d",
				__func__, dev->name, ret);

			printk("[i2cdri][i2c_xec_poll_read](08) %s: %s wait_bus_free failure %d\n",__func__, dev->name, ret);			
			return ret;
		}
	}

	// MCHP I2C spec recommends that for repeated start to write to control
	// register before writing to data register
	//
	printk("[i2cdri][i2c_xec_poll_read](09) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));
	MCHP_I2C_SMB_CTRL_WO(ba) = MCHP_I2C_SMB_CTRL_ESO |
		MCHP_I2C_SMB_CTRL_STA | MCHP_I2C_SMB_CTRL_ACK;
	printk("[i2cdri][i2c_xec_poll_read](10) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));

	// Send slave address 
	printk("[i2cdri][i2c_xec_poll_read](11) MCHP_I2C_SMB_DATA(ba) = 0x%X\n", MCHP_I2C_SMB_DATA(ba));
	MCHP_I2C_SMB_DATA(ba) = (addr | BIT(0));
	printk("[i2cdri][i2c_xec_poll_read](12) MCHP_I2C_SMB_DATA(ba) = 0x%X\n", MCHP_I2C_SMB_DATA(ba));

	ret = wait_completion(dev);
	printk("[i2cdri][i2c_xec_poll_read](13) Send start and ack bits wait_completion = 0x%X\n",ret);

	switch (ret) {
	case 0:	// Success 
		break;

	case -EIO:
		data->error_seen = 1;
		LOG_WRN("%s: No Addr ACK from Slave 0x%x on %s",
			__func__, addr >> 1, dev->name);
		printk("[i2cdri][i2c_xec_poll_read](14) %s: No Addr ACK from Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);
		return ret;

	case -ETIMEDOUT:
		data->previously_in_read = 1;
		data->timeout_seen = 1;
		LOG_ERR("%s: Clk stretch Timeout - Slave 0x%x on %s",
			__func__, addr >> 1, dev->name);
		printk("[i2cdri][i2c_xec_poll_read](15) %s: Clk stretch Timeout - Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);	
		return ret;

	default:
		data->error_seen = 1;
		LOG_ERR("%s: %s wait_completion error %d for address send",
			__func__, dev->name, ret);
		printk("[i2cdri][i2c_xec_poll_read](16) %s: %s wait_completion error %d for address send\n",__func__, dev->name, ret);		
		return ret;
	}

	if (msg.len == 1) {
		// Send NACK for last transaction 
		printk("[i2cdri][i2c_xec_poll_read](17) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));
		MCHP_I2C_SMB_CTRL_WO(ba) = MCHP_I2C_SMB_CTRL_ESO;
		printk("[i2cdri][i2c_xec_poll_read](18) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));
	}

	// Read dummy byte 
	byte = MCHP_I2C_SMB_DATA(ba);
	printk("[i2cdri][i2c_xec_poll_read](19) MCHP_I2C_SMB_DATA(ba) = 0x%X\n", byte);

	for (int i = 0U; i < msg.len; i++) {
		ret = wait_completion(dev);
		printk("[i2cdri][i2c_xec_poll_read](20) Read dummy byte wait_completion = 0x%X\n",ret);
		
		switch (ret) {
		case 0:	// Success 
			break;

		case -EIO:
			LOG_ERR("%s: No Data ACK from Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][i2c_xec_poll_read](21) %s: No Data ACK from Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);	
			return ret;

		case -ETIMEDOUT:
			data->previously_in_read = 1;
			data->timeout_seen = 1;
			LOG_ERR("%s: Clk stretch Timeout - Slave 0x%x on %s",
				__func__, addr >> 1, dev->name);

			printk("[i2cdri][i2c_xec_poll_read](22) %s: Clk stretch Timeout - Slave 0x%x on %s\n",__func__, addr >> 1, dev->name);		
			return ret;

		default:
			data->error_seen = 1;
			LOG_ERR("%s: %s wait_completion error %d for data send",
				__func__, dev->name, ret);

			printk("[i2cdri][i2c_xec_poll_read](23) %s: %s wait_completion error %d for data send\n",__func__, dev->name, ret);		
			return ret;
		}

		if (i == (msg.len - 1)) {
			if (msg.flags & I2C_MSG_STOP) {
				// Send stop and ack bits 
				ctrl = (MCHP_I2C_SMB_CTRL_PIN |
					MCHP_I2C_SMB_CTRL_ESO |
					MCHP_I2C_SMB_CTRL_STO |
					MCHP_I2C_SMB_CTRL_ACK);
				printk("[i2cdri][i2c_xec_poll_read](24) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));	
				MCHP_I2C_SMB_CTRL_WO(ba) = ctrl;
				printk("[i2cdri][i2c_xec_poll_read](25) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));	
				data->pending_stop = 0;
			}
		} else if (i == (msg.len - 2)) {
			// Send NACK for last transaction 
			printk("[i2cdri][i2c_xec_poll_read](26) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));
			MCHP_I2C_SMB_CTRL_WO(ba) = MCHP_I2C_SMB_CTRL_ESO;
			printk("[i2cdri][i2c_xec_poll_read](27) MCHP_I2C_SMB_CTRL_WO(ba) = 0x%X\n", MCHP_I2C_SMB_CTRL_WO(ba));
		}

		msg.buf[i] = MCHP_I2C_SMB_DATA(ba);
		printk("[i2cdri][i2c_xec_poll_read](28) MCHP_I2C_SMB_DATA(ba) = 0x%X\n", msg.buf[i]);
	}

	return 0;
}

static int i2c_xec_transfer(const struct device *dev, struct i2c_msg *msgs,
				uint8_t num_msgs, uint16_t addr)
{
	printk("[i2cdri][i2c_xec_transfer] func entry\n");
	int ret = 0;

#ifdef CONFIG_I2C_TARGET
	struct i2c_xec_data *data = dev->data;

	if (data->slave_attached) {
		LOG_ERR("%s Device is registered as slave", dev->name);
		return -EBUSY;
	}
#endif
	const struct i2c_xec_config *cfg = dev->config;
	printk("[i2cdri][i2c_xec_transfer] cfg->girq_bit = 0x%X\n", cfg->girq_bit);
	printk("[i2cdri][i2c_xec_transfer] cfg->girq_id = 0x%X\n", cfg->girq_id);
	printk("[i2cdri][i2c_xec_transfer] DT_INST_IRQN(0x%X) = 0x%X\n",cfg->port_sel, DT_INST_IRQN(0));
	irq_enable(DT_INST_IRQN(RYAN_I2C_CHANNEL)); //irq_enable(DT_INST_IRQN(0));
	struct i2c_xec_data *data = dev->data;
	k_sem_take(&data->device_sync_sem, K_MSEC(100));

	addr <<= 1;
	for (int i = 0U; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			ret = ryan_i2c_xec_poll_write(dev, msgs[i], addr); //ret = i2c_xec_poll_write(dev, msgs[i], addr); //
			if (ret) {
				LOG_ERR("%s Write error: %d", dev->name, ret);
				return ret;
			}
		} else {
			ret = ryan_i2c_xec_poll_read(dev, msgs[i], addr); //ret = i2c_xec_poll_read(dev, msgs[i], addr); //
			if (ret) {
				LOG_ERR("%s Read error: %d", dev->name, ret);
				return ret;
			}
		}
	}

	return 0;
}

static void ryan_i2c_xec_bus_isr(const struct device *dev)
{
	printk("[i2cdri][ryan_i2c_xec_bus_isr] func entry\n");
	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	struct i2c_xec_data *data = dev->data;
	const struct i2c_target_callbacks *slave_cb = data->slave_cfg->callbacks;
	uint32_t ba = config->base_addr;
	printk("[i2cdri][ryan_i2c_xec_bus_isr] base address = 0x%X\n", ba);

	uint32_t status;

	/* Get current status */
	status = MCHP_I2C_SMB_STS_RO(ba);
	printk("[i2cdri][ryan_i2c_xec_bus_isr] Get current status = 0x%X\n", status);

	/* Bus Error */
	if (status & MCHP_I2C_SMB_STS_BER) {
		printk("[i2cdri][ryan_i2c_xec_bus_isr] Bus Error\n");
		if (slave_cb->stop) {
			slave_cb->stop(data->slave_cfg);
		}
		//restart_slave(ba);
		goto clear_iag;
	}

	/* External stop */
	if (status & MCHP_I2C_SMB_STS_EXT_STOP) {
		printk("[i2cdri][ryan_i2c_xec_bus_isr] External stop\n");
		if (slave_cb->stop) {
			slave_cb->stop(data->slave_cfg);
		}
		//dummy = MCHP_I2C_SMB_DATA(ba);
		//restart_slave(ba);
		goto clear_iag;
	}

clear_iag:
	//const struct i2c_xec_config *cfg = dev->config;
	printk("[i2cdri][ryan_i2c_xec_bus_isr] config->girq_bit = 0x%X\n", config->girq_bit);
	printk("[i2cdri][ryan_i2c_xec_bus_isr] config->girq_id = 0x%X\n", config->girq_id);
	printk("[i2cdri][ryan_i2c_xec_bus_isr] DT_INST_IRQN(0x%X) = 0x%X\n",config->port_sel, DT_INST_IRQN(0));
	irq_disable(DT_INST_IRQN(RYAN_I2C_CHANNEL)); //irq_disable(DT_INST_IRQN(0)); //
	k_sem_give(&data->device_sync_sem);

	printk("[i2cdri][ryan_i2c_xec_bus_isr] clear_iag\n");
	MCHP_GIRQ_SRC(config->girq_id) = BIT(config->girq_bit);
}


static void i2c_xec_bus_isr(const struct device *dev)
{
	printk("[i2cdri][i2c_xec_bus_isr] func entry\n");
#ifdef CONFIG_I2C_TARGET
	const struct i2c_xec_config *config =
		(const struct i2c_xec_config *const) (dev->config);
	struct i2c_xec_data *data = dev->data;
	const struct i2c_target_callbacks *slave_cb = data->slave_cfg->callbacks;
	uint32_t ba = config->base_addr;

	uint32_t status;
	uint8_t val;

	uint8_t dummy = 0U;

	if (!data->slave_attached) {
		return;
	}

	/* Get current status */
	status = MCHP_I2C_SMB_STS_RO(ba);

	/* Bus Error */
	if (status & MCHP_I2C_SMB_STS_BER) {
		if (slave_cb->stop) {
			slave_cb->stop(data->slave_cfg);
		}
		restart_slave(ba);
		goto clear_iag;
	}

	/* External stop */
	if (status & MCHP_I2C_SMB_STS_EXT_STOP) {
		if (slave_cb->stop) {
			slave_cb->stop(data->slave_cfg);
		}
		dummy = MCHP_I2C_SMB_DATA(ba);
		restart_slave(ba);
		goto clear_iag;
	}

	/* Address byte handling */
	if (status & MCHP_I2C_SMB_STS_AAS) {
		uint8_t slv_data = MCHP_I2C_SMB_DATA(ba);

		if (!(slv_data & BIT(I2C_READ_WRITE_POS))) {
			/* Slave receive  */
			data->slave_read = false;
			if (slave_cb->write_requested) {
				slave_cb->write_requested(data->slave_cfg);
			}
			goto clear_iag;
		} else {
			/* Slave transmit */
			data->slave_read = true;
			if (slave_cb->read_requested) {
				slave_cb->read_requested(data->slave_cfg, &val);
			}
			MCHP_I2C_SMB_DATA(ba) = val;
			goto clear_iag;
		}
	}

	/* Slave transmit */
	if (data->slave_read) {
		/* Master has Nacked, then just write a dummy byte */
		if (MCHP_I2C_SMB_STS_RO(ba) & MCHP_I2C_SMB_STS_LRB_AD0) {
			MCHP_I2C_SMB_DATA(ba) = dummy;
		} else {
			if (slave_cb->read_processed) {
				slave_cb->read_processed(data->slave_cfg, &val);
			}
			MCHP_I2C_SMB_DATA(ba) = val;
		}
	} else {
		val = MCHP_I2C_SMB_DATA(ba);
		/* TODO NACK Master */
		if (slave_cb->write_received) {
			slave_cb->write_received(data->slave_cfg, val);
		}
	}

clear_iag:
	MCHP_GIRQ_SRC(config->girq_id) = BIT(config->girq_bit);
#endif
}

#ifdef CONFIG_I2C_TARGET
static int i2c_xec_slave_register(const struct device *dev,
				  struct i2c_target_config *config)
{
	printk("[i2cdri][i2c_xec_slave_register] func entry\n");
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;
	uint32_t ba = cfg->base_addr;
	int ret;
	int counter = 0;

	if (!config) {
		return -EINVAL;
	}

	if (data->slave_attached) {
		return -EBUSY;
	}

	/* Wait for any outstanding transactions to complete so that
	 * the bus is free
	 */
	while (!(MCHP_I2C_SMB_STS_RO(ba) & MCHP_I2C_SMB_STS_NBB)) {
		ret = xec_spin_yield(&counter);

		if (ret < 0) {
			return ret;
		}
	}

	data->slave_cfg = config;

	/* Set own address */
	MCHP_I2C_SMB_OWN_ADDR(ba) = data->slave_cfg->address;
	restart_slave(ba);

	data->slave_attached = true;

	/* Clear before enabling girq bit */
	MCHP_GIRQ_SRC(cfg->girq_id) = BIT(cfg->girq_bit);
	MCHP_GIRQ_ENSET(cfg->girq_id) = BIT(cfg->girq_bit);

	return 0;
}

static int i2c_xec_slave_unregister(const struct device *dev,
				    struct i2c_target_config *config)
{
	printk("[i2cdri][i2c_xec_slave_unregister] func entry\n");
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data = dev->data;

	if (!data->slave_attached) {
		return -EINVAL;
	}

	data->slave_attached = false;

	MCHP_GIRQ_ENCLR(cfg->girq_id) = BIT(cfg->girq_bit);

	return 0;
}
#endif

static const struct i2c_driver_api i2c_xec_driver_api = {
	.configure = i2c_xec_configure,
	.transfer = i2c_xec_transfer,
#ifdef CONFIG_I2C_TARGET
	.slave_register = i2c_xec_slave_register, //.target_register //.slave_register
	.slave_unregister = i2c_xec_slave_unregister, //.target_unregister //.slave_unregister
#endif
};

static int i2c_xec_init(const struct device *dev)
{
	printk("[i2cdri][i2c_xec_init] func entry\n");
	const struct i2c_xec_config *cfg = dev->config;
	struct i2c_xec_data *data =
		(struct i2c_xec_data *const) (dev->data);
	int ret;

	data->pending_stop = 0;
	data->slave_attached = false;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("XEC I2C pinctrl setup failed (%d)", ret);
		return ret;
	}

	if (!device_is_ready(cfg->sda_gpio.port)) {
		LOG_ERR("%s GPIO device is not ready for SDA GPIO", dev->name);
		return -ENODEV;
	}

	if (!device_is_ready(cfg->scl_gpio.port)) {
		LOG_ERR("%s GPIO device is not ready for SCL GPIO", dev->name);
		return -ENODEV;
	}

	/* Default configuration */
	ret = i2c_xec_configure(dev,
				I2C_MODE_CONTROLLER |
				I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (ret) {
		LOG_ERR("%s configure failed %d", dev->name, ret);
		return ret;
	}

#ifdef CONFIG_I2C_TARGET
	const struct i2c_xec_config *config =
	(const struct i2c_xec_config *const) (dev->config);

	config->irq_config_func();
#endif

	/*MCHP_GIRQ_BLK_SETEN(cfg->girq_id);
    IRQ_DIRECT_CONNECT(DT_INST_IRQN(0), //IRQ_CONNECT
            DT_INST_IRQ(0, priority),
            ryan_i2c_xec_bus_isr,
            //DEVICE_DT_INST_GET(0),
			0);
    irq_enable(DT_INST_IRQN(0));*/

	printk("[i2cdri][i2c_xec_init] dev->name = %s\n", dev->name);

	printk("[i2cdri][i2c_xec_init] cfg->base_addr = 0x%X\n", cfg->base_addr);
	printk("[i2cdri][i2c_xec_init] cfg->girq_bit = 0x%X\n", cfg->girq_bit);
	printk("[i2cdri][i2c_xec_init] cfg->girq_id = 0x%X\n", cfg->girq_id);
	printk("[i2cdri][i2c_xec_init] cfg->irq_config_func = 0x%x\n", cfg->irq_config_func);

	printk("[i2cdri][i2c_xec_init] cfg->pcfg->state_cnt = 0x%x\n", cfg->pcfg->state_cnt);
	printk("[i2cdri][i2c_xec_init] cfg->pcfg->states = 0x%x\n", cfg->pcfg->states);

	printk("[i2cdri][i2c_xec_init] cfg->port_sel = 0x%x\n", cfg->port_sel);

	printk("[i2cdri][i2c_xec_init] cfg->scl_gpio.dt_flags = 0x%x\n", cfg->scl_gpio.dt_flags);
	printk("[i2cdri][i2c_xec_init] cfg->scl_gpio.pin = 0x%x\n", cfg->scl_gpio.pin);
	printk("[i2cdri][i2c_xec_init] cfg->scl_gpio.port = 0x%x\n", cfg->scl_gpio.port);

	printk("[i2cdri][i2c_xec_init] cfg->sda_gpio.dt_flags = 0x%x\n", cfg->sda_gpio.dt_flags);
	printk("[i2cdri][i2c_xec_init] cfg->sda_gpio.pin = 0x%x\n", cfg->sda_gpio.pin);
	printk("[i2cdri][i2c_xec_init] cfg->sda_gpio.port = 0x%x\n", cfg->sda_gpio.port);

	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);

	const struct i2c_xec_config *config =
	(const struct i2c_xec_config *const) (dev->config);

	config->irq_config_func();

	MCHP_GIRQ_SRC(cfg->girq_id) = BIT(cfg->girq_bit);
	MCHP_GIRQ_ENSET(cfg->girq_id) = BIT(cfg->girq_bit);

	return 0;
}

#define I2C_XEC_DEVICE(n)						\
									\
	PINCTRL_DT_INST_DEFINE(n);					\
									\
	static void i2c_xec_irq_config_func_##n(void);			\
									\
	static struct i2c_xec_data i2c_xec_data_##n;			\
	static const struct i2c_xec_config i2c_xec_config_##n = {	\
		.base_addr =						\
			DT_INST_REG_ADDR(n),				\
		.port_sel = DT_INST_PROP(n, port_sel),			\
		.girq_id = DT_INST_PROP(n, girq),			\
		.girq_bit = DT_INST_PROP(n, girq_bit),			\
		.sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),	\
		.scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),	\
		.irq_config_func = i2c_xec_irq_config_func_##n,		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
	};								\
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_xec_init, NULL,	\
		&i2c_xec_data_##n, &i2c_xec_config_##n,			\
		POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,			\
		&i2c_xec_driver_api);					\
									\
	static void i2c_xec_irq_config_func_##n(void)			\
	{                                                               \
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    ryan_i2c_xec_bus_isr, 				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_XEC_DEVICE)
