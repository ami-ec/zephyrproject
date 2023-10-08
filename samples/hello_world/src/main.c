/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

#define W25QXX_CMD_PROGRAM_ONE_BYTE 0x02
#define W25QXX_CMD_SECTOR_ERASE 0x20
#define W25QXX_CMD_SPI_WRITE_DISABLE 0x04
#define W25QXX_CMD_SPI_WRITE_ENABLE 0x06
#define W25QXX_CMD_READ_MANUFACTURER_DEVICE_ID 0x90
#define W25QXX_CMD_CHIP_ERASE 0xC7

#define W25QXX_CMD_FAST_READ 0x0B

#define W25QXX_CMD_READ_STATUS_REGISTER_1 0x05
#define W25QXX_CMD_READ_STATUS_REGISTER_2 0x35
#define W25QXX_CMD_READ_STATUS_REGISTER_3 0x15

#define W25QXX_CMD_WRITE_STATUS_REGISTER_1 0x01
#define W25QXX_CMD_WRITE_STATUS_REGISTER_2 0x31
#define W25QXX_CMD_WRITE_STATUS_REGISTER_3 0x11

#define W25QXX_CMD_ENTER_QPI_MODE 0x38
#define W25QXX_CMD_EXIT_QPI_MODE 0xFF

#define W25QXX_CMD_INDIVDUAL_BLOCK_SECTOR_LOCK 0x36
#define W25QXX_CMD_INDIVDUAL_BLOCK_SECTOR_UNLOCK 0x39

#define W25QXX_SPI_BUS_BUSY 0x01

#define EC_ROM_SIZE 0x80000 //0x10000
#define SECTOR_ERASE_SIZE 0x1000
#define ONE_PAGE_PROGRAM_SIZE 0x100
#define START_ADDRESS 0x10000

typedef union
{
	uint8_t	AddressU8[3];
	uint32_t	AddressU32;
} SPI_ADDRESS;

static int wait_spi_free(const struct device *spi, 
						struct spi_config *spi_cfg
						)
{
	printk("[app][wait_spi_free] function entry\n");
	static uint8_t tx_buffer[1] = {W25QXX_CMD_READ_STATUS_REGISTER_1}; //0x05
	static uint8_t rx_buffer[1];
	uint8_t temp;
	int status;

	for(int i=0; i<sizeof(tx_buffer); i++)
	{
     	printk("[app][wait_spi_free] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};

	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	do
	{
		status = spi_transceive(spi, spi_cfg, &tx, &rx);
		printk("[app][wait_spi_free] spi_transceive() status = 0x%02X\n", status);
		if (status) {
			printk("[app][wait_spi_free] could not spi transceive\n");
			return status;
		}

		for(int i=0; i<sizeof(rx_buffer); i++)
		{
     		printk("[app][wait_spi_free] rx_buffer[0x%02X] = 0x%02X\n",i ,rx_buffer[i]);
   		}

		temp = rx_buffer[1] & W25QXX_SPI_BUS_BUSY;
	} while(temp);

	return status;
}

static int spi_write_disable(const struct device *spi, 
							struct spi_config *spi_cfg
							)
{
	printk("[app][spi_write_disable] function entry\n");
	int status;
	static uint8_t tx_buffer[1] = {W25QXX_CMD_SPI_WRITE_DISABLE}; //0x04

	printk("[app][spi_write_disable] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
	for(int i=0; i <sizeof(tx_buffer); i++)
    {
     	printk("[app][spi_write_disable] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	status = wait_spi_free(spi, spi_cfg);
	printk("[app][spi_write_disable] wait_spi_free() status = 0x%02X\n", status);

	return spi_write(spi, spi_cfg, &tx);
}

static int spi_write_enable(const struct device *spi, 
							struct spi_config *spi_cfg
							)
{
	printk("[app][spi_write_enable] function entry\n");
	int status;
	static uint8_t tx_buffer[1] = {W25QXX_CMD_SPI_WRITE_ENABLE}; //0x06

	printk("[app][spi_write_enable] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
	for(int i=0; i <sizeof(tx_buffer); i++)
    {
     	printk("[app][spi_write_enable] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	status = wait_spi_free(spi, spi_cfg);
	printk("[app][spi_write_enable] wait_spi_free() status = 0x%02X\n", status);	

	return spi_write(spi, spi_cfg, &tx);
}

static int spi_individual_block_sector_unlock(const struct device *spi, 
							struct spi_config *spi_cfg, 
							uint32_t StartLockAddress, 
							uint32_t SectorLockSize, 
							uint32_t EcRomSize
							)
{
	printk("[app][spi_individual_block_sector_unlock] function entry\n");		
	static uint8_t tx_buffer[4];
	SPI_ADDRESS SpiAddress={0};
	int status;

	for(
		SpiAddress.AddressU32 = StartLockAddress;
		SpiAddress.AddressU32 < StartLockAddress + EcRomSize;
		SpiAddress.AddressU32 += SectorLockSize
	)
	{
		printk("[app][spi_individual_block_sector_unlock] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		tx_buffer[0] = W25QXX_CMD_INDIVDUAL_BLOCK_SECTOR_UNLOCK; //0x39
		tx_buffer[1] = SpiAddress.AddressU8[2];
		tx_buffer[2] = SpiAddress.AddressU8[1];
		tx_buffer[3] = SpiAddress.AddressU8[0];

		printk("[app][spi_individual_block_sector_unlock] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
    	for(int i=0; i <sizeof(tx_buffer); i++)
    	{
     	   printk("[app][spi_individual_block_sector_unlock] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   		}
	
		const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1
		};

		status = spi_write_enable(spi, spi_cfg);
		printk("[app][spi_individual_block_sector_unlock] spi_write_enable() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_individual_block_sector_unlock] could not spi write enable\n");
			return status;
		}

		status = wait_spi_free(spi, spi_cfg);
		printk("[app][spi_individual_block_sector_unlock] 1 wait_spi_free() status = 0x%02X\n", status);

		status = spi_write(spi, spi_cfg, &tx);
		printk("[app][spi_individual_block_sector_unlock] spi_write() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_individual_block_sector_unlock] could not spi_write()\n");
			return status;
		}

		status = wait_spi_free(spi, spi_cfg);
		printk("[app][spi_individual_block_sector_unlock] 2 wait_spi_free() status = 0x%02X\n", status);

		status = spi_write_disable(spi, spi_cfg);
		printk("[app][spi_individual_block_sector_unlock] spi_write_disable() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_individual_block_sector_unlock] could not spi write disable\n");
			return status;
		}
	}

	return status;
}

static int spi_individual_block_sector_lock(const struct device *spi, 
							struct spi_config *spi_cfg, 
							uint32_t StartLockAddress, 
							uint32_t SectorLockSize, 
							uint32_t EcRomSize
							)
{
	printk("[app][spi_individual_block_sector_lock] function entry\n");		
	static uint8_t tx_buffer[4];
	SPI_ADDRESS SpiAddress={0};
	int status;

	for(
		SpiAddress.AddressU32 = StartLockAddress;
		SpiAddress.AddressU32 < StartLockAddress + EcRomSize;
		SpiAddress.AddressU32 += SectorLockSize
	)
	{
		printk("[app][spi_individual_block_sector_lock] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		tx_buffer[0] = W25QXX_CMD_INDIVDUAL_BLOCK_SECTOR_LOCK; //0x36
		tx_buffer[1] = SpiAddress.AddressU8[2];
		tx_buffer[2] = SpiAddress.AddressU8[1];
		tx_buffer[3] = SpiAddress.AddressU8[0];

		printk("[app][spi_individual_block_sector_lock] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
    	for(int i=0; i <sizeof(tx_buffer); i++)
    	{
     	   printk("[app][spi_individual_block_sector_lock] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   		}
	
		const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1
		};

		status = spi_write_enable(spi, spi_cfg);
		printk("[app][spi_individual_block_sector_lock] spi_write_enable() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_individual_block_sector_lock] could not spi write enable\n");
			return status;
		}

		status = wait_spi_free(spi, spi_cfg);
		printk("[app][spi_individual_block_sector_lock] 1 wait_spi_free() status = 0x%02X\n", status);

		status = spi_write(spi, spi_cfg, &tx);
		printk("[app][spi_individual_block_sector_lock] spi_write() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_individual_block_sector_lock] could not spi_write()\n");
			return status;
		}

		status = wait_spi_free(spi, spi_cfg);
		printk("[app][spi_individual_block_sector_lock] 2 wait_spi_free() status = 0x%02X\n", status);

		status = spi_write_disable(spi, spi_cfg);
		printk("[app][spi_individual_block_sector_lock] spi_write_disable() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_individual_block_sector_lock] could not spi write disable\n");
			return status;
		}
	}

	return status;
}

static int spi_disable_qpi_mode(const struct device *spi, 
							struct spi_config *spi_cfg
							)
{
	printk("[app][spi_disable_qpi_mode] function entry\n");
	static uint8_t tx_buffer[1] = {W25QXX_CMD_EXIT_QPI_MODE}; //0xFF

	printk("[app][spi_disable_qpi_mode] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
	for(int i=0; i <sizeof(tx_buffer); i++)
    {
     	printk("[app][spi_disable_qpi_mode] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	return spi_write(spi, spi_cfg, &tx);
}

static int spi_enable_qpi_mode(const struct device *spi, 
							struct spi_config *spi_cfg
							)
{
	printk("[app][spi_enable_qpi_mode] function entry\n");
	static uint8_t tx_buffer[1] = {W25QXX_CMD_ENTER_QPI_MODE}; //0x38

	printk("[app][spi_enable_qpi_mode] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
	for(int i=0; i <sizeof(tx_buffer); i++)
    {
     	printk("[app][spi_enable_qpi_mode] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	return spi_write(spi, spi_cfg, &tx);
}

static int spi_write_status_register_3(const struct device *spi, 
										struct spi_config *spi_cfg,
										uint8_t data)
{
	printk("[app][spi_write_status_register_3] function entry\n");
	static uint8_t tx_buffer[2];

	tx_buffer[0] = W25QXX_CMD_WRITE_STATUS_REGISTER_3; //0x11
	tx_buffer[1] = data;

	printk("[app][spi_write_status_register_3] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
	for(int i=0; i <sizeof(tx_buffer); i++)
    {
     	printk("[app][spi_write_status_register_3] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 2
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	return spi_write(spi, spi_cfg, &tx);
}

static int spi_write_status_register_2(const struct device *spi, 
										struct spi_config *spi_cfg,
										uint8_t data)
{
	printk("[app][spi_write_status_register_2] function entry\n");
	static uint8_t tx_buffer[2];

	tx_buffer[0] = W25QXX_CMD_WRITE_STATUS_REGISTER_2; //0x31
	tx_buffer[1] = data;

	printk("[app][spi_write_status_register_2] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
	for(int i=0; i <sizeof(tx_buffer); i++)
    {
     	printk("[app][spi_write_status_register_2] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 2
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	return spi_write(spi, spi_cfg, &tx);
}

static int spi_write_status_register_1(const struct device *spi, 
										struct spi_config *spi_cfg,
										uint8_t data)
{
	printk("[app][spi_write_status_register_1] function entry\n");
	static uint8_t tx_buffer[2];

	tx_buffer[0] = W25QXX_CMD_WRITE_STATUS_REGISTER_1; //0x01
	tx_buffer[1] = data;

	printk("[app][spi_write_status_register_1] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
	for(int i=0; i <sizeof(tx_buffer); i++)
    {
     	printk("[app][spi_write_status_register_1] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 2
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	return spi_write(spi, spi_cfg, &tx);
}

static int spi_read_status_register_3(const struct device *spi, 
							struct spi_config *spi_cfg
							)
{
	printk("[app][spi_read_status_register_3] function entry\n");
	static uint8_t tx_buffer[1] = {W25QXX_CMD_READ_STATUS_REGISTER_3}; //0x15
	int status;

	for(int i=0; i<sizeof(tx_buffer); i++)
	{
     	printk("[app][spi_read_status_register_3] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	static uint8_t rx_buffer[1];

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};

	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	status = spi_transceive(spi, spi_cfg, &tx, &rx);
	printk("[app][spi_read_status_register_3] spi_transceive() status = 0x%02X\n", status);
	if (status) {
		printk("[app][spi_read_status_register_3] could not spi transceive\n");
		return status;
	}

	for(int i=0; i<sizeof(rx_buffer); i++)
	{
     	printk("[app][spi_read_status_register_3] rx_buffer[0x%02X] = 0x%02X\n",i ,rx_buffer[i]);
   	}

	return status;
}

static int spi_read_status_register_2(const struct device *spi, 
							struct spi_config *spi_cfg
							)
{
	printk("[app][spi_read_status_register_2] function entry\n");
	static uint8_t tx_buffer[1] = {W25QXX_CMD_READ_STATUS_REGISTER_2}; //0x35
	int status;

	for(int i=0; i<sizeof(tx_buffer); i++)
	{
     	printk("[app][spi_read_status_register_2] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	static uint8_t rx_buffer[1];

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};

	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	status = spi_transceive(spi, spi_cfg, &tx, &rx);
	printk("[app][spi_read_status_register_2] spi_transceive() status = 0x%02X\n", status);
	if (status) {
		printk("[app][spi_read_status_register_2] could not spi transceive\n");
		return status;
	}

	for(int i=0; i<sizeof(rx_buffer); i++)
	{
     	printk("[app][spi_read_status_register_2] rx_buffer[0x%02X] = 0x%02X\n",i ,rx_buffer[i]);
   	}

	return status;
}

static int spi_read_status_register_1(const struct device *spi, 
							struct spi_config *spi_cfg
							)
{
	printk("[app][spi_read_status_register_1] function entry\n");

	static uint8_t tx_buffer[1] = {W25QXX_CMD_READ_STATUS_REGISTER_1}; //0x05
	int status;

	for(int i=0; i<sizeof(tx_buffer); i++)
	{
     	printk("[app][spi_read_status_register_1] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	static uint8_t rx_buffer[1];

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};

	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	status = spi_transceive(spi, spi_cfg, &tx, &rx);
	printk("[app][spi_read_status_register_1] spi_transceive() status = 0x%02X\n", status);
	if (status) {
		printk("[app][spi_read_status_register_1] Could not spi_transceive\n");
		return status;
	}

	for(int i=0; i<sizeof(rx_buffer); i++)
	{
     	printk("[app][spi_read_status_register_1] rx_buffer[0x%02X] = 0x%02X\n",i ,rx_buffer[i]);
   	}

	return status;
}

static int spi_read_data(const struct device *spi, 
						struct spi_config *spi_cfg,
						uint32_t StartWriteAddress, 
						uint8_t *ReadDataBuffter, 
						uint32_t ReadDataSize
						)
{
	printk("[app][spi_read_data] function entry\n");
	
	static uint8_t tx_buffer[5];
	SPI_ADDRESS SpiAddress={0};
	int status = 0;

	for(
		SpiAddress.AddressU32 = StartWriteAddress;
		SpiAddress.AddressU32 < StartWriteAddress + ReadDataSize;
		SpiAddress.AddressU32 += ONE_PAGE_PROGRAM_SIZE
	)
	{
		printk("[app][spi_read_data] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		tx_buffer[0] = W25QXX_CMD_FAST_READ; //0x0B
		tx_buffer[1] = SpiAddress.AddressU8[2];
		tx_buffer[2] = SpiAddress.AddressU8[1];
		tx_buffer[3] = SpiAddress.AddressU8[0];
		tx_buffer[4] = 0; //Dummy

    	for(int i=0; i<sizeof(tx_buffer); i++)
    	{
     	   printk("[app][spi_read_data] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   		}
	
		const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1
		};

		//static uint8_t rx_buffer[ONE_PAGE_PROGRAM_SIZE];

		struct spi_buf rx_buf = {
			.buf = ReadDataBuffter, //rx_buffer,
			.len = ReadDataSize, //sizeof(rx_buffer),
		};

		const struct spi_buf_set rx = {
			.buffers = &rx_buf,
			.count = 1
		};

		status = spi_transceive(spi, spi_cfg, &tx, &rx);
		printk("[app][spi_read_data] spi_transceive() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_read_data] could not spi transceive\n");
			return status;
		}

		/*printk("[app][spi_read_data] rx_buffer:");
		for(int i=0; i <ReadDataSize; i++)
    	{
			if(i % 16 == 0)
			{
				printk("\n");
			}

			printk("0x%02X ", ReadDataBuffter[i]);
   		}
		printk("\n");*/
	}

	return status;
}

static int spi_write_data(const struct device *spi, 
						struct spi_config *spi_cfg,
						uint32_t StartWriteAddress, 
						uint8_t *WriteDataBuffter, 
						uint32_t WriteDataSize
						)
{
	printk("[app][spi_write_data] function entry\n");
	static uint8_t tx_buffer[ONE_PAGE_PROGRAM_SIZE * 10]; //tx_buffer[4 + ONE_PAGE_PROGRAM_SIZE];
	SPI_ADDRESS SpiAddress={0};
	int status;

	for(
		SpiAddress.AddressU32 = StartWriteAddress;
		SpiAddress.AddressU32 < StartWriteAddress + WriteDataSize;
		SpiAddress.AddressU32 += ONE_PAGE_PROGRAM_SIZE
	)
	{
		printk("[app][spi_write_data] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		tx_buffer[0] = W25QXX_CMD_PROGRAM_ONE_BYTE; //0x02
		tx_buffer[1] = SpiAddress.AddressU8[2];
		tx_buffer[2] = SpiAddress.AddressU8[1];
		tx_buffer[3] = SpiAddress.AddressU8[0];

		for(int i=4,x=0; i<(4 + WriteDataSize); i++,x++)
		{
			tx_buffer[i] = WriteDataBuffter[x];
		}

    	for(int i=0; i<(4 + WriteDataSize); i++)
    	{
     	   printk("[app][spi_write_data] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   		}
	
		const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = (4 + WriteDataSize)
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1
		};

		status = spi_write_enable(spi, spi_cfg);
		printk("[app][spi_write_data] spi_write_enable() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_write_data] could not spi write enable\n");
			return status;
		}

		status = wait_spi_free(spi, spi_cfg);
		printk("[app][spi_write_data] 1 wait_spi_free() status = 0x%02X\n", status);

		status = spi_write(spi, spi_cfg, &tx);
		printk("[app][spi_write_data] spi_write() status = 0x%02X\n", status);
		if (status){
			printk("[app][spi_write_data] could not spi_write()\n");
			return status;
		}

		status = wait_spi_free(spi, spi_cfg);
		printk("[app][spi_write_data] 2 wait_spi_free() status = 0x%02X\n", status);

		status = spi_write_disable(spi, spi_cfg);
		printk("[app][spi_write_data] spi_write_disable() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_write_data] could not spi write disable\n");
			return status;
		}
	}

	return status;
}

static int spi_chip_erase(const struct device *spi, 
						struct spi_config *spi_cfg
						)
{
	printk("[app][spi_chip_erase] function entry\n");
	static uint8_t tx_buffer[1] = {W25QXX_CMD_CHIP_ERASE}; //0xC7

	printk("[app][spi_chip_erase] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
	for(int i=0; i <sizeof(tx_buffer); i++)
    {
     	printk("[app][spi_chip_erase] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}
	
	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	return spi_write(spi, spi_cfg, &tx);
}

static int spi_sector_erase(const struct device *spi, 
							struct spi_config *spi_cfg, 
							uint32_t StartEraseAddress, 
							uint32_t SectorEraseSize, 
							uint32_t EcRomSize
							)
{
	printk("[app][spi_sector_erase] function entry\n");		
	static uint8_t tx_buffer[4];
	SPI_ADDRESS SpiAddress={0};
	int status;

	if (SectorEraseSize != SECTOR_ERASE_SIZE)
	{
		printk("[app][spi_sector_erase] SectorEraseSize != SECTOR_ERASE_SIZE\n");		
		return -EPERM;
	}

	for(
		SpiAddress.AddressU32 = StartEraseAddress;
		SpiAddress.AddressU32 < StartEraseAddress + EcRomSize;
		SpiAddress.AddressU32 += SectorEraseSize
	)
	{
		printk("[app][spi_sector_erase] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		tx_buffer[0] = W25QXX_CMD_SECTOR_ERASE; //0x20
		tx_buffer[1] = SpiAddress.AddressU8[2];
		tx_buffer[2] = SpiAddress.AddressU8[1];
		tx_buffer[3] = SpiAddress.AddressU8[0];

		printk("[app][spi_sector_erase] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
    	for(int i=0; i <sizeof(tx_buffer); i++)
    	{
     	   printk("[app][spi_sector_erase] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   		}
	
		const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
		};
		const struct spi_buf_set tx = {
			.buffers = &tx_buf,
			.count = 1
		};

		status = spi_write_enable(spi, spi_cfg);
		printk("[app][spi_sector_erase] spi_write_enable() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_sector_erase] could not spi write enable\n");
			return status;
		}

		status = wait_spi_free(spi, spi_cfg);
		printk("[app][spi_sector_erase] 1 wait_spi_free() status = 0x%02X\n", status);

		status = spi_write(spi, spi_cfg, &tx);
		printk("[app][spi_sector_erase] spi_write() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_sector_erase] could not spi_write()\n");
			return status;
		}

		status = wait_spi_free(spi, spi_cfg);
		printk("[app][spi_sector_erase] 2 wait_spi_free() status = 0x%02X\n", status);

		status = spi_write_disable(spi, spi_cfg);
		printk("[app][spi_sector_erase] spi_write_disable() status = 0x%02X\n", status);
		if (status) {
			printk("[app][spi_sector_erase] could not spi write disable\n");
			return status;
		}
	}

	return status;
}

static int spi_manufacturer_device_id(const struct device *spi, 
									struct spi_config *spi_cfg
									)
{
	printk("[app][spi_manufacturer_device_id] function entry\n");
	static uint8_t tx_buffer[4];
	uint32_t AddressU32 = 0;
	int status;

	tx_buffer[0] = W25QXX_CMD_READ_MANUFACTURER_DEVICE_ID; //0x90
	tx_buffer[1] = (AddressU32 >> 16) & 0xFF;
	tx_buffer[2] = (AddressU32 >> 8) & 0xFF;
	tx_buffer[3] = AddressU32 & 0xFF;

	printk("[app][spi_manufacturer_device_id] sizeof(tx_buffer) = 0x%02X\n", sizeof(tx_buffer));
	for(int i=0; i <sizeof(tx_buffer); i++)
    {
     	printk("[app][spi_manufacturer_device_id] tx_buffer[0x%02X] = 0x%02X\n",i ,tx_buffer[i]);
   	}

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
		};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
		};

	static uint8_t rx_buffer[2];
	
	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};

	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	status = spi_transceive(spi, spi_cfg, &tx, &rx);
	printk("[app][spi_manufacturer_device_id] spi_transceive() status = 0x%02X\n", status);
	if (status) {
		printk("[app][spi_manufacturer_device_id] could not spi transceive\n");
		return status;
	}
	
	printk("[app][spi_manufacturer_device_id] manufacturer id = 0x%02X\n", rx_buffer[0]);
	printk("[app][spi_manufacturer_device_id] devicr id = 0x%02X\n", rx_buffer[1]);

	return status;
}

static int block_sector_unlock(const struct device *spi,
							struct spi_config *spi_cfg
							)
{
	printk("[app][block_sector_unlock] function entry\n");
	uint32_t StartAddress = START_ADDRESS; //0x10000
	uint32_t EraseSize = SECTOR_ERASE_SIZE;
	int status;

	status = spi_individual_block_sector_unlock(spi, spi_cfg, StartAddress, SECTOR_ERASE_SIZE, EraseSize);
	printk("[app][block_sector_unlock] spi_individual_block_sector_unlock() status = 0x%02X\n", status);
	if (status) {
		printk("[app][block_sector_unlock] could not sector erase\n");
		return status;
	}

	return status;
}

static int block_sector_lock(const struct device *spi,
							struct spi_config *spi_cfg
							)
{
	printk("[app][block_sector_lock] function entry\n");
	uint32_t StartAddress = START_ADDRESS; //0x10000
	uint32_t EraseSize = SECTOR_ERASE_SIZE;
	int status;

	status = spi_individual_block_sector_lock(spi, spi_cfg, StartAddress, SECTOR_ERASE_SIZE, EraseSize);
	printk("[app][block_sector_lock] spi_individual_block_sector_lock() status = 0x%02X\n", status);
	if (status) {
		printk("[app][block_sector_lock] could not sector erase\n");
		return status;
	}

	return status;
}

static int disable_qe(const struct device *spi,
					struct spi_config *spi_cfg
					)
{
	printk("[app][disable_qe] function entry\n");	
	int status;

	status = spi_disable_qpi_mode(spi, spi_cfg);
	printk("[app][disable_qe] spi_disable_qpi_mode() status = 0x%02X\n", status);
	if (status) {
		printk("[app][disable_qe] could not disable qpi mode\n");
		return status;
	}

	return status;
}

static int enable_qe(const struct device *spi,
					struct spi_config *spi_cfg
					)
{
	printk("[app][enable_qe] function entry\n");
	int status;

	status = spi_enable_qpi_mode(spi, spi_cfg);
	printk("[app][enable_qe] spi_enable_qpi_mode() status = 0x%02X\n", status);
	if (status) {
		printk("[app][enable_qe] could not enable qpi mode\n");
		return status;
	}

	return status;
}

static int write_260_bytes(const struct device *spi,
			     			struct spi_config *spi_cfg)
{
	printk("[app][write_260_bytes] function entry\n");
	uint32_t StartAddress = START_ADDRESS; //0x10000
	SPI_ADDRESS SpiAddress={0};
	static uint8_t write_data_buffer[260];
	static uint8_t read_data_buffer[260];
	int status, num = 1;

	for(int i=0; i <sizeof(write_data_buffer); i++)
    {
		if(num > 0xFF)
		{
			num = 1;
		}

		write_data_buffer[i] = num++;
    	printk("[app][write_260_bytes] write_data_buffer[0x%02X] = 0x%02X\n",i ,write_data_buffer[i]);
   	}

	status = spi_sector_erase(spi, spi_cfg, StartAddress, SECTOR_ERASE_SIZE, sizeof(write_data_buffer));
	printk("[app][write_260_bytes] spi_sector_erase() status = 0x%02X\n", status);
	if (status) {
		printk("[app][write_260_bytes] could not sector erase\n");
		return status;
	}

	printk("[app][write_260_bytes] verify erase\n");
	for(
		SpiAddress.AddressU32 = StartAddress;
		SpiAddress.AddressU32 < StartAddress + sizeof(read_data_buffer);
		SpiAddress.AddressU32 += ONE_PAGE_PROGRAM_SIZE
	)
	{
		printk("[app][write_260_bytes] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		status = spi_read_data(spi, spi_cfg, SpiAddress.AddressU32, &read_data_buffer, sizeof(read_data_buffer));
		printk("[app][write_260_bytes] spi_read_data() status = 0x%02X\n", status);
		if (status) {
			printk("[app][write_260_bytes] could not write data\n");
			return status;
		}

		printk("[app][write_260_bytes] read_data_buffer:");
		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			if(i % 16 == 0)
			{
				printk("\n");
			}

			printk("0x%02X ", read_data_buffer[i]);

			if(read_data_buffer[i] != 0xFF)
			{
				printk("\n");
				printk("[app][write_260_bytes] read_data_buffer[0x%02X] != 0xFF\n",i);
				return -EPERM;
			}
   		}
		printk("\n");

		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			read_data_buffer[i] = 0x00;
   		}
	}

	for(int i=0; i <sizeof(write_data_buffer); i++)
    {
		write_data_buffer[i] = i + 1;
    	printk("[app][write_260_bytes] write_data_buffer[0x%02X] = 0x%02X\n",i ,write_data_buffer[i]);
   	}

	status = spi_write_data(spi, spi_cfg, StartAddress, &write_data_buffer, sizeof(write_data_buffer));
	printk("[app][write_260_bytes] spi_write_data() status = 0x%02X\n", status);
	if (status) {
		printk("[app][write_260_bytes] could not write data\n");
		return status;
	}

	printk("[app][write_260_bytes] verify write data\n");
	for(
		SpiAddress.AddressU32 = StartAddress;
		SpiAddress.AddressU32 < StartAddress + sizeof(read_data_buffer);
		SpiAddress.AddressU32 += ONE_PAGE_PROGRAM_SIZE
	)
	{
		printk("[app][write_260_bytes] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		status = spi_read_data(spi, spi_cfg, SpiAddress.AddressU32, &read_data_buffer, sizeof(read_data_buffer));
		printk("[app][write_260_bytes] spi_read_data() status = 0x%02X\n", status);
		if (status) {
			printk("[app][write_260_bytes] could not write data\n");
			return status;
		}

		printk("[app][write_260_bytes] read_data_buffer:");
		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			if(i % 16 == 0)
			{
				printk("\n");
			}

			printk("0x%02X ", read_data_buffer[i]);
   		}
		printk("\n");

		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			read_data_buffer[i] = 0x00;
   		}
	}

	return status;
}

static int write_4_bytes(const struct device *spi,
						struct spi_config *spi_cfg)
{
	printk("[app][write_4_bytes] function entry\n");
	uint32_t StartAddress = START_ADDRESS; //0x10000
	SPI_ADDRESS SpiAddress={0};
	static uint8_t write_data_buffer[4];
	static uint8_t read_data_buffer[4];
	int status;

	status = spi_sector_erase(spi, spi_cfg, StartAddress, SECTOR_ERASE_SIZE, sizeof(write_data_buffer));
	printk("[app][write_4_bytes] spi_sector_erase() status = 0x%02X\n", status);
	if (status) {
		printk("[app][write_4_bytes] could not sector erase\n");
		return status;
	}

	printk("[app][write_4_bytes] verify erase\n");
	for(
		SpiAddress.AddressU32 = StartAddress;
		SpiAddress.AddressU32 < StartAddress + sizeof(read_data_buffer);
		SpiAddress.AddressU32 += ONE_PAGE_PROGRAM_SIZE
	)
	{
		printk("[app][write_4_bytes] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		status = spi_read_data(spi, spi_cfg, SpiAddress.AddressU32, &read_data_buffer, sizeof(read_data_buffer));
		printk("[app][write_4_bytes] spi_read_data() status = 0x%02X\n", status);
		if (status) {
			printk("[app][write_4_bytes] could not write data\n");
			return status;
		}

		printk("[app][write_4_bytes] read_data_buffer:");
		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			if(i % 16 == 0)
			{
				printk("\n");
			}

			printk("0x%02X ", read_data_buffer[i]);

			if(read_data_buffer[i] != 0xFF)
			{
				printk("\n");
				printk("[app][write_4_bytes] read_data_buffer[0x%02X] != 0xFF\n",i);
				return -EPERM;
			}
   		}
		printk("\n");

		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			read_data_buffer[i] = 0x00;
   		}
	}

	for(int i=0; i <sizeof(write_data_buffer); i++)
    {
		write_data_buffer[i] = i + 1;
    	printk("[app][write_4_bytes] write_data_buffer[0x%02X] = 0x%02X\n",i ,write_data_buffer[i]);
   	}

	status = spi_write_data(spi, spi_cfg, StartAddress, &write_data_buffer, sizeof(write_data_buffer));
	printk("[app][write_4_bytes] spi_write_data() status = 0x%02X\n", status);
	if (status) {
		printk("[app][write_4_bytes] could not write data\n");
		return status;
	}

	printk("[app][write_4_bytes] verify write data\n");
	for(
		SpiAddress.AddressU32 = StartAddress;
		SpiAddress.AddressU32 < StartAddress + sizeof(read_data_buffer);
		SpiAddress.AddressU32 += ONE_PAGE_PROGRAM_SIZE
	)
	{
		printk("[app][write_4_bytes] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		status = spi_read_data(spi, spi_cfg, SpiAddress.AddressU32, &read_data_buffer, sizeof(read_data_buffer));
		printk("[app][write_4_bytes] spi_read_data() status = 0x%02X\n", status);
		if (status) {
			printk("[app][write_4_bytes] could not write data\n");
			return status;
		}

		printk("[app][write_4_bytes] read_data_buffer:");
		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			if(i % 16 == 0)
			{
				printk("\n");
			}

			printk("0x%02X ", read_data_buffer[i]);
   		}
		printk("\n");

		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			read_data_buffer[i] = 0x00;
   		}
	}

	return status;
}

static int chip_erase(const struct device *spi,
					struct spi_config *spi_cfg
					)
{
	printk("[app][chip_erase] function entry\n");
	uint32_t StartAddress = 0;
	SPI_ADDRESS SpiAddress={0};
	static uint8_t read_data_buffer[ONE_PAGE_PROGRAM_SIZE];
	int status;

	status = spi_write_enable(spi, spi_cfg);
	printk("[app][chip_erase] spi_write_enable() status = 0x%02X\n", status);
	if (status) {
		printk("[app][chip_erase] could not spi write enable\n");
		return status;
	}

	status = wait_spi_free(spi, spi_cfg);
	printk("[app][chip_erase] 1 wait_spi_free() status = 0x%02X\n", status);

	status = spi_chip_erase(spi, spi_cfg);
	printk("[app][chip_erase] spi_chip_erase() status = 0x%02X\n", status);
	if (status) {
		printk("[app][chip_erase] could not spi chip_erase\n");
		return status;
	}

	printk("[app][chip_erase] k_sleep\n");
	k_sleep(K_MSEC(60 * 1000));

	status = wait_spi_free(spi, spi_cfg);
	printk("[app][chip_erase] 2 wait_spi_free() status = 0x%02X\n", status);

	status = spi_write_disable(spi, spi_cfg);
	printk("[app][chip_erase] spi_write_disable() status = 0x%02X\n", status);
	if (status) {
		printk("[app][chip_erase] could not spi write disable\n");
		return status;
	}

	printk("[app][chip_erase] verify erase\n");
	for(
		SpiAddress.AddressU32 = StartAddress;
		SpiAddress.AddressU32 < EC_ROM_SIZE;
		SpiAddress.AddressU32 += ONE_PAGE_PROGRAM_SIZE
	)
	{
		printk("[app][chip_erase] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		status = spi_read_data(spi, spi_cfg, SpiAddress.AddressU32, &read_data_buffer, sizeof(read_data_buffer));
		printk("[app][chip_erase] spi_read_data() status = 0x%02X\n", status);
		if (status) {
			printk("[app][chip_erase] could not write data\n");
			return status;
		}

		printk("[app][chip_erase] read_data_buffer:");
		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			if(i % 16 == 0)
			{
				printk("\n");
			}

			printk("0x%02X ", read_data_buffer[i]);

			if(read_data_buffer[i] != 0xFF)
			{
				printk("\n");
				printk("[app][chip_erase] read_data_buffer[0x%02X] != 0xFF\n",i);
				return -EPERM;
			}
   		}
		printk("\n");

		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			read_data_buffer[i] = 0x00;
   		}
	}

	return status;
}

static int sector_erase(const struct device *spi,
					struct spi_config *spi_cfg
					)
{
	printk("[app][sector_erase] function entry\n");
	uint32_t StartAddress = START_ADDRESS; //0x10000
	uint32_t EraseSize = SECTOR_ERASE_SIZE;
	SPI_ADDRESS SpiAddress={0};
	static uint8_t read_data_buffer[ONE_PAGE_PROGRAM_SIZE];
	int status;

	status = spi_sector_erase(spi, spi_cfg, StartAddress, SECTOR_ERASE_SIZE, EraseSize);
	printk("[app][sector_erase] spi_sector_erase() status = 0x%02X\n", status);
	if (status) {
		printk("[app][sector_erase] could not sector erase\n");
		return status;
	}

	printk("[app][sector_erase] verify erase\n");
	for(
		SpiAddress.AddressU32 = StartAddress;
		SpiAddress.AddressU32 < StartAddress + EraseSize;
		SpiAddress.AddressU32 += ONE_PAGE_PROGRAM_SIZE
	)
	{
		printk("[app][sector_erase] SpiAddress.AddressU32 = 0x%02X\n", SpiAddress.AddressU32);

		status = spi_read_data(spi, spi_cfg, SpiAddress.AddressU32, &read_data_buffer, sizeof(read_data_buffer));
		printk("[app][sector_erase] spi_read_data() status = 0x%02X\n", status);
		if (status) {
			printk("[app][sector_erase] could not write data\n");
			return status;
		}

		printk("[app][sector_erase] read_data_buffer:");
		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			if(i % 16 == 0)
			{
				printk("\n");
			}

			printk("0x%02X ", read_data_buffer[i]);

			if(read_data_buffer[i] != 0xFF)
			{
				printk("\n");
				printk("[app][sector_erase] read_data_buffer[0x%02X] != 0xFF\n",i);
				return -EPERM;
			}
   		}
		printk("\n");

		for(int i=0; i <sizeof(read_data_buffer); i++)
    	{
			read_data_buffer[i] = 0x00;
   		}
	}

	return status;
}

void main(void)
{
    printk("\n[app][main] function entry\n");
    const struct device *spi;
	struct spi_config spi_cfg = {0};
	int status;

	spi = DEVICE_DT_GET(DT_ALIAS(spi0));
	if (!device_is_ready(spi)) {
		printk("[app][main] SPI device %s is not ready\n", spi->name);
		return;
	}

    spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8); //SPI_OP_MODE_MASTER; //SPI_OP_MODE_SLAVE;
	spi_cfg.frequency = 48000U;

	printk("[app][main] spi_cfg.operation = 0x%02X\n", spi_cfg.operation);
	printk("[app][main] spi_cfg.frequency = %d\n", spi_cfg.frequency);


	status = spi_manufacturer_device_id(spi, &spi_cfg);
	printk("[app][main] spi_manufacturer_device_id() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not read spi manufacturer / device id\n");
		return;
	}

/*
	status = sector_erase(spi, &spi_cfg);
	printk("[app][main] sector_erase() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not sector erase\n");
		return;
	}
*/
/*	
	status = chip_erase(spi, &spi_cfg);
	printk("[app][main] chip_erase() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not chip erase\n");
		return;
	}
*/
/*
	status = write_4_bytes(spi, &spi_cfg);
	printk("[app][main] write_4_bytes() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not write 4 bytes\n");
		return;
	}
*/
/*	
	status = write_260_bytes(spi, &spi_cfg);
	printk("[app][main] write_260_bytes() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not write 260 bytes\n");
		return;
	}
*/
/*
	status = spi_read_status_register_1(spi, &spi_cfg);
	printk("[app][main] spi_read_status_register_1() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not read status register_1\n");
		return;
	}

	status = spi_read_status_register_2(spi, &spi_cfg);
	printk("[app][main] spi_read_status_register_2() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not read status register_2\n");
		return;
	}

	status = spi_read_status_register_3(spi, &spi_cfg);
	printk("[app][main] spi_read_status_register_3() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not read status register_3\n");
		return;
	}
*/
/*
	status = enable_qe(spi, &spi_cfg);
	printk("[app][main] enable_qe() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not enable qe\n");
		return;
	}

	status = disable_qe(spi, &spi_cfg);
	printk("[app][main] disable_qe() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not disable qe\n");
		return;
	}
*/
/*
    spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_QUAD;
	spi_cfg.frequency = 48000U;
	printk("[app][main] spi_cfg.operation = 0x%02X\n", spi_cfg.operation);
	printk("[app][main] spi_cfg.frequency = %d\n", spi_cfg.frequency);

	status = spi_write_enable(spi, &spi_cfg);
	printk("[app][main] spi_write_enable() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not spi write enable\n");
		return;
	}

	status = spi_write_status_register_2(spi, &spi_cfg, 0x02);
	printk("[app][main] spi_write_status_register_2() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not spi write status register 2\n");
		return ;
	}

	status = spi_write_disable(spi, &spi_cfg);
	printk("[app][main] spi_write_disable() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not spi write disable\n");
		return;
	}

	status = spi_read_status_register_2(spi, &spi_cfg);
	printk("[app][main] 2 spi_read_status_register_2() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] 2 could not read status register_2\n");
		return;
	}
*/	
/*
	status = block_sector_lock(spi, &spi_cfg);
	printk("[app][main] block_sector_lock() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not lock block / sector\n");
		return;
	}

	status = block_sector_unlock(spi, &spi_cfg);
	printk("[app][main] block_sector_unlock() status = 0x%02X\n", status);
	if (status) {
		printk("[app][main] could not unlock block / sector\n");
		return;
	}
*/
}
