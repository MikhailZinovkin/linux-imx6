/*
 * s25fl.c
 *
 * Driver for Cypress S25FL128S/S25FL256S SPI Flash chips
 *
 * 
 * Author: Podkolzin Igor JSC "ROSS" Moscow
 * Mail: i.podkolzin@ross-jsc.ru
 *
 * Based on sst26.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/string.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <asm-generic/sizes.h>

/* Erases can take up to 3 seconds! */
#define MAX_READY_WAIT_JIFFIES	msecs_to_jiffies(3000)

#define S25FL_CMD_WRSR						0x01		/* Write status register */
#define S25FL_CMD_WRDI						0x04		/* Write disable */
#define S25FL_CMD_RDSR						0x05		/* Read status register */
#define S25FL_CMD_WREN						0x06		/* Write enable */
#define S25FL_CMD_READ						0x13		/* High speed read */
#define S25FL_CMD_RDCR						0x35		/* Read configuration register */

#define S25FL_CMD_SECTOR_ERASE				0x21		/* Erase sector */
#define S25FL_CMD_64K_SECTOR_ERASE			0xDC		/* �������� �������� �� 64K */
#define S25FL_CMD_READ_ID					0x9F		/* Read device ID */
#define S25FL_CMD_PAGE_PROGRAM				0x12		/* Page program */

#define S25FL_STATUS_BUSY					(1 << 0)	/* Chip is busy - reg S25FL_CMD_WRSR*/
#define S25FL_STATUS_WREN					(1 << 1)	/* Write enabled - req S25FL_CMD_WRSR*/
#define S25FL_STATUS_EERR					(1 << 5)	/* ������ �������� ������� */
#define S25FL_STATUS_PERR					(1 << 6)	/* ������ ������ � ������ */

#define S25FL_CONFIG_BP						(1 << 3)	/* Block protection - reg S25FL_CMD_RDCR */
#define S25FL_CONFIG_WEN					(1 << 1)		

#define S25FL_WRITE_HEADER_SIZE			5

//�������, �� ������� ���������� ������� �� 64� � 4� ������� ��� ��������
#define S35FL_ERASE_SECTOR_64K_BOARDER		(SZ_4K * 32)

struct s25fl_flash {
	struct spi_device	*spi;
	struct mutex		lock;
	struct mtd_info		mtd;
};

struct flash_info {
	const char		*name;
	uint16_t		device_id;
	unsigned		page_size;
	unsigned		nr_pages;
	unsigned		erase_size;
};

#define to_s25fl_flash(x) container_of(x, struct s25fl_flash, mtd)

static struct flash_info s25fl_flash_info[] = {
	{
		.name		= "s25fl128s",
		.device_id	= 0x0118,
		.page_size	= SZ_256,
		.nr_pages	= 65536,
		.erase_size	= SZ_64K 
	},
	{
		.name		= "s25fl256s",
		.device_id	= 0x0119,
		.page_size	= SZ_256,
		.nr_pages	= 131072,
		.erase_size	= SZ_64K
	},
};

static int s25fl_status(struct s25fl_flash *flash, int *status)
{
	struct spi_message m;
	struct spi_transfer t;
	unsigned char cmd_resp[2];
	int err;

	spi_message_init(&m);
	memset(&t, 0, sizeof(struct spi_transfer));

	cmd_resp[0] = S25FL_CMD_RDSR;
	cmd_resp[1] = 0xff;

	t.tx_buf = cmd_resp;
	t.rx_buf = cmd_resp;
	t.len = sizeof(cmd_resp);

	spi_message_add_tail(&t, &m);

	err = spi_sync(flash->spi, &m);
	if (err < 0)
		return err;
	*status = cmd_resp[1];
	return 0;
}

static int s25fl_unblock_flash(struct s25fl_flash *flash)
{
	unsigned char command[1];
	int err;

	command[0] = S25FL_CONFIG_BP;

	err = spi_write(flash->spi, command, sizeof( command ) );

	if (err)
		return err;
	return 0;

}

static int s25fl_write_enable(struct s25fl_flash *flash, int enable)
{
	unsigned char command[ 1 ];
	int status, err;

	command[0] = enable ? S25FL_CMD_WREN : S25FL_CMD_WRDI;

	err = spi_write(flash->spi, command, sizeof( command ) );
	if (err)
		return err;

	
	if (enable) {
		err = s25fl_status(flash, &status);
		if (err)
			return err;
		if (!(status & S25FL_STATUS_WREN))
			return -EROFS;
	}

	return 0;
}

/**
 * @brief ������� �������� ���� ���������� �� �����������
 *
 * @param flash - ��������� �� ��������� ����������� ������ ����������
 * @param check_erro - �������������� ��� ��������, ���� ������ ��� ���������� � �������, �� ������, ���� �������� == 0, �� ��� ��������
 * @param check_erro_inv - �������������� ��� ��������, ���� ������ ��� ������� � �������, �� ������, ���� �������� == 0, �� ��� ��������
 * @return int
 **/
static int s25fl_wait_till_ready(struct s25fl_flash *flash, int check_erro, int check_erro_inv )
{
	unsigned long deadline;
	int status, err;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;
	do {
		err = s25fl_status(flash, &status);
		if (err)
			return err;
		if (!(status & S25FL_STATUS_BUSY))
		{
			//��������, ��� ��, ��� ������, ���� ������, � ���� ������� - �� ���� ������
			if( check_erro && status & check_erro )
			{
				printk( KERN_ERR "%s: Error on check status %02x (%02x)\n", __func__, status, check_erro );
				return -EPIPE;
			}
			if( check_erro_inv && !(status & check_erro_inv ) )
			{
				printk( KERN_ERR "%s: Error on check invert status %02x (%02x)\n", __func__, status, check_erro_inv );
				return -EPIPE;
			}
			return 0;
		}

		cond_resched();
	} while (!time_after_eq(jiffies, deadline));

	return -ETIMEDOUT;
}

/**
 * @brief �������, ������� ������� ������ ����� ��������� �������
 *
 * @param flash - ��������� �� �������� ������
 * @param offset - �����, ������� �������
 * @param cmd - �������, ������� ����� ���������� � ������
 * @return int - 0 ��� ��, ����� 0 � ������ ������
 **/
static int s25fl_erase_sector_cmd(struct s25fl_flash *flash, uint32_t offset, unsigned char cmd )
{
	unsigned char command[ S25FL_WRITE_HEADER_SIZE ];
	int err;

	err = s25fl_write_enable(flash, 1);
	if (err)
		return err;

	err = s25fl_wait_till_ready( flash, 0, S25FL_STATUS_WREN );
	if (err)
		return err;

	command[0] = cmd;
	command[1] = offset >> 24;
	command[2] = offset >> 16;
	command[3] = offset >> 8;
	command[4] = offset;

	err = spi_write(flash->spi, command, sizeof( command ) );
	if (err)
		return err;

	err = s25fl_wait_till_ready( flash, S25FL_STATUS_EERR, 0 );
	if (err)
		return err;

	return s25fl_write_enable(flash, 0);
}

/**
 * @brief �������, ������� ������� ������� �������� 64Kb
 *
 * @param flash - ��������� �� ��������� ������
 * @param offset - ����� �� ������, ��� ��������
 * @return int - 0 ��� ��, ����� 0 � ������ ������
 **/
static int s25fl_erase_sector_64k(struct s25fl_flash *flash, uint32_t offset )
{
	//��� �������, ������� ������� � ������� ������ ��������
	if( offset < S35FL_ERASE_SECTOR_64K_BOARDER )
	{
		printk( KERN_ERR "%s: Invalid offest 0x%08x\n", __func__, offset );
		return -EINVAL;
	}

	//����� ������ ���� �������� �� 64�
	if( offset & (SZ_64K - 1) )
	{
		printk( KERN_ERR "%s: Offset not aligin for 64K\n", __func__ );
		return -EINVAL;
	}

	return s25fl_erase_sector_cmd( flash, offset, S25FL_CMD_64K_SECTOR_ERASE );
}

/**
 * @brief �������, ������� ������� ������� �� 4�
 *
 * @param flash - ��������� �� ��������� ������
 * @param offset - ����� �� ������, ��� ��������
 * @return int - 0 ��� ��, ����� 0 � ������ ������
 **/
static int s25fl_erase_sector_4k( struct s25fl_flash *flash, uint32_t offset )
{
	//��� �������, ������� ������� � ������� ������ ��������
	if( offset >= S35FL_ERASE_SECTOR_64K_BOARDER )
	{
		printk( KERN_ERR "%s: Invalid offest 0x%08x\n", __func__, offset );
		return -EINVAL;
	}

	//����� ������ ���� �������� �� 4�
	if( offset & (SZ_4K - 1) )
	{
		printk( KERN_ERR "%s: Offset not aligin for 4K\n", __func__ );
		return -EINVAL;
	}

	return s25fl_erase_sector_cmd( flash, offset, S25FL_CMD_SECTOR_ERASE );
}

/**
 * @brief �������, ������� ������� ������ ������
 *
 * @param flash - ��������� �� ��������� ������
 * @param offset ����� �� ������
 * @return int - 0 ��� ��, ����� 0 � ������ ������
 **/
static int s25fl_erase_sector( struct s25fl_flash *flash, uint32_t offset )
{
	int i;
	int ret;

	//���� �� �� �������� - ��� ��� 64� ����� - �� ������� ��
	if( offset >= S35FL_ERASE_SECTOR_64K_BOARDER )
		return s25fl_erase_sector_64k( flash, offset );

	//��� ��� ������� �� ������ �� 64�, � � ���� ������� ��� �� 4�
	//�� ���� � ����
	for( i = 0; i < flash->mtd.erasesize/SZ_4K; i++ )
	{
		ret = s25fl_erase_sector_4k( flash, offset );
		if( ret )
			return ret;

		offset += SZ_4K; //�������� 4�
	}
	
	return 0;
}

static int s25fl_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct s25fl_flash *flash = to_s25fl_flash(mtd);
	uint32_t addr, end;
	int err;

	/* Sanity checks */
	if ((uint32_t)instr->len % mtd->erasesize)
		return -EINVAL;

	if ((uint32_t)instr->addr % mtd->erasesize)
		return -EINVAL;

	addr = instr->addr;
	end = addr + instr->len;

	mutex_lock(&flash->lock);

	err = s25fl_wait_till_ready( flash, 0, 0 );
	if (err) {
		mutex_unlock(&flash->lock);
		return err;
	}

	while (addr < end) {
		err = s25fl_erase_sector(flash, addr);
		if (err) {
			mutex_unlock(&flash->lock);
			instr->state = MTD_ERASE_FAILED;
			dev_err(&flash->spi->dev, "Erase failed\n");
			return err;
		}

		addr += mtd->erasesize;
	}

	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;

	mtd_erase_callback(instr);

	return 0;
}

//�������� �� �������� ������ ������
static int s25fl_block_is_bad_stub( struct mtd_info *mtd, loff_t ofs)
{
	return 0;
}

static int s25fl_read(struct mtd_info *mtd, loff_t from, size_t len,
		       size_t *retlen, unsigned char *buf)
{
	struct s25fl_flash *flash = to_s25fl_flash(mtd);
	struct spi_transfer transfer[2];
	struct spi_message message;
	unsigned char command[ S25FL_WRITE_HEADER_SIZE ];
	int ret;
	
//printk( "%s: read from flash from=%016llx len=%zu\n", __func__, (unsigned long long)from, len );	

	spi_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));

	command[0] = S25FL_CMD_READ;
	command[1] = from >> 24;
	command[2] = from >> 16;
	command[3] = from >> 8;
	command[4] = from;

	transfer[0].tx_buf = command;
	transfer[0].len = sizeof(command);
	spi_message_add_tail(&transfer[0], &message);

	transfer[1].rx_buf = buf;
	transfer[1].len = len;
	spi_message_add_tail(&transfer[1], &message);

	mutex_lock(&flash->lock);

	/* Wait for previous write/erase to complete */
	ret = s25fl_wait_till_ready( flash, 0, 0 );
	if (ret) {
		mutex_unlock(&flash->lock);
		return ret;
	}

	spi_sync(flash->spi, &message);

	if (retlen && message.actual_length > sizeof(command))
		*retlen += message.actual_length - sizeof(command);

	mutex_unlock(&flash->lock);
	return 0;
}

//������ �� ����� ������ - ������������ ������� - ������������ �� ������� ����� ������
//������������ ������� - ����� ����� ���� �� ������ ��� ���� 
static int s25fl_write_block(struct mtd_info *mtd, loff_t to, size_t len,
			size_t *retlen, const unsigned char *buf)
{
	struct s25fl_flash *flash = to_s25fl_flash(mtd);
	int ret;
	size_t copied = 0;
	unsigned char *command = NULL;
	uint32_t writesize;

	//������ �������� 0 ����?
	if( !len )
	{
		if( retlen )
			*retlen = 0;

		return 0;
	}

//printk( "%s: write flash to=%016llx len=%zu (0x%02x)\n", __func__, (unsigned long long)to, len, buf[ len - 1 ] );	

	//������ �������� ��� ������������ �� �����?? - �� s25fl_write_unaligan
	if ((uint32_t)to % mtd->writesize) //TODO - ����� ��� ���������� �����???? - ������ ��� 64 ������ ������� �� ����� � ����� ����� 64 ������ ���� ��� ������� ��������
	{
		return -EINVAL;
	}

	//������ �������� ������ ��� ���� ����??
	if( len > mtd->writesize )
	{
		printk( KERN_ERR"%s Try write more bytes %zu (%zu)\n", __func__, len, mtd->writesize );
		return -EINVAL;
	}

	writesize = mtd->writesize;
	if(len < mtd->writesize)
		writesize = len;
	else
	 	writesize = mtd->writesize;

	command = kmalloc( writesize + S25FL_WRITE_HEADER_SIZE, GFP_KERNEL );

	if( !command )
		return -ENOMEM;

	mutex_lock( &flash->lock );

	ret = s25fl_write_enable( flash, 1 );

	if( ret )
		goto out;

	ret = s25fl_wait_till_ready( flash, 0, 0 );

	if( ret )
		goto out;


	command[0] = S25FL_CMD_PAGE_PROGRAM;
	command[1] = to >> 24;
	command[2] = to >> 16;
	command[3] = to >> 8;
	command[4] = to;

	if( len < writesize )
		writesize = len;


	memcpy( command + S25FL_WRITE_HEADER_SIZE, buf, writesize );

	ret = spi_write( flash->spi, command, writesize + S25FL_WRITE_HEADER_SIZE );

	if( ret < 0 )
		goto out;

	copied+=writesize;

	ret = s25fl_wait_till_ready( flash, S25FL_STATUS_PERR, 0 );

	if( ret )
		goto out;

out:
	kfree( command );

	ret = s25fl_write_enable(flash, 0);

	if (retlen)
		*retlen = copied;
	mutex_unlock(&flash->lock);
	
	return ret;
}


//������ �� ����� ������ - ������������ ������� - ������������ �� ������� ����� ������
static int s25fl_write( struct mtd_info *mtd, loff_t to, size_t len,
						size_t *retlen, const unsigned char *buf )
{

	size_t writebytes;
	size_t writeret;
	size_t count = len;
	size_t realwrite = 0;
	int ret = 0;

	//������ �������� 0 ����?
	if( !len )
	{
		if( retlen )
		{
			*retlen = 0;
		}

		return 0;
	}

	//������ �������� ��� ������������ �� �����?? - �� s25fl_write_unaligan
	if( ( uint32_t )to % mtd->writesize )
	{
		return -EINVAL;
	}

	while( count )
	{
		if( count > mtd->writesize )
		{
			writebytes = mtd->writesize;
		}

		else
		{
			writebytes = count;
		}

		ret = s25fl_write_block( mtd, to + realwrite, writebytes, &writeret, buf + realwrite );

		if( ret < 0 )
		{
			return ret;
		}

		if( writeret != writebytes )
		{
			return -EINVAL;
		}

		realwrite += writeret;
		count -= writeret;
	}

	if( retlen )
	{
		*retlen = realwrite;
	}
	
	return ret;

}

//������ �� ����� ������ - ����� � ��� ������������ (����� ��� ������ ���� � ����� ��� ������)
//������� �������, ��� �� ����� ���� ����� ���� �������
static int s25fl_write_unaligan( struct mtd_info *mtd, loff_t to, size_t len,
			size_t *retlen, const unsigned char *buff )
{

 	unsigned char *read_buffer; //���� ����� ������, ����� ��� ��� ������
	//������ ������� ������ ����� ������
	loff_t new_to = to - ((uint32_t)to % mtd->writesize); //�� ���� ����� ������ //TODO �� ���������� ����� ���� ��������� 64 ������ ������
	loff_t buffer_seek = (uint32_t)to % mtd->writesize; //����� �� ������, ���� ������ � ����
	size_t buffer_len = mtd->writesize - buffer_seek; //������ �� ����� ������ �� ������
	size_t write_size = 0; //������� ������� ���� �������� � ����� 
	size_t read_len = 0; //������� ������� ��������
	size_t write_len = 0; //������� ������� �������
	int ret = 0;
	size_t i;
	
	//��� ������ �������� � �� ����������� �� ������� �������
	//������ ��� ���� ����� ����������� �������, ������� �� � �������� �����
	//� �������� �� ����� - � ������ ������� ������� ���������� �����������,
	//��� �� ���������� ����������� ����������

	//� ����� �� ������� �������� ������ ����??
	if( !len )
	{
		if( retlen )
		 *retlen = 0;
		return 0;
	}

	//� ����� ������ �������� ��� ����������� ������
	if( !((uint32_t)to % mtd->writesize) )
	{
	 	//��� ��������� - ����� ��������
	 	return s25fl_write( mtd, to, len, retlen, buff );
	}
	
	if( len > buffer_len )
	{
	 	write_size = buffer_len;
	}
	else
	{
	 	write_size = len;
	}

	//������� ������ ��� �����
	read_buffer = kmalloc(mtd->writesize, GFP_KERNEL);
	if( !read_buffer )
	 	return -ENOMEM;
	
	ret = s25fl_read( mtd, new_to, mtd->writesize, &read_len, read_buffer );
	
	if( ret < 0  )
	{
		kfree( read_buffer );
		return ret;
	}
	
	if( !read_len || read_len != mtd->writesize )
	{
		kfree( read_buffer );
		return -EINVAL;
	}

	//� ����� �� ������� �� ������� ����??? - ��� �� ��������� - ������� �������,� ����� ������!!!
	//������ ������ ����� ����� ���� �������
	for( i = buffer_seek; i <  write_size; i++ )
	{
	 	if( read_buffer[i] != 0xff )
		{
			kfree( read_buffer );
			return -EINVAL;
		}
	}
	
	//��������� � ����� ��� �������
	memcpy( read_buffer + buffer_seek, buff, write_size );
	
	//������� ���� ����� � ������
	ret = s25fl_write( mtd, new_to, mtd->writesize, &write_len, read_buffer );
	
	if( ret < 0 )
	{
		kfree( read_buffer );
		return ret;
	}
	
	if( !write_len || write_len != mtd->writesize )
	{
		kfree( read_buffer );
		return -EINVAL;
	}
	
	//��������� �����
	kfree( read_buffer );
	
	//������ ������� �������
	if( retlen )
	{
	 	*retlen = write_size; //�� ��� �������� �������
	}
	
	//� ����� ��� ������ ������ ������ �� �����
	if( !( len - write_size ) )
	{
		//�� �������� ��� ��� ������
		return ret;
	}

	write_len = 0;
	
	//�������� �������� ��� ����������� �������
	ret = s25fl_write( mtd, to + write_size, len - write_size, &write_len, buff + write_size );
	
	if( ret < 0 )
	{
		return ret;
	}

	if( retlen )
	{
	 	*retlen += write_len;
	}
	
	return ret;
}


static struct flash_info* s25fl_match_device(struct spi_device *spi)
{
	struct flash_info *flash_info = NULL;
	struct spi_message m;
	struct spi_transfer t;
	unsigned char cmd_resp[4];
	int i, err;
	uint16_t id;

	spi_message_init(&m);
	memset(&t, 0, sizeof(struct spi_transfer));

	cmd_resp[0] = S25FL_CMD_READ_ID;
	cmd_resp[1] = 0xff;
	cmd_resp[2] = 0xff;
	cmd_resp[3] = 0xff;

	t.tx_buf = cmd_resp;
	t.rx_buf = cmd_resp;
	t.len = sizeof(cmd_resp);
	spi_message_add_tail(&t, &m);

	err = spi_sync(spi, &m);
	if (err < 0) {
		dev_err(&spi->dev, "error reading device id\n");
		return NULL;
	}
printk( "%s: Read ID 0x%02x 0x%02x 0x%02x 0x%02x\n", __func__, cmd_resp[0], cmd_resp[1], cmd_resp[2], cmd_resp[3] );	

	id = (cmd_resp[1] << 8) | cmd_resp[3];

	for (i = 0; i < ARRAY_SIZE(s25fl_flash_info); i++)
		if (s25fl_flash_info[i].device_id == id)
			flash_info = &s25fl_flash_info[i];

	if (!flash_info)
		dev_err(&spi->dev, "unknown id 0x%04x\n", id);

	return flash_info;
}

static int s25fl_probe(struct spi_device *spi)
{
	struct flash_info *flash_info;
	struct s25fl_flash *flash;
	struct flash_platform_data *data;
	int ret;

	flash_info = s25fl_match_device(spi);
	if (!flash_info)
		return -ENODEV;

	flash = kzalloc(sizeof(struct s25fl_flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	flash->spi = spi;

	if(s25fl_unblock_flash(flash) < 0)
		return -ENOMEM;	

	mutex_init(&flash->lock);
	dev_set_drvdata(&spi->dev, flash);

	data = spi->dev.platform_data;
	if (data && data->name)
		flash->mtd.name = data->name;
	else
		flash->mtd.name = dev_name(&spi->dev);

	flash->mtd.type				= MTD_NORFLASH;
	flash->mtd.flags			= MTD_CAP_NORFLASH;
	flash->mtd.erasesize		= flash_info->erase_size;
	flash->mtd.writesize		= flash_info->page_size;
	flash->mtd.writebufsize		= flash_info->page_size;
	flash->mtd.size				= flash_info->page_size * flash_info->nr_pages;
	flash->mtd._erase			= s25fl_erase;
	flash->mtd._read			= s25fl_read;
	flash->mtd._write 			= s25fl_write_unaligan;
	flash->mtd._block_isbad 	= s25fl_block_is_bad_stub;

	dev_info(&spi->dev, "%s (%lld KiB)\n", flash_info->name,
		 (long long)flash->mtd.size >> 10);

	pr_debug("mtd .name = %s, .size = 0x%llx (%lldMiB) "
	      ".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
	      flash->mtd.name,
	      (long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
	      flash->mtd.erasesize, flash->mtd.erasesize / 1024,
	      flash->mtd.numeraseregions);


	ret = mtd_device_parse_register(&flash->mtd, NULL, NULL,
					data ? data->parts : NULL,
					data ? data->nr_parts : 0);
	if (ret) {
		kfree(flash);
		dev_set_drvdata(&spi->dev, NULL);
		return -ENODEV;
	}

	return 0;
}

static int s25fl_remove(struct spi_device *spi)
{
	struct s25fl_flash *flash = dev_get_drvdata(&spi->dev);
	int ret;

	ret = mtd_device_unregister(&flash->mtd);
	if (ret == 0)
		kfree(flash);
	return ret;
}

static struct spi_driver s25fl_driver = {
	.driver = {
		.name	= "s25fl",
		.owner	= THIS_MODULE,
	},
	.probe		= s25fl_probe,
	.remove		= s25fl_remove,
};

module_spi_driver(s25fl_driver);

MODULE_DESCRIPTION("MTD SPI driver for Cypress S25FL128S/S25FL256S Flash chips");
MODULE_AUTHOR("Igor Podkolzin <i.podkolzin@ross-jsc.ru>, " "Evgheny Ilyine <evghin@gmail.com>, "
	      "Andre Renaud");
MODULE_LICENSE("GPL");
