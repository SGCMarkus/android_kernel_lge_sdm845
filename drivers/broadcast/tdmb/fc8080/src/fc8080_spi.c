/*****************************************************************************
    Copyright(c) 2013 FCI Inc. All Rights Reserved

    File name : fc8080_spi.c

    Description : spi interface source file

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

    History :
    ----------------------------------------------------------------------
*******************************************************************************/
#include <linux/input.h>
#include <linux/spi/spi.h>

#include "../inc/broadcast_fc8080.h"
#include "../inc/fci_types.h"
#include "../inc/fc8080_regs.h"
#include "../inc/fci_oal.h"

#define SPI_BMODE       0x00
#define SPI_WMODE       0x04
#define SPI_LMODE       0x08
#define SPI_RD_THRESH   0x30
#define SPI_RD_REG      0x20
#define SPI_READ        0x40
#define SPI_WRITE       0x00
#define SPI_AINC        0x80

#define CHIPID          0
#define DRIVER_NAME "fc8080_spi"

#define TX_DATA_SIZE             128
#define TX_DATA_BUF_SIZE     128
#define RX_DATA_BUF_SIZE     64 * 1024

struct spi_device *fc8080_spi = NULL;

fci_u8 *tx_data;
fci_u8 *tdata_buf;
fci_u8 *rdata_buf;

static DEFINE_MUTEX(lock);
extern struct spi_device *tdmb_fc8080_get_spi_device(void);

int fc8080_spi_write_then_read(
    struct spi_device *spi
    , fci_u8 *txbuf
    , fci_u16 tx_length
    , fci_u8 *rxbuf
    , fci_u16 rx_length)
{
    fci_s32 res = BBM_NOK;

    struct spi_message message;
    struct spi_transfer    x;

    spi_message_init(&message);
    memset(&x, 0, sizeof x);

    spi_message_add_tail(&x, &message);

    memcpy(tdata_buf, txbuf, tx_length);

    x.tx_buf = tdata_buf;
    x.rx_buf = rdata_buf;
    x.len = tx_length + rx_length;

    res = spi_sync(spi, &message);

    memcpy(rxbuf, x.rx_buf + tx_length, rx_length);

    return res;
}

int fc8080_spi_write_then_read_burst(
    struct spi_device *spi
    , fci_u8 *txbuf
    , fci_u16 tx_length
    , fci_u8 *rxbuf
    , fci_u16 rx_length)
{
    fci_s32 res;

    struct spi_message    message;
    struct spi_transfer    x;

    spi_message_init(&message);
    memset(&x, 0, sizeof x);

    spi_message_add_tail(&x, &message);

    x.tx_buf = txbuf;
    x.rx_buf = rxbuf;
    x.len = tx_length + rx_length;

    res = spi_sync(spi, &message);

    return res;
}

static fci_s32 spi_bulkread(HANDLE handle, fci_u16 addr, fci_u8 command, fci_u8 *data,
            fci_u16 length)
{
    fci_s32 res = BBM_OK;

    if(tx_data == NULL) {
        printk("[%s] tx_data is null\n", __func__);
        return BBM_NOK;
    }

    tx_data[0] = (fci_u8) (addr & 0xff);
    tx_data[1] = (fci_u8) ((addr >> 8) & 0xff);
    tx_data[2] = (fci_u8) ((command & 0xfc) | CHIPID);
    tx_data[3] = (fci_u8) (length & 0xff);

    res = fc8080_spi_write_then_read(
        fc8080_spi, &tx_data[0], 4, &data[0], length);

    if (res) {
        printk("fc8080_spi_bulkread fail : %d\n", res);
        return BBM_NOK;
    }

    return BBM_OK;
}

static fci_s32 spi_bulkwrite(HANDLE handle, fci_u16 addr, fci_u8 command, fci_u8 *data,
            fci_u16 length)
{
    fci_s32 res = BBM_OK;
    fci_s32 i = 0;

    if(tx_data == NULL) {
        printk("[%s] tx_data is null\n", __func__);
        return BBM_NOK;
    }

    tx_data[0] = (fci_u8) (addr & 0xff);
    tx_data[1] = (fci_u8) ((addr >> 8) & 0xff);
    tx_data[2] = (fci_u8) ((command & 0xfc) | CHIPID);
    tx_data[3] = (fci_u8) (length & 0xff);

    for (i = 0; i < length; i++) {
        if(length < 6) {
            tx_data[4+i] = data[i];
        }
    }

    res = fc8080_spi_write_then_read(
        fc8080_spi, &tx_data[0], length+4, NULL, 0);

    if (res) {
        printk("fc8080_spi_bulkwrite fail : %d\n", res);
        return BBM_NOK;
    }

    return BBM_OK;
}

static fci_s32 spi_dataread(HANDLE handle, fci_u8 addr, fci_u8 command, fci_u8 *data,
            fci_u32 length)
{
    fci_s32 res = BBM_OK;

    if(tx_data == NULL) {
        printk("[%s] tx_data is null\n", __func__);
        return BBM_NOK;
    }

    tx_data[0] = (fci_u8) (addr & 0xff);
    tx_data[1] = (fci_u8) ((addr >> 8) & 0xff);
    tx_data[2] = (fci_u8) ((command & 0xfc) | CHIPID);
    tx_data[3] = (fci_u8) (length & 0xff);

    if(length > 384) {
        res = fc8080_spi_write_then_read_burst(fc8080_spi
        , &tx_data[0], 4, &data[0], length);
    } else {
        res = fc8080_spi_write_then_read(fc8080_spi
        , &tx_data[0], 4, &data[0], length);
    }

    if (res) {
        printk("fc8080_spi_dataread fail : %d\n", res);
        return BBM_NOK;
    }

    return BBM_OK;
}

int fc8080_spi_init(HANDLE hDevice, fci_u16 param1, fci_u16 param2)
{
    fc8080_spi = tdmb_fc8080_get_spi_device();
    if(fc8080_spi == NULL) {
        printk("fc8080_spi device is not ready \n");
        return BBM_NOK;
    }

    if(tx_data == NULL) {
        tx_data = kmalloc(TX_DATA_SIZE, GFP_DMA | GFP_KERNEL);
        if(!tx_data) {
            printk("[%s] kmalloc of tx_data failed\n", __func__);
            goto FAIL;
        } else {
            printk("[%s] kmalloc of tx_data : %p(%p) succeed\n", __func__, &tx_data, tx_data);
            memset(tx_data, 0x0, TX_DATA_SIZE);
        }
    } else {
        printk("[%s]  tx_data isn't NULL %p(%p)\n", __func__, &tx_data, tx_data);
    }

    if(tdata_buf == NULL) {
        tdata_buf = kmalloc(TX_DATA_BUF_SIZE, GFP_DMA | GFP_KERNEL);
        if(!tdata_buf) {
            printk("[%s] kmalloc of tdata_buf failed\n", __func__);
            goto FAIL;
        } else {
            printk("[%s] kmalloc of tdata_buf : %p(%p) succeed\n", __func__, &tdata_buf, tdata_buf);
            memset(tdata_buf, 0x0, TX_DATA_BUF_SIZE);
        }
    } else {
        printk("[%s] tdata_buf isn't NULL %p(%p)\n", __func__, &tdata_buf, tdata_buf);
    }

    if(rdata_buf == NULL) {
        rdata_buf = kmalloc(RX_DATA_BUF_SIZE, GFP_DMA | GFP_KERNEL);

        if(!rdata_buf) {
            printk("[%s] kmalloc of rdata_buf failed\n", __func__);
            goto FAIL;
        } else {
            printk("[%s] kmalloc of rdata_buf : %p(%p) succeed\n", __func__, &rdata_buf, rdata_buf);
            memset(rdata_buf, 0x0, RX_DATA_BUF_SIZE);
        }
    } else {
        printk("[%s] rdata_buf isn't NULL %p(%p)\n", __func__, &rdata_buf, rdata_buf);
    }
    return BBM_OK;

FAIL :
    if(tx_data) {
         kfree(tx_data);
         printk("[%s] kfree of tx_data\n", __func__);
    }

    if(tdata_buf) {
         kfree(tdata_buf);
         printk("[%s] kfree of tdata_buf\n", __func__);
    }

    if(rdata_buf) {
         kfree(rdata_buf);
         printk("[%s] kfree of rdata_buf\n", __func__);
    }
    return BBM_NOK;
}

fci_s32 fc8080_spi_byteread(HANDLE handle, fci_u16 addr, fci_u8 *data)
{
    fci_s32 res;
    fci_u8 command = SPI_READ;

    mutex_lock(&lock);
    res = spi_bulkread(handle, addr, command, data, 1);
    mutex_unlock(&lock);

    return res;
}

fci_s32 fc8080_spi_wordread(HANDLE handle, fci_u16 addr, fci_u16 *data)
{
    fci_s32 res;
    fci_u8 command = SPI_READ | SPI_AINC;

    mutex_lock(&lock);
    res = spi_bulkread(handle, addr, command, (fci_u8 *) data, 2);
    mutex_unlock(&lock);

    return res;
}

fci_s32 fc8080_spi_longread(HANDLE handle, fci_u16 addr, fci_u32 *data)
{
    fci_s32 res;
    fci_u8 command = SPI_READ | SPI_AINC;

    mutex_lock(&lock);
    res = spi_bulkread(handle, addr, command, (fci_u8 *) data, 4);
    mutex_unlock(&lock);

    return res;
}

fci_s32 fc8080_spi_bulkread(HANDLE handle, fci_u16 addr, fci_u8 *data, fci_u16 length)
{
    fci_s32 i;
    fci_u16 x, y;
    fci_s32 res = BBM_OK;
    fci_u8 command = SPI_READ | SPI_AINC;

    x = length / 255;
    y = length % 255;

    mutex_lock(&lock);
    for (i = 0; i < x; i++, addr += 255) {
        res |= spi_bulkread(handle, addr, command, &data[i * 255], 255);
    }

    if (y) {
        res |= spi_bulkread(handle, addr, command, &data[x * 255], y);
    }
    mutex_unlock(&lock);

    return res;
}

fci_s32 fc8080_spi_bytewrite(HANDLE handle, fci_u16 addr, fci_u8 data)
{
    fci_s32 res;
    fci_u8 command = SPI_WRITE;

    mutex_lock(&lock);
    res = spi_bulkwrite(handle, addr, command, (fci_u8 *) &data, 1);
    mutex_unlock(&lock);

    return res;
}

fci_s32 fc8080_spi_wordwrite(HANDLE handle, fci_u16 addr, fci_u16 data)
{
    fci_s32 res;
    fci_u8 command = SPI_WRITE;

    if ((addr & 0xff00) != 0x0f00) {
        command |= SPI_AINC;
    }

    mutex_lock(&lock);
    res = spi_bulkwrite(handle, addr, command, (fci_u8 *) &data, 2);
    mutex_unlock(&lock);

    return res;
}

fci_s32 fc8080_spi_longwrite(HANDLE handle, fci_u16 addr, fci_u32 data)
{
    fci_s32 res;
    fci_u8 command = SPI_WRITE | SPI_AINC;

    mutex_lock(&lock);
    res = spi_bulkwrite(handle, addr, command, (fci_u8 *) &data, 4);
    mutex_unlock(&lock);

    return res;
}

fci_s32 fc8080_spi_bulkwrite(HANDLE handle, fci_u16 addr, fci_u8 *data, fci_u16 length)
{
    fci_s32 i;
    fci_u16 x, y;
    fci_s32 res = BBM_OK;
    fci_u8 command = SPI_WRITE | SPI_AINC;

    x = length / 255;
    y = length % 255;

    mutex_lock(&lock);
    for (i = 0; i < x; i++, addr += 255) {
        res |= spi_bulkwrite(handle, addr, command, &data[i * 255],
                    255);
    }

    if (y) {
        res |= spi_bulkwrite(handle, addr, command, &data[x * 255], y);
    }
    mutex_unlock(&lock);

    return res;
}

fci_s32 fc8080_spi_dataread(HANDLE handle, fci_u16 addr, fci_u8 *data, fci_u32 length)
{
    fci_s32 res;
    fci_u8 command = SPI_READ | SPI_RD_THRESH;

    mutex_lock(&lock);
    res = spi_dataread(handle, addr, command, data, length);
    mutex_unlock(&lock);

    return res;
}

fci_s32 fc8080_spi_deinit(HANDLE handle)
{
    if(tx_data) {
         kfree(tx_data);
         printk("[%s] kfree of tx_data\n", __func__);
    }

    if(tdata_buf) {
         kfree(tdata_buf);
         printk("[%s] kfree of tdata_buf\n", __func__);
    }

    if(rdata_buf) {
         kfree(rdata_buf);
         printk("[%s] kfree of rdata_buf\n", __func__);
    }
    return BBM_OK;
}
