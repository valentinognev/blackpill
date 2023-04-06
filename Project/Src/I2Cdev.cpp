// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// EFM32 stub port by Nicolas Baldeck <nicolas@pioupiou.fr>
// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2015-01-02 - Initial release


/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2015 Jeff Rowberg, Nicolas Baldeck

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#include "I2Cdev.h"
#include "main.h"
#include <string.h>
#include "projectMain.h"

#define I2C_NUM I2C_NUM_0
#define I2C_MASTER_WRITE	(0)
#define I2C_MASTER_READ		(1)

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); /*assert(0 && #x);*/} } while(0);

I2C_TypeDef* i2cdev=I2C1;

const uint32_t i2c_timeout = 100;
volatile bool i2c_dma_tx_cmplt = false;
volatile bool i2c_dma_rx_cmplt = false;
uint8_t deviceAddress = 0;
uint8_t ubMasterNbDataToTransmit = 0;
uint8_t ubMasterNbDataToReceive = 0;
uint8_t ubMasterXferDirection = 0;

bool transfer_i2c(uint8_t devaddr, uint8_t regaddr, uint8_t* tx_data, uint8_t size_of_tx_data, uint16_t timeout)
{
    deviceAddress = (devaddr << 1) | I2C_MASTER_WRITE;
    uint8_t buf[100];
    buf[0]=regaddr;
    memcpy(buf+1, tx_data, size_of_tx_data);
    ubMasterNbDataToTransmit = size_of_tx_data + 2;
    ubMasterXferDirection = LL_I2C_DIRECTION_WRITE;

    // Activate I2C1 interrupts
    LL_I2C_EnableIT_EVT(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
    // Activate DMA1 interrupts
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_1);

    /* (1) Enable I2C1 **********************************************************/
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    LL_I2C_Disable(I2C1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, ubMasterNbDataToTransmit);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_1, (uint32_t)buf, (uint32_t)LL_I2C_DMA_GetRegAddr(I2C1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_1));

    LL_I2C_Enable(I2C1);
    /* (1) Enable DMA transfer **************************************************/
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    /* (2) Prepare acknowledge for Master data reception ************************/
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

    /* (3) Initiate a Start condition to the Slave device ***********************/
    /* Master Generate Start condition */
    LL_I2C_GenerateStartCondition(I2C1);

    /* (4) Loop until Start Bit transmitted (SB flag raised) ********************/
    uint16_t tout = timeout;
    /* Loop until DMA transfer complete event */
    while (!i2c_dma_tx_cmplt)
    {
       if (LL_SYSTICK_IsActiveCounterFlag() && tout-- == 0)
            return false;
    }
    i2c_dma_tx_cmplt = false;

    /* Generate Stop condition */
    LL_I2C_GenerateStopCondition(I2C1);

    /* End of Master Process */
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    // Deactivate I2C1 interrupts
    LL_I2C_DisableIT_EVT(I2C1);
    LL_I2C_DisableIT_ERR(I2C1);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_1);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_STREAM_1);
    return true;
}

/**
 * @brief  This Function handle Master events to perform a transmission then a reception
 * process
 * @note   This function is composed in different steps :
 *         -1- Configure DMA parameters for Command Code transfer.
 *         -2- Enable DMA transfer.
 *         -3- Prepare acknowledge for Master data reception.
 *         -4- Initiate a Start condition to the Slave device.
 *         -5- Loop until end of transfer completed (DMA TC raised).
 *         -6- Prepare acknowledge for Master data reception.
 *         -7- Initiate a ReStart condition to the Slave device.
 *         -8- Loop until end of transfer completed (DMA TC raised).
 *         -9- Generate a Stop condition to the Slave device.
 *         -10- Clear pending flags, Data Command Code are checking into Slave process.
 * @param  None
 * @retval None
 */
bool receive_i2c(uint8_t devaddr, uint8_t regaddr, uint8_t* rx_data, uint8_t size_of_rx_data, uint16_t timeout)
{
    uint8_t buf[100];
    buf[0] = regaddr;
    ubMasterNbDataToTransmit = 1;
    ubMasterNbDataToReceive = size_of_rx_data;
    ubMasterXferDirection = LL_I2C_DIRECTION_WRITE;

    // Activate I2C1 interrupts
    LL_I2C_EnableIT_EVT(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
    // Activate DMA1 interrupts
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_1);

    /* (1) Enable I2C1 **********************************************************/
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    LL_I2C_Disable(I2C1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, 1);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_1, (uint32_t)&regaddr,(uint32_t)LL_I2C_DMA_GetRegAddr(I2C1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_1));
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, size_of_rx_data);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_0, (uint32_t)LL_I2C_DMA_GetRegAddr(I2C1), (uint32_t)rx_data, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_0));

    LL_I2C_Enable(I2C1);
    deviceAddress = (devaddr << 1) | I2C_MASTER_WRITE;
    /* (1) Enable DMA transfer **************************************************/
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    /* (2) Prepare acknowledge for Master data reception ************************/
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

    /* (3) Initiate a Start condition to the Slave device ***********************/
    /* Master Generate Start condition */
    LL_I2C_GenerateStartCondition(I2C1);

    uint16_t tout = timeout;
    /* Loop until DMA transfer complete event */
    while (!i2c_dma_tx_cmplt)
    {
       if (LL_SYSTICK_IsActiveCounterFlag() && tout-- == 0)
            return false;
    }
    i2c_dma_tx_cmplt = false;

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
    /* (8) Loop until end of master transfer completed (TXE flag raised) then generate STOP
     * condition -8.1- Data consistency are checking into Slave process. */

////////////////////////////////////////////////////////////////////////////////////////////
    deviceAddress = (devaddr << 1) | I2C_MASTER_READ;
    ubMasterXferDirection = LL_I2C_DIRECTION_READ;

    /* (6) Prepare acknowledge for Master data reception ************************/
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

    /* (7) Initiate a ReStart condition to the Slave device *********************/
    /* Master Generate ReStart condition */
    LL_I2C_GenerateStartCondition(I2C1);

      /* (8) Loop until end of transfer completed (DMA TC raised) *****************/
    tout = timeout;
    /* Loop until DMA transfer complete event */
    while (!i2c_dma_rx_cmplt)
    {
       if (LL_SYSTICK_IsActiveCounterFlag() && tout-- == 0)
            return false;
    }
    i2c_dma_rx_cmplt = false;
    /* (9) Generate a Stop condition to the Slave device ************************/
    LL_I2C_GenerateStopCondition(I2C1);

    /* (10) Clear pending flags, Data Command Code are checking into Slave process */
    /* Disable Last DMA bit */
    LL_I2C_DisableLastDMA(I2C1);

    /* Disable acknowledge for Master next data reception */
    LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);

    /* End of Master Process */
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
    // Deactivate I2C1 interrupts
    LL_I2C_DisableIT_EVT(I2C1);
    LL_I2C_DisableIT_ERR(I2C1);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_0);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_STREAM_0);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_1);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_STREAM_1);
    return true;
}

void transfer_complete_callback(void)
{
    i2c_dma_tx_cmplt = true;
}

void transfer_error_callback(void)
{
    NVIC_DisableIRQ(DMA1_Stream1_IRQn);
}

void receive_complete_callback(void)
{
    i2c_dma_rx_cmplt = true;
}
void receive_error_callback(void)
{
    NVIC_DisableIRQ(DMA1_Stream0_IRQn);
}

/**
 * @brief  Function called in case of error detected in I2C IT Handler
 * @param  None
 * @retval None
 */
void Error_Callback(void)
{
    /* Disable I2C1_EV_IRQn */
    NVIC_DisableIRQ(I2C1_EV_IRQn);

    /* Disable I2C1_ER_IRQn */
    NVIC_DisableIRQ(I2C1_ER_IRQn);
}
/** Default constructor.
 */
I2Cdev::I2Cdev() {
}

/** Initialize I2C0
 */
void I2Cdev::initialize() {

}

/** Enable or disable I2C
 * @param isEnabled true = enable, false = disable
 */
void I2Cdev::enable(bool isEnabled) {
  
}

/** Default timeout value for read operations.
 */
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;
/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout, void *wireObj) {
	uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout, wireObj);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout, void *wireObj) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b, timeout, wireObj);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout, wireObj)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout, void *wireObj) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w, timeout, wireObj)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout, void *wireObj) {
    return readBytes(devAddr, regAddr, 1, data, timeout, wireObj);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj) 
{
    if (receive_i2c(devAddr, regAddr, data, length, timeout))
    {
        return length;
    }
    else
    {
        return 0;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, void* wireObj)
{
    return transfer_i2c(devAddr, regAddr, &data, 1, 10);
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param length Number of bytes to write
 * @param data Array of bytes to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(
    uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data, void* wireObj)
{
    return transfer_i2c(devAddr, regAddr, data, length, 10);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data, void *wireObj)
{
	uint8_t data1[] = {(uint8_t)(data>>8), (uint8_t)(data & 0xff)};
	writeBytes(devAddr, regAddr, 2, data1, wireObj);
	return true;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, void *wireObj) {
    for (int _index=0;_index<length;_index++) {
        uint8_t _regAddr = regAddr + (_index * 2);
	    uint8_t data1[] = {(uint8_t)(data[_index]>>8), (uint8_t)(data[_index] & 0xff)};
	    writeBytes(devAddr, _regAddr, 2, data1, wireObj);
    }
	return true;
}


/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data, void *wireObj) {
    uint8_t b;
    readByte(devAddr, regAddr, &b, I2Cdev::readTimeout, wireObj);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b, wireObj);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data, void *wireObj) {
    uint16_t w;
    readWord(devAddr, regAddr, &w, I2Cdev::readTimeout, wireObj);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w, wireObj);
}


/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data, void *wireObj) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b = 0;
    if (readByte(devAddr, regAddr, &b, I2Cdev::readTimeout, wireObj) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b, wireObj);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data, void *wireObj) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devAddr, regAddr, &w, I2Cdev::readTimeout, wireObj) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w, wireObj);
    } else {
        return false;
    }
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout, void *wireObj){
	uint8_t msb[2] = {0,0};
	readBytes(devAddr, regAddr, 2, msb, timeout, wireObj);
	*data = (int16_t)((msb[0] << 8) | msb[1]);
	return 0;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of words read (-1 indicates failure)
 */
int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout, void *wireObj) {
    uint8_t msb[2] = {0,0};
    for (int _index=0;_index<length;_index++) {
        uint8_t _regAddr = regAddr + (_index * 2);
	    readBytes(devAddr, _regAddr, 2, msb, timeout, wireObj);
   	    data[_index] = (int16_t)((msb[0] << 8) | msb[1]);
	}

	return length;
}
