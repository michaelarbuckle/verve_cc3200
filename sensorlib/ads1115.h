#ifndef __ADC1115_H__
#define __ADC1115_H__

/*
 * config
 * 1.I2C bus  standard mode (100 kbps) or fast mode (400 kbps).
 * Standard and fast modes are selected using a value in the I2C Master Timer Period (I2CMTPR) register
that results in an SCL frequency of 100 kbps for standard mode and 400 kbps for fast mode.
 * 2.interrupt registers
 * 3.The BURST command is enabled by setting the BURST bit in the Master Control/Status (I2CMCS) register. The number of bytes transferred by
a BURST request is programmed in the I2C Master Burst Length (I2CMBLEN) register, and a copy of this
value is automatically written to the I2C Master Burst Count (I2CMBCNT)


 * To generate a single transmit cycle, the I2C Master Slave Address (I2CMSA) register is
written with the desired address, the R/S bit is cleared, and the Control register is written with ACK=X (0
or 1), STOP=1, START=1, and RUN=1 to perform the operation and stop. When the operation is
completed (or aborted due an error), the interrupt pin becomes active and the data may be read from the
I2C Master Data (I2CMDR) register.

 All transactions on the bus begin with a START condition (S) and end with a STOP condition (P). A

*/
#define ADC_DEV_ADDR          0x48
#define VPS 0.000125f

int
GetADCRegisterValue(unsigned char ucRegAddr, unsigned char *pucRegValue);
int
SetADCRegisterValue(unsigned char ucRegAddr, unsigned char ucRegValue);
int
ADCBlockRead(unsigned char ucRegAddr,
          unsigned char *pucBlkData,
          unsigned char ucBlkDataSz);
int i2CToADCInit();

#endif
