#include "ad779x.h"
#include "memory.h"
#include "logger.h"

/**
  * @brief  Initializes the peripherals used by the SPI driver.
  * @param  None
  * @retval None
  */
void SPI_LowLevel_Init(void) {
	/*!< GPIO structure */
	GPIO_InitTypeDef GPIO_InitStructure;
	/*!< SPI pins clock enable */
	RCC_APB2PeriphClockCmd(sFLASH_CS_GPIO_CLK | AD_CS_GPIO_CLK | SPI_MOSI_GPIO_CLK | SPI_MISO_GPIO_CLK | SPI_SCLK_GPIO_CLK, ENABLE);
	/*!< SPI Periph clock enable */
	RCC_APB1PeriphClockCmd(SPI_CLK, ENABLE);
	/*! JTAG (PA15 and PB4 pins REMAP */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	/*!< Configure AD_SPI SC pin */
	GPIO_InitStructure.GPIO_Pin   = AD_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(AD_CS_GPIO_PORT, &GPIO_InitStructure);
	/*!< Configure sFLASH_CS pin */
	GPIO_InitStructure.GPIO_Pin   = sFLASH_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(sFLASH_CS_GPIO_PORT, &GPIO_InitStructure);
	/*!< Configure SPI pins: SCK */
	GPIO_InitStructure.GPIO_Pin   = SPI_SCLK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_SCLK_GPIO_PORT, &GPIO_InitStructure);
	/*!< Configure SPI pins: MOSI */
	GPIO_InitStructure.GPIO_Pin   = SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
	/*!< Configure SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin   = SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
	AD_CS_HIGH();
        sFLASH_CS_HIGH();
	}

void AD_Init(void) {
	/*!< SPI structure */
	SPI_InitTypeDef  SPI_InitStructure;
	/*! Initiate all the necessary GPIOs */
	SPI_LowLevel_Init();
	/*!< SPI configuration */
	SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;        
	SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;      
	SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
	// SET SPICLK = PCLK2 / 256 = 2MHz / 4 = 512 kHz 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	/*!< Enable SPI  */
	SPI_Cmd(SPI, DISABLE); // this is from the ugly multilayer wrapper from piton
	SPI_Init(SPI, &SPI_InitStructure);
        SPI_SSOutputCmd(SPI, ENABLE);  // not to write each time ch_high??? WTF? And people say it doesn't work
        SPI_NSSInternalSoftwareConfig(SPI, SPI_NSSInternalSoft_Set); // setting the CS high by default, as I realized, it just for fun :)
        SPI_Cmd(SPI, ENABLE);
	//vTaskDelay(1);
	}

/**
  * @brief  DeInitializes the SPI periphearls
  * @param  None
  * @retval None
  */
void SPI_DeInit(void) {
	/*!< GPIO init*/
        //GPIO_InitTypeDef GPIO_InitStructure;
        /*!< Disable the T_SPI  */
        SPI_Cmd(SPI, DISABLE);
        /*!< DeInitializes the SPI */
        SPI_I2S_DeInit(SPI);
        /*!< SPI Periph clock disable */
        RCC_APB1PeriphClockCmd(SPI_CLK, DISABLE);
        }

/**
  * The next two functions operate with ADC779X CONFIGURATION REGISTER
  * CR: XX  X  X   X   XXX  XX X  X  XXXX
  *     VB  BO UB  BT  GAIN RS RD BF CH
  * VB - Enable BIAS voltage generator 
  *      00 -> disabled 
  *      01 -> BIAS voltage generator connected to AIN1(−)
  *      10 -> BIAS voltage generator connected to AIN2(−)
  *      11 -> BIAS voltage generator connected to AIN3(−)
  * BO - Set to enale the Burn Out Current (1<<6)
  * UB - Clear to enable the Bipolar Mode (0<<5)
  * BT - Set to enable the BOOST regime for BIAS voltage set
  * G  - Gain settings:
  *      000 (0x00) ->   x1 
  *      001 (0x01) ->   x2 
  *      010 (0x02) ->   x4
  *      011 (0x03) ->   x8
  *      100 (0x04) ->  x16
  *      101 (0x05) ->  x32
  *      110 (0x06) ->  x64
  *      111 (0x07) -> x128
  * RS - Referense Selection:
  *      00 - >  External reference applied between REFIN1(+) and REFIN1(−)
  *      01 - >  External reference applied between REFIN2(+) and REFIN2(−)
  *      10 - >  Internal 1.17 V reference
  * RD - Referense Detect function (1<<5). Set enables this function and enables NOREF bit functioning in STATUS REGISTER.
  * BF - Set enables buferization (1<<4). The Manual said that if BUF is set: "...the voltage on any input pin must be limited to 100 mV within the power supply rails." !!!
  * CH - Defines the active Analog Input Channels:
  *      0000 (0x00) - AIN1(+) - AIN1(-)
  *      0001 (0x01) - AIN2(+) - AIN2(-)
  *      0010 (0x02) - AIN3(+) - AIN3(-)
  *      0011 (0x03) - AIN4(+) - AIN4(-)
  *      0100 (0x04) - AIN5(+) - AIN5(-)
  *      0101 (0x05) - AIN6(+) - AIN6(-)
  *      0110 (0x06) - Temperature Sensor 
  *      0111 (0x07) - AVDD monitor
  *      1000 (0x08) - AIN1(-) - AIN1(-)  What The Fuck???
  */          

uint16_t AD_ReadConfig(void) {
	uint8_t tmp0, tmp1;
	/*!< Select the FLASH: Chip Select low */
	AD_CS_LOW();
	/*!< Send the 0b01010000 byte to read the ADC CR */
	tmp0 = AD_SendByte((AD_RD | AD_CONFIG_REG));
	/*!< Send the DUMMY_BYTE to obtain the CR MSB byte */
	tmp0 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Send the DUMMY_BYTE to obtain the CR LSB byte */
	tmp1 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Deselect the AD: Chip Select high */
	AD_CS_HIGH();
	return (tmp0 << 8) | tmp1;
	}

void AD_SetConfig(uint16_t CONF) {
	uint8_t tmp;
	/*!< Select the FLASH: Chip Select low */
	AD_CS_LOW();
	/*!< Send the 0b00001000 byte to write the ADC MR */
	tmp = AD_SendByte((AD_WR | AD_CONFIG_REG));
	/*!< Send the MSB byte of MR*/
	tmp = AD_SendByte(CONF>>8);
	/*!< Send the LSB byte of MR*/
	tmp = AD_SendByte(CONF);
	/*!< Deselect the AD: Chip Select high */
	AD_CS_HIGH();
	}

/**
  * The next two functions operate with AD7794/95 MODE REGISTER
  * MR: XXX  X   00 X   0  XX  0 X   XXXX 
  *     MD   PSW    AMP    CLK   CHD FS
  * MD  - MODE set:
  *       000 -> continuous conversion mode 
  *       001 -> single conversion mode 
  *       010 -> idle mode 
  *       011 -> power down mode 
  *       100 -> internal zero calibration mode 
  *       101 -> internal full-scale calibration mode 
  *       110 -> system zero calibration mode 
  *       111 -> system full-scale calibration mode 
  * PSW - low side power switch to ground
  * AMP - Instrumentation amplifier common-mode bit. Used with CHD, see manual
  * CLK - clock source definition:
  *       00 -> Internal 64 kHz clock. Internal clock is not available at the CLK pin
  *       01 -> Internal 64 kHz clock. This clock is made available at the CLK pin
  *       10 -> External 64 kHz
  *       11 -> External clock. The external clock is divided by 2
  * CHD - CHOP disable bit
  * FS  -Filter Update selection bits:
  *                     Hz     ms
  *      0001 (0x01) - 470      4
  *      0010 (0x02) - 242      8
  *      0011 (0x03) - 123     16
  *      0100 (0x04) -  62     32
  *      0101 (0x05) -  50     40
  *      0110 (0x06) -  39     48
  *      0111 (0x07) -  33.2   60
  *      1000 (0x08) -  19.6  101
  *      1001 (0x09) -  16.7  120 
  *      1010 (0x0A) -  16.7  120 
  *      1011 (0x0B) -  12.5  160
  *      1100 (0x0C) -  10    200
  *      1101 (0x0D) -  8.33  240
  *      1110 (0x0E) -  6.25  320
  *      1111 (0x0F) -  4.17  480
  */          
uint16_t AD_ReadMode(void) {
	uint8_t tmp0, tmp1;
	/*!< Select the FLASH: Chip Select low */
	AD_CS_LOW();
	/*!< Send the 0b01001000 byte to push ADC place the MR into the DR */
	AD_SendByte((AD_RD | AD_MODE_REG));
	/*!< Send the DUMMY_BYTE to obtain the MR MSB byte */
	tmp0 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Send the DUMMY_BYTE to obtain the MR LSB byte */
	tmp1 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Deselect the AD: Chip Select high */
	AD_CS_HIGH();
	return (tmp0 << 8) | tmp1;
	}

void AD_SetMode(uint16_t MODE) {
	/*!< Select the FLASH: Chip Select low */
	AD_CS_LOW();
	/*!< Send the 0b00001000 byte to write the ADC MR */
	AD_SendByte((AD_WR | AD_MODE_REG));
	/*!< Send the MSB byte of MR*/
	AD_SendByte(MODE>>8);
	/*!< Send the LSB byte of MR*/
	AD_SendByte(MODE);
	/*!< Deselect the AD: Chip Select high */
	AD_CS_HIGH();
	}

/**
  * @brief  Sends a byte 0b01000000 through the SPI interface to the ADC.
  * @retval STATUS REGISTER value:
  * SR: X   X    X  0 X   XXX
  *     RDY ERR  NR   AD  CH
  * RDY - Ready Bit. Set when the DR is not ready
  * ERR - Error Bit. Set when the result in DR is clamped to 0 or 1
  * NR  - No-Reference Bit. Set when there is a problem with the reference voltage
  * AD  - Is set to 1 for 24-bit ADC and to zero for 16-bit
  * CH  - Channel selected (there is no table in the MANUAL!!!!): 
  *       000 -> AIN1
  *       001 -> AIN2
  *       010 -> AIN3
  *       011 -> AIN4
  *       100 -> AIN5
  *       101 -> AIN6
  */
uint8_t AD_ReadStatus(void) {
	uint8_t tmp;
	/*!< Select the FLASH: Chip Select low */
	AD_CS_LOW();
	/*!< Send the 0b01000000 byte to push ADC place the SR into the DR */
	tmp = AD_SendByte((AD_RD | AD_STATUS_REG));
	/*!< Send the DUMMY_BYTE to obtain the SR from DR */
	tmp = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Deselect the AD: Chip Select high */
	AD_CS_HIGH();
	return tmp;
	}

uint8_t AD_CheckStatus(void) {
	/*!< Send the 0b01000000 byte to push ADC place the SR into the DR */
	AD_SendByte((AD_RD | AD_STATUS_REG));
	/*!< Send the DUMMY_BYTE to obtain the SR from DR */
	return AD_SendByte(AD_DUMMY_BYTE);
	}


/**
  * @brief Reads ID
  */
uint8_t AD_ReadID(void) {
	uint8_t tmp;
	/*!< Select the ADC: Chip Select low */
	AD_CS_LOW();
	/*!< Send the 0b01010000 byte to push ADC place the ID into the DR */
	tmp = AD_SendByte((AD_RD | AD_ID_REG));
	/*!< Send the DUMMY_BYTE to obtain the ID from DR */
	tmp = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Deselect the ADC: Chip Select high */
	AD_CS_HIGH();
	return tmp;
	}

/**
  * @brief Writes OFFSET REGISTER
  */
void AD_WriteOFF(uint32_t OFF_REG_VAL) {
	/*!< Select the ADC: Chip Select low */
	AD_CS_LOW();
	/*!< Send the a command to read Offset Register */
	AD_SendByte((AD_WR | AD_OFF_REG));
	/*!< Send the OFF_REG_VAL */
	AD_SendByte(OFF_REG_VAL>>16);
	AD_SendByte(OFF_REG_VAL>>8);
	AD_SendByte(OFF_REG_VAL);
	/*!< Deselect the ADC: Chip Select high */
	AD_CS_HIGH();
	}

/**
  * @brief Reads OFFSET REGISTER
  */
uint32_t AD_ReadOFF(void) {
	uint8_t tmp0, tmp1, tmp2;
	/*!< Select the ADC: Chip Select low */
	AD_CS_LOW();
	/*!< Send the a command to read Offset Register */
	AD_SendByte((AD_RD | AD_OFF_REG));
	/*!< Send the DUMMY_BYTE to obtain the 1-st bit of the OFF_REG from DR */
	tmp0 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Send the DUMMY_BYTE to obtain the 2-nd bit of the OFF_REG from DR */
	tmp1 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Send the DUMMY_BYTE to obtain the 3-rd bit of the OFF_REG from DR */
	tmp2 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Deselect the ADC: Chip Select high */
	AD_CS_HIGH();
	return (tmp0 << 16) | (tmp1 << 8) | tmp2;
	}

/**
  * @brief Reads FULL SCALE REGISTER
  */
uint32_t AD_ReadFS(void) {
	uint8_t tmp0, tmp1, tmp2;
	/*!< Select the ADC: Chip Select low */
	AD_CS_LOW();
	/*!< Send the a command to read Offset Register */
	AD_SendByte((AD_RD | AD_FS_REG));
	/*!< Send the DUMMY_BYTE to obtain the 1-st bit of the OFF_REG from DR */
	tmp0 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Send the DUMMY_BYTE to obtain the 2-nd bit of the OFF_REG from DR */
	tmp1 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Send the DUMMY_BYTE to obtain the 3-rd bit of the OFF_REG from DR */
	tmp2 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Deselect the ADC: Chip Select high */
	AD_CS_HIGH();
	return (tmp0 << 16) | (tmp1 << 8) | tmp2;
	}

/**
  * @brief Reads the single conversion result from ADC
  */
uint32_t AD_ReadDataSingle(void) {
	uint8_t tmp0, tmp1, tmp2;
	/*!< Select the ADC: Chip Select low */
	AD_CS_LOW();
	//AD_SendByte((AD_WR | AD_MODE_REG));
	//AD_SendByte()
	//AD_SendByte(0x20);
	//AD_SendByte(FS);
	/*!< Wait for the SR RDY bit to clear */
	while (AD_CheckStatus() & 0x80);
	/*!< Send the command to push ADC place the ID into the DR */
	AD_SendByte((AD_RD | AD_DATA_REG));
	/*!< Send the DUMMY_BYTE to obtain the 1-st bit of the DATA from DR */
	tmp0 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Send the DUMMY_BYTE to obtain the 2-nd bit of the DATA from DR */
	tmp1 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Send the DUMMY_BYTE to obtain the 3-rd bit of the DATA from DR */
	tmp2 = AD_SendByte(AD_DUMMY_BYTE);
	/*!< Deselect the ADC: Chip Select high */
	AD_CS_HIGH();
	return (tmp0 << 16) | (tmp1 << 8) | tmp2;
	}

/**
  * @brief Continuously reads the conversion results from ADC
  * num - is nuber of conversions to read
  * FS - is a Frame Set frequency
  */
void AD_ReadDataCont(uint32_t* data, uint16_t num) {
	uint8_t i,j=1, tmp0, tmp1, tmp2;
	uint32_t tmp;
	/*!< Select the ADC: Chip Select low */
	AD_CS_LOW();
	for (i = 0; i < num; i++) {
		/*!< Wait for the SR RDY bit to clear */
		while (AD_CheckStatus() & 0x80);
		/*!< Send the 0x58 byte to declare read from DATA REGISTER */
		AD_SendByte(AD_RD | AD_DATA_REG);
		/*!< Send the DUMMY_BYTE to obtain the 1-st bit of the DATA from DR */
		tmp0 = AD_SendByte(AD_DUMMY_BYTE);
		/*!< Send the DUMMY_BYTE to obtain the 2-nd bit of the DATA from DR */
		tmp1 = AD_SendByte(AD_DUMMY_BYTE);
		/*!< Send the DUMMY_BYTE to obtain the 3-rd bit of the DATA from DR */
		tmp2 = AD_SendByte(AD_DUMMY_BYTE);
		/*!< Fill the buffer data with the converted value */
		*data = ((tmp0 << 16) | (tmp1 << 8) | tmp2);
		data++;
	}
	/*!< Deselect the ADC: Chip Select high */
	AD_CS_HIGH();
	}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */    
uint8_t AD_SendByte(uint8_t byte) {
        /*!< Loop while DR register in not emplty */
        while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
        /*!< Send byte through the SPI2 peripheral */
        SPI_I2S_SendData(SPI, byte);
        /*!< Wait to receive a byte */
        while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
        /*!< Return the byte read from the SPI bus */
        return SPI_I2S_ReceiveData(SPI);
	}

/**
  * @brief Resets the ADC
  */    
void AD_Reset(void) {
	uint8_t i, K;
	/*!< Select the ADC: Chip Select low */
	AD_CS_LOW();
	/*!< Send 32 ones to ADC */
	for (i = 0; i < 4; i++)
		K = AD_SendByte(0xFF);
	/*!< Deselect the ADC: Chip Select high */
	AD_CS_HIGH();
	}

