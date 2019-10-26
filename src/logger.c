#include "logger.h"


void usb_dp_set()	{
      GPIO_InitTypeDef gpio_init = {
              .GPIO_Speed = GPIO_Speed_10MHz,
              .GPIO_Mode = GPIO_Mode_Out_PP,
              .GPIO_Pin = USB_DP_PU_PIN,
      };

      RCC_APB2PeriphClockCmd(USB_DP_PU_RCC, ENABLE);
      GPIO_Init(USB_DP_PU_GPIO, &gpio_init);
      GPIO_WriteBit(USB_DP_PU_GPIO, USB_DP_PU_PIN, Bit_SET);
}


void usb_dp_reset()	{
      GPIO_InitTypeDef gpio_init = {
              .GPIO_Speed = GPIO_Speed_10MHz,
              .GPIO_Mode = GPIO_Mode_Out_PP,
              .GPIO_Pin = USB_DP_PU_PIN,
      };

      RCC_APB2PeriphClockCmd(USB_DP_PU_RCC, ENABLE);
      GPIO_Init(USB_DP_PU_GPIO, &gpio_init);
      GPIO_WriteBit(USB_DP_PU_GPIO, USB_DP_PU_PIN, Bit_RESET);
}

// global variable for the data read from the external sFLASH
char data[SECTOR_SIZE];

char RTC_Init(void) {
	// Enable clock and power for RTC
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	// Enable access to the BKP domain
	PWR_BackupAccessCmd(ENABLE);
	// Enable RTC in case it is not active
	if((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN) {
		// reset the content of the BKP registers
		RCC_BackupResetCmd(ENABLE);
		RCC_BackupResetCmd(DISABLE);
		// switch to the LSE clock
		RCC_LSEConfig(RCC_LSE_ON);
		while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {}
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		// default prescaler for seconds 
		RTC_SetPrescaler(0x7FFF);
		// eneble RTC
		RCC_RTCCLKCmd(ENABLE);
		// waiting for synchronization
		RTC_WaitForSynchro();
		//vTaskDelay(10);
        	return 1;
	}
	// Configure the TAMPER pin as an ALARM output
	BKP_RTCOutputConfig(BKP_RTCOutputSource_Alarm);
	// set the BLINK mode to make it slow
	SaveBlinkMode(0);
	return 0;
}

/* cheks the usb cable connection */
uint8_t CheckUSB(void) {
        GPIO_InitTypeDef USB_DTC;
        RCC_APB2PeriphClockCmd(USB_DTC_RCC, ENABLE);
        USB_DTC.GPIO_Mode	= GPIO_Mode_IPD;
        USB_DTC.GPIO_Speed	= GPIO_Speed_2MHz;
        USB_DTC.GPIO_Pin	= USB_DTC_PIN;
        GPIO_Init(USB_DTC_GPIO, &USB_DTC);

	return GPIO_ReadInputDataBit(USB_DTC_GPIO, USB_DTC_PIN);
}

/* switches MCU power off */
void PowerOFF(void) {
	// peripheral's initiation
        GPIO_InitTypeDef POWER_OFF;
        RCC_APB2PeriphClockCmd(POWER_OFF_RCC, ENABLE);
        POWER_OFF.GPIO_Mode       = GPIO_Mode_Out_PP;
        POWER_OFF.GPIO_Speed      = GPIO_Speed_2MHz;
        POWER_OFF.GPIO_Pin        = POWER_OFF_PIN;
        GPIO_Init(POWER_OFF_GPIO, &POWER_OFF);
	// have a nice day :)
	GPIO_SetBits(POWER_OFF_GPIO, POWER_OFF_PIN);
}

/* switches TEMPERATURE power ON of OFF */
void T_PWR_Switch(uint8_t swtch) {
	// peripheral's initiation
        GPIO_InitTypeDef POWER;
        RCC_APB2PeriphClockCmd(T_PWR_GPIO_CLK, ENABLE);
        POWER.GPIO_Mode       = GPIO_Mode_Out_PP;
        POWER.GPIO_Speed      = GPIO_Speed_2MHz;
        POWER.GPIO_Pin        = T_PWR_PIN;
        GPIO_Init(T_PWR_GPIO_PORT, &POWER);
	// switch the power
	if (swtch)
		GPIO_SetBits(T_PWR_GPIO_PORT, T_PWR_PIN);
	else
		GPIO_ResetBits(T_PWR_GPIO_PORT, T_PWR_PIN);
}

/* switches PRESSURE power ON or OFF*/
void P_PWR_Switch(uint8_t swtch) {
	// peripheral's initiation
        GPIO_InitTypeDef POWER;
        RCC_APB2PeriphClockCmd(P_PWR_GPIO_CLK, ENABLE);
        POWER.GPIO_Mode       = GPIO_Mode_Out_PP;
        POWER.GPIO_Speed      = GPIO_Speed_2MHz;
        POWER.GPIO_Pin        = P_PWR_PIN;
        GPIO_Init(P_PWR_GPIO_PORT, &POWER);
	// switch the power
	if (swtch)
		GPIO_SetBits(P_PWR_GPIO_PORT, P_PWR_PIN);
	else
		GPIO_ResetBits(P_PWR_GPIO_PORT, P_PWR_PIN);
}

/* makes the TEMPERATURE measurement for the given channel */
int GetVoltT(uint8_t CHAN) {
	uint8_t i, num = 36;
	uint32_t DD[num];
	int V = 0;
	float D1 = 0, D2 = 0, coef = 0;

	coef = (1<<(MY_T_GAIN>>8));

	AD_Reset();
	AD_SetConfig(AD_CR_BV_OFF | AD_CR_BO_OFF | AD_CR_BM | AD_CR_BT_OFF | MY_T_GAIN | AD_CR_RS_1 | AD_CR_RD_ON | AD_CR_BUF_ON | CHAN);
	AD_SetMode(AD_MR_MD_CC | AD_MR_CLK_IN | AD_MR_ACM_OFF | AD_MR_CHOP_ON | MY_T_FREQ);
	AD_ReadDataCont(DD, sizeof(DD)/4);
	for (i = 0; i < sizeof(DD)/4; i++) {
		D1 = DD[i];
	        D2 += (1250000000.0/8388608.0)*(D1 - 8388608.0)/coef;
	}
	coef = num;
	V = D2/coef;
	return V;
}

/* makes the PRESSURE measurement for the given channel */
int GetVoltP(void) {
	uint8_t i, num = 25;
	uint32_t DD[num];
	int V = 0;
	float D1 = 0, D2 = 0, coef = 0;

	coef = (1<<(MY_P_GAIN>>8));

	AD_Reset();
	AD_SetConfig(AD_CR_BV_OFF | AD_CR_BO_OFF | AD_CR_BM | AD_CR_BT_OFF | MY_P_GAIN | AD_CR_RS_2 | AD_CR_RD_ON | AD_CR_BUF_ON | AD_CR_CH6);
	AD_SetMode(AD_MR_MD_CC | AD_MR_CLK_IN | AD_MR_ACM_OFF | AD_MR_CHOP_ON | MY_P_FREQ);
	AD_ReadDataCont(DD, sizeof(DD)/4);
	for (i = 0; i < sizeof(DD)/4; i++) {
		D1 = DD[i];
	        D2 += (3000000000.0/8388608.0)*(D1 - 8388608.0)/coef;
	}
	coef = num;
	V = D2/coef;
	return V;
}


/* returns corrected time since last calibration in seconds */
uint32_t GetTime(void) {
	uint32_t epoch, dT2;
	float dT1;

	RTC_Init();
	epoch = RTC_GetCounter();
	ReadTimeSettings();
	dT1 = epoch - timeSettings.start;
	dT2 = dT1*timeSettings.prescaler;
	return timeSettings.start + dT2;
}

/* reads the current T-address in the FLASH from the BPR of STM */
uint32_t GetAddressT(void) {
	uint16_t tmp1, tmp2;
	tmp1 = BKP_ReadBackupRegister(BKP_DR1);
	tmp2 = BKP_ReadBackupRegister(BKP_DR2);
	return ((tmp1 << 16) | tmp2);
}

/* reads the current T-address in the FLASH from the BPR of STM */
uint32_t GetAddressP(void) {
	uint16_t tmp1, tmp2;
	tmp1 = BKP_ReadBackupRegister(BKP_DR3);
	tmp2 = BKP_ReadBackupRegister(BKP_DR4);
	return ((tmp1 << 16) | tmp2);
}

/* saves the current P-address in the FLASH to the BPR of STM */
void SaveAddressT(uint32_t addr) {
	BKP_WriteBackupRegister(BKP_DR1, addr >> 16);
	BKP_WriteBackupRegister(BKP_DR2, addr);
}

/* saves the current P-address in the FLASH to the BPR of STM */
void SaveAddressP(uint32_t addr) {
	BKP_WriteBackupRegister(BKP_DR3, addr >> 16);
	BKP_WriteBackupRegister(BKP_DR4, addr);
}

/* reads the correction number for the effective data points made since the schedule start time
   it is necessary as the start time can be placed in the past and the programm needs to calculate 
   the correctly the next WakeUp time moment */
uint32_t GetNumberCorrectionT(void) {
	uint16_t tmp1, tmp2;
	tmp1 = BKP_ReadBackupRegister(BKP_DR5);
	tmp2 = BKP_ReadBackupRegister(BKP_DR6);
	return ((tmp1 << 16) | tmp2);
}

uint32_t GetNumberCorrectionP(void) {
	uint16_t tmp1, tmp2;
	tmp1 = BKP_ReadBackupRegister(BKP_DR7);
	tmp2 = BKP_ReadBackupRegister(BKP_DR8);
	return ((tmp1 << 16) | tmp2);
}

/* saves the c points' correction number */
void SaveNumberCorrectionT(uint32_t num) {
	BKP_WriteBackupRegister(BKP_DR5, num >> 16);
	BKP_WriteBackupRegister(BKP_DR6, num);
}

void SaveNumberCorrectionP(uint32_t num) {
	BKP_WriteBackupRegister(BKP_DR7, num >> 16);
	BKP_WriteBackupRegister(BKP_DR8, num);
}

/* reads the BLINK constant 
   1 - means 1/4 sec and 0 - means 1/10 sec */
uint16_t GetBlinkMode(void) {
	return BKP_ReadBackupRegister(BKP_DR9);
}

/* writes the BLINK constant */
void SaveBlinkMode(uint16_t num) {
	BKP_WriteBackupRegister(BKP_DR9, num);
}

/* next functions stores the TEMPERATURE and PRESSURE states for the nex wake-up 
	00 (0) - do nothing
	01 (1) - means DO TEMPERATURE
	10 (2) - means DO PRESSURE
	11 (3) - means both
*/
void SetNstate(uint16_t st) {
	BKP_WriteBackupRegister(BKP_DR10, st);
}

uint16_t GetNstate(void) {
	return BKP_ReadBackupRegister(BKP_DR10);
}

/* makes a voltage measerement */
void MakeTemperatureMeasurement(void) {
	int V;
	uint32_t time, address;
	char T[16];
	// Enable wrtire access to the BKP reisters
	RTC_Init();
	// Get TIME
	time = GetTime();
	T[0] = time >> 24;
	T[1] = time >> 16;
	T[2] = time >> 8;
	T[3] = time;
	// Get VOLT
	T_PWR_Switch(1);
	AD_Init();
	V = GetVoltT(AD_CR_CH1);
	T[4] = V >> 24;
	T[5] = V >> 16;
	T[6] = V >> 8;
	T[7] = V;
	V = GetVoltT(AD_CR_CH2);
	T[8]  = V >> 24;
	T[9]  = V >> 16;
	T[10] = V >> 8;
	T[11] = V;
	V = GetVoltT(AD_CR_CH3);
	T[12] = V >> 24;
	T[13] = V >> 16;
	T[14] = V >> 8;
	T[15] = V;
	T_PWR_Switch(0);
	// Get ADDRESS
	address = GetAddressT();
	// write the result to the FLASH
	sFLASH_Init();
	if ((address & 0x00000FFF) == 0) {
		sFLASH_EraseSector(address, 0);
	}
	address += sFLASH_WriteData(T, address,  16, 0);
	SPI_DeInit();
	// save the address
	SaveAddressT(address);
}

// it makes a PRESSURE measurement
void MakePressureMeasurement(void) {
	int V;
	uint32_t time, address;
	char P[8];
	// Enable wrtire access to the BKP reisters
	RTC_Init();
	// Get TIME
	time = GetTime();
	P[0] = time >> 24;
	P[1] = time >> 16;
	P[2] = time >> 8;
	P[3] = time;
	// Get VOLT
	P_PWR_Switch(1);
	AD_Init();
	V = GetVoltP();
	P[4] = V >> 24;
	P[5] = V >> 16;
	P[6] = V >> 8;
	P[7] = V;
	P_PWR_Switch(0);
	// Get ADDRESS
	address = GetAddressP();
	// write the result to the FLASH
	sFLASH_Init();
	if ((address & 0x00000FFF) == 0) {
		sFLASH_EraseSector(address, 0);
	}
	address += sFLASH_WriteData(P, address, 8, 0);
	SPI_DeInit();
	// save the address
	SaveAddressP(address);
}

// sets the next wake up time and returns the value of the counter to rise the alarm 
uint32_t SetWakeUp(void) {
	uint32_t T_address, P_address, num, T_real_alarm_time, P_real_alarm_time, real_operation_time, stm_operation_time, T_stm_alarm_time, P_stm_alarm_time, stm_time;
	// reading of the settings
	ReadProgramSettings();
	ReadTimeSettings();
	T_address = GetAddressT();
	P_address = GetAddressP();
	// brick the DEVICE if the schedule is in the past
	if (GetTime() > loggerSettings.finish)
                RTC_SetAlarm(0);
	else {
                // RTC seems to produce the pulse at the END of the corresponding second :( So, I need to extract one second from X_stm_alarm_time
		stm_time = RTC_GetCounter();
		// calculation of the wake up time acounting for the RTC drift: TEMPERATURE
		num = (T_address/16) + GetNumberCorrectionT();
		T_real_alarm_time = loggerSettings.start + loggerSettings.T_freq*num;
		real_operation_time = T_real_alarm_time - timeSettings.start;
		stm_operation_time = real_operation_time/timeSettings.prescaler; 
		T_stm_alarm_time = timeSettings.start + stm_operation_time - 1;
		while (T_stm_alarm_time < stm_time + 1) {
			T_stm_alarm_time += loggerSettings.T_freq;
			SaveNumberCorrectionT(GetNumberCorrectionT() + 1);
		}
		// calculation of the wake up time acounting for the RTC drift: PRESSURE
		num = (P_address - loggerSettings.P_addr)/8 + GetNumberCorrectionP();
		P_real_alarm_time = loggerSettings.start + loggerSettings.P_freq*num;
		real_operation_time = P_real_alarm_time - timeSettings.start;
		stm_operation_time = real_operation_time/timeSettings.prescaler; 
		P_stm_alarm_time = timeSettings.start + stm_operation_time - 1;
		while (P_stm_alarm_time < stm_time + 1) {
			P_stm_alarm_time += loggerSettings.P_freq;
			SaveNumberCorrectionP(GetNumberCorrectionP() + 1);
		}
		if (T_stm_alarm_time < P_stm_alarm_time) {
			RTC_SetAlarm(T_stm_alarm_time);
			SetNstate(1);
			return T_stm_alarm_time;
		}
		else if (P_stm_alarm_time < T_stm_alarm_time) {
			RTC_SetAlarm(P_stm_alarm_time);
			SetNstate(2);
			return P_stm_alarm_time;
		}
		else {
			RTC_SetAlarm(T_stm_alarm_time);
			SetNstate(3);
			return T_stm_alarm_time;
		}
	}
}
// switches MCU to 72 MHz on the fly
void SetFreqHigh(void) {
	//INCRASE THE FLASH LATTENCY
	FLASH_SetLatency(FLASH_Latency_2);
	//SET PLL SOURCE AND MULTIPLIER
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
	//ENABLE PLL, WAIT FOR IT TO BE READY, and SET SYSCLK SOURCE AS PLLCLK
	RCC_PLLCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){};
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	//SET HCLK = SYSCLK = 72MHZ
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	//SET PCLK2 = HCLK = 72MHZ
	RCC_PCLK2Config(RCC_HCLK_Div1);
	//SET PCLK1 = HCLK/2 = 36MHZ (maximum available)
	RCC_PCLK1Config(RCC_HCLK_Div2);
	//CORE CLOCK UPDATE
	SystemCoreClockUpdate();
	//USB PIN KICK-UP
        usb_dp_set();
}

// switches MCU to 8 MHz on the fly
void SetFreqLow(void) {
	//SET HSE AS SYSCLK SOURCE, 8MHz
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
	//SET HCLK = SYSCLK / 4 = 2MHz
	RCC_HCLKConfig(RCC_SYSCLK_Div4);
	//SET PCLK1 = HCLK = 2MHZ
	RCC_PCLK1Config(RCC_HCLK_Div1);
	//SET PCLK2 = HCLK = 2MHZ
	RCC_PCLK2Config(RCC_HCLK_Div1);
	//DISABLE PLL
	RCC_PLLCmd(DISABLE);
	//DECREASE THE FLASH LATTENCY
	FLASH_SetLatency(FLASH_Latency_0);
	//CORE CLOCK UPDATE
	SystemCoreClockUpdate();
	//USB PIN RESET AND APB CLOCK DISABLE
        usb_dp_reset();
}
