#include "stdio.h"
#include "stdlib.h"
#include "chat.h"
#include "strtok.h"
#include "version.h"
#include "gpio.h"
#include "logger.h"


#define PROMPT	"> "

char data[SECTOR_SIZE];
//uint32_t DD[100];

enum {
	CMD_HELP = 0,
	CMD_HandShake,
	CMD_SetTime,
	CMD_SetTimePrescaler,
	CMD_GetTime,
	CMD_SetProgramm,
	CMD_GetProgramm,
	CMD_SetDevInfo,
	CMD_VER,
	CMD_SendDataToSTM,
	CMD_SendDataToX86,
	CMD_sleep,
	CMD_alarm,
	CMD_TestSPI,
	CMD_LAST
};

char *cmd_list[CMD_LAST] = {
	"help",
	"hello",
	"SetTime",
	"SetTimePrescaler",
	"GetTime",
	"SetProgramm",
	"GetProgramm",
	"SetDevInfo",
	"ver",
	"SendDataToSTM",
	"SendDataToX86",
	"sleep",
	"alarm",
	"spi"
};

void vChatTask(void *vpars)
{
	char s[64];
	char cmd[64];
	char *c;
	char *tk;
	int i = 0;
	int adc_ret = -1;

	gpio_init();

	while (1) {
		// cdc_write_buf(&cdc_out, PROMPT, sizeof(PROMPT) - 1, 1);
		
		memset(cmd, 0, sizeof(cmd));
		c = cmd;

		while (1) {
			i = cdc_read_buf(&cdc_in, c, 1);
			if (i)
				cdc_write_buf(&cdc_out, c, 1, 1);
			else {
				vTaskDelay(10);
				continue;
			}
			if (*c == '\r') {
				cdc_write_buf(&cdc_out, "\n", 1, 1);
				break;
			}
			if (*c == 8) { /* backspace */
				*c = 0;
				if (c > cmd)
					c -= 1;
				continue;
			}
			if (c + 1 < cmd + sizeof(cmd))
				c += 1;
		};

		sniprintf(s, sizeof(s), "OK\r\n");
		tk = _strtok(cmd, " \n\r");

		if (strcmp(tk, cmd_list[CMD_VER]) == 0) {
			sniprintf(s, sizeof(s), "%s\r\n", __VERSION);

		} else if (strcmp(tk, cmd_list[CMD_HandShake]) == 0) {
			uint32_t coef;
			// hello reply, just a huge and unique number
			cdc_write_buf(&cdc_out, "167321907\r\n", 0, 1);
			// device info
			ReadDevInfo();
			// MODEL
			for (i = 0; i < sizeof(deviceInfo.model); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.model[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// Serial Number
			for (i = 0; i < sizeof(deviceInfo.serial); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.serial[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// MCU unit
			for (i = 0; i < sizeof(deviceInfo.mcu); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.mcu[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// ADC unit
			for (i = 0; i < sizeof(deviceInfo.adc); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.adc[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// FLASH unit
			for (i = 0; i < sizeof(deviceInfo.flash); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.flash[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// T_SENSOR unit
			for (i = 0; i < sizeof(deviceInfo.T_sensor); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.T_sensor[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// P_SENSOR unit
			for (i = 0; i < sizeof(deviceInfo.P_sensor); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.P_sensor[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// alphas for the 1-st TEMPERATURE sensor
			for (i = 0; i < sizeof(deviceInfo.A10); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.A10[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			for (i = 0; i < sizeof(deviceInfo.A11); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.A11[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			for (i = 0; i < sizeof(deviceInfo.A12); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.A12[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// R for the 1-st TEMPERATURE sensor
			for (i = 0; i < sizeof(deviceInfo.R1); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.R1[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// alphas for the 2-nd TEMPERATURE sensor
			for (i = 0; i < sizeof(deviceInfo.A20); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.A20[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			for (i = 0; i < sizeof(deviceInfo.A21); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.A21[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			for (i = 0; i < sizeof(deviceInfo.A22); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.A22[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// R for the 2-nd TEMPERATURE sensor
			for (i = 0; i < sizeof(deviceInfo.R2); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.R2[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// alphas for the 3-rd TEMPERATURE sensor
			for (i = 0; i < sizeof(deviceInfo.A30); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.A30[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			for (i = 0; i < sizeof(deviceInfo.A31); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.A31[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			for (i = 0; i < sizeof(deviceInfo.A32); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.A32[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// R for the 3-rd TEMPERATURE sensor
			for (i = 0; i < sizeof(deviceInfo.R1); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.R3[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// KELLER, Voltage-to-Temperature slope coefficient in nV per K
			for (i = 0; i < sizeof(deviceInfo.P1); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.P1[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// KELLER, Voltage-to-Pressure slope coefficients in nV per mBar
			for (i = 0; i < sizeof(deviceInfo.P2); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.P2[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// KELLER, Voltage-to-Pressure constant at zero pressure in nV
			for (i = 0; i < sizeof(deviceInfo.P3); i++) {
				sniprintf(s, sizeof(s), "%c", deviceInfo.P3[i]);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			cdc_write_buf(&cdc_out, "\r\n", 0, 1);
			// OK
			sniprintf(s, sizeof(s), "OK\n\r");	

		} else if (strcmp(tk, cmd_list[CMD_SetTime]) == 0) {
			uint8_t i = 0;
			uint32_t epoch;
			ReadTimeSettings();
			// initialize RTC
			RTC_Init();
			cdc_write_buf(&cdc_out, "ready\r\n", 0, 1);
			// real time receive form the host
			while (i < 4)
				i += cdc_read_buf(&cdc_in, &data[i], 4);
			// set RTC counter value
			epoch = ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
			RTC_SetCounter(epoch);
			timeSettings.start = epoch;
			// save TimeSettings
			WriteTimeSettings();

		} else if (strcmp(tk, cmd_list[CMD_SetTimePrescaler]) == 0) {
			ReadTimeSettings();
			// obtain the TimePrescaler form the host
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err - Provide the integer value for TimePrescaler in ppb - parts per billion\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			timeSettings.prescaler = atof(tk);
			// save new TimePrescaler value
			WriteTimeSettings();

		} else if (strcmp(tk, cmd_list[CMD_GetTime]) == 0) {
			uint32_t epoch, corr_epoch, prscl;

			RTC_Init();
			ReadTimeSettings();
			epoch = RTC_GetCounter();
			corr_epoch = GetTime();
			prscl = 10000000*timeSettings.prescaler;
			
			sniprintf(s, sizeof(s), "%d\r\n", timeSettings.start);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "%d\r\n", epoch);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "%d\r\n", corr_epoch);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "%d\r\n", prscl);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "");

		} else if (strcmp(tk, cmd_list[CMD_SetProgramm]) == 0) {
			// RTC enable
			RTC_Init();
			// address for the first data bytes write in the FLASH
			uint32_t T_address, P_address;
			T_address = 0x000000;
			// get the frequency of TEMPERATURE acquiring as "every X seconds"
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err - Provide the TEMPERATURE acquiring frequency\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			loggerSettings.T_freq = atoi(tk);

			// get the frequency of PRESSURE acquiring as "every X seconds"
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err - Provide the PRESSURE acquiring frequency\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			loggerSettings.P_freq = atoi(tk);

			// get the initial address for the PRESSURE data storage"
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err - Provide the PRESSURE initial address\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			P_address = atoi(tk);
			loggerSettings.P_addr = P_address;

			// get the start time for data acquire in seconds since epoch 
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err - Provide the data acquiring start time\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			loggerSettings.start = atoi(tk);

			// get the finish time for data acquire in seconds since epoch 
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err - Provide the data acquiring finish time\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			loggerSettings.finish = atoi(tk);

			// save the addresses to the BKP registers
			SaveAddressT(T_address);
			SaveAddressP(P_address);
			// set number corrections to zero
			SaveNumberCorrectionT(0);
			SaveNumberCorrectionP(0);
			// set both T and P Nstates to unity!
			SetNstate(3);
			// save the settings to the MCU FLASH
			WriteProgramSettings();

		} else if (strcmp(tk, cmd_list[CMD_GetProgramm]) == 0) {
			uint32_t T_address, P_address;
			// RTC enable
			RTC_Init();
			// get the address from the BKP registers
			T_address = GetAddressT();
			P_address = GetAddressP();
			// get the settings from the MCU FLASH
			ReadProgramSettings();
			// TODO it is necessary to understand, why it MUST be somewhere here BEFORE the data output
			// otherwise the PySerial does not read the transmitted data, but minicom do :(
			vTaskDelay(10);
			// ADDRESS: initial T-address is always 000000, while the initial P-address is storred implicitly in loggerSettings structure
			sniprintf(s, sizeof(s), "%x\r\n", T_address);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "%x\r\n", P_address);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "%x\r\n", loggerSettings.P_addr);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			// SCHEDULE
			sniprintf(s, sizeof(s), "%d\r\n", loggerSettings.start);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "%d\r\n", loggerSettings.finish);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			// FREQUENCIES
			sniprintf(s, sizeof(s), "%d\r\n", loggerSettings.T_freq);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "%d\r\n", loggerSettings.P_freq);
			cdc_write_buf(&cdc_out, s, strlen(s), 1);
			//
			sniprintf(s, sizeof(s), "OK\r\n");


		} else if (strcmp(tk, cmd_list[CMD_SetDevInfo]) == 0) {
			uint8_t i, rs;
			ReadDevInfo();
			// get the DEVICE model
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.model[rs], 32);
			// get the DEVICE serial number
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.serial[rs], 32);
			// get the MCU name
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.mcu[rs], 32);
			// get the ADC name
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.adc[rs], 32);
			// get the FLASH name
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.flash[rs], 32);
			// get the T_SENSOR name
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.T_sensor[rs], 32);
			// get the P_SENSOR name
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.P_sensor[rs], 32);
			// get the A10 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.A10[rs], 32);
			// get the A11 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.A11[rs], 32);
			// get the A12 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.A12[rs], 32);
			// get the R1 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.R1[rs], 32);
			// get the A20 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.A20[rs], 32);
			// get the A21 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.A21[rs], 32);
			// get the A22 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.A22[rs], 32);
			// get the R2 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.R2[rs], 32);
			// get the A30 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.A30[rs], 32);
			// get the A31 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.A31[rs], 32);
			// get the A32 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.A32[rs], 32);
			// get the R3 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.R3[rs], 32);
			// get the P1 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.P1[rs], 32);
			// get the P2 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.P2[rs], 32);
			// get the P3 coefficient
			rs = 0;
			while (rs < 16) 
				rs += cdc_read_buf(&cdc_in, &deviceInfo.P3[rs], 32);
			// OK
			sniprintf(s, sizeof(s), "OK\n\r");	
			// WRITE the DEVICE INFO
			WriteDevInfo();

		} else if (strcmp(tk, cmd_list[CMD_HELP]) == 0) {
			int i;

			for (i = 0; i < CMD_LAST; i++) {
				char *_s = cmd_list[i];

				cdc_write_buf(&cdc_out, _s, strlen(_s), 1);
				cdc_write_buf(&cdc_out, "\r\n", 2, 1);
			}

		} else if (strcmp(tk, cmd_list[CMD_SendDataToSTM]) == 0) {
			uint32_t sz = 0, rsz = 0, wr = 0, rs = 0;
			uint32_t address;
			// data ammount to receive from the host
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err1 - Provide the data amount in bytes!\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			sz = atoi(tk);
			// address to write in
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err2 - Provide the address to write to!\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			address = strtol(tk, NULL, 16); // it must point at the sector begining!!! like this XXX000 or XXXXX000 for MX25L256
			// turn the FLASH on
			sFLASH_Init();
			while (rsz < sz) {
				// erasing if adress points to the sector begining
				if ((address & 0x000FFF) == 0) {
					sFLASH_EraseSector(address, 0);
				}
				// number of bytes to write to FLASH. MX25L256 allows writing of not more than one page per cycle
				wr = PAGE_SIZE*(rsz/PAGE_SIZE < sz/PAGE_SIZE) + sz%PAGE_SIZE*(rsz/PAGE_SIZE == sz/PAGE_SIZE);
				// declare readiness
				cdc_write_buf(&cdc_out, "ready\r\n", 0, 1);
				// read data fron USB IO bufer
				rs = 0;
				while (rs < wr) 
					rs += cdc_read_buf(&cdc_in, &data[rs], wr);
				rsz += rs;
				// write the data to the flash 
				sFLASH_WriteData(data, address, wr, 0);
				// increment the address
				address += wr;
			}
			// turn the FLASH off
			sFLASH_DeInit();
			// declare finish
			cdc_write_buf(&cdc_out, "done\r\n", 0, 1);
			sniprintf(s, sizeof(s), "");

		} else if (strcmp(tk, cmd_list[CMD_SendDataToX86]) == 0) {
			uint32_t sz = 0, sd = 0, rd = 0;
			uint32_t address;
			// set fast blink mode
			SaveBlinkMode(1);
			// amount of data to send to the host, bytes
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err1 - Provide the data amount in bytes!\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			sz = atoi(tk); 
			// address to read from
			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				cdc_write_buf(&cdc_out, "Err2 - Provide the address to read from!\r\n", 0, 1);
				sniprintf(s, sizeof(s), "");
				goto out;
			}
			address = strtol(tk, NULL, 16);
			// turn the FLASH on
			sFLASH_Init();
			while (sd < sz) {
				// number of bytes to read from the FLASH
				rd = SECTOR_SIZE*(sd/SECTOR_SIZE < sz/SECTOR_SIZE) + sz%SECTOR_SIZE*(sd/SECTOR_SIZE == sz/SECTOR_SIZE);
				// read data from the FLASH to the data bufer
				sFLASH_ReadData(data, address, rd, 0);
				// write the data to the USB IO bufer
				sd += cdc_write_buf(&cdc_out, data, rd, 1);
				// increment the address 
				address += rd;
			}
			// turn the FLASH off
			sFLASH_DeInit();
			sniprintf(s, sizeof(s), "");
			// set slow blink mode
			SaveBlinkMode(0);

		} else if (strcmp(tk, cmd_list[CMD_sleep]) == 0) {
			PowerOFF();

		} else if (strcmp(tk, cmd_list[CMD_alarm]) == 0) {
			uint32_t T_address, P_address, num, T_real_alarm_time, P_real_alarm_time, real_operation_time, stm_operation_time, T_stm_alarm_time, P_stm_alarm_time, stm_time;
			uint16_t tmp;
			//
			RTC_Init();
			// reading of the settings
			ReadProgramSettings();
			ReadTimeSettings();
			T_address = GetAddressT();
			P_address = GetAddressP();
			stm_time = RTC_GetCounter();
			// calculation of the wake up time acounting for the RTC drift: TEMPERATURE
			num = (T_address/16) + GetNumberCorrectionT();
			T_real_alarm_time = loggerSettings.start + loggerSettings.T_freq*num;
			real_operation_time = T_real_alarm_time - timeSettings.start;
			stm_operation_time = real_operation_time/timeSettings.prescaler; 
			T_stm_alarm_time = timeSettings.start + stm_operation_time;
			while (T_stm_alarm_time < stm_time + 1) {
				T_stm_alarm_time += loggerSettings.T_freq;
				SaveNumberCorrectionT(GetNumberCorrectionT() + 1);
			}
			// calculation of the wake up time acounting for the RTC drift: PRESSURE
			num = (P_address - loggerSettings.P_addr)/8 + GetNumberCorrectionP();
			P_real_alarm_time = loggerSettings.start + loggerSettings.P_freq*num;
			real_operation_time = P_real_alarm_time - timeSettings.start;
			stm_operation_time = real_operation_time/timeSettings.prescaler; 
			P_stm_alarm_time = timeSettings.start + stm_operation_time;
			while (P_stm_alarm_time < stm_time + 1) {
				P_stm_alarm_time += loggerSettings.P_freq;
				SaveNumberCorrectionP(GetNumberCorrectionP() + 1);
			}
			if (T_stm_alarm_time < P_stm_alarm_time) {
				RTC_SetAlarm(T_stm_alarm_time);
				SetNstate(1);
				tmp = GetNstate();
				sniprintf(s, sizeof(s), "T-MODE: tmp = %d, DT = %d, DP = %d, ", tmp, tmp & 1, (tmp >> 1) & 1);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
				sniprintf(s, sizeof(s), "P_time = %d, T_time = %d\r\n", P_stm_alarm_time, T_stm_alarm_time);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			else if (P_stm_alarm_time < T_stm_alarm_time) {
				RTC_SetAlarm(P_stm_alarm_time);
				SetNstate(2);
				tmp = GetNstate();
				sniprintf(s, sizeof(s), "P-MODE: tmp = %d, DT = %d, DP = %d, ", tmp, tmp & 1, (tmp >> 1) & 1);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
				sniprintf(s, sizeof(s), "P_time = %d, T_time = %d\r\n", P_stm_alarm_time, T_stm_alarm_time);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}
			else {
				RTC_SetAlarm(T_stm_alarm_time);
				SetNstate(3);
				tmp = GetNstate();
				sniprintf(s, sizeof(s), "B-MODE: tmp = %d, DT = %d, DP = %d, ", tmp, tmp & 1, (tmp >> 1) & 1);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
				sniprintf(s, sizeof(s), "P_time = %d, T_time = %d\r\n", P_stm_alarm_time, T_stm_alarm_time);
				cdc_write_buf(&cdc_out, s, strlen(s), 1);
			}


		} else if (strcmp(tk, cmd_list[CMD_TestSPI]) == 0) {
			uint16_t CR, MR;
			uint32_t ID, SR, V1=333, V2, V3, PV;
			// ADC 
			AD_Init();
			AD_Reset();
			// ADC initial state
			ID = AD_ReadID();
                        MR = AD_ReadMode();
                        CR = AD_ReadConfig();
                        SR = AD_ReadStatus();
			sniprintf(s, sizeof(s), "ADC:  ID - %6x, MR - %4x, CR - %4x, SR - %4x\r\n", ID, MR, CR, SR);
                        cdc_write_buf(&cdc_out, s, strlen(s), 1);
			// TEMP
			T_PWR_Switch(1);
			//vTaskDelay(5000);
			/*!< Calibrations!!! */
			AD_SetConfig(AD_CR_BV_OFF | AD_CR_BO_OFF | AD_CR_BM | AD_CR_BT_OFF | MY_T_GAIN | AD_CR_RS_1 | AD_CR_RD_ON | AD_CR_BUF_ON | AD_CR_CH1);
			AD_SetMode(AD_MR_MD_IZC | AD_MR_CLK_IN | AD_MR_ACM_OFF | AD_MR_CHOP_ON | MY_T_FREQ);
			/*!< Wait for the SR RDY bit to clear */
			//while (AD_CheckStatus() & 0x80);
			vTaskDelay(5000);
			ID = AD_ReadID();
                        MR = AD_ReadMode();
                        CR = AD_ReadConfig();
                        SR = AD_ReadStatus();
			sniprintf(s, sizeof(s), "ADC:  ID - %6x, MR - %4x, CR - %4x, SR - %4x\r\n", ID, MR, CR, SR);
                        cdc_write_buf(&cdc_out, s, strlen(s), 1);
			//AD_SendByte((AD_WR | AD_MODE_REG));
			//AD_SendByte(0xA0);
			//AD_SendByte(MY_T_FREQ);
			/*!< Wait for the SR RDY bit to clear */
			//while (AD_CheckStatus() & 0x80);
			//
			//AD_SetConfig(AD_CR_BV_OFF | AD_CR_BO_OFF | AD_CR_BM | AD_CR_BT_OFF | MY_T_GAIN | AD_CR_RS_1 | AD_CR_RD_ON | AD_CR_BUF_ON | AD_CR_CH1);
			V1 = GetVoltT(AD_CR_CH1);
			V2 = GetVoltT(AD_CR_CH2);
			V3 = GetVoltT(AD_CR_CH3);
			T_PWR_Switch(0);
			// PRESS
			//AD_SetConfig(AD_CR_BV_DIS | AD_CR_BO_OFF | AD_CR_BM | AD_CR_BT_OFF | MY_T_GAIN | AD_CR_RS_2 | AD_CR_RD_ON | AD_CR_BUF_ON | AD_CR_CH6);
			P_PWR_Switch(1);
			PV = GetVoltP();
			P_PWR_Switch(0);
			// ADC final state
			ID = AD_ReadID();
                        MR = AD_ReadMode();
                        CR = AD_ReadConfig();
                        SR = AD_ReadStatus();
			SPI_DeInit();
			sniprintf(s, sizeof(s), "ADC_T:  CH1 = %6x, CH2 = %9d, CH3 = %9d\r\n", V1, V2, V3);
                        cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "ADC_P:   PV = %9d\r\n", PV);
                        cdc_write_buf(&cdc_out, s, strlen(s), 1);
			sniprintf(s, sizeof(s), "ADC:  ID - %6x, MR - %4x, CR - %4x, SR - %4x\r\n", ID, MR, CR, SR);
                        cdc_write_buf(&cdc_out, s, strlen(s), 1);
			// MEMORY
			sFLASH_Init();
                        ID = sFLASH_ReadID();
			SR = sFLASH_DO(sFLASH_CMD_RDSR);
			sniprintf(s, sizeof(s), "sFLASH: ID - %6x; SR - %2x\r\n", ID, SR);
                        cdc_write_buf(&cdc_out, s, strlen(s), 1);
			SPI_DeInit();
			//
			sniprintf(s, sizeof(s), "");

		} else
			sniprintf(s, sizeof(s), "E: try `help`\r\n");
out:
		cdc_write_buf(&cdc_out, s, strlen(s), 1);
	}
}

