#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "stm32f10x_gpio.h"
#include "stdlib.h"
#include "cdcio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "strtok.h"
#include "adc.h"
#include "chat.h"
#include "flash.h"
#include "gpio.h"
#include "logger.h"
#include "alarm.h"
#include "blink.h"


int main(void) {

	flash_load();

	if (CheckUSB()) {
	//if (0) {
		portBASE_TYPE err;
		// set SYSCLK = 72 MHz
		SetFreqHigh();
		// USB configuration
		Set_USBClock();
		USB_Interrupts_Config();
		USB_Init();
		usb_dp_set();
		// switch OFF the mesuring part of the device
		T_PWR_Switch(0);
		P_PWR_Switch(0);
		// run the alarm task for proper alarm operation after the USB communication finished
		err = xTaskCreate(vAlarmTask, "alarm", 64, NULL, tskIDLE_PRIORITY + 1, NULL );
		// run the chat task for communication with X86
		err = xTaskCreate(vChatTask, "chat", 256, NULL, tskIDLE_PRIORITY + 1, NULL );
		// run the blink task
		err = xTaskCreate(vBlinkTask, "blink", 64, NULL, tskIDLE_PRIORITY + 1, NULL );
		// initiate tesks
		vTaskStartScheduler();

		while (1);
	}
	else {
		uint32_t tm = 0; 
		uint16_t st = 0; 
		// reading of the measurement schedule
		ReadProgramSettings();
		// Enable clock and power for RTC
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
		// Enable access to the BKP domain
		PWR_BackupAccessCmd(ENABLE);
		// Configure the TAMPER pin as an ALARM output
	        BKP_RTCOutputConfig(BKP_RTCOutputSource_Alarm);
		RTC_WaitForLastTask();
		// chek, if current time is in the range of the measurement programm frame
		tm = GetTime();
		if ((tm > (loggerSettings.start - 2)) & (tm < (loggerSettings.finish + 2))) {
			// read the STATE
			st = GetNstate();
			// Making a PRESSURE measurement
			if ((st >> 1) & 1)
				MakePressureMeasurement();
			// Making a TEMPERATURE measurement
			if (st & 1)
				MakeTemperatureMeasurement();
		}
		
		// Set the next wake up time
		SetWakeUp();		
		// power off
		PowerOFF(); 
	}
	//err = xTaskCreate(vBlinkTask, "blink", 64, NULL,
	//		  tskIDLE_PRIORITY + 1, NULL );
	//
	//err = xTaskCreate(vChatTask, "chat", 256, NULL,
	//		  tskIDLE_PRIORITY + 1, NULL );

	//err = xTaskCreate(gpio_reset_task, "gpio", 128, NULL,
	//		  tskIDLE_PRIORITY + 1, NULL );

	//vTaskStartScheduler();

	//RTC_Init();

	//while(1);
	}

void vApplicationStackOverflowHook(xTaskHandle xTask,
			           signed portCHAR *pcTaskName )
{
	while(1)
		;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
/* User can add his own implementation to report the file name and line number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
		;
}
#endif
