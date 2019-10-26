#include "alarm.h"
#include "logger.h"

void vAlarmTask(void *vpars) {
	uint32_t tm, wkptm, delay, freq;
	// enable access to RTC
	RTC_Init();
	/// reset the operation correction number
	SaveNumberCorrectionT(0);
	SaveNumberCorrectionP(0);
        // set the BLINK mode to make it slow
        SaveBlinkMode(0);
        // get program settings
        ReadProgramSettings();
        // delay definition
	if (loggerSettings.T_freq < loggerSettings.P_freq)
		freq = loggerSettings.T_freq;
	else
		freq = loggerSettings.P_freq;
        if (freq < 60)
                delay = 60;
        else
                delay = freq;

	while (1) {
		tm = RTC_GetCounter();
		wkptm = SetWakeUp();
                if ((wkptm < tm + delay) & (wkptm != 0))
                        RTC_SetAlarm(wkptm + delay);
                // power off
                PowerOFF();
                // wait a bit
                vTaskDelay(1000);
	}
}

