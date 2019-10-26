#include "hw_config.h"
#include "usb_lib.h"
#include "stdio.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_bkp.h"
#include "memory.h"
#include "ad779x.h"

#define MY_T_FREQ		AD_MR_FS_123	
#define MY_T_GAIN		AD_CR_Gain64
#define MY_P_FREQ		AD_MR_FS_123	
#define MY_P_GAIN		AD_CR_Gain32
//#define PROG_SET_ADDR	0x0801EC00	// 119-th kByte of the STM32F103 medium density devices
#define PROG_SET_ADDR	0x0801F000	// 120-th kByte of the STM32F103 medium density devices
#define TIME_SET_ADDR	0x0801F400	// 121-st kByte of the STM32F103 medium density devices
#define DEV_INFO_ADDR	0x0801F800	// 122-th kByte of the STM32F103 medium density devices

// USB cable detection stuff
#define USB_DTC_RCC		RCC_APB2Periph_GPIOA
#define USB_DTC_GPIO		GPIOA
#define USB_DTC_PIN		GPIO_Pin_3

// USB autoenable stuff
#define USB_DP_PU_RCC		RCC_APB2Periph_GPIOA
#define USB_DP_PU_GPIO		GPIOA
#define USB_DP_PU_PIN		GPIO_Pin_4

// POWER FOR THE PRESSURE PART
#define P_PWR_PIN		GPIO_Pin_11                 /* PB.11 */
#define P_PWR_GPIO_PORT		GPIOB                       /* GPIOB */
#define P_PWR_GPIO_CLK		RCC_APB2Periph_GPIOB

// POWER FOR THE TEMPERATURE PART
#define T_PWR_PIN		GPIO_Pin_10                 /* PB.10 */
#define T_PWR_GPIO_PORT		GPIOB                       /* GPIOB */
#define T_PWR_GPIO_CLK		RCC_APB2Periph_GPIOB

// MCU POWER OFF stuff
#define POWER_OFF_RCC		RCC_APB2Periph_GPIOA
#define POWER_OFF_GPIO		GPIOA
#define POWER_OFF_PIN		GPIO_Pin_1

typedef struct {
	uint32_t T_freq; 
	uint32_t P_freq; 
	uint32_t P_addr; 
	uint32_t start;
	uint32_t finish; 
	uint32_t test1; 
	uint32_t test2; 
	} LoggerSettings;
LoggerSettings loggerSettings;

typedef struct {
	uint32_t start; 
	float prescaler;
	} TimeSettings;
TimeSettings timeSettings;

typedef struct {
	char	model[32]; 
	char	serial[32]; 
	char	mcu[32]; 
	char	adc[32]; 
	char	flash[32];
	char	T_sensor[32]; 
	char	P_sensor[32]; 
	char	A10[32];
	char	A11[32];
	char	A12[32];
	char	R1[32];
	char	A20[32];
	char	A21[32];
	char	A22[32];
	char	R2[32];
	char	A30[32];
	char	A31[32];
	char	A32[32];
	char	R3[32];
	char	P1[32];
	char	P2[32];
	char	P3[32];
	} DeviceInfo;
DeviceInfo deviceInfo;

//+++++++++++++++++++++++++++++++++++++++++++++++

void		PowerOFF(void);
void		MakeTemperatureMeasurement(void);
void		MakePressureMeasurement(void);
void		SetFreqLow(void);
void		SetFreqHigh(void);
void 		SaveAddressT(uint32_t addr);
void 		SaveAddressP(uint32_t addr);
void		SaveNumberCorrectionT(uint32_t num);
void		SaveNumberCorrectionP(uint32_t num);
void		SaveBlinkMode(uint16_t num);
void		SetNstate(uint16_t num);
void		T_PWR_Switch(uint8_t swtch);
void		P_PWR_Switch(uint8_t swtch);
int		GetVoltT(uint8_t CHAN);
int		GetVoltP(void);
char		RTC_Init(void);
uint8_t		CheckUSB(void);
uint16_t	GetBlinkMode(void);
uint16_t	GetNstate(void);
uint32_t	GetTime(void);
uint32_t	GetAddressT(void);
uint32_t	GetAddressP(void);
uint32_t	GetNumberCorrectionT(void);
uint32_t	GetNumberCorrectionP(void);
uint32_t	SetWakeUp(void);

void usb_dp_set();
void usb_dp_reset();

