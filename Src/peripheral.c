#include "stm32746g_discovery_audio.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

extern AUDIO_DrvTypeDef  *audio_drv;
extern UART_HandleTypeDef huart1;

uint8_t wm8994Init(void){
	uint32_t deviceid = 0x00;

	deviceid = wm8994_drv.ReadID(AUDIO_I2C_ADDRESS);
	if((deviceid) == WM8994_ID){
		wm8994_drv.Reset(AUDIO_I2C_ADDRESS);
	 	audio_drv = &wm8994_drv;
	 	audio_drv->Init(AUDIO_I2C_ADDRESS, INPUT_DEVICE_DIGITAL_MICROPHONE_2 | OUTPUT_DEVICE_HEADPHONE, DEFAULT_VOLUME_MIDDLE, AUDIO_FREQUENCY_16K);
	 	return AUDIO_OK;
	 }
	 else{
	 	return AUDIO_ERROR;
	 }
}


void VirtualCOM_Transmit( char *format, ... ){
	char buffer[128];
	uint32_t i=0;
	va_list args;

	va_start(args, format);
	vsprintf(buffer,format,args);
	for(i=0;i<strlen(buffer);i++){
	    while( ((huart1.Instance->ISR) & UART_FLAG_TXE) == 0 );
	    huart1.Instance->TDR=buffer[i];

	}
  	va_end(args);
}
