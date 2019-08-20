#include "Satellite.h"
#include "main.h"
#include <stdio.h>
void CAN_Config_Filter(CAN_HandleTypeDef hcan){
	  /* USER CODE BEGIN CAN1_Init 2*/
	  CAN_FilterTypeDef CAN_Filter;
	  CAN_Filter.FilterBank = 14;
	  CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	  CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	  CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	  CAN_Filter.FilterMaskIdHigh = 0x0000;
	  CAN_Filter.FilterMaskIdLow = 0x0000;
	  CAN_Filter.FilterIdHigh = 0x0000;
	  CAN_Filter.FilterIdLow = 0x0000;
	  CAN_Filter.FilterActivation = ENABLE;
	  if(HAL_CAN_ConfigFilter(&hcan, &CAN_Filter) != HAL_OK){
		  Error_Handler();
	  }
}

void CAN_Transmit(CAN_HandleTypeDef hcan, CAN_TxHeaderTypeDef Message_Header, uint8_t message[],uint32_t destination_mailbox){
	 if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0){
		 if(HAL_CAN_AddTxMessage(&hcan, &Message_Header,message,&destination_mailbox) != HAL_OK){
			 //error handling
		 }
	 }
}

void CAN_Receive(CAN_HandleTypeDef hcan,CAN_RxHeaderTypeDef Message_Header, uint8_t message[]){
	  if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) != 0){
		  if(HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0, &Message_Header,message) != HAL_OK){
			  //Handle errors
		  }
	  }
}

/*@brief mounts sd card
 * @ret return 1 if succesful 0 if unsucessfull
 */
int SD_Config(){
	if(f_mount(&myFatFS,SDPath,1) == FR_OK){
		return 1;
	}
	return 0;
}

int createFile(char name[]){
	if(f_open(&fileObj, name, FA_CREATE_NEW) == FR_OK){
		f_close(&fileObj);
		return 0;
	}
	return 1;
}
/*@bief writes data to Data.txt file on SDCard
 * @param data: data to be written, size: size of data to be written
 * @ret number of bytes written to card
 */
UINT writeData(char name[], char data[], int size){
	UINT bytesWritten = 0;
	if(f_open(&fileObj, name, FA_WRITE | FA_OPEN_APPEND) == FR_OK){
		f_lseek(&fileObj, f_size(&fileObj));
		f_write(&fileObj,data,size,&bytesWritten);
		f_close(&fileObj);
	}
	return bytesWritten;
}

UINT formattedWriteInt(char name[], char data[], int size){
	UINT bytesWritten = 0;
	if(f_open(&fileObj, name, FA_WRITE | FA_OPEN_APPEND) == FR_OK){
		f_lseek(&fileObj, f_size(&fileObj));
		for(int i = 0; i < size; i++){
			f_printf(&fileObj,"%d",data[i]);
			if(i != size-1){
				f_printf(&fileObj,",");
			}
		}
		f_printf(&fileObj,"\n");
		f_close(&fileObj);
	}
	return bytesWritten;
}

UINT formattedWriteFloat(char name[], float data[], int size){
	UINT bytesWritten = 0;
	RTC_TimeTypeDef RTC_Time;
	RTC_DateTypeDef RTC_Date;
	if(f_open(&fileObj, name, FA_WRITE | FA_OPEN_APPEND) == FR_OK){
		f_lseek(&fileObj, f_size(&fileObj));
		HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BIN);
		f_printf(&fileObj,"%d/%d/%d,",RTC_Date.Date,RTC_Date.Month,RTC_Date.Year);
		f_printf(&fileObj,"%d:%d:%d,",RTC_Time.Hours,RTC_Time.Minutes,RTC_Time.Seconds);
		for(int i = 0; i < size; i++){
			char buffer[20];
			floatToString(data[i],buffer);
			f_printf(&fileObj,"%s",buffer);
			if(i != size-1){
				f_printf(&fileObj,",");
			}
		}
		f_printf(&fileObj,"\n");
		f_close(&fileObj);
	}
	return bytesWritten;
}

UINT writeString(char name[], char data[]){
	UINT bytesWritten = 0;
	if(f_open(&fileObj, name, FA_WRITE | FA_OPEN_APPEND) == FR_OK){
		f_lseek(&fileObj, f_size(&fileObj));
		f_printf(&fileObj,"%s",data);
		f_printf(&fileObj,"\n");
		f_close(&fileObj);
	}
	return bytesWritten;
}


int twosCompliment(volatile int data){
	if(data > 32768){
		data = data ^ 0b1111111111111111;
		data = data+1;
		data = data * -1;
	}
	return data;
}

int intToString(int intVal, char * buffer,int len){
	int i = 0;
	printf("%d\n",intVal);
	char temp[20];
	while(intVal > 0){
			temp[i++] = (intVal%10) + '0';
			intVal = intVal/10;
	}
	while(i < len){
			temp[i++] = '0';
	}
	for(int j = 0; j < i;j++){
		buffer[i-1-j] = temp[j];
	}
	buffer[i] = '\0';
	return i;
}
void floatToString(double number, char buffer[]){
	int intVal = (int)number;
	int place = intToString(intVal,buffer,1);
	buffer[place] = '.';
	number -= intVal;
	number*=100000000.0;
	char temp[20];
	int place2 = intToString((int)number, temp,8);
	int i = place+1;
	for(i = place+1; i < place+2+place2; i++){
		buffer[i] = temp[i-place-1];
	}
}

char getFileSize(char fileName[]){
	FIL sizeObj;
	f_open(&sizeObj,fileName,FA_READ);
	char size = (int)f_size(&sizeObj);
	f_close(&sizeObj);
	return size;
}

int initFileSystem(){
	SD_Config();
	createFile("GravData.csv");
	if(getFileSize("GravData.csv") < 1){
		writeString("GravData.csv","Date,Time,GravX,GravY,GravZ");
	}
}
