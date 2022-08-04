/******************** (C) COPYRIGHT 2015 INCUBECN *****************************
* File Name          : board.c
* Author             : Tiko Zhong
* Date First Issued  : 6/15/2022
* Description        : 
*                      
********************************************************************************
* History:
* Jun20,2022: V0.1
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "board.h"
#include "main.h"
#include "internalFlash.h"
#include "inputCmd.h"
#include "outputCmd.h"

/* import handle from main.c variables ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const char ABOUT[] = {"ESL22-FU001-01.01"};
const char COMMON_HELP[] = {
	"common help:"
	"\n %brd.help()"
	"\n %brd.about()"
	"\n %brd.restart()"
	"\n %brd.address()"
	"\n %brd.reg.read(addr)"
	"\n %brd.reg.write(addr,val)"
	"\n %brd.baud.set(bHost,bBus)"
	"\n %brd.baud.get()"
	"\n %brd.rom.format(val)"
	"\n %brd.rom.read_test(startAddr,endAddr)"
	"\n %brd.rom.write_test(startAddr,endAddr)"
	"\n"
};

char addrPre[4] = {0};	//addr precode
u8 boardAddr = 0;
u8 initalDone = 0;
u8 baudHost = 4;		//BAUD[4]=115200
u8 baud485 = 4;
u32 errorCode;// = 0;
/**********************************************
*  PINs Define
**********************************************/
const PIN_T RUNNING = {LED_SWCLK_BOOT_GPIO_Port, LED_SWCLK_BOOT_Pin};

/**********************************************
*  static Devices
**********************************************/
#define RX_POOL_LEN	(MAX_CMD_LEN)
#define TX_POOL_LEN	(MAX_CMD_LEN)
#define	RX_BUF_LEN	(128)
// uart device
static u8 uartRxPool[RX_POOL_LEN] = {0};
static u8 uartRxBuf[2*RX_BUF_LEN] = {0};
static u8 uartTxPool[TX_POOL_LEN] = {0};
UartDev_t console;

// rs485 device
//const PIN_T DE = {DE_GPIO_Port, DE_Pin};
//const PIN_T DET = {DET_GPIO_Port, DET_Pin};
//static u8 rs485RxPool[RX_POOL_LEN] = {0};
//static u8 rs485RxBuf[2*RX_BUF_LEN] = {0};
//static u8 rs485TxPool[TX_POOL_LEN] = {0};
//static s8 rs485AfterSend_1(UART_HandleTypeDef *huart);
//static s8 rs485BeforeSend_1(void);
//Rs485Dev_t rs485;

// storage device
sEEPROM_t sEeprom;
#define EEPROM_SIZE_USR			(256-16*4-16*1)
#define EEPROM_SIZE_REG			(16*4)
#define EEPROM_SIZE_CFG			(16*1)
// define app eeprom base address
#define EEPROM_BASE_USER		(0)
#define EEPROM_BASE_REG			(EEPROM_BASE_USER + EEPROM_SIZE_USR)
#define EEPROM_BASE_CFG			(EEPROM_BASE_REG + EEPROM_SIZE_REG)
static s8 configWrite(void);	// system config write
static s8 configRead(void);		// system config read
sEEPROM_t sEeprom;	// internal flash, use 1 page(2K bytes) to emulate eeprom with 256 bytes.

// gpio_input dev
#define GPIO_INPUTS_LEN	8
const PIN_T GPIO_INPUTS[GPIO_INPUTS_LEN] =
{
	{IN0_GPIO_Port, IN0_Pin},
	{IN1_GPIO_Port, IN1_Pin},
	{IN2_GPIO_Port, IN2_Pin},
	{IN3_GPIO_Port, IN3_Pin},
	{IN4_GPIO_Port, IN4_Pin},
	{IN5_GPIO_Port, IN5_Pin},
	{IN6_GPIO_Port, IN6_Pin},
	{IN7_GPIO_Port, IN7_Pin}
};
INPUT_DEV_T inputDev;
static void inputFallingCB(u8 indx);
static void inputRaisingCB(u8 indx);

// gpio_output dev
#define GPIO_OUTPUTS_LEN	4
const PIN_T GPIO_OUTPUTS[GPIO_OUTPUTS_LEN] =
{
	{OUT0_GPIO_Port, OUT0_Pin},
	{OUT1_GPIO_Port, OUT1_Pin},
	{OUT2_GPIO_Port, OUT2_Pin},
	{OUT3_GPIO_Port, OUT3_Pin}
};
OUTPUT_DEV_T outputDev;

TMC429_DEV stpr;
const PIN_T M_CS = {NULL, 0};
const PIN_T DRV_ENN_CFG6 = {NULL, 0};
const PIN_T SD_MODE = {NULL, 0};
const PIN_T SWN_DIAG0 = {NULL, 0};
const PIN_T SWP_DIAG1 = {NULL, 0};
const PIN_T ENCA_DCIN_CFG5 = {NULL, 0};
const PIN_T ENCB_DCEN_CFG4 = {NULL, 0};
const PIN_T ENCN_DCO = {NULL, 0};

//static u8 getAddr(void);
/* Private function prototypes -----------------------------------------------*/
// after GPIO initial, excute this function to enable
static void sayHello();

void boardPreInit(void){
	setupSimulatedEEPROM(&sEeprom);		// use 2 page for eeprom
	configRead();
}

void boardInit(void){

	sayHello();
	//read board addr
	setupUartDev(&console, &huart2, uartTxPool, RX_POOL_LEN, uartRxPool, RX_POOL_LEN, uartRxBuf, RX_BUF_LEN);

	memset(addrPre,0,4);
	strFormat(addrPre, 4, "%d.", boardAddr);
	print("%d.about(\"%s\")\r\n", boardAddr, ABOUT);

	printS("setup input...");
	outputDevSetup(&outputDev, GPIO_OUTPUTS, GPIO_OUTPUTS_LEN, 0x00000000);
	printS("ok\r\n");

	printS("setup ouput...");
	InputDevSetup(&inputDev, GPIO_INPUTS, GPIO_INPUTS_LEN);
	printS("ok\r\n");
	inputDev.rsrc.fallingCallback = inputFallingCB;
	inputDev.rsrc.raisingCallback =inputRaisingCB;

	printS("setup stepper motor...");
	setupDev_tmc429(
		&stpr,
		"stpr",
		&hspi1,	// spi handle
		&M_CS,				// spi select
		&DRV_ENN_CFG6,		// driver enable
		&SD_MODE,			//
		&SWN_DIAG0,			//
		&SWP_DIAG1,			//
		&ENCA_DCIN_CFG5,	//
		&ENCB_DCEN_CFG4,	//
		&ENCN_DCO			//
	);
	printS("ok\r\n");

	console.StartRcv(&console.rsrc);

	printS("initial complete, need help? Try bellow:");
	print("\n%d.help()", boardAddr);
	print("\n%d.%s.help()", boardAddr, inputDev.rsrc.name);
	print("\n%d.%s.help()", boardAddr, outputDev.rsrc.name);
	print("\n%d.%s.help()", boardAddr, stpr.rsrc.name);
	printS("\r\n");
	initalDone = 1;
}

static void sayHello(){
	u8 i;
	for(i=0;i<10;i++){
		HAL_GPIO_TogglePin(RUNNING.GPIOx, RUNNING.GPIO_Pin);
		HAL_Delay(10); HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(10); HAL_IWDG_Refresh(&hiwdg);
	}
}

void flshTestRead(u16 addr){
	int i,j;
	u8 buff[256] = {0};

	sEeprom.read256(addr, buff);
	for(i=0;i<16;i++){
		for(j=0;j<16;j++)
			print("0x%02x\t", buff[i*16+j]);
		printS("\n");
	}
	printS("\n");
	console.TxPolling(&console.rsrc);
}

void printS(const char* STRING){
	console.Send(&console.rsrc, (const u8*)STRING, strlen(STRING));
}

void print(const char* FORMAT_ORG, ...){
	va_list ap;
	char buf[MAX_CMD_LEN] = {0};
	s16 bytes;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
	va_end(ap);
	//send out
	if(bytes>0)	console.Send(&console.rsrc, (u8*)buf, bytes);
}

s8 ioWrite(u16 addr, const u8 *pDat, u16 nBytes){
	sEeprom.write(EEPROM_BASE_USER + addr, pDat, nBytes);
	return 0;
}

s8 ioRead(u16 addr, u8 *pDat, u16 nBytes){
	sEeprom.read(EEPROM_BASE_USER + addr, pDat, nBytes);
	return 0;
}

s8 ioReadReg(u16 addr, s32 *val){
	return(sEeprom.read(EEPROM_BASE_REG + addr*4, (u8*)val, 4));
}

s8 ioWriteReg(u16 addr, s32 val){
	return(sEeprom.write(EEPROM_BASE_REG + addr*4, (u8*)&val, 4));
}

static s8 configWrite(void){
	u8 buff[16]={0};
	u16 sum = 0,i;
	buff[0] = baudHost;
	buff[1] = baud485;
	buff[2] = boardAddr;
	buff[3] = boardAddr>>8;
	for(i=0;i<14;i++){	sum += buff[i];	}
	buff[14] = sum;
	buff[15] = sum>>8;
	sEeprom.write(EEPROM_SIZE_CFG, buff, 16);
	return 0;
}

static s8 configRead(void){
	u8 buff[16] = {0};
	u16 sum,checkcode,i;
	sEeprom.read(EEPROM_SIZE_CFG, buff, 16);
	for(i=0,sum=0;i<14;i++){	sum += buff[i];	}
	checkcode = buff[15];	checkcode <<= 8;
	checkcode |= buff[14];

	if(sum == checkcode){
		baudHost = buff[0];
		baud485 = buff[1];
		if(baudHost >= 7)	 baudHost = 2;	// 2@115200
		if(baud485 >= 7)	 baud485 = 2;	// 2@115200
		boardAddr = buff[3];	boardAddr <<= 8;
		boardAddr |= buff[2];
	}
	else{
		baudHost = 2;	// 2@115200
		baud485 = 2;	// 2@115200
		boardAddr = 0;
	}

	return 0;
}

void printHelp(u8 brdAddr, void (*xprint)(const char* FORMAT_ORG, ...)){
	xprint("+ok@%d.help()\n%s"
			"%d.%s.help()\n"
			"%d.%s.help()\r\n",
			boardAddr, COMMON_HELP,
			boardAddr, inputDev.rsrc.name,
			boardAddr, outputDev.rsrc.name);
}

u8 brdCmd(const char* CMD, u8 brdAddr, void (*xprint)(const char* FORMAT_ORG, ...)){
	s32 i=0,j=0,k=0, ii;
	u8 buff[256] = {0};
	// common

	if(strncmp(CMD, "about", strlen("about")) == 0){
		xprint("+ok@%d.about(\"%s\")\r\n", brdAddr, ABOUT);
		sayHello();
		return 1;
	}
	else if(strncmp(CMD, "help", strlen("help")) == 0){
		printHelp(brdAddr, xprint);
		return 1;
	}

	else if(strncmp(CMD, "restart", strlen("restart")) == 0){
		HAL_NVIC_SystemReset();
		return 1;
	}

	else if(sscanf(CMD, "address %d", &i)==1){
		boardAddr = i;
		configWrite();
		memset(addrPre,0,4);
		strFormat(addrPre, 4, "%d.", boardAddr);
		xprint("+ok@%d.address(%d)\r\n", brdAddr, i);
		return 1;
	}

	else if(sscanf(CMD, "rom.format %d", &i)==1){
		memset(buff,i,256);
		sEeprom.write(0,buff,256);
		xprint("+ok@%d.rom.format(%d)\r\n", brdAddr, i);
		return 1;
	}
	else if(sscanf(CMD, "rom.write_test %d %d", &i, &j)==2){
		buff[0] = 0;
		for(ii=i;ii<=j;ii++){
			sEeprom.write(ii, buff, 1);
			buff[0] ++;
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(10);
		}
		xprint("+ok@%d.rom.write_test(%d,%d)\r\n", brdAddr, i, j);
		return 1;
	}

	else if(sscanf(CMD, "rom.read_test %d %d", &i, &j)==2){
		k = 0;
		for(ii=i;ii<=j;ii++){
			sEeprom.read(ii, buff, 1);
			k++;
			HAL_IWDG_Refresh(&hiwdg);
			if(k%16==0){
				xprint("%02x\n", buff[0]);
				k = 0;
			}
			else xprint("%02x ", buff[0]);
			console.TxPolling(&console.rsrc);
		}
		xprint("+ok@%d.rom.read_test(%d,%d)\r\n", brdAddr, i, j);
		return 1;
	}

	else if(sscanf(CMD, "reg.write %d %d ", &i, &j)==2){
		if(i>=EEPROM_SIZE_REG/4)	{
			xprint("+err@%d.reg.write(\"0..%d\")\r\n", brdAddr, EEPROM_SIZE_REG/4-1);
			return 1;
		}
		if(ioWriteReg(i&0xffff,j) == 0)	xprint("+ok@%d.reg.write(%d,%d)\r\n", brdAddr, i, j);
		else xprint("+err@%d.reg.write(%d,%d)\r\n", brdAddr, i, j);
		return 1;
	}
	else if(sscanf(CMD, "reg.read %d ", &i)==1){
		if(i>=EEPROM_SIZE_REG/4){
			xprint("+err@%d.reg.read(\"0..%d\")\r\n", brdAddr, EEPROM_SIZE_REG/4-1);
			return 1;
		}
		j = 0;
		ioReadReg(i&0xffff, &j);
		xprint("+ok@%d.reg.read(%d,%d)\r\n", brdAddr, i, j);
		return 1;
	}
	// baud config command
	else if(sscanf(CMD, "baud.set %d %d", &i,&j)==2){
		for(k=0;k<7;k++){
			baudHost = k;
			if(i==BAUD[baudHost])	break;
		}
		for(k=0;k<7;k++){
			baud485 = k;
			if(j==BAUD[baud485])	break;
		}
		configWrite();
		xprint("+ok@%d.baud.set(%d,%d)\r\n", brdAddr, BAUD[baudHost], BAUD[baud485]);
		return 1;
	}
	else if(strncmp(CMD, "baud.get ", strlen("baud.get "))==0){
		configRead();
		xprint("+ok@%d.baud.get(%d,%d)\r\n", brdAddr, BAUD[baudHost], BAUD[baud485]);
		return 1;
	}

	return 0;
}

static void inputFallingCB(u8 indx){
	print("+msg@%d.%s.falling(%d)\n", boardAddr, inputDev.rsrc.name, indx);
}

static void inputRaisingCB(u8 indx){
	print("+msg@%d.%s.raising(%d)\n", boardAddr, inputDev.rsrc.name, indx);
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle){}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(initalDone==0)	return;
	if(huart->Instance == console.rsrc.huart->Instance){
		console.rsrc.flag |= BIT(0);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){
	if(initalDone == 0)	return;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
