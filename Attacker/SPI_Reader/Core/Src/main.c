/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdint.h>
#include "stdio.h"
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// CANguru FSM states definition
//#define IDLE_STATE 0
//#define DATA_RECEIVE_STATE 1

#define IDLE		0
#define ID 			1
#define RTR	 		2
#define IDE			3
#define R0		 	4
#define BRS		 	5
#define ESI		 	6
#define DLC_FD	 	7
#define DATA_FD	 	8
#define DLC			9
#define DATA	 	10
#define CRC_CAN	 	11
#define CRCDEL	 	12
#define CRC_FD	 	13
#define CRCDEL_FD 	14
#define ERRORFLAG 	15
#define ERRORDEL 	16
#define IF		 	17
#define OVERFLAG 	18
#define OVERDEL 	19

#define MAX_SIZE	65536
#define NOM_SIZE	2
#define DATA_SIZE	1
#define RATIO 		NOM_SIZE / DATA_SIZE	// data-nominal bit rate ratio used for sampling
#define SKIP_SIZE 	(6 - (6 / RATIO)) * NOM_SIZE + ((NOM_SIZE + DATA_SIZE) / 2)
#define START_INDEX (NOM_SIZE / 2) - 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
const int CAN_DL_BITS[] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 64, 64, 64, 64, 64, 64, 64, 64};		// CAN DLC -> bits mapping
const int CAN_FD_DL_BITS[] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 96, 128, 160, 192, 256, 384, 512};	// CAN FD DLC -> bits mapping

//uint8_t currentState = IDLE_STATE;
////uint8_t myRxFifo[SIZE];  // Assuming SIZE is the size of your myRxFifo array
//uint8_t newDataArray[MAX_SIZE];  // The array to store data after the first bit 0
//uint8_t readIndex = 0;  // Index for reading from myRxFifo
//uint8_t writeIndex = 0;  // Index for writing to newDataArray


char data[5000];				// buffer used to print with UART
volatile uint8_t RxData;		// temporary variable where the content of RXFIFO is stored
//volatile uint8_t TxData= 0b000000;	//// temporary variable to send data over SPI
uint8_t bits[8];				// split and store bits from RxData
volatile uint8_t bit = 1;			// compute (majority o ignoranza) the read bit and store it
//int majority = 0;				// majority value
uint8_t currentState = IDLE;	// state used in the FSM
uint8_t STUFF = 0;				// stuff bit counter
uint8_t PC = 0;					// counter to update STUFF and CRC_STUFF (Polarity Counter)
int BC = 0;						// number of read bits counter (Bit Counter)
int CC = 0;						// used to update CRC_STUFF (CRC Counter)
uint8_t formError = 0;			// used to signal a form error occurred
uint8_t previousBit = 1;		// used to update PC
int TEC = 0;					// Transmit Error Counter (of the protected ECU)
int IC = 0;						// Intermission Counter, used to go back to IDLE state
int DC = 0;						// Dominant Counter used in ERRORFLAG
int RC = 0;						// Recessive Counter used in ERRORFLAG and ERRORDEL
uint8_t RTRbit;					// store RTR bit, used in Classical CAN
uint8_t IDEbit;					// store IDE bit, used for 29 bit IDs (TODO)
uint8_t ESIbit;					// store ESI bit, maybe useless or useful to double check
uint8_t BRSbit = 0;				// store BRS bit, used in CAN FD
uint8_t CRC_LEN;				// CRC length (15 for Classical CAN, 17 or 21 for FD)
uint8_t CRC_STUFF;				// CRC stuff bit counter (FD version)
uint8_t stuffedBit = 0;			// boolean value used to say if the current bit is a stuff bit
uint8_t CurrentID[11];			// array to store the current frame's ID
uint8_t currentDLC[4];			// array to store the current Data Length Code
uint8_t DL;						// variable computed from currentDLC
uint8_t currentPayload[512];	// array to store the current Payload (not really useful)
int byteSize;					// temporary variable to store the DLC code integer translation
//volatile int sw = 0;
volatile uint8_t myRxFifo[MAX_SIZE];	// FIFO where the IRS stores the read bits
uint8_t myInt = 255;
volatile int readIndex = 0;				// index of the last bit read and then written in myRxFifo
volatile int processIndex = 0;			// index of the next bit to process from myRxFifo
int p = 7;								// actual index used to take the value from myRxFifo
int r = 0;								// actual index used to store the value in myRxFifo
volatile int FIRST = 1;					// flag to signal it's the first frame, set to 0 when we read the first SOF bit
int newP = 0;
int newPI = 0;
uint8_t bitStream[MAX_SIZE];
uint8_t dataStream[MAX_SIZE];
int currentID = 0;
int i = 0;
int tot = 0;
int diff;
int id_array[512];
int protectedID = 0x12;
uint32_t start_time = 0;
uint32_t end_time = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void SPI_Config(void);
void SPI_GPIO_Config(void);
void LED_GPIO_Config(void);

void SPI_CS_Enable(void);
void SPI_CS_Disable(void);
void SPI_Enable(void);
void SPI_Disable(void);

void LED_GPIO_On(void);
void LED_GPIO_Off(void);

void UART_Send_FIFO(int,int);

void FSM_Init(void);
void fix_index(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint16_t myTxData = 0xc0;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SPI_Config();
  SPI_GPIO_Config();
  LED_GPIO_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // enable RXNE interrupt from SPI peripheral
  NVIC_EnableIRQ(SPI1_IRQn);
  HAL_Delay(10);

  // write into TXFIFO - keep everything recessive
  SPI1->DR = 0xffff;


  // start Reading MISO by enabling SPI peripheral and CS pin
  SPI1->CR1 |= (1<<6);				// SPI Peripheral Enable
  GPIOA->BSRR |= (1<<4)<<16;		// CS Enable

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//	  // Measure start time
//	      start_time = HAL_GetTick();
	  // wait until there is something to process in my FIFO
	  while(processIndex >= readIndex){}


	  // ------------------ FIFO INDEX NORMALIZATION ------------------

	  // keep SPI queue full of recessive bits
	  SPI1->DR = 0xffff;


	  if(currentState == IDLE){
		  myInt = myRxFifo[processIndex];

		  if(myInt == 255){		// no need to check (all 1s), check next cell
			  processIndex++;
			  continue;
		  }
		  else{					// i need to get to the exact first 0 subbit, all 0s included

			  bit = (myInt >> p) & 1;	// take the first bit from left
			  while(bit != 0){
				  p--;					// shift position
				  bit = (myInt >> p) & 1;	// get new subbit
			  }
			  p -= START_INDEX;				// when OK, fix index

			  bit = (myInt >> p) & 1;		// take the bit (mmmh)

			  // need to check if it is a real SOF or a single sample failure
			  fix_index();


		  }
	  }
	  // ------------------ UPDATE COUNTERS AND CHECK ERRORS ------------------

//	  // fix p and processIndex if p is > 7
	  fix_index();

	  bit = (myRxFifo[processIndex] >> p) & 1;	// take the actual bit
	  // Print each bit
//	  sprintf(data, "BIT %d = %d\r\n", i, bit);
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);    // debug
	  bitStream[i] = bit;
	  i++;

	  BC++;
	  // reset stuff bit flag
	  stuffedBit = 0;

	  // Polarity Count update
	  if(bit == previousBit)
		  PC++;

	  else{
		  if(PC == 4 && currentState >= ID && currentState <= CRC_CAN){	// if I have 5 identical bit and now a different one, this is the stuffed one
		  	  stuffedBit = 1;	// I should ignore this bit
		  	  STUFF++;			// STUFF count update
	  	  }
	  	  PC = 0;
	  }
	  previousBit = bit;

	  // in states where bit stuffing rule is applied -> from ID to CRC_CAN (FD version is handled below)

	  if(PC == 5 && currentState >= ID && currentState <= CRC_CAN) // BIT STUFFING rule exception
	  		currentState = ERRORFLAG;


	  //	   ------------------ FSM TRANSITIONS AND STATE UPDATE ------------------
	  //	   if the bit I'm reading is not the stuff bit itself OK, otherwise I need to ignore it
	  if(!stuffedBit){

		switch(currentState){

			case IDLE:

				if(bit == 0)	// change state if dominant (SoF), nothing otherwise
	   			currentState = ID;
	   		break;

	  	  	case ID:
	  	  		currentID = (currentID << 1) + bit;
	  	  		CurrentID[BC-2-STUFF] = bit;	// save ID bit by bit

	  			if(BC == 12+STUFF)		// if ID is finished go to RTR
	  			{
	  				currentState = RTR;
	  				id_array[tot] = currentID;

	  		        if (currentID== 18) {
	  		        	LED_GPIO_On();

	  		        	GPIOA->BSRR |= (1<<4)<<16;
	  		        	    // Wait until the transmit buffer is empty
	  		        	    while ((SPI1->SR & SPI_SR_TXE) == 0);
	  		        	    while ((SPI1->SR & SPI_SR_BSY) != 0) {
	  		        	    		SPI1->DR = 0xc0;
	  		        	    		if (SPI1->DR == myTxData)
	  		        	    		{
	  		        	    			char bitStr[18];
	  		        	    			snprintf(bitStr, sizeof(bitStr), "Data sent: %x \r\n", myTxData);
	  		        	    			HAL_UART_Transmit(&hlpuart1, (uint8_t*)bitStr, strlen(bitStr), HAL_MAX_DELAY);
//	  		        	    			end_time = HAL_GetTick();
//	  		        	    	  	  	uint32_t time_diff = end_time - start_time;
//	  		        	    	  	  	sprintf(data, "Time_Elappsed = %lu\r\n", time_diff);
//	  		        	    	  	  	HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
	  		        	    			HAL_Delay(1000);
	  		        	    		    SPI1->DR = 0xffff;

	  		        	    			SPI1->SR &= ~SPI_SR_TXE;
	  		        	    			GPIOA->BSRR |= (1<<4);
	  		        	    			break;
	  		        	    		}

	  		        	       }
//	  		        	    SPI1->DR = 0xffff;
//
//	  		        	    SPI1->SR &= ~SPI_SR_TXE;
//	  		        	    GPIOA->BSRR |= (1<<4);
//	  		        	  break;
	  		        }


	  				sprintf(data, "ID = %d\r\n", currentID);
	  				HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
	  				LED_GPIO_Off();


	  			}
	  	  	break;



	    	case ERRORFLAG:
	    		if (tot == 0)
	    		{
	  	  			//UART_Send_FIFO(processIndex,readIndex);

	    			//sprintf(data, "ERROR %d\r\n",tot);
	  				//HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
	    		}
	  				// need to handle the case where an error is detected during DATA, transmitted at ARB
	    		if(BRSbit == 1){
	    			//UART_Send_FIFO(processIndex,readIndex);

	    			// skip all redundant subbits, deactivate BRS and go to the next bit at the next iteration
	    			p -= SKIP_SIZE;
	    			BRSbit = 0;
	    			continue;
	    		}

	    		if(bit == 0){
	    			formError = 0;
	    			DC++;
	    			if((DC % 8 == 0) && (DC > 0))
	    			{	// exception (6)
	    				//currentState = ERRORDEL;
	    	  			if (currentID == protectedID)
	    	  				TEC += 8;
	    					//sprintf(data, "Exception 6\r\n");
	    					//HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
	    			}
	    		} else
	    		{	// start of Error Delimiter
	    			RC++;
	    			currentState = ERRORDEL;
	    			if (currentID == protectedID)
	    				TEC += 8;
	  					//sprintf(data, "Error Delimiter, TEC = %d, count(0) = %d, count(1) = %d\r\n", TEC, DC, RC);
	  					//HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, strlen(data), 1000);
	    		}
	    	break;

	    	case ERRORDEL:
	    		//UART_Send_Bits();

	    		if(bit == 1)
	    			RC++;
	    		if(formError == 1 && RC == 14)
	    			currentState = IF;
	    		if(formError == 0 && RC == 8)
	    			currentState = IF;
	    	break;

	    	case IF:
	    		if(bit == 0)
	    			currentState = OVERFLAG;
	    		else{
	    			IC++;
	    			if(IC == 3){
	    				tot++;


	    		  		// reset everything but TEC
	    		  		//FSM_Init();
	    				BC = 0;
	    				PC = 0;
	    				CC = 0;
	    				STUFF = 0;
	    				DL = 0;
	    				formError = 0;
	    				CRC_LEN = 0;
	    				CRC_STUFF = 0;
	    				IC = 0;
	    				RC = 0;
	    				DC = 0;
	    				currentState = IDLE;
	    				currentID = 0;
	    				p = 7;
	    			continue;
	    			}
	    		}
	    	break;
	    		}
	    	}

//	 bit processed, update the index
	 if(BRSbit == 1)
		 p -= DATA_SIZE;		// need to read "sub bits" in data phase
	 else
		 p -= NOM_SIZE;		// skip "sub bits" until the new bit



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UCPD_DBN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UCPD_DBN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = UCPD_DBN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void SPI1_IRQHandler(void) {
	/* RXNE Interrupt Callback */

	/* --------------------------- 1 bit at a time, save all 8 subbits ---------------------------------	*/
	//This checks if the RXNE flag in the SPI status register (SR) is set, indicating the receive buffer is not empty
	if(readIndex== MAX_SIZE || processIndex== MAX_SIZE){
		readIndex = 0;
		processIndex = 0;
		}
	if ((SPI1->SR & SPI_SR_RXNE) != RESET) {

		if (!(readIndex > 15 && readIndex < 18))
			SPI1->DR = 0xffff;

	// reads the received data from the SPI data register (DR)
	RxData = *(volatile uint8_t *)&SPI1->DR;

	//check if the received data is not 255 (RxData == 255) when FIRST is true
	if(!(FIRST && (RxData == 255))){
		FIRST = 0; // it is to avoid idle bits at the beginning

		if (readIndex > 15 && readIndex < 18){

			 SPI1->DR = 0xffff;
			//UART_Send_FIFO(0,readIndex);
		}


		myRxFifo[readIndex] = RxData;	// save int in my fifo
		readIndex++;					// Update index

	}
	if (!(readIndex > 15 && readIndex < 18))
		SPI1->DR = 0xffff;

	}
	 // clears the RXNE flag to acknowledge that the receive buffer is now empty
	 SPI1->SR &= ~SPI_SR_RXNE;

}




void SPI_Config(void){
	/* Configure SPI Peripheral */

	RCC->APB2ENR |= (1<<12); 		// Enable SPI1 Peripheral Clock

	// setting bits to 0 is quite useless, I could set all CR1 to 0 and then set needed bits to 1
	// CPOL = 0 and CPHA = 0, default
	SPI1->CR1 |= (1<<1);			// CPOL = 1
	SPI1->CR1 |= (1<<2);			// MSTR = 1, Master mode
	SPI1->CR1 |= (6<<3);			// BR[2:0] = 110 -> 128, 	clk_SPI = 64 MHz/128 = 	500 kHz
	//SPI1->CR1 |= (5<<3);			// BR[2:0] = 101 -> 64, 	clk_SPI = 64 MHz/64 = 	1 MHz
	//SPI1->CR1 |= (4<<3);			// BR[2:0] = 100 -> 32, 	clk_SPI = 64 MHz/32 = 	2 MHz
	//SPI1->CR1 |= (3<<3);			// BR[2:0] = 4 MHz
	SPI1->CR1 &= ~(1<<7);			// LSBFIRST = 0, MSB first
	SPI1->CR1 |= (1<<8) | (1<<9); 	// SSM = 1 and SSI = 1, CS software management
	SPI1->CR1 &= ~(1<<10);			// RXONLY = 0, full-duplex

    SPI1->CR2 = SPI_CR2_FRXTH | SPI_CR2_RXNEIE; // set RX Threshold to 1 (8 bit = Not Empty) and Enable RXNE Interrupt
    // DS will be forced to 8bit = 0111

    //SPI1->CR2 = 0;					// all 0, DS will be forced to 8bit = 0111
}

void SPI_GPIO_Config(void){
	/* Configure SPI Pins - CLK, MOSI, MISO, CS */

	/********************************************/
	/* CLK		->	PA5			Alternate		*/
	/* MISO 	->	PA6 		Alternate	  	*/
	/* MOSI 	->	PA7			Alternate		*/
	/* CS 		->	PA4			Output			*/
	/********************************************/

	RCC->AHB2ENR |= (1<<0); 		// Enable GPIOA Peripheral Clock

	// MODER has reset state = 3, Alt = 2 = 10 and Out = 1 = 01
	GPIOA->MODER &= ~(1<<10) & ~(1<<12) & ~(1<<14) & ~(2<<8);	// PA5,PA6,PA7 in Alternate while PA4 in Output
	GPIOA->OSPEEDR |= (3<<10) | (3<<12) | (3<<14) | (3<<8);		// All HIGH-SPEED = 11
	GPIOA->AFR[0] |= (5<<20) | (5<<24) | (5<<28);				// PA5,PA6,PA7 with AF5 (SPI)
	GPIOA->BSRR |= (0<<5)|(1<<6)|(1<<7)|(1<<4);					// CLK is low when idle, other pins are high
	//GPIOA->BSRR |= (1<<4)<<16;		// CS Enable
}

void LED_GPIO_Config(void){
	/* Configure LED_Green Pin					*/

	/********************************************/
	/* LED_Green		->	PC7					*/
	/********************************************/

	RCC->AHB2ENR |= (1<<2); 		// Enable GPIOC Peripheral Clock

	GPIOC->MODER &= ~(2<<14);		// Output mode for PC7
}

void SPI_CS_Enable(void){
	/* Enable CS Pin by RESETTING it */

	GPIOA->BSRR |= (1<<4)<<16;
}

void SPI_CS_Disable(void){
	/* Disable CS Pin by SETTING it*/

	GPIOA->BSRR |= (1<<4);
}

void SPI_Enable(void){
	/* Enable SPI Peripheral */

	SPI1->CR1 |= (1<<6);			// SPE = 1
}

void SPI_Disable(void){
	/* Disable SPI Peripheral */

	SPI1->CR1 &= ~(1<<6);			// SPE = 0
}

void LED_GPIO_On(void){
	/* Enable LED_Green Pin */

	GPIOC->BSRR |= (1<<7);
}

void LED_GPIO_Off(void){
	/* Disable LED_Green Pin */

	GPIOC->BSRR |= (1<<7)<<16;
}


void FSM_Init(){
	BC = 0;
	PC = 0;
	CC = 0;
	STUFF = 0;
	DL = 0;
	formError = 0;
	CRC_LEN = 0;
	CRC_STUFF = 0;
	IC = 0;
	RC = 0;
	DC = 0;
	currentState = IDLE;
}

void fix_index(){
	while(p < 0){
		p += 8;
		processIndex++;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
