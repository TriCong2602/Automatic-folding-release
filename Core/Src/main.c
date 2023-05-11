/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
volatile char stop,i,j;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	RCC->AHB1ENR |=(1<<0);
	GPIOA->MODER |=(1<<(2*7));
	GPIOA->OSPEEDR |= (0b01<<(2*7)); // medium speed
	GPIOA->OTYPER = 0x0000;
	//KHAI BAO NGAT D11
	RCC->APB2ENR |= (1<<14); //clock EXTI quy dinh la 14
	RCC->AHB1ENR |=(1<<3);
	GPIOD->MODER &= ~(3<<(2*11)); //analog mode
	GPIOD->OSPEEDR |= (1<<(2*11));
	GPIOD->PUPDR |= (1<<(2*11)); //pull-up
	SYSCFG->EXTICR[2] |= (3<<12); //EXTI11
  EXTI->IMR |= (1<<11); //disable mask line 11	
  EXTI->FTSR &= ~(1<<11);
	EXTI->RTSR |= (1<<11);
	NVIC->ISER[1] |= (1<<8); //enable interrupt EXTI11
	//xy lanh M1,M2,M3
	RCC->AHB1ENR |=(1<<4);
	GPIOE->MODER |=(1<<14)|(1<<16)|(1<<18)|(1<<20);
	GPIOE->OSPEEDR |= (1<<14)|(1<<16)|(1<<18)|(1<<20); // medium speed
	GPIOE->OTYPER = 0x0000;	
	
	RCC->AHB1ENR |= (1<<1);
	GPIOB->MODER &= ~(3<<6)|~(3<<8)|~(3<<10);
	GPIOB->OSPEEDR =(1<<6)|(1<<8)|(1<<10);
	GPIOB->PUPDR = (1<<6)|(1<<8)|(1<<10);
	
	RCC->AHB1ENR |= (1<<3);
	GPIOD->MODER &= ~(3<<14)|~(3<<26);
	GPIOD->OSPEEDR |= (1<<14)|(1<<26);
	GPIOD->PUPDR |= (1<<14)|(1<<26);
	
	//KHAI BAO NGAT A15
	
	RCC->AHB1ENR |=(1<<0);
	GPIOA->MODER &= ~(3<<(2*15)); //analog mode
	GPIOA->OSPEEDR |= (1<<(2*15));
	GPIOA->PUPDR |= (1<<(2*15)); //pull-up
	SYSCFG->EXTICR[2] |= (3<<12); //EXTI11
  EXTI->IMR |= (1<<15); //disable mask line 11	
  EXTI->FTSR &= ~(1<<15);
	EXTI->RTSR |= (1<<15);
	NVIC->ISER[1] |= (1<<8); //enable interrupt EXTI11
	//KHAI BAO NGAT C6
	RCC->AHB1ENR |=(1<<2);
	GPIOC->MODER &= ~(3<<(2*6))|~(3<<(2*7)); //analog mode
	GPIOC->OSPEEDR =(0b01<<(2*6))|(0b01<<(2*7));
	GPIOC->PUPDR |= (1<<(2*6))|(1<<(2*7));

  SYSCFG->EXTICR[1] |= (2<<8)|(2<<12); //EXTI6 // chi lay cai duoi thoi 8,12
  EXTI->IMR |= (1<<6)|(1<<7); //disable mask line 6,7	
  EXTI->FTSR &= ~(1<<6)|~(1<<7); // clear falling
	EXTI->RTSR |= (1<<6)|(1<<7); //
	NVIC->ISER[0] |= (1<<23); //
	//KHAI BAO NGAT E14
	RCC->AHB1ENR |=(1<<4);
	GPIOE->MODER &= ~(3<<(2*14)); //analog mode
	GPIOE->OSPEEDR |= (0b01<(2*14));
	GPIOE->PUPDR |= (1<<(2*14));
	SYSCFG->EXTICR[3] |= (4<<8);
	EXTI->IMR |=(1<<14);
	EXTI->FTSR &= ~(1<<14);
	EXTI->RTSR |= (1<<14);
	NVIC->ISER[1] |= (1<<8); //enable interrupt EXTI14
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
   GPIOE->BSRR |= (1<<7)|(1<<8)|(1<<25)|(1<<10);  //set trang thai ban dau
	 GPIOA->ODR = (1<<7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(i==1){
		if ((GPIOA->ODR & (1<<7))==0){
		while((GPIOD->IDR & (1<<13))== 0&&stop==1&&i==1);
		if((GPIOD->IDR &(1<<13))==1<<13) // cam bien ben trai M3
		{
			GPIOE->BSRR |= (1<<7)|(1<<8)|(1<<9)|(1<<26);	
		}
		while((GPIOD->IDR & (1<<7))== 1<<7&&stop==1&&i==1);
		if ((GPIOD->IDR &(1<<7))==0 &&stop==1&&i==1) // cam bien ben phai M3
		{
			GPIOE->BSRR |= (1<<24)|(1<<26);	
     	//HAL_Delay(3000);
		}
		while((GPIOB->IDR & (1<<4))== 0&&stop==1);
		if((GPIOB->IDR &(1<<4))==1<<4  &&stop==1) // cam bien ben duoi M2
		{ 
			GPIOE->BSRR |= (1<<23)|(1<<24);
			HAL_Delay(500);
			GPIOE->BSRR |= (1<<23)|(1<<8);
		//	HAL_Delay(100);
		}
		
		
		
		while((GPIOB->IDR & (1<<5))== 0&&stop==1);
		if((GPIOB->IDR &(1<<5))==1<<5  &&stop==1) // cam bien ben phai M3
		{
			GPIOE->BSRR |= (1<<23)|(1<<8)|(1<<25)|(1<<10);
		}
		while((GPIOB->IDR & (1<<3))== 1<<3&&stop==1);
		if((GPIOB->IDR &(1<<3))==0  &&stop==1) // cam bien ben trai M3
		{
			GPIOE->BSRR |= (1<<23)|(1<<24)|(1<<25)|(1<<10);	
			
		}
		while((GPIOB->IDR & (1<<4))== 0&&stop==1);
		if((GPIOB->IDR &(1<<4))==1<<4  &&stop==1) // cam bien ben phai M3
		{
			GPIOE->BSRR |= (1<<7)|(1<<24)|(1<<25)|(1<<10);
			HAL_Delay(500);
			GPIOE->BSRR |= (1<<7)|(1<<8)|(1<<25)|(1<<10);
			HAL_Delay(1000);
			
			
		}
		
     }
		if(stop==0&&i==0){GPIOE->BSRR |= (1<<8)|(1<<25)|(1<<10);}		
		}
//		else if(i==0){
//      if ((GPIOD->IDR &(1<<11))==0){
//			HAL_Delay(20);
//			if ((GPIOD->IDR &(1<<11))==0){
//		GPIOE->BSRR |= (1<<(10+16))|(1<<9);
//		}}
//		if ((GPIOC->IDR &(1<<6))==0){
//			HAL_Delay(20);
//			if ((GPIOC->IDR &(1<<6))==0){
//		GPIOE->BSRR |= (1<<(9+16))|(1<<10);
//		}}
////		if ((GPIOC->IDR &(1<<7))==0){
////			HAL_Delay(20);
////			if ((GPIOC->IDR &(1<<7))==0){
////		GPIOE->BSRR |= (1<<(8+16));
////		}}
////		if ((GPIOE->IDR &(1<<14))==0){
////			HAL_Delay(20);
////			if ((GPIOE->IDR &(1<<14))==0){
////		GPIOE->BSRR |= (1<<(8));
////		}}
//	}
//      if(stop==0){GPIOE->BSRR |= (1<<8)|(1<<25)|(1<<10);}		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void EXTI15_10_IRQHandler(void)
	{
	 if(EXTI->PR & (1<<11))
		{
				//GPIOB->ODR &= ~(1<<0); // dao trang thai cua B cho no chay
			  
			  
				GPIOA->ODR ^= (1<<7);
				if(stop<1){stop++;}
				else {stop=0;}
			
			  
		
		}
		else if(EXTI->PR & (1<<14)){
		
		    if ((GPIOA->IDR & (1<<15))==0){
				if(j<1){j++;
				GPIOE->BSRR |= (1<<25);}
				else {j=0;GPIOE->BSRR |= (1<<26);}}
		    
		    
	}
		
		else if(EXTI->PR & (1<<15)){
		    if(i<1){i++;}
				else {i=0;}
		}
		
	EXTI->PR |= (1<<11)|(1<<15);
}
void EXTI9_5_IRQHandler(void){
	if(EXTI->PR & (1<<6)){
		
		    if ((GPIOA->IDR & (1<<15))==0){
				GPIOE->ODR ^= (1<<7);}
		    
		    
	}
	else if(EXTI->PR & (1<<7)){
				if ((GPIOA->IDR & (1<<15))==0){
				GPIOE->ODR ^= (1<<8);}
			}
	EXTI->PR |= (1<<6)|(1<<7);
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
