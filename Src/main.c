/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#define LED_R 					1
#define LED_R_PORT			GPIOA
#define LED_B						4
#define LED_B_PORT			GPIOA
#define LED_W						5
#define LED_W_PORT			GPIOA

#define MODE0_INPUT		 	 0
#define MODE1_OUT_10MZ   1
#define MODE2_OUT_02MZ   2
#define MODE3_OUT_50MZ   3

#define CONF_OUT_PUSH_PULL 		0
#define CONF_OUT_OPEN_DRAIN		1
#define CONF_AF_PUSH_PULL 		2
#define CONF_AF_OPEN_DRAIN 		3
#define CONF_INPUT_ANALOG     0
#define CONF_INPUT_FLOAT			1
#define CONF_INPUT_PULL_DWN_PULL_UP 2  // USE ODR reg for setting PULL Up and PULL Down 

#define ENABLE_PORTA  RCC->APB2ENR  |= (1<<2)
#define ENABLE_PORTB  RCC->APB2ENR  |= (1<<3)
#define ENABLE_PORTC	RCC->APB2ENR  |= (1<<4)
#define FCLOCK   8000000
#define BAUDRATE  9600 
#define PORT_A 					GPIOB
#define PIN_A    			    3
#define PORT_B 					GPIOB
#define PIN_B   			    4
#define PORT_C 					GPIOB
#define PIN_C    			    5
#define PORT_D 					GPIOB
#define PIN_D    			    6
#define PORT_E 					GPIOB
#define PIN_E   			    7
#define PORT_F 					GPIOB
#define PIN_F    			    8
#define PORT_G 					GPIOB
#define PIN_G    			    9
#define PORT_DP 					GPIOB
#define PIN_DP    			    10



void GPIO_CONFIG(GPIO_TypeDef * Port ,char Pin ,char Conf ,char Mode )
{				
	if(Port == GPIOA)
	{
		ENABLE_PORTA;
	}
	else if(Port == GPIOB)
	{
		ENABLE_PORTB;
	}
	else if(Port == GPIOC)
	{
		ENABLE_PORTC;
	}
	if(Pin <= (char)7)	
	{ 
		Port->CRL |=    (Conf<<2 | Mode) << (Pin * 4); 
		Port->CRL &= ~(((~(Conf<<2 | Mode)) & 0x0F) << (Pin * 4)); 	
	} 			
	else
	{ 
		Pin -= 8;
		Port->CRH |= 		(Conf<<2 | Mode) << (Pin * 4); // setting bits 
		Port->CRH &= ~(((~(Conf<<2 | Mode)) & 0x0F) << (Pin * 4)); // clear bit 
	}	
				
}
#define STATE_HIGH		1
#define STATE_LOW			0
void GPIO_Write(GPIO_TypeDef * Port ,char Pin , char State)
{
	if( State == STATE_HIGH)
	{
		Port->BSRR = 1<< Pin;
	}
	else if( State == STATE_LOW)
	{
		Port->BRR = 1<< Pin ; 
	}
}

int GPIO_Read(GPIO_TypeDef * Port ,char Pin )
{
	return ((Port->IDR >> Pin ) & 1);
}
/*char * getstring(void)
{
	int i;
	char ch;
	char str[10];
	for(i=0;(ch= UART1_RX())!='\n'&(ch= UART1_RX())!=EOF);i++)
	{
		str[i]=ch;
	}
	return str;
}	*/
int stringcmp(const char*data,const char*str)
{	
			int i=0;
			while(str[i]&&data[i])
			{
				if (str[i]==data[i])
					i++;
				else
					break;
			}
		return str[i] - data[i];
}
int getclick(void)
{
	int count = 0;
	volatile unsigned long time_out;
	
	while((GPIOA->IDR >> 8 & 1) == 1);
	HAL_Delay(5);
	while(1)
	{
		time_out = 15;
		while(((GPIOA->IDR >> 8 & 1) == 1) && --time_out) // when not pressed 
		{
			HAL_Delay(10);
		}
		if(0 == time_out)
		{
			return count;
		}
		HAL_Delay(5);
		while((GPIOA->IDR >> 8 & 1) == 0); // when pressed 
		HAL_Delay(5);
		count++;
	}
	
	
}

char UART1_RX()
{
	// for rx
	while((USART1->SR & USART_SR_RXNE_Msk) != USART_SR_RXNE_Msk);
	return (USART1->DR);
}

void UART1_TX(char data)
{
	// for tx 
		USART1->DR  = data;
		while(!(USART1->SR & USART_SR_TC_Msk));
}
char UART2_RX() // blocking calls 
{
	// for rx
	while((USART2->SR & USART_SR_RXNE_Msk) != USART_SR_RXNE_Msk);
	return (USART2->DR);
}

void UART2_TX(char data)
{
	// for tx 
		USART2->DR  = data;
		while(!(USART2->SR & USART_SR_TC_Msk));
}

int Uart_Rx_NonBlocking(USART_TypeDef * uart,char *Data)
{
	if((uart->SR & USART_SR_RXNE_Msk) == USART_SR_RXNE_Msk) // data reday 
	{
		*Data = uart->DR;
		return 1;
	}
	else // Data not ready 
	{
		return 0;
	}
}

int Baudrateset(float clock, float Baudrate)
{
	float USARTDIV;
	float v;
	USARTDIV=(clock/(Baudrate*16));
	v = USARTDIV-(int)USARTDIV;
	v*=16;
	return ((int)USARTDIV<<4  | ((int)v &0x0F));

}
void Uart_str()
{
	int i;
	char str[]={"Error\n\r"};
	for (i=0;str[i];i++)
	{
		UART2_TX(str[i]);
	}
	
}
void Display(char a)
{
	int i;
	char disp[10]={0x03,0x9F,0x25,0x0D,0x99,0x49,0x41,0x1F,0x01,0x09};
	if (!((a>='0')&&(a<='9')))
	{
		Uart_str();
	}	
	for(i=0;i<8;i++)
	{
		if ((disp[a-'0']>>i)&1)
	{
		GPIO_Write(PORT_DP,i+3,STATE_HIGH);
	}	
	else 
	{
		GPIO_Write(PORT_DP,i+3,STATE_LOW);
	}	
}

	
}
int main(void)
{
  //volatile int click;
	//int i;
	char data;
//	char str[20];
//	enum Led_Enum {
//		Red = 0,
//		Blue,
//		White,
//		End,
//	};
//	const char *led[End]= {"LED R ON\r\n",
//											   "LED B ON\r\n",
//											   "LED W ON\r\n" };
  HAL_Init();
  SystemClock_Config();
  //MX_GPIO_Init();
	
	GPIO_CONFIG( PORT_A , PIN_A , CONF_OUT_PUSH_PULL , MODE2_OUT_02MZ );
	GPIO_CONFIG( PORT_B , PIN_B , CONF_OUT_PUSH_PULL , MODE2_OUT_02MZ );
	GPIO_CONFIG( PORT_C , PIN_C , CONF_OUT_PUSH_PULL , MODE2_OUT_02MZ );
	GPIO_CONFIG( PORT_D , PIN_D , CONF_OUT_PUSH_PULL , MODE2_OUT_02MZ );
	GPIO_CONFIG( PORT_E , PIN_E , CONF_OUT_PUSH_PULL , MODE2_OUT_02MZ );
	GPIO_CONFIG( PORT_F , PIN_F , CONF_OUT_PUSH_PULL , MODE2_OUT_02MZ );
	GPIO_CONFIG( PORT_G , PIN_G , CONF_OUT_PUSH_PULL , MODE2_OUT_02MZ );
	GPIO_CONFIG( PORT_DP , PIN_DP , CONF_OUT_PUSH_PULL , MODE2_OUT_02MZ );
	

	/* UART1 Config Start */
	ENABLE_PORTA;
	ENABLE_PORTB;
	GPIOA->CRH |= 1<<7 | 1<< 11 | 1<<4 ; //Setting GPIOA9 and 10
	GPIOA->CRH &= ~( 1<<5 | 1<<6 | 1<<8 | 1<<9 | 1<<10) ;
  
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
	USART1->BRR = Baudrateset(FCLOCK , BAUDRATE);
	USART1->CR1 = ( USART_CR1_UE | USART_CR1_TE | USART_CR1_RE );
	/* UART 1 Config End  */
												
	/* UART2 Config Start */
	GPIOA->CRL |= 1<<9 | 1<<15 | 1<<11 ; //Setting GPIOA2 and 3
	GPIOA->CRL &= ~( 1<<8 | 1<<10 | 1<<12 | 1<<13 | 1<<14) ;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	
	USART2->BRR = 0x341;
	USART2->CR1 = ( USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
  HAL_NVIC_SetPriority(USART2_IRQn, 15, 0); // 15 is lowest priority 
	HAL_NVIC_EnableIRQ(USART2_IRQn); // For enabling Interupt 
	
	/* UART 2 Config End  */
  while (1)
  {		
//	for rx
//	i=0;		
//	do
//	{
		int i =0 ;
		while(1)
		{
			//data = UART2_RX();
			//UART2_TX(data);
			HAL_Delay(500);
			Display(i+'0');
			i++;
			if(i>=10)
			{
				i =0;
			}
		}
//		}
//		if(Uart_Rx_NonBlocking(USART2,&data))
//		{
//			UART1_TX(data);
//		}
//		if(Uart_Rx_NonBlocking(USART1,&data))
//		{
//			UART2_TX(data);
//		}
			
//		str[i]=data;
//		if (data=='\b'&& (i>0))
//		{
//			i--;
//		}
//		else
//		{
//			i++;
//		}
//	}while(data!='\n');
//	str[i] = '\0';
//		
//		GPIO_Write(LED_R_PORT,LED_R,STATE_LOW);
//		GPIO_Write(LED_B_PORT,LED_B,STATE_LOW);
//		GPIO_Write(LED_W_PORT,LED_W,STATE_LOW);
//		if (stringcmp(led[Red],str)==0)
//		    GPIO_Write(LED_R_PORT,LED_R,STATE_HIGH);
//		else if(stringcmp(led[Blue],str)==0)
//				GPIO_Write(LED_B_PORT , LED_B, STATE_HIGH);
//		else if(stringcmp(led[White],str)==0)
//				GPIO_Write(LED_W_PORT , LED_W, STATE_HIGH);
//		else
//		{
//				GPIO_Write(LED_R_PORT,LED_R,STATE_HIGH);
//				GPIO_Write(LED_B_PORT,LED_B,STATE_HIGH);
//				GPIO_Write(LED_W_PORT,LED_W,STATE_HIGH);
//		}
//			
//		switch(data)
//		{
//			case 'r':
//			case 'R':
//								GPIO_Write(LED_R_PORT,LED_R,STATE_HIGH);
//			break;
//			
//			case 'b':
//			case 'B':
//								GPIO_Write(LED_B_PORT , LED_B, STATE_HIGH);
//			break;
//			
//			case 'w':
//			case 'W':
//								GPIO_Write(LED_W_PORT,LED_W,STATE_HIGH);
//			break;
//			
//			default:
//								GPIO_Write(LED_R_PORT,LED_R,STATE_HIGH);
//								GPIO_Write(LED_B_PORT,LED_B,STATE_HIGH);
//								GPIO_Write(LED_W_PORT,LED_W,STATE_HIGH);
//		}
		
	}
		
}	
		
		
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		
//		if(!((GPIOA->IDR >>8) & 1))
//			GPIOA->BSRR = 7<<9;
//		else 
//			GPIOA->BSRR = 7<<25;

		
		

		//click = getclick();
		/*
		switch(click - '0')
		{
			case 0:
				   // do nothing 
				break;
			case 1: // Turn on Led 1
							GPIOA->BSRR |= 1<<9;
							GPIOA->BRR  |= 3<<10;
				break;
			case 2: // Turn on Led 2
							GPIOA->BSRR |= 1<<10;
							GPIOA->BRR  |= 5<<9;
				break;
			case 3: // Turn on Led 3
							GPIOA->BSRR |= 1<<11;
							GPIOA->BRR  |= 3<<9;
				break;
			
			default: GPIOA->BSRR = 7<<9;
				
		}
		*/
		//click ++;
		
		
		//HAL_Delay(1000);
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		//HAL_Delay(1000);
  /* USER CODE END 3 */



void USART2_IRQHandler(void)
{
	UART2_TX(USART2->DR);
	return;
}


/** System Clock Configuration **/

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
#if 0
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}
#endif

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
