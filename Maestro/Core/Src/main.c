/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"
#include "funciones.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "identificador.h"
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
extern int contador_instrucciones;
extern char str[50];
extern int flag_mensaje_completo;
volatile int flag_INT = 1;
uint8_t pRxData = 'K';
enum Estado {
	Activado, Desactivado, Modo_Homing, Modo_Normal, Error
};
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
int main(void) {
	/* USER CODE BEGIN 1 */
	int cant = 0;
	double instrucciones[50];
	//Flags
	int flag_activacion = 0;
	int flag_homing = 0, flag_cambio = 0;
	//Variables enum
	enum Estado estado = Desactivado;
	//HAL_StatusTypeDef SPI_estado;
	int comando;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USB_DEVICE_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */
	SPI_Transmit_1('D');
	SPI_Transmit_1('-');
	SPI_Transmit_1('P');
	SPI_Transmit_1(':');
	Mi_Timer();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//Generar string
		//Generar comandos
		if (flag_mensaje_completo == 0) {
			cant = identificador(str, instrucciones, contador_instrucciones);
			flag_mensaje_completo = 1;
			contador_instrucciones = 0;
		}
		//Identificar comandos
		if (flag_mensaje_completo == 1) {
			for (int i = 0; i < cant; i++) {
				comando = (int) instrucciones[i];
				switch (comando) {
				case Modo_desactivado:
					if (flag_activacion) {
						SPI_Transmit_1('D');
						SPI_Transmit_1('-');
						SPI_Transmit_1('P');
						SPI_Transmit_1(':');
						Mi_Timer();
						if (pRxData == 'D') {
							estado = Desactivado;
							flag_activacion = 0;
							flag_homing = 0;
							flag_cambio = 1;

						} else {
							i--;
						}
					}
					break;
				case Modo_activado:
					if (!flag_activacion) {
						SPI_Transmit_1('A');
						SPI_Transmit_1('-');
						SPI_Transmit_1('P');
						SPI_Transmit_1(':');
						Mi_Timer();
						if (pRxData == 'A') {
							estado = Activado;
							flag_activacion = 1;
							flag_cambio = 1;
						} else {
							i--;
						}
					}
					break;
				case Modo_homing:
					if (flag_activacion) {
						//Prender un LED
						//Mandar consigna de homing
						SPI_Transmit_1('H');
						SPI_Transmit_1('-');
						//Verificar consigna
						SPI_Transmit_1('P');
						SPI_Transmit_1(':');
						Mi_Timer();
						if (pRxData == 'H') {
							estado = Modo_Homing;
							flag_homing = 1;
							flag_cambio = 1;
						} else {
							i--;
						}
					}
					break;
				case Cin_dir:
					if (flag_activacion && flag_homing) {
						Cin_Dir(instrucciones, i);
						Mi_Timer();
						//Verificar que se logre
						if (pRxData == 'N') {
							estado = Modo_Normal;
							flag_cambio = 1;
						} else {
							i--;
						}
					}
					i += 5;
					break;
				case Cin_inv:
					i += 5;
					break;
				case Resumen:
					break;
				}
				pRxData = 'K';
			}
			flag_mensaje_completo = 2;
			cant = 0;
		}
		switch (estado) {
		case Activado:
			if (flag_cambio) {
				Leds_Activado();
				flag_cambio = 0;
			}
			break;
		case Desactivado:
			if (flag_cambio) {
				//Prender LED
				Leds_Desactivado();
				flag_cambio = 0;
			}
			break;
		case Modo_Homing:
			if (flag_cambio) {
				//Prender LED
				Leds_Homing();
				flag_cambio = 0;
			}
			break;
		case Modo_Normal:
			if (flag_cambio) {
				//Prender LED
				Leds_Normal();
				flag_cambio = 0;
			}
			break;
		case Error:
			Leds_Error();
			break;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/*Va a haber dos funciones slave transmit,
 una para cada esclavo, va a ser una sola interrupcion externa por placa
 La gestion de cada motor lo hace cada esclavo.
 En el struct tenemos que tener: Posicion actual, posicion deseada, dirección
 deseada, etc.*/
//Esta funcion la tenemos que hacer contemplando el modo de trabajo.
void SPI_Transmit_1(uint8_t pTxData) {

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
}
void SPI_Transmit_2(uint8_t pTxData) {

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); //PIN NSS Esclavo 2 Bajo
	//HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//PIN NSS Esclavo 2 Alto
}
void Mi_Timer() {
	/*	long contador = 0;
	 flag_INT = 1;
	 switch (estado) {
	 case 'A':
	 while (flag_INT) {
	 contador++;
	 if (contador == 1000000) {
	 pRxData = 'K';
	 break;
	 }
	 }
	 break;
	 case 'D':
	 while (flag_INT) {
	 contador++;
	 if (contador == 1000000) {
	 pRxData = 'K';
	 break;
	 }
	 }
	 break;
	 case 'H':
	 while (flag_INT);
	 break;
	 case 'N':
	 while (flag_INT);
	 break;
	 }
	 */
	while (flag_INT)
		;
	flag_INT = 1;

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static uint8_t D_transmision = ':';
	flag_INT = 0;
	switch (GPIO_Pin) {
	case GPIO_PIN_8: 	//INT 1 Esclavo 1 (2 motores)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi2, &D_transmision, &pRxData, 1, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		break;
	case GPIO_PIN_9: 	//INT2 Esclavo 2
		break;
	}

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
