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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IDENTIFICADOR.h"
#include "Interpolador.h"
#include <math.h>
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
enum Estado {
	Activado, Desactivado, Modo_Homing, Modo_Normal, Error
};
typedef enum {
	M_error, M_listo, M_realizando
} Modos;
typedef struct {
	float p_obj;
	float p_act;
	float p_fin;
	float p_ini;
	float v_cru;
	float tau;
	float a_nes;
	float t_tot;
	float t_act;
	float kp, ki, kd;
	Modos modo;
} motor;

motor m1, m2;
enum Estado estado = Desactivado;
uint8_t dato_recepcion_SPI, pTxData = 0, cnt_lis;
uint8_t flag_mensaje_completo = 3, flag_cambio = 0, flag_cambio2 = 0,
		flag_activacion = 0, flag_homing = 0, flag_m1_listo = 0, flag_m2_listo =
				1;
uint32_t cont_samp = 0, cont_error = 0;
volatile int cont_datos_SPI = 0, contador_instrucciones = 0;
char str[50] = { 0 };
float error_M, error_ant_M = 0, Ui = 0, Ui_ant = 0, Up, Ud, UPID;
double error_pos = 0;

TIM_OC_InitTypeDef PWM_config = { 0 };
//Seteo de kp,ki,kd;
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
	// Declarar variables
	int cant = 0;
	double instrucciones[50] = { };

	int comando;
	float Vmin;

	// init variables
	m1.p_ini = 0;
	PWM_config.OCMode = TIM_OCMODE_PWM1;
	PWM_config.Pulse = 0;
	PWM_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	PWM_config.OCFastMode = TIM_OCFAST_DISABLE;
	flag_activacion = 0;
	flag_homing = 0;
	m1.kp = 280;
	m1.kd = m1.kp / 3896.96;
	m1.ki = m1.kp * 4.818;
	m2.modo = M_listo;
	m1.modo = M_listo;

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	HAL_SPI_Receive_IT(&hspi2, &dato_recepcion_SPI, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//Generar comando
		if (flag_mensaje_completo == 0) {
			cant = identificador(str, instrucciones, contador_instrucciones);
			flag_mensaje_completo = 1;
			contador_instrucciones = 0;
		}
		// identificar comandos
		if (flag_mensaje_completo == 1) {
			for (int i = 0; i < cant; i++) {
				comando = (int) instrucciones[i];
				switch (comando) {
				case Desactivar:
					//if (flag_activacion) {
						HAL_TIM_Base_Stop_IT(&htim9);
						HAL_GPIO_WritePin(L298_ENA1_GPIO_Port, L298_ENA1_Pin,
								GPIO_PIN_RESET);
						PWM_config.Pulse = 0;
						HAL_TIM_PWM_ConfigChannel(&htim12, &PWM_config,
						TIM_CHANNEL_1);
						HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin,
								GPIO_PIN_RESET);
						HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_ALL);
						HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
						estado = Desactivado;
						flag_activacion = 0;
						flag_homing = 0;
						flag_cambio = 1;

					//} else {
					//}
					break;
				case Activar:
					if (!flag_activacion) {
						estado = Activado;
						flag_activacion = 1;
						flag_cambio = 1;
						HAL_GPIO_WritePin(L298_ENA1_GPIO_Port, L298_ENA1_Pin,
								GPIO_PIN_SET);
						HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
					} else {
					}
					break;
				case Modo_homing:
					if (flag_activacion) {
						flag_cambio = 1;
						estado = Modo_Homing;
					}
					break;
				case Modo_ir:
					if (flag_activacion && flag_homing) {
						cont_samp = 0;
						m1.p_fin = instrucciones[i + 1];
						error_pos = (m1.p_fin - m1.p_ini) / m1.p_ini * 100;
						if (fabs(error_pos) > 0.001) {
							m1.t_tot = instrucciones[i + 3];
							Vmin = fabs(m1.p_fin - m1.p_ini) / m1.t_tot;
							if (VEL_MAX < Vmin) {
								estado = Error;
								i += 3;
								break;
								//No puede realizarse
								__NOP();

							} else if (VEL_MAX > (2 * Vmin)) {
								m1.v_cru = 1.5 * Vmin;
							} else if (VEL_MAX > Vmin
									&& VEL_MAX <= (2 * Vmin)) {
								m1.v_cru = VEL_MAX;
							}
							if ((m1.p_fin - m1.p_ini) < 0) {
								m1.v_cru *= -1;
							}
							m1.tau = (m1.p_ini - m1.p_fin + m1.v_cru * m1.t_tot)
									/ m1.v_cru;
							m1.a_nes =
									pow(m1.v_cru, 2)
											/ (m1.p_ini - m1.p_fin
													+ m1.v_cru * m1.t_tot);

							flag_cambio = 1;
						}
						estado = Modo_Normal;
						i += 3;
					}
					break;
				case error:
					//apagar todo
					flag_homing = 0;
					estado = Error;
					flag_activacion = 0;
					flag_cambio = 1;
					break;
				case Preguntar:
					HAL_SPI_Abort_IT(&hspi2);
					//Revisa el estado, interuumpe y guarada el estado en el puerto SP
					if (estado == Desactivado) {
						pTxData = 'D';
						HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
					} else if (estado == Activado) {
						pTxData = 'A';
						HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
					} else if (estado == Modo_Homing) {
						pTxData = 'H';
						HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
					} else if (estado == Modo_Normal) {
						pTxData = 'N';
						HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
					} else if (estado == Error) {
						pTxData = 'E';
						HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
					}
					HAL_GPIO_WritePin(int1_M_cpt_t_GPIO_Port, int1_M_cpt_t_Pin,
							GPIO_PIN_SET);
					HAL_GPIO_WritePin(int1_M_cpt_t_GPIO_Port, int1_M_cpt_t_Pin,
							GPIO_PIN_RESET);
					__HAL_SPI_CLEAR_OVRFLAG(&hspi2);
					HAL_SPI_Receive_IT(&hspi2, &dato_recepcion_SPI, 1);
					break;
				case Estados:
					//Revisa el estado, interuumpe y guarada el estado en el puerto SP
					break;
				}

			}
			flag_mensaje_completo = 2;
			cant = 0;
		}

		switch (estado) {
		//en la maquina de estado hace la gestion de cada motor los cuales sus datos estan guardados en un struk
		case Desactivado:
			if (flag_cambio == 1) {
				flag_cambio = 0;
			}
			break;
		case Activado:
			if (flag_cambio == 1) {
				HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
				flag_cambio = 0;
			}
			break;
		case Modo_Homing:
			if (flag_cambio == 1) {
				m1.modo = M_realizando;
				HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_RESET);
				PWM_config.Pulse = 1200;
				HAL_TIM_PWM_ConfigChannel(&htim12, &PWM_config,
				TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
				flag_cambio = 0;
				m1.p_ini = 0;
			}
			if (m1.modo == M_listo && m2.modo == M_listo && flag_cambio2) {
				flag_cambio2 = 0;
				HAL_SPI_Abort_IT(&hspi2);
				pTxData = 'L';
				HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
				HAL_GPIO_WritePin(int1_M_cpt_t_GPIO_Port, int1_M_cpt_t_Pin,
						GPIO_PIN_SET);
				HAL_GPIO_WritePin(int1_M_cpt_t_GPIO_Port, int1_M_cpt_t_Pin,
						GPIO_PIN_RESET);
				__HAL_SPI_CLEAR_OVRFLAG(&hspi2);
				HAL_SPI_Receive_IT(&hspi2, &dato_recepcion_SPI, 1);
			}
			break;
		case Modo_Normal:
			if (flag_cambio == 1) {
				m1.modo = M_realizando;
				Up = 0;
				UPID = 0;
				Ui = 0;
				Ud = 0;
				Ui_ant = 0;
				error_ant_M = 0;
				cnt_lis = 0;
				HAL_GPIO_WritePin(L298_ENA1_GPIO_Port, L298_ENA1_Pin,
						GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_RESET);
				PWM_config.Pulse = 0;
				HAL_TIM_PWM_ConfigChannel(&htim12, &PWM_config, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
				HAL_GPIO_WritePin(L298_ENA1_GPIO_Port, L298_ENA1_Pin,
						GPIO_PIN_SET);
				HAL_TIM_Base_Start_IT(&htim9);
				flag_cambio = 0;
			}
			if (m1.modo == M_listo && m2.modo == M_listo && flag_cambio2) {
				flag_cambio2 = 0;
				HAL_SPI_Abort_IT(&hspi2);
				pTxData = 'L';
				HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
				HAL_GPIO_WritePin(int1_M_cpt_t_GPIO_Port, int1_M_cpt_t_Pin,
						GPIO_PIN_SET);
				HAL_GPIO_WritePin(int1_M_cpt_t_GPIO_Port, int1_M_cpt_t_Pin,
						GPIO_PIN_RESET);
				__HAL_SPI_CLEAR_OVRFLAG(&hspi2);
				HAL_SPI_Receive_IT(&hspi2, &dato_recepcion_SPI, 1);
			}
			break;
		case Error:
			Error_Handler();
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	cont_datos_SPI++;
	str[cont_datos_SPI - 1] = dato_recepcion_SPI;
	if(str[cont_datos_SPI - 1]=='\0'){
		cont_datos_SPI--;
	}
	if (str[cont_datos_SPI - 1] == ':') {
		flag_mensaje_completo = 0;
		contador_instrucciones = cont_datos_SPI;
		cont_datos_SPI = 0;
	}
	HAL_SPI_Receive_IT(&hspi2, &dato_recepcion_SPI, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {

	}
	if (htim->Instance == TIM3) {
	}
	if (htim->Instance == TIM9) {
		cont_samp++;
		m1.t_act = TIEMPO_SAMP * cont_samp;
		if (m1.t_act < m1.t_tot) {
			if (m1.t_act <= m1.tau) {
				m1.p_obj = (m1.p_ini + m1.a_nes * pow(m1.t_act, 2) / 2) * 600
						/ 2 / M_PI + 800;
			} else if (m1.t_act > m1.tau && m1.t_act <= (m1.t_tot - m1.tau)) {
				m1.p_obj = (m1.p_ini
						+ m1.a_nes * m1.tau * (m1.t_act - m1.tau / 2)) * 600 / 2
						/ M_PI + 800;
			} else if (m1.t_act > (m1.t_tot - m1.tau) && m1.t_act < m1.t_tot) {
				m1.p_obj = (m1.p_fin
						- m1.a_nes * pow(m1.t_tot - m1.t_act, 2) / 2) * 600 / 2
						/ M_PI + 800;
			}
			error_M = (m1.p_obj - m1.p_act);
		} else {
			error_M = (int) (m1.p_obj - m1.p_act);
			if (!error_M) {
				cnt_lis++;
			} else {
				cnt_lis = 0;
				cont_error++;
			}
			if (cnt_lis == 15) {
				HAL_TIM_Base_Stop_IT(&htim9);
				m1.p_ini = (m1.p_obj - 800) * 2 * M_PI / 600;
				PWM_config.Pulse = 0;
				HAL_TIM_PWM_ConfigChannel(&htim12, &PWM_config, TIM_CHANNEL_1);
				HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_RESET);
				HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
				m1.modo = M_listo;
				flag_cambio2 = 1;
				cont_error = 0;
				cont_samp = 0;
				return;
			}
			if (cont_error > 100000) {
				HAL_TIM_Base_Stop_IT(&htim9);
				flag_cambio = 1;
				estado = Error;
				return;
			}
		}
		m1.p_act = TIM1->CNT;
		Up = m1.kp * error_M;
		Ui = Ui_ant + m1.ki * TIEMPO_SAMP * error_ant_M;
		Ud = m1.kd / TIEMPO_SAMP * (error_M - error_ant_M);
		UPID = Up + Ui + Ud;
		Ui_ant = Ui;
		error_ant_M = error_M;
		if (UPID > 0) {
			UPID += 400;
			if (UPID > 2799) {
				UPID = 2799;
			}
			PWM_config.Pulse = 2799 - UPID;
			HAL_TIM_PWM_ConfigChannel(&htim12, &PWM_config, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(L298_ENA1_GPIO_Port, L298_ENA1_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(L298_ENA1_GPIO_Port, L298_ENA1_Pin, GPIO_PIN_SET);
		} else {
			UPID -= 400;
			if (UPID < -2799) {
				UPID = -2799;
			}
			PWM_config.Pulse = -UPID;

			HAL_TIM_PWM_ConfigChannel(&htim12, &PWM_config, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(L298_ENA1_GPIO_Port, L298_ENA1_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(L298_ENA1_GPIO_Port, L298_ENA1_Pin, GPIO_PIN_SET);
		}

	}
	if (htim->Instance == TIM12) {
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case h1_inter_Pin:
		PWM_config.Pulse = 2799 - 900;
		HAL_TIM_PWM_ConfigChannel(&htim12, &PWM_config, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_SET);
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
		while (HAL_GPIO_ReadPin(h1_inter_GPIO_Port, h1_inter_Pin)) {

		}
		PWM_config.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim12, &PWM_config, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
		HAL_Delay(1);
		htim1.Instance->CNT = 500;
		m1.p_ini = -M_PI;
		flag_homing = 1;
		m1.modo = M_listo;
		flag_cambio2 = 1;
		break;
	case h2_inter_Pin:
		break;
	case pin_error_Pin:
		estado = Error;
		break;
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
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_TIM_Base_Stop_IT(&htim9);
	HAL_GPIO_WritePin(L298_ENA1_GPIO_Port, L298_ENA1_Pin, GPIO_PIN_RESET);
	PWM_config.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim12, &PWM_config,
	TIM_CHANNEL_1);
	HAL_GPIO_WritePin(dir1_GPIO_Port, dir1_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
	estado = Error;
	flag_activacion = 0;
	flag_homing = 0;
	flag_cambio = 0;
	HAL_SPI_Abort_IT(&hspi2);
	pTxData = 'E';
	HAL_SPI_Transmit_IT(&hspi2, &pTxData, 1);
	HAL_GPIO_WritePin(int1_M_cpt_t_GPIO_Port, int1_M_cpt_t_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(int1_M_cpt_t_GPIO_Port, int1_M_cpt_t_Pin, GPIO_PIN_RESET);
	__HAL_SPI_CLEAR_OVRFLAG(&hspi2);
	HAL_SPI_Receive_IT(&hspi2, &dato_recepcion_SPI, 1);
	HAL_SPI_Abort_IT(&hspi2);
	while (1)
		;
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
