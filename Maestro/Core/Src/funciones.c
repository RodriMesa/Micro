/*
 * funciones.c
 *
 *  Created on: 25 ene. 2020
 *      Author: Rodri
 */
#include "funciones.h"
void Leds_Desactivado() {
	HAL_GPIO_WritePin(Led_Error_GPIO_Port, Led_Error_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Activado_GPIO_Port, Led_Activado_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Homing_GPIO_Port, Led_Homing_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Normal_GPIO_Port, Led_Normal_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_SET);
}
void Leds_Activado() {
	//Prender LED
	HAL_GPIO_WritePin(Led_Error_GPIO_Port, Led_Error_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Homing_GPIO_Port, Led_Homing_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Normal_GPIO_Port, Led_Normal_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Activado_GPIO_Port, Led_Activado_Pin, GPIO_PIN_SET);
}
void Leds_Homing() {
	HAL_GPIO_WritePin(Led_Error_GPIO_Port, Led_Error_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Activado_GPIO_Port, Led_Activado_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Normal_GPIO_Port, Led_Normal_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Homing_GPIO_Port, Led_Homing_Pin, GPIO_PIN_SET);
}
void Leds_Normal() {
	HAL_GPIO_WritePin(Led_Error_GPIO_Port, Led_Error_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Activado_GPIO_Port, Led_Activado_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Homing_GPIO_Port, Led_Homing_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Normal_GPIO_Port, Led_Normal_Pin, GPIO_PIN_SET);
}
void Leds_Error() {
	HAL_GPIO_WritePin(Led_Activado_GPIO_Port, Led_Activado_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Desactivado_GPIO_Port, Led_Desactivado_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Homing_GPIO_Port, Led_Homing_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Normal_GPIO_Port, Led_Normal_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_Error_GPIO_Port, Led_Error_Pin, GPIO_PIN_SET);
}
void Cin_Dir(double *instrucciones,int i){
	//Lo comentado es para el segundo esclavo
	int k = 0, l = 0;
	static char s1[25];
//	static char s2[25];
	char s[7];
	s1[0] = 'I';
	s1[1] = '_';
//	s2[0] = 'I';
//	s2[1] = '_';
	for (k = i; k < (i + 2); k++) {
		snprintf(s, 7, "%lf", instrucciones[k + 1]);
		for (l = 0; l < 6; l++) {
			s1[l + (k - i) * 7 + 2] = s[l];
		}
		s1[l + (k - i) * 7 + 2] = '_';
	}
/*	for (k = (i + 2); k < (i + 4); k++) {
		snprintf(s, 7, "%lf", instrucciones[k + 1]);
		for (l = 0; l < 6; l++) {
			s2[l + (k - (i + 2)) * 7 + 2] = s[l];
		}
		s2[l + (k - (i + 2)) * 7 + 2] = '_';
	}*/
	snprintf(s, 7, "%lf", instrucciones[i + 5]);
	for (l = 0; l < 6; l++) {
//		s2[l + 16] = s[l];
		s1[l + 16] = s[l];
	}
	s1[22] = '/';
	s1[23] = 'P';
	s1[24] = ':';
//	s2[22] = '-';
//	s2[23] = 'P';
//	s2[24] = ':';
	//Mandar consigna de cinemática directa
	for (k = 0; k < 25; k++) {
		SPI_Transmit_1(s1[k]);
	}
/*	for (k = 0; k < 25; k++) {
		SPI_Transmit_2(s2[k]);
	}*/
}
