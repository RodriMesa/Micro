/*
 * IDENTIFICADOR.c
 *
 * Created: 11/12/2019 18:47:43
 *  Author: Rodrigo Goñi
 */
#include "identificador.h"
int identificador(char *str, double *instrucciones, int cont) {
	char tmp[50] = { 0 };
	double vec[20];
	char *puntero;
	int i = 0;
	int j = 0;
	int k = 0;
	int l = 0;
	int cont1 = 0;
	double numero;
	while (i < cont) {
		switch (str[i]) {
		case 'r':
		case 'R':
			if (str[i + 1] == '-' || str[i + 1] == ':') {
				instrucciones[j] = Estados;
				j++;
			}
			i++;
			break;
		case 'a':
		case 'A':
			if (str[i + 1] == '-' || str[i + 1] == ':') {
				instrucciones[j] = Activar;
				j++;
			}
			i++;
			break;
		case 'd':
		case 'D':
			if (str[i + 1] == '-' || str[i + 1] == ':') {
				instrucciones[j] = Desactivar;
				j++;
			}
			i++;
			break;
		case 'h':
		case 'H':
			if (str[i + 1] == '-' || str[i + 1] == ':') {
				instrucciones[j] = Modo_homing;
				j++;
			}
			i++;
			break;
		case 'e':
		case 'E':
			if (str[i + 1] == '-' || str[i + 1] == ':') {
				instrucciones[j] = error;
				j++;
			}
			i++;
			break;
		case 'p':
		case 'P':
			if (str[i + 1] == '-' || str[i + 1] == ':') {
				instrucciones[j] = Preguntar;
				j++;
			}
			i++;
			break;
		case 'i':
		case 'I':
			i++;
			while ((str[i] != '-') && (i < cont)) {
				k = 0;
				if (str[i] == '_') {
					i++;
				}
				while ((str[i] != '_') && (i < cont) && (str[i] != '-')) {
					tmp[k] = str[i];
					k++;
					i++;
				}
				numero = strtod(tmp, &puntero);
				vec[cont1] = numero;
				cont1++;
				for (int m = 0; m <= 50; m++) {
					tmp[m] = 0;
				}
			}
			if (cont1 == 3) {
				instrucciones[j] = Modo_ir;
				j++;
				for (l = 0; l < cont1; l++) {
					instrucciones[j] = vec[l];
					j++;
				}
			}
			cont1 = 0;
			break;
		default:
			i++;
		}
	}
	return j;
}
