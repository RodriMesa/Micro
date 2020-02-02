/*
 * identificador.c
 *
 * Created: 10/12/2019 06:15:29 p.m.
 *  Author: Rodri
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
		case 'd':
		case 'D':
			if (str[i + 1] == '-' || str[i + 1] == ':') {
				instrucciones[j] = Modo_desactivado;
				j++;
			}
			i++;
			break;
		case 'a':
		case 'A':
			if (str[i + 1] == '-' || str[i + 1] == ':') {
				instrucciones[j] = Modo_activado;
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
		case 'r':
		case 'R':
			if (str[i + 1] == '-' || str[i + 1] == ':') {
				instrucciones[j] = Resumen;
				j++;
			}
			i++;
			break;
		case 'c':
		case 'C':
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
				for (int m = 0; m < 50; m++) {
					tmp[m] = 0;
				}
			}
			if (cont1 == 5) {
				instrucciones[j] = Cin_dir;
				j++;
				for (l = 0; l < cont1; l++) {
					instrucciones[j] = vec[l];
					j++;
				}
			}
			cont1 = 0;
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
				for (int m = 0; m < 50; m++) {
					tmp[m] = 0;
				}
			}
			if (cont1 == 5) {
				instrucciones[j] = Cin_inv;
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
