/*
* identificador.h
*
* Created: 10/12/2019 06:15:43 p.m.
*  Author: Rodri
*/


#ifndef IDENTIFICADOR_H_
#define IDENTIFICADOR_H_
#include <stdlib.h>
enum Comandos{
	Modo_desactivado,Modo_activado,Modo_homing,Cin_dir, Cin_inv,Resumen	
};
int identificador(char *str,double *instrucciones,int cont);
#endif /* IDENTIFICADOR_H_ */