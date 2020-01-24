/*
 * IDENTIFICADOR.h
 *
 * Created: 11/12/2019 18:47:08
 *  Author: Rodrigo Goñi
 */ 


#ifndef IDENTIFICADOR_H_
#define IDENTIFICADOR_H_
#include <stdlib.h>
enum Comandos{
	Activar,Desactivar,Estados,Modo_homing,Modo_ir, error,Preguntar
};
int identificador(char *str,double *instrucciones,int cont);
#endif /* IDENTIFICADOR_H_ */