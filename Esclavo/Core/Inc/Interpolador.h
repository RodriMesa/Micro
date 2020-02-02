/*
 * Interpolador.h
 *
 *  Created on: 25 ene. 2020
 *      Author: rodry
 */

#ifndef INC_INTERPOLADOR_H_
#define INC_INTERPOLADOR_H_
#include "main.h"
#define TIEMPO_SAMP 0.0002
#define VEL_MAX 11.7705
void interpolador(double pos_init, double pos_fin, double tiempo,float *q_vec);
int interpolador_vel(double pos_init, double pos_fin, double tiempo,float *q_vec,int cant_pun_tot);
#endif /* INC_INTERPOLADOR_H_ */
