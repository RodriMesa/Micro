/*
 * Interpolador.c
 *
 *  Created on: 25 ene. 2020
 *      Author: rodry
 */

#include "interpolador.h"
void interpolador(double pos_init, double pos_fin, double tiempo, float *q_vec) {
	double acel_nes, tau;
	int cant_pun_tot, cant_pun_tau;
	cant_pun_tot = tiempo / TIEMPO_SAMP;
	if (pos_fin - pos_init > 0) {
		acel_nes = (VEL_MAX / (tiempo - (pos_fin - pos_init) / VEL_MAX));
		tau = VEL_MAX / acel_nes;
		cant_pun_tau = tau / TIEMPO_SAMP;
		for (int i = 0; i <= cant_pun_tau; i++) {
			q_vec[i] = pos_init + acel_nes / 2 * pow((TIEMPO_SAMP * i), 2);
		}

		for (int i = cant_pun_tau + 1; i <= (cant_pun_tot - cant_pun_tau);
				i++) {
			q_vec[i] = pos_init - pow(VEL_MAX, 2) / (2 * acel_nes)
					+ VEL_MAX * (TIEMPO_SAMP * i);

		}
		for (int i = (cant_pun_tot - cant_pun_tau) + 1; i < cant_pun_tot; i++) {
			q_vec[i] = pos_fin
					+ (-acel_nes * pow(tiempo, 2) / 2
							+ acel_nes * tiempo * TIEMPO_SAMP * i
							- acel_nes / 2 * pow((TIEMPO_SAMP * i), 2));
		}
	} else {
		acel_nes = (VEL_MAX / (tiempo + (pos_fin - pos_init) / VEL_MAX));
		tau = VEL_MAX / acel_nes;
		cant_pun_tau = tau / TIEMPO_SAMP;
		for (int i = 0; i <= cant_pun_tau; i++) {
			q_vec[i] = pos_init - acel_nes / 2 * pow((TIEMPO_SAMP * i), 2);
		}
		for (int i = cant_pun_tau + 1; i <= (cant_pun_tot - cant_pun_tau);
				i++) {
			q_vec[i] = pos_init + pow(VEL_MAX, 2) / (2 * acel_nes)
					- VEL_MAX * (TIEMPO_SAMP * i);

		}
		for (int i = (cant_pun_tot - cant_pun_tau) + 1; i < cant_pun_tot; i++) {
			q_vec[i] = pos_fin
					- (-acel_nes * pow(tiempo, 2) / 2
							+ acel_nes * tiempo * TIEMPO_SAMP * i
							- acel_nes / 2 * pow((TIEMPO_SAMP * i), 2));
		}
	}
}
int interpolador_vel(double pos_init, double pos_fin, double tiempo,
		float *q_vec,int cant_pun_tot) {
	double acel_nes, tau, pend;
	int cant_pun_tau;
	if (pos_fin - pos_init > 0) {
		acel_nes = (VEL_MAX*2 / (tiempo - (pos_fin - pos_init) / VEL_MAX));
		tau = VEL_MAX / acel_nes;
		cant_pun_tau = tau / TIEMPO_SAMP;
		pend = VEL_MAX / tau;
		for (int i = 0; i <= cant_pun_tau; i++) {
			q_vec[i] = TIEMPO_SAMP * i * pend/ VEL_MAX ;
		}

		for (int i = cant_pun_tau + 1; i <= (cant_pun_tot - cant_pun_tau);
				i++) {
			q_vec[i] =1;
		}
		for (int i = (cant_pun_tot - cant_pun_tau) + 1; i < cant_pun_tot; i++) {
			q_vec[i] = (VEL_MAX - TIEMPO_SAMP * (i-((cant_pun_tot - cant_pun_tau) + 1)) * pend)/ VEL_MAX ;
		}
		return 1;
	} else {
		acel_nes = (VEL_MAX*2 / (tiempo + (pos_fin - pos_init) / VEL_MAX));
		tau = VEL_MAX / acel_nes;
		cant_pun_tau = tau / TIEMPO_SAMP;
		pend = VEL_MAX / tau;
		for (int i = 0; i <= cant_pun_tau; i++) {
			q_vec[i] = TIEMPO_SAMP * i * pend;
		}

		for (int i = cant_pun_tau + 1; i <= (cant_pun_tot - cant_pun_tau);
				i++) {
			q_vec[i] = VEL_MAX;
		}
		for (int i = (cant_pun_tot - cant_pun_tau) + 1; i < cant_pun_tot; i++) {
			q_vec[i] = VEL_MAX - TIEMPO_SAMP * i * pend;
		}
		return 0;
	}

}

