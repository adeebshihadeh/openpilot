#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_3(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_19(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7077736930214747903);
void live_err_fun(double *nom_x, double *delta_x, double *out_4679540747163827133);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8867206427208397913);
void live_H_mod_fun(double *state, double *out_2363300847419814559);
void live_f_fun(double *state, double dt, double *out_6579354977661886921);
void live_F_fun(double *state, double dt, double *out_5501054355011549264);
void live_h_3(double *state, double *unused, double *out_748137226001467749);
void live_H_3(double *state, double *unused, double *out_4585100080245148997);
void live_h_4(double *state, double *unused, double *out_8381455546014449950);
void live_H_4(double *state, double *unused, double *out_2663493959063754732);
void live_h_9(double *state, double *unused, double *out_5801679864646082070);
void live_H_9(double *state, double *unused, double *out_6705040873441958063);
void live_h_10(double *state, double *unused, double *out_7139882230926898653);
void live_H_10(double *state, double *unused, double *out_3605516732528081093);
void live_h_12(double *state, double *unused, double *out_7667811351918480981);
void live_H_12(double *state, double *unused, double *out_556938447324545163);
void live_h_31(double *state, double *unused, double *out_5870513745504082216);
void live_H_31(double *state, double *unused, double *out_3232028222467964984);
void live_h_32(double *state, double *unused, double *out_1012037219448008474);
void live_H_32(double *state, double *unused, double *out_8914350106981185446);
void live_h_13(double *state, double *unused, double *out_7194025741448451155);
void live_H_13(double *state, double *unused, double *out_2844204835887890035);
void live_h_14(double *state, double *unused, double *out_5801679864646082070);
void live_H_14(double *state, double *unused, double *out_6705040873441958063);
void live_h_19(double *state, double *unused, double *out_2700349924702308076);
void live_H_19(double *state, double *unused, double *out_7309023349165162372);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}