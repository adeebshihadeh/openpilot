#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4764429363741635774);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4923486496625013297);
void car_H_mod_fun(double *state, double *out_8284844191623208552);
void car_f_fun(double *state, double dt, double *out_5426936756111410655);
void car_F_fun(double *state, double dt, double *out_1189665854613176299);
void car_h_25(double *state, double *unused, double *out_2384565946978239929);
void car_H_25(double *state, double *unused, double *out_6913636704135587423);
void car_h_24(double *state, double *unused, double *out_5914851978677013851);
void car_H_24(double *state, double *unused, double *out_2445450570523504853);
void car_h_30(double *state, double *unused, double *out_4386269279372111007);
void car_H_30(double *state, double *unused, double *out_2700262206813595857);
void car_h_26(double *state, double *unused, double *out_3218525478295969270);
void car_H_26(double *state, double *unused, double *out_8219872476090428568);
void car_h_27(double *state, double *unused, double *out_7006751976434348243);
void car_H_27(double *state, double *unused, double *out_3987844194650221169);
void car_h_29(double *state, double *unused, double *out_7198821382641964390);
void car_H_29(double *state, double *unused, double *out_6524060975942427810);
void car_h_28(double *state, double *unused, double *out_3143930863498534129);
void car_H_28(double *state, double *unused, double *out_8983461024949551897);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}