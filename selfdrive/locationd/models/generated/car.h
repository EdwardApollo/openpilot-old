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
void car_err_fun(double *nom_x, double *delta_x, double *out_2597874447553363541);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8182813298570237867);
void car_H_mod_fun(double *state, double *out_6306941947207966650);
void car_f_fun(double *state, double dt, double *out_2402943826538611257);
void car_F_fun(double *state, double dt, double *out_6186946395878559235);
void car_h_25(double *state, double *unused, double *out_7844523176759450585);
void car_H_25(double *state, double *unused, double *out_388105742977512244);
void car_h_24(double *state, double *unused, double *out_3716241642962455388);
void car_H_24(double *state, double *unused, double *out_5889753943026838658);
void car_h_30(double *state, double *unused, double *out_8760773842464067664);
void car_H_30(double *state, double *unused, double *out_9220950905737880580);
void car_h_26(double *state, double *unused, double *out_3118043971383110829);
void car_H_26(double *state, double *unused, double *out_6092698897916721517);
void car_h_27(double *state, double *unused, double *out_2086538262765712629);
void car_H_27(double *state, double *unused, double *out_7933368917901255268);
void car_h_29(double *state, double *unused, double *out_684573804615255661);
void car_H_29(double *state, double *unused, double *out_8651234554116134861);
void car_h_28(double *state, double *unused, double *out_3486029231902958175);
void car_H_28(double *state, double *unused, double *out_1128078535209656208);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}