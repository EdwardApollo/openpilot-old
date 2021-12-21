#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3162074725094743607);
void live_err_fun(double *nom_x, double *delta_x, double *out_5754035859235434590);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5787811691745881220);
void live_H_mod_fun(double *state, double *out_7077244422403030034);
void live_f_fun(double *state, double dt, double *out_8051536556320895415);
void live_F_fun(double *state, double dt, double *out_3417269740718499968);
void live_h_4(double *state, double *unused, double *out_8184302504804048816);
void live_H_4(double *state, double *unused, double *out_3295468362103921480);
void live_h_9(double *state, double *unused, double *out_2584603246226752416);
void live_H_9(double *state, double *unused, double *out_3054278715474330835);
void live_h_10(double *state, double *unused, double *out_2090935561331339912);
void live_H_10(double *state, double *unused, double *out_3820411795354228066);
void live_h_12(double *state, double *unused, double *out_5732995365552519949);
void live_H_12(double *state, double *unused, double *out_1723988045928040315);
void live_h_31(double *state, double *unused, double *out_1142810727031743824);
void live_H_31(double *state, double *unused, double *out_4469551078253054024);
void live_h_32(double *state, double *unused, double *out_8646692373654724500);
void live_H_32(double *state, double *unused, double *out_1583069655404599528);
void live_h_13(double *state, double *unused, double *out_4669449068525000525);
void live_H_13(double *state, double *unused, double *out_4867347292809621062);
void live_h_14(double *state, double *unused, double *out_2584603246226752416);
void live_H_14(double *state, double *unused, double *out_3054278715474330835);
void live_h_33(double *state, double *unused, double *out_6688590622439407421);
void live_H_33(double *state, double *unused, double *out_7620108082891911628);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}