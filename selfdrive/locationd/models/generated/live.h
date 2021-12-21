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
void live_H(double *in_vec, double *out_2028871770198805139);
void live_err_fun(double *nom_x, double *delta_x, double *out_8151007758379062661);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2128743399615825523);
void live_H_mod_fun(double *state, double *out_6416191868946148009);
void live_f_fun(double *state, double dt, double *out_3264604968293694097);
void live_F_fun(double *state, double dt, double *out_1741995097926784820);
void live_h_4(double *state, double *unused, double *out_6343403930864150333);
void live_H_4(double *state, double *unused, double *out_3788444243282928713);
void live_h_9(double *state, double *unused, double *out_7503518715580627469);
void live_H_9(double *state, double *unused, double *out_7371080895162175433);
void live_h_10(double *state, double *unused, double *out_8411818234095403400);
void live_H_10(double *state, double *unused, double *out_2755645111163305288);
void live_h_12(double *state, double *unused, double *out_8151414419776971058);
void live_H_12(double *state, double *unused, double *out_2592814133759804283);
void live_h_31(double *state, double *unused, double *out_6307158399764442839);
void live_H_31(double *state, double *unused, double *out_4245608484419158702);
void live_h_32(double *state, double *unused, double *out_2026782489563584114);
void live_H_32(double *state, double *unused, double *out_3700593238552368155);
void live_h_13(double *state, double *unused, double *out_7377470601038055234);
void live_H_13(double *state, double *unused, double *out_5643909537672020686);
void live_h_14(double *state, double *unused, double *out_7503518715580627469);
void live_H_14(double *state, double *unused, double *out_7371080895162175433);
void live_h_33(double *state, double *unused, double *out_4889766644938345450);
void live_H_33(double *state, double *unused, double *out_1095051479780301098);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}