#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7500353067111488423);
void live_err_fun(double *nom_x, double *delta_x, double *out_766172588434086154);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1300041075557829446);
void live_H_mod_fun(double *state, double *out_6253763052524624393);
void live_f_fun(double *state, double dt, double *out_1122240391475756889);
void live_F_fun(double *state, double dt, double *out_4917658642529101547);
void live_h_4(double *state, double *unused, double *out_4834206895910826861);
void live_H_4(double *state, double *unused, double *out_1049374308257679367);
void live_h_9(double *state, double *unused, double *out_884527150099786354);
void live_H_9(double *state, double *unused, double *out_808184661628088722);
void live_h_10(double *state, double *unused, double *out_7349985989310522663);
void live_H_10(double *state, double *unused, double *out_3470071247623489746);
void live_h_12(double *state, double *unused, double *out_3200280810746522172);
void live_H_12(double *state, double *unused, double *out_3970082099774282428);
void live_h_35(double *state, double *unused, double *out_3387869530151147510);
void live_H_35(double *state, double *unused, double *out_6715645132099296137);
void live_h_32(double *state, double *unused, double *out_3922442096717896800);
void live_H_32(double *state, double *unused, double *out_6403023437211851109);
void live_h_13(double *state, double *unused, double *out_742738544544814837);
void live_H_13(double *state, double *unused, double *out_5819643715718069551);
void live_h_14(double *state, double *unused, double *out_884527150099786354);
void live_H_14(double *state, double *unused, double *out_808184661628088722);
void live_h_33(double *state, double *unused, double *out_1140698226418861938);
void live_H_33(double *state, double *unused, double *out_8580541936971397875);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}