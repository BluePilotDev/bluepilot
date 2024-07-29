#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2673684484490617892);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_230892919190133727);
void car_H_mod_fun(double *state, double *out_3465232717738919044);
void car_f_fun(double *state, double dt, double *out_3047169393026212329);
void car_F_fun(double *state, double dt, double *out_594815173963621676);
void car_h_25(double *state, double *unused, double *out_4420541495512235573);
void car_H_25(double *state, double *unused, double *out_6091906089981428809);
void car_h_24(double *state, double *unused, double *out_6466529401607636301);
void car_H_24(double *state, double *unused, double *out_109126589869732617);
void car_h_30(double *state, double *unused, double *out_6014389683595920632);
void car_H_30(double *state, double *unused, double *out_1564209759853820611);
void car_h_26(double *state, double *unused, double *out_6841939516446877451);
void car_H_26(double *state, double *unused, double *out_2350402771107372585);
void car_h_27(double *state, double *unused, double *out_7457327546956568258);
void car_H_27(double *state, double *unused, double *out_3787803831037763828);
void car_h_29(double *state, double *unused, double *out_7182133484672062369);
void car_H_29(double *state, double *unused, double *out_2074441104168212795);
void car_h_28(double *state, double *unused, double *out_9147775381692249218);
void car_H_28(double *state, double *unused, double *out_4038071375733539046);
void car_h_31(double *state, double *unused, double *out_3962340133353673911);
void car_H_31(double *state, double *unused, double *out_1724194668874021109);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}