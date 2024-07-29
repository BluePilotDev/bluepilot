#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2673684484490617892) {
   out_2673684484490617892[0] = delta_x[0] + nom_x[0];
   out_2673684484490617892[1] = delta_x[1] + nom_x[1];
   out_2673684484490617892[2] = delta_x[2] + nom_x[2];
   out_2673684484490617892[3] = delta_x[3] + nom_x[3];
   out_2673684484490617892[4] = delta_x[4] + nom_x[4];
   out_2673684484490617892[5] = delta_x[5] + nom_x[5];
   out_2673684484490617892[6] = delta_x[6] + nom_x[6];
   out_2673684484490617892[7] = delta_x[7] + nom_x[7];
   out_2673684484490617892[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_230892919190133727) {
   out_230892919190133727[0] = -nom_x[0] + true_x[0];
   out_230892919190133727[1] = -nom_x[1] + true_x[1];
   out_230892919190133727[2] = -nom_x[2] + true_x[2];
   out_230892919190133727[3] = -nom_x[3] + true_x[3];
   out_230892919190133727[4] = -nom_x[4] + true_x[4];
   out_230892919190133727[5] = -nom_x[5] + true_x[5];
   out_230892919190133727[6] = -nom_x[6] + true_x[6];
   out_230892919190133727[7] = -nom_x[7] + true_x[7];
   out_230892919190133727[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3465232717738919044) {
   out_3465232717738919044[0] = 1.0;
   out_3465232717738919044[1] = 0;
   out_3465232717738919044[2] = 0;
   out_3465232717738919044[3] = 0;
   out_3465232717738919044[4] = 0;
   out_3465232717738919044[5] = 0;
   out_3465232717738919044[6] = 0;
   out_3465232717738919044[7] = 0;
   out_3465232717738919044[8] = 0;
   out_3465232717738919044[9] = 0;
   out_3465232717738919044[10] = 1.0;
   out_3465232717738919044[11] = 0;
   out_3465232717738919044[12] = 0;
   out_3465232717738919044[13] = 0;
   out_3465232717738919044[14] = 0;
   out_3465232717738919044[15] = 0;
   out_3465232717738919044[16] = 0;
   out_3465232717738919044[17] = 0;
   out_3465232717738919044[18] = 0;
   out_3465232717738919044[19] = 0;
   out_3465232717738919044[20] = 1.0;
   out_3465232717738919044[21] = 0;
   out_3465232717738919044[22] = 0;
   out_3465232717738919044[23] = 0;
   out_3465232717738919044[24] = 0;
   out_3465232717738919044[25] = 0;
   out_3465232717738919044[26] = 0;
   out_3465232717738919044[27] = 0;
   out_3465232717738919044[28] = 0;
   out_3465232717738919044[29] = 0;
   out_3465232717738919044[30] = 1.0;
   out_3465232717738919044[31] = 0;
   out_3465232717738919044[32] = 0;
   out_3465232717738919044[33] = 0;
   out_3465232717738919044[34] = 0;
   out_3465232717738919044[35] = 0;
   out_3465232717738919044[36] = 0;
   out_3465232717738919044[37] = 0;
   out_3465232717738919044[38] = 0;
   out_3465232717738919044[39] = 0;
   out_3465232717738919044[40] = 1.0;
   out_3465232717738919044[41] = 0;
   out_3465232717738919044[42] = 0;
   out_3465232717738919044[43] = 0;
   out_3465232717738919044[44] = 0;
   out_3465232717738919044[45] = 0;
   out_3465232717738919044[46] = 0;
   out_3465232717738919044[47] = 0;
   out_3465232717738919044[48] = 0;
   out_3465232717738919044[49] = 0;
   out_3465232717738919044[50] = 1.0;
   out_3465232717738919044[51] = 0;
   out_3465232717738919044[52] = 0;
   out_3465232717738919044[53] = 0;
   out_3465232717738919044[54] = 0;
   out_3465232717738919044[55] = 0;
   out_3465232717738919044[56] = 0;
   out_3465232717738919044[57] = 0;
   out_3465232717738919044[58] = 0;
   out_3465232717738919044[59] = 0;
   out_3465232717738919044[60] = 1.0;
   out_3465232717738919044[61] = 0;
   out_3465232717738919044[62] = 0;
   out_3465232717738919044[63] = 0;
   out_3465232717738919044[64] = 0;
   out_3465232717738919044[65] = 0;
   out_3465232717738919044[66] = 0;
   out_3465232717738919044[67] = 0;
   out_3465232717738919044[68] = 0;
   out_3465232717738919044[69] = 0;
   out_3465232717738919044[70] = 1.0;
   out_3465232717738919044[71] = 0;
   out_3465232717738919044[72] = 0;
   out_3465232717738919044[73] = 0;
   out_3465232717738919044[74] = 0;
   out_3465232717738919044[75] = 0;
   out_3465232717738919044[76] = 0;
   out_3465232717738919044[77] = 0;
   out_3465232717738919044[78] = 0;
   out_3465232717738919044[79] = 0;
   out_3465232717738919044[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3047169393026212329) {
   out_3047169393026212329[0] = state[0];
   out_3047169393026212329[1] = state[1];
   out_3047169393026212329[2] = state[2];
   out_3047169393026212329[3] = state[3];
   out_3047169393026212329[4] = state[4];
   out_3047169393026212329[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3047169393026212329[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3047169393026212329[7] = state[7];
   out_3047169393026212329[8] = state[8];
}
void F_fun(double *state, double dt, double *out_594815173963621676) {
   out_594815173963621676[0] = 1;
   out_594815173963621676[1] = 0;
   out_594815173963621676[2] = 0;
   out_594815173963621676[3] = 0;
   out_594815173963621676[4] = 0;
   out_594815173963621676[5] = 0;
   out_594815173963621676[6] = 0;
   out_594815173963621676[7] = 0;
   out_594815173963621676[8] = 0;
   out_594815173963621676[9] = 0;
   out_594815173963621676[10] = 1;
   out_594815173963621676[11] = 0;
   out_594815173963621676[12] = 0;
   out_594815173963621676[13] = 0;
   out_594815173963621676[14] = 0;
   out_594815173963621676[15] = 0;
   out_594815173963621676[16] = 0;
   out_594815173963621676[17] = 0;
   out_594815173963621676[18] = 0;
   out_594815173963621676[19] = 0;
   out_594815173963621676[20] = 1;
   out_594815173963621676[21] = 0;
   out_594815173963621676[22] = 0;
   out_594815173963621676[23] = 0;
   out_594815173963621676[24] = 0;
   out_594815173963621676[25] = 0;
   out_594815173963621676[26] = 0;
   out_594815173963621676[27] = 0;
   out_594815173963621676[28] = 0;
   out_594815173963621676[29] = 0;
   out_594815173963621676[30] = 1;
   out_594815173963621676[31] = 0;
   out_594815173963621676[32] = 0;
   out_594815173963621676[33] = 0;
   out_594815173963621676[34] = 0;
   out_594815173963621676[35] = 0;
   out_594815173963621676[36] = 0;
   out_594815173963621676[37] = 0;
   out_594815173963621676[38] = 0;
   out_594815173963621676[39] = 0;
   out_594815173963621676[40] = 1;
   out_594815173963621676[41] = 0;
   out_594815173963621676[42] = 0;
   out_594815173963621676[43] = 0;
   out_594815173963621676[44] = 0;
   out_594815173963621676[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_594815173963621676[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_594815173963621676[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_594815173963621676[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_594815173963621676[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_594815173963621676[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_594815173963621676[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_594815173963621676[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_594815173963621676[53] = -9.8000000000000007*dt;
   out_594815173963621676[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_594815173963621676[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_594815173963621676[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_594815173963621676[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_594815173963621676[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_594815173963621676[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_594815173963621676[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_594815173963621676[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_594815173963621676[62] = 0;
   out_594815173963621676[63] = 0;
   out_594815173963621676[64] = 0;
   out_594815173963621676[65] = 0;
   out_594815173963621676[66] = 0;
   out_594815173963621676[67] = 0;
   out_594815173963621676[68] = 0;
   out_594815173963621676[69] = 0;
   out_594815173963621676[70] = 1;
   out_594815173963621676[71] = 0;
   out_594815173963621676[72] = 0;
   out_594815173963621676[73] = 0;
   out_594815173963621676[74] = 0;
   out_594815173963621676[75] = 0;
   out_594815173963621676[76] = 0;
   out_594815173963621676[77] = 0;
   out_594815173963621676[78] = 0;
   out_594815173963621676[79] = 0;
   out_594815173963621676[80] = 1;
}
void h_25(double *state, double *unused, double *out_4420541495512235573) {
   out_4420541495512235573[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6091906089981428809) {
   out_6091906089981428809[0] = 0;
   out_6091906089981428809[1] = 0;
   out_6091906089981428809[2] = 0;
   out_6091906089981428809[3] = 0;
   out_6091906089981428809[4] = 0;
   out_6091906089981428809[5] = 0;
   out_6091906089981428809[6] = 1;
   out_6091906089981428809[7] = 0;
   out_6091906089981428809[8] = 0;
}
void h_24(double *state, double *unused, double *out_6466529401607636301) {
   out_6466529401607636301[0] = state[4];
   out_6466529401607636301[1] = state[5];
}
void H_24(double *state, double *unused, double *out_109126589869732617) {
   out_109126589869732617[0] = 0;
   out_109126589869732617[1] = 0;
   out_109126589869732617[2] = 0;
   out_109126589869732617[3] = 0;
   out_109126589869732617[4] = 1;
   out_109126589869732617[5] = 0;
   out_109126589869732617[6] = 0;
   out_109126589869732617[7] = 0;
   out_109126589869732617[8] = 0;
   out_109126589869732617[9] = 0;
   out_109126589869732617[10] = 0;
   out_109126589869732617[11] = 0;
   out_109126589869732617[12] = 0;
   out_109126589869732617[13] = 0;
   out_109126589869732617[14] = 1;
   out_109126589869732617[15] = 0;
   out_109126589869732617[16] = 0;
   out_109126589869732617[17] = 0;
}
void h_30(double *state, double *unused, double *out_6014389683595920632) {
   out_6014389683595920632[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1564209759853820611) {
   out_1564209759853820611[0] = 0;
   out_1564209759853820611[1] = 0;
   out_1564209759853820611[2] = 0;
   out_1564209759853820611[3] = 0;
   out_1564209759853820611[4] = 1;
   out_1564209759853820611[5] = 0;
   out_1564209759853820611[6] = 0;
   out_1564209759853820611[7] = 0;
   out_1564209759853820611[8] = 0;
}
void h_26(double *state, double *unused, double *out_6841939516446877451) {
   out_6841939516446877451[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2350402771107372585) {
   out_2350402771107372585[0] = 0;
   out_2350402771107372585[1] = 0;
   out_2350402771107372585[2] = 0;
   out_2350402771107372585[3] = 0;
   out_2350402771107372585[4] = 0;
   out_2350402771107372585[5] = 0;
   out_2350402771107372585[6] = 0;
   out_2350402771107372585[7] = 1;
   out_2350402771107372585[8] = 0;
}
void h_27(double *state, double *unused, double *out_7457327546956568258) {
   out_7457327546956568258[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3787803831037763828) {
   out_3787803831037763828[0] = 0;
   out_3787803831037763828[1] = 0;
   out_3787803831037763828[2] = 0;
   out_3787803831037763828[3] = 1;
   out_3787803831037763828[4] = 0;
   out_3787803831037763828[5] = 0;
   out_3787803831037763828[6] = 0;
   out_3787803831037763828[7] = 0;
   out_3787803831037763828[8] = 0;
}
void h_29(double *state, double *unused, double *out_7182133484672062369) {
   out_7182133484672062369[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2074441104168212795) {
   out_2074441104168212795[0] = 0;
   out_2074441104168212795[1] = 1;
   out_2074441104168212795[2] = 0;
   out_2074441104168212795[3] = 0;
   out_2074441104168212795[4] = 0;
   out_2074441104168212795[5] = 0;
   out_2074441104168212795[6] = 0;
   out_2074441104168212795[7] = 0;
   out_2074441104168212795[8] = 0;
}
void h_28(double *state, double *unused, double *out_9147775381692249218) {
   out_9147775381692249218[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4038071375733539046) {
   out_4038071375733539046[0] = 1;
   out_4038071375733539046[1] = 0;
   out_4038071375733539046[2] = 0;
   out_4038071375733539046[3] = 0;
   out_4038071375733539046[4] = 0;
   out_4038071375733539046[5] = 0;
   out_4038071375733539046[6] = 0;
   out_4038071375733539046[7] = 0;
   out_4038071375733539046[8] = 0;
}
void h_31(double *state, double *unused, double *out_3962340133353673911) {
   out_3962340133353673911[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1724194668874021109) {
   out_1724194668874021109[0] = 0;
   out_1724194668874021109[1] = 0;
   out_1724194668874021109[2] = 0;
   out_1724194668874021109[3] = 0;
   out_1724194668874021109[4] = 0;
   out_1724194668874021109[5] = 0;
   out_1724194668874021109[6] = 0;
   out_1724194668874021109[7] = 0;
   out_1724194668874021109[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2673684484490617892) {
  err_fun(nom_x, delta_x, out_2673684484490617892);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_230892919190133727) {
  inv_err_fun(nom_x, true_x, out_230892919190133727);
}
void car_H_mod_fun(double *state, double *out_3465232717738919044) {
  H_mod_fun(state, out_3465232717738919044);
}
void car_f_fun(double *state, double dt, double *out_3047169393026212329) {
  f_fun(state,  dt, out_3047169393026212329);
}
void car_F_fun(double *state, double dt, double *out_594815173963621676) {
  F_fun(state,  dt, out_594815173963621676);
}
void car_h_25(double *state, double *unused, double *out_4420541495512235573) {
  h_25(state, unused, out_4420541495512235573);
}
void car_H_25(double *state, double *unused, double *out_6091906089981428809) {
  H_25(state, unused, out_6091906089981428809);
}
void car_h_24(double *state, double *unused, double *out_6466529401607636301) {
  h_24(state, unused, out_6466529401607636301);
}
void car_H_24(double *state, double *unused, double *out_109126589869732617) {
  H_24(state, unused, out_109126589869732617);
}
void car_h_30(double *state, double *unused, double *out_6014389683595920632) {
  h_30(state, unused, out_6014389683595920632);
}
void car_H_30(double *state, double *unused, double *out_1564209759853820611) {
  H_30(state, unused, out_1564209759853820611);
}
void car_h_26(double *state, double *unused, double *out_6841939516446877451) {
  h_26(state, unused, out_6841939516446877451);
}
void car_H_26(double *state, double *unused, double *out_2350402771107372585) {
  H_26(state, unused, out_2350402771107372585);
}
void car_h_27(double *state, double *unused, double *out_7457327546956568258) {
  h_27(state, unused, out_7457327546956568258);
}
void car_H_27(double *state, double *unused, double *out_3787803831037763828) {
  H_27(state, unused, out_3787803831037763828);
}
void car_h_29(double *state, double *unused, double *out_7182133484672062369) {
  h_29(state, unused, out_7182133484672062369);
}
void car_H_29(double *state, double *unused, double *out_2074441104168212795) {
  H_29(state, unused, out_2074441104168212795);
}
void car_h_28(double *state, double *unused, double *out_9147775381692249218) {
  h_28(state, unused, out_9147775381692249218);
}
void car_H_28(double *state, double *unused, double *out_4038071375733539046) {
  H_28(state, unused, out_4038071375733539046);
}
void car_h_31(double *state, double *unused, double *out_3962340133353673911) {
  h_31(state, unused, out_3962340133353673911);
}
void car_H_31(double *state, double *unused, double *out_1724194668874021109) {
  H_31(state, unused, out_1724194668874021109);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
