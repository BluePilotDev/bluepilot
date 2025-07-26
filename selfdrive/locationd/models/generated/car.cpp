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
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5404643359353745497) {
   out_5404643359353745497[0] = delta_x[0] + nom_x[0];
   out_5404643359353745497[1] = delta_x[1] + nom_x[1];
   out_5404643359353745497[2] = delta_x[2] + nom_x[2];
   out_5404643359353745497[3] = delta_x[3] + nom_x[3];
   out_5404643359353745497[4] = delta_x[4] + nom_x[4];
   out_5404643359353745497[5] = delta_x[5] + nom_x[5];
   out_5404643359353745497[6] = delta_x[6] + nom_x[6];
   out_5404643359353745497[7] = delta_x[7] + nom_x[7];
   out_5404643359353745497[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5558015477020821215) {
   out_5558015477020821215[0] = -nom_x[0] + true_x[0];
   out_5558015477020821215[1] = -nom_x[1] + true_x[1];
   out_5558015477020821215[2] = -nom_x[2] + true_x[2];
   out_5558015477020821215[3] = -nom_x[3] + true_x[3];
   out_5558015477020821215[4] = -nom_x[4] + true_x[4];
   out_5558015477020821215[5] = -nom_x[5] + true_x[5];
   out_5558015477020821215[6] = -nom_x[6] + true_x[6];
   out_5558015477020821215[7] = -nom_x[7] + true_x[7];
   out_5558015477020821215[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8366575207087506668) {
   out_8366575207087506668[0] = 1.0;
   out_8366575207087506668[1] = 0.0;
   out_8366575207087506668[2] = 0.0;
   out_8366575207087506668[3] = 0.0;
   out_8366575207087506668[4] = 0.0;
   out_8366575207087506668[5] = 0.0;
   out_8366575207087506668[6] = 0.0;
   out_8366575207087506668[7] = 0.0;
   out_8366575207087506668[8] = 0.0;
   out_8366575207087506668[9] = 0.0;
   out_8366575207087506668[10] = 1.0;
   out_8366575207087506668[11] = 0.0;
   out_8366575207087506668[12] = 0.0;
   out_8366575207087506668[13] = 0.0;
   out_8366575207087506668[14] = 0.0;
   out_8366575207087506668[15] = 0.0;
   out_8366575207087506668[16] = 0.0;
   out_8366575207087506668[17] = 0.0;
   out_8366575207087506668[18] = 0.0;
   out_8366575207087506668[19] = 0.0;
   out_8366575207087506668[20] = 1.0;
   out_8366575207087506668[21] = 0.0;
   out_8366575207087506668[22] = 0.0;
   out_8366575207087506668[23] = 0.0;
   out_8366575207087506668[24] = 0.0;
   out_8366575207087506668[25] = 0.0;
   out_8366575207087506668[26] = 0.0;
   out_8366575207087506668[27] = 0.0;
   out_8366575207087506668[28] = 0.0;
   out_8366575207087506668[29] = 0.0;
   out_8366575207087506668[30] = 1.0;
   out_8366575207087506668[31] = 0.0;
   out_8366575207087506668[32] = 0.0;
   out_8366575207087506668[33] = 0.0;
   out_8366575207087506668[34] = 0.0;
   out_8366575207087506668[35] = 0.0;
   out_8366575207087506668[36] = 0.0;
   out_8366575207087506668[37] = 0.0;
   out_8366575207087506668[38] = 0.0;
   out_8366575207087506668[39] = 0.0;
   out_8366575207087506668[40] = 1.0;
   out_8366575207087506668[41] = 0.0;
   out_8366575207087506668[42] = 0.0;
   out_8366575207087506668[43] = 0.0;
   out_8366575207087506668[44] = 0.0;
   out_8366575207087506668[45] = 0.0;
   out_8366575207087506668[46] = 0.0;
   out_8366575207087506668[47] = 0.0;
   out_8366575207087506668[48] = 0.0;
   out_8366575207087506668[49] = 0.0;
   out_8366575207087506668[50] = 1.0;
   out_8366575207087506668[51] = 0.0;
   out_8366575207087506668[52] = 0.0;
   out_8366575207087506668[53] = 0.0;
   out_8366575207087506668[54] = 0.0;
   out_8366575207087506668[55] = 0.0;
   out_8366575207087506668[56] = 0.0;
   out_8366575207087506668[57] = 0.0;
   out_8366575207087506668[58] = 0.0;
   out_8366575207087506668[59] = 0.0;
   out_8366575207087506668[60] = 1.0;
   out_8366575207087506668[61] = 0.0;
   out_8366575207087506668[62] = 0.0;
   out_8366575207087506668[63] = 0.0;
   out_8366575207087506668[64] = 0.0;
   out_8366575207087506668[65] = 0.0;
   out_8366575207087506668[66] = 0.0;
   out_8366575207087506668[67] = 0.0;
   out_8366575207087506668[68] = 0.0;
   out_8366575207087506668[69] = 0.0;
   out_8366575207087506668[70] = 1.0;
   out_8366575207087506668[71] = 0.0;
   out_8366575207087506668[72] = 0.0;
   out_8366575207087506668[73] = 0.0;
   out_8366575207087506668[74] = 0.0;
   out_8366575207087506668[75] = 0.0;
   out_8366575207087506668[76] = 0.0;
   out_8366575207087506668[77] = 0.0;
   out_8366575207087506668[78] = 0.0;
   out_8366575207087506668[79] = 0.0;
   out_8366575207087506668[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1949015885477314473) {
   out_1949015885477314473[0] = state[0];
   out_1949015885477314473[1] = state[1];
   out_1949015885477314473[2] = state[2];
   out_1949015885477314473[3] = state[3];
   out_1949015885477314473[4] = state[4];
   out_1949015885477314473[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1949015885477314473[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1949015885477314473[7] = state[7];
   out_1949015885477314473[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2631349548067560569) {
   out_2631349548067560569[0] = 1;
   out_2631349548067560569[1] = 0;
   out_2631349548067560569[2] = 0;
   out_2631349548067560569[3] = 0;
   out_2631349548067560569[4] = 0;
   out_2631349548067560569[5] = 0;
   out_2631349548067560569[6] = 0;
   out_2631349548067560569[7] = 0;
   out_2631349548067560569[8] = 0;
   out_2631349548067560569[9] = 0;
   out_2631349548067560569[10] = 1;
   out_2631349548067560569[11] = 0;
   out_2631349548067560569[12] = 0;
   out_2631349548067560569[13] = 0;
   out_2631349548067560569[14] = 0;
   out_2631349548067560569[15] = 0;
   out_2631349548067560569[16] = 0;
   out_2631349548067560569[17] = 0;
   out_2631349548067560569[18] = 0;
   out_2631349548067560569[19] = 0;
   out_2631349548067560569[20] = 1;
   out_2631349548067560569[21] = 0;
   out_2631349548067560569[22] = 0;
   out_2631349548067560569[23] = 0;
   out_2631349548067560569[24] = 0;
   out_2631349548067560569[25] = 0;
   out_2631349548067560569[26] = 0;
   out_2631349548067560569[27] = 0;
   out_2631349548067560569[28] = 0;
   out_2631349548067560569[29] = 0;
   out_2631349548067560569[30] = 1;
   out_2631349548067560569[31] = 0;
   out_2631349548067560569[32] = 0;
   out_2631349548067560569[33] = 0;
   out_2631349548067560569[34] = 0;
   out_2631349548067560569[35] = 0;
   out_2631349548067560569[36] = 0;
   out_2631349548067560569[37] = 0;
   out_2631349548067560569[38] = 0;
   out_2631349548067560569[39] = 0;
   out_2631349548067560569[40] = 1;
   out_2631349548067560569[41] = 0;
   out_2631349548067560569[42] = 0;
   out_2631349548067560569[43] = 0;
   out_2631349548067560569[44] = 0;
   out_2631349548067560569[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2631349548067560569[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2631349548067560569[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2631349548067560569[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2631349548067560569[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2631349548067560569[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2631349548067560569[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2631349548067560569[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2631349548067560569[53] = -9.8000000000000007*dt;
   out_2631349548067560569[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2631349548067560569[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2631349548067560569[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2631349548067560569[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2631349548067560569[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2631349548067560569[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2631349548067560569[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2631349548067560569[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2631349548067560569[62] = 0;
   out_2631349548067560569[63] = 0;
   out_2631349548067560569[64] = 0;
   out_2631349548067560569[65] = 0;
   out_2631349548067560569[66] = 0;
   out_2631349548067560569[67] = 0;
   out_2631349548067560569[68] = 0;
   out_2631349548067560569[69] = 0;
   out_2631349548067560569[70] = 1;
   out_2631349548067560569[71] = 0;
   out_2631349548067560569[72] = 0;
   out_2631349548067560569[73] = 0;
   out_2631349548067560569[74] = 0;
   out_2631349548067560569[75] = 0;
   out_2631349548067560569[76] = 0;
   out_2631349548067560569[77] = 0;
   out_2631349548067560569[78] = 0;
   out_2631349548067560569[79] = 0;
   out_2631349548067560569[80] = 1;
}
void h_25(double *state, double *unused, double *out_2124478671266214726) {
   out_2124478671266214726[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7861240764887209956) {
   out_7861240764887209956[0] = 0;
   out_7861240764887209956[1] = 0;
   out_7861240764887209956[2] = 0;
   out_7861240764887209956[3] = 0;
   out_7861240764887209956[4] = 0;
   out_7861240764887209956[5] = 0;
   out_7861240764887209956[6] = 1;
   out_7861240764887209956[7] = 0;
   out_7861240764887209956[8] = 0;
}
void h_24(double *state, double *unused, double *out_7714703363907166482) {
   out_7714703363907166482[0] = state[4];
   out_7714703363907166482[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8331698246930548680) {
   out_8331698246930548680[0] = 0;
   out_8331698246930548680[1] = 0;
   out_8331698246930548680[2] = 0;
   out_8331698246930548680[3] = 0;
   out_8331698246930548680[4] = 1;
   out_8331698246930548680[5] = 0;
   out_8331698246930548680[6] = 0;
   out_8331698246930548680[7] = 0;
   out_8331698246930548680[8] = 0;
   out_8331698246930548680[9] = 0;
   out_8331698246930548680[10] = 0;
   out_8331698246930548680[11] = 0;
   out_8331698246930548680[12] = 0;
   out_8331698246930548680[13] = 0;
   out_8331698246930548680[14] = 1;
   out_8331698246930548680[15] = 0;
   out_8331698246930548680[16] = 0;
   out_8331698246930548680[17] = 0;
}
void h_30(double *state, double *unused, double *out_8895313897616565662) {
   out_8895313897616565662[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7990579712030450026) {
   out_7990579712030450026[0] = 0;
   out_7990579712030450026[1] = 0;
   out_7990579712030450026[2] = 0;
   out_7990579712030450026[3] = 0;
   out_7990579712030450026[4] = 1;
   out_7990579712030450026[5] = 0;
   out_7990579712030450026[6] = 0;
   out_7990579712030450026[7] = 0;
   out_7990579712030450026[8] = 0;
}
void h_26(double *state, double *unused, double *out_755120711826988814) {
   out_755120711826988814[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6843999989948285436) {
   out_6843999989948285436[0] = 0;
   out_6843999989948285436[1] = 0;
   out_6843999989948285436[2] = 0;
   out_6843999989948285436[3] = 0;
   out_6843999989948285436[4] = 0;
   out_6843999989948285436[5] = 0;
   out_6843999989948285436[6] = 0;
   out_6843999989948285436[7] = 1;
   out_6843999989948285436[8] = 0;
}
void h_27(double *state, double *unused, double *out_3067988524167877161) {
   out_3067988524167877161[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8281401049878676679) {
   out_8281401049878676679[0] = 0;
   out_8281401049878676679[1] = 0;
   out_8281401049878676679[2] = 0;
   out_8281401049878676679[3] = 1;
   out_8281401049878676679[4] = 0;
   out_8281401049878676679[5] = 0;
   out_8281401049878676679[6] = 0;
   out_8281401049878676679[7] = 0;
   out_8281401049878676679[8] = 0;
}
void h_29(double *state, double *unused, double *out_6738878072823132571) {
   out_6738878072823132571[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7480348367716057842) {
   out_7480348367716057842[0] = 0;
   out_7480348367716057842[1] = 1;
   out_7480348367716057842[2] = 0;
   out_7480348367716057842[3] = 0;
   out_7480348367716057842[4] = 0;
   out_7480348367716057842[5] = 0;
   out_7480348367716057842[6] = 0;
   out_7480348367716057842[7] = 0;
   out_7480348367716057842[8] = 0;
}
void h_28(double *state, double *unused, double *out_1933684743492968484) {
   out_1933684743492968484[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5516718096150731591) {
   out_5516718096150731591[0] = 1;
   out_5516718096150731591[1] = 0;
   out_5516718096150731591[2] = 0;
   out_5516718096150731591[3] = 0;
   out_5516718096150731591[4] = 0;
   out_5516718096150731591[5] = 0;
   out_5516718096150731591[6] = 0;
   out_5516718096150731591[7] = 0;
   out_5516718096150731591[8] = 0;
}
void h_31(double *state, double *unused, double *out_5072204661621699687) {
   out_5072204661621699687[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7830594803010249528) {
   out_7830594803010249528[0] = 0;
   out_7830594803010249528[1] = 0;
   out_7830594803010249528[2] = 0;
   out_7830594803010249528[3] = 0;
   out_7830594803010249528[4] = 0;
   out_7830594803010249528[5] = 0;
   out_7830594803010249528[6] = 0;
   out_7830594803010249528[7] = 0;
   out_7830594803010249528[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5404643359353745497) {
  err_fun(nom_x, delta_x, out_5404643359353745497);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5558015477020821215) {
  inv_err_fun(nom_x, true_x, out_5558015477020821215);
}
void car_H_mod_fun(double *state, double *out_8366575207087506668) {
  H_mod_fun(state, out_8366575207087506668);
}
void car_f_fun(double *state, double dt, double *out_1949015885477314473) {
  f_fun(state,  dt, out_1949015885477314473);
}
void car_F_fun(double *state, double dt, double *out_2631349548067560569) {
  F_fun(state,  dt, out_2631349548067560569);
}
void car_h_25(double *state, double *unused, double *out_2124478671266214726) {
  h_25(state, unused, out_2124478671266214726);
}
void car_H_25(double *state, double *unused, double *out_7861240764887209956) {
  H_25(state, unused, out_7861240764887209956);
}
void car_h_24(double *state, double *unused, double *out_7714703363907166482) {
  h_24(state, unused, out_7714703363907166482);
}
void car_H_24(double *state, double *unused, double *out_8331698246930548680) {
  H_24(state, unused, out_8331698246930548680);
}
void car_h_30(double *state, double *unused, double *out_8895313897616565662) {
  h_30(state, unused, out_8895313897616565662);
}
void car_H_30(double *state, double *unused, double *out_7990579712030450026) {
  H_30(state, unused, out_7990579712030450026);
}
void car_h_26(double *state, double *unused, double *out_755120711826988814) {
  h_26(state, unused, out_755120711826988814);
}
void car_H_26(double *state, double *unused, double *out_6843999989948285436) {
  H_26(state, unused, out_6843999989948285436);
}
void car_h_27(double *state, double *unused, double *out_3067988524167877161) {
  h_27(state, unused, out_3067988524167877161);
}
void car_H_27(double *state, double *unused, double *out_8281401049878676679) {
  H_27(state, unused, out_8281401049878676679);
}
void car_h_29(double *state, double *unused, double *out_6738878072823132571) {
  h_29(state, unused, out_6738878072823132571);
}
void car_H_29(double *state, double *unused, double *out_7480348367716057842) {
  H_29(state, unused, out_7480348367716057842);
}
void car_h_28(double *state, double *unused, double *out_1933684743492968484) {
  h_28(state, unused, out_1933684743492968484);
}
void car_H_28(double *state, double *unused, double *out_5516718096150731591) {
  H_28(state, unused, out_5516718096150731591);
}
void car_h_31(double *state, double *unused, double *out_5072204661621699687) {
  h_31(state, unused, out_5072204661621699687);
}
void car_H_31(double *state, double *unused, double *out_7830594803010249528) {
  H_31(state, unused, out_7830594803010249528);
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
