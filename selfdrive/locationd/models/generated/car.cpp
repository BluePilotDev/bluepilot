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
void err_fun(double *nom_x, double *delta_x, double *out_7044851216775548991) {
   out_7044851216775548991[0] = delta_x[0] + nom_x[0];
   out_7044851216775548991[1] = delta_x[1] + nom_x[1];
   out_7044851216775548991[2] = delta_x[2] + nom_x[2];
   out_7044851216775548991[3] = delta_x[3] + nom_x[3];
   out_7044851216775548991[4] = delta_x[4] + nom_x[4];
   out_7044851216775548991[5] = delta_x[5] + nom_x[5];
   out_7044851216775548991[6] = delta_x[6] + nom_x[6];
   out_7044851216775548991[7] = delta_x[7] + nom_x[7];
   out_7044851216775548991[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7289501170211592416) {
   out_7289501170211592416[0] = -nom_x[0] + true_x[0];
   out_7289501170211592416[1] = -nom_x[1] + true_x[1];
   out_7289501170211592416[2] = -nom_x[2] + true_x[2];
   out_7289501170211592416[3] = -nom_x[3] + true_x[3];
   out_7289501170211592416[4] = -nom_x[4] + true_x[4];
   out_7289501170211592416[5] = -nom_x[5] + true_x[5];
   out_7289501170211592416[6] = -nom_x[6] + true_x[6];
   out_7289501170211592416[7] = -nom_x[7] + true_x[7];
   out_7289501170211592416[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7606443429705972179) {
   out_7606443429705972179[0] = 1.0;
   out_7606443429705972179[1] = 0;
   out_7606443429705972179[2] = 0;
   out_7606443429705972179[3] = 0;
   out_7606443429705972179[4] = 0;
   out_7606443429705972179[5] = 0;
   out_7606443429705972179[6] = 0;
   out_7606443429705972179[7] = 0;
   out_7606443429705972179[8] = 0;
   out_7606443429705972179[9] = 0;
   out_7606443429705972179[10] = 1.0;
   out_7606443429705972179[11] = 0;
   out_7606443429705972179[12] = 0;
   out_7606443429705972179[13] = 0;
   out_7606443429705972179[14] = 0;
   out_7606443429705972179[15] = 0;
   out_7606443429705972179[16] = 0;
   out_7606443429705972179[17] = 0;
   out_7606443429705972179[18] = 0;
   out_7606443429705972179[19] = 0;
   out_7606443429705972179[20] = 1.0;
   out_7606443429705972179[21] = 0;
   out_7606443429705972179[22] = 0;
   out_7606443429705972179[23] = 0;
   out_7606443429705972179[24] = 0;
   out_7606443429705972179[25] = 0;
   out_7606443429705972179[26] = 0;
   out_7606443429705972179[27] = 0;
   out_7606443429705972179[28] = 0;
   out_7606443429705972179[29] = 0;
   out_7606443429705972179[30] = 1.0;
   out_7606443429705972179[31] = 0;
   out_7606443429705972179[32] = 0;
   out_7606443429705972179[33] = 0;
   out_7606443429705972179[34] = 0;
   out_7606443429705972179[35] = 0;
   out_7606443429705972179[36] = 0;
   out_7606443429705972179[37] = 0;
   out_7606443429705972179[38] = 0;
   out_7606443429705972179[39] = 0;
   out_7606443429705972179[40] = 1.0;
   out_7606443429705972179[41] = 0;
   out_7606443429705972179[42] = 0;
   out_7606443429705972179[43] = 0;
   out_7606443429705972179[44] = 0;
   out_7606443429705972179[45] = 0;
   out_7606443429705972179[46] = 0;
   out_7606443429705972179[47] = 0;
   out_7606443429705972179[48] = 0;
   out_7606443429705972179[49] = 0;
   out_7606443429705972179[50] = 1.0;
   out_7606443429705972179[51] = 0;
   out_7606443429705972179[52] = 0;
   out_7606443429705972179[53] = 0;
   out_7606443429705972179[54] = 0;
   out_7606443429705972179[55] = 0;
   out_7606443429705972179[56] = 0;
   out_7606443429705972179[57] = 0;
   out_7606443429705972179[58] = 0;
   out_7606443429705972179[59] = 0;
   out_7606443429705972179[60] = 1.0;
   out_7606443429705972179[61] = 0;
   out_7606443429705972179[62] = 0;
   out_7606443429705972179[63] = 0;
   out_7606443429705972179[64] = 0;
   out_7606443429705972179[65] = 0;
   out_7606443429705972179[66] = 0;
   out_7606443429705972179[67] = 0;
   out_7606443429705972179[68] = 0;
   out_7606443429705972179[69] = 0;
   out_7606443429705972179[70] = 1.0;
   out_7606443429705972179[71] = 0;
   out_7606443429705972179[72] = 0;
   out_7606443429705972179[73] = 0;
   out_7606443429705972179[74] = 0;
   out_7606443429705972179[75] = 0;
   out_7606443429705972179[76] = 0;
   out_7606443429705972179[77] = 0;
   out_7606443429705972179[78] = 0;
   out_7606443429705972179[79] = 0;
   out_7606443429705972179[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7610697498145313227) {
   out_7610697498145313227[0] = state[0];
   out_7610697498145313227[1] = state[1];
   out_7610697498145313227[2] = state[2];
   out_7610697498145313227[3] = state[3];
   out_7610697498145313227[4] = state[4];
   out_7610697498145313227[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7610697498145313227[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7610697498145313227[7] = state[7];
   out_7610697498145313227[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6340369697461232580) {
   out_6340369697461232580[0] = 1;
   out_6340369697461232580[1] = 0;
   out_6340369697461232580[2] = 0;
   out_6340369697461232580[3] = 0;
   out_6340369697461232580[4] = 0;
   out_6340369697461232580[5] = 0;
   out_6340369697461232580[6] = 0;
   out_6340369697461232580[7] = 0;
   out_6340369697461232580[8] = 0;
   out_6340369697461232580[9] = 0;
   out_6340369697461232580[10] = 1;
   out_6340369697461232580[11] = 0;
   out_6340369697461232580[12] = 0;
   out_6340369697461232580[13] = 0;
   out_6340369697461232580[14] = 0;
   out_6340369697461232580[15] = 0;
   out_6340369697461232580[16] = 0;
   out_6340369697461232580[17] = 0;
   out_6340369697461232580[18] = 0;
   out_6340369697461232580[19] = 0;
   out_6340369697461232580[20] = 1;
   out_6340369697461232580[21] = 0;
   out_6340369697461232580[22] = 0;
   out_6340369697461232580[23] = 0;
   out_6340369697461232580[24] = 0;
   out_6340369697461232580[25] = 0;
   out_6340369697461232580[26] = 0;
   out_6340369697461232580[27] = 0;
   out_6340369697461232580[28] = 0;
   out_6340369697461232580[29] = 0;
   out_6340369697461232580[30] = 1;
   out_6340369697461232580[31] = 0;
   out_6340369697461232580[32] = 0;
   out_6340369697461232580[33] = 0;
   out_6340369697461232580[34] = 0;
   out_6340369697461232580[35] = 0;
   out_6340369697461232580[36] = 0;
   out_6340369697461232580[37] = 0;
   out_6340369697461232580[38] = 0;
   out_6340369697461232580[39] = 0;
   out_6340369697461232580[40] = 1;
   out_6340369697461232580[41] = 0;
   out_6340369697461232580[42] = 0;
   out_6340369697461232580[43] = 0;
   out_6340369697461232580[44] = 0;
   out_6340369697461232580[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6340369697461232580[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6340369697461232580[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6340369697461232580[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6340369697461232580[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6340369697461232580[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6340369697461232580[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6340369697461232580[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6340369697461232580[53] = -9.8000000000000007*dt;
   out_6340369697461232580[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6340369697461232580[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6340369697461232580[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6340369697461232580[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6340369697461232580[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6340369697461232580[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6340369697461232580[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6340369697461232580[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6340369697461232580[62] = 0;
   out_6340369697461232580[63] = 0;
   out_6340369697461232580[64] = 0;
   out_6340369697461232580[65] = 0;
   out_6340369697461232580[66] = 0;
   out_6340369697461232580[67] = 0;
   out_6340369697461232580[68] = 0;
   out_6340369697461232580[69] = 0;
   out_6340369697461232580[70] = 1;
   out_6340369697461232580[71] = 0;
   out_6340369697461232580[72] = 0;
   out_6340369697461232580[73] = 0;
   out_6340369697461232580[74] = 0;
   out_6340369697461232580[75] = 0;
   out_6340369697461232580[76] = 0;
   out_6340369697461232580[77] = 0;
   out_6340369697461232580[78] = 0;
   out_6340369697461232580[79] = 0;
   out_6340369697461232580[80] = 1;
}
void h_25(double *state, double *unused, double *out_187386210251484366) {
   out_187386210251484366[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3477348954605252497) {
   out_3477348954605252497[0] = 0;
   out_3477348954605252497[1] = 0;
   out_3477348954605252497[2] = 0;
   out_3477348954605252497[3] = 0;
   out_3477348954605252497[4] = 0;
   out_3477348954605252497[5] = 0;
   out_3477348954605252497[6] = 1;
   out_3477348954605252497[7] = 0;
   out_3477348954605252497[8] = 0;
}
void h_24(double *state, double *unused, double *out_6741112897477616196) {
   out_6741112897477616196[0] = state[4];
   out_6741112897477616196[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7122041108230277057) {
   out_7122041108230277057[0] = 0;
   out_7122041108230277057[1] = 0;
   out_7122041108230277057[2] = 0;
   out_7122041108230277057[3] = 0;
   out_7122041108230277057[4] = 1;
   out_7122041108230277057[5] = 0;
   out_7122041108230277057[6] = 0;
   out_7122041108230277057[7] = 0;
   out_7122041108230277057[8] = 0;
   out_7122041108230277057[9] = 0;
   out_7122041108230277057[10] = 0;
   out_7122041108230277057[11] = 0;
   out_7122041108230277057[12] = 0;
   out_7122041108230277057[13] = 0;
   out_7122041108230277057[14] = 1;
   out_7122041108230277057[15] = 0;
   out_7122041108230277057[16] = 0;
   out_7122041108230277057[17] = 0;
}
void h_30(double *state, double *unused, double *out_3450063318002537984) {
   out_3450063318002537984[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1050347375522355701) {
   out_1050347375522355701[0] = 0;
   out_1050347375522355701[1] = 0;
   out_1050347375522355701[2] = 0;
   out_1050347375522355701[3] = 0;
   out_1050347375522355701[4] = 1;
   out_1050347375522355701[5] = 0;
   out_1050347375522355701[6] = 0;
   out_1050347375522355701[7] = 0;
   out_1050347375522355701[8] = 0;
}
void h_26(double *state, double *unused, double *out_1551609949658307661) {
   out_1551609949658307661[0] = state[7];
}
void H_26(double *state, double *unused, double *out_264154364268803727) {
   out_264154364268803727[0] = 0;
   out_264154364268803727[1] = 0;
   out_264154364268803727[2] = 0;
   out_264154364268803727[3] = 0;
   out_264154364268803727[4] = 0;
   out_264154364268803727[5] = 0;
   out_264154364268803727[6] = 0;
   out_264154364268803727[7] = 1;
   out_264154364268803727[8] = 0;
}
void h_27(double *state, double *unused, double *out_1484421420982351135) {
   out_1484421420982351135[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3225110687322780612) {
   out_3225110687322780612[0] = 0;
   out_3225110687322780612[1] = 0;
   out_3225110687322780612[2] = 0;
   out_3225110687322780612[3] = 1;
   out_3225110687322780612[4] = 0;
   out_3225110687322780612[5] = 0;
   out_3225110687322780612[6] = 0;
   out_3225110687322780612[7] = 0;
   out_3225110687322780612[8] = 0;
}
void h_29(double *state, double *unused, double *out_8451736585663383629) {
   out_8451736585663383629[0] = state[1];
}
void H_29(double *state, double *unused, double *out_540116031207963517) {
   out_540116031207963517[0] = 0;
   out_540116031207963517[1] = 1;
   out_540116031207963517[2] = 0;
   out_540116031207963517[3] = 0;
   out_540116031207963517[4] = 0;
   out_540116031207963517[5] = 0;
   out_540116031207963517[6] = 0;
   out_540116031207963517[7] = 0;
   out_540116031207963517[8] = 0;
}
void h_28(double *state, double *unused, double *out_4167682114597479486) {
   out_4167682114597479486[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1423514240357362734) {
   out_1423514240357362734[0] = 1;
   out_1423514240357362734[1] = 0;
   out_1423514240357362734[2] = 0;
   out_1423514240357362734[3] = 0;
   out_1423514240357362734[4] = 0;
   out_1423514240357362734[5] = 0;
   out_1423514240357362734[6] = 0;
   out_1423514240357362734[7] = 0;
   out_1423514240357362734[8] = 0;
}
void h_31(double *state, double *unused, double *out_7273172553997403959) {
   out_7273172553997403959[0] = state[8];
}
void H_31(double *state, double *unused, double *out_890362466502155203) {
   out_890362466502155203[0] = 0;
   out_890362466502155203[1] = 0;
   out_890362466502155203[2] = 0;
   out_890362466502155203[3] = 0;
   out_890362466502155203[4] = 0;
   out_890362466502155203[5] = 0;
   out_890362466502155203[6] = 0;
   out_890362466502155203[7] = 0;
   out_890362466502155203[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7044851216775548991) {
  err_fun(nom_x, delta_x, out_7044851216775548991);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7289501170211592416) {
  inv_err_fun(nom_x, true_x, out_7289501170211592416);
}
void car_H_mod_fun(double *state, double *out_7606443429705972179) {
  H_mod_fun(state, out_7606443429705972179);
}
void car_f_fun(double *state, double dt, double *out_7610697498145313227) {
  f_fun(state,  dt, out_7610697498145313227);
}
void car_F_fun(double *state, double dt, double *out_6340369697461232580) {
  F_fun(state,  dt, out_6340369697461232580);
}
void car_h_25(double *state, double *unused, double *out_187386210251484366) {
  h_25(state, unused, out_187386210251484366);
}
void car_H_25(double *state, double *unused, double *out_3477348954605252497) {
  H_25(state, unused, out_3477348954605252497);
}
void car_h_24(double *state, double *unused, double *out_6741112897477616196) {
  h_24(state, unused, out_6741112897477616196);
}
void car_H_24(double *state, double *unused, double *out_7122041108230277057) {
  H_24(state, unused, out_7122041108230277057);
}
void car_h_30(double *state, double *unused, double *out_3450063318002537984) {
  h_30(state, unused, out_3450063318002537984);
}
void car_H_30(double *state, double *unused, double *out_1050347375522355701) {
  H_30(state, unused, out_1050347375522355701);
}
void car_h_26(double *state, double *unused, double *out_1551609949658307661) {
  h_26(state, unused, out_1551609949658307661);
}
void car_H_26(double *state, double *unused, double *out_264154364268803727) {
  H_26(state, unused, out_264154364268803727);
}
void car_h_27(double *state, double *unused, double *out_1484421420982351135) {
  h_27(state, unused, out_1484421420982351135);
}
void car_H_27(double *state, double *unused, double *out_3225110687322780612) {
  H_27(state, unused, out_3225110687322780612);
}
void car_h_29(double *state, double *unused, double *out_8451736585663383629) {
  h_29(state, unused, out_8451736585663383629);
}
void car_H_29(double *state, double *unused, double *out_540116031207963517) {
  H_29(state, unused, out_540116031207963517);
}
void car_h_28(double *state, double *unused, double *out_4167682114597479486) {
  h_28(state, unused, out_4167682114597479486);
}
void car_H_28(double *state, double *unused, double *out_1423514240357362734) {
  H_28(state, unused, out_1423514240357362734);
}
void car_h_31(double *state, double *unused, double *out_7273172553997403959) {
  h_31(state, unused, out_7273172553997403959);
}
void car_H_31(double *state, double *unused, double *out_890362466502155203) {
  H_31(state, unused, out_890362466502155203);
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
