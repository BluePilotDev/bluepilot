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
void err_fun(double *nom_x, double *delta_x, double *out_8199342001798600637) {
   out_8199342001798600637[0] = delta_x[0] + nom_x[0];
   out_8199342001798600637[1] = delta_x[1] + nom_x[1];
   out_8199342001798600637[2] = delta_x[2] + nom_x[2];
   out_8199342001798600637[3] = delta_x[3] + nom_x[3];
   out_8199342001798600637[4] = delta_x[4] + nom_x[4];
   out_8199342001798600637[5] = delta_x[5] + nom_x[5];
   out_8199342001798600637[6] = delta_x[6] + nom_x[6];
   out_8199342001798600637[7] = delta_x[7] + nom_x[7];
   out_8199342001798600637[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4249540853661070632) {
   out_4249540853661070632[0] = -nom_x[0] + true_x[0];
   out_4249540853661070632[1] = -nom_x[1] + true_x[1];
   out_4249540853661070632[2] = -nom_x[2] + true_x[2];
   out_4249540853661070632[3] = -nom_x[3] + true_x[3];
   out_4249540853661070632[4] = -nom_x[4] + true_x[4];
   out_4249540853661070632[5] = -nom_x[5] + true_x[5];
   out_4249540853661070632[6] = -nom_x[6] + true_x[6];
   out_4249540853661070632[7] = -nom_x[7] + true_x[7];
   out_4249540853661070632[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6602744757649736994) {
   out_6602744757649736994[0] = 1.0;
   out_6602744757649736994[1] = 0;
   out_6602744757649736994[2] = 0;
   out_6602744757649736994[3] = 0;
   out_6602744757649736994[4] = 0;
   out_6602744757649736994[5] = 0;
   out_6602744757649736994[6] = 0;
   out_6602744757649736994[7] = 0;
   out_6602744757649736994[8] = 0;
   out_6602744757649736994[9] = 0;
   out_6602744757649736994[10] = 1.0;
   out_6602744757649736994[11] = 0;
   out_6602744757649736994[12] = 0;
   out_6602744757649736994[13] = 0;
   out_6602744757649736994[14] = 0;
   out_6602744757649736994[15] = 0;
   out_6602744757649736994[16] = 0;
   out_6602744757649736994[17] = 0;
   out_6602744757649736994[18] = 0;
   out_6602744757649736994[19] = 0;
   out_6602744757649736994[20] = 1.0;
   out_6602744757649736994[21] = 0;
   out_6602744757649736994[22] = 0;
   out_6602744757649736994[23] = 0;
   out_6602744757649736994[24] = 0;
   out_6602744757649736994[25] = 0;
   out_6602744757649736994[26] = 0;
   out_6602744757649736994[27] = 0;
   out_6602744757649736994[28] = 0;
   out_6602744757649736994[29] = 0;
   out_6602744757649736994[30] = 1.0;
   out_6602744757649736994[31] = 0;
   out_6602744757649736994[32] = 0;
   out_6602744757649736994[33] = 0;
   out_6602744757649736994[34] = 0;
   out_6602744757649736994[35] = 0;
   out_6602744757649736994[36] = 0;
   out_6602744757649736994[37] = 0;
   out_6602744757649736994[38] = 0;
   out_6602744757649736994[39] = 0;
   out_6602744757649736994[40] = 1.0;
   out_6602744757649736994[41] = 0;
   out_6602744757649736994[42] = 0;
   out_6602744757649736994[43] = 0;
   out_6602744757649736994[44] = 0;
   out_6602744757649736994[45] = 0;
   out_6602744757649736994[46] = 0;
   out_6602744757649736994[47] = 0;
   out_6602744757649736994[48] = 0;
   out_6602744757649736994[49] = 0;
   out_6602744757649736994[50] = 1.0;
   out_6602744757649736994[51] = 0;
   out_6602744757649736994[52] = 0;
   out_6602744757649736994[53] = 0;
   out_6602744757649736994[54] = 0;
   out_6602744757649736994[55] = 0;
   out_6602744757649736994[56] = 0;
   out_6602744757649736994[57] = 0;
   out_6602744757649736994[58] = 0;
   out_6602744757649736994[59] = 0;
   out_6602744757649736994[60] = 1.0;
   out_6602744757649736994[61] = 0;
   out_6602744757649736994[62] = 0;
   out_6602744757649736994[63] = 0;
   out_6602744757649736994[64] = 0;
   out_6602744757649736994[65] = 0;
   out_6602744757649736994[66] = 0;
   out_6602744757649736994[67] = 0;
   out_6602744757649736994[68] = 0;
   out_6602744757649736994[69] = 0;
   out_6602744757649736994[70] = 1.0;
   out_6602744757649736994[71] = 0;
   out_6602744757649736994[72] = 0;
   out_6602744757649736994[73] = 0;
   out_6602744757649736994[74] = 0;
   out_6602744757649736994[75] = 0;
   out_6602744757649736994[76] = 0;
   out_6602744757649736994[77] = 0;
   out_6602744757649736994[78] = 0;
   out_6602744757649736994[79] = 0;
   out_6602744757649736994[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7946137206928459356) {
   out_7946137206928459356[0] = state[0];
   out_7946137206928459356[1] = state[1];
   out_7946137206928459356[2] = state[2];
   out_7946137206928459356[3] = state[3];
   out_7946137206928459356[4] = state[4];
   out_7946137206928459356[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7946137206928459356[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7946137206928459356[7] = state[7];
   out_7946137206928459356[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2006759295838051036) {
   out_2006759295838051036[0] = 1;
   out_2006759295838051036[1] = 0;
   out_2006759295838051036[2] = 0;
   out_2006759295838051036[3] = 0;
   out_2006759295838051036[4] = 0;
   out_2006759295838051036[5] = 0;
   out_2006759295838051036[6] = 0;
   out_2006759295838051036[7] = 0;
   out_2006759295838051036[8] = 0;
   out_2006759295838051036[9] = 0;
   out_2006759295838051036[10] = 1;
   out_2006759295838051036[11] = 0;
   out_2006759295838051036[12] = 0;
   out_2006759295838051036[13] = 0;
   out_2006759295838051036[14] = 0;
   out_2006759295838051036[15] = 0;
   out_2006759295838051036[16] = 0;
   out_2006759295838051036[17] = 0;
   out_2006759295838051036[18] = 0;
   out_2006759295838051036[19] = 0;
   out_2006759295838051036[20] = 1;
   out_2006759295838051036[21] = 0;
   out_2006759295838051036[22] = 0;
   out_2006759295838051036[23] = 0;
   out_2006759295838051036[24] = 0;
   out_2006759295838051036[25] = 0;
   out_2006759295838051036[26] = 0;
   out_2006759295838051036[27] = 0;
   out_2006759295838051036[28] = 0;
   out_2006759295838051036[29] = 0;
   out_2006759295838051036[30] = 1;
   out_2006759295838051036[31] = 0;
   out_2006759295838051036[32] = 0;
   out_2006759295838051036[33] = 0;
   out_2006759295838051036[34] = 0;
   out_2006759295838051036[35] = 0;
   out_2006759295838051036[36] = 0;
   out_2006759295838051036[37] = 0;
   out_2006759295838051036[38] = 0;
   out_2006759295838051036[39] = 0;
   out_2006759295838051036[40] = 1;
   out_2006759295838051036[41] = 0;
   out_2006759295838051036[42] = 0;
   out_2006759295838051036[43] = 0;
   out_2006759295838051036[44] = 0;
   out_2006759295838051036[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2006759295838051036[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2006759295838051036[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2006759295838051036[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2006759295838051036[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2006759295838051036[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2006759295838051036[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2006759295838051036[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2006759295838051036[53] = -9.8000000000000007*dt;
   out_2006759295838051036[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2006759295838051036[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2006759295838051036[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2006759295838051036[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2006759295838051036[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2006759295838051036[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2006759295838051036[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2006759295838051036[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2006759295838051036[62] = 0;
   out_2006759295838051036[63] = 0;
   out_2006759295838051036[64] = 0;
   out_2006759295838051036[65] = 0;
   out_2006759295838051036[66] = 0;
   out_2006759295838051036[67] = 0;
   out_2006759295838051036[68] = 0;
   out_2006759295838051036[69] = 0;
   out_2006759295838051036[70] = 1;
   out_2006759295838051036[71] = 0;
   out_2006759295838051036[72] = 0;
   out_2006759295838051036[73] = 0;
   out_2006759295838051036[74] = 0;
   out_2006759295838051036[75] = 0;
   out_2006759295838051036[76] = 0;
   out_2006759295838051036[77] = 0;
   out_2006759295838051036[78] = 0;
   out_2006759295838051036[79] = 0;
   out_2006759295838051036[80] = 1;
}
void h_25(double *state, double *unused, double *out_5412618435515016142) {
   out_5412618435515016142[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3123731362841228441) {
   out_3123731362841228441[0] = 0;
   out_3123731362841228441[1] = 0;
   out_3123731362841228441[2] = 0;
   out_3123731362841228441[3] = 0;
   out_3123731362841228441[4] = 0;
   out_3123731362841228441[5] = 0;
   out_3123731362841228441[6] = 1;
   out_3123731362841228441[7] = 0;
   out_3123731362841228441[8] = 0;
}
void h_24(double *state, double *unused, double *out_2733287191012974439) {
   out_2733287191012974439[0] = state[4];
   out_2733287191012974439[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8668807252760137418) {
   out_8668807252760137418[0] = 0;
   out_8668807252760137418[1] = 0;
   out_8668807252760137418[2] = 0;
   out_8668807252760137418[3] = 0;
   out_8668807252760137418[4] = 1;
   out_8668807252760137418[5] = 0;
   out_8668807252760137418[6] = 0;
   out_8668807252760137418[7] = 0;
   out_8668807252760137418[8] = 0;
   out_8668807252760137418[9] = 0;
   out_8668807252760137418[10] = 0;
   out_8668807252760137418[11] = 0;
   out_8668807252760137418[12] = 0;
   out_8668807252760137418[13] = 0;
   out_8668807252760137418[14] = 1;
   out_8668807252760137418[15] = 0;
   out_8668807252760137418[16] = 0;
   out_8668807252760137418[17] = 0;
}
void h_30(double *state, double *unused, double *out_5137424373230510253) {
   out_5137424373230510253[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1403964967286379757) {
   out_1403964967286379757[0] = 0;
   out_1403964967286379757[1] = 0;
   out_1403964967286379757[2] = 0;
   out_1403964967286379757[3] = 0;
   out_1403964967286379757[4] = 1;
   out_1403964967286379757[5] = 0;
   out_1403964967286379757[6] = 0;
   out_1403964967286379757[7] = 0;
   out_1403964967286379757[8] = 0;
}
void h_26(double *state, double *unused, double *out_1641104382388076224) {
   out_1641104382388076224[0] = state[7];
}
void H_26(double *state, double *unused, double *out_617771956032827783) {
   out_617771956032827783[0] = 0;
   out_617771956032827783[1] = 0;
   out_617771956032827783[2] = 0;
   out_617771956032827783[3] = 0;
   out_617771956032827783[4] = 0;
   out_617771956032827783[5] = 0;
   out_617771956032827783[6] = 0;
   out_617771956032827783[7] = 1;
   out_617771956032827783[8] = 0;
}
void h_27(double *state, double *unused, double *out_465667052043698946) {
   out_465667052043698946[0] = state[3];
}
void H_27(double *state, double *unused, double *out_819629103897563460) {
   out_819629103897563460[0] = 0;
   out_819629103897563460[1] = 0;
   out_819629103897563460[2] = 0;
   out_819629103897563460[3] = 1;
   out_819629103897563460[4] = 0;
   out_819629103897563460[5] = 0;
   out_819629103897563460[6] = 0;
   out_819629103897563460[7] = 0;
   out_819629103897563460[8] = 0;
}
void h_29(double *state, double *unused, double *out_8174210424674842938) {
   out_8174210424674842938[0] = state[1];
}
void H_29(double *state, double *unused, double *out_893733622971987573) {
   out_893733622971987573[0] = 0;
   out_893733622971987573[1] = 1;
   out_893733622971987573[2] = 0;
   out_893733622971987573[3] = 0;
   out_893733622971987573[4] = 0;
   out_893733622971987573[5] = 0;
   out_893733622971987573[6] = 0;
   out_893733622971987573[7] = 0;
   out_893733622971987573[8] = 0;
}
void h_28(double *state, double *unused, double *out_5467340319704544591) {
   out_5467340319704544591[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5976132640041518147) {
   out_5976132640041518147[0] = 1;
   out_5976132640041518147[1] = 0;
   out_5976132640041518147[2] = 0;
   out_5976132640041518147[3] = 0;
   out_5976132640041518147[4] = 0;
   out_5976132640041518147[5] = 0;
   out_5976132640041518147[6] = 0;
   out_5976132640041518147[7] = 0;
   out_5976132640041518147[8] = 0;
}
void h_31(double *state, double *unused, double *out_3961987042886132975) {
   out_3961987042886132975[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1243980058266179259) {
   out_1243980058266179259[0] = 0;
   out_1243980058266179259[1] = 0;
   out_1243980058266179259[2] = 0;
   out_1243980058266179259[3] = 0;
   out_1243980058266179259[4] = 0;
   out_1243980058266179259[5] = 0;
   out_1243980058266179259[6] = 0;
   out_1243980058266179259[7] = 0;
   out_1243980058266179259[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8199342001798600637) {
  err_fun(nom_x, delta_x, out_8199342001798600637);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4249540853661070632) {
  inv_err_fun(nom_x, true_x, out_4249540853661070632);
}
void car_H_mod_fun(double *state, double *out_6602744757649736994) {
  H_mod_fun(state, out_6602744757649736994);
}
void car_f_fun(double *state, double dt, double *out_7946137206928459356) {
  f_fun(state,  dt, out_7946137206928459356);
}
void car_F_fun(double *state, double dt, double *out_2006759295838051036) {
  F_fun(state,  dt, out_2006759295838051036);
}
void car_h_25(double *state, double *unused, double *out_5412618435515016142) {
  h_25(state, unused, out_5412618435515016142);
}
void car_H_25(double *state, double *unused, double *out_3123731362841228441) {
  H_25(state, unused, out_3123731362841228441);
}
void car_h_24(double *state, double *unused, double *out_2733287191012974439) {
  h_24(state, unused, out_2733287191012974439);
}
void car_H_24(double *state, double *unused, double *out_8668807252760137418) {
  H_24(state, unused, out_8668807252760137418);
}
void car_h_30(double *state, double *unused, double *out_5137424373230510253) {
  h_30(state, unused, out_5137424373230510253);
}
void car_H_30(double *state, double *unused, double *out_1403964967286379757) {
  H_30(state, unused, out_1403964967286379757);
}
void car_h_26(double *state, double *unused, double *out_1641104382388076224) {
  h_26(state, unused, out_1641104382388076224);
}
void car_H_26(double *state, double *unused, double *out_617771956032827783) {
  H_26(state, unused, out_617771956032827783);
}
void car_h_27(double *state, double *unused, double *out_465667052043698946) {
  h_27(state, unused, out_465667052043698946);
}
void car_H_27(double *state, double *unused, double *out_819629103897563460) {
  H_27(state, unused, out_819629103897563460);
}
void car_h_29(double *state, double *unused, double *out_8174210424674842938) {
  h_29(state, unused, out_8174210424674842938);
}
void car_H_29(double *state, double *unused, double *out_893733622971987573) {
  H_29(state, unused, out_893733622971987573);
}
void car_h_28(double *state, double *unused, double *out_5467340319704544591) {
  h_28(state, unused, out_5467340319704544591);
}
void car_H_28(double *state, double *unused, double *out_5976132640041518147) {
  H_28(state, unused, out_5976132640041518147);
}
void car_h_31(double *state, double *unused, double *out_3961987042886132975) {
  h_31(state, unused, out_3961987042886132975);
}
void car_H_31(double *state, double *unused, double *out_1243980058266179259) {
  H_31(state, unused, out_1243980058266179259);
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
