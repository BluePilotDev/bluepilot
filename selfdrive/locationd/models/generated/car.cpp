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
void err_fun(double *nom_x, double *delta_x, double *out_5346541195336492582) {
   out_5346541195336492582[0] = delta_x[0] + nom_x[0];
   out_5346541195336492582[1] = delta_x[1] + nom_x[1];
   out_5346541195336492582[2] = delta_x[2] + nom_x[2];
   out_5346541195336492582[3] = delta_x[3] + nom_x[3];
   out_5346541195336492582[4] = delta_x[4] + nom_x[4];
   out_5346541195336492582[5] = delta_x[5] + nom_x[5];
   out_5346541195336492582[6] = delta_x[6] + nom_x[6];
   out_5346541195336492582[7] = delta_x[7] + nom_x[7];
   out_5346541195336492582[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_337463137437584546) {
   out_337463137437584546[0] = -nom_x[0] + true_x[0];
   out_337463137437584546[1] = -nom_x[1] + true_x[1];
   out_337463137437584546[2] = -nom_x[2] + true_x[2];
   out_337463137437584546[3] = -nom_x[3] + true_x[3];
   out_337463137437584546[4] = -nom_x[4] + true_x[4];
   out_337463137437584546[5] = -nom_x[5] + true_x[5];
   out_337463137437584546[6] = -nom_x[6] + true_x[6];
   out_337463137437584546[7] = -nom_x[7] + true_x[7];
   out_337463137437584546[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_982522059003493621) {
   out_982522059003493621[0] = 1.0;
   out_982522059003493621[1] = 0;
   out_982522059003493621[2] = 0;
   out_982522059003493621[3] = 0;
   out_982522059003493621[4] = 0;
   out_982522059003493621[5] = 0;
   out_982522059003493621[6] = 0;
   out_982522059003493621[7] = 0;
   out_982522059003493621[8] = 0;
   out_982522059003493621[9] = 0;
   out_982522059003493621[10] = 1.0;
   out_982522059003493621[11] = 0;
   out_982522059003493621[12] = 0;
   out_982522059003493621[13] = 0;
   out_982522059003493621[14] = 0;
   out_982522059003493621[15] = 0;
   out_982522059003493621[16] = 0;
   out_982522059003493621[17] = 0;
   out_982522059003493621[18] = 0;
   out_982522059003493621[19] = 0;
   out_982522059003493621[20] = 1.0;
   out_982522059003493621[21] = 0;
   out_982522059003493621[22] = 0;
   out_982522059003493621[23] = 0;
   out_982522059003493621[24] = 0;
   out_982522059003493621[25] = 0;
   out_982522059003493621[26] = 0;
   out_982522059003493621[27] = 0;
   out_982522059003493621[28] = 0;
   out_982522059003493621[29] = 0;
   out_982522059003493621[30] = 1.0;
   out_982522059003493621[31] = 0;
   out_982522059003493621[32] = 0;
   out_982522059003493621[33] = 0;
   out_982522059003493621[34] = 0;
   out_982522059003493621[35] = 0;
   out_982522059003493621[36] = 0;
   out_982522059003493621[37] = 0;
   out_982522059003493621[38] = 0;
   out_982522059003493621[39] = 0;
   out_982522059003493621[40] = 1.0;
   out_982522059003493621[41] = 0;
   out_982522059003493621[42] = 0;
   out_982522059003493621[43] = 0;
   out_982522059003493621[44] = 0;
   out_982522059003493621[45] = 0;
   out_982522059003493621[46] = 0;
   out_982522059003493621[47] = 0;
   out_982522059003493621[48] = 0;
   out_982522059003493621[49] = 0;
   out_982522059003493621[50] = 1.0;
   out_982522059003493621[51] = 0;
   out_982522059003493621[52] = 0;
   out_982522059003493621[53] = 0;
   out_982522059003493621[54] = 0;
   out_982522059003493621[55] = 0;
   out_982522059003493621[56] = 0;
   out_982522059003493621[57] = 0;
   out_982522059003493621[58] = 0;
   out_982522059003493621[59] = 0;
   out_982522059003493621[60] = 1.0;
   out_982522059003493621[61] = 0;
   out_982522059003493621[62] = 0;
   out_982522059003493621[63] = 0;
   out_982522059003493621[64] = 0;
   out_982522059003493621[65] = 0;
   out_982522059003493621[66] = 0;
   out_982522059003493621[67] = 0;
   out_982522059003493621[68] = 0;
   out_982522059003493621[69] = 0;
   out_982522059003493621[70] = 1.0;
   out_982522059003493621[71] = 0;
   out_982522059003493621[72] = 0;
   out_982522059003493621[73] = 0;
   out_982522059003493621[74] = 0;
   out_982522059003493621[75] = 0;
   out_982522059003493621[76] = 0;
   out_982522059003493621[77] = 0;
   out_982522059003493621[78] = 0;
   out_982522059003493621[79] = 0;
   out_982522059003493621[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3404434725552331721) {
   out_3404434725552331721[0] = state[0];
   out_3404434725552331721[1] = state[1];
   out_3404434725552331721[2] = state[2];
   out_3404434725552331721[3] = state[3];
   out_3404434725552331721[4] = state[4];
   out_3404434725552331721[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3404434725552331721[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3404434725552331721[7] = state[7];
   out_3404434725552331721[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2484277118229527537) {
   out_2484277118229527537[0] = 1;
   out_2484277118229527537[1] = 0;
   out_2484277118229527537[2] = 0;
   out_2484277118229527537[3] = 0;
   out_2484277118229527537[4] = 0;
   out_2484277118229527537[5] = 0;
   out_2484277118229527537[6] = 0;
   out_2484277118229527537[7] = 0;
   out_2484277118229527537[8] = 0;
   out_2484277118229527537[9] = 0;
   out_2484277118229527537[10] = 1;
   out_2484277118229527537[11] = 0;
   out_2484277118229527537[12] = 0;
   out_2484277118229527537[13] = 0;
   out_2484277118229527537[14] = 0;
   out_2484277118229527537[15] = 0;
   out_2484277118229527537[16] = 0;
   out_2484277118229527537[17] = 0;
   out_2484277118229527537[18] = 0;
   out_2484277118229527537[19] = 0;
   out_2484277118229527537[20] = 1;
   out_2484277118229527537[21] = 0;
   out_2484277118229527537[22] = 0;
   out_2484277118229527537[23] = 0;
   out_2484277118229527537[24] = 0;
   out_2484277118229527537[25] = 0;
   out_2484277118229527537[26] = 0;
   out_2484277118229527537[27] = 0;
   out_2484277118229527537[28] = 0;
   out_2484277118229527537[29] = 0;
   out_2484277118229527537[30] = 1;
   out_2484277118229527537[31] = 0;
   out_2484277118229527537[32] = 0;
   out_2484277118229527537[33] = 0;
   out_2484277118229527537[34] = 0;
   out_2484277118229527537[35] = 0;
   out_2484277118229527537[36] = 0;
   out_2484277118229527537[37] = 0;
   out_2484277118229527537[38] = 0;
   out_2484277118229527537[39] = 0;
   out_2484277118229527537[40] = 1;
   out_2484277118229527537[41] = 0;
   out_2484277118229527537[42] = 0;
   out_2484277118229527537[43] = 0;
   out_2484277118229527537[44] = 0;
   out_2484277118229527537[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2484277118229527537[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2484277118229527537[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2484277118229527537[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2484277118229527537[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2484277118229527537[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2484277118229527537[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2484277118229527537[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2484277118229527537[53] = -9.8000000000000007*dt;
   out_2484277118229527537[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2484277118229527537[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2484277118229527537[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2484277118229527537[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2484277118229527537[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2484277118229527537[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2484277118229527537[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2484277118229527537[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2484277118229527537[62] = 0;
   out_2484277118229527537[63] = 0;
   out_2484277118229527537[64] = 0;
   out_2484277118229527537[65] = 0;
   out_2484277118229527537[66] = 0;
   out_2484277118229527537[67] = 0;
   out_2484277118229527537[68] = 0;
   out_2484277118229527537[69] = 0;
   out_2484277118229527537[70] = 1;
   out_2484277118229527537[71] = 0;
   out_2484277118229527537[72] = 0;
   out_2484277118229527537[73] = 0;
   out_2484277118229527537[74] = 0;
   out_2484277118229527537[75] = 0;
   out_2484277118229527537[76] = 0;
   out_2484277118229527537[77] = 0;
   out_2484277118229527537[78] = 0;
   out_2484277118229527537[79] = 0;
   out_2484277118229527537[80] = 1;
}
void h_25(double *state, double *unused, double *out_8809730598441262111) {
   out_8809730598441262111[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8728899811385676204) {
   out_8728899811385676204[0] = 0;
   out_8728899811385676204[1] = 0;
   out_8728899811385676204[2] = 0;
   out_8728899811385676204[3] = 0;
   out_8728899811385676204[4] = 0;
   out_8728899811385676204[5] = 0;
   out_8728899811385676204[6] = 1;
   out_8728899811385676204[7] = 0;
   out_8728899811385676204[8] = 0;
}
void h_24(double *state, double *unused, double *out_3938368130868346857) {
   out_3938368130868346857[0] = state[4];
   out_3938368130868346857[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5170974212583353068) {
   out_5170974212583353068[0] = 0;
   out_5170974212583353068[1] = 0;
   out_5170974212583353068[2] = 0;
   out_5170974212583353068[3] = 0;
   out_5170974212583353068[4] = 1;
   out_5170974212583353068[5] = 0;
   out_5170974212583353068[6] = 0;
   out_5170974212583353068[7] = 0;
   out_5170974212583353068[8] = 0;
   out_5170974212583353068[9] = 0;
   out_5170974212583353068[10] = 0;
   out_5170974212583353068[11] = 0;
   out_5170974212583353068[12] = 0;
   out_5170974212583353068[13] = 0;
   out_5170974212583353068[14] = 1;
   out_5170974212583353068[15] = 0;
   out_5170974212583353068[16] = 0;
   out_5170974212583353068[17] = 0;
}
void h_30(double *state, double *unused, double *out_8966917266792391256) {
   out_8966917266792391256[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8858238758528916274) {
   out_8858238758528916274[0] = 0;
   out_8858238758528916274[1] = 0;
   out_8858238758528916274[2] = 0;
   out_8858238758528916274[3] = 0;
   out_8858238758528916274[4] = 1;
   out_8858238758528916274[5] = 0;
   out_8858238758528916274[6] = 0;
   out_8858238758528916274[7] = 0;
   out_8858238758528916274[8] = 0;
}
void h_26(double *state, double *unused, double *out_4478153539256314715) {
   out_4478153539256314715[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5976340943449819188) {
   out_5976340943449819188[0] = 0;
   out_5976340943449819188[1] = 0;
   out_5976340943449819188[2] = 0;
   out_5976340943449819188[3] = 0;
   out_5976340943449819188[4] = 0;
   out_5976340943449819188[5] = 0;
   out_5976340943449819188[6] = 0;
   out_5976340943449819188[7] = 1;
   out_5976340943449819188[8] = 0;
}
void h_27(double *state, double *unused, double *out_3139591893343702755) {
   out_3139591893343702755[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7413742003380210431) {
   out_7413742003380210431[0] = 0;
   out_7413742003380210431[1] = 0;
   out_7413742003380210431[2] = 0;
   out_7413742003380210431[3] = 1;
   out_7413742003380210431[4] = 0;
   out_7413742003380210431[5] = 0;
   out_7413742003380210431[6] = 0;
   out_7413742003380210431[7] = 0;
   out_7413742003380210431[8] = 0;
}
void h_29(double *state, double *unused, double *out_2864397831059196866) {
   out_2864397831059196866[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5700379276510659398) {
   out_5700379276510659398[0] = 0;
   out_5700379276510659398[1] = 1;
   out_5700379276510659398[2] = 0;
   out_5700379276510659398[3] = 0;
   out_5700379276510659398[4] = 0;
   out_5700379276510659398[5] = 0;
   out_5700379276510659398[6] = 0;
   out_5700379276510659398[7] = 0;
   out_5700379276510659398[8] = 0;
}
void h_28(double *state, double *unused, double *out_8304389476572783082) {
   out_8304389476572783082[0] = state[0];
}
void H_28(double *state, double *unused, double *out_617980259441128824) {
   out_617980259441128824[0] = 1;
   out_617980259441128824[1] = 0;
   out_617980259441128824[2] = 0;
   out_617980259441128824[3] = 0;
   out_617980259441128824[4] = 0;
   out_617980259441128824[5] = 0;
   out_617980259441128824[6] = 0;
   out_617980259441128824[7] = 0;
   out_617980259441128824[8] = 0;
}
void h_31(double *state, double *unused, double *out_248325666120025041) {
   out_248325666120025041[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8698253849508715776) {
   out_8698253849508715776[0] = 0;
   out_8698253849508715776[1] = 0;
   out_8698253849508715776[2] = 0;
   out_8698253849508715776[3] = 0;
   out_8698253849508715776[4] = 0;
   out_8698253849508715776[5] = 0;
   out_8698253849508715776[6] = 0;
   out_8698253849508715776[7] = 0;
   out_8698253849508715776[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5346541195336492582) {
  err_fun(nom_x, delta_x, out_5346541195336492582);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_337463137437584546) {
  inv_err_fun(nom_x, true_x, out_337463137437584546);
}
void car_H_mod_fun(double *state, double *out_982522059003493621) {
  H_mod_fun(state, out_982522059003493621);
}
void car_f_fun(double *state, double dt, double *out_3404434725552331721) {
  f_fun(state,  dt, out_3404434725552331721);
}
void car_F_fun(double *state, double dt, double *out_2484277118229527537) {
  F_fun(state,  dt, out_2484277118229527537);
}
void car_h_25(double *state, double *unused, double *out_8809730598441262111) {
  h_25(state, unused, out_8809730598441262111);
}
void car_H_25(double *state, double *unused, double *out_8728899811385676204) {
  H_25(state, unused, out_8728899811385676204);
}
void car_h_24(double *state, double *unused, double *out_3938368130868346857) {
  h_24(state, unused, out_3938368130868346857);
}
void car_H_24(double *state, double *unused, double *out_5170974212583353068) {
  H_24(state, unused, out_5170974212583353068);
}
void car_h_30(double *state, double *unused, double *out_8966917266792391256) {
  h_30(state, unused, out_8966917266792391256);
}
void car_H_30(double *state, double *unused, double *out_8858238758528916274) {
  H_30(state, unused, out_8858238758528916274);
}
void car_h_26(double *state, double *unused, double *out_4478153539256314715) {
  h_26(state, unused, out_4478153539256314715);
}
void car_H_26(double *state, double *unused, double *out_5976340943449819188) {
  H_26(state, unused, out_5976340943449819188);
}
void car_h_27(double *state, double *unused, double *out_3139591893343702755) {
  h_27(state, unused, out_3139591893343702755);
}
void car_H_27(double *state, double *unused, double *out_7413742003380210431) {
  H_27(state, unused, out_7413742003380210431);
}
void car_h_29(double *state, double *unused, double *out_2864397831059196866) {
  h_29(state, unused, out_2864397831059196866);
}
void car_H_29(double *state, double *unused, double *out_5700379276510659398) {
  H_29(state, unused, out_5700379276510659398);
}
void car_h_28(double *state, double *unused, double *out_8304389476572783082) {
  h_28(state, unused, out_8304389476572783082);
}
void car_H_28(double *state, double *unused, double *out_617980259441128824) {
  H_28(state, unused, out_617980259441128824);
}
void car_h_31(double *state, double *unused, double *out_248325666120025041) {
  h_31(state, unused, out_248325666120025041);
}
void car_H_31(double *state, double *unused, double *out_8698253849508715776) {
  H_31(state, unused, out_8698253849508715776);
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
