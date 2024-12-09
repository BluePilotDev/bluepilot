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
void err_fun(double *nom_x, double *delta_x, double *out_2126247490056086034) {
   out_2126247490056086034[0] = delta_x[0] + nom_x[0];
   out_2126247490056086034[1] = delta_x[1] + nom_x[1];
   out_2126247490056086034[2] = delta_x[2] + nom_x[2];
   out_2126247490056086034[3] = delta_x[3] + nom_x[3];
   out_2126247490056086034[4] = delta_x[4] + nom_x[4];
   out_2126247490056086034[5] = delta_x[5] + nom_x[5];
   out_2126247490056086034[6] = delta_x[6] + nom_x[6];
   out_2126247490056086034[7] = delta_x[7] + nom_x[7];
   out_2126247490056086034[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4537834234680995888) {
   out_4537834234680995888[0] = -nom_x[0] + true_x[0];
   out_4537834234680995888[1] = -nom_x[1] + true_x[1];
   out_4537834234680995888[2] = -nom_x[2] + true_x[2];
   out_4537834234680995888[3] = -nom_x[3] + true_x[3];
   out_4537834234680995888[4] = -nom_x[4] + true_x[4];
   out_4537834234680995888[5] = -nom_x[5] + true_x[5];
   out_4537834234680995888[6] = -nom_x[6] + true_x[6];
   out_4537834234680995888[7] = -nom_x[7] + true_x[7];
   out_4537834234680995888[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6000733833928406280) {
   out_6000733833928406280[0] = 1.0;
   out_6000733833928406280[1] = 0;
   out_6000733833928406280[2] = 0;
   out_6000733833928406280[3] = 0;
   out_6000733833928406280[4] = 0;
   out_6000733833928406280[5] = 0;
   out_6000733833928406280[6] = 0;
   out_6000733833928406280[7] = 0;
   out_6000733833928406280[8] = 0;
   out_6000733833928406280[9] = 0;
   out_6000733833928406280[10] = 1.0;
   out_6000733833928406280[11] = 0;
   out_6000733833928406280[12] = 0;
   out_6000733833928406280[13] = 0;
   out_6000733833928406280[14] = 0;
   out_6000733833928406280[15] = 0;
   out_6000733833928406280[16] = 0;
   out_6000733833928406280[17] = 0;
   out_6000733833928406280[18] = 0;
   out_6000733833928406280[19] = 0;
   out_6000733833928406280[20] = 1.0;
   out_6000733833928406280[21] = 0;
   out_6000733833928406280[22] = 0;
   out_6000733833928406280[23] = 0;
   out_6000733833928406280[24] = 0;
   out_6000733833928406280[25] = 0;
   out_6000733833928406280[26] = 0;
   out_6000733833928406280[27] = 0;
   out_6000733833928406280[28] = 0;
   out_6000733833928406280[29] = 0;
   out_6000733833928406280[30] = 1.0;
   out_6000733833928406280[31] = 0;
   out_6000733833928406280[32] = 0;
   out_6000733833928406280[33] = 0;
   out_6000733833928406280[34] = 0;
   out_6000733833928406280[35] = 0;
   out_6000733833928406280[36] = 0;
   out_6000733833928406280[37] = 0;
   out_6000733833928406280[38] = 0;
   out_6000733833928406280[39] = 0;
   out_6000733833928406280[40] = 1.0;
   out_6000733833928406280[41] = 0;
   out_6000733833928406280[42] = 0;
   out_6000733833928406280[43] = 0;
   out_6000733833928406280[44] = 0;
   out_6000733833928406280[45] = 0;
   out_6000733833928406280[46] = 0;
   out_6000733833928406280[47] = 0;
   out_6000733833928406280[48] = 0;
   out_6000733833928406280[49] = 0;
   out_6000733833928406280[50] = 1.0;
   out_6000733833928406280[51] = 0;
   out_6000733833928406280[52] = 0;
   out_6000733833928406280[53] = 0;
   out_6000733833928406280[54] = 0;
   out_6000733833928406280[55] = 0;
   out_6000733833928406280[56] = 0;
   out_6000733833928406280[57] = 0;
   out_6000733833928406280[58] = 0;
   out_6000733833928406280[59] = 0;
   out_6000733833928406280[60] = 1.0;
   out_6000733833928406280[61] = 0;
   out_6000733833928406280[62] = 0;
   out_6000733833928406280[63] = 0;
   out_6000733833928406280[64] = 0;
   out_6000733833928406280[65] = 0;
   out_6000733833928406280[66] = 0;
   out_6000733833928406280[67] = 0;
   out_6000733833928406280[68] = 0;
   out_6000733833928406280[69] = 0;
   out_6000733833928406280[70] = 1.0;
   out_6000733833928406280[71] = 0;
   out_6000733833928406280[72] = 0;
   out_6000733833928406280[73] = 0;
   out_6000733833928406280[74] = 0;
   out_6000733833928406280[75] = 0;
   out_6000733833928406280[76] = 0;
   out_6000733833928406280[77] = 0;
   out_6000733833928406280[78] = 0;
   out_6000733833928406280[79] = 0;
   out_6000733833928406280[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8298138295092552985) {
   out_8298138295092552985[0] = state[0];
   out_8298138295092552985[1] = state[1];
   out_8298138295092552985[2] = state[2];
   out_8298138295092552985[3] = state[3];
   out_8298138295092552985[4] = state[4];
   out_8298138295092552985[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8298138295092552985[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8298138295092552985[7] = state[7];
   out_8298138295092552985[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2104160686533451639) {
   out_2104160686533451639[0] = 1;
   out_2104160686533451639[1] = 0;
   out_2104160686533451639[2] = 0;
   out_2104160686533451639[3] = 0;
   out_2104160686533451639[4] = 0;
   out_2104160686533451639[5] = 0;
   out_2104160686533451639[6] = 0;
   out_2104160686533451639[7] = 0;
   out_2104160686533451639[8] = 0;
   out_2104160686533451639[9] = 0;
   out_2104160686533451639[10] = 1;
   out_2104160686533451639[11] = 0;
   out_2104160686533451639[12] = 0;
   out_2104160686533451639[13] = 0;
   out_2104160686533451639[14] = 0;
   out_2104160686533451639[15] = 0;
   out_2104160686533451639[16] = 0;
   out_2104160686533451639[17] = 0;
   out_2104160686533451639[18] = 0;
   out_2104160686533451639[19] = 0;
   out_2104160686533451639[20] = 1;
   out_2104160686533451639[21] = 0;
   out_2104160686533451639[22] = 0;
   out_2104160686533451639[23] = 0;
   out_2104160686533451639[24] = 0;
   out_2104160686533451639[25] = 0;
   out_2104160686533451639[26] = 0;
   out_2104160686533451639[27] = 0;
   out_2104160686533451639[28] = 0;
   out_2104160686533451639[29] = 0;
   out_2104160686533451639[30] = 1;
   out_2104160686533451639[31] = 0;
   out_2104160686533451639[32] = 0;
   out_2104160686533451639[33] = 0;
   out_2104160686533451639[34] = 0;
   out_2104160686533451639[35] = 0;
   out_2104160686533451639[36] = 0;
   out_2104160686533451639[37] = 0;
   out_2104160686533451639[38] = 0;
   out_2104160686533451639[39] = 0;
   out_2104160686533451639[40] = 1;
   out_2104160686533451639[41] = 0;
   out_2104160686533451639[42] = 0;
   out_2104160686533451639[43] = 0;
   out_2104160686533451639[44] = 0;
   out_2104160686533451639[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2104160686533451639[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2104160686533451639[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2104160686533451639[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2104160686533451639[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2104160686533451639[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2104160686533451639[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2104160686533451639[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2104160686533451639[53] = -9.8000000000000007*dt;
   out_2104160686533451639[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2104160686533451639[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2104160686533451639[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2104160686533451639[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2104160686533451639[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2104160686533451639[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2104160686533451639[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2104160686533451639[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2104160686533451639[62] = 0;
   out_2104160686533451639[63] = 0;
   out_2104160686533451639[64] = 0;
   out_2104160686533451639[65] = 0;
   out_2104160686533451639[66] = 0;
   out_2104160686533451639[67] = 0;
   out_2104160686533451639[68] = 0;
   out_2104160686533451639[69] = 0;
   out_2104160686533451639[70] = 1;
   out_2104160686533451639[71] = 0;
   out_2104160686533451639[72] = 0;
   out_2104160686533451639[73] = 0;
   out_2104160686533451639[74] = 0;
   out_2104160686533451639[75] = 0;
   out_2104160686533451639[76] = 0;
   out_2104160686533451639[77] = 0;
   out_2104160686533451639[78] = 0;
   out_2104160686533451639[79] = 0;
   out_2104160686533451639[80] = 1;
}
void h_25(double *state, double *unused, double *out_8159579304704009575) {
   out_8159579304704009575[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7089238263802843305) {
   out_7089238263802843305[0] = 0;
   out_7089238263802843305[1] = 0;
   out_7089238263802843305[2] = 0;
   out_7089238263802843305[3] = 0;
   out_7089238263802843305[4] = 0;
   out_7089238263802843305[5] = 0;
   out_7089238263802843305[6] = 1;
   out_7089238263802843305[7] = 0;
   out_7089238263802843305[8] = 0;
}
void h_24(double *state, double *unused, double *out_2365486038585008535) {
   out_2365486038585008535[0] = state[4];
   out_2365486038585008535[1] = state[5];
}
void H_24(double *state, double *unused, double *out_9184856210901208745) {
   out_9184856210901208745[0] = 0;
   out_9184856210901208745[1] = 0;
   out_9184856210901208745[2] = 0;
   out_9184856210901208745[3] = 0;
   out_9184856210901208745[4] = 1;
   out_9184856210901208745[5] = 0;
   out_9184856210901208745[6] = 0;
   out_9184856210901208745[7] = 0;
   out_9184856210901208745[8] = 0;
   out_9184856210901208745[9] = 0;
   out_9184856210901208745[10] = 0;
   out_9184856210901208745[11] = 0;
   out_9184856210901208745[12] = 0;
   out_9184856210901208745[13] = 0;
   out_9184856210901208745[14] = 1;
   out_9184856210901208745[15] = 0;
   out_9184856210901208745[16] = 0;
   out_9184856210901208745[17] = 0;
}
void h_30(double *state, double *unused, double *out_8434773366988515464) {
   out_8434773366988515464[0] = state[4];
}
void H_30(double *state, double *unused, double *out_172547922311226550) {
   out_172547922311226550[0] = 0;
   out_172547922311226550[1] = 0;
   out_172547922311226550[2] = 0;
   out_172547922311226550[3] = 0;
   out_172547922311226550[4] = 1;
   out_172547922311226550[5] = 0;
   out_172547922311226550[6] = 0;
   out_172547922311226550[7] = 0;
   out_172547922311226550[8] = 0;
}
void h_26(double *state, double *unused, double *out_8329297290277347245) {
   out_8329297290277347245[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7616002491032652087) {
   out_7616002491032652087[0] = 0;
   out_7616002491032652087[1] = 0;
   out_7616002491032652087[2] = 0;
   out_7616002491032652087[3] = 0;
   out_7616002491032652087[4] = 0;
   out_7616002491032652087[5] = 0;
   out_7616002491032652087[6] = 0;
   out_7616002491032652087[7] = 1;
   out_7616002491032652087[8] = 0;
}
void h_27(double *state, double *unused, double *out_7713909259767656438) {
   out_7713909259767656438[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2347311234111651461) {
   out_2347311234111651461[0] = 0;
   out_2347311234111651461[1] = 0;
   out_2347311234111651461[2] = 0;
   out_2347311234111651461[3] = 1;
   out_2347311234111651461[4] = 0;
   out_2347311234111651461[5] = 0;
   out_2347311234111651461[6] = 0;
   out_2347311234111651461[7] = 0;
   out_2347311234111651461[8] = 0;
}
void h_29(double *state, double *unused, double *out_5622221263718662081) {
   out_5622221263718662081[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4060673960981202494) {
   out_4060673960981202494[0] = 0;
   out_4060673960981202494[1] = 1;
   out_4060673960981202494[2] = 0;
   out_4060673960981202494[3] = 0;
   out_4060673960981202494[4] = 0;
   out_4060673960981202494[5] = 0;
   out_4060673960981202494[6] = 0;
   out_4060673960981202494[7] = 0;
   out_4060673960981202494[8] = 0;
}
void h_28(double *state, double *unused, double *out_5109503938958958787) {
   out_5109503938958958787[0] = state[0];
}
void H_28(double *state, double *unused, double *out_9143072978050733068) {
   out_9143072978050733068[0] = 1;
   out_9143072978050733068[1] = 0;
   out_9143072978050733068[2] = 0;
   out_9143072978050733068[3] = 0;
   out_9143072978050733068[4] = 0;
   out_9143072978050733068[5] = 0;
   out_9143072978050733068[6] = 0;
   out_9143072978050733068[7] = 0;
   out_9143072978050733068[8] = 0;
}
void h_31(double *state, double *unused, double *out_1149447484321690859) {
   out_1149447484321690859[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7058592301925882877) {
   out_7058592301925882877[0] = 0;
   out_7058592301925882877[1] = 0;
   out_7058592301925882877[2] = 0;
   out_7058592301925882877[3] = 0;
   out_7058592301925882877[4] = 0;
   out_7058592301925882877[5] = 0;
   out_7058592301925882877[6] = 0;
   out_7058592301925882877[7] = 0;
   out_7058592301925882877[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2126247490056086034) {
  err_fun(nom_x, delta_x, out_2126247490056086034);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4537834234680995888) {
  inv_err_fun(nom_x, true_x, out_4537834234680995888);
}
void car_H_mod_fun(double *state, double *out_6000733833928406280) {
  H_mod_fun(state, out_6000733833928406280);
}
void car_f_fun(double *state, double dt, double *out_8298138295092552985) {
  f_fun(state,  dt, out_8298138295092552985);
}
void car_F_fun(double *state, double dt, double *out_2104160686533451639) {
  F_fun(state,  dt, out_2104160686533451639);
}
void car_h_25(double *state, double *unused, double *out_8159579304704009575) {
  h_25(state, unused, out_8159579304704009575);
}
void car_H_25(double *state, double *unused, double *out_7089238263802843305) {
  H_25(state, unused, out_7089238263802843305);
}
void car_h_24(double *state, double *unused, double *out_2365486038585008535) {
  h_24(state, unused, out_2365486038585008535);
}
void car_H_24(double *state, double *unused, double *out_9184856210901208745) {
  H_24(state, unused, out_9184856210901208745);
}
void car_h_30(double *state, double *unused, double *out_8434773366988515464) {
  h_30(state, unused, out_8434773366988515464);
}
void car_H_30(double *state, double *unused, double *out_172547922311226550) {
  H_30(state, unused, out_172547922311226550);
}
void car_h_26(double *state, double *unused, double *out_8329297290277347245) {
  h_26(state, unused, out_8329297290277347245);
}
void car_H_26(double *state, double *unused, double *out_7616002491032652087) {
  H_26(state, unused, out_7616002491032652087);
}
void car_h_27(double *state, double *unused, double *out_7713909259767656438) {
  h_27(state, unused, out_7713909259767656438);
}
void car_H_27(double *state, double *unused, double *out_2347311234111651461) {
  H_27(state, unused, out_2347311234111651461);
}
void car_h_29(double *state, double *unused, double *out_5622221263718662081) {
  h_29(state, unused, out_5622221263718662081);
}
void car_H_29(double *state, double *unused, double *out_4060673960981202494) {
  H_29(state, unused, out_4060673960981202494);
}
void car_h_28(double *state, double *unused, double *out_5109503938958958787) {
  h_28(state, unused, out_5109503938958958787);
}
void car_H_28(double *state, double *unused, double *out_9143072978050733068) {
  H_28(state, unused, out_9143072978050733068);
}
void car_h_31(double *state, double *unused, double *out_1149447484321690859) {
  h_31(state, unused, out_1149447484321690859);
}
void car_H_31(double *state, double *unused, double *out_7058592301925882877) {
  H_31(state, unused, out_7058592301925882877);
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
