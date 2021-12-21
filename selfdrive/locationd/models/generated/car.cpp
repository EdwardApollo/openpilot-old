#include "car.h"

namespace {
#define DIM 8
#define EDIM 8
#define MEDIM 8
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
const static double MAHA_THRESH_28 = 5.991464547107981;

/******************************************************************************
 *                       Code generated with sympy 1.8                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2597874447553363541) {
   out_2597874447553363541[0] = delta_x[0] + nom_x[0];
   out_2597874447553363541[1] = delta_x[1] + nom_x[1];
   out_2597874447553363541[2] = delta_x[2] + nom_x[2];
   out_2597874447553363541[3] = delta_x[3] + nom_x[3];
   out_2597874447553363541[4] = delta_x[4] + nom_x[4];
   out_2597874447553363541[5] = delta_x[5] + nom_x[5];
   out_2597874447553363541[6] = delta_x[6] + nom_x[6];
   out_2597874447553363541[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8182813298570237867) {
   out_8182813298570237867[0] = -nom_x[0] + true_x[0];
   out_8182813298570237867[1] = -nom_x[1] + true_x[1];
   out_8182813298570237867[2] = -nom_x[2] + true_x[2];
   out_8182813298570237867[3] = -nom_x[3] + true_x[3];
   out_8182813298570237867[4] = -nom_x[4] + true_x[4];
   out_8182813298570237867[5] = -nom_x[5] + true_x[5];
   out_8182813298570237867[6] = -nom_x[6] + true_x[6];
   out_8182813298570237867[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_6306941947207966650) {
   out_6306941947207966650[0] = 1.0;
   out_6306941947207966650[1] = 0.0;
   out_6306941947207966650[2] = 0.0;
   out_6306941947207966650[3] = 0.0;
   out_6306941947207966650[4] = 0.0;
   out_6306941947207966650[5] = 0.0;
   out_6306941947207966650[6] = 0.0;
   out_6306941947207966650[7] = 0.0;
   out_6306941947207966650[8] = 0.0;
   out_6306941947207966650[9] = 1.0;
   out_6306941947207966650[10] = 0.0;
   out_6306941947207966650[11] = 0.0;
   out_6306941947207966650[12] = 0.0;
   out_6306941947207966650[13] = 0.0;
   out_6306941947207966650[14] = 0.0;
   out_6306941947207966650[15] = 0.0;
   out_6306941947207966650[16] = 0.0;
   out_6306941947207966650[17] = 0.0;
   out_6306941947207966650[18] = 1.0;
   out_6306941947207966650[19] = 0.0;
   out_6306941947207966650[20] = 0.0;
   out_6306941947207966650[21] = 0.0;
   out_6306941947207966650[22] = 0.0;
   out_6306941947207966650[23] = 0.0;
   out_6306941947207966650[24] = 0.0;
   out_6306941947207966650[25] = 0.0;
   out_6306941947207966650[26] = 0.0;
   out_6306941947207966650[27] = 1.0;
   out_6306941947207966650[28] = 0.0;
   out_6306941947207966650[29] = 0.0;
   out_6306941947207966650[30] = 0.0;
   out_6306941947207966650[31] = 0.0;
   out_6306941947207966650[32] = 0.0;
   out_6306941947207966650[33] = 0.0;
   out_6306941947207966650[34] = 0.0;
   out_6306941947207966650[35] = 0.0;
   out_6306941947207966650[36] = 1.0;
   out_6306941947207966650[37] = 0.0;
   out_6306941947207966650[38] = 0.0;
   out_6306941947207966650[39] = 0.0;
   out_6306941947207966650[40] = 0.0;
   out_6306941947207966650[41] = 0.0;
   out_6306941947207966650[42] = 0.0;
   out_6306941947207966650[43] = 0.0;
   out_6306941947207966650[44] = 0.0;
   out_6306941947207966650[45] = 1.0;
   out_6306941947207966650[46] = 0.0;
   out_6306941947207966650[47] = 0.0;
   out_6306941947207966650[48] = 0.0;
   out_6306941947207966650[49] = 0.0;
   out_6306941947207966650[50] = 0.0;
   out_6306941947207966650[51] = 0.0;
   out_6306941947207966650[52] = 0.0;
   out_6306941947207966650[53] = 0.0;
   out_6306941947207966650[54] = 1.0;
   out_6306941947207966650[55] = 0.0;
   out_6306941947207966650[56] = 0.0;
   out_6306941947207966650[57] = 0.0;
   out_6306941947207966650[58] = 0.0;
   out_6306941947207966650[59] = 0.0;
   out_6306941947207966650[60] = 0.0;
   out_6306941947207966650[61] = 0.0;
   out_6306941947207966650[62] = 0.0;
   out_6306941947207966650[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_2402943826538611257) {
   out_2402943826538611257[0] = state[0];
   out_2402943826538611257[1] = state[1];
   out_2402943826538611257[2] = state[2];
   out_2402943826538611257[3] = state[3];
   out_2402943826538611257[4] = state[4];
   out_2402943826538611257[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2402943826538611257[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2402943826538611257[7] = state[7];
}
void F_fun(double *state, double dt, double *out_6186946395878559235) {
   out_6186946395878559235[0] = 1;
   out_6186946395878559235[1] = 0;
   out_6186946395878559235[2] = 0;
   out_6186946395878559235[3] = 0;
   out_6186946395878559235[4] = 0;
   out_6186946395878559235[5] = 0;
   out_6186946395878559235[6] = 0;
   out_6186946395878559235[7] = 0;
   out_6186946395878559235[8] = 0;
   out_6186946395878559235[9] = 1;
   out_6186946395878559235[10] = 0;
   out_6186946395878559235[11] = 0;
   out_6186946395878559235[12] = 0;
   out_6186946395878559235[13] = 0;
   out_6186946395878559235[14] = 0;
   out_6186946395878559235[15] = 0;
   out_6186946395878559235[16] = 0;
   out_6186946395878559235[17] = 0;
   out_6186946395878559235[18] = 1;
   out_6186946395878559235[19] = 0;
   out_6186946395878559235[20] = 0;
   out_6186946395878559235[21] = 0;
   out_6186946395878559235[22] = 0;
   out_6186946395878559235[23] = 0;
   out_6186946395878559235[24] = 0;
   out_6186946395878559235[25] = 0;
   out_6186946395878559235[26] = 0;
   out_6186946395878559235[27] = 1;
   out_6186946395878559235[28] = 0;
   out_6186946395878559235[29] = 0;
   out_6186946395878559235[30] = 0;
   out_6186946395878559235[31] = 0;
   out_6186946395878559235[32] = 0;
   out_6186946395878559235[33] = 0;
   out_6186946395878559235[34] = 0;
   out_6186946395878559235[35] = 0;
   out_6186946395878559235[36] = 1;
   out_6186946395878559235[37] = 0;
   out_6186946395878559235[38] = 0;
   out_6186946395878559235[39] = 0;
   out_6186946395878559235[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6186946395878559235[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6186946395878559235[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6186946395878559235[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6186946395878559235[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6186946395878559235[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6186946395878559235[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6186946395878559235[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6186946395878559235[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6186946395878559235[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6186946395878559235[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6186946395878559235[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6186946395878559235[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6186946395878559235[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6186946395878559235[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6186946395878559235[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6186946395878559235[56] = 0;
   out_6186946395878559235[57] = 0;
   out_6186946395878559235[58] = 0;
   out_6186946395878559235[59] = 0;
   out_6186946395878559235[60] = 0;
   out_6186946395878559235[61] = 0;
   out_6186946395878559235[62] = 0;
   out_6186946395878559235[63] = 1;
}
void h_25(double *state, double *unused, double *out_7844523176759450585) {
   out_7844523176759450585[0] = state[6];
}
void H_25(double *state, double *unused, double *out_388105742977512244) {
   out_388105742977512244[0] = 0;
   out_388105742977512244[1] = 0;
   out_388105742977512244[2] = 0;
   out_388105742977512244[3] = 0;
   out_388105742977512244[4] = 0;
   out_388105742977512244[5] = 0;
   out_388105742977512244[6] = 1;
   out_388105742977512244[7] = 0;
}
void h_24(double *state, double *unused, double *out_3716241642962455388) {
   out_3716241642962455388[0] = state[4];
   out_3716241642962455388[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5889753943026838658) {
   out_5889753943026838658[0] = 0;
   out_5889753943026838658[1] = 0;
   out_5889753943026838658[2] = 0;
   out_5889753943026838658[3] = 0;
   out_5889753943026838658[4] = 1;
   out_5889753943026838658[5] = 0;
   out_5889753943026838658[6] = 0;
   out_5889753943026838658[7] = 0;
   out_5889753943026838658[8] = 0;
   out_5889753943026838658[9] = 0;
   out_5889753943026838658[10] = 0;
   out_5889753943026838658[11] = 0;
   out_5889753943026838658[12] = 0;
   out_5889753943026838658[13] = 1;
   out_5889753943026838658[14] = 0;
   out_5889753943026838658[15] = 0;
}
void h_30(double *state, double *unused, double *out_8760773842464067664) {
   out_8760773842464067664[0] = state[4];
}
void H_30(double *state, double *unused, double *out_9220950905737880580) {
   out_9220950905737880580[0] = 0;
   out_9220950905737880580[1] = 0;
   out_9220950905737880580[2] = 0;
   out_9220950905737880580[3] = 0;
   out_9220950905737880580[4] = 1;
   out_9220950905737880580[5] = 0;
   out_9220950905737880580[6] = 0;
   out_9220950905737880580[7] = 0;
}
void h_26(double *state, double *unused, double *out_3118043971383110829) {
   out_3118043971383110829[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6092698897916721517) {
   out_6092698897916721517[0] = 0;
   out_6092698897916721517[1] = 0;
   out_6092698897916721517[2] = 0;
   out_6092698897916721517[3] = 0;
   out_6092698897916721517[4] = 0;
   out_6092698897916721517[5] = 0;
   out_6092698897916721517[6] = 0;
   out_6092698897916721517[7] = 1;
}
void h_27(double *state, double *unused, double *out_2086538262765712629) {
   out_2086538262765712629[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7933368917901255268) {
   out_7933368917901255268[0] = 0;
   out_7933368917901255268[1] = 0;
   out_7933368917901255268[2] = 0;
   out_7933368917901255268[3] = 1;
   out_7933368917901255268[4] = 0;
   out_7933368917901255268[5] = 0;
   out_7933368917901255268[6] = 0;
   out_7933368917901255268[7] = 0;
}
void h_29(double *state, double *unused, double *out_684573804615255661) {
   out_684573804615255661[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8651234554116134861) {
   out_8651234554116134861[0] = 0;
   out_8651234554116134861[1] = 1;
   out_8651234554116134861[2] = 0;
   out_8651234554116134861[3] = 0;
   out_8651234554116134861[4] = 0;
   out_8651234554116134861[5] = 0;
   out_8651234554116134861[6] = 0;
   out_8651234554116134861[7] = 0;
}
void h_28(double *state, double *unused, double *out_3486029231902958175) {
   out_3486029231902958175[0] = state[5];
   out_3486029231902958175[1] = state[6];
}
void H_28(double *state, double *unused, double *out_1128078535209656208) {
   out_1128078535209656208[0] = 0;
   out_1128078535209656208[1] = 0;
   out_1128078535209656208[2] = 0;
   out_1128078535209656208[3] = 0;
   out_1128078535209656208[4] = 0;
   out_1128078535209656208[5] = 1;
   out_1128078535209656208[6] = 0;
   out_1128078535209656208[7] = 0;
   out_1128078535209656208[8] = 0;
   out_1128078535209656208[9] = 0;
   out_1128078535209656208[10] = 0;
   out_1128078535209656208[11] = 0;
   out_1128078535209656208[12] = 0;
   out_1128078535209656208[13] = 0;
   out_1128078535209656208[14] = 1;
   out_1128078535209656208[15] = 0;
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
  update<2, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2597874447553363541) {
  err_fun(nom_x, delta_x, out_2597874447553363541);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8182813298570237867) {
  inv_err_fun(nom_x, true_x, out_8182813298570237867);
}
void car_H_mod_fun(double *state, double *out_6306941947207966650) {
  H_mod_fun(state, out_6306941947207966650);
}
void car_f_fun(double *state, double dt, double *out_2402943826538611257) {
  f_fun(state,  dt, out_2402943826538611257);
}
void car_F_fun(double *state, double dt, double *out_6186946395878559235) {
  F_fun(state,  dt, out_6186946395878559235);
}
void car_h_25(double *state, double *unused, double *out_7844523176759450585) {
  h_25(state, unused, out_7844523176759450585);
}
void car_H_25(double *state, double *unused, double *out_388105742977512244) {
  H_25(state, unused, out_388105742977512244);
}
void car_h_24(double *state, double *unused, double *out_3716241642962455388) {
  h_24(state, unused, out_3716241642962455388);
}
void car_H_24(double *state, double *unused, double *out_5889753943026838658) {
  H_24(state, unused, out_5889753943026838658);
}
void car_h_30(double *state, double *unused, double *out_8760773842464067664) {
  h_30(state, unused, out_8760773842464067664);
}
void car_H_30(double *state, double *unused, double *out_9220950905737880580) {
  H_30(state, unused, out_9220950905737880580);
}
void car_h_26(double *state, double *unused, double *out_3118043971383110829) {
  h_26(state, unused, out_3118043971383110829);
}
void car_H_26(double *state, double *unused, double *out_6092698897916721517) {
  H_26(state, unused, out_6092698897916721517);
}
void car_h_27(double *state, double *unused, double *out_2086538262765712629) {
  h_27(state, unused, out_2086538262765712629);
}
void car_H_27(double *state, double *unused, double *out_7933368917901255268) {
  H_27(state, unused, out_7933368917901255268);
}
void car_h_29(double *state, double *unused, double *out_684573804615255661) {
  h_29(state, unused, out_684573804615255661);
}
void car_H_29(double *state, double *unused, double *out_8651234554116134861) {
  H_29(state, unused, out_8651234554116134861);
}
void car_h_28(double *state, double *unused, double *out_3486029231902958175) {
  h_28(state, unused, out_3486029231902958175);
}
void car_H_28(double *state, double *unused, double *out_1128078535209656208) {
  H_28(state, unused, out_1128078535209656208);
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
  .kinds = { 25, 24, 30, 26, 27, 29, 28 },
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
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
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

ekf_init(car);
