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
void err_fun(double *nom_x, double *delta_x, double *out_4764429363741635774) {
   out_4764429363741635774[0] = delta_x[0] + nom_x[0];
   out_4764429363741635774[1] = delta_x[1] + nom_x[1];
   out_4764429363741635774[2] = delta_x[2] + nom_x[2];
   out_4764429363741635774[3] = delta_x[3] + nom_x[3];
   out_4764429363741635774[4] = delta_x[4] + nom_x[4];
   out_4764429363741635774[5] = delta_x[5] + nom_x[5];
   out_4764429363741635774[6] = delta_x[6] + nom_x[6];
   out_4764429363741635774[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4923486496625013297) {
   out_4923486496625013297[0] = -nom_x[0] + true_x[0];
   out_4923486496625013297[1] = -nom_x[1] + true_x[1];
   out_4923486496625013297[2] = -nom_x[2] + true_x[2];
   out_4923486496625013297[3] = -nom_x[3] + true_x[3];
   out_4923486496625013297[4] = -nom_x[4] + true_x[4];
   out_4923486496625013297[5] = -nom_x[5] + true_x[5];
   out_4923486496625013297[6] = -nom_x[6] + true_x[6];
   out_4923486496625013297[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_8284844191623208552) {
   out_8284844191623208552[0] = 1.0;
   out_8284844191623208552[1] = 0.0;
   out_8284844191623208552[2] = 0.0;
   out_8284844191623208552[3] = 0.0;
   out_8284844191623208552[4] = 0.0;
   out_8284844191623208552[5] = 0.0;
   out_8284844191623208552[6] = 0.0;
   out_8284844191623208552[7] = 0.0;
   out_8284844191623208552[8] = 0.0;
   out_8284844191623208552[9] = 1.0;
   out_8284844191623208552[10] = 0.0;
   out_8284844191623208552[11] = 0.0;
   out_8284844191623208552[12] = 0.0;
   out_8284844191623208552[13] = 0.0;
   out_8284844191623208552[14] = 0.0;
   out_8284844191623208552[15] = 0.0;
   out_8284844191623208552[16] = 0.0;
   out_8284844191623208552[17] = 0.0;
   out_8284844191623208552[18] = 1.0;
   out_8284844191623208552[19] = 0.0;
   out_8284844191623208552[20] = 0.0;
   out_8284844191623208552[21] = 0.0;
   out_8284844191623208552[22] = 0.0;
   out_8284844191623208552[23] = 0.0;
   out_8284844191623208552[24] = 0.0;
   out_8284844191623208552[25] = 0.0;
   out_8284844191623208552[26] = 0.0;
   out_8284844191623208552[27] = 1.0;
   out_8284844191623208552[28] = 0.0;
   out_8284844191623208552[29] = 0.0;
   out_8284844191623208552[30] = 0.0;
   out_8284844191623208552[31] = 0.0;
   out_8284844191623208552[32] = 0.0;
   out_8284844191623208552[33] = 0.0;
   out_8284844191623208552[34] = 0.0;
   out_8284844191623208552[35] = 0.0;
   out_8284844191623208552[36] = 1.0;
   out_8284844191623208552[37] = 0.0;
   out_8284844191623208552[38] = 0.0;
   out_8284844191623208552[39] = 0.0;
   out_8284844191623208552[40] = 0.0;
   out_8284844191623208552[41] = 0.0;
   out_8284844191623208552[42] = 0.0;
   out_8284844191623208552[43] = 0.0;
   out_8284844191623208552[44] = 0.0;
   out_8284844191623208552[45] = 1.0;
   out_8284844191623208552[46] = 0.0;
   out_8284844191623208552[47] = 0.0;
   out_8284844191623208552[48] = 0.0;
   out_8284844191623208552[49] = 0.0;
   out_8284844191623208552[50] = 0.0;
   out_8284844191623208552[51] = 0.0;
   out_8284844191623208552[52] = 0.0;
   out_8284844191623208552[53] = 0.0;
   out_8284844191623208552[54] = 1.0;
   out_8284844191623208552[55] = 0.0;
   out_8284844191623208552[56] = 0.0;
   out_8284844191623208552[57] = 0.0;
   out_8284844191623208552[58] = 0.0;
   out_8284844191623208552[59] = 0.0;
   out_8284844191623208552[60] = 0.0;
   out_8284844191623208552[61] = 0.0;
   out_8284844191623208552[62] = 0.0;
   out_8284844191623208552[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_5426936756111410655) {
   out_5426936756111410655[0] = state[0];
   out_5426936756111410655[1] = state[1];
   out_5426936756111410655[2] = state[2];
   out_5426936756111410655[3] = state[3];
   out_5426936756111410655[4] = state[4];
   out_5426936756111410655[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5426936756111410655[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5426936756111410655[7] = state[7];
}
void F_fun(double *state, double dt, double *out_1189665854613176299) {
   out_1189665854613176299[0] = 1;
   out_1189665854613176299[1] = 0;
   out_1189665854613176299[2] = 0;
   out_1189665854613176299[3] = 0;
   out_1189665854613176299[4] = 0;
   out_1189665854613176299[5] = 0;
   out_1189665854613176299[6] = 0;
   out_1189665854613176299[7] = 0;
   out_1189665854613176299[8] = 0;
   out_1189665854613176299[9] = 1;
   out_1189665854613176299[10] = 0;
   out_1189665854613176299[11] = 0;
   out_1189665854613176299[12] = 0;
   out_1189665854613176299[13] = 0;
   out_1189665854613176299[14] = 0;
   out_1189665854613176299[15] = 0;
   out_1189665854613176299[16] = 0;
   out_1189665854613176299[17] = 0;
   out_1189665854613176299[18] = 1;
   out_1189665854613176299[19] = 0;
   out_1189665854613176299[20] = 0;
   out_1189665854613176299[21] = 0;
   out_1189665854613176299[22] = 0;
   out_1189665854613176299[23] = 0;
   out_1189665854613176299[24] = 0;
   out_1189665854613176299[25] = 0;
   out_1189665854613176299[26] = 0;
   out_1189665854613176299[27] = 1;
   out_1189665854613176299[28] = 0;
   out_1189665854613176299[29] = 0;
   out_1189665854613176299[30] = 0;
   out_1189665854613176299[31] = 0;
   out_1189665854613176299[32] = 0;
   out_1189665854613176299[33] = 0;
   out_1189665854613176299[34] = 0;
   out_1189665854613176299[35] = 0;
   out_1189665854613176299[36] = 1;
   out_1189665854613176299[37] = 0;
   out_1189665854613176299[38] = 0;
   out_1189665854613176299[39] = 0;
   out_1189665854613176299[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1189665854613176299[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1189665854613176299[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1189665854613176299[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1189665854613176299[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1189665854613176299[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1189665854613176299[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1189665854613176299[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1189665854613176299[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1189665854613176299[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1189665854613176299[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1189665854613176299[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1189665854613176299[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1189665854613176299[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1189665854613176299[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1189665854613176299[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1189665854613176299[56] = 0;
   out_1189665854613176299[57] = 0;
   out_1189665854613176299[58] = 0;
   out_1189665854613176299[59] = 0;
   out_1189665854613176299[60] = 0;
   out_1189665854613176299[61] = 0;
   out_1189665854613176299[62] = 0;
   out_1189665854613176299[63] = 1;
}
void h_25(double *state, double *unused, double *out_2384565946978239929) {
   out_2384565946978239929[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6913636704135587423) {
   out_6913636704135587423[0] = 0;
   out_6913636704135587423[1] = 0;
   out_6913636704135587423[2] = 0;
   out_6913636704135587423[3] = 0;
   out_6913636704135587423[4] = 0;
   out_6913636704135587423[5] = 0;
   out_6913636704135587423[6] = 1;
   out_6913636704135587423[7] = 0;
}
void h_24(double *state, double *unused, double *out_5914851978677013851) {
   out_5914851978677013851[0] = state[4];
   out_5914851978677013851[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2445450570523504853) {
   out_2445450570523504853[0] = 0;
   out_2445450570523504853[1] = 0;
   out_2445450570523504853[2] = 0;
   out_2445450570523504853[3] = 0;
   out_2445450570523504853[4] = 1;
   out_2445450570523504853[5] = 0;
   out_2445450570523504853[6] = 0;
   out_2445450570523504853[7] = 0;
   out_2445450570523504853[8] = 0;
   out_2445450570523504853[9] = 0;
   out_2445450570523504853[10] = 0;
   out_2445450570523504853[11] = 0;
   out_2445450570523504853[12] = 0;
   out_2445450570523504853[13] = 1;
   out_2445450570523504853[14] = 0;
   out_2445450570523504853[15] = 0;
}
void h_30(double *state, double *unused, double *out_4386269279372111007) {
   out_4386269279372111007[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2700262206813595857) {
   out_2700262206813595857[0] = 0;
   out_2700262206813595857[1] = 0;
   out_2700262206813595857[2] = 0;
   out_2700262206813595857[3] = 0;
   out_2700262206813595857[4] = 1;
   out_2700262206813595857[5] = 0;
   out_2700262206813595857[6] = 0;
   out_2700262206813595857[7] = 0;
}
void h_26(double *state, double *unused, double *out_3218525478295969270) {
   out_3218525478295969270[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8219872476090428568) {
   out_8219872476090428568[0] = 0;
   out_8219872476090428568[1] = 0;
   out_8219872476090428568[2] = 0;
   out_8219872476090428568[3] = 0;
   out_8219872476090428568[4] = 0;
   out_8219872476090428568[5] = 0;
   out_8219872476090428568[6] = 0;
   out_8219872476090428568[7] = 1;
}
void h_27(double *state, double *unused, double *out_7006751976434348243) {
   out_7006751976434348243[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3987844194650221169) {
   out_3987844194650221169[0] = 0;
   out_3987844194650221169[1] = 0;
   out_3987844194650221169[2] = 0;
   out_3987844194650221169[3] = 1;
   out_3987844194650221169[4] = 0;
   out_3987844194650221169[5] = 0;
   out_3987844194650221169[6] = 0;
   out_3987844194650221169[7] = 0;
}
void h_29(double *state, double *unused, double *out_7198821382641964390) {
   out_7198821382641964390[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6524060975942427810) {
   out_6524060975942427810[0] = 0;
   out_6524060975942427810[1] = 1;
   out_6524060975942427810[2] = 0;
   out_6524060975942427810[3] = 0;
   out_6524060975942427810[4] = 0;
   out_6524060975942427810[5] = 0;
   out_6524060975942427810[6] = 0;
   out_6524060975942427810[7] = 0;
}
void h_28(double *state, double *unused, double *out_3143930863498534129) {
   out_3143930863498534129[0] = state[5];
   out_3143930863498534129[1] = state[6];
}
void H_28(double *state, double *unused, double *out_8983461024949551897) {
   out_8983461024949551897[0] = 0;
   out_8983461024949551897[1] = 0;
   out_8983461024949551897[2] = 0;
   out_8983461024949551897[3] = 0;
   out_8983461024949551897[4] = 0;
   out_8983461024949551897[5] = 1;
   out_8983461024949551897[6] = 0;
   out_8983461024949551897[7] = 0;
   out_8983461024949551897[8] = 0;
   out_8983461024949551897[9] = 0;
   out_8983461024949551897[10] = 0;
   out_8983461024949551897[11] = 0;
   out_8983461024949551897[12] = 0;
   out_8983461024949551897[13] = 0;
   out_8983461024949551897[14] = 1;
   out_8983461024949551897[15] = 0;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4764429363741635774) {
  err_fun(nom_x, delta_x, out_4764429363741635774);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4923486496625013297) {
  inv_err_fun(nom_x, true_x, out_4923486496625013297);
}
void car_H_mod_fun(double *state, double *out_8284844191623208552) {
  H_mod_fun(state, out_8284844191623208552);
}
void car_f_fun(double *state, double dt, double *out_5426936756111410655) {
  f_fun(state,  dt, out_5426936756111410655);
}
void car_F_fun(double *state, double dt, double *out_1189665854613176299) {
  F_fun(state,  dt, out_1189665854613176299);
}
void car_h_25(double *state, double *unused, double *out_2384565946978239929) {
  h_25(state, unused, out_2384565946978239929);
}
void car_H_25(double *state, double *unused, double *out_6913636704135587423) {
  H_25(state, unused, out_6913636704135587423);
}
void car_h_24(double *state, double *unused, double *out_5914851978677013851) {
  h_24(state, unused, out_5914851978677013851);
}
void car_H_24(double *state, double *unused, double *out_2445450570523504853) {
  H_24(state, unused, out_2445450570523504853);
}
void car_h_30(double *state, double *unused, double *out_4386269279372111007) {
  h_30(state, unused, out_4386269279372111007);
}
void car_H_30(double *state, double *unused, double *out_2700262206813595857) {
  H_30(state, unused, out_2700262206813595857);
}
void car_h_26(double *state, double *unused, double *out_3218525478295969270) {
  h_26(state, unused, out_3218525478295969270);
}
void car_H_26(double *state, double *unused, double *out_8219872476090428568) {
  H_26(state, unused, out_8219872476090428568);
}
void car_h_27(double *state, double *unused, double *out_7006751976434348243) {
  h_27(state, unused, out_7006751976434348243);
}
void car_H_27(double *state, double *unused, double *out_3987844194650221169) {
  H_27(state, unused, out_3987844194650221169);
}
void car_h_29(double *state, double *unused, double *out_7198821382641964390) {
  h_29(state, unused, out_7198821382641964390);
}
void car_H_29(double *state, double *unused, double *out_6524060975942427810) {
  H_29(state, unused, out_6524060975942427810);
}
void car_h_28(double *state, double *unused, double *out_3143930863498534129) {
  h_28(state, unused, out_3143930863498534129);
}
void car_H_28(double *state, double *unused, double *out_8983461024949551897) {
  H_28(state, unused, out_8983461024949551897);
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
