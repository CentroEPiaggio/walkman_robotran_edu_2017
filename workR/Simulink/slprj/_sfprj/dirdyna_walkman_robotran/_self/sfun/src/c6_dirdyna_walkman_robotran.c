/* Include files */

#include <stddef.h>
#include "blas.h"
#include "dirdyna_walkman_robotran_sfun.h"
#include "c6_dirdyna_walkman_robotran.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "dirdyna_walkman_robotran_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c6_debug_family_names[14] = { "J_pinv", "p_d_dot", "KP",
  "e_p", "u_p", "u_o", "u", "nargin", "nargout", "J", "ref_dot", "K", "e",
  "q_dot" };

/* Function Declarations */
static void initialize_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void initialize_params_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void enable_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void disable_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void c6_update_debugger_state_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void set_sim_state_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance, const mxArray
   *c6_st);
static void finalize_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void sf_gateway_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void c6_chartstep_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void initSimStructsc6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static void c6_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_q_dot, const char_T *c6_identifier, real_T
  c6_y[31]);
static void c6_b_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[31]);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_c_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct *
  chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_d_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[6]);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_e_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3]);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_f_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[9]);
static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_g_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[186]);
static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_info_helper(const mxArray **c6_info);
static const mxArray *c6_emlrt_marshallOut(const char * c6_u);
static const mxArray *c6_b_emlrt_marshallOut(const uint32_T c6_u);
static void c6_b_info_helper(const mxArray **c6_info);
static void c6_c_info_helper(const mxArray **c6_info);
static void c6_d_info_helper(const mxArray **c6_info);
static void c6_pinv(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                    real_T c6_A[186], real_T c6_X[186]);
static void c6_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_svd(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                   real_T c6_A[186], real_T c6_U[186], real_T c6_S[36], real_T
                   c6_V[36]);
static void c6_eml_switch_helper(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_eml_error(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_eml_xgesvd(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_A[186], real_T c6_U[186], real_T c6_S[6], real_T
  c6_V[36]);
static real_T c6_eml_xnrm2(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_x[186], int32_T c6_ix0);
static void c6_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static real_T c6_abs(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                     real_T c6_x);
static void c6_realmin(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_check_forloop_overflow_error
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance, boolean_T
   c6_overflow);
static real_T c6_eml_div(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x, real_T c6_y);
static void c6_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[186], int32_T c6_ix0,
  real_T c6_b_x[186]);
static void c6_b_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static real_T c6_eml_xdotc(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_x[186], int32_T c6_ix0, real_T c6_y
  [186], int32_T c6_iy0);
static void c6_c_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, int32_T c6_ix0, real_T c6_y[186],
  int32_T c6_iy0, real_T c6_b_y[186]);
static void c6_d_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static real_T c6_b_eml_xnrm2(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_x[6], int32_T c6_ix0);
static void c6_b_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[6], int32_T c6_ix0,
  real_T c6_b_x[6]);
static void c6_b_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[186], int32_T c6_ix0,
  real_T c6_y[31], int32_T c6_iy0, real_T c6_b_y[31]);
static void c6_c_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[31], int32_T c6_ix0,
  real_T c6_y[186], int32_T c6_iy0, real_T c6_b_y[186]);
static real_T c6_b_eml_xdotc(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_x[36], int32_T c6_ix0, real_T c6_y[36],
  int32_T c6_iy0);
static void c6_d_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, int32_T c6_ix0, real_T c6_y[36],
  int32_T c6_iy0, real_T c6_b_y[36]);
static void c6_c_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_x[186], int32_T c6_ix0, real_T c6_b_x
  [186]);
static void c6_d_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_x[36], int32_T c6_ix0, real_T c6_b_x[36]);
static void c6_eps(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance);
static void c6_b_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_b_eml_error(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static real_T c6_sqrt(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                      real_T c6_x);
static void c6_c_eml_error(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_eml_xrotg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_b, real_T *c6_b_a, real_T *c6_b_b,
  real_T *c6_c, real_T *c6_s);
static void c6_eml_xrot(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[36], int32_T c6_ix0, int32_T c6_iy0, real_T c6_c,
  real_T c6_s, real_T c6_b_x[36]);
static void c6_e_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_b_eml_xrot(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[186], int32_T c6_ix0, int32_T c6_iy0, real_T c6_c,
  real_T c6_s, real_T c6_b_x[186]);
static void c6_c_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_eml_xswap(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[36], int32_T c6_ix0, int32_T c6_iy0, real_T
  c6_b_x[36]);
static void c6_f_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_b_eml_xswap(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[186], int32_T c6_ix0, int32_T c6_iy0, real_T
  c6_b_x[186]);
static void c6_eml_xgemm(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_k, real_T c6_A[36], real_T c6_B[186], real_T c6_C
  [186], real_T c6_b_C[186]);
static void c6_below_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_g_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_d_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static void c6_e_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);
static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_h_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_i_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_dirdyna_walkman_robotran,
  const char_T *c6_identifier);
static uint8_T c6_j_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_e_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[186], int32_T c6_ix0);
static void c6_e_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, int32_T c6_ix0, real_T c6_y[186],
  int32_T c6_iy0);
static void c6_f_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[6], int32_T c6_ix0);
static void c6_f_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[186], int32_T c6_ix0,
  real_T c6_y[31], int32_T c6_iy0);
static void c6_g_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[31], int32_T c6_ix0,
  real_T c6_y[186], int32_T c6_iy0);
static void c6_h_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, int32_T c6_ix0, real_T c6_y[36],
  int32_T c6_iy0);
static void c6_g_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_x[186], int32_T c6_ix0);
static void c6_h_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_x[36], int32_T c6_ix0);
static void c6_b_sqrt(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                      real_T *c6_x);
static void c6_b_eml_xrotg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T *c6_a, real_T *c6_b, real_T *c6_c, real_T *c6_s);
static void c6_c_eml_xrot(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[36], int32_T c6_ix0, int32_T c6_iy0, real_T c6_c,
  real_T c6_s);
static void c6_d_eml_xrot(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[186], int32_T c6_ix0, int32_T c6_iy0, real_T c6_c,
  real_T c6_s);
static void c6_c_eml_xswap(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[36], int32_T c6_ix0, int32_T c6_iy0);
static void c6_d_eml_xswap(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[186], int32_T c6_ix0, int32_T c6_iy0);
static void c6_b_eml_xgemm(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_k, real_T c6_A[36], real_T c6_B[186], real_T c6_C
  [186]);
static void init_dsm_address_info(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c6_is_active_c6_dirdyna_walkman_robotran = 0U;
}

static void initialize_params_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c6_update_debugger_state_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  int32_T c6_i0;
  real_T c6_u[31];
  const mxArray *c6_b_y = NULL;
  uint8_T c6_hoistedGlobal;
  uint8_T c6_b_u;
  const mxArray *c6_c_y = NULL;
  real_T (*c6_q_dot)[31];
  c6_q_dot = (real_T (*)[31])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellmatrix(2, 1), false);
  for (c6_i0 = 0; c6_i0 < 31; c6_i0++) {
    c6_u[c6_i0] = (*c6_q_dot)[c6_i0];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 31), false);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  c6_hoistedGlobal = chartInstance->c6_is_active_c6_dirdyna_walkman_robotran;
  c6_b_u = c6_hoistedGlobal;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c6_y, 1, c6_c_y);
  sf_mex_assign(&c6_st, c6_y, false);
  return c6_st;
}

static void set_sim_state_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance, const mxArray
   *c6_st)
{
  const mxArray *c6_u;
  real_T c6_dv0[31];
  int32_T c6_i1;
  real_T (*c6_q_dot)[31];
  c6_q_dot = (real_T (*)[31])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c6_doneDoubleBufferReInit = true;
  c6_u = sf_mex_dup(c6_st);
  c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 0)),
                      "q_dot", c6_dv0);
  for (c6_i1 = 0; c6_i1 < 31; c6_i1++) {
    (*c6_q_dot)[c6_i1] = c6_dv0[c6_i1];
  }

  chartInstance->c6_is_active_c6_dirdyna_walkman_robotran =
    c6_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 1)),
    "is_active_c6_dirdyna_walkman_robotran");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_dirdyna_walkman_robotran(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  int32_T c6_i2;
  int32_T c6_i3;
  int32_T c6_i4;
  int32_T c6_i5;
  int32_T c6_i6;
  real_T (*c6_e)[3];
  real_T (*c6_K)[9];
  real_T (*c6_ref_dot)[3];
  real_T (*c6_q_dot)[31];
  real_T (*c6_J)[186];
  c6_e = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c6_K = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 2);
  c6_ref_dot = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c6_q_dot = (real_T (*)[31])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_J = (real_T (*)[186])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c6_sfEvent);
  for (c6_i2 = 0; c6_i2 < 186; c6_i2++) {
    _SFD_DATA_RANGE_CHECK((*c6_J)[c6_i2], 0U);
  }

  chartInstance->c6_sfEvent = CALL_EVENT;
  c6_chartstep_c6_dirdyna_walkman_robotran(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_dirdyna_walkman_robotranMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c6_i3 = 0; c6_i3 < 31; c6_i3++) {
    _SFD_DATA_RANGE_CHECK((*c6_q_dot)[c6_i3], 1U);
  }

  for (c6_i4 = 0; c6_i4 < 3; c6_i4++) {
    _SFD_DATA_RANGE_CHECK((*c6_ref_dot)[c6_i4], 2U);
  }

  for (c6_i5 = 0; c6_i5 < 9; c6_i5++) {
    _SFD_DATA_RANGE_CHECK((*c6_K)[c6_i5], 3U);
  }

  for (c6_i6 = 0; c6_i6 < 3; c6_i6++) {
    _SFD_DATA_RANGE_CHECK((*c6_e)[c6_i6], 4U);
  }
}

static void c6_chartstep_c6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  int32_T c6_i7;
  real_T c6_J[186];
  int32_T c6_i8;
  real_T c6_ref_dot[3];
  int32_T c6_i9;
  real_T c6_K[9];
  int32_T c6_i10;
  real_T c6_e[3];
  uint32_T c6_debug_family_var_map[14];
  real_T c6_J_pinv[186];
  real_T c6_p_d_dot[3];
  real_T c6_KP[9];
  real_T c6_e_p[3];
  real_T c6_u_p[3];
  real_T c6_u_o[3];
  real_T c6_u[6];
  real_T c6_nargin = 4.0;
  real_T c6_nargout = 1.0;
  real_T c6_q_dot[31];
  int32_T c6_i11;
  int32_T c6_i12;
  int32_T c6_i13;
  int32_T c6_i14;
  real_T c6_b_J[186];
  real_T c6_dv1[186];
  int32_T c6_i15;
  int32_T c6_i16;
  int32_T c6_i17;
  int32_T c6_i18;
  int32_T c6_i19;
  real_T c6_a[9];
  int32_T c6_i20;
  real_T c6_b[3];
  int32_T c6_i21;
  real_T c6_y[3];
  int32_T c6_i22;
  int32_T c6_i23;
  int32_T c6_i24;
  int32_T c6_i25;
  int32_T c6_i26;
  int32_T c6_i27;
  int32_T c6_i28;
  real_T c6_b_a[186];
  int32_T c6_i29;
  real_T c6_b_b[6];
  int32_T c6_i30;
  int32_T c6_i31;
  int32_T c6_i32;
  real_T c6_C[31];
  int32_T c6_i33;
  int32_T c6_i34;
  int32_T c6_i35;
  int32_T c6_i36;
  int32_T c6_i37;
  int32_T c6_i38;
  int32_T c6_i39;
  real_T (*c6_b_q_dot)[31];
  real_T (*c6_b_e)[3];
  real_T (*c6_b_K)[9];
  real_T (*c6_b_ref_dot)[3];
  real_T (*c6_c_J)[186];
  c6_b_e = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c6_b_K = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 2);
  c6_b_ref_dot = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c6_b_q_dot = (real_T (*)[31])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_c_J = (real_T (*)[186])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c6_sfEvent);
  for (c6_i7 = 0; c6_i7 < 186; c6_i7++) {
    c6_J[c6_i7] = (*c6_c_J)[c6_i7];
  }

  for (c6_i8 = 0; c6_i8 < 3; c6_i8++) {
    c6_ref_dot[c6_i8] = (*c6_b_ref_dot)[c6_i8];
  }

  for (c6_i9 = 0; c6_i9 < 9; c6_i9++) {
    c6_K[c6_i9] = (*c6_b_K)[c6_i9];
  }

  for (c6_i10 = 0; c6_i10 < 3; c6_i10++) {
    c6_e[c6_i10] = (*c6_b_e)[c6_i10];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 14U, 14U, c6_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_J_pinv, 0U, c6_g_sf_marshallOut,
    c6_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_p_d_dot, 1U, c6_b_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_KP, 2U, c6_c_sf_marshallOut,
    c6_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_e_p, 3U, c6_b_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_u_p, 4U, c6_b_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_u_o, 5U, c6_b_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_u, 6U, c6_f_sf_marshallOut,
    c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 7U, c6_e_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 8U, c6_e_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_J, 9U, c6_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_ref_dot, 10U, c6_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_K, 11U, c6_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_e, 12U, c6_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_q_dot, 13U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 4);
  c6_i11 = 0;
  for (c6_i12 = 0; c6_i12 < 22; c6_i12++) {
    for (c6_i13 = 0; c6_i13 < 6; c6_i13++) {
      c6_J[c6_i13 + c6_i11] = 0.0;
    }

    c6_i11 += 6;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 5);
  for (c6_i14 = 0; c6_i14 < 186; c6_i14++) {
    c6_b_J[c6_i14] = c6_J[c6_i14];
  }

  c6_pinv(chartInstance, c6_b_J, c6_dv1);
  for (c6_i15 = 0; c6_i15 < 186; c6_i15++) {
    c6_J_pinv[c6_i15] = c6_dv1[c6_i15];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 7);
  for (c6_i16 = 0; c6_i16 < 3; c6_i16++) {
    c6_p_d_dot[c6_i16] = c6_ref_dot[c6_i16];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 8);
  for (c6_i17 = 0; c6_i17 < 9; c6_i17++) {
    c6_KP[c6_i17] = c6_K[c6_i17];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 9);
  for (c6_i18 = 0; c6_i18 < 3; c6_i18++) {
    c6_e_p[c6_i18] = c6_e[c6_i18];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 10);
  for (c6_i19 = 0; c6_i19 < 9; c6_i19++) {
    c6_a[c6_i19] = c6_KP[c6_i19];
  }

  for (c6_i20 = 0; c6_i20 < 3; c6_i20++) {
    c6_b[c6_i20] = c6_e_p[c6_i20];
  }

  c6_d_eml_scalar_eg(chartInstance);
  c6_d_eml_scalar_eg(chartInstance);
  c6_g_threshold(chartInstance);
  for (c6_i21 = 0; c6_i21 < 3; c6_i21++) {
    c6_y[c6_i21] = 0.0;
    c6_i22 = 0;
    for (c6_i23 = 0; c6_i23 < 3; c6_i23++) {
      c6_y[c6_i21] += c6_a[c6_i22 + c6_i21] * c6_b[c6_i23];
      c6_i22 += 3;
    }
  }

  for (c6_i24 = 0; c6_i24 < 3; c6_i24++) {
    c6_u_p[c6_i24] = c6_p_d_dot[c6_i24] + c6_y[c6_i24];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 11);
  for (c6_i25 = 0; c6_i25 < 3; c6_i25++) {
    c6_u_o[c6_i25] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 12);
  for (c6_i26 = 0; c6_i26 < 3; c6_i26++) {
    c6_u[c6_i26] = c6_u_p[c6_i26];
  }

  for (c6_i27 = 0; c6_i27 < 3; c6_i27++) {
    c6_u[c6_i27 + 3] = c6_u_o[c6_i27];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 13);
  for (c6_i28 = 0; c6_i28 < 186; c6_i28++) {
    c6_b_a[c6_i28] = c6_J_pinv[c6_i28];
  }

  for (c6_i29 = 0; c6_i29 < 6; c6_i29++) {
    c6_b_b[c6_i29] = c6_u[c6_i29];
  }

  c6_e_eml_scalar_eg(chartInstance);
  c6_e_eml_scalar_eg(chartInstance);
  for (c6_i30 = 0; c6_i30 < 31; c6_i30++) {
    c6_q_dot[c6_i30] = 0.0;
  }

  for (c6_i31 = 0; c6_i31 < 31; c6_i31++) {
    c6_q_dot[c6_i31] = 0.0;
  }

  for (c6_i32 = 0; c6_i32 < 31; c6_i32++) {
    c6_C[c6_i32] = c6_q_dot[c6_i32];
  }

  for (c6_i33 = 0; c6_i33 < 31; c6_i33++) {
    c6_q_dot[c6_i33] = c6_C[c6_i33];
  }

  c6_g_threshold(chartInstance);
  for (c6_i34 = 0; c6_i34 < 31; c6_i34++) {
    c6_C[c6_i34] = c6_q_dot[c6_i34];
  }

  for (c6_i35 = 0; c6_i35 < 31; c6_i35++) {
    c6_q_dot[c6_i35] = c6_C[c6_i35];
  }

  for (c6_i36 = 0; c6_i36 < 31; c6_i36++) {
    c6_q_dot[c6_i36] = 0.0;
    c6_i37 = 0;
    for (c6_i38 = 0; c6_i38 < 6; c6_i38++) {
      c6_q_dot[c6_i36] += c6_b_a[c6_i37 + c6_i36] * c6_b_b[c6_i38];
      c6_i37 += 31;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -13);
  _SFD_SYMBOL_SCOPE_POP();
  for (c6_i39 = 0; c6_i39 < 31; c6_i39++) {
    (*c6_b_q_dot)[c6_i39] = c6_q_dot[c6_i39];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c6_sfEvent);
}

static void initSimStructsc6_dirdyna_walkman_robotran
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber)
{
  (void)c6_machineNumber;
  (void)c6_chartNumber;
  (void)c6_instanceNumber;
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i40;
  real_T c6_b_inData[31];
  int32_T c6_i41;
  real_T c6_u[31];
  const mxArray *c6_y = NULL;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i40 = 0; c6_i40 < 31; c6_i40++) {
    c6_b_inData[c6_i40] = (*(real_T (*)[31])c6_inData)[c6_i40];
  }

  for (c6_i41 = 0; c6_i41 < 31; c6_i41++) {
    c6_u[c6_i41] = c6_b_inData[c6_i41];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 31), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_q_dot, const char_T *c6_identifier, real_T
  c6_y[31])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_q_dot), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_q_dot);
}

static void c6_b_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[31])
{
  real_T c6_dv2[31];
  int32_T c6_i42;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv2, 1, 0, 0U, 1, 0U, 1, 31);
  for (c6_i42 = 0; c6_i42 < 31; c6_i42++) {
    c6_y[c6_i42] = c6_dv2[c6_i42];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_q_dot;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[31];
  int32_T c6_i43;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_q_dot = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_q_dot), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_q_dot);
  for (c6_i43 = 0; c6_i43 < 31; c6_i43++) {
    (*(real_T (*)[31])c6_outData)[c6_i43] = c6_y[c6_i43];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i44;
  real_T c6_b_inData[3];
  int32_T c6_i45;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i44 = 0; c6_i44 < 3; c6_i44++) {
    c6_b_inData[c6_i44] = (*(real_T (*)[3])c6_inData)[c6_i44];
  }

  for (c6_i45 = 0; c6_i45 < 3; c6_i45++) {
    c6_u[c6_i45] = c6_b_inData[c6_i45];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i46;
  int32_T c6_i47;
  int32_T c6_i48;
  real_T c6_b_inData[9];
  int32_T c6_i49;
  int32_T c6_i50;
  int32_T c6_i51;
  real_T c6_u[9];
  const mxArray *c6_y = NULL;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i46 = 0;
  for (c6_i47 = 0; c6_i47 < 3; c6_i47++) {
    for (c6_i48 = 0; c6_i48 < 3; c6_i48++) {
      c6_b_inData[c6_i48 + c6_i46] = (*(real_T (*)[9])c6_inData)[c6_i48 + c6_i46];
    }

    c6_i46 += 3;
  }

  c6_i49 = 0;
  for (c6_i50 = 0; c6_i50 < 3; c6_i50++) {
    for (c6_i51 = 0; c6_i51 < 3; c6_i51++) {
      c6_u[c6_i51 + c6_i49] = c6_b_inData[c6_i51 + c6_i49];
    }

    c6_i49 += 3;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i52;
  int32_T c6_i53;
  int32_T c6_i54;
  real_T c6_b_inData[186];
  int32_T c6_i55;
  int32_T c6_i56;
  int32_T c6_i57;
  real_T c6_u[186];
  const mxArray *c6_y = NULL;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i52 = 0;
  for (c6_i53 = 0; c6_i53 < 31; c6_i53++) {
    for (c6_i54 = 0; c6_i54 < 6; c6_i54++) {
      c6_b_inData[c6_i54 + c6_i52] = (*(real_T (*)[186])c6_inData)[c6_i54 +
        c6_i52];
    }

    c6_i52 += 6;
  }

  c6_i55 = 0;
  for (c6_i56 = 0; c6_i56 < 31; c6_i56++) {
    for (c6_i57 = 0; c6_i57 < 6; c6_i57++) {
      c6_u[c6_i57 + c6_i55] = c6_b_inData[c6_i57 + c6_i55];
    }

    c6_i55 += 6;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 31), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static real_T c6_c_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct *
  chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_nargout;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_nargout = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_nargout), &c6_thisId);
  sf_mex_destroy(&c6_nargout);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i58;
  real_T c6_b_inData[6];
  int32_T c6_i59;
  real_T c6_u[6];
  const mxArray *c6_y = NULL;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i58 = 0; c6_i58 < 6; c6_i58++) {
    c6_b_inData[c6_i58] = (*(real_T (*)[6])c6_inData)[c6_i58];
  }

  for (c6_i59 = 0; c6_i59 < 6; c6_i59++) {
    c6_u[c6_i59] = c6_b_inData[c6_i59];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_d_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[6])
{
  real_T c6_dv3[6];
  int32_T c6_i60;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv3, 1, 0, 0U, 1, 0U, 1, 6);
  for (c6_i60 = 0; c6_i60 < 6; c6_i60++) {
    c6_y[c6_i60] = c6_dv3[c6_i60];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_u;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[6];
  int32_T c6_i61;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_u = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_u), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_u);
  for (c6_i61 = 0; c6_i61 < 6; c6_i61++) {
    (*(real_T (*)[6])c6_outData)[c6_i61] = c6_y[c6_i61];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static void c6_e_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[3])
{
  real_T c6_dv4[3];
  int32_T c6_i62;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv4, 1, 0, 0U, 1, 0U, 1, 3);
  for (c6_i62 = 0; c6_i62 < 3; c6_i62++) {
    c6_y[c6_i62] = c6_dv4[c6_i62];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_u_o;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[3];
  int32_T c6_i63;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_u_o = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_u_o), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_u_o);
  for (c6_i63 = 0; c6_i63 < 3; c6_i63++) {
    (*(real_T (*)[3])c6_outData)[c6_i63] = c6_y[c6_i63];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static void c6_f_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[9])
{
  real_T c6_dv5[9];
  int32_T c6_i64;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv5, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c6_i64 = 0; c6_i64 < 9; c6_i64++) {
    c6_y[c6_i64] = c6_dv5[c6_i64];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_KP;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[9];
  int32_T c6_i65;
  int32_T c6_i66;
  int32_T c6_i67;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_KP = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_KP), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_KP);
  c6_i65 = 0;
  for (c6_i66 = 0; c6_i66 < 3; c6_i66++) {
    for (c6_i67 = 0; c6_i67 < 3; c6_i67++) {
      (*(real_T (*)[9])c6_outData)[c6_i67 + c6_i65] = c6_y[c6_i67 + c6_i65];
    }

    c6_i65 += 3;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i68;
  int32_T c6_i69;
  int32_T c6_i70;
  real_T c6_b_inData[186];
  int32_T c6_i71;
  int32_T c6_i72;
  int32_T c6_i73;
  real_T c6_u[186];
  const mxArray *c6_y = NULL;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i68 = 0;
  for (c6_i69 = 0; c6_i69 < 6; c6_i69++) {
    for (c6_i70 = 0; c6_i70 < 31; c6_i70++) {
      c6_b_inData[c6_i70 + c6_i68] = (*(real_T (*)[186])c6_inData)[c6_i70 +
        c6_i68];
    }

    c6_i68 += 31;
  }

  c6_i71 = 0;
  for (c6_i72 = 0; c6_i72 < 6; c6_i72++) {
    for (c6_i73 = 0; c6_i73 < 31; c6_i73++) {
      c6_u[c6_i73 + c6_i71] = c6_b_inData[c6_i73 + c6_i71];
    }

    c6_i71 += 31;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 31, 6), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_g_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[186])
{
  real_T c6_dv6[186];
  int32_T c6_i74;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv6, 1, 0, 0U, 1, 0U, 2, 31, 6);
  for (c6_i74 = 0; c6_i74 < 186; c6_i74++) {
    c6_y[c6_i74] = c6_dv6[c6_i74];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_J_pinv;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[186];
  int32_T c6_i75;
  int32_T c6_i76;
  int32_T c6_i77;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_J_pinv = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_J_pinv), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_J_pinv);
  c6_i75 = 0;
  for (c6_i76 = 0; c6_i76 < 6; c6_i76++) {
    for (c6_i77 = 0; c6_i77 < 31; c6_i77++) {
      (*(real_T (*)[186])c6_outData)[c6_i77 + c6_i75] = c6_y[c6_i77 + c6_i75];
    }

    c6_i75 += 31;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_dirdyna_walkman_robotran_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  sf_mex_assign(&c6_nameCaptureInfo, sf_mex_createstruct("structure", 2, 197, 1),
                false);
  c6_info_helper(&c6_nameCaptureInfo);
  c6_b_info_helper(&c6_nameCaptureInfo);
  c6_c_info_helper(&c6_nameCaptureInfo);
  c6_d_info_helper(&c6_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(const mxArray **c6_info)
{
  const mxArray *c6_rhs0 = NULL;
  const mxArray *c6_lhs0 = NULL;
  const mxArray *c6_rhs1 = NULL;
  const mxArray *c6_lhs1 = NULL;
  const mxArray *c6_rhs2 = NULL;
  const mxArray *c6_lhs2 = NULL;
  const mxArray *c6_rhs3 = NULL;
  const mxArray *c6_lhs3 = NULL;
  const mxArray *c6_rhs4 = NULL;
  const mxArray *c6_lhs4 = NULL;
  const mxArray *c6_rhs5 = NULL;
  const mxArray *c6_lhs5 = NULL;
  const mxArray *c6_rhs6 = NULL;
  const mxArray *c6_lhs6 = NULL;
  const mxArray *c6_rhs7 = NULL;
  const mxArray *c6_lhs7 = NULL;
  const mxArray *c6_rhs8 = NULL;
  const mxArray *c6_lhs8 = NULL;
  const mxArray *c6_rhs9 = NULL;
  const mxArray *c6_lhs9 = NULL;
  const mxArray *c6_rhs10 = NULL;
  const mxArray *c6_lhs10 = NULL;
  const mxArray *c6_rhs11 = NULL;
  const mxArray *c6_lhs11 = NULL;
  const mxArray *c6_rhs12 = NULL;
  const mxArray *c6_lhs12 = NULL;
  const mxArray *c6_rhs13 = NULL;
  const mxArray *c6_lhs13 = NULL;
  const mxArray *c6_rhs14 = NULL;
  const mxArray *c6_lhs14 = NULL;
  const mxArray *c6_rhs15 = NULL;
  const mxArray *c6_lhs15 = NULL;
  const mxArray *c6_rhs16 = NULL;
  const mxArray *c6_lhs16 = NULL;
  const mxArray *c6_rhs17 = NULL;
  const mxArray *c6_lhs17 = NULL;
  const mxArray *c6_rhs18 = NULL;
  const mxArray *c6_lhs18 = NULL;
  const mxArray *c6_rhs19 = NULL;
  const mxArray *c6_lhs19 = NULL;
  const mxArray *c6_rhs20 = NULL;
  const mxArray *c6_lhs20 = NULL;
  const mxArray *c6_rhs21 = NULL;
  const mxArray *c6_lhs21 = NULL;
  const mxArray *c6_rhs22 = NULL;
  const mxArray *c6_lhs22 = NULL;
  const mxArray *c6_rhs23 = NULL;
  const mxArray *c6_lhs23 = NULL;
  const mxArray *c6_rhs24 = NULL;
  const mxArray *c6_lhs24 = NULL;
  const mxArray *c6_rhs25 = NULL;
  const mxArray *c6_lhs25 = NULL;
  const mxArray *c6_rhs26 = NULL;
  const mxArray *c6_lhs26 = NULL;
  const mxArray *c6_rhs27 = NULL;
  const mxArray *c6_lhs27 = NULL;
  const mxArray *c6_rhs28 = NULL;
  const mxArray *c6_lhs28 = NULL;
  const mxArray *c6_rhs29 = NULL;
  const mxArray *c6_lhs29 = NULL;
  const mxArray *c6_rhs30 = NULL;
  const mxArray *c6_lhs30 = NULL;
  const mxArray *c6_rhs31 = NULL;
  const mxArray *c6_lhs31 = NULL;
  const mxArray *c6_rhs32 = NULL;
  const mxArray *c6_lhs32 = NULL;
  const mxArray *c6_rhs33 = NULL;
  const mxArray *c6_lhs33 = NULL;
  const mxArray *c6_rhs34 = NULL;
  const mxArray *c6_lhs34 = NULL;
  const mxArray *c6_rhs35 = NULL;
  const mxArray *c6_lhs35 = NULL;
  const mxArray *c6_rhs36 = NULL;
  const mxArray *c6_lhs36 = NULL;
  const mxArray *c6_rhs37 = NULL;
  const mxArray *c6_lhs37 = NULL;
  const mxArray *c6_rhs38 = NULL;
  const mxArray *c6_lhs38 = NULL;
  const mxArray *c6_rhs39 = NULL;
  const mxArray *c6_lhs39 = NULL;
  const mxArray *c6_rhs40 = NULL;
  const mxArray *c6_lhs40 = NULL;
  const mxArray *c6_rhs41 = NULL;
  const mxArray *c6_lhs41 = NULL;
  const mxArray *c6_rhs42 = NULL;
  const mxArray *c6_lhs42 = NULL;
  const mxArray *c6_rhs43 = NULL;
  const mxArray *c6_lhs43 = NULL;
  const mxArray *c6_rhs44 = NULL;
  const mxArray *c6_lhs44 = NULL;
  const mxArray *c6_rhs45 = NULL;
  const mxArray *c6_lhs45 = NULL;
  const mxArray *c6_rhs46 = NULL;
  const mxArray *c6_lhs46 = NULL;
  const mxArray *c6_rhs47 = NULL;
  const mxArray *c6_lhs47 = NULL;
  const mxArray *c6_rhs48 = NULL;
  const mxArray *c6_lhs48 = NULL;
  const mxArray *c6_rhs49 = NULL;
  const mxArray *c6_lhs49 = NULL;
  const mxArray *c6_rhs50 = NULL;
  const mxArray *c6_lhs50 = NULL;
  const mxArray *c6_rhs51 = NULL;
  const mxArray *c6_lhs51 = NULL;
  const mxArray *c6_rhs52 = NULL;
  const mxArray *c6_lhs52 = NULL;
  const mxArray *c6_rhs53 = NULL;
  const mxArray *c6_lhs53 = NULL;
  const mxArray *c6_rhs54 = NULL;
  const mxArray *c6_lhs54 = NULL;
  const mxArray *c6_rhs55 = NULL;
  const mxArray *c6_lhs55 = NULL;
  const mxArray *c6_rhs56 = NULL;
  const mxArray *c6_lhs56 = NULL;
  const mxArray *c6_rhs57 = NULL;
  const mxArray *c6_lhs57 = NULL;
  const mxArray *c6_rhs58 = NULL;
  const mxArray *c6_lhs58 = NULL;
  const mxArray *c6_rhs59 = NULL;
  const mxArray *c6_lhs59 = NULL;
  const mxArray *c6_rhs60 = NULL;
  const mxArray *c6_lhs60 = NULL;
  const mxArray *c6_rhs61 = NULL;
  const mxArray *c6_lhs61 = NULL;
  const mxArray *c6_rhs62 = NULL;
  const mxArray *c6_lhs62 = NULL;
  const mxArray *c6_rhs63 = NULL;
  const mxArray *c6_lhs63 = NULL;
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("pinv"), "name", "name", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818828U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c6_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c6_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c6_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c6_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("svd"), "name", "name", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818832U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c6_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c6_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c6_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("intmax"), "name", "name", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c6_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c6_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("isfinite"), "name", "name", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c6_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c6_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("isinf"), "name", "name", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c6_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c6_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("isnan"), "name", "name", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c6_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c6_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_error"), "name", "name",
                  15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343830358U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c6_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xgesvd"), "name", "name",
                  16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818806U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c6_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m"),
                  "context", "context", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_lapack_xgesvd"), "name",
                  "name", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818810U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c6_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m"),
                  "context", "context", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_matlab_zsvdc"), "name",
                  "name", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1295284866U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c6_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c6_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c6_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c6_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c6_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("min"), "name", "name", 23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1311255318U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c6_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1378295984U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c6_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c6_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c6_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c6_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c6_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c6_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c6_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c6_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("max"), "name", "name", 32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1311255316U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c6_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1378295984U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c6_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c6_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c6_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c6_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c6_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_relop"), "name", "name",
                  38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1342451182U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c6_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "context",
                  "context", 39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326728322U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c6_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c6_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c6_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("intmin"), "name", "name", 42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c6_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c6_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("isnan"), "name", "name", 44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c6_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c6_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c6_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c6_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("max"), "name", "name", 48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1311255316U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c6_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c6_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 50);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 50);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 50);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c6_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 51);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 51);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 51);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c6_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 52);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 52);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 52);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c6_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 53);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 53);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 53);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c6_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 54);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 54);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 54);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 54);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c6_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 55);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 55);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 55);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c6_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 56);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  56);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 56);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c6_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 57);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 57);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c6_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 58);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 58);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c6_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 59);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 59);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 59);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 59);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c6_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 60);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 60);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c6_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 61);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 61);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 61);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c6_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 62);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("length"), "name", "name", 62);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 62);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 62);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c6_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength"),
                  "context", "context", 63);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 63);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c6_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c6_rhs0);
  sf_mex_destroy(&c6_lhs0);
  sf_mex_destroy(&c6_rhs1);
  sf_mex_destroy(&c6_lhs1);
  sf_mex_destroy(&c6_rhs2);
  sf_mex_destroy(&c6_lhs2);
  sf_mex_destroy(&c6_rhs3);
  sf_mex_destroy(&c6_lhs3);
  sf_mex_destroy(&c6_rhs4);
  sf_mex_destroy(&c6_lhs4);
  sf_mex_destroy(&c6_rhs5);
  sf_mex_destroy(&c6_lhs5);
  sf_mex_destroy(&c6_rhs6);
  sf_mex_destroy(&c6_lhs6);
  sf_mex_destroy(&c6_rhs7);
  sf_mex_destroy(&c6_lhs7);
  sf_mex_destroy(&c6_rhs8);
  sf_mex_destroy(&c6_lhs8);
  sf_mex_destroy(&c6_rhs9);
  sf_mex_destroy(&c6_lhs9);
  sf_mex_destroy(&c6_rhs10);
  sf_mex_destroy(&c6_lhs10);
  sf_mex_destroy(&c6_rhs11);
  sf_mex_destroy(&c6_lhs11);
  sf_mex_destroy(&c6_rhs12);
  sf_mex_destroy(&c6_lhs12);
  sf_mex_destroy(&c6_rhs13);
  sf_mex_destroy(&c6_lhs13);
  sf_mex_destroy(&c6_rhs14);
  sf_mex_destroy(&c6_lhs14);
  sf_mex_destroy(&c6_rhs15);
  sf_mex_destroy(&c6_lhs15);
  sf_mex_destroy(&c6_rhs16);
  sf_mex_destroy(&c6_lhs16);
  sf_mex_destroy(&c6_rhs17);
  sf_mex_destroy(&c6_lhs17);
  sf_mex_destroy(&c6_rhs18);
  sf_mex_destroy(&c6_lhs18);
  sf_mex_destroy(&c6_rhs19);
  sf_mex_destroy(&c6_lhs19);
  sf_mex_destroy(&c6_rhs20);
  sf_mex_destroy(&c6_lhs20);
  sf_mex_destroy(&c6_rhs21);
  sf_mex_destroy(&c6_lhs21);
  sf_mex_destroy(&c6_rhs22);
  sf_mex_destroy(&c6_lhs22);
  sf_mex_destroy(&c6_rhs23);
  sf_mex_destroy(&c6_lhs23);
  sf_mex_destroy(&c6_rhs24);
  sf_mex_destroy(&c6_lhs24);
  sf_mex_destroy(&c6_rhs25);
  sf_mex_destroy(&c6_lhs25);
  sf_mex_destroy(&c6_rhs26);
  sf_mex_destroy(&c6_lhs26);
  sf_mex_destroy(&c6_rhs27);
  sf_mex_destroy(&c6_lhs27);
  sf_mex_destroy(&c6_rhs28);
  sf_mex_destroy(&c6_lhs28);
  sf_mex_destroy(&c6_rhs29);
  sf_mex_destroy(&c6_lhs29);
  sf_mex_destroy(&c6_rhs30);
  sf_mex_destroy(&c6_lhs30);
  sf_mex_destroy(&c6_rhs31);
  sf_mex_destroy(&c6_lhs31);
  sf_mex_destroy(&c6_rhs32);
  sf_mex_destroy(&c6_lhs32);
  sf_mex_destroy(&c6_rhs33);
  sf_mex_destroy(&c6_lhs33);
  sf_mex_destroy(&c6_rhs34);
  sf_mex_destroy(&c6_lhs34);
  sf_mex_destroy(&c6_rhs35);
  sf_mex_destroy(&c6_lhs35);
  sf_mex_destroy(&c6_rhs36);
  sf_mex_destroy(&c6_lhs36);
  sf_mex_destroy(&c6_rhs37);
  sf_mex_destroy(&c6_lhs37);
  sf_mex_destroy(&c6_rhs38);
  sf_mex_destroy(&c6_lhs38);
  sf_mex_destroy(&c6_rhs39);
  sf_mex_destroy(&c6_lhs39);
  sf_mex_destroy(&c6_rhs40);
  sf_mex_destroy(&c6_lhs40);
  sf_mex_destroy(&c6_rhs41);
  sf_mex_destroy(&c6_lhs41);
  sf_mex_destroy(&c6_rhs42);
  sf_mex_destroy(&c6_lhs42);
  sf_mex_destroy(&c6_rhs43);
  sf_mex_destroy(&c6_lhs43);
  sf_mex_destroy(&c6_rhs44);
  sf_mex_destroy(&c6_lhs44);
  sf_mex_destroy(&c6_rhs45);
  sf_mex_destroy(&c6_lhs45);
  sf_mex_destroy(&c6_rhs46);
  sf_mex_destroy(&c6_lhs46);
  sf_mex_destroy(&c6_rhs47);
  sf_mex_destroy(&c6_lhs47);
  sf_mex_destroy(&c6_rhs48);
  sf_mex_destroy(&c6_lhs48);
  sf_mex_destroy(&c6_rhs49);
  sf_mex_destroy(&c6_lhs49);
  sf_mex_destroy(&c6_rhs50);
  sf_mex_destroy(&c6_lhs50);
  sf_mex_destroy(&c6_rhs51);
  sf_mex_destroy(&c6_lhs51);
  sf_mex_destroy(&c6_rhs52);
  sf_mex_destroy(&c6_lhs52);
  sf_mex_destroy(&c6_rhs53);
  sf_mex_destroy(&c6_lhs53);
  sf_mex_destroy(&c6_rhs54);
  sf_mex_destroy(&c6_lhs54);
  sf_mex_destroy(&c6_rhs55);
  sf_mex_destroy(&c6_lhs55);
  sf_mex_destroy(&c6_rhs56);
  sf_mex_destroy(&c6_lhs56);
  sf_mex_destroy(&c6_rhs57);
  sf_mex_destroy(&c6_lhs57);
  sf_mex_destroy(&c6_rhs58);
  sf_mex_destroy(&c6_lhs58);
  sf_mex_destroy(&c6_rhs59);
  sf_mex_destroy(&c6_lhs59);
  sf_mex_destroy(&c6_rhs60);
  sf_mex_destroy(&c6_lhs60);
  sf_mex_destroy(&c6_rhs61);
  sf_mex_destroy(&c6_lhs61);
  sf_mex_destroy(&c6_rhs62);
  sf_mex_destroy(&c6_lhs62);
  sf_mex_destroy(&c6_rhs63);
  sf_mex_destroy(&c6_lhs63);
}

static const mxArray *c6_emlrt_marshallOut(const char * c6_u)
{
  const mxArray *c6_y = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c6_u)), false);
  return c6_y;
}

static const mxArray *c6_b_emlrt_marshallOut(const uint32_T c6_u)
{
  const mxArray *c6_y = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 7, 0U, 0U, 0U, 0), false);
  return c6_y;
}

static void c6_b_info_helper(const mxArray **c6_info)
{
  const mxArray *c6_rhs64 = NULL;
  const mxArray *c6_lhs64 = NULL;
  const mxArray *c6_rhs65 = NULL;
  const mxArray *c6_lhs65 = NULL;
  const mxArray *c6_rhs66 = NULL;
  const mxArray *c6_lhs66 = NULL;
  const mxArray *c6_rhs67 = NULL;
  const mxArray *c6_lhs67 = NULL;
  const mxArray *c6_rhs68 = NULL;
  const mxArray *c6_lhs68 = NULL;
  const mxArray *c6_rhs69 = NULL;
  const mxArray *c6_lhs69 = NULL;
  const mxArray *c6_rhs70 = NULL;
  const mxArray *c6_lhs70 = NULL;
  const mxArray *c6_rhs71 = NULL;
  const mxArray *c6_lhs71 = NULL;
  const mxArray *c6_rhs72 = NULL;
  const mxArray *c6_lhs72 = NULL;
  const mxArray *c6_rhs73 = NULL;
  const mxArray *c6_lhs73 = NULL;
  const mxArray *c6_rhs74 = NULL;
  const mxArray *c6_lhs74 = NULL;
  const mxArray *c6_rhs75 = NULL;
  const mxArray *c6_lhs75 = NULL;
  const mxArray *c6_rhs76 = NULL;
  const mxArray *c6_lhs76 = NULL;
  const mxArray *c6_rhs77 = NULL;
  const mxArray *c6_lhs77 = NULL;
  const mxArray *c6_rhs78 = NULL;
  const mxArray *c6_lhs78 = NULL;
  const mxArray *c6_rhs79 = NULL;
  const mxArray *c6_lhs79 = NULL;
  const mxArray *c6_rhs80 = NULL;
  const mxArray *c6_lhs80 = NULL;
  const mxArray *c6_rhs81 = NULL;
  const mxArray *c6_lhs81 = NULL;
  const mxArray *c6_rhs82 = NULL;
  const mxArray *c6_lhs82 = NULL;
  const mxArray *c6_rhs83 = NULL;
  const mxArray *c6_lhs83 = NULL;
  const mxArray *c6_rhs84 = NULL;
  const mxArray *c6_lhs84 = NULL;
  const mxArray *c6_rhs85 = NULL;
  const mxArray *c6_lhs85 = NULL;
  const mxArray *c6_rhs86 = NULL;
  const mxArray *c6_lhs86 = NULL;
  const mxArray *c6_rhs87 = NULL;
  const mxArray *c6_lhs87 = NULL;
  const mxArray *c6_rhs88 = NULL;
  const mxArray *c6_lhs88 = NULL;
  const mxArray *c6_rhs89 = NULL;
  const mxArray *c6_lhs89 = NULL;
  const mxArray *c6_rhs90 = NULL;
  const mxArray *c6_lhs90 = NULL;
  const mxArray *c6_rhs91 = NULL;
  const mxArray *c6_lhs91 = NULL;
  const mxArray *c6_rhs92 = NULL;
  const mxArray *c6_lhs92 = NULL;
  const mxArray *c6_rhs93 = NULL;
  const mxArray *c6_lhs93 = NULL;
  const mxArray *c6_rhs94 = NULL;
  const mxArray *c6_lhs94 = NULL;
  const mxArray *c6_rhs95 = NULL;
  const mxArray *c6_lhs95 = NULL;
  const mxArray *c6_rhs96 = NULL;
  const mxArray *c6_lhs96 = NULL;
  const mxArray *c6_rhs97 = NULL;
  const mxArray *c6_lhs97 = NULL;
  const mxArray *c6_rhs98 = NULL;
  const mxArray *c6_lhs98 = NULL;
  const mxArray *c6_rhs99 = NULL;
  const mxArray *c6_lhs99 = NULL;
  const mxArray *c6_rhs100 = NULL;
  const mxArray *c6_lhs100 = NULL;
  const mxArray *c6_rhs101 = NULL;
  const mxArray *c6_lhs101 = NULL;
  const mxArray *c6_rhs102 = NULL;
  const mxArray *c6_lhs102 = NULL;
  const mxArray *c6_rhs103 = NULL;
  const mxArray *c6_lhs103 = NULL;
  const mxArray *c6_rhs104 = NULL;
  const mxArray *c6_lhs104 = NULL;
  const mxArray *c6_rhs105 = NULL;
  const mxArray *c6_lhs105 = NULL;
  const mxArray *c6_rhs106 = NULL;
  const mxArray *c6_lhs106 = NULL;
  const mxArray *c6_rhs107 = NULL;
  const mxArray *c6_lhs107 = NULL;
  const mxArray *c6_rhs108 = NULL;
  const mxArray *c6_lhs108 = NULL;
  const mxArray *c6_rhs109 = NULL;
  const mxArray *c6_lhs109 = NULL;
  const mxArray *c6_rhs110 = NULL;
  const mxArray *c6_lhs110 = NULL;
  const mxArray *c6_rhs111 = NULL;
  const mxArray *c6_lhs111 = NULL;
  const mxArray *c6_rhs112 = NULL;
  const mxArray *c6_lhs112 = NULL;
  const mxArray *c6_rhs113 = NULL;
  const mxArray *c6_lhs113 = NULL;
  const mxArray *c6_rhs114 = NULL;
  const mxArray *c6_lhs114 = NULL;
  const mxArray *c6_rhs115 = NULL;
  const mxArray *c6_lhs115 = NULL;
  const mxArray *c6_rhs116 = NULL;
  const mxArray *c6_lhs116 = NULL;
  const mxArray *c6_rhs117 = NULL;
  const mxArray *c6_lhs117 = NULL;
  const mxArray *c6_rhs118 = NULL;
  const mxArray *c6_lhs118 = NULL;
  const mxArray *c6_rhs119 = NULL;
  const mxArray *c6_lhs119 = NULL;
  const mxArray *c6_rhs120 = NULL;
  const mxArray *c6_lhs120 = NULL;
  const mxArray *c6_rhs121 = NULL;
  const mxArray *c6_lhs121 = NULL;
  const mxArray *c6_rhs122 = NULL;
  const mxArray *c6_lhs122 = NULL;
  const mxArray *c6_rhs123 = NULL;
  const mxArray *c6_lhs123 = NULL;
  const mxArray *c6_rhs124 = NULL;
  const mxArray *c6_lhs124 = NULL;
  const mxArray *c6_rhs125 = NULL;
  const mxArray *c6_lhs125 = NULL;
  const mxArray *c6_rhs126 = NULL;
  const mxArray *c6_lhs126 = NULL;
  const mxArray *c6_rhs127 = NULL;
  const mxArray *c6_lhs127 = NULL;
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 64);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xnrm2"),
                  "name", "name", 64);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c6_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 65);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("abs"), "name", "name", 65);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 65);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c6_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 66);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 66);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 66);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c6_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 67);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 67);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 67);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c6_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 68);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("realmin"), "name", "name", 68);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 68);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1307651242U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c6_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 69);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_realmin"), "name", "name",
                  69);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 69);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1307651244U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c6_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 70);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 70);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c6_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 71);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 71);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 71);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c6_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 72);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 72);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 72);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c6_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 73);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 73);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 73);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 73);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c6_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 74);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 74);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c6_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 75);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_div"), "name", "name", 75);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 75);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c6_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 76);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 76);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 76);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c6_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 77);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xscal"), "name", "name",
                  77);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 77);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c6_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 78);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 78);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 78);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c6_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 79);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xscal"),
                  "name", "name", 79);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c6_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 80);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 80);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 80);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c6_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 81);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 81);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 81);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c6_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 82);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("length"), "name", "name", 82);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 82);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c6_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 83);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 83);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 83);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c6_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 84);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xscal"),
                  "name", "name", 84);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "resolved", "resolved", 84);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c6_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 85);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 85);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c6_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 86);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 86);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 86);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 86);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c6_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 87);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 87);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 87);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 87);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c6_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 88);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 88);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 88);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c6_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 89);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xdotc"), "name", "name",
                  89);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"),
                  "resolved", "resolved", 89);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980690U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c6_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 90);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 90);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 90);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c6_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 91);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xdotc"),
                  "name", "name", 91);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "resolved", "resolved", 91);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c6_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "context", "context", 92);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xdot"),
                  "name", "name", 92);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "resolved", "resolved", 92);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c6_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 93);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 93);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c6_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 94);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 94);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 94);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 94);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c6_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 95);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("length"), "name", "name", 95);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 95);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c6_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 96);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xdot"),
                  "name", "name", 96);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "resolved", "resolved", 96);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c6_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "context", "context", 97);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xdotx"),
                  "name", "name", 97);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "resolved", "resolved", 97);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c6_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 98);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 98);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 98);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c6_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 99);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 99);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 99);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c6_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 100);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 100);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 100);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c6_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs100), "rhs", "rhs",
                  100);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs100), "lhs", "lhs",
                  100);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 101);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xaxpy"), "name", "name",
                  101);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 101);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"),
                  "resolved", "resolved", 101);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c6_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs101), "rhs", "rhs",
                  101);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs101), "lhs", "lhs",
                  101);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"), "context",
                  "context", 102);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 102);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 102);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 102);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c6_rhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs102), "rhs", "rhs",
                  102);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs102), "lhs", "lhs",
                  102);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"), "context",
                  "context", 103);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xaxpy"),
                  "name", "name", 103);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 103);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "resolved", "resolved", 103);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c6_rhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs103), "rhs", "rhs",
                  103);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs103), "lhs", "lhs",
                  103);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 104);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 104);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 104);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 104);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c6_rhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs104), "rhs", "rhs",
                  104);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs104), "lhs", "lhs",
                  104);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!below_threshold"),
                  "context", "context", 105);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 105);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 105);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 105);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c6_rhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs105), "rhs", "rhs",
                  105);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs105), "lhs", "lhs",
                  105);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!below_threshold"),
                  "context", "context", 106);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("length"), "name", "name", 106);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 106);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 106);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c6_rhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs106), "rhs", "rhs",
                  106);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs106), "lhs", "lhs",
                  106);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 107);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 107);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 107);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c6_rhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs107), "rhs", "rhs",
                  107);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs107), "lhs", "lhs",
                  107);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 108);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xaxpy"),
                  "name", "name", 108);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "resolved", "resolved", 108);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c6_rhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs108), "rhs", "rhs",
                  108);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs108), "lhs", "lhs",
                  108);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 109);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.isaUint"),
                  "name", "name", 109);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 109);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/isaUint.p"),
                  "resolved", "resolved", 109);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c6_rhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs109), "rhs", "rhs",
                  109);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs109), "lhs", "lhs",
                  109);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 110);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 110);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 110);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 110);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c6_rhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs110), "rhs", "rhs",
                  110);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs110), "lhs", "lhs",
                  110);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 111);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 111);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 111);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 111);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c6_rhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs111), "rhs", "rhs",
                  111);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs111), "lhs", "lhs",
                  111);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 112);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 112);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 112);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 112);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c6_rhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs112), "rhs", "rhs",
                  112);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs112), "lhs", "lhs",
                  112);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 113);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 113);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 113);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 113);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c6_rhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs113), "rhs", "rhs",
                  113);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs113), "lhs", "lhs",
                  113);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 114);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("intmin"), "name", "name", 114);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 114);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 114);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c6_rhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs114), "rhs", "rhs",
                  114);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs114), "lhs", "lhs",
                  114);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 115);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("abs"), "name", "name", 115);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 115);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 115);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c6_rhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs115), "rhs", "rhs",
                  115);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs115), "lhs", "lhs",
                  115);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 116);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("realmin"), "name", "name", 116);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 116);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 116);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1307651242U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c6_rhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs116), "rhs", "rhs",
                  116);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs116), "lhs", "lhs",
                  116);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 117);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eps"), "name", "name", 117);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 117);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 117);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c6_rhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs117), "rhs", "rhs",
                  117);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs117), "lhs", "lhs",
                  117);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 118);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 118);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 118);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818782U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c6_rhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs118), "rhs", "rhs",
                  118);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs118), "lhs", "lhs",
                  118);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 119);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_eps"), "name", "name", 119);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 119);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 119);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c6_rhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs119), "rhs", "rhs",
                  119);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs119), "lhs", "lhs",
                  119);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 120);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 120);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 120);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c6_rhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs120), "rhs", "rhs",
                  120);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs120), "lhs", "lhs",
                  120);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 121);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 121);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 121);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c6_rhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs121), "rhs", "rhs",
                  121);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs121), "lhs", "lhs",
                  121);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 122);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_error"), "name", "name",
                  122);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 122);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343830358U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c6_rhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs122), "rhs", "rhs",
                  122);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs122), "lhs", "lhs",
                  122);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 123);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 123);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 123);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c6_rhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs123), "rhs", "rhs",
                  123);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs123), "lhs", "lhs",
                  123);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "context", "context", 124);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 124);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 124);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 124);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c6_rhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs124), "rhs", "rhs",
                  124);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs124), "lhs", "lhs",
                  124);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 125);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 125);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 125);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 125);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c6_rhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs125), "rhs", "rhs",
                  125);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs125), "lhs", "lhs",
                  125);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 126);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 126);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 126);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 126);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c6_rhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs126), "rhs", "rhs",
                  126);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs126), "lhs", "lhs",
                  126);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 127);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 127);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 127);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 127);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c6_rhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs127), "rhs", "rhs",
                  127);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs127), "lhs", "lhs",
                  127);
  sf_mex_destroy(&c6_rhs64);
  sf_mex_destroy(&c6_lhs64);
  sf_mex_destroy(&c6_rhs65);
  sf_mex_destroy(&c6_lhs65);
  sf_mex_destroy(&c6_rhs66);
  sf_mex_destroy(&c6_lhs66);
  sf_mex_destroy(&c6_rhs67);
  sf_mex_destroy(&c6_lhs67);
  sf_mex_destroy(&c6_rhs68);
  sf_mex_destroy(&c6_lhs68);
  sf_mex_destroy(&c6_rhs69);
  sf_mex_destroy(&c6_lhs69);
  sf_mex_destroy(&c6_rhs70);
  sf_mex_destroy(&c6_lhs70);
  sf_mex_destroy(&c6_rhs71);
  sf_mex_destroy(&c6_lhs71);
  sf_mex_destroy(&c6_rhs72);
  sf_mex_destroy(&c6_lhs72);
  sf_mex_destroy(&c6_rhs73);
  sf_mex_destroy(&c6_lhs73);
  sf_mex_destroy(&c6_rhs74);
  sf_mex_destroy(&c6_lhs74);
  sf_mex_destroy(&c6_rhs75);
  sf_mex_destroy(&c6_lhs75);
  sf_mex_destroy(&c6_rhs76);
  sf_mex_destroy(&c6_lhs76);
  sf_mex_destroy(&c6_rhs77);
  sf_mex_destroy(&c6_lhs77);
  sf_mex_destroy(&c6_rhs78);
  sf_mex_destroy(&c6_lhs78);
  sf_mex_destroy(&c6_rhs79);
  sf_mex_destroy(&c6_lhs79);
  sf_mex_destroy(&c6_rhs80);
  sf_mex_destroy(&c6_lhs80);
  sf_mex_destroy(&c6_rhs81);
  sf_mex_destroy(&c6_lhs81);
  sf_mex_destroy(&c6_rhs82);
  sf_mex_destroy(&c6_lhs82);
  sf_mex_destroy(&c6_rhs83);
  sf_mex_destroy(&c6_lhs83);
  sf_mex_destroy(&c6_rhs84);
  sf_mex_destroy(&c6_lhs84);
  sf_mex_destroy(&c6_rhs85);
  sf_mex_destroy(&c6_lhs85);
  sf_mex_destroy(&c6_rhs86);
  sf_mex_destroy(&c6_lhs86);
  sf_mex_destroy(&c6_rhs87);
  sf_mex_destroy(&c6_lhs87);
  sf_mex_destroy(&c6_rhs88);
  sf_mex_destroy(&c6_lhs88);
  sf_mex_destroy(&c6_rhs89);
  sf_mex_destroy(&c6_lhs89);
  sf_mex_destroy(&c6_rhs90);
  sf_mex_destroy(&c6_lhs90);
  sf_mex_destroy(&c6_rhs91);
  sf_mex_destroy(&c6_lhs91);
  sf_mex_destroy(&c6_rhs92);
  sf_mex_destroy(&c6_lhs92);
  sf_mex_destroy(&c6_rhs93);
  sf_mex_destroy(&c6_lhs93);
  sf_mex_destroy(&c6_rhs94);
  sf_mex_destroy(&c6_lhs94);
  sf_mex_destroy(&c6_rhs95);
  sf_mex_destroy(&c6_lhs95);
  sf_mex_destroy(&c6_rhs96);
  sf_mex_destroy(&c6_lhs96);
  sf_mex_destroy(&c6_rhs97);
  sf_mex_destroy(&c6_lhs97);
  sf_mex_destroy(&c6_rhs98);
  sf_mex_destroy(&c6_lhs98);
  sf_mex_destroy(&c6_rhs99);
  sf_mex_destroy(&c6_lhs99);
  sf_mex_destroy(&c6_rhs100);
  sf_mex_destroy(&c6_lhs100);
  sf_mex_destroy(&c6_rhs101);
  sf_mex_destroy(&c6_lhs101);
  sf_mex_destroy(&c6_rhs102);
  sf_mex_destroy(&c6_lhs102);
  sf_mex_destroy(&c6_rhs103);
  sf_mex_destroy(&c6_lhs103);
  sf_mex_destroy(&c6_rhs104);
  sf_mex_destroy(&c6_lhs104);
  sf_mex_destroy(&c6_rhs105);
  sf_mex_destroy(&c6_lhs105);
  sf_mex_destroy(&c6_rhs106);
  sf_mex_destroy(&c6_lhs106);
  sf_mex_destroy(&c6_rhs107);
  sf_mex_destroy(&c6_lhs107);
  sf_mex_destroy(&c6_rhs108);
  sf_mex_destroy(&c6_lhs108);
  sf_mex_destroy(&c6_rhs109);
  sf_mex_destroy(&c6_lhs109);
  sf_mex_destroy(&c6_rhs110);
  sf_mex_destroy(&c6_lhs110);
  sf_mex_destroy(&c6_rhs111);
  sf_mex_destroy(&c6_lhs111);
  sf_mex_destroy(&c6_rhs112);
  sf_mex_destroy(&c6_lhs112);
  sf_mex_destroy(&c6_rhs113);
  sf_mex_destroy(&c6_lhs113);
  sf_mex_destroy(&c6_rhs114);
  sf_mex_destroy(&c6_lhs114);
  sf_mex_destroy(&c6_rhs115);
  sf_mex_destroy(&c6_lhs115);
  sf_mex_destroy(&c6_rhs116);
  sf_mex_destroy(&c6_lhs116);
  sf_mex_destroy(&c6_rhs117);
  sf_mex_destroy(&c6_lhs117);
  sf_mex_destroy(&c6_rhs118);
  sf_mex_destroy(&c6_lhs118);
  sf_mex_destroy(&c6_rhs119);
  sf_mex_destroy(&c6_lhs119);
  sf_mex_destroy(&c6_rhs120);
  sf_mex_destroy(&c6_lhs120);
  sf_mex_destroy(&c6_rhs121);
  sf_mex_destroy(&c6_lhs121);
  sf_mex_destroy(&c6_rhs122);
  sf_mex_destroy(&c6_lhs122);
  sf_mex_destroy(&c6_rhs123);
  sf_mex_destroy(&c6_lhs123);
  sf_mex_destroy(&c6_rhs124);
  sf_mex_destroy(&c6_lhs124);
  sf_mex_destroy(&c6_rhs125);
  sf_mex_destroy(&c6_lhs125);
  sf_mex_destroy(&c6_rhs126);
  sf_mex_destroy(&c6_lhs126);
  sf_mex_destroy(&c6_rhs127);
  sf_mex_destroy(&c6_lhs127);
}

static void c6_c_info_helper(const mxArray **c6_info)
{
  const mxArray *c6_rhs128 = NULL;
  const mxArray *c6_lhs128 = NULL;
  const mxArray *c6_rhs129 = NULL;
  const mxArray *c6_lhs129 = NULL;
  const mxArray *c6_rhs130 = NULL;
  const mxArray *c6_lhs130 = NULL;
  const mxArray *c6_rhs131 = NULL;
  const mxArray *c6_lhs131 = NULL;
  const mxArray *c6_rhs132 = NULL;
  const mxArray *c6_lhs132 = NULL;
  const mxArray *c6_rhs133 = NULL;
  const mxArray *c6_lhs133 = NULL;
  const mxArray *c6_rhs134 = NULL;
  const mxArray *c6_lhs134 = NULL;
  const mxArray *c6_rhs135 = NULL;
  const mxArray *c6_lhs135 = NULL;
  const mxArray *c6_rhs136 = NULL;
  const mxArray *c6_lhs136 = NULL;
  const mxArray *c6_rhs137 = NULL;
  const mxArray *c6_lhs137 = NULL;
  const mxArray *c6_rhs138 = NULL;
  const mxArray *c6_lhs138 = NULL;
  const mxArray *c6_rhs139 = NULL;
  const mxArray *c6_lhs139 = NULL;
  const mxArray *c6_rhs140 = NULL;
  const mxArray *c6_lhs140 = NULL;
  const mxArray *c6_rhs141 = NULL;
  const mxArray *c6_lhs141 = NULL;
  const mxArray *c6_rhs142 = NULL;
  const mxArray *c6_lhs142 = NULL;
  const mxArray *c6_rhs143 = NULL;
  const mxArray *c6_lhs143 = NULL;
  const mxArray *c6_rhs144 = NULL;
  const mxArray *c6_lhs144 = NULL;
  const mxArray *c6_rhs145 = NULL;
  const mxArray *c6_lhs145 = NULL;
  const mxArray *c6_rhs146 = NULL;
  const mxArray *c6_lhs146 = NULL;
  const mxArray *c6_rhs147 = NULL;
  const mxArray *c6_lhs147 = NULL;
  const mxArray *c6_rhs148 = NULL;
  const mxArray *c6_lhs148 = NULL;
  const mxArray *c6_rhs149 = NULL;
  const mxArray *c6_lhs149 = NULL;
  const mxArray *c6_rhs150 = NULL;
  const mxArray *c6_lhs150 = NULL;
  const mxArray *c6_rhs151 = NULL;
  const mxArray *c6_lhs151 = NULL;
  const mxArray *c6_rhs152 = NULL;
  const mxArray *c6_lhs152 = NULL;
  const mxArray *c6_rhs153 = NULL;
  const mxArray *c6_lhs153 = NULL;
  const mxArray *c6_rhs154 = NULL;
  const mxArray *c6_lhs154 = NULL;
  const mxArray *c6_rhs155 = NULL;
  const mxArray *c6_lhs155 = NULL;
  const mxArray *c6_rhs156 = NULL;
  const mxArray *c6_lhs156 = NULL;
  const mxArray *c6_rhs157 = NULL;
  const mxArray *c6_lhs157 = NULL;
  const mxArray *c6_rhs158 = NULL;
  const mxArray *c6_lhs158 = NULL;
  const mxArray *c6_rhs159 = NULL;
  const mxArray *c6_lhs159 = NULL;
  const mxArray *c6_rhs160 = NULL;
  const mxArray *c6_lhs160 = NULL;
  const mxArray *c6_rhs161 = NULL;
  const mxArray *c6_lhs161 = NULL;
  const mxArray *c6_rhs162 = NULL;
  const mxArray *c6_lhs162 = NULL;
  const mxArray *c6_rhs163 = NULL;
  const mxArray *c6_lhs163 = NULL;
  const mxArray *c6_rhs164 = NULL;
  const mxArray *c6_lhs164 = NULL;
  const mxArray *c6_rhs165 = NULL;
  const mxArray *c6_lhs165 = NULL;
  const mxArray *c6_rhs166 = NULL;
  const mxArray *c6_lhs166 = NULL;
  const mxArray *c6_rhs167 = NULL;
  const mxArray *c6_lhs167 = NULL;
  const mxArray *c6_rhs168 = NULL;
  const mxArray *c6_lhs168 = NULL;
  const mxArray *c6_rhs169 = NULL;
  const mxArray *c6_lhs169 = NULL;
  const mxArray *c6_rhs170 = NULL;
  const mxArray *c6_lhs170 = NULL;
  const mxArray *c6_rhs171 = NULL;
  const mxArray *c6_lhs171 = NULL;
  const mxArray *c6_rhs172 = NULL;
  const mxArray *c6_lhs172 = NULL;
  const mxArray *c6_rhs173 = NULL;
  const mxArray *c6_lhs173 = NULL;
  const mxArray *c6_rhs174 = NULL;
  const mxArray *c6_lhs174 = NULL;
  const mxArray *c6_rhs175 = NULL;
  const mxArray *c6_lhs175 = NULL;
  const mxArray *c6_rhs176 = NULL;
  const mxArray *c6_lhs176 = NULL;
  const mxArray *c6_rhs177 = NULL;
  const mxArray *c6_lhs177 = NULL;
  const mxArray *c6_rhs178 = NULL;
  const mxArray *c6_lhs178 = NULL;
  const mxArray *c6_rhs179 = NULL;
  const mxArray *c6_lhs179 = NULL;
  const mxArray *c6_rhs180 = NULL;
  const mxArray *c6_lhs180 = NULL;
  const mxArray *c6_rhs181 = NULL;
  const mxArray *c6_lhs181 = NULL;
  const mxArray *c6_rhs182 = NULL;
  const mxArray *c6_lhs182 = NULL;
  const mxArray *c6_rhs183 = NULL;
  const mxArray *c6_lhs183 = NULL;
  const mxArray *c6_rhs184 = NULL;
  const mxArray *c6_lhs184 = NULL;
  const mxArray *c6_rhs185 = NULL;
  const mxArray *c6_lhs185 = NULL;
  const mxArray *c6_rhs186 = NULL;
  const mxArray *c6_lhs186 = NULL;
  const mxArray *c6_rhs187 = NULL;
  const mxArray *c6_lhs187 = NULL;
  const mxArray *c6_rhs188 = NULL;
  const mxArray *c6_lhs188 = NULL;
  const mxArray *c6_rhs189 = NULL;
  const mxArray *c6_lhs189 = NULL;
  const mxArray *c6_rhs190 = NULL;
  const mxArray *c6_lhs190 = NULL;
  const mxArray *c6_rhs191 = NULL;
  const mxArray *c6_lhs191 = NULL;
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 128);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("isnan"), "name", "name", 128);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 128);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 128);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c6_rhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs128), "rhs", "rhs",
                  128);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs128), "lhs", "lhs",
                  128);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 129);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 129);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 129);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 129);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c6_rhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs129), "rhs", "rhs",
                  129);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs129), "lhs", "lhs",
                  129);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 130);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 130);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 130);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 130);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c6_rhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs130), "rhs", "rhs",
                  130);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs130), "lhs", "lhs",
                  130);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 131);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_relop"), "name", "name",
                  131);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 131);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 131);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1342451182U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c6_rhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs131), "rhs", "rhs",
                  131);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs131), "lhs", "lhs",
                  131);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 132);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("sqrt"), "name", "name", 132);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 132);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 132);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c6_rhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs132), "rhs", "rhs",
                  132);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs132), "lhs", "lhs",
                  132);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 133);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_error"), "name", "name",
                  133);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 133);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 133);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343830358U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c6_rhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs133), "rhs", "rhs",
                  133);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs133), "lhs", "lhs",
                  133);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 134);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 134);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 134);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 134);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818738U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c6_rhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs134), "rhs", "rhs",
                  134);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs134), "lhs", "lhs",
                  134);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 135);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xrotg"), "name", "name",
                  135);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 135);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"),
                  "resolved", "resolved", 135);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c6_rhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs135), "rhs", "rhs",
                  135);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs135), "lhs", "lhs",
                  135);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"), "context",
                  "context", 136);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 136);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 136);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 136);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c6_rhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs136), "rhs", "rhs",
                  136);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs136), "lhs", "lhs",
                  136);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"), "context",
                  "context", 137);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xrotg"),
                  "name", "name", 137);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "resolved", "resolved", 137);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c6_rhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs137), "rhs", "rhs",
                  137);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs137), "lhs", "lhs",
                  137);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "context", "context", 138);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 138);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 138);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 138);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c6_rhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs138), "rhs", "rhs",
                  138);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs138), "lhs", "lhs",
                  138);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "context", "context", 139);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xrotg"),
                  "name", "name", 139);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 139);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "resolved", "resolved", 139);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c6_rhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs139), "rhs", "rhs",
                  139);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs139), "lhs", "lhs",
                  139);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 140);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("abs"), "name", "name", 140);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 140);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 140);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c6_rhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs140), "rhs", "rhs",
                  140);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs140), "lhs", "lhs",
                  140);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 141);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("mrdivide"), "name", "name",
                  141);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 141);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 141);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1388460096U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1370009886U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c6_rhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs141), "rhs", "rhs",
                  141);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs141), "lhs", "lhs",
                  141);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 142);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 142);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 142);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 142);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c6_rhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs142), "rhs", "rhs",
                  142);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs142), "lhs", "lhs",
                  142);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 143);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("rdivide"), "name", "name", 143);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 143);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 143);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c6_rhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs143), "rhs", "rhs",
                  143);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs143), "lhs", "lhs",
                  143);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 144);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 144);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 144);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c6_rhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs144), "rhs", "rhs",
                  144);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs144), "lhs", "lhs",
                  144);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 145);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 145);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 145);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 145);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c6_rhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs145), "rhs", "rhs",
                  145);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs145), "lhs", "lhs",
                  145);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 146);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_div"), "name", "name", 146);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 146);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 146);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c6_rhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs146), "rhs", "rhs",
                  146);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs146), "lhs", "lhs",
                  146);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 147);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("sqrt"), "name", "name", 147);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 147);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c6_rhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs147), "rhs", "rhs",
                  147);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs147), "lhs", "lhs",
                  147);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p!eml_ceval_xrotg"),
                  "context", "context", 148);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 148);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 148);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 148);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c6_rhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs148), "rhs", "rhs",
                  148);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs148), "lhs", "lhs",
                  148);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 149);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xrot"), "name", "name",
                  149);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "resolved",
                  "resolved", 149);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c6_rhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs149), "rhs", "rhs",
                  149);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs149), "lhs", "lhs",
                  149);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "context",
                  "context", 150);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 150);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 150);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c6_rhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs150), "rhs", "rhs",
                  150);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs150), "lhs", "lhs",
                  150);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "context",
                  "context", 151);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xrot"),
                  "name", "name", 151);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "resolved", "resolved", 151);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c6_rhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs151), "rhs", "rhs",
                  151);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs151), "lhs", "lhs",
                  151);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 152);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 152);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 152);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 152);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c6_rhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs152), "rhs", "rhs",
                  152);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs152), "lhs", "lhs",
                  152);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p!below_threshold"),
                  "context", "context", 153);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 153);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 153);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 153);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c6_rhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs153), "rhs", "rhs",
                  153);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs153), "lhs", "lhs",
                  153);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 154);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 154);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 154);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c6_rhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs154), "rhs", "rhs",
                  154);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs154), "lhs", "lhs",
                  154);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 155);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xrot"),
                  "name", "name", 155);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "resolved", "resolved", 155);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c6_rhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs155), "rhs", "rhs",
                  155);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs155), "lhs", "lhs",
                  155);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "context", "context", 156);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 156);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 156);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 156);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c6_rhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs156), "rhs", "rhs",
                  156);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs156), "lhs", "lhs",
                  156);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "context", "context", 157);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 157);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 157);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 157);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c6_rhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs157), "rhs", "rhs",
                  157);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs157), "lhs", "lhs",
                  157);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 158);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xswap"), "name", "name",
                  158);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"),
                  "resolved", "resolved", 158);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c6_rhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs158), "rhs", "rhs",
                  158);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs158), "lhs", "lhs",
                  158);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 159);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 159);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 159);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 159);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c6_rhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs159), "rhs", "rhs",
                  159);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs159), "lhs", "lhs",
                  159);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 160);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xswap"),
                  "name", "name", 160);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 160);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "resolved", "resolved", 160);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c6_rhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs160), "rhs", "rhs",
                  160);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs160), "lhs", "lhs",
                  160);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 161);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 161);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 161);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 161);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c6_rhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs161), "rhs", "rhs",
                  161);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs161), "lhs", "lhs",
                  161);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p!below_threshold"),
                  "context", "context", 162);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 162);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 162);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 162);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c6_rhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs162), "rhs", "rhs",
                  162);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs162), "lhs", "lhs",
                  162);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 163);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xswap"),
                  "name", "name", 163);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 163);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "resolved", "resolved", 163);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c6_rhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs163), "rhs", "rhs",
                  163);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs163), "lhs", "lhs",
                  163);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 164);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("abs"), "name", "name", 164);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 164);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 164);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c6_rhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs164), "rhs", "rhs",
                  164);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs164), "lhs", "lhs",
                  164);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 165);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 165);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 165);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 165);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c6_rhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs165), "rhs", "rhs",
                  165);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs165), "lhs", "lhs",
                  165);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 166);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 166);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 166);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 166);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c6_rhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs166), "rhs", "rhs",
                  166);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs166), "lhs", "lhs",
                  166);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 167);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 167);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 167);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 167);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 167);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 167);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 167);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 167);
  sf_mex_assign(&c6_rhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs167), "rhs", "rhs",
                  167);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs167), "lhs", "lhs",
                  167);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 168);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 168);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 168);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 168);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 168);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 168);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 168);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 168);
  sf_mex_assign(&c6_rhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs168), "rhs", "rhs",
                  168);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs168), "lhs", "lhs",
                  168);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 169);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eps"), "name", "name", 169);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 169);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 169);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 169);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 169);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 169);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 169);
  sf_mex_assign(&c6_rhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs169), "rhs", "rhs",
                  169);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs169), "lhs", "lhs",
                  169);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 170);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 170);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 170);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 170);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 170);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 170);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 170);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 170);
  sf_mex_assign(&c6_rhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs170), "rhs", "rhs",
                  170);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs170), "lhs", "lhs",
                  170);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 171);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 171);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 171);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 171);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 171);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 171);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 171);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 171);
  sf_mex_assign(&c6_rhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs171), "rhs", "rhs",
                  171);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs171), "lhs", "lhs",
                  171);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 172);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_div"), "name", "name", 172);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 172);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 172);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 172);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 172);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 172);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 172);
  sf_mex_assign(&c6_rhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs172), "rhs", "rhs",
                  172);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs172), "lhs", "lhs",
                  172);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 173);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xscal"), "name", "name",
                  173);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 173);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 173);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 173);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 173);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 173);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 173);
  sf_mex_assign(&c6_rhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs173), "rhs", "rhs",
                  173);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs173), "lhs", "lhs",
                  173);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 174);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 174);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 174);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 174);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 174);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 174);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 174);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 174);
  sf_mex_assign(&c6_rhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs174), "rhs", "rhs",
                  174);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs174), "lhs", "lhs",
                  174);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 175);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  175);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 175);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 175);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980690U), "fileTimeLo",
                  "fileTimeLo", 175);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 175);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 175);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 175);
  sf_mex_assign(&c6_rhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs175), "rhs", "rhs",
                  175);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs175), "lhs", "lhs",
                  175);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 176);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 176);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 176);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 176);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 176);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 176);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 176);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 176);
  sf_mex_assign(&c6_rhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs176), "rhs", "rhs",
                  176);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs176), "lhs", "lhs",
                  176);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 177);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 177);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 177);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 177);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 177);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 177);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 177);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 177);
  sf_mex_assign(&c6_rhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs177), "rhs", "rhs",
                  177);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs177), "lhs", "lhs",
                  177);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 178);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 178);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 178);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 178);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 178);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 178);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 178);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 178);
  sf_mex_assign(&c6_rhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs178), "rhs", "rhs",
                  178);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs178), "lhs", "lhs",
                  178);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 179);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 179);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 179);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 179);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 179);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 179);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 179);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 179);
  sf_mex_assign(&c6_rhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs179), "rhs", "rhs",
                  179);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs179), "lhs", "lhs",
                  179);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 180);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("min"), "name", "name", 180);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 180);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 180);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1311255318U), "fileTimeLo",
                  "fileTimeLo", 180);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 180);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 180);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 180);
  sf_mex_assign(&c6_rhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs180), "rhs", "rhs",
                  180);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs180), "lhs", "lhs",
                  180);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 181);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 181);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 181);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 181);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 181);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 181);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 181);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 181);
  sf_mex_assign(&c6_rhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs181), "rhs", "rhs",
                  181);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs181), "lhs", "lhs",
                  181);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 182);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 182);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 182);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 182);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 182);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 182);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 182);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 182);
  sf_mex_assign(&c6_rhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs182), "rhs", "rhs",
                  182);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs182), "lhs", "lhs",
                  182);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 183);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 183);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 183);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 183);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 183);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 183);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 183);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 183);
  sf_mex_assign(&c6_rhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs183), "rhs", "rhs",
                  183);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs183), "lhs", "lhs",
                  183);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 184);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 184);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 184);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 184);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 184);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 184);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 184);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 184);
  sf_mex_assign(&c6_rhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs184), "rhs", "rhs",
                  184);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs184), "lhs", "lhs",
                  184);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 185);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 185);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 185);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 185);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 185);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 185);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 185);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 185);
  sf_mex_assign(&c6_rhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs185), "rhs", "rhs",
                  185);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs185), "lhs", "lhs",
                  185);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 186);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 186);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 186);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 186);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 186);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 186);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 186);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 186);
  sf_mex_assign(&c6_rhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs186), "rhs", "rhs",
                  186);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs186), "lhs", "lhs",
                  186);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 187);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 187);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 187);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 187);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 187);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 187);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 187);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 187);
  sf_mex_assign(&c6_rhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs187), "rhs", "rhs",
                  187);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs187), "lhs", "lhs",
                  187);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 188);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 188);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 188);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 188);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 188);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 188);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 188);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 188);
  sf_mex_assign(&c6_rhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs188), "rhs", "rhs",
                  188);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs188), "lhs", "lhs",
                  188);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 189);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 189);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 189);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 189);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 189);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 189);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 189);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 189);
  sf_mex_assign(&c6_rhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs189), "rhs", "rhs",
                  189);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs189), "lhs", "lhs",
                  189);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!ceval_xgemm"),
                  "context", "context", 190);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.blas.size_ptr"),
                  "name", "name", 190);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 190);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/size_ptr.p"),
                  "resolved", "resolved", 190);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 190);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 190);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 190);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 190);
  sf_mex_assign(&c6_rhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs190), "rhs", "rhs",
                  190);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs190), "lhs", "lhs",
                  190);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!ceval_xgemm"),
                  "context", "context", 191);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("coder.internal.c_cast"),
                  "name", "name", 191);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("int32"), "dominantType",
                  "dominantType", 191);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/c_cast.p"),
                  "resolved", "resolved", 191);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 191);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 191);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 191);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 191);
  sf_mex_assign(&c6_rhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs191), "rhs", "rhs",
                  191);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs191), "lhs", "lhs",
                  191);
  sf_mex_destroy(&c6_rhs128);
  sf_mex_destroy(&c6_lhs128);
  sf_mex_destroy(&c6_rhs129);
  sf_mex_destroy(&c6_lhs129);
  sf_mex_destroy(&c6_rhs130);
  sf_mex_destroy(&c6_lhs130);
  sf_mex_destroy(&c6_rhs131);
  sf_mex_destroy(&c6_lhs131);
  sf_mex_destroy(&c6_rhs132);
  sf_mex_destroy(&c6_lhs132);
  sf_mex_destroy(&c6_rhs133);
  sf_mex_destroy(&c6_lhs133);
  sf_mex_destroy(&c6_rhs134);
  sf_mex_destroy(&c6_lhs134);
  sf_mex_destroy(&c6_rhs135);
  sf_mex_destroy(&c6_lhs135);
  sf_mex_destroy(&c6_rhs136);
  sf_mex_destroy(&c6_lhs136);
  sf_mex_destroy(&c6_rhs137);
  sf_mex_destroy(&c6_lhs137);
  sf_mex_destroy(&c6_rhs138);
  sf_mex_destroy(&c6_lhs138);
  sf_mex_destroy(&c6_rhs139);
  sf_mex_destroy(&c6_lhs139);
  sf_mex_destroy(&c6_rhs140);
  sf_mex_destroy(&c6_lhs140);
  sf_mex_destroy(&c6_rhs141);
  sf_mex_destroy(&c6_lhs141);
  sf_mex_destroy(&c6_rhs142);
  sf_mex_destroy(&c6_lhs142);
  sf_mex_destroy(&c6_rhs143);
  sf_mex_destroy(&c6_lhs143);
  sf_mex_destroy(&c6_rhs144);
  sf_mex_destroy(&c6_lhs144);
  sf_mex_destroy(&c6_rhs145);
  sf_mex_destroy(&c6_lhs145);
  sf_mex_destroy(&c6_rhs146);
  sf_mex_destroy(&c6_lhs146);
  sf_mex_destroy(&c6_rhs147);
  sf_mex_destroy(&c6_lhs147);
  sf_mex_destroy(&c6_rhs148);
  sf_mex_destroy(&c6_lhs148);
  sf_mex_destroy(&c6_rhs149);
  sf_mex_destroy(&c6_lhs149);
  sf_mex_destroy(&c6_rhs150);
  sf_mex_destroy(&c6_lhs150);
  sf_mex_destroy(&c6_rhs151);
  sf_mex_destroy(&c6_lhs151);
  sf_mex_destroy(&c6_rhs152);
  sf_mex_destroy(&c6_lhs152);
  sf_mex_destroy(&c6_rhs153);
  sf_mex_destroy(&c6_lhs153);
  sf_mex_destroy(&c6_rhs154);
  sf_mex_destroy(&c6_lhs154);
  sf_mex_destroy(&c6_rhs155);
  sf_mex_destroy(&c6_lhs155);
  sf_mex_destroy(&c6_rhs156);
  sf_mex_destroy(&c6_lhs156);
  sf_mex_destroy(&c6_rhs157);
  sf_mex_destroy(&c6_lhs157);
  sf_mex_destroy(&c6_rhs158);
  sf_mex_destroy(&c6_lhs158);
  sf_mex_destroy(&c6_rhs159);
  sf_mex_destroy(&c6_lhs159);
  sf_mex_destroy(&c6_rhs160);
  sf_mex_destroy(&c6_lhs160);
  sf_mex_destroy(&c6_rhs161);
  sf_mex_destroy(&c6_lhs161);
  sf_mex_destroy(&c6_rhs162);
  sf_mex_destroy(&c6_lhs162);
  sf_mex_destroy(&c6_rhs163);
  sf_mex_destroy(&c6_lhs163);
  sf_mex_destroy(&c6_rhs164);
  sf_mex_destroy(&c6_lhs164);
  sf_mex_destroy(&c6_rhs165);
  sf_mex_destroy(&c6_lhs165);
  sf_mex_destroy(&c6_rhs166);
  sf_mex_destroy(&c6_lhs166);
  sf_mex_destroy(&c6_rhs167);
  sf_mex_destroy(&c6_lhs167);
  sf_mex_destroy(&c6_rhs168);
  sf_mex_destroy(&c6_lhs168);
  sf_mex_destroy(&c6_rhs169);
  sf_mex_destroy(&c6_lhs169);
  sf_mex_destroy(&c6_rhs170);
  sf_mex_destroy(&c6_lhs170);
  sf_mex_destroy(&c6_rhs171);
  sf_mex_destroy(&c6_lhs171);
  sf_mex_destroy(&c6_rhs172);
  sf_mex_destroy(&c6_lhs172);
  sf_mex_destroy(&c6_rhs173);
  sf_mex_destroy(&c6_lhs173);
  sf_mex_destroy(&c6_rhs174);
  sf_mex_destroy(&c6_lhs174);
  sf_mex_destroy(&c6_rhs175);
  sf_mex_destroy(&c6_lhs175);
  sf_mex_destroy(&c6_rhs176);
  sf_mex_destroy(&c6_lhs176);
  sf_mex_destroy(&c6_rhs177);
  sf_mex_destroy(&c6_lhs177);
  sf_mex_destroy(&c6_rhs178);
  sf_mex_destroy(&c6_lhs178);
  sf_mex_destroy(&c6_rhs179);
  sf_mex_destroy(&c6_lhs179);
  sf_mex_destroy(&c6_rhs180);
  sf_mex_destroy(&c6_lhs180);
  sf_mex_destroy(&c6_rhs181);
  sf_mex_destroy(&c6_lhs181);
  sf_mex_destroy(&c6_rhs182);
  sf_mex_destroy(&c6_lhs182);
  sf_mex_destroy(&c6_rhs183);
  sf_mex_destroy(&c6_lhs183);
  sf_mex_destroy(&c6_rhs184);
  sf_mex_destroy(&c6_lhs184);
  sf_mex_destroy(&c6_rhs185);
  sf_mex_destroy(&c6_lhs185);
  sf_mex_destroy(&c6_rhs186);
  sf_mex_destroy(&c6_lhs186);
  sf_mex_destroy(&c6_rhs187);
  sf_mex_destroy(&c6_lhs187);
  sf_mex_destroy(&c6_rhs188);
  sf_mex_destroy(&c6_lhs188);
  sf_mex_destroy(&c6_rhs189);
  sf_mex_destroy(&c6_lhs189);
  sf_mex_destroy(&c6_rhs190);
  sf_mex_destroy(&c6_lhs190);
  sf_mex_destroy(&c6_rhs191);
  sf_mex_destroy(&c6_lhs191);
}

static void c6_d_info_helper(const mxArray **c6_info)
{
  const mxArray *c6_rhs192 = NULL;
  const mxArray *c6_lhs192 = NULL;
  const mxArray *c6_rhs193 = NULL;
  const mxArray *c6_lhs193 = NULL;
  const mxArray *c6_rhs194 = NULL;
  const mxArray *c6_lhs194 = NULL;
  const mxArray *c6_rhs195 = NULL;
  const mxArray *c6_lhs195 = NULL;
  const mxArray *c6_rhs196 = NULL;
  const mxArray *c6_lhs196 = NULL;
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 192);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 192);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 192);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 192);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1383877294U), "fileTimeLo",
                  "fileTimeLo", 192);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 192);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 192);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 192);
  sf_mex_assign(&c6_rhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs192), "rhs", "rhs",
                  192);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs192), "lhs", "lhs",
                  192);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 193);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 193);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 193);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 193);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 193);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 193);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 193);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 193);
  sf_mex_assign(&c6_rhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs193), "rhs", "rhs",
                  193);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs193), "lhs", "lhs",
                  193);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 194);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 194);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 194);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 194);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 194);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 194);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 194);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 194);
  sf_mex_assign(&c6_rhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs194), "rhs", "rhs",
                  194);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs194), "lhs", "lhs",
                  194);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 195);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 195);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 195);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 195);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 195);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 195);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 195);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 195);
  sf_mex_assign(&c6_rhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs195), "rhs", "rhs",
                  195);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs195), "lhs", "lhs",
                  195);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 196);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  196);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 196);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 196);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1375980690U), "fileTimeLo",
                  "fileTimeLo", 196);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 196);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 196);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 196);
  sf_mex_assign(&c6_rhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c6_lhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs196), "rhs", "rhs",
                  196);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs196), "lhs", "lhs",
                  196);
  sf_mex_destroy(&c6_rhs192);
  sf_mex_destroy(&c6_lhs192);
  sf_mex_destroy(&c6_rhs193);
  sf_mex_destroy(&c6_lhs193);
  sf_mex_destroy(&c6_rhs194);
  sf_mex_destroy(&c6_lhs194);
  sf_mex_destroy(&c6_rhs195);
  sf_mex_destroy(&c6_lhs195);
  sf_mex_destroy(&c6_rhs196);
  sf_mex_destroy(&c6_lhs196);
}

static void c6_pinv(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                    real_T c6_A[186], real_T c6_X[186])
{
  int32_T c6_i78;
  int32_T c6_i79;
  int32_T c6_i80;
  int32_T c6_i81;
  real_T c6_U[186];
  int32_T c6_i82;
  real_T c6_b_X[186];
  int32_T c6_i83;
  real_T c6_b_U[186];
  real_T c6_V[36];
  real_T c6_S[36];
  real_T c6_tol;
  int32_T c6_r;
  int32_T c6_k;
  int32_T c6_b_k;
  int32_T c6_a;
  int32_T c6_b_a;
  int32_T c6_vcol;
  int32_T c6_b_r;
  int32_T c6_b;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_j;
  int32_T c6_b_j;
  real_T c6_d1;
  int32_T c6_c_a;
  int32_T c6_d_a;
  int32_T c6_i84;
  real_T c6_b_V[36];
  int32_T c6_i85;
  real_T c6_c_U[186];
  int32_T c6_i86;
  int32_T c6_i87;
  int32_T c6_i88;
  int32_T c6_i89;
  boolean_T exitg1;
  c6_i78 = 0;
  for (c6_i79 = 0; c6_i79 < 6; c6_i79++) {
    c6_i80 = 0;
    for (c6_i81 = 0; c6_i81 < 31; c6_i81++) {
      c6_U[c6_i81 + c6_i78] = c6_A[c6_i80 + c6_i79];
      c6_i80 += 6;
    }

    c6_i78 += 31;
  }

  c6_eml_scalar_eg(chartInstance);
  for (c6_i82 = 0; c6_i82 < 186; c6_i82++) {
    c6_b_X[c6_i82] = 0.0;
  }

  for (c6_i83 = 0; c6_i83 < 186; c6_i83++) {
    c6_b_U[c6_i83] = c6_U[c6_i83];
  }

  c6_svd(chartInstance, c6_b_U, c6_U, c6_S, c6_V);
  c6_eps(chartInstance);
  c6_tol = 31.0 * c6_S[0] * 2.2204460492503131E-16;
  c6_r = 0;
  c6_k = 1;
  exitg1 = false;
  while ((exitg1 == false) && (c6_k < 7)) {
    c6_b_k = c6_k;
    if (!(c6_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 2, 0) - 1)) -
          1] > c6_tol)) {
      exitg1 = true;
    } else {
      c6_a = c6_r;
      c6_b_a = c6_a + 1;
      c6_r = c6_b_a;
      c6_k++;
    }
  }

  if (c6_r > 0) {
    c6_vcol = 1;
    c6_b_r = c6_r;
    c6_b = c6_b_r;
    c6_b_b = c6_b;
    if (1 > c6_b_b) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = (c6_b_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_j = 1; c6_j <= c6_b_r; c6_j++) {
      c6_b_j = c6_j;
      c6_d1 = c6_eml_div(chartInstance, 1.0, c6_S[(_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 6, 1, 0) + 6 *
        (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
        c6_b_j), 1, 6, 2, 0) - 1)) - 1]);
      c6_h_eml_xscal(chartInstance, c6_d1, c6_V, c6_vcol);
      c6_c_a = c6_vcol;
      c6_d_a = c6_c_a + 6;
      c6_vcol = c6_d_a;
    }

    for (c6_i84 = 0; c6_i84 < 36; c6_i84++) {
      c6_b_V[c6_i84] = c6_V[c6_i84];
    }

    for (c6_i85 = 0; c6_i85 < 186; c6_i85++) {
      c6_c_U[c6_i85] = c6_U[c6_i85];
    }

    c6_b_eml_xgemm(chartInstance, c6_r, c6_b_V, c6_c_U, c6_b_X);
  }

  c6_i86 = 0;
  for (c6_i87 = 0; c6_i87 < 6; c6_i87++) {
    c6_i88 = 0;
    for (c6_i89 = 0; c6_i89 < 31; c6_i89++) {
      c6_X[c6_i89 + c6_i86] = c6_b_X[c6_i88 + c6_i87];
      c6_i88 += 6;
    }

    c6_i86 += 31;
  }
}

static void c6_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_svd(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                   real_T c6_A[186], real_T c6_U[186], real_T c6_S[36], real_T
                   c6_V[36])
{
  int32_T c6_k;
  int32_T c6_b_k;
  real_T c6_x;
  real_T c6_b_x;
  boolean_T c6_b;
  boolean_T c6_b0;
  real_T c6_c_x;
  boolean_T c6_b_b;
  boolean_T c6_b1;
  boolean_T c6_c_b;
  int32_T c6_i90;
  real_T c6_b_A[186];
  real_T c6_s[6];
  int32_T c6_i91;
  int32_T c6_c_k;
  real_T c6_d_k;
  for (c6_k = 1; c6_k < 187; c6_k++) {
    c6_b_k = c6_k;
    c6_x = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_b_k), 1, 186, 1, 0) - 1];
    c6_b_x = c6_x;
    c6_b = muDoubleScalarIsInf(c6_b_x);
    c6_b0 = !c6_b;
    c6_c_x = c6_x;
    c6_b_b = muDoubleScalarIsNaN(c6_c_x);
    c6_b1 = !c6_b_b;
    c6_c_b = (c6_b0 && c6_b1);
    if (!c6_c_b) {
      c6_eml_error(chartInstance);
    }
  }

  for (c6_i90 = 0; c6_i90 < 186; c6_i90++) {
    c6_b_A[c6_i90] = c6_A[c6_i90];
  }

  c6_eml_xgesvd(chartInstance, c6_b_A, c6_U, c6_s, c6_V);
  for (c6_i91 = 0; c6_i91 < 36; c6_i91++) {
    c6_S[c6_i91] = 0.0;
  }

  for (c6_c_k = 0; c6_c_k < 6; c6_c_k++) {
    c6_d_k = 1.0 + (real_T)c6_c_k;
    c6_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c6_d_k),
           1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c6_d_k), 1, 6, 2, 0) - 1)) - 1] =
      c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c6_d_k), 1, 6, 1, 0) - 1];
  }
}

static void c6_eml_switch_helper(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_eml_error(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  int32_T c6_i92;
  static char_T c6_cv0[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'm', 'a', 't', 'r', 'i', 'x', 'W', 'i',
    't', 'h', 'N', 'a', 'N', 'I', 'n', 'f' };

  char_T c6_u[33];
  const mxArray *c6_y = NULL;
  (void)chartInstance;
  for (c6_i92 = 0; c6_i92 < 33; c6_i92++) {
    c6_u[c6_i92] = c6_cv0[c6_i92];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 33), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c6_y));
}

static void c6_eml_xgesvd(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_A[186], real_T c6_U[186], real_T c6_S[6], real_T
  c6_V[36])
{
  int32_T c6_i93;
  real_T c6_b_A[186];
  int32_T c6_i94;
  real_T c6_s[6];
  int32_T c6_i95;
  real_T c6_e[6];
  int32_T c6_i96;
  real_T c6_work[31];
  int32_T c6_i97;
  int32_T c6_i98;
  real_T c6_Vf[36];
  int32_T c6_q;
  int32_T c6_b_q;
  int32_T c6_a;
  int32_T c6_b_a;
  int32_T c6_qp1;
  int32_T c6_c_a;
  int32_T c6_d_a;
  int32_T c6_qm1;
  int32_T c6_b;
  int32_T c6_b_b;
  int32_T c6_c;
  int32_T c6_e_a;
  int32_T c6_c_b;
  int32_T c6_f_a;
  int32_T c6_d_b;
  int32_T c6_qq;
  int32_T c6_e_b;
  int32_T c6_f_b;
  int32_T c6_nmq;
  int32_T c6_g_a;
  int32_T c6_h_a;
  int32_T c6_nmqp1;
  int32_T c6_i99;
  real_T c6_c_A[186];
  real_T c6_nrm;
  real_T c6_absx;
  real_T c6_d;
  real_T c6_y;
  real_T c6_d2;
  int32_T c6_b_qp1;
  int32_T c6_i_a;
  int32_T c6_j_a;
  boolean_T c6_overflow;
  int32_T c6_jj;
  int32_T c6_b_jj;
  int32_T c6_k_a;
  int32_T c6_l_a;
  int32_T c6_b_c;
  int32_T c6_g_b;
  int32_T c6_h_b;
  int32_T c6_c_c;
  int32_T c6_m_a;
  int32_T c6_i_b;
  int32_T c6_n_a;
  int32_T c6_j_b;
  int32_T c6_qjj;
  int32_T c6_i100;
  real_T c6_d_A[186];
  int32_T c6_i101;
  real_T c6_e_A[186];
  real_T c6_t;
  int32_T c6_c_q;
  int32_T c6_o_a;
  int32_T c6_p_a;
  boolean_T c6_b_overflow;
  int32_T c6_ii;
  int32_T c6_b_ii;
  int32_T c6_k_b;
  int32_T c6_l_b;
  int32_T c6_pmq;
  int32_T c6_i102;
  real_T c6_b_e[6];
  real_T c6_b_absx;
  real_T c6_b_d;
  real_T c6_b_y;
  real_T c6_d3;
  int32_T c6_c_qp1;
  int32_T c6_q_a;
  int32_T c6_r_a;
  boolean_T c6_c_overflow;
  int32_T c6_c_ii;
  int32_T c6_d_qp1;
  int32_T c6_s_a;
  int32_T c6_t_a;
  boolean_T c6_d_overflow;
  int32_T c6_c_jj;
  int32_T c6_u_a;
  int32_T c6_v_a;
  int32_T c6_d_c;
  int32_T c6_m_b;
  int32_T c6_n_b;
  int32_T c6_e_c;
  int32_T c6_w_a;
  int32_T c6_o_b;
  int32_T c6_x_a;
  int32_T c6_p_b;
  int32_T c6_qp1jj;
  int32_T c6_i103;
  real_T c6_f_A[186];
  int32_T c6_e_qp1;
  int32_T c6_y_a;
  int32_T c6_ab_a;
  boolean_T c6_e_overflow;
  int32_T c6_d_jj;
  int32_T c6_bb_a;
  int32_T c6_cb_a;
  int32_T c6_f_c;
  int32_T c6_q_b;
  int32_T c6_r_b;
  int32_T c6_g_c;
  int32_T c6_db_a;
  int32_T c6_s_b;
  int32_T c6_eb_a;
  int32_T c6_t_b;
  int32_T c6_i104;
  real_T c6_b_work[31];
  int32_T c6_f_qp1;
  int32_T c6_fb_a;
  int32_T c6_gb_a;
  boolean_T c6_f_overflow;
  int32_T c6_d_ii;
  int32_T c6_m;
  int32_T c6_d_q;
  int32_T c6_hb_a;
  int32_T c6_ib_a;
  int32_T c6_u_b;
  int32_T c6_v_b;
  int32_T c6_jb_a;
  int32_T c6_kb_a;
  int32_T c6_lb_a;
  int32_T c6_mb_a;
  int32_T c6_h_c;
  int32_T c6_w_b;
  int32_T c6_x_b;
  int32_T c6_i_c;
  int32_T c6_nb_a;
  int32_T c6_y_b;
  int32_T c6_ob_a;
  int32_T c6_ab_b;
  int32_T c6_g_qp1;
  int32_T c6_pb_a;
  int32_T c6_qb_a;
  boolean_T c6_g_overflow;
  int32_T c6_e_jj;
  int32_T c6_rb_a;
  int32_T c6_sb_a;
  int32_T c6_j_c;
  int32_T c6_bb_b;
  int32_T c6_cb_b;
  int32_T c6_k_c;
  int32_T c6_tb_a;
  int32_T c6_db_b;
  int32_T c6_ub_a;
  int32_T c6_eb_b;
  int32_T c6_i105;
  real_T c6_b_U[186];
  int32_T c6_i106;
  real_T c6_c_U[186];
  int32_T c6_e_q;
  int32_T c6_vb_a;
  int32_T c6_wb_a;
  boolean_T c6_h_overflow;
  int32_T c6_e_ii;
  int32_T c6_xb_a;
  int32_T c6_yb_a;
  int32_T c6_i107;
  int32_T c6_fb_b;
  int32_T c6_gb_b;
  boolean_T c6_i_overflow;
  int32_T c6_f_ii;
  int32_T c6_g_ii;
  int32_T c6_f_q;
  int32_T c6_ac_a;
  int32_T c6_bc_a;
  int32_T c6_hb_b;
  int32_T c6_ib_b;
  int32_T c6_cc_a;
  int32_T c6_dc_a;
  int32_T c6_l_c;
  int32_T c6_jb_b;
  int32_T c6_kb_b;
  int32_T c6_m_c;
  int32_T c6_ec_a;
  int32_T c6_lb_b;
  int32_T c6_fc_a;
  int32_T c6_mb_b;
  int32_T c6_qp1q;
  int32_T c6_h_qp1;
  int32_T c6_gc_a;
  int32_T c6_hc_a;
  boolean_T c6_j_overflow;
  int32_T c6_f_jj;
  int32_T c6_ic_a;
  int32_T c6_jc_a;
  int32_T c6_n_c;
  int32_T c6_nb_b;
  int32_T c6_ob_b;
  int32_T c6_o_c;
  int32_T c6_kc_a;
  int32_T c6_pb_b;
  int32_T c6_lc_a;
  int32_T c6_qb_b;
  int32_T c6_i108;
  real_T c6_b_Vf[36];
  int32_T c6_i109;
  real_T c6_c_Vf[36];
  int32_T c6_h_ii;
  int32_T c6_g_q;
  real_T c6_rt;
  real_T c6_r;
  int32_T c6_mc_a;
  int32_T c6_nc_a;
  int32_T c6_p_c;
  int32_T c6_rb_b;
  int32_T c6_sb_b;
  int32_T c6_q_c;
  int32_T c6_tb_b;
  int32_T c6_ub_b;
  int32_T c6_colq;
  int32_T c6_oc_a;
  int32_T c6_pc_a;
  int32_T c6_r_c;
  int32_T c6_qc_a;
  int32_T c6_rc_a;
  int32_T c6_s_c;
  int32_T c6_vb_b;
  int32_T c6_wb_b;
  int32_T c6_t_c;
  int32_T c6_xb_b;
  int32_T c6_yb_b;
  int32_T c6_colqp1;
  real_T c6_iter;
  real_T c6_tiny;
  real_T c6_snorm;
  int32_T c6_i_ii;
  real_T c6_varargin_1;
  real_T c6_varargin_2;
  real_T c6_b_varargin_2;
  real_T c6_varargin_3;
  real_T c6_x;
  real_T c6_c_y;
  real_T c6_b_x;
  real_T c6_d_y;
  real_T c6_xk;
  real_T c6_yk;
  real_T c6_c_x;
  real_T c6_e_y;
  real_T c6_maxval;
  real_T c6_b_varargin_1;
  real_T c6_c_varargin_2;
  real_T c6_d_varargin_2;
  real_T c6_b_varargin_3;
  real_T c6_d_x;
  real_T c6_f_y;
  real_T c6_e_x;
  real_T c6_g_y;
  real_T c6_b_xk;
  real_T c6_b_yk;
  real_T c6_f_x;
  real_T c6_h_y;
  int32_T c6_sc_a;
  int32_T c6_tc_a;
  int32_T c6_uc_a;
  int32_T c6_vc_a;
  int32_T c6_i110;
  int32_T c6_wc_a;
  int32_T c6_xc_a;
  boolean_T c6_k_overflow;
  int32_T c6_j_ii;
  int32_T c6_yc_a;
  int32_T c6_ad_a;
  int32_T c6_u_c;
  real_T c6_test0;
  real_T c6_ztest0;
  int32_T c6_bd_a;
  int32_T c6_cd_a;
  int32_T c6_v_c;
  real_T c6_kase;
  int32_T c6_qs;
  int32_T c6_b_m;
  int32_T c6_h_q;
  int32_T c6_dd_a;
  int32_T c6_ac_b;
  int32_T c6_ed_a;
  int32_T c6_bc_b;
  boolean_T c6_l_overflow;
  int32_T c6_k_ii;
  real_T c6_test;
  int32_T c6_fd_a;
  int32_T c6_gd_a;
  int32_T c6_w_c;
  int32_T c6_hd_a;
  int32_T c6_id_a;
  int32_T c6_x_c;
  real_T c6_ztest;
  int32_T c6_jd_a;
  int32_T c6_kd_a;
  int32_T c6_ld_a;
  int32_T c6_md_a;
  int32_T c6_y_c;
  real_T c6_f;
  int32_T c6_nd_a;
  int32_T c6_od_a;
  int32_T c6_ab_c;
  int32_T c6_pd_a;
  int32_T c6_qd_a;
  int32_T c6_i111;
  int32_T c6_i_q;
  int32_T c6_rd_a;
  int32_T c6_cc_b;
  int32_T c6_sd_a;
  int32_T c6_dc_b;
  boolean_T c6_m_overflow;
  int32_T c6_k;
  int32_T c6_b_k;
  real_T c6_t1;
  real_T c6_b_t1;
  real_T c6_b_f;
  real_T c6_sn;
  real_T c6_cs;
  real_T c6_b_cs;
  real_T c6_b_sn;
  int32_T c6_td_a;
  int32_T c6_ud_a;
  int32_T c6_km1;
  int32_T c6_vd_a;
  int32_T c6_wd_a;
  int32_T c6_bb_c;
  int32_T c6_ec_b;
  int32_T c6_fc_b;
  int32_T c6_cb_c;
  int32_T c6_gc_b;
  int32_T c6_hc_b;
  int32_T c6_colk;
  int32_T c6_xd_a;
  int32_T c6_yd_a;
  int32_T c6_db_c;
  int32_T c6_ic_b;
  int32_T c6_jc_b;
  int32_T c6_eb_c;
  int32_T c6_kc_b;
  int32_T c6_lc_b;
  int32_T c6_colm;
  int32_T c6_ae_a;
  int32_T c6_be_a;
  int32_T c6_j_q;
  int32_T c6_c_m;
  int32_T c6_ce_a;
  int32_T c6_mc_b;
  int32_T c6_de_a;
  int32_T c6_nc_b;
  boolean_T c6_n_overflow;
  int32_T c6_c_k;
  real_T c6_c_t1;
  real_T c6_unusedU0;
  real_T c6_c_sn;
  real_T c6_c_cs;
  int32_T c6_ee_a;
  int32_T c6_fe_a;
  int32_T c6_fb_c;
  int32_T c6_oc_b;
  int32_T c6_pc_b;
  int32_T c6_gb_c;
  int32_T c6_qc_b;
  int32_T c6_rc_b;
  int32_T c6_ge_a;
  int32_T c6_he_a;
  int32_T c6_hb_c;
  int32_T c6_sc_b;
  int32_T c6_tc_b;
  int32_T c6_ib_c;
  int32_T c6_uc_b;
  int32_T c6_vc_b;
  int32_T c6_colqm1;
  int32_T c6_ie_a;
  int32_T c6_je_a;
  int32_T c6_mm1;
  real_T c6_d4;
  real_T c6_d5;
  real_T c6_d6;
  real_T c6_d7;
  real_T c6_d8;
  real_T c6_c_varargin_1[5];
  int32_T c6_ixstart;
  real_T c6_mtmp;
  real_T c6_g_x;
  boolean_T c6_wc_b;
  int32_T c6_ix;
  int32_T c6_b_ix;
  real_T c6_h_x;
  boolean_T c6_xc_b;
  int32_T c6_ke_a;
  int32_T c6_le_a;
  int32_T c6_i112;
  int32_T c6_me_a;
  int32_T c6_ne_a;
  boolean_T c6_o_overflow;
  int32_T c6_c_ix;
  real_T c6_oe_a;
  real_T c6_yc_b;
  boolean_T c6_p;
  real_T c6_b_mtmp;
  real_T c6_scale;
  real_T c6_sm;
  real_T c6_smm1;
  real_T c6_emm1;
  real_T c6_sqds;
  real_T c6_eqds;
  real_T c6_ad_b;
  real_T c6_jb_c;
  real_T c6_shift;
  real_T c6_g;
  int32_T c6_k_q;
  int32_T c6_b_mm1;
  int32_T c6_pe_a;
  int32_T c6_bd_b;
  int32_T c6_qe_a;
  int32_T c6_cd_b;
  boolean_T c6_p_overflow;
  int32_T c6_d_k;
  int32_T c6_re_a;
  int32_T c6_se_a;
  int32_T c6_te_a;
  int32_T c6_ue_a;
  int32_T c6_kp1;
  real_T c6_c_f;
  real_T c6_unusedU1;
  real_T c6_d_sn;
  real_T c6_d_cs;
  int32_T c6_ve_a;
  int32_T c6_we_a;
  int32_T c6_kb_c;
  int32_T c6_dd_b;
  int32_T c6_ed_b;
  int32_T c6_lb_c;
  int32_T c6_fd_b;
  int32_T c6_gd_b;
  int32_T c6_hd_b;
  int32_T c6_id_b;
  int32_T c6_mb_c;
  int32_T c6_jd_b;
  int32_T c6_kd_b;
  int32_T c6_colkp1;
  real_T c6_d_f;
  real_T c6_unusedU2;
  real_T c6_e_sn;
  real_T c6_e_cs;
  int32_T c6_xe_a;
  int32_T c6_ye_a;
  int32_T c6_nb_c;
  int32_T c6_ld_b;
  int32_T c6_md_b;
  int32_T c6_ob_c;
  int32_T c6_nd_b;
  int32_T c6_od_b;
  int32_T c6_pd_b;
  int32_T c6_qd_b;
  int32_T c6_pb_c;
  int32_T c6_rd_b;
  int32_T c6_sd_b;
  int32_T c6_af_a;
  int32_T c6_bf_a;
  int32_T c6_qb_c;
  int32_T c6_e_k;
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_i;
  int32_T c6_b_i;
  int32_T c6_rb_c;
  int32_T c6_cf_a;
  int32_T c6_sb_c;
  int32_T c6_td_b;
  int32_T c6_ud_b;
  int32_T c6_df_a;
  int32_T c6_ef_a;
  int32_T c6_tb_c;
  int32_T c6_ff_a;
  int32_T c6_ub_c;
  int32_T c6_vd_b;
  int32_T c6_wd_b;
  int32_T c6_vb_c;
  int32_T c6_xd_b;
  int32_T c6_yd_b;
  int32_T c6_wb_c;
  int32_T c6_gf_a;
  int32_T c6_xb_c;
  int32_T c6_ae_b;
  int32_T c6_be_b;
  int32_T c6_yb_c;
  int32_T c6_ce_b;
  int32_T c6_de_b;
  int32_T c6_hf_a;
  int32_T c6_if_a;
  int32_T c6_ee_b;
  int32_T c6_fe_b;
  int32_T c6_jf_a;
  int32_T c6_kf_a;
  int32_T c6_lf_a;
  int32_T c6_ge_b;
  int32_T c6_he_b;
  int32_T c6_ie_b;
  int32_T c6_je_b;
  int32_T c6_mf_a;
  int32_T c6_ke_b;
  int32_T c6_le_b;
  int32_T c6_me_b;
  int32_T c6_ne_b;
  int32_T c6_nf_a;
  real_T c6_d9;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T exitg5;
  boolean_T guard11 = false;
  for (c6_i93 = 0; c6_i93 < 186; c6_i93++) {
    c6_b_A[c6_i93] = c6_A[c6_i93];
  }

  c6_eml_scalar_eg(chartInstance);
  for (c6_i94 = 0; c6_i94 < 6; c6_i94++) {
    c6_s[c6_i94] = 0.0;
  }

  for (c6_i95 = 0; c6_i95 < 6; c6_i95++) {
    c6_e[c6_i95] = 0.0;
  }

  for (c6_i96 = 0; c6_i96 < 31; c6_i96++) {
    c6_work[c6_i96] = 0.0;
  }

  for (c6_i97 = 0; c6_i97 < 186; c6_i97++) {
    c6_U[c6_i97] = 0.0;
  }

  for (c6_i98 = 0; c6_i98 < 36; c6_i98++) {
    c6_Vf[c6_i98] = 0.0;
  }

  for (c6_q = 1; c6_q < 7; c6_q++) {
    c6_b_q = c6_q;
    c6_a = c6_b_q;
    c6_b_a = c6_a + 1;
    c6_qp1 = c6_b_a;
    c6_c_a = c6_b_q;
    c6_d_a = c6_c_a;
    c6_qm1 = c6_d_a;
    c6_b = c6_qm1 - 1;
    c6_b_b = c6_b;
    c6_c = 31 * c6_b_b;
    c6_e_a = c6_b_q;
    c6_c_b = c6_c;
    c6_f_a = c6_e_a;
    c6_d_b = c6_c_b;
    c6_qq = c6_f_a + c6_d_b;
    c6_e_b = c6_b_q;
    c6_f_b = c6_e_b;
    c6_nmq = 31 - c6_f_b;
    c6_g_a = c6_nmq;
    c6_h_a = c6_g_a + 1;
    c6_nmqp1 = c6_h_a;
    if (c6_b_q <= 6) {
      for (c6_i99 = 0; c6_i99 < 186; c6_i99++) {
        c6_c_A[c6_i99] = c6_b_A[c6_i99];
      }

      c6_nrm = c6_eml_xnrm2(chartInstance, c6_nmqp1, c6_c_A, c6_qq);
      if (c6_nrm > 0.0) {
        c6_absx = c6_nrm;
        c6_d = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_qq), 1, 186, 1, 0) - 1];
        if (c6_d < 0.0) {
          c6_y = -c6_absx;
        } else {
          c6_y = c6_absx;
        }

        c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1] = c6_y;
        c6_d2 = c6_eml_div(chartInstance, 1.0, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1]);
        c6_e_eml_xscal(chartInstance, c6_nmqp1, c6_d2, c6_b_A, c6_qq);
        c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_qq), 1, 186, 1, 0) - 1] =
          c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_qq), 1, 186, 1, 0) - 1] + 1.0;
        c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1] = -c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1];
      } else {
        c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1] = 0.0;
      }
    }

    c6_b_qp1 = c6_qp1;
    c6_i_a = c6_b_qp1;
    c6_j_a = c6_i_a;
    if (c6_j_a > 6) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = false;
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_jj = c6_b_qp1; c6_jj < 7; c6_jj++) {
      c6_b_jj = c6_jj;
      c6_k_a = c6_b_jj;
      c6_l_a = c6_k_a;
      c6_b_c = c6_l_a;
      c6_g_b = c6_b_c - 1;
      c6_h_b = c6_g_b;
      c6_c_c = 31 * c6_h_b;
      c6_m_a = c6_b_q;
      c6_i_b = c6_c_c;
      c6_n_a = c6_m_a;
      c6_j_b = c6_i_b;
      c6_qjj = c6_n_a + c6_j_b;
      if (c6_b_q <= 6) {
        if (c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_b_q), 1, 6, 1, 0) - 1] != 0.0) {
          for (c6_i100 = 0; c6_i100 < 186; c6_i100++) {
            c6_d_A[c6_i100] = c6_b_A[c6_i100];
          }

          for (c6_i101 = 0; c6_i101 < 186; c6_i101++) {
            c6_e_A[c6_i101] = c6_b_A[c6_i101];
          }

          c6_t = c6_eml_xdotc(chartInstance, c6_nmqp1, c6_d_A, c6_qq, c6_e_A,
                              c6_qjj);
          c6_t = -c6_eml_div(chartInstance, c6_t, c6_b_A
                             [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 31, 1, 0) + 31 *
                               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 2, 0) - 1)) - 1]);
          c6_e_eml_xaxpy(chartInstance, c6_nmqp1, c6_t, c6_qq, c6_b_A, c6_qjj);
        }
      }

      c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c6_b_jj), 1, 6, 1, 0) - 1] = c6_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_qjj), 1, 186, 1, 0) - 1];
    }

    if (c6_b_q <= 6) {
      c6_c_q = c6_b_q;
      c6_o_a = c6_c_q;
      c6_p_a = c6_o_a;
      if (c6_p_a > 31) {
        c6_b_overflow = false;
      } else {
        c6_eml_switch_helper(chartInstance);
        c6_b_overflow = false;
      }

      if (c6_b_overflow) {
        c6_check_forloop_overflow_error(chartInstance, c6_b_overflow);
      }

      for (c6_ii = c6_c_q; c6_ii < 32; c6_ii++) {
        c6_b_ii = c6_ii;
        c6_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c6_b_ii), 1, 31, 1, 0) + 31 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c6_b_q), 1, 6, 2, 0) - 1)) - 1] = c6_b_A
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_b_ii), 1, 31, 1, 0) + 31 * (_SFD_EML_ARRAY_BOUNDS_CHECK
             ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 2, 0) -
             1)) - 1];
      }
    }

    if (c6_b_q <= 4) {
      c6_k_b = c6_b_q;
      c6_l_b = c6_k_b;
      c6_pmq = 6 - c6_l_b;
      for (c6_i102 = 0; c6_i102 < 6; c6_i102++) {
        c6_b_e[c6_i102] = c6_e[c6_i102];
      }

      c6_nrm = c6_b_eml_xnrm2(chartInstance, c6_pmq, c6_b_e, c6_qp1);
      if (c6_nrm == 0.0) {
        c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1] = 0.0;
      } else {
        c6_b_absx = c6_nrm;
        c6_b_d = c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_qp1), 1, 6, 1, 0) - 1];
        if (c6_b_d < 0.0) {
          c6_b_y = -c6_b_absx;
        } else {
          c6_b_y = c6_b_absx;
        }

        c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1] = c6_b_y;
        c6_d3 = c6_eml_div(chartInstance, 1.0, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1]);
        c6_f_eml_xscal(chartInstance, c6_pmq, c6_d3, c6_e, c6_qp1);
        c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_qp1), 1, 6, 1, 0) - 1] = c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_qp1), 1, 6, 1, 0) - 1]
          + 1.0;
      }

      c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c6_b_q), 1, 6, 1, 0) - 1] = -c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1];
      if (c6_qp1 <= 31) {
        if (c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_b_q), 1, 6, 1, 0) - 1] != 0.0) {
          c6_c_qp1 = c6_qp1;
          c6_q_a = c6_c_qp1;
          c6_r_a = c6_q_a;
          if (c6_r_a > 31) {
            c6_c_overflow = false;
          } else {
            c6_eml_switch_helper(chartInstance);
            c6_c_overflow = false;
          }

          if (c6_c_overflow) {
            c6_check_forloop_overflow_error(chartInstance, c6_c_overflow);
          }

          for (c6_c_ii = c6_c_qp1; c6_c_ii < 32; c6_c_ii++) {
            c6_b_ii = c6_c_ii;
            c6_work[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c6_b_ii), 1, 31, 1, 0) - 1] = 0.0;
          }

          c6_d_qp1 = c6_qp1;
          c6_s_a = c6_d_qp1;
          c6_t_a = c6_s_a;
          if (c6_t_a > 6) {
            c6_d_overflow = false;
          } else {
            c6_eml_switch_helper(chartInstance);
            c6_d_overflow = false;
          }

          if (c6_d_overflow) {
            c6_check_forloop_overflow_error(chartInstance, c6_d_overflow);
          }

          for (c6_c_jj = c6_d_qp1; c6_c_jj < 7; c6_c_jj++) {
            c6_b_jj = c6_c_jj;
            c6_u_a = c6_b_jj;
            c6_v_a = c6_u_a;
            c6_d_c = c6_v_a;
            c6_m_b = c6_d_c - 1;
            c6_n_b = c6_m_b;
            c6_e_c = 31 * c6_n_b;
            c6_w_a = c6_qp1;
            c6_o_b = c6_e_c;
            c6_x_a = c6_w_a;
            c6_p_b = c6_o_b;
            c6_qp1jj = c6_x_a + c6_p_b;
            for (c6_i103 = 0; c6_i103 < 186; c6_i103++) {
              c6_f_A[c6_i103] = c6_b_A[c6_i103];
            }

            c6_f_eml_xaxpy(chartInstance, c6_nmq,
                           c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c6_b_jj), 1, 6, 1, 0) - 1], c6_f_A,
                           c6_qp1jj, c6_work, c6_qp1);
          }

          c6_e_qp1 = c6_qp1;
          c6_y_a = c6_e_qp1;
          c6_ab_a = c6_y_a;
          if (c6_ab_a > 6) {
            c6_e_overflow = false;
          } else {
            c6_eml_switch_helper(chartInstance);
            c6_e_overflow = false;
          }

          if (c6_e_overflow) {
            c6_check_forloop_overflow_error(chartInstance, c6_e_overflow);
          }

          for (c6_d_jj = c6_e_qp1; c6_d_jj < 7; c6_d_jj++) {
            c6_b_jj = c6_d_jj;
            c6_t = c6_eml_div(chartInstance, -c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_jj), 1, 6, 1, 0)
                              - 1], c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_qp1), 1, 6, 1, 0) - 1]);
            c6_bb_a = c6_b_jj;
            c6_cb_a = c6_bb_a;
            c6_f_c = c6_cb_a;
            c6_q_b = c6_f_c - 1;
            c6_r_b = c6_q_b;
            c6_g_c = 31 * c6_r_b;
            c6_db_a = c6_qp1;
            c6_s_b = c6_g_c;
            c6_eb_a = c6_db_a;
            c6_t_b = c6_s_b;
            c6_qp1jj = c6_eb_a + c6_t_b;
            for (c6_i104 = 0; c6_i104 < 31; c6_i104++) {
              c6_b_work[c6_i104] = c6_work[c6_i104];
            }

            c6_g_eml_xaxpy(chartInstance, c6_nmq, c6_t, c6_b_work, c6_qp1,
                           c6_b_A, c6_qp1jj);
          }
        }
      }

      c6_f_qp1 = c6_qp1;
      c6_fb_a = c6_f_qp1;
      c6_gb_a = c6_fb_a;
      if (c6_gb_a > 6) {
        c6_f_overflow = false;
      } else {
        c6_eml_switch_helper(chartInstance);
        c6_f_overflow = false;
      }

      if (c6_f_overflow) {
        c6_check_forloop_overflow_error(chartInstance, c6_f_overflow);
      }

      for (c6_d_ii = c6_f_qp1; c6_d_ii < 7; c6_d_ii++) {
        c6_b_ii = c6_d_ii;
        c6_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c6_b_ii), 1, 6, 1, 0) + 6 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c6_b_q), 1, 6, 2, 0) - 1)) - 1] =
          c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_ii), 1, 6, 1, 0) - 1];
      }
    }
  }

  c6_m = 6;
  c6_e[4] = c6_b_A[159];
  c6_e[5] = 0.0;
  for (c6_d_q = 6; c6_d_q > 0; c6_d_q--) {
    c6_b_q = c6_d_q;
    c6_hb_a = c6_b_q;
    c6_ib_a = c6_hb_a;
    c6_qp1 = c6_ib_a;
    c6_u_b = c6_b_q;
    c6_v_b = c6_u_b;
    c6_nmq = 31 - c6_v_b;
    c6_jb_a = c6_nmq;
    c6_kb_a = c6_jb_a + 1;
    c6_nmqp1 = c6_kb_a;
    c6_lb_a = c6_b_q;
    c6_mb_a = c6_lb_a;
    c6_h_c = c6_mb_a;
    c6_w_b = c6_h_c - 1;
    c6_x_b = c6_w_b;
    c6_i_c = 31 * c6_x_b;
    c6_nb_a = c6_b_q;
    c6_y_b = c6_i_c;
    c6_ob_a = c6_nb_a;
    c6_ab_b = c6_y_b;
    c6_qq = c6_ob_a + c6_ab_b;
    if (c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1] != 0.0) {
      c6_g_qp1 = c6_qp1 + 1;
      c6_pb_a = c6_g_qp1;
      c6_qb_a = c6_pb_a;
      if (c6_qb_a > 6) {
        c6_g_overflow = false;
      } else {
        c6_eml_switch_helper(chartInstance);
        c6_g_overflow = false;
      }

      if (c6_g_overflow) {
        c6_check_forloop_overflow_error(chartInstance, c6_g_overflow);
      }

      for (c6_e_jj = c6_g_qp1; c6_e_jj < 7; c6_e_jj++) {
        c6_b_jj = c6_e_jj;
        c6_rb_a = c6_b_jj;
        c6_sb_a = c6_rb_a;
        c6_j_c = c6_sb_a;
        c6_bb_b = c6_j_c - 1;
        c6_cb_b = c6_bb_b;
        c6_k_c = 31 * c6_cb_b;
        c6_tb_a = c6_b_q;
        c6_db_b = c6_k_c;
        c6_ub_a = c6_tb_a;
        c6_eb_b = c6_db_b;
        c6_qjj = c6_ub_a + c6_eb_b;
        for (c6_i105 = 0; c6_i105 < 186; c6_i105++) {
          c6_b_U[c6_i105] = c6_U[c6_i105];
        }

        for (c6_i106 = 0; c6_i106 < 186; c6_i106++) {
          c6_c_U[c6_i106] = c6_U[c6_i106];
        }

        c6_t = c6_eml_xdotc(chartInstance, c6_nmqp1, c6_b_U, c6_qq, c6_c_U,
                            c6_qjj);
        c6_t = -c6_eml_div(chartInstance, c6_t, c6_U[_SFD_EML_ARRAY_BOUNDS_CHECK
                           ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_qq),
                            1, 186, 1, 0) - 1]);
        c6_e_eml_xaxpy(chartInstance, c6_nmqp1, c6_t, c6_qq, c6_U, c6_qjj);
      }

      c6_e_q = c6_b_q;
      c6_vb_a = c6_e_q;
      c6_wb_a = c6_vb_a;
      if (c6_wb_a > 31) {
        c6_h_overflow = false;
      } else {
        c6_eml_switch_helper(chartInstance);
        c6_h_overflow = false;
      }

      if (c6_h_overflow) {
        c6_check_forloop_overflow_error(chartInstance, c6_h_overflow);
      }

      for (c6_e_ii = c6_e_q; c6_e_ii < 32; c6_e_ii++) {
        c6_b_ii = c6_e_ii;
        c6_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c6_b_ii), 1, 31, 1, 0) + 31 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c6_b_q), 1, 6, 2, 0) - 1)) - 1] = -c6_U
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_b_ii), 1, 31, 1, 0) + 31 * (_SFD_EML_ARRAY_BOUNDS_CHECK
             ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 2, 0) -
             1)) - 1];
      }

      c6_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c6_qq), 1, 186, 1, 0) - 1] = c6_U[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_qq), 1, 186, 1, 0) - 1] + 1.0;
      c6_xb_a = c6_b_q;
      c6_yb_a = c6_xb_a - 1;
      c6_i107 = c6_yb_a;
      c6_fb_b = c6_i107;
      c6_gb_b = c6_fb_b;
      if (1 > c6_gb_b) {
        c6_i_overflow = false;
      } else {
        c6_eml_switch_helper(chartInstance);
        c6_i_overflow = (c6_gb_b > 2147483646);
      }

      if (c6_i_overflow) {
        c6_check_forloop_overflow_error(chartInstance, c6_i_overflow);
      }

      for (c6_f_ii = 1; c6_f_ii <= c6_i107; c6_f_ii++) {
        c6_b_ii = c6_f_ii;
        c6_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c6_b_ii), 1, 31, 1, 0) + 31 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c6_b_q), 1, 6, 2, 0) - 1)) - 1] = 0.0;
      }
    } else {
      for (c6_g_ii = 1; c6_g_ii < 32; c6_g_ii++) {
        c6_b_ii = c6_g_ii;
        c6_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c6_b_ii), 1, 31, 1, 0) + 31 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c6_b_q), 1, 6, 2, 0) - 1)) - 1] = 0.0;
      }

      c6_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c6_qq), 1, 186, 1, 0) - 1] = 1.0;
    }
  }

  for (c6_f_q = 6; c6_f_q > 0; c6_f_q--) {
    c6_b_q = c6_f_q;
    if (c6_b_q <= 4) {
      if (c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_q), 1, 6, 1, 0) - 1] != 0.0) {
        c6_ac_a = c6_b_q;
        c6_bc_a = c6_ac_a + 1;
        c6_qp1 = c6_bc_a;
        c6_hb_b = c6_b_q;
        c6_ib_b = c6_hb_b;
        c6_pmq = 6 - c6_ib_b;
        c6_cc_a = c6_b_q;
        c6_dc_a = c6_cc_a;
        c6_l_c = c6_dc_a;
        c6_jb_b = c6_l_c - 1;
        c6_kb_b = c6_jb_b;
        c6_m_c = 6 * c6_kb_b;
        c6_ec_a = c6_qp1;
        c6_lb_b = c6_m_c;
        c6_fc_a = c6_ec_a;
        c6_mb_b = c6_lb_b;
        c6_qp1q = c6_fc_a + c6_mb_b;
        c6_h_qp1 = c6_qp1;
        c6_gc_a = c6_h_qp1;
        c6_hc_a = c6_gc_a;
        if (c6_hc_a > 6) {
          c6_j_overflow = false;
        } else {
          c6_eml_switch_helper(chartInstance);
          c6_j_overflow = false;
        }

        if (c6_j_overflow) {
          c6_check_forloop_overflow_error(chartInstance, c6_j_overflow);
        }

        for (c6_f_jj = c6_h_qp1; c6_f_jj < 7; c6_f_jj++) {
          c6_b_jj = c6_f_jj;
          c6_ic_a = c6_b_jj;
          c6_jc_a = c6_ic_a;
          c6_n_c = c6_jc_a;
          c6_nb_b = c6_n_c - 1;
          c6_ob_b = c6_nb_b;
          c6_o_c = 6 * c6_ob_b;
          c6_kc_a = c6_qp1;
          c6_pb_b = c6_o_c;
          c6_lc_a = c6_kc_a;
          c6_qb_b = c6_pb_b;
          c6_qp1jj = c6_lc_a + c6_qb_b;
          for (c6_i108 = 0; c6_i108 < 36; c6_i108++) {
            c6_b_Vf[c6_i108] = c6_Vf[c6_i108];
          }

          for (c6_i109 = 0; c6_i109 < 36; c6_i109++) {
            c6_c_Vf[c6_i109] = c6_Vf[c6_i109];
          }

          c6_t = c6_b_eml_xdotc(chartInstance, c6_pmq, c6_b_Vf, c6_qp1q, c6_c_Vf,
                                c6_qp1jj);
          c6_t = -c6_eml_div(chartInstance, c6_t,
                             c6_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_qp1q), 1, 36, 1, 0) - 1]);
          c6_h_eml_xaxpy(chartInstance, c6_pmq, c6_t, c6_qp1q, c6_Vf, c6_qp1jj);
        }
      }
    }

    for (c6_h_ii = 1; c6_h_ii < 7; c6_h_ii++) {
      c6_b_ii = c6_h_ii;
      c6_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c6_b_ii), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 2, 0)
              - 1)) - 1] = 0.0;
    }

    c6_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c6_b_q), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 2, 0) - 1))
      - 1] = 1.0;
  }

  for (c6_g_q = 1; c6_g_q < 7; c6_g_q++) {
    c6_b_q = c6_g_q;
    if (c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1] != 0.0) {
      c6_rt = c6_abs(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1]);
      c6_r = c6_eml_div(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1], c6_rt);
      c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c6_b_q), 1, 6, 1, 0) - 1] = c6_rt;
      if (c6_b_q < 6) {
        c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1] = c6_eml_div(chartInstance,
          c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1], c6_r);
      }

      if (c6_b_q <= 31) {
        c6_mc_a = c6_b_q;
        c6_nc_a = c6_mc_a;
        c6_p_c = c6_nc_a;
        c6_rb_b = c6_p_c - 1;
        c6_sb_b = c6_rb_b;
        c6_q_c = 31 * c6_sb_b;
        c6_tb_b = c6_q_c;
        c6_ub_b = c6_tb_b;
        c6_colq = c6_ub_b;
        c6_g_eml_xscal(chartInstance, c6_r, c6_U, c6_colq + 1);
      }
    }

    if (c6_b_q < 6) {
      if (c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_q), 1, 6, 1, 0) - 1] != 0.0) {
        c6_rt = c6_abs(chartInstance, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1]);
        c6_r = c6_eml_div(chartInstance, c6_rt, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK
                          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q),
                           1, 6, 1, 0) - 1]);
        c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_q), 1, 6, 1, 0) - 1] = c6_rt;
        c6_oc_a = c6_b_q;
        c6_pc_a = c6_oc_a;
        c6_r_c = c6_pc_a;
        c6_qc_a = c6_b_q;
        c6_rc_a = c6_qc_a;
        c6_s_c = c6_rc_a;
        c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c6_r_c + 1)), 1, 6, 1, 0) - 1] =
          c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c6_s_c + 1)), 1, 6, 1, 0) - 1] * c6_r;
        c6_vb_b = c6_b_q;
        c6_wb_b = c6_vb_b;
        c6_t_c = 6 * c6_wb_b;
        c6_xb_b = c6_t_c;
        c6_yb_b = c6_xb_b;
        c6_colqp1 = c6_yb_b;
        c6_h_eml_xscal(chartInstance, c6_r, c6_Vf, c6_colqp1 + 1);
      }
    }
  }

  c6_iter = 0.0;
  c6_realmin(chartInstance);
  c6_eps(chartInstance);
  c6_tiny = c6_eml_div(chartInstance, 2.2250738585072014E-308,
                       2.2204460492503131E-16);
  c6_snorm = 0.0;
  for (c6_i_ii = 1; c6_i_ii < 7; c6_i_ii++) {
    c6_b_ii = c6_i_ii;
    c6_varargin_1 = c6_abs(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_ii), 1, 6, 1, 0) - 1]);
    c6_varargin_2 = c6_abs(chartInstance, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_ii), 1, 6, 1, 0) - 1]);
    c6_b_varargin_2 = c6_varargin_1;
    c6_varargin_3 = c6_varargin_2;
    c6_x = c6_b_varargin_2;
    c6_c_y = c6_varargin_3;
    c6_b_x = c6_x;
    c6_d_y = c6_c_y;
    c6_b_eml_scalar_eg(chartInstance);
    c6_xk = c6_b_x;
    c6_yk = c6_d_y;
    c6_c_x = c6_xk;
    c6_e_y = c6_yk;
    c6_b_eml_scalar_eg(chartInstance);
    c6_maxval = muDoubleScalarMax(c6_c_x, c6_e_y);
    c6_b_varargin_1 = c6_snorm;
    c6_c_varargin_2 = c6_maxval;
    c6_d_varargin_2 = c6_b_varargin_1;
    c6_b_varargin_3 = c6_c_varargin_2;
    c6_d_x = c6_d_varargin_2;
    c6_f_y = c6_b_varargin_3;
    c6_e_x = c6_d_x;
    c6_g_y = c6_f_y;
    c6_b_eml_scalar_eg(chartInstance);
    c6_b_xk = c6_e_x;
    c6_b_yk = c6_g_y;
    c6_f_x = c6_b_xk;
    c6_h_y = c6_b_yk;
    c6_b_eml_scalar_eg(chartInstance);
    c6_snorm = muDoubleScalarMax(c6_f_x, c6_h_y);
  }

  exitg1 = false;
  while ((exitg1 == false) && (c6_m > 0)) {
    if (c6_iter >= 75.0) {
      c6_b_eml_error(chartInstance);
      exitg1 = true;
    } else {
      c6_sc_a = c6_m;
      c6_tc_a = c6_sc_a - 1;
      c6_b_q = c6_tc_a;
      c6_uc_a = c6_m;
      c6_vc_a = c6_uc_a - 1;
      c6_i110 = c6_vc_a;
      c6_wc_a = c6_i110;
      c6_xc_a = c6_wc_a;
      if (c6_xc_a < 0) {
        c6_k_overflow = false;
      } else {
        c6_eml_switch_helper(chartInstance);
        c6_k_overflow = false;
      }

      if (c6_k_overflow) {
        c6_check_forloop_overflow_error(chartInstance, c6_k_overflow);
      }

      c6_j_ii = c6_i110;
      guard3 = false;
      guard4 = false;
      exitg5 = false;
      while ((exitg5 == false) && (c6_j_ii > -1)) {
        c6_b_ii = c6_j_ii;
        c6_b_q = c6_b_ii;
        if (c6_b_ii == 0) {
          exitg5 = true;
        } else {
          c6_yc_a = c6_b_ii;
          c6_ad_a = c6_yc_a;
          c6_u_c = c6_ad_a;
          c6_test0 = c6_abs(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_ii), 1,
            6, 1, 0) - 1]) + c6_abs(chartInstance,
            c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                                      (real_T)(c6_u_c + 1)), 1, 6, 1, 0) - 1]);
          c6_ztest0 = c6_abs(chartInstance, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_ii),
            1, 6, 1, 0) - 1]);
          c6_eps(chartInstance);
          if (c6_ztest0 <= 2.2204460492503131E-16 * c6_test0) {
            guard4 = true;
            exitg5 = true;
          } else if (c6_ztest0 <= c6_tiny) {
            guard4 = true;
            exitg5 = true;
          } else {
            guard11 = false;
            if (c6_iter > 20.0) {
              c6_eps(chartInstance);
              if (c6_ztest0 <= 2.2204460492503131E-16 * c6_snorm) {
                guard3 = true;
                exitg5 = true;
              } else {
                guard11 = true;
              }
            } else {
              guard11 = true;
            }

            if (guard11 == true) {
              c6_j_ii--;
              guard3 = false;
              guard4 = false;
            }
          }
        }
      }

      if (guard4 == true) {
        guard3 = true;
      }

      if (guard3 == true) {
        c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_ii), 1, 6, 1, 0) - 1] = 0.0;
      }

      c6_bd_a = c6_m;
      c6_cd_a = c6_bd_a;
      c6_v_c = c6_cd_a;
      if (c6_b_q == c6_v_c - 1) {
        c6_kase = 4.0;
      } else {
        c6_qs = c6_m;
        c6_b_m = c6_m;
        c6_h_q = c6_b_q;
        c6_dd_a = c6_b_m;
        c6_ac_b = c6_h_q;
        c6_ed_a = c6_dd_a;
        c6_bc_b = c6_ac_b;
        if (c6_ed_a < c6_bc_b) {
          c6_l_overflow = false;
        } else {
          c6_eml_switch_helper(chartInstance);
          c6_l_overflow = (c6_bc_b < -2147483647);
        }

        if (c6_l_overflow) {
          c6_check_forloop_overflow_error(chartInstance, c6_l_overflow);
        }

        c6_k_ii = c6_b_m;
        guard2 = false;
        exitg4 = false;
        while ((exitg4 == false) && (c6_k_ii >= c6_h_q)) {
          c6_b_ii = c6_k_ii;
          c6_qs = c6_b_ii;
          if (c6_b_ii == c6_b_q) {
            exitg4 = true;
          } else {
            c6_test = 0.0;
            if (c6_b_ii < c6_m) {
              c6_test = c6_abs(chartInstance, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_ii), 1, 6, 1, 0)
                               - 1]);
            }

            c6_fd_a = c6_b_q;
            c6_gd_a = c6_fd_a;
            c6_w_c = c6_gd_a;
            if (c6_b_ii > c6_w_c + 1) {
              c6_hd_a = c6_b_ii;
              c6_id_a = c6_hd_a;
              c6_x_c = c6_id_a;
              c6_test += c6_abs(chartInstance, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)(c6_x_c - 1)), 1, 6,
                1, 0) - 1]);
            }

            c6_ztest = c6_abs(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_ii),
              1, 6, 1, 0) - 1]);
            c6_eps(chartInstance);
            if (c6_ztest <= 2.2204460492503131E-16 * c6_test) {
              guard2 = true;
              exitg4 = true;
            } else if (c6_ztest <= c6_tiny) {
              guard2 = true;
              exitg4 = true;
            } else {
              c6_k_ii--;
              guard2 = false;
            }
          }
        }

        if (guard2 == true) {
          c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ii), 1, 6, 1, 0) - 1] = 0.0;
        }

        if (c6_qs == c6_b_q) {
          c6_kase = 3.0;
        } else if (c6_qs == c6_m) {
          c6_kase = 1.0;
        } else {
          c6_kase = 2.0;
          c6_b_q = c6_qs;
        }
      }

      c6_jd_a = c6_b_q;
      c6_kd_a = c6_jd_a + 1;
      c6_b_q = c6_kd_a;
      switch ((int32_T)_SFD_INTEGER_CHECK("", c6_kase)) {
       case 1:
        c6_ld_a = c6_m;
        c6_md_a = c6_ld_a;
        c6_y_c = c6_md_a;
        c6_f = c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)(c6_y_c - 1)), 1, 6, 1, 0) - 1];
        c6_nd_a = c6_m;
        c6_od_a = c6_nd_a;
        c6_ab_c = c6_od_a;
        c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c6_ab_c - 1)), 1, 6, 1, 0) - 1] = 0.0;
        c6_pd_a = c6_m;
        c6_qd_a = c6_pd_a - 1;
        c6_i111 = c6_qd_a;
        c6_i_q = c6_b_q;
        c6_rd_a = c6_i111;
        c6_cc_b = c6_i_q;
        c6_sd_a = c6_rd_a;
        c6_dc_b = c6_cc_b;
        if (c6_sd_a < c6_dc_b) {
          c6_m_overflow = false;
        } else {
          c6_eml_switch_helper(chartInstance);
          c6_m_overflow = (c6_dc_b < -2147483647);
        }

        if (c6_m_overflow) {
          c6_check_forloop_overflow_error(chartInstance, c6_m_overflow);
        }

        for (c6_k = c6_i111; c6_k >= c6_i_q; c6_k--) {
          c6_b_k = c6_k;
          c6_t1 = c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) - 1];
          c6_b_t1 = c6_t1;
          c6_b_f = c6_f;
          c6_b_eml_xrotg(chartInstance, &c6_b_t1, &c6_b_f, &c6_cs, &c6_sn);
          c6_t1 = c6_b_t1;
          c6_f = c6_b_f;
          c6_b_cs = c6_cs;
          c6_b_sn = c6_sn;
          c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) - 1] = c6_t1;
          if (c6_b_k > c6_b_q) {
            c6_td_a = c6_b_k;
            c6_ud_a = c6_td_a - 1;
            c6_km1 = c6_ud_a;
            c6_f = -c6_b_sn * c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c6_km1), 1, 6, 1, 0) - 1];
            c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_km1), 1, 6, 1, 0) - 1] =
              c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c6_km1), 1, 6, 1, 0) - 1] * c6_b_cs;
          }

          c6_vd_a = c6_b_k;
          c6_wd_a = c6_vd_a;
          c6_bb_c = c6_wd_a;
          c6_ec_b = c6_bb_c - 1;
          c6_fc_b = c6_ec_b;
          c6_cb_c = 6 * c6_fc_b;
          c6_gc_b = c6_cb_c;
          c6_hc_b = c6_gc_b;
          c6_colk = c6_hc_b;
          c6_xd_a = c6_m;
          c6_yd_a = c6_xd_a;
          c6_db_c = c6_yd_a;
          c6_ic_b = c6_db_c - 1;
          c6_jc_b = c6_ic_b;
          c6_eb_c = 6 * c6_jc_b;
          c6_kc_b = c6_eb_c;
          c6_lc_b = c6_kc_b;
          c6_colm = c6_lc_b;
          c6_c_eml_xrot(chartInstance, c6_Vf, c6_colk + 1, c6_colm + 1, c6_b_cs,
                        c6_b_sn);
        }
        break;

       case 2:
        c6_ae_a = c6_b_q;
        c6_be_a = c6_ae_a - 1;
        c6_qm1 = c6_be_a;
        c6_f = c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_qm1), 1, 6, 1, 0) - 1];
        c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_qm1), 1, 6, 1, 0) - 1] = 0.0;
        c6_j_q = c6_b_q;
        c6_c_m = c6_m;
        c6_ce_a = c6_j_q;
        c6_mc_b = c6_c_m;
        c6_de_a = c6_ce_a;
        c6_nc_b = c6_mc_b;
        if (c6_de_a > c6_nc_b) {
          c6_n_overflow = false;
        } else {
          c6_eml_switch_helper(chartInstance);
          c6_n_overflow = (c6_nc_b > 2147483646);
        }

        if (c6_n_overflow) {
          c6_check_forloop_overflow_error(chartInstance, c6_n_overflow);
        }

        for (c6_c_k = c6_j_q; c6_c_k <= c6_c_m; c6_c_k++) {
          c6_b_k = c6_c_k;
          c6_t1 = c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) - 1];
          c6_c_t1 = c6_t1;
          c6_unusedU0 = c6_f;
          c6_b_eml_xrotg(chartInstance, &c6_c_t1, &c6_unusedU0, &c6_c_cs,
                         &c6_c_sn);
          c6_t1 = c6_c_t1;
          c6_b_cs = c6_c_cs;
          c6_b_sn = c6_c_sn;
          c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) - 1] = c6_t1;
          c6_f = -c6_b_sn * c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) - 1];
          c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) - 1] = c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK
            ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) -
            1] * c6_b_cs;
          c6_ee_a = c6_b_k;
          c6_fe_a = c6_ee_a;
          c6_fb_c = c6_fe_a;
          c6_oc_b = c6_fb_c - 1;
          c6_pc_b = c6_oc_b;
          c6_gb_c = 31 * c6_pc_b;
          c6_qc_b = c6_gb_c;
          c6_rc_b = c6_qc_b;
          c6_colk = c6_rc_b;
          c6_ge_a = c6_qm1;
          c6_he_a = c6_ge_a;
          c6_hb_c = c6_he_a;
          c6_sc_b = c6_hb_c - 1;
          c6_tc_b = c6_sc_b;
          c6_ib_c = 31 * c6_tc_b;
          c6_uc_b = c6_ib_c;
          c6_vc_b = c6_uc_b;
          c6_colqm1 = c6_vc_b;
          c6_d_eml_xrot(chartInstance, c6_U, c6_colk + 1, c6_colqm1 + 1, c6_b_cs,
                        c6_b_sn);
        }
        break;

       case 3:
        c6_ie_a = c6_m;
        c6_je_a = c6_ie_a - 1;
        c6_mm1 = c6_je_a;
        c6_d4 = c6_abs(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_m), 1, 6, 1, 0) - 1]);
        c6_d5 = c6_abs(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_mm1), 1, 6, 1, 0) - 1]);
        c6_d6 = c6_abs(chartInstance, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_mm1), 1, 6, 1, 0) - 1]);
        c6_d7 = c6_abs(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1]);
        c6_d8 = c6_abs(chartInstance, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1]);
        c6_c_varargin_1[0] = c6_d4;
        c6_c_varargin_1[1] = c6_d5;
        c6_c_varargin_1[2] = c6_d6;
        c6_c_varargin_1[3] = c6_d7;
        c6_c_varargin_1[4] = c6_d8;
        c6_ixstart = 1;
        c6_mtmp = c6_c_varargin_1[0];
        c6_g_x = c6_mtmp;
        c6_wc_b = muDoubleScalarIsNaN(c6_g_x);
        if (c6_wc_b) {
          c6_eml_switch_helper(chartInstance);
          c6_ix = 2;
          exitg2 = false;
          while ((exitg2 == false) && (c6_ix < 6)) {
            c6_b_ix = c6_ix;
            c6_ixstart = c6_b_ix;
            c6_h_x = c6_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c6_b_ix), 1, 5, 1, 0) - 1];
            c6_xc_b = muDoubleScalarIsNaN(c6_h_x);
            if (!c6_xc_b) {
              c6_mtmp = c6_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c6_b_ix), 1, 5, 1, 0) - 1];
              exitg2 = true;
            } else {
              c6_ix++;
            }
          }
        }

        if (c6_ixstart < 5) {
          c6_ke_a = c6_ixstart;
          c6_le_a = c6_ke_a + 1;
          c6_i112 = c6_le_a;
          c6_me_a = c6_i112;
          c6_ne_a = c6_me_a;
          if (c6_ne_a > 5) {
            c6_o_overflow = false;
          } else {
            c6_eml_switch_helper(chartInstance);
            c6_o_overflow = false;
          }

          if (c6_o_overflow) {
            c6_check_forloop_overflow_error(chartInstance, c6_o_overflow);
          }

          for (c6_c_ix = c6_i112; c6_c_ix < 6; c6_c_ix++) {
            c6_b_ix = c6_c_ix;
            c6_oe_a = c6_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c6_b_ix), 1, 5, 1, 0) - 1];
            c6_yc_b = c6_mtmp;
            c6_p = (c6_oe_a > c6_yc_b);
            if (c6_p) {
              c6_mtmp = c6_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c6_b_ix), 1, 5, 1, 0) - 1];
            }
          }
        }

        c6_b_mtmp = c6_mtmp;
        c6_scale = c6_b_mtmp;
        c6_sm = c6_eml_div(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_m), 1, 6, 1, 0) - 1],
                           c6_scale);
        c6_smm1 = c6_eml_div(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_mm1), 1,
          6, 1, 0) - 1], c6_scale);
        c6_emm1 = c6_eml_div(chartInstance, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_mm1), 1,
          6, 1, 0) - 1], c6_scale);
        c6_sqds = c6_eml_div(chartInstance, c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1,
          6, 1, 0) - 1], c6_scale);
        c6_eqds = c6_eml_div(chartInstance, c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1,
          6, 1, 0) - 1], c6_scale);
        c6_ad_b = c6_eml_div(chartInstance, (c6_smm1 + c6_sm) * (c6_smm1 - c6_sm)
                             + c6_emm1 * c6_emm1, 2.0);
        c6_jb_c = c6_sm * c6_emm1;
        c6_jb_c *= c6_jb_c;
        c6_shift = 0.0;
        guard1 = false;
        if (c6_ad_b != 0.0) {
          guard1 = true;
        } else {
          if (c6_jb_c != 0.0) {
            guard1 = true;
          }
        }

        if (guard1 == true) {
          c6_shift = c6_ad_b * c6_ad_b + c6_jb_c;
          c6_b_sqrt(chartInstance, &c6_shift);
          if (c6_ad_b < 0.0) {
            c6_shift = -c6_shift;
          }

          c6_shift = c6_eml_div(chartInstance, c6_jb_c, c6_ad_b + c6_shift);
        }

        c6_f = (c6_sqds + c6_sm) * (c6_sqds - c6_sm) + c6_shift;
        c6_g = c6_sqds * c6_eqds;
        c6_k_q = c6_b_q;
        c6_b_mm1 = c6_mm1;
        c6_pe_a = c6_k_q;
        c6_bd_b = c6_b_mm1;
        c6_qe_a = c6_pe_a;
        c6_cd_b = c6_bd_b;
        if (c6_qe_a > c6_cd_b) {
          c6_p_overflow = false;
        } else {
          c6_eml_switch_helper(chartInstance);
          c6_p_overflow = (c6_cd_b > 2147483646);
        }

        if (c6_p_overflow) {
          c6_check_forloop_overflow_error(chartInstance, c6_p_overflow);
        }

        for (c6_d_k = c6_k_q; c6_d_k <= c6_b_mm1; c6_d_k++) {
          c6_b_k = c6_d_k;
          c6_re_a = c6_b_k;
          c6_se_a = c6_re_a;
          c6_km1 = c6_se_a;
          c6_te_a = c6_b_k;
          c6_ue_a = c6_te_a + 1;
          c6_kp1 = c6_ue_a;
          c6_c_f = c6_f;
          c6_unusedU1 = c6_g;
          c6_b_eml_xrotg(chartInstance, &c6_c_f, &c6_unusedU1, &c6_d_cs,
                         &c6_d_sn);
          c6_f = c6_c_f;
          c6_b_cs = c6_d_cs;
          c6_b_sn = c6_d_sn;
          if (c6_b_k > c6_b_q) {
            c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)(c6_km1 - 1)), 1, 6, 1, 0) - 1] = c6_f;
          }

          c6_f = c6_b_cs * c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) - 1] + c6_b_sn *
            c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) - 1];
          c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) - 1] = c6_b_cs *
            c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) - 1] - c6_b_sn *
            c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) - 1];
          c6_g = c6_b_sn * c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_kp1), 1, 6, 1, 0) - 1];
          c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_kp1), 1, 6, 1, 0) - 1] = c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK
            ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_kp1), 1, 6, 1, 0) -
            1] * c6_b_cs;
          c6_ve_a = c6_b_k;
          c6_we_a = c6_ve_a;
          c6_kb_c = c6_we_a;
          c6_dd_b = c6_kb_c - 1;
          c6_ed_b = c6_dd_b;
          c6_lb_c = 6 * c6_ed_b;
          c6_fd_b = c6_lb_c;
          c6_gd_b = c6_fd_b;
          c6_colk = c6_gd_b;
          c6_hd_b = c6_b_k;
          c6_id_b = c6_hd_b;
          c6_mb_c = 6 * c6_id_b;
          c6_jd_b = c6_mb_c;
          c6_kd_b = c6_jd_b;
          c6_colkp1 = c6_kd_b;
          c6_c_eml_xrot(chartInstance, c6_Vf, c6_colk + 1, c6_colkp1 + 1,
                        c6_b_cs, c6_b_sn);
          c6_d_f = c6_f;
          c6_unusedU2 = c6_g;
          c6_b_eml_xrotg(chartInstance, &c6_d_f, &c6_unusedU2, &c6_e_cs,
                         &c6_e_sn);
          c6_f = c6_d_f;
          c6_b_cs = c6_e_cs;
          c6_b_sn = c6_e_sn;
          c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) - 1] = c6_f;
          c6_f = c6_b_cs * c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) - 1] + c6_b_sn *
            c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_kp1), 1, 6, 1, 0) - 1];
          c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_kp1), 1, 6, 1, 0) - 1] = -c6_b_sn *
            c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_k), 1, 6, 1, 0) - 1] + c6_b_cs *
            c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_kp1), 1, 6, 1, 0) - 1];
          c6_g = c6_b_sn * c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_kp1), 1, 6, 1, 0) - 1];
          c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_kp1), 1, 6, 1, 0) - 1] = c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK
            ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_kp1), 1, 6, 1, 0) -
            1] * c6_b_cs;
          if (c6_b_k < 31) {
            c6_xe_a = c6_b_k;
            c6_ye_a = c6_xe_a;
            c6_nb_c = c6_ye_a;
            c6_ld_b = c6_nb_c - 1;
            c6_md_b = c6_ld_b;
            c6_ob_c = 31 * c6_md_b;
            c6_nd_b = c6_ob_c;
            c6_od_b = c6_nd_b;
            c6_colk = c6_od_b;
            c6_pd_b = c6_b_k;
            c6_qd_b = c6_pd_b;
            c6_pb_c = 31 * c6_qd_b;
            c6_rd_b = c6_pb_c;
            c6_sd_b = c6_rd_b;
            c6_colkp1 = c6_sd_b;
            c6_d_eml_xrot(chartInstance, c6_U, c6_colk + 1, c6_colkp1 + 1,
                          c6_b_cs, c6_b_sn);
          }
        }

        c6_af_a = c6_m;
        c6_bf_a = c6_af_a;
        c6_qb_c = c6_bf_a;
        c6_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)(c6_qb_c - 1)), 1, 6, 1, 0) - 1] = c6_f;
        c6_iter++;
        break;

       default:
        if (c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_b_q), 1, 6, 1, 0) - 1] < 0.0) {
          c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_q), 1, 6, 1, 0) - 1] =
            -c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_q), 1, 6, 1, 0) - 1];
          c6_cf_a = c6_b_q;
          c6_if_a = c6_cf_a;
          c6_rb_c = c6_if_a;
          c6_td_b = c6_rb_c - 1;
          c6_ee_b = c6_td_b;
          c6_sb_c = 6 * c6_ee_b;
          c6_ud_b = c6_sb_c;
          c6_fe_b = c6_ud_b;
          c6_colq = c6_fe_b;
          c6_c_eml_scalar_eg(chartInstance);
          c6_d9 = -1.0;
          c6_h_eml_xscal(chartInstance, c6_d9, c6_Vf, c6_colq + 1);
        }

        c6_df_a = c6_b_q;
        c6_jf_a = c6_df_a + 1;
        c6_qp1 = c6_jf_a;
        exitg3 = false;
        while ((exitg3 == false) && (c6_b_q < 6)) {
          if (c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c6_b_q), 1, 6, 1, 0) - 1] <
              c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c6_qp1), 1, 6, 1, 0) - 1]) {
            c6_rt = c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c6_b_q), 1, 6, 1, 0) - 1];
            c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_b_q), 1, 6, 1, 0) - 1] =
              c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c6_qp1), 1, 6, 1, 0) - 1];
            c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_qp1), 1, 6, 1, 0) - 1] = c6_rt;
            if (c6_b_q < 6) {
              c6_ff_a = c6_b_q;
              c6_lf_a = c6_ff_a;
              c6_tb_c = c6_lf_a;
              c6_vd_b = c6_tb_c - 1;
              c6_ge_b = c6_vd_b;
              c6_ub_c = 6 * c6_ge_b;
              c6_wd_b = c6_ub_c;
              c6_he_b = c6_wd_b;
              c6_colq = c6_he_b;
              c6_xd_b = c6_b_q;
              c6_ie_b = c6_xd_b;
              c6_vb_c = 6 * c6_ie_b;
              c6_yd_b = c6_vb_c;
              c6_je_b = c6_yd_b;
              c6_colqp1 = c6_je_b;
              c6_c_eml_xswap(chartInstance, c6_Vf, c6_colq + 1, c6_colqp1 + 1);
            }

            if (c6_b_q < 31) {
              c6_gf_a = c6_b_q;
              c6_mf_a = c6_gf_a;
              c6_wb_c = c6_mf_a;
              c6_ae_b = c6_wb_c - 1;
              c6_ke_b = c6_ae_b;
              c6_xb_c = 31 * c6_ke_b;
              c6_be_b = c6_xb_c;
              c6_le_b = c6_be_b;
              c6_colq = c6_le_b;
              c6_ce_b = c6_b_q;
              c6_me_b = c6_ce_b;
              c6_yb_c = 31 * c6_me_b;
              c6_de_b = c6_yb_c;
              c6_ne_b = c6_de_b;
              c6_colqp1 = c6_ne_b;
              c6_d_eml_xswap(chartInstance, c6_U, c6_colq + 1, c6_colqp1 + 1);
            }

            c6_b_q = c6_qp1;
            c6_hf_a = c6_b_q;
            c6_nf_a = c6_hf_a + 1;
            c6_qp1 = c6_nf_a;
          } else {
            exitg3 = true;
          }
        }

        c6_iter = 0.0;
        c6_ef_a = c6_m;
        c6_kf_a = c6_ef_a - 1;
        c6_m = c6_kf_a;
        break;
      }
    }
  }

  for (c6_e_k = 1; c6_e_k < 7; c6_e_k++) {
    c6_b_k = c6_e_k;
    c6_S[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_b_k), 1, 6, 1, 0) - 1] = c6_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) - 1];
  }

  for (c6_j = 1; c6_j < 7; c6_j++) {
    c6_b_j = c6_j;
    for (c6_i = 1; c6_i < 7; c6_i++) {
      c6_b_i = c6_i;
      c6_V[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c6_b_i), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 6, 2, 0) - 1))
        - 1] = c6_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c6_b_i), 1, 6, 1, 0) + 6 *
                      (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 6, 2, 0) - 1)) - 1];
    }
  }
}

static real_T c6_eml_xnrm2(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_x[186], int32_T c6_ix0)
{
  real_T c6_y;
  int32_T c6_b_n;
  int32_T c6_b_ix0;
  int32_T c6_c_n;
  int32_T c6_c_ix0;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_scale;
  int32_T c6_kstart;
  int32_T c6_a;
  int32_T c6_c;
  int32_T c6_b_a;
  int32_T c6_b_c;
  int32_T c6_c_a;
  int32_T c6_b;
  int32_T c6_kend;
  int32_T c6_b_kstart;
  int32_T c6_b_kend;
  int32_T c6_d_a;
  int32_T c6_b_b;
  int32_T c6_e_a;
  int32_T c6_c_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_b_k;
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_absxk;
  real_T c6_t;
  c6_b_n = c6_n;
  c6_b_ix0 = c6_ix0;
  c6_threshold(chartInstance);
  c6_c_n = c6_b_n;
  c6_c_ix0 = c6_b_ix0;
  c6_y = 0.0;
  if (c6_c_n < 1) {
  } else if (c6_c_n == 1) {
    c6_b_x = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_c_ix0), 1, 186, 1, 0) - 1];
    c6_c_x = c6_b_x;
    c6_y = muDoubleScalarAbs(c6_c_x);
  } else {
    c6_realmin(chartInstance);
    c6_scale = 2.2250738585072014E-308;
    c6_kstart = c6_c_ix0;
    c6_a = c6_c_n;
    c6_c = c6_a;
    c6_b_a = c6_c - 1;
    c6_b_c = c6_b_a;
    c6_c_a = c6_kstart;
    c6_b = c6_b_c;
    c6_kend = c6_c_a + c6_b;
    c6_b_kstart = c6_kstart;
    c6_b_kend = c6_kend;
    c6_d_a = c6_b_kstart;
    c6_b_b = c6_b_kend;
    c6_e_a = c6_d_a;
    c6_c_b = c6_b_b;
    if (c6_e_a > c6_c_b) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = (c6_c_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_k = c6_b_kstart; c6_k <= c6_b_kend; c6_k++) {
      c6_b_k = c6_k;
      c6_d_x = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_b_k), 1, 186, 1, 0) - 1];
      c6_e_x = c6_d_x;
      c6_absxk = muDoubleScalarAbs(c6_e_x);
      if (c6_absxk > c6_scale) {
        c6_t = c6_scale / c6_absxk;
        c6_y = 1.0 + c6_y * c6_t * c6_t;
        c6_scale = c6_absxk;
      } else {
        c6_t = c6_absxk / c6_scale;
        c6_y += c6_t * c6_t;
      }
    }

    c6_y = c6_scale * muDoubleScalarSqrt(c6_y);
  }

  return c6_y;
}

static void c6_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c6_abs(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                     real_T c6_x)
{
  real_T c6_b_x;
  (void)chartInstance;
  c6_b_x = c6_x;
  return muDoubleScalarAbs(c6_b_x);
}

static void c6_realmin(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_check_forloop_overflow_error
  (SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance, boolean_T
   c6_overflow)
{
  int32_T c6_i113;
  static char_T c6_cv1[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c6_u[34];
  const mxArray *c6_y = NULL;
  int32_T c6_i114;
  static char_T c6_cv2[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c6_b_u[23];
  const mxArray *c6_b_y = NULL;
  (void)chartInstance;
  if (!c6_overflow) {
  } else {
    for (c6_i113 = 0; c6_i113 < 34; c6_i113++) {
      c6_u[c6_i113] = c6_cv1[c6_i113];
    }

    c6_y = NULL;
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  false);
    for (c6_i114 = 0; c6_i114 < 23; c6_i114++) {
      c6_b_u[c6_i114] = c6_cv2[c6_i114];
    }

    c6_b_y = NULL;
    sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c6_y, 14, c6_b_y));
  }
}

static real_T c6_eml_div(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x, real_T c6_y)
{
  real_T c6_b_x;
  real_T c6_b_y;
  (void)chartInstance;
  c6_b_x = c6_x;
  c6_b_y = c6_y;
  return c6_b_x / c6_b_y;
}

static void c6_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[186], int32_T c6_ix0,
  real_T c6_b_x[186])
{
  int32_T c6_i115;
  for (c6_i115 = 0; c6_i115 < 186; c6_i115++) {
    c6_b_x[c6_i115] = c6_x[c6_i115];
  }

  c6_e_eml_xscal(chartInstance, c6_n, c6_a, c6_b_x, c6_ix0);
}

static void c6_b_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c6_eml_xdotc(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_x[186], int32_T c6_ix0, real_T c6_y
  [186], int32_T c6_iy0)
{
  real_T c6_d;
  int32_T c6_b_n;
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_n;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_d_n;
  int32_T c6_d_ix0;
  int32_T c6_d_iy0;
  int32_T c6_e_n;
  int32_T c6_e_ix0;
  int32_T c6_e_iy0;
  int32_T c6_ix;
  int32_T c6_iy;
  int32_T c6_f_n;
  int32_T c6_b;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_a;
  int32_T c6_b_a;
  c6_b_n = c6_n;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_c_n = c6_b_n;
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  c6_c_threshold(chartInstance);
  c6_d_n = c6_c_n;
  c6_d_ix0 = c6_c_ix0;
  c6_d_iy0 = c6_c_iy0;
  c6_e_n = c6_d_n;
  c6_e_ix0 = c6_d_ix0;
  c6_e_iy0 = c6_d_iy0;
  c6_d = 0.0;
  if (c6_e_n < 1) {
  } else {
    c6_ix = c6_e_ix0;
    c6_iy = c6_e_iy0;
    c6_f_n = c6_e_n;
    c6_b = c6_f_n;
    c6_b_b = c6_b;
    if (1 > c6_b_b) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = (c6_b_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_k = 1; c6_k <= c6_f_n; c6_k++) {
      c6_d += c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_ix), 1, 186, 1, 0) - 1] *
        c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c6_iy), 1, 186, 1, 0) - 1];
      c6_a = c6_ix + 1;
      c6_ix = c6_a;
      c6_b_a = c6_iy + 1;
      c6_iy = c6_b_a;
    }
  }

  return c6_d;
}

static void c6_c_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, int32_T c6_ix0, real_T c6_y[186],
  int32_T c6_iy0, real_T c6_b_y[186])
{
  int32_T c6_i116;
  for (c6_i116 = 0; c6_i116 < 186; c6_i116++) {
    c6_b_y[c6_i116] = c6_y[c6_i116];
  }

  c6_e_eml_xaxpy(chartInstance, c6_n, c6_a, c6_ix0, c6_b_y, c6_iy0);
}

static void c6_d_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c6_b_eml_xnrm2(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_x[6], int32_T c6_ix0)
{
  real_T c6_y;
  int32_T c6_b_n;
  int32_T c6_b_ix0;
  int32_T c6_c_n;
  int32_T c6_c_ix0;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_scale;
  int32_T c6_kstart;
  int32_T c6_a;
  int32_T c6_c;
  int32_T c6_b_a;
  int32_T c6_b_c;
  int32_T c6_c_a;
  int32_T c6_b;
  int32_T c6_kend;
  int32_T c6_b_kstart;
  int32_T c6_b_kend;
  int32_T c6_d_a;
  int32_T c6_b_b;
  int32_T c6_e_a;
  int32_T c6_c_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_b_k;
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_absxk;
  real_T c6_t;
  c6_b_n = c6_n;
  c6_b_ix0 = c6_ix0;
  c6_threshold(chartInstance);
  c6_c_n = c6_b_n;
  c6_c_ix0 = c6_b_ix0;
  c6_y = 0.0;
  if (c6_c_n < 1) {
  } else if (c6_c_n == 1) {
    c6_b_x = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_c_ix0), 1, 6, 1, 0) - 1];
    c6_c_x = c6_b_x;
    c6_y = muDoubleScalarAbs(c6_c_x);
  } else {
    c6_realmin(chartInstance);
    c6_scale = 2.2250738585072014E-308;
    c6_kstart = c6_c_ix0;
    c6_a = c6_c_n;
    c6_c = c6_a;
    c6_b_a = c6_c - 1;
    c6_b_c = c6_b_a;
    c6_c_a = c6_kstart;
    c6_b = c6_b_c;
    c6_kend = c6_c_a + c6_b;
    c6_b_kstart = c6_kstart;
    c6_b_kend = c6_kend;
    c6_d_a = c6_b_kstart;
    c6_b_b = c6_b_kend;
    c6_e_a = c6_d_a;
    c6_c_b = c6_b_b;
    if (c6_e_a > c6_c_b) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = (c6_c_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_k = c6_b_kstart; c6_k <= c6_b_kend; c6_k++) {
      c6_b_k = c6_k;
      c6_d_x = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_b_k), 1, 6, 1, 0) - 1];
      c6_e_x = c6_d_x;
      c6_absxk = muDoubleScalarAbs(c6_e_x);
      if (c6_absxk > c6_scale) {
        c6_t = c6_scale / c6_absxk;
        c6_y = 1.0 + c6_y * c6_t * c6_t;
        c6_scale = c6_absxk;
      } else {
        c6_t = c6_absxk / c6_scale;
        c6_y += c6_t * c6_t;
      }
    }

    c6_y = c6_scale * muDoubleScalarSqrt(c6_y);
  }

  return c6_y;
}

static void c6_b_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[6], int32_T c6_ix0,
  real_T c6_b_x[6])
{
  int32_T c6_i117;
  for (c6_i117 = 0; c6_i117 < 6; c6_i117++) {
    c6_b_x[c6_i117] = c6_x[c6_i117];
  }

  c6_f_eml_xscal(chartInstance, c6_n, c6_a, c6_b_x, c6_ix0);
}

static void c6_b_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[186], int32_T c6_ix0,
  real_T c6_y[31], int32_T c6_iy0, real_T c6_b_y[31])
{
  int32_T c6_i118;
  int32_T c6_i119;
  real_T c6_b_x[186];
  for (c6_i118 = 0; c6_i118 < 31; c6_i118++) {
    c6_b_y[c6_i118] = c6_y[c6_i118];
  }

  for (c6_i119 = 0; c6_i119 < 186; c6_i119++) {
    c6_b_x[c6_i119] = c6_x[c6_i119];
  }

  c6_f_eml_xaxpy(chartInstance, c6_n, c6_a, c6_b_x, c6_ix0, c6_b_y, c6_iy0);
}

static void c6_c_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[31], int32_T c6_ix0,
  real_T c6_y[186], int32_T c6_iy0, real_T c6_b_y[186])
{
  int32_T c6_i120;
  int32_T c6_i121;
  real_T c6_b_x[31];
  for (c6_i120 = 0; c6_i120 < 186; c6_i120++) {
    c6_b_y[c6_i120] = c6_y[c6_i120];
  }

  for (c6_i121 = 0; c6_i121 < 31; c6_i121++) {
    c6_b_x[c6_i121] = c6_x[c6_i121];
  }

  c6_g_eml_xaxpy(chartInstance, c6_n, c6_a, c6_b_x, c6_ix0, c6_b_y, c6_iy0);
}

static real_T c6_b_eml_xdotc(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_x[36], int32_T c6_ix0, real_T c6_y[36],
  int32_T c6_iy0)
{
  real_T c6_d;
  int32_T c6_b_n;
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_n;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_d_n;
  int32_T c6_d_ix0;
  int32_T c6_d_iy0;
  int32_T c6_e_n;
  int32_T c6_e_ix0;
  int32_T c6_e_iy0;
  int32_T c6_ix;
  int32_T c6_iy;
  int32_T c6_f_n;
  int32_T c6_b;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_a;
  int32_T c6_b_a;
  c6_b_n = c6_n;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_c_n = c6_b_n;
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  c6_c_threshold(chartInstance);
  c6_d_n = c6_c_n;
  c6_d_ix0 = c6_c_ix0;
  c6_d_iy0 = c6_c_iy0;
  c6_e_n = c6_d_n;
  c6_e_ix0 = c6_d_ix0;
  c6_e_iy0 = c6_d_iy0;
  c6_d = 0.0;
  if (c6_e_n < 1) {
  } else {
    c6_ix = c6_e_ix0;
    c6_iy = c6_e_iy0;
    c6_f_n = c6_e_n;
    c6_b = c6_f_n;
    c6_b_b = c6_b;
    if (1 > c6_b_b) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = (c6_b_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_k = 1; c6_k <= c6_f_n; c6_k++) {
      c6_d += c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_ix), 1, 36, 1, 0) - 1] * c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_iy), 1, 36, 1, 0) - 1];
      c6_a = c6_ix + 1;
      c6_ix = c6_a;
      c6_b_a = c6_iy + 1;
      c6_iy = c6_b_a;
    }
  }

  return c6_d;
}

static void c6_d_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, int32_T c6_ix0, real_T c6_y[36],
  int32_T c6_iy0, real_T c6_b_y[36])
{
  int32_T c6_i122;
  for (c6_i122 = 0; c6_i122 < 36; c6_i122++) {
    c6_b_y[c6_i122] = c6_y[c6_i122];
  }

  c6_h_eml_xaxpy(chartInstance, c6_n, c6_a, c6_ix0, c6_b_y, c6_iy0);
}

static void c6_c_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_x[186], int32_T c6_ix0, real_T c6_b_x
  [186])
{
  int32_T c6_i123;
  for (c6_i123 = 0; c6_i123 < 186; c6_i123++) {
    c6_b_x[c6_i123] = c6_x[c6_i123];
  }

  c6_g_eml_xscal(chartInstance, c6_a, c6_b_x, c6_ix0);
}

static void c6_d_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_x[36], int32_T c6_ix0, real_T c6_b_x[36])
{
  int32_T c6_i124;
  for (c6_i124 = 0; c6_i124 < 36; c6_i124++) {
    c6_b_x[c6_i124] = c6_x[c6_i124];
  }

  c6_h_eml_xscal(chartInstance, c6_a, c6_b_x, c6_ix0);
}

static void c6_eps(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c6_b_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_b_eml_error(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  int32_T c6_i125;
  static char_T c6_cv3[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'N', 'o', 'C', 'o', 'n', 'v', 'e', 'r',
    'g', 'e', 'n', 'c', 'e' };

  char_T c6_u[30];
  const mxArray *c6_y = NULL;
  (void)chartInstance;
  for (c6_i125 = 0; c6_i125 < 30; c6_i125++) {
    c6_u[c6_i125] = c6_cv3[c6_i125];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c6_y));
}

static real_T c6_sqrt(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                      real_T c6_x)
{
  real_T c6_b_x;
  c6_b_x = c6_x;
  c6_b_sqrt(chartInstance, &c6_b_x);
  return c6_b_x;
}

static void c6_c_eml_error(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  int32_T c6_i126;
  static char_T c6_cv4[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c6_u[30];
  const mxArray *c6_y = NULL;
  int32_T c6_i127;
  static char_T c6_cv5[4] = { 's', 'q', 'r', 't' };

  char_T c6_b_u[4];
  const mxArray *c6_b_y = NULL;
  (void)chartInstance;
  for (c6_i126 = 0; c6_i126 < 30; c6_i126++) {
    c6_u[c6_i126] = c6_cv4[c6_i126];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c6_i127 = 0; c6_i127 < 4; c6_i127++) {
    c6_b_u[c6_i127] = c6_cv5[c6_i127];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c6_y, 14, c6_b_y));
}

static void c6_eml_xrotg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_b, real_T *c6_b_a, real_T *c6_b_b,
  real_T *c6_c, real_T *c6_s)
{
  *c6_b_a = c6_a;
  *c6_b_b = c6_b;
  c6_b_eml_xrotg(chartInstance, c6_b_a, c6_b_b, c6_c, c6_s);
}

static void c6_eml_xrot(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[36], int32_T c6_ix0, int32_T c6_iy0, real_T c6_c,
  real_T c6_s, real_T c6_b_x[36])
{
  int32_T c6_i128;
  for (c6_i128 = 0; c6_i128 < 36; c6_i128++) {
    c6_b_x[c6_i128] = c6_x[c6_i128];
  }

  c6_c_eml_xrot(chartInstance, c6_b_x, c6_ix0, c6_iy0, c6_c, c6_s);
}

static void c6_e_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_b_eml_xrot(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[186], int32_T c6_ix0, int32_T c6_iy0, real_T c6_c,
  real_T c6_s, real_T c6_b_x[186])
{
  int32_T c6_i129;
  for (c6_i129 = 0; c6_i129 < 186; c6_i129++) {
    c6_b_x[c6_i129] = c6_x[c6_i129];
  }

  c6_d_eml_xrot(chartInstance, c6_b_x, c6_ix0, c6_iy0, c6_c, c6_s);
}

static void c6_c_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_eml_xswap(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[36], int32_T c6_ix0, int32_T c6_iy0, real_T
  c6_b_x[36])
{
  int32_T c6_i130;
  for (c6_i130 = 0; c6_i130 < 36; c6_i130++) {
    c6_b_x[c6_i130] = c6_x[c6_i130];
  }

  c6_c_eml_xswap(chartInstance, c6_b_x, c6_ix0, c6_iy0);
}

static void c6_f_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_b_eml_xswap(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[186], int32_T c6_ix0, int32_T c6_iy0, real_T
  c6_b_x[186])
{
  int32_T c6_i131;
  for (c6_i131 = 0; c6_i131 < 186; c6_i131++) {
    c6_b_x[c6_i131] = c6_x[c6_i131];
  }

  c6_d_eml_xswap(chartInstance, c6_b_x, c6_ix0, c6_iy0);
}

static void c6_eml_xgemm(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_k, real_T c6_A[36], real_T c6_B[186], real_T c6_C
  [186], real_T c6_b_C[186])
{
  int32_T c6_i132;
  int32_T c6_i133;
  real_T c6_b_A[36];
  int32_T c6_i134;
  real_T c6_b_B[186];
  for (c6_i132 = 0; c6_i132 < 186; c6_i132++) {
    c6_b_C[c6_i132] = c6_C[c6_i132];
  }

  for (c6_i133 = 0; c6_i133 < 36; c6_i133++) {
    c6_b_A[c6_i133] = c6_A[c6_i133];
  }

  for (c6_i134 = 0; c6_i134 < 186; c6_i134++) {
    c6_b_B[c6_i134] = c6_B[c6_i134];
  }

  c6_b_eml_xgemm(chartInstance, c6_k, c6_b_A, c6_b_B, c6_b_C);
}

static void c6_below_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  c6_g_threshold(chartInstance);
}

static void c6_g_threshold(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_d_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c6_e_eml_scalar_eg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static int32_T c6_h_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i135;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i135, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i135;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
    chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_i_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_dirdyna_walkman_robotran,
  const char_T *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_dirdyna_walkman_robotran), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_dirdyna_walkman_robotran);
  return c6_y;
}

static uint8_T c6_j_emlrt_marshallIn(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_e_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[186], int32_T c6_ix0)
{
  int32_T c6_b_n;
  real_T c6_b_a;
  int32_T c6_b_ix0;
  int32_T c6_c_n;
  real_T c6_c_a;
  int32_T c6_c_ix0;
  int32_T c6_d_ix0;
  int32_T c6_d_a;
  int32_T c6_c;
  int32_T c6_b;
  int32_T c6_b_c;
  int32_T c6_e_a;
  int32_T c6_b_b;
  int32_T c6_i136;
  int32_T c6_f_a;
  int32_T c6_c_b;
  int32_T c6_g_a;
  int32_T c6_d_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_b_k;
  c6_b_n = c6_n;
  c6_b_a = c6_a;
  c6_b_ix0 = c6_ix0;
  c6_b_threshold(chartInstance);
  c6_c_n = c6_b_n;
  c6_c_a = c6_b_a;
  c6_c_ix0 = c6_b_ix0;
  c6_d_ix0 = c6_c_ix0;
  c6_d_a = c6_c_n;
  c6_c = c6_d_a;
  c6_b = c6_c - 1;
  c6_b_c = c6_b;
  c6_e_a = c6_c_ix0;
  c6_b_b = c6_b_c;
  c6_i136 = c6_e_a + c6_b_b;
  c6_f_a = c6_d_ix0;
  c6_c_b = c6_i136;
  c6_g_a = c6_f_a;
  c6_d_b = c6_c_b;
  if (c6_g_a > c6_d_b) {
    c6_overflow = false;
  } else {
    c6_eml_switch_helper(chartInstance);
    c6_overflow = (c6_d_b > 2147483646);
  }

  if (c6_overflow) {
    c6_check_forloop_overflow_error(chartInstance, c6_overflow);
  }

  for (c6_k = c6_d_ix0; c6_k <= c6_i136; c6_k++) {
    c6_b_k = c6_k;
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_b_k), 1, 186, 1, 0) - 1] = c6_c_a * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 186, 1, 0) - 1];
  }
}

static void c6_e_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, int32_T c6_ix0, real_T c6_y[186],
  int32_T c6_iy0)
{
  int32_T c6_b_n;
  real_T c6_b_a;
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_n;
  real_T c6_c_a;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_d_a;
  int32_T c6_ix;
  int32_T c6_e_a;
  int32_T c6_iy;
  int32_T c6_f_a;
  int32_T c6_i137;
  int32_T c6_b;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_g_a;
  int32_T c6_c;
  int32_T c6_h_a;
  int32_T c6_b_c;
  int32_T c6_i_a;
  int32_T c6_c_c;
  int32_T c6_j_a;
  int32_T c6_k_a;
  c6_b_n = c6_n;
  c6_b_a = c6_a;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_d_threshold(chartInstance);
  c6_c_n = c6_b_n;
  c6_c_a = c6_b_a;
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  if (c6_c_n < 1) {
  } else if (c6_c_a == 0.0) {
  } else {
    c6_d_a = c6_c_ix0 - 1;
    c6_ix = c6_d_a;
    c6_e_a = c6_c_iy0 - 1;
    c6_iy = c6_e_a;
    c6_f_a = c6_c_n - 1;
    c6_i137 = c6_f_a;
    c6_b = c6_i137;
    c6_b_b = c6_b;
    if (0 > c6_b_b) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = (c6_b_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_k = 0; c6_k <= c6_i137; c6_k++) {
      c6_g_a = c6_iy;
      c6_c = c6_g_a;
      c6_h_a = c6_iy;
      c6_b_c = c6_h_a;
      c6_i_a = c6_ix;
      c6_c_c = c6_i_a;
      c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_c + 1)), 1, 186, 1, 0) - 1] =
        c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_b_c + 1)), 1, 186, 1, 0) - 1] + c6_c_a *
        c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_c_c + 1)), 1, 186, 1, 0) - 1];
      c6_j_a = c6_ix + 1;
      c6_ix = c6_j_a;
      c6_k_a = c6_iy + 1;
      c6_iy = c6_k_a;
    }
  }
}

static void c6_f_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[6], int32_T c6_ix0)
{
  int32_T c6_b_n;
  real_T c6_b_a;
  int32_T c6_b_ix0;
  int32_T c6_c_n;
  real_T c6_c_a;
  int32_T c6_c_ix0;
  int32_T c6_d_ix0;
  int32_T c6_d_a;
  int32_T c6_c;
  int32_T c6_b;
  int32_T c6_b_c;
  int32_T c6_e_a;
  int32_T c6_b_b;
  int32_T c6_i138;
  int32_T c6_f_a;
  int32_T c6_c_b;
  int32_T c6_g_a;
  int32_T c6_d_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_b_k;
  c6_b_n = c6_n;
  c6_b_a = c6_a;
  c6_b_ix0 = c6_ix0;
  c6_b_threshold(chartInstance);
  c6_c_n = c6_b_n;
  c6_c_a = c6_b_a;
  c6_c_ix0 = c6_b_ix0;
  c6_d_ix0 = c6_c_ix0;
  c6_d_a = c6_c_n;
  c6_c = c6_d_a;
  c6_b = c6_c - 1;
  c6_b_c = c6_b;
  c6_e_a = c6_c_ix0;
  c6_b_b = c6_b_c;
  c6_i138 = c6_e_a + c6_b_b;
  c6_f_a = c6_d_ix0;
  c6_c_b = c6_i138;
  c6_g_a = c6_f_a;
  c6_d_b = c6_c_b;
  if (c6_g_a > c6_d_b) {
    c6_overflow = false;
  } else {
    c6_eml_switch_helper(chartInstance);
    c6_overflow = (c6_d_b > 2147483646);
  }

  if (c6_overflow) {
    c6_check_forloop_overflow_error(chartInstance, c6_overflow);
  }

  for (c6_k = c6_d_ix0; c6_k <= c6_i138; c6_k++) {
    c6_b_k = c6_k;
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_b_k), 1, 6, 1, 0) - 1] = c6_c_a * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 6, 1, 0) - 1];
  }
}

static void c6_f_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[186], int32_T c6_ix0,
  real_T c6_y[31], int32_T c6_iy0)
{
  int32_T c6_b_n;
  real_T c6_b_a;
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_n;
  real_T c6_c_a;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_d_a;
  int32_T c6_ix;
  int32_T c6_e_a;
  int32_T c6_iy;
  int32_T c6_f_a;
  int32_T c6_i139;
  int32_T c6_b;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_g_a;
  int32_T c6_c;
  int32_T c6_h_a;
  int32_T c6_b_c;
  int32_T c6_i_a;
  int32_T c6_c_c;
  int32_T c6_j_a;
  int32_T c6_k_a;
  c6_b_n = c6_n;
  c6_b_a = c6_a;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_d_threshold(chartInstance);
  c6_c_n = c6_b_n;
  c6_c_a = c6_b_a;
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  if (c6_c_n < 1) {
  } else if (c6_c_a == 0.0) {
  } else {
    c6_d_a = c6_c_ix0 - 1;
    c6_ix = c6_d_a;
    c6_e_a = c6_c_iy0 - 1;
    c6_iy = c6_e_a;
    c6_f_a = c6_c_n - 1;
    c6_i139 = c6_f_a;
    c6_b = c6_i139;
    c6_b_b = c6_b;
    if (0 > c6_b_b) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = (c6_b_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_k = 0; c6_k <= c6_i139; c6_k++) {
      c6_g_a = c6_iy;
      c6_c = c6_g_a;
      c6_h_a = c6_iy;
      c6_b_c = c6_h_a;
      c6_i_a = c6_ix;
      c6_c_c = c6_i_a;
      c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_c + 1)), 1, 31, 1, 0) - 1] =
        c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_b_c + 1)), 1, 31, 1, 0) - 1] + c6_c_a *
        c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_c_c + 1)), 1, 186, 1, 0) - 1];
      c6_j_a = c6_ix + 1;
      c6_ix = c6_j_a;
      c6_k_a = c6_iy + 1;
      c6_iy = c6_k_a;
    }
  }
}

static void c6_g_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, real_T c6_x[31], int32_T c6_ix0,
  real_T c6_y[186], int32_T c6_iy0)
{
  int32_T c6_b_n;
  real_T c6_b_a;
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_n;
  real_T c6_c_a;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_d_a;
  int32_T c6_ix;
  int32_T c6_e_a;
  int32_T c6_iy;
  int32_T c6_f_a;
  int32_T c6_i140;
  int32_T c6_b;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_g_a;
  int32_T c6_c;
  int32_T c6_h_a;
  int32_T c6_b_c;
  int32_T c6_i_a;
  int32_T c6_c_c;
  int32_T c6_j_a;
  int32_T c6_k_a;
  c6_b_n = c6_n;
  c6_b_a = c6_a;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_d_threshold(chartInstance);
  c6_c_n = c6_b_n;
  c6_c_a = c6_b_a;
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  if (c6_c_n < 1) {
  } else if (c6_c_a == 0.0) {
  } else {
    c6_d_a = c6_c_ix0 - 1;
    c6_ix = c6_d_a;
    c6_e_a = c6_c_iy0 - 1;
    c6_iy = c6_e_a;
    c6_f_a = c6_c_n - 1;
    c6_i140 = c6_f_a;
    c6_b = c6_i140;
    c6_b_b = c6_b;
    if (0 > c6_b_b) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = (c6_b_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_k = 0; c6_k <= c6_i140; c6_k++) {
      c6_g_a = c6_iy;
      c6_c = c6_g_a;
      c6_h_a = c6_iy;
      c6_b_c = c6_h_a;
      c6_i_a = c6_ix;
      c6_c_c = c6_i_a;
      c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_c + 1)), 1, 186, 1, 0) - 1] =
        c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_b_c + 1)), 1, 186, 1, 0) - 1] + c6_c_a *
        c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_c_c + 1)), 1, 31, 1, 0) - 1];
      c6_j_a = c6_ix + 1;
      c6_ix = c6_j_a;
      c6_k_a = c6_iy + 1;
      c6_iy = c6_k_a;
    }
  }
}

static void c6_h_eml_xaxpy(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_n, real_T c6_a, int32_T c6_ix0, real_T c6_y[36],
  int32_T c6_iy0)
{
  int32_T c6_b_n;
  real_T c6_b_a;
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_n;
  real_T c6_c_a;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_d_a;
  int32_T c6_ix;
  int32_T c6_e_a;
  int32_T c6_iy;
  int32_T c6_f_a;
  int32_T c6_i141;
  int32_T c6_b;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_g_a;
  int32_T c6_c;
  int32_T c6_h_a;
  int32_T c6_b_c;
  int32_T c6_i_a;
  int32_T c6_c_c;
  int32_T c6_j_a;
  int32_T c6_k_a;
  c6_b_n = c6_n;
  c6_b_a = c6_a;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_d_threshold(chartInstance);
  c6_c_n = c6_b_n;
  c6_c_a = c6_b_a;
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  if (c6_c_n < 1) {
  } else if (c6_c_a == 0.0) {
  } else {
    c6_d_a = c6_c_ix0 - 1;
    c6_ix = c6_d_a;
    c6_e_a = c6_c_iy0 - 1;
    c6_iy = c6_e_a;
    c6_f_a = c6_c_n - 1;
    c6_i141 = c6_f_a;
    c6_b = c6_i141;
    c6_b_b = c6_b;
    if (0 > c6_b_b) {
      c6_overflow = false;
    } else {
      c6_eml_switch_helper(chartInstance);
      c6_overflow = (c6_b_b > 2147483646);
    }

    if (c6_overflow) {
      c6_check_forloop_overflow_error(chartInstance, c6_overflow);
    }

    for (c6_k = 0; c6_k <= c6_i141; c6_k++) {
      c6_g_a = c6_iy;
      c6_c = c6_g_a;
      c6_h_a = c6_iy;
      c6_b_c = c6_h_a;
      c6_i_a = c6_ix;
      c6_c_c = c6_i_a;
      c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_c + 1)), 1, 36, 1, 0) - 1] =
        c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_b_c + 1)), 1, 36, 1, 0) - 1] + c6_c_a *
        c6_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c6_c_c + 1)), 1, 36, 1, 0) - 1];
      c6_j_a = c6_ix + 1;
      c6_ix = c6_j_a;
      c6_k_a = c6_iy + 1;
      c6_iy = c6_k_a;
    }
  }
}

static void c6_g_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_x[186], int32_T c6_ix0)
{
  real_T c6_b_a;
  int32_T c6_b_ix0;
  real_T c6_c_a;
  int32_T c6_c_ix0;
  int32_T c6_d_ix0;
  int32_T c6_d_a;
  int32_T c6_i142;
  int32_T c6_e_a;
  int32_T c6_b;
  int32_T c6_f_a;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_b_k;
  c6_b_a = c6_a;
  c6_b_ix0 = c6_ix0;
  c6_b_threshold(chartInstance);
  c6_c_a = c6_b_a;
  c6_c_ix0 = c6_b_ix0;
  c6_d_ix0 = c6_c_ix0;
  c6_d_a = c6_c_ix0 + 30;
  c6_i142 = c6_d_a;
  c6_e_a = c6_d_ix0;
  c6_b = c6_i142;
  c6_f_a = c6_e_a;
  c6_b_b = c6_b;
  if (c6_f_a > c6_b_b) {
    c6_overflow = false;
  } else {
    c6_eml_switch_helper(chartInstance);
    c6_overflow = (c6_b_b > 2147483646);
  }

  if (c6_overflow) {
    c6_check_forloop_overflow_error(chartInstance, c6_overflow);
  }

  for (c6_k = c6_d_ix0; c6_k <= c6_i142; c6_k++) {
    c6_b_k = c6_k;
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_b_k), 1, 186, 1, 0) - 1] = c6_c_a * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 186, 1, 0) - 1];
  }
}

static void c6_h_eml_xscal(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_a, real_T c6_x[36], int32_T c6_ix0)
{
  real_T c6_b_a;
  int32_T c6_b_ix0;
  real_T c6_c_a;
  int32_T c6_c_ix0;
  int32_T c6_d_ix0;
  int32_T c6_d_a;
  int32_T c6_i143;
  int32_T c6_e_a;
  int32_T c6_b;
  int32_T c6_f_a;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  int32_T c6_k;
  int32_T c6_b_k;
  c6_b_a = c6_a;
  c6_b_ix0 = c6_ix0;
  c6_b_threshold(chartInstance);
  c6_c_a = c6_b_a;
  c6_c_ix0 = c6_b_ix0;
  c6_d_ix0 = c6_c_ix0;
  c6_d_a = c6_c_ix0 + 5;
  c6_i143 = c6_d_a;
  c6_e_a = c6_d_ix0;
  c6_b = c6_i143;
  c6_f_a = c6_e_a;
  c6_b_b = c6_b;
  if (c6_f_a > c6_b_b) {
    c6_overflow = false;
  } else {
    c6_eml_switch_helper(chartInstance);
    c6_overflow = (c6_b_b > 2147483646);
  }

  if (c6_overflow) {
    c6_check_forloop_overflow_error(chartInstance, c6_overflow);
  }

  for (c6_k = c6_d_ix0; c6_k <= c6_i143; c6_k++) {
    c6_b_k = c6_k;
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_b_k), 1, 36, 1, 0) - 1] = c6_c_a * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_k), 1, 36, 1, 0) - 1];
  }
}

static void c6_b_sqrt(SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance,
                      real_T *c6_x)
{
  if (*c6_x < 0.0) {
    c6_c_eml_error(chartInstance);
  }

  *c6_x = muDoubleScalarSqrt(*c6_x);
}

static void c6_b_eml_xrotg(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T *c6_a, real_T *c6_b, real_T *c6_c, real_T *c6_s)
{
  real_T c6_b_a;
  real_T c6_b_b;
  real_T c6_c_b;
  real_T c6_c_a;
  real_T c6_d_a;
  real_T c6_d_b;
  real_T c6_e_b;
  real_T c6_e_a;
  real_T c6_b_c;
  real_T c6_b_s;
  double * c6_a_t;
  double * c6_b_t;
  double * c6_c_t;
  double * c6_s_t;
  real_T c6_c_c;
  real_T c6_c_s;
  (void)chartInstance;
  c6_b_a = *c6_a;
  c6_b_b = *c6_b;
  c6_c_b = c6_b_b;
  c6_c_a = c6_b_a;
  c6_d_a = c6_c_a;
  c6_d_b = c6_c_b;
  c6_e_b = c6_d_b;
  c6_e_a = c6_d_a;
  c6_b_c = 0.0;
  c6_b_s = 0.0;
  c6_a_t = (double *)(&c6_e_a);
  c6_b_t = (double *)(&c6_e_b);
  c6_c_t = (double *)(&c6_b_c);
  c6_s_t = (double *)(&c6_b_s);
  drotg(c6_a_t, c6_b_t, c6_c_t, c6_s_t);
  c6_c_a = c6_e_a;
  c6_c_b = c6_e_b;
  c6_c_c = c6_b_c;
  c6_c_s = c6_b_s;
  *c6_a = c6_c_a;
  *c6_b = c6_c_b;
  *c6_c = c6_c_c;
  *c6_s = c6_c_s;
}

static void c6_c_eml_xrot(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[36], int32_T c6_ix0, int32_T c6_iy0, real_T c6_c,
  real_T c6_s)
{
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  real_T c6_b_c;
  real_T c6_b_s;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  real_T c6_c_c;
  real_T c6_c_s;
  int32_T c6_ix;
  int32_T c6_iy;
  int32_T c6_k;
  real_T c6_temp;
  int32_T c6_a;
  int32_T c6_b_a;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_b_c = c6_c;
  c6_b_s = c6_s;
  c6_e_threshold(chartInstance);
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  c6_c_c = c6_b_c;
  c6_c_s = c6_b_s;
  c6_ix = c6_c_ix0;
  c6_iy = c6_c_iy0;
  for (c6_k = 1; c6_k < 7; c6_k++) {
    c6_temp = c6_c_c * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c6_ix), 1, 36, 1, 0) - 1] + c6_c_s *
      c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_iy), 1, 36, 1, 0) - 1];
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_iy), 1, 36, 1, 0) - 1] = c6_c_c * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_iy), 1, 36, 1, 0) - 1] - c6_c_s
      * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_ix), 1, 36, 1, 0) - 1];
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_ix), 1, 36, 1, 0) - 1] = c6_temp;
    c6_a = c6_iy + 1;
    c6_iy = c6_a;
    c6_b_a = c6_ix + 1;
    c6_ix = c6_b_a;
  }
}

static void c6_d_eml_xrot(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[186], int32_T c6_ix0, int32_T c6_iy0, real_T c6_c,
  real_T c6_s)
{
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  real_T c6_b_c;
  real_T c6_b_s;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  real_T c6_c_c;
  real_T c6_c_s;
  int32_T c6_ix;
  int32_T c6_iy;
  int32_T c6_k;
  real_T c6_temp;
  int32_T c6_a;
  int32_T c6_b_a;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_b_c = c6_c;
  c6_b_s = c6_s;
  c6_e_threshold(chartInstance);
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  c6_c_c = c6_b_c;
  c6_c_s = c6_b_s;
  c6_ix = c6_c_ix0;
  c6_iy = c6_c_iy0;
  for (c6_k = 1; c6_k < 32; c6_k++) {
    c6_temp = c6_c_c * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c6_ix), 1, 186, 1, 0) - 1] + c6_c_s *
      c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_iy), 1, 186, 1, 0) - 1];
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_iy), 1, 186, 1, 0) - 1] = c6_c_c * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_iy), 1, 186, 1, 0) - 1] -
      c6_c_s * c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c6_ix), 1, 186, 1, 0) - 1];
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_ix), 1, 186, 1, 0) - 1] = c6_temp;
    c6_a = c6_iy + 1;
    c6_iy = c6_a;
    c6_b_a = c6_ix + 1;
    c6_ix = c6_b_a;
  }
}

static void c6_c_eml_xswap(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[36], int32_T c6_ix0, int32_T c6_iy0)
{
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_ix;
  int32_T c6_iy;
  int32_T c6_k;
  real_T c6_temp;
  int32_T c6_a;
  int32_T c6_b_a;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_f_threshold(chartInstance);
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  c6_ix = c6_c_ix0;
  c6_iy = c6_c_iy0;
  for (c6_k = 1; c6_k < 7; c6_k++) {
    c6_temp = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c6_ix), 1, 36, 1, 0) - 1];
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_ix), 1, 36, 1, 0) - 1] = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c6_iy), 1, 36, 1, 0) - 1];
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_iy), 1, 36, 1, 0) - 1] = c6_temp;
    c6_a = c6_ix + 1;
    c6_ix = c6_a;
    c6_b_a = c6_iy + 1;
    c6_iy = c6_b_a;
  }
}

static void c6_d_eml_xswap(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, real_T c6_x[186], int32_T c6_ix0, int32_T c6_iy0)
{
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_ix;
  int32_T c6_iy;
  int32_T c6_k;
  real_T c6_temp;
  int32_T c6_a;
  int32_T c6_b_a;
  c6_b_ix0 = c6_ix0;
  c6_b_iy0 = c6_iy0;
  c6_f_threshold(chartInstance);
  c6_c_ix0 = c6_b_ix0;
  c6_c_iy0 = c6_b_iy0;
  c6_ix = c6_c_ix0;
  c6_iy = c6_c_iy0;
  for (c6_k = 1; c6_k < 32; c6_k++) {
    c6_temp = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c6_ix), 1, 186, 1, 0) - 1];
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_ix), 1, 186, 1, 0) - 1] = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c6_iy), 1, 186, 1, 0) - 1];
    c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c6_iy), 1, 186, 1, 0) - 1] = c6_temp;
    c6_a = c6_ix + 1;
    c6_ix = c6_a;
    c6_b_a = c6_iy + 1;
    c6_iy = c6_b_a;
  }
}

static void c6_b_eml_xgemm(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance, int32_T c6_k, real_T c6_A[36], real_T c6_B[186], real_T c6_C
  [186])
{
  int32_T c6_b_k;
  int32_T c6_c_k;
  real_T c6_alpha1;
  real_T c6_beta1;
  char_T c6_TRANSB;
  char_T c6_TRANSA;
  ptrdiff_t c6_m_t;
  ptrdiff_t c6_n_t;
  int32_T c6_var;
  ptrdiff_t c6_k_t;
  ptrdiff_t c6_lda_t;
  ptrdiff_t c6_ldb_t;
  ptrdiff_t c6_ldc_t;
  double * c6_alpha1_t;
  double * c6_Aia0_t;
  double * c6_Bib0_t;
  double * c6_beta1_t;
  double * c6_Cic0_t;
  c6_b_k = c6_k;
  c6_below_threshold(chartInstance);
  if (c6_b_k < 1) {
  } else {
    c6_c_k = c6_b_k;
    c6_alpha1 = 1.0;
    c6_beta1 = 0.0;
    c6_TRANSB = 'C';
    c6_TRANSA = 'N';
    c6_m_t = (ptrdiff_t)(6);
    c6_n_t = (ptrdiff_t)(31);
    c6_var = c6_c_k;
    c6_k_t = (ptrdiff_t)(c6_var);
    c6_lda_t = (ptrdiff_t)(6);
    c6_ldb_t = (ptrdiff_t)(31);
    c6_ldc_t = (ptrdiff_t)(6);
    c6_alpha1_t = (double *)(&c6_alpha1);
    c6_Aia0_t = (double *)(&c6_A[0]);
    c6_Bib0_t = (double *)(&c6_B[0]);
    c6_beta1_t = (double *)(&c6_beta1);
    c6_Cic0_t = (double *)(&c6_C[0]);
    dgemm(&c6_TRANSA, &c6_TRANSB, &c6_m_t, &c6_n_t, &c6_k_t, c6_alpha1_t,
          c6_Aia0_t, &c6_lda_t, c6_Bib0_t, &c6_ldb_t, c6_beta1_t, c6_Cic0_t,
          &c6_ldc_t);
  }
}

static void init_dsm_address_info(SFc6_dirdyna_walkman_robotranInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c6_dirdyna_walkman_robotran_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3497131476U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2709864304U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(45574379U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3408811374U);
}

mxArray *sf_c6_dirdyna_walkman_robotran_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("FifxjyaWlBuBe6bcejF1ZF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(31);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(31);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c6_dirdyna_walkman_robotran_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c6_dirdyna_walkman_robotran_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c6_dirdyna_walkman_robotran(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[13],T\"q_dot\",},{M[8],M[0],T\"is_active_c6_dirdyna_walkman_robotran\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_dirdyna_walkman_robotran_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _dirdyna_walkman_robotranMachineNumber_,
           6,
           1,
           1,
           0,
           5,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_dirdyna_walkman_robotranMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_dirdyna_walkman_robotranMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _dirdyna_walkman_robotranMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"J");
          _SFD_SET_DATA_PROPS(1,2,0,1,"q_dot");
          _SFD_SET_DATA_PROPS(2,1,1,0,"ref_dot");
          _SFD_SET_DATA_PROPS(3,1,1,0,"K");
          _SFD_SET_DATA_PROPS(4,1,1,0,"e");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,246);

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 31;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)
            c6_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c6_J)[186];
          real_T (*c6_q_dot)[31];
          real_T (*c6_ref_dot)[3];
          real_T (*c6_K)[9];
          real_T (*c6_e)[3];
          c6_e = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
          c6_K = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 2);
          c6_ref_dot = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c6_q_dot = (real_T (*)[31])ssGetOutputPortSignal(chartInstance->S, 1);
          c6_J = (real_T (*)[186])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c6_J);
          _SFD_SET_DATA_VALUE_PTR(1U, *c6_q_dot);
          _SFD_SET_DATA_VALUE_PTR(2U, *c6_ref_dot);
          _SFD_SET_DATA_VALUE_PTR(3U, *c6_K);
          _SFD_SET_DATA_VALUE_PTR(4U, *c6_e);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _dirdyna_walkman_robotranMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "s6Kfj7r5yRNCy3e7vWyS0C";
}

static void sf_opaque_initialize_c6_dirdyna_walkman_robotran(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_dirdyna_walkman_robotranInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c6_dirdyna_walkman_robotran
    ((SFc6_dirdyna_walkman_robotranInstanceStruct*) chartInstanceVar);
  initialize_c6_dirdyna_walkman_robotran
    ((SFc6_dirdyna_walkman_robotranInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c6_dirdyna_walkman_robotran(void *chartInstanceVar)
{
  enable_c6_dirdyna_walkman_robotran
    ((SFc6_dirdyna_walkman_robotranInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c6_dirdyna_walkman_robotran(void *chartInstanceVar)
{
  disable_c6_dirdyna_walkman_robotran
    ((SFc6_dirdyna_walkman_robotranInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c6_dirdyna_walkman_robotran(void *chartInstanceVar)
{
  sf_gateway_c6_dirdyna_walkman_robotran
    ((SFc6_dirdyna_walkman_robotranInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_dirdyna_walkman_robotran
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_dirdyna_walkman_robotran
    ((SFc6_dirdyna_walkman_robotranInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_dirdyna_walkman_robotran();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c6_dirdyna_walkman_robotran(SimStruct* S,
  const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c6_dirdyna_walkman_robotran();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_dirdyna_walkman_robotran
    ((SFc6_dirdyna_walkman_robotranInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_dirdyna_walkman_robotran
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c6_dirdyna_walkman_robotran(S);
}

static void sf_opaque_set_sim_state_c6_dirdyna_walkman_robotran(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c6_dirdyna_walkman_robotran(S, st);
}

static void sf_opaque_terminate_c6_dirdyna_walkman_robotran(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_dirdyna_walkman_robotranInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_dirdyna_walkman_robotran_optimization_info();
    }

    finalize_c6_dirdyna_walkman_robotran
      ((SFc6_dirdyna_walkman_robotranInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc6_dirdyna_walkman_robotran
    ((SFc6_dirdyna_walkman_robotranInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_dirdyna_walkman_robotran(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c6_dirdyna_walkman_robotran
      ((SFc6_dirdyna_walkman_robotranInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_dirdyna_walkman_robotran(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_dirdyna_walkman_robotran_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,6,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,6);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4198198213U));
  ssSetChecksum1(S,(4014371058U));
  ssSetChecksum2(S,(4273981737U));
  ssSetChecksum3(S,(3555259573U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c6_dirdyna_walkman_robotran(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_dirdyna_walkman_robotran(SimStruct *S)
{
  SFc6_dirdyna_walkman_robotranInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc6_dirdyna_walkman_robotranInstanceStruct *)utMalloc(sizeof
    (SFc6_dirdyna_walkman_robotranInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_dirdyna_walkman_robotranInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c6_dirdyna_walkman_robotran;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c6_dirdyna_walkman_robotran_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_dirdyna_walkman_robotran(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_dirdyna_walkman_robotran(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_dirdyna_walkman_robotran(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_dirdyna_walkman_robotran_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
