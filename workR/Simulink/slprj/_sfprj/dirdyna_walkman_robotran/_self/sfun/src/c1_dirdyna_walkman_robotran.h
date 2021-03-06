#ifndef __c1_dirdyna_walkman_robotran_h__
#define __c1_dirdyna_walkman_robotran_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_struct_gv6kCpleLWxYRfe8hNnr9B_tag
#define struct_struct_gv6kCpleLWxYRfe8hNnr9B_tag

struct struct_gv6kCpleLWxYRfe8hNnr9B_tag
{
  real_T q[55];
  real_T qd[55];
  real_T qdd[55];
  real_T m[55];
  real_T g[3];
  real_T l[165];
  real_T In[495];
  real_T dpt[171];
  real_T frc[165];
  real_T trq[165];
};

#endif                                 /*struct_struct_gv6kCpleLWxYRfe8hNnr9B_tag*/

#ifndef typedef_c1_struct_gv6kCpleLWxYRfe8hNnr9B
#define typedef_c1_struct_gv6kCpleLWxYRfe8hNnr9B

typedef struct struct_gv6kCpleLWxYRfe8hNnr9B_tag
  c1_struct_gv6kCpleLWxYRfe8hNnr9B;

#endif                                 /*typedef_c1_struct_gv6kCpleLWxYRfe8hNnr9B*/

#ifndef typedef_SFc1_dirdyna_walkman_robotranInstanceStruct
#define typedef_SFc1_dirdyna_walkman_robotranInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_dirdyna_walkman_robotran;
  c1_struct_gv6kCpleLWxYRfe8hNnr9B c1_s;
  real_T c1_actuated_joints_idx[31];
} SFc1_dirdyna_walkman_robotranInstanceStruct;

#endif                                 /*typedef_SFc1_dirdyna_walkman_robotranInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_dirdyna_walkman_robotran_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_dirdyna_walkman_robotran_get_check_sum(mxArray *plhs[]);
extern void c1_dirdyna_walkman_robotran_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
