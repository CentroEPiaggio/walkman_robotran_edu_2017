/* Include files */

#include "dirdyna_walkman_robotran_sfun.h"
#include "dirdyna_walkman_robotran_sfun_debug_macros.h"
#include "c1_dirdyna_walkman_robotran.h"
#include "c3_dirdyna_walkman_robotran.h"
#include "c6_dirdyna_walkman_robotran.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _dirdyna_walkman_robotranMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void dirdyna_walkman_robotran_initializer(void)
{
}

void dirdyna_walkman_robotran_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_dirdyna_walkman_robotran_method_dispatcher(SimStruct
  *simstructPtr, unsigned int chartFileNumber, const char* specsCksum, int_T
  method, void *data)
{
  if (chartFileNumber==1) {
    c1_dirdyna_walkman_robotran_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_dirdyna_walkman_robotran_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_dirdyna_walkman_robotran_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_dirdyna_walkman_robotran_process_check_sum_call( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(185756893U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3550408847U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2849529822U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3419492271U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2437751677U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(772913958U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2485815682U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2175339265U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_dirdyna_walkman_robotran_get_check_sum(mxArray *
            plhs[]);
          sf_c1_dirdyna_walkman_robotran_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_dirdyna_walkman_robotran_get_check_sum(mxArray *
            plhs[]);
          sf_c3_dirdyna_walkman_robotran_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_dirdyna_walkman_robotran_get_check_sum(mxArray *
            plhs[]);
          sf_c6_dirdyna_walkman_robotran_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2083502392U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1110276785U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3258378658U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3926592909U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3762418336U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2021718854U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(795036281U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3446709164U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_dirdyna_walkman_robotran_autoinheritance_info( int nlhs, mxArray
  * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "lrlgr0nQuiaMNT5nZQb5hF") == 0) {
          extern mxArray
            *sf_c1_dirdyna_walkman_robotran_get_autoinheritance_info(void);
          plhs[0] = sf_c1_dirdyna_walkman_robotran_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "GvFoSfTf4tkoc1G3tdEKzE") == 0) {
          extern mxArray
            *sf_c3_dirdyna_walkman_robotran_get_autoinheritance_info(void);
          plhs[0] = sf_c3_dirdyna_walkman_robotran_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "FifxjyaWlBuBe6bcejF1ZF") == 0) {
          extern mxArray
            *sf_c6_dirdyna_walkman_robotran_get_autoinheritance_info(void);
          plhs[0] = sf_c6_dirdyna_walkman_robotran_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_dirdyna_walkman_robotran_get_eml_resolved_functions_info( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray
          *sf_c1_dirdyna_walkman_robotran_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_dirdyna_walkman_robotran_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_dirdyna_walkman_robotran_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_dirdyna_walkman_robotran_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_dirdyna_walkman_robotran_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_dirdyna_walkman_robotran_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_dirdyna_walkman_robotran_third_party_uses_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "MWoORVvjERRzIUO3f3upBE") == 0) {
          extern mxArray *sf_c1_dirdyna_walkman_robotran_third_party_uses_info
            (void);
          plhs[0] = sf_c1_dirdyna_walkman_robotran_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "hLNZZb063HmvdpNMqdf0j") == 0) {
          extern mxArray *sf_c3_dirdyna_walkman_robotran_third_party_uses_info
            (void);
          plhs[0] = sf_c3_dirdyna_walkman_robotran_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "s6Kfj7r5yRNCy3e7vWyS0C") == 0) {
          extern mxArray *sf_c6_dirdyna_walkman_robotran_third_party_uses_info
            (void);
          plhs[0] = sf_c6_dirdyna_walkman_robotran_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_dirdyna_walkman_robotran_updateBuildInfo_args_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "MWoORVvjERRzIUO3f3upBE") == 0) {
          extern mxArray
            *sf_c1_dirdyna_walkman_robotran_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_dirdyna_walkman_robotran_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "hLNZZb063HmvdpNMqdf0j") == 0) {
          extern mxArray
            *sf_c3_dirdyna_walkman_robotran_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_dirdyna_walkman_robotran_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "s6Kfj7r5yRNCy3e7vWyS0C") == 0) {
          extern mxArray
            *sf_c6_dirdyna_walkman_robotran_updateBuildInfo_args_info(void);
          plhs[0] = sf_c6_dirdyna_walkman_robotran_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void dirdyna_walkman_robotran_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _dirdyna_walkman_robotranMachineNumber_ = sf_debug_initialize_machine
    (debugInstance,"dirdyna_walkman_robotran","sfun",0,3,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _dirdyna_walkman_robotranMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _dirdyna_walkman_robotranMachineNumber_,0);
}

void dirdyna_walkman_robotran_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_dirdyna_walkman_robotran_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "dirdyna_walkman_robotran", "dirdyna_walkman_robotran");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_dirdyna_walkman_robotran_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
