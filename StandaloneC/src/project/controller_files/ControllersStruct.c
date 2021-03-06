//---------------------------
// Nicolas Van der Noot
//
// Creation : 24-Jan-2014
// Last update : Sat Nov  7 16:13:17 2015
//---------------------------

#include <stdlib.h>

#include "ControllersStruct.h"


// ---- Controllers initialization ---- //
 
// ControllerInputs
ControllerInputs * init_ControllerInputs(void)
{
    ControllerInputs *cvs;

    int i;

    cvs = (ControllerInputs*) malloc(sizeof(ControllerInputs));

    cvs->t = 0.0;

    for (i=0;i<31;i++)
    {
        cvs->q[i] = 0.0;
    }

    for (i=0;i<31;i++)
    {
        cvs->qd[i] = 0.0;
    }

    for (i=0;i<31;i++)
    {
        cvs->Qq[i] = 0.0;
    }

    for (i=0;i<31;i++)
    {
        cvs->q_mot[i] = 0.0;
    }

    for (i=0;i<31;i++)
    {
        cvs->qd_mot[i] = 0.0;
    }

    for (i=0;i<3;i++)
    {
        cvs->F_Rfoot[i] = 0.0;
    }

    for (i=0;i<3;i++)
    {
        cvs->F_Lfoot[i] = 0.0;
    }

    for (i=0;i<3;i++)
    {
        cvs->T_Rfoot[i] = 0.0;
    }

    for (i=0;i<3;i++)
    {
        cvs->T_Lfoot[i] = 0.0;
    }

    for (i=0;i<2;i++)
    {
        cvs->FT_sensor_list[i] = init_FT_sensor_Struct();
    }

    for (i=0;i<2;i++)
    {
        cvs->IMU_list[i] = init_IMU_Struct();
    }

    return cvs;
}

// ControllerOutputs
ControllerOutputs * init_ControllerOutputs(void)
{
    ControllerOutputs *cvs;

    int i;

    cvs = (ControllerOutputs*) malloc(sizeof(ControllerOutputs));

    for (i=0;i<31+1;i++)
    {
        cvs->q_ref[i] = 0.0;
    }

    for (i=0;i<31+1;i++)
    {
        cvs->qd_ref[i] = 0.0;
    }

    for (i=0;i<31+1;i++)
    {
        cvs->Qq_ref[i] = 0.0;
    }

    for (i=0;i<31+1;i++)
    {
        cvs->servo_type[i] = 0;
    }

    return cvs;
}

// ControllerStruct
ControllerStruct * init_ControllerStruct(void)
{
    ControllerStruct *cvs;

    int i;

    cvs = (ControllerStruct*) malloc(sizeof(ControllerStruct));

    cvs->Inputs = init_ControllerInputs();

    cvs->Outputs = init_ControllerOutputs();

    for (i=0;i<20;i++)
    {
        cvs->out[i] = 0.0;
    }

    cvs->q_ref_r_sh_sag = 0.0;

    cvs->q_ref_r_sh_lat = 0.0;

    cvs->q_ref_r_sh_yaw = 0.0;

    cvs->q_ref_r_elb = 0.0;

    cvs->q_ref_l_sh_sag = 0.0;

    cvs->q_ref_l_sh_lat = 0.0;

    cvs->q_ref_l_sh_yaw = 0.0;

    cvs->q_ref_l_elb = 0.0;

    return cvs;
}

// ---- Controllers: free ---- //

// ControllerInputs
void free_ControllerInputs(ControllerInputs *cvs)
{
    int i;

    for (i=0;i<2;i++)
    {
        free_FT_sensor_Struct(cvs->FT_sensor_list[i]);
    }

    for (i=0;i<2;i++)
    {
        free_IMU_Struct(cvs->IMU_list[i]);
    }

    free(cvs);
}

// ControllerOutputs
void free_ControllerOutputs(ControllerOutputs *cvs)
{
    free(cvs);
}

// ControllerStruct
void free_ControllerStruct(ControllerStruct *cvs)
{
    free_ControllerInputs(cvs->Inputs);

    free_ControllerOutputs(cvs->Outputs);

    free(cvs);
}

