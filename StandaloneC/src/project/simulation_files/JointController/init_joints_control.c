//---------------------------
// Timothee Habra
// Danilo Caporale (Torque PID tuning... sob)
//
// Creation : 8-4-2015
// Last update : 8-4-2015
//
// Last update: 26-4-2017
// Initialize the low level joints controller (motor control board)
//
//---------------------------

#include "user_all_id.h"
#include "ActuatorsDefinitions.h"
#include "MBSdataStruct.h"
#include "simu_def.h"
#include "diapason.h"

void load_torque_gains()
{
    int ret;
  char gain_name[10];
  float gain_value;
  FILE *fp = fopen("torque_pid_gains.txt","r");
  if (fp==NULL)
    {
      printf("Error while reading torque_pid_gains.txt\n");
      return -1;
    }

  while(!feof(fp))
  {
    ret = fscanf(fp,"%s %f",gain_name,&gain_value);
    printf("letto: %s %f\n",gain_name,gain_value);

    // There sure is a better way to do this...
    // HIP_SAG
    if(strcmp(gain_name,"P_HIP_SAG")==0)
    {
      P_HIP_SAG = gain_value;
    }
    if(strcmp(gain_name,"I_HIP_SAG")==0)
    {
      I_HIP_SAG = gain_value;
    }
    if(strcmp(gain_name,"D_HIP_SAG")==0)
    {
      D_HIP_SAG = gain_value;
    }

        // HIP_LAT
    if(strcmp(gain_name,"P_HIP_LAT")==0)
    {
      P_HIP_LAT = gain_value;
    }
    if(strcmp(gain_name,"I_HIP_LAT")==0)
    {
      I_HIP_LAT = gain_value;
    }
    if(strcmp(gain_name,"D_HIP_LAT")==0)
    {
      D_HIP_LAT = gain_value;
    }

        // HIP_TRANS
    if(strcmp(gain_name,"P_HIP_TRANS")==0)
    {
      P_HIP_TRANS = gain_value;
    }
    if(strcmp(gain_name,"I_HIP_TRANS")==0)
    {
      I_HIP_TRANS = gain_value;
    }
    if(strcmp(gain_name,"D_HIP_TRANS")==0)
    {
      D_HIP_TRANS = gain_value;
    }

    // KNEE_SAG
    if(strcmp(gain_name,"P_KNEE_SAG")==0)
    {
      P_KNEE_SAG = gain_value;
    }
    if(strcmp(gain_name,"I_KNEE_SAG")==0)
    {
      I_KNEE_SAG = gain_value;
    }
    if(strcmp(gain_name,"D_KNEE_SAG")==0)
    {
      D_KNEE_SAG = gain_value;
    }

        // ANK_LAT
    if(strcmp(gain_name,"P_ANK_LAT")==0)
    {
      P_ANK_LAT = gain_value;
    }
    if(strcmp(gain_name,"I_ANK_LAT")==0)
    {
      I_ANK_LAT = gain_value;
    }
    if(strcmp(gain_name,"D_ANK_LAT")==0)
    {
      D_ANK_LAT = gain_value;
    }

    // ANK_SAG
    if(strcmp(gain_name,"P_ANK_SAG")==0)
    {
      P_ANK_SAG = gain_value;
    }
    if(strcmp(gain_name,"I_ANK_SAG")==0)
    {
      I_ANK_SAG = gain_value;
    }
    if(strcmp(gain_name,"D_ANK_SAG")==0)
    {
      D_ANK_SAG = gain_value;
    }

// torso
            // WAIST_LAT
    if(strcmp(gain_name,"P_WAIST_LAT")==0)
    {
      P_WAIST_LAT = gain_value;
    }
    if(strcmp(gain_name,"I_WAIST_LAT")==0)
    {
      I_WAIST_LAT = gain_value;
    }
    if(strcmp(gain_name,"D_WAIST_LAT")==0)
    {
      D_WAIST_LAT = gain_value;
    }

                // WAIST_SAG
    if(strcmp(gain_name,"P_WAIST_SAG")==0)
    {
      P_WAIST_SAG = gain_value;
    }
    if(strcmp(gain_name,"I_WAIST_SAG")==0)
    {
      I_WAIST_SAG = gain_value;
    }
    if(strcmp(gain_name,"D_WAIST_SAG")==0)
    {
      D_WAIST_SAG = gain_value;
    }

                    // WAIST_TRANS
    if(strcmp(gain_name,"P_WAIST_TRANS")==0)
    {
      P_WAIST_TRANS = gain_value;
    }
    if(strcmp(gain_name,"I_WAIST_TRANS")==0)
    {
      I_WAIST_TRANS = gain_value;
    }
    if(strcmp(gain_name,"D_WAIST_TRANS")==0)
    {
      D_WAIST_TRANS = gain_value;
    }

// ARMS
                        // SH_SAG
    if(strcmp(gain_name,"P_SH_SAG")==0)
    {
      P_SH_SAG = gain_value;
    }
    if(strcmp(gain_name,"I_SH_SAG")==0)
    {
      I_SH_SAG = gain_value;
    }
    if(strcmp(gain_name,"D_SH_SAG")==0)
    {
      D_SH_SAG = gain_value;
    }

                        // SH_LAT
    if(strcmp(gain_name,"P_SH_LAT")==0)
    {
      P_SH_LAT = gain_value;
    }
    if(strcmp(gain_name,"I_SH_LAT")==0)
    {
      I_SH_LAT = gain_value;
    }
    if(strcmp(gain_name,"D_SH_LAT")==0)
    {
      D_SH_LAT = gain_value;
    }

                        // SH_TRANS
    if(strcmp(gain_name,"P_SH_TRANS")==0)
    {
      P_SH_TRANS = gain_value;
    }
    if(strcmp(gain_name,"I_SH_TRANS")==0)
    {
      I_SH_TRANS = gain_value;
    }
    if(strcmp(gain_name,"D_SH_TRANS")==0)
    {
      D_SH_TRANS = gain_value;
    }

                       // ELB
    if(strcmp(gain_name,"P_ELB")==0)
    {
      P_ELB = gain_value;
    }
    if(strcmp(gain_name,"I_ELB")==0)
    {
      I_ELB = gain_value;
    }
    if(strcmp(gain_name,"D_ELB")==0)
    {
      D_ELB = gain_value;
    }

                           // FORE_ARM_PLATE
    if(strcmp(gain_name,"P_FORE_ARM_PLATE")==0)
    {
      P_FORE_ARM_PLATE = gain_value;
    }
    if(strcmp(gain_name,"I_FORE_ARM_PLATE")==0)
    {
      I_FORE_ARM_PLATE = gain_value;
    }
    if(strcmp(gain_name,"D_FORE_ARM_PLATE")==0)
    {
      D_FORE_ARM_PLATE = gain_value;
    }

                               // WRJ1
    if(strcmp(gain_name,"P_WRJ1")==0)
    {
      P_WRJ1 = gain_value;
    }
    if(strcmp(gain_name,"I_WRJ1")==0)
    {
      I_WRJ1 = gain_value;
    }
    if(strcmp(gain_name,"D_WRJ1")==0)
    {
      D_WRJ1 = gain_value;
    }

                                   // WRJ2
    if(strcmp(gain_name,"P_WRJ2")==0)
    {
      P_WRJ2 = gain_value;
    }
    if(strcmp(gain_name,"I_WRJ2")==0)
    {
      I_WRJ2 = gain_value;
    }
    if(strcmp(gain_name,"D_WRJ2")==0)
    {
      D_WRJ2 = gain_value;
    }

                                   // NECK_YAW
    if(strcmp(gain_name,"P_NECK_YAW")==0)
    {
      P_NECK_YAW = gain_value;
    }
    if(strcmp(gain_name,"I_NECK_YAW")==0)
    {
      I_NECK_YAW = gain_value;
    }
    if(strcmp(gain_name,"D_NECK_YAW")==0)
    {
      D_NECK_YAW = gain_value;
    }

                                   // NECK_PITCH
    if(strcmp(gain_name,"P_NECK_PITCH")==0)
    {
      P_NECK_PITCH = gain_value;
    }
    if(strcmp(gain_name,"I_NECK_PITCH")==0)
    {
      I_NECK_PITCH = gain_value;
    }
    if(strcmp(gain_name,"D_NECK_PITCH")==0)
    {
      D_NECK_PITCH = gain_value;
    }
  }

  fclose(fp);
}

// Initializes joints controller (PID, references, ...)
void init_joints_control(MBSdataStruct *MBSdata)
{

    int i,j;
    UserIOStruct *uvs;
    ControllerStruct *cvs;

    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;

    /******************************
    // servo type (position control, torque control, impedance control)
    /******************************/
    // e.g. uvs->servo_type[R_HIP_LAT_MOT+1] = POSITION_CTRL;

    for (i = 0; i < COMAN_NB_JOINT_ACTUATED; i++)
	{
         cvs->Outputs->servo_type[i+1] = POSITION_CTRL;  // position control by default
	}

	#ifndef STANDALONE
	MBSdata->q0   = (double*) calloc(MBSdata->njoint+1,sizeof(double));
	MBSdata->qd0  = (double*) calloc(MBSdata->njoint+1,sizeof(double));
	MBSdata->qdd0 = (double*) calloc(MBSdata->njoint+1,sizeof(double));
	MBSdata->q0[0]   = (double) MBSdata->njoint;
	MBSdata->qd0[0]  = (double) MBSdata->njoint;
	MBSdata->qdd0[0] = (double) MBSdata->njoint;
	for(i=1;i<=MBSdata->njoint;i++)
	{
		MBSdata->q0[i]   = MBSdata->q[i];
		MBSdata->qd0[i]  = 0.;
		MBSdata->qdd0[i] = 0.;
	}
	#endif


    /******************************
    // references (position, torque)
    /******************************/

    for (i = 0; i < COMAN_NB_JOINT_ACTUATED; i++)
    {
        cvs->Outputs->q_ref[i+1]  = MBSdata->q0[uvs->actuatorsStruct->JointIds[i]];  // MBS initial position
        cvs->Outputs->Qq_ref[i+1] = 0.;  // zero
    }


    /******************************
    // position control pid
    /******************************/

    // legs
    i = R_HIP_SAG_MOT;
    j = L_HIP_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 240.;
    uvs->PIDs_pos->i[i+1] = 0.;
    uvs->PIDs_pos->d[i+1] = 0.;

    i = R_HIP_LAT_MOT;
    j = L_HIP_LAT_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 52.;

    i = R_HIP_TRANS_MOT;
    j = L_HIP_TRANS_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 12.;

    i = R_KNEE_SAG_MOT;
    j = L_KNEE_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 300.;

    i = R_ANK_LAT_MOT;
    j = L_ANK_LAT_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 250.;

    i = R_ANK_SAG_MOT;
    j = L_ANK_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 280.;


    // torso
    i = WAIST_LAT_MOT;
    uvs->PIDs_pos->p[i+1] = 40.;

    i = WAIST_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = 50.;

    i = WAIST_TRANS_MOT;
    uvs->PIDs_pos->p[i+1] = 40.;

    // arms
    i = R_SH_SAG_MOT;
    j = L_SH_SAG_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    i = R_SH_LAT_MOT;
    j = L_SH_LAT_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 80.;

    i = R_SH_TRANS_MOT;
    j = L_SH_TRANS_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    i = R_ELB_MOT;
    j = L_ELB_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 40.;

    i = R_FORE_ARM_PLATE_MOT;
    j = L_FORE_ARM_PLATE_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    i = R_WRJ1_MOT;
    j = L_WRJ1_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    i = R_WRJ2_MOT;
    j = L_WRJ2_MOT;
    uvs->PIDs_pos->p[i+1] = uvs->PIDs_pos->p[j+1] = 10.;

    // head
    i = NECK_YAW_MOT;
    uvs->PIDs_pos->p[i+1] = 10.;

    i = NECK_PITCH_MOT;
    uvs->PIDs_pos->p[i+1] = 10.;

    /******************************
    // Torque control pid
    /******************************/

    // Gains for each joint are defined here    
    load_torque_gains();
    // legs
    i = R_HIP_SAG_MOT;
    j = L_HIP_SAG_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_HIP_SAG;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_HIP_SAG;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_HIP_SAG;

    i = R_HIP_LAT_MOT;
    j = L_HIP_LAT_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_HIP_LAT;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_HIP_LAT;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_HIP_LAT;

    i = R_HIP_TRANS_MOT;
    j = L_HIP_TRANS_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_HIP_TRANS;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_HIP_TRANS;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_HIP_TRANS;
    
    i = R_KNEE_SAG_MOT;
    j = L_KNEE_SAG_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_KNEE_SAG;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_KNEE_SAG;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_KNEE_SAG;


    i = R_ANK_LAT_MOT;
    j = L_ANK_LAT_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_ANK_LAT;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_ANK_LAT;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_ANK_LAT;
    
    i = R_ANK_SAG_MOT;
    j = L_ANK_SAG_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_ANK_SAG;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_ANK_SAG;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_ANK_SAG;

    // torso
    i = WAIST_LAT_MOT;
    uvs->PIDs_torque->p[i+1] = P_WAIST_LAT;
    uvs->PIDs_torque->i[i+1] = I_WAIST_LAT;
    uvs->PIDs_torque->d[i+1] = D_WAIST_LAT;
    
    i = WAIST_SAG_MOT;
    uvs->PIDs_torque->p[i+1] = P_WAIST_SAG;
    uvs->PIDs_torque->i[i+1] = I_WAIST_SAG;
    uvs->PIDs_torque->d[i+1] = D_WAIST_SAG;
    
    i = WAIST_TRANS_MOT;
    uvs->PIDs_torque->p[i+1] = P_WAIST_TRANS;
    uvs->PIDs_torque->i[i+1] = I_WAIST_TRANS;
    uvs->PIDs_torque->d[i+1] = D_WAIST_TRANS;
    
    // arms
    i = R_SH_SAG_MOT;
    j = L_SH_SAG_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_SH_SAG;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_SH_SAG;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_SH_SAG;
    
    i = R_SH_LAT_MOT;
    j = L_SH_LAT_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_SH_LAT;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_SH_LAT;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_SH_LAT;
    
    i = R_SH_TRANS_MOT;
    j = L_SH_TRANS_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_SH_TRANS;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_SH_TRANS;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_SH_TRANS;
    
    i = R_ELB_MOT;
    j = L_ELB_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_ELB;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_ELB;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_ELB;
    
    i = R_FORE_ARM_PLATE_MOT;
    j = L_FORE_ARM_PLATE_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_FORE_ARM_PLATE;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_FORE_ARM_PLATE;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_FORE_ARM_PLATE;
    
    i = R_WRJ1_MOT;
    j = L_WRJ1_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_WRJ1;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_WRJ1;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_WRJ1;
    
    i = R_WRJ2_MOT;
    j = L_WRJ2_MOT;
    uvs->PIDs_torque->p[i+1] = uvs->PIDs_torque->p[j+1] = P_WRJ2;
    uvs->PIDs_torque->i[i+1] = uvs->PIDs_torque->i[j+1] = I_WRJ2;
    uvs->PIDs_torque->d[i+1] = uvs->PIDs_torque->d[j+1] = D_WRJ2;
    
    // head
    i = NECK_YAW_MOT;
    uvs->PIDs_torque->p[i+1] = P_NECK_YAW;
    uvs->PIDs_torque->i[i+1] = I_NECK_YAW;
    uvs->PIDs_torque->d[i+1] = D_NECK_YAW;
    
    i = NECK_PITCH_MOT;
    uvs->PIDs_torque->p[i+1] = P_NECK_PITCH;
    uvs->PIDs_torque->i[i+1] = I_NECK_PITCH;
    uvs->PIDs_torque->d[i+1] = D_NECK_PITCH;
    
    /******************************
    // Impedance control pid
    /******************************/

    // TBD
    i = R_HIP_SAG_MOT;
    uvs->k_stiff[i+1] = 0.;
    uvs->k_damp[i+1] = 0.;

    
}
