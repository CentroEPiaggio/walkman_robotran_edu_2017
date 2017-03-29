function q_ref = home_position_control(u)
%#codegen

q_ref= zeros(31,1);

%% 

% motor index (as definined in ActuatorsDefinition.h)

% right leg
R_HIP_LAT_MOT   = 0;
R_HIP_TRANS_MOT = 1;
R_HIP_SAG_MOT   = 2;
R_KNEE_SAG_MOT  = 3;
R_ANK_SAG_MOT   = 4;
R_ANK_LAT_MOT   = 5;

%  left leg
L_HIP_LAT_MOT   = 6;
L_HIP_TRANS_MOT = 7;
L_HIP_SAG_MOT   = 8;
L_KNEE_SAG_MOT  = 9;
L_ANK_SAG_MOT   = 10;
L_ANK_LAT_MOT   = 11;

% waist
WAIST_LAT_MOT   = 12;
WAIST_SAG_MOT   = 13;
WAIST_TRANS_MOT = 14;

% right arm
R_SH_SAG_MOT            = 15;
R_SH_LAT_MOT            = 16;
R_SH_TRANS_MOT          = 17;
R_ELB_MOT               = 18;
R_FORE_ARM_PLATE_MOT    = 19;
R_WRJ1_MOT              = 20;
R_WRJ2_MOT              = 21;

% left arm
L_SH_SAG_MOT            = 22;
L_SH_LAT_MOT            = 23;
L_SH_TRANS_MOT          = 24;
L_ELB_MOT               = 25;
L_FORE_ARM_PLATE_MOT    = 26;
L_WRJ1_MOT              = 27;
L_WRJ2_MOT              = 28;


%% 


% set homing references position (express in radian with Robotran convention)

q_ref(R_HIP_SAG_MOT+1)  = -0.5;
q_ref(R_KNEE_SAG_MOT+1) = 1;
q_ref(R_ANK_SAG_MOT+1)  = -0.5;

q_ref(L_HIP_SAG_MOT+1)  = -0.5;
q_ref(L_KNEE_SAG_MOT+1) = 1;
q_ref(L_ANK_SAG_MOT+1)  = -0.5;

q_ref(WAIST_SAG_MOT+1)  = 0.5538;

q_ref(R_SH_SAG_MOT+1)  = 0.3;
q_ref(R_SH_LAT_MOT+1)  = 0.25;
q_ref(R_ELB_MOT+1)      = -1.2;

q_ref(L_SH_SAG_MOT+1)  = 0.3;
q_ref(L_SH_LAT_MOT+1)  = -0.25;
q_ref(L_ELB_MOT+1)     = -1.2;


