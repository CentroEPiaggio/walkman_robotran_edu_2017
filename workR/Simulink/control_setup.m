% T3 : +1.011324
%% setup index for joints
JOINTS_IDX = containers.Map();
JOINTS_IDX('R_HIP_LAT_MOT')     = 0;
JOINTS_IDX('R_HIP_TRANS_MOT')   = 1;
JOINTS_IDX('R_HIP_SAG_MOT')     = 2;
JOINTS_IDX('R_KNEE_SAG_MOT')    = 3;
JOINTS_IDX('R_ANK_SAG_MOT')     = 4;
JOINTS_IDX('R_ANK_LAT_MOT')     = 5;
JOINTS_IDX_R_LEG = 1+[0:5];
%  left leg
JOINTS_IDX('L_HIP_LAT_MOT')     = 6;
JOINTS_IDX('L_HIP_TRANS_MOT')   = 7;
JOINTS_IDX('L_HIP_SAG_MOT')     = 8;
JOINTS_IDX('L_KNEE_SAG_MOT')    = 9;
JOINTS_IDX('L_ANK_SAG_MOT')     = 10;
JOINTS_IDX('L_ANK_LAT_MOT')     = 11;
JOINTS_IDX_L_LEG = 1+[6:11];
% waist
JOINTS_IDX('WAIST_LAT_MOT')     = 12;
JOINTS_IDX('WAIST_SAG_MOT')     = 13;
JOINTS_IDX('WAIST_TRANS_MOT')   = 14;
JOINTS_IDX_WAIST = 1+[12:14];
% right arm
JOINTS_IDX('R_SH_SAG_MOT')      = 15;
JOINTS_IDX('R_SH_LAT_MOT')      = 16;
JOINTS_IDX('R_SH_TRANS_MOT')    = 17;
JOINTS_IDX('R_ELB_MOT')         = 18;
JOINTS_IDX('R_FORE_ARM_PLATE_MOT')  = 19;
JOINTS_IDX('R_WRJ1_MOT')        = 20;
JOINTS_IDX('R_WRJ2_MOT')        = 21;
JOINTS_IDX_R_ARM = 1+[15:21];

% left arm
JOINTS_IDX('L_SH_SAG_MOT')      = 22;
JOINTS_IDX('L_SH_LAT_MOT')      = 23;
JOINTS_IDX('L_SH_TRANS_MOT')    = 24;
JOINTS_IDX('L_ELB_MOT')         = 25;
JOINTS_IDX('L_FORE_ARM_PLATE_MOT')  = 26;
JOINTS_IDX('L_WRJ1_MOT')        = 27;
JOINTS_IDX('L_WRJ2_MOT')        = 28;
JOINTS_IDX_L_ARM = 1+[22:28];

%% setup control mode
% 0: position control
% 1: torque control
CTRL_MODE = zeros(31,1);

CTRL_MODE_R_LEG = [0;0;0;0;0;0];
CTRL_MODE_L_LEG = [0;0;0;0;0;0];
CTRL_MODE_WAIST = [0;1;0];
CTRL_MODE_R_ARM = [1;1;1;1;1;1;1];
CTRL_MODE_L_ARM = [1;1;1;1;1;1;1];

CTRL_MODE(JOINTS_IDX_R_LEG) = CTRL_MODE_R_LEG;
CTRL_MODE(JOINTS_IDX_L_LEG) = CTRL_MODE_L_LEG;
CTRL_MODE(JOINTS_IDX_WAIST) = CTRL_MODE_WAIST;
CTRL_MODE(JOINTS_IDX_R_ARM) = CTRL_MODE_R_ARM;
CTRL_MODE(JOINTS_IDX_L_ARM) = CTRL_MODE_L_ARM;

%% actuated joints
joints_map = mbs_data__.qu;
actuated_joints_idx = joints_mapping(7:end);

%% control gains
KP = -diag([1 %rleg
1
1
1
1
1
1 %lleg
1
1
1
1
1
1 % waist
800
1
1000 % rarm
1000
1000
1000
100
100
100
1000 % larm
1000
1000
1000
100
100
100
1 % neck
1]);
KD = -1*diag([1 %rleg
1
1
1
1
1
1 %lleg
1
1
1
1
1
1 % waist
10
1
10 % rarm
10
10
10
1
1
1
10 % larm
10
10
10
1
1
1
1 % neck
1]);
%% Initialize variables
s.q = mbs_data__.q;
s.qd = mbs_data__.qd;
s.qdd = mbs_data__.qdd;

s.m = mbs_data__.m;
s.g = mbs_data__.g;
s.l = mbs_data__.l;
s.In = mbs_data__.In;
s.dpt = mbs_data__.dpt;

s.frc = mbs_data__.frc;
s.trq = mbs_data__.trq;
tsim = 0;
%
load left_hand_trajectory.mat