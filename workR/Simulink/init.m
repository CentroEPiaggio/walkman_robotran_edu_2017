% this is needed to fix the anim file simulink block callback StopFcn
model_name = 'dirdyna_walkman_robotran';

%% load model parameters
[mbs_data__, mbs_info__] = mbs_load('walkman_robotran','default');
masses = mbs_data__.m(mbs_data__.m>0);
m_tot = sum(masses);

% get index of CoM sensors from list of S_ and F_sensors
sensor_list = [mbs_info__.allsensors(:,3); mbs_info__.allextforces(:,4)];
sensor_list_com = [];
for ii=1:length(mbs_data__.q)
    CoM_i_string = sprintf('CoM_%d', ii);
    sensor_list_i = find(strcmp(sensor_list, CoM_i_string));
    if ~isempty(sensor_list_i)
        sensor_list_com = [sensor_list_com; sensor_list_i];
    end
end
% sensor_list_com = find(strcmp(sensor_list,'DWYTorsoCg'));

left_hand_sensor = find(strcmp(sensor_list,'Lhand_sens'));
right_hand_sensor = find(strcmp(sensor_list,'Rhand_sens'));

torso_sensor = find(strcmp(sensor_list,'DWYTorsoCg'));

joints_mapping = mbs_data__.qu;%[1:6 7:12 19:24 32:34 37:43 46:52 53:54]; % mbs_data.qu vs mbs_data.qc

LF_sensor_idx = find(strcmp(sensor_list, 'LFoot_sens'));
RF_sensor_idx = find(strcmp(sensor_list, 'RFoot_sens'));

Waist_cg_sensor_idx = find(strcmp(sensor_list, 'WaistCg'));

%% homing position
q_homing = mbs_data__.q;
q_homing = q_homing(joints_mapping(7:end));

%% controller gains
KP_gain_ik = diag([1 1 1])*2;
KO_gain_ik = diag([1 1 1])*2;
K_gain_ik = blkdiag(KP_gain_ik,KO_gain_ik);
start_control_time = 0;

load left_hand_trajectory.mat;

% sampling time
Ts = 0.0005;
Tfin = 10;

generate_left_hand_trajectories