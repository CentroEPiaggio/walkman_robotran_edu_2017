%% 1. Initialization and Project Loading [mbs_load]
%--------------------------------------------------------------------------
close all; clear variables; clc;                                            % Cleaning of the Matlab workspace
global MBS_user;                                                            % Declaration of the global user structure
MBS_user.process = '';                                                      % Initialisation of the user field "process"

% Project loading
prjname = 'walkman_robotran';
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs" 


%% 2. Inverse dynamics [mbs_exe_invdyn]
%--------------------------------------------------------------------------
MBS_user.process = 'invdyn';


Joint_id = [7:18];

mbs_data = mbs_set_qa(mbs_data,Joint_id);                                   % Set variables [Joint_id] as actuated 

opt.invdyn = {'motion','trajectory','framerate',1000,...
    'save2file','yes','renamefile','no','verbose','yes'};
% other options : 'time', 'visualize', 'clearmbsglobal'                     % Help about options on www.robotran.be

[mbs_invdyn,mbs_data] = mbs_exe_invdyn(mbs_data,opt.invdyn);                % Inverse dynamics process
