%
% Automates the compilation procedure of the model
%
% Developed by Nicolas Van der Noot and Allan Barrea - 2013
%

%% Initialization
clc;
clear all;
close all;

% compile all files (1) or not (0)
compile_all = 1;
    
% project name
prjname = 'walkman_robotran';
    
% debug (1 to debug, 0 otherwise)
debug = 1;

% compiler flags (except for the first one,  add -D before each flag) 
%    If you use the 'Short feet' version, you must add the flag 'SHORT_FEET' (-DSHORT_FEET)
%    If you use the 'Compliant feet' version, you must add the flag 'COMP_FEET' (-DCOMP_FEET)
%    If you use other versions, you must remove the flags 'SHORT_FEET' and 'COMP_FEET'
%    If you use the long arms version, you must add the flag 'LONG_ARMS' (-DLONG_ARMS), only available with the 'SHORT_FEET' flag (-DSHORT_FEET)
define = 'DIRDYNARED -DSTOP_SIMU -DMESH_GSM ';


%% Gen user_sf_IO.c & user_sf_IO.h if needed

genUserIO = 0;

userIOName = '../../StandaloneC/src/project/user_files/user_sf_IO.c';
simuVarName = './simu_variables.m';

if (compile_all) || (~exist(userIOName, 'file')) || (~exist(simuVarName, 'file'))
    genUserIO = 1;
else
    user_sf_IO_c = dir(userIOName);
    simu_variables_m = dir(simuVarName);

    if (simu_variables_m.datenum > user_sf_IO_c.datenum)   
        genUserIO = 1;
    end
end

% if genUserIO   
%     fprintf('\n>>> Generating user_IO files...\n');
%     
%     [simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct] = simu_variables();
%     
%     [nb_none, ~] = size(simu_vars_none);
%     if nb_none > 0
%         simu_vars_none = [simu_vars_none , repmat({'none'},nb_none,1)];
%     end
% 
%     [nb_in, ~] = size(simu_vars_in);
%     if nb_in > 0
%         simu_vars_in = [simu_vars_in , repmat({'in'},nb_in,1)];
%     end
% 
%     [nb_out, ~] = size(simu_vars_out);
%     
%     if nb_out > 0
%         simu_vars_out = [simu_vars_out , repmat({'out'},nb_out,1)];
%     end
%     simu_vars_out = [simu_vars_out ; {'stop_simu' , 'int', [1], 'out'}];
% 
%     [nb_struct, ~] = size(simu_vars_struct);
%     if nb_struct > 0
%         simu_vars_struct = [simu_vars_struct , repmat({'structure'},nb_struct,1)];
%     end
%     
%     simu_vars = cat(1,simu_vars_none, simu_vars_in, simu_vars_out, simu_vars_struct);    
%     addpath('make_generate/');
%     gen_user_IO(simu_vars, prjname); 
%     rmpath('make_generate/');
% else
%     fprintf('\n>>> No need to generate user_IO files...\n');
% end

%% Gen ControllerStruct.c & ControllerStruct.h if needed

genCont = 0;

% contStructName = '../../StandaloneC/src/project/controller_files/ControllersStruct.c';
% contVarName = './control_variables.m';
% 
% if (compile_all) || (~exist(contStructName, 'file')) || (~exist(contVarName, 'file'))
%     genCont = 1;
% else
%     ControllersStruct_c = dir(contStructName);
%     control_variables_m = dir(contVarName);
% 
%     if (control_variables_m.datenum > ControllersStruct_c.datenum)  
%         genCont = 1;
%     end
% end
% 
% if genCont   
%     fprintf('\n>>> Generating ModelStruct files...\n');
%     
%     controllers_struct = control_variables();
%     addpath('make_generate/');
%     gen_controller_struct(controllers_struct);
%     rmpath('make_generate/');
% else
%     fprintf('\n>>> No need to generate ModelStruct files...\n');
% end

%% Building MEX file
fprintf('\n>>> Building MEX file...\n');

if genUserIO || genCont
    compile_all = 1;
end

% project directories (generic & symbolic directories (with associated files) set in 'make_generate/mbs_make_sf.m')
mbspathdef;

simulation_dir = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files');
actuators_dir = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','Actuators');
jointcontroller_dir = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','JointController');
sensors_dir = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','Sensors');
FloatingBaseRestart_dir = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','FloatingBaseRestart');

a = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','contactGeom','gestion');
b = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','contactGeom','config');
c = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','contactGeom','shapes');
d = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','contactGeom','utilities','contact');
e = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','contactGeom','utilities','elements');
f = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','contactGeom','utilities','kinematics');

s = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','interface');
t = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','main');
u = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','low_filter');
v = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','specific_foot','generic');
w = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','specific_foot','generic');
x = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','specific_ground','flat');
y = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','specific_ground','generic');
z = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','specific_ground','equation');
aa = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','specific_ground','equation','linear');
bb = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','specific_whole_feet','rect');
cc = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','simulation_files','meshGCM','specific_whole_feet','generic');

controller_dir = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','controller_files');
interface_dir  = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','interface_controller');
rob_int_dir    = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','interface_controller','Robotran_controller');
useful_dir     = fullfile(mbsprjpath,prjname,'StandaloneC','src','generic','useful');
user_dir       = fullfile(mbsprjpath,prjname,'StandaloneC','src','project','user_files');
%robotran_dir = fullfile(mbsprjpath,prjname,'StandaloneC','src','generic','robotran');

all_project_dir = {simulation_dir, controller_dir, interface_dir, rob_int_dir, useful_dir, user_dir, actuators_dir, jointcontroller_dir, sensors_dir,FloatingBaseRestart_dir, s,t,u,v,w,x,y,z,aa,bb,cc, a,b,c,d,e,f};

addpath('make_generate/');
mbs_make_sf(compile_all, prjname, debug, define, all_project_dir);
rmpath('make_generate/');

%% Loading project
fprintf('\n>>> Loading project...\n');
[mbs_data, mbs_info] = mbs_load(prjname,'default');
fprintf('\n');
