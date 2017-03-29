% INITIALIZING ROBOTRAN SIMULATION  
clc;clear all;  
global MBS_user;  

% Project loading  
prjname = 'walkman_robotran';  
[mbs_data, mbs_info] = mbs_load(prjname,'default');    
mbs_data_ini = mbs_data;                               

qdriven=[7:42];
mbs_data=mbs_set_qdriven(mbs_data,qdriven);
qu=[1:6];
mbs_data=mbs_set_qu(mbs_data,qu);

MBS_user.GRF=zeros(6,2);  

MBS_user.tau=zeros(42-6,1);

%% Defining Contact properties (NEW CONTACT MODEL)
% A mesh grid of contact points under sole of robot should be  
% defined. Contact surface can have a 3D shape and is not necessarily flat  
% surface. Mesh grid can be finer in more critical area of sole which is  
% in contact with ground  
 
 
%[-X1 X2] and [-Y1 Y2] introduce possible contact range of foot 
X1=0.10;X2=0.20; Y1=0.08;Y2=0.08;  
 
% x, y and z contains position coordinate of all gird points which can be 
% contact with ground. These points can be palced in arbitrary positions 
% based on different applications 
% x=-X1:0.01*8:X2;  
% y=-Y1:0.01*10:Y2;  
  
x=-X1:0.01*1:X2;  
y=-Y1:0.01*1.25:Y2;  
m=size(x);  
n=size(y);  
 
Msize=m(2)*n(2);%Mesh size is number of all points under each foot  
 z=0;  
disp(['Mesh size =  ', num2str(Msize)]);  
 
% rn is mesh points in body coordinate system. rn shows shape of sole 
rn=zeros(Msize,3);  
for i=1:m(2)  
    for j=1:n(2)  
rn((j+(i-1)*n(2)),:)=[x(i),y(j),z];  
    end  
end  
  
MBS_user.rn=rn';% Mesh vector  
MBS_user.Msize=Msize;  

%% The ground's friction coeficient is set in this section  
MBS_user.mu_grf=0.6; %0.8  Fx=mu*Fz friction coef.  
   
% coefficients for linear  
% MBS_user.LinearModelGRF=1;  
MBS_user.ContactCof=[278000/2 300]*2/Msize; %K D   
  
% MBS_user.ContactCof=[278000 300]*4/Msize; %K D   

MBS_user.ContactCof=[278000 300]*6/Msize; %K D   

%% Temporary variables to store the values of initial contact points:   
MBS_user.temp_grfx=zeros(1,2*Msize);%used to store the value of initial X contact point with the ground.   
MBS_user.temp_grfy=zeros(1,2*Msize);%used to store the value of initial Y contact point with the ground.  
%% These flags are used for horizontal friction    
MBS_user.flag_grfx=zeros(1,2*Msize);%flag used to store the initial X contact point with the ground.  
MBS_user.flag_grfy=zeros(1,2*Msize);%flag used to store the initial Y contact point with the ground.   

%% numerical simulation

myopt.dirdyn_simulation =  {'motion','simulation','verbose','yes','visualize','no','save2file','yes','renamefile','no',... 
    'odemethod','ode45','time',0:0.005:1,'framerate',1000}; 
[mbs_dirdyn,mbs_data_dirdyn] = mbs_exe_dirdyn(mbs_data,myopt.dirdyn_simulation); 
 

%% Ground reaction force:
mbs_dirdyn.GRF=MBS_user.resdirdyn.GRF;

RF=MBS_user.resdirdyn.GRF(:,1,:);
RF=reshape(RF,6,length(mbs_dirdyn.tsim));

% RF is composed of 
% RF(1,:): Fx friction force, 
% RF(2,:): Fy friction force,
% RF(3,:): Fz vertical force,
% RF(4,:): Tx Lateral moment, 
% RF(5,:): Ty sagittal moment,
% RF(6,:): Tz Yaw moment,
% The same also goes for LF (Left Foot)

LF=MBS_user.resdirdyn.GRF(:,2,:);
LF=reshape(LF,6,length(mbs_dirdyn.tsim));

figure(1)
plot(mbs_dirdyn.tsim, RF(3,:));
title('Ground Reaction Force (Right Foot)');% 
xlabel('t (sec)'); grid on;
ylabel('N')

figure(2)
plot(mbs_dirdyn.tsim, LF(3,:));
title('Ground Reaction Force (Left Foot)');
xlabel('t (sec)'); grid on;
ylabel('N')
