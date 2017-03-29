function Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)
%
% PxF(3,1) : absolute position vector of the external force application point 
% RxF(3,3) : absolute rotation matrix of the body
% VxF(3,1) : absolute velocity vector of the external force application point 
% OMxF(3,1) : absolute angular velocity vector of the body
% AxF(3,1) : absolute acceleration vector of the external force application point 
% OMPxF(3,1) : absolute angular acceleration vector of the body
%
% => All above vectors are expressed in the inertial reference frame !
%
% mbs_data : multibody data structure
% tsim : current time
% ixF : index of the external force sensor ('F' type in MBsysPad)
%        (can be obtained via the 'mbs_get_F_sensor_id' function)
%
% Swr(9,1) = [Fx; Fy; Fz; Mx; My; Mz; dxF];
%   - Force components (expressed in the inertial frame) : Fx, Fy, Fz
%   - Pure torque components (expressed in the inertial frame) : Mx, My, Mz
%   - Application point local coordinates vector (expressed in the body-fixed frame) : dxF(1:3,1);
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

Fx=0.0; Fy=0.0; Fz=0.0;
Mx=0.0; My=0.0; Mz=0.0;
idpt = mbs_data.xfidpt(ixF);
dxF = mbs_data.dpt(:,idpt);

%/*-- Begin of user code --*/
% 
% Use the 'mbs_get_F_sensor_id' function to get easily the force sensor
% indices, e.g. :
% F1 = mbs_get_F_sensor_id(MBS_info,'myFsensor_1');
% [F2,F3] = mbs_get_F_sensor_id(MBS_info,'myFsensor_2','myFsensor_3');
%
% switch(ixF)
%     case F1
%         ; % instructions for case 1
%     case F2
%         ; % instructions for case 2
%     case F3
%         ; % ...
% end

  
% Contact Model of foot written by Mohamad Mosadeghzad  
% mohamad.mosadeghzad@iit.it or m_mzad83@yahoo.com  
% This Contact model is written to increase accuracy and speed of  
% simulation. A mesh grid of contact points under sole of robot should be  
% defined. Contact surface can have a 3D shape and is not necessarily flat  
% surface. Mesh grid can be finer in more critical area of sole which is  
% in contact with ground  
         
mu=MBS_user.mu_grf;     
rn=MBS_user.rn;  
Msize=MBS_user.Msize;%Mesh size  
  
  
% Just one Force sensor must be considered under foot and index of sensors under each foot should be ixF=1 and ixF=2  
       
if  ((ixF==1)||(ixF==2))  
     if ixF==1  
         N=0;  
     else  
         N=Msize;  
     end  
   
v=zeros(3, Msize);  
FTx=zeros(1, Msize);  
FTy=zeros(1, Msize);  
FTz=zeros(1, Msize);  
  
  
r0=RxF'*(rn); % rn is mesh points in body coordinate system. rn shows shape of sole   
r=PxF'*ones(1,Msize) + r0;%% r is position vector of mesh points in global coordinate system  
  
  
% Cross Product of OMxF and r0   
v(1,:)=-OMxF(3)*r0(2,:)+OMxF(2)*r0(3,:);  
v(2,:)= OMxF(3)*r0(1,:)-OMxF(1)*r0(3,:);  
v(3,:)=-OMxF(2)*r0(1,:)+OMxF(1)*r0(2,:);  
  
v = v+ VxF'*ones(1,Msize);%% v is velocity vector of mesh points in global coordinate system  
  
             
% K=MBS_user.ContactCof(1);D=MBS_user.ContactCof(2);  
  
threshold=0.0000001;  
for i=1:Msize  
      
  
if (r(3,i)<=threshold)  
  
        % Should APPLY DAMPING IN BOTH DIRECTIONS, BOTH WHEN penetration and when the foot is going out of the ground.   
  
        % apply the linear model for the normal force:  
  
        FTz(i)= -MBS_user.ContactCof(1)*r(3,i) - MBS_user.ContactCof(2)*v(3,i);   
  
        % The force is applied only to push the robot, not to pull toward  
        % the ground.  
        if FTz(i)<0     
        FTz(i)=0;     
        end     
        %----------------------------------------------------------                     
        % Horizontal friction force in X direction:     
        if MBS_user.flag_grfx(i+N)==0     
           MBS_user.temp_grfx(i+N)=r(1,i); %store the initial point of contact.     
           MBS_user.flag_grfx(i+N)=1;     
        end     
  
        FTx(i)=-MBS_user.ContactCof(2)*v(1,i)-MBS_user.ContactCof(1)*(r(1,i)-MBS_user.temp_grfx(i+N)); % friction force can be positive or negative.     
  
        %----------------------------------------------------------     
        % Horizontal friction force in Y direction:     
        if MBS_user.flag_grfy(i+N)==0     
           MBS_user.temp_grfy(i+N)=r(2,i); %store the initial point of contact.     
           MBS_user.flag_grfy(i+N)=1;     
        end     
  
      FTy(i)=-MBS_user.ContactCof(2)*v(2,i)-MBS_user.ContactCof(1)*(r(2,i)-MBS_user.temp_grfy(i+N)); % friction force can be positive or negative.     
  
     % chech if each points of contact is slipping or not  
     fr=sqrt(FTx(i)^2+FTy(i)^2);    
     if fr>mu*FTz(i)    
         fr=mu*FTz(i);         
         Alpha=atan2(v(2,i),v(1,i));    
         FTx(i)=-fr*cos(Alpha);    
         FTy(i)=-fr*sin(Alpha);    
         MBS_user.flag_grfx(i+N)=0;     
         MBS_user.flag_grfy(i+N)=0;     
     end    
  
  
else     
    FTx(i)=0;     
    FTy(i)=0;     
    FTz(i)=0;     
    MBS_user.flag_grfx(i+N)=0;     
    MBS_user.flag_grfy(i+N)=0;     
end;     
        
  
end % for loop for all Mesh points of contact (Msize)  
  
  
Fx=sum(FTx);     
Fy=sum(FTy);     
Fz=sum(FTz);  
  
T=zeros(3,Msize);  
  
% Calculate resultant Torque relative to Position of sensor F  
% Cross Product of r0 and FT   
T(1,:)=-r0(3,:).*FTy+r0(2,:).*FTz;  
T(2,:)=+r0(3,:).*FTx-r0(1,:).*FTz;  
T(3,:)=-r0(2,:).*FTx+r0(1,:).*FTy;  
  
  
% calculating Total Torque  
Mx=sum(T(1,:));   
My=sum(T(2,:));    
Mz=sum(T(3,:));    
  
end  

%/*-- End of user code --*/

Swr = [Fx; Fy; Fz; Mx; My; Mz; dxF];

MBS_user.GRF(:,ixF)=Swr(1:6);   
   
MBS_user.GRFPxF(:,ixF)=[Fx*PxF(1); Fy*PxF(2)];    

return