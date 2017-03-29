% this is needed to fix the anim file simulink block callback StopFcn
model_name = 'dirdyna_walkman_robotran';

%% copy this FIX in simulink anim block callback StopFcn
% varname = get_param(gcb,'varname');                                     
% filename = get_param(gcb,'filename');                                   
% tq = evalin('base',varname);                                            
% tt = tq.time;                                                           
% qq = tq.signals.values;                                                 
% % userData = get_param(gcb,'UserData');                                     
% userData = get_param([model_name '/S-Function_dirdynared'],'UserData'); %FIX
% mbs_data = userData.mbs_data;                                           
% mbs_info = userData.mbs_info;                                           
% overwrite = str2double(get_param(gcb,'overwrite'));                     
% if isnan(overwrite)
%     overwrite = false;
% end
% samplingRate = str2double(get_param(gcb,'samplingRate'));               
% % mbs_gen_simdir(mbs_data,mbs_info,tt,qq,filename,overwrite,samplingRate);
% mbs_gen_simdir(mbs_info.mbsname,tt,qq,filename,overwrite,samplingRate) %FIX