figure
idx = 1:7;
plot(t,tau_des(:,JOINTS_IDX_L_ARM(idx)),t,tau_mot(:,JOINTS_IDX_L_ARM(idx)))
% legend('HIP','KNEE','ANK','location','best')
grid on
figure
plot(t,q_ref(:,JOINTS_IDX_L_ARM(idx)),t,q_link(:,JOINTS_IDX_L_ARM(idx)));
grid on
%%
figure
idx = 1:2;
plot(t,tau_des(:,JOINTS_IDX_WAIST(idx)),t,tau_mot(:,JOINTS_IDX_WAIST(idx)))
