tref = 0:Ts:Tfin;

hand_pos_0 = [0.3508    0.5128    0.7822];
lh_x = linspace(0.3508,0.4,length(tref));
lh_y = linspace(hand_pos_0(2),0.35,length(tref))-0.1*sin(0.4*tref);
lh_z = 0.8+0.05*cos(0.4*tref);
% figure
% plot3(lh_x,lh_y,lh_z)
left_hand_trajectory = [tref(:), lh_x(:), lh_y(:), lh_z(:)];

save left_hand_trajectory.mat left_hand_trajectory;