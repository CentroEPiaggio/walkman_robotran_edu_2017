tref = 0:Ts:Tfin;

hand_pos_0 = [0.3508    0.5128    0.7822];
hand_pos_f = [0.4508    0.6128    0.8822];

x_pos = linspace(hand_pos_0(1),hand_pos_f(1),length(tref));
y_pos = linspace(hand_pos_0(2),hand_pos_f(2),length(tref));
z_pos = linspace(hand_pos_0(3),hand_pos_f(3),length(tref));
left_hand_trajectory = [tref(:), x_pos(:), y_pos(:), z_pos(:)];

save left_hand_trajectory.mat left_hand_trajectory;