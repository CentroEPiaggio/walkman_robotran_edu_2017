%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Thu Oct 29 11:32:57 2015
%
%	==> Project name : walkman_robotran
%	==> using XML input file 
%
%	==> Number of joints : 55
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 26911
%
%	==> Generation Time :  0.530 seconds
%	==> Post-Processing :  0.500 seconds
%
%-------------------------------------------------------------
%
function [sens] = gensensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,55);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C4 = cos(q(4));
  S4 = sin(q(4));
  C5 = cos(q(5));
  S5 = sin(q(5));
  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C7 = cos(q(7));
  S7 = sin(q(7));
  C8 = cos(q(8));
  S8 = sin(q(8));
  C9 = cos(q(9));
  S9 = sin(q(9));
  C10 = cos(q(10));
  S10 = sin(q(10));
  C11 = cos(q(11));
  S11 = sin(q(11));
  C12 = cos(q(12));
  S12 = sin(q(12));
  C13 = cos(q(13));
  S13 = sin(q(13));
  C14 = cos(q(14));
  S14 = sin(q(14));
  C15 = cos(q(15));
  S15 = sin(q(15));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C19 = cos(q(19));
  S19 = sin(q(19));
  C20 = cos(q(20));
  S20 = sin(q(20));
  C21 = cos(q(21));
  S21 = sin(q(21));
  C22 = cos(q(22));
  S22 = sin(q(22));
  C23 = cos(q(23));
  S23 = sin(q(23));
  C24 = cos(q(24));
  S24 = sin(q(24));
  C25 = cos(q(25));
  S25 = sin(q(25));
  C26 = cos(q(26));
  S26 = sin(q(26));
  C27 = cos(q(27));
  S27 = sin(q(27));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C31 = cos(q(31));
  S31 = sin(q(31));
  C32 = cos(q(32));
  S32 = sin(q(32));
  C33 = cos(q(33));
  S33 = sin(q(33));
  C34 = cos(q(34));
  S34 = sin(q(34));

% = = Block_0_0_0_0_0_5 = = 
 
% Trigonometric Variables  

  C35 = cos(q(35));
  S35 = sin(q(35));
  C36 = cos(q(36));
  S36 = sin(q(36));
  C37 = cos(q(37));
  S37 = sin(q(37));
  C38 = cos(q(38));
  S38 = sin(q(38));
  C39 = cos(q(39));
  S39 = sin(q(39));
  C40 = cos(q(40));
  S40 = sin(q(40));
  C41 = cos(q(41));
  S41 = sin(q(41));
  C42 = cos(q(42));
  S42 = sin(q(42));
  C43 = cos(q(43));
  S43 = sin(q(43));

% = = Block_0_0_0_0_0_6 = = 
 
% Trigonometric Variables  

  C44 = cos(q(44));
  S44 = sin(q(44));
  C45 = cos(q(45));
  S45 = sin(q(45));
  C46 = cos(q(46));
  S46 = sin(q(46));
  C47 = cos(q(47));
  S47 = sin(q(47));
  C48 = cos(q(48));
  S48 = sin(q(48));
  C49 = cos(q(49));
  S49 = sin(q(49));
  C50 = cos(q(50));
  S50 = sin(q(50));
  C51 = cos(q(51));
  S51 = sin(q(51));
  C52 = cos(q(52));
  S52 = sin(q(52));

% = = Block_0_0_0_0_0_7 = = 
 
% Augmented Joint Position Vectors   

  Dz553 = q(55)+s.dpt(3,57);
 
% Trigonometric Variables  

  C53 = cos(q(53));
  S53 = sin(q(53));
  C54 = cos(q(54));
  S54 = sin(q(54));

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.A(1) = qdd(1);
 
% 
case 2, 


% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
 
% 
case 3, 


% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
 
% 
case 4, 


% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C4;
    sens.R(2,3) = S4;
    sens.R(3,2) = -S4;
    sens.R(3,3) = C4;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = qd(4);
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = qdd(4);
 
% 
case 5, 


% = = Block_1_0_0_5_0_1 = = 
 
% Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    OMcp4_25 = qd(5)*C4;
    OMcp4_35 = qd(5)*S4;
    OPcp4_25 = qdd(5)*C4-qd(4)*qd(5)*S4;
    OPcp4_35 = qdd(5)*S4+qd(4)*qd(5)*C4;

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = C5;
    sens.R(1,2) = ROcp4_25;
    sens.R(1,3) = ROcp4_35;
    sens.R(2,2) = C4;
    sens.R(2,3) = S4;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp4_85;
    sens.R(3,3) = ROcp4_95;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = qd(4);
    sens.OM(2) = OMcp4_25;
    sens.OM(3) = OMcp4_35;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = qdd(4);
    sens.OMP(2) = OPcp4_25;
    sens.OMP(3) = OPcp4_35;
 
% 
case 6, 


% = = Block_1_0_0_6_0_1 = = 
 
% Sensor Kinematics 


    ROcp5_25 = S4*S5;
    ROcp5_35 = -C4*S5;
    ROcp5_85 = -S4*C5;
    ROcp5_95 = C4*C5;
    ROcp5_16 = C5*C6;
    ROcp5_26 = ROcp5_25*C6+C4*S6;
    ROcp5_36 = ROcp5_35*C6+S4*S6;
    ROcp5_46 = -C5*S6;
    ROcp5_56 = -(ROcp5_25*S6-C4*C6);
    ROcp5_66 = -(ROcp5_35*S6-S4*C6);
    OMcp5_25 = qd(5)*C4;
    OMcp5_35 = qd(5)*S4;
    OMcp5_16 = qd(4)+qd(6)*S5;
    OMcp5_26 = OMcp5_25+ROcp5_85*qd(6);
    OMcp5_36 = OMcp5_35+ROcp5_95*qd(6);
    OPcp5_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp5_26 = ROcp5_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp5_35*S5-ROcp5_95*qd(4));
    OPcp5_36 = ROcp5_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp5_25*S5-ROcp5_85*qd(4));

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = ROcp5_16;
    sens.R(1,2) = ROcp5_26;
    sens.R(1,3) = ROcp5_36;
    sens.R(2,1) = ROcp5_46;
    sens.R(2,2) = ROcp5_56;
    sens.R(2,3) = ROcp5_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp5_85;
    sens.R(3,3) = ROcp5_95;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = OMcp5_16;
    sens.OM(2) = OMcp5_26;
    sens.OM(3) = OMcp5_36;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = OPcp5_16;
    sens.OMP(2) = OPcp5_26;
    sens.OMP(3) = OPcp5_36;
 
% 
case 7, 


% = = Block_1_0_0_7_0_1 = = 
 
% Sensor Kinematics 


    ROcp6_25 = S4*S5;
    ROcp6_35 = -C4*S5;
    ROcp6_85 = -S4*C5;
    ROcp6_95 = C4*C5;
    ROcp6_16 = C5*C6;
    ROcp6_26 = ROcp6_25*C6+C4*S6;
    ROcp6_36 = ROcp6_35*C6+S4*S6;
    ROcp6_46 = -C5*S6;
    ROcp6_56 = -(ROcp6_25*S6-C4*C6);
    ROcp6_66 = -(ROcp6_35*S6-S4*C6);
    OMcp6_25 = qd(5)*C4;
    OMcp6_35 = qd(5)*S4;
    OMcp6_16 = qd(4)+qd(6)*S5;
    OMcp6_26 = OMcp6_25+ROcp6_85*qd(6);
    OMcp6_36 = OMcp6_35+ROcp6_95*qd(6);
    OPcp6_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp6_26 = ROcp6_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp6_35*S5-ROcp6_95*qd(4));
    OPcp6_36 = ROcp6_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp6_25*S5-ROcp6_85*qd(4));

% = = Block_1_0_0_7_0_2 = = 
 
% Sensor Kinematics 


    ROcp6_47 = ROcp6_46*C7+S5*S7;
    ROcp6_57 = ROcp6_56*C7+ROcp6_85*S7;
    ROcp6_67 = ROcp6_66*C7+ROcp6_95*S7;
    ROcp6_77 = -(ROcp6_46*S7-S5*C7);
    ROcp6_87 = -(ROcp6_56*S7-ROcp6_85*C7);
    ROcp6_97 = -(ROcp6_66*S7-ROcp6_95*C7);
    RLcp6_17 = ROcp6_16*s.dpt(1,1)+ROcp6_46*s.dpt(2,1);
    RLcp6_27 = ROcp6_26*s.dpt(1,1)+ROcp6_56*s.dpt(2,1);
    RLcp6_37 = ROcp6_36*s.dpt(1,1)+ROcp6_66*s.dpt(2,1);
    POcp6_17 = RLcp6_17+q(1);
    POcp6_27 = RLcp6_27+q(2);
    POcp6_37 = RLcp6_37+q(3);
    OMcp6_17 = OMcp6_16+ROcp6_16*qd(7);
    OMcp6_27 = OMcp6_26+ROcp6_26*qd(7);
    OMcp6_37 = OMcp6_36+ROcp6_36*qd(7);
    ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
    ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
    ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
    VIcp6_17 = ORcp6_17+qd(1);
    VIcp6_27 = ORcp6_27+qd(2);
    VIcp6_37 = ORcp6_37+qd(3);
    OPcp6_17 = OPcp6_16+ROcp6_16*qdd(7)+qd(7)*(OMcp6_26*ROcp6_36-OMcp6_36*ROcp6_26);
    OPcp6_27 = OPcp6_26+ROcp6_26*qdd(7)-qd(7)*(OMcp6_16*ROcp6_36-OMcp6_36*ROcp6_16);
    OPcp6_37 = OPcp6_36+ROcp6_36*qdd(7)+qd(7)*(OMcp6_16*ROcp6_26-OMcp6_26*ROcp6_16);
    ACcp6_17 = qdd(1)+OMcp6_26*ORcp6_37-OMcp6_36*ORcp6_27+OPcp6_26*RLcp6_37-OPcp6_36*RLcp6_27;
    ACcp6_27 = qdd(2)-OMcp6_16*ORcp6_37+OMcp6_36*ORcp6_17-OPcp6_16*RLcp6_37+OPcp6_36*RLcp6_17;
    ACcp6_37 = qdd(3)+OMcp6_16*ORcp6_27-OMcp6_26*ORcp6_17+OPcp6_16*RLcp6_27-OPcp6_26*RLcp6_17;

% = = Block_1_0_0_7_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp6_17;
    sens.P(2) = POcp6_27;
    sens.P(3) = POcp6_37;
    sens.R(1,1) = ROcp6_16;
    sens.R(1,2) = ROcp6_26;
    sens.R(1,3) = ROcp6_36;
    sens.R(2,1) = ROcp6_47;
    sens.R(2,2) = ROcp6_57;
    sens.R(2,3) = ROcp6_67;
    sens.R(3,1) = ROcp6_77;
    sens.R(3,2) = ROcp6_87;
    sens.R(3,3) = ROcp6_97;
    sens.V(1) = VIcp6_17;
    sens.V(2) = VIcp6_27;
    sens.V(3) = VIcp6_37;
    sens.OM(1) = OMcp6_17;
    sens.OM(2) = OMcp6_27;
    sens.OM(3) = OMcp6_37;
    sens.A(1) = ACcp6_17;
    sens.A(2) = ACcp6_27;
    sens.A(3) = ACcp6_37;
    sens.OMP(1) = OPcp6_17;
    sens.OMP(2) = OPcp6_27;
    sens.OMP(3) = OPcp6_37;
 
% 
case 8, 


% = = Block_1_0_0_8_0_1 = = 
 
% Sensor Kinematics 


    ROcp7_25 = S4*S5;
    ROcp7_35 = -C4*S5;
    ROcp7_85 = -S4*C5;
    ROcp7_95 = C4*C5;
    ROcp7_16 = C5*C6;
    ROcp7_26 = ROcp7_25*C6+C4*S6;
    ROcp7_36 = ROcp7_35*C6+S4*S6;
    ROcp7_46 = -C5*S6;
    ROcp7_56 = -(ROcp7_25*S6-C4*C6);
    ROcp7_66 = -(ROcp7_35*S6-S4*C6);
    OMcp7_25 = qd(5)*C4;
    OMcp7_35 = qd(5)*S4;
    OMcp7_16 = qd(4)+qd(6)*S5;
    OMcp7_26 = OMcp7_25+ROcp7_85*qd(6);
    OMcp7_36 = OMcp7_35+ROcp7_95*qd(6);
    OPcp7_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp7_26 = ROcp7_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp7_35*S5-ROcp7_95*qd(4));
    OPcp7_36 = ROcp7_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp7_25*S5-ROcp7_85*qd(4));

% = = Block_1_0_0_8_0_2 = = 
 
% Sensor Kinematics 


    ROcp7_47 = ROcp7_46*C7+S5*S7;
    ROcp7_57 = ROcp7_56*C7+ROcp7_85*S7;
    ROcp7_67 = ROcp7_66*C7+ROcp7_95*S7;
    ROcp7_77 = -(ROcp7_46*S7-S5*C7);
    ROcp7_87 = -(ROcp7_56*S7-ROcp7_85*C7);
    ROcp7_97 = -(ROcp7_66*S7-ROcp7_95*C7);
    ROcp7_18 = ROcp7_16*C8+ROcp7_47*S8;
    ROcp7_28 = ROcp7_26*C8+ROcp7_57*S8;
    ROcp7_38 = ROcp7_36*C8+ROcp7_67*S8;
    ROcp7_48 = -(ROcp7_16*S8-ROcp7_47*C8);
    ROcp7_58 = -(ROcp7_26*S8-ROcp7_57*C8);
    ROcp7_68 = -(ROcp7_36*S8-ROcp7_67*C8);
    RLcp7_17 = ROcp7_16*s.dpt(1,1)+ROcp7_46*s.dpt(2,1);
    RLcp7_27 = ROcp7_26*s.dpt(1,1)+ROcp7_56*s.dpt(2,1);
    RLcp7_37 = ROcp7_36*s.dpt(1,1)+ROcp7_66*s.dpt(2,1);
    OMcp7_17 = OMcp7_16+ROcp7_16*qd(7);
    OMcp7_27 = OMcp7_26+ROcp7_26*qd(7);
    OMcp7_37 = OMcp7_36+ROcp7_36*qd(7);
    ORcp7_17 = OMcp7_26*RLcp7_37-OMcp7_36*RLcp7_27;
    ORcp7_27 = -(OMcp7_16*RLcp7_37-OMcp7_36*RLcp7_17);
    ORcp7_37 = OMcp7_16*RLcp7_27-OMcp7_26*RLcp7_17;
    OPcp7_17 = OPcp7_16+ROcp7_16*qdd(7)+qd(7)*(OMcp7_26*ROcp7_36-OMcp7_36*ROcp7_26);
    OPcp7_27 = OPcp7_26+ROcp7_26*qdd(7)-qd(7)*(OMcp7_16*ROcp7_36-OMcp7_36*ROcp7_16);
    OPcp7_37 = OPcp7_36+ROcp7_36*qdd(7)+qd(7)*(OMcp7_16*ROcp7_26-OMcp7_26*ROcp7_16);
    RLcp7_18 = ROcp7_16*s.dpt(1,5)+ROcp7_47*s.dpt(2,5)+ROcp7_77*s.dpt(3,5);
    RLcp7_28 = ROcp7_26*s.dpt(1,5)+ROcp7_57*s.dpt(2,5)+ROcp7_87*s.dpt(3,5);
    RLcp7_38 = ROcp7_36*s.dpt(1,5)+ROcp7_67*s.dpt(2,5)+ROcp7_97*s.dpt(3,5);
    POcp7_18 = RLcp7_17+RLcp7_18+q(1);
    POcp7_28 = RLcp7_27+RLcp7_28+q(2);
    POcp7_38 = RLcp7_37+RLcp7_38+q(3);
    OMcp7_18 = OMcp7_17+ROcp7_77*qd(8);
    OMcp7_28 = OMcp7_27+ROcp7_87*qd(8);
    OMcp7_38 = OMcp7_37+ROcp7_97*qd(8);
    ORcp7_18 = OMcp7_27*RLcp7_38-OMcp7_37*RLcp7_28;
    ORcp7_28 = -(OMcp7_17*RLcp7_38-OMcp7_37*RLcp7_18);
    ORcp7_38 = OMcp7_17*RLcp7_28-OMcp7_27*RLcp7_18;
    VIcp7_18 = ORcp7_17+ORcp7_18+qd(1);
    VIcp7_28 = ORcp7_27+ORcp7_28+qd(2);
    VIcp7_38 = ORcp7_37+ORcp7_38+qd(3);
    OPcp7_18 = OPcp7_17+ROcp7_77*qdd(8)+qd(8)*(OMcp7_27*ROcp7_97-OMcp7_37*ROcp7_87);
    OPcp7_28 = OPcp7_27+ROcp7_87*qdd(8)-qd(8)*(OMcp7_17*ROcp7_97-OMcp7_37*ROcp7_77);
    OPcp7_38 = OPcp7_37+ROcp7_97*qdd(8)+qd(8)*(OMcp7_17*ROcp7_87-OMcp7_27*ROcp7_77);
    ACcp7_18 = qdd(1)+OMcp7_26*ORcp7_37+OMcp7_27*ORcp7_38-OMcp7_36*ORcp7_27-OMcp7_37*ORcp7_28+OPcp7_26*RLcp7_37+OPcp7_27*RLcp7_38-OPcp7_36*RLcp7_27...
 -OPcp7_37*RLcp7_28;
    ACcp7_28 = qdd(2)-OMcp7_16*ORcp7_37-OMcp7_17*ORcp7_38+OMcp7_36*ORcp7_17+OMcp7_37*ORcp7_18-OPcp7_16*RLcp7_37-OPcp7_17*RLcp7_38+OPcp7_36*RLcp7_17...
 +OPcp7_37*RLcp7_18;
    ACcp7_38 = qdd(3)+OMcp7_16*ORcp7_27+OMcp7_17*ORcp7_28-OMcp7_26*ORcp7_17-OMcp7_27*ORcp7_18+OPcp7_16*RLcp7_27+OPcp7_17*RLcp7_28-OPcp7_26*RLcp7_17...
 -OPcp7_27*RLcp7_18;

% = = Block_1_0_0_8_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp7_18;
    sens.P(2) = POcp7_28;
    sens.P(3) = POcp7_38;
    sens.R(1,1) = ROcp7_18;
    sens.R(1,2) = ROcp7_28;
    sens.R(1,3) = ROcp7_38;
    sens.R(2,1) = ROcp7_48;
    sens.R(2,2) = ROcp7_58;
    sens.R(2,3) = ROcp7_68;
    sens.R(3,1) = ROcp7_77;
    sens.R(3,2) = ROcp7_87;
    sens.R(3,3) = ROcp7_97;
    sens.V(1) = VIcp7_18;
    sens.V(2) = VIcp7_28;
    sens.V(3) = VIcp7_38;
    sens.OM(1) = OMcp7_18;
    sens.OM(2) = OMcp7_28;
    sens.OM(3) = OMcp7_38;
    sens.A(1) = ACcp7_18;
    sens.A(2) = ACcp7_28;
    sens.A(3) = ACcp7_38;
    sens.OMP(1) = OPcp7_18;
    sens.OMP(2) = OPcp7_28;
    sens.OMP(3) = OPcp7_38;
 
% 
case 9, 


% = = Block_1_0_0_9_0_1 = = 
 
% Sensor Kinematics 


    ROcp8_25 = S4*S5;
    ROcp8_35 = -C4*S5;
    ROcp8_85 = -S4*C5;
    ROcp8_95 = C4*C5;
    ROcp8_16 = C5*C6;
    ROcp8_26 = ROcp8_25*C6+C4*S6;
    ROcp8_36 = ROcp8_35*C6+S4*S6;
    ROcp8_46 = -C5*S6;
    ROcp8_56 = -(ROcp8_25*S6-C4*C6);
    ROcp8_66 = -(ROcp8_35*S6-S4*C6);
    OMcp8_25 = qd(5)*C4;
    OMcp8_35 = qd(5)*S4;
    OMcp8_16 = qd(4)+qd(6)*S5;
    OMcp8_26 = OMcp8_25+ROcp8_85*qd(6);
    OMcp8_36 = OMcp8_35+ROcp8_95*qd(6);
    OPcp8_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp8_26 = ROcp8_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp8_35*S5-ROcp8_95*qd(4));
    OPcp8_36 = ROcp8_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp8_25*S5-ROcp8_85*qd(4));

% = = Block_1_0_0_9_0_2 = = 
 
% Sensor Kinematics 


    ROcp8_47 = ROcp8_46*C7+S5*S7;
    ROcp8_57 = ROcp8_56*C7+ROcp8_85*S7;
    ROcp8_67 = ROcp8_66*C7+ROcp8_95*S7;
    ROcp8_77 = -(ROcp8_46*S7-S5*C7);
    ROcp8_87 = -(ROcp8_56*S7-ROcp8_85*C7);
    ROcp8_97 = -(ROcp8_66*S7-ROcp8_95*C7);
    ROcp8_18 = ROcp8_16*C8+ROcp8_47*S8;
    ROcp8_28 = ROcp8_26*C8+ROcp8_57*S8;
    ROcp8_38 = ROcp8_36*C8+ROcp8_67*S8;
    ROcp8_48 = -(ROcp8_16*S8-ROcp8_47*C8);
    ROcp8_58 = -(ROcp8_26*S8-ROcp8_57*C8);
    ROcp8_68 = -(ROcp8_36*S8-ROcp8_67*C8);
    ROcp8_19 = ROcp8_18*C9-ROcp8_77*S9;
    ROcp8_29 = ROcp8_28*C9-ROcp8_87*S9;
    ROcp8_39 = ROcp8_38*C9-ROcp8_97*S9;
    ROcp8_79 = ROcp8_18*S9+ROcp8_77*C9;
    ROcp8_89 = ROcp8_28*S9+ROcp8_87*C9;
    ROcp8_99 = ROcp8_38*S9+ROcp8_97*C9;
    RLcp8_17 = ROcp8_16*s.dpt(1,1)+ROcp8_46*s.dpt(2,1);
    RLcp8_27 = ROcp8_26*s.dpt(1,1)+ROcp8_56*s.dpt(2,1);
    RLcp8_37 = ROcp8_36*s.dpt(1,1)+ROcp8_66*s.dpt(2,1);
    OMcp8_17 = OMcp8_16+ROcp8_16*qd(7);
    OMcp8_27 = OMcp8_26+ROcp8_26*qd(7);
    OMcp8_37 = OMcp8_36+ROcp8_36*qd(7);
    ORcp8_17 = OMcp8_26*RLcp8_37-OMcp8_36*RLcp8_27;
    ORcp8_27 = -(OMcp8_16*RLcp8_37-OMcp8_36*RLcp8_17);
    ORcp8_37 = OMcp8_16*RLcp8_27-OMcp8_26*RLcp8_17;
    OPcp8_17 = OPcp8_16+ROcp8_16*qdd(7)+qd(7)*(OMcp8_26*ROcp8_36-OMcp8_36*ROcp8_26);
    OPcp8_27 = OPcp8_26+ROcp8_26*qdd(7)-qd(7)*(OMcp8_16*ROcp8_36-OMcp8_36*ROcp8_16);
    OPcp8_37 = OPcp8_36+ROcp8_36*qdd(7)+qd(7)*(OMcp8_16*ROcp8_26-OMcp8_26*ROcp8_16);
    RLcp8_18 = ROcp8_16*s.dpt(1,5)+ROcp8_47*s.dpt(2,5)+ROcp8_77*s.dpt(3,5);
    RLcp8_28 = ROcp8_26*s.dpt(1,5)+ROcp8_57*s.dpt(2,5)+ROcp8_87*s.dpt(3,5);
    RLcp8_38 = ROcp8_36*s.dpt(1,5)+ROcp8_67*s.dpt(2,5)+ROcp8_97*s.dpt(3,5);
    POcp8_18 = RLcp8_17+RLcp8_18+q(1);
    POcp8_28 = RLcp8_27+RLcp8_28+q(2);
    POcp8_38 = RLcp8_37+RLcp8_38+q(3);
    OMcp8_18 = OMcp8_17+ROcp8_77*qd(8);
    OMcp8_28 = OMcp8_27+ROcp8_87*qd(8);
    OMcp8_38 = OMcp8_37+ROcp8_97*qd(8);
    ORcp8_18 = OMcp8_27*RLcp8_38-OMcp8_37*RLcp8_28;
    ORcp8_28 = -(OMcp8_17*RLcp8_38-OMcp8_37*RLcp8_18);
    ORcp8_38 = OMcp8_17*RLcp8_28-OMcp8_27*RLcp8_18;
    VIcp8_18 = ORcp8_17+ORcp8_18+qd(1);
    VIcp8_28 = ORcp8_27+ORcp8_28+qd(2);
    VIcp8_38 = ORcp8_37+ORcp8_38+qd(3);
    ACcp8_18 = qdd(1)+OMcp8_26*ORcp8_37+OMcp8_27*ORcp8_38-OMcp8_36*ORcp8_27-OMcp8_37*ORcp8_28+OPcp8_26*RLcp8_37+OPcp8_27*RLcp8_38-OPcp8_36*RLcp8_27...
 -OPcp8_37*RLcp8_28;
    ACcp8_28 = qdd(2)-OMcp8_16*ORcp8_37-OMcp8_17*ORcp8_38+OMcp8_36*ORcp8_17+OMcp8_37*ORcp8_18-OPcp8_16*RLcp8_37-OPcp8_17*RLcp8_38+OPcp8_36*RLcp8_17...
 +OPcp8_37*RLcp8_18;
    ACcp8_38 = qdd(3)+OMcp8_16*ORcp8_27+OMcp8_17*ORcp8_28-OMcp8_26*ORcp8_17-OMcp8_27*ORcp8_18+OPcp8_16*RLcp8_27+OPcp8_17*RLcp8_28-OPcp8_26*RLcp8_17...
 -OPcp8_27*RLcp8_18;
    OMcp8_19 = OMcp8_18+ROcp8_48*qd(9);
    OMcp8_29 = OMcp8_28+ROcp8_58*qd(9);
    OMcp8_39 = OMcp8_38+ROcp8_68*qd(9);
    OPcp8_19 = OPcp8_17+ROcp8_48*qdd(9)+ROcp8_77*qdd(8)+qd(8)*(OMcp8_27*ROcp8_97-OMcp8_37*ROcp8_87)+qd(9)*(OMcp8_28*ROcp8_68-OMcp8_38*ROcp8_58);
    OPcp8_29 = OPcp8_27+ROcp8_58*qdd(9)+ROcp8_87*qdd(8)-qd(8)*(OMcp8_17*ROcp8_97-OMcp8_37*ROcp8_77)-qd(9)*(OMcp8_18*ROcp8_68-OMcp8_38*ROcp8_48);
    OPcp8_39 = OPcp8_37+ROcp8_68*qdd(9)+ROcp8_97*qdd(8)+qd(8)*(OMcp8_17*ROcp8_87-OMcp8_27*ROcp8_77)+qd(9)*(OMcp8_18*ROcp8_58-OMcp8_28*ROcp8_48);

% = = Block_1_0_0_9_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp8_18;
    sens.P(2) = POcp8_28;
    sens.P(3) = POcp8_38;
    sens.R(1,1) = ROcp8_19;
    sens.R(1,2) = ROcp8_29;
    sens.R(1,3) = ROcp8_39;
    sens.R(2,1) = ROcp8_48;
    sens.R(2,2) = ROcp8_58;
    sens.R(2,3) = ROcp8_68;
    sens.R(3,1) = ROcp8_79;
    sens.R(3,2) = ROcp8_89;
    sens.R(3,3) = ROcp8_99;
    sens.V(1) = VIcp8_18;
    sens.V(2) = VIcp8_28;
    sens.V(3) = VIcp8_38;
    sens.OM(1) = OMcp8_19;
    sens.OM(2) = OMcp8_29;
    sens.OM(3) = OMcp8_39;
    sens.A(1) = ACcp8_18;
    sens.A(2) = ACcp8_28;
    sens.A(3) = ACcp8_38;
    sens.OMP(1) = OPcp8_19;
    sens.OMP(2) = OPcp8_29;
    sens.OMP(3) = OPcp8_39;
 
% 
case 10, 


% = = Block_1_0_0_10_0_1 = = 
 
% Sensor Kinematics 


    ROcp9_25 = S4*S5;
    ROcp9_35 = -C4*S5;
    ROcp9_85 = -S4*C5;
    ROcp9_95 = C4*C5;
    ROcp9_16 = C5*C6;
    ROcp9_26 = ROcp9_25*C6+C4*S6;
    ROcp9_36 = ROcp9_35*C6+S4*S6;
    ROcp9_46 = -C5*S6;
    ROcp9_56 = -(ROcp9_25*S6-C4*C6);
    ROcp9_66 = -(ROcp9_35*S6-S4*C6);
    OMcp9_25 = qd(5)*C4;
    OMcp9_35 = qd(5)*S4;
    OMcp9_16 = qd(4)+qd(6)*S5;
    OMcp9_26 = OMcp9_25+ROcp9_85*qd(6);
    OMcp9_36 = OMcp9_35+ROcp9_95*qd(6);
    OPcp9_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp9_26 = ROcp9_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp9_35*S5-ROcp9_95*qd(4));
    OPcp9_36 = ROcp9_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp9_25*S5-ROcp9_85*qd(4));

% = = Block_1_0_0_10_0_2 = = 
 
% Sensor Kinematics 


    ROcp9_47 = ROcp9_46*C7+S5*S7;
    ROcp9_57 = ROcp9_56*C7+ROcp9_85*S7;
    ROcp9_67 = ROcp9_66*C7+ROcp9_95*S7;
    ROcp9_77 = -(ROcp9_46*S7-S5*C7);
    ROcp9_87 = -(ROcp9_56*S7-ROcp9_85*C7);
    ROcp9_97 = -(ROcp9_66*S7-ROcp9_95*C7);
    ROcp9_18 = ROcp9_16*C8+ROcp9_47*S8;
    ROcp9_28 = ROcp9_26*C8+ROcp9_57*S8;
    ROcp9_38 = ROcp9_36*C8+ROcp9_67*S8;
    ROcp9_48 = -(ROcp9_16*S8-ROcp9_47*C8);
    ROcp9_58 = -(ROcp9_26*S8-ROcp9_57*C8);
    ROcp9_68 = -(ROcp9_36*S8-ROcp9_67*C8);
    ROcp9_19 = ROcp9_18*C9-ROcp9_77*S9;
    ROcp9_29 = ROcp9_28*C9-ROcp9_87*S9;
    ROcp9_39 = ROcp9_38*C9-ROcp9_97*S9;
    ROcp9_79 = ROcp9_18*S9+ROcp9_77*C9;
    ROcp9_89 = ROcp9_28*S9+ROcp9_87*C9;
    ROcp9_99 = ROcp9_38*S9+ROcp9_97*C9;
    ROcp9_110 = ROcp9_19*C10-ROcp9_79*S10;
    ROcp9_210 = ROcp9_29*C10-ROcp9_89*S10;
    ROcp9_310 = ROcp9_39*C10-ROcp9_99*S10;
    ROcp9_710 = ROcp9_19*S10+ROcp9_79*C10;
    ROcp9_810 = ROcp9_29*S10+ROcp9_89*C10;
    ROcp9_910 = ROcp9_39*S10+ROcp9_99*C10;
    RLcp9_17 = ROcp9_16*s.dpt(1,1)+ROcp9_46*s.dpt(2,1);
    RLcp9_27 = ROcp9_26*s.dpt(1,1)+ROcp9_56*s.dpt(2,1);
    RLcp9_37 = ROcp9_36*s.dpt(1,1)+ROcp9_66*s.dpt(2,1);
    OMcp9_17 = OMcp9_16+ROcp9_16*qd(7);
    OMcp9_27 = OMcp9_26+ROcp9_26*qd(7);
    OMcp9_37 = OMcp9_36+ROcp9_36*qd(7);
    ORcp9_17 = OMcp9_26*RLcp9_37-OMcp9_36*RLcp9_27;
    ORcp9_27 = -(OMcp9_16*RLcp9_37-OMcp9_36*RLcp9_17);
    ORcp9_37 = OMcp9_16*RLcp9_27-OMcp9_26*RLcp9_17;
    OPcp9_17 = OPcp9_16+ROcp9_16*qdd(7)+qd(7)*(OMcp9_26*ROcp9_36-OMcp9_36*ROcp9_26);
    OPcp9_27 = OPcp9_26+ROcp9_26*qdd(7)-qd(7)*(OMcp9_16*ROcp9_36-OMcp9_36*ROcp9_16);
    OPcp9_37 = OPcp9_36+ROcp9_36*qdd(7)+qd(7)*(OMcp9_16*ROcp9_26-OMcp9_26*ROcp9_16);
    RLcp9_18 = ROcp9_16*s.dpt(1,5)+ROcp9_47*s.dpt(2,5)+ROcp9_77*s.dpt(3,5);
    RLcp9_28 = ROcp9_26*s.dpt(1,5)+ROcp9_57*s.dpt(2,5)+ROcp9_87*s.dpt(3,5);
    RLcp9_38 = ROcp9_36*s.dpt(1,5)+ROcp9_67*s.dpt(2,5)+ROcp9_97*s.dpt(3,5);
    OMcp9_18 = OMcp9_17+ROcp9_77*qd(8);
    OMcp9_28 = OMcp9_27+ROcp9_87*qd(8);
    OMcp9_38 = OMcp9_37+ROcp9_97*qd(8);
    ORcp9_18 = OMcp9_27*RLcp9_38-OMcp9_37*RLcp9_28;
    ORcp9_28 = -(OMcp9_17*RLcp9_38-OMcp9_37*RLcp9_18);
    ORcp9_38 = OMcp9_17*RLcp9_28-OMcp9_27*RLcp9_18;
    OMcp9_19 = OMcp9_18+ROcp9_48*qd(9);
    OMcp9_29 = OMcp9_28+ROcp9_58*qd(9);
    OMcp9_39 = OMcp9_38+ROcp9_68*qd(9);
    OPcp9_19 = OPcp9_17+ROcp9_48*qdd(9)+ROcp9_77*qdd(8)+qd(8)*(OMcp9_27*ROcp9_97-OMcp9_37*ROcp9_87)+qd(9)*(OMcp9_28*ROcp9_68-OMcp9_38*ROcp9_58);
    OPcp9_29 = OPcp9_27+ROcp9_58*qdd(9)+ROcp9_87*qdd(8)-qd(8)*(OMcp9_17*ROcp9_97-OMcp9_37*ROcp9_77)-qd(9)*(OMcp9_18*ROcp9_68-OMcp9_38*ROcp9_48);
    OPcp9_39 = OPcp9_37+ROcp9_68*qdd(9)+ROcp9_97*qdd(8)+qd(8)*(OMcp9_17*ROcp9_87-OMcp9_27*ROcp9_77)+qd(9)*(OMcp9_18*ROcp9_58-OMcp9_28*ROcp9_48);
    RLcp9_110 = ROcp9_79*s.dpt(3,7);
    RLcp9_210 = ROcp9_89*s.dpt(3,7);
    RLcp9_310 = ROcp9_99*s.dpt(3,7);
    POcp9_110 = RLcp9_110+RLcp9_17+RLcp9_18+q(1);
    POcp9_210 = RLcp9_210+RLcp9_27+RLcp9_28+q(2);
    POcp9_310 = RLcp9_310+RLcp9_37+RLcp9_38+q(3);
    OMcp9_110 = OMcp9_19+ROcp9_48*qd(10);
    OMcp9_210 = OMcp9_29+ROcp9_58*qd(10);
    OMcp9_310 = OMcp9_39+ROcp9_68*qd(10);
    ORcp9_110 = OMcp9_29*RLcp9_310-OMcp9_39*RLcp9_210;
    ORcp9_210 = -(OMcp9_19*RLcp9_310-OMcp9_39*RLcp9_110);
    ORcp9_310 = OMcp9_19*RLcp9_210-OMcp9_29*RLcp9_110;
    VIcp9_110 = ORcp9_110+ORcp9_17+ORcp9_18+qd(1);
    VIcp9_210 = ORcp9_210+ORcp9_27+ORcp9_28+qd(2);
    VIcp9_310 = ORcp9_310+ORcp9_37+ORcp9_38+qd(3);
    OPcp9_110 = OPcp9_19+ROcp9_48*qdd(10)+qd(10)*(OMcp9_29*ROcp9_68-OMcp9_39*ROcp9_58);
    OPcp9_210 = OPcp9_29+ROcp9_58*qdd(10)-qd(10)*(OMcp9_19*ROcp9_68-OMcp9_39*ROcp9_48);
    OPcp9_310 = OPcp9_39+ROcp9_68*qdd(10)+qd(10)*(OMcp9_19*ROcp9_58-OMcp9_29*ROcp9_48);
    ACcp9_110 = qdd(1)+OMcp9_26*ORcp9_37+OMcp9_27*ORcp9_38+OMcp9_29*ORcp9_310-OMcp9_36*ORcp9_27-OMcp9_37*ORcp9_28-OMcp9_39*ORcp9_210+OPcp9_26*...
 RLcp9_37+OPcp9_27*RLcp9_38+OPcp9_29*RLcp9_310-OPcp9_36*RLcp9_27-OPcp9_37*RLcp9_28-OPcp9_39*RLcp9_210;
    ACcp9_210 = qdd(2)-OMcp9_16*ORcp9_37-OMcp9_17*ORcp9_38-OMcp9_19*ORcp9_310+OMcp9_36*ORcp9_17+OMcp9_37*ORcp9_18+OMcp9_39*ORcp9_110-OPcp9_16*...
 RLcp9_37-OPcp9_17*RLcp9_38-OPcp9_19*RLcp9_310+OPcp9_36*RLcp9_17+OPcp9_37*RLcp9_18+OPcp9_39*RLcp9_110;
    ACcp9_310 = qdd(3)+OMcp9_16*ORcp9_27+OMcp9_17*ORcp9_28+OMcp9_19*ORcp9_210-OMcp9_26*ORcp9_17-OMcp9_27*ORcp9_18-OMcp9_29*ORcp9_110+OPcp9_16*...
 RLcp9_27+OPcp9_17*RLcp9_28+OPcp9_19*RLcp9_210-OPcp9_26*RLcp9_17-OPcp9_27*RLcp9_18-OPcp9_29*RLcp9_110;

% = = Block_1_0_0_10_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp9_110;
    sens.P(2) = POcp9_210;
    sens.P(3) = POcp9_310;
    sens.R(1,1) = ROcp9_110;
    sens.R(1,2) = ROcp9_210;
    sens.R(1,3) = ROcp9_310;
    sens.R(2,1) = ROcp9_48;
    sens.R(2,2) = ROcp9_58;
    sens.R(2,3) = ROcp9_68;
    sens.R(3,1) = ROcp9_710;
    sens.R(3,2) = ROcp9_810;
    sens.R(3,3) = ROcp9_910;
    sens.V(1) = VIcp9_110;
    sens.V(2) = VIcp9_210;
    sens.V(3) = VIcp9_310;
    sens.OM(1) = OMcp9_110;
    sens.OM(2) = OMcp9_210;
    sens.OM(3) = OMcp9_310;
    sens.A(1) = ACcp9_110;
    sens.A(2) = ACcp9_210;
    sens.A(3) = ACcp9_310;
    sens.OMP(1) = OPcp9_110;
    sens.OMP(2) = OPcp9_210;
    sens.OMP(3) = OPcp9_310;
 
% 
case 11, 


% = = Block_1_0_0_11_0_1 = = 
 
% Sensor Kinematics 


    ROcp10_25 = S4*S5;
    ROcp10_35 = -C4*S5;
    ROcp10_85 = -S4*C5;
    ROcp10_95 = C4*C5;
    ROcp10_16 = C5*C6;
    ROcp10_26 = ROcp10_25*C6+C4*S6;
    ROcp10_36 = ROcp10_35*C6+S4*S6;
    ROcp10_46 = -C5*S6;
    ROcp10_56 = -(ROcp10_25*S6-C4*C6);
    ROcp10_66 = -(ROcp10_35*S6-S4*C6);
    OMcp10_25 = qd(5)*C4;
    OMcp10_35 = qd(5)*S4;
    OMcp10_16 = qd(4)+qd(6)*S5;
    OMcp10_26 = OMcp10_25+ROcp10_85*qd(6);
    OMcp10_36 = OMcp10_35+ROcp10_95*qd(6);
    OPcp10_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp10_26 = ROcp10_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp10_35*S5-ROcp10_95*qd(4));
    OPcp10_36 = ROcp10_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp10_25*S5-ROcp10_85*qd(4));

% = = Block_1_0_0_11_0_2 = = 
 
% Sensor Kinematics 


    ROcp10_47 = ROcp10_46*C7+S5*S7;
    ROcp10_57 = ROcp10_56*C7+ROcp10_85*S7;
    ROcp10_67 = ROcp10_66*C7+ROcp10_95*S7;
    ROcp10_77 = -(ROcp10_46*S7-S5*C7);
    ROcp10_87 = -(ROcp10_56*S7-ROcp10_85*C7);
    ROcp10_97 = -(ROcp10_66*S7-ROcp10_95*C7);
    ROcp10_18 = ROcp10_16*C8+ROcp10_47*S8;
    ROcp10_28 = ROcp10_26*C8+ROcp10_57*S8;
    ROcp10_38 = ROcp10_36*C8+ROcp10_67*S8;
    ROcp10_48 = -(ROcp10_16*S8-ROcp10_47*C8);
    ROcp10_58 = -(ROcp10_26*S8-ROcp10_57*C8);
    ROcp10_68 = -(ROcp10_36*S8-ROcp10_67*C8);
    ROcp10_19 = ROcp10_18*C9-ROcp10_77*S9;
    ROcp10_29 = ROcp10_28*C9-ROcp10_87*S9;
    ROcp10_39 = ROcp10_38*C9-ROcp10_97*S9;
    ROcp10_79 = ROcp10_18*S9+ROcp10_77*C9;
    ROcp10_89 = ROcp10_28*S9+ROcp10_87*C9;
    ROcp10_99 = ROcp10_38*S9+ROcp10_97*C9;
    ROcp10_110 = ROcp10_19*C10-ROcp10_79*S10;
    ROcp10_210 = ROcp10_29*C10-ROcp10_89*S10;
    ROcp10_310 = ROcp10_39*C10-ROcp10_99*S10;
    ROcp10_710 = ROcp10_19*S10+ROcp10_79*C10;
    ROcp10_810 = ROcp10_29*S10+ROcp10_89*C10;
    ROcp10_910 = ROcp10_39*S10+ROcp10_99*C10;
    ROcp10_111 = ROcp10_110*C11-ROcp10_710*S11;
    ROcp10_211 = ROcp10_210*C11-ROcp10_810*S11;
    ROcp10_311 = ROcp10_310*C11-ROcp10_910*S11;
    ROcp10_711 = ROcp10_110*S11+ROcp10_710*C11;
    ROcp10_811 = ROcp10_210*S11+ROcp10_810*C11;
    ROcp10_911 = ROcp10_310*S11+ROcp10_910*C11;
    RLcp10_17 = ROcp10_16*s.dpt(1,1)+ROcp10_46*s.dpt(2,1);
    RLcp10_27 = ROcp10_26*s.dpt(1,1)+ROcp10_56*s.dpt(2,1);
    RLcp10_37 = ROcp10_36*s.dpt(1,1)+ROcp10_66*s.dpt(2,1);
    OMcp10_17 = OMcp10_16+ROcp10_16*qd(7);
    OMcp10_27 = OMcp10_26+ROcp10_26*qd(7);
    OMcp10_37 = OMcp10_36+ROcp10_36*qd(7);
    ORcp10_17 = OMcp10_26*RLcp10_37-OMcp10_36*RLcp10_27;
    ORcp10_27 = -(OMcp10_16*RLcp10_37-OMcp10_36*RLcp10_17);
    ORcp10_37 = OMcp10_16*RLcp10_27-OMcp10_26*RLcp10_17;
    OPcp10_17 = OPcp10_16+ROcp10_16*qdd(7)+qd(7)*(OMcp10_26*ROcp10_36-OMcp10_36*ROcp10_26);
    OPcp10_27 = OPcp10_26+ROcp10_26*qdd(7)-qd(7)*(OMcp10_16*ROcp10_36-OMcp10_36*ROcp10_16);
    OPcp10_37 = OPcp10_36+ROcp10_36*qdd(7)+qd(7)*(OMcp10_16*ROcp10_26-OMcp10_26*ROcp10_16);
    RLcp10_18 = ROcp10_16*s.dpt(1,5)+ROcp10_47*s.dpt(2,5)+ROcp10_77*s.dpt(3,5);
    RLcp10_28 = ROcp10_26*s.dpt(1,5)+ROcp10_57*s.dpt(2,5)+ROcp10_87*s.dpt(3,5);
    RLcp10_38 = ROcp10_36*s.dpt(1,5)+ROcp10_67*s.dpt(2,5)+ROcp10_97*s.dpt(3,5);
    OMcp10_18 = OMcp10_17+ROcp10_77*qd(8);
    OMcp10_28 = OMcp10_27+ROcp10_87*qd(8);
    OMcp10_38 = OMcp10_37+ROcp10_97*qd(8);
    ORcp10_18 = OMcp10_27*RLcp10_38-OMcp10_37*RLcp10_28;
    ORcp10_28 = -(OMcp10_17*RLcp10_38-OMcp10_37*RLcp10_18);
    ORcp10_38 = OMcp10_17*RLcp10_28-OMcp10_27*RLcp10_18;
    OMcp10_19 = OMcp10_18+ROcp10_48*qd(9);
    OMcp10_29 = OMcp10_28+ROcp10_58*qd(9);
    OMcp10_39 = OMcp10_38+ROcp10_68*qd(9);
    OPcp10_19 = OPcp10_17+ROcp10_48*qdd(9)+ROcp10_77*qdd(8)+qd(8)*(OMcp10_27*ROcp10_97-OMcp10_37*ROcp10_87)+qd(9)*(OMcp10_28*ROcp10_68-OMcp10_38*...
 ROcp10_58);
    OPcp10_29 = OPcp10_27+ROcp10_58*qdd(9)+ROcp10_87*qdd(8)-qd(8)*(OMcp10_17*ROcp10_97-OMcp10_37*ROcp10_77)-qd(9)*(OMcp10_18*ROcp10_68-OMcp10_38*...
 ROcp10_48);
    OPcp10_39 = OPcp10_37+ROcp10_68*qdd(9)+ROcp10_97*qdd(8)+qd(8)*(OMcp10_17*ROcp10_87-OMcp10_27*ROcp10_77)+qd(9)*(OMcp10_18*ROcp10_58-OMcp10_28*...
 ROcp10_48);
    RLcp10_110 = ROcp10_79*s.dpt(3,7);
    RLcp10_210 = ROcp10_89*s.dpt(3,7);
    RLcp10_310 = ROcp10_99*s.dpt(3,7);
    OMcp10_110 = OMcp10_19+ROcp10_48*qd(10);
    OMcp10_210 = OMcp10_29+ROcp10_58*qd(10);
    OMcp10_310 = OMcp10_39+ROcp10_68*qd(10);
    ORcp10_110 = OMcp10_29*RLcp10_310-OMcp10_39*RLcp10_210;
    ORcp10_210 = -(OMcp10_19*RLcp10_310-OMcp10_39*RLcp10_110);
    ORcp10_310 = OMcp10_19*RLcp10_210-OMcp10_29*RLcp10_110;
    OPcp10_110 = OPcp10_19+ROcp10_48*qdd(10)+qd(10)*(OMcp10_29*ROcp10_68-OMcp10_39*ROcp10_58);
    OPcp10_210 = OPcp10_29+ROcp10_58*qdd(10)-qd(10)*(OMcp10_19*ROcp10_68-OMcp10_39*ROcp10_48);
    OPcp10_310 = OPcp10_39+ROcp10_68*qdd(10)+qd(10)*(OMcp10_19*ROcp10_58-OMcp10_29*ROcp10_48);
    RLcp10_111 = ROcp10_710*s.dpt(3,8);
    RLcp10_211 = ROcp10_810*s.dpt(3,8);
    RLcp10_311 = ROcp10_910*s.dpt(3,8);
    POcp10_111 = RLcp10_110+RLcp10_111+RLcp10_17+RLcp10_18+q(1);
    POcp10_211 = RLcp10_210+RLcp10_211+RLcp10_27+RLcp10_28+q(2);
    POcp10_311 = RLcp10_310+RLcp10_311+RLcp10_37+RLcp10_38+q(3);
    OMcp10_111 = OMcp10_110+ROcp10_48*qd(11);
    OMcp10_211 = OMcp10_210+ROcp10_58*qd(11);
    OMcp10_311 = OMcp10_310+ROcp10_68*qd(11);
    ORcp10_111 = OMcp10_210*RLcp10_311-OMcp10_310*RLcp10_211;
    ORcp10_211 = -(OMcp10_110*RLcp10_311-OMcp10_310*RLcp10_111);
    ORcp10_311 = OMcp10_110*RLcp10_211-OMcp10_210*RLcp10_111;
    VIcp10_111 = ORcp10_110+ORcp10_111+ORcp10_17+ORcp10_18+qd(1);
    VIcp10_211 = ORcp10_210+ORcp10_211+ORcp10_27+ORcp10_28+qd(2);
    VIcp10_311 = ORcp10_310+ORcp10_311+ORcp10_37+ORcp10_38+qd(3);
    OPcp10_111 = OPcp10_110+ROcp10_48*qdd(11)+qd(11)*(OMcp10_210*ROcp10_68-OMcp10_310*ROcp10_58);
    OPcp10_211 = OPcp10_210+ROcp10_58*qdd(11)-qd(11)*(OMcp10_110*ROcp10_68-OMcp10_310*ROcp10_48);
    OPcp10_311 = OPcp10_310+ROcp10_68*qdd(11)+qd(11)*(OMcp10_110*ROcp10_58-OMcp10_210*ROcp10_48);
    ACcp10_111 = qdd(1)+OMcp10_210*ORcp10_311+OMcp10_26*ORcp10_37+OMcp10_27*ORcp10_38+OMcp10_29*ORcp10_310-OMcp10_310*ORcp10_211-OMcp10_36*...
 ORcp10_27-OMcp10_37*ORcp10_28-OMcp10_39*ORcp10_210+OPcp10_210*RLcp10_311+OPcp10_26*RLcp10_37+OPcp10_27*RLcp10_38+OPcp10_29*RLcp10_310-OPcp10_310*...
 RLcp10_211-OPcp10_36*RLcp10_27-OPcp10_37*RLcp10_28-OPcp10_39*RLcp10_210;
    ACcp10_211 = qdd(2)-OMcp10_110*ORcp10_311-OMcp10_16*ORcp10_37-OMcp10_17*ORcp10_38-OMcp10_19*ORcp10_310+OMcp10_310*ORcp10_111+OMcp10_36*...
 ORcp10_17+OMcp10_37*ORcp10_18+OMcp10_39*ORcp10_110-OPcp10_110*RLcp10_311-OPcp10_16*RLcp10_37-OPcp10_17*RLcp10_38-OPcp10_19*RLcp10_310+OPcp10_310*...
 RLcp10_111+OPcp10_36*RLcp10_17+OPcp10_37*RLcp10_18+OPcp10_39*RLcp10_110;
    ACcp10_311 = qdd(3)+OMcp10_110*ORcp10_211+OMcp10_16*ORcp10_27+OMcp10_17*ORcp10_28+OMcp10_19*ORcp10_210-OMcp10_210*ORcp10_111-OMcp10_26*...
 ORcp10_17-OMcp10_27*ORcp10_18-OMcp10_29*ORcp10_110+OPcp10_110*RLcp10_211+OPcp10_16*RLcp10_27+OPcp10_17*RLcp10_28+OPcp10_19*RLcp10_210-OPcp10_210*...
 RLcp10_111-OPcp10_26*RLcp10_17-OPcp10_27*RLcp10_18-OPcp10_29*RLcp10_110;

% = = Block_1_0_0_11_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp10_111;
    sens.P(2) = POcp10_211;
    sens.P(3) = POcp10_311;
    sens.R(1,1) = ROcp10_111;
    sens.R(1,2) = ROcp10_211;
    sens.R(1,3) = ROcp10_311;
    sens.R(2,1) = ROcp10_48;
    sens.R(2,2) = ROcp10_58;
    sens.R(2,3) = ROcp10_68;
    sens.R(3,1) = ROcp10_711;
    sens.R(3,2) = ROcp10_811;
    sens.R(3,3) = ROcp10_911;
    sens.V(1) = VIcp10_111;
    sens.V(2) = VIcp10_211;
    sens.V(3) = VIcp10_311;
    sens.OM(1) = OMcp10_111;
    sens.OM(2) = OMcp10_211;
    sens.OM(3) = OMcp10_311;
    sens.A(1) = ACcp10_111;
    sens.A(2) = ACcp10_211;
    sens.A(3) = ACcp10_311;
    sens.OMP(1) = OPcp10_111;
    sens.OMP(2) = OPcp10_211;
    sens.OMP(3) = OPcp10_311;
 
% 
case 12, 


% = = Block_1_0_0_12_0_1 = = 
 
% Sensor Kinematics 


    ROcp11_25 = S4*S5;
    ROcp11_35 = -C4*S5;
    ROcp11_85 = -S4*C5;
    ROcp11_95 = C4*C5;
    ROcp11_16 = C5*C6;
    ROcp11_26 = ROcp11_25*C6+C4*S6;
    ROcp11_36 = ROcp11_35*C6+S4*S6;
    ROcp11_46 = -C5*S6;
    ROcp11_56 = -(ROcp11_25*S6-C4*C6);
    ROcp11_66 = -(ROcp11_35*S6-S4*C6);
    OMcp11_25 = qd(5)*C4;
    OMcp11_35 = qd(5)*S4;
    OMcp11_16 = qd(4)+qd(6)*S5;
    OMcp11_26 = OMcp11_25+ROcp11_85*qd(6);
    OMcp11_36 = OMcp11_35+ROcp11_95*qd(6);
    OPcp11_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp11_26 = ROcp11_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp11_35*S5-ROcp11_95*qd(4));
    OPcp11_36 = ROcp11_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp11_25*S5-ROcp11_85*qd(4));

% = = Block_1_0_0_12_0_2 = = 
 
% Sensor Kinematics 


    ROcp11_47 = ROcp11_46*C7+S5*S7;
    ROcp11_57 = ROcp11_56*C7+ROcp11_85*S7;
    ROcp11_67 = ROcp11_66*C7+ROcp11_95*S7;
    ROcp11_77 = -(ROcp11_46*S7-S5*C7);
    ROcp11_87 = -(ROcp11_56*S7-ROcp11_85*C7);
    ROcp11_97 = -(ROcp11_66*S7-ROcp11_95*C7);
    ROcp11_18 = ROcp11_16*C8+ROcp11_47*S8;
    ROcp11_28 = ROcp11_26*C8+ROcp11_57*S8;
    ROcp11_38 = ROcp11_36*C8+ROcp11_67*S8;
    ROcp11_48 = -(ROcp11_16*S8-ROcp11_47*C8);
    ROcp11_58 = -(ROcp11_26*S8-ROcp11_57*C8);
    ROcp11_68 = -(ROcp11_36*S8-ROcp11_67*C8);
    ROcp11_19 = ROcp11_18*C9-ROcp11_77*S9;
    ROcp11_29 = ROcp11_28*C9-ROcp11_87*S9;
    ROcp11_39 = ROcp11_38*C9-ROcp11_97*S9;
    ROcp11_79 = ROcp11_18*S9+ROcp11_77*C9;
    ROcp11_89 = ROcp11_28*S9+ROcp11_87*C9;
    ROcp11_99 = ROcp11_38*S9+ROcp11_97*C9;
    ROcp11_110 = ROcp11_19*C10-ROcp11_79*S10;
    ROcp11_210 = ROcp11_29*C10-ROcp11_89*S10;
    ROcp11_310 = ROcp11_39*C10-ROcp11_99*S10;
    ROcp11_710 = ROcp11_19*S10+ROcp11_79*C10;
    ROcp11_810 = ROcp11_29*S10+ROcp11_89*C10;
    ROcp11_910 = ROcp11_39*S10+ROcp11_99*C10;
    ROcp11_111 = ROcp11_110*C11-ROcp11_710*S11;
    ROcp11_211 = ROcp11_210*C11-ROcp11_810*S11;
    ROcp11_311 = ROcp11_310*C11-ROcp11_910*S11;
    ROcp11_711 = ROcp11_110*S11+ROcp11_710*C11;
    ROcp11_811 = ROcp11_210*S11+ROcp11_810*C11;
    ROcp11_911 = ROcp11_310*S11+ROcp11_910*C11;
    ROcp11_412 = ROcp11_48*C12+ROcp11_711*S12;
    ROcp11_512 = ROcp11_58*C12+ROcp11_811*S12;
    ROcp11_612 = ROcp11_68*C12+ROcp11_911*S12;
    ROcp11_712 = -(ROcp11_48*S12-ROcp11_711*C12);
    ROcp11_812 = -(ROcp11_58*S12-ROcp11_811*C12);
    ROcp11_912 = -(ROcp11_68*S12-ROcp11_911*C12);
    RLcp11_17 = ROcp11_16*s.dpt(1,1)+ROcp11_46*s.dpt(2,1);
    RLcp11_27 = ROcp11_26*s.dpt(1,1)+ROcp11_56*s.dpt(2,1);
    RLcp11_37 = ROcp11_36*s.dpt(1,1)+ROcp11_66*s.dpt(2,1);
    OMcp11_17 = OMcp11_16+ROcp11_16*qd(7);
    OMcp11_27 = OMcp11_26+ROcp11_26*qd(7);
    OMcp11_37 = OMcp11_36+ROcp11_36*qd(7);
    ORcp11_17 = OMcp11_26*RLcp11_37-OMcp11_36*RLcp11_27;
    ORcp11_27 = -(OMcp11_16*RLcp11_37-OMcp11_36*RLcp11_17);
    ORcp11_37 = OMcp11_16*RLcp11_27-OMcp11_26*RLcp11_17;
    OPcp11_17 = OPcp11_16+ROcp11_16*qdd(7)+qd(7)*(OMcp11_26*ROcp11_36-OMcp11_36*ROcp11_26);
    OPcp11_27 = OPcp11_26+ROcp11_26*qdd(7)-qd(7)*(OMcp11_16*ROcp11_36-OMcp11_36*ROcp11_16);
    OPcp11_37 = OPcp11_36+ROcp11_36*qdd(7)+qd(7)*(OMcp11_16*ROcp11_26-OMcp11_26*ROcp11_16);
    RLcp11_18 = ROcp11_16*s.dpt(1,5)+ROcp11_47*s.dpt(2,5)+ROcp11_77*s.dpt(3,5);
    RLcp11_28 = ROcp11_26*s.dpt(1,5)+ROcp11_57*s.dpt(2,5)+ROcp11_87*s.dpt(3,5);
    RLcp11_38 = ROcp11_36*s.dpt(1,5)+ROcp11_67*s.dpt(2,5)+ROcp11_97*s.dpt(3,5);
    OMcp11_18 = OMcp11_17+ROcp11_77*qd(8);
    OMcp11_28 = OMcp11_27+ROcp11_87*qd(8);
    OMcp11_38 = OMcp11_37+ROcp11_97*qd(8);
    ORcp11_18 = OMcp11_27*RLcp11_38-OMcp11_37*RLcp11_28;
    ORcp11_28 = -(OMcp11_17*RLcp11_38-OMcp11_37*RLcp11_18);
    ORcp11_38 = OMcp11_17*RLcp11_28-OMcp11_27*RLcp11_18;
    OMcp11_19 = OMcp11_18+ROcp11_48*qd(9);
    OMcp11_29 = OMcp11_28+ROcp11_58*qd(9);
    OMcp11_39 = OMcp11_38+ROcp11_68*qd(9);
    OPcp11_19 = OPcp11_17+ROcp11_48*qdd(9)+ROcp11_77*qdd(8)+qd(8)*(OMcp11_27*ROcp11_97-OMcp11_37*ROcp11_87)+qd(9)*(OMcp11_28*ROcp11_68-OMcp11_38*...
 ROcp11_58);
    OPcp11_29 = OPcp11_27+ROcp11_58*qdd(9)+ROcp11_87*qdd(8)-qd(8)*(OMcp11_17*ROcp11_97-OMcp11_37*ROcp11_77)-qd(9)*(OMcp11_18*ROcp11_68-OMcp11_38*...
 ROcp11_48);
    OPcp11_39 = OPcp11_37+ROcp11_68*qdd(9)+ROcp11_97*qdd(8)+qd(8)*(OMcp11_17*ROcp11_87-OMcp11_27*ROcp11_77)+qd(9)*(OMcp11_18*ROcp11_58-OMcp11_28*...
 ROcp11_48);
    RLcp11_110 = ROcp11_79*s.dpt(3,7);
    RLcp11_210 = ROcp11_89*s.dpt(3,7);
    RLcp11_310 = ROcp11_99*s.dpt(3,7);
    OMcp11_110 = OMcp11_19+ROcp11_48*qd(10);
    OMcp11_210 = OMcp11_29+ROcp11_58*qd(10);
    OMcp11_310 = OMcp11_39+ROcp11_68*qd(10);
    ORcp11_110 = OMcp11_29*RLcp11_310-OMcp11_39*RLcp11_210;
    ORcp11_210 = -(OMcp11_19*RLcp11_310-OMcp11_39*RLcp11_110);
    ORcp11_310 = OMcp11_19*RLcp11_210-OMcp11_29*RLcp11_110;
    OPcp11_110 = OPcp11_19+ROcp11_48*qdd(10)+qd(10)*(OMcp11_29*ROcp11_68-OMcp11_39*ROcp11_58);
    OPcp11_210 = OPcp11_29+ROcp11_58*qdd(10)-qd(10)*(OMcp11_19*ROcp11_68-OMcp11_39*ROcp11_48);
    OPcp11_310 = OPcp11_39+ROcp11_68*qdd(10)+qd(10)*(OMcp11_19*ROcp11_58-OMcp11_29*ROcp11_48);
    RLcp11_111 = ROcp11_710*s.dpt(3,8);
    RLcp11_211 = ROcp11_810*s.dpt(3,8);
    RLcp11_311 = ROcp11_910*s.dpt(3,8);
    POcp11_111 = RLcp11_110+RLcp11_111+RLcp11_17+RLcp11_18+q(1);
    POcp11_211 = RLcp11_210+RLcp11_211+RLcp11_27+RLcp11_28+q(2);
    POcp11_311 = RLcp11_310+RLcp11_311+RLcp11_37+RLcp11_38+q(3);
    OMcp11_111 = OMcp11_110+ROcp11_48*qd(11);
    OMcp11_211 = OMcp11_210+ROcp11_58*qd(11);
    OMcp11_311 = OMcp11_310+ROcp11_68*qd(11);
    ORcp11_111 = OMcp11_210*RLcp11_311-OMcp11_310*RLcp11_211;
    ORcp11_211 = -(OMcp11_110*RLcp11_311-OMcp11_310*RLcp11_111);
    ORcp11_311 = OMcp11_110*RLcp11_211-OMcp11_210*RLcp11_111;
    VIcp11_111 = ORcp11_110+ORcp11_111+ORcp11_17+ORcp11_18+qd(1);
    VIcp11_211 = ORcp11_210+ORcp11_211+ORcp11_27+ORcp11_28+qd(2);
    VIcp11_311 = ORcp11_310+ORcp11_311+ORcp11_37+ORcp11_38+qd(3);
    ACcp11_111 = qdd(1)+OMcp11_210*ORcp11_311+OMcp11_26*ORcp11_37+OMcp11_27*ORcp11_38+OMcp11_29*ORcp11_310-OMcp11_310*ORcp11_211-OMcp11_36*...
 ORcp11_27-OMcp11_37*ORcp11_28-OMcp11_39*ORcp11_210+OPcp11_210*RLcp11_311+OPcp11_26*RLcp11_37+OPcp11_27*RLcp11_38+OPcp11_29*RLcp11_310-OPcp11_310*...
 RLcp11_211-OPcp11_36*RLcp11_27-OPcp11_37*RLcp11_28-OPcp11_39*RLcp11_210;
    ACcp11_211 = qdd(2)-OMcp11_110*ORcp11_311-OMcp11_16*ORcp11_37-OMcp11_17*ORcp11_38-OMcp11_19*ORcp11_310+OMcp11_310*ORcp11_111+OMcp11_36*...
 ORcp11_17+OMcp11_37*ORcp11_18+OMcp11_39*ORcp11_110-OPcp11_110*RLcp11_311-OPcp11_16*RLcp11_37-OPcp11_17*RLcp11_38-OPcp11_19*RLcp11_310+OPcp11_310*...
 RLcp11_111+OPcp11_36*RLcp11_17+OPcp11_37*RLcp11_18+OPcp11_39*RLcp11_110;
    ACcp11_311 = qdd(3)+OMcp11_110*ORcp11_211+OMcp11_16*ORcp11_27+OMcp11_17*ORcp11_28+OMcp11_19*ORcp11_210-OMcp11_210*ORcp11_111-OMcp11_26*...
 ORcp11_17-OMcp11_27*ORcp11_18-OMcp11_29*ORcp11_110+OPcp11_110*RLcp11_211+OPcp11_16*RLcp11_27+OPcp11_17*RLcp11_28+OPcp11_19*RLcp11_210-OPcp11_210*...
 RLcp11_111-OPcp11_26*RLcp11_17-OPcp11_27*RLcp11_18-OPcp11_29*RLcp11_110;
    OMcp11_112 = OMcp11_111+ROcp11_111*qd(12);
    OMcp11_212 = OMcp11_211+ROcp11_211*qd(12);
    OMcp11_312 = OMcp11_311+ROcp11_311*qd(12);
    OPcp11_112 = OPcp11_110+ROcp11_111*qdd(12)+ROcp11_48*qdd(11)+qd(11)*(OMcp11_210*ROcp11_68-OMcp11_310*ROcp11_58)+qd(12)*(OMcp11_211*ROcp11_311-...
 OMcp11_311*ROcp11_211);
    OPcp11_212 = OPcp11_210+ROcp11_211*qdd(12)+ROcp11_58*qdd(11)-qd(11)*(OMcp11_110*ROcp11_68-OMcp11_310*ROcp11_48)-qd(12)*(OMcp11_111*ROcp11_311-...
 OMcp11_311*ROcp11_111);
    OPcp11_312 = OPcp11_310+ROcp11_311*qdd(12)+ROcp11_68*qdd(11)+qd(11)*(OMcp11_110*ROcp11_58-OMcp11_210*ROcp11_48)+qd(12)*(OMcp11_111*ROcp11_211-...
 OMcp11_211*ROcp11_111);

% = = Block_1_0_0_12_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp11_111;
    sens.P(2) = POcp11_211;
    sens.P(3) = POcp11_311;
    sens.R(1,1) = ROcp11_111;
    sens.R(1,2) = ROcp11_211;
    sens.R(1,3) = ROcp11_311;
    sens.R(2,1) = ROcp11_412;
    sens.R(2,2) = ROcp11_512;
    sens.R(2,3) = ROcp11_612;
    sens.R(3,1) = ROcp11_712;
    sens.R(3,2) = ROcp11_812;
    sens.R(3,3) = ROcp11_912;
    sens.V(1) = VIcp11_111;
    sens.V(2) = VIcp11_211;
    sens.V(3) = VIcp11_311;
    sens.OM(1) = OMcp11_112;
    sens.OM(2) = OMcp11_212;
    sens.OM(3) = OMcp11_312;
    sens.A(1) = ACcp11_111;
    sens.A(2) = ACcp11_211;
    sens.A(3) = ACcp11_311;
    sens.OMP(1) = OPcp11_112;
    sens.OMP(2) = OPcp11_212;
    sens.OMP(3) = OPcp11_312;
 
% 
case 13, 


% = = Block_1_0_0_13_0_1 = = 
 
% Sensor Kinematics 


    ROcp12_25 = S4*S5;
    ROcp12_35 = -C4*S5;
    ROcp12_85 = -S4*C5;
    ROcp12_95 = C4*C5;
    ROcp12_16 = C5*C6;
    ROcp12_26 = ROcp12_25*C6+C4*S6;
    ROcp12_36 = ROcp12_35*C6+S4*S6;
    ROcp12_46 = -C5*S6;
    ROcp12_56 = -(ROcp12_25*S6-C4*C6);
    ROcp12_66 = -(ROcp12_35*S6-S4*C6);
    OMcp12_25 = qd(5)*C4;
    OMcp12_35 = qd(5)*S4;
    OMcp12_16 = qd(4)+qd(6)*S5;
    OMcp12_26 = OMcp12_25+ROcp12_85*qd(6);
    OMcp12_36 = OMcp12_35+ROcp12_95*qd(6);
    OPcp12_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp12_26 = ROcp12_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp12_35*S5-ROcp12_95*qd(4));
    OPcp12_36 = ROcp12_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp12_25*S5-ROcp12_85*qd(4));

% = = Block_1_0_0_13_0_2 = = 
 
% Sensor Kinematics 


    ROcp12_47 = ROcp12_46*C7+S5*S7;
    ROcp12_57 = ROcp12_56*C7+ROcp12_85*S7;
    ROcp12_67 = ROcp12_66*C7+ROcp12_95*S7;
    ROcp12_77 = -(ROcp12_46*S7-S5*C7);
    ROcp12_87 = -(ROcp12_56*S7-ROcp12_85*C7);
    ROcp12_97 = -(ROcp12_66*S7-ROcp12_95*C7);
    ROcp12_18 = ROcp12_16*C8+ROcp12_47*S8;
    ROcp12_28 = ROcp12_26*C8+ROcp12_57*S8;
    ROcp12_38 = ROcp12_36*C8+ROcp12_67*S8;
    ROcp12_48 = -(ROcp12_16*S8-ROcp12_47*C8);
    ROcp12_58 = -(ROcp12_26*S8-ROcp12_57*C8);
    ROcp12_68 = -(ROcp12_36*S8-ROcp12_67*C8);
    ROcp12_19 = ROcp12_18*C9-ROcp12_77*S9;
    ROcp12_29 = ROcp12_28*C9-ROcp12_87*S9;
    ROcp12_39 = ROcp12_38*C9-ROcp12_97*S9;
    ROcp12_79 = ROcp12_18*S9+ROcp12_77*C9;
    ROcp12_89 = ROcp12_28*S9+ROcp12_87*C9;
    ROcp12_99 = ROcp12_38*S9+ROcp12_97*C9;
    ROcp12_110 = ROcp12_19*C10-ROcp12_79*S10;
    ROcp12_210 = ROcp12_29*C10-ROcp12_89*S10;
    ROcp12_310 = ROcp12_39*C10-ROcp12_99*S10;
    ROcp12_710 = ROcp12_19*S10+ROcp12_79*C10;
    ROcp12_810 = ROcp12_29*S10+ROcp12_89*C10;
    ROcp12_910 = ROcp12_39*S10+ROcp12_99*C10;
    ROcp12_111 = ROcp12_110*C11-ROcp12_710*S11;
    ROcp12_211 = ROcp12_210*C11-ROcp12_810*S11;
    ROcp12_311 = ROcp12_310*C11-ROcp12_910*S11;
    ROcp12_711 = ROcp12_110*S11+ROcp12_710*C11;
    ROcp12_811 = ROcp12_210*S11+ROcp12_810*C11;
    ROcp12_911 = ROcp12_310*S11+ROcp12_910*C11;
    ROcp12_412 = ROcp12_48*C12+ROcp12_711*S12;
    ROcp12_512 = ROcp12_58*C12+ROcp12_811*S12;
    ROcp12_612 = ROcp12_68*C12+ROcp12_911*S12;
    ROcp12_712 = -(ROcp12_48*S12-ROcp12_711*C12);
    ROcp12_812 = -(ROcp12_58*S12-ROcp12_811*C12);
    ROcp12_912 = -(ROcp12_68*S12-ROcp12_911*C12);
    ROcp12_113 = ROcp12_111*C13+ROcp12_412*S13;
    ROcp12_213 = ROcp12_211*C13+ROcp12_512*S13;
    ROcp12_313 = ROcp12_311*C13+ROcp12_612*S13;
    ROcp12_413 = -(ROcp12_111*S13-ROcp12_412*C13);
    ROcp12_513 = -(ROcp12_211*S13-ROcp12_512*C13);
    ROcp12_613 = -(ROcp12_311*S13-ROcp12_612*C13);
    RLcp12_17 = ROcp12_16*s.dpt(1,1)+ROcp12_46*s.dpt(2,1);
    RLcp12_27 = ROcp12_26*s.dpt(1,1)+ROcp12_56*s.dpt(2,1);
    RLcp12_37 = ROcp12_36*s.dpt(1,1)+ROcp12_66*s.dpt(2,1);
    OMcp12_17 = OMcp12_16+ROcp12_16*qd(7);
    OMcp12_27 = OMcp12_26+ROcp12_26*qd(7);
    OMcp12_37 = OMcp12_36+ROcp12_36*qd(7);
    ORcp12_17 = OMcp12_26*RLcp12_37-OMcp12_36*RLcp12_27;
    ORcp12_27 = -(OMcp12_16*RLcp12_37-OMcp12_36*RLcp12_17);
    ORcp12_37 = OMcp12_16*RLcp12_27-OMcp12_26*RLcp12_17;
    OPcp12_17 = OPcp12_16+ROcp12_16*qdd(7)+qd(7)*(OMcp12_26*ROcp12_36-OMcp12_36*ROcp12_26);
    OPcp12_27 = OPcp12_26+ROcp12_26*qdd(7)-qd(7)*(OMcp12_16*ROcp12_36-OMcp12_36*ROcp12_16);
    OPcp12_37 = OPcp12_36+ROcp12_36*qdd(7)+qd(7)*(OMcp12_16*ROcp12_26-OMcp12_26*ROcp12_16);
    RLcp12_18 = ROcp12_16*s.dpt(1,5)+ROcp12_47*s.dpt(2,5)+ROcp12_77*s.dpt(3,5);
    RLcp12_28 = ROcp12_26*s.dpt(1,5)+ROcp12_57*s.dpt(2,5)+ROcp12_87*s.dpt(3,5);
    RLcp12_38 = ROcp12_36*s.dpt(1,5)+ROcp12_67*s.dpt(2,5)+ROcp12_97*s.dpt(3,5);
    OMcp12_18 = OMcp12_17+ROcp12_77*qd(8);
    OMcp12_28 = OMcp12_27+ROcp12_87*qd(8);
    OMcp12_38 = OMcp12_37+ROcp12_97*qd(8);
    ORcp12_18 = OMcp12_27*RLcp12_38-OMcp12_37*RLcp12_28;
    ORcp12_28 = -(OMcp12_17*RLcp12_38-OMcp12_37*RLcp12_18);
    ORcp12_38 = OMcp12_17*RLcp12_28-OMcp12_27*RLcp12_18;
    OMcp12_19 = OMcp12_18+ROcp12_48*qd(9);
    OMcp12_29 = OMcp12_28+ROcp12_58*qd(9);
    OMcp12_39 = OMcp12_38+ROcp12_68*qd(9);
    OPcp12_19 = OPcp12_17+ROcp12_48*qdd(9)+ROcp12_77*qdd(8)+qd(8)*(OMcp12_27*ROcp12_97-OMcp12_37*ROcp12_87)+qd(9)*(OMcp12_28*ROcp12_68-OMcp12_38*...
 ROcp12_58);
    OPcp12_29 = OPcp12_27+ROcp12_58*qdd(9)+ROcp12_87*qdd(8)-qd(8)*(OMcp12_17*ROcp12_97-OMcp12_37*ROcp12_77)-qd(9)*(OMcp12_18*ROcp12_68-OMcp12_38*...
 ROcp12_48);
    OPcp12_39 = OPcp12_37+ROcp12_68*qdd(9)+ROcp12_97*qdd(8)+qd(8)*(OMcp12_17*ROcp12_87-OMcp12_27*ROcp12_77)+qd(9)*(OMcp12_18*ROcp12_58-OMcp12_28*...
 ROcp12_48);
    RLcp12_110 = ROcp12_79*s.dpt(3,7);
    RLcp12_210 = ROcp12_89*s.dpt(3,7);
    RLcp12_310 = ROcp12_99*s.dpt(3,7);
    OMcp12_110 = OMcp12_19+ROcp12_48*qd(10);
    OMcp12_210 = OMcp12_29+ROcp12_58*qd(10);
    OMcp12_310 = OMcp12_39+ROcp12_68*qd(10);
    ORcp12_110 = OMcp12_29*RLcp12_310-OMcp12_39*RLcp12_210;
    ORcp12_210 = -(OMcp12_19*RLcp12_310-OMcp12_39*RLcp12_110);
    ORcp12_310 = OMcp12_19*RLcp12_210-OMcp12_29*RLcp12_110;
    OPcp12_110 = OPcp12_19+ROcp12_48*qdd(10)+qd(10)*(OMcp12_29*ROcp12_68-OMcp12_39*ROcp12_58);
    OPcp12_210 = OPcp12_29+ROcp12_58*qdd(10)-qd(10)*(OMcp12_19*ROcp12_68-OMcp12_39*ROcp12_48);
    OPcp12_310 = OPcp12_39+ROcp12_68*qdd(10)+qd(10)*(OMcp12_19*ROcp12_58-OMcp12_29*ROcp12_48);
    RLcp12_111 = ROcp12_710*s.dpt(3,8);
    RLcp12_211 = ROcp12_810*s.dpt(3,8);
    RLcp12_311 = ROcp12_910*s.dpt(3,8);
    OMcp12_111 = OMcp12_110+ROcp12_48*qd(11);
    OMcp12_211 = OMcp12_210+ROcp12_58*qd(11);
    OMcp12_311 = OMcp12_310+ROcp12_68*qd(11);
    ORcp12_111 = OMcp12_210*RLcp12_311-OMcp12_310*RLcp12_211;
    ORcp12_211 = -(OMcp12_110*RLcp12_311-OMcp12_310*RLcp12_111);
    ORcp12_311 = OMcp12_110*RLcp12_211-OMcp12_210*RLcp12_111;
    OMcp12_112 = OMcp12_111+ROcp12_111*qd(12);
    OMcp12_212 = OMcp12_211+ROcp12_211*qd(12);
    OMcp12_312 = OMcp12_311+ROcp12_311*qd(12);
    OPcp12_112 = OPcp12_110+ROcp12_111*qdd(12)+ROcp12_48*qdd(11)+qd(11)*(OMcp12_210*ROcp12_68-OMcp12_310*ROcp12_58)+qd(12)*(OMcp12_211*ROcp12_311-...
 OMcp12_311*ROcp12_211);
    OPcp12_212 = OPcp12_210+ROcp12_211*qdd(12)+ROcp12_58*qdd(11)-qd(11)*(OMcp12_110*ROcp12_68-OMcp12_310*ROcp12_48)-qd(12)*(OMcp12_111*ROcp12_311-...
 OMcp12_311*ROcp12_111);
    OPcp12_312 = OPcp12_310+ROcp12_311*qdd(12)+ROcp12_68*qdd(11)+qd(11)*(OMcp12_110*ROcp12_58-OMcp12_210*ROcp12_48)+qd(12)*(OMcp12_111*ROcp12_211-...
 OMcp12_211*ROcp12_111);
    RLcp12_113 = ROcp12_111*s.dpt(1,10)+ROcp12_712*s.dpt(3,10);
    RLcp12_213 = ROcp12_211*s.dpt(1,10)+ROcp12_812*s.dpt(3,10);
    RLcp12_313 = ROcp12_311*s.dpt(1,10)+ROcp12_912*s.dpt(3,10);
    POcp12_113 = RLcp12_110+RLcp12_111+RLcp12_113+RLcp12_17+RLcp12_18+q(1);
    POcp12_213 = RLcp12_210+RLcp12_211+RLcp12_213+RLcp12_27+RLcp12_28+q(2);
    POcp12_313 = RLcp12_310+RLcp12_311+RLcp12_313+RLcp12_37+RLcp12_38+q(3);
    ORcp12_113 = OMcp12_212*RLcp12_313-OMcp12_312*RLcp12_213;
    ORcp12_213 = -(OMcp12_112*RLcp12_313-OMcp12_312*RLcp12_113);
    ORcp12_313 = OMcp12_112*RLcp12_213-OMcp12_212*RLcp12_113;
    VIcp12_113 = ORcp12_110+ORcp12_111+ORcp12_113+ORcp12_17+ORcp12_18+qd(1);
    VIcp12_213 = ORcp12_210+ORcp12_211+ORcp12_213+ORcp12_27+ORcp12_28+qd(2);
    VIcp12_313 = ORcp12_310+ORcp12_311+ORcp12_313+ORcp12_37+ORcp12_38+qd(3);
    ACcp12_113 = qdd(1)+OMcp12_210*ORcp12_311+OMcp12_212*ORcp12_313+OMcp12_26*ORcp12_37+OMcp12_27*ORcp12_38+OMcp12_29*ORcp12_310-OMcp12_310*...
 ORcp12_211-OMcp12_312*ORcp12_213-OMcp12_36*ORcp12_27-OMcp12_37*ORcp12_28-OMcp12_39*ORcp12_210+OPcp12_210*RLcp12_311+OPcp12_212*RLcp12_313+OPcp12_26*...
 RLcp12_37+OPcp12_27*RLcp12_38+OPcp12_29*RLcp12_310-OPcp12_310*RLcp12_211-OPcp12_312*RLcp12_213-OPcp12_36*RLcp12_27-OPcp12_37*RLcp12_28-OPcp12_39*...
 RLcp12_210;
    ACcp12_213 = qdd(2)-OMcp12_110*ORcp12_311-OMcp12_112*ORcp12_313-OMcp12_16*ORcp12_37-OMcp12_17*ORcp12_38-OMcp12_19*ORcp12_310+OMcp12_310*...
 ORcp12_111+OMcp12_312*ORcp12_113+OMcp12_36*ORcp12_17+OMcp12_37*ORcp12_18+OMcp12_39*ORcp12_110-OPcp12_110*RLcp12_311-OPcp12_112*RLcp12_313-OPcp12_16*...
 RLcp12_37-OPcp12_17*RLcp12_38-OPcp12_19*RLcp12_310+OPcp12_310*RLcp12_111+OPcp12_312*RLcp12_113+OPcp12_36*RLcp12_17+OPcp12_37*RLcp12_18+OPcp12_39*...
 RLcp12_110;
    ACcp12_313 = qdd(3)+OMcp12_110*ORcp12_211+OMcp12_112*ORcp12_213+OMcp12_16*ORcp12_27+OMcp12_17*ORcp12_28+OMcp12_19*ORcp12_210-OMcp12_210*...
 ORcp12_111-OMcp12_212*ORcp12_113-OMcp12_26*ORcp12_17-OMcp12_27*ORcp12_18-OMcp12_29*ORcp12_110+OPcp12_110*RLcp12_211+OPcp12_112*RLcp12_213+OPcp12_16*...
 RLcp12_27+OPcp12_17*RLcp12_28+OPcp12_19*RLcp12_210-OPcp12_210*RLcp12_111-OPcp12_212*RLcp12_113-OPcp12_26*RLcp12_17-OPcp12_27*RLcp12_18-OPcp12_29*...
 RLcp12_110;

% = = Block_1_0_0_13_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp12_113;
    sens.P(2) = POcp12_213;
    sens.P(3) = POcp12_313;
    sens.R(1,1) = ROcp12_113;
    sens.R(1,2) = ROcp12_213;
    sens.R(1,3) = ROcp12_313;
    sens.R(2,1) = ROcp12_413;
    sens.R(2,2) = ROcp12_513;
    sens.R(2,3) = ROcp12_613;
    sens.R(3,1) = ROcp12_712;
    sens.R(3,2) = ROcp12_812;
    sens.R(3,3) = ROcp12_912;
    sens.V(1) = VIcp12_113;
    sens.V(2) = VIcp12_213;
    sens.V(3) = VIcp12_313;
    sens.OM(1) = OMcp12_112;
    sens.OM(2) = OMcp12_212;
    sens.OM(3) = OMcp12_312;
    sens.A(1) = ACcp12_113;
    sens.A(2) = ACcp12_213;
    sens.A(3) = ACcp12_313;
    sens.OMP(1) = OPcp12_112;
    sens.OMP(2) = OPcp12_212;
    sens.OMP(3) = OPcp12_312;
 
% 
case 14, 


% = = Block_1_0_0_14_0_1 = = 
 
% Sensor Kinematics 


    ROcp13_25 = S4*S5;
    ROcp13_35 = -C4*S5;
    ROcp13_85 = -S4*C5;
    ROcp13_95 = C4*C5;
    ROcp13_16 = C5*C6;
    ROcp13_26 = ROcp13_25*C6+C4*S6;
    ROcp13_36 = ROcp13_35*C6+S4*S6;
    ROcp13_46 = -C5*S6;
    ROcp13_56 = -(ROcp13_25*S6-C4*C6);
    ROcp13_66 = -(ROcp13_35*S6-S4*C6);
    OMcp13_25 = qd(5)*C4;
    OMcp13_35 = qd(5)*S4;
    OMcp13_16 = qd(4)+qd(6)*S5;
    OMcp13_26 = OMcp13_25+ROcp13_85*qd(6);
    OMcp13_36 = OMcp13_35+ROcp13_95*qd(6);
    OPcp13_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp13_26 = ROcp13_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp13_35*S5-ROcp13_95*qd(4));
    OPcp13_36 = ROcp13_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp13_25*S5-ROcp13_85*qd(4));

% = = Block_1_0_0_14_0_2 = = 
 
% Sensor Kinematics 


    ROcp13_47 = ROcp13_46*C7+S5*S7;
    ROcp13_57 = ROcp13_56*C7+ROcp13_85*S7;
    ROcp13_67 = ROcp13_66*C7+ROcp13_95*S7;
    ROcp13_77 = -(ROcp13_46*S7-S5*C7);
    ROcp13_87 = -(ROcp13_56*S7-ROcp13_85*C7);
    ROcp13_97 = -(ROcp13_66*S7-ROcp13_95*C7);
    ROcp13_18 = ROcp13_16*C8+ROcp13_47*S8;
    ROcp13_28 = ROcp13_26*C8+ROcp13_57*S8;
    ROcp13_38 = ROcp13_36*C8+ROcp13_67*S8;
    ROcp13_48 = -(ROcp13_16*S8-ROcp13_47*C8);
    ROcp13_58 = -(ROcp13_26*S8-ROcp13_57*C8);
    ROcp13_68 = -(ROcp13_36*S8-ROcp13_67*C8);
    ROcp13_19 = ROcp13_18*C9-ROcp13_77*S9;
    ROcp13_29 = ROcp13_28*C9-ROcp13_87*S9;
    ROcp13_39 = ROcp13_38*C9-ROcp13_97*S9;
    ROcp13_79 = ROcp13_18*S9+ROcp13_77*C9;
    ROcp13_89 = ROcp13_28*S9+ROcp13_87*C9;
    ROcp13_99 = ROcp13_38*S9+ROcp13_97*C9;
    ROcp13_110 = ROcp13_19*C10-ROcp13_79*S10;
    ROcp13_210 = ROcp13_29*C10-ROcp13_89*S10;
    ROcp13_310 = ROcp13_39*C10-ROcp13_99*S10;
    ROcp13_710 = ROcp13_19*S10+ROcp13_79*C10;
    ROcp13_810 = ROcp13_29*S10+ROcp13_89*C10;
    ROcp13_910 = ROcp13_39*S10+ROcp13_99*C10;
    ROcp13_111 = ROcp13_110*C11-ROcp13_710*S11;
    ROcp13_211 = ROcp13_210*C11-ROcp13_810*S11;
    ROcp13_311 = ROcp13_310*C11-ROcp13_910*S11;
    ROcp13_711 = ROcp13_110*S11+ROcp13_710*C11;
    ROcp13_811 = ROcp13_210*S11+ROcp13_810*C11;
    ROcp13_911 = ROcp13_310*S11+ROcp13_910*C11;
    ROcp13_412 = ROcp13_48*C12+ROcp13_711*S12;
    ROcp13_512 = ROcp13_58*C12+ROcp13_811*S12;
    ROcp13_612 = ROcp13_68*C12+ROcp13_911*S12;
    ROcp13_712 = -(ROcp13_48*S12-ROcp13_711*C12);
    ROcp13_812 = -(ROcp13_58*S12-ROcp13_811*C12);
    ROcp13_912 = -(ROcp13_68*S12-ROcp13_911*C12);
    ROcp13_113 = ROcp13_111*C13+ROcp13_412*S13;
    ROcp13_213 = ROcp13_211*C13+ROcp13_512*S13;
    ROcp13_313 = ROcp13_311*C13+ROcp13_612*S13;
    ROcp13_413 = -(ROcp13_111*S13-ROcp13_412*C13);
    ROcp13_513 = -(ROcp13_211*S13-ROcp13_512*C13);
    ROcp13_613 = -(ROcp13_311*S13-ROcp13_612*C13);
    ROcp13_114 = ROcp13_113*C14-ROcp13_712*S14;
    ROcp13_214 = ROcp13_213*C14-ROcp13_812*S14;
    ROcp13_314 = ROcp13_313*C14-ROcp13_912*S14;
    ROcp13_714 = ROcp13_113*S14+ROcp13_712*C14;
    ROcp13_814 = ROcp13_213*S14+ROcp13_812*C14;
    ROcp13_914 = ROcp13_313*S14+ROcp13_912*C14;
    RLcp13_17 = ROcp13_16*s.dpt(1,1)+ROcp13_46*s.dpt(2,1);
    RLcp13_27 = ROcp13_26*s.dpt(1,1)+ROcp13_56*s.dpt(2,1);
    RLcp13_37 = ROcp13_36*s.dpt(1,1)+ROcp13_66*s.dpt(2,1);
    OMcp13_17 = OMcp13_16+ROcp13_16*qd(7);
    OMcp13_27 = OMcp13_26+ROcp13_26*qd(7);
    OMcp13_37 = OMcp13_36+ROcp13_36*qd(7);
    ORcp13_17 = OMcp13_26*RLcp13_37-OMcp13_36*RLcp13_27;
    ORcp13_27 = -(OMcp13_16*RLcp13_37-OMcp13_36*RLcp13_17);
    ORcp13_37 = OMcp13_16*RLcp13_27-OMcp13_26*RLcp13_17;
    OPcp13_17 = OPcp13_16+ROcp13_16*qdd(7)+qd(7)*(OMcp13_26*ROcp13_36-OMcp13_36*ROcp13_26);
    OPcp13_27 = OPcp13_26+ROcp13_26*qdd(7)-qd(7)*(OMcp13_16*ROcp13_36-OMcp13_36*ROcp13_16);
    OPcp13_37 = OPcp13_36+ROcp13_36*qdd(7)+qd(7)*(OMcp13_16*ROcp13_26-OMcp13_26*ROcp13_16);
    RLcp13_18 = ROcp13_16*s.dpt(1,5)+ROcp13_47*s.dpt(2,5)+ROcp13_77*s.dpt(3,5);
    RLcp13_28 = ROcp13_26*s.dpt(1,5)+ROcp13_57*s.dpt(2,5)+ROcp13_87*s.dpt(3,5);
    RLcp13_38 = ROcp13_36*s.dpt(1,5)+ROcp13_67*s.dpt(2,5)+ROcp13_97*s.dpt(3,5);
    OMcp13_18 = OMcp13_17+ROcp13_77*qd(8);
    OMcp13_28 = OMcp13_27+ROcp13_87*qd(8);
    OMcp13_38 = OMcp13_37+ROcp13_97*qd(8);
    ORcp13_18 = OMcp13_27*RLcp13_38-OMcp13_37*RLcp13_28;
    ORcp13_28 = -(OMcp13_17*RLcp13_38-OMcp13_37*RLcp13_18);
    ORcp13_38 = OMcp13_17*RLcp13_28-OMcp13_27*RLcp13_18;
    OMcp13_19 = OMcp13_18+ROcp13_48*qd(9);
    OMcp13_29 = OMcp13_28+ROcp13_58*qd(9);
    OMcp13_39 = OMcp13_38+ROcp13_68*qd(9);
    OPcp13_19 = OPcp13_17+ROcp13_48*qdd(9)+ROcp13_77*qdd(8)+qd(8)*(OMcp13_27*ROcp13_97-OMcp13_37*ROcp13_87)+qd(9)*(OMcp13_28*ROcp13_68-OMcp13_38*...
 ROcp13_58);
    OPcp13_29 = OPcp13_27+ROcp13_58*qdd(9)+ROcp13_87*qdd(8)-qd(8)*(OMcp13_17*ROcp13_97-OMcp13_37*ROcp13_77)-qd(9)*(OMcp13_18*ROcp13_68-OMcp13_38*...
 ROcp13_48);
    OPcp13_39 = OPcp13_37+ROcp13_68*qdd(9)+ROcp13_97*qdd(8)+qd(8)*(OMcp13_17*ROcp13_87-OMcp13_27*ROcp13_77)+qd(9)*(OMcp13_18*ROcp13_58-OMcp13_28*...
 ROcp13_48);
    RLcp13_110 = ROcp13_79*s.dpt(3,7);
    RLcp13_210 = ROcp13_89*s.dpt(3,7);
    RLcp13_310 = ROcp13_99*s.dpt(3,7);
    OMcp13_110 = OMcp13_19+ROcp13_48*qd(10);
    OMcp13_210 = OMcp13_29+ROcp13_58*qd(10);
    OMcp13_310 = OMcp13_39+ROcp13_68*qd(10);
    ORcp13_110 = OMcp13_29*RLcp13_310-OMcp13_39*RLcp13_210;
    ORcp13_210 = -(OMcp13_19*RLcp13_310-OMcp13_39*RLcp13_110);
    ORcp13_310 = OMcp13_19*RLcp13_210-OMcp13_29*RLcp13_110;
    OPcp13_110 = OPcp13_19+ROcp13_48*qdd(10)+qd(10)*(OMcp13_29*ROcp13_68-OMcp13_39*ROcp13_58);
    OPcp13_210 = OPcp13_29+ROcp13_58*qdd(10)-qd(10)*(OMcp13_19*ROcp13_68-OMcp13_39*ROcp13_48);
    OPcp13_310 = OPcp13_39+ROcp13_68*qdd(10)+qd(10)*(OMcp13_19*ROcp13_58-OMcp13_29*ROcp13_48);
    RLcp13_111 = ROcp13_710*s.dpt(3,8);
    RLcp13_211 = ROcp13_810*s.dpt(3,8);
    RLcp13_311 = ROcp13_910*s.dpt(3,8);
    OMcp13_111 = OMcp13_110+ROcp13_48*qd(11);
    OMcp13_211 = OMcp13_210+ROcp13_58*qd(11);
    OMcp13_311 = OMcp13_310+ROcp13_68*qd(11);
    ORcp13_111 = OMcp13_210*RLcp13_311-OMcp13_310*RLcp13_211;
    ORcp13_211 = -(OMcp13_110*RLcp13_311-OMcp13_310*RLcp13_111);
    ORcp13_311 = OMcp13_110*RLcp13_211-OMcp13_210*RLcp13_111;
    OMcp13_112 = OMcp13_111+ROcp13_111*qd(12);
    OMcp13_212 = OMcp13_211+ROcp13_211*qd(12);
    OMcp13_312 = OMcp13_311+ROcp13_311*qd(12);
    OPcp13_112 = OPcp13_110+ROcp13_111*qdd(12)+ROcp13_48*qdd(11)+qd(11)*(OMcp13_210*ROcp13_68-OMcp13_310*ROcp13_58)+qd(12)*(OMcp13_211*ROcp13_311-...
 OMcp13_311*ROcp13_211);
    OPcp13_212 = OPcp13_210+ROcp13_211*qdd(12)+ROcp13_58*qdd(11)-qd(11)*(OMcp13_110*ROcp13_68-OMcp13_310*ROcp13_48)-qd(12)*(OMcp13_111*ROcp13_311-...
 OMcp13_311*ROcp13_111);
    OPcp13_312 = OPcp13_310+ROcp13_311*qdd(12)+ROcp13_68*qdd(11)+qd(11)*(OMcp13_110*ROcp13_58-OMcp13_210*ROcp13_48)+qd(12)*(OMcp13_111*ROcp13_211-...
 OMcp13_211*ROcp13_111);
    RLcp13_113 = ROcp13_111*s.dpt(1,10)+ROcp13_712*s.dpt(3,10);
    RLcp13_213 = ROcp13_211*s.dpt(1,10)+ROcp13_812*s.dpt(3,10);
    RLcp13_313 = ROcp13_311*s.dpt(1,10)+ROcp13_912*s.dpt(3,10);
    POcp13_113 = RLcp13_110+RLcp13_111+RLcp13_113+RLcp13_17+RLcp13_18+q(1);
    POcp13_213 = RLcp13_210+RLcp13_211+RLcp13_213+RLcp13_27+RLcp13_28+q(2);
    POcp13_313 = RLcp13_310+RLcp13_311+RLcp13_313+RLcp13_37+RLcp13_38+q(3);
    ORcp13_113 = OMcp13_212*RLcp13_313-OMcp13_312*RLcp13_213;
    ORcp13_213 = -(OMcp13_112*RLcp13_313-OMcp13_312*RLcp13_113);
    ORcp13_313 = OMcp13_112*RLcp13_213-OMcp13_212*RLcp13_113;
    VIcp13_113 = ORcp13_110+ORcp13_111+ORcp13_113+ORcp13_17+ORcp13_18+qd(1);
    VIcp13_213 = ORcp13_210+ORcp13_211+ORcp13_213+ORcp13_27+ORcp13_28+qd(2);
    VIcp13_313 = ORcp13_310+ORcp13_311+ORcp13_313+ORcp13_37+ORcp13_38+qd(3);
    ACcp13_113 = qdd(1)+OMcp13_210*ORcp13_311+OMcp13_212*ORcp13_313+OMcp13_26*ORcp13_37+OMcp13_27*ORcp13_38+OMcp13_29*ORcp13_310-OMcp13_310*...
 ORcp13_211-OMcp13_312*ORcp13_213-OMcp13_36*ORcp13_27-OMcp13_37*ORcp13_28-OMcp13_39*ORcp13_210+OPcp13_210*RLcp13_311+OPcp13_212*RLcp13_313+OPcp13_26*...
 RLcp13_37+OPcp13_27*RLcp13_38+OPcp13_29*RLcp13_310-OPcp13_310*RLcp13_211-OPcp13_312*RLcp13_213-OPcp13_36*RLcp13_27-OPcp13_37*RLcp13_28-OPcp13_39*...
 RLcp13_210;
    ACcp13_213 = qdd(2)-OMcp13_110*ORcp13_311-OMcp13_112*ORcp13_313-OMcp13_16*ORcp13_37-OMcp13_17*ORcp13_38-OMcp13_19*ORcp13_310+OMcp13_310*...
 ORcp13_111+OMcp13_312*ORcp13_113+OMcp13_36*ORcp13_17+OMcp13_37*ORcp13_18+OMcp13_39*ORcp13_110-OPcp13_110*RLcp13_311-OPcp13_112*RLcp13_313-OPcp13_16*...
 RLcp13_37-OPcp13_17*RLcp13_38-OPcp13_19*RLcp13_310+OPcp13_310*RLcp13_111+OPcp13_312*RLcp13_113+OPcp13_36*RLcp13_17+OPcp13_37*RLcp13_18+OPcp13_39*...
 RLcp13_110;
    ACcp13_313 = qdd(3)+OMcp13_110*ORcp13_211+OMcp13_112*ORcp13_213+OMcp13_16*ORcp13_27+OMcp13_17*ORcp13_28+OMcp13_19*ORcp13_210-OMcp13_210*...
 ORcp13_111-OMcp13_212*ORcp13_113-OMcp13_26*ORcp13_17-OMcp13_27*ORcp13_18-OMcp13_29*ORcp13_110+OPcp13_110*RLcp13_211+OPcp13_112*RLcp13_213+OPcp13_16*...
 RLcp13_27+OPcp13_17*RLcp13_28+OPcp13_19*RLcp13_210-OPcp13_210*RLcp13_111-OPcp13_212*RLcp13_113-OPcp13_26*RLcp13_17-OPcp13_27*RLcp13_18-OPcp13_29*...
 RLcp13_110;

% = = Block_1_0_0_14_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp13_113;
    sens.P(2) = POcp13_213;
    sens.P(3) = POcp13_313;
    sens.R(1,1) = ROcp13_114;
    sens.R(1,2) = ROcp13_214;
    sens.R(1,3) = ROcp13_314;
    sens.R(2,1) = ROcp13_413;
    sens.R(2,2) = ROcp13_513;
    sens.R(2,3) = ROcp13_613;
    sens.R(3,1) = ROcp13_714;
    sens.R(3,2) = ROcp13_814;
    sens.R(3,3) = ROcp13_914;
    sens.V(1) = VIcp13_113;
    sens.V(2) = VIcp13_213;
    sens.V(3) = VIcp13_313;
    sens.OM(1) = OMcp13_112;
    sens.OM(2) = OMcp13_212;
    sens.OM(3) = OMcp13_312;
    sens.A(1) = ACcp13_113;
    sens.A(2) = ACcp13_213;
    sens.A(3) = ACcp13_313;
    sens.OMP(1) = OPcp13_112;
    sens.OMP(2) = OPcp13_212;
    sens.OMP(3) = OPcp13_312;
 
% 
case 15, 


% = = Block_1_0_0_15_0_1 = = 
 
% Sensor Kinematics 


    ROcp14_25 = S4*S5;
    ROcp14_35 = -C4*S5;
    ROcp14_85 = -S4*C5;
    ROcp14_95 = C4*C5;
    ROcp14_16 = C5*C6;
    ROcp14_26 = ROcp14_25*C6+C4*S6;
    ROcp14_36 = ROcp14_35*C6+S4*S6;
    ROcp14_46 = -C5*S6;
    ROcp14_56 = -(ROcp14_25*S6-C4*C6);
    ROcp14_66 = -(ROcp14_35*S6-S4*C6);
    OMcp14_25 = qd(5)*C4;
    OMcp14_35 = qd(5)*S4;
    OMcp14_16 = qd(4)+qd(6)*S5;
    OMcp14_26 = OMcp14_25+ROcp14_85*qd(6);
    OMcp14_36 = OMcp14_35+ROcp14_95*qd(6);
    OPcp14_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp14_26 = ROcp14_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp14_35*S5-ROcp14_95*qd(4));
    OPcp14_36 = ROcp14_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp14_25*S5-ROcp14_85*qd(4));

% = = Block_1_0_0_15_0_2 = = 
 
% Sensor Kinematics 


    ROcp14_47 = ROcp14_46*C7+S5*S7;
    ROcp14_57 = ROcp14_56*C7+ROcp14_85*S7;
    ROcp14_67 = ROcp14_66*C7+ROcp14_95*S7;
    ROcp14_77 = -(ROcp14_46*S7-S5*C7);
    ROcp14_87 = -(ROcp14_56*S7-ROcp14_85*C7);
    ROcp14_97 = -(ROcp14_66*S7-ROcp14_95*C7);
    ROcp14_18 = ROcp14_16*C8+ROcp14_47*S8;
    ROcp14_28 = ROcp14_26*C8+ROcp14_57*S8;
    ROcp14_38 = ROcp14_36*C8+ROcp14_67*S8;
    ROcp14_48 = -(ROcp14_16*S8-ROcp14_47*C8);
    ROcp14_58 = -(ROcp14_26*S8-ROcp14_57*C8);
    ROcp14_68 = -(ROcp14_36*S8-ROcp14_67*C8);
    ROcp14_19 = ROcp14_18*C9-ROcp14_77*S9;
    ROcp14_29 = ROcp14_28*C9-ROcp14_87*S9;
    ROcp14_39 = ROcp14_38*C9-ROcp14_97*S9;
    ROcp14_79 = ROcp14_18*S9+ROcp14_77*C9;
    ROcp14_89 = ROcp14_28*S9+ROcp14_87*C9;
    ROcp14_99 = ROcp14_38*S9+ROcp14_97*C9;
    ROcp14_110 = ROcp14_19*C10-ROcp14_79*S10;
    ROcp14_210 = ROcp14_29*C10-ROcp14_89*S10;
    ROcp14_310 = ROcp14_39*C10-ROcp14_99*S10;
    ROcp14_710 = ROcp14_19*S10+ROcp14_79*C10;
    ROcp14_810 = ROcp14_29*S10+ROcp14_89*C10;
    ROcp14_910 = ROcp14_39*S10+ROcp14_99*C10;
    ROcp14_111 = ROcp14_110*C11-ROcp14_710*S11;
    ROcp14_211 = ROcp14_210*C11-ROcp14_810*S11;
    ROcp14_311 = ROcp14_310*C11-ROcp14_910*S11;
    ROcp14_711 = ROcp14_110*S11+ROcp14_710*C11;
    ROcp14_811 = ROcp14_210*S11+ROcp14_810*C11;
    ROcp14_911 = ROcp14_310*S11+ROcp14_910*C11;
    ROcp14_412 = ROcp14_48*C12+ROcp14_711*S12;
    ROcp14_512 = ROcp14_58*C12+ROcp14_811*S12;
    ROcp14_612 = ROcp14_68*C12+ROcp14_911*S12;
    ROcp14_712 = -(ROcp14_48*S12-ROcp14_711*C12);
    ROcp14_812 = -(ROcp14_58*S12-ROcp14_811*C12);
    ROcp14_912 = -(ROcp14_68*S12-ROcp14_911*C12);
    ROcp14_113 = ROcp14_111*C13+ROcp14_412*S13;
    ROcp14_213 = ROcp14_211*C13+ROcp14_512*S13;
    ROcp14_313 = ROcp14_311*C13+ROcp14_612*S13;
    ROcp14_413 = -(ROcp14_111*S13-ROcp14_412*C13);
    ROcp14_513 = -(ROcp14_211*S13-ROcp14_512*C13);
    ROcp14_613 = -(ROcp14_311*S13-ROcp14_612*C13);
    ROcp14_114 = ROcp14_113*C14-ROcp14_712*S14;
    ROcp14_214 = ROcp14_213*C14-ROcp14_812*S14;
    ROcp14_314 = ROcp14_313*C14-ROcp14_912*S14;
    ROcp14_714 = ROcp14_113*S14+ROcp14_712*C14;
    ROcp14_814 = ROcp14_213*S14+ROcp14_812*C14;
    ROcp14_914 = ROcp14_313*S14+ROcp14_912*C14;
    ROcp14_415 = ROcp14_413*C15+ROcp14_714*S15;
    ROcp14_515 = ROcp14_513*C15+ROcp14_814*S15;
    ROcp14_615 = ROcp14_613*C15+ROcp14_914*S15;
    ROcp14_715 = -(ROcp14_413*S15-ROcp14_714*C15);
    ROcp14_815 = -(ROcp14_513*S15-ROcp14_814*C15);
    ROcp14_915 = -(ROcp14_613*S15-ROcp14_914*C15);
    RLcp14_17 = ROcp14_16*s.dpt(1,1)+ROcp14_46*s.dpt(2,1);
    RLcp14_27 = ROcp14_26*s.dpt(1,1)+ROcp14_56*s.dpt(2,1);
    RLcp14_37 = ROcp14_36*s.dpt(1,1)+ROcp14_66*s.dpt(2,1);
    OMcp14_17 = OMcp14_16+ROcp14_16*qd(7);
    OMcp14_27 = OMcp14_26+ROcp14_26*qd(7);
    OMcp14_37 = OMcp14_36+ROcp14_36*qd(7);
    ORcp14_17 = OMcp14_26*RLcp14_37-OMcp14_36*RLcp14_27;
    ORcp14_27 = -(OMcp14_16*RLcp14_37-OMcp14_36*RLcp14_17);
    ORcp14_37 = OMcp14_16*RLcp14_27-OMcp14_26*RLcp14_17;
    OPcp14_17 = OPcp14_16+ROcp14_16*qdd(7)+qd(7)*(OMcp14_26*ROcp14_36-OMcp14_36*ROcp14_26);
    OPcp14_27 = OPcp14_26+ROcp14_26*qdd(7)-qd(7)*(OMcp14_16*ROcp14_36-OMcp14_36*ROcp14_16);
    OPcp14_37 = OPcp14_36+ROcp14_36*qdd(7)+qd(7)*(OMcp14_16*ROcp14_26-OMcp14_26*ROcp14_16);
    RLcp14_18 = ROcp14_16*s.dpt(1,5)+ROcp14_47*s.dpt(2,5)+ROcp14_77*s.dpt(3,5);
    RLcp14_28 = ROcp14_26*s.dpt(1,5)+ROcp14_57*s.dpt(2,5)+ROcp14_87*s.dpt(3,5);
    RLcp14_38 = ROcp14_36*s.dpt(1,5)+ROcp14_67*s.dpt(2,5)+ROcp14_97*s.dpt(3,5);
    OMcp14_18 = OMcp14_17+ROcp14_77*qd(8);
    OMcp14_28 = OMcp14_27+ROcp14_87*qd(8);
    OMcp14_38 = OMcp14_37+ROcp14_97*qd(8);
    ORcp14_18 = OMcp14_27*RLcp14_38-OMcp14_37*RLcp14_28;
    ORcp14_28 = -(OMcp14_17*RLcp14_38-OMcp14_37*RLcp14_18);
    ORcp14_38 = OMcp14_17*RLcp14_28-OMcp14_27*RLcp14_18;
    OMcp14_19 = OMcp14_18+ROcp14_48*qd(9);
    OMcp14_29 = OMcp14_28+ROcp14_58*qd(9);
    OMcp14_39 = OMcp14_38+ROcp14_68*qd(9);
    OPcp14_19 = OPcp14_17+ROcp14_48*qdd(9)+ROcp14_77*qdd(8)+qd(8)*(OMcp14_27*ROcp14_97-OMcp14_37*ROcp14_87)+qd(9)*(OMcp14_28*ROcp14_68-OMcp14_38*...
 ROcp14_58);
    OPcp14_29 = OPcp14_27+ROcp14_58*qdd(9)+ROcp14_87*qdd(8)-qd(8)*(OMcp14_17*ROcp14_97-OMcp14_37*ROcp14_77)-qd(9)*(OMcp14_18*ROcp14_68-OMcp14_38*...
 ROcp14_48);
    OPcp14_39 = OPcp14_37+ROcp14_68*qdd(9)+ROcp14_97*qdd(8)+qd(8)*(OMcp14_17*ROcp14_87-OMcp14_27*ROcp14_77)+qd(9)*(OMcp14_18*ROcp14_58-OMcp14_28*...
 ROcp14_48);
    RLcp14_110 = ROcp14_79*s.dpt(3,7);
    RLcp14_210 = ROcp14_89*s.dpt(3,7);
    RLcp14_310 = ROcp14_99*s.dpt(3,7);
    OMcp14_110 = OMcp14_19+ROcp14_48*qd(10);
    OMcp14_210 = OMcp14_29+ROcp14_58*qd(10);
    OMcp14_310 = OMcp14_39+ROcp14_68*qd(10);
    ORcp14_110 = OMcp14_29*RLcp14_310-OMcp14_39*RLcp14_210;
    ORcp14_210 = -(OMcp14_19*RLcp14_310-OMcp14_39*RLcp14_110);
    ORcp14_310 = OMcp14_19*RLcp14_210-OMcp14_29*RLcp14_110;
    OPcp14_110 = OPcp14_19+ROcp14_48*qdd(10)+qd(10)*(OMcp14_29*ROcp14_68-OMcp14_39*ROcp14_58);
    OPcp14_210 = OPcp14_29+ROcp14_58*qdd(10)-qd(10)*(OMcp14_19*ROcp14_68-OMcp14_39*ROcp14_48);
    OPcp14_310 = OPcp14_39+ROcp14_68*qdd(10)+qd(10)*(OMcp14_19*ROcp14_58-OMcp14_29*ROcp14_48);
    RLcp14_111 = ROcp14_710*s.dpt(3,8);
    RLcp14_211 = ROcp14_810*s.dpt(3,8);
    RLcp14_311 = ROcp14_910*s.dpt(3,8);
    OMcp14_111 = OMcp14_110+ROcp14_48*qd(11);
    OMcp14_211 = OMcp14_210+ROcp14_58*qd(11);
    OMcp14_311 = OMcp14_310+ROcp14_68*qd(11);
    ORcp14_111 = OMcp14_210*RLcp14_311-OMcp14_310*RLcp14_211;
    ORcp14_211 = -(OMcp14_110*RLcp14_311-OMcp14_310*RLcp14_111);
    ORcp14_311 = OMcp14_110*RLcp14_211-OMcp14_210*RLcp14_111;
    OMcp14_112 = OMcp14_111+ROcp14_111*qd(12);
    OMcp14_212 = OMcp14_211+ROcp14_211*qd(12);
    OMcp14_312 = OMcp14_311+ROcp14_311*qd(12);
    OPcp14_112 = OPcp14_110+ROcp14_111*qdd(12)+ROcp14_48*qdd(11)+qd(11)*(OMcp14_210*ROcp14_68-OMcp14_310*ROcp14_58)+qd(12)*(OMcp14_211*ROcp14_311-...
 OMcp14_311*ROcp14_211);
    OPcp14_212 = OPcp14_210+ROcp14_211*qdd(12)+ROcp14_58*qdd(11)-qd(11)*(OMcp14_110*ROcp14_68-OMcp14_310*ROcp14_48)-qd(12)*(OMcp14_111*ROcp14_311-...
 OMcp14_311*ROcp14_111);
    OPcp14_312 = OPcp14_310+ROcp14_311*qdd(12)+ROcp14_68*qdd(11)+qd(11)*(OMcp14_110*ROcp14_58-OMcp14_210*ROcp14_48)+qd(12)*(OMcp14_111*ROcp14_211-...
 OMcp14_211*ROcp14_111);
    RLcp14_113 = ROcp14_111*s.dpt(1,10)+ROcp14_712*s.dpt(3,10);
    RLcp14_213 = ROcp14_211*s.dpt(1,10)+ROcp14_812*s.dpt(3,10);
    RLcp14_313 = ROcp14_311*s.dpt(1,10)+ROcp14_912*s.dpt(3,10);
    POcp14_113 = RLcp14_110+RLcp14_111+RLcp14_113+RLcp14_17+RLcp14_18+q(1);
    POcp14_213 = RLcp14_210+RLcp14_211+RLcp14_213+RLcp14_27+RLcp14_28+q(2);
    POcp14_313 = RLcp14_310+RLcp14_311+RLcp14_313+RLcp14_37+RLcp14_38+q(3);
    ORcp14_113 = OMcp14_212*RLcp14_313-OMcp14_312*RLcp14_213;
    ORcp14_213 = -(OMcp14_112*RLcp14_313-OMcp14_312*RLcp14_113);
    ORcp14_313 = OMcp14_112*RLcp14_213-OMcp14_212*RLcp14_113;
    VIcp14_113 = ORcp14_110+ORcp14_111+ORcp14_113+ORcp14_17+ORcp14_18+qd(1);
    VIcp14_213 = ORcp14_210+ORcp14_211+ORcp14_213+ORcp14_27+ORcp14_28+qd(2);
    VIcp14_313 = ORcp14_310+ORcp14_311+ORcp14_313+ORcp14_37+ORcp14_38+qd(3);
    ACcp14_113 = qdd(1)+OMcp14_210*ORcp14_311+OMcp14_212*ORcp14_313+OMcp14_26*ORcp14_37+OMcp14_27*ORcp14_38+OMcp14_29*ORcp14_310-OMcp14_310*...
 ORcp14_211-OMcp14_312*ORcp14_213-OMcp14_36*ORcp14_27-OMcp14_37*ORcp14_28-OMcp14_39*ORcp14_210+OPcp14_210*RLcp14_311+OPcp14_212*RLcp14_313+OPcp14_26*...
 RLcp14_37+OPcp14_27*RLcp14_38+OPcp14_29*RLcp14_310-OPcp14_310*RLcp14_211-OPcp14_312*RLcp14_213-OPcp14_36*RLcp14_27-OPcp14_37*RLcp14_28-OPcp14_39*...
 RLcp14_210;
    ACcp14_213 = qdd(2)-OMcp14_110*ORcp14_311-OMcp14_112*ORcp14_313-OMcp14_16*ORcp14_37-OMcp14_17*ORcp14_38-OMcp14_19*ORcp14_310+OMcp14_310*...
 ORcp14_111+OMcp14_312*ORcp14_113+OMcp14_36*ORcp14_17+OMcp14_37*ORcp14_18+OMcp14_39*ORcp14_110-OPcp14_110*RLcp14_311-OPcp14_112*RLcp14_313-OPcp14_16*...
 RLcp14_37-OPcp14_17*RLcp14_38-OPcp14_19*RLcp14_310+OPcp14_310*RLcp14_111+OPcp14_312*RLcp14_113+OPcp14_36*RLcp14_17+OPcp14_37*RLcp14_18+OPcp14_39*...
 RLcp14_110;
    ACcp14_313 = qdd(3)+OMcp14_110*ORcp14_211+OMcp14_112*ORcp14_213+OMcp14_16*ORcp14_27+OMcp14_17*ORcp14_28+OMcp14_19*ORcp14_210-OMcp14_210*...
 ORcp14_111-OMcp14_212*ORcp14_113-OMcp14_26*ORcp14_17-OMcp14_27*ORcp14_18-OMcp14_29*ORcp14_110+OPcp14_110*RLcp14_211+OPcp14_112*RLcp14_213+OPcp14_16*...
 RLcp14_27+OPcp14_17*RLcp14_28+OPcp14_19*RLcp14_210-OPcp14_210*RLcp14_111-OPcp14_212*RLcp14_113-OPcp14_26*RLcp14_17-OPcp14_27*RLcp14_18-OPcp14_29*...
 RLcp14_110;

% = = Block_1_0_0_15_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp14_113;
    sens.P(2) = POcp14_213;
    sens.P(3) = POcp14_313;
    sens.R(1,1) = ROcp14_114;
    sens.R(1,2) = ROcp14_214;
    sens.R(1,3) = ROcp14_314;
    sens.R(2,1) = ROcp14_415;
    sens.R(2,2) = ROcp14_515;
    sens.R(2,3) = ROcp14_615;
    sens.R(3,1) = ROcp14_715;
    sens.R(3,2) = ROcp14_815;
    sens.R(3,3) = ROcp14_915;
    sens.V(1) = VIcp14_113;
    sens.V(2) = VIcp14_213;
    sens.V(3) = VIcp14_313;
    sens.OM(1) = OMcp14_112;
    sens.OM(2) = OMcp14_212;
    sens.OM(3) = OMcp14_312;
    sens.A(1) = ACcp14_113;
    sens.A(2) = ACcp14_213;
    sens.A(3) = ACcp14_313;
    sens.OMP(1) = OPcp14_112;
    sens.OMP(2) = OPcp14_212;
    sens.OMP(3) = OPcp14_312;
 
% 
case 16, 


% = = Block_1_0_0_16_0_1 = = 
 
% Sensor Kinematics 


    ROcp15_25 = S4*S5;
    ROcp15_35 = -C4*S5;
    ROcp15_85 = -S4*C5;
    ROcp15_95 = C4*C5;
    ROcp15_16 = C5*C6;
    ROcp15_26 = ROcp15_25*C6+C4*S6;
    ROcp15_36 = ROcp15_35*C6+S4*S6;
    ROcp15_46 = -C5*S6;
    ROcp15_56 = -(ROcp15_25*S6-C4*C6);
    ROcp15_66 = -(ROcp15_35*S6-S4*C6);
    OMcp15_25 = qd(5)*C4;
    OMcp15_35 = qd(5)*S4;
    OMcp15_16 = qd(4)+qd(6)*S5;
    OMcp15_26 = OMcp15_25+ROcp15_85*qd(6);
    OMcp15_36 = OMcp15_35+ROcp15_95*qd(6);
    OPcp15_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp15_26 = ROcp15_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp15_35*S5-ROcp15_95*qd(4));
    OPcp15_36 = ROcp15_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp15_25*S5-ROcp15_85*qd(4));

% = = Block_1_0_0_16_0_2 = = 
 
% Sensor Kinematics 


    ROcp15_47 = ROcp15_46*C7+S5*S7;
    ROcp15_57 = ROcp15_56*C7+ROcp15_85*S7;
    ROcp15_67 = ROcp15_66*C7+ROcp15_95*S7;
    ROcp15_77 = -(ROcp15_46*S7-S5*C7);
    ROcp15_87 = -(ROcp15_56*S7-ROcp15_85*C7);
    ROcp15_97 = -(ROcp15_66*S7-ROcp15_95*C7);
    ROcp15_18 = ROcp15_16*C8+ROcp15_47*S8;
    ROcp15_28 = ROcp15_26*C8+ROcp15_57*S8;
    ROcp15_38 = ROcp15_36*C8+ROcp15_67*S8;
    ROcp15_48 = -(ROcp15_16*S8-ROcp15_47*C8);
    ROcp15_58 = -(ROcp15_26*S8-ROcp15_57*C8);
    ROcp15_68 = -(ROcp15_36*S8-ROcp15_67*C8);
    ROcp15_19 = ROcp15_18*C9-ROcp15_77*S9;
    ROcp15_29 = ROcp15_28*C9-ROcp15_87*S9;
    ROcp15_39 = ROcp15_38*C9-ROcp15_97*S9;
    ROcp15_79 = ROcp15_18*S9+ROcp15_77*C9;
    ROcp15_89 = ROcp15_28*S9+ROcp15_87*C9;
    ROcp15_99 = ROcp15_38*S9+ROcp15_97*C9;
    ROcp15_110 = ROcp15_19*C10-ROcp15_79*S10;
    ROcp15_210 = ROcp15_29*C10-ROcp15_89*S10;
    ROcp15_310 = ROcp15_39*C10-ROcp15_99*S10;
    ROcp15_710 = ROcp15_19*S10+ROcp15_79*C10;
    ROcp15_810 = ROcp15_29*S10+ROcp15_89*C10;
    ROcp15_910 = ROcp15_39*S10+ROcp15_99*C10;
    ROcp15_111 = ROcp15_110*C11-ROcp15_710*S11;
    ROcp15_211 = ROcp15_210*C11-ROcp15_810*S11;
    ROcp15_311 = ROcp15_310*C11-ROcp15_910*S11;
    ROcp15_711 = ROcp15_110*S11+ROcp15_710*C11;
    ROcp15_811 = ROcp15_210*S11+ROcp15_810*C11;
    ROcp15_911 = ROcp15_310*S11+ROcp15_910*C11;
    ROcp15_412 = ROcp15_48*C12+ROcp15_711*S12;
    ROcp15_512 = ROcp15_58*C12+ROcp15_811*S12;
    ROcp15_612 = ROcp15_68*C12+ROcp15_911*S12;
    ROcp15_712 = -(ROcp15_48*S12-ROcp15_711*C12);
    ROcp15_812 = -(ROcp15_58*S12-ROcp15_811*C12);
    ROcp15_912 = -(ROcp15_68*S12-ROcp15_911*C12);
    ROcp15_113 = ROcp15_111*C13+ROcp15_412*S13;
    ROcp15_213 = ROcp15_211*C13+ROcp15_512*S13;
    ROcp15_313 = ROcp15_311*C13+ROcp15_612*S13;
    ROcp15_413 = -(ROcp15_111*S13-ROcp15_412*C13);
    ROcp15_513 = -(ROcp15_211*S13-ROcp15_512*C13);
    ROcp15_613 = -(ROcp15_311*S13-ROcp15_612*C13);
    ROcp15_114 = ROcp15_113*C14-ROcp15_712*S14;
    ROcp15_214 = ROcp15_213*C14-ROcp15_812*S14;
    ROcp15_314 = ROcp15_313*C14-ROcp15_912*S14;
    ROcp15_714 = ROcp15_113*S14+ROcp15_712*C14;
    ROcp15_814 = ROcp15_213*S14+ROcp15_812*C14;
    ROcp15_914 = ROcp15_313*S14+ROcp15_912*C14;
    ROcp15_415 = ROcp15_413*C15+ROcp15_714*S15;
    ROcp15_515 = ROcp15_513*C15+ROcp15_814*S15;
    ROcp15_615 = ROcp15_613*C15+ROcp15_914*S15;
    ROcp15_715 = -(ROcp15_413*S15-ROcp15_714*C15);
    ROcp15_815 = -(ROcp15_513*S15-ROcp15_814*C15);
    ROcp15_915 = -(ROcp15_613*S15-ROcp15_914*C15);
    RLcp15_17 = ROcp15_16*s.dpt(1,1)+ROcp15_46*s.dpt(2,1);
    RLcp15_27 = ROcp15_26*s.dpt(1,1)+ROcp15_56*s.dpt(2,1);
    RLcp15_37 = ROcp15_36*s.dpt(1,1)+ROcp15_66*s.dpt(2,1);
    OMcp15_17 = OMcp15_16+ROcp15_16*qd(7);
    OMcp15_27 = OMcp15_26+ROcp15_26*qd(7);
    OMcp15_37 = OMcp15_36+ROcp15_36*qd(7);
    ORcp15_17 = OMcp15_26*RLcp15_37-OMcp15_36*RLcp15_27;
    ORcp15_27 = -(OMcp15_16*RLcp15_37-OMcp15_36*RLcp15_17);
    ORcp15_37 = OMcp15_16*RLcp15_27-OMcp15_26*RLcp15_17;
    OPcp15_17 = OPcp15_16+ROcp15_16*qdd(7)+qd(7)*(OMcp15_26*ROcp15_36-OMcp15_36*ROcp15_26);
    OPcp15_27 = OPcp15_26+ROcp15_26*qdd(7)-qd(7)*(OMcp15_16*ROcp15_36-OMcp15_36*ROcp15_16);
    OPcp15_37 = OPcp15_36+ROcp15_36*qdd(7)+qd(7)*(OMcp15_16*ROcp15_26-OMcp15_26*ROcp15_16);
    RLcp15_18 = ROcp15_16*s.dpt(1,5)+ROcp15_47*s.dpt(2,5)+ROcp15_77*s.dpt(3,5);
    RLcp15_28 = ROcp15_26*s.dpt(1,5)+ROcp15_57*s.dpt(2,5)+ROcp15_87*s.dpt(3,5);
    RLcp15_38 = ROcp15_36*s.dpt(1,5)+ROcp15_67*s.dpt(2,5)+ROcp15_97*s.dpt(3,5);
    OMcp15_18 = OMcp15_17+ROcp15_77*qd(8);
    OMcp15_28 = OMcp15_27+ROcp15_87*qd(8);
    OMcp15_38 = OMcp15_37+ROcp15_97*qd(8);
    ORcp15_18 = OMcp15_27*RLcp15_38-OMcp15_37*RLcp15_28;
    ORcp15_28 = -(OMcp15_17*RLcp15_38-OMcp15_37*RLcp15_18);
    ORcp15_38 = OMcp15_17*RLcp15_28-OMcp15_27*RLcp15_18;
    OMcp15_19 = OMcp15_18+ROcp15_48*qd(9);
    OMcp15_29 = OMcp15_28+ROcp15_58*qd(9);
    OMcp15_39 = OMcp15_38+ROcp15_68*qd(9);
    OPcp15_19 = OPcp15_17+ROcp15_48*qdd(9)+ROcp15_77*qdd(8)+qd(8)*(OMcp15_27*ROcp15_97-OMcp15_37*ROcp15_87)+qd(9)*(OMcp15_28*ROcp15_68-OMcp15_38*...
 ROcp15_58);
    OPcp15_29 = OPcp15_27+ROcp15_58*qdd(9)+ROcp15_87*qdd(8)-qd(8)*(OMcp15_17*ROcp15_97-OMcp15_37*ROcp15_77)-qd(9)*(OMcp15_18*ROcp15_68-OMcp15_38*...
 ROcp15_48);
    OPcp15_39 = OPcp15_37+ROcp15_68*qdd(9)+ROcp15_97*qdd(8)+qd(8)*(OMcp15_17*ROcp15_87-OMcp15_27*ROcp15_77)+qd(9)*(OMcp15_18*ROcp15_58-OMcp15_28*...
 ROcp15_48);
    RLcp15_110 = ROcp15_79*s.dpt(3,7);
    RLcp15_210 = ROcp15_89*s.dpt(3,7);
    RLcp15_310 = ROcp15_99*s.dpt(3,7);
    OMcp15_110 = OMcp15_19+ROcp15_48*qd(10);
    OMcp15_210 = OMcp15_29+ROcp15_58*qd(10);
    OMcp15_310 = OMcp15_39+ROcp15_68*qd(10);
    ORcp15_110 = OMcp15_29*RLcp15_310-OMcp15_39*RLcp15_210;
    ORcp15_210 = -(OMcp15_19*RLcp15_310-OMcp15_39*RLcp15_110);
    ORcp15_310 = OMcp15_19*RLcp15_210-OMcp15_29*RLcp15_110;
    OPcp15_110 = OPcp15_19+ROcp15_48*qdd(10)+qd(10)*(OMcp15_29*ROcp15_68-OMcp15_39*ROcp15_58);
    OPcp15_210 = OPcp15_29+ROcp15_58*qdd(10)-qd(10)*(OMcp15_19*ROcp15_68-OMcp15_39*ROcp15_48);
    OPcp15_310 = OPcp15_39+ROcp15_68*qdd(10)+qd(10)*(OMcp15_19*ROcp15_58-OMcp15_29*ROcp15_48);
    RLcp15_111 = ROcp15_710*s.dpt(3,8);
    RLcp15_211 = ROcp15_810*s.dpt(3,8);
    RLcp15_311 = ROcp15_910*s.dpt(3,8);
    OMcp15_111 = OMcp15_110+ROcp15_48*qd(11);
    OMcp15_211 = OMcp15_210+ROcp15_58*qd(11);
    OMcp15_311 = OMcp15_310+ROcp15_68*qd(11);
    ORcp15_111 = OMcp15_210*RLcp15_311-OMcp15_310*RLcp15_211;
    ORcp15_211 = -(OMcp15_110*RLcp15_311-OMcp15_310*RLcp15_111);
    ORcp15_311 = OMcp15_110*RLcp15_211-OMcp15_210*RLcp15_111;
    OMcp15_112 = OMcp15_111+ROcp15_111*qd(12);
    OMcp15_212 = OMcp15_211+ROcp15_211*qd(12);
    OMcp15_312 = OMcp15_311+ROcp15_311*qd(12);
    OPcp15_112 = OPcp15_110+ROcp15_111*qdd(12)+ROcp15_48*qdd(11)+qd(11)*(OMcp15_210*ROcp15_68-OMcp15_310*ROcp15_58)+qd(12)*(OMcp15_211*ROcp15_311-...
 OMcp15_311*ROcp15_211);
    OPcp15_212 = OPcp15_210+ROcp15_211*qdd(12)+ROcp15_58*qdd(11)-qd(11)*(OMcp15_110*ROcp15_68-OMcp15_310*ROcp15_48)-qd(12)*(OMcp15_111*ROcp15_311-...
 OMcp15_311*ROcp15_111);
    OPcp15_312 = OPcp15_310+ROcp15_311*qdd(12)+ROcp15_68*qdd(11)+qd(11)*(OMcp15_110*ROcp15_58-OMcp15_210*ROcp15_48)+qd(12)*(OMcp15_111*ROcp15_211-...
 OMcp15_211*ROcp15_111);
    RLcp15_113 = ROcp15_111*s.dpt(1,10)+ROcp15_712*s.dpt(3,10);
    RLcp15_213 = ROcp15_211*s.dpt(1,10)+ROcp15_812*s.dpt(3,10);
    RLcp15_313 = ROcp15_311*s.dpt(1,10)+ROcp15_912*s.dpt(3,10);
    ORcp15_113 = OMcp15_212*RLcp15_313-OMcp15_312*RLcp15_213;
    ORcp15_213 = -(OMcp15_112*RLcp15_313-OMcp15_312*RLcp15_113);
    ORcp15_313 = OMcp15_112*RLcp15_213-OMcp15_212*RLcp15_113;
    RLcp15_116 = ROcp15_715*q(16);
    RLcp15_216 = ROcp15_815*q(16);
    RLcp15_316 = ROcp15_915*q(16);
    POcp15_116 = RLcp15_110+RLcp15_111+RLcp15_113+RLcp15_116+RLcp15_17+RLcp15_18+q(1);
    POcp15_216 = RLcp15_210+RLcp15_211+RLcp15_213+RLcp15_216+RLcp15_27+RLcp15_28+q(2);
    POcp15_316 = RLcp15_310+RLcp15_311+RLcp15_313+RLcp15_316+RLcp15_37+RLcp15_38+q(3);
    ORcp15_116 = OMcp15_212*RLcp15_316-OMcp15_312*RLcp15_216;
    ORcp15_216 = -(OMcp15_112*RLcp15_316-OMcp15_312*RLcp15_116);
    ORcp15_316 = OMcp15_112*RLcp15_216-OMcp15_212*RLcp15_116;
    VIcp15_116 = ORcp15_110+ORcp15_111+ORcp15_113+ORcp15_116+ORcp15_17+ORcp15_18+qd(1);
    VIcp15_216 = ORcp15_210+ORcp15_211+ORcp15_213+ORcp15_216+ORcp15_27+ORcp15_28+qd(2);
    VIcp15_316 = ORcp15_310+ORcp15_311+ORcp15_313+ORcp15_316+ORcp15_37+ORcp15_38+qd(3);
    ACcp15_116 = qdd(1)+OMcp15_210*ORcp15_311+OMcp15_212*ORcp15_313+OMcp15_212*ORcp15_316+OMcp15_26*ORcp15_37+OMcp15_27*ORcp15_38+OMcp15_29*...
 ORcp15_310-OMcp15_310*ORcp15_211-OMcp15_312*ORcp15_213-OMcp15_312*ORcp15_216-OMcp15_36*ORcp15_27-OMcp15_37*ORcp15_28-OMcp15_39*ORcp15_210+OPcp15_210*...
 RLcp15_311+OPcp15_212*RLcp15_313+OPcp15_212*RLcp15_316+OPcp15_26*RLcp15_37+OPcp15_27*RLcp15_38+OPcp15_29*RLcp15_310-OPcp15_310*RLcp15_211-OPcp15_312*...
 RLcp15_213-OPcp15_312*RLcp15_216-OPcp15_36*RLcp15_27-OPcp15_37*RLcp15_28-OPcp15_39*RLcp15_210;
    ACcp15_216 = qdd(2)-OMcp15_110*ORcp15_311-OMcp15_112*ORcp15_313-OMcp15_112*ORcp15_316-OMcp15_16*ORcp15_37-OMcp15_17*ORcp15_38-OMcp15_19*...
 ORcp15_310+OMcp15_310*ORcp15_111+OMcp15_312*ORcp15_113+OMcp15_312*ORcp15_116+OMcp15_36*ORcp15_17+OMcp15_37*ORcp15_18+OMcp15_39*ORcp15_110-OPcp15_110*...
 RLcp15_311-OPcp15_112*RLcp15_313-OPcp15_112*RLcp15_316-OPcp15_16*RLcp15_37-OPcp15_17*RLcp15_38-OPcp15_19*RLcp15_310+OPcp15_310*RLcp15_111+OPcp15_312*...
 RLcp15_113+OPcp15_312*RLcp15_116+OPcp15_36*RLcp15_17+OPcp15_37*RLcp15_18+OPcp15_39*RLcp15_110;
    ACcp15_316 = qdd(3)+OMcp15_110*ORcp15_211+OMcp15_112*ORcp15_213+OMcp15_112*ORcp15_216+OMcp15_16*ORcp15_27+OMcp15_17*ORcp15_28+OMcp15_19*...
 ORcp15_210-OMcp15_210*ORcp15_111-OMcp15_212*ORcp15_113-OMcp15_212*ORcp15_116-OMcp15_26*ORcp15_17-OMcp15_27*ORcp15_18-OMcp15_29*ORcp15_110+OPcp15_110*...
 RLcp15_211+OPcp15_112*RLcp15_213+OPcp15_112*RLcp15_216+OPcp15_16*RLcp15_27+OPcp15_17*RLcp15_28+OPcp15_19*RLcp15_210-OPcp15_210*RLcp15_111-OPcp15_212*...
 RLcp15_113-OPcp15_212*RLcp15_116-OPcp15_26*RLcp15_17-OPcp15_27*RLcp15_18-OPcp15_29*RLcp15_110;

% = = Block_1_0_0_16_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp15_116;
    sens.P(2) = POcp15_216;
    sens.P(3) = POcp15_316;
    sens.R(1,1) = ROcp15_114;
    sens.R(1,2) = ROcp15_214;
    sens.R(1,3) = ROcp15_314;
    sens.R(2,1) = ROcp15_415;
    sens.R(2,2) = ROcp15_515;
    sens.R(2,3) = ROcp15_615;
    sens.R(3,1) = ROcp15_715;
    sens.R(3,2) = ROcp15_815;
    sens.R(3,3) = ROcp15_915;
    sens.V(1) = VIcp15_116;
    sens.V(2) = VIcp15_216;
    sens.V(3) = VIcp15_316;
    sens.OM(1) = OMcp15_112;
    sens.OM(2) = OMcp15_212;
    sens.OM(3) = OMcp15_312;
    sens.A(1) = ACcp15_116;
    sens.A(2) = ACcp15_216;
    sens.A(3) = ACcp15_316;
    sens.OMP(1) = OPcp15_112;
    sens.OMP(2) = OPcp15_212;
    sens.OMP(3) = OPcp15_312;
 
% 
case 17, 


% = = Block_1_0_0_17_0_1 = = 
 
% Sensor Kinematics 


    ROcp16_25 = S4*S5;
    ROcp16_35 = -C4*S5;
    ROcp16_85 = -S4*C5;
    ROcp16_95 = C4*C5;
    ROcp16_16 = C5*C6;
    ROcp16_26 = ROcp16_25*C6+C4*S6;
    ROcp16_36 = ROcp16_35*C6+S4*S6;
    ROcp16_46 = -C5*S6;
    ROcp16_56 = -(ROcp16_25*S6-C4*C6);
    ROcp16_66 = -(ROcp16_35*S6-S4*C6);
    OMcp16_25 = qd(5)*C4;
    OMcp16_35 = qd(5)*S4;
    OMcp16_16 = qd(4)+qd(6)*S5;
    OMcp16_26 = OMcp16_25+ROcp16_85*qd(6);
    OMcp16_36 = OMcp16_35+ROcp16_95*qd(6);
    OPcp16_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp16_26 = ROcp16_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp16_35*S5-ROcp16_95*qd(4));
    OPcp16_36 = ROcp16_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp16_25*S5-ROcp16_85*qd(4));

% = = Block_1_0_0_17_0_2 = = 
 
% Sensor Kinematics 


    ROcp16_47 = ROcp16_46*C7+S5*S7;
    ROcp16_57 = ROcp16_56*C7+ROcp16_85*S7;
    ROcp16_67 = ROcp16_66*C7+ROcp16_95*S7;
    ROcp16_77 = -(ROcp16_46*S7-S5*C7);
    ROcp16_87 = -(ROcp16_56*S7-ROcp16_85*C7);
    ROcp16_97 = -(ROcp16_66*S7-ROcp16_95*C7);
    ROcp16_18 = ROcp16_16*C8+ROcp16_47*S8;
    ROcp16_28 = ROcp16_26*C8+ROcp16_57*S8;
    ROcp16_38 = ROcp16_36*C8+ROcp16_67*S8;
    ROcp16_48 = -(ROcp16_16*S8-ROcp16_47*C8);
    ROcp16_58 = -(ROcp16_26*S8-ROcp16_57*C8);
    ROcp16_68 = -(ROcp16_36*S8-ROcp16_67*C8);
    ROcp16_19 = ROcp16_18*C9-ROcp16_77*S9;
    ROcp16_29 = ROcp16_28*C9-ROcp16_87*S9;
    ROcp16_39 = ROcp16_38*C9-ROcp16_97*S9;
    ROcp16_79 = ROcp16_18*S9+ROcp16_77*C9;
    ROcp16_89 = ROcp16_28*S9+ROcp16_87*C9;
    ROcp16_99 = ROcp16_38*S9+ROcp16_97*C9;
    ROcp16_110 = ROcp16_19*C10-ROcp16_79*S10;
    ROcp16_210 = ROcp16_29*C10-ROcp16_89*S10;
    ROcp16_310 = ROcp16_39*C10-ROcp16_99*S10;
    ROcp16_710 = ROcp16_19*S10+ROcp16_79*C10;
    ROcp16_810 = ROcp16_29*S10+ROcp16_89*C10;
    ROcp16_910 = ROcp16_39*S10+ROcp16_99*C10;
    ROcp16_111 = ROcp16_110*C11-ROcp16_710*S11;
    ROcp16_211 = ROcp16_210*C11-ROcp16_810*S11;
    ROcp16_311 = ROcp16_310*C11-ROcp16_910*S11;
    ROcp16_711 = ROcp16_110*S11+ROcp16_710*C11;
    ROcp16_811 = ROcp16_210*S11+ROcp16_810*C11;
    ROcp16_911 = ROcp16_310*S11+ROcp16_910*C11;
    ROcp16_412 = ROcp16_48*C12+ROcp16_711*S12;
    ROcp16_512 = ROcp16_58*C12+ROcp16_811*S12;
    ROcp16_612 = ROcp16_68*C12+ROcp16_911*S12;
    ROcp16_712 = -(ROcp16_48*S12-ROcp16_711*C12);
    ROcp16_812 = -(ROcp16_58*S12-ROcp16_811*C12);
    ROcp16_912 = -(ROcp16_68*S12-ROcp16_911*C12);
    ROcp16_113 = ROcp16_111*C13+ROcp16_412*S13;
    ROcp16_213 = ROcp16_211*C13+ROcp16_512*S13;
    ROcp16_313 = ROcp16_311*C13+ROcp16_612*S13;
    ROcp16_413 = -(ROcp16_111*S13-ROcp16_412*C13);
    ROcp16_513 = -(ROcp16_211*S13-ROcp16_512*C13);
    ROcp16_613 = -(ROcp16_311*S13-ROcp16_612*C13);
    ROcp16_114 = ROcp16_113*C14-ROcp16_712*S14;
    ROcp16_214 = ROcp16_213*C14-ROcp16_812*S14;
    ROcp16_314 = ROcp16_313*C14-ROcp16_912*S14;
    ROcp16_714 = ROcp16_113*S14+ROcp16_712*C14;
    ROcp16_814 = ROcp16_213*S14+ROcp16_812*C14;
    ROcp16_914 = ROcp16_313*S14+ROcp16_912*C14;
    ROcp16_415 = ROcp16_413*C15+ROcp16_714*S15;
    ROcp16_515 = ROcp16_513*C15+ROcp16_814*S15;
    ROcp16_615 = ROcp16_613*C15+ROcp16_914*S15;
    ROcp16_715 = -(ROcp16_413*S15-ROcp16_714*C15);
    ROcp16_815 = -(ROcp16_513*S15-ROcp16_814*C15);
    ROcp16_915 = -(ROcp16_613*S15-ROcp16_914*C15);
    RLcp16_17 = ROcp16_16*s.dpt(1,1)+ROcp16_46*s.dpt(2,1);
    RLcp16_27 = ROcp16_26*s.dpt(1,1)+ROcp16_56*s.dpt(2,1);
    RLcp16_37 = ROcp16_36*s.dpt(1,1)+ROcp16_66*s.dpt(2,1);
    OMcp16_17 = OMcp16_16+ROcp16_16*qd(7);
    OMcp16_27 = OMcp16_26+ROcp16_26*qd(7);
    OMcp16_37 = OMcp16_36+ROcp16_36*qd(7);
    ORcp16_17 = OMcp16_26*RLcp16_37-OMcp16_36*RLcp16_27;
    ORcp16_27 = -(OMcp16_16*RLcp16_37-OMcp16_36*RLcp16_17);
    ORcp16_37 = OMcp16_16*RLcp16_27-OMcp16_26*RLcp16_17;
    OPcp16_17 = OPcp16_16+ROcp16_16*qdd(7)+qd(7)*(OMcp16_26*ROcp16_36-OMcp16_36*ROcp16_26);
    OPcp16_27 = OPcp16_26+ROcp16_26*qdd(7)-qd(7)*(OMcp16_16*ROcp16_36-OMcp16_36*ROcp16_16);
    OPcp16_37 = OPcp16_36+ROcp16_36*qdd(7)+qd(7)*(OMcp16_16*ROcp16_26-OMcp16_26*ROcp16_16);
    RLcp16_18 = ROcp16_16*s.dpt(1,5)+ROcp16_47*s.dpt(2,5)+ROcp16_77*s.dpt(3,5);
    RLcp16_28 = ROcp16_26*s.dpt(1,5)+ROcp16_57*s.dpt(2,5)+ROcp16_87*s.dpt(3,5);
    RLcp16_38 = ROcp16_36*s.dpt(1,5)+ROcp16_67*s.dpt(2,5)+ROcp16_97*s.dpt(3,5);
    OMcp16_18 = OMcp16_17+ROcp16_77*qd(8);
    OMcp16_28 = OMcp16_27+ROcp16_87*qd(8);
    OMcp16_38 = OMcp16_37+ROcp16_97*qd(8);
    ORcp16_18 = OMcp16_27*RLcp16_38-OMcp16_37*RLcp16_28;
    ORcp16_28 = -(OMcp16_17*RLcp16_38-OMcp16_37*RLcp16_18);
    ORcp16_38 = OMcp16_17*RLcp16_28-OMcp16_27*RLcp16_18;
    OMcp16_19 = OMcp16_18+ROcp16_48*qd(9);
    OMcp16_29 = OMcp16_28+ROcp16_58*qd(9);
    OMcp16_39 = OMcp16_38+ROcp16_68*qd(9);
    OPcp16_19 = OPcp16_17+ROcp16_48*qdd(9)+ROcp16_77*qdd(8)+qd(8)*(OMcp16_27*ROcp16_97-OMcp16_37*ROcp16_87)+qd(9)*(OMcp16_28*ROcp16_68-OMcp16_38*...
 ROcp16_58);
    OPcp16_29 = OPcp16_27+ROcp16_58*qdd(9)+ROcp16_87*qdd(8)-qd(8)*(OMcp16_17*ROcp16_97-OMcp16_37*ROcp16_77)-qd(9)*(OMcp16_18*ROcp16_68-OMcp16_38*...
 ROcp16_48);
    OPcp16_39 = OPcp16_37+ROcp16_68*qdd(9)+ROcp16_97*qdd(8)+qd(8)*(OMcp16_17*ROcp16_87-OMcp16_27*ROcp16_77)+qd(9)*(OMcp16_18*ROcp16_58-OMcp16_28*...
 ROcp16_48);
    RLcp16_110 = ROcp16_79*s.dpt(3,7);
    RLcp16_210 = ROcp16_89*s.dpt(3,7);
    RLcp16_310 = ROcp16_99*s.dpt(3,7);
    OMcp16_110 = OMcp16_19+ROcp16_48*qd(10);
    OMcp16_210 = OMcp16_29+ROcp16_58*qd(10);
    OMcp16_310 = OMcp16_39+ROcp16_68*qd(10);
    ORcp16_110 = OMcp16_29*RLcp16_310-OMcp16_39*RLcp16_210;
    ORcp16_210 = -(OMcp16_19*RLcp16_310-OMcp16_39*RLcp16_110);
    ORcp16_310 = OMcp16_19*RLcp16_210-OMcp16_29*RLcp16_110;
    OPcp16_110 = OPcp16_19+ROcp16_48*qdd(10)+qd(10)*(OMcp16_29*ROcp16_68-OMcp16_39*ROcp16_58);
    OPcp16_210 = OPcp16_29+ROcp16_58*qdd(10)-qd(10)*(OMcp16_19*ROcp16_68-OMcp16_39*ROcp16_48);
    OPcp16_310 = OPcp16_39+ROcp16_68*qdd(10)+qd(10)*(OMcp16_19*ROcp16_58-OMcp16_29*ROcp16_48);
    RLcp16_111 = ROcp16_710*s.dpt(3,8);
    RLcp16_211 = ROcp16_810*s.dpt(3,8);
    RLcp16_311 = ROcp16_910*s.dpt(3,8);
    OMcp16_111 = OMcp16_110+ROcp16_48*qd(11);
    OMcp16_211 = OMcp16_210+ROcp16_58*qd(11);
    OMcp16_311 = OMcp16_310+ROcp16_68*qd(11);
    ORcp16_111 = OMcp16_210*RLcp16_311-OMcp16_310*RLcp16_211;
    ORcp16_211 = -(OMcp16_110*RLcp16_311-OMcp16_310*RLcp16_111);
    ORcp16_311 = OMcp16_110*RLcp16_211-OMcp16_210*RLcp16_111;
    OMcp16_112 = OMcp16_111+ROcp16_111*qd(12);
    OMcp16_212 = OMcp16_211+ROcp16_211*qd(12);
    OMcp16_312 = OMcp16_311+ROcp16_311*qd(12);
    OPcp16_112 = OPcp16_110+ROcp16_111*qdd(12)+ROcp16_48*qdd(11)+qd(11)*(OMcp16_210*ROcp16_68-OMcp16_310*ROcp16_58)+qd(12)*(OMcp16_211*ROcp16_311-...
 OMcp16_311*ROcp16_211);
    OPcp16_212 = OPcp16_210+ROcp16_211*qdd(12)+ROcp16_58*qdd(11)-qd(11)*(OMcp16_110*ROcp16_68-OMcp16_310*ROcp16_48)-qd(12)*(OMcp16_111*ROcp16_311-...
 OMcp16_311*ROcp16_111);
    OPcp16_312 = OPcp16_310+ROcp16_311*qdd(12)+ROcp16_68*qdd(11)+qd(11)*(OMcp16_110*ROcp16_58-OMcp16_210*ROcp16_48)+qd(12)*(OMcp16_111*ROcp16_211-...
 OMcp16_211*ROcp16_111);
    RLcp16_113 = ROcp16_111*s.dpt(1,10)+ROcp16_712*s.dpt(3,10);
    RLcp16_213 = ROcp16_211*s.dpt(1,10)+ROcp16_812*s.dpt(3,10);
    RLcp16_313 = ROcp16_311*s.dpt(1,10)+ROcp16_912*s.dpt(3,10);
    ORcp16_113 = OMcp16_212*RLcp16_313-OMcp16_312*RLcp16_213;
    ORcp16_213 = -(OMcp16_112*RLcp16_313-OMcp16_312*RLcp16_113);
    ORcp16_313 = OMcp16_112*RLcp16_213-OMcp16_212*RLcp16_113;
    RLcp16_116 = ROcp16_715*q(16);
    RLcp16_216 = ROcp16_815*q(16);
    RLcp16_316 = ROcp16_915*q(16);
    ORcp16_116 = OMcp16_212*RLcp16_316-OMcp16_312*RLcp16_216;
    ORcp16_216 = -(OMcp16_112*RLcp16_316-OMcp16_312*RLcp16_116);
    ORcp16_316 = OMcp16_112*RLcp16_216-OMcp16_212*RLcp16_116;
    RLcp16_117 = ROcp16_415*q(17);
    RLcp16_217 = ROcp16_515*q(17);
    RLcp16_317 = ROcp16_615*q(17);
    POcp16_117 = RLcp16_110+RLcp16_111+RLcp16_113+RLcp16_116+RLcp16_117+RLcp16_17+RLcp16_18+q(1);
    POcp16_217 = RLcp16_210+RLcp16_211+RLcp16_213+RLcp16_216+RLcp16_217+RLcp16_27+RLcp16_28+q(2);
    POcp16_317 = RLcp16_310+RLcp16_311+RLcp16_313+RLcp16_316+RLcp16_317+RLcp16_37+RLcp16_38+q(3);
    ORcp16_117 = OMcp16_212*RLcp16_317-OMcp16_312*RLcp16_217;
    ORcp16_217 = -(OMcp16_112*RLcp16_317-OMcp16_312*RLcp16_117);
    ORcp16_317 = OMcp16_112*RLcp16_217-OMcp16_212*RLcp16_117;
    VIcp16_117 = ORcp16_110+ORcp16_111+ORcp16_113+ORcp16_116+ORcp16_117+ORcp16_17+ORcp16_18+qd(1);
    VIcp16_217 = ORcp16_210+ORcp16_211+ORcp16_213+ORcp16_216+ORcp16_217+ORcp16_27+ORcp16_28+qd(2);
    VIcp16_317 = ORcp16_310+ORcp16_311+ORcp16_313+ORcp16_316+ORcp16_317+ORcp16_37+ORcp16_38+qd(3);
    ACcp16_117 = qdd(1)+OMcp16_210*ORcp16_311+OMcp16_212*ORcp16_313+OMcp16_212*ORcp16_316+OMcp16_212*ORcp16_317+OMcp16_26*ORcp16_37+OMcp16_27*...
 ORcp16_38+OMcp16_29*ORcp16_310-OMcp16_310*ORcp16_211-OMcp16_312*ORcp16_213-OMcp16_312*ORcp16_216-OMcp16_312*ORcp16_217-OMcp16_36*ORcp16_27-OMcp16_37*...
 ORcp16_28-OMcp16_39*ORcp16_210+OPcp16_210*RLcp16_311+OPcp16_212*RLcp16_313+OPcp16_212*RLcp16_316+OPcp16_212*RLcp16_317+OPcp16_26*RLcp16_37+OPcp16_27*...
 RLcp16_38+OPcp16_29*RLcp16_310-OPcp16_310*RLcp16_211-OPcp16_312*RLcp16_213-OPcp16_312*RLcp16_216-OPcp16_312*RLcp16_217-OPcp16_36*RLcp16_27-OPcp16_37*...
 RLcp16_28-OPcp16_39*RLcp16_210;
    ACcp16_217 = qdd(2)-OMcp16_110*ORcp16_311-OMcp16_112*ORcp16_313-OMcp16_112*ORcp16_316-OMcp16_112*ORcp16_317-OMcp16_16*ORcp16_37-OMcp16_17*...
 ORcp16_38-OMcp16_19*ORcp16_310+OMcp16_310*ORcp16_111+OMcp16_312*ORcp16_113+OMcp16_312*ORcp16_116+OMcp16_312*ORcp16_117+OMcp16_36*ORcp16_17+OMcp16_37*...
 ORcp16_18+OMcp16_39*ORcp16_110-OPcp16_110*RLcp16_311-OPcp16_112*RLcp16_313-OPcp16_112*RLcp16_316-OPcp16_112*RLcp16_317-OPcp16_16*RLcp16_37-OPcp16_17*...
 RLcp16_38-OPcp16_19*RLcp16_310+OPcp16_310*RLcp16_111+OPcp16_312*RLcp16_113+OPcp16_312*RLcp16_116+OPcp16_312*RLcp16_117+OPcp16_36*RLcp16_17+OPcp16_37*...
 RLcp16_18+OPcp16_39*RLcp16_110;
    ACcp16_317 = qdd(3)+OMcp16_110*ORcp16_211+OMcp16_112*ORcp16_213+OMcp16_112*ORcp16_216+OMcp16_112*ORcp16_217+OMcp16_16*ORcp16_27+OMcp16_17*...
 ORcp16_28+OMcp16_19*ORcp16_210-OMcp16_210*ORcp16_111-OMcp16_212*ORcp16_113-OMcp16_212*ORcp16_116-OMcp16_212*ORcp16_117-OMcp16_26*ORcp16_17-OMcp16_27*...
 ORcp16_18-OMcp16_29*ORcp16_110+OPcp16_110*RLcp16_211+OPcp16_112*RLcp16_213+OPcp16_112*RLcp16_216+OPcp16_112*RLcp16_217+OPcp16_16*RLcp16_27+OPcp16_17*...
 RLcp16_28+OPcp16_19*RLcp16_210-OPcp16_210*RLcp16_111-OPcp16_212*RLcp16_113-OPcp16_212*RLcp16_116-OPcp16_212*RLcp16_117-OPcp16_26*RLcp16_17-OPcp16_27*...
 RLcp16_18-OPcp16_29*RLcp16_110;

% = = Block_1_0_0_17_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp16_117;
    sens.P(2) = POcp16_217;
    sens.P(3) = POcp16_317;
    sens.R(1,1) = ROcp16_114;
    sens.R(1,2) = ROcp16_214;
    sens.R(1,3) = ROcp16_314;
    sens.R(2,1) = ROcp16_415;
    sens.R(2,2) = ROcp16_515;
    sens.R(2,3) = ROcp16_615;
    sens.R(3,1) = ROcp16_715;
    sens.R(3,2) = ROcp16_815;
    sens.R(3,3) = ROcp16_915;
    sens.V(1) = VIcp16_117;
    sens.V(2) = VIcp16_217;
    sens.V(3) = VIcp16_317;
    sens.OM(1) = OMcp16_112;
    sens.OM(2) = OMcp16_212;
    sens.OM(3) = OMcp16_312;
    sens.A(1) = ACcp16_117;
    sens.A(2) = ACcp16_217;
    sens.A(3) = ACcp16_317;
    sens.OMP(1) = OPcp16_112;
    sens.OMP(2) = OPcp16_212;
    sens.OMP(3) = OPcp16_312;
 
% 
case 18, 


% = = Block_1_0_0_18_0_1 = = 
 
% Sensor Kinematics 


    ROcp17_25 = S4*S5;
    ROcp17_35 = -C4*S5;
    ROcp17_85 = -S4*C5;
    ROcp17_95 = C4*C5;
    ROcp17_16 = C5*C6;
    ROcp17_26 = ROcp17_25*C6+C4*S6;
    ROcp17_36 = ROcp17_35*C6+S4*S6;
    ROcp17_46 = -C5*S6;
    ROcp17_56 = -(ROcp17_25*S6-C4*C6);
    ROcp17_66 = -(ROcp17_35*S6-S4*C6);
    OMcp17_25 = qd(5)*C4;
    OMcp17_35 = qd(5)*S4;
    OMcp17_16 = qd(4)+qd(6)*S5;
    OMcp17_26 = OMcp17_25+ROcp17_85*qd(6);
    OMcp17_36 = OMcp17_35+ROcp17_95*qd(6);
    OPcp17_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp17_26 = ROcp17_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp17_35*S5-ROcp17_95*qd(4));
    OPcp17_36 = ROcp17_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp17_25*S5-ROcp17_85*qd(4));

% = = Block_1_0_0_18_0_2 = = 
 
% Sensor Kinematics 


    ROcp17_47 = ROcp17_46*C7+S5*S7;
    ROcp17_57 = ROcp17_56*C7+ROcp17_85*S7;
    ROcp17_67 = ROcp17_66*C7+ROcp17_95*S7;
    ROcp17_77 = -(ROcp17_46*S7-S5*C7);
    ROcp17_87 = -(ROcp17_56*S7-ROcp17_85*C7);
    ROcp17_97 = -(ROcp17_66*S7-ROcp17_95*C7);
    ROcp17_18 = ROcp17_16*C8+ROcp17_47*S8;
    ROcp17_28 = ROcp17_26*C8+ROcp17_57*S8;
    ROcp17_38 = ROcp17_36*C8+ROcp17_67*S8;
    ROcp17_48 = -(ROcp17_16*S8-ROcp17_47*C8);
    ROcp17_58 = -(ROcp17_26*S8-ROcp17_57*C8);
    ROcp17_68 = -(ROcp17_36*S8-ROcp17_67*C8);
    ROcp17_19 = ROcp17_18*C9-ROcp17_77*S9;
    ROcp17_29 = ROcp17_28*C9-ROcp17_87*S9;
    ROcp17_39 = ROcp17_38*C9-ROcp17_97*S9;
    ROcp17_79 = ROcp17_18*S9+ROcp17_77*C9;
    ROcp17_89 = ROcp17_28*S9+ROcp17_87*C9;
    ROcp17_99 = ROcp17_38*S9+ROcp17_97*C9;
    ROcp17_110 = ROcp17_19*C10-ROcp17_79*S10;
    ROcp17_210 = ROcp17_29*C10-ROcp17_89*S10;
    ROcp17_310 = ROcp17_39*C10-ROcp17_99*S10;
    ROcp17_710 = ROcp17_19*S10+ROcp17_79*C10;
    ROcp17_810 = ROcp17_29*S10+ROcp17_89*C10;
    ROcp17_910 = ROcp17_39*S10+ROcp17_99*C10;
    ROcp17_111 = ROcp17_110*C11-ROcp17_710*S11;
    ROcp17_211 = ROcp17_210*C11-ROcp17_810*S11;
    ROcp17_311 = ROcp17_310*C11-ROcp17_910*S11;
    ROcp17_711 = ROcp17_110*S11+ROcp17_710*C11;
    ROcp17_811 = ROcp17_210*S11+ROcp17_810*C11;
    ROcp17_911 = ROcp17_310*S11+ROcp17_910*C11;
    ROcp17_412 = ROcp17_48*C12+ROcp17_711*S12;
    ROcp17_512 = ROcp17_58*C12+ROcp17_811*S12;
    ROcp17_612 = ROcp17_68*C12+ROcp17_911*S12;
    ROcp17_712 = -(ROcp17_48*S12-ROcp17_711*C12);
    ROcp17_812 = -(ROcp17_58*S12-ROcp17_811*C12);
    ROcp17_912 = -(ROcp17_68*S12-ROcp17_911*C12);
    ROcp17_113 = ROcp17_111*C13+ROcp17_412*S13;
    ROcp17_213 = ROcp17_211*C13+ROcp17_512*S13;
    ROcp17_313 = ROcp17_311*C13+ROcp17_612*S13;
    ROcp17_413 = -(ROcp17_111*S13-ROcp17_412*C13);
    ROcp17_513 = -(ROcp17_211*S13-ROcp17_512*C13);
    ROcp17_613 = -(ROcp17_311*S13-ROcp17_612*C13);
    ROcp17_114 = ROcp17_113*C14-ROcp17_712*S14;
    ROcp17_214 = ROcp17_213*C14-ROcp17_812*S14;
    ROcp17_314 = ROcp17_313*C14-ROcp17_912*S14;
    ROcp17_714 = ROcp17_113*S14+ROcp17_712*C14;
    ROcp17_814 = ROcp17_213*S14+ROcp17_812*C14;
    ROcp17_914 = ROcp17_313*S14+ROcp17_912*C14;
    ROcp17_415 = ROcp17_413*C15+ROcp17_714*S15;
    ROcp17_515 = ROcp17_513*C15+ROcp17_814*S15;
    ROcp17_615 = ROcp17_613*C15+ROcp17_914*S15;
    ROcp17_715 = -(ROcp17_413*S15-ROcp17_714*C15);
    ROcp17_815 = -(ROcp17_513*S15-ROcp17_814*C15);
    ROcp17_915 = -(ROcp17_613*S15-ROcp17_914*C15);
    RLcp17_17 = ROcp17_16*s.dpt(1,1)+ROcp17_46*s.dpt(2,1);
    RLcp17_27 = ROcp17_26*s.dpt(1,1)+ROcp17_56*s.dpt(2,1);
    RLcp17_37 = ROcp17_36*s.dpt(1,1)+ROcp17_66*s.dpt(2,1);
    OMcp17_17 = OMcp17_16+ROcp17_16*qd(7);
    OMcp17_27 = OMcp17_26+ROcp17_26*qd(7);
    OMcp17_37 = OMcp17_36+ROcp17_36*qd(7);
    ORcp17_17 = OMcp17_26*RLcp17_37-OMcp17_36*RLcp17_27;
    ORcp17_27 = -(OMcp17_16*RLcp17_37-OMcp17_36*RLcp17_17);
    ORcp17_37 = OMcp17_16*RLcp17_27-OMcp17_26*RLcp17_17;
    OPcp17_17 = OPcp17_16+ROcp17_16*qdd(7)+qd(7)*(OMcp17_26*ROcp17_36-OMcp17_36*ROcp17_26);
    OPcp17_27 = OPcp17_26+ROcp17_26*qdd(7)-qd(7)*(OMcp17_16*ROcp17_36-OMcp17_36*ROcp17_16);
    OPcp17_37 = OPcp17_36+ROcp17_36*qdd(7)+qd(7)*(OMcp17_16*ROcp17_26-OMcp17_26*ROcp17_16);
    RLcp17_18 = ROcp17_16*s.dpt(1,5)+ROcp17_47*s.dpt(2,5)+ROcp17_77*s.dpt(3,5);
    RLcp17_28 = ROcp17_26*s.dpt(1,5)+ROcp17_57*s.dpt(2,5)+ROcp17_87*s.dpt(3,5);
    RLcp17_38 = ROcp17_36*s.dpt(1,5)+ROcp17_67*s.dpt(2,5)+ROcp17_97*s.dpt(3,5);
    OMcp17_18 = OMcp17_17+ROcp17_77*qd(8);
    OMcp17_28 = OMcp17_27+ROcp17_87*qd(8);
    OMcp17_38 = OMcp17_37+ROcp17_97*qd(8);
    ORcp17_18 = OMcp17_27*RLcp17_38-OMcp17_37*RLcp17_28;
    ORcp17_28 = -(OMcp17_17*RLcp17_38-OMcp17_37*RLcp17_18);
    ORcp17_38 = OMcp17_17*RLcp17_28-OMcp17_27*RLcp17_18;
    OMcp17_19 = OMcp17_18+ROcp17_48*qd(9);
    OMcp17_29 = OMcp17_28+ROcp17_58*qd(9);
    OMcp17_39 = OMcp17_38+ROcp17_68*qd(9);
    OPcp17_19 = OPcp17_17+ROcp17_48*qdd(9)+ROcp17_77*qdd(8)+qd(8)*(OMcp17_27*ROcp17_97-OMcp17_37*ROcp17_87)+qd(9)*(OMcp17_28*ROcp17_68-OMcp17_38*...
 ROcp17_58);
    OPcp17_29 = OPcp17_27+ROcp17_58*qdd(9)+ROcp17_87*qdd(8)-qd(8)*(OMcp17_17*ROcp17_97-OMcp17_37*ROcp17_77)-qd(9)*(OMcp17_18*ROcp17_68-OMcp17_38*...
 ROcp17_48);
    OPcp17_39 = OPcp17_37+ROcp17_68*qdd(9)+ROcp17_97*qdd(8)+qd(8)*(OMcp17_17*ROcp17_87-OMcp17_27*ROcp17_77)+qd(9)*(OMcp17_18*ROcp17_58-OMcp17_28*...
 ROcp17_48);
    RLcp17_110 = ROcp17_79*s.dpt(3,7);
    RLcp17_210 = ROcp17_89*s.dpt(3,7);
    RLcp17_310 = ROcp17_99*s.dpt(3,7);
    OMcp17_110 = OMcp17_19+ROcp17_48*qd(10);
    OMcp17_210 = OMcp17_29+ROcp17_58*qd(10);
    OMcp17_310 = OMcp17_39+ROcp17_68*qd(10);
    ORcp17_110 = OMcp17_29*RLcp17_310-OMcp17_39*RLcp17_210;
    ORcp17_210 = -(OMcp17_19*RLcp17_310-OMcp17_39*RLcp17_110);
    ORcp17_310 = OMcp17_19*RLcp17_210-OMcp17_29*RLcp17_110;
    OPcp17_110 = OPcp17_19+ROcp17_48*qdd(10)+qd(10)*(OMcp17_29*ROcp17_68-OMcp17_39*ROcp17_58);
    OPcp17_210 = OPcp17_29+ROcp17_58*qdd(10)-qd(10)*(OMcp17_19*ROcp17_68-OMcp17_39*ROcp17_48);
    OPcp17_310 = OPcp17_39+ROcp17_68*qdd(10)+qd(10)*(OMcp17_19*ROcp17_58-OMcp17_29*ROcp17_48);
    RLcp17_111 = ROcp17_710*s.dpt(3,8);
    RLcp17_211 = ROcp17_810*s.dpt(3,8);
    RLcp17_311 = ROcp17_910*s.dpt(3,8);
    OMcp17_111 = OMcp17_110+ROcp17_48*qd(11);
    OMcp17_211 = OMcp17_210+ROcp17_58*qd(11);
    OMcp17_311 = OMcp17_310+ROcp17_68*qd(11);
    ORcp17_111 = OMcp17_210*RLcp17_311-OMcp17_310*RLcp17_211;
    ORcp17_211 = -(OMcp17_110*RLcp17_311-OMcp17_310*RLcp17_111);
    ORcp17_311 = OMcp17_110*RLcp17_211-OMcp17_210*RLcp17_111;
    OMcp17_112 = OMcp17_111+ROcp17_111*qd(12);
    OMcp17_212 = OMcp17_211+ROcp17_211*qd(12);
    OMcp17_312 = OMcp17_311+ROcp17_311*qd(12);
    OPcp17_112 = OPcp17_110+ROcp17_111*qdd(12)+ROcp17_48*qdd(11)+qd(11)*(OMcp17_210*ROcp17_68-OMcp17_310*ROcp17_58)+qd(12)*(OMcp17_211*ROcp17_311-...
 OMcp17_311*ROcp17_211);
    OPcp17_212 = OPcp17_210+ROcp17_211*qdd(12)+ROcp17_58*qdd(11)-qd(11)*(OMcp17_110*ROcp17_68-OMcp17_310*ROcp17_48)-qd(12)*(OMcp17_111*ROcp17_311-...
 OMcp17_311*ROcp17_111);
    OPcp17_312 = OPcp17_310+ROcp17_311*qdd(12)+ROcp17_68*qdd(11)+qd(11)*(OMcp17_110*ROcp17_58-OMcp17_210*ROcp17_48)+qd(12)*(OMcp17_111*ROcp17_211-...
 OMcp17_211*ROcp17_111);
    RLcp17_113 = ROcp17_111*s.dpt(1,10)+ROcp17_712*s.dpt(3,10);
    RLcp17_213 = ROcp17_211*s.dpt(1,10)+ROcp17_812*s.dpt(3,10);
    RLcp17_313 = ROcp17_311*s.dpt(1,10)+ROcp17_912*s.dpt(3,10);
    ORcp17_113 = OMcp17_212*RLcp17_313-OMcp17_312*RLcp17_213;
    ORcp17_213 = -(OMcp17_112*RLcp17_313-OMcp17_312*RLcp17_113);
    ORcp17_313 = OMcp17_112*RLcp17_213-OMcp17_212*RLcp17_113;
    RLcp17_116 = ROcp17_715*q(16);
    RLcp17_216 = ROcp17_815*q(16);
    RLcp17_316 = ROcp17_915*q(16);
    ORcp17_116 = OMcp17_212*RLcp17_316-OMcp17_312*RLcp17_216;
    ORcp17_216 = -(OMcp17_112*RLcp17_316-OMcp17_312*RLcp17_116);
    ORcp17_316 = OMcp17_112*RLcp17_216-OMcp17_212*RLcp17_116;
    RLcp17_117 = ROcp17_415*q(17);
    RLcp17_217 = ROcp17_515*q(17);
    RLcp17_317 = ROcp17_615*q(17);
    ORcp17_117 = OMcp17_212*RLcp17_317-OMcp17_312*RLcp17_217;
    ORcp17_217 = -(OMcp17_112*RLcp17_317-OMcp17_312*RLcp17_117);
    ORcp17_317 = OMcp17_112*RLcp17_217-OMcp17_212*RLcp17_117;
    RLcp17_118 = ROcp17_114*q(18);
    RLcp17_218 = ROcp17_214*q(18);
    RLcp17_318 = ROcp17_314*q(18);
    POcp17_118 = RLcp17_110+RLcp17_111+RLcp17_113+RLcp17_116+RLcp17_117+RLcp17_118+RLcp17_17+RLcp17_18+q(1);
    POcp17_218 = RLcp17_210+RLcp17_211+RLcp17_213+RLcp17_216+RLcp17_217+RLcp17_218+RLcp17_27+RLcp17_28+q(2);
    POcp17_318 = RLcp17_310+RLcp17_311+RLcp17_313+RLcp17_316+RLcp17_317+RLcp17_318+RLcp17_37+RLcp17_38+q(3);
    ORcp17_118 = OMcp17_212*RLcp17_318-OMcp17_312*RLcp17_218;
    ORcp17_218 = -(OMcp17_112*RLcp17_318-OMcp17_312*RLcp17_118);
    ORcp17_318 = OMcp17_112*RLcp17_218-OMcp17_212*RLcp17_118;
    VIcp17_118 = ORcp17_110+ORcp17_111+ORcp17_113+ORcp17_116+ORcp17_117+ORcp17_118+ORcp17_17+ORcp17_18+qd(1);
    VIcp17_218 = ORcp17_210+ORcp17_211+ORcp17_213+ORcp17_216+ORcp17_217+ORcp17_218+ORcp17_27+ORcp17_28+qd(2);
    VIcp17_318 = ORcp17_310+ORcp17_311+ORcp17_313+ORcp17_316+ORcp17_317+ORcp17_318+ORcp17_37+ORcp17_38+qd(3);
    ACcp17_118 = qdd(1)+OMcp17_210*ORcp17_311+OMcp17_212*ORcp17_313+OMcp17_212*ORcp17_316+OMcp17_212*ORcp17_317+OMcp17_212*ORcp17_318+OMcp17_26*...
 ORcp17_37+OMcp17_27*ORcp17_38+OMcp17_29*ORcp17_310-OMcp17_310*ORcp17_211-OMcp17_312*ORcp17_213-OMcp17_312*ORcp17_216-OMcp17_312*ORcp17_217-OMcp17_312...
 *ORcp17_218-OMcp17_36*ORcp17_27-OMcp17_37*ORcp17_28-OMcp17_39*ORcp17_210+OPcp17_210*RLcp17_311+OPcp17_212*RLcp17_313+OPcp17_212*RLcp17_316+OPcp17_212...
 *RLcp17_317+OPcp17_212*RLcp17_318+OPcp17_26*RLcp17_37+OPcp17_27*RLcp17_38+OPcp17_29*RLcp17_310-OPcp17_310*RLcp17_211-OPcp17_312*RLcp17_213-OPcp17_312...
 *RLcp17_216-OPcp17_312*RLcp17_217-OPcp17_312*RLcp17_218-OPcp17_36*RLcp17_27-OPcp17_37*RLcp17_28-OPcp17_39*RLcp17_210;
    ACcp17_218 = qdd(2)-OMcp17_110*ORcp17_311-OMcp17_112*ORcp17_313-OMcp17_112*ORcp17_316-OMcp17_112*ORcp17_317-OMcp17_112*ORcp17_318-OMcp17_16*...
 ORcp17_37-OMcp17_17*ORcp17_38-OMcp17_19*ORcp17_310+OMcp17_310*ORcp17_111+OMcp17_312*ORcp17_113+OMcp17_312*ORcp17_116+OMcp17_312*ORcp17_117+OMcp17_312...
 *ORcp17_118+OMcp17_36*ORcp17_17+OMcp17_37*ORcp17_18+OMcp17_39*ORcp17_110-OPcp17_110*RLcp17_311-OPcp17_112*RLcp17_313-OPcp17_112*RLcp17_316-OPcp17_112...
 *RLcp17_317-OPcp17_112*RLcp17_318-OPcp17_16*RLcp17_37-OPcp17_17*RLcp17_38-OPcp17_19*RLcp17_310+OPcp17_310*RLcp17_111+OPcp17_312*RLcp17_113+OPcp17_312...
 *RLcp17_116+OPcp17_312*RLcp17_117+OPcp17_312*RLcp17_118+OPcp17_36*RLcp17_17+OPcp17_37*RLcp17_18+OPcp17_39*RLcp17_110;
    ACcp17_318 = qdd(3)+OMcp17_110*ORcp17_211+OMcp17_112*ORcp17_213+OMcp17_112*ORcp17_216+OMcp17_112*ORcp17_217+OMcp17_112*ORcp17_218+OMcp17_16*...
 ORcp17_27+OMcp17_17*ORcp17_28+OMcp17_19*ORcp17_210-OMcp17_210*ORcp17_111-OMcp17_212*ORcp17_113-OMcp17_212*ORcp17_116-OMcp17_212*ORcp17_117-OMcp17_212...
 *ORcp17_118-OMcp17_26*ORcp17_17-OMcp17_27*ORcp17_18-OMcp17_29*ORcp17_110+OPcp17_110*RLcp17_211+OPcp17_112*RLcp17_213+OPcp17_112*RLcp17_216+OPcp17_112...
 *RLcp17_217+OPcp17_112*RLcp17_218+OPcp17_16*RLcp17_27+OPcp17_17*RLcp17_28+OPcp17_19*RLcp17_210-OPcp17_210*RLcp17_111-OPcp17_212*RLcp17_113-OPcp17_212...
 *RLcp17_116-OPcp17_212*RLcp17_117-OPcp17_212*RLcp17_118-OPcp17_26*RLcp17_17-OPcp17_27*RLcp17_18-OPcp17_29*RLcp17_110;

% = = Block_1_0_0_18_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp17_118;
    sens.P(2) = POcp17_218;
    sens.P(3) = POcp17_318;
    sens.R(1,1) = ROcp17_114;
    sens.R(1,2) = ROcp17_214;
    sens.R(1,3) = ROcp17_314;
    sens.R(2,1) = ROcp17_415;
    sens.R(2,2) = ROcp17_515;
    sens.R(2,3) = ROcp17_615;
    sens.R(3,1) = ROcp17_715;
    sens.R(3,2) = ROcp17_815;
    sens.R(3,3) = ROcp17_915;
    sens.V(1) = VIcp17_118;
    sens.V(2) = VIcp17_218;
    sens.V(3) = VIcp17_318;
    sens.OM(1) = OMcp17_112;
    sens.OM(2) = OMcp17_212;
    sens.OM(3) = OMcp17_312;
    sens.A(1) = ACcp17_118;
    sens.A(2) = ACcp17_218;
    sens.A(3) = ACcp17_318;
    sens.OMP(1) = OPcp17_112;
    sens.OMP(2) = OPcp17_212;
    sens.OMP(3) = OPcp17_312;
 
% 
case 19, 


% = = Block_1_0_0_19_0_1 = = 
 
% Sensor Kinematics 


    ROcp18_25 = S4*S5;
    ROcp18_35 = -C4*S5;
    ROcp18_85 = -S4*C5;
    ROcp18_95 = C4*C5;
    ROcp18_16 = C5*C6;
    ROcp18_26 = ROcp18_25*C6+C4*S6;
    ROcp18_36 = ROcp18_35*C6+S4*S6;
    ROcp18_46 = -C5*S6;
    ROcp18_56 = -(ROcp18_25*S6-C4*C6);
    ROcp18_66 = -(ROcp18_35*S6-S4*C6);
    OMcp18_25 = qd(5)*C4;
    OMcp18_35 = qd(5)*S4;
    OMcp18_16 = qd(4)+qd(6)*S5;
    OMcp18_26 = OMcp18_25+ROcp18_85*qd(6);
    OMcp18_36 = OMcp18_35+ROcp18_95*qd(6);
    OPcp18_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp18_26 = ROcp18_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp18_35*S5-ROcp18_95*qd(4));
    OPcp18_36 = ROcp18_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp18_25*S5-ROcp18_85*qd(4));

% = = Block_1_0_0_19_0_3 = = 
 
% Sensor Kinematics 


    ROcp18_419 = ROcp18_46*C19+S19*S5;
    ROcp18_519 = ROcp18_56*C19+ROcp18_85*S19;
    ROcp18_619 = ROcp18_66*C19+ROcp18_95*S19;
    ROcp18_719 = -(ROcp18_46*S19-C19*S5);
    ROcp18_819 = -(ROcp18_56*S19-ROcp18_85*C19);
    ROcp18_919 = -(ROcp18_66*S19-ROcp18_95*C19);
    RLcp18_119 = ROcp18_16*s.dpt(1,2)+ROcp18_46*s.dpt(2,2);
    RLcp18_219 = ROcp18_26*s.dpt(1,2)+ROcp18_56*s.dpt(2,2);
    RLcp18_319 = ROcp18_36*s.dpt(1,2)+ROcp18_66*s.dpt(2,2);
    POcp18_119 = RLcp18_119+q(1);
    POcp18_219 = RLcp18_219+q(2);
    POcp18_319 = RLcp18_319+q(3);
    OMcp18_119 = OMcp18_16+ROcp18_16*qd(19);
    OMcp18_219 = OMcp18_26+ROcp18_26*qd(19);
    OMcp18_319 = OMcp18_36+ROcp18_36*qd(19);
    ORcp18_119 = OMcp18_26*RLcp18_319-OMcp18_36*RLcp18_219;
    ORcp18_219 = -(OMcp18_16*RLcp18_319-OMcp18_36*RLcp18_119);
    ORcp18_319 = OMcp18_16*RLcp18_219-OMcp18_26*RLcp18_119;
    VIcp18_119 = ORcp18_119+qd(1);
    VIcp18_219 = ORcp18_219+qd(2);
    VIcp18_319 = ORcp18_319+qd(3);
    OPcp18_119 = OPcp18_16+ROcp18_16*qdd(19)+qd(19)*(OMcp18_26*ROcp18_36-OMcp18_36*ROcp18_26);
    OPcp18_219 = OPcp18_26+ROcp18_26*qdd(19)-qd(19)*(OMcp18_16*ROcp18_36-OMcp18_36*ROcp18_16);
    OPcp18_319 = OPcp18_36+ROcp18_36*qdd(19)+qd(19)*(OMcp18_16*ROcp18_26-OMcp18_26*ROcp18_16);
    ACcp18_119 = qdd(1)+OMcp18_26*ORcp18_319-OMcp18_36*ORcp18_219+OPcp18_26*RLcp18_319-OPcp18_36*RLcp18_219;
    ACcp18_219 = qdd(2)-OMcp18_16*ORcp18_319+OMcp18_36*ORcp18_119-OPcp18_16*RLcp18_319+OPcp18_36*RLcp18_119;
    ACcp18_319 = qdd(3)+OMcp18_16*ORcp18_219-OMcp18_26*ORcp18_119+OPcp18_16*RLcp18_219-OPcp18_26*RLcp18_119;

% = = Block_1_0_0_19_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp18_119;
    sens.P(2) = POcp18_219;
    sens.P(3) = POcp18_319;
    sens.R(1,1) = ROcp18_16;
    sens.R(1,2) = ROcp18_26;
    sens.R(1,3) = ROcp18_36;
    sens.R(2,1) = ROcp18_419;
    sens.R(2,2) = ROcp18_519;
    sens.R(2,3) = ROcp18_619;
    sens.R(3,1) = ROcp18_719;
    sens.R(3,2) = ROcp18_819;
    sens.R(3,3) = ROcp18_919;
    sens.V(1) = VIcp18_119;
    sens.V(2) = VIcp18_219;
    sens.V(3) = VIcp18_319;
    sens.OM(1) = OMcp18_119;
    sens.OM(2) = OMcp18_219;
    sens.OM(3) = OMcp18_319;
    sens.A(1) = ACcp18_119;
    sens.A(2) = ACcp18_219;
    sens.A(3) = ACcp18_319;
    sens.OMP(1) = OPcp18_119;
    sens.OMP(2) = OPcp18_219;
    sens.OMP(3) = OPcp18_319;
 
% 
case 20, 


% = = Block_1_0_0_20_0_1 = = 
 
% Sensor Kinematics 


    ROcp19_25 = S4*S5;
    ROcp19_35 = -C4*S5;
    ROcp19_85 = -S4*C5;
    ROcp19_95 = C4*C5;
    ROcp19_16 = C5*C6;
    ROcp19_26 = ROcp19_25*C6+C4*S6;
    ROcp19_36 = ROcp19_35*C6+S4*S6;
    ROcp19_46 = -C5*S6;
    ROcp19_56 = -(ROcp19_25*S6-C4*C6);
    ROcp19_66 = -(ROcp19_35*S6-S4*C6);
    OMcp19_25 = qd(5)*C4;
    OMcp19_35 = qd(5)*S4;
    OMcp19_16 = qd(4)+qd(6)*S5;
    OMcp19_26 = OMcp19_25+ROcp19_85*qd(6);
    OMcp19_36 = OMcp19_35+ROcp19_95*qd(6);
    OPcp19_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp19_26 = ROcp19_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp19_35*S5-ROcp19_95*qd(4));
    OPcp19_36 = ROcp19_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp19_25*S5-ROcp19_85*qd(4));

% = = Block_1_0_0_20_0_3 = = 
 
% Sensor Kinematics 


    ROcp19_419 = ROcp19_46*C19+S19*S5;
    ROcp19_519 = ROcp19_56*C19+ROcp19_85*S19;
    ROcp19_619 = ROcp19_66*C19+ROcp19_95*S19;
    ROcp19_719 = -(ROcp19_46*S19-C19*S5);
    ROcp19_819 = -(ROcp19_56*S19-ROcp19_85*C19);
    ROcp19_919 = -(ROcp19_66*S19-ROcp19_95*C19);
    ROcp19_120 = ROcp19_16*C20+ROcp19_419*S20;
    ROcp19_220 = ROcp19_26*C20+ROcp19_519*S20;
    ROcp19_320 = ROcp19_36*C20+ROcp19_619*S20;
    ROcp19_420 = -(ROcp19_16*S20-ROcp19_419*C20);
    ROcp19_520 = -(ROcp19_26*S20-ROcp19_519*C20);
    ROcp19_620 = -(ROcp19_36*S20-ROcp19_619*C20);
    RLcp19_119 = ROcp19_16*s.dpt(1,2)+ROcp19_46*s.dpt(2,2);
    RLcp19_219 = ROcp19_26*s.dpt(1,2)+ROcp19_56*s.dpt(2,2);
    RLcp19_319 = ROcp19_36*s.dpt(1,2)+ROcp19_66*s.dpt(2,2);
    OMcp19_119 = OMcp19_16+ROcp19_16*qd(19);
    OMcp19_219 = OMcp19_26+ROcp19_26*qd(19);
    OMcp19_319 = OMcp19_36+ROcp19_36*qd(19);
    ORcp19_119 = OMcp19_26*RLcp19_319-OMcp19_36*RLcp19_219;
    ORcp19_219 = -(OMcp19_16*RLcp19_319-OMcp19_36*RLcp19_119);
    ORcp19_319 = OMcp19_16*RLcp19_219-OMcp19_26*RLcp19_119;
    OPcp19_119 = OPcp19_16+ROcp19_16*qdd(19)+qd(19)*(OMcp19_26*ROcp19_36-OMcp19_36*ROcp19_26);
    OPcp19_219 = OPcp19_26+ROcp19_26*qdd(19)-qd(19)*(OMcp19_16*ROcp19_36-OMcp19_36*ROcp19_16);
    OPcp19_319 = OPcp19_36+ROcp19_36*qdd(19)+qd(19)*(OMcp19_16*ROcp19_26-OMcp19_26*ROcp19_16);
    RLcp19_120 = ROcp19_16*s.dpt(1,12)+ROcp19_419*s.dpt(2,12)+ROcp19_719*s.dpt(3,12);
    RLcp19_220 = ROcp19_26*s.dpt(1,12)+ROcp19_519*s.dpt(2,12)+ROcp19_819*s.dpt(3,12);
    RLcp19_320 = ROcp19_36*s.dpt(1,12)+ROcp19_619*s.dpt(2,12)+ROcp19_919*s.dpt(3,12);
    POcp19_120 = RLcp19_119+RLcp19_120+q(1);
    POcp19_220 = RLcp19_219+RLcp19_220+q(2);
    POcp19_320 = RLcp19_319+RLcp19_320+q(3);
    OMcp19_120 = OMcp19_119+ROcp19_719*qd(20);
    OMcp19_220 = OMcp19_219+ROcp19_819*qd(20);
    OMcp19_320 = OMcp19_319+ROcp19_919*qd(20);
    ORcp19_120 = OMcp19_219*RLcp19_320-OMcp19_319*RLcp19_220;
    ORcp19_220 = -(OMcp19_119*RLcp19_320-OMcp19_319*RLcp19_120);
    ORcp19_320 = OMcp19_119*RLcp19_220-OMcp19_219*RLcp19_120;
    VIcp19_120 = ORcp19_119+ORcp19_120+qd(1);
    VIcp19_220 = ORcp19_219+ORcp19_220+qd(2);
    VIcp19_320 = ORcp19_319+ORcp19_320+qd(3);
    OPcp19_120 = OPcp19_119+ROcp19_719*qdd(20)+qd(20)*(OMcp19_219*ROcp19_919-OMcp19_319*ROcp19_819);
    OPcp19_220 = OPcp19_219+ROcp19_819*qdd(20)-qd(20)*(OMcp19_119*ROcp19_919-OMcp19_319*ROcp19_719);
    OPcp19_320 = OPcp19_319+ROcp19_919*qdd(20)+qd(20)*(OMcp19_119*ROcp19_819-OMcp19_219*ROcp19_719);
    ACcp19_120 = qdd(1)+OMcp19_219*ORcp19_320+OMcp19_26*ORcp19_319-OMcp19_319*ORcp19_220-OMcp19_36*ORcp19_219+OPcp19_219*RLcp19_320+OPcp19_26*...
 RLcp19_319-OPcp19_319*RLcp19_220-OPcp19_36*RLcp19_219;
    ACcp19_220 = qdd(2)-OMcp19_119*ORcp19_320-OMcp19_16*ORcp19_319+OMcp19_319*ORcp19_120+OMcp19_36*ORcp19_119-OPcp19_119*RLcp19_320-OPcp19_16*...
 RLcp19_319+OPcp19_319*RLcp19_120+OPcp19_36*RLcp19_119;
    ACcp19_320 = qdd(3)+OMcp19_119*ORcp19_220+OMcp19_16*ORcp19_219-OMcp19_219*ORcp19_120-OMcp19_26*ORcp19_119+OPcp19_119*RLcp19_220+OPcp19_16*...
 RLcp19_219-OPcp19_219*RLcp19_120-OPcp19_26*RLcp19_119;

% = = Block_1_0_0_20_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp19_120;
    sens.P(2) = POcp19_220;
    sens.P(3) = POcp19_320;
    sens.R(1,1) = ROcp19_120;
    sens.R(1,2) = ROcp19_220;
    sens.R(1,3) = ROcp19_320;
    sens.R(2,1) = ROcp19_420;
    sens.R(2,2) = ROcp19_520;
    sens.R(2,3) = ROcp19_620;
    sens.R(3,1) = ROcp19_719;
    sens.R(3,2) = ROcp19_819;
    sens.R(3,3) = ROcp19_919;
    sens.V(1) = VIcp19_120;
    sens.V(2) = VIcp19_220;
    sens.V(3) = VIcp19_320;
    sens.OM(1) = OMcp19_120;
    sens.OM(2) = OMcp19_220;
    sens.OM(3) = OMcp19_320;
    sens.A(1) = ACcp19_120;
    sens.A(2) = ACcp19_220;
    sens.A(3) = ACcp19_320;
    sens.OMP(1) = OPcp19_120;
    sens.OMP(2) = OPcp19_220;
    sens.OMP(3) = OPcp19_320;
 
% 
case 21, 


% = = Block_1_0_0_21_0_1 = = 
 
% Sensor Kinematics 


    ROcp20_25 = S4*S5;
    ROcp20_35 = -C4*S5;
    ROcp20_85 = -S4*C5;
    ROcp20_95 = C4*C5;
    ROcp20_16 = C5*C6;
    ROcp20_26 = ROcp20_25*C6+C4*S6;
    ROcp20_36 = ROcp20_35*C6+S4*S6;
    ROcp20_46 = -C5*S6;
    ROcp20_56 = -(ROcp20_25*S6-C4*C6);
    ROcp20_66 = -(ROcp20_35*S6-S4*C6);
    OMcp20_25 = qd(5)*C4;
    OMcp20_35 = qd(5)*S4;
    OMcp20_16 = qd(4)+qd(6)*S5;
    OMcp20_26 = OMcp20_25+ROcp20_85*qd(6);
    OMcp20_36 = OMcp20_35+ROcp20_95*qd(6);
    OPcp20_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp20_26 = ROcp20_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp20_35*S5-ROcp20_95*qd(4));
    OPcp20_36 = ROcp20_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp20_25*S5-ROcp20_85*qd(4));

% = = Block_1_0_0_21_0_3 = = 
 
% Sensor Kinematics 


    ROcp20_419 = ROcp20_46*C19+S19*S5;
    ROcp20_519 = ROcp20_56*C19+ROcp20_85*S19;
    ROcp20_619 = ROcp20_66*C19+ROcp20_95*S19;
    ROcp20_719 = -(ROcp20_46*S19-C19*S5);
    ROcp20_819 = -(ROcp20_56*S19-ROcp20_85*C19);
    ROcp20_919 = -(ROcp20_66*S19-ROcp20_95*C19);
    ROcp20_120 = ROcp20_16*C20+ROcp20_419*S20;
    ROcp20_220 = ROcp20_26*C20+ROcp20_519*S20;
    ROcp20_320 = ROcp20_36*C20+ROcp20_619*S20;
    ROcp20_420 = -(ROcp20_16*S20-ROcp20_419*C20);
    ROcp20_520 = -(ROcp20_26*S20-ROcp20_519*C20);
    ROcp20_620 = -(ROcp20_36*S20-ROcp20_619*C20);
    ROcp20_121 = ROcp20_120*C21-ROcp20_719*S21;
    ROcp20_221 = ROcp20_220*C21-ROcp20_819*S21;
    ROcp20_321 = ROcp20_320*C21-ROcp20_919*S21;
    ROcp20_721 = ROcp20_120*S21+ROcp20_719*C21;
    ROcp20_821 = ROcp20_220*S21+ROcp20_819*C21;
    ROcp20_921 = ROcp20_320*S21+ROcp20_919*C21;
    RLcp20_119 = ROcp20_16*s.dpt(1,2)+ROcp20_46*s.dpt(2,2);
    RLcp20_219 = ROcp20_26*s.dpt(1,2)+ROcp20_56*s.dpt(2,2);
    RLcp20_319 = ROcp20_36*s.dpt(1,2)+ROcp20_66*s.dpt(2,2);
    OMcp20_119 = OMcp20_16+ROcp20_16*qd(19);
    OMcp20_219 = OMcp20_26+ROcp20_26*qd(19);
    OMcp20_319 = OMcp20_36+ROcp20_36*qd(19);
    ORcp20_119 = OMcp20_26*RLcp20_319-OMcp20_36*RLcp20_219;
    ORcp20_219 = -(OMcp20_16*RLcp20_319-OMcp20_36*RLcp20_119);
    ORcp20_319 = OMcp20_16*RLcp20_219-OMcp20_26*RLcp20_119;
    OPcp20_119 = OPcp20_16+ROcp20_16*qdd(19)+qd(19)*(OMcp20_26*ROcp20_36-OMcp20_36*ROcp20_26);
    OPcp20_219 = OPcp20_26+ROcp20_26*qdd(19)-qd(19)*(OMcp20_16*ROcp20_36-OMcp20_36*ROcp20_16);
    OPcp20_319 = OPcp20_36+ROcp20_36*qdd(19)+qd(19)*(OMcp20_16*ROcp20_26-OMcp20_26*ROcp20_16);
    RLcp20_120 = ROcp20_16*s.dpt(1,12)+ROcp20_419*s.dpt(2,12)+ROcp20_719*s.dpt(3,12);
    RLcp20_220 = ROcp20_26*s.dpt(1,12)+ROcp20_519*s.dpt(2,12)+ROcp20_819*s.dpt(3,12);
    RLcp20_320 = ROcp20_36*s.dpt(1,12)+ROcp20_619*s.dpt(2,12)+ROcp20_919*s.dpt(3,12);
    POcp20_120 = RLcp20_119+RLcp20_120+q(1);
    POcp20_220 = RLcp20_219+RLcp20_220+q(2);
    POcp20_320 = RLcp20_319+RLcp20_320+q(3);
    OMcp20_120 = OMcp20_119+ROcp20_719*qd(20);
    OMcp20_220 = OMcp20_219+ROcp20_819*qd(20);
    OMcp20_320 = OMcp20_319+ROcp20_919*qd(20);
    ORcp20_120 = OMcp20_219*RLcp20_320-OMcp20_319*RLcp20_220;
    ORcp20_220 = -(OMcp20_119*RLcp20_320-OMcp20_319*RLcp20_120);
    ORcp20_320 = OMcp20_119*RLcp20_220-OMcp20_219*RLcp20_120;
    VIcp20_120 = ORcp20_119+ORcp20_120+qd(1);
    VIcp20_220 = ORcp20_219+ORcp20_220+qd(2);
    VIcp20_320 = ORcp20_319+ORcp20_320+qd(3);
    ACcp20_120 = qdd(1)+OMcp20_219*ORcp20_320+OMcp20_26*ORcp20_319-OMcp20_319*ORcp20_220-OMcp20_36*ORcp20_219+OPcp20_219*RLcp20_320+OPcp20_26*...
 RLcp20_319-OPcp20_319*RLcp20_220-OPcp20_36*RLcp20_219;
    ACcp20_220 = qdd(2)-OMcp20_119*ORcp20_320-OMcp20_16*ORcp20_319+OMcp20_319*ORcp20_120+OMcp20_36*ORcp20_119-OPcp20_119*RLcp20_320-OPcp20_16*...
 RLcp20_319+OPcp20_319*RLcp20_120+OPcp20_36*RLcp20_119;
    ACcp20_320 = qdd(3)+OMcp20_119*ORcp20_220+OMcp20_16*ORcp20_219-OMcp20_219*ORcp20_120-OMcp20_26*ORcp20_119+OPcp20_119*RLcp20_220+OPcp20_16*...
 RLcp20_219-OPcp20_219*RLcp20_120-OPcp20_26*RLcp20_119;
    OMcp20_121 = OMcp20_120+ROcp20_420*qd(21);
    OMcp20_221 = OMcp20_220+ROcp20_520*qd(21);
    OMcp20_321 = OMcp20_320+ROcp20_620*qd(21);
    OPcp20_121 = OPcp20_119+ROcp20_420*qdd(21)+ROcp20_719*qdd(20)+qd(20)*(OMcp20_219*ROcp20_919-OMcp20_319*ROcp20_819)+qd(21)*(OMcp20_220*...
 ROcp20_620-OMcp20_320*ROcp20_520);
    OPcp20_221 = OPcp20_219+ROcp20_520*qdd(21)+ROcp20_819*qdd(20)-qd(20)*(OMcp20_119*ROcp20_919-OMcp20_319*ROcp20_719)-qd(21)*(OMcp20_120*...
 ROcp20_620-OMcp20_320*ROcp20_420);
    OPcp20_321 = OPcp20_319+ROcp20_620*qdd(21)+ROcp20_919*qdd(20)+qd(20)*(OMcp20_119*ROcp20_819-OMcp20_219*ROcp20_719)+qd(21)*(OMcp20_120*...
 ROcp20_520-OMcp20_220*ROcp20_420);

% = = Block_1_0_0_21_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp20_120;
    sens.P(2) = POcp20_220;
    sens.P(3) = POcp20_320;
    sens.R(1,1) = ROcp20_121;
    sens.R(1,2) = ROcp20_221;
    sens.R(1,3) = ROcp20_321;
    sens.R(2,1) = ROcp20_420;
    sens.R(2,2) = ROcp20_520;
    sens.R(2,3) = ROcp20_620;
    sens.R(3,1) = ROcp20_721;
    sens.R(3,2) = ROcp20_821;
    sens.R(3,3) = ROcp20_921;
    sens.V(1) = VIcp20_120;
    sens.V(2) = VIcp20_220;
    sens.V(3) = VIcp20_320;
    sens.OM(1) = OMcp20_121;
    sens.OM(2) = OMcp20_221;
    sens.OM(3) = OMcp20_321;
    sens.A(1) = ACcp20_120;
    sens.A(2) = ACcp20_220;
    sens.A(3) = ACcp20_320;
    sens.OMP(1) = OPcp20_121;
    sens.OMP(2) = OPcp20_221;
    sens.OMP(3) = OPcp20_321;
 
% 
case 22, 


% = = Block_1_0_0_22_0_1 = = 
 
% Sensor Kinematics 


    ROcp21_25 = S4*S5;
    ROcp21_35 = -C4*S5;
    ROcp21_85 = -S4*C5;
    ROcp21_95 = C4*C5;
    ROcp21_16 = C5*C6;
    ROcp21_26 = ROcp21_25*C6+C4*S6;
    ROcp21_36 = ROcp21_35*C6+S4*S6;
    ROcp21_46 = -C5*S6;
    ROcp21_56 = -(ROcp21_25*S6-C4*C6);
    ROcp21_66 = -(ROcp21_35*S6-S4*C6);
    OMcp21_25 = qd(5)*C4;
    OMcp21_35 = qd(5)*S4;
    OMcp21_16 = qd(4)+qd(6)*S5;
    OMcp21_26 = OMcp21_25+ROcp21_85*qd(6);
    OMcp21_36 = OMcp21_35+ROcp21_95*qd(6);
    OPcp21_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp21_26 = ROcp21_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp21_35*S5-ROcp21_95*qd(4));
    OPcp21_36 = ROcp21_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp21_25*S5-ROcp21_85*qd(4));

% = = Block_1_0_0_22_0_3 = = 
 
% Sensor Kinematics 


    ROcp21_419 = ROcp21_46*C19+S19*S5;
    ROcp21_519 = ROcp21_56*C19+ROcp21_85*S19;
    ROcp21_619 = ROcp21_66*C19+ROcp21_95*S19;
    ROcp21_719 = -(ROcp21_46*S19-C19*S5);
    ROcp21_819 = -(ROcp21_56*S19-ROcp21_85*C19);
    ROcp21_919 = -(ROcp21_66*S19-ROcp21_95*C19);
    ROcp21_120 = ROcp21_16*C20+ROcp21_419*S20;
    ROcp21_220 = ROcp21_26*C20+ROcp21_519*S20;
    ROcp21_320 = ROcp21_36*C20+ROcp21_619*S20;
    ROcp21_420 = -(ROcp21_16*S20-ROcp21_419*C20);
    ROcp21_520 = -(ROcp21_26*S20-ROcp21_519*C20);
    ROcp21_620 = -(ROcp21_36*S20-ROcp21_619*C20);
    ROcp21_121 = ROcp21_120*C21-ROcp21_719*S21;
    ROcp21_221 = ROcp21_220*C21-ROcp21_819*S21;
    ROcp21_321 = ROcp21_320*C21-ROcp21_919*S21;
    ROcp21_721 = ROcp21_120*S21+ROcp21_719*C21;
    ROcp21_821 = ROcp21_220*S21+ROcp21_819*C21;
    ROcp21_921 = ROcp21_320*S21+ROcp21_919*C21;
    ROcp21_122 = ROcp21_121*C22-ROcp21_721*S22;
    ROcp21_222 = ROcp21_221*C22-ROcp21_821*S22;
    ROcp21_322 = ROcp21_321*C22-ROcp21_921*S22;
    ROcp21_722 = ROcp21_121*S22+ROcp21_721*C22;
    ROcp21_822 = ROcp21_221*S22+ROcp21_821*C22;
    ROcp21_922 = ROcp21_321*S22+ROcp21_921*C22;
    RLcp21_119 = ROcp21_16*s.dpt(1,2)+ROcp21_46*s.dpt(2,2);
    RLcp21_219 = ROcp21_26*s.dpt(1,2)+ROcp21_56*s.dpt(2,2);
    RLcp21_319 = ROcp21_36*s.dpt(1,2)+ROcp21_66*s.dpt(2,2);
    OMcp21_119 = OMcp21_16+ROcp21_16*qd(19);
    OMcp21_219 = OMcp21_26+ROcp21_26*qd(19);
    OMcp21_319 = OMcp21_36+ROcp21_36*qd(19);
    ORcp21_119 = OMcp21_26*RLcp21_319-OMcp21_36*RLcp21_219;
    ORcp21_219 = -(OMcp21_16*RLcp21_319-OMcp21_36*RLcp21_119);
    ORcp21_319 = OMcp21_16*RLcp21_219-OMcp21_26*RLcp21_119;
    OPcp21_119 = OPcp21_16+ROcp21_16*qdd(19)+qd(19)*(OMcp21_26*ROcp21_36-OMcp21_36*ROcp21_26);
    OPcp21_219 = OPcp21_26+ROcp21_26*qdd(19)-qd(19)*(OMcp21_16*ROcp21_36-OMcp21_36*ROcp21_16);
    OPcp21_319 = OPcp21_36+ROcp21_36*qdd(19)+qd(19)*(OMcp21_16*ROcp21_26-OMcp21_26*ROcp21_16);
    RLcp21_120 = ROcp21_16*s.dpt(1,12)+ROcp21_419*s.dpt(2,12)+ROcp21_719*s.dpt(3,12);
    RLcp21_220 = ROcp21_26*s.dpt(1,12)+ROcp21_519*s.dpt(2,12)+ROcp21_819*s.dpt(3,12);
    RLcp21_320 = ROcp21_36*s.dpt(1,12)+ROcp21_619*s.dpt(2,12)+ROcp21_919*s.dpt(3,12);
    OMcp21_120 = OMcp21_119+ROcp21_719*qd(20);
    OMcp21_220 = OMcp21_219+ROcp21_819*qd(20);
    OMcp21_320 = OMcp21_319+ROcp21_919*qd(20);
    ORcp21_120 = OMcp21_219*RLcp21_320-OMcp21_319*RLcp21_220;
    ORcp21_220 = -(OMcp21_119*RLcp21_320-OMcp21_319*RLcp21_120);
    ORcp21_320 = OMcp21_119*RLcp21_220-OMcp21_219*RLcp21_120;
    OMcp21_121 = OMcp21_120+ROcp21_420*qd(21);
    OMcp21_221 = OMcp21_220+ROcp21_520*qd(21);
    OMcp21_321 = OMcp21_320+ROcp21_620*qd(21);
    OPcp21_121 = OPcp21_119+ROcp21_420*qdd(21)+ROcp21_719*qdd(20)+qd(20)*(OMcp21_219*ROcp21_919-OMcp21_319*ROcp21_819)+qd(21)*(OMcp21_220*...
 ROcp21_620-OMcp21_320*ROcp21_520);
    OPcp21_221 = OPcp21_219+ROcp21_520*qdd(21)+ROcp21_819*qdd(20)-qd(20)*(OMcp21_119*ROcp21_919-OMcp21_319*ROcp21_719)-qd(21)*(OMcp21_120*...
 ROcp21_620-OMcp21_320*ROcp21_420);
    OPcp21_321 = OPcp21_319+ROcp21_620*qdd(21)+ROcp21_919*qdd(20)+qd(20)*(OMcp21_119*ROcp21_819-OMcp21_219*ROcp21_719)+qd(21)*(OMcp21_120*...
 ROcp21_520-OMcp21_220*ROcp21_420);
    RLcp21_122 = ROcp21_721*s.dpt(3,14);
    RLcp21_222 = ROcp21_821*s.dpt(3,14);
    RLcp21_322 = ROcp21_921*s.dpt(3,14);
    POcp21_122 = RLcp21_119+RLcp21_120+RLcp21_122+q(1);
    POcp21_222 = RLcp21_219+RLcp21_220+RLcp21_222+q(2);
    POcp21_322 = RLcp21_319+RLcp21_320+RLcp21_322+q(3);
    OMcp21_122 = OMcp21_121+ROcp21_420*qd(22);
    OMcp21_222 = OMcp21_221+ROcp21_520*qd(22);
    OMcp21_322 = OMcp21_321+ROcp21_620*qd(22);
    ORcp21_122 = OMcp21_221*RLcp21_322-OMcp21_321*RLcp21_222;
    ORcp21_222 = -(OMcp21_121*RLcp21_322-OMcp21_321*RLcp21_122);
    ORcp21_322 = OMcp21_121*RLcp21_222-OMcp21_221*RLcp21_122;
    VIcp21_122 = ORcp21_119+ORcp21_120+ORcp21_122+qd(1);
    VIcp21_222 = ORcp21_219+ORcp21_220+ORcp21_222+qd(2);
    VIcp21_322 = ORcp21_319+ORcp21_320+ORcp21_322+qd(3);
    OPcp21_122 = OPcp21_121+ROcp21_420*qdd(22)+qd(22)*(OMcp21_221*ROcp21_620-OMcp21_321*ROcp21_520);
    OPcp21_222 = OPcp21_221+ROcp21_520*qdd(22)-qd(22)*(OMcp21_121*ROcp21_620-OMcp21_321*ROcp21_420);
    OPcp21_322 = OPcp21_321+ROcp21_620*qdd(22)+qd(22)*(OMcp21_121*ROcp21_520-OMcp21_221*ROcp21_420);
    ACcp21_122 = qdd(1)+OMcp21_219*ORcp21_320+OMcp21_221*ORcp21_322+OMcp21_26*ORcp21_319-OMcp21_319*ORcp21_220-OMcp21_321*ORcp21_222-OMcp21_36*...
 ORcp21_219+OPcp21_219*RLcp21_320+OPcp21_221*RLcp21_322+OPcp21_26*RLcp21_319-OPcp21_319*RLcp21_220-OPcp21_321*RLcp21_222-OPcp21_36*RLcp21_219;
    ACcp21_222 = qdd(2)-OMcp21_119*ORcp21_320-OMcp21_121*ORcp21_322-OMcp21_16*ORcp21_319+OMcp21_319*ORcp21_120+OMcp21_321*ORcp21_122+OMcp21_36*...
 ORcp21_119-OPcp21_119*RLcp21_320-OPcp21_121*RLcp21_322-OPcp21_16*RLcp21_319+OPcp21_319*RLcp21_120+OPcp21_321*RLcp21_122+OPcp21_36*RLcp21_119;
    ACcp21_322 = qdd(3)+OMcp21_119*ORcp21_220+OMcp21_121*ORcp21_222+OMcp21_16*ORcp21_219-OMcp21_219*ORcp21_120-OMcp21_221*ORcp21_122-OMcp21_26*...
 ORcp21_119+OPcp21_119*RLcp21_220+OPcp21_121*RLcp21_222+OPcp21_16*RLcp21_219-OPcp21_219*RLcp21_120-OPcp21_221*RLcp21_122-OPcp21_26*RLcp21_119;

% = = Block_1_0_0_22_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp21_122;
    sens.P(2) = POcp21_222;
    sens.P(3) = POcp21_322;
    sens.R(1,1) = ROcp21_122;
    sens.R(1,2) = ROcp21_222;
    sens.R(1,3) = ROcp21_322;
    sens.R(2,1) = ROcp21_420;
    sens.R(2,2) = ROcp21_520;
    sens.R(2,3) = ROcp21_620;
    sens.R(3,1) = ROcp21_722;
    sens.R(3,2) = ROcp21_822;
    sens.R(3,3) = ROcp21_922;
    sens.V(1) = VIcp21_122;
    sens.V(2) = VIcp21_222;
    sens.V(3) = VIcp21_322;
    sens.OM(1) = OMcp21_122;
    sens.OM(2) = OMcp21_222;
    sens.OM(3) = OMcp21_322;
    sens.A(1) = ACcp21_122;
    sens.A(2) = ACcp21_222;
    sens.A(3) = ACcp21_322;
    sens.OMP(1) = OPcp21_122;
    sens.OMP(2) = OPcp21_222;
    sens.OMP(3) = OPcp21_322;
 
% 
case 23, 


% = = Block_1_0_0_23_0_1 = = 
 
% Sensor Kinematics 


    ROcp22_25 = S4*S5;
    ROcp22_35 = -C4*S5;
    ROcp22_85 = -S4*C5;
    ROcp22_95 = C4*C5;
    ROcp22_16 = C5*C6;
    ROcp22_26 = ROcp22_25*C6+C4*S6;
    ROcp22_36 = ROcp22_35*C6+S4*S6;
    ROcp22_46 = -C5*S6;
    ROcp22_56 = -(ROcp22_25*S6-C4*C6);
    ROcp22_66 = -(ROcp22_35*S6-S4*C6);
    OMcp22_25 = qd(5)*C4;
    OMcp22_35 = qd(5)*S4;
    OMcp22_16 = qd(4)+qd(6)*S5;
    OMcp22_26 = OMcp22_25+ROcp22_85*qd(6);
    OMcp22_36 = OMcp22_35+ROcp22_95*qd(6);
    OPcp22_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp22_26 = ROcp22_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp22_35*S5-ROcp22_95*qd(4));
    OPcp22_36 = ROcp22_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp22_25*S5-ROcp22_85*qd(4));

% = = Block_1_0_0_23_0_3 = = 
 
% Sensor Kinematics 


    ROcp22_419 = ROcp22_46*C19+S19*S5;
    ROcp22_519 = ROcp22_56*C19+ROcp22_85*S19;
    ROcp22_619 = ROcp22_66*C19+ROcp22_95*S19;
    ROcp22_719 = -(ROcp22_46*S19-C19*S5);
    ROcp22_819 = -(ROcp22_56*S19-ROcp22_85*C19);
    ROcp22_919 = -(ROcp22_66*S19-ROcp22_95*C19);
    ROcp22_120 = ROcp22_16*C20+ROcp22_419*S20;
    ROcp22_220 = ROcp22_26*C20+ROcp22_519*S20;
    ROcp22_320 = ROcp22_36*C20+ROcp22_619*S20;
    ROcp22_420 = -(ROcp22_16*S20-ROcp22_419*C20);
    ROcp22_520 = -(ROcp22_26*S20-ROcp22_519*C20);
    ROcp22_620 = -(ROcp22_36*S20-ROcp22_619*C20);
    ROcp22_121 = ROcp22_120*C21-ROcp22_719*S21;
    ROcp22_221 = ROcp22_220*C21-ROcp22_819*S21;
    ROcp22_321 = ROcp22_320*C21-ROcp22_919*S21;
    ROcp22_721 = ROcp22_120*S21+ROcp22_719*C21;
    ROcp22_821 = ROcp22_220*S21+ROcp22_819*C21;
    ROcp22_921 = ROcp22_320*S21+ROcp22_919*C21;
    ROcp22_122 = ROcp22_121*C22-ROcp22_721*S22;
    ROcp22_222 = ROcp22_221*C22-ROcp22_821*S22;
    ROcp22_322 = ROcp22_321*C22-ROcp22_921*S22;
    ROcp22_722 = ROcp22_121*S22+ROcp22_721*C22;
    ROcp22_822 = ROcp22_221*S22+ROcp22_821*C22;
    ROcp22_922 = ROcp22_321*S22+ROcp22_921*C22;
    ROcp22_123 = ROcp22_122*C23-ROcp22_722*S23;
    ROcp22_223 = ROcp22_222*C23-ROcp22_822*S23;
    ROcp22_323 = ROcp22_322*C23-ROcp22_922*S23;
    ROcp22_723 = ROcp22_122*S23+ROcp22_722*C23;
    ROcp22_823 = ROcp22_222*S23+ROcp22_822*C23;
    ROcp22_923 = ROcp22_322*S23+ROcp22_922*C23;
    RLcp22_119 = ROcp22_16*s.dpt(1,2)+ROcp22_46*s.dpt(2,2);
    RLcp22_219 = ROcp22_26*s.dpt(1,2)+ROcp22_56*s.dpt(2,2);
    RLcp22_319 = ROcp22_36*s.dpt(1,2)+ROcp22_66*s.dpt(2,2);
    OMcp22_119 = OMcp22_16+ROcp22_16*qd(19);
    OMcp22_219 = OMcp22_26+ROcp22_26*qd(19);
    OMcp22_319 = OMcp22_36+ROcp22_36*qd(19);
    ORcp22_119 = OMcp22_26*RLcp22_319-OMcp22_36*RLcp22_219;
    ORcp22_219 = -(OMcp22_16*RLcp22_319-OMcp22_36*RLcp22_119);
    ORcp22_319 = OMcp22_16*RLcp22_219-OMcp22_26*RLcp22_119;
    OPcp22_119 = OPcp22_16+ROcp22_16*qdd(19)+qd(19)*(OMcp22_26*ROcp22_36-OMcp22_36*ROcp22_26);
    OPcp22_219 = OPcp22_26+ROcp22_26*qdd(19)-qd(19)*(OMcp22_16*ROcp22_36-OMcp22_36*ROcp22_16);
    OPcp22_319 = OPcp22_36+ROcp22_36*qdd(19)+qd(19)*(OMcp22_16*ROcp22_26-OMcp22_26*ROcp22_16);
    RLcp22_120 = ROcp22_16*s.dpt(1,12)+ROcp22_419*s.dpt(2,12)+ROcp22_719*s.dpt(3,12);
    RLcp22_220 = ROcp22_26*s.dpt(1,12)+ROcp22_519*s.dpt(2,12)+ROcp22_819*s.dpt(3,12);
    RLcp22_320 = ROcp22_36*s.dpt(1,12)+ROcp22_619*s.dpt(2,12)+ROcp22_919*s.dpt(3,12);
    OMcp22_120 = OMcp22_119+ROcp22_719*qd(20);
    OMcp22_220 = OMcp22_219+ROcp22_819*qd(20);
    OMcp22_320 = OMcp22_319+ROcp22_919*qd(20);
    ORcp22_120 = OMcp22_219*RLcp22_320-OMcp22_319*RLcp22_220;
    ORcp22_220 = -(OMcp22_119*RLcp22_320-OMcp22_319*RLcp22_120);
    ORcp22_320 = OMcp22_119*RLcp22_220-OMcp22_219*RLcp22_120;
    OMcp22_121 = OMcp22_120+ROcp22_420*qd(21);
    OMcp22_221 = OMcp22_220+ROcp22_520*qd(21);
    OMcp22_321 = OMcp22_320+ROcp22_620*qd(21);
    OPcp22_121 = OPcp22_119+ROcp22_420*qdd(21)+ROcp22_719*qdd(20)+qd(20)*(OMcp22_219*ROcp22_919-OMcp22_319*ROcp22_819)+qd(21)*(OMcp22_220*...
 ROcp22_620-OMcp22_320*ROcp22_520);
    OPcp22_221 = OPcp22_219+ROcp22_520*qdd(21)+ROcp22_819*qdd(20)-qd(20)*(OMcp22_119*ROcp22_919-OMcp22_319*ROcp22_719)-qd(21)*(OMcp22_120*...
 ROcp22_620-OMcp22_320*ROcp22_420);
    OPcp22_321 = OPcp22_319+ROcp22_620*qdd(21)+ROcp22_919*qdd(20)+qd(20)*(OMcp22_119*ROcp22_819-OMcp22_219*ROcp22_719)+qd(21)*(OMcp22_120*...
 ROcp22_520-OMcp22_220*ROcp22_420);
    RLcp22_122 = ROcp22_721*s.dpt(3,14);
    RLcp22_222 = ROcp22_821*s.dpt(3,14);
    RLcp22_322 = ROcp22_921*s.dpt(3,14);
    OMcp22_122 = OMcp22_121+ROcp22_420*qd(22);
    OMcp22_222 = OMcp22_221+ROcp22_520*qd(22);
    OMcp22_322 = OMcp22_321+ROcp22_620*qd(22);
    ORcp22_122 = OMcp22_221*RLcp22_322-OMcp22_321*RLcp22_222;
    ORcp22_222 = -(OMcp22_121*RLcp22_322-OMcp22_321*RLcp22_122);
    ORcp22_322 = OMcp22_121*RLcp22_222-OMcp22_221*RLcp22_122;
    OPcp22_122 = OPcp22_121+ROcp22_420*qdd(22)+qd(22)*(OMcp22_221*ROcp22_620-OMcp22_321*ROcp22_520);
    OPcp22_222 = OPcp22_221+ROcp22_520*qdd(22)-qd(22)*(OMcp22_121*ROcp22_620-OMcp22_321*ROcp22_420);
    OPcp22_322 = OPcp22_321+ROcp22_620*qdd(22)+qd(22)*(OMcp22_121*ROcp22_520-OMcp22_221*ROcp22_420);
    RLcp22_123 = ROcp22_722*s.dpt(3,15);
    RLcp22_223 = ROcp22_822*s.dpt(3,15);
    RLcp22_323 = ROcp22_922*s.dpt(3,15);
    POcp22_123 = RLcp22_119+RLcp22_120+RLcp22_122+RLcp22_123+q(1);
    POcp22_223 = RLcp22_219+RLcp22_220+RLcp22_222+RLcp22_223+q(2);
    POcp22_323 = RLcp22_319+RLcp22_320+RLcp22_322+RLcp22_323+q(3);
    OMcp22_123 = OMcp22_122+ROcp22_420*qd(23);
    OMcp22_223 = OMcp22_222+ROcp22_520*qd(23);
    OMcp22_323 = OMcp22_322+ROcp22_620*qd(23);
    ORcp22_123 = OMcp22_222*RLcp22_323-OMcp22_322*RLcp22_223;
    ORcp22_223 = -(OMcp22_122*RLcp22_323-OMcp22_322*RLcp22_123);
    ORcp22_323 = OMcp22_122*RLcp22_223-OMcp22_222*RLcp22_123;
    VIcp22_123 = ORcp22_119+ORcp22_120+ORcp22_122+ORcp22_123+qd(1);
    VIcp22_223 = ORcp22_219+ORcp22_220+ORcp22_222+ORcp22_223+qd(2);
    VIcp22_323 = ORcp22_319+ORcp22_320+ORcp22_322+ORcp22_323+qd(3);
    OPcp22_123 = OPcp22_122+ROcp22_420*qdd(23)+qd(23)*(OMcp22_222*ROcp22_620-OMcp22_322*ROcp22_520);
    OPcp22_223 = OPcp22_222+ROcp22_520*qdd(23)-qd(23)*(OMcp22_122*ROcp22_620-OMcp22_322*ROcp22_420);
    OPcp22_323 = OPcp22_322+ROcp22_620*qdd(23)+qd(23)*(OMcp22_122*ROcp22_520-OMcp22_222*ROcp22_420);
    ACcp22_123 = qdd(1)+OMcp22_219*ORcp22_320+OMcp22_221*ORcp22_322+OMcp22_222*ORcp22_323+OMcp22_26*ORcp22_319-OMcp22_319*ORcp22_220-OMcp22_321*...
 ORcp22_222-OMcp22_322*ORcp22_223-OMcp22_36*ORcp22_219+OPcp22_219*RLcp22_320+OPcp22_221*RLcp22_322+OPcp22_222*RLcp22_323+OPcp22_26*RLcp22_319-...
 OPcp22_319*RLcp22_220-OPcp22_321*RLcp22_222-OPcp22_322*RLcp22_223-OPcp22_36*RLcp22_219;
    ACcp22_223 = qdd(2)-OMcp22_119*ORcp22_320-OMcp22_121*ORcp22_322-OMcp22_122*ORcp22_323-OMcp22_16*ORcp22_319+OMcp22_319*ORcp22_120+OMcp22_321*...
 ORcp22_122+OMcp22_322*ORcp22_123+OMcp22_36*ORcp22_119-OPcp22_119*RLcp22_320-OPcp22_121*RLcp22_322-OPcp22_122*RLcp22_323-OPcp22_16*RLcp22_319+...
 OPcp22_319*RLcp22_120+OPcp22_321*RLcp22_122+OPcp22_322*RLcp22_123+OPcp22_36*RLcp22_119;
    ACcp22_323 = qdd(3)+OMcp22_119*ORcp22_220+OMcp22_121*ORcp22_222+OMcp22_122*ORcp22_223+OMcp22_16*ORcp22_219-OMcp22_219*ORcp22_120-OMcp22_221*...
 ORcp22_122-OMcp22_222*ORcp22_123-OMcp22_26*ORcp22_119+OPcp22_119*RLcp22_220+OPcp22_121*RLcp22_222+OPcp22_122*RLcp22_223+OPcp22_16*RLcp22_219-...
 OPcp22_219*RLcp22_120-OPcp22_221*RLcp22_122-OPcp22_222*RLcp22_123-OPcp22_26*RLcp22_119;

% = = Block_1_0_0_23_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp22_123;
    sens.P(2) = POcp22_223;
    sens.P(3) = POcp22_323;
    sens.R(1,1) = ROcp22_123;
    sens.R(1,2) = ROcp22_223;
    sens.R(1,3) = ROcp22_323;
    sens.R(2,1) = ROcp22_420;
    sens.R(2,2) = ROcp22_520;
    sens.R(2,3) = ROcp22_620;
    sens.R(3,1) = ROcp22_723;
    sens.R(3,2) = ROcp22_823;
    sens.R(3,3) = ROcp22_923;
    sens.V(1) = VIcp22_123;
    sens.V(2) = VIcp22_223;
    sens.V(3) = VIcp22_323;
    sens.OM(1) = OMcp22_123;
    sens.OM(2) = OMcp22_223;
    sens.OM(3) = OMcp22_323;
    sens.A(1) = ACcp22_123;
    sens.A(2) = ACcp22_223;
    sens.A(3) = ACcp22_323;
    sens.OMP(1) = OPcp22_123;
    sens.OMP(2) = OPcp22_223;
    sens.OMP(3) = OPcp22_323;
 
% 
case 24, 


% = = Block_1_0_0_24_0_1 = = 
 
% Sensor Kinematics 


    ROcp23_25 = S4*S5;
    ROcp23_35 = -C4*S5;
    ROcp23_85 = -S4*C5;
    ROcp23_95 = C4*C5;
    ROcp23_16 = C5*C6;
    ROcp23_26 = ROcp23_25*C6+C4*S6;
    ROcp23_36 = ROcp23_35*C6+S4*S6;
    ROcp23_46 = -C5*S6;
    ROcp23_56 = -(ROcp23_25*S6-C4*C6);
    ROcp23_66 = -(ROcp23_35*S6-S4*C6);
    OMcp23_25 = qd(5)*C4;
    OMcp23_35 = qd(5)*S4;
    OMcp23_16 = qd(4)+qd(6)*S5;
    OMcp23_26 = OMcp23_25+ROcp23_85*qd(6);
    OMcp23_36 = OMcp23_35+ROcp23_95*qd(6);
    OPcp23_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp23_26 = ROcp23_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp23_35*S5-ROcp23_95*qd(4));
    OPcp23_36 = ROcp23_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp23_25*S5-ROcp23_85*qd(4));

% = = Block_1_0_0_24_0_3 = = 
 
% Sensor Kinematics 


    ROcp23_419 = ROcp23_46*C19+S19*S5;
    ROcp23_519 = ROcp23_56*C19+ROcp23_85*S19;
    ROcp23_619 = ROcp23_66*C19+ROcp23_95*S19;
    ROcp23_719 = -(ROcp23_46*S19-C19*S5);
    ROcp23_819 = -(ROcp23_56*S19-ROcp23_85*C19);
    ROcp23_919 = -(ROcp23_66*S19-ROcp23_95*C19);
    ROcp23_120 = ROcp23_16*C20+ROcp23_419*S20;
    ROcp23_220 = ROcp23_26*C20+ROcp23_519*S20;
    ROcp23_320 = ROcp23_36*C20+ROcp23_619*S20;
    ROcp23_420 = -(ROcp23_16*S20-ROcp23_419*C20);
    ROcp23_520 = -(ROcp23_26*S20-ROcp23_519*C20);
    ROcp23_620 = -(ROcp23_36*S20-ROcp23_619*C20);
    ROcp23_121 = ROcp23_120*C21-ROcp23_719*S21;
    ROcp23_221 = ROcp23_220*C21-ROcp23_819*S21;
    ROcp23_321 = ROcp23_320*C21-ROcp23_919*S21;
    ROcp23_721 = ROcp23_120*S21+ROcp23_719*C21;
    ROcp23_821 = ROcp23_220*S21+ROcp23_819*C21;
    ROcp23_921 = ROcp23_320*S21+ROcp23_919*C21;
    ROcp23_122 = ROcp23_121*C22-ROcp23_721*S22;
    ROcp23_222 = ROcp23_221*C22-ROcp23_821*S22;
    ROcp23_322 = ROcp23_321*C22-ROcp23_921*S22;
    ROcp23_722 = ROcp23_121*S22+ROcp23_721*C22;
    ROcp23_822 = ROcp23_221*S22+ROcp23_821*C22;
    ROcp23_922 = ROcp23_321*S22+ROcp23_921*C22;
    ROcp23_123 = ROcp23_122*C23-ROcp23_722*S23;
    ROcp23_223 = ROcp23_222*C23-ROcp23_822*S23;
    ROcp23_323 = ROcp23_322*C23-ROcp23_922*S23;
    ROcp23_723 = ROcp23_122*S23+ROcp23_722*C23;
    ROcp23_823 = ROcp23_222*S23+ROcp23_822*C23;
    ROcp23_923 = ROcp23_322*S23+ROcp23_922*C23;
    ROcp23_424 = ROcp23_420*C24+ROcp23_723*S24;
    ROcp23_524 = ROcp23_520*C24+ROcp23_823*S24;
    ROcp23_624 = ROcp23_620*C24+ROcp23_923*S24;
    ROcp23_724 = -(ROcp23_420*S24-ROcp23_723*C24);
    ROcp23_824 = -(ROcp23_520*S24-ROcp23_823*C24);
    ROcp23_924 = -(ROcp23_620*S24-ROcp23_923*C24);
    RLcp23_119 = ROcp23_16*s.dpt(1,2)+ROcp23_46*s.dpt(2,2);
    RLcp23_219 = ROcp23_26*s.dpt(1,2)+ROcp23_56*s.dpt(2,2);
    RLcp23_319 = ROcp23_36*s.dpt(1,2)+ROcp23_66*s.dpt(2,2);
    OMcp23_119 = OMcp23_16+ROcp23_16*qd(19);
    OMcp23_219 = OMcp23_26+ROcp23_26*qd(19);
    OMcp23_319 = OMcp23_36+ROcp23_36*qd(19);
    ORcp23_119 = OMcp23_26*RLcp23_319-OMcp23_36*RLcp23_219;
    ORcp23_219 = -(OMcp23_16*RLcp23_319-OMcp23_36*RLcp23_119);
    ORcp23_319 = OMcp23_16*RLcp23_219-OMcp23_26*RLcp23_119;
    OPcp23_119 = OPcp23_16+ROcp23_16*qdd(19)+qd(19)*(OMcp23_26*ROcp23_36-OMcp23_36*ROcp23_26);
    OPcp23_219 = OPcp23_26+ROcp23_26*qdd(19)-qd(19)*(OMcp23_16*ROcp23_36-OMcp23_36*ROcp23_16);
    OPcp23_319 = OPcp23_36+ROcp23_36*qdd(19)+qd(19)*(OMcp23_16*ROcp23_26-OMcp23_26*ROcp23_16);
    RLcp23_120 = ROcp23_16*s.dpt(1,12)+ROcp23_419*s.dpt(2,12)+ROcp23_719*s.dpt(3,12);
    RLcp23_220 = ROcp23_26*s.dpt(1,12)+ROcp23_519*s.dpt(2,12)+ROcp23_819*s.dpt(3,12);
    RLcp23_320 = ROcp23_36*s.dpt(1,12)+ROcp23_619*s.dpt(2,12)+ROcp23_919*s.dpt(3,12);
    OMcp23_120 = OMcp23_119+ROcp23_719*qd(20);
    OMcp23_220 = OMcp23_219+ROcp23_819*qd(20);
    OMcp23_320 = OMcp23_319+ROcp23_919*qd(20);
    ORcp23_120 = OMcp23_219*RLcp23_320-OMcp23_319*RLcp23_220;
    ORcp23_220 = -(OMcp23_119*RLcp23_320-OMcp23_319*RLcp23_120);
    ORcp23_320 = OMcp23_119*RLcp23_220-OMcp23_219*RLcp23_120;
    OMcp23_121 = OMcp23_120+ROcp23_420*qd(21);
    OMcp23_221 = OMcp23_220+ROcp23_520*qd(21);
    OMcp23_321 = OMcp23_320+ROcp23_620*qd(21);
    OPcp23_121 = OPcp23_119+ROcp23_420*qdd(21)+ROcp23_719*qdd(20)+qd(20)*(OMcp23_219*ROcp23_919-OMcp23_319*ROcp23_819)+qd(21)*(OMcp23_220*...
 ROcp23_620-OMcp23_320*ROcp23_520);
    OPcp23_221 = OPcp23_219+ROcp23_520*qdd(21)+ROcp23_819*qdd(20)-qd(20)*(OMcp23_119*ROcp23_919-OMcp23_319*ROcp23_719)-qd(21)*(OMcp23_120*...
 ROcp23_620-OMcp23_320*ROcp23_420);
    OPcp23_321 = OPcp23_319+ROcp23_620*qdd(21)+ROcp23_919*qdd(20)+qd(20)*(OMcp23_119*ROcp23_819-OMcp23_219*ROcp23_719)+qd(21)*(OMcp23_120*...
 ROcp23_520-OMcp23_220*ROcp23_420);
    RLcp23_122 = ROcp23_721*s.dpt(3,14);
    RLcp23_222 = ROcp23_821*s.dpt(3,14);
    RLcp23_322 = ROcp23_921*s.dpt(3,14);
    OMcp23_122 = OMcp23_121+ROcp23_420*qd(22);
    OMcp23_222 = OMcp23_221+ROcp23_520*qd(22);
    OMcp23_322 = OMcp23_321+ROcp23_620*qd(22);
    ORcp23_122 = OMcp23_221*RLcp23_322-OMcp23_321*RLcp23_222;
    ORcp23_222 = -(OMcp23_121*RLcp23_322-OMcp23_321*RLcp23_122);
    ORcp23_322 = OMcp23_121*RLcp23_222-OMcp23_221*RLcp23_122;
    OPcp23_122 = OPcp23_121+ROcp23_420*qdd(22)+qd(22)*(OMcp23_221*ROcp23_620-OMcp23_321*ROcp23_520);
    OPcp23_222 = OPcp23_221+ROcp23_520*qdd(22)-qd(22)*(OMcp23_121*ROcp23_620-OMcp23_321*ROcp23_420);
    OPcp23_322 = OPcp23_321+ROcp23_620*qdd(22)+qd(22)*(OMcp23_121*ROcp23_520-OMcp23_221*ROcp23_420);
    RLcp23_123 = ROcp23_722*s.dpt(3,15);
    RLcp23_223 = ROcp23_822*s.dpt(3,15);
    RLcp23_323 = ROcp23_922*s.dpt(3,15);
    POcp23_123 = RLcp23_119+RLcp23_120+RLcp23_122+RLcp23_123+q(1);
    POcp23_223 = RLcp23_219+RLcp23_220+RLcp23_222+RLcp23_223+q(2);
    POcp23_323 = RLcp23_319+RLcp23_320+RLcp23_322+RLcp23_323+q(3);
    OMcp23_123 = OMcp23_122+ROcp23_420*qd(23);
    OMcp23_223 = OMcp23_222+ROcp23_520*qd(23);
    OMcp23_323 = OMcp23_322+ROcp23_620*qd(23);
    ORcp23_123 = OMcp23_222*RLcp23_323-OMcp23_322*RLcp23_223;
    ORcp23_223 = -(OMcp23_122*RLcp23_323-OMcp23_322*RLcp23_123);
    ORcp23_323 = OMcp23_122*RLcp23_223-OMcp23_222*RLcp23_123;
    VIcp23_123 = ORcp23_119+ORcp23_120+ORcp23_122+ORcp23_123+qd(1);
    VIcp23_223 = ORcp23_219+ORcp23_220+ORcp23_222+ORcp23_223+qd(2);
    VIcp23_323 = ORcp23_319+ORcp23_320+ORcp23_322+ORcp23_323+qd(3);
    ACcp23_123 = qdd(1)+OMcp23_219*ORcp23_320+OMcp23_221*ORcp23_322+OMcp23_222*ORcp23_323+OMcp23_26*ORcp23_319-OMcp23_319*ORcp23_220-OMcp23_321*...
 ORcp23_222-OMcp23_322*ORcp23_223-OMcp23_36*ORcp23_219+OPcp23_219*RLcp23_320+OPcp23_221*RLcp23_322+OPcp23_222*RLcp23_323+OPcp23_26*RLcp23_319-...
 OPcp23_319*RLcp23_220-OPcp23_321*RLcp23_222-OPcp23_322*RLcp23_223-OPcp23_36*RLcp23_219;
    ACcp23_223 = qdd(2)-OMcp23_119*ORcp23_320-OMcp23_121*ORcp23_322-OMcp23_122*ORcp23_323-OMcp23_16*ORcp23_319+OMcp23_319*ORcp23_120+OMcp23_321*...
 ORcp23_122+OMcp23_322*ORcp23_123+OMcp23_36*ORcp23_119-OPcp23_119*RLcp23_320-OPcp23_121*RLcp23_322-OPcp23_122*RLcp23_323-OPcp23_16*RLcp23_319+...
 OPcp23_319*RLcp23_120+OPcp23_321*RLcp23_122+OPcp23_322*RLcp23_123+OPcp23_36*RLcp23_119;
    ACcp23_323 = qdd(3)+OMcp23_119*ORcp23_220+OMcp23_121*ORcp23_222+OMcp23_122*ORcp23_223+OMcp23_16*ORcp23_219-OMcp23_219*ORcp23_120-OMcp23_221*...
 ORcp23_122-OMcp23_222*ORcp23_123-OMcp23_26*ORcp23_119+OPcp23_119*RLcp23_220+OPcp23_121*RLcp23_222+OPcp23_122*RLcp23_223+OPcp23_16*RLcp23_219-...
 OPcp23_219*RLcp23_120-OPcp23_221*RLcp23_122-OPcp23_222*RLcp23_123-OPcp23_26*RLcp23_119;
    OMcp23_124 = OMcp23_123+ROcp23_123*qd(24);
    OMcp23_224 = OMcp23_223+ROcp23_223*qd(24);
    OMcp23_324 = OMcp23_323+ROcp23_323*qd(24);
    OPcp23_124 = OPcp23_122+ROcp23_123*qdd(24)+ROcp23_420*qdd(23)+qd(23)*(OMcp23_222*ROcp23_620-OMcp23_322*ROcp23_520)+qd(24)*(OMcp23_223*...
 ROcp23_323-OMcp23_323*ROcp23_223);
    OPcp23_224 = OPcp23_222+ROcp23_223*qdd(24)+ROcp23_520*qdd(23)-qd(23)*(OMcp23_122*ROcp23_620-OMcp23_322*ROcp23_420)-qd(24)*(OMcp23_123*...
 ROcp23_323-OMcp23_323*ROcp23_123);
    OPcp23_324 = OPcp23_322+ROcp23_323*qdd(24)+ROcp23_620*qdd(23)+qd(23)*(OMcp23_122*ROcp23_520-OMcp23_222*ROcp23_420)+qd(24)*(OMcp23_123*...
 ROcp23_223-OMcp23_223*ROcp23_123);

% = = Block_1_0_0_24_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp23_123;
    sens.P(2) = POcp23_223;
    sens.P(3) = POcp23_323;
    sens.R(1,1) = ROcp23_123;
    sens.R(1,2) = ROcp23_223;
    sens.R(1,3) = ROcp23_323;
    sens.R(2,1) = ROcp23_424;
    sens.R(2,2) = ROcp23_524;
    sens.R(2,3) = ROcp23_624;
    sens.R(3,1) = ROcp23_724;
    sens.R(3,2) = ROcp23_824;
    sens.R(3,3) = ROcp23_924;
    sens.V(1) = VIcp23_123;
    sens.V(2) = VIcp23_223;
    sens.V(3) = VIcp23_323;
    sens.OM(1) = OMcp23_124;
    sens.OM(2) = OMcp23_224;
    sens.OM(3) = OMcp23_324;
    sens.A(1) = ACcp23_123;
    sens.A(2) = ACcp23_223;
    sens.A(3) = ACcp23_323;
    sens.OMP(1) = OPcp23_124;
    sens.OMP(2) = OPcp23_224;
    sens.OMP(3) = OPcp23_324;
 
% 
case 25, 


% = = Block_1_0_0_25_0_1 = = 
 
% Sensor Kinematics 


    ROcp24_25 = S4*S5;
    ROcp24_35 = -C4*S5;
    ROcp24_85 = -S4*C5;
    ROcp24_95 = C4*C5;
    ROcp24_16 = C5*C6;
    ROcp24_26 = ROcp24_25*C6+C4*S6;
    ROcp24_36 = ROcp24_35*C6+S4*S6;
    ROcp24_46 = -C5*S6;
    ROcp24_56 = -(ROcp24_25*S6-C4*C6);
    ROcp24_66 = -(ROcp24_35*S6-S4*C6);
    OMcp24_25 = qd(5)*C4;
    OMcp24_35 = qd(5)*S4;
    OMcp24_16 = qd(4)+qd(6)*S5;
    OMcp24_26 = OMcp24_25+ROcp24_85*qd(6);
    OMcp24_36 = OMcp24_35+ROcp24_95*qd(6);
    OPcp24_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp24_26 = ROcp24_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp24_35*S5-ROcp24_95*qd(4));
    OPcp24_36 = ROcp24_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp24_25*S5-ROcp24_85*qd(4));

% = = Block_1_0_0_25_0_3 = = 
 
% Sensor Kinematics 


    ROcp24_419 = ROcp24_46*C19+S19*S5;
    ROcp24_519 = ROcp24_56*C19+ROcp24_85*S19;
    ROcp24_619 = ROcp24_66*C19+ROcp24_95*S19;
    ROcp24_719 = -(ROcp24_46*S19-C19*S5);
    ROcp24_819 = -(ROcp24_56*S19-ROcp24_85*C19);
    ROcp24_919 = -(ROcp24_66*S19-ROcp24_95*C19);
    ROcp24_120 = ROcp24_16*C20+ROcp24_419*S20;
    ROcp24_220 = ROcp24_26*C20+ROcp24_519*S20;
    ROcp24_320 = ROcp24_36*C20+ROcp24_619*S20;
    ROcp24_420 = -(ROcp24_16*S20-ROcp24_419*C20);
    ROcp24_520 = -(ROcp24_26*S20-ROcp24_519*C20);
    ROcp24_620 = -(ROcp24_36*S20-ROcp24_619*C20);
    ROcp24_121 = ROcp24_120*C21-ROcp24_719*S21;
    ROcp24_221 = ROcp24_220*C21-ROcp24_819*S21;
    ROcp24_321 = ROcp24_320*C21-ROcp24_919*S21;
    ROcp24_721 = ROcp24_120*S21+ROcp24_719*C21;
    ROcp24_821 = ROcp24_220*S21+ROcp24_819*C21;
    ROcp24_921 = ROcp24_320*S21+ROcp24_919*C21;
    ROcp24_122 = ROcp24_121*C22-ROcp24_721*S22;
    ROcp24_222 = ROcp24_221*C22-ROcp24_821*S22;
    ROcp24_322 = ROcp24_321*C22-ROcp24_921*S22;
    ROcp24_722 = ROcp24_121*S22+ROcp24_721*C22;
    ROcp24_822 = ROcp24_221*S22+ROcp24_821*C22;
    ROcp24_922 = ROcp24_321*S22+ROcp24_921*C22;
    ROcp24_123 = ROcp24_122*C23-ROcp24_722*S23;
    ROcp24_223 = ROcp24_222*C23-ROcp24_822*S23;
    ROcp24_323 = ROcp24_322*C23-ROcp24_922*S23;
    ROcp24_723 = ROcp24_122*S23+ROcp24_722*C23;
    ROcp24_823 = ROcp24_222*S23+ROcp24_822*C23;
    ROcp24_923 = ROcp24_322*S23+ROcp24_922*C23;
    ROcp24_424 = ROcp24_420*C24+ROcp24_723*S24;
    ROcp24_524 = ROcp24_520*C24+ROcp24_823*S24;
    ROcp24_624 = ROcp24_620*C24+ROcp24_923*S24;
    ROcp24_724 = -(ROcp24_420*S24-ROcp24_723*C24);
    ROcp24_824 = -(ROcp24_520*S24-ROcp24_823*C24);
    ROcp24_924 = -(ROcp24_620*S24-ROcp24_923*C24);
    ROcp24_125 = ROcp24_123*C25+ROcp24_424*S25;
    ROcp24_225 = ROcp24_223*C25+ROcp24_524*S25;
    ROcp24_325 = ROcp24_323*C25+ROcp24_624*S25;
    ROcp24_425 = -(ROcp24_123*S25-ROcp24_424*C25);
    ROcp24_525 = -(ROcp24_223*S25-ROcp24_524*C25);
    ROcp24_625 = -(ROcp24_323*S25-ROcp24_624*C25);
    RLcp24_119 = ROcp24_16*s.dpt(1,2)+ROcp24_46*s.dpt(2,2);
    RLcp24_219 = ROcp24_26*s.dpt(1,2)+ROcp24_56*s.dpt(2,2);
    RLcp24_319 = ROcp24_36*s.dpt(1,2)+ROcp24_66*s.dpt(2,2);
    OMcp24_119 = OMcp24_16+ROcp24_16*qd(19);
    OMcp24_219 = OMcp24_26+ROcp24_26*qd(19);
    OMcp24_319 = OMcp24_36+ROcp24_36*qd(19);
    ORcp24_119 = OMcp24_26*RLcp24_319-OMcp24_36*RLcp24_219;
    ORcp24_219 = -(OMcp24_16*RLcp24_319-OMcp24_36*RLcp24_119);
    ORcp24_319 = OMcp24_16*RLcp24_219-OMcp24_26*RLcp24_119;
    OPcp24_119 = OPcp24_16+ROcp24_16*qdd(19)+qd(19)*(OMcp24_26*ROcp24_36-OMcp24_36*ROcp24_26);
    OPcp24_219 = OPcp24_26+ROcp24_26*qdd(19)-qd(19)*(OMcp24_16*ROcp24_36-OMcp24_36*ROcp24_16);
    OPcp24_319 = OPcp24_36+ROcp24_36*qdd(19)+qd(19)*(OMcp24_16*ROcp24_26-OMcp24_26*ROcp24_16);
    RLcp24_120 = ROcp24_16*s.dpt(1,12)+ROcp24_419*s.dpt(2,12)+ROcp24_719*s.dpt(3,12);
    RLcp24_220 = ROcp24_26*s.dpt(1,12)+ROcp24_519*s.dpt(2,12)+ROcp24_819*s.dpt(3,12);
    RLcp24_320 = ROcp24_36*s.dpt(1,12)+ROcp24_619*s.dpt(2,12)+ROcp24_919*s.dpt(3,12);
    OMcp24_120 = OMcp24_119+ROcp24_719*qd(20);
    OMcp24_220 = OMcp24_219+ROcp24_819*qd(20);
    OMcp24_320 = OMcp24_319+ROcp24_919*qd(20);
    ORcp24_120 = OMcp24_219*RLcp24_320-OMcp24_319*RLcp24_220;
    ORcp24_220 = -(OMcp24_119*RLcp24_320-OMcp24_319*RLcp24_120);
    ORcp24_320 = OMcp24_119*RLcp24_220-OMcp24_219*RLcp24_120;
    OMcp24_121 = OMcp24_120+ROcp24_420*qd(21);
    OMcp24_221 = OMcp24_220+ROcp24_520*qd(21);
    OMcp24_321 = OMcp24_320+ROcp24_620*qd(21);
    OPcp24_121 = OPcp24_119+ROcp24_420*qdd(21)+ROcp24_719*qdd(20)+qd(20)*(OMcp24_219*ROcp24_919-OMcp24_319*ROcp24_819)+qd(21)*(OMcp24_220*...
 ROcp24_620-OMcp24_320*ROcp24_520);
    OPcp24_221 = OPcp24_219+ROcp24_520*qdd(21)+ROcp24_819*qdd(20)-qd(20)*(OMcp24_119*ROcp24_919-OMcp24_319*ROcp24_719)-qd(21)*(OMcp24_120*...
 ROcp24_620-OMcp24_320*ROcp24_420);
    OPcp24_321 = OPcp24_319+ROcp24_620*qdd(21)+ROcp24_919*qdd(20)+qd(20)*(OMcp24_119*ROcp24_819-OMcp24_219*ROcp24_719)+qd(21)*(OMcp24_120*...
 ROcp24_520-OMcp24_220*ROcp24_420);
    RLcp24_122 = ROcp24_721*s.dpt(3,14);
    RLcp24_222 = ROcp24_821*s.dpt(3,14);
    RLcp24_322 = ROcp24_921*s.dpt(3,14);
    OMcp24_122 = OMcp24_121+ROcp24_420*qd(22);
    OMcp24_222 = OMcp24_221+ROcp24_520*qd(22);
    OMcp24_322 = OMcp24_321+ROcp24_620*qd(22);
    ORcp24_122 = OMcp24_221*RLcp24_322-OMcp24_321*RLcp24_222;
    ORcp24_222 = -(OMcp24_121*RLcp24_322-OMcp24_321*RLcp24_122);
    ORcp24_322 = OMcp24_121*RLcp24_222-OMcp24_221*RLcp24_122;
    OPcp24_122 = OPcp24_121+ROcp24_420*qdd(22)+qd(22)*(OMcp24_221*ROcp24_620-OMcp24_321*ROcp24_520);
    OPcp24_222 = OPcp24_221+ROcp24_520*qdd(22)-qd(22)*(OMcp24_121*ROcp24_620-OMcp24_321*ROcp24_420);
    OPcp24_322 = OPcp24_321+ROcp24_620*qdd(22)+qd(22)*(OMcp24_121*ROcp24_520-OMcp24_221*ROcp24_420);
    RLcp24_123 = ROcp24_722*s.dpt(3,15);
    RLcp24_223 = ROcp24_822*s.dpt(3,15);
    RLcp24_323 = ROcp24_922*s.dpt(3,15);
    OMcp24_123 = OMcp24_122+ROcp24_420*qd(23);
    OMcp24_223 = OMcp24_222+ROcp24_520*qd(23);
    OMcp24_323 = OMcp24_322+ROcp24_620*qd(23);
    ORcp24_123 = OMcp24_222*RLcp24_323-OMcp24_322*RLcp24_223;
    ORcp24_223 = -(OMcp24_122*RLcp24_323-OMcp24_322*RLcp24_123);
    ORcp24_323 = OMcp24_122*RLcp24_223-OMcp24_222*RLcp24_123;
    OMcp24_124 = OMcp24_123+ROcp24_123*qd(24);
    OMcp24_224 = OMcp24_223+ROcp24_223*qd(24);
    OMcp24_324 = OMcp24_323+ROcp24_323*qd(24);
    OPcp24_124 = OPcp24_122+ROcp24_123*qdd(24)+ROcp24_420*qdd(23)+qd(23)*(OMcp24_222*ROcp24_620-OMcp24_322*ROcp24_520)+qd(24)*(OMcp24_223*...
 ROcp24_323-OMcp24_323*ROcp24_223);
    OPcp24_224 = OPcp24_222+ROcp24_223*qdd(24)+ROcp24_520*qdd(23)-qd(23)*(OMcp24_122*ROcp24_620-OMcp24_322*ROcp24_420)-qd(24)*(OMcp24_123*...
 ROcp24_323-OMcp24_323*ROcp24_123);
    OPcp24_324 = OPcp24_322+ROcp24_323*qdd(24)+ROcp24_620*qdd(23)+qd(23)*(OMcp24_122*ROcp24_520-OMcp24_222*ROcp24_420)+qd(24)*(OMcp24_123*...
 ROcp24_223-OMcp24_223*ROcp24_123);
    RLcp24_125 = ROcp24_123*s.dpt(1,17)+ROcp24_724*s.dpt(3,17);
    RLcp24_225 = ROcp24_223*s.dpt(1,17)+ROcp24_824*s.dpt(3,17);
    RLcp24_325 = ROcp24_323*s.dpt(1,17)+ROcp24_924*s.dpt(3,17);
    POcp24_125 = RLcp24_119+RLcp24_120+RLcp24_122+RLcp24_123+RLcp24_125+q(1);
    POcp24_225 = RLcp24_219+RLcp24_220+RLcp24_222+RLcp24_223+RLcp24_225+q(2);
    POcp24_325 = RLcp24_319+RLcp24_320+RLcp24_322+RLcp24_323+RLcp24_325+q(3);
    ORcp24_125 = OMcp24_224*RLcp24_325-OMcp24_324*RLcp24_225;
    ORcp24_225 = -(OMcp24_124*RLcp24_325-OMcp24_324*RLcp24_125);
    ORcp24_325 = OMcp24_124*RLcp24_225-OMcp24_224*RLcp24_125;
    VIcp24_125 = ORcp24_119+ORcp24_120+ORcp24_122+ORcp24_123+ORcp24_125+qd(1);
    VIcp24_225 = ORcp24_219+ORcp24_220+ORcp24_222+ORcp24_223+ORcp24_225+qd(2);
    VIcp24_325 = ORcp24_319+ORcp24_320+ORcp24_322+ORcp24_323+ORcp24_325+qd(3);
    ACcp24_125 = qdd(1)+OMcp24_219*ORcp24_320+OMcp24_221*ORcp24_322+OMcp24_222*ORcp24_323+OMcp24_224*ORcp24_325+OMcp24_26*ORcp24_319-OMcp24_319*...
 ORcp24_220-OMcp24_321*ORcp24_222-OMcp24_322*ORcp24_223-OMcp24_324*ORcp24_225-OMcp24_36*ORcp24_219+OPcp24_219*RLcp24_320+OPcp24_221*RLcp24_322+...
 OPcp24_222*RLcp24_323+OPcp24_224*RLcp24_325+OPcp24_26*RLcp24_319-OPcp24_319*RLcp24_220-OPcp24_321*RLcp24_222-OPcp24_322*RLcp24_223-OPcp24_324*...
 RLcp24_225-OPcp24_36*RLcp24_219;
    ACcp24_225 = qdd(2)-OMcp24_119*ORcp24_320-OMcp24_121*ORcp24_322-OMcp24_122*ORcp24_323-OMcp24_124*ORcp24_325-OMcp24_16*ORcp24_319+OMcp24_319*...
 ORcp24_120+OMcp24_321*ORcp24_122+OMcp24_322*ORcp24_123+OMcp24_324*ORcp24_125+OMcp24_36*ORcp24_119-OPcp24_119*RLcp24_320-OPcp24_121*RLcp24_322-...
 OPcp24_122*RLcp24_323-OPcp24_124*RLcp24_325-OPcp24_16*RLcp24_319+OPcp24_319*RLcp24_120+OPcp24_321*RLcp24_122+OPcp24_322*RLcp24_123+OPcp24_324*...
 RLcp24_125+OPcp24_36*RLcp24_119;
    ACcp24_325 = qdd(3)+OMcp24_119*ORcp24_220+OMcp24_121*ORcp24_222+OMcp24_122*ORcp24_223+OMcp24_124*ORcp24_225+OMcp24_16*ORcp24_219-OMcp24_219*...
 ORcp24_120-OMcp24_221*ORcp24_122-OMcp24_222*ORcp24_123-OMcp24_224*ORcp24_125-OMcp24_26*ORcp24_119+OPcp24_119*RLcp24_220+OPcp24_121*RLcp24_222+...
 OPcp24_122*RLcp24_223+OPcp24_124*RLcp24_225+OPcp24_16*RLcp24_219-OPcp24_219*RLcp24_120-OPcp24_221*RLcp24_122-OPcp24_222*RLcp24_123-OPcp24_224*...
 RLcp24_125-OPcp24_26*RLcp24_119;

% = = Block_1_0_0_25_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp24_125;
    sens.P(2) = POcp24_225;
    sens.P(3) = POcp24_325;
    sens.R(1,1) = ROcp24_125;
    sens.R(1,2) = ROcp24_225;
    sens.R(1,3) = ROcp24_325;
    sens.R(2,1) = ROcp24_425;
    sens.R(2,2) = ROcp24_525;
    sens.R(2,3) = ROcp24_625;
    sens.R(3,1) = ROcp24_724;
    sens.R(3,2) = ROcp24_824;
    sens.R(3,3) = ROcp24_924;
    sens.V(1) = VIcp24_125;
    sens.V(2) = VIcp24_225;
    sens.V(3) = VIcp24_325;
    sens.OM(1) = OMcp24_124;
    sens.OM(2) = OMcp24_224;
    sens.OM(3) = OMcp24_324;
    sens.A(1) = ACcp24_125;
    sens.A(2) = ACcp24_225;
    sens.A(3) = ACcp24_325;
    sens.OMP(1) = OPcp24_124;
    sens.OMP(2) = OPcp24_224;
    sens.OMP(3) = OPcp24_324;
 
% 
case 26, 


% = = Block_1_0_0_26_0_1 = = 
 
% Sensor Kinematics 


    ROcp25_25 = S4*S5;
    ROcp25_35 = -C4*S5;
    ROcp25_85 = -S4*C5;
    ROcp25_95 = C4*C5;
    ROcp25_16 = C5*C6;
    ROcp25_26 = ROcp25_25*C6+C4*S6;
    ROcp25_36 = ROcp25_35*C6+S4*S6;
    ROcp25_46 = -C5*S6;
    ROcp25_56 = -(ROcp25_25*S6-C4*C6);
    ROcp25_66 = -(ROcp25_35*S6-S4*C6);
    OMcp25_25 = qd(5)*C4;
    OMcp25_35 = qd(5)*S4;
    OMcp25_16 = qd(4)+qd(6)*S5;
    OMcp25_26 = OMcp25_25+ROcp25_85*qd(6);
    OMcp25_36 = OMcp25_35+ROcp25_95*qd(6);
    OPcp25_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp25_26 = ROcp25_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp25_35*S5-ROcp25_95*qd(4));
    OPcp25_36 = ROcp25_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp25_25*S5-ROcp25_85*qd(4));

% = = Block_1_0_0_26_0_3 = = 
 
% Sensor Kinematics 


    ROcp25_419 = ROcp25_46*C19+S19*S5;
    ROcp25_519 = ROcp25_56*C19+ROcp25_85*S19;
    ROcp25_619 = ROcp25_66*C19+ROcp25_95*S19;
    ROcp25_719 = -(ROcp25_46*S19-C19*S5);
    ROcp25_819 = -(ROcp25_56*S19-ROcp25_85*C19);
    ROcp25_919 = -(ROcp25_66*S19-ROcp25_95*C19);
    ROcp25_120 = ROcp25_16*C20+ROcp25_419*S20;
    ROcp25_220 = ROcp25_26*C20+ROcp25_519*S20;
    ROcp25_320 = ROcp25_36*C20+ROcp25_619*S20;
    ROcp25_420 = -(ROcp25_16*S20-ROcp25_419*C20);
    ROcp25_520 = -(ROcp25_26*S20-ROcp25_519*C20);
    ROcp25_620 = -(ROcp25_36*S20-ROcp25_619*C20);
    ROcp25_121 = ROcp25_120*C21-ROcp25_719*S21;
    ROcp25_221 = ROcp25_220*C21-ROcp25_819*S21;
    ROcp25_321 = ROcp25_320*C21-ROcp25_919*S21;
    ROcp25_721 = ROcp25_120*S21+ROcp25_719*C21;
    ROcp25_821 = ROcp25_220*S21+ROcp25_819*C21;
    ROcp25_921 = ROcp25_320*S21+ROcp25_919*C21;
    ROcp25_122 = ROcp25_121*C22-ROcp25_721*S22;
    ROcp25_222 = ROcp25_221*C22-ROcp25_821*S22;
    ROcp25_322 = ROcp25_321*C22-ROcp25_921*S22;
    ROcp25_722 = ROcp25_121*S22+ROcp25_721*C22;
    ROcp25_822 = ROcp25_221*S22+ROcp25_821*C22;
    ROcp25_922 = ROcp25_321*S22+ROcp25_921*C22;
    ROcp25_123 = ROcp25_122*C23-ROcp25_722*S23;
    ROcp25_223 = ROcp25_222*C23-ROcp25_822*S23;
    ROcp25_323 = ROcp25_322*C23-ROcp25_922*S23;
    ROcp25_723 = ROcp25_122*S23+ROcp25_722*C23;
    ROcp25_823 = ROcp25_222*S23+ROcp25_822*C23;
    ROcp25_923 = ROcp25_322*S23+ROcp25_922*C23;
    ROcp25_424 = ROcp25_420*C24+ROcp25_723*S24;
    ROcp25_524 = ROcp25_520*C24+ROcp25_823*S24;
    ROcp25_624 = ROcp25_620*C24+ROcp25_923*S24;
    ROcp25_724 = -(ROcp25_420*S24-ROcp25_723*C24);
    ROcp25_824 = -(ROcp25_520*S24-ROcp25_823*C24);
    ROcp25_924 = -(ROcp25_620*S24-ROcp25_923*C24);
    ROcp25_125 = ROcp25_123*C25+ROcp25_424*S25;
    ROcp25_225 = ROcp25_223*C25+ROcp25_524*S25;
    ROcp25_325 = ROcp25_323*C25+ROcp25_624*S25;
    ROcp25_425 = -(ROcp25_123*S25-ROcp25_424*C25);
    ROcp25_525 = -(ROcp25_223*S25-ROcp25_524*C25);
    ROcp25_625 = -(ROcp25_323*S25-ROcp25_624*C25);
    ROcp25_126 = ROcp25_125*C26-ROcp25_724*S26;
    ROcp25_226 = ROcp25_225*C26-ROcp25_824*S26;
    ROcp25_326 = ROcp25_325*C26-ROcp25_924*S26;
    ROcp25_726 = ROcp25_125*S26+ROcp25_724*C26;
    ROcp25_826 = ROcp25_225*S26+ROcp25_824*C26;
    ROcp25_926 = ROcp25_325*S26+ROcp25_924*C26;
    RLcp25_119 = ROcp25_16*s.dpt(1,2)+ROcp25_46*s.dpt(2,2);
    RLcp25_219 = ROcp25_26*s.dpt(1,2)+ROcp25_56*s.dpt(2,2);
    RLcp25_319 = ROcp25_36*s.dpt(1,2)+ROcp25_66*s.dpt(2,2);
    OMcp25_119 = OMcp25_16+ROcp25_16*qd(19);
    OMcp25_219 = OMcp25_26+ROcp25_26*qd(19);
    OMcp25_319 = OMcp25_36+ROcp25_36*qd(19);
    ORcp25_119 = OMcp25_26*RLcp25_319-OMcp25_36*RLcp25_219;
    ORcp25_219 = -(OMcp25_16*RLcp25_319-OMcp25_36*RLcp25_119);
    ORcp25_319 = OMcp25_16*RLcp25_219-OMcp25_26*RLcp25_119;
    OPcp25_119 = OPcp25_16+ROcp25_16*qdd(19)+qd(19)*(OMcp25_26*ROcp25_36-OMcp25_36*ROcp25_26);
    OPcp25_219 = OPcp25_26+ROcp25_26*qdd(19)-qd(19)*(OMcp25_16*ROcp25_36-OMcp25_36*ROcp25_16);
    OPcp25_319 = OPcp25_36+ROcp25_36*qdd(19)+qd(19)*(OMcp25_16*ROcp25_26-OMcp25_26*ROcp25_16);
    RLcp25_120 = ROcp25_16*s.dpt(1,12)+ROcp25_419*s.dpt(2,12)+ROcp25_719*s.dpt(3,12);
    RLcp25_220 = ROcp25_26*s.dpt(1,12)+ROcp25_519*s.dpt(2,12)+ROcp25_819*s.dpt(3,12);
    RLcp25_320 = ROcp25_36*s.dpt(1,12)+ROcp25_619*s.dpt(2,12)+ROcp25_919*s.dpt(3,12);
    OMcp25_120 = OMcp25_119+ROcp25_719*qd(20);
    OMcp25_220 = OMcp25_219+ROcp25_819*qd(20);
    OMcp25_320 = OMcp25_319+ROcp25_919*qd(20);
    ORcp25_120 = OMcp25_219*RLcp25_320-OMcp25_319*RLcp25_220;
    ORcp25_220 = -(OMcp25_119*RLcp25_320-OMcp25_319*RLcp25_120);
    ORcp25_320 = OMcp25_119*RLcp25_220-OMcp25_219*RLcp25_120;
    OMcp25_121 = OMcp25_120+ROcp25_420*qd(21);
    OMcp25_221 = OMcp25_220+ROcp25_520*qd(21);
    OMcp25_321 = OMcp25_320+ROcp25_620*qd(21);
    OPcp25_121 = OPcp25_119+ROcp25_420*qdd(21)+ROcp25_719*qdd(20)+qd(20)*(OMcp25_219*ROcp25_919-OMcp25_319*ROcp25_819)+qd(21)*(OMcp25_220*...
 ROcp25_620-OMcp25_320*ROcp25_520);
    OPcp25_221 = OPcp25_219+ROcp25_520*qdd(21)+ROcp25_819*qdd(20)-qd(20)*(OMcp25_119*ROcp25_919-OMcp25_319*ROcp25_719)-qd(21)*(OMcp25_120*...
 ROcp25_620-OMcp25_320*ROcp25_420);
    OPcp25_321 = OPcp25_319+ROcp25_620*qdd(21)+ROcp25_919*qdd(20)+qd(20)*(OMcp25_119*ROcp25_819-OMcp25_219*ROcp25_719)+qd(21)*(OMcp25_120*...
 ROcp25_520-OMcp25_220*ROcp25_420);
    RLcp25_122 = ROcp25_721*s.dpt(3,14);
    RLcp25_222 = ROcp25_821*s.dpt(3,14);
    RLcp25_322 = ROcp25_921*s.dpt(3,14);
    OMcp25_122 = OMcp25_121+ROcp25_420*qd(22);
    OMcp25_222 = OMcp25_221+ROcp25_520*qd(22);
    OMcp25_322 = OMcp25_321+ROcp25_620*qd(22);
    ORcp25_122 = OMcp25_221*RLcp25_322-OMcp25_321*RLcp25_222;
    ORcp25_222 = -(OMcp25_121*RLcp25_322-OMcp25_321*RLcp25_122);
    ORcp25_322 = OMcp25_121*RLcp25_222-OMcp25_221*RLcp25_122;
    OPcp25_122 = OPcp25_121+ROcp25_420*qdd(22)+qd(22)*(OMcp25_221*ROcp25_620-OMcp25_321*ROcp25_520);
    OPcp25_222 = OPcp25_221+ROcp25_520*qdd(22)-qd(22)*(OMcp25_121*ROcp25_620-OMcp25_321*ROcp25_420);
    OPcp25_322 = OPcp25_321+ROcp25_620*qdd(22)+qd(22)*(OMcp25_121*ROcp25_520-OMcp25_221*ROcp25_420);
    RLcp25_123 = ROcp25_722*s.dpt(3,15);
    RLcp25_223 = ROcp25_822*s.dpt(3,15);
    RLcp25_323 = ROcp25_922*s.dpt(3,15);
    OMcp25_123 = OMcp25_122+ROcp25_420*qd(23);
    OMcp25_223 = OMcp25_222+ROcp25_520*qd(23);
    OMcp25_323 = OMcp25_322+ROcp25_620*qd(23);
    ORcp25_123 = OMcp25_222*RLcp25_323-OMcp25_322*RLcp25_223;
    ORcp25_223 = -(OMcp25_122*RLcp25_323-OMcp25_322*RLcp25_123);
    ORcp25_323 = OMcp25_122*RLcp25_223-OMcp25_222*RLcp25_123;
    OMcp25_124 = OMcp25_123+ROcp25_123*qd(24);
    OMcp25_224 = OMcp25_223+ROcp25_223*qd(24);
    OMcp25_324 = OMcp25_323+ROcp25_323*qd(24);
    OPcp25_124 = OPcp25_122+ROcp25_123*qdd(24)+ROcp25_420*qdd(23)+qd(23)*(OMcp25_222*ROcp25_620-OMcp25_322*ROcp25_520)+qd(24)*(OMcp25_223*...
 ROcp25_323-OMcp25_323*ROcp25_223);
    OPcp25_224 = OPcp25_222+ROcp25_223*qdd(24)+ROcp25_520*qdd(23)-qd(23)*(OMcp25_122*ROcp25_620-OMcp25_322*ROcp25_420)-qd(24)*(OMcp25_123*...
 ROcp25_323-OMcp25_323*ROcp25_123);
    OPcp25_324 = OPcp25_322+ROcp25_323*qdd(24)+ROcp25_620*qdd(23)+qd(23)*(OMcp25_122*ROcp25_520-OMcp25_222*ROcp25_420)+qd(24)*(OMcp25_123*...
 ROcp25_223-OMcp25_223*ROcp25_123);
    RLcp25_125 = ROcp25_123*s.dpt(1,17)+ROcp25_724*s.dpt(3,17);
    RLcp25_225 = ROcp25_223*s.dpt(1,17)+ROcp25_824*s.dpt(3,17);
    RLcp25_325 = ROcp25_323*s.dpt(1,17)+ROcp25_924*s.dpt(3,17);
    POcp25_125 = RLcp25_119+RLcp25_120+RLcp25_122+RLcp25_123+RLcp25_125+q(1);
    POcp25_225 = RLcp25_219+RLcp25_220+RLcp25_222+RLcp25_223+RLcp25_225+q(2);
    POcp25_325 = RLcp25_319+RLcp25_320+RLcp25_322+RLcp25_323+RLcp25_325+q(3);
    ORcp25_125 = OMcp25_224*RLcp25_325-OMcp25_324*RLcp25_225;
    ORcp25_225 = -(OMcp25_124*RLcp25_325-OMcp25_324*RLcp25_125);
    ORcp25_325 = OMcp25_124*RLcp25_225-OMcp25_224*RLcp25_125;
    VIcp25_125 = ORcp25_119+ORcp25_120+ORcp25_122+ORcp25_123+ORcp25_125+qd(1);
    VIcp25_225 = ORcp25_219+ORcp25_220+ORcp25_222+ORcp25_223+ORcp25_225+qd(2);
    VIcp25_325 = ORcp25_319+ORcp25_320+ORcp25_322+ORcp25_323+ORcp25_325+qd(3);
    ACcp25_125 = qdd(1)+OMcp25_219*ORcp25_320+OMcp25_221*ORcp25_322+OMcp25_222*ORcp25_323+OMcp25_224*ORcp25_325+OMcp25_26*ORcp25_319-OMcp25_319*...
 ORcp25_220-OMcp25_321*ORcp25_222-OMcp25_322*ORcp25_223-OMcp25_324*ORcp25_225-OMcp25_36*ORcp25_219+OPcp25_219*RLcp25_320+OPcp25_221*RLcp25_322+...
 OPcp25_222*RLcp25_323+OPcp25_224*RLcp25_325+OPcp25_26*RLcp25_319-OPcp25_319*RLcp25_220-OPcp25_321*RLcp25_222-OPcp25_322*RLcp25_223-OPcp25_324*...
 RLcp25_225-OPcp25_36*RLcp25_219;
    ACcp25_225 = qdd(2)-OMcp25_119*ORcp25_320-OMcp25_121*ORcp25_322-OMcp25_122*ORcp25_323-OMcp25_124*ORcp25_325-OMcp25_16*ORcp25_319+OMcp25_319*...
 ORcp25_120+OMcp25_321*ORcp25_122+OMcp25_322*ORcp25_123+OMcp25_324*ORcp25_125+OMcp25_36*ORcp25_119-OPcp25_119*RLcp25_320-OPcp25_121*RLcp25_322-...
 OPcp25_122*RLcp25_323-OPcp25_124*RLcp25_325-OPcp25_16*RLcp25_319+OPcp25_319*RLcp25_120+OPcp25_321*RLcp25_122+OPcp25_322*RLcp25_123+OPcp25_324*...
 RLcp25_125+OPcp25_36*RLcp25_119;
    ACcp25_325 = qdd(3)+OMcp25_119*ORcp25_220+OMcp25_121*ORcp25_222+OMcp25_122*ORcp25_223+OMcp25_124*ORcp25_225+OMcp25_16*ORcp25_219-OMcp25_219*...
 ORcp25_120-OMcp25_221*ORcp25_122-OMcp25_222*ORcp25_123-OMcp25_224*ORcp25_125-OMcp25_26*ORcp25_119+OPcp25_119*RLcp25_220+OPcp25_121*RLcp25_222+...
 OPcp25_122*RLcp25_223+OPcp25_124*RLcp25_225+OPcp25_16*RLcp25_219-OPcp25_219*RLcp25_120-OPcp25_221*RLcp25_122-OPcp25_222*RLcp25_123-OPcp25_224*...
 RLcp25_125-OPcp25_26*RLcp25_119;

% = = Block_1_0_0_26_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp25_125;
    sens.P(2) = POcp25_225;
    sens.P(3) = POcp25_325;
    sens.R(1,1) = ROcp25_126;
    sens.R(1,2) = ROcp25_226;
    sens.R(1,3) = ROcp25_326;
    sens.R(2,1) = ROcp25_425;
    sens.R(2,2) = ROcp25_525;
    sens.R(2,3) = ROcp25_625;
    sens.R(3,1) = ROcp25_726;
    sens.R(3,2) = ROcp25_826;
    sens.R(3,3) = ROcp25_926;
    sens.V(1) = VIcp25_125;
    sens.V(2) = VIcp25_225;
    sens.V(3) = VIcp25_325;
    sens.OM(1) = OMcp25_124;
    sens.OM(2) = OMcp25_224;
    sens.OM(3) = OMcp25_324;
    sens.A(1) = ACcp25_125;
    sens.A(2) = ACcp25_225;
    sens.A(3) = ACcp25_325;
    sens.OMP(1) = OPcp25_124;
    sens.OMP(2) = OPcp25_224;
    sens.OMP(3) = OPcp25_324;
 
% 
case 27, 


% = = Block_1_0_0_27_0_1 = = 
 
% Sensor Kinematics 


    ROcp26_25 = S4*S5;
    ROcp26_35 = -C4*S5;
    ROcp26_85 = -S4*C5;
    ROcp26_95 = C4*C5;
    ROcp26_16 = C5*C6;
    ROcp26_26 = ROcp26_25*C6+C4*S6;
    ROcp26_36 = ROcp26_35*C6+S4*S6;
    ROcp26_46 = -C5*S6;
    ROcp26_56 = -(ROcp26_25*S6-C4*C6);
    ROcp26_66 = -(ROcp26_35*S6-S4*C6);
    OMcp26_25 = qd(5)*C4;
    OMcp26_35 = qd(5)*S4;
    OMcp26_16 = qd(4)+qd(6)*S5;
    OMcp26_26 = OMcp26_25+ROcp26_85*qd(6);
    OMcp26_36 = OMcp26_35+ROcp26_95*qd(6);
    OPcp26_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp26_26 = ROcp26_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp26_35*S5-ROcp26_95*qd(4));
    OPcp26_36 = ROcp26_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp26_25*S5-ROcp26_85*qd(4));

% = = Block_1_0_0_27_0_3 = = 
 
% Sensor Kinematics 


    ROcp26_419 = ROcp26_46*C19+S19*S5;
    ROcp26_519 = ROcp26_56*C19+ROcp26_85*S19;
    ROcp26_619 = ROcp26_66*C19+ROcp26_95*S19;
    ROcp26_719 = -(ROcp26_46*S19-C19*S5);
    ROcp26_819 = -(ROcp26_56*S19-ROcp26_85*C19);
    ROcp26_919 = -(ROcp26_66*S19-ROcp26_95*C19);
    ROcp26_120 = ROcp26_16*C20+ROcp26_419*S20;
    ROcp26_220 = ROcp26_26*C20+ROcp26_519*S20;
    ROcp26_320 = ROcp26_36*C20+ROcp26_619*S20;
    ROcp26_420 = -(ROcp26_16*S20-ROcp26_419*C20);
    ROcp26_520 = -(ROcp26_26*S20-ROcp26_519*C20);
    ROcp26_620 = -(ROcp26_36*S20-ROcp26_619*C20);
    ROcp26_121 = ROcp26_120*C21-ROcp26_719*S21;
    ROcp26_221 = ROcp26_220*C21-ROcp26_819*S21;
    ROcp26_321 = ROcp26_320*C21-ROcp26_919*S21;
    ROcp26_721 = ROcp26_120*S21+ROcp26_719*C21;
    ROcp26_821 = ROcp26_220*S21+ROcp26_819*C21;
    ROcp26_921 = ROcp26_320*S21+ROcp26_919*C21;
    ROcp26_122 = ROcp26_121*C22-ROcp26_721*S22;
    ROcp26_222 = ROcp26_221*C22-ROcp26_821*S22;
    ROcp26_322 = ROcp26_321*C22-ROcp26_921*S22;
    ROcp26_722 = ROcp26_121*S22+ROcp26_721*C22;
    ROcp26_822 = ROcp26_221*S22+ROcp26_821*C22;
    ROcp26_922 = ROcp26_321*S22+ROcp26_921*C22;
    ROcp26_123 = ROcp26_122*C23-ROcp26_722*S23;
    ROcp26_223 = ROcp26_222*C23-ROcp26_822*S23;
    ROcp26_323 = ROcp26_322*C23-ROcp26_922*S23;
    ROcp26_723 = ROcp26_122*S23+ROcp26_722*C23;
    ROcp26_823 = ROcp26_222*S23+ROcp26_822*C23;
    ROcp26_923 = ROcp26_322*S23+ROcp26_922*C23;
    ROcp26_424 = ROcp26_420*C24+ROcp26_723*S24;
    ROcp26_524 = ROcp26_520*C24+ROcp26_823*S24;
    ROcp26_624 = ROcp26_620*C24+ROcp26_923*S24;
    ROcp26_724 = -(ROcp26_420*S24-ROcp26_723*C24);
    ROcp26_824 = -(ROcp26_520*S24-ROcp26_823*C24);
    ROcp26_924 = -(ROcp26_620*S24-ROcp26_923*C24);
    ROcp26_125 = ROcp26_123*C25+ROcp26_424*S25;
    ROcp26_225 = ROcp26_223*C25+ROcp26_524*S25;
    ROcp26_325 = ROcp26_323*C25+ROcp26_624*S25;
    ROcp26_425 = -(ROcp26_123*S25-ROcp26_424*C25);
    ROcp26_525 = -(ROcp26_223*S25-ROcp26_524*C25);
    ROcp26_625 = -(ROcp26_323*S25-ROcp26_624*C25);
    ROcp26_126 = ROcp26_125*C26-ROcp26_724*S26;
    ROcp26_226 = ROcp26_225*C26-ROcp26_824*S26;
    ROcp26_326 = ROcp26_325*C26-ROcp26_924*S26;
    ROcp26_726 = ROcp26_125*S26+ROcp26_724*C26;
    ROcp26_826 = ROcp26_225*S26+ROcp26_824*C26;
    ROcp26_926 = ROcp26_325*S26+ROcp26_924*C26;
    ROcp26_427 = ROcp26_425*C27+ROcp26_726*S27;
    ROcp26_527 = ROcp26_525*C27+ROcp26_826*S27;
    ROcp26_627 = ROcp26_625*C27+ROcp26_926*S27;
    ROcp26_727 = -(ROcp26_425*S27-ROcp26_726*C27);
    ROcp26_827 = -(ROcp26_525*S27-ROcp26_826*C27);
    ROcp26_927 = -(ROcp26_625*S27-ROcp26_926*C27);
    RLcp26_119 = ROcp26_16*s.dpt(1,2)+ROcp26_46*s.dpt(2,2);
    RLcp26_219 = ROcp26_26*s.dpt(1,2)+ROcp26_56*s.dpt(2,2);
    RLcp26_319 = ROcp26_36*s.dpt(1,2)+ROcp26_66*s.dpt(2,2);
    OMcp26_119 = OMcp26_16+ROcp26_16*qd(19);
    OMcp26_219 = OMcp26_26+ROcp26_26*qd(19);
    OMcp26_319 = OMcp26_36+ROcp26_36*qd(19);
    ORcp26_119 = OMcp26_26*RLcp26_319-OMcp26_36*RLcp26_219;
    ORcp26_219 = -(OMcp26_16*RLcp26_319-OMcp26_36*RLcp26_119);
    ORcp26_319 = OMcp26_16*RLcp26_219-OMcp26_26*RLcp26_119;
    OPcp26_119 = OPcp26_16+ROcp26_16*qdd(19)+qd(19)*(OMcp26_26*ROcp26_36-OMcp26_36*ROcp26_26);
    OPcp26_219 = OPcp26_26+ROcp26_26*qdd(19)-qd(19)*(OMcp26_16*ROcp26_36-OMcp26_36*ROcp26_16);
    OPcp26_319 = OPcp26_36+ROcp26_36*qdd(19)+qd(19)*(OMcp26_16*ROcp26_26-OMcp26_26*ROcp26_16);
    RLcp26_120 = ROcp26_16*s.dpt(1,12)+ROcp26_419*s.dpt(2,12)+ROcp26_719*s.dpt(3,12);
    RLcp26_220 = ROcp26_26*s.dpt(1,12)+ROcp26_519*s.dpt(2,12)+ROcp26_819*s.dpt(3,12);
    RLcp26_320 = ROcp26_36*s.dpt(1,12)+ROcp26_619*s.dpt(2,12)+ROcp26_919*s.dpt(3,12);
    OMcp26_120 = OMcp26_119+ROcp26_719*qd(20);
    OMcp26_220 = OMcp26_219+ROcp26_819*qd(20);
    OMcp26_320 = OMcp26_319+ROcp26_919*qd(20);
    ORcp26_120 = OMcp26_219*RLcp26_320-OMcp26_319*RLcp26_220;
    ORcp26_220 = -(OMcp26_119*RLcp26_320-OMcp26_319*RLcp26_120);
    ORcp26_320 = OMcp26_119*RLcp26_220-OMcp26_219*RLcp26_120;
    OMcp26_121 = OMcp26_120+ROcp26_420*qd(21);
    OMcp26_221 = OMcp26_220+ROcp26_520*qd(21);
    OMcp26_321 = OMcp26_320+ROcp26_620*qd(21);
    OPcp26_121 = OPcp26_119+ROcp26_420*qdd(21)+ROcp26_719*qdd(20)+qd(20)*(OMcp26_219*ROcp26_919-OMcp26_319*ROcp26_819)+qd(21)*(OMcp26_220*...
 ROcp26_620-OMcp26_320*ROcp26_520);
    OPcp26_221 = OPcp26_219+ROcp26_520*qdd(21)+ROcp26_819*qdd(20)-qd(20)*(OMcp26_119*ROcp26_919-OMcp26_319*ROcp26_719)-qd(21)*(OMcp26_120*...
 ROcp26_620-OMcp26_320*ROcp26_420);
    OPcp26_321 = OPcp26_319+ROcp26_620*qdd(21)+ROcp26_919*qdd(20)+qd(20)*(OMcp26_119*ROcp26_819-OMcp26_219*ROcp26_719)+qd(21)*(OMcp26_120*...
 ROcp26_520-OMcp26_220*ROcp26_420);
    RLcp26_122 = ROcp26_721*s.dpt(3,14);
    RLcp26_222 = ROcp26_821*s.dpt(3,14);
    RLcp26_322 = ROcp26_921*s.dpt(3,14);
    OMcp26_122 = OMcp26_121+ROcp26_420*qd(22);
    OMcp26_222 = OMcp26_221+ROcp26_520*qd(22);
    OMcp26_322 = OMcp26_321+ROcp26_620*qd(22);
    ORcp26_122 = OMcp26_221*RLcp26_322-OMcp26_321*RLcp26_222;
    ORcp26_222 = -(OMcp26_121*RLcp26_322-OMcp26_321*RLcp26_122);
    ORcp26_322 = OMcp26_121*RLcp26_222-OMcp26_221*RLcp26_122;
    OPcp26_122 = OPcp26_121+ROcp26_420*qdd(22)+qd(22)*(OMcp26_221*ROcp26_620-OMcp26_321*ROcp26_520);
    OPcp26_222 = OPcp26_221+ROcp26_520*qdd(22)-qd(22)*(OMcp26_121*ROcp26_620-OMcp26_321*ROcp26_420);
    OPcp26_322 = OPcp26_321+ROcp26_620*qdd(22)+qd(22)*(OMcp26_121*ROcp26_520-OMcp26_221*ROcp26_420);
    RLcp26_123 = ROcp26_722*s.dpt(3,15);
    RLcp26_223 = ROcp26_822*s.dpt(3,15);
    RLcp26_323 = ROcp26_922*s.dpt(3,15);
    OMcp26_123 = OMcp26_122+ROcp26_420*qd(23);
    OMcp26_223 = OMcp26_222+ROcp26_520*qd(23);
    OMcp26_323 = OMcp26_322+ROcp26_620*qd(23);
    ORcp26_123 = OMcp26_222*RLcp26_323-OMcp26_322*RLcp26_223;
    ORcp26_223 = -(OMcp26_122*RLcp26_323-OMcp26_322*RLcp26_123);
    ORcp26_323 = OMcp26_122*RLcp26_223-OMcp26_222*RLcp26_123;
    OMcp26_124 = OMcp26_123+ROcp26_123*qd(24);
    OMcp26_224 = OMcp26_223+ROcp26_223*qd(24);
    OMcp26_324 = OMcp26_323+ROcp26_323*qd(24);
    OPcp26_124 = OPcp26_122+ROcp26_123*qdd(24)+ROcp26_420*qdd(23)+qd(23)*(OMcp26_222*ROcp26_620-OMcp26_322*ROcp26_520)+qd(24)*(OMcp26_223*...
 ROcp26_323-OMcp26_323*ROcp26_223);
    OPcp26_224 = OPcp26_222+ROcp26_223*qdd(24)+ROcp26_520*qdd(23)-qd(23)*(OMcp26_122*ROcp26_620-OMcp26_322*ROcp26_420)-qd(24)*(OMcp26_123*...
 ROcp26_323-OMcp26_323*ROcp26_123);
    OPcp26_324 = OPcp26_322+ROcp26_323*qdd(24)+ROcp26_620*qdd(23)+qd(23)*(OMcp26_122*ROcp26_520-OMcp26_222*ROcp26_420)+qd(24)*(OMcp26_123*...
 ROcp26_223-OMcp26_223*ROcp26_123);
    RLcp26_125 = ROcp26_123*s.dpt(1,17)+ROcp26_724*s.dpt(3,17);
    RLcp26_225 = ROcp26_223*s.dpt(1,17)+ROcp26_824*s.dpt(3,17);
    RLcp26_325 = ROcp26_323*s.dpt(1,17)+ROcp26_924*s.dpt(3,17);
    POcp26_125 = RLcp26_119+RLcp26_120+RLcp26_122+RLcp26_123+RLcp26_125+q(1);
    POcp26_225 = RLcp26_219+RLcp26_220+RLcp26_222+RLcp26_223+RLcp26_225+q(2);
    POcp26_325 = RLcp26_319+RLcp26_320+RLcp26_322+RLcp26_323+RLcp26_325+q(3);
    ORcp26_125 = OMcp26_224*RLcp26_325-OMcp26_324*RLcp26_225;
    ORcp26_225 = -(OMcp26_124*RLcp26_325-OMcp26_324*RLcp26_125);
    ORcp26_325 = OMcp26_124*RLcp26_225-OMcp26_224*RLcp26_125;
    VIcp26_125 = ORcp26_119+ORcp26_120+ORcp26_122+ORcp26_123+ORcp26_125+qd(1);
    VIcp26_225 = ORcp26_219+ORcp26_220+ORcp26_222+ORcp26_223+ORcp26_225+qd(2);
    VIcp26_325 = ORcp26_319+ORcp26_320+ORcp26_322+ORcp26_323+ORcp26_325+qd(3);
    ACcp26_125 = qdd(1)+OMcp26_219*ORcp26_320+OMcp26_221*ORcp26_322+OMcp26_222*ORcp26_323+OMcp26_224*ORcp26_325+OMcp26_26*ORcp26_319-OMcp26_319*...
 ORcp26_220-OMcp26_321*ORcp26_222-OMcp26_322*ORcp26_223-OMcp26_324*ORcp26_225-OMcp26_36*ORcp26_219+OPcp26_219*RLcp26_320+OPcp26_221*RLcp26_322+...
 OPcp26_222*RLcp26_323+OPcp26_224*RLcp26_325+OPcp26_26*RLcp26_319-OPcp26_319*RLcp26_220-OPcp26_321*RLcp26_222-OPcp26_322*RLcp26_223-OPcp26_324*...
 RLcp26_225-OPcp26_36*RLcp26_219;
    ACcp26_225 = qdd(2)-OMcp26_119*ORcp26_320-OMcp26_121*ORcp26_322-OMcp26_122*ORcp26_323-OMcp26_124*ORcp26_325-OMcp26_16*ORcp26_319+OMcp26_319*...
 ORcp26_120+OMcp26_321*ORcp26_122+OMcp26_322*ORcp26_123+OMcp26_324*ORcp26_125+OMcp26_36*ORcp26_119-OPcp26_119*RLcp26_320-OPcp26_121*RLcp26_322-...
 OPcp26_122*RLcp26_323-OPcp26_124*RLcp26_325-OPcp26_16*RLcp26_319+OPcp26_319*RLcp26_120+OPcp26_321*RLcp26_122+OPcp26_322*RLcp26_123+OPcp26_324*...
 RLcp26_125+OPcp26_36*RLcp26_119;
    ACcp26_325 = qdd(3)+OMcp26_119*ORcp26_220+OMcp26_121*ORcp26_222+OMcp26_122*ORcp26_223+OMcp26_124*ORcp26_225+OMcp26_16*ORcp26_219-OMcp26_219*...
 ORcp26_120-OMcp26_221*ORcp26_122-OMcp26_222*ORcp26_123-OMcp26_224*ORcp26_125-OMcp26_26*ORcp26_119+OPcp26_119*RLcp26_220+OPcp26_121*RLcp26_222+...
 OPcp26_122*RLcp26_223+OPcp26_124*RLcp26_225+OPcp26_16*RLcp26_219-OPcp26_219*RLcp26_120-OPcp26_221*RLcp26_122-OPcp26_222*RLcp26_123-OPcp26_224*...
 RLcp26_125-OPcp26_26*RLcp26_119;

% = = Block_1_0_0_27_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp26_125;
    sens.P(2) = POcp26_225;
    sens.P(3) = POcp26_325;
    sens.R(1,1) = ROcp26_126;
    sens.R(1,2) = ROcp26_226;
    sens.R(1,3) = ROcp26_326;
    sens.R(2,1) = ROcp26_427;
    sens.R(2,2) = ROcp26_527;
    sens.R(2,3) = ROcp26_627;
    sens.R(3,1) = ROcp26_727;
    sens.R(3,2) = ROcp26_827;
    sens.R(3,3) = ROcp26_927;
    sens.V(1) = VIcp26_125;
    sens.V(2) = VIcp26_225;
    sens.V(3) = VIcp26_325;
    sens.OM(1) = OMcp26_124;
    sens.OM(2) = OMcp26_224;
    sens.OM(3) = OMcp26_324;
    sens.A(1) = ACcp26_125;
    sens.A(2) = ACcp26_225;
    sens.A(3) = ACcp26_325;
    sens.OMP(1) = OPcp26_124;
    sens.OMP(2) = OPcp26_224;
    sens.OMP(3) = OPcp26_324;
 
% 
case 28, 


% = = Block_1_0_0_28_0_1 = = 
 
% Sensor Kinematics 


    ROcp27_25 = S4*S5;
    ROcp27_35 = -C4*S5;
    ROcp27_85 = -S4*C5;
    ROcp27_95 = C4*C5;
    ROcp27_16 = C5*C6;
    ROcp27_26 = ROcp27_25*C6+C4*S6;
    ROcp27_36 = ROcp27_35*C6+S4*S6;
    ROcp27_46 = -C5*S6;
    ROcp27_56 = -(ROcp27_25*S6-C4*C6);
    ROcp27_66 = -(ROcp27_35*S6-S4*C6);
    OMcp27_25 = qd(5)*C4;
    OMcp27_35 = qd(5)*S4;
    OMcp27_16 = qd(4)+qd(6)*S5;
    OMcp27_26 = OMcp27_25+ROcp27_85*qd(6);
    OMcp27_36 = OMcp27_35+ROcp27_95*qd(6);
    OPcp27_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp27_26 = ROcp27_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp27_35*S5-ROcp27_95*qd(4));
    OPcp27_36 = ROcp27_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp27_25*S5-ROcp27_85*qd(4));

% = = Block_1_0_0_28_0_3 = = 
 
% Sensor Kinematics 


    ROcp27_419 = ROcp27_46*C19+S19*S5;
    ROcp27_519 = ROcp27_56*C19+ROcp27_85*S19;
    ROcp27_619 = ROcp27_66*C19+ROcp27_95*S19;
    ROcp27_719 = -(ROcp27_46*S19-C19*S5);
    ROcp27_819 = -(ROcp27_56*S19-ROcp27_85*C19);
    ROcp27_919 = -(ROcp27_66*S19-ROcp27_95*C19);
    ROcp27_120 = ROcp27_16*C20+ROcp27_419*S20;
    ROcp27_220 = ROcp27_26*C20+ROcp27_519*S20;
    ROcp27_320 = ROcp27_36*C20+ROcp27_619*S20;
    ROcp27_420 = -(ROcp27_16*S20-ROcp27_419*C20);
    ROcp27_520 = -(ROcp27_26*S20-ROcp27_519*C20);
    ROcp27_620 = -(ROcp27_36*S20-ROcp27_619*C20);
    ROcp27_121 = ROcp27_120*C21-ROcp27_719*S21;
    ROcp27_221 = ROcp27_220*C21-ROcp27_819*S21;
    ROcp27_321 = ROcp27_320*C21-ROcp27_919*S21;
    ROcp27_721 = ROcp27_120*S21+ROcp27_719*C21;
    ROcp27_821 = ROcp27_220*S21+ROcp27_819*C21;
    ROcp27_921 = ROcp27_320*S21+ROcp27_919*C21;
    ROcp27_122 = ROcp27_121*C22-ROcp27_721*S22;
    ROcp27_222 = ROcp27_221*C22-ROcp27_821*S22;
    ROcp27_322 = ROcp27_321*C22-ROcp27_921*S22;
    ROcp27_722 = ROcp27_121*S22+ROcp27_721*C22;
    ROcp27_822 = ROcp27_221*S22+ROcp27_821*C22;
    ROcp27_922 = ROcp27_321*S22+ROcp27_921*C22;
    ROcp27_123 = ROcp27_122*C23-ROcp27_722*S23;
    ROcp27_223 = ROcp27_222*C23-ROcp27_822*S23;
    ROcp27_323 = ROcp27_322*C23-ROcp27_922*S23;
    ROcp27_723 = ROcp27_122*S23+ROcp27_722*C23;
    ROcp27_823 = ROcp27_222*S23+ROcp27_822*C23;
    ROcp27_923 = ROcp27_322*S23+ROcp27_922*C23;
    ROcp27_424 = ROcp27_420*C24+ROcp27_723*S24;
    ROcp27_524 = ROcp27_520*C24+ROcp27_823*S24;
    ROcp27_624 = ROcp27_620*C24+ROcp27_923*S24;
    ROcp27_724 = -(ROcp27_420*S24-ROcp27_723*C24);
    ROcp27_824 = -(ROcp27_520*S24-ROcp27_823*C24);
    ROcp27_924 = -(ROcp27_620*S24-ROcp27_923*C24);
    ROcp27_125 = ROcp27_123*C25+ROcp27_424*S25;
    ROcp27_225 = ROcp27_223*C25+ROcp27_524*S25;
    ROcp27_325 = ROcp27_323*C25+ROcp27_624*S25;
    ROcp27_425 = -(ROcp27_123*S25-ROcp27_424*C25);
    ROcp27_525 = -(ROcp27_223*S25-ROcp27_524*C25);
    ROcp27_625 = -(ROcp27_323*S25-ROcp27_624*C25);
    ROcp27_126 = ROcp27_125*C26-ROcp27_724*S26;
    ROcp27_226 = ROcp27_225*C26-ROcp27_824*S26;
    ROcp27_326 = ROcp27_325*C26-ROcp27_924*S26;
    ROcp27_726 = ROcp27_125*S26+ROcp27_724*C26;
    ROcp27_826 = ROcp27_225*S26+ROcp27_824*C26;
    ROcp27_926 = ROcp27_325*S26+ROcp27_924*C26;
    ROcp27_427 = ROcp27_425*C27+ROcp27_726*S27;
    ROcp27_527 = ROcp27_525*C27+ROcp27_826*S27;
    ROcp27_627 = ROcp27_625*C27+ROcp27_926*S27;
    ROcp27_727 = -(ROcp27_425*S27-ROcp27_726*C27);
    ROcp27_827 = -(ROcp27_525*S27-ROcp27_826*C27);
    ROcp27_927 = -(ROcp27_625*S27-ROcp27_926*C27);
    RLcp27_119 = ROcp27_16*s.dpt(1,2)+ROcp27_46*s.dpt(2,2);
    RLcp27_219 = ROcp27_26*s.dpt(1,2)+ROcp27_56*s.dpt(2,2);
    RLcp27_319 = ROcp27_36*s.dpt(1,2)+ROcp27_66*s.dpt(2,2);
    OMcp27_119 = OMcp27_16+ROcp27_16*qd(19);
    OMcp27_219 = OMcp27_26+ROcp27_26*qd(19);
    OMcp27_319 = OMcp27_36+ROcp27_36*qd(19);
    ORcp27_119 = OMcp27_26*RLcp27_319-OMcp27_36*RLcp27_219;
    ORcp27_219 = -(OMcp27_16*RLcp27_319-OMcp27_36*RLcp27_119);
    ORcp27_319 = OMcp27_16*RLcp27_219-OMcp27_26*RLcp27_119;
    OPcp27_119 = OPcp27_16+ROcp27_16*qdd(19)+qd(19)*(OMcp27_26*ROcp27_36-OMcp27_36*ROcp27_26);
    OPcp27_219 = OPcp27_26+ROcp27_26*qdd(19)-qd(19)*(OMcp27_16*ROcp27_36-OMcp27_36*ROcp27_16);
    OPcp27_319 = OPcp27_36+ROcp27_36*qdd(19)+qd(19)*(OMcp27_16*ROcp27_26-OMcp27_26*ROcp27_16);
    RLcp27_120 = ROcp27_16*s.dpt(1,12)+ROcp27_419*s.dpt(2,12)+ROcp27_719*s.dpt(3,12);
    RLcp27_220 = ROcp27_26*s.dpt(1,12)+ROcp27_519*s.dpt(2,12)+ROcp27_819*s.dpt(3,12);
    RLcp27_320 = ROcp27_36*s.dpt(1,12)+ROcp27_619*s.dpt(2,12)+ROcp27_919*s.dpt(3,12);
    OMcp27_120 = OMcp27_119+ROcp27_719*qd(20);
    OMcp27_220 = OMcp27_219+ROcp27_819*qd(20);
    OMcp27_320 = OMcp27_319+ROcp27_919*qd(20);
    ORcp27_120 = OMcp27_219*RLcp27_320-OMcp27_319*RLcp27_220;
    ORcp27_220 = -(OMcp27_119*RLcp27_320-OMcp27_319*RLcp27_120);
    ORcp27_320 = OMcp27_119*RLcp27_220-OMcp27_219*RLcp27_120;
    OMcp27_121 = OMcp27_120+ROcp27_420*qd(21);
    OMcp27_221 = OMcp27_220+ROcp27_520*qd(21);
    OMcp27_321 = OMcp27_320+ROcp27_620*qd(21);
    OPcp27_121 = OPcp27_119+ROcp27_420*qdd(21)+ROcp27_719*qdd(20)+qd(20)*(OMcp27_219*ROcp27_919-OMcp27_319*ROcp27_819)+qd(21)*(OMcp27_220*...
 ROcp27_620-OMcp27_320*ROcp27_520);
    OPcp27_221 = OPcp27_219+ROcp27_520*qdd(21)+ROcp27_819*qdd(20)-qd(20)*(OMcp27_119*ROcp27_919-OMcp27_319*ROcp27_719)-qd(21)*(OMcp27_120*...
 ROcp27_620-OMcp27_320*ROcp27_420);
    OPcp27_321 = OPcp27_319+ROcp27_620*qdd(21)+ROcp27_919*qdd(20)+qd(20)*(OMcp27_119*ROcp27_819-OMcp27_219*ROcp27_719)+qd(21)*(OMcp27_120*...
 ROcp27_520-OMcp27_220*ROcp27_420);
    RLcp27_122 = ROcp27_721*s.dpt(3,14);
    RLcp27_222 = ROcp27_821*s.dpt(3,14);
    RLcp27_322 = ROcp27_921*s.dpt(3,14);
    OMcp27_122 = OMcp27_121+ROcp27_420*qd(22);
    OMcp27_222 = OMcp27_221+ROcp27_520*qd(22);
    OMcp27_322 = OMcp27_321+ROcp27_620*qd(22);
    ORcp27_122 = OMcp27_221*RLcp27_322-OMcp27_321*RLcp27_222;
    ORcp27_222 = -(OMcp27_121*RLcp27_322-OMcp27_321*RLcp27_122);
    ORcp27_322 = OMcp27_121*RLcp27_222-OMcp27_221*RLcp27_122;
    OPcp27_122 = OPcp27_121+ROcp27_420*qdd(22)+qd(22)*(OMcp27_221*ROcp27_620-OMcp27_321*ROcp27_520);
    OPcp27_222 = OPcp27_221+ROcp27_520*qdd(22)-qd(22)*(OMcp27_121*ROcp27_620-OMcp27_321*ROcp27_420);
    OPcp27_322 = OPcp27_321+ROcp27_620*qdd(22)+qd(22)*(OMcp27_121*ROcp27_520-OMcp27_221*ROcp27_420);
    RLcp27_123 = ROcp27_722*s.dpt(3,15);
    RLcp27_223 = ROcp27_822*s.dpt(3,15);
    RLcp27_323 = ROcp27_922*s.dpt(3,15);
    OMcp27_123 = OMcp27_122+ROcp27_420*qd(23);
    OMcp27_223 = OMcp27_222+ROcp27_520*qd(23);
    OMcp27_323 = OMcp27_322+ROcp27_620*qd(23);
    ORcp27_123 = OMcp27_222*RLcp27_323-OMcp27_322*RLcp27_223;
    ORcp27_223 = -(OMcp27_122*RLcp27_323-OMcp27_322*RLcp27_123);
    ORcp27_323 = OMcp27_122*RLcp27_223-OMcp27_222*RLcp27_123;
    OMcp27_124 = OMcp27_123+ROcp27_123*qd(24);
    OMcp27_224 = OMcp27_223+ROcp27_223*qd(24);
    OMcp27_324 = OMcp27_323+ROcp27_323*qd(24);
    OPcp27_124 = OPcp27_122+ROcp27_123*qdd(24)+ROcp27_420*qdd(23)+qd(23)*(OMcp27_222*ROcp27_620-OMcp27_322*ROcp27_520)+qd(24)*(OMcp27_223*...
 ROcp27_323-OMcp27_323*ROcp27_223);
    OPcp27_224 = OPcp27_222+ROcp27_223*qdd(24)+ROcp27_520*qdd(23)-qd(23)*(OMcp27_122*ROcp27_620-OMcp27_322*ROcp27_420)-qd(24)*(OMcp27_123*...
 ROcp27_323-OMcp27_323*ROcp27_123);
    OPcp27_324 = OPcp27_322+ROcp27_323*qdd(24)+ROcp27_620*qdd(23)+qd(23)*(OMcp27_122*ROcp27_520-OMcp27_222*ROcp27_420)+qd(24)*(OMcp27_123*...
 ROcp27_223-OMcp27_223*ROcp27_123);
    RLcp27_125 = ROcp27_123*s.dpt(1,17)+ROcp27_724*s.dpt(3,17);
    RLcp27_225 = ROcp27_223*s.dpt(1,17)+ROcp27_824*s.dpt(3,17);
    RLcp27_325 = ROcp27_323*s.dpt(1,17)+ROcp27_924*s.dpt(3,17);
    ORcp27_125 = OMcp27_224*RLcp27_325-OMcp27_324*RLcp27_225;
    ORcp27_225 = -(OMcp27_124*RLcp27_325-OMcp27_324*RLcp27_125);
    ORcp27_325 = OMcp27_124*RLcp27_225-OMcp27_224*RLcp27_125;
    RLcp27_128 = ROcp27_727*q(28);
    RLcp27_228 = ROcp27_827*q(28);
    RLcp27_328 = ROcp27_927*q(28);
    POcp27_128 = RLcp27_119+RLcp27_120+RLcp27_122+RLcp27_123+RLcp27_125+RLcp27_128+q(1);
    POcp27_228 = RLcp27_219+RLcp27_220+RLcp27_222+RLcp27_223+RLcp27_225+RLcp27_228+q(2);
    POcp27_328 = RLcp27_319+RLcp27_320+RLcp27_322+RLcp27_323+RLcp27_325+RLcp27_328+q(3);
    ORcp27_128 = OMcp27_224*RLcp27_328-OMcp27_324*RLcp27_228;
    ORcp27_228 = -(OMcp27_124*RLcp27_328-OMcp27_324*RLcp27_128);
    ORcp27_328 = OMcp27_124*RLcp27_228-OMcp27_224*RLcp27_128;
    VIcp27_128 = ORcp27_119+ORcp27_120+ORcp27_122+ORcp27_123+ORcp27_125+ORcp27_128+qd(1);
    VIcp27_228 = ORcp27_219+ORcp27_220+ORcp27_222+ORcp27_223+ORcp27_225+ORcp27_228+qd(2);
    VIcp27_328 = ORcp27_319+ORcp27_320+ORcp27_322+ORcp27_323+ORcp27_325+ORcp27_328+qd(3);
    ACcp27_128 = qdd(1)+OMcp27_219*ORcp27_320+OMcp27_221*ORcp27_322+OMcp27_222*ORcp27_323+OMcp27_224*ORcp27_325+OMcp27_224*ORcp27_328+OMcp27_26*...
 ORcp27_319-OMcp27_319*ORcp27_220-OMcp27_321*ORcp27_222-OMcp27_322*ORcp27_223-OMcp27_324*ORcp27_225-OMcp27_324*ORcp27_228-OMcp27_36*ORcp27_219+...
 OPcp27_219*RLcp27_320+OPcp27_221*RLcp27_322+OPcp27_222*RLcp27_323+OPcp27_224*RLcp27_325+OPcp27_224*RLcp27_328+OPcp27_26*RLcp27_319-OPcp27_319*...
 RLcp27_220-OPcp27_321*RLcp27_222-OPcp27_322*RLcp27_223-OPcp27_324*RLcp27_225-OPcp27_324*RLcp27_228-OPcp27_36*RLcp27_219;
    ACcp27_228 = qdd(2)-OMcp27_119*ORcp27_320-OMcp27_121*ORcp27_322-OMcp27_122*ORcp27_323-OMcp27_124*ORcp27_325-OMcp27_124*ORcp27_328-OMcp27_16*...
 ORcp27_319+OMcp27_319*ORcp27_120+OMcp27_321*ORcp27_122+OMcp27_322*ORcp27_123+OMcp27_324*ORcp27_125+OMcp27_324*ORcp27_128+OMcp27_36*ORcp27_119-...
 OPcp27_119*RLcp27_320-OPcp27_121*RLcp27_322-OPcp27_122*RLcp27_323-OPcp27_124*RLcp27_325-OPcp27_124*RLcp27_328-OPcp27_16*RLcp27_319+OPcp27_319*...
 RLcp27_120+OPcp27_321*RLcp27_122+OPcp27_322*RLcp27_123+OPcp27_324*RLcp27_125+OPcp27_324*RLcp27_128+OPcp27_36*RLcp27_119;
    ACcp27_328 = qdd(3)+OMcp27_119*ORcp27_220+OMcp27_121*ORcp27_222+OMcp27_122*ORcp27_223+OMcp27_124*ORcp27_225+OMcp27_124*ORcp27_228+OMcp27_16*...
 ORcp27_219-OMcp27_219*ORcp27_120-OMcp27_221*ORcp27_122-OMcp27_222*ORcp27_123-OMcp27_224*ORcp27_125-OMcp27_224*ORcp27_128-OMcp27_26*ORcp27_119+...
 OPcp27_119*RLcp27_220+OPcp27_121*RLcp27_222+OPcp27_122*RLcp27_223+OPcp27_124*RLcp27_225+OPcp27_124*RLcp27_228+OPcp27_16*RLcp27_219-OPcp27_219*...
 RLcp27_120-OPcp27_221*RLcp27_122-OPcp27_222*RLcp27_123-OPcp27_224*RLcp27_125-OPcp27_224*RLcp27_128-OPcp27_26*RLcp27_119;

% = = Block_1_0_0_28_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp27_128;
    sens.P(2) = POcp27_228;
    sens.P(3) = POcp27_328;
    sens.R(1,1) = ROcp27_126;
    sens.R(1,2) = ROcp27_226;
    sens.R(1,3) = ROcp27_326;
    sens.R(2,1) = ROcp27_427;
    sens.R(2,2) = ROcp27_527;
    sens.R(2,3) = ROcp27_627;
    sens.R(3,1) = ROcp27_727;
    sens.R(3,2) = ROcp27_827;
    sens.R(3,3) = ROcp27_927;
    sens.V(1) = VIcp27_128;
    sens.V(2) = VIcp27_228;
    sens.V(3) = VIcp27_328;
    sens.OM(1) = OMcp27_124;
    sens.OM(2) = OMcp27_224;
    sens.OM(3) = OMcp27_324;
    sens.A(1) = ACcp27_128;
    sens.A(2) = ACcp27_228;
    sens.A(3) = ACcp27_328;
    sens.OMP(1) = OPcp27_124;
    sens.OMP(2) = OPcp27_224;
    sens.OMP(3) = OPcp27_324;
 
% 
case 29, 


% = = Block_1_0_0_29_0_1 = = 
 
% Sensor Kinematics 


    ROcp28_25 = S4*S5;
    ROcp28_35 = -C4*S5;
    ROcp28_85 = -S4*C5;
    ROcp28_95 = C4*C5;
    ROcp28_16 = C5*C6;
    ROcp28_26 = ROcp28_25*C6+C4*S6;
    ROcp28_36 = ROcp28_35*C6+S4*S6;
    ROcp28_46 = -C5*S6;
    ROcp28_56 = -(ROcp28_25*S6-C4*C6);
    ROcp28_66 = -(ROcp28_35*S6-S4*C6);
    OMcp28_25 = qd(5)*C4;
    OMcp28_35 = qd(5)*S4;
    OMcp28_16 = qd(4)+qd(6)*S5;
    OMcp28_26 = OMcp28_25+ROcp28_85*qd(6);
    OMcp28_36 = OMcp28_35+ROcp28_95*qd(6);
    OPcp28_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp28_26 = ROcp28_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp28_35*S5-ROcp28_95*qd(4));
    OPcp28_36 = ROcp28_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp28_25*S5-ROcp28_85*qd(4));

% = = Block_1_0_0_29_0_3 = = 
 
% Sensor Kinematics 


    ROcp28_419 = ROcp28_46*C19+S19*S5;
    ROcp28_519 = ROcp28_56*C19+ROcp28_85*S19;
    ROcp28_619 = ROcp28_66*C19+ROcp28_95*S19;
    ROcp28_719 = -(ROcp28_46*S19-C19*S5);
    ROcp28_819 = -(ROcp28_56*S19-ROcp28_85*C19);
    ROcp28_919 = -(ROcp28_66*S19-ROcp28_95*C19);
    ROcp28_120 = ROcp28_16*C20+ROcp28_419*S20;
    ROcp28_220 = ROcp28_26*C20+ROcp28_519*S20;
    ROcp28_320 = ROcp28_36*C20+ROcp28_619*S20;
    ROcp28_420 = -(ROcp28_16*S20-ROcp28_419*C20);
    ROcp28_520 = -(ROcp28_26*S20-ROcp28_519*C20);
    ROcp28_620 = -(ROcp28_36*S20-ROcp28_619*C20);
    ROcp28_121 = ROcp28_120*C21-ROcp28_719*S21;
    ROcp28_221 = ROcp28_220*C21-ROcp28_819*S21;
    ROcp28_321 = ROcp28_320*C21-ROcp28_919*S21;
    ROcp28_721 = ROcp28_120*S21+ROcp28_719*C21;
    ROcp28_821 = ROcp28_220*S21+ROcp28_819*C21;
    ROcp28_921 = ROcp28_320*S21+ROcp28_919*C21;
    ROcp28_122 = ROcp28_121*C22-ROcp28_721*S22;
    ROcp28_222 = ROcp28_221*C22-ROcp28_821*S22;
    ROcp28_322 = ROcp28_321*C22-ROcp28_921*S22;
    ROcp28_722 = ROcp28_121*S22+ROcp28_721*C22;
    ROcp28_822 = ROcp28_221*S22+ROcp28_821*C22;
    ROcp28_922 = ROcp28_321*S22+ROcp28_921*C22;
    ROcp28_123 = ROcp28_122*C23-ROcp28_722*S23;
    ROcp28_223 = ROcp28_222*C23-ROcp28_822*S23;
    ROcp28_323 = ROcp28_322*C23-ROcp28_922*S23;
    ROcp28_723 = ROcp28_122*S23+ROcp28_722*C23;
    ROcp28_823 = ROcp28_222*S23+ROcp28_822*C23;
    ROcp28_923 = ROcp28_322*S23+ROcp28_922*C23;
    ROcp28_424 = ROcp28_420*C24+ROcp28_723*S24;
    ROcp28_524 = ROcp28_520*C24+ROcp28_823*S24;
    ROcp28_624 = ROcp28_620*C24+ROcp28_923*S24;
    ROcp28_724 = -(ROcp28_420*S24-ROcp28_723*C24);
    ROcp28_824 = -(ROcp28_520*S24-ROcp28_823*C24);
    ROcp28_924 = -(ROcp28_620*S24-ROcp28_923*C24);
    ROcp28_125 = ROcp28_123*C25+ROcp28_424*S25;
    ROcp28_225 = ROcp28_223*C25+ROcp28_524*S25;
    ROcp28_325 = ROcp28_323*C25+ROcp28_624*S25;
    ROcp28_425 = -(ROcp28_123*S25-ROcp28_424*C25);
    ROcp28_525 = -(ROcp28_223*S25-ROcp28_524*C25);
    ROcp28_625 = -(ROcp28_323*S25-ROcp28_624*C25);
    ROcp28_126 = ROcp28_125*C26-ROcp28_724*S26;
    ROcp28_226 = ROcp28_225*C26-ROcp28_824*S26;
    ROcp28_326 = ROcp28_325*C26-ROcp28_924*S26;
    ROcp28_726 = ROcp28_125*S26+ROcp28_724*C26;
    ROcp28_826 = ROcp28_225*S26+ROcp28_824*C26;
    ROcp28_926 = ROcp28_325*S26+ROcp28_924*C26;
    ROcp28_427 = ROcp28_425*C27+ROcp28_726*S27;
    ROcp28_527 = ROcp28_525*C27+ROcp28_826*S27;
    ROcp28_627 = ROcp28_625*C27+ROcp28_926*S27;
    ROcp28_727 = -(ROcp28_425*S27-ROcp28_726*C27);
    ROcp28_827 = -(ROcp28_525*S27-ROcp28_826*C27);
    ROcp28_927 = -(ROcp28_625*S27-ROcp28_926*C27);
    RLcp28_119 = ROcp28_16*s.dpt(1,2)+ROcp28_46*s.dpt(2,2);
    RLcp28_219 = ROcp28_26*s.dpt(1,2)+ROcp28_56*s.dpt(2,2);
    RLcp28_319 = ROcp28_36*s.dpt(1,2)+ROcp28_66*s.dpt(2,2);
    OMcp28_119 = OMcp28_16+ROcp28_16*qd(19);
    OMcp28_219 = OMcp28_26+ROcp28_26*qd(19);
    OMcp28_319 = OMcp28_36+ROcp28_36*qd(19);
    ORcp28_119 = OMcp28_26*RLcp28_319-OMcp28_36*RLcp28_219;
    ORcp28_219 = -(OMcp28_16*RLcp28_319-OMcp28_36*RLcp28_119);
    ORcp28_319 = OMcp28_16*RLcp28_219-OMcp28_26*RLcp28_119;
    OPcp28_119 = OPcp28_16+ROcp28_16*qdd(19)+qd(19)*(OMcp28_26*ROcp28_36-OMcp28_36*ROcp28_26);
    OPcp28_219 = OPcp28_26+ROcp28_26*qdd(19)-qd(19)*(OMcp28_16*ROcp28_36-OMcp28_36*ROcp28_16);
    OPcp28_319 = OPcp28_36+ROcp28_36*qdd(19)+qd(19)*(OMcp28_16*ROcp28_26-OMcp28_26*ROcp28_16);
    RLcp28_120 = ROcp28_16*s.dpt(1,12)+ROcp28_419*s.dpt(2,12)+ROcp28_719*s.dpt(3,12);
    RLcp28_220 = ROcp28_26*s.dpt(1,12)+ROcp28_519*s.dpt(2,12)+ROcp28_819*s.dpt(3,12);
    RLcp28_320 = ROcp28_36*s.dpt(1,12)+ROcp28_619*s.dpt(2,12)+ROcp28_919*s.dpt(3,12);
    OMcp28_120 = OMcp28_119+ROcp28_719*qd(20);
    OMcp28_220 = OMcp28_219+ROcp28_819*qd(20);
    OMcp28_320 = OMcp28_319+ROcp28_919*qd(20);
    ORcp28_120 = OMcp28_219*RLcp28_320-OMcp28_319*RLcp28_220;
    ORcp28_220 = -(OMcp28_119*RLcp28_320-OMcp28_319*RLcp28_120);
    ORcp28_320 = OMcp28_119*RLcp28_220-OMcp28_219*RLcp28_120;
    OMcp28_121 = OMcp28_120+ROcp28_420*qd(21);
    OMcp28_221 = OMcp28_220+ROcp28_520*qd(21);
    OMcp28_321 = OMcp28_320+ROcp28_620*qd(21);
    OPcp28_121 = OPcp28_119+ROcp28_420*qdd(21)+ROcp28_719*qdd(20)+qd(20)*(OMcp28_219*ROcp28_919-OMcp28_319*ROcp28_819)+qd(21)*(OMcp28_220*...
 ROcp28_620-OMcp28_320*ROcp28_520);
    OPcp28_221 = OPcp28_219+ROcp28_520*qdd(21)+ROcp28_819*qdd(20)-qd(20)*(OMcp28_119*ROcp28_919-OMcp28_319*ROcp28_719)-qd(21)*(OMcp28_120*...
 ROcp28_620-OMcp28_320*ROcp28_420);
    OPcp28_321 = OPcp28_319+ROcp28_620*qdd(21)+ROcp28_919*qdd(20)+qd(20)*(OMcp28_119*ROcp28_819-OMcp28_219*ROcp28_719)+qd(21)*(OMcp28_120*...
 ROcp28_520-OMcp28_220*ROcp28_420);
    RLcp28_122 = ROcp28_721*s.dpt(3,14);
    RLcp28_222 = ROcp28_821*s.dpt(3,14);
    RLcp28_322 = ROcp28_921*s.dpt(3,14);
    OMcp28_122 = OMcp28_121+ROcp28_420*qd(22);
    OMcp28_222 = OMcp28_221+ROcp28_520*qd(22);
    OMcp28_322 = OMcp28_321+ROcp28_620*qd(22);
    ORcp28_122 = OMcp28_221*RLcp28_322-OMcp28_321*RLcp28_222;
    ORcp28_222 = -(OMcp28_121*RLcp28_322-OMcp28_321*RLcp28_122);
    ORcp28_322 = OMcp28_121*RLcp28_222-OMcp28_221*RLcp28_122;
    OPcp28_122 = OPcp28_121+ROcp28_420*qdd(22)+qd(22)*(OMcp28_221*ROcp28_620-OMcp28_321*ROcp28_520);
    OPcp28_222 = OPcp28_221+ROcp28_520*qdd(22)-qd(22)*(OMcp28_121*ROcp28_620-OMcp28_321*ROcp28_420);
    OPcp28_322 = OPcp28_321+ROcp28_620*qdd(22)+qd(22)*(OMcp28_121*ROcp28_520-OMcp28_221*ROcp28_420);
    RLcp28_123 = ROcp28_722*s.dpt(3,15);
    RLcp28_223 = ROcp28_822*s.dpt(3,15);
    RLcp28_323 = ROcp28_922*s.dpt(3,15);
    OMcp28_123 = OMcp28_122+ROcp28_420*qd(23);
    OMcp28_223 = OMcp28_222+ROcp28_520*qd(23);
    OMcp28_323 = OMcp28_322+ROcp28_620*qd(23);
    ORcp28_123 = OMcp28_222*RLcp28_323-OMcp28_322*RLcp28_223;
    ORcp28_223 = -(OMcp28_122*RLcp28_323-OMcp28_322*RLcp28_123);
    ORcp28_323 = OMcp28_122*RLcp28_223-OMcp28_222*RLcp28_123;
    OMcp28_124 = OMcp28_123+ROcp28_123*qd(24);
    OMcp28_224 = OMcp28_223+ROcp28_223*qd(24);
    OMcp28_324 = OMcp28_323+ROcp28_323*qd(24);
    OPcp28_124 = OPcp28_122+ROcp28_123*qdd(24)+ROcp28_420*qdd(23)+qd(23)*(OMcp28_222*ROcp28_620-OMcp28_322*ROcp28_520)+qd(24)*(OMcp28_223*...
 ROcp28_323-OMcp28_323*ROcp28_223);
    OPcp28_224 = OPcp28_222+ROcp28_223*qdd(24)+ROcp28_520*qdd(23)-qd(23)*(OMcp28_122*ROcp28_620-OMcp28_322*ROcp28_420)-qd(24)*(OMcp28_123*...
 ROcp28_323-OMcp28_323*ROcp28_123);
    OPcp28_324 = OPcp28_322+ROcp28_323*qdd(24)+ROcp28_620*qdd(23)+qd(23)*(OMcp28_122*ROcp28_520-OMcp28_222*ROcp28_420)+qd(24)*(OMcp28_123*...
 ROcp28_223-OMcp28_223*ROcp28_123);
    RLcp28_125 = ROcp28_123*s.dpt(1,17)+ROcp28_724*s.dpt(3,17);
    RLcp28_225 = ROcp28_223*s.dpt(1,17)+ROcp28_824*s.dpt(3,17);
    RLcp28_325 = ROcp28_323*s.dpt(1,17)+ROcp28_924*s.dpt(3,17);
    ORcp28_125 = OMcp28_224*RLcp28_325-OMcp28_324*RLcp28_225;
    ORcp28_225 = -(OMcp28_124*RLcp28_325-OMcp28_324*RLcp28_125);
    ORcp28_325 = OMcp28_124*RLcp28_225-OMcp28_224*RLcp28_125;
    RLcp28_128 = ROcp28_727*q(28);
    RLcp28_228 = ROcp28_827*q(28);
    RLcp28_328 = ROcp28_927*q(28);
    ORcp28_128 = OMcp28_224*RLcp28_328-OMcp28_324*RLcp28_228;
    ORcp28_228 = -(OMcp28_124*RLcp28_328-OMcp28_324*RLcp28_128);
    ORcp28_328 = OMcp28_124*RLcp28_228-OMcp28_224*RLcp28_128;
    RLcp28_129 = ROcp28_427*q(29);
    RLcp28_229 = ROcp28_527*q(29);
    RLcp28_329 = ROcp28_627*q(29);
    POcp28_129 = RLcp28_119+RLcp28_120+RLcp28_122+RLcp28_123+RLcp28_125+RLcp28_128+RLcp28_129+q(1);
    POcp28_229 = RLcp28_219+RLcp28_220+RLcp28_222+RLcp28_223+RLcp28_225+RLcp28_228+RLcp28_229+q(2);
    POcp28_329 = RLcp28_319+RLcp28_320+RLcp28_322+RLcp28_323+RLcp28_325+RLcp28_328+RLcp28_329+q(3);
    ORcp28_129 = OMcp28_224*RLcp28_329-OMcp28_324*RLcp28_229;
    ORcp28_229 = -(OMcp28_124*RLcp28_329-OMcp28_324*RLcp28_129);
    ORcp28_329 = OMcp28_124*RLcp28_229-OMcp28_224*RLcp28_129;
    VIcp28_129 = ORcp28_119+ORcp28_120+ORcp28_122+ORcp28_123+ORcp28_125+ORcp28_128+ORcp28_129+qd(1);
    VIcp28_229 = ORcp28_219+ORcp28_220+ORcp28_222+ORcp28_223+ORcp28_225+ORcp28_228+ORcp28_229+qd(2);
    VIcp28_329 = ORcp28_319+ORcp28_320+ORcp28_322+ORcp28_323+ORcp28_325+ORcp28_328+ORcp28_329+qd(3);
    ACcp28_129 = qdd(1)+OMcp28_219*ORcp28_320+OMcp28_221*ORcp28_322+OMcp28_222*ORcp28_323+OMcp28_224*ORcp28_325+OMcp28_224*ORcp28_328+OMcp28_224*...
 ORcp28_329+OMcp28_26*ORcp28_319-OMcp28_319*ORcp28_220-OMcp28_321*ORcp28_222-OMcp28_322*ORcp28_223-OMcp28_324*ORcp28_225-OMcp28_324*ORcp28_228-...
 OMcp28_324*ORcp28_229-OMcp28_36*ORcp28_219+OPcp28_219*RLcp28_320+OPcp28_221*RLcp28_322+OPcp28_222*RLcp28_323+OPcp28_224*RLcp28_325+OPcp28_224*...
 RLcp28_328+OPcp28_224*RLcp28_329+OPcp28_26*RLcp28_319-OPcp28_319*RLcp28_220-OPcp28_321*RLcp28_222-OPcp28_322*RLcp28_223-OPcp28_324*RLcp28_225-...
 OPcp28_324*RLcp28_228-OPcp28_324*RLcp28_229-OPcp28_36*RLcp28_219;
    ACcp28_229 = qdd(2)-OMcp28_119*ORcp28_320-OMcp28_121*ORcp28_322-OMcp28_122*ORcp28_323-OMcp28_124*ORcp28_325-OMcp28_124*ORcp28_328-OMcp28_124*...
 ORcp28_329-OMcp28_16*ORcp28_319+OMcp28_319*ORcp28_120+OMcp28_321*ORcp28_122+OMcp28_322*ORcp28_123+OMcp28_324*ORcp28_125+OMcp28_324*ORcp28_128+...
 OMcp28_324*ORcp28_129+OMcp28_36*ORcp28_119-OPcp28_119*RLcp28_320-OPcp28_121*RLcp28_322-OPcp28_122*RLcp28_323-OPcp28_124*RLcp28_325-OPcp28_124*...
 RLcp28_328-OPcp28_124*RLcp28_329-OPcp28_16*RLcp28_319+OPcp28_319*RLcp28_120+OPcp28_321*RLcp28_122+OPcp28_322*RLcp28_123+OPcp28_324*RLcp28_125+...
 OPcp28_324*RLcp28_128+OPcp28_324*RLcp28_129+OPcp28_36*RLcp28_119;
    ACcp28_329 = qdd(3)+OMcp28_119*ORcp28_220+OMcp28_121*ORcp28_222+OMcp28_122*ORcp28_223+OMcp28_124*ORcp28_225+OMcp28_124*ORcp28_228+OMcp28_124*...
 ORcp28_229+OMcp28_16*ORcp28_219-OMcp28_219*ORcp28_120-OMcp28_221*ORcp28_122-OMcp28_222*ORcp28_123-OMcp28_224*ORcp28_125-OMcp28_224*ORcp28_128-...
 OMcp28_224*ORcp28_129-OMcp28_26*ORcp28_119+OPcp28_119*RLcp28_220+OPcp28_121*RLcp28_222+OPcp28_122*RLcp28_223+OPcp28_124*RLcp28_225+OPcp28_124*...
 RLcp28_228+OPcp28_124*RLcp28_229+OPcp28_16*RLcp28_219-OPcp28_219*RLcp28_120-OPcp28_221*RLcp28_122-OPcp28_222*RLcp28_123-OPcp28_224*RLcp28_125-...
 OPcp28_224*RLcp28_128-OPcp28_224*RLcp28_129-OPcp28_26*RLcp28_119;

% = = Block_1_0_0_29_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp28_129;
    sens.P(2) = POcp28_229;
    sens.P(3) = POcp28_329;
    sens.R(1,1) = ROcp28_126;
    sens.R(1,2) = ROcp28_226;
    sens.R(1,3) = ROcp28_326;
    sens.R(2,1) = ROcp28_427;
    sens.R(2,2) = ROcp28_527;
    sens.R(2,3) = ROcp28_627;
    sens.R(3,1) = ROcp28_727;
    sens.R(3,2) = ROcp28_827;
    sens.R(3,3) = ROcp28_927;
    sens.V(1) = VIcp28_129;
    sens.V(2) = VIcp28_229;
    sens.V(3) = VIcp28_329;
    sens.OM(1) = OMcp28_124;
    sens.OM(2) = OMcp28_224;
    sens.OM(3) = OMcp28_324;
    sens.A(1) = ACcp28_129;
    sens.A(2) = ACcp28_229;
    sens.A(3) = ACcp28_329;
    sens.OMP(1) = OPcp28_124;
    sens.OMP(2) = OPcp28_224;
    sens.OMP(3) = OPcp28_324;
 
% 
case 30, 


% = = Block_1_0_0_30_0_1 = = 
 
% Sensor Kinematics 


    ROcp29_25 = S4*S5;
    ROcp29_35 = -C4*S5;
    ROcp29_85 = -S4*C5;
    ROcp29_95 = C4*C5;
    ROcp29_16 = C5*C6;
    ROcp29_26 = ROcp29_25*C6+C4*S6;
    ROcp29_36 = ROcp29_35*C6+S4*S6;
    ROcp29_46 = -C5*S6;
    ROcp29_56 = -(ROcp29_25*S6-C4*C6);
    ROcp29_66 = -(ROcp29_35*S6-S4*C6);
    OMcp29_25 = qd(5)*C4;
    OMcp29_35 = qd(5)*S4;
    OMcp29_16 = qd(4)+qd(6)*S5;
    OMcp29_26 = OMcp29_25+ROcp29_85*qd(6);
    OMcp29_36 = OMcp29_35+ROcp29_95*qd(6);
    OPcp29_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp29_26 = ROcp29_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp29_35*S5-ROcp29_95*qd(4));
    OPcp29_36 = ROcp29_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp29_25*S5-ROcp29_85*qd(4));

% = = Block_1_0_0_30_0_3 = = 
 
% Sensor Kinematics 


    ROcp29_419 = ROcp29_46*C19+S19*S5;
    ROcp29_519 = ROcp29_56*C19+ROcp29_85*S19;
    ROcp29_619 = ROcp29_66*C19+ROcp29_95*S19;
    ROcp29_719 = -(ROcp29_46*S19-C19*S5);
    ROcp29_819 = -(ROcp29_56*S19-ROcp29_85*C19);
    ROcp29_919 = -(ROcp29_66*S19-ROcp29_95*C19);
    ROcp29_120 = ROcp29_16*C20+ROcp29_419*S20;
    ROcp29_220 = ROcp29_26*C20+ROcp29_519*S20;
    ROcp29_320 = ROcp29_36*C20+ROcp29_619*S20;
    ROcp29_420 = -(ROcp29_16*S20-ROcp29_419*C20);
    ROcp29_520 = -(ROcp29_26*S20-ROcp29_519*C20);
    ROcp29_620 = -(ROcp29_36*S20-ROcp29_619*C20);
    ROcp29_121 = ROcp29_120*C21-ROcp29_719*S21;
    ROcp29_221 = ROcp29_220*C21-ROcp29_819*S21;
    ROcp29_321 = ROcp29_320*C21-ROcp29_919*S21;
    ROcp29_721 = ROcp29_120*S21+ROcp29_719*C21;
    ROcp29_821 = ROcp29_220*S21+ROcp29_819*C21;
    ROcp29_921 = ROcp29_320*S21+ROcp29_919*C21;
    ROcp29_122 = ROcp29_121*C22-ROcp29_721*S22;
    ROcp29_222 = ROcp29_221*C22-ROcp29_821*S22;
    ROcp29_322 = ROcp29_321*C22-ROcp29_921*S22;
    ROcp29_722 = ROcp29_121*S22+ROcp29_721*C22;
    ROcp29_822 = ROcp29_221*S22+ROcp29_821*C22;
    ROcp29_922 = ROcp29_321*S22+ROcp29_921*C22;
    ROcp29_123 = ROcp29_122*C23-ROcp29_722*S23;
    ROcp29_223 = ROcp29_222*C23-ROcp29_822*S23;
    ROcp29_323 = ROcp29_322*C23-ROcp29_922*S23;
    ROcp29_723 = ROcp29_122*S23+ROcp29_722*C23;
    ROcp29_823 = ROcp29_222*S23+ROcp29_822*C23;
    ROcp29_923 = ROcp29_322*S23+ROcp29_922*C23;
    ROcp29_424 = ROcp29_420*C24+ROcp29_723*S24;
    ROcp29_524 = ROcp29_520*C24+ROcp29_823*S24;
    ROcp29_624 = ROcp29_620*C24+ROcp29_923*S24;
    ROcp29_724 = -(ROcp29_420*S24-ROcp29_723*C24);
    ROcp29_824 = -(ROcp29_520*S24-ROcp29_823*C24);
    ROcp29_924 = -(ROcp29_620*S24-ROcp29_923*C24);
    ROcp29_125 = ROcp29_123*C25+ROcp29_424*S25;
    ROcp29_225 = ROcp29_223*C25+ROcp29_524*S25;
    ROcp29_325 = ROcp29_323*C25+ROcp29_624*S25;
    ROcp29_425 = -(ROcp29_123*S25-ROcp29_424*C25);
    ROcp29_525 = -(ROcp29_223*S25-ROcp29_524*C25);
    ROcp29_625 = -(ROcp29_323*S25-ROcp29_624*C25);
    ROcp29_126 = ROcp29_125*C26-ROcp29_724*S26;
    ROcp29_226 = ROcp29_225*C26-ROcp29_824*S26;
    ROcp29_326 = ROcp29_325*C26-ROcp29_924*S26;
    ROcp29_726 = ROcp29_125*S26+ROcp29_724*C26;
    ROcp29_826 = ROcp29_225*S26+ROcp29_824*C26;
    ROcp29_926 = ROcp29_325*S26+ROcp29_924*C26;
    ROcp29_427 = ROcp29_425*C27+ROcp29_726*S27;
    ROcp29_527 = ROcp29_525*C27+ROcp29_826*S27;
    ROcp29_627 = ROcp29_625*C27+ROcp29_926*S27;
    ROcp29_727 = -(ROcp29_425*S27-ROcp29_726*C27);
    ROcp29_827 = -(ROcp29_525*S27-ROcp29_826*C27);
    ROcp29_927 = -(ROcp29_625*S27-ROcp29_926*C27);
    RLcp29_119 = ROcp29_16*s.dpt(1,2)+ROcp29_46*s.dpt(2,2);
    RLcp29_219 = ROcp29_26*s.dpt(1,2)+ROcp29_56*s.dpt(2,2);
    RLcp29_319 = ROcp29_36*s.dpt(1,2)+ROcp29_66*s.dpt(2,2);
    OMcp29_119 = OMcp29_16+ROcp29_16*qd(19);
    OMcp29_219 = OMcp29_26+ROcp29_26*qd(19);
    OMcp29_319 = OMcp29_36+ROcp29_36*qd(19);
    ORcp29_119 = OMcp29_26*RLcp29_319-OMcp29_36*RLcp29_219;
    ORcp29_219 = -(OMcp29_16*RLcp29_319-OMcp29_36*RLcp29_119);
    ORcp29_319 = OMcp29_16*RLcp29_219-OMcp29_26*RLcp29_119;
    OPcp29_119 = OPcp29_16+ROcp29_16*qdd(19)+qd(19)*(OMcp29_26*ROcp29_36-OMcp29_36*ROcp29_26);
    OPcp29_219 = OPcp29_26+ROcp29_26*qdd(19)-qd(19)*(OMcp29_16*ROcp29_36-OMcp29_36*ROcp29_16);
    OPcp29_319 = OPcp29_36+ROcp29_36*qdd(19)+qd(19)*(OMcp29_16*ROcp29_26-OMcp29_26*ROcp29_16);
    RLcp29_120 = ROcp29_16*s.dpt(1,12)+ROcp29_419*s.dpt(2,12)+ROcp29_719*s.dpt(3,12);
    RLcp29_220 = ROcp29_26*s.dpt(1,12)+ROcp29_519*s.dpt(2,12)+ROcp29_819*s.dpt(3,12);
    RLcp29_320 = ROcp29_36*s.dpt(1,12)+ROcp29_619*s.dpt(2,12)+ROcp29_919*s.dpt(3,12);
    OMcp29_120 = OMcp29_119+ROcp29_719*qd(20);
    OMcp29_220 = OMcp29_219+ROcp29_819*qd(20);
    OMcp29_320 = OMcp29_319+ROcp29_919*qd(20);
    ORcp29_120 = OMcp29_219*RLcp29_320-OMcp29_319*RLcp29_220;
    ORcp29_220 = -(OMcp29_119*RLcp29_320-OMcp29_319*RLcp29_120);
    ORcp29_320 = OMcp29_119*RLcp29_220-OMcp29_219*RLcp29_120;
    OMcp29_121 = OMcp29_120+ROcp29_420*qd(21);
    OMcp29_221 = OMcp29_220+ROcp29_520*qd(21);
    OMcp29_321 = OMcp29_320+ROcp29_620*qd(21);
    OPcp29_121 = OPcp29_119+ROcp29_420*qdd(21)+ROcp29_719*qdd(20)+qd(20)*(OMcp29_219*ROcp29_919-OMcp29_319*ROcp29_819)+qd(21)*(OMcp29_220*...
 ROcp29_620-OMcp29_320*ROcp29_520);
    OPcp29_221 = OPcp29_219+ROcp29_520*qdd(21)+ROcp29_819*qdd(20)-qd(20)*(OMcp29_119*ROcp29_919-OMcp29_319*ROcp29_719)-qd(21)*(OMcp29_120*...
 ROcp29_620-OMcp29_320*ROcp29_420);
    OPcp29_321 = OPcp29_319+ROcp29_620*qdd(21)+ROcp29_919*qdd(20)+qd(20)*(OMcp29_119*ROcp29_819-OMcp29_219*ROcp29_719)+qd(21)*(OMcp29_120*...
 ROcp29_520-OMcp29_220*ROcp29_420);
    RLcp29_122 = ROcp29_721*s.dpt(3,14);
    RLcp29_222 = ROcp29_821*s.dpt(3,14);
    RLcp29_322 = ROcp29_921*s.dpt(3,14);
    OMcp29_122 = OMcp29_121+ROcp29_420*qd(22);
    OMcp29_222 = OMcp29_221+ROcp29_520*qd(22);
    OMcp29_322 = OMcp29_321+ROcp29_620*qd(22);
    ORcp29_122 = OMcp29_221*RLcp29_322-OMcp29_321*RLcp29_222;
    ORcp29_222 = -(OMcp29_121*RLcp29_322-OMcp29_321*RLcp29_122);
    ORcp29_322 = OMcp29_121*RLcp29_222-OMcp29_221*RLcp29_122;
    OPcp29_122 = OPcp29_121+ROcp29_420*qdd(22)+qd(22)*(OMcp29_221*ROcp29_620-OMcp29_321*ROcp29_520);
    OPcp29_222 = OPcp29_221+ROcp29_520*qdd(22)-qd(22)*(OMcp29_121*ROcp29_620-OMcp29_321*ROcp29_420);
    OPcp29_322 = OPcp29_321+ROcp29_620*qdd(22)+qd(22)*(OMcp29_121*ROcp29_520-OMcp29_221*ROcp29_420);
    RLcp29_123 = ROcp29_722*s.dpt(3,15);
    RLcp29_223 = ROcp29_822*s.dpt(3,15);
    RLcp29_323 = ROcp29_922*s.dpt(3,15);
    OMcp29_123 = OMcp29_122+ROcp29_420*qd(23);
    OMcp29_223 = OMcp29_222+ROcp29_520*qd(23);
    OMcp29_323 = OMcp29_322+ROcp29_620*qd(23);
    ORcp29_123 = OMcp29_222*RLcp29_323-OMcp29_322*RLcp29_223;
    ORcp29_223 = -(OMcp29_122*RLcp29_323-OMcp29_322*RLcp29_123);
    ORcp29_323 = OMcp29_122*RLcp29_223-OMcp29_222*RLcp29_123;
    OMcp29_124 = OMcp29_123+ROcp29_123*qd(24);
    OMcp29_224 = OMcp29_223+ROcp29_223*qd(24);
    OMcp29_324 = OMcp29_323+ROcp29_323*qd(24);
    OPcp29_124 = OPcp29_122+ROcp29_123*qdd(24)+ROcp29_420*qdd(23)+qd(23)*(OMcp29_222*ROcp29_620-OMcp29_322*ROcp29_520)+qd(24)*(OMcp29_223*...
 ROcp29_323-OMcp29_323*ROcp29_223);
    OPcp29_224 = OPcp29_222+ROcp29_223*qdd(24)+ROcp29_520*qdd(23)-qd(23)*(OMcp29_122*ROcp29_620-OMcp29_322*ROcp29_420)-qd(24)*(OMcp29_123*...
 ROcp29_323-OMcp29_323*ROcp29_123);
    OPcp29_324 = OPcp29_322+ROcp29_323*qdd(24)+ROcp29_620*qdd(23)+qd(23)*(OMcp29_122*ROcp29_520-OMcp29_222*ROcp29_420)+qd(24)*(OMcp29_123*...
 ROcp29_223-OMcp29_223*ROcp29_123);
    RLcp29_125 = ROcp29_123*s.dpt(1,17)+ROcp29_724*s.dpt(3,17);
    RLcp29_225 = ROcp29_223*s.dpt(1,17)+ROcp29_824*s.dpt(3,17);
    RLcp29_325 = ROcp29_323*s.dpt(1,17)+ROcp29_924*s.dpt(3,17);
    ORcp29_125 = OMcp29_224*RLcp29_325-OMcp29_324*RLcp29_225;
    ORcp29_225 = -(OMcp29_124*RLcp29_325-OMcp29_324*RLcp29_125);
    ORcp29_325 = OMcp29_124*RLcp29_225-OMcp29_224*RLcp29_125;
    RLcp29_128 = ROcp29_727*q(28);
    RLcp29_228 = ROcp29_827*q(28);
    RLcp29_328 = ROcp29_927*q(28);
    ORcp29_128 = OMcp29_224*RLcp29_328-OMcp29_324*RLcp29_228;
    ORcp29_228 = -(OMcp29_124*RLcp29_328-OMcp29_324*RLcp29_128);
    ORcp29_328 = OMcp29_124*RLcp29_228-OMcp29_224*RLcp29_128;
    RLcp29_129 = ROcp29_427*q(29);
    RLcp29_229 = ROcp29_527*q(29);
    RLcp29_329 = ROcp29_627*q(29);
    ORcp29_129 = OMcp29_224*RLcp29_329-OMcp29_324*RLcp29_229;
    ORcp29_229 = -(OMcp29_124*RLcp29_329-OMcp29_324*RLcp29_129);
    ORcp29_329 = OMcp29_124*RLcp29_229-OMcp29_224*RLcp29_129;
    RLcp29_130 = ROcp29_126*q(30);
    RLcp29_230 = ROcp29_226*q(30);
    RLcp29_330 = ROcp29_326*q(30);
    POcp29_130 = RLcp29_119+RLcp29_120+RLcp29_122+RLcp29_123+RLcp29_125+RLcp29_128+RLcp29_129+RLcp29_130+q(1);
    POcp29_230 = RLcp29_219+RLcp29_220+RLcp29_222+RLcp29_223+RLcp29_225+RLcp29_228+RLcp29_229+RLcp29_230+q(2);
    POcp29_330 = RLcp29_319+RLcp29_320+RLcp29_322+RLcp29_323+RLcp29_325+RLcp29_328+RLcp29_329+RLcp29_330+q(3);
    ORcp29_130 = OMcp29_224*RLcp29_330-OMcp29_324*RLcp29_230;
    ORcp29_230 = -(OMcp29_124*RLcp29_330-OMcp29_324*RLcp29_130);
    ORcp29_330 = OMcp29_124*RLcp29_230-OMcp29_224*RLcp29_130;
    VIcp29_130 = ORcp29_119+ORcp29_120+ORcp29_122+ORcp29_123+ORcp29_125+ORcp29_128+ORcp29_129+ORcp29_130+qd(1);
    VIcp29_230 = ORcp29_219+ORcp29_220+ORcp29_222+ORcp29_223+ORcp29_225+ORcp29_228+ORcp29_229+ORcp29_230+qd(2);
    VIcp29_330 = ORcp29_319+ORcp29_320+ORcp29_322+ORcp29_323+ORcp29_325+ORcp29_328+ORcp29_329+ORcp29_330+qd(3);
    ACcp29_130 = qdd(1)+OMcp29_219*ORcp29_320+OMcp29_221*ORcp29_322+OMcp29_222*ORcp29_323+OMcp29_224*ORcp29_325+OMcp29_224*ORcp29_328+OMcp29_224*...
 ORcp29_329+OMcp29_224*ORcp29_330+OMcp29_26*ORcp29_319-OMcp29_319*ORcp29_220-OMcp29_321*ORcp29_222-OMcp29_322*ORcp29_223-OMcp29_324*ORcp29_225-...
 OMcp29_324*ORcp29_228-OMcp29_324*ORcp29_229-OMcp29_324*ORcp29_230-OMcp29_36*ORcp29_219+OPcp29_219*RLcp29_320+OPcp29_221*RLcp29_322+OPcp29_222*...
 RLcp29_323+OPcp29_224*RLcp29_325+OPcp29_224*RLcp29_328+OPcp29_224*RLcp29_329+OPcp29_224*RLcp29_330+OPcp29_26*RLcp29_319-OPcp29_319*RLcp29_220-...
 OPcp29_321*RLcp29_222-OPcp29_322*RLcp29_223-OPcp29_324*RLcp29_225-OPcp29_324*RLcp29_228-OPcp29_324*RLcp29_229-OPcp29_324*RLcp29_230-OPcp29_36*...
 RLcp29_219;
    ACcp29_230 = qdd(2)-OMcp29_119*ORcp29_320-OMcp29_121*ORcp29_322-OMcp29_122*ORcp29_323-OMcp29_124*ORcp29_325-OMcp29_124*ORcp29_328-OMcp29_124*...
 ORcp29_329-OMcp29_124*ORcp29_330-OMcp29_16*ORcp29_319+OMcp29_319*ORcp29_120+OMcp29_321*ORcp29_122+OMcp29_322*ORcp29_123+OMcp29_324*ORcp29_125+...
 OMcp29_324*ORcp29_128+OMcp29_324*ORcp29_129+OMcp29_324*ORcp29_130+OMcp29_36*ORcp29_119-OPcp29_119*RLcp29_320-OPcp29_121*RLcp29_322-OPcp29_122*...
 RLcp29_323-OPcp29_124*RLcp29_325-OPcp29_124*RLcp29_328-OPcp29_124*RLcp29_329-OPcp29_124*RLcp29_330-OPcp29_16*RLcp29_319+OPcp29_319*RLcp29_120+...
 OPcp29_321*RLcp29_122+OPcp29_322*RLcp29_123+OPcp29_324*RLcp29_125+OPcp29_324*RLcp29_128+OPcp29_324*RLcp29_129+OPcp29_324*RLcp29_130+OPcp29_36*...
 RLcp29_119;
    ACcp29_330 = qdd(3)+OMcp29_119*ORcp29_220+OMcp29_121*ORcp29_222+OMcp29_122*ORcp29_223+OMcp29_124*ORcp29_225+OMcp29_124*ORcp29_228+OMcp29_124*...
 ORcp29_229+OMcp29_124*ORcp29_230+OMcp29_16*ORcp29_219-OMcp29_219*ORcp29_120-OMcp29_221*ORcp29_122-OMcp29_222*ORcp29_123-OMcp29_224*ORcp29_125-...
 OMcp29_224*ORcp29_128-OMcp29_224*ORcp29_129-OMcp29_224*ORcp29_130-OMcp29_26*ORcp29_119+OPcp29_119*RLcp29_220+OPcp29_121*RLcp29_222+OPcp29_122*...
 RLcp29_223+OPcp29_124*RLcp29_225+OPcp29_124*RLcp29_228+OPcp29_124*RLcp29_229+OPcp29_124*RLcp29_230+OPcp29_16*RLcp29_219-OPcp29_219*RLcp29_120-...
 OPcp29_221*RLcp29_122-OPcp29_222*RLcp29_123-OPcp29_224*RLcp29_125-OPcp29_224*RLcp29_128-OPcp29_224*RLcp29_129-OPcp29_224*RLcp29_130-OPcp29_26*...
 RLcp29_119;

% = = Block_1_0_0_30_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp29_130;
    sens.P(2) = POcp29_230;
    sens.P(3) = POcp29_330;
    sens.R(1,1) = ROcp29_126;
    sens.R(1,2) = ROcp29_226;
    sens.R(1,3) = ROcp29_326;
    sens.R(2,1) = ROcp29_427;
    sens.R(2,2) = ROcp29_527;
    sens.R(2,3) = ROcp29_627;
    sens.R(3,1) = ROcp29_727;
    sens.R(3,2) = ROcp29_827;
    sens.R(3,3) = ROcp29_927;
    sens.V(1) = VIcp29_130;
    sens.V(2) = VIcp29_230;
    sens.V(3) = VIcp29_330;
    sens.OM(1) = OMcp29_124;
    sens.OM(2) = OMcp29_224;
    sens.OM(3) = OMcp29_324;
    sens.A(1) = ACcp29_130;
    sens.A(2) = ACcp29_230;
    sens.A(3) = ACcp29_330;
    sens.OMP(1) = OPcp29_124;
    sens.OMP(2) = OPcp29_224;
    sens.OMP(3) = OPcp29_324;
 
% 
case 31, 


% = = Block_1_0_0_31_0_1 = = 
 
% Sensor Kinematics 


    ROcp30_25 = S4*S5;
    ROcp30_35 = -C4*S5;
    ROcp30_85 = -S4*C5;
    ROcp30_95 = C4*C5;
    ROcp30_16 = C5*C6;
    ROcp30_26 = ROcp30_25*C6+C4*S6;
    ROcp30_36 = ROcp30_35*C6+S4*S6;
    ROcp30_46 = -C5*S6;
    ROcp30_56 = -(ROcp30_25*S6-C4*C6);
    ROcp30_66 = -(ROcp30_35*S6-S4*C6);
    OMcp30_25 = qd(5)*C4;
    OMcp30_35 = qd(5)*S4;
    OMcp30_16 = qd(4)+qd(6)*S5;
    OMcp30_26 = OMcp30_25+ROcp30_85*qd(6);
    OMcp30_36 = OMcp30_35+ROcp30_95*qd(6);
    OPcp30_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp30_26 = ROcp30_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp30_35*S5-ROcp30_95*qd(4));
    OPcp30_36 = ROcp30_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp30_25*S5-ROcp30_85*qd(4));

% = = Block_1_0_0_31_0_4 = = 
 
% Sensor Kinematics 


    ROcp30_131 = ROcp30_16*C31-S31*S5;
    ROcp30_231 = ROcp30_26*C31-ROcp30_85*S31;
    ROcp30_331 = ROcp30_36*C31-ROcp30_95*S31;
    ROcp30_731 = ROcp30_16*S31+C31*S5;
    ROcp30_831 = ROcp30_26*S31+ROcp30_85*C31;
    ROcp30_931 = ROcp30_36*S31+ROcp30_95*C31;
    RLcp30_131 = ROcp30_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp30_231 = ROcp30_26*s.dpt(1,3)+ROcp30_85*s.dpt(3,3);
    RLcp30_331 = ROcp30_36*s.dpt(1,3)+ROcp30_95*s.dpt(3,3);
    POcp30_131 = RLcp30_131+q(1);
    POcp30_231 = RLcp30_231+q(2);
    POcp30_331 = RLcp30_331+q(3);
    OMcp30_131 = OMcp30_16+ROcp30_46*qd(31);
    OMcp30_231 = OMcp30_26+ROcp30_56*qd(31);
    OMcp30_331 = OMcp30_36+ROcp30_66*qd(31);
    ORcp30_131 = OMcp30_26*RLcp30_331-OMcp30_36*RLcp30_231;
    ORcp30_231 = -(OMcp30_16*RLcp30_331-OMcp30_36*RLcp30_131);
    ORcp30_331 = OMcp30_16*RLcp30_231-OMcp30_26*RLcp30_131;
    VIcp30_131 = ORcp30_131+qd(1);
    VIcp30_231 = ORcp30_231+qd(2);
    VIcp30_331 = ORcp30_331+qd(3);
    OPcp30_131 = OPcp30_16+ROcp30_46*qdd(31)+qd(31)*(OMcp30_26*ROcp30_66-OMcp30_36*ROcp30_56);
    OPcp30_231 = OPcp30_26+ROcp30_56*qdd(31)-qd(31)*(OMcp30_16*ROcp30_66-OMcp30_36*ROcp30_46);
    OPcp30_331 = OPcp30_36+ROcp30_66*qdd(31)+qd(31)*(OMcp30_16*ROcp30_56-OMcp30_26*ROcp30_46);
    ACcp30_131 = qdd(1)+OMcp30_26*ORcp30_331-OMcp30_36*ORcp30_231+OPcp30_26*RLcp30_331-OPcp30_36*RLcp30_231;
    ACcp30_231 = qdd(2)-OMcp30_16*ORcp30_331+OMcp30_36*ORcp30_131-OPcp30_16*RLcp30_331+OPcp30_36*RLcp30_131;
    ACcp30_331 = qdd(3)+OMcp30_16*ORcp30_231-OMcp30_26*ORcp30_131+OPcp30_16*RLcp30_231-OPcp30_26*RLcp30_131;

% = = Block_1_0_0_31_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp30_131;
    sens.P(2) = POcp30_231;
    sens.P(3) = POcp30_331;
    sens.R(1,1) = ROcp30_131;
    sens.R(1,2) = ROcp30_231;
    sens.R(1,3) = ROcp30_331;
    sens.R(2,1) = ROcp30_46;
    sens.R(2,2) = ROcp30_56;
    sens.R(2,3) = ROcp30_66;
    sens.R(3,1) = ROcp30_731;
    sens.R(3,2) = ROcp30_831;
    sens.R(3,3) = ROcp30_931;
    sens.V(1) = VIcp30_131;
    sens.V(2) = VIcp30_231;
    sens.V(3) = VIcp30_331;
    sens.OM(1) = OMcp30_131;
    sens.OM(2) = OMcp30_231;
    sens.OM(3) = OMcp30_331;
    sens.A(1) = ACcp30_131;
    sens.A(2) = ACcp30_231;
    sens.A(3) = ACcp30_331;
    sens.OMP(1) = OPcp30_131;
    sens.OMP(2) = OPcp30_231;
    sens.OMP(3) = OPcp30_331;
 
% 
case 32, 


% = = Block_1_0_0_32_0_1 = = 
 
% Sensor Kinematics 


    ROcp31_25 = S4*S5;
    ROcp31_35 = -C4*S5;
    ROcp31_85 = -S4*C5;
    ROcp31_95 = C4*C5;
    ROcp31_16 = C5*C6;
    ROcp31_26 = ROcp31_25*C6+C4*S6;
    ROcp31_36 = ROcp31_35*C6+S4*S6;
    ROcp31_46 = -C5*S6;
    ROcp31_56 = -(ROcp31_25*S6-C4*C6);
    ROcp31_66 = -(ROcp31_35*S6-S4*C6);
    OMcp31_25 = qd(5)*C4;
    OMcp31_35 = qd(5)*S4;
    OMcp31_16 = qd(4)+qd(6)*S5;
    OMcp31_26 = OMcp31_25+ROcp31_85*qd(6);
    OMcp31_36 = OMcp31_35+ROcp31_95*qd(6);
    OPcp31_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp31_26 = ROcp31_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp31_35*S5-ROcp31_95*qd(4));
    OPcp31_36 = ROcp31_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp31_25*S5-ROcp31_85*qd(4));

% = = Block_1_0_0_32_0_4 = = 
 
% Sensor Kinematics 


    ROcp31_131 = ROcp31_16*C31-S31*S5;
    ROcp31_231 = ROcp31_26*C31-ROcp31_85*S31;
    ROcp31_331 = ROcp31_36*C31-ROcp31_95*S31;
    ROcp31_731 = ROcp31_16*S31+C31*S5;
    ROcp31_831 = ROcp31_26*S31+ROcp31_85*C31;
    ROcp31_931 = ROcp31_36*S31+ROcp31_95*C31;
    ROcp31_432 = ROcp31_46*C32+ROcp31_731*S32;
    ROcp31_532 = ROcp31_56*C32+ROcp31_831*S32;
    ROcp31_632 = ROcp31_66*C32+ROcp31_931*S32;
    ROcp31_732 = -(ROcp31_46*S32-ROcp31_731*C32);
    ROcp31_832 = -(ROcp31_56*S32-ROcp31_831*C32);
    ROcp31_932 = -(ROcp31_66*S32-ROcp31_931*C32);
    RLcp31_131 = ROcp31_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp31_231 = ROcp31_26*s.dpt(1,3)+ROcp31_85*s.dpt(3,3);
    RLcp31_331 = ROcp31_36*s.dpt(1,3)+ROcp31_95*s.dpt(3,3);
    POcp31_131 = RLcp31_131+q(1);
    POcp31_231 = RLcp31_231+q(2);
    POcp31_331 = RLcp31_331+q(3);
    OMcp31_131 = OMcp31_16+ROcp31_46*qd(31);
    OMcp31_231 = OMcp31_26+ROcp31_56*qd(31);
    OMcp31_331 = OMcp31_36+ROcp31_66*qd(31);
    ORcp31_131 = OMcp31_26*RLcp31_331-OMcp31_36*RLcp31_231;
    ORcp31_231 = -(OMcp31_16*RLcp31_331-OMcp31_36*RLcp31_131);
    ORcp31_331 = OMcp31_16*RLcp31_231-OMcp31_26*RLcp31_131;
    VIcp31_131 = ORcp31_131+qd(1);
    VIcp31_231 = ORcp31_231+qd(2);
    VIcp31_331 = ORcp31_331+qd(3);
    ACcp31_131 = qdd(1)+OMcp31_26*ORcp31_331-OMcp31_36*ORcp31_231+OPcp31_26*RLcp31_331-OPcp31_36*RLcp31_231;
    ACcp31_231 = qdd(2)-OMcp31_16*ORcp31_331+OMcp31_36*ORcp31_131-OPcp31_16*RLcp31_331+OPcp31_36*RLcp31_131;
    ACcp31_331 = qdd(3)+OMcp31_16*ORcp31_231-OMcp31_26*ORcp31_131+OPcp31_16*RLcp31_231-OPcp31_26*RLcp31_131;
    OMcp31_132 = OMcp31_131+ROcp31_131*qd(32);
    OMcp31_232 = OMcp31_231+ROcp31_231*qd(32);
    OMcp31_332 = OMcp31_331+ROcp31_331*qd(32);
    OPcp31_132 = OPcp31_16+ROcp31_131*qdd(32)+ROcp31_46*qdd(31)+qd(31)*(OMcp31_26*ROcp31_66-OMcp31_36*ROcp31_56)+qd(32)*(OMcp31_231*ROcp31_331-...
 OMcp31_331*ROcp31_231);
    OPcp31_232 = OPcp31_26+ROcp31_231*qdd(32)+ROcp31_56*qdd(31)-qd(31)*(OMcp31_16*ROcp31_66-OMcp31_36*ROcp31_46)-qd(32)*(OMcp31_131*ROcp31_331-...
 OMcp31_331*ROcp31_131);
    OPcp31_332 = OPcp31_36+ROcp31_331*qdd(32)+ROcp31_66*qdd(31)+qd(31)*(OMcp31_16*ROcp31_56-OMcp31_26*ROcp31_46)+qd(32)*(OMcp31_131*ROcp31_231-...
 OMcp31_231*ROcp31_131);

% = = Block_1_0_0_32_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp31_131;
    sens.P(2) = POcp31_231;
    sens.P(3) = POcp31_331;
    sens.R(1,1) = ROcp31_131;
    sens.R(1,2) = ROcp31_231;
    sens.R(1,3) = ROcp31_331;
    sens.R(2,1) = ROcp31_432;
    sens.R(2,2) = ROcp31_532;
    sens.R(2,3) = ROcp31_632;
    sens.R(3,1) = ROcp31_732;
    sens.R(3,2) = ROcp31_832;
    sens.R(3,3) = ROcp31_932;
    sens.V(1) = VIcp31_131;
    sens.V(2) = VIcp31_231;
    sens.V(3) = VIcp31_331;
    sens.OM(1) = OMcp31_132;
    sens.OM(2) = OMcp31_232;
    sens.OM(3) = OMcp31_332;
    sens.A(1) = ACcp31_131;
    sens.A(2) = ACcp31_231;
    sens.A(3) = ACcp31_331;
    sens.OMP(1) = OPcp31_132;
    sens.OMP(2) = OPcp31_232;
    sens.OMP(3) = OPcp31_332;
 
% 
case 33, 


% = = Block_1_0_0_33_0_1 = = 
 
% Sensor Kinematics 


    ROcp32_25 = S4*S5;
    ROcp32_35 = -C4*S5;
    ROcp32_85 = -S4*C5;
    ROcp32_95 = C4*C5;
    ROcp32_16 = C5*C6;
    ROcp32_26 = ROcp32_25*C6+C4*S6;
    ROcp32_36 = ROcp32_35*C6+S4*S6;
    ROcp32_46 = -C5*S6;
    ROcp32_56 = -(ROcp32_25*S6-C4*C6);
    ROcp32_66 = -(ROcp32_35*S6-S4*C6);
    OMcp32_25 = qd(5)*C4;
    OMcp32_35 = qd(5)*S4;
    OMcp32_16 = qd(4)+qd(6)*S5;
    OMcp32_26 = OMcp32_25+ROcp32_85*qd(6);
    OMcp32_36 = OMcp32_35+ROcp32_95*qd(6);
    OPcp32_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp32_26 = ROcp32_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp32_35*S5-ROcp32_95*qd(4));
    OPcp32_36 = ROcp32_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp32_25*S5-ROcp32_85*qd(4));

% = = Block_1_0_0_33_0_4 = = 
 
% Sensor Kinematics 


    ROcp32_131 = ROcp32_16*C31-S31*S5;
    ROcp32_231 = ROcp32_26*C31-ROcp32_85*S31;
    ROcp32_331 = ROcp32_36*C31-ROcp32_95*S31;
    ROcp32_731 = ROcp32_16*S31+C31*S5;
    ROcp32_831 = ROcp32_26*S31+ROcp32_85*C31;
    ROcp32_931 = ROcp32_36*S31+ROcp32_95*C31;
    ROcp32_432 = ROcp32_46*C32+ROcp32_731*S32;
    ROcp32_532 = ROcp32_56*C32+ROcp32_831*S32;
    ROcp32_632 = ROcp32_66*C32+ROcp32_931*S32;
    ROcp32_732 = -(ROcp32_46*S32-ROcp32_731*C32);
    ROcp32_832 = -(ROcp32_56*S32-ROcp32_831*C32);
    ROcp32_932 = -(ROcp32_66*S32-ROcp32_931*C32);
    ROcp32_133 = ROcp32_131*C33-ROcp32_732*S33;
    ROcp32_233 = ROcp32_231*C33-ROcp32_832*S33;
    ROcp32_333 = ROcp32_331*C33-ROcp32_932*S33;
    ROcp32_733 = ROcp32_131*S33+ROcp32_732*C33;
    ROcp32_833 = ROcp32_231*S33+ROcp32_832*C33;
    ROcp32_933 = ROcp32_331*S33+ROcp32_932*C33;
    RLcp32_131 = ROcp32_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp32_231 = ROcp32_26*s.dpt(1,3)+ROcp32_85*s.dpt(3,3);
    RLcp32_331 = ROcp32_36*s.dpt(1,3)+ROcp32_95*s.dpt(3,3);
    POcp32_131 = RLcp32_131+q(1);
    POcp32_231 = RLcp32_231+q(2);
    POcp32_331 = RLcp32_331+q(3);
    OMcp32_131 = OMcp32_16+ROcp32_46*qd(31);
    OMcp32_231 = OMcp32_26+ROcp32_56*qd(31);
    OMcp32_331 = OMcp32_36+ROcp32_66*qd(31);
    ORcp32_131 = OMcp32_26*RLcp32_331-OMcp32_36*RLcp32_231;
    ORcp32_231 = -(OMcp32_16*RLcp32_331-OMcp32_36*RLcp32_131);
    ORcp32_331 = OMcp32_16*RLcp32_231-OMcp32_26*RLcp32_131;
    VIcp32_131 = ORcp32_131+qd(1);
    VIcp32_231 = ORcp32_231+qd(2);
    VIcp32_331 = ORcp32_331+qd(3);
    ACcp32_131 = qdd(1)+OMcp32_26*ORcp32_331-OMcp32_36*ORcp32_231+OPcp32_26*RLcp32_331-OPcp32_36*RLcp32_231;
    ACcp32_231 = qdd(2)-OMcp32_16*ORcp32_331+OMcp32_36*ORcp32_131-OPcp32_16*RLcp32_331+OPcp32_36*RLcp32_131;
    ACcp32_331 = qdd(3)+OMcp32_16*ORcp32_231-OMcp32_26*ORcp32_131+OPcp32_16*RLcp32_231-OPcp32_26*RLcp32_131;
    OMcp32_132 = OMcp32_131+ROcp32_131*qd(32);
    OMcp32_232 = OMcp32_231+ROcp32_231*qd(32);
    OMcp32_332 = OMcp32_331+ROcp32_331*qd(32);
    OMcp32_133 = OMcp32_132+ROcp32_432*qd(33);
    OMcp32_233 = OMcp32_232+ROcp32_532*qd(33);
    OMcp32_333 = OMcp32_332+ROcp32_632*qd(33);
    OPcp32_133 = OPcp32_16+ROcp32_131*qdd(32)+ROcp32_432*qdd(33)+ROcp32_46*qdd(31)+qd(31)*(OMcp32_26*ROcp32_66-OMcp32_36*ROcp32_56)+qd(32)*(...
 OMcp32_231*ROcp32_331-OMcp32_331*ROcp32_231)+qd(33)*(OMcp32_232*ROcp32_632-OMcp32_332*ROcp32_532);
    OPcp32_233 = OPcp32_26+ROcp32_231*qdd(32)+ROcp32_532*qdd(33)+ROcp32_56*qdd(31)-qd(31)*(OMcp32_16*ROcp32_66-OMcp32_36*ROcp32_46)-qd(32)*(...
 OMcp32_131*ROcp32_331-OMcp32_331*ROcp32_131)-qd(33)*(OMcp32_132*ROcp32_632-OMcp32_332*ROcp32_432);
    OPcp32_333 = OPcp32_36+ROcp32_331*qdd(32)+ROcp32_632*qdd(33)+ROcp32_66*qdd(31)+qd(31)*(OMcp32_16*ROcp32_56-OMcp32_26*ROcp32_46)+qd(32)*(...
 OMcp32_131*ROcp32_231-OMcp32_231*ROcp32_131)+qd(33)*(OMcp32_132*ROcp32_532-OMcp32_232*ROcp32_432);

% = = Block_1_0_0_33_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp32_131;
    sens.P(2) = POcp32_231;
    sens.P(3) = POcp32_331;
    sens.R(1,1) = ROcp32_133;
    sens.R(1,2) = ROcp32_233;
    sens.R(1,3) = ROcp32_333;
    sens.R(2,1) = ROcp32_432;
    sens.R(2,2) = ROcp32_532;
    sens.R(2,3) = ROcp32_632;
    sens.R(3,1) = ROcp32_733;
    sens.R(3,2) = ROcp32_833;
    sens.R(3,3) = ROcp32_933;
    sens.V(1) = VIcp32_131;
    sens.V(2) = VIcp32_231;
    sens.V(3) = VIcp32_331;
    sens.OM(1) = OMcp32_133;
    sens.OM(2) = OMcp32_233;
    sens.OM(3) = OMcp32_333;
    sens.A(1) = ACcp32_131;
    sens.A(2) = ACcp32_231;
    sens.A(3) = ACcp32_331;
    sens.OMP(1) = OPcp32_133;
    sens.OMP(2) = OPcp32_233;
    sens.OMP(3) = OPcp32_333;
 
% 
case 34, 


% = = Block_1_0_0_34_0_1 = = 
 
% Sensor Kinematics 


    ROcp33_25 = S4*S5;
    ROcp33_35 = -C4*S5;
    ROcp33_85 = -S4*C5;
    ROcp33_95 = C4*C5;
    ROcp33_16 = C5*C6;
    ROcp33_26 = ROcp33_25*C6+C4*S6;
    ROcp33_36 = ROcp33_35*C6+S4*S6;
    ROcp33_46 = -C5*S6;
    ROcp33_56 = -(ROcp33_25*S6-C4*C6);
    ROcp33_66 = -(ROcp33_35*S6-S4*C6);
    OMcp33_25 = qd(5)*C4;
    OMcp33_35 = qd(5)*S4;
    OMcp33_16 = qd(4)+qd(6)*S5;
    OMcp33_26 = OMcp33_25+ROcp33_85*qd(6);
    OMcp33_36 = OMcp33_35+ROcp33_95*qd(6);
    OPcp33_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp33_26 = ROcp33_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp33_35*S5-ROcp33_95*qd(4));
    OPcp33_36 = ROcp33_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp33_25*S5-ROcp33_85*qd(4));

% = = Block_1_0_0_34_0_4 = = 
 
% Sensor Kinematics 


    ROcp33_131 = ROcp33_16*C31-S31*S5;
    ROcp33_231 = ROcp33_26*C31-ROcp33_85*S31;
    ROcp33_331 = ROcp33_36*C31-ROcp33_95*S31;
    ROcp33_731 = ROcp33_16*S31+C31*S5;
    ROcp33_831 = ROcp33_26*S31+ROcp33_85*C31;
    ROcp33_931 = ROcp33_36*S31+ROcp33_95*C31;
    ROcp33_432 = ROcp33_46*C32+ROcp33_731*S32;
    ROcp33_532 = ROcp33_56*C32+ROcp33_831*S32;
    ROcp33_632 = ROcp33_66*C32+ROcp33_931*S32;
    ROcp33_732 = -(ROcp33_46*S32-ROcp33_731*C32);
    ROcp33_832 = -(ROcp33_56*S32-ROcp33_831*C32);
    ROcp33_932 = -(ROcp33_66*S32-ROcp33_931*C32);
    ROcp33_133 = ROcp33_131*C33-ROcp33_732*S33;
    ROcp33_233 = ROcp33_231*C33-ROcp33_832*S33;
    ROcp33_333 = ROcp33_331*C33-ROcp33_932*S33;
    ROcp33_733 = ROcp33_131*S33+ROcp33_732*C33;
    ROcp33_833 = ROcp33_231*S33+ROcp33_832*C33;
    ROcp33_933 = ROcp33_331*S33+ROcp33_932*C33;
    ROcp33_134 = ROcp33_133*C34+ROcp33_432*S34;
    ROcp33_234 = ROcp33_233*C34+ROcp33_532*S34;
    ROcp33_334 = ROcp33_333*C34+ROcp33_632*S34;
    ROcp33_434 = -(ROcp33_133*S34-ROcp33_432*C34);
    ROcp33_534 = -(ROcp33_233*S34-ROcp33_532*C34);
    ROcp33_634 = -(ROcp33_333*S34-ROcp33_632*C34);
    RLcp33_131 = ROcp33_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp33_231 = ROcp33_26*s.dpt(1,3)+ROcp33_85*s.dpt(3,3);
    RLcp33_331 = ROcp33_36*s.dpt(1,3)+ROcp33_95*s.dpt(3,3);
    OMcp33_131 = OMcp33_16+ROcp33_46*qd(31);
    OMcp33_231 = OMcp33_26+ROcp33_56*qd(31);
    OMcp33_331 = OMcp33_36+ROcp33_66*qd(31);
    ORcp33_131 = OMcp33_26*RLcp33_331-OMcp33_36*RLcp33_231;
    ORcp33_231 = -(OMcp33_16*RLcp33_331-OMcp33_36*RLcp33_131);
    ORcp33_331 = OMcp33_16*RLcp33_231-OMcp33_26*RLcp33_131;
    OMcp33_132 = OMcp33_131+ROcp33_131*qd(32);
    OMcp33_232 = OMcp33_231+ROcp33_231*qd(32);
    OMcp33_332 = OMcp33_331+ROcp33_331*qd(32);
    OMcp33_133 = OMcp33_132+ROcp33_432*qd(33);
    OMcp33_233 = OMcp33_232+ROcp33_532*qd(33);
    OMcp33_333 = OMcp33_332+ROcp33_632*qd(33);
    OPcp33_133 = OPcp33_16+ROcp33_131*qdd(32)+ROcp33_432*qdd(33)+ROcp33_46*qdd(31)+qd(31)*(OMcp33_26*ROcp33_66-OMcp33_36*ROcp33_56)+qd(32)*(...
 OMcp33_231*ROcp33_331-OMcp33_331*ROcp33_231)+qd(33)*(OMcp33_232*ROcp33_632-OMcp33_332*ROcp33_532);
    OPcp33_233 = OPcp33_26+ROcp33_231*qdd(32)+ROcp33_532*qdd(33)+ROcp33_56*qdd(31)-qd(31)*(OMcp33_16*ROcp33_66-OMcp33_36*ROcp33_46)-qd(32)*(...
 OMcp33_131*ROcp33_331-OMcp33_331*ROcp33_131)-qd(33)*(OMcp33_132*ROcp33_632-OMcp33_332*ROcp33_432);
    OPcp33_333 = OPcp33_36+ROcp33_331*qdd(32)+ROcp33_632*qdd(33)+ROcp33_66*qdd(31)+qd(31)*(OMcp33_16*ROcp33_56-OMcp33_26*ROcp33_46)+qd(32)*(...
 OMcp33_131*ROcp33_231-OMcp33_231*ROcp33_131)+qd(33)*(OMcp33_132*ROcp33_532-OMcp33_232*ROcp33_432);
    RLcp33_134 = ROcp33_733*s.dpt(3,21);
    RLcp33_234 = ROcp33_833*s.dpt(3,21);
    RLcp33_334 = ROcp33_933*s.dpt(3,21);
    POcp33_134 = RLcp33_131+RLcp33_134+q(1);
    POcp33_234 = RLcp33_231+RLcp33_234+q(2);
    POcp33_334 = RLcp33_331+RLcp33_334+q(3);
    OMcp33_134 = OMcp33_133+ROcp33_733*qd(34);
    OMcp33_234 = OMcp33_233+ROcp33_833*qd(34);
    OMcp33_334 = OMcp33_333+ROcp33_933*qd(34);
    ORcp33_134 = OMcp33_233*RLcp33_334-OMcp33_333*RLcp33_234;
    ORcp33_234 = -(OMcp33_133*RLcp33_334-OMcp33_333*RLcp33_134);
    ORcp33_334 = OMcp33_133*RLcp33_234-OMcp33_233*RLcp33_134;
    VIcp33_134 = ORcp33_131+ORcp33_134+qd(1);
    VIcp33_234 = ORcp33_231+ORcp33_234+qd(2);
    VIcp33_334 = ORcp33_331+ORcp33_334+qd(3);
    OPcp33_134 = OPcp33_133+ROcp33_733*qdd(34)+qd(34)*(OMcp33_233*ROcp33_933-OMcp33_333*ROcp33_833);
    OPcp33_234 = OPcp33_233+ROcp33_833*qdd(34)-qd(34)*(OMcp33_133*ROcp33_933-OMcp33_333*ROcp33_733);
    OPcp33_334 = OPcp33_333+ROcp33_933*qdd(34)+qd(34)*(OMcp33_133*ROcp33_833-OMcp33_233*ROcp33_733);
    ACcp33_134 = qdd(1)+OMcp33_233*ORcp33_334+OMcp33_26*ORcp33_331-OMcp33_333*ORcp33_234-OMcp33_36*ORcp33_231+OPcp33_233*RLcp33_334+OPcp33_26*...
 RLcp33_331-OPcp33_333*RLcp33_234-OPcp33_36*RLcp33_231;
    ACcp33_234 = qdd(2)-OMcp33_133*ORcp33_334-OMcp33_16*ORcp33_331+OMcp33_333*ORcp33_134+OMcp33_36*ORcp33_131-OPcp33_133*RLcp33_334-OPcp33_16*...
 RLcp33_331+OPcp33_333*RLcp33_134+OPcp33_36*RLcp33_131;
    ACcp33_334 = qdd(3)+OMcp33_133*ORcp33_234+OMcp33_16*ORcp33_231-OMcp33_233*ORcp33_134-OMcp33_26*ORcp33_131+OPcp33_133*RLcp33_234+OPcp33_16*...
 RLcp33_231-OPcp33_233*RLcp33_134-OPcp33_26*RLcp33_131;

% = = Block_1_0_0_34_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp33_134;
    sens.P(2) = POcp33_234;
    sens.P(3) = POcp33_334;
    sens.R(1,1) = ROcp33_134;
    sens.R(1,2) = ROcp33_234;
    sens.R(1,3) = ROcp33_334;
    sens.R(2,1) = ROcp33_434;
    sens.R(2,2) = ROcp33_534;
    sens.R(2,3) = ROcp33_634;
    sens.R(3,1) = ROcp33_733;
    sens.R(3,2) = ROcp33_833;
    sens.R(3,3) = ROcp33_933;
    sens.V(1) = VIcp33_134;
    sens.V(2) = VIcp33_234;
    sens.V(3) = VIcp33_334;
    sens.OM(1) = OMcp33_134;
    sens.OM(2) = OMcp33_234;
    sens.OM(3) = OMcp33_334;
    sens.A(1) = ACcp33_134;
    sens.A(2) = ACcp33_234;
    sens.A(3) = ACcp33_334;
    sens.OMP(1) = OPcp33_134;
    sens.OMP(2) = OPcp33_234;
    sens.OMP(3) = OPcp33_334;
 
% 
case 35, 


% = = Block_1_0_0_35_0_1 = = 
 
% Sensor Kinematics 


    ROcp34_25 = S4*S5;
    ROcp34_35 = -C4*S5;
    ROcp34_85 = -S4*C5;
    ROcp34_95 = C4*C5;
    ROcp34_16 = C5*C6;
    ROcp34_26 = ROcp34_25*C6+C4*S6;
    ROcp34_36 = ROcp34_35*C6+S4*S6;
    ROcp34_46 = -C5*S6;
    ROcp34_56 = -(ROcp34_25*S6-C4*C6);
    ROcp34_66 = -(ROcp34_35*S6-S4*C6);
    OMcp34_25 = qd(5)*C4;
    OMcp34_35 = qd(5)*S4;
    OMcp34_16 = qd(4)+qd(6)*S5;
    OMcp34_26 = OMcp34_25+ROcp34_85*qd(6);
    OMcp34_36 = OMcp34_35+ROcp34_95*qd(6);
    OPcp34_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp34_26 = ROcp34_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp34_35*S5-ROcp34_95*qd(4));
    OPcp34_36 = ROcp34_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp34_25*S5-ROcp34_85*qd(4));

% = = Block_1_0_0_35_0_4 = = 
 
% Sensor Kinematics 


    ROcp34_131 = ROcp34_16*C31-S31*S5;
    ROcp34_231 = ROcp34_26*C31-ROcp34_85*S31;
    ROcp34_331 = ROcp34_36*C31-ROcp34_95*S31;
    ROcp34_731 = ROcp34_16*S31+C31*S5;
    ROcp34_831 = ROcp34_26*S31+ROcp34_85*C31;
    ROcp34_931 = ROcp34_36*S31+ROcp34_95*C31;
    ROcp34_432 = ROcp34_46*C32+ROcp34_731*S32;
    ROcp34_532 = ROcp34_56*C32+ROcp34_831*S32;
    ROcp34_632 = ROcp34_66*C32+ROcp34_931*S32;
    ROcp34_732 = -(ROcp34_46*S32-ROcp34_731*C32);
    ROcp34_832 = -(ROcp34_56*S32-ROcp34_831*C32);
    ROcp34_932 = -(ROcp34_66*S32-ROcp34_931*C32);
    ROcp34_133 = ROcp34_131*C33-ROcp34_732*S33;
    ROcp34_233 = ROcp34_231*C33-ROcp34_832*S33;
    ROcp34_333 = ROcp34_331*C33-ROcp34_932*S33;
    ROcp34_733 = ROcp34_131*S33+ROcp34_732*C33;
    ROcp34_833 = ROcp34_231*S33+ROcp34_832*C33;
    ROcp34_933 = ROcp34_331*S33+ROcp34_932*C33;
    ROcp34_134 = ROcp34_133*C34+ROcp34_432*S34;
    ROcp34_234 = ROcp34_233*C34+ROcp34_532*S34;
    ROcp34_334 = ROcp34_333*C34+ROcp34_632*S34;
    ROcp34_434 = -(ROcp34_133*S34-ROcp34_432*C34);
    ROcp34_534 = -(ROcp34_233*S34-ROcp34_532*C34);
    ROcp34_634 = -(ROcp34_333*S34-ROcp34_632*C34);
    RLcp34_131 = ROcp34_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp34_231 = ROcp34_26*s.dpt(1,3)+ROcp34_85*s.dpt(3,3);
    RLcp34_331 = ROcp34_36*s.dpt(1,3)+ROcp34_95*s.dpt(3,3);
    OMcp34_131 = OMcp34_16+ROcp34_46*qd(31);
    OMcp34_231 = OMcp34_26+ROcp34_56*qd(31);
    OMcp34_331 = OMcp34_36+ROcp34_66*qd(31);
    ORcp34_131 = OMcp34_26*RLcp34_331-OMcp34_36*RLcp34_231;
    ORcp34_231 = -(OMcp34_16*RLcp34_331-OMcp34_36*RLcp34_131);
    ORcp34_331 = OMcp34_16*RLcp34_231-OMcp34_26*RLcp34_131;
    OMcp34_132 = OMcp34_131+ROcp34_131*qd(32);
    OMcp34_232 = OMcp34_231+ROcp34_231*qd(32);
    OMcp34_332 = OMcp34_331+ROcp34_331*qd(32);
    OMcp34_133 = OMcp34_132+ROcp34_432*qd(33);
    OMcp34_233 = OMcp34_232+ROcp34_532*qd(33);
    OMcp34_333 = OMcp34_332+ROcp34_632*qd(33);
    OPcp34_133 = OPcp34_16+ROcp34_131*qdd(32)+ROcp34_432*qdd(33)+ROcp34_46*qdd(31)+qd(31)*(OMcp34_26*ROcp34_66-OMcp34_36*ROcp34_56)+qd(32)*(...
 OMcp34_231*ROcp34_331-OMcp34_331*ROcp34_231)+qd(33)*(OMcp34_232*ROcp34_632-OMcp34_332*ROcp34_532);
    OPcp34_233 = OPcp34_26+ROcp34_231*qdd(32)+ROcp34_532*qdd(33)+ROcp34_56*qdd(31)-qd(31)*(OMcp34_16*ROcp34_66-OMcp34_36*ROcp34_46)-qd(32)*(...
 OMcp34_131*ROcp34_331-OMcp34_331*ROcp34_131)-qd(33)*(OMcp34_132*ROcp34_632-OMcp34_332*ROcp34_432);
    OPcp34_333 = OPcp34_36+ROcp34_331*qdd(32)+ROcp34_632*qdd(33)+ROcp34_66*qdd(31)+qd(31)*(OMcp34_16*ROcp34_56-OMcp34_26*ROcp34_46)+qd(32)*(...
 OMcp34_131*ROcp34_231-OMcp34_231*ROcp34_131)+qd(33)*(OMcp34_132*ROcp34_532-OMcp34_232*ROcp34_432);
    RLcp34_134 = ROcp34_733*s.dpt(3,21);
    RLcp34_234 = ROcp34_833*s.dpt(3,21);
    RLcp34_334 = ROcp34_933*s.dpt(3,21);
    OMcp34_134 = OMcp34_133+ROcp34_733*qd(34);
    OMcp34_234 = OMcp34_233+ROcp34_833*qd(34);
    OMcp34_334 = OMcp34_333+ROcp34_933*qd(34);
    ORcp34_134 = OMcp34_233*RLcp34_334-OMcp34_333*RLcp34_234;
    ORcp34_234 = -(OMcp34_133*RLcp34_334-OMcp34_333*RLcp34_134);
    ORcp34_334 = OMcp34_133*RLcp34_234-OMcp34_233*RLcp34_134;
    OPcp34_134 = OPcp34_133+ROcp34_733*qdd(34)+qd(34)*(OMcp34_233*ROcp34_933-OMcp34_333*ROcp34_833);
    OPcp34_234 = OPcp34_233+ROcp34_833*qdd(34)-qd(34)*(OMcp34_133*ROcp34_933-OMcp34_333*ROcp34_733);
    OPcp34_334 = OPcp34_333+ROcp34_933*qdd(34)+qd(34)*(OMcp34_133*ROcp34_833-OMcp34_233*ROcp34_733);

% = = Block_1_0_0_35_0_5 = = 
 
% Sensor Kinematics 


    ROcp34_435 = ROcp34_434*C35+ROcp34_733*S35;
    ROcp34_535 = ROcp34_534*C35+ROcp34_833*S35;
    ROcp34_635 = ROcp34_634*C35+ROcp34_933*S35;
    ROcp34_735 = -(ROcp34_434*S35-ROcp34_733*C35);
    ROcp34_835 = -(ROcp34_534*S35-ROcp34_833*C35);
    ROcp34_935 = -(ROcp34_634*S35-ROcp34_933*C35);
    RLcp34_135 = ROcp34_134*s.dpt(1,23)+ROcp34_434*s.dpt(2,23)+ROcp34_733*s.dpt(3,23);
    RLcp34_235 = ROcp34_234*s.dpt(1,23)+ROcp34_534*s.dpt(2,23)+ROcp34_833*s.dpt(3,23);
    RLcp34_335 = ROcp34_334*s.dpt(1,23)+ROcp34_634*s.dpt(2,23)+ROcp34_933*s.dpt(3,23);
    POcp34_135 = RLcp34_131+RLcp34_134+RLcp34_135+q(1);
    POcp34_235 = RLcp34_231+RLcp34_234+RLcp34_235+q(2);
    POcp34_335 = RLcp34_331+RLcp34_334+RLcp34_335+q(3);
    ORcp34_135 = OMcp34_234*RLcp34_335-OMcp34_334*RLcp34_235;
    ORcp34_235 = -(OMcp34_134*RLcp34_335-OMcp34_334*RLcp34_135);
    ORcp34_335 = OMcp34_134*RLcp34_235-OMcp34_234*RLcp34_135;
    VIcp34_135 = ORcp34_131+ORcp34_134+ORcp34_135+qd(1);
    VIcp34_235 = ORcp34_231+ORcp34_234+ORcp34_235+qd(2);
    VIcp34_335 = ORcp34_331+ORcp34_334+ORcp34_335+qd(3);
    ACcp34_135 = qdd(1)+OMcp34_233*ORcp34_334+OMcp34_234*ORcp34_335+OMcp34_26*ORcp34_331-OMcp34_333*ORcp34_234-OMcp34_334*ORcp34_235-OMcp34_36*...
 ORcp34_231+OPcp34_233*RLcp34_334+OPcp34_234*RLcp34_335+OPcp34_26*RLcp34_331-OPcp34_333*RLcp34_234-OPcp34_334*RLcp34_235-OPcp34_36*RLcp34_231;
    ACcp34_235 = qdd(2)-OMcp34_133*ORcp34_334-OMcp34_134*ORcp34_335-OMcp34_16*ORcp34_331+OMcp34_333*ORcp34_134+OMcp34_334*ORcp34_135+OMcp34_36*...
 ORcp34_131-OPcp34_133*RLcp34_334-OPcp34_134*RLcp34_335-OPcp34_16*RLcp34_331+OPcp34_333*RLcp34_134+OPcp34_334*RLcp34_135+OPcp34_36*RLcp34_131;
    ACcp34_335 = qdd(3)+OMcp34_133*ORcp34_234+OMcp34_134*ORcp34_235+OMcp34_16*ORcp34_231-OMcp34_233*ORcp34_134-OMcp34_234*ORcp34_135-OMcp34_26*...
 ORcp34_131+OPcp34_133*RLcp34_234+OPcp34_134*RLcp34_235+OPcp34_16*RLcp34_231-OPcp34_233*RLcp34_134-OPcp34_234*RLcp34_135-OPcp34_26*RLcp34_131;

% = = Block_1_0_0_35_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp34_135;
    sens.P(2) = POcp34_235;
    sens.P(3) = POcp34_335;
    sens.R(1,1) = ROcp34_134;
    sens.R(1,2) = ROcp34_234;
    sens.R(1,3) = ROcp34_334;
    sens.R(2,1) = ROcp34_435;
    sens.R(2,2) = ROcp34_535;
    sens.R(2,3) = ROcp34_635;
    sens.R(3,1) = ROcp34_735;
    sens.R(3,2) = ROcp34_835;
    sens.R(3,3) = ROcp34_935;
    sens.V(1) = VIcp34_135;
    sens.V(2) = VIcp34_235;
    sens.V(3) = VIcp34_335;
    sens.OM(1) = OMcp34_134;
    sens.OM(2) = OMcp34_234;
    sens.OM(3) = OMcp34_334;
    sens.A(1) = ACcp34_135;
    sens.A(2) = ACcp34_235;
    sens.A(3) = ACcp34_335;
    sens.OMP(1) = OPcp34_134;
    sens.OMP(2) = OPcp34_234;
    sens.OMP(3) = OPcp34_334;
 
% 
case 36, 


% = = Block_1_0_0_36_0_1 = = 
 
% Sensor Kinematics 


    ROcp35_25 = S4*S5;
    ROcp35_35 = -C4*S5;
    ROcp35_85 = -S4*C5;
    ROcp35_95 = C4*C5;
    ROcp35_16 = C5*C6;
    ROcp35_26 = ROcp35_25*C6+C4*S6;
    ROcp35_36 = ROcp35_35*C6+S4*S6;
    ROcp35_46 = -C5*S6;
    ROcp35_56 = -(ROcp35_25*S6-C4*C6);
    ROcp35_66 = -(ROcp35_35*S6-S4*C6);
    OMcp35_25 = qd(5)*C4;
    OMcp35_35 = qd(5)*S4;
    OMcp35_16 = qd(4)+qd(6)*S5;
    OMcp35_26 = OMcp35_25+ROcp35_85*qd(6);
    OMcp35_36 = OMcp35_35+ROcp35_95*qd(6);
    OPcp35_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp35_26 = ROcp35_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp35_35*S5-ROcp35_95*qd(4));
    OPcp35_36 = ROcp35_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp35_25*S5-ROcp35_85*qd(4));

% = = Block_1_0_0_36_0_4 = = 
 
% Sensor Kinematics 


    ROcp35_131 = ROcp35_16*C31-S31*S5;
    ROcp35_231 = ROcp35_26*C31-ROcp35_85*S31;
    ROcp35_331 = ROcp35_36*C31-ROcp35_95*S31;
    ROcp35_731 = ROcp35_16*S31+C31*S5;
    ROcp35_831 = ROcp35_26*S31+ROcp35_85*C31;
    ROcp35_931 = ROcp35_36*S31+ROcp35_95*C31;
    ROcp35_432 = ROcp35_46*C32+ROcp35_731*S32;
    ROcp35_532 = ROcp35_56*C32+ROcp35_831*S32;
    ROcp35_632 = ROcp35_66*C32+ROcp35_931*S32;
    ROcp35_732 = -(ROcp35_46*S32-ROcp35_731*C32);
    ROcp35_832 = -(ROcp35_56*S32-ROcp35_831*C32);
    ROcp35_932 = -(ROcp35_66*S32-ROcp35_931*C32);
    ROcp35_133 = ROcp35_131*C33-ROcp35_732*S33;
    ROcp35_233 = ROcp35_231*C33-ROcp35_832*S33;
    ROcp35_333 = ROcp35_331*C33-ROcp35_932*S33;
    ROcp35_733 = ROcp35_131*S33+ROcp35_732*C33;
    ROcp35_833 = ROcp35_231*S33+ROcp35_832*C33;
    ROcp35_933 = ROcp35_331*S33+ROcp35_932*C33;
    ROcp35_134 = ROcp35_133*C34+ROcp35_432*S34;
    ROcp35_234 = ROcp35_233*C34+ROcp35_532*S34;
    ROcp35_334 = ROcp35_333*C34+ROcp35_632*S34;
    ROcp35_434 = -(ROcp35_133*S34-ROcp35_432*C34);
    ROcp35_534 = -(ROcp35_233*S34-ROcp35_532*C34);
    ROcp35_634 = -(ROcp35_333*S34-ROcp35_632*C34);
    RLcp35_131 = ROcp35_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp35_231 = ROcp35_26*s.dpt(1,3)+ROcp35_85*s.dpt(3,3);
    RLcp35_331 = ROcp35_36*s.dpt(1,3)+ROcp35_95*s.dpt(3,3);
    OMcp35_131 = OMcp35_16+ROcp35_46*qd(31);
    OMcp35_231 = OMcp35_26+ROcp35_56*qd(31);
    OMcp35_331 = OMcp35_36+ROcp35_66*qd(31);
    ORcp35_131 = OMcp35_26*RLcp35_331-OMcp35_36*RLcp35_231;
    ORcp35_231 = -(OMcp35_16*RLcp35_331-OMcp35_36*RLcp35_131);
    ORcp35_331 = OMcp35_16*RLcp35_231-OMcp35_26*RLcp35_131;
    OMcp35_132 = OMcp35_131+ROcp35_131*qd(32);
    OMcp35_232 = OMcp35_231+ROcp35_231*qd(32);
    OMcp35_332 = OMcp35_331+ROcp35_331*qd(32);
    OMcp35_133 = OMcp35_132+ROcp35_432*qd(33);
    OMcp35_233 = OMcp35_232+ROcp35_532*qd(33);
    OMcp35_333 = OMcp35_332+ROcp35_632*qd(33);
    OPcp35_133 = OPcp35_16+ROcp35_131*qdd(32)+ROcp35_432*qdd(33)+ROcp35_46*qdd(31)+qd(31)*(OMcp35_26*ROcp35_66-OMcp35_36*ROcp35_56)+qd(32)*(...
 OMcp35_231*ROcp35_331-OMcp35_331*ROcp35_231)+qd(33)*(OMcp35_232*ROcp35_632-OMcp35_332*ROcp35_532);
    OPcp35_233 = OPcp35_26+ROcp35_231*qdd(32)+ROcp35_532*qdd(33)+ROcp35_56*qdd(31)-qd(31)*(OMcp35_16*ROcp35_66-OMcp35_36*ROcp35_46)-qd(32)*(...
 OMcp35_131*ROcp35_331-OMcp35_331*ROcp35_131)-qd(33)*(OMcp35_132*ROcp35_632-OMcp35_332*ROcp35_432);
    OPcp35_333 = OPcp35_36+ROcp35_331*qdd(32)+ROcp35_632*qdd(33)+ROcp35_66*qdd(31)+qd(31)*(OMcp35_16*ROcp35_56-OMcp35_26*ROcp35_46)+qd(32)*(...
 OMcp35_131*ROcp35_231-OMcp35_231*ROcp35_131)+qd(33)*(OMcp35_132*ROcp35_532-OMcp35_232*ROcp35_432);
    RLcp35_134 = ROcp35_733*s.dpt(3,21);
    RLcp35_234 = ROcp35_833*s.dpt(3,21);
    RLcp35_334 = ROcp35_933*s.dpt(3,21);
    OMcp35_134 = OMcp35_133+ROcp35_733*qd(34);
    OMcp35_234 = OMcp35_233+ROcp35_833*qd(34);
    OMcp35_334 = OMcp35_333+ROcp35_933*qd(34);
    ORcp35_134 = OMcp35_233*RLcp35_334-OMcp35_333*RLcp35_234;
    ORcp35_234 = -(OMcp35_133*RLcp35_334-OMcp35_333*RLcp35_134);
    ORcp35_334 = OMcp35_133*RLcp35_234-OMcp35_233*RLcp35_134;
    OPcp35_134 = OPcp35_133+ROcp35_733*qdd(34)+qd(34)*(OMcp35_233*ROcp35_933-OMcp35_333*ROcp35_833);
    OPcp35_234 = OPcp35_233+ROcp35_833*qdd(34)-qd(34)*(OMcp35_133*ROcp35_933-OMcp35_333*ROcp35_733);
    OPcp35_334 = OPcp35_333+ROcp35_933*qdd(34)+qd(34)*(OMcp35_133*ROcp35_833-OMcp35_233*ROcp35_733);

% = = Block_1_0_0_36_0_5 = = 
 
% Sensor Kinematics 


    ROcp35_435 = ROcp35_434*C35+ROcp35_733*S35;
    ROcp35_535 = ROcp35_534*C35+ROcp35_833*S35;
    ROcp35_635 = ROcp35_634*C35+ROcp35_933*S35;
    ROcp35_735 = -(ROcp35_434*S35-ROcp35_733*C35);
    ROcp35_835 = -(ROcp35_534*S35-ROcp35_833*C35);
    ROcp35_935 = -(ROcp35_634*S35-ROcp35_933*C35);
    ROcp35_136 = ROcp35_134*C36+ROcp35_435*S36;
    ROcp35_236 = ROcp35_234*C36+ROcp35_535*S36;
    ROcp35_336 = ROcp35_334*C36+ROcp35_635*S36;
    ROcp35_436 = -(ROcp35_134*S36-ROcp35_435*C36);
    ROcp35_536 = -(ROcp35_234*S36-ROcp35_535*C36);
    ROcp35_636 = -(ROcp35_334*S36-ROcp35_635*C36);
    RLcp35_135 = ROcp35_134*s.dpt(1,23)+ROcp35_434*s.dpt(2,23)+ROcp35_733*s.dpt(3,23);
    RLcp35_235 = ROcp35_234*s.dpt(1,23)+ROcp35_534*s.dpt(2,23)+ROcp35_833*s.dpt(3,23);
    RLcp35_335 = ROcp35_334*s.dpt(1,23)+ROcp35_634*s.dpt(2,23)+ROcp35_933*s.dpt(3,23);
    POcp35_135 = RLcp35_131+RLcp35_134+RLcp35_135+q(1);
    POcp35_235 = RLcp35_231+RLcp35_234+RLcp35_235+q(2);
    POcp35_335 = RLcp35_331+RLcp35_334+RLcp35_335+q(3);
    ORcp35_135 = OMcp35_234*RLcp35_335-OMcp35_334*RLcp35_235;
    ORcp35_235 = -(OMcp35_134*RLcp35_335-OMcp35_334*RLcp35_135);
    ORcp35_335 = OMcp35_134*RLcp35_235-OMcp35_234*RLcp35_135;
    VIcp35_135 = ORcp35_131+ORcp35_134+ORcp35_135+qd(1);
    VIcp35_235 = ORcp35_231+ORcp35_234+ORcp35_235+qd(2);
    VIcp35_335 = ORcp35_331+ORcp35_334+ORcp35_335+qd(3);
    ACcp35_135 = qdd(1)+OMcp35_233*ORcp35_334+OMcp35_234*ORcp35_335+OMcp35_26*ORcp35_331-OMcp35_333*ORcp35_234-OMcp35_334*ORcp35_235-OMcp35_36*...
 ORcp35_231+OPcp35_233*RLcp35_334+OPcp35_234*RLcp35_335+OPcp35_26*RLcp35_331-OPcp35_333*RLcp35_234-OPcp35_334*RLcp35_235-OPcp35_36*RLcp35_231;
    ACcp35_235 = qdd(2)-OMcp35_133*ORcp35_334-OMcp35_134*ORcp35_335-OMcp35_16*ORcp35_331+OMcp35_333*ORcp35_134+OMcp35_334*ORcp35_135+OMcp35_36*...
 ORcp35_131-OPcp35_133*RLcp35_334-OPcp35_134*RLcp35_335-OPcp35_16*RLcp35_331+OPcp35_333*RLcp35_134+OPcp35_334*RLcp35_135+OPcp35_36*RLcp35_131;
    ACcp35_335 = qdd(3)+OMcp35_133*ORcp35_234+OMcp35_134*ORcp35_235+OMcp35_16*ORcp35_231-OMcp35_233*ORcp35_134-OMcp35_234*ORcp35_135-OMcp35_26*...
 ORcp35_131+OPcp35_133*RLcp35_234+OPcp35_134*RLcp35_235+OPcp35_16*RLcp35_231-OPcp35_233*RLcp35_134-OPcp35_234*RLcp35_135-OPcp35_26*RLcp35_131;

% = = Block_1_0_0_36_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp35_135;
    sens.P(2) = POcp35_235;
    sens.P(3) = POcp35_335;
    sens.R(1,1) = ROcp35_136;
    sens.R(1,2) = ROcp35_236;
    sens.R(1,3) = ROcp35_336;
    sens.R(2,1) = ROcp35_436;
    sens.R(2,2) = ROcp35_536;
    sens.R(2,3) = ROcp35_636;
    sens.R(3,1) = ROcp35_735;
    sens.R(3,2) = ROcp35_835;
    sens.R(3,3) = ROcp35_935;
    sens.V(1) = VIcp35_135;
    sens.V(2) = VIcp35_235;
    sens.V(3) = VIcp35_335;
    sens.OM(1) = OMcp35_134;
    sens.OM(2) = OMcp35_234;
    sens.OM(3) = OMcp35_334;
    sens.A(1) = ACcp35_135;
    sens.A(2) = ACcp35_235;
    sens.A(3) = ACcp35_335;
    sens.OMP(1) = OPcp35_134;
    sens.OMP(2) = OPcp35_234;
    sens.OMP(3) = OPcp35_334;
 
% 
case 37, 


% = = Block_1_0_0_37_0_1 = = 
 
% Sensor Kinematics 


    ROcp36_25 = S4*S5;
    ROcp36_35 = -C4*S5;
    ROcp36_85 = -S4*C5;
    ROcp36_95 = C4*C5;
    ROcp36_16 = C5*C6;
    ROcp36_26 = ROcp36_25*C6+C4*S6;
    ROcp36_36 = ROcp36_35*C6+S4*S6;
    ROcp36_46 = -C5*S6;
    ROcp36_56 = -(ROcp36_25*S6-C4*C6);
    ROcp36_66 = -(ROcp36_35*S6-S4*C6);
    OMcp36_25 = qd(5)*C4;
    OMcp36_35 = qd(5)*S4;
    OMcp36_16 = qd(4)+qd(6)*S5;
    OMcp36_26 = OMcp36_25+ROcp36_85*qd(6);
    OMcp36_36 = OMcp36_35+ROcp36_95*qd(6);
    OPcp36_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp36_26 = ROcp36_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp36_35*S5-ROcp36_95*qd(4));
    OPcp36_36 = ROcp36_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp36_25*S5-ROcp36_85*qd(4));

% = = Block_1_0_0_37_0_4 = = 
 
% Sensor Kinematics 


    ROcp36_131 = ROcp36_16*C31-S31*S5;
    ROcp36_231 = ROcp36_26*C31-ROcp36_85*S31;
    ROcp36_331 = ROcp36_36*C31-ROcp36_95*S31;
    ROcp36_731 = ROcp36_16*S31+C31*S5;
    ROcp36_831 = ROcp36_26*S31+ROcp36_85*C31;
    ROcp36_931 = ROcp36_36*S31+ROcp36_95*C31;
    ROcp36_432 = ROcp36_46*C32+ROcp36_731*S32;
    ROcp36_532 = ROcp36_56*C32+ROcp36_831*S32;
    ROcp36_632 = ROcp36_66*C32+ROcp36_931*S32;
    ROcp36_732 = -(ROcp36_46*S32-ROcp36_731*C32);
    ROcp36_832 = -(ROcp36_56*S32-ROcp36_831*C32);
    ROcp36_932 = -(ROcp36_66*S32-ROcp36_931*C32);
    ROcp36_133 = ROcp36_131*C33-ROcp36_732*S33;
    ROcp36_233 = ROcp36_231*C33-ROcp36_832*S33;
    ROcp36_333 = ROcp36_331*C33-ROcp36_932*S33;
    ROcp36_733 = ROcp36_131*S33+ROcp36_732*C33;
    ROcp36_833 = ROcp36_231*S33+ROcp36_832*C33;
    ROcp36_933 = ROcp36_331*S33+ROcp36_932*C33;
    ROcp36_134 = ROcp36_133*C34+ROcp36_432*S34;
    ROcp36_234 = ROcp36_233*C34+ROcp36_532*S34;
    ROcp36_334 = ROcp36_333*C34+ROcp36_632*S34;
    ROcp36_434 = -(ROcp36_133*S34-ROcp36_432*C34);
    ROcp36_534 = -(ROcp36_233*S34-ROcp36_532*C34);
    ROcp36_634 = -(ROcp36_333*S34-ROcp36_632*C34);
    RLcp36_131 = ROcp36_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp36_231 = ROcp36_26*s.dpt(1,3)+ROcp36_85*s.dpt(3,3);
    RLcp36_331 = ROcp36_36*s.dpt(1,3)+ROcp36_95*s.dpt(3,3);
    OMcp36_131 = OMcp36_16+ROcp36_46*qd(31);
    OMcp36_231 = OMcp36_26+ROcp36_56*qd(31);
    OMcp36_331 = OMcp36_36+ROcp36_66*qd(31);
    ORcp36_131 = OMcp36_26*RLcp36_331-OMcp36_36*RLcp36_231;
    ORcp36_231 = -(OMcp36_16*RLcp36_331-OMcp36_36*RLcp36_131);
    ORcp36_331 = OMcp36_16*RLcp36_231-OMcp36_26*RLcp36_131;
    OMcp36_132 = OMcp36_131+ROcp36_131*qd(32);
    OMcp36_232 = OMcp36_231+ROcp36_231*qd(32);
    OMcp36_332 = OMcp36_331+ROcp36_331*qd(32);
    OMcp36_133 = OMcp36_132+ROcp36_432*qd(33);
    OMcp36_233 = OMcp36_232+ROcp36_532*qd(33);
    OMcp36_333 = OMcp36_332+ROcp36_632*qd(33);
    OPcp36_133 = OPcp36_16+ROcp36_131*qdd(32)+ROcp36_432*qdd(33)+ROcp36_46*qdd(31)+qd(31)*(OMcp36_26*ROcp36_66-OMcp36_36*ROcp36_56)+qd(32)*(...
 OMcp36_231*ROcp36_331-OMcp36_331*ROcp36_231)+qd(33)*(OMcp36_232*ROcp36_632-OMcp36_332*ROcp36_532);
    OPcp36_233 = OPcp36_26+ROcp36_231*qdd(32)+ROcp36_532*qdd(33)+ROcp36_56*qdd(31)-qd(31)*(OMcp36_16*ROcp36_66-OMcp36_36*ROcp36_46)-qd(32)*(...
 OMcp36_131*ROcp36_331-OMcp36_331*ROcp36_131)-qd(33)*(OMcp36_132*ROcp36_632-OMcp36_332*ROcp36_432);
    OPcp36_333 = OPcp36_36+ROcp36_331*qdd(32)+ROcp36_632*qdd(33)+ROcp36_66*qdd(31)+qd(31)*(OMcp36_16*ROcp36_56-OMcp36_26*ROcp36_46)+qd(32)*(...
 OMcp36_131*ROcp36_231-OMcp36_231*ROcp36_131)+qd(33)*(OMcp36_132*ROcp36_532-OMcp36_232*ROcp36_432);
    RLcp36_134 = ROcp36_733*s.dpt(3,21);
    RLcp36_234 = ROcp36_833*s.dpt(3,21);
    RLcp36_334 = ROcp36_933*s.dpt(3,21);
    OMcp36_134 = OMcp36_133+ROcp36_733*qd(34);
    OMcp36_234 = OMcp36_233+ROcp36_833*qd(34);
    OMcp36_334 = OMcp36_333+ROcp36_933*qd(34);
    ORcp36_134 = OMcp36_233*RLcp36_334-OMcp36_333*RLcp36_234;
    ORcp36_234 = -(OMcp36_133*RLcp36_334-OMcp36_333*RLcp36_134);
    ORcp36_334 = OMcp36_133*RLcp36_234-OMcp36_233*RLcp36_134;
    OPcp36_134 = OPcp36_133+ROcp36_733*qdd(34)+qd(34)*(OMcp36_233*ROcp36_933-OMcp36_333*ROcp36_833);
    OPcp36_234 = OPcp36_233+ROcp36_833*qdd(34)-qd(34)*(OMcp36_133*ROcp36_933-OMcp36_333*ROcp36_733);
    OPcp36_334 = OPcp36_333+ROcp36_933*qdd(34)+qd(34)*(OMcp36_133*ROcp36_833-OMcp36_233*ROcp36_733);

% = = Block_1_0_0_37_0_5 = = 
 
% Sensor Kinematics 


    ROcp36_435 = ROcp36_434*C35+ROcp36_733*S35;
    ROcp36_535 = ROcp36_534*C35+ROcp36_833*S35;
    ROcp36_635 = ROcp36_634*C35+ROcp36_933*S35;
    ROcp36_735 = -(ROcp36_434*S35-ROcp36_733*C35);
    ROcp36_835 = -(ROcp36_534*S35-ROcp36_833*C35);
    ROcp36_935 = -(ROcp36_634*S35-ROcp36_933*C35);
    ROcp36_136 = ROcp36_134*C36+ROcp36_435*S36;
    ROcp36_236 = ROcp36_234*C36+ROcp36_535*S36;
    ROcp36_336 = ROcp36_334*C36+ROcp36_635*S36;
    ROcp36_436 = -(ROcp36_134*S36-ROcp36_435*C36);
    ROcp36_536 = -(ROcp36_234*S36-ROcp36_535*C36);
    ROcp36_636 = -(ROcp36_334*S36-ROcp36_635*C36);
    ROcp36_137 = ROcp36_136*C37-ROcp36_735*S37;
    ROcp36_237 = ROcp36_236*C37-ROcp36_835*S37;
    ROcp36_337 = ROcp36_336*C37-ROcp36_935*S37;
    ROcp36_737 = ROcp36_136*S37+ROcp36_735*C37;
    ROcp36_837 = ROcp36_236*S37+ROcp36_835*C37;
    ROcp36_937 = ROcp36_336*S37+ROcp36_935*C37;
    RLcp36_135 = ROcp36_134*s.dpt(1,23)+ROcp36_434*s.dpt(2,23)+ROcp36_733*s.dpt(3,23);
    RLcp36_235 = ROcp36_234*s.dpt(1,23)+ROcp36_534*s.dpt(2,23)+ROcp36_833*s.dpt(3,23);
    RLcp36_335 = ROcp36_334*s.dpt(1,23)+ROcp36_634*s.dpt(2,23)+ROcp36_933*s.dpt(3,23);
    POcp36_135 = RLcp36_131+RLcp36_134+RLcp36_135+q(1);
    POcp36_235 = RLcp36_231+RLcp36_234+RLcp36_235+q(2);
    POcp36_335 = RLcp36_331+RLcp36_334+RLcp36_335+q(3);
    ORcp36_135 = OMcp36_234*RLcp36_335-OMcp36_334*RLcp36_235;
    ORcp36_235 = -(OMcp36_134*RLcp36_335-OMcp36_334*RLcp36_135);
    ORcp36_335 = OMcp36_134*RLcp36_235-OMcp36_234*RLcp36_135;
    VIcp36_135 = ORcp36_131+ORcp36_134+ORcp36_135+qd(1);
    VIcp36_235 = ORcp36_231+ORcp36_234+ORcp36_235+qd(2);
    VIcp36_335 = ORcp36_331+ORcp36_334+ORcp36_335+qd(3);
    ACcp36_135 = qdd(1)+OMcp36_233*ORcp36_334+OMcp36_234*ORcp36_335+OMcp36_26*ORcp36_331-OMcp36_333*ORcp36_234-OMcp36_334*ORcp36_235-OMcp36_36*...
 ORcp36_231+OPcp36_233*RLcp36_334+OPcp36_234*RLcp36_335+OPcp36_26*RLcp36_331-OPcp36_333*RLcp36_234-OPcp36_334*RLcp36_235-OPcp36_36*RLcp36_231;
    ACcp36_235 = qdd(2)-OMcp36_133*ORcp36_334-OMcp36_134*ORcp36_335-OMcp36_16*ORcp36_331+OMcp36_333*ORcp36_134+OMcp36_334*ORcp36_135+OMcp36_36*...
 ORcp36_131-OPcp36_133*RLcp36_334-OPcp36_134*RLcp36_335-OPcp36_16*RLcp36_331+OPcp36_333*RLcp36_134+OPcp36_334*RLcp36_135+OPcp36_36*RLcp36_131;
    ACcp36_335 = qdd(3)+OMcp36_133*ORcp36_234+OMcp36_134*ORcp36_235+OMcp36_16*ORcp36_231-OMcp36_233*ORcp36_134-OMcp36_234*ORcp36_135-OMcp36_26*...
 ORcp36_131+OPcp36_133*RLcp36_234+OPcp36_134*RLcp36_235+OPcp36_16*RLcp36_231-OPcp36_233*RLcp36_134-OPcp36_234*RLcp36_135-OPcp36_26*RLcp36_131;
    OMcp36_137 = OMcp36_134+ROcp36_436*qd(37);
    OMcp36_237 = OMcp36_234+ROcp36_536*qd(37);
    OMcp36_337 = OMcp36_334+ROcp36_636*qd(37);
    OPcp36_137 = OPcp36_134+ROcp36_436*qdd(37)+qd(37)*(OMcp36_234*ROcp36_636-OMcp36_334*ROcp36_536);
    OPcp36_237 = OPcp36_234+ROcp36_536*qdd(37)-qd(37)*(OMcp36_134*ROcp36_636-OMcp36_334*ROcp36_436);
    OPcp36_337 = OPcp36_334+ROcp36_636*qdd(37)+qd(37)*(OMcp36_134*ROcp36_536-OMcp36_234*ROcp36_436);

% = = Block_1_0_0_37_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp36_135;
    sens.P(2) = POcp36_235;
    sens.P(3) = POcp36_335;
    sens.R(1,1) = ROcp36_137;
    sens.R(1,2) = ROcp36_237;
    sens.R(1,3) = ROcp36_337;
    sens.R(2,1) = ROcp36_436;
    sens.R(2,2) = ROcp36_536;
    sens.R(2,3) = ROcp36_636;
    sens.R(3,1) = ROcp36_737;
    sens.R(3,2) = ROcp36_837;
    sens.R(3,3) = ROcp36_937;
    sens.V(1) = VIcp36_135;
    sens.V(2) = VIcp36_235;
    sens.V(3) = VIcp36_335;
    sens.OM(1) = OMcp36_137;
    sens.OM(2) = OMcp36_237;
    sens.OM(3) = OMcp36_337;
    sens.A(1) = ACcp36_135;
    sens.A(2) = ACcp36_235;
    sens.A(3) = ACcp36_335;
    sens.OMP(1) = OPcp36_137;
    sens.OMP(2) = OPcp36_237;
    sens.OMP(3) = OPcp36_337;
 
% 
case 38, 


% = = Block_1_0_0_38_0_1 = = 
 
% Sensor Kinematics 


    ROcp37_25 = S4*S5;
    ROcp37_35 = -C4*S5;
    ROcp37_85 = -S4*C5;
    ROcp37_95 = C4*C5;
    ROcp37_16 = C5*C6;
    ROcp37_26 = ROcp37_25*C6+C4*S6;
    ROcp37_36 = ROcp37_35*C6+S4*S6;
    ROcp37_46 = -C5*S6;
    ROcp37_56 = -(ROcp37_25*S6-C4*C6);
    ROcp37_66 = -(ROcp37_35*S6-S4*C6);
    OMcp37_25 = qd(5)*C4;
    OMcp37_35 = qd(5)*S4;
    OMcp37_16 = qd(4)+qd(6)*S5;
    OMcp37_26 = OMcp37_25+ROcp37_85*qd(6);
    OMcp37_36 = OMcp37_35+ROcp37_95*qd(6);
    OPcp37_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp37_26 = ROcp37_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp37_35*S5-ROcp37_95*qd(4));
    OPcp37_36 = ROcp37_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp37_25*S5-ROcp37_85*qd(4));

% = = Block_1_0_0_38_0_4 = = 
 
% Sensor Kinematics 


    ROcp37_131 = ROcp37_16*C31-S31*S5;
    ROcp37_231 = ROcp37_26*C31-ROcp37_85*S31;
    ROcp37_331 = ROcp37_36*C31-ROcp37_95*S31;
    ROcp37_731 = ROcp37_16*S31+C31*S5;
    ROcp37_831 = ROcp37_26*S31+ROcp37_85*C31;
    ROcp37_931 = ROcp37_36*S31+ROcp37_95*C31;
    ROcp37_432 = ROcp37_46*C32+ROcp37_731*S32;
    ROcp37_532 = ROcp37_56*C32+ROcp37_831*S32;
    ROcp37_632 = ROcp37_66*C32+ROcp37_931*S32;
    ROcp37_732 = -(ROcp37_46*S32-ROcp37_731*C32);
    ROcp37_832 = -(ROcp37_56*S32-ROcp37_831*C32);
    ROcp37_932 = -(ROcp37_66*S32-ROcp37_931*C32);
    ROcp37_133 = ROcp37_131*C33-ROcp37_732*S33;
    ROcp37_233 = ROcp37_231*C33-ROcp37_832*S33;
    ROcp37_333 = ROcp37_331*C33-ROcp37_932*S33;
    ROcp37_733 = ROcp37_131*S33+ROcp37_732*C33;
    ROcp37_833 = ROcp37_231*S33+ROcp37_832*C33;
    ROcp37_933 = ROcp37_331*S33+ROcp37_932*C33;
    ROcp37_134 = ROcp37_133*C34+ROcp37_432*S34;
    ROcp37_234 = ROcp37_233*C34+ROcp37_532*S34;
    ROcp37_334 = ROcp37_333*C34+ROcp37_632*S34;
    ROcp37_434 = -(ROcp37_133*S34-ROcp37_432*C34);
    ROcp37_534 = -(ROcp37_233*S34-ROcp37_532*C34);
    ROcp37_634 = -(ROcp37_333*S34-ROcp37_632*C34);
    RLcp37_131 = ROcp37_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp37_231 = ROcp37_26*s.dpt(1,3)+ROcp37_85*s.dpt(3,3);
    RLcp37_331 = ROcp37_36*s.dpt(1,3)+ROcp37_95*s.dpt(3,3);
    OMcp37_131 = OMcp37_16+ROcp37_46*qd(31);
    OMcp37_231 = OMcp37_26+ROcp37_56*qd(31);
    OMcp37_331 = OMcp37_36+ROcp37_66*qd(31);
    ORcp37_131 = OMcp37_26*RLcp37_331-OMcp37_36*RLcp37_231;
    ORcp37_231 = -(OMcp37_16*RLcp37_331-OMcp37_36*RLcp37_131);
    ORcp37_331 = OMcp37_16*RLcp37_231-OMcp37_26*RLcp37_131;
    OMcp37_132 = OMcp37_131+ROcp37_131*qd(32);
    OMcp37_232 = OMcp37_231+ROcp37_231*qd(32);
    OMcp37_332 = OMcp37_331+ROcp37_331*qd(32);
    OMcp37_133 = OMcp37_132+ROcp37_432*qd(33);
    OMcp37_233 = OMcp37_232+ROcp37_532*qd(33);
    OMcp37_333 = OMcp37_332+ROcp37_632*qd(33);
    OPcp37_133 = OPcp37_16+ROcp37_131*qdd(32)+ROcp37_432*qdd(33)+ROcp37_46*qdd(31)+qd(31)*(OMcp37_26*ROcp37_66-OMcp37_36*ROcp37_56)+qd(32)*(...
 OMcp37_231*ROcp37_331-OMcp37_331*ROcp37_231)+qd(33)*(OMcp37_232*ROcp37_632-OMcp37_332*ROcp37_532);
    OPcp37_233 = OPcp37_26+ROcp37_231*qdd(32)+ROcp37_532*qdd(33)+ROcp37_56*qdd(31)-qd(31)*(OMcp37_16*ROcp37_66-OMcp37_36*ROcp37_46)-qd(32)*(...
 OMcp37_131*ROcp37_331-OMcp37_331*ROcp37_131)-qd(33)*(OMcp37_132*ROcp37_632-OMcp37_332*ROcp37_432);
    OPcp37_333 = OPcp37_36+ROcp37_331*qdd(32)+ROcp37_632*qdd(33)+ROcp37_66*qdd(31)+qd(31)*(OMcp37_16*ROcp37_56-OMcp37_26*ROcp37_46)+qd(32)*(...
 OMcp37_131*ROcp37_231-OMcp37_231*ROcp37_131)+qd(33)*(OMcp37_132*ROcp37_532-OMcp37_232*ROcp37_432);
    RLcp37_134 = ROcp37_733*s.dpt(3,21);
    RLcp37_234 = ROcp37_833*s.dpt(3,21);
    RLcp37_334 = ROcp37_933*s.dpt(3,21);
    OMcp37_134 = OMcp37_133+ROcp37_733*qd(34);
    OMcp37_234 = OMcp37_233+ROcp37_833*qd(34);
    OMcp37_334 = OMcp37_333+ROcp37_933*qd(34);
    ORcp37_134 = OMcp37_233*RLcp37_334-OMcp37_333*RLcp37_234;
    ORcp37_234 = -(OMcp37_133*RLcp37_334-OMcp37_333*RLcp37_134);
    ORcp37_334 = OMcp37_133*RLcp37_234-OMcp37_233*RLcp37_134;
    OPcp37_134 = OPcp37_133+ROcp37_733*qdd(34)+qd(34)*(OMcp37_233*ROcp37_933-OMcp37_333*ROcp37_833);
    OPcp37_234 = OPcp37_233+ROcp37_833*qdd(34)-qd(34)*(OMcp37_133*ROcp37_933-OMcp37_333*ROcp37_733);
    OPcp37_334 = OPcp37_333+ROcp37_933*qdd(34)+qd(34)*(OMcp37_133*ROcp37_833-OMcp37_233*ROcp37_733);

% = = Block_1_0_0_38_0_5 = = 
 
% Sensor Kinematics 


    ROcp37_435 = ROcp37_434*C35+ROcp37_733*S35;
    ROcp37_535 = ROcp37_534*C35+ROcp37_833*S35;
    ROcp37_635 = ROcp37_634*C35+ROcp37_933*S35;
    ROcp37_735 = -(ROcp37_434*S35-ROcp37_733*C35);
    ROcp37_835 = -(ROcp37_534*S35-ROcp37_833*C35);
    ROcp37_935 = -(ROcp37_634*S35-ROcp37_933*C35);
    ROcp37_136 = ROcp37_134*C36+ROcp37_435*S36;
    ROcp37_236 = ROcp37_234*C36+ROcp37_535*S36;
    ROcp37_336 = ROcp37_334*C36+ROcp37_635*S36;
    ROcp37_436 = -(ROcp37_134*S36-ROcp37_435*C36);
    ROcp37_536 = -(ROcp37_234*S36-ROcp37_535*C36);
    ROcp37_636 = -(ROcp37_334*S36-ROcp37_635*C36);
    ROcp37_137 = ROcp37_136*C37-ROcp37_735*S37;
    ROcp37_237 = ROcp37_236*C37-ROcp37_835*S37;
    ROcp37_337 = ROcp37_336*C37-ROcp37_935*S37;
    ROcp37_737 = ROcp37_136*S37+ROcp37_735*C37;
    ROcp37_837 = ROcp37_236*S37+ROcp37_835*C37;
    ROcp37_937 = ROcp37_336*S37+ROcp37_935*C37;
    ROcp37_438 = ROcp37_436*C38+ROcp37_737*S38;
    ROcp37_538 = ROcp37_536*C38+ROcp37_837*S38;
    ROcp37_638 = ROcp37_636*C38+ROcp37_937*S38;
    ROcp37_738 = -(ROcp37_436*S38-ROcp37_737*C38);
    ROcp37_838 = -(ROcp37_536*S38-ROcp37_837*C38);
    ROcp37_938 = -(ROcp37_636*S38-ROcp37_937*C38);
    RLcp37_135 = ROcp37_134*s.dpt(1,23)+ROcp37_434*s.dpt(2,23)+ROcp37_733*s.dpt(3,23);
    RLcp37_235 = ROcp37_234*s.dpt(1,23)+ROcp37_534*s.dpt(2,23)+ROcp37_833*s.dpt(3,23);
    RLcp37_335 = ROcp37_334*s.dpt(1,23)+ROcp37_634*s.dpt(2,23)+ROcp37_933*s.dpt(3,23);
    ORcp37_135 = OMcp37_234*RLcp37_335-OMcp37_334*RLcp37_235;
    ORcp37_235 = -(OMcp37_134*RLcp37_335-OMcp37_334*RLcp37_135);
    ORcp37_335 = OMcp37_134*RLcp37_235-OMcp37_234*RLcp37_135;
    OMcp37_137 = OMcp37_134+ROcp37_436*qd(37);
    OMcp37_237 = OMcp37_234+ROcp37_536*qd(37);
    OMcp37_337 = OMcp37_334+ROcp37_636*qd(37);
    OPcp37_137 = OPcp37_134+ROcp37_436*qdd(37)+qd(37)*(OMcp37_234*ROcp37_636-OMcp37_334*ROcp37_536);
    OPcp37_237 = OPcp37_234+ROcp37_536*qdd(37)-qd(37)*(OMcp37_134*ROcp37_636-OMcp37_334*ROcp37_436);
    OPcp37_337 = OPcp37_334+ROcp37_636*qdd(37)+qd(37)*(OMcp37_134*ROcp37_536-OMcp37_234*ROcp37_436);
    RLcp37_138 = ROcp37_436*s.dpt(2,27)+ROcp37_737*s.dpt(3,27);
    RLcp37_238 = ROcp37_536*s.dpt(2,27)+ROcp37_837*s.dpt(3,27);
    RLcp37_338 = ROcp37_636*s.dpt(2,27)+ROcp37_937*s.dpt(3,27);
    POcp37_138 = RLcp37_131+RLcp37_134+RLcp37_135+RLcp37_138+q(1);
    POcp37_238 = RLcp37_231+RLcp37_234+RLcp37_235+RLcp37_238+q(2);
    POcp37_338 = RLcp37_331+RLcp37_334+RLcp37_335+RLcp37_338+q(3);
    OMcp37_138 = OMcp37_137+ROcp37_137*qd(38);
    OMcp37_238 = OMcp37_237+ROcp37_237*qd(38);
    OMcp37_338 = OMcp37_337+ROcp37_337*qd(38);
    ORcp37_138 = OMcp37_237*RLcp37_338-OMcp37_337*RLcp37_238;
    ORcp37_238 = -(OMcp37_137*RLcp37_338-OMcp37_337*RLcp37_138);
    ORcp37_338 = OMcp37_137*RLcp37_238-OMcp37_237*RLcp37_138;
    VIcp37_138 = ORcp37_131+ORcp37_134+ORcp37_135+ORcp37_138+qd(1);
    VIcp37_238 = ORcp37_231+ORcp37_234+ORcp37_235+ORcp37_238+qd(2);
    VIcp37_338 = ORcp37_331+ORcp37_334+ORcp37_335+ORcp37_338+qd(3);
    OPcp37_138 = OPcp37_137+ROcp37_137*qdd(38)+qd(38)*(OMcp37_237*ROcp37_337-OMcp37_337*ROcp37_237);
    OPcp37_238 = OPcp37_237+ROcp37_237*qdd(38)-qd(38)*(OMcp37_137*ROcp37_337-OMcp37_337*ROcp37_137);
    OPcp37_338 = OPcp37_337+ROcp37_337*qdd(38)+qd(38)*(OMcp37_137*ROcp37_237-OMcp37_237*ROcp37_137);
    ACcp37_138 = qdd(1)+OMcp37_233*ORcp37_334+OMcp37_234*ORcp37_335+OMcp37_237*ORcp37_338+OMcp37_26*ORcp37_331-OMcp37_333*ORcp37_234-OMcp37_334*...
 ORcp37_235-OMcp37_337*ORcp37_238-OMcp37_36*ORcp37_231+OPcp37_233*RLcp37_334+OPcp37_234*RLcp37_335+OPcp37_237*RLcp37_338+OPcp37_26*RLcp37_331-...
 OPcp37_333*RLcp37_234-OPcp37_334*RLcp37_235-OPcp37_337*RLcp37_238-OPcp37_36*RLcp37_231;
    ACcp37_238 = qdd(2)-OMcp37_133*ORcp37_334-OMcp37_134*ORcp37_335-OMcp37_137*ORcp37_338-OMcp37_16*ORcp37_331+OMcp37_333*ORcp37_134+OMcp37_334*...
 ORcp37_135+OMcp37_337*ORcp37_138+OMcp37_36*ORcp37_131-OPcp37_133*RLcp37_334-OPcp37_134*RLcp37_335-OPcp37_137*RLcp37_338-OPcp37_16*RLcp37_331+...
 OPcp37_333*RLcp37_134+OPcp37_334*RLcp37_135+OPcp37_337*RLcp37_138+OPcp37_36*RLcp37_131;
    ACcp37_338 = qdd(3)+OMcp37_133*ORcp37_234+OMcp37_134*ORcp37_235+OMcp37_137*ORcp37_238+OMcp37_16*ORcp37_231-OMcp37_233*ORcp37_134-OMcp37_234*...
 ORcp37_135-OMcp37_237*ORcp37_138-OMcp37_26*ORcp37_131+OPcp37_133*RLcp37_234+OPcp37_134*RLcp37_235+OPcp37_137*RLcp37_238+OPcp37_16*RLcp37_231-...
 OPcp37_233*RLcp37_134-OPcp37_234*RLcp37_135-OPcp37_237*RLcp37_138-OPcp37_26*RLcp37_131;

% = = Block_1_0_0_38_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp37_138;
    sens.P(2) = POcp37_238;
    sens.P(3) = POcp37_338;
    sens.R(1,1) = ROcp37_137;
    sens.R(1,2) = ROcp37_237;
    sens.R(1,3) = ROcp37_337;
    sens.R(2,1) = ROcp37_438;
    sens.R(2,2) = ROcp37_538;
    sens.R(2,3) = ROcp37_638;
    sens.R(3,1) = ROcp37_738;
    sens.R(3,2) = ROcp37_838;
    sens.R(3,3) = ROcp37_938;
    sens.V(1) = VIcp37_138;
    sens.V(2) = VIcp37_238;
    sens.V(3) = VIcp37_338;
    sens.OM(1) = OMcp37_138;
    sens.OM(2) = OMcp37_238;
    sens.OM(3) = OMcp37_338;
    sens.A(1) = ACcp37_138;
    sens.A(2) = ACcp37_238;
    sens.A(3) = ACcp37_338;
    sens.OMP(1) = OPcp37_138;
    sens.OMP(2) = OPcp37_238;
    sens.OMP(3) = OPcp37_338;
 
% 
case 39, 


% = = Block_1_0_0_39_0_1 = = 
 
% Sensor Kinematics 


    ROcp38_25 = S4*S5;
    ROcp38_35 = -C4*S5;
    ROcp38_85 = -S4*C5;
    ROcp38_95 = C4*C5;
    ROcp38_16 = C5*C6;
    ROcp38_26 = ROcp38_25*C6+C4*S6;
    ROcp38_36 = ROcp38_35*C6+S4*S6;
    ROcp38_46 = -C5*S6;
    ROcp38_56 = -(ROcp38_25*S6-C4*C6);
    ROcp38_66 = -(ROcp38_35*S6-S4*C6);
    OMcp38_25 = qd(5)*C4;
    OMcp38_35 = qd(5)*S4;
    OMcp38_16 = qd(4)+qd(6)*S5;
    OMcp38_26 = OMcp38_25+ROcp38_85*qd(6);
    OMcp38_36 = OMcp38_35+ROcp38_95*qd(6);
    OPcp38_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp38_26 = ROcp38_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp38_35*S5-ROcp38_95*qd(4));
    OPcp38_36 = ROcp38_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp38_25*S5-ROcp38_85*qd(4));

% = = Block_1_0_0_39_0_4 = = 
 
% Sensor Kinematics 


    ROcp38_131 = ROcp38_16*C31-S31*S5;
    ROcp38_231 = ROcp38_26*C31-ROcp38_85*S31;
    ROcp38_331 = ROcp38_36*C31-ROcp38_95*S31;
    ROcp38_731 = ROcp38_16*S31+C31*S5;
    ROcp38_831 = ROcp38_26*S31+ROcp38_85*C31;
    ROcp38_931 = ROcp38_36*S31+ROcp38_95*C31;
    ROcp38_432 = ROcp38_46*C32+ROcp38_731*S32;
    ROcp38_532 = ROcp38_56*C32+ROcp38_831*S32;
    ROcp38_632 = ROcp38_66*C32+ROcp38_931*S32;
    ROcp38_732 = -(ROcp38_46*S32-ROcp38_731*C32);
    ROcp38_832 = -(ROcp38_56*S32-ROcp38_831*C32);
    ROcp38_932 = -(ROcp38_66*S32-ROcp38_931*C32);
    ROcp38_133 = ROcp38_131*C33-ROcp38_732*S33;
    ROcp38_233 = ROcp38_231*C33-ROcp38_832*S33;
    ROcp38_333 = ROcp38_331*C33-ROcp38_932*S33;
    ROcp38_733 = ROcp38_131*S33+ROcp38_732*C33;
    ROcp38_833 = ROcp38_231*S33+ROcp38_832*C33;
    ROcp38_933 = ROcp38_331*S33+ROcp38_932*C33;
    ROcp38_134 = ROcp38_133*C34+ROcp38_432*S34;
    ROcp38_234 = ROcp38_233*C34+ROcp38_532*S34;
    ROcp38_334 = ROcp38_333*C34+ROcp38_632*S34;
    ROcp38_434 = -(ROcp38_133*S34-ROcp38_432*C34);
    ROcp38_534 = -(ROcp38_233*S34-ROcp38_532*C34);
    ROcp38_634 = -(ROcp38_333*S34-ROcp38_632*C34);
    RLcp38_131 = ROcp38_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp38_231 = ROcp38_26*s.dpt(1,3)+ROcp38_85*s.dpt(3,3);
    RLcp38_331 = ROcp38_36*s.dpt(1,3)+ROcp38_95*s.dpt(3,3);
    OMcp38_131 = OMcp38_16+ROcp38_46*qd(31);
    OMcp38_231 = OMcp38_26+ROcp38_56*qd(31);
    OMcp38_331 = OMcp38_36+ROcp38_66*qd(31);
    ORcp38_131 = OMcp38_26*RLcp38_331-OMcp38_36*RLcp38_231;
    ORcp38_231 = -(OMcp38_16*RLcp38_331-OMcp38_36*RLcp38_131);
    ORcp38_331 = OMcp38_16*RLcp38_231-OMcp38_26*RLcp38_131;
    OMcp38_132 = OMcp38_131+ROcp38_131*qd(32);
    OMcp38_232 = OMcp38_231+ROcp38_231*qd(32);
    OMcp38_332 = OMcp38_331+ROcp38_331*qd(32);
    OMcp38_133 = OMcp38_132+ROcp38_432*qd(33);
    OMcp38_233 = OMcp38_232+ROcp38_532*qd(33);
    OMcp38_333 = OMcp38_332+ROcp38_632*qd(33);
    OPcp38_133 = OPcp38_16+ROcp38_131*qdd(32)+ROcp38_432*qdd(33)+ROcp38_46*qdd(31)+qd(31)*(OMcp38_26*ROcp38_66-OMcp38_36*ROcp38_56)+qd(32)*(...
 OMcp38_231*ROcp38_331-OMcp38_331*ROcp38_231)+qd(33)*(OMcp38_232*ROcp38_632-OMcp38_332*ROcp38_532);
    OPcp38_233 = OPcp38_26+ROcp38_231*qdd(32)+ROcp38_532*qdd(33)+ROcp38_56*qdd(31)-qd(31)*(OMcp38_16*ROcp38_66-OMcp38_36*ROcp38_46)-qd(32)*(...
 OMcp38_131*ROcp38_331-OMcp38_331*ROcp38_131)-qd(33)*(OMcp38_132*ROcp38_632-OMcp38_332*ROcp38_432);
    OPcp38_333 = OPcp38_36+ROcp38_331*qdd(32)+ROcp38_632*qdd(33)+ROcp38_66*qdd(31)+qd(31)*(OMcp38_16*ROcp38_56-OMcp38_26*ROcp38_46)+qd(32)*(...
 OMcp38_131*ROcp38_231-OMcp38_231*ROcp38_131)+qd(33)*(OMcp38_132*ROcp38_532-OMcp38_232*ROcp38_432);
    RLcp38_134 = ROcp38_733*s.dpt(3,21);
    RLcp38_234 = ROcp38_833*s.dpt(3,21);
    RLcp38_334 = ROcp38_933*s.dpt(3,21);
    OMcp38_134 = OMcp38_133+ROcp38_733*qd(34);
    OMcp38_234 = OMcp38_233+ROcp38_833*qd(34);
    OMcp38_334 = OMcp38_333+ROcp38_933*qd(34);
    ORcp38_134 = OMcp38_233*RLcp38_334-OMcp38_333*RLcp38_234;
    ORcp38_234 = -(OMcp38_133*RLcp38_334-OMcp38_333*RLcp38_134);
    ORcp38_334 = OMcp38_133*RLcp38_234-OMcp38_233*RLcp38_134;
    OPcp38_134 = OPcp38_133+ROcp38_733*qdd(34)+qd(34)*(OMcp38_233*ROcp38_933-OMcp38_333*ROcp38_833);
    OPcp38_234 = OPcp38_233+ROcp38_833*qdd(34)-qd(34)*(OMcp38_133*ROcp38_933-OMcp38_333*ROcp38_733);
    OPcp38_334 = OPcp38_333+ROcp38_933*qdd(34)+qd(34)*(OMcp38_133*ROcp38_833-OMcp38_233*ROcp38_733);

% = = Block_1_0_0_39_0_5 = = 
 
% Sensor Kinematics 


    ROcp38_435 = ROcp38_434*C35+ROcp38_733*S35;
    ROcp38_535 = ROcp38_534*C35+ROcp38_833*S35;
    ROcp38_635 = ROcp38_634*C35+ROcp38_933*S35;
    ROcp38_735 = -(ROcp38_434*S35-ROcp38_733*C35);
    ROcp38_835 = -(ROcp38_534*S35-ROcp38_833*C35);
    ROcp38_935 = -(ROcp38_634*S35-ROcp38_933*C35);
    ROcp38_136 = ROcp38_134*C36+ROcp38_435*S36;
    ROcp38_236 = ROcp38_234*C36+ROcp38_535*S36;
    ROcp38_336 = ROcp38_334*C36+ROcp38_635*S36;
    ROcp38_436 = -(ROcp38_134*S36-ROcp38_435*C36);
    ROcp38_536 = -(ROcp38_234*S36-ROcp38_535*C36);
    ROcp38_636 = -(ROcp38_334*S36-ROcp38_635*C36);
    ROcp38_137 = ROcp38_136*C37-ROcp38_735*S37;
    ROcp38_237 = ROcp38_236*C37-ROcp38_835*S37;
    ROcp38_337 = ROcp38_336*C37-ROcp38_935*S37;
    ROcp38_737 = ROcp38_136*S37+ROcp38_735*C37;
    ROcp38_837 = ROcp38_236*S37+ROcp38_835*C37;
    ROcp38_937 = ROcp38_336*S37+ROcp38_935*C37;
    ROcp38_438 = ROcp38_436*C38+ROcp38_737*S38;
    ROcp38_538 = ROcp38_536*C38+ROcp38_837*S38;
    ROcp38_638 = ROcp38_636*C38+ROcp38_937*S38;
    ROcp38_738 = -(ROcp38_436*S38-ROcp38_737*C38);
    ROcp38_838 = -(ROcp38_536*S38-ROcp38_837*C38);
    ROcp38_938 = -(ROcp38_636*S38-ROcp38_937*C38);
    ROcp38_139 = ROcp38_137*C39+ROcp38_438*S39;
    ROcp38_239 = ROcp38_237*C39+ROcp38_538*S39;
    ROcp38_339 = ROcp38_337*C39+ROcp38_638*S39;
    ROcp38_439 = -(ROcp38_137*S39-ROcp38_438*C39);
    ROcp38_539 = -(ROcp38_237*S39-ROcp38_538*C39);
    ROcp38_639 = -(ROcp38_337*S39-ROcp38_638*C39);
    RLcp38_135 = ROcp38_134*s.dpt(1,23)+ROcp38_434*s.dpt(2,23)+ROcp38_733*s.dpt(3,23);
    RLcp38_235 = ROcp38_234*s.dpt(1,23)+ROcp38_534*s.dpt(2,23)+ROcp38_833*s.dpt(3,23);
    RLcp38_335 = ROcp38_334*s.dpt(1,23)+ROcp38_634*s.dpt(2,23)+ROcp38_933*s.dpt(3,23);
    ORcp38_135 = OMcp38_234*RLcp38_335-OMcp38_334*RLcp38_235;
    ORcp38_235 = -(OMcp38_134*RLcp38_335-OMcp38_334*RLcp38_135);
    ORcp38_335 = OMcp38_134*RLcp38_235-OMcp38_234*RLcp38_135;
    OMcp38_137 = OMcp38_134+ROcp38_436*qd(37);
    OMcp38_237 = OMcp38_234+ROcp38_536*qd(37);
    OMcp38_337 = OMcp38_334+ROcp38_636*qd(37);
    OPcp38_137 = OPcp38_134+ROcp38_436*qdd(37)+qd(37)*(OMcp38_234*ROcp38_636-OMcp38_334*ROcp38_536);
    OPcp38_237 = OPcp38_234+ROcp38_536*qdd(37)-qd(37)*(OMcp38_134*ROcp38_636-OMcp38_334*ROcp38_436);
    OPcp38_337 = OPcp38_334+ROcp38_636*qdd(37)+qd(37)*(OMcp38_134*ROcp38_536-OMcp38_234*ROcp38_436);
    RLcp38_138 = ROcp38_436*s.dpt(2,27)+ROcp38_737*s.dpt(3,27);
    RLcp38_238 = ROcp38_536*s.dpt(2,27)+ROcp38_837*s.dpt(3,27);
    RLcp38_338 = ROcp38_636*s.dpt(2,27)+ROcp38_937*s.dpt(3,27);
    OMcp38_138 = OMcp38_137+ROcp38_137*qd(38);
    OMcp38_238 = OMcp38_237+ROcp38_237*qd(38);
    OMcp38_338 = OMcp38_337+ROcp38_337*qd(38);
    ORcp38_138 = OMcp38_237*RLcp38_338-OMcp38_337*RLcp38_238;
    ORcp38_238 = -(OMcp38_137*RLcp38_338-OMcp38_337*RLcp38_138);
    ORcp38_338 = OMcp38_137*RLcp38_238-OMcp38_237*RLcp38_138;
    OPcp38_138 = OPcp38_137+ROcp38_137*qdd(38)+qd(38)*(OMcp38_237*ROcp38_337-OMcp38_337*ROcp38_237);
    OPcp38_238 = OPcp38_237+ROcp38_237*qdd(38)-qd(38)*(OMcp38_137*ROcp38_337-OMcp38_337*ROcp38_137);
    OPcp38_338 = OPcp38_337+ROcp38_337*qdd(38)+qd(38)*(OMcp38_137*ROcp38_237-OMcp38_237*ROcp38_137);
    RLcp38_139 = ROcp38_738*s.dpt(3,29);
    RLcp38_239 = ROcp38_838*s.dpt(3,29);
    RLcp38_339 = ROcp38_938*s.dpt(3,29);
    POcp38_139 = RLcp38_131+RLcp38_134+RLcp38_135+RLcp38_138+RLcp38_139+q(1);
    POcp38_239 = RLcp38_231+RLcp38_234+RLcp38_235+RLcp38_238+RLcp38_239+q(2);
    POcp38_339 = RLcp38_331+RLcp38_334+RLcp38_335+RLcp38_338+RLcp38_339+q(3);
    OMcp38_139 = OMcp38_138+ROcp38_738*qd(39);
    OMcp38_239 = OMcp38_238+ROcp38_838*qd(39);
    OMcp38_339 = OMcp38_338+ROcp38_938*qd(39);
    ORcp38_139 = OMcp38_238*RLcp38_339-OMcp38_338*RLcp38_239;
    ORcp38_239 = -(OMcp38_138*RLcp38_339-OMcp38_338*RLcp38_139);
    ORcp38_339 = OMcp38_138*RLcp38_239-OMcp38_238*RLcp38_139;
    VIcp38_139 = ORcp38_131+ORcp38_134+ORcp38_135+ORcp38_138+ORcp38_139+qd(1);
    VIcp38_239 = ORcp38_231+ORcp38_234+ORcp38_235+ORcp38_238+ORcp38_239+qd(2);
    VIcp38_339 = ORcp38_331+ORcp38_334+ORcp38_335+ORcp38_338+ORcp38_339+qd(3);
    OPcp38_139 = OPcp38_138+ROcp38_738*qdd(39)+qd(39)*(OMcp38_238*ROcp38_938-OMcp38_338*ROcp38_838);
    OPcp38_239 = OPcp38_238+ROcp38_838*qdd(39)-qd(39)*(OMcp38_138*ROcp38_938-OMcp38_338*ROcp38_738);
    OPcp38_339 = OPcp38_338+ROcp38_938*qdd(39)+qd(39)*(OMcp38_138*ROcp38_838-OMcp38_238*ROcp38_738);
    ACcp38_139 = qdd(1)+OMcp38_233*ORcp38_334+OMcp38_234*ORcp38_335+OMcp38_237*ORcp38_338+OMcp38_238*ORcp38_339+OMcp38_26*ORcp38_331-OMcp38_333*...
 ORcp38_234-OMcp38_334*ORcp38_235-OMcp38_337*ORcp38_238-OMcp38_338*ORcp38_239-OMcp38_36*ORcp38_231+OPcp38_233*RLcp38_334+OPcp38_234*RLcp38_335+...
 OPcp38_237*RLcp38_338+OPcp38_238*RLcp38_339+OPcp38_26*RLcp38_331-OPcp38_333*RLcp38_234-OPcp38_334*RLcp38_235-OPcp38_337*RLcp38_238-OPcp38_338*...
 RLcp38_239-OPcp38_36*RLcp38_231;
    ACcp38_239 = qdd(2)-OMcp38_133*ORcp38_334-OMcp38_134*ORcp38_335-OMcp38_137*ORcp38_338-OMcp38_138*ORcp38_339-OMcp38_16*ORcp38_331+OMcp38_333*...
 ORcp38_134+OMcp38_334*ORcp38_135+OMcp38_337*ORcp38_138+OMcp38_338*ORcp38_139+OMcp38_36*ORcp38_131-OPcp38_133*RLcp38_334-OPcp38_134*RLcp38_335-...
 OPcp38_137*RLcp38_338-OPcp38_138*RLcp38_339-OPcp38_16*RLcp38_331+OPcp38_333*RLcp38_134+OPcp38_334*RLcp38_135+OPcp38_337*RLcp38_138+OPcp38_338*...
 RLcp38_139+OPcp38_36*RLcp38_131;
    ACcp38_339 = qdd(3)+OMcp38_133*ORcp38_234+OMcp38_134*ORcp38_235+OMcp38_137*ORcp38_238+OMcp38_138*ORcp38_239+OMcp38_16*ORcp38_231-OMcp38_233*...
 ORcp38_134-OMcp38_234*ORcp38_135-OMcp38_237*ORcp38_138-OMcp38_238*ORcp38_139-OMcp38_26*ORcp38_131+OPcp38_133*RLcp38_234+OPcp38_134*RLcp38_235+...
 OPcp38_137*RLcp38_238+OPcp38_138*RLcp38_239+OPcp38_16*RLcp38_231-OPcp38_233*RLcp38_134-OPcp38_234*RLcp38_135-OPcp38_237*RLcp38_138-OPcp38_238*...
 RLcp38_139-OPcp38_26*RLcp38_131;

% = = Block_1_0_0_39_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp38_139;
    sens.P(2) = POcp38_239;
    sens.P(3) = POcp38_339;
    sens.R(1,1) = ROcp38_139;
    sens.R(1,2) = ROcp38_239;
    sens.R(1,3) = ROcp38_339;
    sens.R(2,1) = ROcp38_439;
    sens.R(2,2) = ROcp38_539;
    sens.R(2,3) = ROcp38_639;
    sens.R(3,1) = ROcp38_738;
    sens.R(3,2) = ROcp38_838;
    sens.R(3,3) = ROcp38_938;
    sens.V(1) = VIcp38_139;
    sens.V(2) = VIcp38_239;
    sens.V(3) = VIcp38_339;
    sens.OM(1) = OMcp38_139;
    sens.OM(2) = OMcp38_239;
    sens.OM(3) = OMcp38_339;
    sens.A(1) = ACcp38_139;
    sens.A(2) = ACcp38_239;
    sens.A(3) = ACcp38_339;
    sens.OMP(1) = OPcp38_139;
    sens.OMP(2) = OPcp38_239;
    sens.OMP(3) = OPcp38_339;
 
% 
case 40, 


% = = Block_1_0_0_40_0_1 = = 
 
% Sensor Kinematics 


    ROcp39_25 = S4*S5;
    ROcp39_35 = -C4*S5;
    ROcp39_85 = -S4*C5;
    ROcp39_95 = C4*C5;
    ROcp39_16 = C5*C6;
    ROcp39_26 = ROcp39_25*C6+C4*S6;
    ROcp39_36 = ROcp39_35*C6+S4*S6;
    ROcp39_46 = -C5*S6;
    ROcp39_56 = -(ROcp39_25*S6-C4*C6);
    ROcp39_66 = -(ROcp39_35*S6-S4*C6);
    OMcp39_25 = qd(5)*C4;
    OMcp39_35 = qd(5)*S4;
    OMcp39_16 = qd(4)+qd(6)*S5;
    OMcp39_26 = OMcp39_25+ROcp39_85*qd(6);
    OMcp39_36 = OMcp39_35+ROcp39_95*qd(6);
    OPcp39_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp39_26 = ROcp39_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp39_35*S5-ROcp39_95*qd(4));
    OPcp39_36 = ROcp39_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp39_25*S5-ROcp39_85*qd(4));

% = = Block_1_0_0_40_0_4 = = 
 
% Sensor Kinematics 


    ROcp39_131 = ROcp39_16*C31-S31*S5;
    ROcp39_231 = ROcp39_26*C31-ROcp39_85*S31;
    ROcp39_331 = ROcp39_36*C31-ROcp39_95*S31;
    ROcp39_731 = ROcp39_16*S31+C31*S5;
    ROcp39_831 = ROcp39_26*S31+ROcp39_85*C31;
    ROcp39_931 = ROcp39_36*S31+ROcp39_95*C31;
    ROcp39_432 = ROcp39_46*C32+ROcp39_731*S32;
    ROcp39_532 = ROcp39_56*C32+ROcp39_831*S32;
    ROcp39_632 = ROcp39_66*C32+ROcp39_931*S32;
    ROcp39_732 = -(ROcp39_46*S32-ROcp39_731*C32);
    ROcp39_832 = -(ROcp39_56*S32-ROcp39_831*C32);
    ROcp39_932 = -(ROcp39_66*S32-ROcp39_931*C32);
    ROcp39_133 = ROcp39_131*C33-ROcp39_732*S33;
    ROcp39_233 = ROcp39_231*C33-ROcp39_832*S33;
    ROcp39_333 = ROcp39_331*C33-ROcp39_932*S33;
    ROcp39_733 = ROcp39_131*S33+ROcp39_732*C33;
    ROcp39_833 = ROcp39_231*S33+ROcp39_832*C33;
    ROcp39_933 = ROcp39_331*S33+ROcp39_932*C33;
    ROcp39_134 = ROcp39_133*C34+ROcp39_432*S34;
    ROcp39_234 = ROcp39_233*C34+ROcp39_532*S34;
    ROcp39_334 = ROcp39_333*C34+ROcp39_632*S34;
    ROcp39_434 = -(ROcp39_133*S34-ROcp39_432*C34);
    ROcp39_534 = -(ROcp39_233*S34-ROcp39_532*C34);
    ROcp39_634 = -(ROcp39_333*S34-ROcp39_632*C34);
    RLcp39_131 = ROcp39_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp39_231 = ROcp39_26*s.dpt(1,3)+ROcp39_85*s.dpt(3,3);
    RLcp39_331 = ROcp39_36*s.dpt(1,3)+ROcp39_95*s.dpt(3,3);
    OMcp39_131 = OMcp39_16+ROcp39_46*qd(31);
    OMcp39_231 = OMcp39_26+ROcp39_56*qd(31);
    OMcp39_331 = OMcp39_36+ROcp39_66*qd(31);
    ORcp39_131 = OMcp39_26*RLcp39_331-OMcp39_36*RLcp39_231;
    ORcp39_231 = -(OMcp39_16*RLcp39_331-OMcp39_36*RLcp39_131);
    ORcp39_331 = OMcp39_16*RLcp39_231-OMcp39_26*RLcp39_131;
    OMcp39_132 = OMcp39_131+ROcp39_131*qd(32);
    OMcp39_232 = OMcp39_231+ROcp39_231*qd(32);
    OMcp39_332 = OMcp39_331+ROcp39_331*qd(32);
    OMcp39_133 = OMcp39_132+ROcp39_432*qd(33);
    OMcp39_233 = OMcp39_232+ROcp39_532*qd(33);
    OMcp39_333 = OMcp39_332+ROcp39_632*qd(33);
    OPcp39_133 = OPcp39_16+ROcp39_131*qdd(32)+ROcp39_432*qdd(33)+ROcp39_46*qdd(31)+qd(31)*(OMcp39_26*ROcp39_66-OMcp39_36*ROcp39_56)+qd(32)*(...
 OMcp39_231*ROcp39_331-OMcp39_331*ROcp39_231)+qd(33)*(OMcp39_232*ROcp39_632-OMcp39_332*ROcp39_532);
    OPcp39_233 = OPcp39_26+ROcp39_231*qdd(32)+ROcp39_532*qdd(33)+ROcp39_56*qdd(31)-qd(31)*(OMcp39_16*ROcp39_66-OMcp39_36*ROcp39_46)-qd(32)*(...
 OMcp39_131*ROcp39_331-OMcp39_331*ROcp39_131)-qd(33)*(OMcp39_132*ROcp39_632-OMcp39_332*ROcp39_432);
    OPcp39_333 = OPcp39_36+ROcp39_331*qdd(32)+ROcp39_632*qdd(33)+ROcp39_66*qdd(31)+qd(31)*(OMcp39_16*ROcp39_56-OMcp39_26*ROcp39_46)+qd(32)*(...
 OMcp39_131*ROcp39_231-OMcp39_231*ROcp39_131)+qd(33)*(OMcp39_132*ROcp39_532-OMcp39_232*ROcp39_432);
    RLcp39_134 = ROcp39_733*s.dpt(3,21);
    RLcp39_234 = ROcp39_833*s.dpt(3,21);
    RLcp39_334 = ROcp39_933*s.dpt(3,21);
    OMcp39_134 = OMcp39_133+ROcp39_733*qd(34);
    OMcp39_234 = OMcp39_233+ROcp39_833*qd(34);
    OMcp39_334 = OMcp39_333+ROcp39_933*qd(34);
    ORcp39_134 = OMcp39_233*RLcp39_334-OMcp39_333*RLcp39_234;
    ORcp39_234 = -(OMcp39_133*RLcp39_334-OMcp39_333*RLcp39_134);
    ORcp39_334 = OMcp39_133*RLcp39_234-OMcp39_233*RLcp39_134;
    OPcp39_134 = OPcp39_133+ROcp39_733*qdd(34)+qd(34)*(OMcp39_233*ROcp39_933-OMcp39_333*ROcp39_833);
    OPcp39_234 = OPcp39_233+ROcp39_833*qdd(34)-qd(34)*(OMcp39_133*ROcp39_933-OMcp39_333*ROcp39_733);
    OPcp39_334 = OPcp39_333+ROcp39_933*qdd(34)+qd(34)*(OMcp39_133*ROcp39_833-OMcp39_233*ROcp39_733);

% = = Block_1_0_0_40_0_5 = = 
 
% Sensor Kinematics 


    ROcp39_435 = ROcp39_434*C35+ROcp39_733*S35;
    ROcp39_535 = ROcp39_534*C35+ROcp39_833*S35;
    ROcp39_635 = ROcp39_634*C35+ROcp39_933*S35;
    ROcp39_735 = -(ROcp39_434*S35-ROcp39_733*C35);
    ROcp39_835 = -(ROcp39_534*S35-ROcp39_833*C35);
    ROcp39_935 = -(ROcp39_634*S35-ROcp39_933*C35);
    ROcp39_136 = ROcp39_134*C36+ROcp39_435*S36;
    ROcp39_236 = ROcp39_234*C36+ROcp39_535*S36;
    ROcp39_336 = ROcp39_334*C36+ROcp39_635*S36;
    ROcp39_436 = -(ROcp39_134*S36-ROcp39_435*C36);
    ROcp39_536 = -(ROcp39_234*S36-ROcp39_535*C36);
    ROcp39_636 = -(ROcp39_334*S36-ROcp39_635*C36);
    ROcp39_137 = ROcp39_136*C37-ROcp39_735*S37;
    ROcp39_237 = ROcp39_236*C37-ROcp39_835*S37;
    ROcp39_337 = ROcp39_336*C37-ROcp39_935*S37;
    ROcp39_737 = ROcp39_136*S37+ROcp39_735*C37;
    ROcp39_837 = ROcp39_236*S37+ROcp39_835*C37;
    ROcp39_937 = ROcp39_336*S37+ROcp39_935*C37;
    ROcp39_438 = ROcp39_436*C38+ROcp39_737*S38;
    ROcp39_538 = ROcp39_536*C38+ROcp39_837*S38;
    ROcp39_638 = ROcp39_636*C38+ROcp39_937*S38;
    ROcp39_738 = -(ROcp39_436*S38-ROcp39_737*C38);
    ROcp39_838 = -(ROcp39_536*S38-ROcp39_837*C38);
    ROcp39_938 = -(ROcp39_636*S38-ROcp39_937*C38);
    ROcp39_139 = ROcp39_137*C39+ROcp39_438*S39;
    ROcp39_239 = ROcp39_237*C39+ROcp39_538*S39;
    ROcp39_339 = ROcp39_337*C39+ROcp39_638*S39;
    ROcp39_439 = -(ROcp39_137*S39-ROcp39_438*C39);
    ROcp39_539 = -(ROcp39_237*S39-ROcp39_538*C39);
    ROcp39_639 = -(ROcp39_337*S39-ROcp39_638*C39);
    ROcp39_140 = ROcp39_139*C40-ROcp39_738*S40;
    ROcp39_240 = ROcp39_239*C40-ROcp39_838*S40;
    ROcp39_340 = ROcp39_339*C40-ROcp39_938*S40;
    ROcp39_740 = ROcp39_139*S40+ROcp39_738*C40;
    ROcp39_840 = ROcp39_239*S40+ROcp39_838*C40;
    ROcp39_940 = ROcp39_339*S40+ROcp39_938*C40;
    RLcp39_135 = ROcp39_134*s.dpt(1,23)+ROcp39_434*s.dpt(2,23)+ROcp39_733*s.dpt(3,23);
    RLcp39_235 = ROcp39_234*s.dpt(1,23)+ROcp39_534*s.dpt(2,23)+ROcp39_833*s.dpt(3,23);
    RLcp39_335 = ROcp39_334*s.dpt(1,23)+ROcp39_634*s.dpt(2,23)+ROcp39_933*s.dpt(3,23);
    ORcp39_135 = OMcp39_234*RLcp39_335-OMcp39_334*RLcp39_235;
    ORcp39_235 = -(OMcp39_134*RLcp39_335-OMcp39_334*RLcp39_135);
    ORcp39_335 = OMcp39_134*RLcp39_235-OMcp39_234*RLcp39_135;
    OMcp39_137 = OMcp39_134+ROcp39_436*qd(37);
    OMcp39_237 = OMcp39_234+ROcp39_536*qd(37);
    OMcp39_337 = OMcp39_334+ROcp39_636*qd(37);
    OPcp39_137 = OPcp39_134+ROcp39_436*qdd(37)+qd(37)*(OMcp39_234*ROcp39_636-OMcp39_334*ROcp39_536);
    OPcp39_237 = OPcp39_234+ROcp39_536*qdd(37)-qd(37)*(OMcp39_134*ROcp39_636-OMcp39_334*ROcp39_436);
    OPcp39_337 = OPcp39_334+ROcp39_636*qdd(37)+qd(37)*(OMcp39_134*ROcp39_536-OMcp39_234*ROcp39_436);
    RLcp39_138 = ROcp39_436*s.dpt(2,27)+ROcp39_737*s.dpt(3,27);
    RLcp39_238 = ROcp39_536*s.dpt(2,27)+ROcp39_837*s.dpt(3,27);
    RLcp39_338 = ROcp39_636*s.dpt(2,27)+ROcp39_937*s.dpt(3,27);
    OMcp39_138 = OMcp39_137+ROcp39_137*qd(38);
    OMcp39_238 = OMcp39_237+ROcp39_237*qd(38);
    OMcp39_338 = OMcp39_337+ROcp39_337*qd(38);
    ORcp39_138 = OMcp39_237*RLcp39_338-OMcp39_337*RLcp39_238;
    ORcp39_238 = -(OMcp39_137*RLcp39_338-OMcp39_337*RLcp39_138);
    ORcp39_338 = OMcp39_137*RLcp39_238-OMcp39_237*RLcp39_138;
    OPcp39_138 = OPcp39_137+ROcp39_137*qdd(38)+qd(38)*(OMcp39_237*ROcp39_337-OMcp39_337*ROcp39_237);
    OPcp39_238 = OPcp39_237+ROcp39_237*qdd(38)-qd(38)*(OMcp39_137*ROcp39_337-OMcp39_337*ROcp39_137);
    OPcp39_338 = OPcp39_337+ROcp39_337*qdd(38)+qd(38)*(OMcp39_137*ROcp39_237-OMcp39_237*ROcp39_137);
    RLcp39_139 = ROcp39_738*s.dpt(3,29);
    RLcp39_239 = ROcp39_838*s.dpt(3,29);
    RLcp39_339 = ROcp39_938*s.dpt(3,29);
    OMcp39_139 = OMcp39_138+ROcp39_738*qd(39);
    OMcp39_239 = OMcp39_238+ROcp39_838*qd(39);
    OMcp39_339 = OMcp39_338+ROcp39_938*qd(39);
    ORcp39_139 = OMcp39_238*RLcp39_339-OMcp39_338*RLcp39_239;
    ORcp39_239 = -(OMcp39_138*RLcp39_339-OMcp39_338*RLcp39_139);
    ORcp39_339 = OMcp39_138*RLcp39_239-OMcp39_238*RLcp39_139;
    OPcp39_139 = OPcp39_138+ROcp39_738*qdd(39)+qd(39)*(OMcp39_238*ROcp39_938-OMcp39_338*ROcp39_838);
    OPcp39_239 = OPcp39_238+ROcp39_838*qdd(39)-qd(39)*(OMcp39_138*ROcp39_938-OMcp39_338*ROcp39_738);
    OPcp39_339 = OPcp39_338+ROcp39_938*qdd(39)+qd(39)*(OMcp39_138*ROcp39_838-OMcp39_238*ROcp39_738);
    RLcp39_140 = ROcp39_139*s.dpt(1,31)+ROcp39_738*s.dpt(3,31);
    RLcp39_240 = ROcp39_239*s.dpt(1,31)+ROcp39_838*s.dpt(3,31);
    RLcp39_340 = ROcp39_339*s.dpt(1,31)+ROcp39_938*s.dpt(3,31);
    POcp39_140 = RLcp39_131+RLcp39_134+RLcp39_135+RLcp39_138+RLcp39_139+RLcp39_140+q(1);
    POcp39_240 = RLcp39_231+RLcp39_234+RLcp39_235+RLcp39_238+RLcp39_239+RLcp39_240+q(2);
    POcp39_340 = RLcp39_331+RLcp39_334+RLcp39_335+RLcp39_338+RLcp39_339+RLcp39_340+q(3);
    OMcp39_140 = OMcp39_139+ROcp39_439*qd(40);
    OMcp39_240 = OMcp39_239+ROcp39_539*qd(40);
    OMcp39_340 = OMcp39_339+ROcp39_639*qd(40);
    ORcp39_140 = OMcp39_239*RLcp39_340-OMcp39_339*RLcp39_240;
    ORcp39_240 = -(OMcp39_139*RLcp39_340-OMcp39_339*RLcp39_140);
    ORcp39_340 = OMcp39_139*RLcp39_240-OMcp39_239*RLcp39_140;
    VIcp39_140 = ORcp39_131+ORcp39_134+ORcp39_135+ORcp39_138+ORcp39_139+ORcp39_140+qd(1);
    VIcp39_240 = ORcp39_231+ORcp39_234+ORcp39_235+ORcp39_238+ORcp39_239+ORcp39_240+qd(2);
    VIcp39_340 = ORcp39_331+ORcp39_334+ORcp39_335+ORcp39_338+ORcp39_339+ORcp39_340+qd(3);
    OPcp39_140 = OPcp39_139+ROcp39_439*qdd(40)+qd(40)*(OMcp39_239*ROcp39_639-OMcp39_339*ROcp39_539);
    OPcp39_240 = OPcp39_239+ROcp39_539*qdd(40)-qd(40)*(OMcp39_139*ROcp39_639-OMcp39_339*ROcp39_439);
    OPcp39_340 = OPcp39_339+ROcp39_639*qdd(40)+qd(40)*(OMcp39_139*ROcp39_539-OMcp39_239*ROcp39_439);
    ACcp39_140 = qdd(1)+OMcp39_233*ORcp39_334+OMcp39_234*ORcp39_335+OMcp39_237*ORcp39_338+OMcp39_238*ORcp39_339+OMcp39_239*ORcp39_340+OMcp39_26*...
 ORcp39_331-OMcp39_333*ORcp39_234-OMcp39_334*ORcp39_235-OMcp39_337*ORcp39_238-OMcp39_338*ORcp39_239-OMcp39_339*ORcp39_240-OMcp39_36*ORcp39_231+...
 OPcp39_233*RLcp39_334+OPcp39_234*RLcp39_335+OPcp39_237*RLcp39_338+OPcp39_238*RLcp39_339+OPcp39_239*RLcp39_340+OPcp39_26*RLcp39_331-OPcp39_333*...
 RLcp39_234-OPcp39_334*RLcp39_235-OPcp39_337*RLcp39_238-OPcp39_338*RLcp39_239-OPcp39_339*RLcp39_240-OPcp39_36*RLcp39_231;
    ACcp39_240 = qdd(2)-OMcp39_133*ORcp39_334-OMcp39_134*ORcp39_335-OMcp39_137*ORcp39_338-OMcp39_138*ORcp39_339-OMcp39_139*ORcp39_340-OMcp39_16*...
 ORcp39_331+OMcp39_333*ORcp39_134+OMcp39_334*ORcp39_135+OMcp39_337*ORcp39_138+OMcp39_338*ORcp39_139+OMcp39_339*ORcp39_140+OMcp39_36*ORcp39_131-...
 OPcp39_133*RLcp39_334-OPcp39_134*RLcp39_335-OPcp39_137*RLcp39_338-OPcp39_138*RLcp39_339-OPcp39_139*RLcp39_340-OPcp39_16*RLcp39_331+OPcp39_333*...
 RLcp39_134+OPcp39_334*RLcp39_135+OPcp39_337*RLcp39_138+OPcp39_338*RLcp39_139+OPcp39_339*RLcp39_140+OPcp39_36*RLcp39_131;
    ACcp39_340 = qdd(3)+OMcp39_133*ORcp39_234+OMcp39_134*ORcp39_235+OMcp39_137*ORcp39_238+OMcp39_138*ORcp39_239+OMcp39_139*ORcp39_240+OMcp39_16*...
 ORcp39_231-OMcp39_233*ORcp39_134-OMcp39_234*ORcp39_135-OMcp39_237*ORcp39_138-OMcp39_238*ORcp39_139-OMcp39_239*ORcp39_140-OMcp39_26*ORcp39_131+...
 OPcp39_133*RLcp39_234+OPcp39_134*RLcp39_235+OPcp39_137*RLcp39_238+OPcp39_138*RLcp39_239+OPcp39_139*RLcp39_240+OPcp39_16*RLcp39_231-OPcp39_233*...
 RLcp39_134-OPcp39_234*RLcp39_135-OPcp39_237*RLcp39_138-OPcp39_238*RLcp39_139-OPcp39_239*RLcp39_140-OPcp39_26*RLcp39_131;

% = = Block_1_0_0_40_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp39_140;
    sens.P(2) = POcp39_240;
    sens.P(3) = POcp39_340;
    sens.R(1,1) = ROcp39_140;
    sens.R(1,2) = ROcp39_240;
    sens.R(1,3) = ROcp39_340;
    sens.R(2,1) = ROcp39_439;
    sens.R(2,2) = ROcp39_539;
    sens.R(2,3) = ROcp39_639;
    sens.R(3,1) = ROcp39_740;
    sens.R(3,2) = ROcp39_840;
    sens.R(3,3) = ROcp39_940;
    sens.V(1) = VIcp39_140;
    sens.V(2) = VIcp39_240;
    sens.V(3) = VIcp39_340;
    sens.OM(1) = OMcp39_140;
    sens.OM(2) = OMcp39_240;
    sens.OM(3) = OMcp39_340;
    sens.A(1) = ACcp39_140;
    sens.A(2) = ACcp39_240;
    sens.A(3) = ACcp39_340;
    sens.OMP(1) = OPcp39_140;
    sens.OMP(2) = OPcp39_240;
    sens.OMP(3) = OPcp39_340;
 
% 
case 41, 


% = = Block_1_0_0_41_0_1 = = 
 
% Sensor Kinematics 


    ROcp40_25 = S4*S5;
    ROcp40_35 = -C4*S5;
    ROcp40_85 = -S4*C5;
    ROcp40_95 = C4*C5;
    ROcp40_16 = C5*C6;
    ROcp40_26 = ROcp40_25*C6+C4*S6;
    ROcp40_36 = ROcp40_35*C6+S4*S6;
    ROcp40_46 = -C5*S6;
    ROcp40_56 = -(ROcp40_25*S6-C4*C6);
    ROcp40_66 = -(ROcp40_35*S6-S4*C6);
    OMcp40_25 = qd(5)*C4;
    OMcp40_35 = qd(5)*S4;
    OMcp40_16 = qd(4)+qd(6)*S5;
    OMcp40_26 = OMcp40_25+ROcp40_85*qd(6);
    OMcp40_36 = OMcp40_35+ROcp40_95*qd(6);
    OPcp40_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp40_26 = ROcp40_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp40_35*S5-ROcp40_95*qd(4));
    OPcp40_36 = ROcp40_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp40_25*S5-ROcp40_85*qd(4));

% = = Block_1_0_0_41_0_4 = = 
 
% Sensor Kinematics 


    ROcp40_131 = ROcp40_16*C31-S31*S5;
    ROcp40_231 = ROcp40_26*C31-ROcp40_85*S31;
    ROcp40_331 = ROcp40_36*C31-ROcp40_95*S31;
    ROcp40_731 = ROcp40_16*S31+C31*S5;
    ROcp40_831 = ROcp40_26*S31+ROcp40_85*C31;
    ROcp40_931 = ROcp40_36*S31+ROcp40_95*C31;
    ROcp40_432 = ROcp40_46*C32+ROcp40_731*S32;
    ROcp40_532 = ROcp40_56*C32+ROcp40_831*S32;
    ROcp40_632 = ROcp40_66*C32+ROcp40_931*S32;
    ROcp40_732 = -(ROcp40_46*S32-ROcp40_731*C32);
    ROcp40_832 = -(ROcp40_56*S32-ROcp40_831*C32);
    ROcp40_932 = -(ROcp40_66*S32-ROcp40_931*C32);
    ROcp40_133 = ROcp40_131*C33-ROcp40_732*S33;
    ROcp40_233 = ROcp40_231*C33-ROcp40_832*S33;
    ROcp40_333 = ROcp40_331*C33-ROcp40_932*S33;
    ROcp40_733 = ROcp40_131*S33+ROcp40_732*C33;
    ROcp40_833 = ROcp40_231*S33+ROcp40_832*C33;
    ROcp40_933 = ROcp40_331*S33+ROcp40_932*C33;
    ROcp40_134 = ROcp40_133*C34+ROcp40_432*S34;
    ROcp40_234 = ROcp40_233*C34+ROcp40_532*S34;
    ROcp40_334 = ROcp40_333*C34+ROcp40_632*S34;
    ROcp40_434 = -(ROcp40_133*S34-ROcp40_432*C34);
    ROcp40_534 = -(ROcp40_233*S34-ROcp40_532*C34);
    ROcp40_634 = -(ROcp40_333*S34-ROcp40_632*C34);
    RLcp40_131 = ROcp40_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp40_231 = ROcp40_26*s.dpt(1,3)+ROcp40_85*s.dpt(3,3);
    RLcp40_331 = ROcp40_36*s.dpt(1,3)+ROcp40_95*s.dpt(3,3);
    OMcp40_131 = OMcp40_16+ROcp40_46*qd(31);
    OMcp40_231 = OMcp40_26+ROcp40_56*qd(31);
    OMcp40_331 = OMcp40_36+ROcp40_66*qd(31);
    ORcp40_131 = OMcp40_26*RLcp40_331-OMcp40_36*RLcp40_231;
    ORcp40_231 = -(OMcp40_16*RLcp40_331-OMcp40_36*RLcp40_131);
    ORcp40_331 = OMcp40_16*RLcp40_231-OMcp40_26*RLcp40_131;
    OMcp40_132 = OMcp40_131+ROcp40_131*qd(32);
    OMcp40_232 = OMcp40_231+ROcp40_231*qd(32);
    OMcp40_332 = OMcp40_331+ROcp40_331*qd(32);
    OMcp40_133 = OMcp40_132+ROcp40_432*qd(33);
    OMcp40_233 = OMcp40_232+ROcp40_532*qd(33);
    OMcp40_333 = OMcp40_332+ROcp40_632*qd(33);
    OPcp40_133 = OPcp40_16+ROcp40_131*qdd(32)+ROcp40_432*qdd(33)+ROcp40_46*qdd(31)+qd(31)*(OMcp40_26*ROcp40_66-OMcp40_36*ROcp40_56)+qd(32)*(...
 OMcp40_231*ROcp40_331-OMcp40_331*ROcp40_231)+qd(33)*(OMcp40_232*ROcp40_632-OMcp40_332*ROcp40_532);
    OPcp40_233 = OPcp40_26+ROcp40_231*qdd(32)+ROcp40_532*qdd(33)+ROcp40_56*qdd(31)-qd(31)*(OMcp40_16*ROcp40_66-OMcp40_36*ROcp40_46)-qd(32)*(...
 OMcp40_131*ROcp40_331-OMcp40_331*ROcp40_131)-qd(33)*(OMcp40_132*ROcp40_632-OMcp40_332*ROcp40_432);
    OPcp40_333 = OPcp40_36+ROcp40_331*qdd(32)+ROcp40_632*qdd(33)+ROcp40_66*qdd(31)+qd(31)*(OMcp40_16*ROcp40_56-OMcp40_26*ROcp40_46)+qd(32)*(...
 OMcp40_131*ROcp40_231-OMcp40_231*ROcp40_131)+qd(33)*(OMcp40_132*ROcp40_532-OMcp40_232*ROcp40_432);
    RLcp40_134 = ROcp40_733*s.dpt(3,21);
    RLcp40_234 = ROcp40_833*s.dpt(3,21);
    RLcp40_334 = ROcp40_933*s.dpt(3,21);
    OMcp40_134 = OMcp40_133+ROcp40_733*qd(34);
    OMcp40_234 = OMcp40_233+ROcp40_833*qd(34);
    OMcp40_334 = OMcp40_333+ROcp40_933*qd(34);
    ORcp40_134 = OMcp40_233*RLcp40_334-OMcp40_333*RLcp40_234;
    ORcp40_234 = -(OMcp40_133*RLcp40_334-OMcp40_333*RLcp40_134);
    ORcp40_334 = OMcp40_133*RLcp40_234-OMcp40_233*RLcp40_134;
    OPcp40_134 = OPcp40_133+ROcp40_733*qdd(34)+qd(34)*(OMcp40_233*ROcp40_933-OMcp40_333*ROcp40_833);
    OPcp40_234 = OPcp40_233+ROcp40_833*qdd(34)-qd(34)*(OMcp40_133*ROcp40_933-OMcp40_333*ROcp40_733);
    OPcp40_334 = OPcp40_333+ROcp40_933*qdd(34)+qd(34)*(OMcp40_133*ROcp40_833-OMcp40_233*ROcp40_733);

% = = Block_1_0_0_41_0_5 = = 
 
% Sensor Kinematics 


    ROcp40_435 = ROcp40_434*C35+ROcp40_733*S35;
    ROcp40_535 = ROcp40_534*C35+ROcp40_833*S35;
    ROcp40_635 = ROcp40_634*C35+ROcp40_933*S35;
    ROcp40_735 = -(ROcp40_434*S35-ROcp40_733*C35);
    ROcp40_835 = -(ROcp40_534*S35-ROcp40_833*C35);
    ROcp40_935 = -(ROcp40_634*S35-ROcp40_933*C35);
    ROcp40_136 = ROcp40_134*C36+ROcp40_435*S36;
    ROcp40_236 = ROcp40_234*C36+ROcp40_535*S36;
    ROcp40_336 = ROcp40_334*C36+ROcp40_635*S36;
    ROcp40_436 = -(ROcp40_134*S36-ROcp40_435*C36);
    ROcp40_536 = -(ROcp40_234*S36-ROcp40_535*C36);
    ROcp40_636 = -(ROcp40_334*S36-ROcp40_635*C36);
    ROcp40_137 = ROcp40_136*C37-ROcp40_735*S37;
    ROcp40_237 = ROcp40_236*C37-ROcp40_835*S37;
    ROcp40_337 = ROcp40_336*C37-ROcp40_935*S37;
    ROcp40_737 = ROcp40_136*S37+ROcp40_735*C37;
    ROcp40_837 = ROcp40_236*S37+ROcp40_835*C37;
    ROcp40_937 = ROcp40_336*S37+ROcp40_935*C37;
    ROcp40_438 = ROcp40_436*C38+ROcp40_737*S38;
    ROcp40_538 = ROcp40_536*C38+ROcp40_837*S38;
    ROcp40_638 = ROcp40_636*C38+ROcp40_937*S38;
    ROcp40_738 = -(ROcp40_436*S38-ROcp40_737*C38);
    ROcp40_838 = -(ROcp40_536*S38-ROcp40_837*C38);
    ROcp40_938 = -(ROcp40_636*S38-ROcp40_937*C38);
    ROcp40_139 = ROcp40_137*C39+ROcp40_438*S39;
    ROcp40_239 = ROcp40_237*C39+ROcp40_538*S39;
    ROcp40_339 = ROcp40_337*C39+ROcp40_638*S39;
    ROcp40_439 = -(ROcp40_137*S39-ROcp40_438*C39);
    ROcp40_539 = -(ROcp40_237*S39-ROcp40_538*C39);
    ROcp40_639 = -(ROcp40_337*S39-ROcp40_638*C39);
    ROcp40_140 = ROcp40_139*C40-ROcp40_738*S40;
    ROcp40_240 = ROcp40_239*C40-ROcp40_838*S40;
    ROcp40_340 = ROcp40_339*C40-ROcp40_938*S40;
    ROcp40_740 = ROcp40_139*S40+ROcp40_738*C40;
    ROcp40_840 = ROcp40_239*S40+ROcp40_838*C40;
    ROcp40_940 = ROcp40_339*S40+ROcp40_938*C40;
    ROcp40_141 = ROcp40_140*C41+ROcp40_439*S41;
    ROcp40_241 = ROcp40_240*C41+ROcp40_539*S41;
    ROcp40_341 = ROcp40_340*C41+ROcp40_639*S41;
    ROcp40_441 = -(ROcp40_140*S41-ROcp40_439*C41);
    ROcp40_541 = -(ROcp40_240*S41-ROcp40_539*C41);
    ROcp40_641 = -(ROcp40_340*S41-ROcp40_639*C41);
    RLcp40_135 = ROcp40_134*s.dpt(1,23)+ROcp40_434*s.dpt(2,23)+ROcp40_733*s.dpt(3,23);
    RLcp40_235 = ROcp40_234*s.dpt(1,23)+ROcp40_534*s.dpt(2,23)+ROcp40_833*s.dpt(3,23);
    RLcp40_335 = ROcp40_334*s.dpt(1,23)+ROcp40_634*s.dpt(2,23)+ROcp40_933*s.dpt(3,23);
    ORcp40_135 = OMcp40_234*RLcp40_335-OMcp40_334*RLcp40_235;
    ORcp40_235 = -(OMcp40_134*RLcp40_335-OMcp40_334*RLcp40_135);
    ORcp40_335 = OMcp40_134*RLcp40_235-OMcp40_234*RLcp40_135;
    OMcp40_137 = OMcp40_134+ROcp40_436*qd(37);
    OMcp40_237 = OMcp40_234+ROcp40_536*qd(37);
    OMcp40_337 = OMcp40_334+ROcp40_636*qd(37);
    OPcp40_137 = OPcp40_134+ROcp40_436*qdd(37)+qd(37)*(OMcp40_234*ROcp40_636-OMcp40_334*ROcp40_536);
    OPcp40_237 = OPcp40_234+ROcp40_536*qdd(37)-qd(37)*(OMcp40_134*ROcp40_636-OMcp40_334*ROcp40_436);
    OPcp40_337 = OPcp40_334+ROcp40_636*qdd(37)+qd(37)*(OMcp40_134*ROcp40_536-OMcp40_234*ROcp40_436);
    RLcp40_138 = ROcp40_436*s.dpt(2,27)+ROcp40_737*s.dpt(3,27);
    RLcp40_238 = ROcp40_536*s.dpt(2,27)+ROcp40_837*s.dpt(3,27);
    RLcp40_338 = ROcp40_636*s.dpt(2,27)+ROcp40_937*s.dpt(3,27);
    OMcp40_138 = OMcp40_137+ROcp40_137*qd(38);
    OMcp40_238 = OMcp40_237+ROcp40_237*qd(38);
    OMcp40_338 = OMcp40_337+ROcp40_337*qd(38);
    ORcp40_138 = OMcp40_237*RLcp40_338-OMcp40_337*RLcp40_238;
    ORcp40_238 = -(OMcp40_137*RLcp40_338-OMcp40_337*RLcp40_138);
    ORcp40_338 = OMcp40_137*RLcp40_238-OMcp40_237*RLcp40_138;
    OPcp40_138 = OPcp40_137+ROcp40_137*qdd(38)+qd(38)*(OMcp40_237*ROcp40_337-OMcp40_337*ROcp40_237);
    OPcp40_238 = OPcp40_237+ROcp40_237*qdd(38)-qd(38)*(OMcp40_137*ROcp40_337-OMcp40_337*ROcp40_137);
    OPcp40_338 = OPcp40_337+ROcp40_337*qdd(38)+qd(38)*(OMcp40_137*ROcp40_237-OMcp40_237*ROcp40_137);
    RLcp40_139 = ROcp40_738*s.dpt(3,29);
    RLcp40_239 = ROcp40_838*s.dpt(3,29);
    RLcp40_339 = ROcp40_938*s.dpt(3,29);
    OMcp40_139 = OMcp40_138+ROcp40_738*qd(39);
    OMcp40_239 = OMcp40_238+ROcp40_838*qd(39);
    OMcp40_339 = OMcp40_338+ROcp40_938*qd(39);
    ORcp40_139 = OMcp40_238*RLcp40_339-OMcp40_338*RLcp40_239;
    ORcp40_239 = -(OMcp40_138*RLcp40_339-OMcp40_338*RLcp40_139);
    ORcp40_339 = OMcp40_138*RLcp40_239-OMcp40_238*RLcp40_139;
    OPcp40_139 = OPcp40_138+ROcp40_738*qdd(39)+qd(39)*(OMcp40_238*ROcp40_938-OMcp40_338*ROcp40_838);
    OPcp40_239 = OPcp40_238+ROcp40_838*qdd(39)-qd(39)*(OMcp40_138*ROcp40_938-OMcp40_338*ROcp40_738);
    OPcp40_339 = OPcp40_338+ROcp40_938*qdd(39)+qd(39)*(OMcp40_138*ROcp40_838-OMcp40_238*ROcp40_738);
    RLcp40_140 = ROcp40_139*s.dpt(1,31)+ROcp40_738*s.dpt(3,31);
    RLcp40_240 = ROcp40_239*s.dpt(1,31)+ROcp40_838*s.dpt(3,31);
    RLcp40_340 = ROcp40_339*s.dpt(1,31)+ROcp40_938*s.dpt(3,31);
    OMcp40_140 = OMcp40_139+ROcp40_439*qd(40);
    OMcp40_240 = OMcp40_239+ROcp40_539*qd(40);
    OMcp40_340 = OMcp40_339+ROcp40_639*qd(40);
    ORcp40_140 = OMcp40_239*RLcp40_340-OMcp40_339*RLcp40_240;
    ORcp40_240 = -(OMcp40_139*RLcp40_340-OMcp40_339*RLcp40_140);
    ORcp40_340 = OMcp40_139*RLcp40_240-OMcp40_239*RLcp40_140;
    OPcp40_140 = OPcp40_139+ROcp40_439*qdd(40)+qd(40)*(OMcp40_239*ROcp40_639-OMcp40_339*ROcp40_539);
    OPcp40_240 = OPcp40_239+ROcp40_539*qdd(40)-qd(40)*(OMcp40_139*ROcp40_639-OMcp40_339*ROcp40_439);
    OPcp40_340 = OPcp40_339+ROcp40_639*qdd(40)+qd(40)*(OMcp40_139*ROcp40_539-OMcp40_239*ROcp40_439);
    RLcp40_141 = ROcp40_140*s.dpt(1,33)+ROcp40_740*s.dpt(3,33);
    RLcp40_241 = ROcp40_240*s.dpt(1,33)+ROcp40_840*s.dpt(3,33);
    RLcp40_341 = ROcp40_340*s.dpt(1,33)+ROcp40_940*s.dpt(3,33);
    POcp40_141 = RLcp40_131+RLcp40_134+RLcp40_135+RLcp40_138+RLcp40_139+RLcp40_140+RLcp40_141+q(1);
    POcp40_241 = RLcp40_231+RLcp40_234+RLcp40_235+RLcp40_238+RLcp40_239+RLcp40_240+RLcp40_241+q(2);
    POcp40_341 = RLcp40_331+RLcp40_334+RLcp40_335+RLcp40_338+RLcp40_339+RLcp40_340+RLcp40_341+q(3);
    OMcp40_141 = OMcp40_140+ROcp40_740*qd(41);
    OMcp40_241 = OMcp40_240+ROcp40_840*qd(41);
    OMcp40_341 = OMcp40_340+ROcp40_940*qd(41);
    ORcp40_141 = OMcp40_240*RLcp40_341-OMcp40_340*RLcp40_241;
    ORcp40_241 = -(OMcp40_140*RLcp40_341-OMcp40_340*RLcp40_141);
    ORcp40_341 = OMcp40_140*RLcp40_241-OMcp40_240*RLcp40_141;
    VIcp40_141 = ORcp40_131+ORcp40_134+ORcp40_135+ORcp40_138+ORcp40_139+ORcp40_140+ORcp40_141+qd(1);
    VIcp40_241 = ORcp40_231+ORcp40_234+ORcp40_235+ORcp40_238+ORcp40_239+ORcp40_240+ORcp40_241+qd(2);
    VIcp40_341 = ORcp40_331+ORcp40_334+ORcp40_335+ORcp40_338+ORcp40_339+ORcp40_340+ORcp40_341+qd(3);
    OPcp40_141 = OPcp40_140+ROcp40_740*qdd(41)+qd(41)*(OMcp40_240*ROcp40_940-OMcp40_340*ROcp40_840);
    OPcp40_241 = OPcp40_240+ROcp40_840*qdd(41)-qd(41)*(OMcp40_140*ROcp40_940-OMcp40_340*ROcp40_740);
    OPcp40_341 = OPcp40_340+ROcp40_940*qdd(41)+qd(41)*(OMcp40_140*ROcp40_840-OMcp40_240*ROcp40_740);
    ACcp40_141 = qdd(1)+OMcp40_233*ORcp40_334+OMcp40_234*ORcp40_335+OMcp40_237*ORcp40_338+OMcp40_238*ORcp40_339+OMcp40_239*ORcp40_340+OMcp40_240*...
 ORcp40_341+OMcp40_26*ORcp40_331-OMcp40_333*ORcp40_234-OMcp40_334*ORcp40_235-OMcp40_337*ORcp40_238-OMcp40_338*ORcp40_239-OMcp40_339*ORcp40_240-...
 OMcp40_340*ORcp40_241-OMcp40_36*ORcp40_231+OPcp40_233*RLcp40_334+OPcp40_234*RLcp40_335+OPcp40_237*RLcp40_338+OPcp40_238*RLcp40_339+OPcp40_239*...
 RLcp40_340+OPcp40_240*RLcp40_341+OPcp40_26*RLcp40_331-OPcp40_333*RLcp40_234-OPcp40_334*RLcp40_235-OPcp40_337*RLcp40_238-OPcp40_338*RLcp40_239-...
 OPcp40_339*RLcp40_240-OPcp40_340*RLcp40_241-OPcp40_36*RLcp40_231;
    ACcp40_241 = qdd(2)-OMcp40_133*ORcp40_334-OMcp40_134*ORcp40_335-OMcp40_137*ORcp40_338-OMcp40_138*ORcp40_339-OMcp40_139*ORcp40_340-OMcp40_140*...
 ORcp40_341-OMcp40_16*ORcp40_331+OMcp40_333*ORcp40_134+OMcp40_334*ORcp40_135+OMcp40_337*ORcp40_138+OMcp40_338*ORcp40_139+OMcp40_339*ORcp40_140+...
 OMcp40_340*ORcp40_141+OMcp40_36*ORcp40_131-OPcp40_133*RLcp40_334-OPcp40_134*RLcp40_335-OPcp40_137*RLcp40_338-OPcp40_138*RLcp40_339-OPcp40_139*...
 RLcp40_340-OPcp40_140*RLcp40_341-OPcp40_16*RLcp40_331+OPcp40_333*RLcp40_134+OPcp40_334*RLcp40_135+OPcp40_337*RLcp40_138+OPcp40_338*RLcp40_139+...
 OPcp40_339*RLcp40_140+OPcp40_340*RLcp40_141+OPcp40_36*RLcp40_131;
    ACcp40_341 = qdd(3)+OMcp40_133*ORcp40_234+OMcp40_134*ORcp40_235+OMcp40_137*ORcp40_238+OMcp40_138*ORcp40_239+OMcp40_139*ORcp40_240+OMcp40_140*...
 ORcp40_241+OMcp40_16*ORcp40_231-OMcp40_233*ORcp40_134-OMcp40_234*ORcp40_135-OMcp40_237*ORcp40_138-OMcp40_238*ORcp40_139-OMcp40_239*ORcp40_140-...
 OMcp40_240*ORcp40_141-OMcp40_26*ORcp40_131+OPcp40_133*RLcp40_234+OPcp40_134*RLcp40_235+OPcp40_137*RLcp40_238+OPcp40_138*RLcp40_239+OPcp40_139*...
 RLcp40_240+OPcp40_140*RLcp40_241+OPcp40_16*RLcp40_231-OPcp40_233*RLcp40_134-OPcp40_234*RLcp40_135-OPcp40_237*RLcp40_138-OPcp40_238*RLcp40_139-...
 OPcp40_239*RLcp40_140-OPcp40_240*RLcp40_141-OPcp40_26*RLcp40_131;

% = = Block_1_0_0_41_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp40_141;
    sens.P(2) = POcp40_241;
    sens.P(3) = POcp40_341;
    sens.R(1,1) = ROcp40_141;
    sens.R(1,2) = ROcp40_241;
    sens.R(1,3) = ROcp40_341;
    sens.R(2,1) = ROcp40_441;
    sens.R(2,2) = ROcp40_541;
    sens.R(2,3) = ROcp40_641;
    sens.R(3,1) = ROcp40_740;
    sens.R(3,2) = ROcp40_840;
    sens.R(3,3) = ROcp40_940;
    sens.V(1) = VIcp40_141;
    sens.V(2) = VIcp40_241;
    sens.V(3) = VIcp40_341;
    sens.OM(1) = OMcp40_141;
    sens.OM(2) = OMcp40_241;
    sens.OM(3) = OMcp40_341;
    sens.A(1) = ACcp40_141;
    sens.A(2) = ACcp40_241;
    sens.A(3) = ACcp40_341;
    sens.OMP(1) = OPcp40_141;
    sens.OMP(2) = OPcp40_241;
    sens.OMP(3) = OPcp40_341;
 
% 
case 42, 


% = = Block_1_0_0_42_0_1 = = 
 
% Sensor Kinematics 


    ROcp41_25 = S4*S5;
    ROcp41_35 = -C4*S5;
    ROcp41_85 = -S4*C5;
    ROcp41_95 = C4*C5;
    ROcp41_16 = C5*C6;
    ROcp41_26 = ROcp41_25*C6+C4*S6;
    ROcp41_36 = ROcp41_35*C6+S4*S6;
    ROcp41_46 = -C5*S6;
    ROcp41_56 = -(ROcp41_25*S6-C4*C6);
    ROcp41_66 = -(ROcp41_35*S6-S4*C6);
    OMcp41_25 = qd(5)*C4;
    OMcp41_35 = qd(5)*S4;
    OMcp41_16 = qd(4)+qd(6)*S5;
    OMcp41_26 = OMcp41_25+ROcp41_85*qd(6);
    OMcp41_36 = OMcp41_35+ROcp41_95*qd(6);
    OPcp41_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp41_26 = ROcp41_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp41_35*S5-ROcp41_95*qd(4));
    OPcp41_36 = ROcp41_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp41_25*S5-ROcp41_85*qd(4));

% = = Block_1_0_0_42_0_4 = = 
 
% Sensor Kinematics 


    ROcp41_131 = ROcp41_16*C31-S31*S5;
    ROcp41_231 = ROcp41_26*C31-ROcp41_85*S31;
    ROcp41_331 = ROcp41_36*C31-ROcp41_95*S31;
    ROcp41_731 = ROcp41_16*S31+C31*S5;
    ROcp41_831 = ROcp41_26*S31+ROcp41_85*C31;
    ROcp41_931 = ROcp41_36*S31+ROcp41_95*C31;
    ROcp41_432 = ROcp41_46*C32+ROcp41_731*S32;
    ROcp41_532 = ROcp41_56*C32+ROcp41_831*S32;
    ROcp41_632 = ROcp41_66*C32+ROcp41_931*S32;
    ROcp41_732 = -(ROcp41_46*S32-ROcp41_731*C32);
    ROcp41_832 = -(ROcp41_56*S32-ROcp41_831*C32);
    ROcp41_932 = -(ROcp41_66*S32-ROcp41_931*C32);
    ROcp41_133 = ROcp41_131*C33-ROcp41_732*S33;
    ROcp41_233 = ROcp41_231*C33-ROcp41_832*S33;
    ROcp41_333 = ROcp41_331*C33-ROcp41_932*S33;
    ROcp41_733 = ROcp41_131*S33+ROcp41_732*C33;
    ROcp41_833 = ROcp41_231*S33+ROcp41_832*C33;
    ROcp41_933 = ROcp41_331*S33+ROcp41_932*C33;
    ROcp41_134 = ROcp41_133*C34+ROcp41_432*S34;
    ROcp41_234 = ROcp41_233*C34+ROcp41_532*S34;
    ROcp41_334 = ROcp41_333*C34+ROcp41_632*S34;
    ROcp41_434 = -(ROcp41_133*S34-ROcp41_432*C34);
    ROcp41_534 = -(ROcp41_233*S34-ROcp41_532*C34);
    ROcp41_634 = -(ROcp41_333*S34-ROcp41_632*C34);
    RLcp41_131 = ROcp41_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp41_231 = ROcp41_26*s.dpt(1,3)+ROcp41_85*s.dpt(3,3);
    RLcp41_331 = ROcp41_36*s.dpt(1,3)+ROcp41_95*s.dpt(3,3);
    OMcp41_131 = OMcp41_16+ROcp41_46*qd(31);
    OMcp41_231 = OMcp41_26+ROcp41_56*qd(31);
    OMcp41_331 = OMcp41_36+ROcp41_66*qd(31);
    ORcp41_131 = OMcp41_26*RLcp41_331-OMcp41_36*RLcp41_231;
    ORcp41_231 = -(OMcp41_16*RLcp41_331-OMcp41_36*RLcp41_131);
    ORcp41_331 = OMcp41_16*RLcp41_231-OMcp41_26*RLcp41_131;
    OMcp41_132 = OMcp41_131+ROcp41_131*qd(32);
    OMcp41_232 = OMcp41_231+ROcp41_231*qd(32);
    OMcp41_332 = OMcp41_331+ROcp41_331*qd(32);
    OMcp41_133 = OMcp41_132+ROcp41_432*qd(33);
    OMcp41_233 = OMcp41_232+ROcp41_532*qd(33);
    OMcp41_333 = OMcp41_332+ROcp41_632*qd(33);
    OPcp41_133 = OPcp41_16+ROcp41_131*qdd(32)+ROcp41_432*qdd(33)+ROcp41_46*qdd(31)+qd(31)*(OMcp41_26*ROcp41_66-OMcp41_36*ROcp41_56)+qd(32)*(...
 OMcp41_231*ROcp41_331-OMcp41_331*ROcp41_231)+qd(33)*(OMcp41_232*ROcp41_632-OMcp41_332*ROcp41_532);
    OPcp41_233 = OPcp41_26+ROcp41_231*qdd(32)+ROcp41_532*qdd(33)+ROcp41_56*qdd(31)-qd(31)*(OMcp41_16*ROcp41_66-OMcp41_36*ROcp41_46)-qd(32)*(...
 OMcp41_131*ROcp41_331-OMcp41_331*ROcp41_131)-qd(33)*(OMcp41_132*ROcp41_632-OMcp41_332*ROcp41_432);
    OPcp41_333 = OPcp41_36+ROcp41_331*qdd(32)+ROcp41_632*qdd(33)+ROcp41_66*qdd(31)+qd(31)*(OMcp41_16*ROcp41_56-OMcp41_26*ROcp41_46)+qd(32)*(...
 OMcp41_131*ROcp41_231-OMcp41_231*ROcp41_131)+qd(33)*(OMcp41_132*ROcp41_532-OMcp41_232*ROcp41_432);
    RLcp41_134 = ROcp41_733*s.dpt(3,21);
    RLcp41_234 = ROcp41_833*s.dpt(3,21);
    RLcp41_334 = ROcp41_933*s.dpt(3,21);
    OMcp41_134 = OMcp41_133+ROcp41_733*qd(34);
    OMcp41_234 = OMcp41_233+ROcp41_833*qd(34);
    OMcp41_334 = OMcp41_333+ROcp41_933*qd(34);
    ORcp41_134 = OMcp41_233*RLcp41_334-OMcp41_333*RLcp41_234;
    ORcp41_234 = -(OMcp41_133*RLcp41_334-OMcp41_333*RLcp41_134);
    ORcp41_334 = OMcp41_133*RLcp41_234-OMcp41_233*RLcp41_134;
    OPcp41_134 = OPcp41_133+ROcp41_733*qdd(34)+qd(34)*(OMcp41_233*ROcp41_933-OMcp41_333*ROcp41_833);
    OPcp41_234 = OPcp41_233+ROcp41_833*qdd(34)-qd(34)*(OMcp41_133*ROcp41_933-OMcp41_333*ROcp41_733);
    OPcp41_334 = OPcp41_333+ROcp41_933*qdd(34)+qd(34)*(OMcp41_133*ROcp41_833-OMcp41_233*ROcp41_733);

% = = Block_1_0_0_42_0_5 = = 
 
% Sensor Kinematics 


    ROcp41_435 = ROcp41_434*C35+ROcp41_733*S35;
    ROcp41_535 = ROcp41_534*C35+ROcp41_833*S35;
    ROcp41_635 = ROcp41_634*C35+ROcp41_933*S35;
    ROcp41_735 = -(ROcp41_434*S35-ROcp41_733*C35);
    ROcp41_835 = -(ROcp41_534*S35-ROcp41_833*C35);
    ROcp41_935 = -(ROcp41_634*S35-ROcp41_933*C35);
    ROcp41_136 = ROcp41_134*C36+ROcp41_435*S36;
    ROcp41_236 = ROcp41_234*C36+ROcp41_535*S36;
    ROcp41_336 = ROcp41_334*C36+ROcp41_635*S36;
    ROcp41_436 = -(ROcp41_134*S36-ROcp41_435*C36);
    ROcp41_536 = -(ROcp41_234*S36-ROcp41_535*C36);
    ROcp41_636 = -(ROcp41_334*S36-ROcp41_635*C36);
    ROcp41_137 = ROcp41_136*C37-ROcp41_735*S37;
    ROcp41_237 = ROcp41_236*C37-ROcp41_835*S37;
    ROcp41_337 = ROcp41_336*C37-ROcp41_935*S37;
    ROcp41_737 = ROcp41_136*S37+ROcp41_735*C37;
    ROcp41_837 = ROcp41_236*S37+ROcp41_835*C37;
    ROcp41_937 = ROcp41_336*S37+ROcp41_935*C37;
    ROcp41_438 = ROcp41_436*C38+ROcp41_737*S38;
    ROcp41_538 = ROcp41_536*C38+ROcp41_837*S38;
    ROcp41_638 = ROcp41_636*C38+ROcp41_937*S38;
    ROcp41_738 = -(ROcp41_436*S38-ROcp41_737*C38);
    ROcp41_838 = -(ROcp41_536*S38-ROcp41_837*C38);
    ROcp41_938 = -(ROcp41_636*S38-ROcp41_937*C38);
    ROcp41_139 = ROcp41_137*C39+ROcp41_438*S39;
    ROcp41_239 = ROcp41_237*C39+ROcp41_538*S39;
    ROcp41_339 = ROcp41_337*C39+ROcp41_638*S39;
    ROcp41_439 = -(ROcp41_137*S39-ROcp41_438*C39);
    ROcp41_539 = -(ROcp41_237*S39-ROcp41_538*C39);
    ROcp41_639 = -(ROcp41_337*S39-ROcp41_638*C39);
    ROcp41_140 = ROcp41_139*C40-ROcp41_738*S40;
    ROcp41_240 = ROcp41_239*C40-ROcp41_838*S40;
    ROcp41_340 = ROcp41_339*C40-ROcp41_938*S40;
    ROcp41_740 = ROcp41_139*S40+ROcp41_738*C40;
    ROcp41_840 = ROcp41_239*S40+ROcp41_838*C40;
    ROcp41_940 = ROcp41_339*S40+ROcp41_938*C40;
    ROcp41_141 = ROcp41_140*C41+ROcp41_439*S41;
    ROcp41_241 = ROcp41_240*C41+ROcp41_539*S41;
    ROcp41_341 = ROcp41_340*C41+ROcp41_639*S41;
    ROcp41_441 = -(ROcp41_140*S41-ROcp41_439*C41);
    ROcp41_541 = -(ROcp41_240*S41-ROcp41_539*C41);
    ROcp41_641 = -(ROcp41_340*S41-ROcp41_639*C41);
    ROcp41_142 = ROcp41_141*C42-ROcp41_740*S42;
    ROcp41_242 = ROcp41_241*C42-ROcp41_840*S42;
    ROcp41_342 = ROcp41_341*C42-ROcp41_940*S42;
    ROcp41_742 = ROcp41_141*S42+ROcp41_740*C42;
    ROcp41_842 = ROcp41_241*S42+ROcp41_840*C42;
    ROcp41_942 = ROcp41_341*S42+ROcp41_940*C42;
    RLcp41_135 = ROcp41_134*s.dpt(1,23)+ROcp41_434*s.dpt(2,23)+ROcp41_733*s.dpt(3,23);
    RLcp41_235 = ROcp41_234*s.dpt(1,23)+ROcp41_534*s.dpt(2,23)+ROcp41_833*s.dpt(3,23);
    RLcp41_335 = ROcp41_334*s.dpt(1,23)+ROcp41_634*s.dpt(2,23)+ROcp41_933*s.dpt(3,23);
    ORcp41_135 = OMcp41_234*RLcp41_335-OMcp41_334*RLcp41_235;
    ORcp41_235 = -(OMcp41_134*RLcp41_335-OMcp41_334*RLcp41_135);
    ORcp41_335 = OMcp41_134*RLcp41_235-OMcp41_234*RLcp41_135;
    OMcp41_137 = OMcp41_134+ROcp41_436*qd(37);
    OMcp41_237 = OMcp41_234+ROcp41_536*qd(37);
    OMcp41_337 = OMcp41_334+ROcp41_636*qd(37);
    OPcp41_137 = OPcp41_134+ROcp41_436*qdd(37)+qd(37)*(OMcp41_234*ROcp41_636-OMcp41_334*ROcp41_536);
    OPcp41_237 = OPcp41_234+ROcp41_536*qdd(37)-qd(37)*(OMcp41_134*ROcp41_636-OMcp41_334*ROcp41_436);
    OPcp41_337 = OPcp41_334+ROcp41_636*qdd(37)+qd(37)*(OMcp41_134*ROcp41_536-OMcp41_234*ROcp41_436);
    RLcp41_138 = ROcp41_436*s.dpt(2,27)+ROcp41_737*s.dpt(3,27);
    RLcp41_238 = ROcp41_536*s.dpt(2,27)+ROcp41_837*s.dpt(3,27);
    RLcp41_338 = ROcp41_636*s.dpt(2,27)+ROcp41_937*s.dpt(3,27);
    OMcp41_138 = OMcp41_137+ROcp41_137*qd(38);
    OMcp41_238 = OMcp41_237+ROcp41_237*qd(38);
    OMcp41_338 = OMcp41_337+ROcp41_337*qd(38);
    ORcp41_138 = OMcp41_237*RLcp41_338-OMcp41_337*RLcp41_238;
    ORcp41_238 = -(OMcp41_137*RLcp41_338-OMcp41_337*RLcp41_138);
    ORcp41_338 = OMcp41_137*RLcp41_238-OMcp41_237*RLcp41_138;
    OPcp41_138 = OPcp41_137+ROcp41_137*qdd(38)+qd(38)*(OMcp41_237*ROcp41_337-OMcp41_337*ROcp41_237);
    OPcp41_238 = OPcp41_237+ROcp41_237*qdd(38)-qd(38)*(OMcp41_137*ROcp41_337-OMcp41_337*ROcp41_137);
    OPcp41_338 = OPcp41_337+ROcp41_337*qdd(38)+qd(38)*(OMcp41_137*ROcp41_237-OMcp41_237*ROcp41_137);
    RLcp41_139 = ROcp41_738*s.dpt(3,29);
    RLcp41_239 = ROcp41_838*s.dpt(3,29);
    RLcp41_339 = ROcp41_938*s.dpt(3,29);
    OMcp41_139 = OMcp41_138+ROcp41_738*qd(39);
    OMcp41_239 = OMcp41_238+ROcp41_838*qd(39);
    OMcp41_339 = OMcp41_338+ROcp41_938*qd(39);
    ORcp41_139 = OMcp41_238*RLcp41_339-OMcp41_338*RLcp41_239;
    ORcp41_239 = -(OMcp41_138*RLcp41_339-OMcp41_338*RLcp41_139);
    ORcp41_339 = OMcp41_138*RLcp41_239-OMcp41_238*RLcp41_139;
    OPcp41_139 = OPcp41_138+ROcp41_738*qdd(39)+qd(39)*(OMcp41_238*ROcp41_938-OMcp41_338*ROcp41_838);
    OPcp41_239 = OPcp41_238+ROcp41_838*qdd(39)-qd(39)*(OMcp41_138*ROcp41_938-OMcp41_338*ROcp41_738);
    OPcp41_339 = OPcp41_338+ROcp41_938*qdd(39)+qd(39)*(OMcp41_138*ROcp41_838-OMcp41_238*ROcp41_738);
    RLcp41_140 = ROcp41_139*s.dpt(1,31)+ROcp41_738*s.dpt(3,31);
    RLcp41_240 = ROcp41_239*s.dpt(1,31)+ROcp41_838*s.dpt(3,31);
    RLcp41_340 = ROcp41_339*s.dpt(1,31)+ROcp41_938*s.dpt(3,31);
    OMcp41_140 = OMcp41_139+ROcp41_439*qd(40);
    OMcp41_240 = OMcp41_239+ROcp41_539*qd(40);
    OMcp41_340 = OMcp41_339+ROcp41_639*qd(40);
    ORcp41_140 = OMcp41_239*RLcp41_340-OMcp41_339*RLcp41_240;
    ORcp41_240 = -(OMcp41_139*RLcp41_340-OMcp41_339*RLcp41_140);
    ORcp41_340 = OMcp41_139*RLcp41_240-OMcp41_239*RLcp41_140;
    OPcp41_140 = OPcp41_139+ROcp41_439*qdd(40)+qd(40)*(OMcp41_239*ROcp41_639-OMcp41_339*ROcp41_539);
    OPcp41_240 = OPcp41_239+ROcp41_539*qdd(40)-qd(40)*(OMcp41_139*ROcp41_639-OMcp41_339*ROcp41_439);
    OPcp41_340 = OPcp41_339+ROcp41_639*qdd(40)+qd(40)*(OMcp41_139*ROcp41_539-OMcp41_239*ROcp41_439);
    RLcp41_141 = ROcp41_140*s.dpt(1,33)+ROcp41_740*s.dpt(3,33);
    RLcp41_241 = ROcp41_240*s.dpt(1,33)+ROcp41_840*s.dpt(3,33);
    RLcp41_341 = ROcp41_340*s.dpt(1,33)+ROcp41_940*s.dpt(3,33);
    POcp41_141 = RLcp41_131+RLcp41_134+RLcp41_135+RLcp41_138+RLcp41_139+RLcp41_140+RLcp41_141+q(1);
    POcp41_241 = RLcp41_231+RLcp41_234+RLcp41_235+RLcp41_238+RLcp41_239+RLcp41_240+RLcp41_241+q(2);
    POcp41_341 = RLcp41_331+RLcp41_334+RLcp41_335+RLcp41_338+RLcp41_339+RLcp41_340+RLcp41_341+q(3);
    OMcp41_141 = OMcp41_140+ROcp41_740*qd(41);
    OMcp41_241 = OMcp41_240+ROcp41_840*qd(41);
    OMcp41_341 = OMcp41_340+ROcp41_940*qd(41);
    ORcp41_141 = OMcp41_240*RLcp41_341-OMcp41_340*RLcp41_241;
    ORcp41_241 = -(OMcp41_140*RLcp41_341-OMcp41_340*RLcp41_141);
    ORcp41_341 = OMcp41_140*RLcp41_241-OMcp41_240*RLcp41_141;
    VIcp41_141 = ORcp41_131+ORcp41_134+ORcp41_135+ORcp41_138+ORcp41_139+ORcp41_140+ORcp41_141+qd(1);
    VIcp41_241 = ORcp41_231+ORcp41_234+ORcp41_235+ORcp41_238+ORcp41_239+ORcp41_240+ORcp41_241+qd(2);
    VIcp41_341 = ORcp41_331+ORcp41_334+ORcp41_335+ORcp41_338+ORcp41_339+ORcp41_340+ORcp41_341+qd(3);
    ACcp41_141 = qdd(1)+OMcp41_233*ORcp41_334+OMcp41_234*ORcp41_335+OMcp41_237*ORcp41_338+OMcp41_238*ORcp41_339+OMcp41_239*ORcp41_340+OMcp41_240*...
 ORcp41_341+OMcp41_26*ORcp41_331-OMcp41_333*ORcp41_234-OMcp41_334*ORcp41_235-OMcp41_337*ORcp41_238-OMcp41_338*ORcp41_239-OMcp41_339*ORcp41_240-...
 OMcp41_340*ORcp41_241-OMcp41_36*ORcp41_231+OPcp41_233*RLcp41_334+OPcp41_234*RLcp41_335+OPcp41_237*RLcp41_338+OPcp41_238*RLcp41_339+OPcp41_239*...
 RLcp41_340+OPcp41_240*RLcp41_341+OPcp41_26*RLcp41_331-OPcp41_333*RLcp41_234-OPcp41_334*RLcp41_235-OPcp41_337*RLcp41_238-OPcp41_338*RLcp41_239-...
 OPcp41_339*RLcp41_240-OPcp41_340*RLcp41_241-OPcp41_36*RLcp41_231;
    ACcp41_241 = qdd(2)-OMcp41_133*ORcp41_334-OMcp41_134*ORcp41_335-OMcp41_137*ORcp41_338-OMcp41_138*ORcp41_339-OMcp41_139*ORcp41_340-OMcp41_140*...
 ORcp41_341-OMcp41_16*ORcp41_331+OMcp41_333*ORcp41_134+OMcp41_334*ORcp41_135+OMcp41_337*ORcp41_138+OMcp41_338*ORcp41_139+OMcp41_339*ORcp41_140+...
 OMcp41_340*ORcp41_141+OMcp41_36*ORcp41_131-OPcp41_133*RLcp41_334-OPcp41_134*RLcp41_335-OPcp41_137*RLcp41_338-OPcp41_138*RLcp41_339-OPcp41_139*...
 RLcp41_340-OPcp41_140*RLcp41_341-OPcp41_16*RLcp41_331+OPcp41_333*RLcp41_134+OPcp41_334*RLcp41_135+OPcp41_337*RLcp41_138+OPcp41_338*RLcp41_139+...
 OPcp41_339*RLcp41_140+OPcp41_340*RLcp41_141+OPcp41_36*RLcp41_131;
    ACcp41_341 = qdd(3)+OMcp41_133*ORcp41_234+OMcp41_134*ORcp41_235+OMcp41_137*ORcp41_238+OMcp41_138*ORcp41_239+OMcp41_139*ORcp41_240+OMcp41_140*...
 ORcp41_241+OMcp41_16*ORcp41_231-OMcp41_233*ORcp41_134-OMcp41_234*ORcp41_135-OMcp41_237*ORcp41_138-OMcp41_238*ORcp41_139-OMcp41_239*ORcp41_140-...
 OMcp41_240*ORcp41_141-OMcp41_26*ORcp41_131+OPcp41_133*RLcp41_234+OPcp41_134*RLcp41_235+OPcp41_137*RLcp41_238+OPcp41_138*RLcp41_239+OPcp41_139*...
 RLcp41_240+OPcp41_140*RLcp41_241+OPcp41_16*RLcp41_231-OPcp41_233*RLcp41_134-OPcp41_234*RLcp41_135-OPcp41_237*RLcp41_138-OPcp41_238*RLcp41_139-...
 OPcp41_239*RLcp41_140-OPcp41_240*RLcp41_141-OPcp41_26*RLcp41_131;
    OMcp41_142 = OMcp41_141+ROcp41_441*qd(42);
    OMcp41_242 = OMcp41_241+ROcp41_541*qd(42);
    OMcp41_342 = OMcp41_341+ROcp41_641*qd(42);
    OPcp41_142 = OPcp41_140+ROcp41_441*qdd(42)+ROcp41_740*qdd(41)+qd(41)*(OMcp41_240*ROcp41_940-OMcp41_340*ROcp41_840)+qd(42)*(OMcp41_241*...
 ROcp41_641-OMcp41_341*ROcp41_541);
    OPcp41_242 = OPcp41_240+ROcp41_541*qdd(42)+ROcp41_840*qdd(41)-qd(41)*(OMcp41_140*ROcp41_940-OMcp41_340*ROcp41_740)-qd(42)*(OMcp41_141*...
 ROcp41_641-OMcp41_341*ROcp41_441);
    OPcp41_342 = OPcp41_340+ROcp41_641*qdd(42)+ROcp41_940*qdd(41)+qd(41)*(OMcp41_140*ROcp41_840-OMcp41_240*ROcp41_740)+qd(42)*(OMcp41_141*...
 ROcp41_541-OMcp41_241*ROcp41_441);

% = = Block_1_0_0_42_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp41_141;
    sens.P(2) = POcp41_241;
    sens.P(3) = POcp41_341;
    sens.R(1,1) = ROcp41_142;
    sens.R(1,2) = ROcp41_242;
    sens.R(1,3) = ROcp41_342;
    sens.R(2,1) = ROcp41_441;
    sens.R(2,2) = ROcp41_541;
    sens.R(2,3) = ROcp41_641;
    sens.R(3,1) = ROcp41_742;
    sens.R(3,2) = ROcp41_842;
    sens.R(3,3) = ROcp41_942;
    sens.V(1) = VIcp41_141;
    sens.V(2) = VIcp41_241;
    sens.V(3) = VIcp41_341;
    sens.OM(1) = OMcp41_142;
    sens.OM(2) = OMcp41_242;
    sens.OM(3) = OMcp41_342;
    sens.A(1) = ACcp41_141;
    sens.A(2) = ACcp41_241;
    sens.A(3) = ACcp41_341;
    sens.OMP(1) = OPcp41_142;
    sens.OMP(2) = OPcp41_242;
    sens.OMP(3) = OPcp41_342;
 
% 
case 43, 


% = = Block_1_0_0_43_0_1 = = 
 
% Sensor Kinematics 


    ROcp42_25 = S4*S5;
    ROcp42_35 = -C4*S5;
    ROcp42_85 = -S4*C5;
    ROcp42_95 = C4*C5;
    ROcp42_16 = C5*C6;
    ROcp42_26 = ROcp42_25*C6+C4*S6;
    ROcp42_36 = ROcp42_35*C6+S4*S6;
    ROcp42_46 = -C5*S6;
    ROcp42_56 = -(ROcp42_25*S6-C4*C6);
    ROcp42_66 = -(ROcp42_35*S6-S4*C6);
    OMcp42_25 = qd(5)*C4;
    OMcp42_35 = qd(5)*S4;
    OMcp42_16 = qd(4)+qd(6)*S5;
    OMcp42_26 = OMcp42_25+ROcp42_85*qd(6);
    OMcp42_36 = OMcp42_35+ROcp42_95*qd(6);
    OPcp42_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp42_26 = ROcp42_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp42_35*S5-ROcp42_95*qd(4));
    OPcp42_36 = ROcp42_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp42_25*S5-ROcp42_85*qd(4));

% = = Block_1_0_0_43_0_4 = = 
 
% Sensor Kinematics 


    ROcp42_131 = ROcp42_16*C31-S31*S5;
    ROcp42_231 = ROcp42_26*C31-ROcp42_85*S31;
    ROcp42_331 = ROcp42_36*C31-ROcp42_95*S31;
    ROcp42_731 = ROcp42_16*S31+C31*S5;
    ROcp42_831 = ROcp42_26*S31+ROcp42_85*C31;
    ROcp42_931 = ROcp42_36*S31+ROcp42_95*C31;
    ROcp42_432 = ROcp42_46*C32+ROcp42_731*S32;
    ROcp42_532 = ROcp42_56*C32+ROcp42_831*S32;
    ROcp42_632 = ROcp42_66*C32+ROcp42_931*S32;
    ROcp42_732 = -(ROcp42_46*S32-ROcp42_731*C32);
    ROcp42_832 = -(ROcp42_56*S32-ROcp42_831*C32);
    ROcp42_932 = -(ROcp42_66*S32-ROcp42_931*C32);
    ROcp42_133 = ROcp42_131*C33-ROcp42_732*S33;
    ROcp42_233 = ROcp42_231*C33-ROcp42_832*S33;
    ROcp42_333 = ROcp42_331*C33-ROcp42_932*S33;
    ROcp42_733 = ROcp42_131*S33+ROcp42_732*C33;
    ROcp42_833 = ROcp42_231*S33+ROcp42_832*C33;
    ROcp42_933 = ROcp42_331*S33+ROcp42_932*C33;
    ROcp42_134 = ROcp42_133*C34+ROcp42_432*S34;
    ROcp42_234 = ROcp42_233*C34+ROcp42_532*S34;
    ROcp42_334 = ROcp42_333*C34+ROcp42_632*S34;
    ROcp42_434 = -(ROcp42_133*S34-ROcp42_432*C34);
    ROcp42_534 = -(ROcp42_233*S34-ROcp42_532*C34);
    ROcp42_634 = -(ROcp42_333*S34-ROcp42_632*C34);
    RLcp42_131 = ROcp42_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp42_231 = ROcp42_26*s.dpt(1,3)+ROcp42_85*s.dpt(3,3);
    RLcp42_331 = ROcp42_36*s.dpt(1,3)+ROcp42_95*s.dpt(3,3);
    OMcp42_131 = OMcp42_16+ROcp42_46*qd(31);
    OMcp42_231 = OMcp42_26+ROcp42_56*qd(31);
    OMcp42_331 = OMcp42_36+ROcp42_66*qd(31);
    ORcp42_131 = OMcp42_26*RLcp42_331-OMcp42_36*RLcp42_231;
    ORcp42_231 = -(OMcp42_16*RLcp42_331-OMcp42_36*RLcp42_131);
    ORcp42_331 = OMcp42_16*RLcp42_231-OMcp42_26*RLcp42_131;
    OMcp42_132 = OMcp42_131+ROcp42_131*qd(32);
    OMcp42_232 = OMcp42_231+ROcp42_231*qd(32);
    OMcp42_332 = OMcp42_331+ROcp42_331*qd(32);
    OMcp42_133 = OMcp42_132+ROcp42_432*qd(33);
    OMcp42_233 = OMcp42_232+ROcp42_532*qd(33);
    OMcp42_333 = OMcp42_332+ROcp42_632*qd(33);
    OPcp42_133 = OPcp42_16+ROcp42_131*qdd(32)+ROcp42_432*qdd(33)+ROcp42_46*qdd(31)+qd(31)*(OMcp42_26*ROcp42_66-OMcp42_36*ROcp42_56)+qd(32)*(...
 OMcp42_231*ROcp42_331-OMcp42_331*ROcp42_231)+qd(33)*(OMcp42_232*ROcp42_632-OMcp42_332*ROcp42_532);
    OPcp42_233 = OPcp42_26+ROcp42_231*qdd(32)+ROcp42_532*qdd(33)+ROcp42_56*qdd(31)-qd(31)*(OMcp42_16*ROcp42_66-OMcp42_36*ROcp42_46)-qd(32)*(...
 OMcp42_131*ROcp42_331-OMcp42_331*ROcp42_131)-qd(33)*(OMcp42_132*ROcp42_632-OMcp42_332*ROcp42_432);
    OPcp42_333 = OPcp42_36+ROcp42_331*qdd(32)+ROcp42_632*qdd(33)+ROcp42_66*qdd(31)+qd(31)*(OMcp42_16*ROcp42_56-OMcp42_26*ROcp42_46)+qd(32)*(...
 OMcp42_131*ROcp42_231-OMcp42_231*ROcp42_131)+qd(33)*(OMcp42_132*ROcp42_532-OMcp42_232*ROcp42_432);
    RLcp42_134 = ROcp42_733*s.dpt(3,21);
    RLcp42_234 = ROcp42_833*s.dpt(3,21);
    RLcp42_334 = ROcp42_933*s.dpt(3,21);
    OMcp42_134 = OMcp42_133+ROcp42_733*qd(34);
    OMcp42_234 = OMcp42_233+ROcp42_833*qd(34);
    OMcp42_334 = OMcp42_333+ROcp42_933*qd(34);
    ORcp42_134 = OMcp42_233*RLcp42_334-OMcp42_333*RLcp42_234;
    ORcp42_234 = -(OMcp42_133*RLcp42_334-OMcp42_333*RLcp42_134);
    ORcp42_334 = OMcp42_133*RLcp42_234-OMcp42_233*RLcp42_134;
    OPcp42_134 = OPcp42_133+ROcp42_733*qdd(34)+qd(34)*(OMcp42_233*ROcp42_933-OMcp42_333*ROcp42_833);
    OPcp42_234 = OPcp42_233+ROcp42_833*qdd(34)-qd(34)*(OMcp42_133*ROcp42_933-OMcp42_333*ROcp42_733);
    OPcp42_334 = OPcp42_333+ROcp42_933*qdd(34)+qd(34)*(OMcp42_133*ROcp42_833-OMcp42_233*ROcp42_733);

% = = Block_1_0_0_43_0_5 = = 
 
% Sensor Kinematics 


    ROcp42_435 = ROcp42_434*C35+ROcp42_733*S35;
    ROcp42_535 = ROcp42_534*C35+ROcp42_833*S35;
    ROcp42_635 = ROcp42_634*C35+ROcp42_933*S35;
    ROcp42_735 = -(ROcp42_434*S35-ROcp42_733*C35);
    ROcp42_835 = -(ROcp42_534*S35-ROcp42_833*C35);
    ROcp42_935 = -(ROcp42_634*S35-ROcp42_933*C35);
    ROcp42_136 = ROcp42_134*C36+ROcp42_435*S36;
    ROcp42_236 = ROcp42_234*C36+ROcp42_535*S36;
    ROcp42_336 = ROcp42_334*C36+ROcp42_635*S36;
    ROcp42_436 = -(ROcp42_134*S36-ROcp42_435*C36);
    ROcp42_536 = -(ROcp42_234*S36-ROcp42_535*C36);
    ROcp42_636 = -(ROcp42_334*S36-ROcp42_635*C36);
    ROcp42_137 = ROcp42_136*C37-ROcp42_735*S37;
    ROcp42_237 = ROcp42_236*C37-ROcp42_835*S37;
    ROcp42_337 = ROcp42_336*C37-ROcp42_935*S37;
    ROcp42_737 = ROcp42_136*S37+ROcp42_735*C37;
    ROcp42_837 = ROcp42_236*S37+ROcp42_835*C37;
    ROcp42_937 = ROcp42_336*S37+ROcp42_935*C37;
    ROcp42_438 = ROcp42_436*C38+ROcp42_737*S38;
    ROcp42_538 = ROcp42_536*C38+ROcp42_837*S38;
    ROcp42_638 = ROcp42_636*C38+ROcp42_937*S38;
    ROcp42_738 = -(ROcp42_436*S38-ROcp42_737*C38);
    ROcp42_838 = -(ROcp42_536*S38-ROcp42_837*C38);
    ROcp42_938 = -(ROcp42_636*S38-ROcp42_937*C38);
    ROcp42_139 = ROcp42_137*C39+ROcp42_438*S39;
    ROcp42_239 = ROcp42_237*C39+ROcp42_538*S39;
    ROcp42_339 = ROcp42_337*C39+ROcp42_638*S39;
    ROcp42_439 = -(ROcp42_137*S39-ROcp42_438*C39);
    ROcp42_539 = -(ROcp42_237*S39-ROcp42_538*C39);
    ROcp42_639 = -(ROcp42_337*S39-ROcp42_638*C39);
    ROcp42_140 = ROcp42_139*C40-ROcp42_738*S40;
    ROcp42_240 = ROcp42_239*C40-ROcp42_838*S40;
    ROcp42_340 = ROcp42_339*C40-ROcp42_938*S40;
    ROcp42_740 = ROcp42_139*S40+ROcp42_738*C40;
    ROcp42_840 = ROcp42_239*S40+ROcp42_838*C40;
    ROcp42_940 = ROcp42_339*S40+ROcp42_938*C40;
    ROcp42_141 = ROcp42_140*C41+ROcp42_439*S41;
    ROcp42_241 = ROcp42_240*C41+ROcp42_539*S41;
    ROcp42_341 = ROcp42_340*C41+ROcp42_639*S41;
    ROcp42_441 = -(ROcp42_140*S41-ROcp42_439*C41);
    ROcp42_541 = -(ROcp42_240*S41-ROcp42_539*C41);
    ROcp42_641 = -(ROcp42_340*S41-ROcp42_639*C41);
    ROcp42_142 = ROcp42_141*C42-ROcp42_740*S42;
    ROcp42_242 = ROcp42_241*C42-ROcp42_840*S42;
    ROcp42_342 = ROcp42_341*C42-ROcp42_940*S42;
    ROcp42_742 = ROcp42_141*S42+ROcp42_740*C42;
    ROcp42_842 = ROcp42_241*S42+ROcp42_840*C42;
    ROcp42_942 = ROcp42_341*S42+ROcp42_940*C42;
    ROcp42_443 = ROcp42_441*C43+ROcp42_742*S43;
    ROcp42_543 = ROcp42_541*C43+ROcp42_842*S43;
    ROcp42_643 = ROcp42_641*C43+ROcp42_942*S43;
    ROcp42_743 = -(ROcp42_441*S43-ROcp42_742*C43);
    ROcp42_843 = -(ROcp42_541*S43-ROcp42_842*C43);
    ROcp42_943 = -(ROcp42_641*S43-ROcp42_942*C43);
    RLcp42_135 = ROcp42_134*s.dpt(1,23)+ROcp42_434*s.dpt(2,23)+ROcp42_733*s.dpt(3,23);
    RLcp42_235 = ROcp42_234*s.dpt(1,23)+ROcp42_534*s.dpt(2,23)+ROcp42_833*s.dpt(3,23);
    RLcp42_335 = ROcp42_334*s.dpt(1,23)+ROcp42_634*s.dpt(2,23)+ROcp42_933*s.dpt(3,23);
    ORcp42_135 = OMcp42_234*RLcp42_335-OMcp42_334*RLcp42_235;
    ORcp42_235 = -(OMcp42_134*RLcp42_335-OMcp42_334*RLcp42_135);
    ORcp42_335 = OMcp42_134*RLcp42_235-OMcp42_234*RLcp42_135;
    OMcp42_137 = OMcp42_134+ROcp42_436*qd(37);
    OMcp42_237 = OMcp42_234+ROcp42_536*qd(37);
    OMcp42_337 = OMcp42_334+ROcp42_636*qd(37);
    OPcp42_137 = OPcp42_134+ROcp42_436*qdd(37)+qd(37)*(OMcp42_234*ROcp42_636-OMcp42_334*ROcp42_536);
    OPcp42_237 = OPcp42_234+ROcp42_536*qdd(37)-qd(37)*(OMcp42_134*ROcp42_636-OMcp42_334*ROcp42_436);
    OPcp42_337 = OPcp42_334+ROcp42_636*qdd(37)+qd(37)*(OMcp42_134*ROcp42_536-OMcp42_234*ROcp42_436);
    RLcp42_138 = ROcp42_436*s.dpt(2,27)+ROcp42_737*s.dpt(3,27);
    RLcp42_238 = ROcp42_536*s.dpt(2,27)+ROcp42_837*s.dpt(3,27);
    RLcp42_338 = ROcp42_636*s.dpt(2,27)+ROcp42_937*s.dpt(3,27);
    OMcp42_138 = OMcp42_137+ROcp42_137*qd(38);
    OMcp42_238 = OMcp42_237+ROcp42_237*qd(38);
    OMcp42_338 = OMcp42_337+ROcp42_337*qd(38);
    ORcp42_138 = OMcp42_237*RLcp42_338-OMcp42_337*RLcp42_238;
    ORcp42_238 = -(OMcp42_137*RLcp42_338-OMcp42_337*RLcp42_138);
    ORcp42_338 = OMcp42_137*RLcp42_238-OMcp42_237*RLcp42_138;
    OPcp42_138 = OPcp42_137+ROcp42_137*qdd(38)+qd(38)*(OMcp42_237*ROcp42_337-OMcp42_337*ROcp42_237);
    OPcp42_238 = OPcp42_237+ROcp42_237*qdd(38)-qd(38)*(OMcp42_137*ROcp42_337-OMcp42_337*ROcp42_137);
    OPcp42_338 = OPcp42_337+ROcp42_337*qdd(38)+qd(38)*(OMcp42_137*ROcp42_237-OMcp42_237*ROcp42_137);
    RLcp42_139 = ROcp42_738*s.dpt(3,29);
    RLcp42_239 = ROcp42_838*s.dpt(3,29);
    RLcp42_339 = ROcp42_938*s.dpt(3,29);
    OMcp42_139 = OMcp42_138+ROcp42_738*qd(39);
    OMcp42_239 = OMcp42_238+ROcp42_838*qd(39);
    OMcp42_339 = OMcp42_338+ROcp42_938*qd(39);
    ORcp42_139 = OMcp42_238*RLcp42_339-OMcp42_338*RLcp42_239;
    ORcp42_239 = -(OMcp42_138*RLcp42_339-OMcp42_338*RLcp42_139);
    ORcp42_339 = OMcp42_138*RLcp42_239-OMcp42_238*RLcp42_139;
    OPcp42_139 = OPcp42_138+ROcp42_738*qdd(39)+qd(39)*(OMcp42_238*ROcp42_938-OMcp42_338*ROcp42_838);
    OPcp42_239 = OPcp42_238+ROcp42_838*qdd(39)-qd(39)*(OMcp42_138*ROcp42_938-OMcp42_338*ROcp42_738);
    OPcp42_339 = OPcp42_338+ROcp42_938*qdd(39)+qd(39)*(OMcp42_138*ROcp42_838-OMcp42_238*ROcp42_738);
    RLcp42_140 = ROcp42_139*s.dpt(1,31)+ROcp42_738*s.dpt(3,31);
    RLcp42_240 = ROcp42_239*s.dpt(1,31)+ROcp42_838*s.dpt(3,31);
    RLcp42_340 = ROcp42_339*s.dpt(1,31)+ROcp42_938*s.dpt(3,31);
    OMcp42_140 = OMcp42_139+ROcp42_439*qd(40);
    OMcp42_240 = OMcp42_239+ROcp42_539*qd(40);
    OMcp42_340 = OMcp42_339+ROcp42_639*qd(40);
    ORcp42_140 = OMcp42_239*RLcp42_340-OMcp42_339*RLcp42_240;
    ORcp42_240 = -(OMcp42_139*RLcp42_340-OMcp42_339*RLcp42_140);
    ORcp42_340 = OMcp42_139*RLcp42_240-OMcp42_239*RLcp42_140;
    OPcp42_140 = OPcp42_139+ROcp42_439*qdd(40)+qd(40)*(OMcp42_239*ROcp42_639-OMcp42_339*ROcp42_539);
    OPcp42_240 = OPcp42_239+ROcp42_539*qdd(40)-qd(40)*(OMcp42_139*ROcp42_639-OMcp42_339*ROcp42_439);
    OPcp42_340 = OPcp42_339+ROcp42_639*qdd(40)+qd(40)*(OMcp42_139*ROcp42_539-OMcp42_239*ROcp42_439);
    RLcp42_141 = ROcp42_140*s.dpt(1,33)+ROcp42_740*s.dpt(3,33);
    RLcp42_241 = ROcp42_240*s.dpt(1,33)+ROcp42_840*s.dpt(3,33);
    RLcp42_341 = ROcp42_340*s.dpt(1,33)+ROcp42_940*s.dpt(3,33);
    OMcp42_141 = OMcp42_140+ROcp42_740*qd(41);
    OMcp42_241 = OMcp42_240+ROcp42_840*qd(41);
    OMcp42_341 = OMcp42_340+ROcp42_940*qd(41);
    ORcp42_141 = OMcp42_240*RLcp42_341-OMcp42_340*RLcp42_241;
    ORcp42_241 = -(OMcp42_140*RLcp42_341-OMcp42_340*RLcp42_141);
    ORcp42_341 = OMcp42_140*RLcp42_241-OMcp42_240*RLcp42_141;
    OMcp42_142 = OMcp42_141+ROcp42_441*qd(42);
    OMcp42_242 = OMcp42_241+ROcp42_541*qd(42);
    OMcp42_342 = OMcp42_341+ROcp42_641*qd(42);
    OPcp42_142 = OPcp42_140+ROcp42_441*qdd(42)+ROcp42_740*qdd(41)+qd(41)*(OMcp42_240*ROcp42_940-OMcp42_340*ROcp42_840)+qd(42)*(OMcp42_241*...
 ROcp42_641-OMcp42_341*ROcp42_541);
    OPcp42_242 = OPcp42_240+ROcp42_541*qdd(42)+ROcp42_840*qdd(41)-qd(41)*(OMcp42_140*ROcp42_940-OMcp42_340*ROcp42_740)-qd(42)*(OMcp42_141*...
 ROcp42_641-OMcp42_341*ROcp42_441);
    OPcp42_342 = OPcp42_340+ROcp42_641*qdd(42)+ROcp42_940*qdd(41)+qd(41)*(OMcp42_140*ROcp42_840-OMcp42_240*ROcp42_740)+qd(42)*(OMcp42_141*...
 ROcp42_541-OMcp42_241*ROcp42_441);
    RLcp42_143 = ROcp42_742*s.dpt(3,37);
    RLcp42_243 = ROcp42_842*s.dpt(3,37);
    RLcp42_343 = ROcp42_942*s.dpt(3,37);
    POcp42_143 = RLcp42_131+RLcp42_134+RLcp42_135+RLcp42_138+RLcp42_139+RLcp42_140+RLcp42_141+RLcp42_143+q(1);
    POcp42_243 = RLcp42_231+RLcp42_234+RLcp42_235+RLcp42_238+RLcp42_239+RLcp42_240+RLcp42_241+RLcp42_243+q(2);
    POcp42_343 = RLcp42_331+RLcp42_334+RLcp42_335+RLcp42_338+RLcp42_339+RLcp42_340+RLcp42_341+RLcp42_343+q(3);
    OMcp42_143 = OMcp42_142+ROcp42_142*qd(43);
    OMcp42_243 = OMcp42_242+ROcp42_242*qd(43);
    OMcp42_343 = OMcp42_342+ROcp42_342*qd(43);
    ORcp42_143 = OMcp42_242*RLcp42_343-OMcp42_342*RLcp42_243;
    ORcp42_243 = -(OMcp42_142*RLcp42_343-OMcp42_342*RLcp42_143);
    ORcp42_343 = OMcp42_142*RLcp42_243-OMcp42_242*RLcp42_143;
    VIcp42_143 = ORcp42_131+ORcp42_134+ORcp42_135+ORcp42_138+ORcp42_139+ORcp42_140+ORcp42_141+ORcp42_143+qd(1);
    VIcp42_243 = ORcp42_231+ORcp42_234+ORcp42_235+ORcp42_238+ORcp42_239+ORcp42_240+ORcp42_241+ORcp42_243+qd(2);
    VIcp42_343 = ORcp42_331+ORcp42_334+ORcp42_335+ORcp42_338+ORcp42_339+ORcp42_340+ORcp42_341+ORcp42_343+qd(3);
    OPcp42_143 = OPcp42_142+ROcp42_142*qdd(43)+qd(43)*(OMcp42_242*ROcp42_342-OMcp42_342*ROcp42_242);
    OPcp42_243 = OPcp42_242+ROcp42_242*qdd(43)-qd(43)*(OMcp42_142*ROcp42_342-OMcp42_342*ROcp42_142);
    OPcp42_343 = OPcp42_342+ROcp42_342*qdd(43)+qd(43)*(OMcp42_142*ROcp42_242-OMcp42_242*ROcp42_142);
    ACcp42_143 = qdd(1)+OMcp42_233*ORcp42_334+OMcp42_234*ORcp42_335+OMcp42_237*ORcp42_338+OMcp42_238*ORcp42_339+OMcp42_239*ORcp42_340+OMcp42_240*...
 ORcp42_341+OMcp42_242*ORcp42_343+OMcp42_26*ORcp42_331-OMcp42_333*ORcp42_234-OMcp42_334*ORcp42_235-OMcp42_337*ORcp42_238-OMcp42_338*ORcp42_239-...
 OMcp42_339*ORcp42_240-OMcp42_340*ORcp42_241-OMcp42_342*ORcp42_243-OMcp42_36*ORcp42_231+OPcp42_233*RLcp42_334+OPcp42_234*RLcp42_335+OPcp42_237*...
 RLcp42_338+OPcp42_238*RLcp42_339+OPcp42_239*RLcp42_340+OPcp42_240*RLcp42_341+OPcp42_242*RLcp42_343+OPcp42_26*RLcp42_331-OPcp42_333*RLcp42_234-...
 OPcp42_334*RLcp42_235-OPcp42_337*RLcp42_238-OPcp42_338*RLcp42_239-OPcp42_339*RLcp42_240-OPcp42_340*RLcp42_241-OPcp42_342*RLcp42_243-OPcp42_36*...
 RLcp42_231;
    ACcp42_243 = qdd(2)-OMcp42_133*ORcp42_334-OMcp42_134*ORcp42_335-OMcp42_137*ORcp42_338-OMcp42_138*ORcp42_339-OMcp42_139*ORcp42_340-OMcp42_140*...
 ORcp42_341-OMcp42_142*ORcp42_343-OMcp42_16*ORcp42_331+OMcp42_333*ORcp42_134+OMcp42_334*ORcp42_135+OMcp42_337*ORcp42_138+OMcp42_338*ORcp42_139+...
 OMcp42_339*ORcp42_140+OMcp42_340*ORcp42_141+OMcp42_342*ORcp42_143+OMcp42_36*ORcp42_131-OPcp42_133*RLcp42_334-OPcp42_134*RLcp42_335-OPcp42_137*...
 RLcp42_338-OPcp42_138*RLcp42_339-OPcp42_139*RLcp42_340-OPcp42_140*RLcp42_341-OPcp42_142*RLcp42_343-OPcp42_16*RLcp42_331+OPcp42_333*RLcp42_134+...
 OPcp42_334*RLcp42_135+OPcp42_337*RLcp42_138+OPcp42_338*RLcp42_139+OPcp42_339*RLcp42_140+OPcp42_340*RLcp42_141+OPcp42_342*RLcp42_143+OPcp42_36*...
 RLcp42_131;
    ACcp42_343 = qdd(3)+OMcp42_133*ORcp42_234+OMcp42_134*ORcp42_235+OMcp42_137*ORcp42_238+OMcp42_138*ORcp42_239+OMcp42_139*ORcp42_240+OMcp42_140*...
 ORcp42_241+OMcp42_142*ORcp42_243+OMcp42_16*ORcp42_231-OMcp42_233*ORcp42_134-OMcp42_234*ORcp42_135-OMcp42_237*ORcp42_138-OMcp42_238*ORcp42_139-...
 OMcp42_239*ORcp42_140-OMcp42_240*ORcp42_141-OMcp42_242*ORcp42_143-OMcp42_26*ORcp42_131+OPcp42_133*RLcp42_234+OPcp42_134*RLcp42_235+OPcp42_137*...
 RLcp42_238+OPcp42_138*RLcp42_239+OPcp42_139*RLcp42_240+OPcp42_140*RLcp42_241+OPcp42_142*RLcp42_243+OPcp42_16*RLcp42_231-OPcp42_233*RLcp42_134-...
 OPcp42_234*RLcp42_135-OPcp42_237*RLcp42_138-OPcp42_238*RLcp42_139-OPcp42_239*RLcp42_140-OPcp42_240*RLcp42_141-OPcp42_242*RLcp42_143-OPcp42_26*...
 RLcp42_131;

% = = Block_1_0_0_43_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp42_143;
    sens.P(2) = POcp42_243;
    sens.P(3) = POcp42_343;
    sens.R(1,1) = ROcp42_142;
    sens.R(1,2) = ROcp42_242;
    sens.R(1,3) = ROcp42_342;
    sens.R(2,1) = ROcp42_443;
    sens.R(2,2) = ROcp42_543;
    sens.R(2,3) = ROcp42_643;
    sens.R(3,1) = ROcp42_743;
    sens.R(3,2) = ROcp42_843;
    sens.R(3,3) = ROcp42_943;
    sens.V(1) = VIcp42_143;
    sens.V(2) = VIcp42_243;
    sens.V(3) = VIcp42_343;
    sens.OM(1) = OMcp42_143;
    sens.OM(2) = OMcp42_243;
    sens.OM(3) = OMcp42_343;
    sens.A(1) = ACcp42_143;
    sens.A(2) = ACcp42_243;
    sens.A(3) = ACcp42_343;
    sens.OMP(1) = OPcp42_143;
    sens.OMP(2) = OPcp42_243;
    sens.OMP(3) = OPcp42_343;
 
% 
case 44, 


% = = Block_1_0_0_44_0_1 = = 
 
% Sensor Kinematics 


    ROcp43_25 = S4*S5;
    ROcp43_35 = -C4*S5;
    ROcp43_85 = -S4*C5;
    ROcp43_95 = C4*C5;
    ROcp43_16 = C5*C6;
    ROcp43_26 = ROcp43_25*C6+C4*S6;
    ROcp43_36 = ROcp43_35*C6+S4*S6;
    ROcp43_46 = -C5*S6;
    ROcp43_56 = -(ROcp43_25*S6-C4*C6);
    ROcp43_66 = -(ROcp43_35*S6-S4*C6);
    OMcp43_25 = qd(5)*C4;
    OMcp43_35 = qd(5)*S4;
    OMcp43_16 = qd(4)+qd(6)*S5;
    OMcp43_26 = OMcp43_25+ROcp43_85*qd(6);
    OMcp43_36 = OMcp43_35+ROcp43_95*qd(6);
    OPcp43_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp43_26 = ROcp43_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp43_35*S5-ROcp43_95*qd(4));
    OPcp43_36 = ROcp43_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp43_25*S5-ROcp43_85*qd(4));

% = = Block_1_0_0_44_0_4 = = 
 
% Sensor Kinematics 


    ROcp43_131 = ROcp43_16*C31-S31*S5;
    ROcp43_231 = ROcp43_26*C31-ROcp43_85*S31;
    ROcp43_331 = ROcp43_36*C31-ROcp43_95*S31;
    ROcp43_731 = ROcp43_16*S31+C31*S5;
    ROcp43_831 = ROcp43_26*S31+ROcp43_85*C31;
    ROcp43_931 = ROcp43_36*S31+ROcp43_95*C31;
    ROcp43_432 = ROcp43_46*C32+ROcp43_731*S32;
    ROcp43_532 = ROcp43_56*C32+ROcp43_831*S32;
    ROcp43_632 = ROcp43_66*C32+ROcp43_931*S32;
    ROcp43_732 = -(ROcp43_46*S32-ROcp43_731*C32);
    ROcp43_832 = -(ROcp43_56*S32-ROcp43_831*C32);
    ROcp43_932 = -(ROcp43_66*S32-ROcp43_931*C32);
    ROcp43_133 = ROcp43_131*C33-ROcp43_732*S33;
    ROcp43_233 = ROcp43_231*C33-ROcp43_832*S33;
    ROcp43_333 = ROcp43_331*C33-ROcp43_932*S33;
    ROcp43_733 = ROcp43_131*S33+ROcp43_732*C33;
    ROcp43_833 = ROcp43_231*S33+ROcp43_832*C33;
    ROcp43_933 = ROcp43_331*S33+ROcp43_932*C33;
    ROcp43_134 = ROcp43_133*C34+ROcp43_432*S34;
    ROcp43_234 = ROcp43_233*C34+ROcp43_532*S34;
    ROcp43_334 = ROcp43_333*C34+ROcp43_632*S34;
    ROcp43_434 = -(ROcp43_133*S34-ROcp43_432*C34);
    ROcp43_534 = -(ROcp43_233*S34-ROcp43_532*C34);
    ROcp43_634 = -(ROcp43_333*S34-ROcp43_632*C34);
    RLcp43_131 = ROcp43_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp43_231 = ROcp43_26*s.dpt(1,3)+ROcp43_85*s.dpt(3,3);
    RLcp43_331 = ROcp43_36*s.dpt(1,3)+ROcp43_95*s.dpt(3,3);
    OMcp43_131 = OMcp43_16+ROcp43_46*qd(31);
    OMcp43_231 = OMcp43_26+ROcp43_56*qd(31);
    OMcp43_331 = OMcp43_36+ROcp43_66*qd(31);
    ORcp43_131 = OMcp43_26*RLcp43_331-OMcp43_36*RLcp43_231;
    ORcp43_231 = -(OMcp43_16*RLcp43_331-OMcp43_36*RLcp43_131);
    ORcp43_331 = OMcp43_16*RLcp43_231-OMcp43_26*RLcp43_131;
    OMcp43_132 = OMcp43_131+ROcp43_131*qd(32);
    OMcp43_232 = OMcp43_231+ROcp43_231*qd(32);
    OMcp43_332 = OMcp43_331+ROcp43_331*qd(32);
    OMcp43_133 = OMcp43_132+ROcp43_432*qd(33);
    OMcp43_233 = OMcp43_232+ROcp43_532*qd(33);
    OMcp43_333 = OMcp43_332+ROcp43_632*qd(33);
    OPcp43_133 = OPcp43_16+ROcp43_131*qdd(32)+ROcp43_432*qdd(33)+ROcp43_46*qdd(31)+qd(31)*(OMcp43_26*ROcp43_66-OMcp43_36*ROcp43_56)+qd(32)*(...
 OMcp43_231*ROcp43_331-OMcp43_331*ROcp43_231)+qd(33)*(OMcp43_232*ROcp43_632-OMcp43_332*ROcp43_532);
    OPcp43_233 = OPcp43_26+ROcp43_231*qdd(32)+ROcp43_532*qdd(33)+ROcp43_56*qdd(31)-qd(31)*(OMcp43_16*ROcp43_66-OMcp43_36*ROcp43_46)-qd(32)*(...
 OMcp43_131*ROcp43_331-OMcp43_331*ROcp43_131)-qd(33)*(OMcp43_132*ROcp43_632-OMcp43_332*ROcp43_432);
    OPcp43_333 = OPcp43_36+ROcp43_331*qdd(32)+ROcp43_632*qdd(33)+ROcp43_66*qdd(31)+qd(31)*(OMcp43_16*ROcp43_56-OMcp43_26*ROcp43_46)+qd(32)*(...
 OMcp43_131*ROcp43_231-OMcp43_231*ROcp43_131)+qd(33)*(OMcp43_132*ROcp43_532-OMcp43_232*ROcp43_432);
    RLcp43_134 = ROcp43_733*s.dpt(3,21);
    RLcp43_234 = ROcp43_833*s.dpt(3,21);
    RLcp43_334 = ROcp43_933*s.dpt(3,21);
    OMcp43_134 = OMcp43_133+ROcp43_733*qd(34);
    OMcp43_234 = OMcp43_233+ROcp43_833*qd(34);
    OMcp43_334 = OMcp43_333+ROcp43_933*qd(34);
    ORcp43_134 = OMcp43_233*RLcp43_334-OMcp43_333*RLcp43_234;
    ORcp43_234 = -(OMcp43_133*RLcp43_334-OMcp43_333*RLcp43_134);
    ORcp43_334 = OMcp43_133*RLcp43_234-OMcp43_233*RLcp43_134;
    OPcp43_134 = OPcp43_133+ROcp43_733*qdd(34)+qd(34)*(OMcp43_233*ROcp43_933-OMcp43_333*ROcp43_833);
    OPcp43_234 = OPcp43_233+ROcp43_833*qdd(34)-qd(34)*(OMcp43_133*ROcp43_933-OMcp43_333*ROcp43_733);
    OPcp43_334 = OPcp43_333+ROcp43_933*qdd(34)+qd(34)*(OMcp43_133*ROcp43_833-OMcp43_233*ROcp43_733);

% = = Block_1_0_0_44_0_6 = = 
 
% Sensor Kinematics 


    ROcp43_444 = ROcp43_434*C44+ROcp43_733*S44;
    ROcp43_544 = ROcp43_534*C44+ROcp43_833*S44;
    ROcp43_644 = ROcp43_634*C44+ROcp43_933*S44;
    ROcp43_744 = -(ROcp43_434*S44-ROcp43_733*C44);
    ROcp43_844 = -(ROcp43_534*S44-ROcp43_833*C44);
    ROcp43_944 = -(ROcp43_634*S44-ROcp43_933*C44);
    RLcp43_144 = ROcp43_134*s.dpt(1,24)+ROcp43_434*s.dpt(2,24)+ROcp43_733*s.dpt(3,24);
    RLcp43_244 = ROcp43_234*s.dpt(1,24)+ROcp43_534*s.dpt(2,24)+ROcp43_833*s.dpt(3,24);
    RLcp43_344 = ROcp43_334*s.dpt(1,24)+ROcp43_634*s.dpt(2,24)+ROcp43_933*s.dpt(3,24);
    POcp43_144 = RLcp43_131+RLcp43_134+RLcp43_144+q(1);
    POcp43_244 = RLcp43_231+RLcp43_234+RLcp43_244+q(2);
    POcp43_344 = RLcp43_331+RLcp43_334+RLcp43_344+q(3);
    ORcp43_144 = OMcp43_234*RLcp43_344-OMcp43_334*RLcp43_244;
    ORcp43_244 = -(OMcp43_134*RLcp43_344-OMcp43_334*RLcp43_144);
    ORcp43_344 = OMcp43_134*RLcp43_244-OMcp43_234*RLcp43_144;
    VIcp43_144 = ORcp43_131+ORcp43_134+ORcp43_144+qd(1);
    VIcp43_244 = ORcp43_231+ORcp43_234+ORcp43_244+qd(2);
    VIcp43_344 = ORcp43_331+ORcp43_334+ORcp43_344+qd(3);
    ACcp43_144 = qdd(1)+OMcp43_233*ORcp43_334+OMcp43_234*ORcp43_344+OMcp43_26*ORcp43_331-OMcp43_333*ORcp43_234-OMcp43_334*ORcp43_244-OMcp43_36*...
 ORcp43_231+OPcp43_233*RLcp43_334+OPcp43_234*RLcp43_344+OPcp43_26*RLcp43_331-OPcp43_333*RLcp43_234-OPcp43_334*RLcp43_244-OPcp43_36*RLcp43_231;
    ACcp43_244 = qdd(2)-OMcp43_133*ORcp43_334-OMcp43_134*ORcp43_344-OMcp43_16*ORcp43_331+OMcp43_333*ORcp43_134+OMcp43_334*ORcp43_144+OMcp43_36*...
 ORcp43_131-OPcp43_133*RLcp43_334-OPcp43_134*RLcp43_344-OPcp43_16*RLcp43_331+OPcp43_333*RLcp43_134+OPcp43_334*RLcp43_144+OPcp43_36*RLcp43_131;
    ACcp43_344 = qdd(3)+OMcp43_133*ORcp43_234+OMcp43_134*ORcp43_244+OMcp43_16*ORcp43_231-OMcp43_233*ORcp43_134-OMcp43_234*ORcp43_144-OMcp43_26*...
 ORcp43_131+OPcp43_133*RLcp43_234+OPcp43_134*RLcp43_244+OPcp43_16*RLcp43_231-OPcp43_233*RLcp43_134-OPcp43_234*RLcp43_144-OPcp43_26*RLcp43_131;

% = = Block_1_0_0_44_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp43_144;
    sens.P(2) = POcp43_244;
    sens.P(3) = POcp43_344;
    sens.R(1,1) = ROcp43_134;
    sens.R(1,2) = ROcp43_234;
    sens.R(1,3) = ROcp43_334;
    sens.R(2,1) = ROcp43_444;
    sens.R(2,2) = ROcp43_544;
    sens.R(2,3) = ROcp43_644;
    sens.R(3,1) = ROcp43_744;
    sens.R(3,2) = ROcp43_844;
    sens.R(3,3) = ROcp43_944;
    sens.V(1) = VIcp43_144;
    sens.V(2) = VIcp43_244;
    sens.V(3) = VIcp43_344;
    sens.OM(1) = OMcp43_134;
    sens.OM(2) = OMcp43_234;
    sens.OM(3) = OMcp43_334;
    sens.A(1) = ACcp43_144;
    sens.A(2) = ACcp43_244;
    sens.A(3) = ACcp43_344;
    sens.OMP(1) = OPcp43_134;
    sens.OMP(2) = OPcp43_234;
    sens.OMP(3) = OPcp43_334;
 
% 
case 45, 


% = = Block_1_0_0_45_0_1 = = 
 
% Sensor Kinematics 


    ROcp44_25 = S4*S5;
    ROcp44_35 = -C4*S5;
    ROcp44_85 = -S4*C5;
    ROcp44_95 = C4*C5;
    ROcp44_16 = C5*C6;
    ROcp44_26 = ROcp44_25*C6+C4*S6;
    ROcp44_36 = ROcp44_35*C6+S4*S6;
    ROcp44_46 = -C5*S6;
    ROcp44_56 = -(ROcp44_25*S6-C4*C6);
    ROcp44_66 = -(ROcp44_35*S6-S4*C6);
    OMcp44_25 = qd(5)*C4;
    OMcp44_35 = qd(5)*S4;
    OMcp44_16 = qd(4)+qd(6)*S5;
    OMcp44_26 = OMcp44_25+ROcp44_85*qd(6);
    OMcp44_36 = OMcp44_35+ROcp44_95*qd(6);
    OPcp44_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp44_26 = ROcp44_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp44_35*S5-ROcp44_95*qd(4));
    OPcp44_36 = ROcp44_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp44_25*S5-ROcp44_85*qd(4));

% = = Block_1_0_0_45_0_4 = = 
 
% Sensor Kinematics 


    ROcp44_131 = ROcp44_16*C31-S31*S5;
    ROcp44_231 = ROcp44_26*C31-ROcp44_85*S31;
    ROcp44_331 = ROcp44_36*C31-ROcp44_95*S31;
    ROcp44_731 = ROcp44_16*S31+C31*S5;
    ROcp44_831 = ROcp44_26*S31+ROcp44_85*C31;
    ROcp44_931 = ROcp44_36*S31+ROcp44_95*C31;
    ROcp44_432 = ROcp44_46*C32+ROcp44_731*S32;
    ROcp44_532 = ROcp44_56*C32+ROcp44_831*S32;
    ROcp44_632 = ROcp44_66*C32+ROcp44_931*S32;
    ROcp44_732 = -(ROcp44_46*S32-ROcp44_731*C32);
    ROcp44_832 = -(ROcp44_56*S32-ROcp44_831*C32);
    ROcp44_932 = -(ROcp44_66*S32-ROcp44_931*C32);
    ROcp44_133 = ROcp44_131*C33-ROcp44_732*S33;
    ROcp44_233 = ROcp44_231*C33-ROcp44_832*S33;
    ROcp44_333 = ROcp44_331*C33-ROcp44_932*S33;
    ROcp44_733 = ROcp44_131*S33+ROcp44_732*C33;
    ROcp44_833 = ROcp44_231*S33+ROcp44_832*C33;
    ROcp44_933 = ROcp44_331*S33+ROcp44_932*C33;
    ROcp44_134 = ROcp44_133*C34+ROcp44_432*S34;
    ROcp44_234 = ROcp44_233*C34+ROcp44_532*S34;
    ROcp44_334 = ROcp44_333*C34+ROcp44_632*S34;
    ROcp44_434 = -(ROcp44_133*S34-ROcp44_432*C34);
    ROcp44_534 = -(ROcp44_233*S34-ROcp44_532*C34);
    ROcp44_634 = -(ROcp44_333*S34-ROcp44_632*C34);
    RLcp44_131 = ROcp44_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp44_231 = ROcp44_26*s.dpt(1,3)+ROcp44_85*s.dpt(3,3);
    RLcp44_331 = ROcp44_36*s.dpt(1,3)+ROcp44_95*s.dpt(3,3);
    OMcp44_131 = OMcp44_16+ROcp44_46*qd(31);
    OMcp44_231 = OMcp44_26+ROcp44_56*qd(31);
    OMcp44_331 = OMcp44_36+ROcp44_66*qd(31);
    ORcp44_131 = OMcp44_26*RLcp44_331-OMcp44_36*RLcp44_231;
    ORcp44_231 = -(OMcp44_16*RLcp44_331-OMcp44_36*RLcp44_131);
    ORcp44_331 = OMcp44_16*RLcp44_231-OMcp44_26*RLcp44_131;
    OMcp44_132 = OMcp44_131+ROcp44_131*qd(32);
    OMcp44_232 = OMcp44_231+ROcp44_231*qd(32);
    OMcp44_332 = OMcp44_331+ROcp44_331*qd(32);
    OMcp44_133 = OMcp44_132+ROcp44_432*qd(33);
    OMcp44_233 = OMcp44_232+ROcp44_532*qd(33);
    OMcp44_333 = OMcp44_332+ROcp44_632*qd(33);
    OPcp44_133 = OPcp44_16+ROcp44_131*qdd(32)+ROcp44_432*qdd(33)+ROcp44_46*qdd(31)+qd(31)*(OMcp44_26*ROcp44_66-OMcp44_36*ROcp44_56)+qd(32)*(...
 OMcp44_231*ROcp44_331-OMcp44_331*ROcp44_231)+qd(33)*(OMcp44_232*ROcp44_632-OMcp44_332*ROcp44_532);
    OPcp44_233 = OPcp44_26+ROcp44_231*qdd(32)+ROcp44_532*qdd(33)+ROcp44_56*qdd(31)-qd(31)*(OMcp44_16*ROcp44_66-OMcp44_36*ROcp44_46)-qd(32)*(...
 OMcp44_131*ROcp44_331-OMcp44_331*ROcp44_131)-qd(33)*(OMcp44_132*ROcp44_632-OMcp44_332*ROcp44_432);
    OPcp44_333 = OPcp44_36+ROcp44_331*qdd(32)+ROcp44_632*qdd(33)+ROcp44_66*qdd(31)+qd(31)*(OMcp44_16*ROcp44_56-OMcp44_26*ROcp44_46)+qd(32)*(...
 OMcp44_131*ROcp44_231-OMcp44_231*ROcp44_131)+qd(33)*(OMcp44_132*ROcp44_532-OMcp44_232*ROcp44_432);
    RLcp44_134 = ROcp44_733*s.dpt(3,21);
    RLcp44_234 = ROcp44_833*s.dpt(3,21);
    RLcp44_334 = ROcp44_933*s.dpt(3,21);
    OMcp44_134 = OMcp44_133+ROcp44_733*qd(34);
    OMcp44_234 = OMcp44_233+ROcp44_833*qd(34);
    OMcp44_334 = OMcp44_333+ROcp44_933*qd(34);
    ORcp44_134 = OMcp44_233*RLcp44_334-OMcp44_333*RLcp44_234;
    ORcp44_234 = -(OMcp44_133*RLcp44_334-OMcp44_333*RLcp44_134);
    ORcp44_334 = OMcp44_133*RLcp44_234-OMcp44_233*RLcp44_134;
    OPcp44_134 = OPcp44_133+ROcp44_733*qdd(34)+qd(34)*(OMcp44_233*ROcp44_933-OMcp44_333*ROcp44_833);
    OPcp44_234 = OPcp44_233+ROcp44_833*qdd(34)-qd(34)*(OMcp44_133*ROcp44_933-OMcp44_333*ROcp44_733);
    OPcp44_334 = OPcp44_333+ROcp44_933*qdd(34)+qd(34)*(OMcp44_133*ROcp44_833-OMcp44_233*ROcp44_733);

% = = Block_1_0_0_45_0_6 = = 
 
% Sensor Kinematics 


    ROcp44_444 = ROcp44_434*C44+ROcp44_733*S44;
    ROcp44_544 = ROcp44_534*C44+ROcp44_833*S44;
    ROcp44_644 = ROcp44_634*C44+ROcp44_933*S44;
    ROcp44_744 = -(ROcp44_434*S44-ROcp44_733*C44);
    ROcp44_844 = -(ROcp44_534*S44-ROcp44_833*C44);
    ROcp44_944 = -(ROcp44_634*S44-ROcp44_933*C44);
    ROcp44_145 = ROcp44_134*C45+ROcp44_444*S45;
    ROcp44_245 = ROcp44_234*C45+ROcp44_544*S45;
    ROcp44_345 = ROcp44_334*C45+ROcp44_644*S45;
    ROcp44_445 = -(ROcp44_134*S45-ROcp44_444*C45);
    ROcp44_545 = -(ROcp44_234*S45-ROcp44_544*C45);
    ROcp44_645 = -(ROcp44_334*S45-ROcp44_644*C45);
    RLcp44_144 = ROcp44_134*s.dpt(1,24)+ROcp44_434*s.dpt(2,24)+ROcp44_733*s.dpt(3,24);
    RLcp44_244 = ROcp44_234*s.dpt(1,24)+ROcp44_534*s.dpt(2,24)+ROcp44_833*s.dpt(3,24);
    RLcp44_344 = ROcp44_334*s.dpt(1,24)+ROcp44_634*s.dpt(2,24)+ROcp44_933*s.dpt(3,24);
    POcp44_144 = RLcp44_131+RLcp44_134+RLcp44_144+q(1);
    POcp44_244 = RLcp44_231+RLcp44_234+RLcp44_244+q(2);
    POcp44_344 = RLcp44_331+RLcp44_334+RLcp44_344+q(3);
    ORcp44_144 = OMcp44_234*RLcp44_344-OMcp44_334*RLcp44_244;
    ORcp44_244 = -(OMcp44_134*RLcp44_344-OMcp44_334*RLcp44_144);
    ORcp44_344 = OMcp44_134*RLcp44_244-OMcp44_234*RLcp44_144;
    VIcp44_144 = ORcp44_131+ORcp44_134+ORcp44_144+qd(1);
    VIcp44_244 = ORcp44_231+ORcp44_234+ORcp44_244+qd(2);
    VIcp44_344 = ORcp44_331+ORcp44_334+ORcp44_344+qd(3);
    ACcp44_144 = qdd(1)+OMcp44_233*ORcp44_334+OMcp44_234*ORcp44_344+OMcp44_26*ORcp44_331-OMcp44_333*ORcp44_234-OMcp44_334*ORcp44_244-OMcp44_36*...
 ORcp44_231+OPcp44_233*RLcp44_334+OPcp44_234*RLcp44_344+OPcp44_26*RLcp44_331-OPcp44_333*RLcp44_234-OPcp44_334*RLcp44_244-OPcp44_36*RLcp44_231;
    ACcp44_244 = qdd(2)-OMcp44_133*ORcp44_334-OMcp44_134*ORcp44_344-OMcp44_16*ORcp44_331+OMcp44_333*ORcp44_134+OMcp44_334*ORcp44_144+OMcp44_36*...
 ORcp44_131-OPcp44_133*RLcp44_334-OPcp44_134*RLcp44_344-OPcp44_16*RLcp44_331+OPcp44_333*RLcp44_134+OPcp44_334*RLcp44_144+OPcp44_36*RLcp44_131;
    ACcp44_344 = qdd(3)+OMcp44_133*ORcp44_234+OMcp44_134*ORcp44_244+OMcp44_16*ORcp44_231-OMcp44_233*ORcp44_134-OMcp44_234*ORcp44_144-OMcp44_26*...
 ORcp44_131+OPcp44_133*RLcp44_234+OPcp44_134*RLcp44_244+OPcp44_16*RLcp44_231-OPcp44_233*RLcp44_134-OPcp44_234*RLcp44_144-OPcp44_26*RLcp44_131;

% = = Block_1_0_0_45_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp44_144;
    sens.P(2) = POcp44_244;
    sens.P(3) = POcp44_344;
    sens.R(1,1) = ROcp44_145;
    sens.R(1,2) = ROcp44_245;
    sens.R(1,3) = ROcp44_345;
    sens.R(2,1) = ROcp44_445;
    sens.R(2,2) = ROcp44_545;
    sens.R(2,3) = ROcp44_645;
    sens.R(3,1) = ROcp44_744;
    sens.R(3,2) = ROcp44_844;
    sens.R(3,3) = ROcp44_944;
    sens.V(1) = VIcp44_144;
    sens.V(2) = VIcp44_244;
    sens.V(3) = VIcp44_344;
    sens.OM(1) = OMcp44_134;
    sens.OM(2) = OMcp44_234;
    sens.OM(3) = OMcp44_334;
    sens.A(1) = ACcp44_144;
    sens.A(2) = ACcp44_244;
    sens.A(3) = ACcp44_344;
    sens.OMP(1) = OPcp44_134;
    sens.OMP(2) = OPcp44_234;
    sens.OMP(3) = OPcp44_334;
 
% 
case 46, 


% = = Block_1_0_0_46_0_1 = = 
 
% Sensor Kinematics 


    ROcp45_25 = S4*S5;
    ROcp45_35 = -C4*S5;
    ROcp45_85 = -S4*C5;
    ROcp45_95 = C4*C5;
    ROcp45_16 = C5*C6;
    ROcp45_26 = ROcp45_25*C6+C4*S6;
    ROcp45_36 = ROcp45_35*C6+S4*S6;
    ROcp45_46 = -C5*S6;
    ROcp45_56 = -(ROcp45_25*S6-C4*C6);
    ROcp45_66 = -(ROcp45_35*S6-S4*C6);
    OMcp45_25 = qd(5)*C4;
    OMcp45_35 = qd(5)*S4;
    OMcp45_16 = qd(4)+qd(6)*S5;
    OMcp45_26 = OMcp45_25+ROcp45_85*qd(6);
    OMcp45_36 = OMcp45_35+ROcp45_95*qd(6);
    OPcp45_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp45_26 = ROcp45_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp45_35*S5-ROcp45_95*qd(4));
    OPcp45_36 = ROcp45_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp45_25*S5-ROcp45_85*qd(4));

% = = Block_1_0_0_46_0_4 = = 
 
% Sensor Kinematics 


    ROcp45_131 = ROcp45_16*C31-S31*S5;
    ROcp45_231 = ROcp45_26*C31-ROcp45_85*S31;
    ROcp45_331 = ROcp45_36*C31-ROcp45_95*S31;
    ROcp45_731 = ROcp45_16*S31+C31*S5;
    ROcp45_831 = ROcp45_26*S31+ROcp45_85*C31;
    ROcp45_931 = ROcp45_36*S31+ROcp45_95*C31;
    ROcp45_432 = ROcp45_46*C32+ROcp45_731*S32;
    ROcp45_532 = ROcp45_56*C32+ROcp45_831*S32;
    ROcp45_632 = ROcp45_66*C32+ROcp45_931*S32;
    ROcp45_732 = -(ROcp45_46*S32-ROcp45_731*C32);
    ROcp45_832 = -(ROcp45_56*S32-ROcp45_831*C32);
    ROcp45_932 = -(ROcp45_66*S32-ROcp45_931*C32);
    ROcp45_133 = ROcp45_131*C33-ROcp45_732*S33;
    ROcp45_233 = ROcp45_231*C33-ROcp45_832*S33;
    ROcp45_333 = ROcp45_331*C33-ROcp45_932*S33;
    ROcp45_733 = ROcp45_131*S33+ROcp45_732*C33;
    ROcp45_833 = ROcp45_231*S33+ROcp45_832*C33;
    ROcp45_933 = ROcp45_331*S33+ROcp45_932*C33;
    ROcp45_134 = ROcp45_133*C34+ROcp45_432*S34;
    ROcp45_234 = ROcp45_233*C34+ROcp45_532*S34;
    ROcp45_334 = ROcp45_333*C34+ROcp45_632*S34;
    ROcp45_434 = -(ROcp45_133*S34-ROcp45_432*C34);
    ROcp45_534 = -(ROcp45_233*S34-ROcp45_532*C34);
    ROcp45_634 = -(ROcp45_333*S34-ROcp45_632*C34);
    RLcp45_131 = ROcp45_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp45_231 = ROcp45_26*s.dpt(1,3)+ROcp45_85*s.dpt(3,3);
    RLcp45_331 = ROcp45_36*s.dpt(1,3)+ROcp45_95*s.dpt(3,3);
    OMcp45_131 = OMcp45_16+ROcp45_46*qd(31);
    OMcp45_231 = OMcp45_26+ROcp45_56*qd(31);
    OMcp45_331 = OMcp45_36+ROcp45_66*qd(31);
    ORcp45_131 = OMcp45_26*RLcp45_331-OMcp45_36*RLcp45_231;
    ORcp45_231 = -(OMcp45_16*RLcp45_331-OMcp45_36*RLcp45_131);
    ORcp45_331 = OMcp45_16*RLcp45_231-OMcp45_26*RLcp45_131;
    OMcp45_132 = OMcp45_131+ROcp45_131*qd(32);
    OMcp45_232 = OMcp45_231+ROcp45_231*qd(32);
    OMcp45_332 = OMcp45_331+ROcp45_331*qd(32);
    OMcp45_133 = OMcp45_132+ROcp45_432*qd(33);
    OMcp45_233 = OMcp45_232+ROcp45_532*qd(33);
    OMcp45_333 = OMcp45_332+ROcp45_632*qd(33);
    OPcp45_133 = OPcp45_16+ROcp45_131*qdd(32)+ROcp45_432*qdd(33)+ROcp45_46*qdd(31)+qd(31)*(OMcp45_26*ROcp45_66-OMcp45_36*ROcp45_56)+qd(32)*(...
 OMcp45_231*ROcp45_331-OMcp45_331*ROcp45_231)+qd(33)*(OMcp45_232*ROcp45_632-OMcp45_332*ROcp45_532);
    OPcp45_233 = OPcp45_26+ROcp45_231*qdd(32)+ROcp45_532*qdd(33)+ROcp45_56*qdd(31)-qd(31)*(OMcp45_16*ROcp45_66-OMcp45_36*ROcp45_46)-qd(32)*(...
 OMcp45_131*ROcp45_331-OMcp45_331*ROcp45_131)-qd(33)*(OMcp45_132*ROcp45_632-OMcp45_332*ROcp45_432);
    OPcp45_333 = OPcp45_36+ROcp45_331*qdd(32)+ROcp45_632*qdd(33)+ROcp45_66*qdd(31)+qd(31)*(OMcp45_16*ROcp45_56-OMcp45_26*ROcp45_46)+qd(32)*(...
 OMcp45_131*ROcp45_231-OMcp45_231*ROcp45_131)+qd(33)*(OMcp45_132*ROcp45_532-OMcp45_232*ROcp45_432);
    RLcp45_134 = ROcp45_733*s.dpt(3,21);
    RLcp45_234 = ROcp45_833*s.dpt(3,21);
    RLcp45_334 = ROcp45_933*s.dpt(3,21);
    OMcp45_134 = OMcp45_133+ROcp45_733*qd(34);
    OMcp45_234 = OMcp45_233+ROcp45_833*qd(34);
    OMcp45_334 = OMcp45_333+ROcp45_933*qd(34);
    ORcp45_134 = OMcp45_233*RLcp45_334-OMcp45_333*RLcp45_234;
    ORcp45_234 = -(OMcp45_133*RLcp45_334-OMcp45_333*RLcp45_134);
    ORcp45_334 = OMcp45_133*RLcp45_234-OMcp45_233*RLcp45_134;
    OPcp45_134 = OPcp45_133+ROcp45_733*qdd(34)+qd(34)*(OMcp45_233*ROcp45_933-OMcp45_333*ROcp45_833);
    OPcp45_234 = OPcp45_233+ROcp45_833*qdd(34)-qd(34)*(OMcp45_133*ROcp45_933-OMcp45_333*ROcp45_733);
    OPcp45_334 = OPcp45_333+ROcp45_933*qdd(34)+qd(34)*(OMcp45_133*ROcp45_833-OMcp45_233*ROcp45_733);

% = = Block_1_0_0_46_0_6 = = 
 
% Sensor Kinematics 


    ROcp45_444 = ROcp45_434*C44+ROcp45_733*S44;
    ROcp45_544 = ROcp45_534*C44+ROcp45_833*S44;
    ROcp45_644 = ROcp45_634*C44+ROcp45_933*S44;
    ROcp45_744 = -(ROcp45_434*S44-ROcp45_733*C44);
    ROcp45_844 = -(ROcp45_534*S44-ROcp45_833*C44);
    ROcp45_944 = -(ROcp45_634*S44-ROcp45_933*C44);
    ROcp45_145 = ROcp45_134*C45+ROcp45_444*S45;
    ROcp45_245 = ROcp45_234*C45+ROcp45_544*S45;
    ROcp45_345 = ROcp45_334*C45+ROcp45_644*S45;
    ROcp45_445 = -(ROcp45_134*S45-ROcp45_444*C45);
    ROcp45_545 = -(ROcp45_234*S45-ROcp45_544*C45);
    ROcp45_645 = -(ROcp45_334*S45-ROcp45_644*C45);
    ROcp45_146 = ROcp45_145*C46-ROcp45_744*S46;
    ROcp45_246 = ROcp45_245*C46-ROcp45_844*S46;
    ROcp45_346 = ROcp45_345*C46-ROcp45_944*S46;
    ROcp45_746 = ROcp45_145*S46+ROcp45_744*C46;
    ROcp45_846 = ROcp45_245*S46+ROcp45_844*C46;
    ROcp45_946 = ROcp45_345*S46+ROcp45_944*C46;
    RLcp45_144 = ROcp45_134*s.dpt(1,24)+ROcp45_434*s.dpt(2,24)+ROcp45_733*s.dpt(3,24);
    RLcp45_244 = ROcp45_234*s.dpt(1,24)+ROcp45_534*s.dpt(2,24)+ROcp45_833*s.dpt(3,24);
    RLcp45_344 = ROcp45_334*s.dpt(1,24)+ROcp45_634*s.dpt(2,24)+ROcp45_933*s.dpt(3,24);
    POcp45_144 = RLcp45_131+RLcp45_134+RLcp45_144+q(1);
    POcp45_244 = RLcp45_231+RLcp45_234+RLcp45_244+q(2);
    POcp45_344 = RLcp45_331+RLcp45_334+RLcp45_344+q(3);
    ORcp45_144 = OMcp45_234*RLcp45_344-OMcp45_334*RLcp45_244;
    ORcp45_244 = -(OMcp45_134*RLcp45_344-OMcp45_334*RLcp45_144);
    ORcp45_344 = OMcp45_134*RLcp45_244-OMcp45_234*RLcp45_144;
    VIcp45_144 = ORcp45_131+ORcp45_134+ORcp45_144+qd(1);
    VIcp45_244 = ORcp45_231+ORcp45_234+ORcp45_244+qd(2);
    VIcp45_344 = ORcp45_331+ORcp45_334+ORcp45_344+qd(3);
    ACcp45_144 = qdd(1)+OMcp45_233*ORcp45_334+OMcp45_234*ORcp45_344+OMcp45_26*ORcp45_331-OMcp45_333*ORcp45_234-OMcp45_334*ORcp45_244-OMcp45_36*...
 ORcp45_231+OPcp45_233*RLcp45_334+OPcp45_234*RLcp45_344+OPcp45_26*RLcp45_331-OPcp45_333*RLcp45_234-OPcp45_334*RLcp45_244-OPcp45_36*RLcp45_231;
    ACcp45_244 = qdd(2)-OMcp45_133*ORcp45_334-OMcp45_134*ORcp45_344-OMcp45_16*ORcp45_331+OMcp45_333*ORcp45_134+OMcp45_334*ORcp45_144+OMcp45_36*...
 ORcp45_131-OPcp45_133*RLcp45_334-OPcp45_134*RLcp45_344-OPcp45_16*RLcp45_331+OPcp45_333*RLcp45_134+OPcp45_334*RLcp45_144+OPcp45_36*RLcp45_131;
    ACcp45_344 = qdd(3)+OMcp45_133*ORcp45_234+OMcp45_134*ORcp45_244+OMcp45_16*ORcp45_231-OMcp45_233*ORcp45_134-OMcp45_234*ORcp45_144-OMcp45_26*...
 ORcp45_131+OPcp45_133*RLcp45_234+OPcp45_134*RLcp45_244+OPcp45_16*RLcp45_231-OPcp45_233*RLcp45_134-OPcp45_234*RLcp45_144-OPcp45_26*RLcp45_131;
    OMcp45_146 = OMcp45_134+ROcp45_445*qd(46);
    OMcp45_246 = OMcp45_234+ROcp45_545*qd(46);
    OMcp45_346 = OMcp45_334+ROcp45_645*qd(46);
    OPcp45_146 = OPcp45_134+ROcp45_445*qdd(46)+qd(46)*(OMcp45_234*ROcp45_645-OMcp45_334*ROcp45_545);
    OPcp45_246 = OPcp45_234+ROcp45_545*qdd(46)-qd(46)*(OMcp45_134*ROcp45_645-OMcp45_334*ROcp45_445);
    OPcp45_346 = OPcp45_334+ROcp45_645*qdd(46)+qd(46)*(OMcp45_134*ROcp45_545-OMcp45_234*ROcp45_445);

% = = Block_1_0_0_46_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp45_144;
    sens.P(2) = POcp45_244;
    sens.P(3) = POcp45_344;
    sens.R(1,1) = ROcp45_146;
    sens.R(1,2) = ROcp45_246;
    sens.R(1,3) = ROcp45_346;
    sens.R(2,1) = ROcp45_445;
    sens.R(2,2) = ROcp45_545;
    sens.R(2,3) = ROcp45_645;
    sens.R(3,1) = ROcp45_746;
    sens.R(3,2) = ROcp45_846;
    sens.R(3,3) = ROcp45_946;
    sens.V(1) = VIcp45_144;
    sens.V(2) = VIcp45_244;
    sens.V(3) = VIcp45_344;
    sens.OM(1) = OMcp45_146;
    sens.OM(2) = OMcp45_246;
    sens.OM(3) = OMcp45_346;
    sens.A(1) = ACcp45_144;
    sens.A(2) = ACcp45_244;
    sens.A(3) = ACcp45_344;
    sens.OMP(1) = OPcp45_146;
    sens.OMP(2) = OPcp45_246;
    sens.OMP(3) = OPcp45_346;
 
% 
case 47, 


% = = Block_1_0_0_47_0_1 = = 
 
% Sensor Kinematics 


    ROcp46_25 = S4*S5;
    ROcp46_35 = -C4*S5;
    ROcp46_85 = -S4*C5;
    ROcp46_95 = C4*C5;
    ROcp46_16 = C5*C6;
    ROcp46_26 = ROcp46_25*C6+C4*S6;
    ROcp46_36 = ROcp46_35*C6+S4*S6;
    ROcp46_46 = -C5*S6;
    ROcp46_56 = -(ROcp46_25*S6-C4*C6);
    ROcp46_66 = -(ROcp46_35*S6-S4*C6);
    OMcp46_25 = qd(5)*C4;
    OMcp46_35 = qd(5)*S4;
    OMcp46_16 = qd(4)+qd(6)*S5;
    OMcp46_26 = OMcp46_25+ROcp46_85*qd(6);
    OMcp46_36 = OMcp46_35+ROcp46_95*qd(6);
    OPcp46_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp46_26 = ROcp46_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp46_35*S5-ROcp46_95*qd(4));
    OPcp46_36 = ROcp46_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp46_25*S5-ROcp46_85*qd(4));

% = = Block_1_0_0_47_0_4 = = 
 
% Sensor Kinematics 


    ROcp46_131 = ROcp46_16*C31-S31*S5;
    ROcp46_231 = ROcp46_26*C31-ROcp46_85*S31;
    ROcp46_331 = ROcp46_36*C31-ROcp46_95*S31;
    ROcp46_731 = ROcp46_16*S31+C31*S5;
    ROcp46_831 = ROcp46_26*S31+ROcp46_85*C31;
    ROcp46_931 = ROcp46_36*S31+ROcp46_95*C31;
    ROcp46_432 = ROcp46_46*C32+ROcp46_731*S32;
    ROcp46_532 = ROcp46_56*C32+ROcp46_831*S32;
    ROcp46_632 = ROcp46_66*C32+ROcp46_931*S32;
    ROcp46_732 = -(ROcp46_46*S32-ROcp46_731*C32);
    ROcp46_832 = -(ROcp46_56*S32-ROcp46_831*C32);
    ROcp46_932 = -(ROcp46_66*S32-ROcp46_931*C32);
    ROcp46_133 = ROcp46_131*C33-ROcp46_732*S33;
    ROcp46_233 = ROcp46_231*C33-ROcp46_832*S33;
    ROcp46_333 = ROcp46_331*C33-ROcp46_932*S33;
    ROcp46_733 = ROcp46_131*S33+ROcp46_732*C33;
    ROcp46_833 = ROcp46_231*S33+ROcp46_832*C33;
    ROcp46_933 = ROcp46_331*S33+ROcp46_932*C33;
    ROcp46_134 = ROcp46_133*C34+ROcp46_432*S34;
    ROcp46_234 = ROcp46_233*C34+ROcp46_532*S34;
    ROcp46_334 = ROcp46_333*C34+ROcp46_632*S34;
    ROcp46_434 = -(ROcp46_133*S34-ROcp46_432*C34);
    ROcp46_534 = -(ROcp46_233*S34-ROcp46_532*C34);
    ROcp46_634 = -(ROcp46_333*S34-ROcp46_632*C34);
    RLcp46_131 = ROcp46_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp46_231 = ROcp46_26*s.dpt(1,3)+ROcp46_85*s.dpt(3,3);
    RLcp46_331 = ROcp46_36*s.dpt(1,3)+ROcp46_95*s.dpt(3,3);
    OMcp46_131 = OMcp46_16+ROcp46_46*qd(31);
    OMcp46_231 = OMcp46_26+ROcp46_56*qd(31);
    OMcp46_331 = OMcp46_36+ROcp46_66*qd(31);
    ORcp46_131 = OMcp46_26*RLcp46_331-OMcp46_36*RLcp46_231;
    ORcp46_231 = -(OMcp46_16*RLcp46_331-OMcp46_36*RLcp46_131);
    ORcp46_331 = OMcp46_16*RLcp46_231-OMcp46_26*RLcp46_131;
    OMcp46_132 = OMcp46_131+ROcp46_131*qd(32);
    OMcp46_232 = OMcp46_231+ROcp46_231*qd(32);
    OMcp46_332 = OMcp46_331+ROcp46_331*qd(32);
    OMcp46_133 = OMcp46_132+ROcp46_432*qd(33);
    OMcp46_233 = OMcp46_232+ROcp46_532*qd(33);
    OMcp46_333 = OMcp46_332+ROcp46_632*qd(33);
    OPcp46_133 = OPcp46_16+ROcp46_131*qdd(32)+ROcp46_432*qdd(33)+ROcp46_46*qdd(31)+qd(31)*(OMcp46_26*ROcp46_66-OMcp46_36*ROcp46_56)+qd(32)*(...
 OMcp46_231*ROcp46_331-OMcp46_331*ROcp46_231)+qd(33)*(OMcp46_232*ROcp46_632-OMcp46_332*ROcp46_532);
    OPcp46_233 = OPcp46_26+ROcp46_231*qdd(32)+ROcp46_532*qdd(33)+ROcp46_56*qdd(31)-qd(31)*(OMcp46_16*ROcp46_66-OMcp46_36*ROcp46_46)-qd(32)*(...
 OMcp46_131*ROcp46_331-OMcp46_331*ROcp46_131)-qd(33)*(OMcp46_132*ROcp46_632-OMcp46_332*ROcp46_432);
    OPcp46_333 = OPcp46_36+ROcp46_331*qdd(32)+ROcp46_632*qdd(33)+ROcp46_66*qdd(31)+qd(31)*(OMcp46_16*ROcp46_56-OMcp46_26*ROcp46_46)+qd(32)*(...
 OMcp46_131*ROcp46_231-OMcp46_231*ROcp46_131)+qd(33)*(OMcp46_132*ROcp46_532-OMcp46_232*ROcp46_432);
    RLcp46_134 = ROcp46_733*s.dpt(3,21);
    RLcp46_234 = ROcp46_833*s.dpt(3,21);
    RLcp46_334 = ROcp46_933*s.dpt(3,21);
    OMcp46_134 = OMcp46_133+ROcp46_733*qd(34);
    OMcp46_234 = OMcp46_233+ROcp46_833*qd(34);
    OMcp46_334 = OMcp46_333+ROcp46_933*qd(34);
    ORcp46_134 = OMcp46_233*RLcp46_334-OMcp46_333*RLcp46_234;
    ORcp46_234 = -(OMcp46_133*RLcp46_334-OMcp46_333*RLcp46_134);
    ORcp46_334 = OMcp46_133*RLcp46_234-OMcp46_233*RLcp46_134;
    OPcp46_134 = OPcp46_133+ROcp46_733*qdd(34)+qd(34)*(OMcp46_233*ROcp46_933-OMcp46_333*ROcp46_833);
    OPcp46_234 = OPcp46_233+ROcp46_833*qdd(34)-qd(34)*(OMcp46_133*ROcp46_933-OMcp46_333*ROcp46_733);
    OPcp46_334 = OPcp46_333+ROcp46_933*qdd(34)+qd(34)*(OMcp46_133*ROcp46_833-OMcp46_233*ROcp46_733);

% = = Block_1_0_0_47_0_6 = = 
 
% Sensor Kinematics 


    ROcp46_444 = ROcp46_434*C44+ROcp46_733*S44;
    ROcp46_544 = ROcp46_534*C44+ROcp46_833*S44;
    ROcp46_644 = ROcp46_634*C44+ROcp46_933*S44;
    ROcp46_744 = -(ROcp46_434*S44-ROcp46_733*C44);
    ROcp46_844 = -(ROcp46_534*S44-ROcp46_833*C44);
    ROcp46_944 = -(ROcp46_634*S44-ROcp46_933*C44);
    ROcp46_145 = ROcp46_134*C45+ROcp46_444*S45;
    ROcp46_245 = ROcp46_234*C45+ROcp46_544*S45;
    ROcp46_345 = ROcp46_334*C45+ROcp46_644*S45;
    ROcp46_445 = -(ROcp46_134*S45-ROcp46_444*C45);
    ROcp46_545 = -(ROcp46_234*S45-ROcp46_544*C45);
    ROcp46_645 = -(ROcp46_334*S45-ROcp46_644*C45);
    ROcp46_146 = ROcp46_145*C46-ROcp46_744*S46;
    ROcp46_246 = ROcp46_245*C46-ROcp46_844*S46;
    ROcp46_346 = ROcp46_345*C46-ROcp46_944*S46;
    ROcp46_746 = ROcp46_145*S46+ROcp46_744*C46;
    ROcp46_846 = ROcp46_245*S46+ROcp46_844*C46;
    ROcp46_946 = ROcp46_345*S46+ROcp46_944*C46;
    ROcp46_447 = ROcp46_445*C47+ROcp46_746*S47;
    ROcp46_547 = ROcp46_545*C47+ROcp46_846*S47;
    ROcp46_647 = ROcp46_645*C47+ROcp46_946*S47;
    ROcp46_747 = -(ROcp46_445*S47-ROcp46_746*C47);
    ROcp46_847 = -(ROcp46_545*S47-ROcp46_846*C47);
    ROcp46_947 = -(ROcp46_645*S47-ROcp46_946*C47);
    RLcp46_144 = ROcp46_134*s.dpt(1,24)+ROcp46_434*s.dpt(2,24)+ROcp46_733*s.dpt(3,24);
    RLcp46_244 = ROcp46_234*s.dpt(1,24)+ROcp46_534*s.dpt(2,24)+ROcp46_833*s.dpt(3,24);
    RLcp46_344 = ROcp46_334*s.dpt(1,24)+ROcp46_634*s.dpt(2,24)+ROcp46_933*s.dpt(3,24);
    ORcp46_144 = OMcp46_234*RLcp46_344-OMcp46_334*RLcp46_244;
    ORcp46_244 = -(OMcp46_134*RLcp46_344-OMcp46_334*RLcp46_144);
    ORcp46_344 = OMcp46_134*RLcp46_244-OMcp46_234*RLcp46_144;
    OMcp46_146 = OMcp46_134+ROcp46_445*qd(46);
    OMcp46_246 = OMcp46_234+ROcp46_545*qd(46);
    OMcp46_346 = OMcp46_334+ROcp46_645*qd(46);
    OPcp46_146 = OPcp46_134+ROcp46_445*qdd(46)+qd(46)*(OMcp46_234*ROcp46_645-OMcp46_334*ROcp46_545);
    OPcp46_246 = OPcp46_234+ROcp46_545*qdd(46)-qd(46)*(OMcp46_134*ROcp46_645-OMcp46_334*ROcp46_445);
    OPcp46_346 = OPcp46_334+ROcp46_645*qdd(46)+qd(46)*(OMcp46_134*ROcp46_545-OMcp46_234*ROcp46_445);
    RLcp46_147 = ROcp46_445*s.dpt(2,41)+ROcp46_746*s.dpt(3,41);
    RLcp46_247 = ROcp46_545*s.dpt(2,41)+ROcp46_846*s.dpt(3,41);
    RLcp46_347 = ROcp46_645*s.dpt(2,41)+ROcp46_946*s.dpt(3,41);
    POcp46_147 = RLcp46_131+RLcp46_134+RLcp46_144+RLcp46_147+q(1);
    POcp46_247 = RLcp46_231+RLcp46_234+RLcp46_244+RLcp46_247+q(2);
    POcp46_347 = RLcp46_331+RLcp46_334+RLcp46_344+RLcp46_347+q(3);
    OMcp46_147 = OMcp46_146+ROcp46_146*qd(47);
    OMcp46_247 = OMcp46_246+ROcp46_246*qd(47);
    OMcp46_347 = OMcp46_346+ROcp46_346*qd(47);
    ORcp46_147 = OMcp46_246*RLcp46_347-OMcp46_346*RLcp46_247;
    ORcp46_247 = -(OMcp46_146*RLcp46_347-OMcp46_346*RLcp46_147);
    ORcp46_347 = OMcp46_146*RLcp46_247-OMcp46_246*RLcp46_147;
    VIcp46_147 = ORcp46_131+ORcp46_134+ORcp46_144+ORcp46_147+qd(1);
    VIcp46_247 = ORcp46_231+ORcp46_234+ORcp46_244+ORcp46_247+qd(2);
    VIcp46_347 = ORcp46_331+ORcp46_334+ORcp46_344+ORcp46_347+qd(3);
    OPcp46_147 = OPcp46_146+ROcp46_146*qdd(47)+qd(47)*(OMcp46_246*ROcp46_346-OMcp46_346*ROcp46_246);
    OPcp46_247 = OPcp46_246+ROcp46_246*qdd(47)-qd(47)*(OMcp46_146*ROcp46_346-OMcp46_346*ROcp46_146);
    OPcp46_347 = OPcp46_346+ROcp46_346*qdd(47)+qd(47)*(OMcp46_146*ROcp46_246-OMcp46_246*ROcp46_146);
    ACcp46_147 = qdd(1)+OMcp46_233*ORcp46_334+OMcp46_234*ORcp46_344+OMcp46_246*ORcp46_347+OMcp46_26*ORcp46_331-OMcp46_333*ORcp46_234-OMcp46_334*...
 ORcp46_244-OMcp46_346*ORcp46_247-OMcp46_36*ORcp46_231+OPcp46_233*RLcp46_334+OPcp46_234*RLcp46_344+OPcp46_246*RLcp46_347+OPcp46_26*RLcp46_331-...
 OPcp46_333*RLcp46_234-OPcp46_334*RLcp46_244-OPcp46_346*RLcp46_247-OPcp46_36*RLcp46_231;
    ACcp46_247 = qdd(2)-OMcp46_133*ORcp46_334-OMcp46_134*ORcp46_344-OMcp46_146*ORcp46_347-OMcp46_16*ORcp46_331+OMcp46_333*ORcp46_134+OMcp46_334*...
 ORcp46_144+OMcp46_346*ORcp46_147+OMcp46_36*ORcp46_131-OPcp46_133*RLcp46_334-OPcp46_134*RLcp46_344-OPcp46_146*RLcp46_347-OPcp46_16*RLcp46_331+...
 OPcp46_333*RLcp46_134+OPcp46_334*RLcp46_144+OPcp46_346*RLcp46_147+OPcp46_36*RLcp46_131;
    ACcp46_347 = qdd(3)+OMcp46_133*ORcp46_234+OMcp46_134*ORcp46_244+OMcp46_146*ORcp46_247+OMcp46_16*ORcp46_231-OMcp46_233*ORcp46_134-OMcp46_234*...
 ORcp46_144-OMcp46_246*ORcp46_147-OMcp46_26*ORcp46_131+OPcp46_133*RLcp46_234+OPcp46_134*RLcp46_244+OPcp46_146*RLcp46_247+OPcp46_16*RLcp46_231-...
 OPcp46_233*RLcp46_134-OPcp46_234*RLcp46_144-OPcp46_246*RLcp46_147-OPcp46_26*RLcp46_131;

% = = Block_1_0_0_47_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp46_147;
    sens.P(2) = POcp46_247;
    sens.P(3) = POcp46_347;
    sens.R(1,1) = ROcp46_146;
    sens.R(1,2) = ROcp46_246;
    sens.R(1,3) = ROcp46_346;
    sens.R(2,1) = ROcp46_447;
    sens.R(2,2) = ROcp46_547;
    sens.R(2,3) = ROcp46_647;
    sens.R(3,1) = ROcp46_747;
    sens.R(3,2) = ROcp46_847;
    sens.R(3,3) = ROcp46_947;
    sens.V(1) = VIcp46_147;
    sens.V(2) = VIcp46_247;
    sens.V(3) = VIcp46_347;
    sens.OM(1) = OMcp46_147;
    sens.OM(2) = OMcp46_247;
    sens.OM(3) = OMcp46_347;
    sens.A(1) = ACcp46_147;
    sens.A(2) = ACcp46_247;
    sens.A(3) = ACcp46_347;
    sens.OMP(1) = OPcp46_147;
    sens.OMP(2) = OPcp46_247;
    sens.OMP(3) = OPcp46_347;
 
% 
case 48, 


% = = Block_1_0_0_48_0_1 = = 
 
% Sensor Kinematics 


    ROcp47_25 = S4*S5;
    ROcp47_35 = -C4*S5;
    ROcp47_85 = -S4*C5;
    ROcp47_95 = C4*C5;
    ROcp47_16 = C5*C6;
    ROcp47_26 = ROcp47_25*C6+C4*S6;
    ROcp47_36 = ROcp47_35*C6+S4*S6;
    ROcp47_46 = -C5*S6;
    ROcp47_56 = -(ROcp47_25*S6-C4*C6);
    ROcp47_66 = -(ROcp47_35*S6-S4*C6);
    OMcp47_25 = qd(5)*C4;
    OMcp47_35 = qd(5)*S4;
    OMcp47_16 = qd(4)+qd(6)*S5;
    OMcp47_26 = OMcp47_25+ROcp47_85*qd(6);
    OMcp47_36 = OMcp47_35+ROcp47_95*qd(6);
    OPcp47_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp47_26 = ROcp47_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp47_35*S5-ROcp47_95*qd(4));
    OPcp47_36 = ROcp47_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp47_25*S5-ROcp47_85*qd(4));

% = = Block_1_0_0_48_0_4 = = 
 
% Sensor Kinematics 


    ROcp47_131 = ROcp47_16*C31-S31*S5;
    ROcp47_231 = ROcp47_26*C31-ROcp47_85*S31;
    ROcp47_331 = ROcp47_36*C31-ROcp47_95*S31;
    ROcp47_731 = ROcp47_16*S31+C31*S5;
    ROcp47_831 = ROcp47_26*S31+ROcp47_85*C31;
    ROcp47_931 = ROcp47_36*S31+ROcp47_95*C31;
    ROcp47_432 = ROcp47_46*C32+ROcp47_731*S32;
    ROcp47_532 = ROcp47_56*C32+ROcp47_831*S32;
    ROcp47_632 = ROcp47_66*C32+ROcp47_931*S32;
    ROcp47_732 = -(ROcp47_46*S32-ROcp47_731*C32);
    ROcp47_832 = -(ROcp47_56*S32-ROcp47_831*C32);
    ROcp47_932 = -(ROcp47_66*S32-ROcp47_931*C32);
    ROcp47_133 = ROcp47_131*C33-ROcp47_732*S33;
    ROcp47_233 = ROcp47_231*C33-ROcp47_832*S33;
    ROcp47_333 = ROcp47_331*C33-ROcp47_932*S33;
    ROcp47_733 = ROcp47_131*S33+ROcp47_732*C33;
    ROcp47_833 = ROcp47_231*S33+ROcp47_832*C33;
    ROcp47_933 = ROcp47_331*S33+ROcp47_932*C33;
    ROcp47_134 = ROcp47_133*C34+ROcp47_432*S34;
    ROcp47_234 = ROcp47_233*C34+ROcp47_532*S34;
    ROcp47_334 = ROcp47_333*C34+ROcp47_632*S34;
    ROcp47_434 = -(ROcp47_133*S34-ROcp47_432*C34);
    ROcp47_534 = -(ROcp47_233*S34-ROcp47_532*C34);
    ROcp47_634 = -(ROcp47_333*S34-ROcp47_632*C34);
    RLcp47_131 = ROcp47_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp47_231 = ROcp47_26*s.dpt(1,3)+ROcp47_85*s.dpt(3,3);
    RLcp47_331 = ROcp47_36*s.dpt(1,3)+ROcp47_95*s.dpt(3,3);
    OMcp47_131 = OMcp47_16+ROcp47_46*qd(31);
    OMcp47_231 = OMcp47_26+ROcp47_56*qd(31);
    OMcp47_331 = OMcp47_36+ROcp47_66*qd(31);
    ORcp47_131 = OMcp47_26*RLcp47_331-OMcp47_36*RLcp47_231;
    ORcp47_231 = -(OMcp47_16*RLcp47_331-OMcp47_36*RLcp47_131);
    ORcp47_331 = OMcp47_16*RLcp47_231-OMcp47_26*RLcp47_131;
    OMcp47_132 = OMcp47_131+ROcp47_131*qd(32);
    OMcp47_232 = OMcp47_231+ROcp47_231*qd(32);
    OMcp47_332 = OMcp47_331+ROcp47_331*qd(32);
    OMcp47_133 = OMcp47_132+ROcp47_432*qd(33);
    OMcp47_233 = OMcp47_232+ROcp47_532*qd(33);
    OMcp47_333 = OMcp47_332+ROcp47_632*qd(33);
    OPcp47_133 = OPcp47_16+ROcp47_131*qdd(32)+ROcp47_432*qdd(33)+ROcp47_46*qdd(31)+qd(31)*(OMcp47_26*ROcp47_66-OMcp47_36*ROcp47_56)+qd(32)*(...
 OMcp47_231*ROcp47_331-OMcp47_331*ROcp47_231)+qd(33)*(OMcp47_232*ROcp47_632-OMcp47_332*ROcp47_532);
    OPcp47_233 = OPcp47_26+ROcp47_231*qdd(32)+ROcp47_532*qdd(33)+ROcp47_56*qdd(31)-qd(31)*(OMcp47_16*ROcp47_66-OMcp47_36*ROcp47_46)-qd(32)*(...
 OMcp47_131*ROcp47_331-OMcp47_331*ROcp47_131)-qd(33)*(OMcp47_132*ROcp47_632-OMcp47_332*ROcp47_432);
    OPcp47_333 = OPcp47_36+ROcp47_331*qdd(32)+ROcp47_632*qdd(33)+ROcp47_66*qdd(31)+qd(31)*(OMcp47_16*ROcp47_56-OMcp47_26*ROcp47_46)+qd(32)*(...
 OMcp47_131*ROcp47_231-OMcp47_231*ROcp47_131)+qd(33)*(OMcp47_132*ROcp47_532-OMcp47_232*ROcp47_432);
    RLcp47_134 = ROcp47_733*s.dpt(3,21);
    RLcp47_234 = ROcp47_833*s.dpt(3,21);
    RLcp47_334 = ROcp47_933*s.dpt(3,21);
    OMcp47_134 = OMcp47_133+ROcp47_733*qd(34);
    OMcp47_234 = OMcp47_233+ROcp47_833*qd(34);
    OMcp47_334 = OMcp47_333+ROcp47_933*qd(34);
    ORcp47_134 = OMcp47_233*RLcp47_334-OMcp47_333*RLcp47_234;
    ORcp47_234 = -(OMcp47_133*RLcp47_334-OMcp47_333*RLcp47_134);
    ORcp47_334 = OMcp47_133*RLcp47_234-OMcp47_233*RLcp47_134;
    OPcp47_134 = OPcp47_133+ROcp47_733*qdd(34)+qd(34)*(OMcp47_233*ROcp47_933-OMcp47_333*ROcp47_833);
    OPcp47_234 = OPcp47_233+ROcp47_833*qdd(34)-qd(34)*(OMcp47_133*ROcp47_933-OMcp47_333*ROcp47_733);
    OPcp47_334 = OPcp47_333+ROcp47_933*qdd(34)+qd(34)*(OMcp47_133*ROcp47_833-OMcp47_233*ROcp47_733);

% = = Block_1_0_0_48_0_6 = = 
 
% Sensor Kinematics 


    ROcp47_444 = ROcp47_434*C44+ROcp47_733*S44;
    ROcp47_544 = ROcp47_534*C44+ROcp47_833*S44;
    ROcp47_644 = ROcp47_634*C44+ROcp47_933*S44;
    ROcp47_744 = -(ROcp47_434*S44-ROcp47_733*C44);
    ROcp47_844 = -(ROcp47_534*S44-ROcp47_833*C44);
    ROcp47_944 = -(ROcp47_634*S44-ROcp47_933*C44);
    ROcp47_145 = ROcp47_134*C45+ROcp47_444*S45;
    ROcp47_245 = ROcp47_234*C45+ROcp47_544*S45;
    ROcp47_345 = ROcp47_334*C45+ROcp47_644*S45;
    ROcp47_445 = -(ROcp47_134*S45-ROcp47_444*C45);
    ROcp47_545 = -(ROcp47_234*S45-ROcp47_544*C45);
    ROcp47_645 = -(ROcp47_334*S45-ROcp47_644*C45);
    ROcp47_146 = ROcp47_145*C46-ROcp47_744*S46;
    ROcp47_246 = ROcp47_245*C46-ROcp47_844*S46;
    ROcp47_346 = ROcp47_345*C46-ROcp47_944*S46;
    ROcp47_746 = ROcp47_145*S46+ROcp47_744*C46;
    ROcp47_846 = ROcp47_245*S46+ROcp47_844*C46;
    ROcp47_946 = ROcp47_345*S46+ROcp47_944*C46;
    ROcp47_447 = ROcp47_445*C47+ROcp47_746*S47;
    ROcp47_547 = ROcp47_545*C47+ROcp47_846*S47;
    ROcp47_647 = ROcp47_645*C47+ROcp47_946*S47;
    ROcp47_747 = -(ROcp47_445*S47-ROcp47_746*C47);
    ROcp47_847 = -(ROcp47_545*S47-ROcp47_846*C47);
    ROcp47_947 = -(ROcp47_645*S47-ROcp47_946*C47);
    ROcp47_148 = ROcp47_146*C48+ROcp47_447*S48;
    ROcp47_248 = ROcp47_246*C48+ROcp47_547*S48;
    ROcp47_348 = ROcp47_346*C48+ROcp47_647*S48;
    ROcp47_448 = -(ROcp47_146*S48-ROcp47_447*C48);
    ROcp47_548 = -(ROcp47_246*S48-ROcp47_547*C48);
    ROcp47_648 = -(ROcp47_346*S48-ROcp47_647*C48);
    RLcp47_144 = ROcp47_134*s.dpt(1,24)+ROcp47_434*s.dpt(2,24)+ROcp47_733*s.dpt(3,24);
    RLcp47_244 = ROcp47_234*s.dpt(1,24)+ROcp47_534*s.dpt(2,24)+ROcp47_833*s.dpt(3,24);
    RLcp47_344 = ROcp47_334*s.dpt(1,24)+ROcp47_634*s.dpt(2,24)+ROcp47_933*s.dpt(3,24);
    ORcp47_144 = OMcp47_234*RLcp47_344-OMcp47_334*RLcp47_244;
    ORcp47_244 = -(OMcp47_134*RLcp47_344-OMcp47_334*RLcp47_144);
    ORcp47_344 = OMcp47_134*RLcp47_244-OMcp47_234*RLcp47_144;
    OMcp47_146 = OMcp47_134+ROcp47_445*qd(46);
    OMcp47_246 = OMcp47_234+ROcp47_545*qd(46);
    OMcp47_346 = OMcp47_334+ROcp47_645*qd(46);
    OPcp47_146 = OPcp47_134+ROcp47_445*qdd(46)+qd(46)*(OMcp47_234*ROcp47_645-OMcp47_334*ROcp47_545);
    OPcp47_246 = OPcp47_234+ROcp47_545*qdd(46)-qd(46)*(OMcp47_134*ROcp47_645-OMcp47_334*ROcp47_445);
    OPcp47_346 = OPcp47_334+ROcp47_645*qdd(46)+qd(46)*(OMcp47_134*ROcp47_545-OMcp47_234*ROcp47_445);
    RLcp47_147 = ROcp47_445*s.dpt(2,41)+ROcp47_746*s.dpt(3,41);
    RLcp47_247 = ROcp47_545*s.dpt(2,41)+ROcp47_846*s.dpt(3,41);
    RLcp47_347 = ROcp47_645*s.dpt(2,41)+ROcp47_946*s.dpt(3,41);
    OMcp47_147 = OMcp47_146+ROcp47_146*qd(47);
    OMcp47_247 = OMcp47_246+ROcp47_246*qd(47);
    OMcp47_347 = OMcp47_346+ROcp47_346*qd(47);
    ORcp47_147 = OMcp47_246*RLcp47_347-OMcp47_346*RLcp47_247;
    ORcp47_247 = -(OMcp47_146*RLcp47_347-OMcp47_346*RLcp47_147);
    ORcp47_347 = OMcp47_146*RLcp47_247-OMcp47_246*RLcp47_147;
    OPcp47_147 = OPcp47_146+ROcp47_146*qdd(47)+qd(47)*(OMcp47_246*ROcp47_346-OMcp47_346*ROcp47_246);
    OPcp47_247 = OPcp47_246+ROcp47_246*qdd(47)-qd(47)*(OMcp47_146*ROcp47_346-OMcp47_346*ROcp47_146);
    OPcp47_347 = OPcp47_346+ROcp47_346*qdd(47)+qd(47)*(OMcp47_146*ROcp47_246-OMcp47_246*ROcp47_146);
    RLcp47_148 = ROcp47_747*s.dpt(3,43);
    RLcp47_248 = ROcp47_847*s.dpt(3,43);
    RLcp47_348 = ROcp47_947*s.dpt(3,43);
    POcp47_148 = RLcp47_131+RLcp47_134+RLcp47_144+RLcp47_147+RLcp47_148+q(1);
    POcp47_248 = RLcp47_231+RLcp47_234+RLcp47_244+RLcp47_247+RLcp47_248+q(2);
    POcp47_348 = RLcp47_331+RLcp47_334+RLcp47_344+RLcp47_347+RLcp47_348+q(3);
    OMcp47_148 = OMcp47_147+ROcp47_747*qd(48);
    OMcp47_248 = OMcp47_247+ROcp47_847*qd(48);
    OMcp47_348 = OMcp47_347+ROcp47_947*qd(48);
    ORcp47_148 = OMcp47_247*RLcp47_348-OMcp47_347*RLcp47_248;
    ORcp47_248 = -(OMcp47_147*RLcp47_348-OMcp47_347*RLcp47_148);
    ORcp47_348 = OMcp47_147*RLcp47_248-OMcp47_247*RLcp47_148;
    VIcp47_148 = ORcp47_131+ORcp47_134+ORcp47_144+ORcp47_147+ORcp47_148+qd(1);
    VIcp47_248 = ORcp47_231+ORcp47_234+ORcp47_244+ORcp47_247+ORcp47_248+qd(2);
    VIcp47_348 = ORcp47_331+ORcp47_334+ORcp47_344+ORcp47_347+ORcp47_348+qd(3);
    OPcp47_148 = OPcp47_147+ROcp47_747*qdd(48)+qd(48)*(OMcp47_247*ROcp47_947-OMcp47_347*ROcp47_847);
    OPcp47_248 = OPcp47_247+ROcp47_847*qdd(48)-qd(48)*(OMcp47_147*ROcp47_947-OMcp47_347*ROcp47_747);
    OPcp47_348 = OPcp47_347+ROcp47_947*qdd(48)+qd(48)*(OMcp47_147*ROcp47_847-OMcp47_247*ROcp47_747);
    ACcp47_148 = qdd(1)+OMcp47_233*ORcp47_334+OMcp47_234*ORcp47_344+OMcp47_246*ORcp47_347+OMcp47_247*ORcp47_348+OMcp47_26*ORcp47_331-OMcp47_333*...
 ORcp47_234-OMcp47_334*ORcp47_244-OMcp47_346*ORcp47_247-OMcp47_347*ORcp47_248-OMcp47_36*ORcp47_231+OPcp47_233*RLcp47_334+OPcp47_234*RLcp47_344+...
 OPcp47_246*RLcp47_347+OPcp47_247*RLcp47_348+OPcp47_26*RLcp47_331-OPcp47_333*RLcp47_234-OPcp47_334*RLcp47_244-OPcp47_346*RLcp47_247-OPcp47_347*...
 RLcp47_248-OPcp47_36*RLcp47_231;
    ACcp47_248 = qdd(2)-OMcp47_133*ORcp47_334-OMcp47_134*ORcp47_344-OMcp47_146*ORcp47_347-OMcp47_147*ORcp47_348-OMcp47_16*ORcp47_331+OMcp47_333*...
 ORcp47_134+OMcp47_334*ORcp47_144+OMcp47_346*ORcp47_147+OMcp47_347*ORcp47_148+OMcp47_36*ORcp47_131-OPcp47_133*RLcp47_334-OPcp47_134*RLcp47_344-...
 OPcp47_146*RLcp47_347-OPcp47_147*RLcp47_348-OPcp47_16*RLcp47_331+OPcp47_333*RLcp47_134+OPcp47_334*RLcp47_144+OPcp47_346*RLcp47_147+OPcp47_347*...
 RLcp47_148+OPcp47_36*RLcp47_131;
    ACcp47_348 = qdd(3)+OMcp47_133*ORcp47_234+OMcp47_134*ORcp47_244+OMcp47_146*ORcp47_247+OMcp47_147*ORcp47_248+OMcp47_16*ORcp47_231-OMcp47_233*...
 ORcp47_134-OMcp47_234*ORcp47_144-OMcp47_246*ORcp47_147-OMcp47_247*ORcp47_148-OMcp47_26*ORcp47_131+OPcp47_133*RLcp47_234+OPcp47_134*RLcp47_244+...
 OPcp47_146*RLcp47_247+OPcp47_147*RLcp47_248+OPcp47_16*RLcp47_231-OPcp47_233*RLcp47_134-OPcp47_234*RLcp47_144-OPcp47_246*RLcp47_147-OPcp47_247*...
 RLcp47_148-OPcp47_26*RLcp47_131;

% = = Block_1_0_0_48_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp47_148;
    sens.P(2) = POcp47_248;
    sens.P(3) = POcp47_348;
    sens.R(1,1) = ROcp47_148;
    sens.R(1,2) = ROcp47_248;
    sens.R(1,3) = ROcp47_348;
    sens.R(2,1) = ROcp47_448;
    sens.R(2,2) = ROcp47_548;
    sens.R(2,3) = ROcp47_648;
    sens.R(3,1) = ROcp47_747;
    sens.R(3,2) = ROcp47_847;
    sens.R(3,3) = ROcp47_947;
    sens.V(1) = VIcp47_148;
    sens.V(2) = VIcp47_248;
    sens.V(3) = VIcp47_348;
    sens.OM(1) = OMcp47_148;
    sens.OM(2) = OMcp47_248;
    sens.OM(3) = OMcp47_348;
    sens.A(1) = ACcp47_148;
    sens.A(2) = ACcp47_248;
    sens.A(3) = ACcp47_348;
    sens.OMP(1) = OPcp47_148;
    sens.OMP(2) = OPcp47_248;
    sens.OMP(3) = OPcp47_348;
 
% 
case 49, 


% = = Block_1_0_0_49_0_1 = = 
 
% Sensor Kinematics 


    ROcp48_25 = S4*S5;
    ROcp48_35 = -C4*S5;
    ROcp48_85 = -S4*C5;
    ROcp48_95 = C4*C5;
    ROcp48_16 = C5*C6;
    ROcp48_26 = ROcp48_25*C6+C4*S6;
    ROcp48_36 = ROcp48_35*C6+S4*S6;
    ROcp48_46 = -C5*S6;
    ROcp48_56 = -(ROcp48_25*S6-C4*C6);
    ROcp48_66 = -(ROcp48_35*S6-S4*C6);
    OMcp48_25 = qd(5)*C4;
    OMcp48_35 = qd(5)*S4;
    OMcp48_16 = qd(4)+qd(6)*S5;
    OMcp48_26 = OMcp48_25+ROcp48_85*qd(6);
    OMcp48_36 = OMcp48_35+ROcp48_95*qd(6);
    OPcp48_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp48_26 = ROcp48_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp48_35*S5-ROcp48_95*qd(4));
    OPcp48_36 = ROcp48_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp48_25*S5-ROcp48_85*qd(4));

% = = Block_1_0_0_49_0_4 = = 
 
% Sensor Kinematics 


    ROcp48_131 = ROcp48_16*C31-S31*S5;
    ROcp48_231 = ROcp48_26*C31-ROcp48_85*S31;
    ROcp48_331 = ROcp48_36*C31-ROcp48_95*S31;
    ROcp48_731 = ROcp48_16*S31+C31*S5;
    ROcp48_831 = ROcp48_26*S31+ROcp48_85*C31;
    ROcp48_931 = ROcp48_36*S31+ROcp48_95*C31;
    ROcp48_432 = ROcp48_46*C32+ROcp48_731*S32;
    ROcp48_532 = ROcp48_56*C32+ROcp48_831*S32;
    ROcp48_632 = ROcp48_66*C32+ROcp48_931*S32;
    ROcp48_732 = -(ROcp48_46*S32-ROcp48_731*C32);
    ROcp48_832 = -(ROcp48_56*S32-ROcp48_831*C32);
    ROcp48_932 = -(ROcp48_66*S32-ROcp48_931*C32);
    ROcp48_133 = ROcp48_131*C33-ROcp48_732*S33;
    ROcp48_233 = ROcp48_231*C33-ROcp48_832*S33;
    ROcp48_333 = ROcp48_331*C33-ROcp48_932*S33;
    ROcp48_733 = ROcp48_131*S33+ROcp48_732*C33;
    ROcp48_833 = ROcp48_231*S33+ROcp48_832*C33;
    ROcp48_933 = ROcp48_331*S33+ROcp48_932*C33;
    ROcp48_134 = ROcp48_133*C34+ROcp48_432*S34;
    ROcp48_234 = ROcp48_233*C34+ROcp48_532*S34;
    ROcp48_334 = ROcp48_333*C34+ROcp48_632*S34;
    ROcp48_434 = -(ROcp48_133*S34-ROcp48_432*C34);
    ROcp48_534 = -(ROcp48_233*S34-ROcp48_532*C34);
    ROcp48_634 = -(ROcp48_333*S34-ROcp48_632*C34);
    RLcp48_131 = ROcp48_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp48_231 = ROcp48_26*s.dpt(1,3)+ROcp48_85*s.dpt(3,3);
    RLcp48_331 = ROcp48_36*s.dpt(1,3)+ROcp48_95*s.dpt(3,3);
    OMcp48_131 = OMcp48_16+ROcp48_46*qd(31);
    OMcp48_231 = OMcp48_26+ROcp48_56*qd(31);
    OMcp48_331 = OMcp48_36+ROcp48_66*qd(31);
    ORcp48_131 = OMcp48_26*RLcp48_331-OMcp48_36*RLcp48_231;
    ORcp48_231 = -(OMcp48_16*RLcp48_331-OMcp48_36*RLcp48_131);
    ORcp48_331 = OMcp48_16*RLcp48_231-OMcp48_26*RLcp48_131;
    OMcp48_132 = OMcp48_131+ROcp48_131*qd(32);
    OMcp48_232 = OMcp48_231+ROcp48_231*qd(32);
    OMcp48_332 = OMcp48_331+ROcp48_331*qd(32);
    OMcp48_133 = OMcp48_132+ROcp48_432*qd(33);
    OMcp48_233 = OMcp48_232+ROcp48_532*qd(33);
    OMcp48_333 = OMcp48_332+ROcp48_632*qd(33);
    OPcp48_133 = OPcp48_16+ROcp48_131*qdd(32)+ROcp48_432*qdd(33)+ROcp48_46*qdd(31)+qd(31)*(OMcp48_26*ROcp48_66-OMcp48_36*ROcp48_56)+qd(32)*(...
 OMcp48_231*ROcp48_331-OMcp48_331*ROcp48_231)+qd(33)*(OMcp48_232*ROcp48_632-OMcp48_332*ROcp48_532);
    OPcp48_233 = OPcp48_26+ROcp48_231*qdd(32)+ROcp48_532*qdd(33)+ROcp48_56*qdd(31)-qd(31)*(OMcp48_16*ROcp48_66-OMcp48_36*ROcp48_46)-qd(32)*(...
 OMcp48_131*ROcp48_331-OMcp48_331*ROcp48_131)-qd(33)*(OMcp48_132*ROcp48_632-OMcp48_332*ROcp48_432);
    OPcp48_333 = OPcp48_36+ROcp48_331*qdd(32)+ROcp48_632*qdd(33)+ROcp48_66*qdd(31)+qd(31)*(OMcp48_16*ROcp48_56-OMcp48_26*ROcp48_46)+qd(32)*(...
 OMcp48_131*ROcp48_231-OMcp48_231*ROcp48_131)+qd(33)*(OMcp48_132*ROcp48_532-OMcp48_232*ROcp48_432);
    RLcp48_134 = ROcp48_733*s.dpt(3,21);
    RLcp48_234 = ROcp48_833*s.dpt(3,21);
    RLcp48_334 = ROcp48_933*s.dpt(3,21);
    OMcp48_134 = OMcp48_133+ROcp48_733*qd(34);
    OMcp48_234 = OMcp48_233+ROcp48_833*qd(34);
    OMcp48_334 = OMcp48_333+ROcp48_933*qd(34);
    ORcp48_134 = OMcp48_233*RLcp48_334-OMcp48_333*RLcp48_234;
    ORcp48_234 = -(OMcp48_133*RLcp48_334-OMcp48_333*RLcp48_134);
    ORcp48_334 = OMcp48_133*RLcp48_234-OMcp48_233*RLcp48_134;
    OPcp48_134 = OPcp48_133+ROcp48_733*qdd(34)+qd(34)*(OMcp48_233*ROcp48_933-OMcp48_333*ROcp48_833);
    OPcp48_234 = OPcp48_233+ROcp48_833*qdd(34)-qd(34)*(OMcp48_133*ROcp48_933-OMcp48_333*ROcp48_733);
    OPcp48_334 = OPcp48_333+ROcp48_933*qdd(34)+qd(34)*(OMcp48_133*ROcp48_833-OMcp48_233*ROcp48_733);

% = = Block_1_0_0_49_0_6 = = 
 
% Sensor Kinematics 


    ROcp48_444 = ROcp48_434*C44+ROcp48_733*S44;
    ROcp48_544 = ROcp48_534*C44+ROcp48_833*S44;
    ROcp48_644 = ROcp48_634*C44+ROcp48_933*S44;
    ROcp48_744 = -(ROcp48_434*S44-ROcp48_733*C44);
    ROcp48_844 = -(ROcp48_534*S44-ROcp48_833*C44);
    ROcp48_944 = -(ROcp48_634*S44-ROcp48_933*C44);
    ROcp48_145 = ROcp48_134*C45+ROcp48_444*S45;
    ROcp48_245 = ROcp48_234*C45+ROcp48_544*S45;
    ROcp48_345 = ROcp48_334*C45+ROcp48_644*S45;
    ROcp48_445 = -(ROcp48_134*S45-ROcp48_444*C45);
    ROcp48_545 = -(ROcp48_234*S45-ROcp48_544*C45);
    ROcp48_645 = -(ROcp48_334*S45-ROcp48_644*C45);
    ROcp48_146 = ROcp48_145*C46-ROcp48_744*S46;
    ROcp48_246 = ROcp48_245*C46-ROcp48_844*S46;
    ROcp48_346 = ROcp48_345*C46-ROcp48_944*S46;
    ROcp48_746 = ROcp48_145*S46+ROcp48_744*C46;
    ROcp48_846 = ROcp48_245*S46+ROcp48_844*C46;
    ROcp48_946 = ROcp48_345*S46+ROcp48_944*C46;
    ROcp48_447 = ROcp48_445*C47+ROcp48_746*S47;
    ROcp48_547 = ROcp48_545*C47+ROcp48_846*S47;
    ROcp48_647 = ROcp48_645*C47+ROcp48_946*S47;
    ROcp48_747 = -(ROcp48_445*S47-ROcp48_746*C47);
    ROcp48_847 = -(ROcp48_545*S47-ROcp48_846*C47);
    ROcp48_947 = -(ROcp48_645*S47-ROcp48_946*C47);
    ROcp48_148 = ROcp48_146*C48+ROcp48_447*S48;
    ROcp48_248 = ROcp48_246*C48+ROcp48_547*S48;
    ROcp48_348 = ROcp48_346*C48+ROcp48_647*S48;
    ROcp48_448 = -(ROcp48_146*S48-ROcp48_447*C48);
    ROcp48_548 = -(ROcp48_246*S48-ROcp48_547*C48);
    ROcp48_648 = -(ROcp48_346*S48-ROcp48_647*C48);
    ROcp48_149 = ROcp48_148*C49-ROcp48_747*S49;
    ROcp48_249 = ROcp48_248*C49-ROcp48_847*S49;
    ROcp48_349 = ROcp48_348*C49-ROcp48_947*S49;
    ROcp48_749 = ROcp48_148*S49+ROcp48_747*C49;
    ROcp48_849 = ROcp48_248*S49+ROcp48_847*C49;
    ROcp48_949 = ROcp48_348*S49+ROcp48_947*C49;
    RLcp48_144 = ROcp48_134*s.dpt(1,24)+ROcp48_434*s.dpt(2,24)+ROcp48_733*s.dpt(3,24);
    RLcp48_244 = ROcp48_234*s.dpt(1,24)+ROcp48_534*s.dpt(2,24)+ROcp48_833*s.dpt(3,24);
    RLcp48_344 = ROcp48_334*s.dpt(1,24)+ROcp48_634*s.dpt(2,24)+ROcp48_933*s.dpt(3,24);
    ORcp48_144 = OMcp48_234*RLcp48_344-OMcp48_334*RLcp48_244;
    ORcp48_244 = -(OMcp48_134*RLcp48_344-OMcp48_334*RLcp48_144);
    ORcp48_344 = OMcp48_134*RLcp48_244-OMcp48_234*RLcp48_144;
    OMcp48_146 = OMcp48_134+ROcp48_445*qd(46);
    OMcp48_246 = OMcp48_234+ROcp48_545*qd(46);
    OMcp48_346 = OMcp48_334+ROcp48_645*qd(46);
    OPcp48_146 = OPcp48_134+ROcp48_445*qdd(46)+qd(46)*(OMcp48_234*ROcp48_645-OMcp48_334*ROcp48_545);
    OPcp48_246 = OPcp48_234+ROcp48_545*qdd(46)-qd(46)*(OMcp48_134*ROcp48_645-OMcp48_334*ROcp48_445);
    OPcp48_346 = OPcp48_334+ROcp48_645*qdd(46)+qd(46)*(OMcp48_134*ROcp48_545-OMcp48_234*ROcp48_445);
    RLcp48_147 = ROcp48_445*s.dpt(2,41)+ROcp48_746*s.dpt(3,41);
    RLcp48_247 = ROcp48_545*s.dpt(2,41)+ROcp48_846*s.dpt(3,41);
    RLcp48_347 = ROcp48_645*s.dpt(2,41)+ROcp48_946*s.dpt(3,41);
    OMcp48_147 = OMcp48_146+ROcp48_146*qd(47);
    OMcp48_247 = OMcp48_246+ROcp48_246*qd(47);
    OMcp48_347 = OMcp48_346+ROcp48_346*qd(47);
    ORcp48_147 = OMcp48_246*RLcp48_347-OMcp48_346*RLcp48_247;
    ORcp48_247 = -(OMcp48_146*RLcp48_347-OMcp48_346*RLcp48_147);
    ORcp48_347 = OMcp48_146*RLcp48_247-OMcp48_246*RLcp48_147;
    OPcp48_147 = OPcp48_146+ROcp48_146*qdd(47)+qd(47)*(OMcp48_246*ROcp48_346-OMcp48_346*ROcp48_246);
    OPcp48_247 = OPcp48_246+ROcp48_246*qdd(47)-qd(47)*(OMcp48_146*ROcp48_346-OMcp48_346*ROcp48_146);
    OPcp48_347 = OPcp48_346+ROcp48_346*qdd(47)+qd(47)*(OMcp48_146*ROcp48_246-OMcp48_246*ROcp48_146);
    RLcp48_148 = ROcp48_747*s.dpt(3,43);
    RLcp48_248 = ROcp48_847*s.dpt(3,43);
    RLcp48_348 = ROcp48_947*s.dpt(3,43);
    OMcp48_148 = OMcp48_147+ROcp48_747*qd(48);
    OMcp48_248 = OMcp48_247+ROcp48_847*qd(48);
    OMcp48_348 = OMcp48_347+ROcp48_947*qd(48);
    ORcp48_148 = OMcp48_247*RLcp48_348-OMcp48_347*RLcp48_248;
    ORcp48_248 = -(OMcp48_147*RLcp48_348-OMcp48_347*RLcp48_148);
    ORcp48_348 = OMcp48_147*RLcp48_248-OMcp48_247*RLcp48_148;
    OPcp48_148 = OPcp48_147+ROcp48_747*qdd(48)+qd(48)*(OMcp48_247*ROcp48_947-OMcp48_347*ROcp48_847);
    OPcp48_248 = OPcp48_247+ROcp48_847*qdd(48)-qd(48)*(OMcp48_147*ROcp48_947-OMcp48_347*ROcp48_747);
    OPcp48_348 = OPcp48_347+ROcp48_947*qdd(48)+qd(48)*(OMcp48_147*ROcp48_847-OMcp48_247*ROcp48_747);
    RLcp48_149 = ROcp48_148*s.dpt(1,46)+ROcp48_747*s.dpt(3,46);
    RLcp48_249 = ROcp48_248*s.dpt(1,46)+ROcp48_847*s.dpt(3,46);
    RLcp48_349 = ROcp48_348*s.dpt(1,46)+ROcp48_947*s.dpt(3,46);
    POcp48_149 = RLcp48_131+RLcp48_134+RLcp48_144+RLcp48_147+RLcp48_148+RLcp48_149+q(1);
    POcp48_249 = RLcp48_231+RLcp48_234+RLcp48_244+RLcp48_247+RLcp48_248+RLcp48_249+q(2);
    POcp48_349 = RLcp48_331+RLcp48_334+RLcp48_344+RLcp48_347+RLcp48_348+RLcp48_349+q(3);
    OMcp48_149 = OMcp48_148+ROcp48_448*qd(49);
    OMcp48_249 = OMcp48_248+ROcp48_548*qd(49);
    OMcp48_349 = OMcp48_348+ROcp48_648*qd(49);
    ORcp48_149 = OMcp48_248*RLcp48_349-OMcp48_348*RLcp48_249;
    ORcp48_249 = -(OMcp48_148*RLcp48_349-OMcp48_348*RLcp48_149);
    ORcp48_349 = OMcp48_148*RLcp48_249-OMcp48_248*RLcp48_149;
    VIcp48_149 = ORcp48_131+ORcp48_134+ORcp48_144+ORcp48_147+ORcp48_148+ORcp48_149+qd(1);
    VIcp48_249 = ORcp48_231+ORcp48_234+ORcp48_244+ORcp48_247+ORcp48_248+ORcp48_249+qd(2);
    VIcp48_349 = ORcp48_331+ORcp48_334+ORcp48_344+ORcp48_347+ORcp48_348+ORcp48_349+qd(3);
    OPcp48_149 = OPcp48_148+ROcp48_448*qdd(49)+qd(49)*(OMcp48_248*ROcp48_648-OMcp48_348*ROcp48_548);
    OPcp48_249 = OPcp48_248+ROcp48_548*qdd(49)-qd(49)*(OMcp48_148*ROcp48_648-OMcp48_348*ROcp48_448);
    OPcp48_349 = OPcp48_348+ROcp48_648*qdd(49)+qd(49)*(OMcp48_148*ROcp48_548-OMcp48_248*ROcp48_448);
    ACcp48_149 = qdd(1)+OMcp48_233*ORcp48_334+OMcp48_234*ORcp48_344+OMcp48_246*ORcp48_347+OMcp48_247*ORcp48_348+OMcp48_248*ORcp48_349+OMcp48_26*...
 ORcp48_331-OMcp48_333*ORcp48_234-OMcp48_334*ORcp48_244-OMcp48_346*ORcp48_247-OMcp48_347*ORcp48_248-OMcp48_348*ORcp48_249-OMcp48_36*ORcp48_231+...
 OPcp48_233*RLcp48_334+OPcp48_234*RLcp48_344+OPcp48_246*RLcp48_347+OPcp48_247*RLcp48_348+OPcp48_248*RLcp48_349+OPcp48_26*RLcp48_331-OPcp48_333*...
 RLcp48_234-OPcp48_334*RLcp48_244-OPcp48_346*RLcp48_247-OPcp48_347*RLcp48_248-OPcp48_348*RLcp48_249-OPcp48_36*RLcp48_231;
    ACcp48_249 = qdd(2)-OMcp48_133*ORcp48_334-OMcp48_134*ORcp48_344-OMcp48_146*ORcp48_347-OMcp48_147*ORcp48_348-OMcp48_148*ORcp48_349-OMcp48_16*...
 ORcp48_331+OMcp48_333*ORcp48_134+OMcp48_334*ORcp48_144+OMcp48_346*ORcp48_147+OMcp48_347*ORcp48_148+OMcp48_348*ORcp48_149+OMcp48_36*ORcp48_131-...
 OPcp48_133*RLcp48_334-OPcp48_134*RLcp48_344-OPcp48_146*RLcp48_347-OPcp48_147*RLcp48_348-OPcp48_148*RLcp48_349-OPcp48_16*RLcp48_331+OPcp48_333*...
 RLcp48_134+OPcp48_334*RLcp48_144+OPcp48_346*RLcp48_147+OPcp48_347*RLcp48_148+OPcp48_348*RLcp48_149+OPcp48_36*RLcp48_131;
    ACcp48_349 = qdd(3)+OMcp48_133*ORcp48_234+OMcp48_134*ORcp48_244+OMcp48_146*ORcp48_247+OMcp48_147*ORcp48_248+OMcp48_148*ORcp48_249+OMcp48_16*...
 ORcp48_231-OMcp48_233*ORcp48_134-OMcp48_234*ORcp48_144-OMcp48_246*ORcp48_147-OMcp48_247*ORcp48_148-OMcp48_248*ORcp48_149-OMcp48_26*ORcp48_131+...
 OPcp48_133*RLcp48_234+OPcp48_134*RLcp48_244+OPcp48_146*RLcp48_247+OPcp48_147*RLcp48_248+OPcp48_148*RLcp48_249+OPcp48_16*RLcp48_231-OPcp48_233*...
 RLcp48_134-OPcp48_234*RLcp48_144-OPcp48_246*RLcp48_147-OPcp48_247*RLcp48_148-OPcp48_248*RLcp48_149-OPcp48_26*RLcp48_131;

% = = Block_1_0_0_49_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp48_149;
    sens.P(2) = POcp48_249;
    sens.P(3) = POcp48_349;
    sens.R(1,1) = ROcp48_149;
    sens.R(1,2) = ROcp48_249;
    sens.R(1,3) = ROcp48_349;
    sens.R(2,1) = ROcp48_448;
    sens.R(2,2) = ROcp48_548;
    sens.R(2,3) = ROcp48_648;
    sens.R(3,1) = ROcp48_749;
    sens.R(3,2) = ROcp48_849;
    sens.R(3,3) = ROcp48_949;
    sens.V(1) = VIcp48_149;
    sens.V(2) = VIcp48_249;
    sens.V(3) = VIcp48_349;
    sens.OM(1) = OMcp48_149;
    sens.OM(2) = OMcp48_249;
    sens.OM(3) = OMcp48_349;
    sens.A(1) = ACcp48_149;
    sens.A(2) = ACcp48_249;
    sens.A(3) = ACcp48_349;
    sens.OMP(1) = OPcp48_149;
    sens.OMP(2) = OPcp48_249;
    sens.OMP(3) = OPcp48_349;
 
% 
case 50, 


% = = Block_1_0_0_50_0_1 = = 
 
% Sensor Kinematics 


    ROcp49_25 = S4*S5;
    ROcp49_35 = -C4*S5;
    ROcp49_85 = -S4*C5;
    ROcp49_95 = C4*C5;
    ROcp49_16 = C5*C6;
    ROcp49_26 = ROcp49_25*C6+C4*S6;
    ROcp49_36 = ROcp49_35*C6+S4*S6;
    ROcp49_46 = -C5*S6;
    ROcp49_56 = -(ROcp49_25*S6-C4*C6);
    ROcp49_66 = -(ROcp49_35*S6-S4*C6);
    OMcp49_25 = qd(5)*C4;
    OMcp49_35 = qd(5)*S4;
    OMcp49_16 = qd(4)+qd(6)*S5;
    OMcp49_26 = OMcp49_25+ROcp49_85*qd(6);
    OMcp49_36 = OMcp49_35+ROcp49_95*qd(6);
    OPcp49_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp49_26 = ROcp49_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp49_35*S5-ROcp49_95*qd(4));
    OPcp49_36 = ROcp49_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp49_25*S5-ROcp49_85*qd(4));

% = = Block_1_0_0_50_0_4 = = 
 
% Sensor Kinematics 


    ROcp49_131 = ROcp49_16*C31-S31*S5;
    ROcp49_231 = ROcp49_26*C31-ROcp49_85*S31;
    ROcp49_331 = ROcp49_36*C31-ROcp49_95*S31;
    ROcp49_731 = ROcp49_16*S31+C31*S5;
    ROcp49_831 = ROcp49_26*S31+ROcp49_85*C31;
    ROcp49_931 = ROcp49_36*S31+ROcp49_95*C31;
    ROcp49_432 = ROcp49_46*C32+ROcp49_731*S32;
    ROcp49_532 = ROcp49_56*C32+ROcp49_831*S32;
    ROcp49_632 = ROcp49_66*C32+ROcp49_931*S32;
    ROcp49_732 = -(ROcp49_46*S32-ROcp49_731*C32);
    ROcp49_832 = -(ROcp49_56*S32-ROcp49_831*C32);
    ROcp49_932 = -(ROcp49_66*S32-ROcp49_931*C32);
    ROcp49_133 = ROcp49_131*C33-ROcp49_732*S33;
    ROcp49_233 = ROcp49_231*C33-ROcp49_832*S33;
    ROcp49_333 = ROcp49_331*C33-ROcp49_932*S33;
    ROcp49_733 = ROcp49_131*S33+ROcp49_732*C33;
    ROcp49_833 = ROcp49_231*S33+ROcp49_832*C33;
    ROcp49_933 = ROcp49_331*S33+ROcp49_932*C33;
    ROcp49_134 = ROcp49_133*C34+ROcp49_432*S34;
    ROcp49_234 = ROcp49_233*C34+ROcp49_532*S34;
    ROcp49_334 = ROcp49_333*C34+ROcp49_632*S34;
    ROcp49_434 = -(ROcp49_133*S34-ROcp49_432*C34);
    ROcp49_534 = -(ROcp49_233*S34-ROcp49_532*C34);
    ROcp49_634 = -(ROcp49_333*S34-ROcp49_632*C34);
    RLcp49_131 = ROcp49_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp49_231 = ROcp49_26*s.dpt(1,3)+ROcp49_85*s.dpt(3,3);
    RLcp49_331 = ROcp49_36*s.dpt(1,3)+ROcp49_95*s.dpt(3,3);
    OMcp49_131 = OMcp49_16+ROcp49_46*qd(31);
    OMcp49_231 = OMcp49_26+ROcp49_56*qd(31);
    OMcp49_331 = OMcp49_36+ROcp49_66*qd(31);
    ORcp49_131 = OMcp49_26*RLcp49_331-OMcp49_36*RLcp49_231;
    ORcp49_231 = -(OMcp49_16*RLcp49_331-OMcp49_36*RLcp49_131);
    ORcp49_331 = OMcp49_16*RLcp49_231-OMcp49_26*RLcp49_131;
    OMcp49_132 = OMcp49_131+ROcp49_131*qd(32);
    OMcp49_232 = OMcp49_231+ROcp49_231*qd(32);
    OMcp49_332 = OMcp49_331+ROcp49_331*qd(32);
    OMcp49_133 = OMcp49_132+ROcp49_432*qd(33);
    OMcp49_233 = OMcp49_232+ROcp49_532*qd(33);
    OMcp49_333 = OMcp49_332+ROcp49_632*qd(33);
    OPcp49_133 = OPcp49_16+ROcp49_131*qdd(32)+ROcp49_432*qdd(33)+ROcp49_46*qdd(31)+qd(31)*(OMcp49_26*ROcp49_66-OMcp49_36*ROcp49_56)+qd(32)*(...
 OMcp49_231*ROcp49_331-OMcp49_331*ROcp49_231)+qd(33)*(OMcp49_232*ROcp49_632-OMcp49_332*ROcp49_532);
    OPcp49_233 = OPcp49_26+ROcp49_231*qdd(32)+ROcp49_532*qdd(33)+ROcp49_56*qdd(31)-qd(31)*(OMcp49_16*ROcp49_66-OMcp49_36*ROcp49_46)-qd(32)*(...
 OMcp49_131*ROcp49_331-OMcp49_331*ROcp49_131)-qd(33)*(OMcp49_132*ROcp49_632-OMcp49_332*ROcp49_432);
    OPcp49_333 = OPcp49_36+ROcp49_331*qdd(32)+ROcp49_632*qdd(33)+ROcp49_66*qdd(31)+qd(31)*(OMcp49_16*ROcp49_56-OMcp49_26*ROcp49_46)+qd(32)*(...
 OMcp49_131*ROcp49_231-OMcp49_231*ROcp49_131)+qd(33)*(OMcp49_132*ROcp49_532-OMcp49_232*ROcp49_432);
    RLcp49_134 = ROcp49_733*s.dpt(3,21);
    RLcp49_234 = ROcp49_833*s.dpt(3,21);
    RLcp49_334 = ROcp49_933*s.dpt(3,21);
    OMcp49_134 = OMcp49_133+ROcp49_733*qd(34);
    OMcp49_234 = OMcp49_233+ROcp49_833*qd(34);
    OMcp49_334 = OMcp49_333+ROcp49_933*qd(34);
    ORcp49_134 = OMcp49_233*RLcp49_334-OMcp49_333*RLcp49_234;
    ORcp49_234 = -(OMcp49_133*RLcp49_334-OMcp49_333*RLcp49_134);
    ORcp49_334 = OMcp49_133*RLcp49_234-OMcp49_233*RLcp49_134;
    OPcp49_134 = OPcp49_133+ROcp49_733*qdd(34)+qd(34)*(OMcp49_233*ROcp49_933-OMcp49_333*ROcp49_833);
    OPcp49_234 = OPcp49_233+ROcp49_833*qdd(34)-qd(34)*(OMcp49_133*ROcp49_933-OMcp49_333*ROcp49_733);
    OPcp49_334 = OPcp49_333+ROcp49_933*qdd(34)+qd(34)*(OMcp49_133*ROcp49_833-OMcp49_233*ROcp49_733);

% = = Block_1_0_0_50_0_6 = = 
 
% Sensor Kinematics 


    ROcp49_444 = ROcp49_434*C44+ROcp49_733*S44;
    ROcp49_544 = ROcp49_534*C44+ROcp49_833*S44;
    ROcp49_644 = ROcp49_634*C44+ROcp49_933*S44;
    ROcp49_744 = -(ROcp49_434*S44-ROcp49_733*C44);
    ROcp49_844 = -(ROcp49_534*S44-ROcp49_833*C44);
    ROcp49_944 = -(ROcp49_634*S44-ROcp49_933*C44);
    ROcp49_145 = ROcp49_134*C45+ROcp49_444*S45;
    ROcp49_245 = ROcp49_234*C45+ROcp49_544*S45;
    ROcp49_345 = ROcp49_334*C45+ROcp49_644*S45;
    ROcp49_445 = -(ROcp49_134*S45-ROcp49_444*C45);
    ROcp49_545 = -(ROcp49_234*S45-ROcp49_544*C45);
    ROcp49_645 = -(ROcp49_334*S45-ROcp49_644*C45);
    ROcp49_146 = ROcp49_145*C46-ROcp49_744*S46;
    ROcp49_246 = ROcp49_245*C46-ROcp49_844*S46;
    ROcp49_346 = ROcp49_345*C46-ROcp49_944*S46;
    ROcp49_746 = ROcp49_145*S46+ROcp49_744*C46;
    ROcp49_846 = ROcp49_245*S46+ROcp49_844*C46;
    ROcp49_946 = ROcp49_345*S46+ROcp49_944*C46;
    ROcp49_447 = ROcp49_445*C47+ROcp49_746*S47;
    ROcp49_547 = ROcp49_545*C47+ROcp49_846*S47;
    ROcp49_647 = ROcp49_645*C47+ROcp49_946*S47;
    ROcp49_747 = -(ROcp49_445*S47-ROcp49_746*C47);
    ROcp49_847 = -(ROcp49_545*S47-ROcp49_846*C47);
    ROcp49_947 = -(ROcp49_645*S47-ROcp49_946*C47);
    ROcp49_148 = ROcp49_146*C48+ROcp49_447*S48;
    ROcp49_248 = ROcp49_246*C48+ROcp49_547*S48;
    ROcp49_348 = ROcp49_346*C48+ROcp49_647*S48;
    ROcp49_448 = -(ROcp49_146*S48-ROcp49_447*C48);
    ROcp49_548 = -(ROcp49_246*S48-ROcp49_547*C48);
    ROcp49_648 = -(ROcp49_346*S48-ROcp49_647*C48);
    ROcp49_149 = ROcp49_148*C49-ROcp49_747*S49;
    ROcp49_249 = ROcp49_248*C49-ROcp49_847*S49;
    ROcp49_349 = ROcp49_348*C49-ROcp49_947*S49;
    ROcp49_749 = ROcp49_148*S49+ROcp49_747*C49;
    ROcp49_849 = ROcp49_248*S49+ROcp49_847*C49;
    ROcp49_949 = ROcp49_348*S49+ROcp49_947*C49;
    ROcp49_150 = ROcp49_149*C50+ROcp49_448*S50;
    ROcp49_250 = ROcp49_249*C50+ROcp49_548*S50;
    ROcp49_350 = ROcp49_349*C50+ROcp49_648*S50;
    ROcp49_450 = -(ROcp49_149*S50-ROcp49_448*C50);
    ROcp49_550 = -(ROcp49_249*S50-ROcp49_548*C50);
    ROcp49_650 = -(ROcp49_349*S50-ROcp49_648*C50);
    RLcp49_144 = ROcp49_134*s.dpt(1,24)+ROcp49_434*s.dpt(2,24)+ROcp49_733*s.dpt(3,24);
    RLcp49_244 = ROcp49_234*s.dpt(1,24)+ROcp49_534*s.dpt(2,24)+ROcp49_833*s.dpt(3,24);
    RLcp49_344 = ROcp49_334*s.dpt(1,24)+ROcp49_634*s.dpt(2,24)+ROcp49_933*s.dpt(3,24);
    ORcp49_144 = OMcp49_234*RLcp49_344-OMcp49_334*RLcp49_244;
    ORcp49_244 = -(OMcp49_134*RLcp49_344-OMcp49_334*RLcp49_144);
    ORcp49_344 = OMcp49_134*RLcp49_244-OMcp49_234*RLcp49_144;
    OMcp49_146 = OMcp49_134+ROcp49_445*qd(46);
    OMcp49_246 = OMcp49_234+ROcp49_545*qd(46);
    OMcp49_346 = OMcp49_334+ROcp49_645*qd(46);
    OPcp49_146 = OPcp49_134+ROcp49_445*qdd(46)+qd(46)*(OMcp49_234*ROcp49_645-OMcp49_334*ROcp49_545);
    OPcp49_246 = OPcp49_234+ROcp49_545*qdd(46)-qd(46)*(OMcp49_134*ROcp49_645-OMcp49_334*ROcp49_445);
    OPcp49_346 = OPcp49_334+ROcp49_645*qdd(46)+qd(46)*(OMcp49_134*ROcp49_545-OMcp49_234*ROcp49_445);
    RLcp49_147 = ROcp49_445*s.dpt(2,41)+ROcp49_746*s.dpt(3,41);
    RLcp49_247 = ROcp49_545*s.dpt(2,41)+ROcp49_846*s.dpt(3,41);
    RLcp49_347 = ROcp49_645*s.dpt(2,41)+ROcp49_946*s.dpt(3,41);
    OMcp49_147 = OMcp49_146+ROcp49_146*qd(47);
    OMcp49_247 = OMcp49_246+ROcp49_246*qd(47);
    OMcp49_347 = OMcp49_346+ROcp49_346*qd(47);
    ORcp49_147 = OMcp49_246*RLcp49_347-OMcp49_346*RLcp49_247;
    ORcp49_247 = -(OMcp49_146*RLcp49_347-OMcp49_346*RLcp49_147);
    ORcp49_347 = OMcp49_146*RLcp49_247-OMcp49_246*RLcp49_147;
    OPcp49_147 = OPcp49_146+ROcp49_146*qdd(47)+qd(47)*(OMcp49_246*ROcp49_346-OMcp49_346*ROcp49_246);
    OPcp49_247 = OPcp49_246+ROcp49_246*qdd(47)-qd(47)*(OMcp49_146*ROcp49_346-OMcp49_346*ROcp49_146);
    OPcp49_347 = OPcp49_346+ROcp49_346*qdd(47)+qd(47)*(OMcp49_146*ROcp49_246-OMcp49_246*ROcp49_146);
    RLcp49_148 = ROcp49_747*s.dpt(3,43);
    RLcp49_248 = ROcp49_847*s.dpt(3,43);
    RLcp49_348 = ROcp49_947*s.dpt(3,43);
    OMcp49_148 = OMcp49_147+ROcp49_747*qd(48);
    OMcp49_248 = OMcp49_247+ROcp49_847*qd(48);
    OMcp49_348 = OMcp49_347+ROcp49_947*qd(48);
    ORcp49_148 = OMcp49_247*RLcp49_348-OMcp49_347*RLcp49_248;
    ORcp49_248 = -(OMcp49_147*RLcp49_348-OMcp49_347*RLcp49_148);
    ORcp49_348 = OMcp49_147*RLcp49_248-OMcp49_247*RLcp49_148;
    OPcp49_148 = OPcp49_147+ROcp49_747*qdd(48)+qd(48)*(OMcp49_247*ROcp49_947-OMcp49_347*ROcp49_847);
    OPcp49_248 = OPcp49_247+ROcp49_847*qdd(48)-qd(48)*(OMcp49_147*ROcp49_947-OMcp49_347*ROcp49_747);
    OPcp49_348 = OPcp49_347+ROcp49_947*qdd(48)+qd(48)*(OMcp49_147*ROcp49_847-OMcp49_247*ROcp49_747);
    RLcp49_149 = ROcp49_148*s.dpt(1,46)+ROcp49_747*s.dpt(3,46);
    RLcp49_249 = ROcp49_248*s.dpt(1,46)+ROcp49_847*s.dpt(3,46);
    RLcp49_349 = ROcp49_348*s.dpt(1,46)+ROcp49_947*s.dpt(3,46);
    OMcp49_149 = OMcp49_148+ROcp49_448*qd(49);
    OMcp49_249 = OMcp49_248+ROcp49_548*qd(49);
    OMcp49_349 = OMcp49_348+ROcp49_648*qd(49);
    ORcp49_149 = OMcp49_248*RLcp49_349-OMcp49_348*RLcp49_249;
    ORcp49_249 = -(OMcp49_148*RLcp49_349-OMcp49_348*RLcp49_149);
    ORcp49_349 = OMcp49_148*RLcp49_249-OMcp49_248*RLcp49_149;
    OPcp49_149 = OPcp49_148+ROcp49_448*qdd(49)+qd(49)*(OMcp49_248*ROcp49_648-OMcp49_348*ROcp49_548);
    OPcp49_249 = OPcp49_248+ROcp49_548*qdd(49)-qd(49)*(OMcp49_148*ROcp49_648-OMcp49_348*ROcp49_448);
    OPcp49_349 = OPcp49_348+ROcp49_648*qdd(49)+qd(49)*(OMcp49_148*ROcp49_548-OMcp49_248*ROcp49_448);
    RLcp49_150 = ROcp49_149*s.dpt(1,48)+ROcp49_749*s.dpt(3,48);
    RLcp49_250 = ROcp49_249*s.dpt(1,48)+ROcp49_849*s.dpt(3,48);
    RLcp49_350 = ROcp49_349*s.dpt(1,48)+ROcp49_949*s.dpt(3,48);
    POcp49_150 = RLcp49_131+RLcp49_134+RLcp49_144+RLcp49_147+RLcp49_148+RLcp49_149+RLcp49_150+q(1);
    POcp49_250 = RLcp49_231+RLcp49_234+RLcp49_244+RLcp49_247+RLcp49_248+RLcp49_249+RLcp49_250+q(2);
    POcp49_350 = RLcp49_331+RLcp49_334+RLcp49_344+RLcp49_347+RLcp49_348+RLcp49_349+RLcp49_350+q(3);
    OMcp49_150 = OMcp49_149+ROcp49_749*qd(50);
    OMcp49_250 = OMcp49_249+ROcp49_849*qd(50);
    OMcp49_350 = OMcp49_349+ROcp49_949*qd(50);
    ORcp49_150 = OMcp49_249*RLcp49_350-OMcp49_349*RLcp49_250;
    ORcp49_250 = -(OMcp49_149*RLcp49_350-OMcp49_349*RLcp49_150);
    ORcp49_350 = OMcp49_149*RLcp49_250-OMcp49_249*RLcp49_150;
    VIcp49_150 = ORcp49_131+ORcp49_134+ORcp49_144+ORcp49_147+ORcp49_148+ORcp49_149+ORcp49_150+qd(1);
    VIcp49_250 = ORcp49_231+ORcp49_234+ORcp49_244+ORcp49_247+ORcp49_248+ORcp49_249+ORcp49_250+qd(2);
    VIcp49_350 = ORcp49_331+ORcp49_334+ORcp49_344+ORcp49_347+ORcp49_348+ORcp49_349+ORcp49_350+qd(3);
    OPcp49_150 = OPcp49_149+ROcp49_749*qdd(50)+qd(50)*(OMcp49_249*ROcp49_949-OMcp49_349*ROcp49_849);
    OPcp49_250 = OPcp49_249+ROcp49_849*qdd(50)-qd(50)*(OMcp49_149*ROcp49_949-OMcp49_349*ROcp49_749);
    OPcp49_350 = OPcp49_349+ROcp49_949*qdd(50)+qd(50)*(OMcp49_149*ROcp49_849-OMcp49_249*ROcp49_749);
    ACcp49_150 = qdd(1)+OMcp49_233*ORcp49_334+OMcp49_234*ORcp49_344+OMcp49_246*ORcp49_347+OMcp49_247*ORcp49_348+OMcp49_248*ORcp49_349+OMcp49_249*...
 ORcp49_350+OMcp49_26*ORcp49_331-OMcp49_333*ORcp49_234-OMcp49_334*ORcp49_244-OMcp49_346*ORcp49_247-OMcp49_347*ORcp49_248-OMcp49_348*ORcp49_249-...
 OMcp49_349*ORcp49_250-OMcp49_36*ORcp49_231+OPcp49_233*RLcp49_334+OPcp49_234*RLcp49_344+OPcp49_246*RLcp49_347+OPcp49_247*RLcp49_348+OPcp49_248*...
 RLcp49_349+OPcp49_249*RLcp49_350+OPcp49_26*RLcp49_331-OPcp49_333*RLcp49_234-OPcp49_334*RLcp49_244-OPcp49_346*RLcp49_247-OPcp49_347*RLcp49_248-...
 OPcp49_348*RLcp49_249-OPcp49_349*RLcp49_250-OPcp49_36*RLcp49_231;
    ACcp49_250 = qdd(2)-OMcp49_133*ORcp49_334-OMcp49_134*ORcp49_344-OMcp49_146*ORcp49_347-OMcp49_147*ORcp49_348-OMcp49_148*ORcp49_349-OMcp49_149*...
 ORcp49_350-OMcp49_16*ORcp49_331+OMcp49_333*ORcp49_134+OMcp49_334*ORcp49_144+OMcp49_346*ORcp49_147+OMcp49_347*ORcp49_148+OMcp49_348*ORcp49_149+...
 OMcp49_349*ORcp49_150+OMcp49_36*ORcp49_131-OPcp49_133*RLcp49_334-OPcp49_134*RLcp49_344-OPcp49_146*RLcp49_347-OPcp49_147*RLcp49_348-OPcp49_148*...
 RLcp49_349-OPcp49_149*RLcp49_350-OPcp49_16*RLcp49_331+OPcp49_333*RLcp49_134+OPcp49_334*RLcp49_144+OPcp49_346*RLcp49_147+OPcp49_347*RLcp49_148+...
 OPcp49_348*RLcp49_149+OPcp49_349*RLcp49_150+OPcp49_36*RLcp49_131;
    ACcp49_350 = qdd(3)+OMcp49_133*ORcp49_234+OMcp49_134*ORcp49_244+OMcp49_146*ORcp49_247+OMcp49_147*ORcp49_248+OMcp49_148*ORcp49_249+OMcp49_149*...
 ORcp49_250+OMcp49_16*ORcp49_231-OMcp49_233*ORcp49_134-OMcp49_234*ORcp49_144-OMcp49_246*ORcp49_147-OMcp49_247*ORcp49_148-OMcp49_248*ORcp49_149-...
 OMcp49_249*ORcp49_150-OMcp49_26*ORcp49_131+OPcp49_133*RLcp49_234+OPcp49_134*RLcp49_244+OPcp49_146*RLcp49_247+OPcp49_147*RLcp49_248+OPcp49_148*...
 RLcp49_249+OPcp49_149*RLcp49_250+OPcp49_16*RLcp49_231-OPcp49_233*RLcp49_134-OPcp49_234*RLcp49_144-OPcp49_246*RLcp49_147-OPcp49_247*RLcp49_148-...
 OPcp49_248*RLcp49_149-OPcp49_249*RLcp49_150-OPcp49_26*RLcp49_131;

% = = Block_1_0_0_50_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp49_150;
    sens.P(2) = POcp49_250;
    sens.P(3) = POcp49_350;
    sens.R(1,1) = ROcp49_150;
    sens.R(1,2) = ROcp49_250;
    sens.R(1,3) = ROcp49_350;
    sens.R(2,1) = ROcp49_450;
    sens.R(2,2) = ROcp49_550;
    sens.R(2,3) = ROcp49_650;
    sens.R(3,1) = ROcp49_749;
    sens.R(3,2) = ROcp49_849;
    sens.R(3,3) = ROcp49_949;
    sens.V(1) = VIcp49_150;
    sens.V(2) = VIcp49_250;
    sens.V(3) = VIcp49_350;
    sens.OM(1) = OMcp49_150;
    sens.OM(2) = OMcp49_250;
    sens.OM(3) = OMcp49_350;
    sens.A(1) = ACcp49_150;
    sens.A(2) = ACcp49_250;
    sens.A(3) = ACcp49_350;
    sens.OMP(1) = OPcp49_150;
    sens.OMP(2) = OPcp49_250;
    sens.OMP(3) = OPcp49_350;
 
% 
case 51, 


% = = Block_1_0_0_51_0_1 = = 
 
% Sensor Kinematics 


    ROcp50_25 = S4*S5;
    ROcp50_35 = -C4*S5;
    ROcp50_85 = -S4*C5;
    ROcp50_95 = C4*C5;
    ROcp50_16 = C5*C6;
    ROcp50_26 = ROcp50_25*C6+C4*S6;
    ROcp50_36 = ROcp50_35*C6+S4*S6;
    ROcp50_46 = -C5*S6;
    ROcp50_56 = -(ROcp50_25*S6-C4*C6);
    ROcp50_66 = -(ROcp50_35*S6-S4*C6);
    OMcp50_25 = qd(5)*C4;
    OMcp50_35 = qd(5)*S4;
    OMcp50_16 = qd(4)+qd(6)*S5;
    OMcp50_26 = OMcp50_25+ROcp50_85*qd(6);
    OMcp50_36 = OMcp50_35+ROcp50_95*qd(6);
    OPcp50_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp50_26 = ROcp50_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp50_35*S5-ROcp50_95*qd(4));
    OPcp50_36 = ROcp50_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp50_25*S5-ROcp50_85*qd(4));

% = = Block_1_0_0_51_0_4 = = 
 
% Sensor Kinematics 


    ROcp50_131 = ROcp50_16*C31-S31*S5;
    ROcp50_231 = ROcp50_26*C31-ROcp50_85*S31;
    ROcp50_331 = ROcp50_36*C31-ROcp50_95*S31;
    ROcp50_731 = ROcp50_16*S31+C31*S5;
    ROcp50_831 = ROcp50_26*S31+ROcp50_85*C31;
    ROcp50_931 = ROcp50_36*S31+ROcp50_95*C31;
    ROcp50_432 = ROcp50_46*C32+ROcp50_731*S32;
    ROcp50_532 = ROcp50_56*C32+ROcp50_831*S32;
    ROcp50_632 = ROcp50_66*C32+ROcp50_931*S32;
    ROcp50_732 = -(ROcp50_46*S32-ROcp50_731*C32);
    ROcp50_832 = -(ROcp50_56*S32-ROcp50_831*C32);
    ROcp50_932 = -(ROcp50_66*S32-ROcp50_931*C32);
    ROcp50_133 = ROcp50_131*C33-ROcp50_732*S33;
    ROcp50_233 = ROcp50_231*C33-ROcp50_832*S33;
    ROcp50_333 = ROcp50_331*C33-ROcp50_932*S33;
    ROcp50_733 = ROcp50_131*S33+ROcp50_732*C33;
    ROcp50_833 = ROcp50_231*S33+ROcp50_832*C33;
    ROcp50_933 = ROcp50_331*S33+ROcp50_932*C33;
    ROcp50_134 = ROcp50_133*C34+ROcp50_432*S34;
    ROcp50_234 = ROcp50_233*C34+ROcp50_532*S34;
    ROcp50_334 = ROcp50_333*C34+ROcp50_632*S34;
    ROcp50_434 = -(ROcp50_133*S34-ROcp50_432*C34);
    ROcp50_534 = -(ROcp50_233*S34-ROcp50_532*C34);
    ROcp50_634 = -(ROcp50_333*S34-ROcp50_632*C34);
    RLcp50_131 = ROcp50_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp50_231 = ROcp50_26*s.dpt(1,3)+ROcp50_85*s.dpt(3,3);
    RLcp50_331 = ROcp50_36*s.dpt(1,3)+ROcp50_95*s.dpt(3,3);
    OMcp50_131 = OMcp50_16+ROcp50_46*qd(31);
    OMcp50_231 = OMcp50_26+ROcp50_56*qd(31);
    OMcp50_331 = OMcp50_36+ROcp50_66*qd(31);
    ORcp50_131 = OMcp50_26*RLcp50_331-OMcp50_36*RLcp50_231;
    ORcp50_231 = -(OMcp50_16*RLcp50_331-OMcp50_36*RLcp50_131);
    ORcp50_331 = OMcp50_16*RLcp50_231-OMcp50_26*RLcp50_131;
    OMcp50_132 = OMcp50_131+ROcp50_131*qd(32);
    OMcp50_232 = OMcp50_231+ROcp50_231*qd(32);
    OMcp50_332 = OMcp50_331+ROcp50_331*qd(32);
    OMcp50_133 = OMcp50_132+ROcp50_432*qd(33);
    OMcp50_233 = OMcp50_232+ROcp50_532*qd(33);
    OMcp50_333 = OMcp50_332+ROcp50_632*qd(33);
    OPcp50_133 = OPcp50_16+ROcp50_131*qdd(32)+ROcp50_432*qdd(33)+ROcp50_46*qdd(31)+qd(31)*(OMcp50_26*ROcp50_66-OMcp50_36*ROcp50_56)+qd(32)*(...
 OMcp50_231*ROcp50_331-OMcp50_331*ROcp50_231)+qd(33)*(OMcp50_232*ROcp50_632-OMcp50_332*ROcp50_532);
    OPcp50_233 = OPcp50_26+ROcp50_231*qdd(32)+ROcp50_532*qdd(33)+ROcp50_56*qdd(31)-qd(31)*(OMcp50_16*ROcp50_66-OMcp50_36*ROcp50_46)-qd(32)*(...
 OMcp50_131*ROcp50_331-OMcp50_331*ROcp50_131)-qd(33)*(OMcp50_132*ROcp50_632-OMcp50_332*ROcp50_432);
    OPcp50_333 = OPcp50_36+ROcp50_331*qdd(32)+ROcp50_632*qdd(33)+ROcp50_66*qdd(31)+qd(31)*(OMcp50_16*ROcp50_56-OMcp50_26*ROcp50_46)+qd(32)*(...
 OMcp50_131*ROcp50_231-OMcp50_231*ROcp50_131)+qd(33)*(OMcp50_132*ROcp50_532-OMcp50_232*ROcp50_432);
    RLcp50_134 = ROcp50_733*s.dpt(3,21);
    RLcp50_234 = ROcp50_833*s.dpt(3,21);
    RLcp50_334 = ROcp50_933*s.dpt(3,21);
    OMcp50_134 = OMcp50_133+ROcp50_733*qd(34);
    OMcp50_234 = OMcp50_233+ROcp50_833*qd(34);
    OMcp50_334 = OMcp50_333+ROcp50_933*qd(34);
    ORcp50_134 = OMcp50_233*RLcp50_334-OMcp50_333*RLcp50_234;
    ORcp50_234 = -(OMcp50_133*RLcp50_334-OMcp50_333*RLcp50_134);
    ORcp50_334 = OMcp50_133*RLcp50_234-OMcp50_233*RLcp50_134;
    OPcp50_134 = OPcp50_133+ROcp50_733*qdd(34)+qd(34)*(OMcp50_233*ROcp50_933-OMcp50_333*ROcp50_833);
    OPcp50_234 = OPcp50_233+ROcp50_833*qdd(34)-qd(34)*(OMcp50_133*ROcp50_933-OMcp50_333*ROcp50_733);
    OPcp50_334 = OPcp50_333+ROcp50_933*qdd(34)+qd(34)*(OMcp50_133*ROcp50_833-OMcp50_233*ROcp50_733);

% = = Block_1_0_0_51_0_6 = = 
 
% Sensor Kinematics 


    ROcp50_444 = ROcp50_434*C44+ROcp50_733*S44;
    ROcp50_544 = ROcp50_534*C44+ROcp50_833*S44;
    ROcp50_644 = ROcp50_634*C44+ROcp50_933*S44;
    ROcp50_744 = -(ROcp50_434*S44-ROcp50_733*C44);
    ROcp50_844 = -(ROcp50_534*S44-ROcp50_833*C44);
    ROcp50_944 = -(ROcp50_634*S44-ROcp50_933*C44);
    ROcp50_145 = ROcp50_134*C45+ROcp50_444*S45;
    ROcp50_245 = ROcp50_234*C45+ROcp50_544*S45;
    ROcp50_345 = ROcp50_334*C45+ROcp50_644*S45;
    ROcp50_445 = -(ROcp50_134*S45-ROcp50_444*C45);
    ROcp50_545 = -(ROcp50_234*S45-ROcp50_544*C45);
    ROcp50_645 = -(ROcp50_334*S45-ROcp50_644*C45);
    ROcp50_146 = ROcp50_145*C46-ROcp50_744*S46;
    ROcp50_246 = ROcp50_245*C46-ROcp50_844*S46;
    ROcp50_346 = ROcp50_345*C46-ROcp50_944*S46;
    ROcp50_746 = ROcp50_145*S46+ROcp50_744*C46;
    ROcp50_846 = ROcp50_245*S46+ROcp50_844*C46;
    ROcp50_946 = ROcp50_345*S46+ROcp50_944*C46;
    ROcp50_447 = ROcp50_445*C47+ROcp50_746*S47;
    ROcp50_547 = ROcp50_545*C47+ROcp50_846*S47;
    ROcp50_647 = ROcp50_645*C47+ROcp50_946*S47;
    ROcp50_747 = -(ROcp50_445*S47-ROcp50_746*C47);
    ROcp50_847 = -(ROcp50_545*S47-ROcp50_846*C47);
    ROcp50_947 = -(ROcp50_645*S47-ROcp50_946*C47);
    ROcp50_148 = ROcp50_146*C48+ROcp50_447*S48;
    ROcp50_248 = ROcp50_246*C48+ROcp50_547*S48;
    ROcp50_348 = ROcp50_346*C48+ROcp50_647*S48;
    ROcp50_448 = -(ROcp50_146*S48-ROcp50_447*C48);
    ROcp50_548 = -(ROcp50_246*S48-ROcp50_547*C48);
    ROcp50_648 = -(ROcp50_346*S48-ROcp50_647*C48);
    ROcp50_149 = ROcp50_148*C49-ROcp50_747*S49;
    ROcp50_249 = ROcp50_248*C49-ROcp50_847*S49;
    ROcp50_349 = ROcp50_348*C49-ROcp50_947*S49;
    ROcp50_749 = ROcp50_148*S49+ROcp50_747*C49;
    ROcp50_849 = ROcp50_248*S49+ROcp50_847*C49;
    ROcp50_949 = ROcp50_348*S49+ROcp50_947*C49;
    ROcp50_150 = ROcp50_149*C50+ROcp50_448*S50;
    ROcp50_250 = ROcp50_249*C50+ROcp50_548*S50;
    ROcp50_350 = ROcp50_349*C50+ROcp50_648*S50;
    ROcp50_450 = -(ROcp50_149*S50-ROcp50_448*C50);
    ROcp50_550 = -(ROcp50_249*S50-ROcp50_548*C50);
    ROcp50_650 = -(ROcp50_349*S50-ROcp50_648*C50);
    ROcp50_151 = ROcp50_150*C51-ROcp50_749*S51;
    ROcp50_251 = ROcp50_250*C51-ROcp50_849*S51;
    ROcp50_351 = ROcp50_350*C51-ROcp50_949*S51;
    ROcp50_751 = ROcp50_150*S51+ROcp50_749*C51;
    ROcp50_851 = ROcp50_250*S51+ROcp50_849*C51;
    ROcp50_951 = ROcp50_350*S51+ROcp50_949*C51;
    RLcp50_144 = ROcp50_134*s.dpt(1,24)+ROcp50_434*s.dpt(2,24)+ROcp50_733*s.dpt(3,24);
    RLcp50_244 = ROcp50_234*s.dpt(1,24)+ROcp50_534*s.dpt(2,24)+ROcp50_833*s.dpt(3,24);
    RLcp50_344 = ROcp50_334*s.dpt(1,24)+ROcp50_634*s.dpt(2,24)+ROcp50_933*s.dpt(3,24);
    ORcp50_144 = OMcp50_234*RLcp50_344-OMcp50_334*RLcp50_244;
    ORcp50_244 = -(OMcp50_134*RLcp50_344-OMcp50_334*RLcp50_144);
    ORcp50_344 = OMcp50_134*RLcp50_244-OMcp50_234*RLcp50_144;
    OMcp50_146 = OMcp50_134+ROcp50_445*qd(46);
    OMcp50_246 = OMcp50_234+ROcp50_545*qd(46);
    OMcp50_346 = OMcp50_334+ROcp50_645*qd(46);
    OPcp50_146 = OPcp50_134+ROcp50_445*qdd(46)+qd(46)*(OMcp50_234*ROcp50_645-OMcp50_334*ROcp50_545);
    OPcp50_246 = OPcp50_234+ROcp50_545*qdd(46)-qd(46)*(OMcp50_134*ROcp50_645-OMcp50_334*ROcp50_445);
    OPcp50_346 = OPcp50_334+ROcp50_645*qdd(46)+qd(46)*(OMcp50_134*ROcp50_545-OMcp50_234*ROcp50_445);
    RLcp50_147 = ROcp50_445*s.dpt(2,41)+ROcp50_746*s.dpt(3,41);
    RLcp50_247 = ROcp50_545*s.dpt(2,41)+ROcp50_846*s.dpt(3,41);
    RLcp50_347 = ROcp50_645*s.dpt(2,41)+ROcp50_946*s.dpt(3,41);
    OMcp50_147 = OMcp50_146+ROcp50_146*qd(47);
    OMcp50_247 = OMcp50_246+ROcp50_246*qd(47);
    OMcp50_347 = OMcp50_346+ROcp50_346*qd(47);
    ORcp50_147 = OMcp50_246*RLcp50_347-OMcp50_346*RLcp50_247;
    ORcp50_247 = -(OMcp50_146*RLcp50_347-OMcp50_346*RLcp50_147);
    ORcp50_347 = OMcp50_146*RLcp50_247-OMcp50_246*RLcp50_147;
    OPcp50_147 = OPcp50_146+ROcp50_146*qdd(47)+qd(47)*(OMcp50_246*ROcp50_346-OMcp50_346*ROcp50_246);
    OPcp50_247 = OPcp50_246+ROcp50_246*qdd(47)-qd(47)*(OMcp50_146*ROcp50_346-OMcp50_346*ROcp50_146);
    OPcp50_347 = OPcp50_346+ROcp50_346*qdd(47)+qd(47)*(OMcp50_146*ROcp50_246-OMcp50_246*ROcp50_146);
    RLcp50_148 = ROcp50_747*s.dpt(3,43);
    RLcp50_248 = ROcp50_847*s.dpt(3,43);
    RLcp50_348 = ROcp50_947*s.dpt(3,43);
    OMcp50_148 = OMcp50_147+ROcp50_747*qd(48);
    OMcp50_248 = OMcp50_247+ROcp50_847*qd(48);
    OMcp50_348 = OMcp50_347+ROcp50_947*qd(48);
    ORcp50_148 = OMcp50_247*RLcp50_348-OMcp50_347*RLcp50_248;
    ORcp50_248 = -(OMcp50_147*RLcp50_348-OMcp50_347*RLcp50_148);
    ORcp50_348 = OMcp50_147*RLcp50_248-OMcp50_247*RLcp50_148;
    OPcp50_148 = OPcp50_147+ROcp50_747*qdd(48)+qd(48)*(OMcp50_247*ROcp50_947-OMcp50_347*ROcp50_847);
    OPcp50_248 = OPcp50_247+ROcp50_847*qdd(48)-qd(48)*(OMcp50_147*ROcp50_947-OMcp50_347*ROcp50_747);
    OPcp50_348 = OPcp50_347+ROcp50_947*qdd(48)+qd(48)*(OMcp50_147*ROcp50_847-OMcp50_247*ROcp50_747);
    RLcp50_149 = ROcp50_148*s.dpt(1,46)+ROcp50_747*s.dpt(3,46);
    RLcp50_249 = ROcp50_248*s.dpt(1,46)+ROcp50_847*s.dpt(3,46);
    RLcp50_349 = ROcp50_348*s.dpt(1,46)+ROcp50_947*s.dpt(3,46);
    OMcp50_149 = OMcp50_148+ROcp50_448*qd(49);
    OMcp50_249 = OMcp50_248+ROcp50_548*qd(49);
    OMcp50_349 = OMcp50_348+ROcp50_648*qd(49);
    ORcp50_149 = OMcp50_248*RLcp50_349-OMcp50_348*RLcp50_249;
    ORcp50_249 = -(OMcp50_148*RLcp50_349-OMcp50_348*RLcp50_149);
    ORcp50_349 = OMcp50_148*RLcp50_249-OMcp50_248*RLcp50_149;
    OPcp50_149 = OPcp50_148+ROcp50_448*qdd(49)+qd(49)*(OMcp50_248*ROcp50_648-OMcp50_348*ROcp50_548);
    OPcp50_249 = OPcp50_248+ROcp50_548*qdd(49)-qd(49)*(OMcp50_148*ROcp50_648-OMcp50_348*ROcp50_448);
    OPcp50_349 = OPcp50_348+ROcp50_648*qdd(49)+qd(49)*(OMcp50_148*ROcp50_548-OMcp50_248*ROcp50_448);
    RLcp50_150 = ROcp50_149*s.dpt(1,48)+ROcp50_749*s.dpt(3,48);
    RLcp50_250 = ROcp50_249*s.dpt(1,48)+ROcp50_849*s.dpt(3,48);
    RLcp50_350 = ROcp50_349*s.dpt(1,48)+ROcp50_949*s.dpt(3,48);
    POcp50_150 = RLcp50_131+RLcp50_134+RLcp50_144+RLcp50_147+RLcp50_148+RLcp50_149+RLcp50_150+q(1);
    POcp50_250 = RLcp50_231+RLcp50_234+RLcp50_244+RLcp50_247+RLcp50_248+RLcp50_249+RLcp50_250+q(2);
    POcp50_350 = RLcp50_331+RLcp50_334+RLcp50_344+RLcp50_347+RLcp50_348+RLcp50_349+RLcp50_350+q(3);
    OMcp50_150 = OMcp50_149+ROcp50_749*qd(50);
    OMcp50_250 = OMcp50_249+ROcp50_849*qd(50);
    OMcp50_350 = OMcp50_349+ROcp50_949*qd(50);
    ORcp50_150 = OMcp50_249*RLcp50_350-OMcp50_349*RLcp50_250;
    ORcp50_250 = -(OMcp50_149*RLcp50_350-OMcp50_349*RLcp50_150);
    ORcp50_350 = OMcp50_149*RLcp50_250-OMcp50_249*RLcp50_150;
    VIcp50_150 = ORcp50_131+ORcp50_134+ORcp50_144+ORcp50_147+ORcp50_148+ORcp50_149+ORcp50_150+qd(1);
    VIcp50_250 = ORcp50_231+ORcp50_234+ORcp50_244+ORcp50_247+ORcp50_248+ORcp50_249+ORcp50_250+qd(2);
    VIcp50_350 = ORcp50_331+ORcp50_334+ORcp50_344+ORcp50_347+ORcp50_348+ORcp50_349+ORcp50_350+qd(3);
    ACcp50_150 = qdd(1)+OMcp50_233*ORcp50_334+OMcp50_234*ORcp50_344+OMcp50_246*ORcp50_347+OMcp50_247*ORcp50_348+OMcp50_248*ORcp50_349+OMcp50_249*...
 ORcp50_350+OMcp50_26*ORcp50_331-OMcp50_333*ORcp50_234-OMcp50_334*ORcp50_244-OMcp50_346*ORcp50_247-OMcp50_347*ORcp50_248-OMcp50_348*ORcp50_249-...
 OMcp50_349*ORcp50_250-OMcp50_36*ORcp50_231+OPcp50_233*RLcp50_334+OPcp50_234*RLcp50_344+OPcp50_246*RLcp50_347+OPcp50_247*RLcp50_348+OPcp50_248*...
 RLcp50_349+OPcp50_249*RLcp50_350+OPcp50_26*RLcp50_331-OPcp50_333*RLcp50_234-OPcp50_334*RLcp50_244-OPcp50_346*RLcp50_247-OPcp50_347*RLcp50_248-...
 OPcp50_348*RLcp50_249-OPcp50_349*RLcp50_250-OPcp50_36*RLcp50_231;
    ACcp50_250 = qdd(2)-OMcp50_133*ORcp50_334-OMcp50_134*ORcp50_344-OMcp50_146*ORcp50_347-OMcp50_147*ORcp50_348-OMcp50_148*ORcp50_349-OMcp50_149*...
 ORcp50_350-OMcp50_16*ORcp50_331+OMcp50_333*ORcp50_134+OMcp50_334*ORcp50_144+OMcp50_346*ORcp50_147+OMcp50_347*ORcp50_148+OMcp50_348*ORcp50_149+...
 OMcp50_349*ORcp50_150+OMcp50_36*ORcp50_131-OPcp50_133*RLcp50_334-OPcp50_134*RLcp50_344-OPcp50_146*RLcp50_347-OPcp50_147*RLcp50_348-OPcp50_148*...
 RLcp50_349-OPcp50_149*RLcp50_350-OPcp50_16*RLcp50_331+OPcp50_333*RLcp50_134+OPcp50_334*RLcp50_144+OPcp50_346*RLcp50_147+OPcp50_347*RLcp50_148+...
 OPcp50_348*RLcp50_149+OPcp50_349*RLcp50_150+OPcp50_36*RLcp50_131;
    ACcp50_350 = qdd(3)+OMcp50_133*ORcp50_234+OMcp50_134*ORcp50_244+OMcp50_146*ORcp50_247+OMcp50_147*ORcp50_248+OMcp50_148*ORcp50_249+OMcp50_149*...
 ORcp50_250+OMcp50_16*ORcp50_231-OMcp50_233*ORcp50_134-OMcp50_234*ORcp50_144-OMcp50_246*ORcp50_147-OMcp50_247*ORcp50_148-OMcp50_248*ORcp50_149-...
 OMcp50_249*ORcp50_150-OMcp50_26*ORcp50_131+OPcp50_133*RLcp50_234+OPcp50_134*RLcp50_244+OPcp50_146*RLcp50_247+OPcp50_147*RLcp50_248+OPcp50_148*...
 RLcp50_249+OPcp50_149*RLcp50_250+OPcp50_16*RLcp50_231-OPcp50_233*RLcp50_134-OPcp50_234*RLcp50_144-OPcp50_246*RLcp50_147-OPcp50_247*RLcp50_148-...
 OPcp50_248*RLcp50_149-OPcp50_249*RLcp50_150-OPcp50_26*RLcp50_131;
    OMcp50_151 = OMcp50_150+ROcp50_450*qd(51);
    OMcp50_251 = OMcp50_250+ROcp50_550*qd(51);
    OMcp50_351 = OMcp50_350+ROcp50_650*qd(51);
    OPcp50_151 = OPcp50_149+ROcp50_450*qdd(51)+ROcp50_749*qdd(50)+qd(50)*(OMcp50_249*ROcp50_949-OMcp50_349*ROcp50_849)+qd(51)*(OMcp50_250*...
 ROcp50_650-OMcp50_350*ROcp50_550);
    OPcp50_251 = OPcp50_249+ROcp50_550*qdd(51)+ROcp50_849*qdd(50)-qd(50)*(OMcp50_149*ROcp50_949-OMcp50_349*ROcp50_749)-qd(51)*(OMcp50_150*...
 ROcp50_650-OMcp50_350*ROcp50_450);
    OPcp50_351 = OPcp50_349+ROcp50_650*qdd(51)+ROcp50_949*qdd(50)+qd(50)*(OMcp50_149*ROcp50_849-OMcp50_249*ROcp50_749)+qd(51)*(OMcp50_150*...
 ROcp50_550-OMcp50_250*ROcp50_450);

% = = Block_1_0_0_51_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp50_150;
    sens.P(2) = POcp50_250;
    sens.P(3) = POcp50_350;
    sens.R(1,1) = ROcp50_151;
    sens.R(1,2) = ROcp50_251;
    sens.R(1,3) = ROcp50_351;
    sens.R(2,1) = ROcp50_450;
    sens.R(2,2) = ROcp50_550;
    sens.R(2,3) = ROcp50_650;
    sens.R(3,1) = ROcp50_751;
    sens.R(3,2) = ROcp50_851;
    sens.R(3,3) = ROcp50_951;
    sens.V(1) = VIcp50_150;
    sens.V(2) = VIcp50_250;
    sens.V(3) = VIcp50_350;
    sens.OM(1) = OMcp50_151;
    sens.OM(2) = OMcp50_251;
    sens.OM(3) = OMcp50_351;
    sens.A(1) = ACcp50_150;
    sens.A(2) = ACcp50_250;
    sens.A(3) = ACcp50_350;
    sens.OMP(1) = OPcp50_151;
    sens.OMP(2) = OPcp50_251;
    sens.OMP(3) = OPcp50_351;
 
% 
case 52, 


% = = Block_1_0_0_52_0_1 = = 
 
% Sensor Kinematics 


    ROcp51_25 = S4*S5;
    ROcp51_35 = -C4*S5;
    ROcp51_85 = -S4*C5;
    ROcp51_95 = C4*C5;
    ROcp51_16 = C5*C6;
    ROcp51_26 = ROcp51_25*C6+C4*S6;
    ROcp51_36 = ROcp51_35*C6+S4*S6;
    ROcp51_46 = -C5*S6;
    ROcp51_56 = -(ROcp51_25*S6-C4*C6);
    ROcp51_66 = -(ROcp51_35*S6-S4*C6);
    OMcp51_25 = qd(5)*C4;
    OMcp51_35 = qd(5)*S4;
    OMcp51_16 = qd(4)+qd(6)*S5;
    OMcp51_26 = OMcp51_25+ROcp51_85*qd(6);
    OMcp51_36 = OMcp51_35+ROcp51_95*qd(6);
    OPcp51_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp51_26 = ROcp51_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp51_35*S5-ROcp51_95*qd(4));
    OPcp51_36 = ROcp51_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp51_25*S5-ROcp51_85*qd(4));

% = = Block_1_0_0_52_0_4 = = 
 
% Sensor Kinematics 


    ROcp51_131 = ROcp51_16*C31-S31*S5;
    ROcp51_231 = ROcp51_26*C31-ROcp51_85*S31;
    ROcp51_331 = ROcp51_36*C31-ROcp51_95*S31;
    ROcp51_731 = ROcp51_16*S31+C31*S5;
    ROcp51_831 = ROcp51_26*S31+ROcp51_85*C31;
    ROcp51_931 = ROcp51_36*S31+ROcp51_95*C31;
    ROcp51_432 = ROcp51_46*C32+ROcp51_731*S32;
    ROcp51_532 = ROcp51_56*C32+ROcp51_831*S32;
    ROcp51_632 = ROcp51_66*C32+ROcp51_931*S32;
    ROcp51_732 = -(ROcp51_46*S32-ROcp51_731*C32);
    ROcp51_832 = -(ROcp51_56*S32-ROcp51_831*C32);
    ROcp51_932 = -(ROcp51_66*S32-ROcp51_931*C32);
    ROcp51_133 = ROcp51_131*C33-ROcp51_732*S33;
    ROcp51_233 = ROcp51_231*C33-ROcp51_832*S33;
    ROcp51_333 = ROcp51_331*C33-ROcp51_932*S33;
    ROcp51_733 = ROcp51_131*S33+ROcp51_732*C33;
    ROcp51_833 = ROcp51_231*S33+ROcp51_832*C33;
    ROcp51_933 = ROcp51_331*S33+ROcp51_932*C33;
    ROcp51_134 = ROcp51_133*C34+ROcp51_432*S34;
    ROcp51_234 = ROcp51_233*C34+ROcp51_532*S34;
    ROcp51_334 = ROcp51_333*C34+ROcp51_632*S34;
    ROcp51_434 = -(ROcp51_133*S34-ROcp51_432*C34);
    ROcp51_534 = -(ROcp51_233*S34-ROcp51_532*C34);
    ROcp51_634 = -(ROcp51_333*S34-ROcp51_632*C34);
    RLcp51_131 = ROcp51_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp51_231 = ROcp51_26*s.dpt(1,3)+ROcp51_85*s.dpt(3,3);
    RLcp51_331 = ROcp51_36*s.dpt(1,3)+ROcp51_95*s.dpt(3,3);
    OMcp51_131 = OMcp51_16+ROcp51_46*qd(31);
    OMcp51_231 = OMcp51_26+ROcp51_56*qd(31);
    OMcp51_331 = OMcp51_36+ROcp51_66*qd(31);
    ORcp51_131 = OMcp51_26*RLcp51_331-OMcp51_36*RLcp51_231;
    ORcp51_231 = -(OMcp51_16*RLcp51_331-OMcp51_36*RLcp51_131);
    ORcp51_331 = OMcp51_16*RLcp51_231-OMcp51_26*RLcp51_131;
    OMcp51_132 = OMcp51_131+ROcp51_131*qd(32);
    OMcp51_232 = OMcp51_231+ROcp51_231*qd(32);
    OMcp51_332 = OMcp51_331+ROcp51_331*qd(32);
    OMcp51_133 = OMcp51_132+ROcp51_432*qd(33);
    OMcp51_233 = OMcp51_232+ROcp51_532*qd(33);
    OMcp51_333 = OMcp51_332+ROcp51_632*qd(33);
    OPcp51_133 = OPcp51_16+ROcp51_131*qdd(32)+ROcp51_432*qdd(33)+ROcp51_46*qdd(31)+qd(31)*(OMcp51_26*ROcp51_66-OMcp51_36*ROcp51_56)+qd(32)*(...
 OMcp51_231*ROcp51_331-OMcp51_331*ROcp51_231)+qd(33)*(OMcp51_232*ROcp51_632-OMcp51_332*ROcp51_532);
    OPcp51_233 = OPcp51_26+ROcp51_231*qdd(32)+ROcp51_532*qdd(33)+ROcp51_56*qdd(31)-qd(31)*(OMcp51_16*ROcp51_66-OMcp51_36*ROcp51_46)-qd(32)*(...
 OMcp51_131*ROcp51_331-OMcp51_331*ROcp51_131)-qd(33)*(OMcp51_132*ROcp51_632-OMcp51_332*ROcp51_432);
    OPcp51_333 = OPcp51_36+ROcp51_331*qdd(32)+ROcp51_632*qdd(33)+ROcp51_66*qdd(31)+qd(31)*(OMcp51_16*ROcp51_56-OMcp51_26*ROcp51_46)+qd(32)*(...
 OMcp51_131*ROcp51_231-OMcp51_231*ROcp51_131)+qd(33)*(OMcp51_132*ROcp51_532-OMcp51_232*ROcp51_432);
    RLcp51_134 = ROcp51_733*s.dpt(3,21);
    RLcp51_234 = ROcp51_833*s.dpt(3,21);
    RLcp51_334 = ROcp51_933*s.dpt(3,21);
    OMcp51_134 = OMcp51_133+ROcp51_733*qd(34);
    OMcp51_234 = OMcp51_233+ROcp51_833*qd(34);
    OMcp51_334 = OMcp51_333+ROcp51_933*qd(34);
    ORcp51_134 = OMcp51_233*RLcp51_334-OMcp51_333*RLcp51_234;
    ORcp51_234 = -(OMcp51_133*RLcp51_334-OMcp51_333*RLcp51_134);
    ORcp51_334 = OMcp51_133*RLcp51_234-OMcp51_233*RLcp51_134;
    OPcp51_134 = OPcp51_133+ROcp51_733*qdd(34)+qd(34)*(OMcp51_233*ROcp51_933-OMcp51_333*ROcp51_833);
    OPcp51_234 = OPcp51_233+ROcp51_833*qdd(34)-qd(34)*(OMcp51_133*ROcp51_933-OMcp51_333*ROcp51_733);
    OPcp51_334 = OPcp51_333+ROcp51_933*qdd(34)+qd(34)*(OMcp51_133*ROcp51_833-OMcp51_233*ROcp51_733);

% = = Block_1_0_0_52_0_6 = = 
 
% Sensor Kinematics 


    ROcp51_444 = ROcp51_434*C44+ROcp51_733*S44;
    ROcp51_544 = ROcp51_534*C44+ROcp51_833*S44;
    ROcp51_644 = ROcp51_634*C44+ROcp51_933*S44;
    ROcp51_744 = -(ROcp51_434*S44-ROcp51_733*C44);
    ROcp51_844 = -(ROcp51_534*S44-ROcp51_833*C44);
    ROcp51_944 = -(ROcp51_634*S44-ROcp51_933*C44);
    ROcp51_145 = ROcp51_134*C45+ROcp51_444*S45;
    ROcp51_245 = ROcp51_234*C45+ROcp51_544*S45;
    ROcp51_345 = ROcp51_334*C45+ROcp51_644*S45;
    ROcp51_445 = -(ROcp51_134*S45-ROcp51_444*C45);
    ROcp51_545 = -(ROcp51_234*S45-ROcp51_544*C45);
    ROcp51_645 = -(ROcp51_334*S45-ROcp51_644*C45);
    ROcp51_146 = ROcp51_145*C46-ROcp51_744*S46;
    ROcp51_246 = ROcp51_245*C46-ROcp51_844*S46;
    ROcp51_346 = ROcp51_345*C46-ROcp51_944*S46;
    ROcp51_746 = ROcp51_145*S46+ROcp51_744*C46;
    ROcp51_846 = ROcp51_245*S46+ROcp51_844*C46;
    ROcp51_946 = ROcp51_345*S46+ROcp51_944*C46;
    ROcp51_447 = ROcp51_445*C47+ROcp51_746*S47;
    ROcp51_547 = ROcp51_545*C47+ROcp51_846*S47;
    ROcp51_647 = ROcp51_645*C47+ROcp51_946*S47;
    ROcp51_747 = -(ROcp51_445*S47-ROcp51_746*C47);
    ROcp51_847 = -(ROcp51_545*S47-ROcp51_846*C47);
    ROcp51_947 = -(ROcp51_645*S47-ROcp51_946*C47);
    ROcp51_148 = ROcp51_146*C48+ROcp51_447*S48;
    ROcp51_248 = ROcp51_246*C48+ROcp51_547*S48;
    ROcp51_348 = ROcp51_346*C48+ROcp51_647*S48;
    ROcp51_448 = -(ROcp51_146*S48-ROcp51_447*C48);
    ROcp51_548 = -(ROcp51_246*S48-ROcp51_547*C48);
    ROcp51_648 = -(ROcp51_346*S48-ROcp51_647*C48);
    ROcp51_149 = ROcp51_148*C49-ROcp51_747*S49;
    ROcp51_249 = ROcp51_248*C49-ROcp51_847*S49;
    ROcp51_349 = ROcp51_348*C49-ROcp51_947*S49;
    ROcp51_749 = ROcp51_148*S49+ROcp51_747*C49;
    ROcp51_849 = ROcp51_248*S49+ROcp51_847*C49;
    ROcp51_949 = ROcp51_348*S49+ROcp51_947*C49;
    ROcp51_150 = ROcp51_149*C50+ROcp51_448*S50;
    ROcp51_250 = ROcp51_249*C50+ROcp51_548*S50;
    ROcp51_350 = ROcp51_349*C50+ROcp51_648*S50;
    ROcp51_450 = -(ROcp51_149*S50-ROcp51_448*C50);
    ROcp51_550 = -(ROcp51_249*S50-ROcp51_548*C50);
    ROcp51_650 = -(ROcp51_349*S50-ROcp51_648*C50);
    ROcp51_151 = ROcp51_150*C51-ROcp51_749*S51;
    ROcp51_251 = ROcp51_250*C51-ROcp51_849*S51;
    ROcp51_351 = ROcp51_350*C51-ROcp51_949*S51;
    ROcp51_751 = ROcp51_150*S51+ROcp51_749*C51;
    ROcp51_851 = ROcp51_250*S51+ROcp51_849*C51;
    ROcp51_951 = ROcp51_350*S51+ROcp51_949*C51;
    ROcp51_452 = ROcp51_450*C52+ROcp51_751*S52;
    ROcp51_552 = ROcp51_550*C52+ROcp51_851*S52;
    ROcp51_652 = ROcp51_650*C52+ROcp51_951*S52;
    ROcp51_752 = -(ROcp51_450*S52-ROcp51_751*C52);
    ROcp51_852 = -(ROcp51_550*S52-ROcp51_851*C52);
    ROcp51_952 = -(ROcp51_650*S52-ROcp51_951*C52);
    RLcp51_144 = ROcp51_134*s.dpt(1,24)+ROcp51_434*s.dpt(2,24)+ROcp51_733*s.dpt(3,24);
    RLcp51_244 = ROcp51_234*s.dpt(1,24)+ROcp51_534*s.dpt(2,24)+ROcp51_833*s.dpt(3,24);
    RLcp51_344 = ROcp51_334*s.dpt(1,24)+ROcp51_634*s.dpt(2,24)+ROcp51_933*s.dpt(3,24);
    ORcp51_144 = OMcp51_234*RLcp51_344-OMcp51_334*RLcp51_244;
    ORcp51_244 = -(OMcp51_134*RLcp51_344-OMcp51_334*RLcp51_144);
    ORcp51_344 = OMcp51_134*RLcp51_244-OMcp51_234*RLcp51_144;
    OMcp51_146 = OMcp51_134+ROcp51_445*qd(46);
    OMcp51_246 = OMcp51_234+ROcp51_545*qd(46);
    OMcp51_346 = OMcp51_334+ROcp51_645*qd(46);
    OPcp51_146 = OPcp51_134+ROcp51_445*qdd(46)+qd(46)*(OMcp51_234*ROcp51_645-OMcp51_334*ROcp51_545);
    OPcp51_246 = OPcp51_234+ROcp51_545*qdd(46)-qd(46)*(OMcp51_134*ROcp51_645-OMcp51_334*ROcp51_445);
    OPcp51_346 = OPcp51_334+ROcp51_645*qdd(46)+qd(46)*(OMcp51_134*ROcp51_545-OMcp51_234*ROcp51_445);
    RLcp51_147 = ROcp51_445*s.dpt(2,41)+ROcp51_746*s.dpt(3,41);
    RLcp51_247 = ROcp51_545*s.dpt(2,41)+ROcp51_846*s.dpt(3,41);
    RLcp51_347 = ROcp51_645*s.dpt(2,41)+ROcp51_946*s.dpt(3,41);
    OMcp51_147 = OMcp51_146+ROcp51_146*qd(47);
    OMcp51_247 = OMcp51_246+ROcp51_246*qd(47);
    OMcp51_347 = OMcp51_346+ROcp51_346*qd(47);
    ORcp51_147 = OMcp51_246*RLcp51_347-OMcp51_346*RLcp51_247;
    ORcp51_247 = -(OMcp51_146*RLcp51_347-OMcp51_346*RLcp51_147);
    ORcp51_347 = OMcp51_146*RLcp51_247-OMcp51_246*RLcp51_147;
    OPcp51_147 = OPcp51_146+ROcp51_146*qdd(47)+qd(47)*(OMcp51_246*ROcp51_346-OMcp51_346*ROcp51_246);
    OPcp51_247 = OPcp51_246+ROcp51_246*qdd(47)-qd(47)*(OMcp51_146*ROcp51_346-OMcp51_346*ROcp51_146);
    OPcp51_347 = OPcp51_346+ROcp51_346*qdd(47)+qd(47)*(OMcp51_146*ROcp51_246-OMcp51_246*ROcp51_146);
    RLcp51_148 = ROcp51_747*s.dpt(3,43);
    RLcp51_248 = ROcp51_847*s.dpt(3,43);
    RLcp51_348 = ROcp51_947*s.dpt(3,43);
    OMcp51_148 = OMcp51_147+ROcp51_747*qd(48);
    OMcp51_248 = OMcp51_247+ROcp51_847*qd(48);
    OMcp51_348 = OMcp51_347+ROcp51_947*qd(48);
    ORcp51_148 = OMcp51_247*RLcp51_348-OMcp51_347*RLcp51_248;
    ORcp51_248 = -(OMcp51_147*RLcp51_348-OMcp51_347*RLcp51_148);
    ORcp51_348 = OMcp51_147*RLcp51_248-OMcp51_247*RLcp51_148;
    OPcp51_148 = OPcp51_147+ROcp51_747*qdd(48)+qd(48)*(OMcp51_247*ROcp51_947-OMcp51_347*ROcp51_847);
    OPcp51_248 = OPcp51_247+ROcp51_847*qdd(48)-qd(48)*(OMcp51_147*ROcp51_947-OMcp51_347*ROcp51_747);
    OPcp51_348 = OPcp51_347+ROcp51_947*qdd(48)+qd(48)*(OMcp51_147*ROcp51_847-OMcp51_247*ROcp51_747);
    RLcp51_149 = ROcp51_148*s.dpt(1,46)+ROcp51_747*s.dpt(3,46);
    RLcp51_249 = ROcp51_248*s.dpt(1,46)+ROcp51_847*s.dpt(3,46);
    RLcp51_349 = ROcp51_348*s.dpt(1,46)+ROcp51_947*s.dpt(3,46);
    OMcp51_149 = OMcp51_148+ROcp51_448*qd(49);
    OMcp51_249 = OMcp51_248+ROcp51_548*qd(49);
    OMcp51_349 = OMcp51_348+ROcp51_648*qd(49);
    ORcp51_149 = OMcp51_248*RLcp51_349-OMcp51_348*RLcp51_249;
    ORcp51_249 = -(OMcp51_148*RLcp51_349-OMcp51_348*RLcp51_149);
    ORcp51_349 = OMcp51_148*RLcp51_249-OMcp51_248*RLcp51_149;
    OPcp51_149 = OPcp51_148+ROcp51_448*qdd(49)+qd(49)*(OMcp51_248*ROcp51_648-OMcp51_348*ROcp51_548);
    OPcp51_249 = OPcp51_248+ROcp51_548*qdd(49)-qd(49)*(OMcp51_148*ROcp51_648-OMcp51_348*ROcp51_448);
    OPcp51_349 = OPcp51_348+ROcp51_648*qdd(49)+qd(49)*(OMcp51_148*ROcp51_548-OMcp51_248*ROcp51_448);
    RLcp51_150 = ROcp51_149*s.dpt(1,48)+ROcp51_749*s.dpt(3,48);
    RLcp51_250 = ROcp51_249*s.dpt(1,48)+ROcp51_849*s.dpt(3,48);
    RLcp51_350 = ROcp51_349*s.dpt(1,48)+ROcp51_949*s.dpt(3,48);
    OMcp51_150 = OMcp51_149+ROcp51_749*qd(50);
    OMcp51_250 = OMcp51_249+ROcp51_849*qd(50);
    OMcp51_350 = OMcp51_349+ROcp51_949*qd(50);
    ORcp51_150 = OMcp51_249*RLcp51_350-OMcp51_349*RLcp51_250;
    ORcp51_250 = -(OMcp51_149*RLcp51_350-OMcp51_349*RLcp51_150);
    ORcp51_350 = OMcp51_149*RLcp51_250-OMcp51_249*RLcp51_150;
    OMcp51_151 = OMcp51_150+ROcp51_450*qd(51);
    OMcp51_251 = OMcp51_250+ROcp51_550*qd(51);
    OMcp51_351 = OMcp51_350+ROcp51_650*qd(51);
    OPcp51_151 = OPcp51_149+ROcp51_450*qdd(51)+ROcp51_749*qdd(50)+qd(50)*(OMcp51_249*ROcp51_949-OMcp51_349*ROcp51_849)+qd(51)*(OMcp51_250*...
 ROcp51_650-OMcp51_350*ROcp51_550);
    OPcp51_251 = OPcp51_249+ROcp51_550*qdd(51)+ROcp51_849*qdd(50)-qd(50)*(OMcp51_149*ROcp51_949-OMcp51_349*ROcp51_749)-qd(51)*(OMcp51_150*...
 ROcp51_650-OMcp51_350*ROcp51_450);
    OPcp51_351 = OPcp51_349+ROcp51_650*qdd(51)+ROcp51_949*qdd(50)+qd(50)*(OMcp51_149*ROcp51_849-OMcp51_249*ROcp51_749)+qd(51)*(OMcp51_150*...
 ROcp51_550-OMcp51_250*ROcp51_450);
    RLcp51_152 = ROcp51_751*s.dpt(3,52);
    RLcp51_252 = ROcp51_851*s.dpt(3,52);
    RLcp51_352 = ROcp51_951*s.dpt(3,52);
    POcp51_152 = RLcp51_131+RLcp51_134+RLcp51_144+RLcp51_147+RLcp51_148+RLcp51_149+RLcp51_150+RLcp51_152+q(1);
    POcp51_252 = RLcp51_231+RLcp51_234+RLcp51_244+RLcp51_247+RLcp51_248+RLcp51_249+RLcp51_250+RLcp51_252+q(2);
    POcp51_352 = RLcp51_331+RLcp51_334+RLcp51_344+RLcp51_347+RLcp51_348+RLcp51_349+RLcp51_350+RLcp51_352+q(3);
    OMcp51_152 = OMcp51_151+ROcp51_151*qd(52);
    OMcp51_252 = OMcp51_251+ROcp51_251*qd(52);
    OMcp51_352 = OMcp51_351+ROcp51_351*qd(52);
    ORcp51_152 = OMcp51_251*RLcp51_352-OMcp51_351*RLcp51_252;
    ORcp51_252 = -(OMcp51_151*RLcp51_352-OMcp51_351*RLcp51_152);
    ORcp51_352 = OMcp51_151*RLcp51_252-OMcp51_251*RLcp51_152;
    VIcp51_152 = ORcp51_131+ORcp51_134+ORcp51_144+ORcp51_147+ORcp51_148+ORcp51_149+ORcp51_150+ORcp51_152+qd(1);
    VIcp51_252 = ORcp51_231+ORcp51_234+ORcp51_244+ORcp51_247+ORcp51_248+ORcp51_249+ORcp51_250+ORcp51_252+qd(2);
    VIcp51_352 = ORcp51_331+ORcp51_334+ORcp51_344+ORcp51_347+ORcp51_348+ORcp51_349+ORcp51_350+ORcp51_352+qd(3);
    OPcp51_152 = OPcp51_151+ROcp51_151*qdd(52)+qd(52)*(OMcp51_251*ROcp51_351-OMcp51_351*ROcp51_251);
    OPcp51_252 = OPcp51_251+ROcp51_251*qdd(52)-qd(52)*(OMcp51_151*ROcp51_351-OMcp51_351*ROcp51_151);
    OPcp51_352 = OPcp51_351+ROcp51_351*qdd(52)+qd(52)*(OMcp51_151*ROcp51_251-OMcp51_251*ROcp51_151);
    ACcp51_152 = qdd(1)+OMcp51_233*ORcp51_334+OMcp51_234*ORcp51_344+OMcp51_246*ORcp51_347+OMcp51_247*ORcp51_348+OMcp51_248*ORcp51_349+OMcp51_249*...
 ORcp51_350+OMcp51_251*ORcp51_352+OMcp51_26*ORcp51_331-OMcp51_333*ORcp51_234-OMcp51_334*ORcp51_244-OMcp51_346*ORcp51_247-OMcp51_347*ORcp51_248-...
 OMcp51_348*ORcp51_249-OMcp51_349*ORcp51_250-OMcp51_351*ORcp51_252-OMcp51_36*ORcp51_231+OPcp51_233*RLcp51_334+OPcp51_234*RLcp51_344+OPcp51_246*...
 RLcp51_347+OPcp51_247*RLcp51_348+OPcp51_248*RLcp51_349+OPcp51_249*RLcp51_350+OPcp51_251*RLcp51_352+OPcp51_26*RLcp51_331-OPcp51_333*RLcp51_234-...
 OPcp51_334*RLcp51_244-OPcp51_346*RLcp51_247-OPcp51_347*RLcp51_248-OPcp51_348*RLcp51_249-OPcp51_349*RLcp51_250-OPcp51_351*RLcp51_252-OPcp51_36*...
 RLcp51_231;
    ACcp51_252 = qdd(2)-OMcp51_133*ORcp51_334-OMcp51_134*ORcp51_344-OMcp51_146*ORcp51_347-OMcp51_147*ORcp51_348-OMcp51_148*ORcp51_349-OMcp51_149*...
 ORcp51_350-OMcp51_151*ORcp51_352-OMcp51_16*ORcp51_331+OMcp51_333*ORcp51_134+OMcp51_334*ORcp51_144+OMcp51_346*ORcp51_147+OMcp51_347*ORcp51_148+...
 OMcp51_348*ORcp51_149+OMcp51_349*ORcp51_150+OMcp51_351*ORcp51_152+OMcp51_36*ORcp51_131-OPcp51_133*RLcp51_334-OPcp51_134*RLcp51_344-OPcp51_146*...
 RLcp51_347-OPcp51_147*RLcp51_348-OPcp51_148*RLcp51_349-OPcp51_149*RLcp51_350-OPcp51_151*RLcp51_352-OPcp51_16*RLcp51_331+OPcp51_333*RLcp51_134+...
 OPcp51_334*RLcp51_144+OPcp51_346*RLcp51_147+OPcp51_347*RLcp51_148+OPcp51_348*RLcp51_149+OPcp51_349*RLcp51_150+OPcp51_351*RLcp51_152+OPcp51_36*...
 RLcp51_131;
    ACcp51_352 = qdd(3)+OMcp51_133*ORcp51_234+OMcp51_134*ORcp51_244+OMcp51_146*ORcp51_247+OMcp51_147*ORcp51_248+OMcp51_148*ORcp51_249+OMcp51_149*...
 ORcp51_250+OMcp51_151*ORcp51_252+OMcp51_16*ORcp51_231-OMcp51_233*ORcp51_134-OMcp51_234*ORcp51_144-OMcp51_246*ORcp51_147-OMcp51_247*ORcp51_148-...
 OMcp51_248*ORcp51_149-OMcp51_249*ORcp51_150-OMcp51_251*ORcp51_152-OMcp51_26*ORcp51_131+OPcp51_133*RLcp51_234+OPcp51_134*RLcp51_244+OPcp51_146*...
 RLcp51_247+OPcp51_147*RLcp51_248+OPcp51_148*RLcp51_249+OPcp51_149*RLcp51_250+OPcp51_151*RLcp51_252+OPcp51_16*RLcp51_231-OPcp51_233*RLcp51_134-...
 OPcp51_234*RLcp51_144-OPcp51_246*RLcp51_147-OPcp51_247*RLcp51_148-OPcp51_248*RLcp51_149-OPcp51_249*RLcp51_150-OPcp51_251*RLcp51_152-OPcp51_26*...
 RLcp51_131;

% = = Block_1_0_0_52_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp51_152;
    sens.P(2) = POcp51_252;
    sens.P(3) = POcp51_352;
    sens.R(1,1) = ROcp51_151;
    sens.R(1,2) = ROcp51_251;
    sens.R(1,3) = ROcp51_351;
    sens.R(2,1) = ROcp51_452;
    sens.R(2,2) = ROcp51_552;
    sens.R(2,3) = ROcp51_652;
    sens.R(3,1) = ROcp51_752;
    sens.R(3,2) = ROcp51_852;
    sens.R(3,3) = ROcp51_952;
    sens.V(1) = VIcp51_152;
    sens.V(2) = VIcp51_252;
    sens.V(3) = VIcp51_352;
    sens.OM(1) = OMcp51_152;
    sens.OM(2) = OMcp51_252;
    sens.OM(3) = OMcp51_352;
    sens.A(1) = ACcp51_152;
    sens.A(2) = ACcp51_252;
    sens.A(3) = ACcp51_352;
    sens.OMP(1) = OPcp51_152;
    sens.OMP(2) = OPcp51_252;
    sens.OMP(3) = OPcp51_352;
 
% 
case 53, 


% = = Block_1_0_0_53_0_1 = = 
 
% Sensor Kinematics 


    ROcp52_25 = S4*S5;
    ROcp52_35 = -C4*S5;
    ROcp52_85 = -S4*C5;
    ROcp52_95 = C4*C5;
    ROcp52_16 = C5*C6;
    ROcp52_26 = ROcp52_25*C6+C4*S6;
    ROcp52_36 = ROcp52_35*C6+S4*S6;
    ROcp52_46 = -C5*S6;
    ROcp52_56 = -(ROcp52_25*S6-C4*C6);
    ROcp52_66 = -(ROcp52_35*S6-S4*C6);
    OMcp52_25 = qd(5)*C4;
    OMcp52_35 = qd(5)*S4;
    OMcp52_16 = qd(4)+qd(6)*S5;
    OMcp52_26 = OMcp52_25+ROcp52_85*qd(6);
    OMcp52_36 = OMcp52_35+ROcp52_95*qd(6);
    OPcp52_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp52_26 = ROcp52_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp52_35*S5-ROcp52_95*qd(4));
    OPcp52_36 = ROcp52_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp52_25*S5-ROcp52_85*qd(4));

% = = Block_1_0_0_53_0_4 = = 
 
% Sensor Kinematics 


    ROcp52_131 = ROcp52_16*C31-S31*S5;
    ROcp52_231 = ROcp52_26*C31-ROcp52_85*S31;
    ROcp52_331 = ROcp52_36*C31-ROcp52_95*S31;
    ROcp52_731 = ROcp52_16*S31+C31*S5;
    ROcp52_831 = ROcp52_26*S31+ROcp52_85*C31;
    ROcp52_931 = ROcp52_36*S31+ROcp52_95*C31;
    ROcp52_432 = ROcp52_46*C32+ROcp52_731*S32;
    ROcp52_532 = ROcp52_56*C32+ROcp52_831*S32;
    ROcp52_632 = ROcp52_66*C32+ROcp52_931*S32;
    ROcp52_732 = -(ROcp52_46*S32-ROcp52_731*C32);
    ROcp52_832 = -(ROcp52_56*S32-ROcp52_831*C32);
    ROcp52_932 = -(ROcp52_66*S32-ROcp52_931*C32);
    ROcp52_133 = ROcp52_131*C33-ROcp52_732*S33;
    ROcp52_233 = ROcp52_231*C33-ROcp52_832*S33;
    ROcp52_333 = ROcp52_331*C33-ROcp52_932*S33;
    ROcp52_733 = ROcp52_131*S33+ROcp52_732*C33;
    ROcp52_833 = ROcp52_231*S33+ROcp52_832*C33;
    ROcp52_933 = ROcp52_331*S33+ROcp52_932*C33;
    ROcp52_134 = ROcp52_133*C34+ROcp52_432*S34;
    ROcp52_234 = ROcp52_233*C34+ROcp52_532*S34;
    ROcp52_334 = ROcp52_333*C34+ROcp52_632*S34;
    ROcp52_434 = -(ROcp52_133*S34-ROcp52_432*C34);
    ROcp52_534 = -(ROcp52_233*S34-ROcp52_532*C34);
    ROcp52_634 = -(ROcp52_333*S34-ROcp52_632*C34);
    RLcp52_131 = ROcp52_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp52_231 = ROcp52_26*s.dpt(1,3)+ROcp52_85*s.dpt(3,3);
    RLcp52_331 = ROcp52_36*s.dpt(1,3)+ROcp52_95*s.dpt(3,3);
    OMcp52_131 = OMcp52_16+ROcp52_46*qd(31);
    OMcp52_231 = OMcp52_26+ROcp52_56*qd(31);
    OMcp52_331 = OMcp52_36+ROcp52_66*qd(31);
    ORcp52_131 = OMcp52_26*RLcp52_331-OMcp52_36*RLcp52_231;
    ORcp52_231 = -(OMcp52_16*RLcp52_331-OMcp52_36*RLcp52_131);
    ORcp52_331 = OMcp52_16*RLcp52_231-OMcp52_26*RLcp52_131;
    OMcp52_132 = OMcp52_131+ROcp52_131*qd(32);
    OMcp52_232 = OMcp52_231+ROcp52_231*qd(32);
    OMcp52_332 = OMcp52_331+ROcp52_331*qd(32);
    OMcp52_133 = OMcp52_132+ROcp52_432*qd(33);
    OMcp52_233 = OMcp52_232+ROcp52_532*qd(33);
    OMcp52_333 = OMcp52_332+ROcp52_632*qd(33);
    OPcp52_133 = OPcp52_16+ROcp52_131*qdd(32)+ROcp52_432*qdd(33)+ROcp52_46*qdd(31)+qd(31)*(OMcp52_26*ROcp52_66-OMcp52_36*ROcp52_56)+qd(32)*(...
 OMcp52_231*ROcp52_331-OMcp52_331*ROcp52_231)+qd(33)*(OMcp52_232*ROcp52_632-OMcp52_332*ROcp52_532);
    OPcp52_233 = OPcp52_26+ROcp52_231*qdd(32)+ROcp52_532*qdd(33)+ROcp52_56*qdd(31)-qd(31)*(OMcp52_16*ROcp52_66-OMcp52_36*ROcp52_46)-qd(32)*(...
 OMcp52_131*ROcp52_331-OMcp52_331*ROcp52_131)-qd(33)*(OMcp52_132*ROcp52_632-OMcp52_332*ROcp52_432);
    OPcp52_333 = OPcp52_36+ROcp52_331*qdd(32)+ROcp52_632*qdd(33)+ROcp52_66*qdd(31)+qd(31)*(OMcp52_16*ROcp52_56-OMcp52_26*ROcp52_46)+qd(32)*(...
 OMcp52_131*ROcp52_231-OMcp52_231*ROcp52_131)+qd(33)*(OMcp52_132*ROcp52_532-OMcp52_232*ROcp52_432);
    RLcp52_134 = ROcp52_733*s.dpt(3,21);
    RLcp52_234 = ROcp52_833*s.dpt(3,21);
    RLcp52_334 = ROcp52_933*s.dpt(3,21);
    POcp52_134 = RLcp52_131+RLcp52_134+q(1);
    POcp52_234 = RLcp52_231+RLcp52_234+q(2);
    POcp52_334 = RLcp52_331+RLcp52_334+q(3);
    OMcp52_134 = OMcp52_133+ROcp52_733*qd(34);
    OMcp52_234 = OMcp52_233+ROcp52_833*qd(34);
    OMcp52_334 = OMcp52_333+ROcp52_933*qd(34);
    ORcp52_134 = OMcp52_233*RLcp52_334-OMcp52_333*RLcp52_234;
    ORcp52_234 = -(OMcp52_133*RLcp52_334-OMcp52_333*RLcp52_134);
    ORcp52_334 = OMcp52_133*RLcp52_234-OMcp52_233*RLcp52_134;
    VIcp52_134 = ORcp52_131+ORcp52_134+qd(1);
    VIcp52_234 = ORcp52_231+ORcp52_234+qd(2);
    VIcp52_334 = ORcp52_331+ORcp52_334+qd(3);
    ACcp52_134 = qdd(1)+OMcp52_233*ORcp52_334+OMcp52_26*ORcp52_331-OMcp52_333*ORcp52_234-OMcp52_36*ORcp52_231+OPcp52_233*RLcp52_334+OPcp52_26*...
 RLcp52_331-OPcp52_333*RLcp52_234-OPcp52_36*RLcp52_231;
    ACcp52_234 = qdd(2)-OMcp52_133*ORcp52_334-OMcp52_16*ORcp52_331+OMcp52_333*ORcp52_134+OMcp52_36*ORcp52_131-OPcp52_133*RLcp52_334-OPcp52_16*...
 RLcp52_331+OPcp52_333*RLcp52_134+OPcp52_36*RLcp52_131;
    ACcp52_334 = qdd(3)+OMcp52_133*ORcp52_234+OMcp52_16*ORcp52_231-OMcp52_233*ORcp52_134-OMcp52_26*ORcp52_131+OPcp52_133*RLcp52_234+OPcp52_16*...
 RLcp52_231-OPcp52_233*RLcp52_134-OPcp52_26*RLcp52_131;

% = = Block_1_0_0_53_0_7 = = 
 
% Sensor Kinematics 


    ROcp52_153 = ROcp52_134*C53+ROcp52_434*S53;
    ROcp52_253 = ROcp52_234*C53+ROcp52_534*S53;
    ROcp52_353 = ROcp52_334*C53+ROcp52_634*S53;
    ROcp52_453 = -(ROcp52_134*S53-ROcp52_434*C53);
    ROcp52_553 = -(ROcp52_234*S53-ROcp52_534*C53);
    ROcp52_653 = -(ROcp52_334*S53-ROcp52_634*C53);
    OMcp52_153 = OMcp52_134+ROcp52_733*qd(53);
    OMcp52_253 = OMcp52_234+ROcp52_833*qd(53);
    OMcp52_353 = OMcp52_334+ROcp52_933*qd(53);
    OPcp52_153 = OPcp52_133+ROcp52_733*qdd(34)+ROcp52_733*qdd(53)+qd(34)*(OMcp52_233*ROcp52_933-OMcp52_333*ROcp52_833)+qd(53)*(OMcp52_234*...
 ROcp52_933-OMcp52_334*ROcp52_833);
    OPcp52_253 = OPcp52_233+ROcp52_833*qdd(34)+ROcp52_833*qdd(53)-qd(34)*(OMcp52_133*ROcp52_933-OMcp52_333*ROcp52_733)-qd(53)*(OMcp52_134*...
 ROcp52_933-OMcp52_334*ROcp52_733);
    OPcp52_353 = OPcp52_333+ROcp52_933*qdd(34)+ROcp52_933*qdd(53)+qd(34)*(OMcp52_133*ROcp52_833-OMcp52_233*ROcp52_733)+qd(53)*(OMcp52_134*...
 ROcp52_833-OMcp52_234*ROcp52_733);

% = = Block_1_0_0_53_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp52_134;
    sens.P(2) = POcp52_234;
    sens.P(3) = POcp52_334;
    sens.R(1,1) = ROcp52_153;
    sens.R(1,2) = ROcp52_253;
    sens.R(1,3) = ROcp52_353;
    sens.R(2,1) = ROcp52_453;
    sens.R(2,2) = ROcp52_553;
    sens.R(2,3) = ROcp52_653;
    sens.R(3,1) = ROcp52_733;
    sens.R(3,2) = ROcp52_833;
    sens.R(3,3) = ROcp52_933;
    sens.V(1) = VIcp52_134;
    sens.V(2) = VIcp52_234;
    sens.V(3) = VIcp52_334;
    sens.OM(1) = OMcp52_153;
    sens.OM(2) = OMcp52_253;
    sens.OM(3) = OMcp52_353;
    sens.A(1) = ACcp52_134;
    sens.A(2) = ACcp52_234;
    sens.A(3) = ACcp52_334;
    sens.OMP(1) = OPcp52_153;
    sens.OMP(2) = OPcp52_253;
    sens.OMP(3) = OPcp52_353;
 
% 
case 54, 


% = = Block_1_0_0_54_0_1 = = 
 
% Sensor Kinematics 


    ROcp53_25 = S4*S5;
    ROcp53_35 = -C4*S5;
    ROcp53_85 = -S4*C5;
    ROcp53_95 = C4*C5;
    ROcp53_16 = C5*C6;
    ROcp53_26 = ROcp53_25*C6+C4*S6;
    ROcp53_36 = ROcp53_35*C6+S4*S6;
    ROcp53_46 = -C5*S6;
    ROcp53_56 = -(ROcp53_25*S6-C4*C6);
    ROcp53_66 = -(ROcp53_35*S6-S4*C6);
    OMcp53_25 = qd(5)*C4;
    OMcp53_35 = qd(5)*S4;
    OMcp53_16 = qd(4)+qd(6)*S5;
    OMcp53_26 = OMcp53_25+ROcp53_85*qd(6);
    OMcp53_36 = OMcp53_35+ROcp53_95*qd(6);
    OPcp53_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp53_26 = ROcp53_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp53_35*S5-ROcp53_95*qd(4));
    OPcp53_36 = ROcp53_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp53_25*S5-ROcp53_85*qd(4));

% = = Block_1_0_0_54_0_4 = = 
 
% Sensor Kinematics 


    ROcp53_131 = ROcp53_16*C31-S31*S5;
    ROcp53_231 = ROcp53_26*C31-ROcp53_85*S31;
    ROcp53_331 = ROcp53_36*C31-ROcp53_95*S31;
    ROcp53_731 = ROcp53_16*S31+C31*S5;
    ROcp53_831 = ROcp53_26*S31+ROcp53_85*C31;
    ROcp53_931 = ROcp53_36*S31+ROcp53_95*C31;
    ROcp53_432 = ROcp53_46*C32+ROcp53_731*S32;
    ROcp53_532 = ROcp53_56*C32+ROcp53_831*S32;
    ROcp53_632 = ROcp53_66*C32+ROcp53_931*S32;
    ROcp53_732 = -(ROcp53_46*S32-ROcp53_731*C32);
    ROcp53_832 = -(ROcp53_56*S32-ROcp53_831*C32);
    ROcp53_932 = -(ROcp53_66*S32-ROcp53_931*C32);
    ROcp53_133 = ROcp53_131*C33-ROcp53_732*S33;
    ROcp53_233 = ROcp53_231*C33-ROcp53_832*S33;
    ROcp53_333 = ROcp53_331*C33-ROcp53_932*S33;
    ROcp53_733 = ROcp53_131*S33+ROcp53_732*C33;
    ROcp53_833 = ROcp53_231*S33+ROcp53_832*C33;
    ROcp53_933 = ROcp53_331*S33+ROcp53_932*C33;
    ROcp53_134 = ROcp53_133*C34+ROcp53_432*S34;
    ROcp53_234 = ROcp53_233*C34+ROcp53_532*S34;
    ROcp53_334 = ROcp53_333*C34+ROcp53_632*S34;
    ROcp53_434 = -(ROcp53_133*S34-ROcp53_432*C34);
    ROcp53_534 = -(ROcp53_233*S34-ROcp53_532*C34);
    ROcp53_634 = -(ROcp53_333*S34-ROcp53_632*C34);
    RLcp53_131 = ROcp53_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp53_231 = ROcp53_26*s.dpt(1,3)+ROcp53_85*s.dpt(3,3);
    RLcp53_331 = ROcp53_36*s.dpt(1,3)+ROcp53_95*s.dpt(3,3);
    OMcp53_131 = OMcp53_16+ROcp53_46*qd(31);
    OMcp53_231 = OMcp53_26+ROcp53_56*qd(31);
    OMcp53_331 = OMcp53_36+ROcp53_66*qd(31);
    ORcp53_131 = OMcp53_26*RLcp53_331-OMcp53_36*RLcp53_231;
    ORcp53_231 = -(OMcp53_16*RLcp53_331-OMcp53_36*RLcp53_131);
    ORcp53_331 = OMcp53_16*RLcp53_231-OMcp53_26*RLcp53_131;
    OMcp53_132 = OMcp53_131+ROcp53_131*qd(32);
    OMcp53_232 = OMcp53_231+ROcp53_231*qd(32);
    OMcp53_332 = OMcp53_331+ROcp53_331*qd(32);
    OMcp53_133 = OMcp53_132+ROcp53_432*qd(33);
    OMcp53_233 = OMcp53_232+ROcp53_532*qd(33);
    OMcp53_333 = OMcp53_332+ROcp53_632*qd(33);
    OPcp53_133 = OPcp53_16+ROcp53_131*qdd(32)+ROcp53_432*qdd(33)+ROcp53_46*qdd(31)+qd(31)*(OMcp53_26*ROcp53_66-OMcp53_36*ROcp53_56)+qd(32)*(...
 OMcp53_231*ROcp53_331-OMcp53_331*ROcp53_231)+qd(33)*(OMcp53_232*ROcp53_632-OMcp53_332*ROcp53_532);
    OPcp53_233 = OPcp53_26+ROcp53_231*qdd(32)+ROcp53_532*qdd(33)+ROcp53_56*qdd(31)-qd(31)*(OMcp53_16*ROcp53_66-OMcp53_36*ROcp53_46)-qd(32)*(...
 OMcp53_131*ROcp53_331-OMcp53_331*ROcp53_131)-qd(33)*(OMcp53_132*ROcp53_632-OMcp53_332*ROcp53_432);
    OPcp53_333 = OPcp53_36+ROcp53_331*qdd(32)+ROcp53_632*qdd(33)+ROcp53_66*qdd(31)+qd(31)*(OMcp53_16*ROcp53_56-OMcp53_26*ROcp53_46)+qd(32)*(...
 OMcp53_131*ROcp53_231-OMcp53_231*ROcp53_131)+qd(33)*(OMcp53_132*ROcp53_532-OMcp53_232*ROcp53_432);
    RLcp53_134 = ROcp53_733*s.dpt(3,21);
    RLcp53_234 = ROcp53_833*s.dpt(3,21);
    RLcp53_334 = ROcp53_933*s.dpt(3,21);
    OMcp53_134 = OMcp53_133+ROcp53_733*qd(34);
    OMcp53_234 = OMcp53_233+ROcp53_833*qd(34);
    OMcp53_334 = OMcp53_333+ROcp53_933*qd(34);
    ORcp53_134 = OMcp53_233*RLcp53_334-OMcp53_333*RLcp53_234;
    ORcp53_234 = -(OMcp53_133*RLcp53_334-OMcp53_333*RLcp53_134);
    ORcp53_334 = OMcp53_133*RLcp53_234-OMcp53_233*RLcp53_134;

% = = Block_1_0_0_54_0_7 = = 
 
% Sensor Kinematics 


    ROcp53_153 = ROcp53_134*C53+ROcp53_434*S53;
    ROcp53_253 = ROcp53_234*C53+ROcp53_534*S53;
    ROcp53_353 = ROcp53_334*C53+ROcp53_634*S53;
    ROcp53_453 = -(ROcp53_134*S53-ROcp53_434*C53);
    ROcp53_553 = -(ROcp53_234*S53-ROcp53_534*C53);
    ROcp53_653 = -(ROcp53_334*S53-ROcp53_634*C53);
    ROcp53_154 = ROcp53_153*C54-ROcp53_733*S54;
    ROcp53_254 = ROcp53_253*C54-ROcp53_833*S54;
    ROcp53_354 = ROcp53_353*C54-ROcp53_933*S54;
    ROcp53_754 = ROcp53_153*S54+ROcp53_733*C54;
    ROcp53_854 = ROcp53_253*S54+ROcp53_833*C54;
    ROcp53_954 = ROcp53_353*S54+ROcp53_933*C54;
    OMcp53_153 = OMcp53_134+ROcp53_733*qd(53);
    OMcp53_253 = OMcp53_234+ROcp53_833*qd(53);
    OMcp53_353 = OMcp53_334+ROcp53_933*qd(53);
    OPcp53_153 = OPcp53_133+ROcp53_733*qdd(34)+ROcp53_733*qdd(53)+qd(34)*(OMcp53_233*ROcp53_933-OMcp53_333*ROcp53_833)+qd(53)*(OMcp53_234*...
 ROcp53_933-OMcp53_334*ROcp53_833);
    OPcp53_253 = OPcp53_233+ROcp53_833*qdd(34)+ROcp53_833*qdd(53)-qd(34)*(OMcp53_133*ROcp53_933-OMcp53_333*ROcp53_733)-qd(53)*(OMcp53_134*...
 ROcp53_933-OMcp53_334*ROcp53_733);
    OPcp53_353 = OPcp53_333+ROcp53_933*qdd(34)+ROcp53_933*qdd(53)+qd(34)*(OMcp53_133*ROcp53_833-OMcp53_233*ROcp53_733)+qd(53)*(OMcp53_134*...
 ROcp53_833-OMcp53_234*ROcp53_733);
    RLcp53_154 = ROcp53_733*s.dpt(3,56);
    RLcp53_254 = ROcp53_833*s.dpt(3,56);
    RLcp53_354 = ROcp53_933*s.dpt(3,56);
    POcp53_154 = RLcp53_131+RLcp53_134+RLcp53_154+q(1);
    POcp53_254 = RLcp53_231+RLcp53_234+RLcp53_254+q(2);
    POcp53_354 = RLcp53_331+RLcp53_334+RLcp53_354+q(3);
    OMcp53_154 = OMcp53_153+ROcp53_453*qd(54);
    OMcp53_254 = OMcp53_253+ROcp53_553*qd(54);
    OMcp53_354 = OMcp53_353+ROcp53_653*qd(54);
    ORcp53_154 = OMcp53_253*RLcp53_354-OMcp53_353*RLcp53_254;
    ORcp53_254 = -(OMcp53_153*RLcp53_354-OMcp53_353*RLcp53_154);
    ORcp53_354 = OMcp53_153*RLcp53_254-OMcp53_253*RLcp53_154;
    VIcp53_154 = ORcp53_131+ORcp53_134+ORcp53_154+qd(1);
    VIcp53_254 = ORcp53_231+ORcp53_234+ORcp53_254+qd(2);
    VIcp53_354 = ORcp53_331+ORcp53_334+ORcp53_354+qd(3);
    OPcp53_154 = OPcp53_153+ROcp53_453*qdd(54)+qd(54)*(OMcp53_253*ROcp53_653-OMcp53_353*ROcp53_553);
    OPcp53_254 = OPcp53_253+ROcp53_553*qdd(54)-qd(54)*(OMcp53_153*ROcp53_653-OMcp53_353*ROcp53_453);
    OPcp53_354 = OPcp53_353+ROcp53_653*qdd(54)+qd(54)*(OMcp53_153*ROcp53_553-OMcp53_253*ROcp53_453);
    ACcp53_154 = qdd(1)+OMcp53_233*ORcp53_334+OMcp53_253*ORcp53_354+OMcp53_26*ORcp53_331-OMcp53_333*ORcp53_234-OMcp53_353*ORcp53_254-OMcp53_36*...
 ORcp53_231+OPcp53_233*RLcp53_334+OPcp53_253*RLcp53_354+OPcp53_26*RLcp53_331-OPcp53_333*RLcp53_234-OPcp53_353*RLcp53_254-OPcp53_36*RLcp53_231;
    ACcp53_254 = qdd(2)-OMcp53_133*ORcp53_334-OMcp53_153*ORcp53_354-OMcp53_16*ORcp53_331+OMcp53_333*ORcp53_134+OMcp53_353*ORcp53_154+OMcp53_36*...
 ORcp53_131-OPcp53_133*RLcp53_334-OPcp53_153*RLcp53_354-OPcp53_16*RLcp53_331+OPcp53_333*RLcp53_134+OPcp53_353*RLcp53_154+OPcp53_36*RLcp53_131;
    ACcp53_354 = qdd(3)+OMcp53_133*ORcp53_234+OMcp53_153*ORcp53_254+OMcp53_16*ORcp53_231-OMcp53_233*ORcp53_134-OMcp53_253*ORcp53_154-OMcp53_26*...
 ORcp53_131+OPcp53_133*RLcp53_234+OPcp53_153*RLcp53_254+OPcp53_16*RLcp53_231-OPcp53_233*RLcp53_134-OPcp53_253*RLcp53_154-OPcp53_26*RLcp53_131;

% = = Block_1_0_0_54_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp53_154;
    sens.P(2) = POcp53_254;
    sens.P(3) = POcp53_354;
    sens.R(1,1) = ROcp53_154;
    sens.R(1,2) = ROcp53_254;
    sens.R(1,3) = ROcp53_354;
    sens.R(2,1) = ROcp53_453;
    sens.R(2,2) = ROcp53_553;
    sens.R(2,3) = ROcp53_653;
    sens.R(3,1) = ROcp53_754;
    sens.R(3,2) = ROcp53_854;
    sens.R(3,3) = ROcp53_954;
    sens.V(1) = VIcp53_154;
    sens.V(2) = VIcp53_254;
    sens.V(3) = VIcp53_354;
    sens.OM(1) = OMcp53_154;
    sens.OM(2) = OMcp53_254;
    sens.OM(3) = OMcp53_354;
    sens.A(1) = ACcp53_154;
    sens.A(2) = ACcp53_254;
    sens.A(3) = ACcp53_354;
    sens.OMP(1) = OPcp53_154;
    sens.OMP(2) = OPcp53_254;
    sens.OMP(3) = OPcp53_354;
 
% 
case 55, 


% = = Block_1_0_0_55_0_1 = = 
 
% Sensor Kinematics 


    ROcp54_25 = S4*S5;
    ROcp54_35 = -C4*S5;
    ROcp54_85 = -S4*C5;
    ROcp54_95 = C4*C5;
    ROcp54_16 = C5*C6;
    ROcp54_26 = ROcp54_25*C6+C4*S6;
    ROcp54_36 = ROcp54_35*C6+S4*S6;
    ROcp54_46 = -C5*S6;
    ROcp54_56 = -(ROcp54_25*S6-C4*C6);
    ROcp54_66 = -(ROcp54_35*S6-S4*C6);
    OMcp54_25 = qd(5)*C4;
    OMcp54_35 = qd(5)*S4;
    OMcp54_16 = qd(4)+qd(6)*S5;
    OMcp54_26 = OMcp54_25+ROcp54_85*qd(6);
    OMcp54_36 = OMcp54_35+ROcp54_95*qd(6);
    OPcp54_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp54_26 = ROcp54_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp54_35*S5-ROcp54_95*qd(4));
    OPcp54_36 = ROcp54_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp54_25*S5-ROcp54_85*qd(4));

% = = Block_1_0_0_55_0_4 = = 
 
% Sensor Kinematics 


    ROcp54_131 = ROcp54_16*C31-S31*S5;
    ROcp54_231 = ROcp54_26*C31-ROcp54_85*S31;
    ROcp54_331 = ROcp54_36*C31-ROcp54_95*S31;
    ROcp54_731 = ROcp54_16*S31+C31*S5;
    ROcp54_831 = ROcp54_26*S31+ROcp54_85*C31;
    ROcp54_931 = ROcp54_36*S31+ROcp54_95*C31;
    ROcp54_432 = ROcp54_46*C32+ROcp54_731*S32;
    ROcp54_532 = ROcp54_56*C32+ROcp54_831*S32;
    ROcp54_632 = ROcp54_66*C32+ROcp54_931*S32;
    ROcp54_732 = -(ROcp54_46*S32-ROcp54_731*C32);
    ROcp54_832 = -(ROcp54_56*S32-ROcp54_831*C32);
    ROcp54_932 = -(ROcp54_66*S32-ROcp54_931*C32);
    ROcp54_133 = ROcp54_131*C33-ROcp54_732*S33;
    ROcp54_233 = ROcp54_231*C33-ROcp54_832*S33;
    ROcp54_333 = ROcp54_331*C33-ROcp54_932*S33;
    ROcp54_733 = ROcp54_131*S33+ROcp54_732*C33;
    ROcp54_833 = ROcp54_231*S33+ROcp54_832*C33;
    ROcp54_933 = ROcp54_331*S33+ROcp54_932*C33;
    ROcp54_134 = ROcp54_133*C34+ROcp54_432*S34;
    ROcp54_234 = ROcp54_233*C34+ROcp54_532*S34;
    ROcp54_334 = ROcp54_333*C34+ROcp54_632*S34;
    ROcp54_434 = -(ROcp54_133*S34-ROcp54_432*C34);
    ROcp54_534 = -(ROcp54_233*S34-ROcp54_532*C34);
    ROcp54_634 = -(ROcp54_333*S34-ROcp54_632*C34);
    RLcp54_131 = ROcp54_16*s.dpt(1,3)+s.dpt(3,3)*S5;
    RLcp54_231 = ROcp54_26*s.dpt(1,3)+ROcp54_85*s.dpt(3,3);
    RLcp54_331 = ROcp54_36*s.dpt(1,3)+ROcp54_95*s.dpt(3,3);
    OMcp54_131 = OMcp54_16+ROcp54_46*qd(31);
    OMcp54_231 = OMcp54_26+ROcp54_56*qd(31);
    OMcp54_331 = OMcp54_36+ROcp54_66*qd(31);
    ORcp54_131 = OMcp54_26*RLcp54_331-OMcp54_36*RLcp54_231;
    ORcp54_231 = -(OMcp54_16*RLcp54_331-OMcp54_36*RLcp54_131);
    ORcp54_331 = OMcp54_16*RLcp54_231-OMcp54_26*RLcp54_131;
    OMcp54_132 = OMcp54_131+ROcp54_131*qd(32);
    OMcp54_232 = OMcp54_231+ROcp54_231*qd(32);
    OMcp54_332 = OMcp54_331+ROcp54_331*qd(32);
    OMcp54_133 = OMcp54_132+ROcp54_432*qd(33);
    OMcp54_233 = OMcp54_232+ROcp54_532*qd(33);
    OMcp54_333 = OMcp54_332+ROcp54_632*qd(33);
    OPcp54_133 = OPcp54_16+ROcp54_131*qdd(32)+ROcp54_432*qdd(33)+ROcp54_46*qdd(31)+qd(31)*(OMcp54_26*ROcp54_66-OMcp54_36*ROcp54_56)+qd(32)*(...
 OMcp54_231*ROcp54_331-OMcp54_331*ROcp54_231)+qd(33)*(OMcp54_232*ROcp54_632-OMcp54_332*ROcp54_532);
    OPcp54_233 = OPcp54_26+ROcp54_231*qdd(32)+ROcp54_532*qdd(33)+ROcp54_56*qdd(31)-qd(31)*(OMcp54_16*ROcp54_66-OMcp54_36*ROcp54_46)-qd(32)*(...
 OMcp54_131*ROcp54_331-OMcp54_331*ROcp54_131)-qd(33)*(OMcp54_132*ROcp54_632-OMcp54_332*ROcp54_432);
    OPcp54_333 = OPcp54_36+ROcp54_331*qdd(32)+ROcp54_632*qdd(33)+ROcp54_66*qdd(31)+qd(31)*(OMcp54_16*ROcp54_56-OMcp54_26*ROcp54_46)+qd(32)*(...
 OMcp54_131*ROcp54_231-OMcp54_231*ROcp54_131)+qd(33)*(OMcp54_132*ROcp54_532-OMcp54_232*ROcp54_432);
    RLcp54_134 = ROcp54_733*s.dpt(3,21);
    RLcp54_234 = ROcp54_833*s.dpt(3,21);
    RLcp54_334 = ROcp54_933*s.dpt(3,21);
    OMcp54_134 = OMcp54_133+ROcp54_733*qd(34);
    OMcp54_234 = OMcp54_233+ROcp54_833*qd(34);
    OMcp54_334 = OMcp54_333+ROcp54_933*qd(34);
    ORcp54_134 = OMcp54_233*RLcp54_334-OMcp54_333*RLcp54_234;
    ORcp54_234 = -(OMcp54_133*RLcp54_334-OMcp54_333*RLcp54_134);
    ORcp54_334 = OMcp54_133*RLcp54_234-OMcp54_233*RLcp54_134;

% = = Block_1_0_0_55_0_7 = = 
 
% Sensor Kinematics 


    ROcp54_153 = ROcp54_134*C53+ROcp54_434*S53;
    ROcp54_253 = ROcp54_234*C53+ROcp54_534*S53;
    ROcp54_353 = ROcp54_334*C53+ROcp54_634*S53;
    ROcp54_453 = -(ROcp54_134*S53-ROcp54_434*C53);
    ROcp54_553 = -(ROcp54_234*S53-ROcp54_534*C53);
    ROcp54_653 = -(ROcp54_334*S53-ROcp54_634*C53);
    ROcp54_154 = ROcp54_153*C54-ROcp54_733*S54;
    ROcp54_254 = ROcp54_253*C54-ROcp54_833*S54;
    ROcp54_354 = ROcp54_353*C54-ROcp54_933*S54;
    ROcp54_754 = ROcp54_153*S54+ROcp54_733*C54;
    ROcp54_854 = ROcp54_253*S54+ROcp54_833*C54;
    ROcp54_954 = ROcp54_353*S54+ROcp54_933*C54;
    OMcp54_153 = OMcp54_134+ROcp54_733*qd(53);
    OMcp54_253 = OMcp54_234+ROcp54_833*qd(53);
    OMcp54_353 = OMcp54_334+ROcp54_933*qd(53);
    OPcp54_153 = OPcp54_133+ROcp54_733*qdd(34)+ROcp54_733*qdd(53)+qd(34)*(OMcp54_233*ROcp54_933-OMcp54_333*ROcp54_833)+qd(53)*(OMcp54_234*...
 ROcp54_933-OMcp54_334*ROcp54_833);
    OPcp54_253 = OPcp54_233+ROcp54_833*qdd(34)+ROcp54_833*qdd(53)-qd(34)*(OMcp54_133*ROcp54_933-OMcp54_333*ROcp54_733)-qd(53)*(OMcp54_134*...
 ROcp54_933-OMcp54_334*ROcp54_733);
    OPcp54_353 = OPcp54_333+ROcp54_933*qdd(34)+ROcp54_933*qdd(53)+qd(34)*(OMcp54_133*ROcp54_833-OMcp54_233*ROcp54_733)+qd(53)*(OMcp54_134*...
 ROcp54_833-OMcp54_234*ROcp54_733);
    RLcp54_154 = ROcp54_733*s.dpt(3,56);
    RLcp54_254 = ROcp54_833*s.dpt(3,56);
    RLcp54_354 = ROcp54_933*s.dpt(3,56);
    OMcp54_154 = OMcp54_153+ROcp54_453*qd(54);
    OMcp54_254 = OMcp54_253+ROcp54_553*qd(54);
    OMcp54_354 = OMcp54_353+ROcp54_653*qd(54);
    ORcp54_154 = OMcp54_253*RLcp54_354-OMcp54_353*RLcp54_254;
    ORcp54_254 = -(OMcp54_153*RLcp54_354-OMcp54_353*RLcp54_154);
    ORcp54_354 = OMcp54_153*RLcp54_254-OMcp54_253*RLcp54_154;
    OPcp54_154 = OPcp54_153+ROcp54_453*qdd(54)+qd(54)*(OMcp54_253*ROcp54_653-OMcp54_353*ROcp54_553);
    OPcp54_254 = OPcp54_253+ROcp54_553*qdd(54)-qd(54)*(OMcp54_153*ROcp54_653-OMcp54_353*ROcp54_453);
    OPcp54_354 = OPcp54_353+ROcp54_653*qdd(54)+qd(54)*(OMcp54_153*ROcp54_553-OMcp54_253*ROcp54_453);
    RLcp54_155 = Dz553*ROcp54_754+ROcp54_154*s.dpt(1,57);
    RLcp54_255 = Dz553*ROcp54_854+ROcp54_254*s.dpt(1,57);
    RLcp54_355 = Dz553*ROcp54_954+ROcp54_354*s.dpt(1,57);
    POcp54_155 = RLcp54_131+RLcp54_134+RLcp54_154+RLcp54_155+q(1);
    POcp54_255 = RLcp54_231+RLcp54_234+RLcp54_254+RLcp54_255+q(2);
    POcp54_355 = RLcp54_331+RLcp54_334+RLcp54_354+RLcp54_355+q(3);
    ORcp54_155 = OMcp54_254*RLcp54_355-OMcp54_354*RLcp54_255;
    ORcp54_255 = -(OMcp54_154*RLcp54_355-OMcp54_354*RLcp54_155);
    ORcp54_355 = OMcp54_154*RLcp54_255-OMcp54_254*RLcp54_155;
    VIcp54_155 = ORcp54_131+ORcp54_134+ORcp54_154+ORcp54_155+qd(1)+ROcp54_754*qd(55);
    VIcp54_255 = ORcp54_231+ORcp54_234+ORcp54_254+ORcp54_255+qd(2)+ROcp54_854*qd(55);
    VIcp54_355 = ORcp54_331+ORcp54_334+ORcp54_354+ORcp54_355+qd(3)+ROcp54_954*qd(55);
    ACcp54_155 = qdd(1)+OMcp54_233*ORcp54_334+OMcp54_253*ORcp54_354+OMcp54_254*ORcp54_355+OMcp54_26*ORcp54_331-OMcp54_333*ORcp54_234-OMcp54_353*...
 ORcp54_254-OMcp54_354*ORcp54_255-OMcp54_36*ORcp54_231+OPcp54_233*RLcp54_334+OPcp54_253*RLcp54_354+OPcp54_254*RLcp54_355+OPcp54_26*RLcp54_331-...
 OPcp54_333*RLcp54_234-OPcp54_353*RLcp54_254-OPcp54_354*RLcp54_255-OPcp54_36*RLcp54_231+ROcp54_754*qdd(55)+(2.0)*qd(55)*(OMcp54_254*ROcp54_954-OMcp54_354*...
 ROcp54_854);
    ACcp54_255 = qdd(2)-OMcp54_133*ORcp54_334-OMcp54_153*ORcp54_354-OMcp54_154*ORcp54_355-OMcp54_16*ORcp54_331+OMcp54_333*ORcp54_134+OMcp54_353*...
 ORcp54_154+OMcp54_354*ORcp54_155+OMcp54_36*ORcp54_131-OPcp54_133*RLcp54_334-OPcp54_153*RLcp54_354-OPcp54_154*RLcp54_355-OPcp54_16*RLcp54_331+...
 OPcp54_333*RLcp54_134+OPcp54_353*RLcp54_154+OPcp54_354*RLcp54_155+OPcp54_36*RLcp54_131+ROcp54_854*qdd(55)-(2.0)*qd(55)*(OMcp54_154*ROcp54_954-OMcp54_354*...
 ROcp54_754);
    ACcp54_355 = qdd(3)+OMcp54_133*ORcp54_234+OMcp54_153*ORcp54_254+OMcp54_154*ORcp54_255+OMcp54_16*ORcp54_231-OMcp54_233*ORcp54_134-OMcp54_253*...
 ORcp54_154-OMcp54_254*ORcp54_155-OMcp54_26*ORcp54_131+OPcp54_133*RLcp54_234+OPcp54_153*RLcp54_254+OPcp54_154*RLcp54_255+OPcp54_16*RLcp54_231-...
 OPcp54_233*RLcp54_134-OPcp54_253*RLcp54_154-OPcp54_254*RLcp54_155-OPcp54_26*RLcp54_131+ROcp54_954*qdd(55)+(2.0)*qd(55)*(OMcp54_154*ROcp54_854-OMcp54_254*...
 ROcp54_754);

% = = Block_1_0_0_55_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp54_155;
    sens.P(2) = POcp54_255;
    sens.P(3) = POcp54_355;
    sens.R(1,1) = ROcp54_154;
    sens.R(1,2) = ROcp54_254;
    sens.R(1,3) = ROcp54_354;
    sens.R(2,1) = ROcp54_453;
    sens.R(2,2) = ROcp54_553;
    sens.R(2,3) = ROcp54_653;
    sens.R(3,1) = ROcp54_754;
    sens.R(3,2) = ROcp54_854;
    sens.R(3,3) = ROcp54_954;
    sens.V(1) = VIcp54_155;
    sens.V(2) = VIcp54_255;
    sens.V(3) = VIcp54_355;
    sens.OM(1) = OMcp54_154;
    sens.OM(2) = OMcp54_254;
    sens.OM(3) = OMcp54_354;
    sens.A(1) = ACcp54_155;
    sens.A(2) = ACcp54_255;
    sens.A(3) = ACcp54_355;
    sens.OMP(1) = OPcp54_154;
    sens.OMP(2) = OPcp54_254;
    sens.OMP(3) = OPcp54_354;

end


% ====== END Task 1 ====== 

  

