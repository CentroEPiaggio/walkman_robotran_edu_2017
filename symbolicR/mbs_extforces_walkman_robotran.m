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
%	==> Generation Date : Mon Apr  4 18:57:26 2016
%
%	==> Project name : walkman_robotran
%	==> using XML input file 
%
%	==> Number of joints : 55
%
%	==> Function : F19 : External Forces
%	==> Flops complexity : 1722
%
%	==> Generation Time :  0.050 seconds
%	==> Post-Processing :  0.050 seconds
%
%-------------------------------------------------------------
%
function [frc,trq] = extforces(s,tsim,usrfun)

 frc = zeros(3,55);
 trq = zeros(3,55);

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

% = = Block_0_0_1_1_0_1 = = 
 
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
  OMcp22_26 = OMcp22_25+qd(6)*ROcp22_85;
  OMcp22_36 = OMcp22_35+qd(6)*ROcp22_95;
  OPcp22_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp22_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp22_95-OMcp22_35*S5)-qdd(5)*C4-qdd(6)*ROcp22_85);
  OPcp22_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp22_85-OMcp22_25*S5)+qdd(5)*S4+qdd(6)*ROcp22_95;

% = = Block_0_0_1_1_0_2 = = 
 
% Sensor Kinematics 


  ROcp22_47 = ROcp22_46*C7+S5*S7;
  ROcp22_57 = ROcp22_56*C7+ROcp22_85*S7;
  ROcp22_67 = ROcp22_66*C7+ROcp22_95*S7;
  ROcp22_77 = -(ROcp22_46*S7-S5*C7);
  ROcp22_87 = -(ROcp22_56*S7-ROcp22_85*C7);
  ROcp22_97 = -(ROcp22_66*S7-ROcp22_95*C7);
  ROcp22_18 = ROcp22_16*C8+ROcp22_47*S8;
  ROcp22_28 = ROcp22_26*C8+ROcp22_57*S8;
  ROcp22_38 = ROcp22_36*C8+ROcp22_67*S8;
  ROcp22_48 = -(ROcp22_16*S8-ROcp22_47*C8);
  ROcp22_58 = -(ROcp22_26*S8-ROcp22_57*C8);
  ROcp22_68 = -(ROcp22_36*S8-ROcp22_67*C8);
  ROcp22_19 = ROcp22_18*C9-ROcp22_77*S9;
  ROcp22_29 = ROcp22_28*C9-ROcp22_87*S9;
  ROcp22_39 = ROcp22_38*C9-ROcp22_97*S9;
  ROcp22_79 = ROcp22_18*S9+ROcp22_77*C9;
  ROcp22_89 = ROcp22_28*S9+ROcp22_87*C9;
  ROcp22_99 = ROcp22_38*S9+ROcp22_97*C9;
  ROcp22_110 = ROcp22_19*C10-ROcp22_79*S10;
  ROcp22_210 = ROcp22_29*C10-ROcp22_89*S10;
  ROcp22_310 = ROcp22_39*C10-ROcp22_99*S10;
  ROcp22_710 = ROcp22_19*S10+ROcp22_79*C10;
  ROcp22_810 = ROcp22_29*S10+ROcp22_89*C10;
  ROcp22_910 = ROcp22_39*S10+ROcp22_99*C10;
  ROcp22_111 = ROcp22_110*C11-ROcp22_710*S11;
  ROcp22_211 = ROcp22_210*C11-ROcp22_810*S11;
  ROcp22_311 = ROcp22_310*C11-ROcp22_910*S11;
  ROcp22_711 = ROcp22_110*S11+ROcp22_710*C11;
  ROcp22_811 = ROcp22_210*S11+ROcp22_810*C11;
  ROcp22_911 = ROcp22_310*S11+ROcp22_910*C11;
  ROcp22_412 = ROcp22_48*C12+ROcp22_711*S12;
  ROcp22_512 = ROcp22_58*C12+ROcp22_811*S12;
  ROcp22_612 = ROcp22_68*C12+ROcp22_911*S12;
  ROcp22_712 = -(ROcp22_48*S12-ROcp22_711*C12);
  ROcp22_812 = -(ROcp22_58*S12-ROcp22_811*C12);
  ROcp22_912 = -(ROcp22_68*S12-ROcp22_911*C12);
  ROcp22_113 = ROcp22_111*C13+ROcp22_412*S13;
  ROcp22_213 = ROcp22_211*C13+ROcp22_512*S13;
  ROcp22_313 = ROcp22_311*C13+ROcp22_612*S13;
  ROcp22_413 = -(ROcp22_111*S13-ROcp22_412*C13);
  ROcp22_513 = -(ROcp22_211*S13-ROcp22_512*C13);
  ROcp22_613 = -(ROcp22_311*S13-ROcp22_612*C13);
  ROcp22_114 = ROcp22_113*C14-ROcp22_712*S14;
  ROcp22_214 = ROcp22_213*C14-ROcp22_812*S14;
  ROcp22_314 = ROcp22_313*C14-ROcp22_912*S14;
  ROcp22_714 = ROcp22_113*S14+ROcp22_712*C14;
  ROcp22_814 = ROcp22_213*S14+ROcp22_812*C14;
  ROcp22_914 = ROcp22_313*S14+ROcp22_912*C14;
  ROcp22_415 = ROcp22_413*C15+ROcp22_714*S15;
  ROcp22_515 = ROcp22_513*C15+ROcp22_814*S15;
  ROcp22_615 = ROcp22_613*C15+ROcp22_914*S15;
  ROcp22_715 = -(ROcp22_413*S15-ROcp22_714*C15);
  ROcp22_815 = -(ROcp22_513*S15-ROcp22_814*C15);
  ROcp22_915 = -(ROcp22_613*S15-ROcp22_914*C15);
  RLcp22_17 = ROcp22_16*s.dpt(1,1)+ROcp22_46*s.dpt(2,1);
  RLcp22_27 = ROcp22_26*s.dpt(1,1)+ROcp22_56*s.dpt(2,1);
  RLcp22_37 = ROcp22_36*s.dpt(1,1)+ROcp22_66*s.dpt(2,1);
  OMcp22_17 = OMcp22_16+qd(7)*ROcp22_16;
  OMcp22_27 = OMcp22_26+qd(7)*ROcp22_26;
  OMcp22_37 = OMcp22_36+qd(7)*ROcp22_36;
  ORcp22_17 = OMcp22_26*RLcp22_37-OMcp22_36*RLcp22_27;
  ORcp22_27 = -(OMcp22_16*RLcp22_37-OMcp22_36*RLcp22_17);
  ORcp22_37 = OMcp22_16*RLcp22_27-OMcp22_26*RLcp22_17;
  OPcp22_17 = OPcp22_16+qd(7)*(OMcp22_26*ROcp22_36-OMcp22_36*ROcp22_26)+qdd(7)*ROcp22_16;
  OPcp22_27 = OPcp22_26-qd(7)*(OMcp22_16*ROcp22_36-OMcp22_36*ROcp22_16)+qdd(7)*ROcp22_26;
  OPcp22_37 = OPcp22_36+qd(7)*(OMcp22_16*ROcp22_26-OMcp22_26*ROcp22_16)+qdd(7)*ROcp22_36;
  RLcp22_18 = ROcp22_16*s.dpt(1,5)+ROcp22_47*s.dpt(2,5)+ROcp22_77*s.dpt(3,5);
  RLcp22_28 = ROcp22_26*s.dpt(1,5)+ROcp22_57*s.dpt(2,5)+ROcp22_87*s.dpt(3,5);
  RLcp22_38 = ROcp22_36*s.dpt(1,5)+ROcp22_67*s.dpt(2,5)+ROcp22_97*s.dpt(3,5);
  OMcp22_18 = OMcp22_17+qd(8)*ROcp22_77;
  OMcp22_28 = OMcp22_27+qd(8)*ROcp22_87;
  OMcp22_38 = OMcp22_37+qd(8)*ROcp22_97;
  ORcp22_18 = OMcp22_27*RLcp22_38-OMcp22_37*RLcp22_28;
  ORcp22_28 = -(OMcp22_17*RLcp22_38-OMcp22_37*RLcp22_18);
  ORcp22_38 = OMcp22_17*RLcp22_28-OMcp22_27*RLcp22_18;
  OMcp22_19 = OMcp22_18+qd(9)*ROcp22_48;
  OMcp22_29 = OMcp22_28+qd(9)*ROcp22_58;
  OMcp22_39 = OMcp22_38+qd(9)*ROcp22_68;
  OPcp22_19 = OPcp22_17+qd(8)*(OMcp22_27*ROcp22_97-OMcp22_37*ROcp22_87)+qd(9)*(OMcp22_28*ROcp22_68-OMcp22_38*ROcp22_58)+qdd(8)*ROcp22_77+qdd(9)*...
 ROcp22_48;
  OPcp22_29 = OPcp22_27-qd(8)*(OMcp22_17*ROcp22_97-OMcp22_37*ROcp22_77)-qd(9)*(OMcp22_18*ROcp22_68-OMcp22_38*ROcp22_48)+qdd(8)*ROcp22_87+qdd(9)*...
 ROcp22_58;
  OPcp22_39 = OPcp22_37+qd(8)*(OMcp22_17*ROcp22_87-OMcp22_27*ROcp22_77)+qd(9)*(OMcp22_18*ROcp22_58-OMcp22_28*ROcp22_48)+qdd(8)*ROcp22_97+qdd(9)*...
 ROcp22_68;
  RLcp22_110 = ROcp22_79*s.dpt(3,7);
  RLcp22_210 = ROcp22_89*s.dpt(3,7);
  RLcp22_310 = ROcp22_99*s.dpt(3,7);
  OMcp22_110 = OMcp22_19+qd(10)*ROcp22_48;
  OMcp22_210 = OMcp22_29+qd(10)*ROcp22_58;
  OMcp22_310 = OMcp22_39+qd(10)*ROcp22_68;
  ORcp22_110 = OMcp22_29*RLcp22_310-OMcp22_39*RLcp22_210;
  ORcp22_210 = -(OMcp22_19*RLcp22_310-OMcp22_39*RLcp22_110);
  ORcp22_310 = OMcp22_19*RLcp22_210-OMcp22_29*RLcp22_110;
  OPcp22_110 = OPcp22_19+qd(10)*(OMcp22_29*ROcp22_68-OMcp22_39*ROcp22_58)+qdd(10)*ROcp22_48;
  OPcp22_210 = OPcp22_29-qd(10)*(OMcp22_19*ROcp22_68-OMcp22_39*ROcp22_48)+qdd(10)*ROcp22_58;
  OPcp22_310 = OPcp22_39+qd(10)*(OMcp22_19*ROcp22_58-OMcp22_29*ROcp22_48)+qdd(10)*ROcp22_68;
  RLcp22_111 = ROcp22_710*s.dpt(3,8);
  RLcp22_211 = ROcp22_810*s.dpt(3,8);
  RLcp22_311 = ROcp22_910*s.dpt(3,8);
  OMcp22_111 = OMcp22_110+qd(11)*ROcp22_48;
  OMcp22_211 = OMcp22_210+qd(11)*ROcp22_58;
  OMcp22_311 = OMcp22_310+qd(11)*ROcp22_68;
  ORcp22_111 = OMcp22_210*RLcp22_311-OMcp22_310*RLcp22_211;
  ORcp22_211 = -(OMcp22_110*RLcp22_311-OMcp22_310*RLcp22_111);
  ORcp22_311 = OMcp22_110*RLcp22_211-OMcp22_210*RLcp22_111;
  OMcp22_112 = OMcp22_111+qd(12)*ROcp22_111;
  OMcp22_212 = OMcp22_211+qd(12)*ROcp22_211;
  OMcp22_312 = OMcp22_311+qd(12)*ROcp22_311;
  OPcp22_112 = OPcp22_110+qd(11)*(OMcp22_210*ROcp22_68-OMcp22_310*ROcp22_58)+qd(12)*(OMcp22_211*ROcp22_311-OMcp22_311*ROcp22_211)+qdd(11)*...
 ROcp22_48+qdd(12)*ROcp22_111;
  OPcp22_212 = OPcp22_210-qd(11)*(OMcp22_110*ROcp22_68-OMcp22_310*ROcp22_48)-qd(12)*(OMcp22_111*ROcp22_311-OMcp22_311*ROcp22_111)+qdd(11)*...
 ROcp22_58+qdd(12)*ROcp22_211;
  OPcp22_312 = OPcp22_310+qd(11)*(OMcp22_110*ROcp22_58-OMcp22_210*ROcp22_48)+qd(12)*(OMcp22_111*ROcp22_211-OMcp22_211*ROcp22_111)+qdd(11)*...
 ROcp22_68+qdd(12)*ROcp22_311;
  RLcp22_113 = ROcp22_111*s.dpt(1,10)+ROcp22_712*s.dpt(3,10);
  RLcp22_213 = ROcp22_211*s.dpt(1,10)+ROcp22_812*s.dpt(3,10);
  RLcp22_313 = ROcp22_311*s.dpt(1,10)+ROcp22_912*s.dpt(3,10);
  ORcp22_113 = OMcp22_212*RLcp22_313-OMcp22_312*RLcp22_213;
  ORcp22_213 = -(OMcp22_112*RLcp22_313-OMcp22_312*RLcp22_113);
  ORcp22_313 = OMcp22_112*RLcp22_213-OMcp22_212*RLcp22_113;
  RLcp22_116 = q(16)*ROcp22_715;
  RLcp22_216 = q(16)*ROcp22_815;
  RLcp22_316 = q(16)*ROcp22_915;
  ORcp22_116 = OMcp22_212*RLcp22_316-OMcp22_312*RLcp22_216;
  ORcp22_216 = -(OMcp22_112*RLcp22_316-OMcp22_312*RLcp22_116);
  ORcp22_316 = OMcp22_112*RLcp22_216-OMcp22_212*RLcp22_116;
  RLcp22_117 = q(17)*ROcp22_415;
  RLcp22_217 = q(17)*ROcp22_515;
  RLcp22_317 = q(17)*ROcp22_615;
  ORcp22_117 = OMcp22_212*RLcp22_317-OMcp22_312*RLcp22_217;
  ORcp22_217 = -(OMcp22_112*RLcp22_317-OMcp22_312*RLcp22_117);
  ORcp22_317 = OMcp22_112*RLcp22_217-OMcp22_212*RLcp22_117;
  RLcp22_118 = q(18)*ROcp22_114;
  RLcp22_218 = q(18)*ROcp22_214;
  RLcp22_318 = q(18)*ROcp22_314;
  ORcp22_118 = OMcp22_212*RLcp22_318-OMcp22_312*RLcp22_218;
  ORcp22_218 = -(OMcp22_112*RLcp22_318-OMcp22_312*RLcp22_118);
  ORcp22_318 = OMcp22_112*RLcp22_218-OMcp22_212*RLcp22_118;
  RLcp22_178 = ROcp22_715*s.dpt(3,11);
  RLcp22_278 = ROcp22_815*s.dpt(3,11);
  RLcp22_378 = ROcp22_915*s.dpt(3,11);
  ORcp22_178 = OMcp22_212*RLcp22_378-OMcp22_312*RLcp22_278;
  ORcp22_278 = -(OMcp22_112*RLcp22_378-OMcp22_312*RLcp22_178);
  ORcp22_378 = OMcp22_112*RLcp22_278-OMcp22_212*RLcp22_178;
  PxF1(1) = q(1)+RLcp22_110+RLcp22_111+RLcp22_113+RLcp22_116+RLcp22_117+RLcp22_118+RLcp22_17+RLcp22_178+RLcp22_18;
  PxF1(2) = q(2)+RLcp22_210+RLcp22_211+RLcp22_213+RLcp22_216+RLcp22_217+RLcp22_218+RLcp22_27+RLcp22_278+RLcp22_28;
  PxF1(3) = q(3)+RLcp22_310+RLcp22_311+RLcp22_313+RLcp22_316+RLcp22_317+RLcp22_318+RLcp22_37+RLcp22_378+RLcp22_38;
  RxF1(1,1) = ROcp22_114;
  RxF1(1,2) = ROcp22_214;
  RxF1(1,3) = ROcp22_314;
  RxF1(2,1) = ROcp22_415;
  RxF1(2,2) = ROcp22_515;
  RxF1(2,3) = ROcp22_615;
  RxF1(3,1) = ROcp22_715;
  RxF1(3,2) = ROcp22_815;
  RxF1(3,3) = ROcp22_915;
  VxF1(1) = qd(1)+ORcp22_110+ORcp22_111+ORcp22_113+ORcp22_116+ORcp22_117+ORcp22_118+ORcp22_17+ORcp22_178+ORcp22_18;
  VxF1(2) = qd(2)+ORcp22_210+ORcp22_211+ORcp22_213+ORcp22_216+ORcp22_217+ORcp22_218+ORcp22_27+ORcp22_278+ORcp22_28;
  VxF1(3) = qd(3)+ORcp22_310+ORcp22_311+ORcp22_313+ORcp22_316+ORcp22_317+ORcp22_318+ORcp22_37+ORcp22_378+ORcp22_38;
  OMxF1(1) = OMcp22_112;
  OMxF1(2) = OMcp22_212;
  OMxF1(3) = OMcp22_312;
  AxF1(1) = qdd(1)+OMcp22_210*ORcp22_311+OMcp22_212*ORcp22_313+OMcp22_212*ORcp22_316+OMcp22_212*ORcp22_317+OMcp22_212*ORcp22_318+OMcp22_212*...
 ORcp22_378+OMcp22_26*ORcp22_37+OMcp22_27*ORcp22_38+OMcp22_29*ORcp22_310-OMcp22_310*ORcp22_211-OMcp22_312*ORcp22_213-OMcp22_312*ORcp22_216-OMcp22_312*...
 ORcp22_217-OMcp22_312*ORcp22_218-OMcp22_312*ORcp22_278-OMcp22_36*ORcp22_27-OMcp22_37*ORcp22_28-OMcp22_39*ORcp22_210+OPcp22_210*RLcp22_311+OPcp22_212*...
 RLcp22_313+OPcp22_212*RLcp22_316+OPcp22_212*RLcp22_317+OPcp22_212*RLcp22_318+OPcp22_212*RLcp22_378+OPcp22_26*RLcp22_37+OPcp22_27*RLcp22_38+OPcp22_29*...
 RLcp22_310-OPcp22_310*RLcp22_211-OPcp22_312*RLcp22_213-OPcp22_312*RLcp22_216-OPcp22_312*RLcp22_217-OPcp22_312*RLcp22_218-OPcp22_312*RLcp22_278-...
 OPcp22_36*RLcp22_27-OPcp22_37*RLcp22_28-OPcp22_39*RLcp22_210;
  AxF1(2) = qdd(2)-OMcp22_110*ORcp22_311-OMcp22_112*ORcp22_313-OMcp22_112*ORcp22_316-OMcp22_112*ORcp22_317-OMcp22_112*ORcp22_318-OMcp22_112*...
 ORcp22_378-OMcp22_16*ORcp22_37-OMcp22_17*ORcp22_38-OMcp22_19*ORcp22_310+OMcp22_310*ORcp22_111+OMcp22_312*ORcp22_113+OMcp22_312*ORcp22_116+OMcp22_312*...
 ORcp22_117+OMcp22_312*ORcp22_118+OMcp22_312*ORcp22_178+OMcp22_36*ORcp22_17+OMcp22_37*ORcp22_18+OMcp22_39*ORcp22_110-OPcp22_110*RLcp22_311-OPcp22_112*...
 RLcp22_313-OPcp22_112*RLcp22_316-OPcp22_112*RLcp22_317-OPcp22_112*RLcp22_318-OPcp22_112*RLcp22_378-OPcp22_16*RLcp22_37-OPcp22_17*RLcp22_38-OPcp22_19*...
 RLcp22_310+OPcp22_310*RLcp22_111+OPcp22_312*RLcp22_113+OPcp22_312*RLcp22_116+OPcp22_312*RLcp22_117+OPcp22_312*RLcp22_118+OPcp22_312*RLcp22_178+...
 OPcp22_36*RLcp22_17+OPcp22_37*RLcp22_18+OPcp22_39*RLcp22_110;
  AxF1(3) = qdd(3)+OMcp22_110*ORcp22_211+OMcp22_112*ORcp22_213+OMcp22_112*ORcp22_216+OMcp22_112*ORcp22_217+OMcp22_112*ORcp22_218+OMcp22_112*...
 ORcp22_278+OMcp22_16*ORcp22_27+OMcp22_17*ORcp22_28+OMcp22_19*ORcp22_210-OMcp22_210*ORcp22_111-OMcp22_212*ORcp22_113-OMcp22_212*ORcp22_116-OMcp22_212*...
 ORcp22_117-OMcp22_212*ORcp22_118-OMcp22_212*ORcp22_178-OMcp22_26*ORcp22_17-OMcp22_27*ORcp22_18-OMcp22_29*ORcp22_110+OPcp22_110*RLcp22_211+OPcp22_112*...
 RLcp22_213+OPcp22_112*RLcp22_216+OPcp22_112*RLcp22_217+OPcp22_112*RLcp22_218+OPcp22_112*RLcp22_278+OPcp22_16*RLcp22_27+OPcp22_17*RLcp22_28+OPcp22_19*...
 RLcp22_210-OPcp22_210*RLcp22_111-OPcp22_212*RLcp22_113-OPcp22_212*RLcp22_116-OPcp22_212*RLcp22_117-OPcp22_212*RLcp22_118-OPcp22_212*RLcp22_178-...
 OPcp22_26*RLcp22_17-OPcp22_27*RLcp22_18-OPcp22_29*RLcp22_110;
  OMPxF1(1) = OPcp22_112;
  OMPxF1(2) = OPcp22_212;
  OMPxF1(3) = OPcp22_312;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc123 = ROcp22_114*SWr1(1)+ROcp22_214*SWr1(2)+ROcp22_314*SWr1(3);
  xfrc223 = ROcp22_415*SWr1(1)+ROcp22_515*SWr1(2)+ROcp22_615*SWr1(3);
  xfrc323 = ROcp22_715*SWr1(1)+ROcp22_815*SWr1(2)+ROcp22_915*SWr1(3);
  frc(1,18) = s.frc(1,18)+xfrc123;
  frc(2,18) = s.frc(2,18)+xfrc223;
  frc(3,18) = s.frc(3,18)+xfrc323;
  xtrq123 = ROcp22_114*SWr1(4)+ROcp22_214*SWr1(5)+ROcp22_314*SWr1(6);
  xtrq223 = ROcp22_415*SWr1(4)+ROcp22_515*SWr1(5)+ROcp22_615*SWr1(6);
  xtrq323 = ROcp22_715*SWr1(4)+ROcp22_815*SWr1(5)+ROcp22_915*SWr1(6);
  trq(1,18) = s.trq(1,18)+xtrq123-xfrc223*(SWr1(9)-s.l(3,18))+xfrc323*(SWr1(8)-s.l(2,18));
  trq(2,18) = s.trq(2,18)+xtrq223+xfrc123*(SWr1(9)-s.l(3,18))-xfrc323*(SWr1(7)-s.l(1,18));
  trq(3,18) = s.trq(3,18)+xtrq323-xfrc123*(SWr1(8)-s.l(2,18))+xfrc223*(SWr1(7)-s.l(1,18));

% = = Block_0_0_1_2_0_1 = = 
 
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
  OMcp23_26 = OMcp23_25+qd(6)*ROcp23_85;
  OMcp23_36 = OMcp23_35+qd(6)*ROcp23_95;
  OPcp23_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp23_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp23_95-OMcp23_35*S5)-qdd(5)*C4-qdd(6)*ROcp23_85);
  OPcp23_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp23_85-OMcp23_25*S5)+qdd(5)*S4+qdd(6)*ROcp23_95;

% = = Block_0_0_1_2_0_3 = = 
 
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
  ROcp23_125 = ROcp23_123*C25+ROcp23_424*S25;
  ROcp23_225 = ROcp23_223*C25+ROcp23_524*S25;
  ROcp23_325 = ROcp23_323*C25+ROcp23_624*S25;
  ROcp23_425 = -(ROcp23_123*S25-ROcp23_424*C25);
  ROcp23_525 = -(ROcp23_223*S25-ROcp23_524*C25);
  ROcp23_625 = -(ROcp23_323*S25-ROcp23_624*C25);
  ROcp23_126 = ROcp23_125*C26-ROcp23_724*S26;
  ROcp23_226 = ROcp23_225*C26-ROcp23_824*S26;
  ROcp23_326 = ROcp23_325*C26-ROcp23_924*S26;
  ROcp23_726 = ROcp23_125*S26+ROcp23_724*C26;
  ROcp23_826 = ROcp23_225*S26+ROcp23_824*C26;
  ROcp23_926 = ROcp23_325*S26+ROcp23_924*C26;
  ROcp23_427 = ROcp23_425*C27+ROcp23_726*S27;
  ROcp23_527 = ROcp23_525*C27+ROcp23_826*S27;
  ROcp23_627 = ROcp23_625*C27+ROcp23_926*S27;
  ROcp23_727 = -(ROcp23_425*S27-ROcp23_726*C27);
  ROcp23_827 = -(ROcp23_525*S27-ROcp23_826*C27);
  ROcp23_927 = -(ROcp23_625*S27-ROcp23_926*C27);
  RLcp23_119 = ROcp23_16*s.dpt(1,2)+ROcp23_46*s.dpt(2,2);
  RLcp23_219 = ROcp23_26*s.dpt(1,2)+ROcp23_56*s.dpt(2,2);
  RLcp23_319 = ROcp23_36*s.dpt(1,2)+ROcp23_66*s.dpt(2,2);
  OMcp23_119 = OMcp23_16+qd(19)*ROcp23_16;
  OMcp23_219 = OMcp23_26+qd(19)*ROcp23_26;
  OMcp23_319 = OMcp23_36+qd(19)*ROcp23_36;
  ORcp23_119 = OMcp23_26*RLcp23_319-OMcp23_36*RLcp23_219;
  ORcp23_219 = -(OMcp23_16*RLcp23_319-OMcp23_36*RLcp23_119);
  ORcp23_319 = OMcp23_16*RLcp23_219-OMcp23_26*RLcp23_119;
  OPcp23_119 = OPcp23_16+qd(19)*(OMcp23_26*ROcp23_36-OMcp23_36*ROcp23_26)+qdd(19)*ROcp23_16;
  OPcp23_219 = OPcp23_26-qd(19)*(OMcp23_16*ROcp23_36-OMcp23_36*ROcp23_16)+qdd(19)*ROcp23_26;
  OPcp23_319 = OPcp23_36+qd(19)*(OMcp23_16*ROcp23_26-OMcp23_26*ROcp23_16)+qdd(19)*ROcp23_36;
  RLcp23_120 = ROcp23_16*s.dpt(1,12)+ROcp23_419*s.dpt(2,12)+ROcp23_719*s.dpt(3,12);
  RLcp23_220 = ROcp23_26*s.dpt(1,12)+ROcp23_519*s.dpt(2,12)+ROcp23_819*s.dpt(3,12);
  RLcp23_320 = ROcp23_36*s.dpt(1,12)+ROcp23_619*s.dpt(2,12)+ROcp23_919*s.dpt(3,12);
  OMcp23_120 = OMcp23_119+qd(20)*ROcp23_719;
  OMcp23_220 = OMcp23_219+qd(20)*ROcp23_819;
  OMcp23_320 = OMcp23_319+qd(20)*ROcp23_919;
  ORcp23_120 = OMcp23_219*RLcp23_320-OMcp23_319*RLcp23_220;
  ORcp23_220 = -(OMcp23_119*RLcp23_320-OMcp23_319*RLcp23_120);
  ORcp23_320 = OMcp23_119*RLcp23_220-OMcp23_219*RLcp23_120;
  OMcp23_121 = OMcp23_120+qd(21)*ROcp23_420;
  OMcp23_221 = OMcp23_220+qd(21)*ROcp23_520;
  OMcp23_321 = OMcp23_320+qd(21)*ROcp23_620;
  OPcp23_121 = OPcp23_119+qd(20)*(OMcp23_219*ROcp23_919-OMcp23_319*ROcp23_819)+qd(21)*(OMcp23_220*ROcp23_620-OMcp23_320*ROcp23_520)+qdd(20)*...
 ROcp23_719+qdd(21)*ROcp23_420;
  OPcp23_221 = OPcp23_219-qd(20)*(OMcp23_119*ROcp23_919-OMcp23_319*ROcp23_719)-qd(21)*(OMcp23_120*ROcp23_620-OMcp23_320*ROcp23_420)+qdd(20)*...
 ROcp23_819+qdd(21)*ROcp23_520;
  OPcp23_321 = OPcp23_319+qd(20)*(OMcp23_119*ROcp23_819-OMcp23_219*ROcp23_719)+qd(21)*(OMcp23_120*ROcp23_520-OMcp23_220*ROcp23_420)+qdd(20)*...
 ROcp23_919+qdd(21)*ROcp23_620;
  RLcp23_122 = ROcp23_721*s.dpt(3,14);
  RLcp23_222 = ROcp23_821*s.dpt(3,14);
  RLcp23_322 = ROcp23_921*s.dpt(3,14);
  OMcp23_122 = OMcp23_121+qd(22)*ROcp23_420;
  OMcp23_222 = OMcp23_221+qd(22)*ROcp23_520;
  OMcp23_322 = OMcp23_321+qd(22)*ROcp23_620;
  ORcp23_122 = OMcp23_221*RLcp23_322-OMcp23_321*RLcp23_222;
  ORcp23_222 = -(OMcp23_121*RLcp23_322-OMcp23_321*RLcp23_122);
  ORcp23_322 = OMcp23_121*RLcp23_222-OMcp23_221*RLcp23_122;
  OPcp23_122 = OPcp23_121+qd(22)*(OMcp23_221*ROcp23_620-OMcp23_321*ROcp23_520)+qdd(22)*ROcp23_420;
  OPcp23_222 = OPcp23_221-qd(22)*(OMcp23_121*ROcp23_620-OMcp23_321*ROcp23_420)+qdd(22)*ROcp23_520;
  OPcp23_322 = OPcp23_321+qd(22)*(OMcp23_121*ROcp23_520-OMcp23_221*ROcp23_420)+qdd(22)*ROcp23_620;
  RLcp23_123 = ROcp23_722*s.dpt(3,15);
  RLcp23_223 = ROcp23_822*s.dpt(3,15);
  RLcp23_323 = ROcp23_922*s.dpt(3,15);
  OMcp23_123 = OMcp23_122+qd(23)*ROcp23_420;
  OMcp23_223 = OMcp23_222+qd(23)*ROcp23_520;
  OMcp23_323 = OMcp23_322+qd(23)*ROcp23_620;
  ORcp23_123 = OMcp23_222*RLcp23_323-OMcp23_322*RLcp23_223;
  ORcp23_223 = -(OMcp23_122*RLcp23_323-OMcp23_322*RLcp23_123);
  ORcp23_323 = OMcp23_122*RLcp23_223-OMcp23_222*RLcp23_123;
  OMcp23_124 = OMcp23_123+qd(24)*ROcp23_123;
  OMcp23_224 = OMcp23_223+qd(24)*ROcp23_223;
  OMcp23_324 = OMcp23_323+qd(24)*ROcp23_323;
  OPcp23_124 = OPcp23_122+qd(23)*(OMcp23_222*ROcp23_620-OMcp23_322*ROcp23_520)+qd(24)*(OMcp23_223*ROcp23_323-OMcp23_323*ROcp23_223)+qdd(23)*...
 ROcp23_420+qdd(24)*ROcp23_123;
  OPcp23_224 = OPcp23_222-qd(23)*(OMcp23_122*ROcp23_620-OMcp23_322*ROcp23_420)-qd(24)*(OMcp23_123*ROcp23_323-OMcp23_323*ROcp23_123)+qdd(23)*...
 ROcp23_520+qdd(24)*ROcp23_223;
  OPcp23_324 = OPcp23_322+qd(23)*(OMcp23_122*ROcp23_520-OMcp23_222*ROcp23_420)+qd(24)*(OMcp23_123*ROcp23_223-OMcp23_223*ROcp23_123)+qdd(23)*...
 ROcp23_620+qdd(24)*ROcp23_323;
  RLcp23_125 = ROcp23_123*s.dpt(1,17)+ROcp23_724*s.dpt(3,17);
  RLcp23_225 = ROcp23_223*s.dpt(1,17)+ROcp23_824*s.dpt(3,17);
  RLcp23_325 = ROcp23_323*s.dpt(1,17)+ROcp23_924*s.dpt(3,17);
  ORcp23_125 = OMcp23_224*RLcp23_325-OMcp23_324*RLcp23_225;
  ORcp23_225 = -(OMcp23_124*RLcp23_325-OMcp23_324*RLcp23_125);
  ORcp23_325 = OMcp23_124*RLcp23_225-OMcp23_224*RLcp23_125;
  RLcp23_128 = q(28)*ROcp23_727;
  RLcp23_228 = q(28)*ROcp23_827;
  RLcp23_328 = q(28)*ROcp23_927;
  ORcp23_128 = OMcp23_224*RLcp23_328-OMcp23_324*RLcp23_228;
  ORcp23_228 = -(OMcp23_124*RLcp23_328-OMcp23_324*RLcp23_128);
  ORcp23_328 = OMcp23_124*RLcp23_228-OMcp23_224*RLcp23_128;
  RLcp23_129 = q(29)*ROcp23_427;
  RLcp23_229 = q(29)*ROcp23_527;
  RLcp23_329 = q(29)*ROcp23_627;
  ORcp23_129 = OMcp23_224*RLcp23_329-OMcp23_324*RLcp23_229;
  ORcp23_229 = -(OMcp23_124*RLcp23_329-OMcp23_324*RLcp23_129);
  ORcp23_329 = OMcp23_124*RLcp23_229-OMcp23_224*RLcp23_129;
  RLcp23_130 = q(30)*ROcp23_126;
  RLcp23_230 = q(30)*ROcp23_226;
  RLcp23_330 = q(30)*ROcp23_326;
  ORcp23_130 = OMcp23_224*RLcp23_330-OMcp23_324*RLcp23_230;
  ORcp23_230 = -(OMcp23_124*RLcp23_330-OMcp23_324*RLcp23_130);
  ORcp23_330 = OMcp23_124*RLcp23_230-OMcp23_224*RLcp23_130;
  RLcp23_179 = ROcp23_727*s.dpt(3,18);
  RLcp23_279 = ROcp23_827*s.dpt(3,18);
  RLcp23_379 = ROcp23_927*s.dpt(3,18);
  ORcp23_179 = OMcp23_224*RLcp23_379-OMcp23_324*RLcp23_279;
  ORcp23_279 = -(OMcp23_124*RLcp23_379-OMcp23_324*RLcp23_179);
  ORcp23_379 = OMcp23_124*RLcp23_279-OMcp23_224*RLcp23_179;
  PxF2(1) = q(1)+RLcp23_119+RLcp23_120+RLcp23_122+RLcp23_123+RLcp23_125+RLcp23_128+RLcp23_129+RLcp23_130+RLcp23_179;
  PxF2(2) = q(2)+RLcp23_219+RLcp23_220+RLcp23_222+RLcp23_223+RLcp23_225+RLcp23_228+RLcp23_229+RLcp23_230+RLcp23_279;
  PxF2(3) = q(3)+RLcp23_319+RLcp23_320+RLcp23_322+RLcp23_323+RLcp23_325+RLcp23_328+RLcp23_329+RLcp23_330+RLcp23_379;
  RxF2(1,1) = ROcp23_126;
  RxF2(1,2) = ROcp23_226;
  RxF2(1,3) = ROcp23_326;
  RxF2(2,1) = ROcp23_427;
  RxF2(2,2) = ROcp23_527;
  RxF2(2,3) = ROcp23_627;
  RxF2(3,1) = ROcp23_727;
  RxF2(3,2) = ROcp23_827;
  RxF2(3,3) = ROcp23_927;
  VxF2(1) = qd(1)+ORcp23_119+ORcp23_120+ORcp23_122+ORcp23_123+ORcp23_125+ORcp23_128+ORcp23_129+ORcp23_130+ORcp23_179;
  VxF2(2) = qd(2)+ORcp23_219+ORcp23_220+ORcp23_222+ORcp23_223+ORcp23_225+ORcp23_228+ORcp23_229+ORcp23_230+ORcp23_279;
  VxF2(3) = qd(3)+ORcp23_319+ORcp23_320+ORcp23_322+ORcp23_323+ORcp23_325+ORcp23_328+ORcp23_329+ORcp23_330+ORcp23_379;
  OMxF2(1) = OMcp23_124;
  OMxF2(2) = OMcp23_224;
  OMxF2(3) = OMcp23_324;
  AxF2(1) = qdd(1)+OMcp23_219*ORcp23_320+OMcp23_221*ORcp23_322+OMcp23_222*ORcp23_323+OMcp23_224*ORcp23_325+OMcp23_224*ORcp23_328+OMcp23_224*...
 ORcp23_329+OMcp23_224*ORcp23_330+OMcp23_224*ORcp23_379+OMcp23_26*ORcp23_319-OMcp23_319*ORcp23_220-OMcp23_321*ORcp23_222-OMcp23_322*ORcp23_223-...
 OMcp23_324*ORcp23_225-OMcp23_324*ORcp23_228-OMcp23_324*ORcp23_229-OMcp23_324*ORcp23_230-OMcp23_324*ORcp23_279-OMcp23_36*ORcp23_219+OPcp23_219*...
 RLcp23_320+OPcp23_221*RLcp23_322+OPcp23_222*RLcp23_323+OPcp23_224*RLcp23_325+OPcp23_224*RLcp23_328+OPcp23_224*RLcp23_329+OPcp23_224*RLcp23_330+...
 OPcp23_224*RLcp23_379+OPcp23_26*RLcp23_319-OPcp23_319*RLcp23_220-OPcp23_321*RLcp23_222-OPcp23_322*RLcp23_223-OPcp23_324*RLcp23_225-OPcp23_324*...
 RLcp23_228-OPcp23_324*RLcp23_229-OPcp23_324*RLcp23_230-OPcp23_324*RLcp23_279-OPcp23_36*RLcp23_219;
  AxF2(2) = qdd(2)-OMcp23_119*ORcp23_320-OMcp23_121*ORcp23_322-OMcp23_122*ORcp23_323-OMcp23_124*ORcp23_325-OMcp23_124*ORcp23_328-OMcp23_124*...
 ORcp23_329-OMcp23_124*ORcp23_330-OMcp23_124*ORcp23_379-OMcp23_16*ORcp23_319+OMcp23_319*ORcp23_120+OMcp23_321*ORcp23_122+OMcp23_322*ORcp23_123+...
 OMcp23_324*ORcp23_125+OMcp23_324*ORcp23_128+OMcp23_324*ORcp23_129+OMcp23_324*ORcp23_130+OMcp23_324*ORcp23_179+OMcp23_36*ORcp23_119-OPcp23_119*...
 RLcp23_320-OPcp23_121*RLcp23_322-OPcp23_122*RLcp23_323-OPcp23_124*RLcp23_325-OPcp23_124*RLcp23_328-OPcp23_124*RLcp23_329-OPcp23_124*RLcp23_330-...
 OPcp23_124*RLcp23_379-OPcp23_16*RLcp23_319+OPcp23_319*RLcp23_120+OPcp23_321*RLcp23_122+OPcp23_322*RLcp23_123+OPcp23_324*RLcp23_125+OPcp23_324*...
 RLcp23_128+OPcp23_324*RLcp23_129+OPcp23_324*RLcp23_130+OPcp23_324*RLcp23_179+OPcp23_36*RLcp23_119;
  AxF2(3) = qdd(3)+OMcp23_119*ORcp23_220+OMcp23_121*ORcp23_222+OMcp23_122*ORcp23_223+OMcp23_124*ORcp23_225+OMcp23_124*ORcp23_228+OMcp23_124*...
 ORcp23_229+OMcp23_124*ORcp23_230+OMcp23_124*ORcp23_279+OMcp23_16*ORcp23_219-OMcp23_219*ORcp23_120-OMcp23_221*ORcp23_122-OMcp23_222*ORcp23_123-...
 OMcp23_224*ORcp23_125-OMcp23_224*ORcp23_128-OMcp23_224*ORcp23_129-OMcp23_224*ORcp23_130-OMcp23_224*ORcp23_179-OMcp23_26*ORcp23_119+OPcp23_119*...
 RLcp23_220+OPcp23_121*RLcp23_222+OPcp23_122*RLcp23_223+OPcp23_124*RLcp23_225+OPcp23_124*RLcp23_228+OPcp23_124*RLcp23_229+OPcp23_124*RLcp23_230+...
 OPcp23_124*RLcp23_279+OPcp23_16*RLcp23_219-OPcp23_219*RLcp23_120-OPcp23_221*RLcp23_122-OPcp23_222*RLcp23_123-OPcp23_224*RLcp23_125-OPcp23_224*...
 RLcp23_128-OPcp23_224*RLcp23_129-OPcp23_224*RLcp23_130-OPcp23_224*RLcp23_179-OPcp23_26*RLcp23_119;
  OMPxF2(1) = OPcp23_124;
  OMPxF2(2) = OPcp23_224;
  OMPxF2(3) = OPcp23_324;
 
% Sensor Forces Computation 

  SWr2 = usrfun.fext(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc124 = ROcp23_126*SWr2(1)+ROcp23_226*SWr2(2)+ROcp23_326*SWr2(3);
  xfrc224 = ROcp23_427*SWr2(1)+ROcp23_527*SWr2(2)+ROcp23_627*SWr2(3);
  xfrc324 = ROcp23_727*SWr2(1)+ROcp23_827*SWr2(2)+ROcp23_927*SWr2(3);
  frc(1,30) = s.frc(1,30)+xfrc124;
  frc(2,30) = s.frc(2,30)+xfrc224;
  frc(3,30) = s.frc(3,30)+xfrc324;
  xtrq124 = ROcp23_126*SWr2(4)+ROcp23_226*SWr2(5)+ROcp23_326*SWr2(6);
  xtrq224 = ROcp23_427*SWr2(4)+ROcp23_527*SWr2(5)+ROcp23_627*SWr2(6);
  xtrq324 = ROcp23_727*SWr2(4)+ROcp23_827*SWr2(5)+ROcp23_927*SWr2(6);
  trq(1,30) = s.trq(1,30)+xtrq124-xfrc224*(SWr2(9)-s.l(3,30))+xfrc324*(SWr2(8)-s.l(2,30));
  trq(2,30) = s.trq(2,30)+xtrq224+xfrc124*(SWr2(9)-s.l(3,30))-xfrc324*(SWr2(7)-s.l(1,30));
  trq(3,30) = s.trq(3,30)+xtrq324-xfrc124*(SWr2(8)-s.l(2,30))+xfrc224*(SWr2(7)-s.l(1,30));

% = = Block_0_0_1_2_1_0 = = 
 
% Symbolic Outputs  

  frc(1,6) = s.frc(1,6);
  frc(2,6) = s.frc(2,6);
  frc(3,6) = s.frc(3,6);
  frc(1,7) = s.frc(1,7);
  frc(2,7) = s.frc(2,7);
  frc(3,7) = s.frc(3,7);
  frc(1,8) = s.frc(1,8);
  frc(2,8) = s.frc(2,8);
  frc(3,8) = s.frc(3,8);
  frc(1,9) = s.frc(1,9);
  frc(2,9) = s.frc(2,9);
  frc(3,9) = s.frc(3,9);
  frc(1,10) = s.frc(1,10);
  frc(2,10) = s.frc(2,10);
  frc(3,10) = s.frc(3,10);
  frc(1,11) = s.frc(1,11);
  frc(2,11) = s.frc(2,11);
  frc(3,11) = s.frc(3,11);
  frc(1,12) = s.frc(1,12);
  frc(2,12) = s.frc(2,12);
  frc(3,12) = s.frc(3,12);
  frc(1,19) = s.frc(1,19);
  frc(2,19) = s.frc(2,19);
  frc(3,19) = s.frc(3,19);
  frc(1,20) = s.frc(1,20);
  frc(2,20) = s.frc(2,20);
  frc(3,20) = s.frc(3,20);
  frc(1,21) = s.frc(1,21);
  frc(2,21) = s.frc(2,21);
  frc(3,21) = s.frc(3,21);
  frc(1,22) = s.frc(1,22);
  frc(2,22) = s.frc(2,22);
  frc(3,22) = s.frc(3,22);
  frc(1,23) = s.frc(1,23);
  frc(2,23) = s.frc(2,23);
  frc(3,23) = s.frc(3,23);
  frc(1,24) = s.frc(1,24);
  frc(2,24) = s.frc(2,24);
  frc(3,24) = s.frc(3,24);
  frc(1,32) = s.frc(1,32);
  frc(2,32) = s.frc(2,32);
  frc(3,32) = s.frc(3,32);
  frc(1,33) = s.frc(1,33);
  frc(2,33) = s.frc(2,33);
  frc(3,33) = s.frc(3,33);
  frc(1,34) = s.frc(1,34);
  frc(2,34) = s.frc(2,34);
  frc(3,34) = s.frc(3,34);
  frc(1,37) = s.frc(1,37);
  frc(2,37) = s.frc(2,37);
  frc(3,37) = s.frc(3,37);
  frc(1,38) = s.frc(1,38);
  frc(2,38) = s.frc(2,38);
  frc(3,38) = s.frc(3,38);
  frc(1,39) = s.frc(1,39);
  frc(2,39) = s.frc(2,39);
  frc(3,39) = s.frc(3,39);
  frc(1,40) = s.frc(1,40);
  frc(2,40) = s.frc(2,40);
  frc(3,40) = s.frc(3,40);
  frc(1,41) = s.frc(1,41);
  frc(2,41) = s.frc(2,41);
  frc(3,41) = s.frc(3,41);
  frc(1,42) = s.frc(1,42);
  frc(2,42) = s.frc(2,42);
  frc(3,42) = s.frc(3,42);
  frc(1,43) = s.frc(1,43);
  frc(2,43) = s.frc(2,43);
  frc(3,43) = s.frc(3,43);
  frc(1,46) = s.frc(1,46);
  frc(2,46) = s.frc(2,46);
  frc(3,46) = s.frc(3,46);
  frc(1,47) = s.frc(1,47);
  frc(2,47) = s.frc(2,47);
  frc(3,47) = s.frc(3,47);
  frc(1,48) = s.frc(1,48);
  frc(2,48) = s.frc(2,48);
  frc(3,48) = s.frc(3,48);
  frc(1,49) = s.frc(1,49);
  frc(2,49) = s.frc(2,49);
  frc(3,49) = s.frc(3,49);
  frc(1,50) = s.frc(1,50);
  frc(2,50) = s.frc(2,50);
  frc(3,50) = s.frc(3,50);
  frc(1,51) = s.frc(1,51);
  frc(2,51) = s.frc(2,51);
  frc(3,51) = s.frc(3,51);
  frc(1,52) = s.frc(1,52);
  frc(2,52) = s.frc(2,52);
  frc(3,52) = s.frc(3,52);
  frc(1,53) = s.frc(1,53);
  frc(2,53) = s.frc(2,53);
  frc(3,53) = s.frc(3,53);
  frc(1,54) = s.frc(1,54);
  frc(2,54) = s.frc(2,54);
  frc(3,54) = s.frc(3,54);
  frc(1,55) = s.frc(1,55);
  frc(2,55) = s.frc(2,55);
  frc(3,55) = s.frc(3,55);
  trq(1,6) = s.trq(1,6);
  trq(2,6) = s.trq(2,6);
  trq(3,6) = s.trq(3,6);
  trq(1,7) = s.trq(1,7);
  trq(2,7) = s.trq(2,7);
  trq(3,7) = s.trq(3,7);
  trq(1,8) = s.trq(1,8);
  trq(2,8) = s.trq(2,8);
  trq(3,8) = s.trq(3,8);
  trq(1,9) = s.trq(1,9);
  trq(2,9) = s.trq(2,9);
  trq(3,9) = s.trq(3,9);
  trq(1,10) = s.trq(1,10);
  trq(2,10) = s.trq(2,10);
  trq(3,10) = s.trq(3,10);
  trq(1,11) = s.trq(1,11);
  trq(2,11) = s.trq(2,11);
  trq(3,11) = s.trq(3,11);
  trq(1,12) = s.trq(1,12);
  trq(2,12) = s.trq(2,12);
  trq(3,12) = s.trq(3,12);
  trq(1,19) = s.trq(1,19);
  trq(2,19) = s.trq(2,19);
  trq(3,19) = s.trq(3,19);
  trq(1,20) = s.trq(1,20);
  trq(2,20) = s.trq(2,20);
  trq(3,20) = s.trq(3,20);
  trq(1,21) = s.trq(1,21);
  trq(2,21) = s.trq(2,21);
  trq(3,21) = s.trq(3,21);
  trq(1,22) = s.trq(1,22);
  trq(2,22) = s.trq(2,22);
  trq(3,22) = s.trq(3,22);
  trq(1,23) = s.trq(1,23);
  trq(2,23) = s.trq(2,23);
  trq(3,23) = s.trq(3,23);
  trq(1,24) = s.trq(1,24);
  trq(2,24) = s.trq(2,24);
  trq(3,24) = s.trq(3,24);
  trq(1,32) = s.trq(1,32);
  trq(2,32) = s.trq(2,32);
  trq(3,32) = s.trq(3,32);
  trq(1,33) = s.trq(1,33);
  trq(2,33) = s.trq(2,33);
  trq(3,33) = s.trq(3,33);
  trq(1,34) = s.trq(1,34);
  trq(2,34) = s.trq(2,34);
  trq(3,34) = s.trq(3,34);
  trq(1,37) = s.trq(1,37);
  trq(2,37) = s.trq(2,37);
  trq(3,37) = s.trq(3,37);
  trq(1,38) = s.trq(1,38);
  trq(2,38) = s.trq(2,38);
  trq(3,38) = s.trq(3,38);
  trq(1,39) = s.trq(1,39);
  trq(2,39) = s.trq(2,39);
  trq(3,39) = s.trq(3,39);
  trq(1,40) = s.trq(1,40);
  trq(2,40) = s.trq(2,40);
  trq(3,40) = s.trq(3,40);
  trq(1,41) = s.trq(1,41);
  trq(2,41) = s.trq(2,41);
  trq(3,41) = s.trq(3,41);
  trq(1,42) = s.trq(1,42);
  trq(2,42) = s.trq(2,42);
  trq(3,42) = s.trq(3,42);
  trq(1,43) = s.trq(1,43);
  trq(2,43) = s.trq(2,43);
  trq(3,43) = s.trq(3,43);
  trq(1,46) = s.trq(1,46);
  trq(2,46) = s.trq(2,46);
  trq(3,46) = s.trq(3,46);
  trq(1,47) = s.trq(1,47);
  trq(2,47) = s.trq(2,47);
  trq(3,47) = s.trq(3,47);
  trq(1,48) = s.trq(1,48);
  trq(2,48) = s.trq(2,48);
  trq(3,48) = s.trq(3,48);
  trq(1,49) = s.trq(1,49);
  trq(2,49) = s.trq(2,49);
  trq(3,49) = s.trq(3,49);
  trq(1,50) = s.trq(1,50);
  trq(2,50) = s.trq(2,50);
  trq(3,50) = s.trq(3,50);
  trq(1,51) = s.trq(1,51);
  trq(2,51) = s.trq(2,51);
  trq(3,51) = s.trq(3,51);
  trq(1,52) = s.trq(1,52);
  trq(2,52) = s.trq(2,52);
  trq(3,52) = s.trq(3,52);
  trq(1,53) = s.trq(1,53);
  trq(2,53) = s.trq(2,53);
  trq(3,53) = s.trq(3,53);
  trq(1,54) = s.trq(1,54);
  trq(2,54) = s.trq(2,54);
  trq(3,54) = s.trq(3,54);
  trq(1,55) = s.trq(1,55);
  trq(2,55) = s.trq(2,55);
  trq(3,55) = s.trq(3,55);

% ====== END Task 0 ====== 

  

