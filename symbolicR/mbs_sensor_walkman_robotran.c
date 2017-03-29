//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Thu Oct 29 11:32:55 2015
//
//	==> Project name : walkman_robotran
//	==> using XML input file 
//
//	==> Number of joints : 55
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 26236
//
//	==> Generation Time :  0.440 seconds
//	==> Post-Processing :  0.620 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void  sensor(MBSsensorStruct *sens, 
              MBSdataStruct *s,
              int isens)
{ 
 
#include "mbs_sensor_walkman_robotran.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

  C7 = cos(q[7]);
  S7 = sin(q[7]);
  C8 = cos(q[8]);
  S8 = sin(q[8]);
  C9 = cos(q[9]);
  S9 = sin(q[9]);
  C10 = cos(q[10]);
  S10 = sin(q[10]);
  C11 = cos(q[11]);
  S11 = sin(q[11]);
  C12 = cos(q[12]);
  S12 = sin(q[12]);
  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);
  C21 = cos(q[21]);
  S21 = sin(q[21]);
  C22 = cos(q[22]);
  S22 = sin(q[22]);
  C23 = cos(q[23]);
  S23 = sin(q[23]);
  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C31 = cos(q[31]);
  S31 = sin(q[31]);
  C32 = cos(q[32]);
  S32 = sin(q[32]);
  C33 = cos(q[33]);
  S33 = sin(q[33]);
  C34 = cos(q[34]);
  S34 = sin(q[34]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C35 = cos(q[35]);
  S35 = sin(q[35]);
  C36 = cos(q[36]);
  S36 = sin(q[36]);
  C37 = cos(q[37]);
  S37 = sin(q[37]);
  C38 = cos(q[38]);
  S38 = sin(q[38]);
  C39 = cos(q[39]);
  S39 = sin(q[39]);
  C40 = cos(q[40]);
  S40 = sin(q[40]);
  C41 = cos(q[41]);
  S41 = sin(q[41]);
  C42 = cos(q[42]);
  S42 = sin(q[42]);
  C43 = cos(q[43]);
  S43 = sin(q[43]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C44 = cos(q[44]);
  S44 = sin(q[44]);
  C45 = cos(q[45]);
  S45 = sin(q[45]);
  C46 = cos(q[46]);
  S46 = sin(q[46]);
  C47 = cos(q[47]);
  S47 = sin(q[47]);
  C48 = cos(q[48]);
  S48 = sin(q[48]);
  C49 = cos(q[49]);
  S49 = sin(q[49]);
  C50 = cos(q[50]);
  S50 = sin(q[50]);
  C51 = cos(q[51]);
  S51 = sin(q[51]);
  C52 = cos(q[52]);
  S52 = sin(q[52]);

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_0_1 = = 
 
// Sensor Kinematics 


    ROcp0_25 = S4*S5;
    ROcp0_35 = -C4*S5;
    ROcp0_85 = -S4*C5;
    ROcp0_95 = C4*C5;
    ROcp0_16 = C5*C6;
    ROcp0_26 = ROcp0_25*C6+C4*S6;
    ROcp0_36 = ROcp0_35*C6+S4*S6;
    ROcp0_46 = -C5*S6;
    ROcp0_56 = -(ROcp0_25*S6-C4*C6);
    ROcp0_66 = -(ROcp0_35*S6-S4*C6);
    OMcp0_25 = qd[5]*C4;
    OMcp0_35 = qd[5]*S4;
    OMcp0_16 = qd[4]+qd[6]*S5;
    OMcp0_26 = OMcp0_25+ROcp0_85*qd[6];
    OMcp0_36 = OMcp0_35+ROcp0_95*qd[6];
    OPcp0_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp0_26 = ROcp0_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp0_35*S5-ROcp0_95*qd[4]);
    OPcp0_36 = ROcp0_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp0_25*S5-ROcp0_85*qd[4]);
    RLcp0_156 = ROcp0_16*s->dpt[1][4]+ROcp0_46*s->dpt[2][4]+s->dpt[3][4]*S5;
    RLcp0_256 = ROcp0_26*s->dpt[1][4]+ROcp0_56*s->dpt[2][4]+ROcp0_85*s->dpt[3][4];
    RLcp0_356 = ROcp0_36*s->dpt[1][4]+ROcp0_66*s->dpt[2][4]+ROcp0_95*s->dpt[3][4];
    POcp0_156 = RLcp0_156+q[1];
    POcp0_256 = RLcp0_256+q[2];
    POcp0_356 = RLcp0_356+q[3];
    JTcp0_156_5 = -(RLcp0_256*S4-RLcp0_356*C4);
    JTcp0_256_5 = RLcp0_156*S4;
    JTcp0_356_5 = -RLcp0_156*C4;
    JTcp0_156_6 = -(RLcp0_256*ROcp0_95-RLcp0_356*ROcp0_85);
    JTcp0_256_6 = RLcp0_156*ROcp0_95-RLcp0_356*S5;
    JTcp0_356_6 = -(RLcp0_156*ROcp0_85-RLcp0_256*S5);
    ORcp0_156 = OMcp0_26*RLcp0_356-OMcp0_36*RLcp0_256;
    ORcp0_256 = -(OMcp0_16*RLcp0_356-OMcp0_36*RLcp0_156);
    ORcp0_356 = OMcp0_16*RLcp0_256-OMcp0_26*RLcp0_156;
    VIcp0_156 = ORcp0_156+qd[1];
    VIcp0_256 = ORcp0_256+qd[2];
    VIcp0_356 = ORcp0_356+qd[3];
    ACcp0_156 = qdd[1]+OMcp0_26*ORcp0_356-OMcp0_36*ORcp0_256+OPcp0_26*RLcp0_356-OPcp0_36*RLcp0_256;
    ACcp0_256 = qdd[2]-OMcp0_16*ORcp0_356+OMcp0_36*ORcp0_156-OPcp0_16*RLcp0_356+OPcp0_36*RLcp0_156;
    ACcp0_356 = qdd[3]+OMcp0_16*ORcp0_256-OMcp0_26*ORcp0_156+OPcp0_16*RLcp0_256-OPcp0_26*RLcp0_156;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_156;
    sens->P[2] = POcp0_256;
    sens->P[3] = POcp0_356;
    sens->R[1][1] = ROcp0_16;
    sens->R[1][2] = ROcp0_26;
    sens->R[1][3] = ROcp0_36;
    sens->R[2][1] = ROcp0_46;
    sens->R[2][2] = ROcp0_56;
    sens->R[2][3] = ROcp0_66;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp0_85;
    sens->R[3][3] = ROcp0_95;
    sens->V[1] = VIcp0_156;
    sens->V[2] = VIcp0_256;
    sens->V[3] = VIcp0_356;
    sens->OM[1] = OMcp0_16;
    sens->OM[2] = OMcp0_26;
    sens->OM[3] = OMcp0_36;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp0_156_5;
    sens->J[1][6] = JTcp0_156_6;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = -RLcp0_356;
    sens->J[2][5] = JTcp0_256_5;
    sens->J[2][6] = JTcp0_256_6;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = RLcp0_256;
    sens->J[3][5] = JTcp0_356_5;
    sens->J[3][6] = JTcp0_356_6;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp0_85;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp0_95;
    sens->A[1] = ACcp0_156;
    sens->A[2] = ACcp0_256;
    sens->A[3] = ACcp0_356;
    sens->OMP[1] = OPcp0_16;
    sens->OMP[2] = OPcp0_26;
    sens->OMP[3] = OPcp0_36;
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_0_1 = = 
 
// Sensor Kinematics 


    ROcp1_25 = S4*S5;
    ROcp1_35 = -C4*S5;
    ROcp1_85 = -S4*C5;
    ROcp1_95 = C4*C5;
    ROcp1_16 = C5*C6;
    ROcp1_26 = ROcp1_25*C6+C4*S6;
    ROcp1_36 = ROcp1_35*C6+S4*S6;
    ROcp1_46 = -C5*S6;
    ROcp1_56 = -(ROcp1_25*S6-C4*C6);
    ROcp1_66 = -(ROcp1_35*S6-S4*C6);
    OMcp1_25 = qd[5]*C4;
    OMcp1_35 = qd[5]*S4;
    OMcp1_16 = qd[4]+qd[6]*S5;
    OMcp1_26 = OMcp1_25+ROcp1_85*qd[6];
    OMcp1_36 = OMcp1_35+ROcp1_95*qd[6];
    OPcp1_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp1_26 = ROcp1_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp1_35*S5-ROcp1_95*qd[4]);
    OPcp1_36 = ROcp1_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp1_25*S5-ROcp1_85*qd[4]);

// = = Block_1_0_0_2_0_2 = = 
 
// Sensor Kinematics 


    ROcp1_47 = ROcp1_46*C7+S5*S7;
    ROcp1_57 = ROcp1_56*C7+ROcp1_85*S7;
    ROcp1_67 = ROcp1_66*C7+ROcp1_95*S7;
    ROcp1_77 = -(ROcp1_46*S7-S5*C7);
    ROcp1_87 = -(ROcp1_56*S7-ROcp1_85*C7);
    ROcp1_97 = -(ROcp1_66*S7-ROcp1_95*C7);
    ROcp1_18 = ROcp1_16*C8+ROcp1_47*S8;
    ROcp1_28 = ROcp1_26*C8+ROcp1_57*S8;
    ROcp1_38 = ROcp1_36*C8+ROcp1_67*S8;
    ROcp1_48 = -(ROcp1_16*S8-ROcp1_47*C8);
    ROcp1_58 = -(ROcp1_26*S8-ROcp1_57*C8);
    ROcp1_68 = -(ROcp1_36*S8-ROcp1_67*C8);
    ROcp1_19 = ROcp1_18*C9-ROcp1_77*S9;
    ROcp1_29 = ROcp1_28*C9-ROcp1_87*S9;
    ROcp1_39 = ROcp1_38*C9-ROcp1_97*S9;
    ROcp1_79 = ROcp1_18*S9+ROcp1_77*C9;
    ROcp1_89 = ROcp1_28*S9+ROcp1_87*C9;
    ROcp1_99 = ROcp1_38*S9+ROcp1_97*C9;
    ROcp1_110 = ROcp1_19*C10-ROcp1_79*S10;
    ROcp1_210 = ROcp1_29*C10-ROcp1_89*S10;
    ROcp1_310 = ROcp1_39*C10-ROcp1_99*S10;
    ROcp1_710 = ROcp1_19*S10+ROcp1_79*C10;
    ROcp1_810 = ROcp1_29*S10+ROcp1_89*C10;
    ROcp1_910 = ROcp1_39*S10+ROcp1_99*C10;
    ROcp1_111 = ROcp1_110*C11-ROcp1_710*S11;
    ROcp1_211 = ROcp1_210*C11-ROcp1_810*S11;
    ROcp1_311 = ROcp1_310*C11-ROcp1_910*S11;
    ROcp1_711 = ROcp1_110*S11+ROcp1_710*C11;
    ROcp1_811 = ROcp1_210*S11+ROcp1_810*C11;
    ROcp1_911 = ROcp1_310*S11+ROcp1_910*C11;
    ROcp1_412 = ROcp1_48*C12+ROcp1_711*S12;
    ROcp1_512 = ROcp1_58*C12+ROcp1_811*S12;
    ROcp1_612 = ROcp1_68*C12+ROcp1_911*S12;
    ROcp1_712 = -(ROcp1_48*S12-ROcp1_711*C12);
    ROcp1_812 = -(ROcp1_58*S12-ROcp1_811*C12);
    ROcp1_912 = -(ROcp1_68*S12-ROcp1_911*C12);
    ROcp1_113 = ROcp1_111*C13+ROcp1_412*S13;
    ROcp1_213 = ROcp1_211*C13+ROcp1_512*S13;
    ROcp1_313 = ROcp1_311*C13+ROcp1_612*S13;
    ROcp1_413 = -(ROcp1_111*S13-ROcp1_412*C13);
    ROcp1_513 = -(ROcp1_211*S13-ROcp1_512*C13);
    ROcp1_613 = -(ROcp1_311*S13-ROcp1_612*C13);
    ROcp1_114 = ROcp1_113*C14-ROcp1_712*S14;
    ROcp1_214 = ROcp1_213*C14-ROcp1_812*S14;
    ROcp1_314 = ROcp1_313*C14-ROcp1_912*S14;
    ROcp1_714 = ROcp1_113*S14+ROcp1_712*C14;
    ROcp1_814 = ROcp1_213*S14+ROcp1_812*C14;
    ROcp1_914 = ROcp1_313*S14+ROcp1_912*C14;
    ROcp1_415 = ROcp1_413*C15+ROcp1_714*S15;
    ROcp1_515 = ROcp1_513*C15+ROcp1_814*S15;
    ROcp1_615 = ROcp1_613*C15+ROcp1_914*S15;
    ROcp1_715 = -(ROcp1_413*S15-ROcp1_714*C15);
    ROcp1_815 = -(ROcp1_513*S15-ROcp1_814*C15);
    ROcp1_915 = -(ROcp1_613*S15-ROcp1_914*C15);
    RLcp1_17 = ROcp1_16*s->dpt[1][1]+ROcp1_46*s->dpt[2][1];
    RLcp1_27 = ROcp1_26*s->dpt[1][1]+ROcp1_56*s->dpt[2][1];
    RLcp1_37 = ROcp1_36*s->dpt[1][1]+ROcp1_66*s->dpt[2][1];
    OMcp1_17 = OMcp1_16+ROcp1_16*qd[7];
    OMcp1_27 = OMcp1_26+ROcp1_26*qd[7];
    OMcp1_37 = OMcp1_36+ROcp1_36*qd[7];
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27;
    ORcp1_27 = -(OMcp1_16*RLcp1_37-OMcp1_36*RLcp1_17);
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17;
    OPcp1_17 = OPcp1_16+ROcp1_16*qdd[7]+qd[7]*(OMcp1_26*ROcp1_36-OMcp1_36*ROcp1_26);
    OPcp1_27 = OPcp1_26+ROcp1_26*qdd[7]-qd[7]*(OMcp1_16*ROcp1_36-OMcp1_36*ROcp1_16);
    OPcp1_37 = OPcp1_36+ROcp1_36*qdd[7]+qd[7]*(OMcp1_16*ROcp1_26-OMcp1_26*ROcp1_16);
    RLcp1_18 = ROcp1_16*s->dpt[1][5]+ROcp1_47*s->dpt[2][5]+ROcp1_77*s->dpt[3][5];
    RLcp1_28 = ROcp1_26*s->dpt[1][5]+ROcp1_57*s->dpt[2][5]+ROcp1_87*s->dpt[3][5];
    RLcp1_38 = ROcp1_36*s->dpt[1][5]+ROcp1_67*s->dpt[2][5]+ROcp1_97*s->dpt[3][5];
    OMcp1_18 = OMcp1_17+ROcp1_77*qd[8];
    OMcp1_28 = OMcp1_27+ROcp1_87*qd[8];
    OMcp1_38 = OMcp1_37+ROcp1_97*qd[8];
    ORcp1_18 = OMcp1_27*RLcp1_38-OMcp1_37*RLcp1_28;
    ORcp1_28 = -(OMcp1_17*RLcp1_38-OMcp1_37*RLcp1_18);
    ORcp1_38 = OMcp1_17*RLcp1_28-OMcp1_27*RLcp1_18;
    OMcp1_19 = OMcp1_18+ROcp1_48*qd[9];
    OMcp1_29 = OMcp1_28+ROcp1_58*qd[9];
    OMcp1_39 = OMcp1_38+ROcp1_68*qd[9];
    OPcp1_19 = OPcp1_17+ROcp1_48*qdd[9]+ROcp1_77*qdd[8]+qd[8]*(OMcp1_27*ROcp1_97-OMcp1_37*ROcp1_87)+qd[9]*(OMcp1_28*
 ROcp1_68-OMcp1_38*ROcp1_58);
    OPcp1_29 = OPcp1_27+ROcp1_58*qdd[9]+ROcp1_87*qdd[8]-qd[8]*(OMcp1_17*ROcp1_97-OMcp1_37*ROcp1_77)-qd[9]*(OMcp1_18*
 ROcp1_68-OMcp1_38*ROcp1_48);
    OPcp1_39 = OPcp1_37+ROcp1_68*qdd[9]+ROcp1_97*qdd[8]+qd[8]*(OMcp1_17*ROcp1_87-OMcp1_27*ROcp1_77)+qd[9]*(OMcp1_18*
 ROcp1_58-OMcp1_28*ROcp1_48);
    RLcp1_110 = ROcp1_79*s->dpt[3][7];
    RLcp1_210 = ROcp1_89*s->dpt[3][7];
    RLcp1_310 = ROcp1_99*s->dpt[3][7];
    OMcp1_110 = OMcp1_19+ROcp1_48*qd[10];
    OMcp1_210 = OMcp1_29+ROcp1_58*qd[10];
    OMcp1_310 = OMcp1_39+ROcp1_68*qd[10];
    ORcp1_110 = OMcp1_29*RLcp1_310-OMcp1_39*RLcp1_210;
    ORcp1_210 = -(OMcp1_19*RLcp1_310-OMcp1_39*RLcp1_110);
    ORcp1_310 = OMcp1_19*RLcp1_210-OMcp1_29*RLcp1_110;
    OPcp1_110 = OPcp1_19+ROcp1_48*qdd[10]+qd[10]*(OMcp1_29*ROcp1_68-OMcp1_39*ROcp1_58);
    OPcp1_210 = OPcp1_29+ROcp1_58*qdd[10]-qd[10]*(OMcp1_19*ROcp1_68-OMcp1_39*ROcp1_48);
    OPcp1_310 = OPcp1_39+ROcp1_68*qdd[10]+qd[10]*(OMcp1_19*ROcp1_58-OMcp1_29*ROcp1_48);
    RLcp1_111 = ROcp1_710*s->dpt[3][8];
    RLcp1_211 = ROcp1_810*s->dpt[3][8];
    RLcp1_311 = ROcp1_910*s->dpt[3][8];
    OMcp1_111 = OMcp1_110+ROcp1_48*qd[11];
    OMcp1_211 = OMcp1_210+ROcp1_58*qd[11];
    OMcp1_311 = OMcp1_310+ROcp1_68*qd[11];
    ORcp1_111 = OMcp1_210*RLcp1_311-OMcp1_310*RLcp1_211;
    ORcp1_211 = -(OMcp1_110*RLcp1_311-OMcp1_310*RLcp1_111);
    ORcp1_311 = OMcp1_110*RLcp1_211-OMcp1_210*RLcp1_111;
    OMcp1_112 = OMcp1_111+ROcp1_111*qd[12];
    OMcp1_212 = OMcp1_211+ROcp1_211*qd[12];
    OMcp1_312 = OMcp1_311+ROcp1_311*qd[12];
    OPcp1_112 = OPcp1_110+ROcp1_111*qdd[12]+ROcp1_48*qdd[11]+qd[11]*(OMcp1_210*ROcp1_68-OMcp1_310*ROcp1_58)+qd[12]*(
 OMcp1_211*ROcp1_311-OMcp1_311*ROcp1_211);
    OPcp1_212 = OPcp1_210+ROcp1_211*qdd[12]+ROcp1_58*qdd[11]-qd[11]*(OMcp1_110*ROcp1_68-OMcp1_310*ROcp1_48)-qd[12]*(
 OMcp1_111*ROcp1_311-OMcp1_311*ROcp1_111);
    OPcp1_312 = OPcp1_310+ROcp1_311*qdd[12]+ROcp1_68*qdd[11]+qd[11]*(OMcp1_110*ROcp1_58-OMcp1_210*ROcp1_48)+qd[12]*(
 OMcp1_111*ROcp1_211-OMcp1_211*ROcp1_111);
    RLcp1_113 = ROcp1_111*s->dpt[1][10]+ROcp1_712*s->dpt[3][10];
    RLcp1_213 = ROcp1_211*s->dpt[1][10]+ROcp1_812*s->dpt[3][10];
    RLcp1_313 = ROcp1_311*s->dpt[1][10]+ROcp1_912*s->dpt[3][10];
    ORcp1_113 = OMcp1_212*RLcp1_313-OMcp1_312*RLcp1_213;
    ORcp1_213 = -(OMcp1_112*RLcp1_313-OMcp1_312*RLcp1_113);
    ORcp1_313 = OMcp1_112*RLcp1_213-OMcp1_212*RLcp1_113;
    RLcp1_116 = ROcp1_715*q[16];
    RLcp1_216 = ROcp1_815*q[16];
    RLcp1_316 = ROcp1_915*q[16];
    ORcp1_116 = OMcp1_212*RLcp1_316-OMcp1_312*RLcp1_216;
    ORcp1_216 = -(OMcp1_112*RLcp1_316-OMcp1_312*RLcp1_116);
    ORcp1_316 = OMcp1_112*RLcp1_216-OMcp1_212*RLcp1_116;
    RLcp1_117 = ROcp1_415*q[17];
    RLcp1_217 = ROcp1_515*q[17];
    RLcp1_317 = ROcp1_615*q[17];
    ORcp1_117 = OMcp1_212*RLcp1_317-OMcp1_312*RLcp1_217;
    ORcp1_217 = -(OMcp1_112*RLcp1_317-OMcp1_312*RLcp1_117);
    ORcp1_317 = OMcp1_112*RLcp1_217-OMcp1_212*RLcp1_117;
    RLcp1_118 = ROcp1_114*q[18];
    RLcp1_218 = ROcp1_214*q[18];
    RLcp1_318 = ROcp1_314*q[18];
    ORcp1_118 = OMcp1_212*RLcp1_318-OMcp1_312*RLcp1_218;
    ORcp1_218 = -(OMcp1_112*RLcp1_318-OMcp1_312*RLcp1_118);
    ORcp1_318 = OMcp1_112*RLcp1_218-OMcp1_212*RLcp1_118;
    RLcp1_157 = ROcp1_715*s->dpt[3][11];
    RLcp1_257 = ROcp1_815*s->dpt[3][11];
    RLcp1_357 = ROcp1_915*s->dpt[3][11];
    POcp1_157 = RLcp1_110+RLcp1_111+RLcp1_113+RLcp1_116+RLcp1_117+RLcp1_118+RLcp1_157+RLcp1_17+RLcp1_18+q[1];
    POcp1_257 = RLcp1_210+RLcp1_211+RLcp1_213+RLcp1_216+RLcp1_217+RLcp1_218+RLcp1_257+RLcp1_27+RLcp1_28+q[2];
    POcp1_357 = RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357+RLcp1_37+RLcp1_38+q[3];
    JTcp1_257_4 = -(RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357+RLcp1_37+RLcp1_38);
    JTcp1_357_4 = RLcp1_210+RLcp1_211+RLcp1_213+RLcp1_216+RLcp1_217+RLcp1_218+RLcp1_257+RLcp1_27+RLcp1_28;
    JTcp1_157_5 = C4*(RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_37+RLcp1_38)-S4*(RLcp1_210+
 RLcp1_211)-S4*(RLcp1_213+RLcp1_216)-S4*(RLcp1_217+RLcp1_218)-S4*(RLcp1_27+RLcp1_28)-RLcp1_257*S4+RLcp1_357*C4;
    JTcp1_257_5 = S4*(RLcp1_110+RLcp1_111+RLcp1_113+RLcp1_116+RLcp1_117+RLcp1_118+RLcp1_157+RLcp1_17+RLcp1_18);
    JTcp1_357_5 = -C4*(RLcp1_110+RLcp1_111+RLcp1_113+RLcp1_116+RLcp1_117+RLcp1_118+RLcp1_157+RLcp1_17+RLcp1_18);
    JTcp1_157_6 = ROcp1_85*(RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_37+RLcp1_38)-ROcp1_95*(
 RLcp1_210+RLcp1_211)-ROcp1_95*(RLcp1_213+RLcp1_216)-ROcp1_95*(RLcp1_217+RLcp1_218)-ROcp1_95*(RLcp1_27+RLcp1_28)-RLcp1_257*
 ROcp1_95+RLcp1_357*ROcp1_85;
    JTcp1_257_6 = -(RLcp1_357*S5-ROcp1_95*(RLcp1_110+RLcp1_111+RLcp1_113+RLcp1_116+RLcp1_117+RLcp1_118+RLcp1_157+RLcp1_17+
 RLcp1_18)+S5*(RLcp1_310+RLcp1_311)+S5*(RLcp1_313+RLcp1_316)+S5*(RLcp1_317+RLcp1_318)+S5*(RLcp1_37+RLcp1_38));
    JTcp1_357_6 = RLcp1_257*S5-ROcp1_85*(RLcp1_110+RLcp1_111+RLcp1_113+RLcp1_116+RLcp1_117+RLcp1_118+RLcp1_157+RLcp1_17+
 RLcp1_18)+S5*(RLcp1_210+RLcp1_211)+S5*(RLcp1_213+RLcp1_216)+S5*(RLcp1_217+RLcp1_218)+S5*(RLcp1_27+RLcp1_28);
    JTcp1_157_7 = ROcp1_26*(RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357+RLcp1_38)-ROcp1_36*(
 RLcp1_210+RLcp1_28)-ROcp1_36*(RLcp1_211+RLcp1_213)-ROcp1_36*(RLcp1_216+RLcp1_217)-ROcp1_36*(RLcp1_218+RLcp1_257);
    JTcp1_257_7 = -(ROcp1_16*(RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357+RLcp1_38)-ROcp1_36*(
 RLcp1_110+RLcp1_18)-ROcp1_36*(RLcp1_111+RLcp1_113)-ROcp1_36*(RLcp1_116+RLcp1_117)-ROcp1_36*(RLcp1_118+RLcp1_157));
    JTcp1_357_7 = ROcp1_16*(RLcp1_210+RLcp1_211+RLcp1_213+RLcp1_216+RLcp1_217+RLcp1_218+RLcp1_257+RLcp1_28)-ROcp1_26*(
 RLcp1_110+RLcp1_18)-ROcp1_26*(RLcp1_111+RLcp1_113)-ROcp1_26*(RLcp1_116+RLcp1_117)-ROcp1_26*(RLcp1_118+RLcp1_157);
    JTcp1_157_8 = ROcp1_87*(RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318)-ROcp1_97*(RLcp1_210+RLcp1_211)-
 ROcp1_97*(RLcp1_213+RLcp1_216)-ROcp1_97*(RLcp1_217+RLcp1_218)-RLcp1_257*ROcp1_97+RLcp1_357*ROcp1_87;
    JTcp1_257_8 = RLcp1_157*ROcp1_97-RLcp1_357*ROcp1_77-ROcp1_77*(RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+
 RLcp1_318)+ROcp1_97*(RLcp1_110+RLcp1_111)+ROcp1_97*(RLcp1_113+RLcp1_116)+ROcp1_97*(RLcp1_117+RLcp1_118);
    JTcp1_357_8 = ROcp1_77*(RLcp1_210+RLcp1_211+RLcp1_213+RLcp1_216+RLcp1_217+RLcp1_218)-ROcp1_87*(RLcp1_110+RLcp1_111)-
 ROcp1_87*(RLcp1_113+RLcp1_116)-ROcp1_87*(RLcp1_117+RLcp1_118)-RLcp1_157*ROcp1_87+RLcp1_257*ROcp1_77;
    JTcp1_157_9 = ROcp1_58*(RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318)-ROcp1_68*(RLcp1_210+RLcp1_211)-
 ROcp1_68*(RLcp1_213+RLcp1_216)-ROcp1_68*(RLcp1_217+RLcp1_218)-RLcp1_257*ROcp1_68+RLcp1_357*ROcp1_58;
    JTcp1_257_9 = RLcp1_157*ROcp1_68-RLcp1_357*ROcp1_48-ROcp1_48*(RLcp1_310+RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+
 RLcp1_318)+ROcp1_68*(RLcp1_110+RLcp1_111)+ROcp1_68*(RLcp1_113+RLcp1_116)+ROcp1_68*(RLcp1_117+RLcp1_118);
    JTcp1_357_9 = ROcp1_48*(RLcp1_210+RLcp1_211+RLcp1_213+RLcp1_216+RLcp1_217+RLcp1_218)-ROcp1_58*(RLcp1_110+RLcp1_111)-
 ROcp1_58*(RLcp1_113+RLcp1_116)-ROcp1_58*(RLcp1_117+RLcp1_118)-RLcp1_157*ROcp1_58+RLcp1_257*ROcp1_48;
    JTcp1_157_10 = ROcp1_58*(RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357)-ROcp1_68*(RLcp1_211+RLcp1_213)-
 ROcp1_68*(RLcp1_216+RLcp1_217)-ROcp1_68*(RLcp1_218+RLcp1_257);
    JTcp1_257_10 = -(ROcp1_48*(RLcp1_311+RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357)-ROcp1_68*(RLcp1_111+RLcp1_113)
 -ROcp1_68*(RLcp1_116+RLcp1_117)-ROcp1_68*(RLcp1_118+RLcp1_157));
    JTcp1_357_10 = ROcp1_48*(RLcp1_211+RLcp1_213+RLcp1_216+RLcp1_217+RLcp1_218+RLcp1_257)-ROcp1_58*(RLcp1_111+RLcp1_113)-
 ROcp1_58*(RLcp1_116+RLcp1_117)-ROcp1_58*(RLcp1_118+RLcp1_157);
    JTcp1_157_11 = ROcp1_58*(RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318)-ROcp1_68*(RLcp1_213+RLcp1_216)-ROcp1_68*(RLcp1_217+
 RLcp1_218)-RLcp1_257*ROcp1_68+RLcp1_357*ROcp1_58;
    JTcp1_257_11 = RLcp1_157*ROcp1_68-RLcp1_357*ROcp1_48-ROcp1_48*(RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318)+ROcp1_68*(
 RLcp1_113+RLcp1_116)+ROcp1_68*(RLcp1_117+RLcp1_118);
    JTcp1_357_11 = ROcp1_48*(RLcp1_213+RLcp1_216+RLcp1_217+RLcp1_218)-ROcp1_58*(RLcp1_113+RLcp1_116)-ROcp1_58*(RLcp1_117+
 RLcp1_118)-RLcp1_157*ROcp1_58+RLcp1_257*ROcp1_48;
    JTcp1_157_12 = ROcp1_211*(RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318)-ROcp1_311*(RLcp1_213+RLcp1_216)-ROcp1_311*(
 RLcp1_217+RLcp1_218)-RLcp1_257*ROcp1_311+RLcp1_357*ROcp1_211;
    JTcp1_257_12 = RLcp1_157*ROcp1_311-RLcp1_357*ROcp1_111-ROcp1_111*(RLcp1_313+RLcp1_316+RLcp1_317+RLcp1_318)+ROcp1_311*(
 RLcp1_113+RLcp1_116)+ROcp1_311*(RLcp1_117+RLcp1_118);
    JTcp1_357_12 = ROcp1_111*(RLcp1_213+RLcp1_216+RLcp1_217+RLcp1_218)-ROcp1_211*(RLcp1_113+RLcp1_116)-ROcp1_211*(
 RLcp1_117+RLcp1_118)-RLcp1_157*ROcp1_211+RLcp1_257*ROcp1_111;
    JTcp1_157_13 = ROcp1_812*(RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357)-ROcp1_912*(RLcp1_216+RLcp1_217)-ROcp1_912*(
 RLcp1_218+RLcp1_257);
    JTcp1_257_13 = -(ROcp1_712*(RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357)-ROcp1_912*(RLcp1_116+RLcp1_117)-ROcp1_912*(
 RLcp1_118+RLcp1_157));
    JTcp1_357_13 = ROcp1_712*(RLcp1_216+RLcp1_217+RLcp1_218+RLcp1_257)-ROcp1_812*(RLcp1_116+RLcp1_117)-ROcp1_812*(
 RLcp1_118+RLcp1_157);
    JTcp1_157_14 = ROcp1_513*(RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357)-ROcp1_613*(RLcp1_216+RLcp1_217)-ROcp1_613*(
 RLcp1_218+RLcp1_257);
    JTcp1_257_14 = -(ROcp1_413*(RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357)-ROcp1_613*(RLcp1_116+RLcp1_117)-ROcp1_613*(
 RLcp1_118+RLcp1_157));
    JTcp1_357_14 = ROcp1_413*(RLcp1_216+RLcp1_217+RLcp1_218+RLcp1_257)-ROcp1_513*(RLcp1_116+RLcp1_117)-ROcp1_513*(
 RLcp1_118+RLcp1_157);
    JTcp1_157_15 = ROcp1_214*(RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357)-ROcp1_314*(RLcp1_216+RLcp1_217)-ROcp1_314*(
 RLcp1_218+RLcp1_257);
    JTcp1_257_15 = -(ROcp1_114*(RLcp1_316+RLcp1_317+RLcp1_318+RLcp1_357)-ROcp1_314*(RLcp1_116+RLcp1_117)-ROcp1_314*(
 RLcp1_118+RLcp1_157));
    JTcp1_357_15 = ROcp1_114*(RLcp1_216+RLcp1_217+RLcp1_218+RLcp1_257)-ROcp1_214*(RLcp1_116+RLcp1_117)-ROcp1_214*(
 RLcp1_118+RLcp1_157);
    ORcp1_157 = OMcp1_212*RLcp1_357-OMcp1_312*RLcp1_257;
    ORcp1_257 = -(OMcp1_112*RLcp1_357-OMcp1_312*RLcp1_157);
    ORcp1_357 = OMcp1_112*RLcp1_257-OMcp1_212*RLcp1_157;
    VIcp1_157 = ORcp1_110+ORcp1_111+ORcp1_113+ORcp1_116+ORcp1_117+ORcp1_118+ORcp1_157+ORcp1_17+ORcp1_18+qd[1];
    VIcp1_257 = ORcp1_210+ORcp1_211+ORcp1_213+ORcp1_216+ORcp1_217+ORcp1_218+ORcp1_257+ORcp1_27+ORcp1_28+qd[2];
    VIcp1_357 = ORcp1_310+ORcp1_311+ORcp1_313+ORcp1_316+ORcp1_317+ORcp1_318+ORcp1_357+ORcp1_37+ORcp1_38+qd[3];
    ACcp1_157 = qdd[1]+OMcp1_210*ORcp1_311+OMcp1_212*ORcp1_313+OMcp1_212*ORcp1_316+OMcp1_212*ORcp1_317+OMcp1_212*ORcp1_318
 +OMcp1_212*ORcp1_357+OMcp1_26*ORcp1_37+OMcp1_27*ORcp1_38+OMcp1_29*ORcp1_310-OMcp1_310*ORcp1_211-OMcp1_312*ORcp1_213-
 OMcp1_312*ORcp1_216-OMcp1_312*ORcp1_217-OMcp1_312*ORcp1_218-OMcp1_312*ORcp1_257-OMcp1_36*ORcp1_27-OMcp1_37*ORcp1_28-OMcp1_39
 *ORcp1_210+OPcp1_210*RLcp1_311+OPcp1_212*RLcp1_313+OPcp1_212*RLcp1_316+OPcp1_212*RLcp1_317+OPcp1_212*RLcp1_318+OPcp1_212*
 RLcp1_357+OPcp1_26*RLcp1_37+OPcp1_27*RLcp1_38+OPcp1_29*RLcp1_310-OPcp1_310*RLcp1_211-OPcp1_312*RLcp1_213-OPcp1_312*RLcp1_216
 -OPcp1_312*RLcp1_217-OPcp1_312*RLcp1_218-OPcp1_312*RLcp1_257-OPcp1_36*RLcp1_27-OPcp1_37*RLcp1_28-OPcp1_39*RLcp1_210;
    ACcp1_257 = qdd[2]-OMcp1_110*ORcp1_311-OMcp1_112*ORcp1_313-OMcp1_112*ORcp1_316-OMcp1_112*ORcp1_317-OMcp1_112*ORcp1_318
 -OMcp1_112*ORcp1_357-OMcp1_16*ORcp1_37-OMcp1_17*ORcp1_38-OMcp1_19*ORcp1_310+OMcp1_310*ORcp1_111+OMcp1_312*ORcp1_113+
 OMcp1_312*ORcp1_116+OMcp1_312*ORcp1_117+OMcp1_312*ORcp1_118+OMcp1_312*ORcp1_157+OMcp1_36*ORcp1_17+OMcp1_37*ORcp1_18+OMcp1_39
 *ORcp1_110-OPcp1_110*RLcp1_311-OPcp1_112*RLcp1_313-OPcp1_112*RLcp1_316-OPcp1_112*RLcp1_317-OPcp1_112*RLcp1_318-OPcp1_112*
 RLcp1_357-OPcp1_16*RLcp1_37-OPcp1_17*RLcp1_38-OPcp1_19*RLcp1_310+OPcp1_310*RLcp1_111+OPcp1_312*RLcp1_113+OPcp1_312*RLcp1_116
 +OPcp1_312*RLcp1_117+OPcp1_312*RLcp1_118+OPcp1_312*RLcp1_157+OPcp1_36*RLcp1_17+OPcp1_37*RLcp1_18+OPcp1_39*RLcp1_110;
    ACcp1_357 = qdd[3]+OMcp1_110*ORcp1_211+OMcp1_112*ORcp1_213+OMcp1_112*ORcp1_216+OMcp1_112*ORcp1_217+OMcp1_112*ORcp1_218
 +OMcp1_112*ORcp1_257+OMcp1_16*ORcp1_27+OMcp1_17*ORcp1_28+OMcp1_19*ORcp1_210-OMcp1_210*ORcp1_111-OMcp1_212*ORcp1_113-
 OMcp1_212*ORcp1_116-OMcp1_212*ORcp1_117-OMcp1_212*ORcp1_118-OMcp1_212*ORcp1_157-OMcp1_26*ORcp1_17-OMcp1_27*ORcp1_18-OMcp1_29
 *ORcp1_110+OPcp1_110*RLcp1_211+OPcp1_112*RLcp1_213+OPcp1_112*RLcp1_216+OPcp1_112*RLcp1_217+OPcp1_112*RLcp1_218+OPcp1_112*
 RLcp1_257+OPcp1_16*RLcp1_27+OPcp1_17*RLcp1_28+OPcp1_19*RLcp1_210-OPcp1_210*RLcp1_111-OPcp1_212*RLcp1_113-OPcp1_212*RLcp1_116
 -OPcp1_212*RLcp1_117-OPcp1_212*RLcp1_118-OPcp1_212*RLcp1_157-OPcp1_26*RLcp1_17-OPcp1_27*RLcp1_18-OPcp1_29*RLcp1_110;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_157;
    sens->P[2] = POcp1_257;
    sens->P[3] = POcp1_357;
    sens->R[1][1] = ROcp1_114;
    sens->R[1][2] = ROcp1_214;
    sens->R[1][3] = ROcp1_314;
    sens->R[2][1] = ROcp1_415;
    sens->R[2][2] = ROcp1_515;
    sens->R[2][3] = ROcp1_615;
    sens->R[3][1] = ROcp1_715;
    sens->R[3][2] = ROcp1_815;
    sens->R[3][3] = ROcp1_915;
    sens->V[1] = VIcp1_157;
    sens->V[2] = VIcp1_257;
    sens->V[3] = VIcp1_357;
    sens->OM[1] = OMcp1_112;
    sens->OM[2] = OMcp1_212;
    sens->OM[3] = OMcp1_312;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp1_157_5;
    sens->J[1][6] = JTcp1_157_6;
    sens->J[1][7] = JTcp1_157_7;
    sens->J[1][8] = JTcp1_157_8;
    sens->J[1][9] = JTcp1_157_9;
    sens->J[1][10] = JTcp1_157_10;
    sens->J[1][11] = JTcp1_157_11;
    sens->J[1][12] = JTcp1_157_12;
    sens->J[1][13] = JTcp1_157_13;
    sens->J[1][14] = JTcp1_157_14;
    sens->J[1][15] = JTcp1_157_15;
    sens->J[1][16] = ROcp1_715;
    sens->J[1][17] = ROcp1_415;
    sens->J[1][18] = ROcp1_114;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp1_257_4;
    sens->J[2][5] = JTcp1_257_5;
    sens->J[2][6] = JTcp1_257_6;
    sens->J[2][7] = JTcp1_257_7;
    sens->J[2][8] = JTcp1_257_8;
    sens->J[2][9] = JTcp1_257_9;
    sens->J[2][10] = JTcp1_257_10;
    sens->J[2][11] = JTcp1_257_11;
    sens->J[2][12] = JTcp1_257_12;
    sens->J[2][13] = JTcp1_257_13;
    sens->J[2][14] = JTcp1_257_14;
    sens->J[2][15] = JTcp1_257_15;
    sens->J[2][16] = ROcp1_815;
    sens->J[2][17] = ROcp1_515;
    sens->J[2][18] = ROcp1_214;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp1_357_4;
    sens->J[3][5] = JTcp1_357_5;
    sens->J[3][6] = JTcp1_357_6;
    sens->J[3][7] = JTcp1_357_7;
    sens->J[3][8] = JTcp1_357_8;
    sens->J[3][9] = JTcp1_357_9;
    sens->J[3][10] = JTcp1_357_10;
    sens->J[3][11] = JTcp1_357_11;
    sens->J[3][12] = JTcp1_357_12;
    sens->J[3][13] = JTcp1_357_13;
    sens->J[3][14] = JTcp1_357_14;
    sens->J[3][15] = JTcp1_357_15;
    sens->J[3][16] = ROcp1_915;
    sens->J[3][17] = ROcp1_615;
    sens->J[3][18] = ROcp1_314;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp1_16;
    sens->J[4][8] = ROcp1_77;
    sens->J[4][9] = ROcp1_48;
    sens->J[4][10] = ROcp1_48;
    sens->J[4][11] = ROcp1_48;
    sens->J[4][12] = ROcp1_111;
    sens->J[4][13] = ROcp1_712;
    sens->J[4][14] = ROcp1_413;
    sens->J[4][15] = ROcp1_114;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp1_85;
    sens->J[5][7] = ROcp1_26;
    sens->J[5][8] = ROcp1_87;
    sens->J[5][9] = ROcp1_58;
    sens->J[5][10] = ROcp1_58;
    sens->J[5][11] = ROcp1_58;
    sens->J[5][12] = ROcp1_211;
    sens->J[5][13] = ROcp1_812;
    sens->J[5][14] = ROcp1_513;
    sens->J[5][15] = ROcp1_214;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp1_95;
    sens->J[6][7] = ROcp1_36;
    sens->J[6][8] = ROcp1_97;
    sens->J[6][9] = ROcp1_68;
    sens->J[6][10] = ROcp1_68;
    sens->J[6][11] = ROcp1_68;
    sens->J[6][12] = ROcp1_311;
    sens->J[6][13] = ROcp1_912;
    sens->J[6][14] = ROcp1_613;
    sens->J[6][15] = ROcp1_314;
    sens->A[1] = ACcp1_157;
    sens->A[2] = ACcp1_257;
    sens->A[3] = ACcp1_357;
    sens->OMP[1] = OPcp1_112;
    sens->OMP[2] = OPcp1_212;
    sens->OMP[3] = OPcp1_312;
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_0_1 = = 
 
// Sensor Kinematics 


    ROcp2_25 = S4*S5;
    ROcp2_35 = -C4*S5;
    ROcp2_85 = -S4*C5;
    ROcp2_95 = C4*C5;
    ROcp2_16 = C5*C6;
    ROcp2_26 = ROcp2_25*C6+C4*S6;
    ROcp2_36 = ROcp2_35*C6+S4*S6;
    ROcp2_46 = -C5*S6;
    ROcp2_56 = -(ROcp2_25*S6-C4*C6);
    ROcp2_66 = -(ROcp2_35*S6-S4*C6);
    OMcp2_25 = qd[5]*C4;
    OMcp2_35 = qd[5]*S4;
    OMcp2_16 = qd[4]+qd[6]*S5;
    OMcp2_26 = OMcp2_25+ROcp2_85*qd[6];
    OMcp2_36 = OMcp2_35+ROcp2_95*qd[6];
    OPcp2_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp2_26 = ROcp2_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp2_35*S5-ROcp2_95*qd[4]);
    OPcp2_36 = ROcp2_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp2_25*S5-ROcp2_85*qd[4]);

// = = Block_1_0_0_3_0_3 = = 
 
// Sensor Kinematics 


    ROcp2_419 = ROcp2_46*C19+S19*S5;
    ROcp2_519 = ROcp2_56*C19+ROcp2_85*S19;
    ROcp2_619 = ROcp2_66*C19+ROcp2_95*S19;
    ROcp2_719 = -(ROcp2_46*S19-C19*S5);
    ROcp2_819 = -(ROcp2_56*S19-ROcp2_85*C19);
    ROcp2_919 = -(ROcp2_66*S19-ROcp2_95*C19);
    ROcp2_120 = ROcp2_16*C20+ROcp2_419*S20;
    ROcp2_220 = ROcp2_26*C20+ROcp2_519*S20;
    ROcp2_320 = ROcp2_36*C20+ROcp2_619*S20;
    ROcp2_420 = -(ROcp2_16*S20-ROcp2_419*C20);
    ROcp2_520 = -(ROcp2_26*S20-ROcp2_519*C20);
    ROcp2_620 = -(ROcp2_36*S20-ROcp2_619*C20);
    ROcp2_121 = ROcp2_120*C21-ROcp2_719*S21;
    ROcp2_221 = ROcp2_220*C21-ROcp2_819*S21;
    ROcp2_321 = ROcp2_320*C21-ROcp2_919*S21;
    ROcp2_721 = ROcp2_120*S21+ROcp2_719*C21;
    ROcp2_821 = ROcp2_220*S21+ROcp2_819*C21;
    ROcp2_921 = ROcp2_320*S21+ROcp2_919*C21;
    ROcp2_122 = ROcp2_121*C22-ROcp2_721*S22;
    ROcp2_222 = ROcp2_221*C22-ROcp2_821*S22;
    ROcp2_322 = ROcp2_321*C22-ROcp2_921*S22;
    ROcp2_722 = ROcp2_121*S22+ROcp2_721*C22;
    ROcp2_822 = ROcp2_221*S22+ROcp2_821*C22;
    ROcp2_922 = ROcp2_321*S22+ROcp2_921*C22;
    ROcp2_123 = ROcp2_122*C23-ROcp2_722*S23;
    ROcp2_223 = ROcp2_222*C23-ROcp2_822*S23;
    ROcp2_323 = ROcp2_322*C23-ROcp2_922*S23;
    ROcp2_723 = ROcp2_122*S23+ROcp2_722*C23;
    ROcp2_823 = ROcp2_222*S23+ROcp2_822*C23;
    ROcp2_923 = ROcp2_322*S23+ROcp2_922*C23;
    ROcp2_424 = ROcp2_420*C24+ROcp2_723*S24;
    ROcp2_524 = ROcp2_520*C24+ROcp2_823*S24;
    ROcp2_624 = ROcp2_620*C24+ROcp2_923*S24;
    ROcp2_724 = -(ROcp2_420*S24-ROcp2_723*C24);
    ROcp2_824 = -(ROcp2_520*S24-ROcp2_823*C24);
    ROcp2_924 = -(ROcp2_620*S24-ROcp2_923*C24);
    ROcp2_125 = ROcp2_123*C25+ROcp2_424*S25;
    ROcp2_225 = ROcp2_223*C25+ROcp2_524*S25;
    ROcp2_325 = ROcp2_323*C25+ROcp2_624*S25;
    ROcp2_425 = -(ROcp2_123*S25-ROcp2_424*C25);
    ROcp2_525 = -(ROcp2_223*S25-ROcp2_524*C25);
    ROcp2_625 = -(ROcp2_323*S25-ROcp2_624*C25);
    ROcp2_126 = ROcp2_125*C26-ROcp2_724*S26;
    ROcp2_226 = ROcp2_225*C26-ROcp2_824*S26;
    ROcp2_326 = ROcp2_325*C26-ROcp2_924*S26;
    ROcp2_726 = ROcp2_125*S26+ROcp2_724*C26;
    ROcp2_826 = ROcp2_225*S26+ROcp2_824*C26;
    ROcp2_926 = ROcp2_325*S26+ROcp2_924*C26;
    ROcp2_427 = ROcp2_425*C27+ROcp2_726*S27;
    ROcp2_527 = ROcp2_525*C27+ROcp2_826*S27;
    ROcp2_627 = ROcp2_625*C27+ROcp2_926*S27;
    ROcp2_727 = -(ROcp2_425*S27-ROcp2_726*C27);
    ROcp2_827 = -(ROcp2_525*S27-ROcp2_826*C27);
    ROcp2_927 = -(ROcp2_625*S27-ROcp2_926*C27);
    RLcp2_119 = ROcp2_16*s->dpt[1][2]+ROcp2_46*s->dpt[2][2];
    RLcp2_219 = ROcp2_26*s->dpt[1][2]+ROcp2_56*s->dpt[2][2];
    RLcp2_319 = ROcp2_36*s->dpt[1][2]+ROcp2_66*s->dpt[2][2];
    OMcp2_119 = OMcp2_16+ROcp2_16*qd[19];
    OMcp2_219 = OMcp2_26+ROcp2_26*qd[19];
    OMcp2_319 = OMcp2_36+ROcp2_36*qd[19];
    ORcp2_119 = OMcp2_26*RLcp2_319-OMcp2_36*RLcp2_219;
    ORcp2_219 = -(OMcp2_16*RLcp2_319-OMcp2_36*RLcp2_119);
    ORcp2_319 = OMcp2_16*RLcp2_219-OMcp2_26*RLcp2_119;
    OPcp2_119 = OPcp2_16+ROcp2_16*qdd[19]+qd[19]*(OMcp2_26*ROcp2_36-OMcp2_36*ROcp2_26);
    OPcp2_219 = OPcp2_26+ROcp2_26*qdd[19]-qd[19]*(OMcp2_16*ROcp2_36-OMcp2_36*ROcp2_16);
    OPcp2_319 = OPcp2_36+ROcp2_36*qdd[19]+qd[19]*(OMcp2_16*ROcp2_26-OMcp2_26*ROcp2_16);
    RLcp2_120 = ROcp2_16*s->dpt[1][12]+ROcp2_419*s->dpt[2][12]+ROcp2_719*s->dpt[3][12];
    RLcp2_220 = ROcp2_26*s->dpt[1][12]+ROcp2_519*s->dpt[2][12]+ROcp2_819*s->dpt[3][12];
    RLcp2_320 = ROcp2_36*s->dpt[1][12]+ROcp2_619*s->dpt[2][12]+ROcp2_919*s->dpt[3][12];
    OMcp2_120 = OMcp2_119+ROcp2_719*qd[20];
    OMcp2_220 = OMcp2_219+ROcp2_819*qd[20];
    OMcp2_320 = OMcp2_319+ROcp2_919*qd[20];
    ORcp2_120 = OMcp2_219*RLcp2_320-OMcp2_319*RLcp2_220;
    ORcp2_220 = -(OMcp2_119*RLcp2_320-OMcp2_319*RLcp2_120);
    ORcp2_320 = OMcp2_119*RLcp2_220-OMcp2_219*RLcp2_120;
    OMcp2_121 = OMcp2_120+ROcp2_420*qd[21];
    OMcp2_221 = OMcp2_220+ROcp2_520*qd[21];
    OMcp2_321 = OMcp2_320+ROcp2_620*qd[21];
    OPcp2_121 = OPcp2_119+ROcp2_420*qdd[21]+ROcp2_719*qdd[20]+qd[20]*(OMcp2_219*ROcp2_919-OMcp2_319*ROcp2_819)+qd[21]*(
 OMcp2_220*ROcp2_620-OMcp2_320*ROcp2_520);
    OPcp2_221 = OPcp2_219+ROcp2_520*qdd[21]+ROcp2_819*qdd[20]-qd[20]*(OMcp2_119*ROcp2_919-OMcp2_319*ROcp2_719)-qd[21]*(
 OMcp2_120*ROcp2_620-OMcp2_320*ROcp2_420);
    OPcp2_321 = OPcp2_319+ROcp2_620*qdd[21]+ROcp2_919*qdd[20]+qd[20]*(OMcp2_119*ROcp2_819-OMcp2_219*ROcp2_719)+qd[21]*(
 OMcp2_120*ROcp2_520-OMcp2_220*ROcp2_420);
    RLcp2_122 = ROcp2_721*s->dpt[3][14];
    RLcp2_222 = ROcp2_821*s->dpt[3][14];
    RLcp2_322 = ROcp2_921*s->dpt[3][14];
    OMcp2_122 = OMcp2_121+ROcp2_420*qd[22];
    OMcp2_222 = OMcp2_221+ROcp2_520*qd[22];
    OMcp2_322 = OMcp2_321+ROcp2_620*qd[22];
    ORcp2_122 = OMcp2_221*RLcp2_322-OMcp2_321*RLcp2_222;
    ORcp2_222 = -(OMcp2_121*RLcp2_322-OMcp2_321*RLcp2_122);
    ORcp2_322 = OMcp2_121*RLcp2_222-OMcp2_221*RLcp2_122;
    OPcp2_122 = OPcp2_121+ROcp2_420*qdd[22]+qd[22]*(OMcp2_221*ROcp2_620-OMcp2_321*ROcp2_520);
    OPcp2_222 = OPcp2_221+ROcp2_520*qdd[22]-qd[22]*(OMcp2_121*ROcp2_620-OMcp2_321*ROcp2_420);
    OPcp2_322 = OPcp2_321+ROcp2_620*qdd[22]+qd[22]*(OMcp2_121*ROcp2_520-OMcp2_221*ROcp2_420);
    RLcp2_123 = ROcp2_722*s->dpt[3][15];
    RLcp2_223 = ROcp2_822*s->dpt[3][15];
    RLcp2_323 = ROcp2_922*s->dpt[3][15];
    OMcp2_123 = OMcp2_122+ROcp2_420*qd[23];
    OMcp2_223 = OMcp2_222+ROcp2_520*qd[23];
    OMcp2_323 = OMcp2_322+ROcp2_620*qd[23];
    ORcp2_123 = OMcp2_222*RLcp2_323-OMcp2_322*RLcp2_223;
    ORcp2_223 = -(OMcp2_122*RLcp2_323-OMcp2_322*RLcp2_123);
    ORcp2_323 = OMcp2_122*RLcp2_223-OMcp2_222*RLcp2_123;
    OMcp2_124 = OMcp2_123+ROcp2_123*qd[24];
    OMcp2_224 = OMcp2_223+ROcp2_223*qd[24];
    OMcp2_324 = OMcp2_323+ROcp2_323*qd[24];
    OPcp2_124 = OPcp2_122+ROcp2_123*qdd[24]+ROcp2_420*qdd[23]+qd[23]*(OMcp2_222*ROcp2_620-OMcp2_322*ROcp2_520)+qd[24]*(
 OMcp2_223*ROcp2_323-OMcp2_323*ROcp2_223);
    OPcp2_224 = OPcp2_222+ROcp2_223*qdd[24]+ROcp2_520*qdd[23]-qd[23]*(OMcp2_122*ROcp2_620-OMcp2_322*ROcp2_420)-qd[24]*(
 OMcp2_123*ROcp2_323-OMcp2_323*ROcp2_123);
    OPcp2_324 = OPcp2_322+ROcp2_323*qdd[24]+ROcp2_620*qdd[23]+qd[23]*(OMcp2_122*ROcp2_520-OMcp2_222*ROcp2_420)+qd[24]*(
 OMcp2_123*ROcp2_223-OMcp2_223*ROcp2_123);
    RLcp2_125 = ROcp2_123*s->dpt[1][17]+ROcp2_724*s->dpt[3][17];
    RLcp2_225 = ROcp2_223*s->dpt[1][17]+ROcp2_824*s->dpt[3][17];
    RLcp2_325 = ROcp2_323*s->dpt[1][17]+ROcp2_924*s->dpt[3][17];
    ORcp2_125 = OMcp2_224*RLcp2_325-OMcp2_324*RLcp2_225;
    ORcp2_225 = -(OMcp2_124*RLcp2_325-OMcp2_324*RLcp2_125);
    ORcp2_325 = OMcp2_124*RLcp2_225-OMcp2_224*RLcp2_125;
    RLcp2_128 = ROcp2_727*q[28];
    RLcp2_228 = ROcp2_827*q[28];
    RLcp2_328 = ROcp2_927*q[28];
    ORcp2_128 = OMcp2_224*RLcp2_328-OMcp2_324*RLcp2_228;
    ORcp2_228 = -(OMcp2_124*RLcp2_328-OMcp2_324*RLcp2_128);
    ORcp2_328 = OMcp2_124*RLcp2_228-OMcp2_224*RLcp2_128;
    RLcp2_129 = ROcp2_427*q[29];
    RLcp2_229 = ROcp2_527*q[29];
    RLcp2_329 = ROcp2_627*q[29];
    ORcp2_129 = OMcp2_224*RLcp2_329-OMcp2_324*RLcp2_229;
    ORcp2_229 = -(OMcp2_124*RLcp2_329-OMcp2_324*RLcp2_129);
    ORcp2_329 = OMcp2_124*RLcp2_229-OMcp2_224*RLcp2_129;
    RLcp2_130 = ROcp2_126*q[30];
    RLcp2_230 = ROcp2_226*q[30];
    RLcp2_330 = ROcp2_326*q[30];
    ORcp2_130 = OMcp2_224*RLcp2_330-OMcp2_324*RLcp2_230;
    ORcp2_230 = -(OMcp2_124*RLcp2_330-OMcp2_324*RLcp2_130);
    ORcp2_330 = OMcp2_124*RLcp2_230-OMcp2_224*RLcp2_130;
    RLcp2_158 = ROcp2_727*s->dpt[3][18];
    RLcp2_258 = ROcp2_827*s->dpt[3][18];
    RLcp2_358 = ROcp2_927*s->dpt[3][18];
    POcp2_158 = RLcp2_119+RLcp2_120+RLcp2_122+RLcp2_123+RLcp2_125+RLcp2_128+RLcp2_129+RLcp2_130+RLcp2_158+q[1];
    POcp2_258 = RLcp2_219+RLcp2_220+RLcp2_222+RLcp2_223+RLcp2_225+RLcp2_228+RLcp2_229+RLcp2_230+RLcp2_258+q[2];
    POcp2_358 = RLcp2_319+RLcp2_320+RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358+q[3];
    JTcp2_258_4 = -(RLcp2_319+RLcp2_320+RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358);
    JTcp2_358_4 = RLcp2_219+RLcp2_220+RLcp2_222+RLcp2_223+RLcp2_225+RLcp2_228+RLcp2_229+RLcp2_230+RLcp2_258;
    JTcp2_158_5 = C4*(RLcp2_319+RLcp2_320+RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330)-S4*(RLcp2_219+
 RLcp2_220)-S4*(RLcp2_222+RLcp2_223)-S4*(RLcp2_225+RLcp2_228)-S4*(RLcp2_229+RLcp2_230)-RLcp2_258*S4+RLcp2_358*C4;
    JTcp2_258_5 = S4*(RLcp2_119+RLcp2_120+RLcp2_122+RLcp2_123+RLcp2_125+RLcp2_128+RLcp2_129+RLcp2_130+RLcp2_158);
    JTcp2_358_5 = -C4*(RLcp2_119+RLcp2_120+RLcp2_122+RLcp2_123+RLcp2_125+RLcp2_128+RLcp2_129+RLcp2_130+RLcp2_158);
    JTcp2_158_6 = ROcp2_85*(RLcp2_319+RLcp2_320+RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330)-ROcp2_95*(
 RLcp2_219+RLcp2_220)-ROcp2_95*(RLcp2_222+RLcp2_223)-ROcp2_95*(RLcp2_225+RLcp2_228)-ROcp2_95*(RLcp2_229+RLcp2_230)-RLcp2_258*
 ROcp2_95+RLcp2_358*ROcp2_85;
    JTcp2_258_6 = -(RLcp2_358*S5-ROcp2_95*(RLcp2_119+RLcp2_120+RLcp2_122+RLcp2_123+RLcp2_125+RLcp2_128+RLcp2_129+RLcp2_130
 +RLcp2_158)+S5*(RLcp2_319+RLcp2_320)+S5*(RLcp2_322+RLcp2_323)+S5*(RLcp2_325+RLcp2_328)+S5*(RLcp2_329+RLcp2_330));
    JTcp2_358_6 = RLcp2_258*S5-ROcp2_85*(RLcp2_119+RLcp2_120+RLcp2_122+RLcp2_123+RLcp2_125+RLcp2_128+RLcp2_129+RLcp2_130+
 RLcp2_158)+S5*(RLcp2_219+RLcp2_220)+S5*(RLcp2_222+RLcp2_223)+S5*(RLcp2_225+RLcp2_228)+S5*(RLcp2_229+RLcp2_230);
    JTcp2_158_7 = ROcp2_26*(RLcp2_320+RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_36*(
 RLcp2_220+RLcp2_222)-ROcp2_36*(RLcp2_223+RLcp2_225)-ROcp2_36*(RLcp2_228+RLcp2_229)-ROcp2_36*(RLcp2_230+RLcp2_258);
    JTcp2_258_7 = -(ROcp2_16*(RLcp2_320+RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_36*(
 RLcp2_120+RLcp2_122)-ROcp2_36*(RLcp2_123+RLcp2_125)-ROcp2_36*(RLcp2_128+RLcp2_129)-ROcp2_36*(RLcp2_130+RLcp2_158));
    JTcp2_358_7 = ROcp2_16*(RLcp2_220+RLcp2_222+RLcp2_223+RLcp2_225+RLcp2_228+RLcp2_229+RLcp2_230+RLcp2_258)-ROcp2_26*(
 RLcp2_120+RLcp2_122)-ROcp2_26*(RLcp2_123+RLcp2_125)-ROcp2_26*(RLcp2_128+RLcp2_129)-ROcp2_26*(RLcp2_130+RLcp2_158);
    JTcp2_158_8 = ROcp2_819*(RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330)-ROcp2_919*(RLcp2_222+RLcp2_223)-
 ROcp2_919*(RLcp2_225+RLcp2_228)-ROcp2_919*(RLcp2_229+RLcp2_230)-RLcp2_258*ROcp2_919+RLcp2_358*ROcp2_819;
    JTcp2_258_8 = RLcp2_158*ROcp2_919-RLcp2_358*ROcp2_719-ROcp2_719*(RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+
 RLcp2_330)+ROcp2_919*(RLcp2_122+RLcp2_123)+ROcp2_919*(RLcp2_125+RLcp2_128)+ROcp2_919*(RLcp2_129+RLcp2_130);
    JTcp2_358_8 = ROcp2_719*(RLcp2_222+RLcp2_223+RLcp2_225+RLcp2_228+RLcp2_229+RLcp2_230)-ROcp2_819*(RLcp2_122+RLcp2_123)-
 ROcp2_819*(RLcp2_125+RLcp2_128)-ROcp2_819*(RLcp2_129+RLcp2_130)-RLcp2_158*ROcp2_819+RLcp2_258*ROcp2_719;
    JTcp2_158_9 = ROcp2_520*(RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330)-ROcp2_620*(RLcp2_222+RLcp2_223)-
 ROcp2_620*(RLcp2_225+RLcp2_228)-ROcp2_620*(RLcp2_229+RLcp2_230)-RLcp2_258*ROcp2_620+RLcp2_358*ROcp2_520;
    JTcp2_258_9 = RLcp2_158*ROcp2_620-RLcp2_358*ROcp2_420-ROcp2_420*(RLcp2_322+RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+
 RLcp2_330)+ROcp2_620*(RLcp2_122+RLcp2_123)+ROcp2_620*(RLcp2_125+RLcp2_128)+ROcp2_620*(RLcp2_129+RLcp2_130);
    JTcp2_358_9 = ROcp2_420*(RLcp2_222+RLcp2_223+RLcp2_225+RLcp2_228+RLcp2_229+RLcp2_230)-ROcp2_520*(RLcp2_122+RLcp2_123)-
 ROcp2_520*(RLcp2_125+RLcp2_128)-ROcp2_520*(RLcp2_129+RLcp2_130)-RLcp2_158*ROcp2_520+RLcp2_258*ROcp2_420;
    JTcp2_158_10 = ROcp2_520*(RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_620*(RLcp2_223+RLcp2_225)
 -ROcp2_620*(RLcp2_228+RLcp2_229)-ROcp2_620*(RLcp2_230+RLcp2_258);
    JTcp2_258_10 = -(ROcp2_420*(RLcp2_323+RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_620*(RLcp2_123+
 RLcp2_125)-ROcp2_620*(RLcp2_128+RLcp2_129)-ROcp2_620*(RLcp2_130+RLcp2_158));
    JTcp2_358_10 = ROcp2_420*(RLcp2_223+RLcp2_225+RLcp2_228+RLcp2_229+RLcp2_230+RLcp2_258)-ROcp2_520*(RLcp2_123+RLcp2_125)
 -ROcp2_520*(RLcp2_128+RLcp2_129)-ROcp2_520*(RLcp2_130+RLcp2_158);
    JTcp2_158_11 = ROcp2_520*(RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330)-ROcp2_620*(RLcp2_225+RLcp2_228)-ROcp2_620*(
 RLcp2_229+RLcp2_230)-RLcp2_258*ROcp2_620+RLcp2_358*ROcp2_520;
    JTcp2_258_11 = RLcp2_158*ROcp2_620-RLcp2_358*ROcp2_420-ROcp2_420*(RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330)+ROcp2_620*(
 RLcp2_125+RLcp2_128)+ROcp2_620*(RLcp2_129+RLcp2_130);
    JTcp2_358_11 = ROcp2_420*(RLcp2_225+RLcp2_228+RLcp2_229+RLcp2_230)-ROcp2_520*(RLcp2_125+RLcp2_128)-ROcp2_520*(
 RLcp2_129+RLcp2_130)-RLcp2_158*ROcp2_520+RLcp2_258*ROcp2_420;
    JTcp2_158_12 = ROcp2_223*(RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330)-ROcp2_323*(RLcp2_225+RLcp2_228)-ROcp2_323*(
 RLcp2_229+RLcp2_230)-RLcp2_258*ROcp2_323+RLcp2_358*ROcp2_223;
    JTcp2_258_12 = RLcp2_158*ROcp2_323-RLcp2_358*ROcp2_123-ROcp2_123*(RLcp2_325+RLcp2_328+RLcp2_329+RLcp2_330)+ROcp2_323*(
 RLcp2_125+RLcp2_128)+ROcp2_323*(RLcp2_129+RLcp2_130);
    JTcp2_358_12 = ROcp2_123*(RLcp2_225+RLcp2_228+RLcp2_229+RLcp2_230)-ROcp2_223*(RLcp2_125+RLcp2_128)-ROcp2_223*(
 RLcp2_129+RLcp2_130)-RLcp2_158*ROcp2_223+RLcp2_258*ROcp2_123;
    JTcp2_158_13 = ROcp2_824*(RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_924*(RLcp2_228+RLcp2_229)-ROcp2_924*(
 RLcp2_230+RLcp2_258);
    JTcp2_258_13 = -(ROcp2_724*(RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_924*(RLcp2_128+RLcp2_129)-ROcp2_924*(
 RLcp2_130+RLcp2_158));
    JTcp2_358_13 = ROcp2_724*(RLcp2_228+RLcp2_229+RLcp2_230+RLcp2_258)-ROcp2_824*(RLcp2_128+RLcp2_129)-ROcp2_824*(
 RLcp2_130+RLcp2_158);
    JTcp2_158_14 = ROcp2_525*(RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_625*(RLcp2_228+RLcp2_229)-ROcp2_625*(
 RLcp2_230+RLcp2_258);
    JTcp2_258_14 = -(ROcp2_425*(RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_625*(RLcp2_128+RLcp2_129)-ROcp2_625*(
 RLcp2_130+RLcp2_158));
    JTcp2_358_14 = ROcp2_425*(RLcp2_228+RLcp2_229+RLcp2_230+RLcp2_258)-ROcp2_525*(RLcp2_128+RLcp2_129)-ROcp2_525*(
 RLcp2_130+RLcp2_158);
    JTcp2_158_15 = ROcp2_226*(RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_326*(RLcp2_228+RLcp2_229)-ROcp2_326*(
 RLcp2_230+RLcp2_258);
    JTcp2_258_15 = -(ROcp2_126*(RLcp2_328+RLcp2_329+RLcp2_330+RLcp2_358)-ROcp2_326*(RLcp2_128+RLcp2_129)-ROcp2_326*(
 RLcp2_130+RLcp2_158));
    JTcp2_358_15 = ROcp2_126*(RLcp2_228+RLcp2_229+RLcp2_230+RLcp2_258)-ROcp2_226*(RLcp2_128+RLcp2_129)-ROcp2_226*(
 RLcp2_130+RLcp2_158);
    ORcp2_158 = OMcp2_224*RLcp2_358-OMcp2_324*RLcp2_258;
    ORcp2_258 = -(OMcp2_124*RLcp2_358-OMcp2_324*RLcp2_158);
    ORcp2_358 = OMcp2_124*RLcp2_258-OMcp2_224*RLcp2_158;
    VIcp2_158 = ORcp2_119+ORcp2_120+ORcp2_122+ORcp2_123+ORcp2_125+ORcp2_128+ORcp2_129+ORcp2_130+ORcp2_158+qd[1];
    VIcp2_258 = ORcp2_219+ORcp2_220+ORcp2_222+ORcp2_223+ORcp2_225+ORcp2_228+ORcp2_229+ORcp2_230+ORcp2_258+qd[2];
    VIcp2_358 = ORcp2_319+ORcp2_320+ORcp2_322+ORcp2_323+ORcp2_325+ORcp2_328+ORcp2_329+ORcp2_330+ORcp2_358+qd[3];
    ACcp2_158 = qdd[1]+OMcp2_219*ORcp2_320+OMcp2_221*ORcp2_322+OMcp2_222*ORcp2_323+OMcp2_224*ORcp2_325+OMcp2_224*ORcp2_328
 +OMcp2_224*ORcp2_329+OMcp2_224*ORcp2_330+OMcp2_224*ORcp2_358+OMcp2_26*ORcp2_319-OMcp2_319*ORcp2_220-OMcp2_321*ORcp2_222-
 OMcp2_322*ORcp2_223-OMcp2_324*ORcp2_225-OMcp2_324*ORcp2_228-OMcp2_324*ORcp2_229-OMcp2_324*ORcp2_230-OMcp2_324*ORcp2_258-
 OMcp2_36*ORcp2_219+OPcp2_219*RLcp2_320+OPcp2_221*RLcp2_322+OPcp2_222*RLcp2_323+OPcp2_224*RLcp2_325+OPcp2_224*RLcp2_328+
 OPcp2_224*RLcp2_329+OPcp2_224*RLcp2_330+OPcp2_224*RLcp2_358+OPcp2_26*RLcp2_319-OPcp2_319*RLcp2_220-OPcp2_321*RLcp2_222-
 OPcp2_322*RLcp2_223-OPcp2_324*RLcp2_225-OPcp2_324*RLcp2_228-OPcp2_324*RLcp2_229-OPcp2_324*RLcp2_230-OPcp2_324*RLcp2_258-
 OPcp2_36*RLcp2_219;
    ACcp2_258 = qdd[2]-OMcp2_119*ORcp2_320-OMcp2_121*ORcp2_322-OMcp2_122*ORcp2_323-OMcp2_124*ORcp2_325-OMcp2_124*ORcp2_328
 -OMcp2_124*ORcp2_329-OMcp2_124*ORcp2_330-OMcp2_124*ORcp2_358-OMcp2_16*ORcp2_319+OMcp2_319*ORcp2_120+OMcp2_321*ORcp2_122+
 OMcp2_322*ORcp2_123+OMcp2_324*ORcp2_125+OMcp2_324*ORcp2_128+OMcp2_324*ORcp2_129+OMcp2_324*ORcp2_130+OMcp2_324*ORcp2_158+
 OMcp2_36*ORcp2_119-OPcp2_119*RLcp2_320-OPcp2_121*RLcp2_322-OPcp2_122*RLcp2_323-OPcp2_124*RLcp2_325-OPcp2_124*RLcp2_328-
 OPcp2_124*RLcp2_329-OPcp2_124*RLcp2_330-OPcp2_124*RLcp2_358-OPcp2_16*RLcp2_319+OPcp2_319*RLcp2_120+OPcp2_321*RLcp2_122+
 OPcp2_322*RLcp2_123+OPcp2_324*RLcp2_125+OPcp2_324*RLcp2_128+OPcp2_324*RLcp2_129+OPcp2_324*RLcp2_130+OPcp2_324*RLcp2_158+
 OPcp2_36*RLcp2_119;
    ACcp2_358 = qdd[3]+OMcp2_119*ORcp2_220+OMcp2_121*ORcp2_222+OMcp2_122*ORcp2_223+OMcp2_124*ORcp2_225+OMcp2_124*ORcp2_228
 +OMcp2_124*ORcp2_229+OMcp2_124*ORcp2_230+OMcp2_124*ORcp2_258+OMcp2_16*ORcp2_219-OMcp2_219*ORcp2_120-OMcp2_221*ORcp2_122-
 OMcp2_222*ORcp2_123-OMcp2_224*ORcp2_125-OMcp2_224*ORcp2_128-OMcp2_224*ORcp2_129-OMcp2_224*ORcp2_130-OMcp2_224*ORcp2_158-
 OMcp2_26*ORcp2_119+OPcp2_119*RLcp2_220+OPcp2_121*RLcp2_222+OPcp2_122*RLcp2_223+OPcp2_124*RLcp2_225+OPcp2_124*RLcp2_228+
 OPcp2_124*RLcp2_229+OPcp2_124*RLcp2_230+OPcp2_124*RLcp2_258+OPcp2_16*RLcp2_219-OPcp2_219*RLcp2_120-OPcp2_221*RLcp2_122-
 OPcp2_222*RLcp2_123-OPcp2_224*RLcp2_125-OPcp2_224*RLcp2_128-OPcp2_224*RLcp2_129-OPcp2_224*RLcp2_130-OPcp2_224*RLcp2_158-
 OPcp2_26*RLcp2_119;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_158;
    sens->P[2] = POcp2_258;
    sens->P[3] = POcp2_358;
    sens->R[1][1] = ROcp2_126;
    sens->R[1][2] = ROcp2_226;
    sens->R[1][3] = ROcp2_326;
    sens->R[2][1] = ROcp2_427;
    sens->R[2][2] = ROcp2_527;
    sens->R[2][3] = ROcp2_627;
    sens->R[3][1] = ROcp2_727;
    sens->R[3][2] = ROcp2_827;
    sens->R[3][3] = ROcp2_927;
    sens->V[1] = VIcp2_158;
    sens->V[2] = VIcp2_258;
    sens->V[3] = VIcp2_358;
    sens->OM[1] = OMcp2_124;
    sens->OM[2] = OMcp2_224;
    sens->OM[3] = OMcp2_324;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp2_158_5;
    sens->J[1][6] = JTcp2_158_6;
    sens->J[1][19] = JTcp2_158_7;
    sens->J[1][20] = JTcp2_158_8;
    sens->J[1][21] = JTcp2_158_9;
    sens->J[1][22] = JTcp2_158_10;
    sens->J[1][23] = JTcp2_158_11;
    sens->J[1][24] = JTcp2_158_12;
    sens->J[1][25] = JTcp2_158_13;
    sens->J[1][26] = JTcp2_158_14;
    sens->J[1][27] = JTcp2_158_15;
    sens->J[1][28] = ROcp2_727;
    sens->J[1][29] = ROcp2_427;
    sens->J[1][30] = ROcp2_126;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp2_258_4;
    sens->J[2][5] = JTcp2_258_5;
    sens->J[2][6] = JTcp2_258_6;
    sens->J[2][19] = JTcp2_258_7;
    sens->J[2][20] = JTcp2_258_8;
    sens->J[2][21] = JTcp2_258_9;
    sens->J[2][22] = JTcp2_258_10;
    sens->J[2][23] = JTcp2_258_11;
    sens->J[2][24] = JTcp2_258_12;
    sens->J[2][25] = JTcp2_258_13;
    sens->J[2][26] = JTcp2_258_14;
    sens->J[2][27] = JTcp2_258_15;
    sens->J[2][28] = ROcp2_827;
    sens->J[2][29] = ROcp2_527;
    sens->J[2][30] = ROcp2_226;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp2_358_4;
    sens->J[3][5] = JTcp2_358_5;
    sens->J[3][6] = JTcp2_358_6;
    sens->J[3][19] = JTcp2_358_7;
    sens->J[3][20] = JTcp2_358_8;
    sens->J[3][21] = JTcp2_358_9;
    sens->J[3][22] = JTcp2_358_10;
    sens->J[3][23] = JTcp2_358_11;
    sens->J[3][24] = JTcp2_358_12;
    sens->J[3][25] = JTcp2_358_13;
    sens->J[3][26] = JTcp2_358_14;
    sens->J[3][27] = JTcp2_358_15;
    sens->J[3][28] = ROcp2_927;
    sens->J[3][29] = ROcp2_627;
    sens->J[3][30] = ROcp2_326;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp2_16;
    sens->J[4][20] = ROcp2_719;
    sens->J[4][21] = ROcp2_420;
    sens->J[4][22] = ROcp2_420;
    sens->J[4][23] = ROcp2_420;
    sens->J[4][24] = ROcp2_123;
    sens->J[4][25] = ROcp2_724;
    sens->J[4][26] = ROcp2_425;
    sens->J[4][27] = ROcp2_126;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp2_85;
    sens->J[5][19] = ROcp2_26;
    sens->J[5][20] = ROcp2_819;
    sens->J[5][21] = ROcp2_520;
    sens->J[5][22] = ROcp2_520;
    sens->J[5][23] = ROcp2_520;
    sens->J[5][24] = ROcp2_223;
    sens->J[5][25] = ROcp2_824;
    sens->J[5][26] = ROcp2_525;
    sens->J[5][27] = ROcp2_226;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp2_95;
    sens->J[6][19] = ROcp2_36;
    sens->J[6][20] = ROcp2_919;
    sens->J[6][21] = ROcp2_620;
    sens->J[6][22] = ROcp2_620;
    sens->J[6][23] = ROcp2_620;
    sens->J[6][24] = ROcp2_323;
    sens->J[6][25] = ROcp2_924;
    sens->J[6][26] = ROcp2_625;
    sens->J[6][27] = ROcp2_326;
    sens->A[1] = ACcp2_158;
    sens->A[2] = ACcp2_258;
    sens->A[3] = ACcp2_358;
    sens->OMP[1] = OPcp2_124;
    sens->OMP[2] = OPcp2_224;
    sens->OMP[3] = OPcp2_324;
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_0_1 = = 
 
// Sensor Kinematics 


    ROcp3_25 = S4*S5;
    ROcp3_35 = -C4*S5;
    ROcp3_85 = -S4*C5;
    ROcp3_95 = C4*C5;
    ROcp3_16 = C5*C6;
    ROcp3_26 = ROcp3_25*C6+C4*S6;
    ROcp3_36 = ROcp3_35*C6+S4*S6;
    ROcp3_46 = -C5*S6;
    ROcp3_56 = -(ROcp3_25*S6-C4*C6);
    ROcp3_66 = -(ROcp3_35*S6-S4*C6);
    OMcp3_25 = qd[5]*C4;
    OMcp3_35 = qd[5]*S4;
    OMcp3_16 = qd[4]+qd[6]*S5;
    OMcp3_26 = OMcp3_25+ROcp3_85*qd[6];
    OMcp3_36 = OMcp3_35+ROcp3_95*qd[6];
    OPcp3_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp3_26 = ROcp3_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp3_35*S5-ROcp3_95*qd[4]);
    OPcp3_36 = ROcp3_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp3_25*S5-ROcp3_85*qd[4]);

// = = Block_1_0_0_4_0_4 = = 
 
// Sensor Kinematics 


    ROcp3_131 = ROcp3_16*C31-S31*S5;
    ROcp3_231 = ROcp3_26*C31-ROcp3_85*S31;
    ROcp3_331 = ROcp3_36*C31-ROcp3_95*S31;
    ROcp3_731 = ROcp3_16*S31+C31*S5;
    ROcp3_831 = ROcp3_26*S31+ROcp3_85*C31;
    ROcp3_931 = ROcp3_36*S31+ROcp3_95*C31;
    ROcp3_432 = ROcp3_46*C32+ROcp3_731*S32;
    ROcp3_532 = ROcp3_56*C32+ROcp3_831*S32;
    ROcp3_632 = ROcp3_66*C32+ROcp3_931*S32;
    ROcp3_732 = -(ROcp3_46*S32-ROcp3_731*C32);
    ROcp3_832 = -(ROcp3_56*S32-ROcp3_831*C32);
    ROcp3_932 = -(ROcp3_66*S32-ROcp3_931*C32);
    RLcp3_131 = ROcp3_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp3_231 = ROcp3_26*s->dpt[1][3]+ROcp3_85*s->dpt[3][3];
    RLcp3_331 = ROcp3_36*s->dpt[1][3]+ROcp3_95*s->dpt[3][3];
    OMcp3_131 = OMcp3_16+ROcp3_46*qd[31];
    OMcp3_231 = OMcp3_26+ROcp3_56*qd[31];
    OMcp3_331 = OMcp3_36+ROcp3_66*qd[31];
    ORcp3_131 = OMcp3_26*RLcp3_331-OMcp3_36*RLcp3_231;
    ORcp3_231 = -(OMcp3_16*RLcp3_331-OMcp3_36*RLcp3_131);
    ORcp3_331 = OMcp3_16*RLcp3_231-OMcp3_26*RLcp3_131;
    OMcp3_132 = OMcp3_131+ROcp3_131*qd[32];
    OMcp3_232 = OMcp3_231+ROcp3_231*qd[32];
    OMcp3_332 = OMcp3_331+ROcp3_331*qd[32];
    OPcp3_132 = OPcp3_16+ROcp3_131*qdd[32]+ROcp3_46*qdd[31]+qd[31]*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)+qd[32]*(OMcp3_231
 *ROcp3_331-OMcp3_331*ROcp3_231);
    OPcp3_232 = OPcp3_26+ROcp3_231*qdd[32]+ROcp3_56*qdd[31]-qd[31]*(OMcp3_16*ROcp3_66-OMcp3_36*ROcp3_46)-qd[32]*(OMcp3_131
 *ROcp3_331-OMcp3_331*ROcp3_131);
    OPcp3_332 = OPcp3_36+ROcp3_331*qdd[32]+ROcp3_66*qdd[31]+qd[31]*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)+qd[32]*(OMcp3_131
 *ROcp3_231-OMcp3_231*ROcp3_131);
    RLcp3_159 = ROcp3_131*s->dpt[1][20]+ROcp3_432*s->dpt[2][20]+ROcp3_732*s->dpt[3][20];
    RLcp3_259 = ROcp3_231*s->dpt[1][20]+ROcp3_532*s->dpt[2][20]+ROcp3_832*s->dpt[3][20];
    RLcp3_359 = ROcp3_331*s->dpt[1][20]+ROcp3_632*s->dpt[2][20]+ROcp3_932*s->dpt[3][20];
    POcp3_159 = RLcp3_131+RLcp3_159+q[1];
    POcp3_259 = RLcp3_231+RLcp3_259+q[2];
    POcp3_359 = RLcp3_331+RLcp3_359+q[3];
    JTcp3_259_4 = -(RLcp3_331+RLcp3_359);
    JTcp3_359_4 = RLcp3_231+RLcp3_259;
    JTcp3_159_5 = C4*(RLcp3_331+RLcp3_359)-S4*(RLcp3_231+RLcp3_259);
    JTcp3_259_5 = S4*(RLcp3_131+RLcp3_159);
    JTcp3_359_5 = -C4*(RLcp3_131+RLcp3_159);
    JTcp3_159_6 = ROcp3_85*(RLcp3_331+RLcp3_359)-ROcp3_95*(RLcp3_231+RLcp3_259);
    JTcp3_259_6 = ROcp3_95*(RLcp3_131+RLcp3_159)-S5*(RLcp3_331+RLcp3_359);
    JTcp3_359_6 = -(ROcp3_85*(RLcp3_131+RLcp3_159)-S5*(RLcp3_231+RLcp3_259));
    JTcp3_159_7 = -(RLcp3_259*ROcp3_66-RLcp3_359*ROcp3_56);
    JTcp3_259_7 = RLcp3_159*ROcp3_66-RLcp3_359*ROcp3_46;
    JTcp3_359_7 = -(RLcp3_159*ROcp3_56-RLcp3_259*ROcp3_46);
    JTcp3_159_8 = -(RLcp3_259*ROcp3_331-RLcp3_359*ROcp3_231);
    JTcp3_259_8 = RLcp3_159*ROcp3_331-RLcp3_359*ROcp3_131;
    JTcp3_359_8 = -(RLcp3_159*ROcp3_231-RLcp3_259*ROcp3_131);
    ORcp3_159 = OMcp3_232*RLcp3_359-OMcp3_332*RLcp3_259;
    ORcp3_259 = -(OMcp3_132*RLcp3_359-OMcp3_332*RLcp3_159);
    ORcp3_359 = OMcp3_132*RLcp3_259-OMcp3_232*RLcp3_159;
    VIcp3_159 = ORcp3_131+ORcp3_159+qd[1];
    VIcp3_259 = ORcp3_231+ORcp3_259+qd[2];
    VIcp3_359 = ORcp3_331+ORcp3_359+qd[3];
    ACcp3_159 = qdd[1]+OMcp3_232*ORcp3_359+OMcp3_26*ORcp3_331-OMcp3_332*ORcp3_259-OMcp3_36*ORcp3_231+OPcp3_232*RLcp3_359+
 OPcp3_26*RLcp3_331-OPcp3_332*RLcp3_259-OPcp3_36*RLcp3_231;
    ACcp3_259 = qdd[2]-OMcp3_132*ORcp3_359-OMcp3_16*ORcp3_331+OMcp3_332*ORcp3_159+OMcp3_36*ORcp3_131-OPcp3_132*RLcp3_359-
 OPcp3_16*RLcp3_331+OPcp3_332*RLcp3_159+OPcp3_36*RLcp3_131;
    ACcp3_359 = qdd[3]+OMcp3_132*ORcp3_259+OMcp3_16*ORcp3_231-OMcp3_232*ORcp3_159-OMcp3_26*ORcp3_131+OPcp3_132*RLcp3_259+
 OPcp3_16*RLcp3_231-OPcp3_232*RLcp3_159-OPcp3_26*RLcp3_131;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_159;
    sens->P[2] = POcp3_259;
    sens->P[3] = POcp3_359;
    sens->R[1][1] = ROcp3_131;
    sens->R[1][2] = ROcp3_231;
    sens->R[1][3] = ROcp3_331;
    sens->R[2][1] = ROcp3_432;
    sens->R[2][2] = ROcp3_532;
    sens->R[2][3] = ROcp3_632;
    sens->R[3][1] = ROcp3_732;
    sens->R[3][2] = ROcp3_832;
    sens->R[3][3] = ROcp3_932;
    sens->V[1] = VIcp3_159;
    sens->V[2] = VIcp3_259;
    sens->V[3] = VIcp3_359;
    sens->OM[1] = OMcp3_132;
    sens->OM[2] = OMcp3_232;
    sens->OM[3] = OMcp3_332;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp3_159_5;
    sens->J[1][6] = JTcp3_159_6;
    sens->J[1][31] = JTcp3_159_7;
    sens->J[1][32] = JTcp3_159_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp3_259_4;
    sens->J[2][5] = JTcp3_259_5;
    sens->J[2][6] = JTcp3_259_6;
    sens->J[2][31] = JTcp3_259_7;
    sens->J[2][32] = JTcp3_259_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp3_359_4;
    sens->J[3][5] = JTcp3_359_5;
    sens->J[3][6] = JTcp3_359_6;
    sens->J[3][31] = JTcp3_359_7;
    sens->J[3][32] = JTcp3_359_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp3_46;
    sens->J[4][32] = ROcp3_131;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp3_85;
    sens->J[5][31] = ROcp3_56;
    sens->J[5][32] = ROcp3_231;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp3_95;
    sens->J[6][31] = ROcp3_66;
    sens->J[6][32] = ROcp3_331;
    sens->A[1] = ACcp3_159;
    sens->A[2] = ACcp3_259;
    sens->A[3] = ACcp3_359;
    sens->OMP[1] = OPcp3_132;
    sens->OMP[2] = OPcp3_232;
    sens->OMP[3] = OPcp3_332;
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
// Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    ROcp4_16 = C5*C6;
    ROcp4_26 = ROcp4_25*C6+C4*S6;
    ROcp4_36 = ROcp4_35*C6+S4*S6;
    ROcp4_46 = -C5*S6;
    ROcp4_56 = -(ROcp4_25*S6-C4*C6);
    ROcp4_66 = -(ROcp4_35*S6-S4*C6);
    OMcp4_25 = qd[5]*C4;
    OMcp4_35 = qd[5]*S4;
    OMcp4_16 = qd[4]+qd[6]*S5;
    OMcp4_26 = OMcp4_25+ROcp4_85*qd[6];
    OMcp4_36 = OMcp4_35+ROcp4_95*qd[6];
    OPcp4_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp4_26 = ROcp4_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp4_35*S5-ROcp4_95*qd[4]);
    OPcp4_36 = ROcp4_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp4_25*S5-ROcp4_85*qd[4]);

// = = Block_1_0_0_5_0_4 = = 
 
// Sensor Kinematics 


    ROcp4_131 = ROcp4_16*C31-S31*S5;
    ROcp4_231 = ROcp4_26*C31-ROcp4_85*S31;
    ROcp4_331 = ROcp4_36*C31-ROcp4_95*S31;
    ROcp4_731 = ROcp4_16*S31+C31*S5;
    ROcp4_831 = ROcp4_26*S31+ROcp4_85*C31;
    ROcp4_931 = ROcp4_36*S31+ROcp4_95*C31;
    ROcp4_432 = ROcp4_46*C32+ROcp4_731*S32;
    ROcp4_532 = ROcp4_56*C32+ROcp4_831*S32;
    ROcp4_632 = ROcp4_66*C32+ROcp4_931*S32;
    ROcp4_732 = -(ROcp4_46*S32-ROcp4_731*C32);
    ROcp4_832 = -(ROcp4_56*S32-ROcp4_831*C32);
    ROcp4_932 = -(ROcp4_66*S32-ROcp4_931*C32);
    ROcp4_133 = ROcp4_131*C33-ROcp4_732*S33;
    ROcp4_233 = ROcp4_231*C33-ROcp4_832*S33;
    ROcp4_333 = ROcp4_331*C33-ROcp4_932*S33;
    ROcp4_733 = ROcp4_131*S33+ROcp4_732*C33;
    ROcp4_833 = ROcp4_231*S33+ROcp4_832*C33;
    ROcp4_933 = ROcp4_331*S33+ROcp4_932*C33;
    RLcp4_131 = ROcp4_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp4_231 = ROcp4_26*s->dpt[1][3]+ROcp4_85*s->dpt[3][3];
    RLcp4_331 = ROcp4_36*s->dpt[1][3]+ROcp4_95*s->dpt[3][3];
    OMcp4_131 = OMcp4_16+ROcp4_46*qd[31];
    OMcp4_231 = OMcp4_26+ROcp4_56*qd[31];
    OMcp4_331 = OMcp4_36+ROcp4_66*qd[31];
    ORcp4_131 = OMcp4_26*RLcp4_331-OMcp4_36*RLcp4_231;
    ORcp4_231 = -(OMcp4_16*RLcp4_331-OMcp4_36*RLcp4_131);
    ORcp4_331 = OMcp4_16*RLcp4_231-OMcp4_26*RLcp4_131;
    OMcp4_132 = OMcp4_131+ROcp4_131*qd[32];
    OMcp4_232 = OMcp4_231+ROcp4_231*qd[32];
    OMcp4_332 = OMcp4_331+ROcp4_331*qd[32];
    OMcp4_133 = OMcp4_132+ROcp4_432*qd[33];
    OMcp4_233 = OMcp4_232+ROcp4_532*qd[33];
    OMcp4_333 = OMcp4_332+ROcp4_632*qd[33];
    OPcp4_133 = OPcp4_16+ROcp4_131*qdd[32]+ROcp4_432*qdd[33]+ROcp4_46*qdd[31]+qd[31]*(OMcp4_26*ROcp4_66-OMcp4_36*ROcp4_56)
 +qd[32]*(OMcp4_231*ROcp4_331-OMcp4_331*ROcp4_231)+qd[33]*(OMcp4_232*ROcp4_632-OMcp4_332*ROcp4_532);
    OPcp4_233 = OPcp4_26+ROcp4_231*qdd[32]+ROcp4_532*qdd[33]+ROcp4_56*qdd[31]-qd[31]*(OMcp4_16*ROcp4_66-OMcp4_36*ROcp4_46)
 -qd[32]*(OMcp4_131*ROcp4_331-OMcp4_331*ROcp4_131)-qd[33]*(OMcp4_132*ROcp4_632-OMcp4_332*ROcp4_432);
    OPcp4_333 = OPcp4_36+ROcp4_331*qdd[32]+ROcp4_632*qdd[33]+ROcp4_66*qdd[31]+qd[31]*(OMcp4_16*ROcp4_56-OMcp4_26*ROcp4_46)
 +qd[32]*(OMcp4_131*ROcp4_231-OMcp4_231*ROcp4_131)+qd[33]*(OMcp4_132*ROcp4_532-OMcp4_232*ROcp4_432);
    RLcp4_160 = ROcp4_133*s->dpt[1][22]+ROcp4_432*s->dpt[2][22]+ROcp4_733*s->dpt[3][22];
    RLcp4_260 = ROcp4_233*s->dpt[1][22]+ROcp4_532*s->dpt[2][22]+ROcp4_833*s->dpt[3][22];
    RLcp4_360 = ROcp4_333*s->dpt[1][22]+ROcp4_632*s->dpt[2][22]+ROcp4_933*s->dpt[3][22];
    POcp4_160 = RLcp4_131+RLcp4_160+q[1];
    POcp4_260 = RLcp4_231+RLcp4_260+q[2];
    POcp4_360 = RLcp4_331+RLcp4_360+q[3];
    JTcp4_260_4 = -(RLcp4_331+RLcp4_360);
    JTcp4_360_4 = RLcp4_231+RLcp4_260;
    JTcp4_160_5 = C4*(RLcp4_331+RLcp4_360)-S4*(RLcp4_231+RLcp4_260);
    JTcp4_260_5 = S4*(RLcp4_131+RLcp4_160);
    JTcp4_360_5 = -C4*(RLcp4_131+RLcp4_160);
    JTcp4_160_6 = ROcp4_85*(RLcp4_331+RLcp4_360)-ROcp4_95*(RLcp4_231+RLcp4_260);
    JTcp4_260_6 = ROcp4_95*(RLcp4_131+RLcp4_160)-S5*(RLcp4_331+RLcp4_360);
    JTcp4_360_6 = -(ROcp4_85*(RLcp4_131+RLcp4_160)-S5*(RLcp4_231+RLcp4_260));
    JTcp4_160_7 = -(RLcp4_260*ROcp4_66-RLcp4_360*ROcp4_56);
    JTcp4_260_7 = RLcp4_160*ROcp4_66-RLcp4_360*ROcp4_46;
    JTcp4_360_7 = -(RLcp4_160*ROcp4_56-RLcp4_260*ROcp4_46);
    JTcp4_160_8 = -(RLcp4_260*ROcp4_331-RLcp4_360*ROcp4_231);
    JTcp4_260_8 = RLcp4_160*ROcp4_331-RLcp4_360*ROcp4_131;
    JTcp4_360_8 = -(RLcp4_160*ROcp4_231-RLcp4_260*ROcp4_131);
    JTcp4_160_9 = -(RLcp4_260*ROcp4_632-RLcp4_360*ROcp4_532);
    JTcp4_260_9 = RLcp4_160*ROcp4_632-RLcp4_360*ROcp4_432;
    JTcp4_360_9 = -(RLcp4_160*ROcp4_532-RLcp4_260*ROcp4_432);
    ORcp4_160 = OMcp4_233*RLcp4_360-OMcp4_333*RLcp4_260;
    ORcp4_260 = -(OMcp4_133*RLcp4_360-OMcp4_333*RLcp4_160);
    ORcp4_360 = OMcp4_133*RLcp4_260-OMcp4_233*RLcp4_160;
    VIcp4_160 = ORcp4_131+ORcp4_160+qd[1];
    VIcp4_260 = ORcp4_231+ORcp4_260+qd[2];
    VIcp4_360 = ORcp4_331+ORcp4_360+qd[3];
    ACcp4_160 = qdd[1]+OMcp4_233*ORcp4_360+OMcp4_26*ORcp4_331-OMcp4_333*ORcp4_260-OMcp4_36*ORcp4_231+OPcp4_233*RLcp4_360+
 OPcp4_26*RLcp4_331-OPcp4_333*RLcp4_260-OPcp4_36*RLcp4_231;
    ACcp4_260 = qdd[2]-OMcp4_133*ORcp4_360-OMcp4_16*ORcp4_331+OMcp4_333*ORcp4_160+OMcp4_36*ORcp4_131-OPcp4_133*RLcp4_360-
 OPcp4_16*RLcp4_331+OPcp4_333*RLcp4_160+OPcp4_36*RLcp4_131;
    ACcp4_360 = qdd[3]+OMcp4_133*ORcp4_260+OMcp4_16*ORcp4_231-OMcp4_233*ORcp4_160-OMcp4_26*ORcp4_131+OPcp4_133*RLcp4_260+
 OPcp4_16*RLcp4_231-OPcp4_233*RLcp4_160-OPcp4_26*RLcp4_131;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_160;
    sens->P[2] = POcp4_260;
    sens->P[3] = POcp4_360;
    sens->R[1][1] = ROcp4_133;
    sens->R[1][2] = ROcp4_233;
    sens->R[1][3] = ROcp4_333;
    sens->R[2][1] = ROcp4_432;
    sens->R[2][2] = ROcp4_532;
    sens->R[2][3] = ROcp4_632;
    sens->R[3][1] = ROcp4_733;
    sens->R[3][2] = ROcp4_833;
    sens->R[3][3] = ROcp4_933;
    sens->V[1] = VIcp4_160;
    sens->V[2] = VIcp4_260;
    sens->V[3] = VIcp4_360;
    sens->OM[1] = OMcp4_133;
    sens->OM[2] = OMcp4_233;
    sens->OM[3] = OMcp4_333;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp4_160_5;
    sens->J[1][6] = JTcp4_160_6;
    sens->J[1][31] = JTcp4_160_7;
    sens->J[1][32] = JTcp4_160_8;
    sens->J[1][33] = JTcp4_160_9;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp4_260_4;
    sens->J[2][5] = JTcp4_260_5;
    sens->J[2][6] = JTcp4_260_6;
    sens->J[2][31] = JTcp4_260_7;
    sens->J[2][32] = JTcp4_260_8;
    sens->J[2][33] = JTcp4_260_9;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp4_360_4;
    sens->J[3][5] = JTcp4_360_5;
    sens->J[3][6] = JTcp4_360_6;
    sens->J[3][31] = JTcp4_360_7;
    sens->J[3][32] = JTcp4_360_8;
    sens->J[3][33] = JTcp4_360_9;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp4_46;
    sens->J[4][32] = ROcp4_131;
    sens->J[4][33] = ROcp4_432;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp4_85;
    sens->J[5][31] = ROcp4_56;
    sens->J[5][32] = ROcp4_231;
    sens->J[5][33] = ROcp4_532;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp4_95;
    sens->J[6][31] = ROcp4_66;
    sens->J[6][32] = ROcp4_331;
    sens->J[6][33] = ROcp4_632;
    sens->A[1] = ACcp4_160;
    sens->A[2] = ACcp4_260;
    sens->A[3] = ACcp4_360;
    sens->OMP[1] = OPcp4_133;
    sens->OMP[2] = OPcp4_233;
    sens->OMP[3] = OPcp4_333;
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp5_25 = qd[5]*C4;
    OMcp5_35 = qd[5]*S4;
    OMcp5_16 = qd[4]+qd[6]*S5;
    OMcp5_26 = OMcp5_25+ROcp5_85*qd[6];
    OMcp5_36 = OMcp5_35+ROcp5_95*qd[6];
    OPcp5_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp5_26 = ROcp5_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp5_35*S5-ROcp5_95*qd[4]);
    OPcp5_36 = ROcp5_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp5_25*S5-ROcp5_85*qd[4]);

// = = Block_1_0_0_6_0_4 = = 
 
// Sensor Kinematics 


    ROcp5_131 = ROcp5_16*C31-S31*S5;
    ROcp5_231 = ROcp5_26*C31-ROcp5_85*S31;
    ROcp5_331 = ROcp5_36*C31-ROcp5_95*S31;
    ROcp5_731 = ROcp5_16*S31+C31*S5;
    ROcp5_831 = ROcp5_26*S31+ROcp5_85*C31;
    ROcp5_931 = ROcp5_36*S31+ROcp5_95*C31;
    ROcp5_432 = ROcp5_46*C32+ROcp5_731*S32;
    ROcp5_532 = ROcp5_56*C32+ROcp5_831*S32;
    ROcp5_632 = ROcp5_66*C32+ROcp5_931*S32;
    ROcp5_732 = -(ROcp5_46*S32-ROcp5_731*C32);
    ROcp5_832 = -(ROcp5_56*S32-ROcp5_831*C32);
    ROcp5_932 = -(ROcp5_66*S32-ROcp5_931*C32);
    ROcp5_133 = ROcp5_131*C33-ROcp5_732*S33;
    ROcp5_233 = ROcp5_231*C33-ROcp5_832*S33;
    ROcp5_333 = ROcp5_331*C33-ROcp5_932*S33;
    ROcp5_733 = ROcp5_131*S33+ROcp5_732*C33;
    ROcp5_833 = ROcp5_231*S33+ROcp5_832*C33;
    ROcp5_933 = ROcp5_331*S33+ROcp5_932*C33;
    ROcp5_134 = ROcp5_133*C34+ROcp5_432*S34;
    ROcp5_234 = ROcp5_233*C34+ROcp5_532*S34;
    ROcp5_334 = ROcp5_333*C34+ROcp5_632*S34;
    ROcp5_434 = -(ROcp5_133*S34-ROcp5_432*C34);
    ROcp5_534 = -(ROcp5_233*S34-ROcp5_532*C34);
    ROcp5_634 = -(ROcp5_333*S34-ROcp5_632*C34);
    RLcp5_131 = ROcp5_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp5_231 = ROcp5_26*s->dpt[1][3]+ROcp5_85*s->dpt[3][3];
    RLcp5_331 = ROcp5_36*s->dpt[1][3]+ROcp5_95*s->dpt[3][3];
    OMcp5_131 = OMcp5_16+ROcp5_46*qd[31];
    OMcp5_231 = OMcp5_26+ROcp5_56*qd[31];
    OMcp5_331 = OMcp5_36+ROcp5_66*qd[31];
    ORcp5_131 = OMcp5_26*RLcp5_331-OMcp5_36*RLcp5_231;
    ORcp5_231 = -(OMcp5_16*RLcp5_331-OMcp5_36*RLcp5_131);
    ORcp5_331 = OMcp5_16*RLcp5_231-OMcp5_26*RLcp5_131;
    OMcp5_132 = OMcp5_131+ROcp5_131*qd[32];
    OMcp5_232 = OMcp5_231+ROcp5_231*qd[32];
    OMcp5_332 = OMcp5_331+ROcp5_331*qd[32];
    OMcp5_133 = OMcp5_132+ROcp5_432*qd[33];
    OMcp5_233 = OMcp5_232+ROcp5_532*qd[33];
    OMcp5_333 = OMcp5_332+ROcp5_632*qd[33];
    OPcp5_133 = OPcp5_16+ROcp5_131*qdd[32]+ROcp5_432*qdd[33]+ROcp5_46*qdd[31]+qd[31]*(OMcp5_26*ROcp5_66-OMcp5_36*ROcp5_56)
 +qd[32]*(OMcp5_231*ROcp5_331-OMcp5_331*ROcp5_231)+qd[33]*(OMcp5_232*ROcp5_632-OMcp5_332*ROcp5_532);
    OPcp5_233 = OPcp5_26+ROcp5_231*qdd[32]+ROcp5_532*qdd[33]+ROcp5_56*qdd[31]-qd[31]*(OMcp5_16*ROcp5_66-OMcp5_36*ROcp5_46)
 -qd[32]*(OMcp5_131*ROcp5_331-OMcp5_331*ROcp5_131)-qd[33]*(OMcp5_132*ROcp5_632-OMcp5_332*ROcp5_432);
    OPcp5_333 = OPcp5_36+ROcp5_331*qdd[32]+ROcp5_632*qdd[33]+ROcp5_66*qdd[31]+qd[31]*(OMcp5_16*ROcp5_56-OMcp5_26*ROcp5_46)
 +qd[32]*(OMcp5_131*ROcp5_231-OMcp5_231*ROcp5_131)+qd[33]*(OMcp5_132*ROcp5_532-OMcp5_232*ROcp5_432);
    RLcp5_134 = ROcp5_733*s->dpt[3][21];
    RLcp5_234 = ROcp5_833*s->dpt[3][21];
    RLcp5_334 = ROcp5_933*s->dpt[3][21];
    OMcp5_134 = OMcp5_133+ROcp5_733*qd[34];
    OMcp5_234 = OMcp5_233+ROcp5_833*qd[34];
    OMcp5_334 = OMcp5_333+ROcp5_933*qd[34];
    ORcp5_134 = OMcp5_233*RLcp5_334-OMcp5_333*RLcp5_234;
    ORcp5_234 = -(OMcp5_133*RLcp5_334-OMcp5_333*RLcp5_134);
    ORcp5_334 = OMcp5_133*RLcp5_234-OMcp5_233*RLcp5_134;
    OPcp5_134 = OPcp5_133+ROcp5_733*qdd[34]+qd[34]*(OMcp5_233*ROcp5_933-OMcp5_333*ROcp5_833);
    OPcp5_234 = OPcp5_233+ROcp5_833*qdd[34]-qd[34]*(OMcp5_133*ROcp5_933-OMcp5_333*ROcp5_733);
    OPcp5_334 = OPcp5_333+ROcp5_933*qdd[34]+qd[34]*(OMcp5_133*ROcp5_833-OMcp5_233*ROcp5_733);
    RLcp5_161 = ROcp5_134*s->dpt[1][26]+ROcp5_434*s->dpt[2][26]+ROcp5_733*s->dpt[3][26];
    RLcp5_261 = ROcp5_234*s->dpt[1][26]+ROcp5_534*s->dpt[2][26]+ROcp5_833*s->dpt[3][26];
    RLcp5_361 = ROcp5_334*s->dpt[1][26]+ROcp5_634*s->dpt[2][26]+ROcp5_933*s->dpt[3][26];
    POcp5_161 = RLcp5_131+RLcp5_134+RLcp5_161+q[1];
    POcp5_261 = RLcp5_231+RLcp5_234+RLcp5_261+q[2];
    POcp5_361 = RLcp5_331+RLcp5_334+RLcp5_361+q[3];
    JTcp5_261_4 = -(RLcp5_331+RLcp5_334+RLcp5_361);
    JTcp5_361_4 = RLcp5_231+RLcp5_234+RLcp5_261;
    JTcp5_161_5 = C4*(RLcp5_331+RLcp5_334)-S4*(RLcp5_231+RLcp5_234)-RLcp5_261*S4+RLcp5_361*C4;
    JTcp5_261_5 = S4*(RLcp5_131+RLcp5_134+RLcp5_161);
    JTcp5_361_5 = -C4*(RLcp5_131+RLcp5_134+RLcp5_161);
    JTcp5_161_6 = ROcp5_85*(RLcp5_331+RLcp5_334)-ROcp5_95*(RLcp5_231+RLcp5_234)-RLcp5_261*ROcp5_95+RLcp5_361*ROcp5_85;
    JTcp5_261_6 = -(RLcp5_361*S5-ROcp5_95*(RLcp5_131+RLcp5_134+RLcp5_161)+S5*(RLcp5_331+RLcp5_334));
    JTcp5_361_6 = RLcp5_261*S5-ROcp5_85*(RLcp5_131+RLcp5_134+RLcp5_161)+S5*(RLcp5_231+RLcp5_234);
    JTcp5_161_7 = ROcp5_56*(RLcp5_334+RLcp5_361)-ROcp5_66*(RLcp5_234+RLcp5_261);
    JTcp5_261_7 = -(ROcp5_46*(RLcp5_334+RLcp5_361)-ROcp5_66*(RLcp5_134+RLcp5_161));
    JTcp5_361_7 = ROcp5_46*(RLcp5_234+RLcp5_261)-ROcp5_56*(RLcp5_134+RLcp5_161);
    JTcp5_161_8 = ROcp5_231*(RLcp5_334+RLcp5_361)-ROcp5_331*(RLcp5_234+RLcp5_261);
    JTcp5_261_8 = -(ROcp5_131*(RLcp5_334+RLcp5_361)-ROcp5_331*(RLcp5_134+RLcp5_161));
    JTcp5_361_8 = ROcp5_131*(RLcp5_234+RLcp5_261)-ROcp5_231*(RLcp5_134+RLcp5_161);
    JTcp5_161_9 = ROcp5_532*(RLcp5_334+RLcp5_361)-ROcp5_632*(RLcp5_234+RLcp5_261);
    JTcp5_261_9 = -(ROcp5_432*(RLcp5_334+RLcp5_361)-ROcp5_632*(RLcp5_134+RLcp5_161));
    JTcp5_361_9 = ROcp5_432*(RLcp5_234+RLcp5_261)-ROcp5_532*(RLcp5_134+RLcp5_161);
    JTcp5_161_10 = -(RLcp5_261*ROcp5_933-RLcp5_361*ROcp5_833);
    JTcp5_261_10 = RLcp5_161*ROcp5_933-RLcp5_361*ROcp5_733;
    JTcp5_361_10 = -(RLcp5_161*ROcp5_833-RLcp5_261*ROcp5_733);
    ORcp5_161 = OMcp5_234*RLcp5_361-OMcp5_334*RLcp5_261;
    ORcp5_261 = -(OMcp5_134*RLcp5_361-OMcp5_334*RLcp5_161);
    ORcp5_361 = OMcp5_134*RLcp5_261-OMcp5_234*RLcp5_161;
    VIcp5_161 = ORcp5_131+ORcp5_134+ORcp5_161+qd[1];
    VIcp5_261 = ORcp5_231+ORcp5_234+ORcp5_261+qd[2];
    VIcp5_361 = ORcp5_331+ORcp5_334+ORcp5_361+qd[3];
    ACcp5_161 = qdd[1]+OMcp5_233*ORcp5_334+OMcp5_234*ORcp5_361+OMcp5_26*ORcp5_331-OMcp5_333*ORcp5_234-OMcp5_334*ORcp5_261-
 OMcp5_36*ORcp5_231+OPcp5_233*RLcp5_334+OPcp5_234*RLcp5_361+OPcp5_26*RLcp5_331-OPcp5_333*RLcp5_234-OPcp5_334*RLcp5_261-
 OPcp5_36*RLcp5_231;
    ACcp5_261 = qdd[2]-OMcp5_133*ORcp5_334-OMcp5_134*ORcp5_361-OMcp5_16*ORcp5_331+OMcp5_333*ORcp5_134+OMcp5_334*ORcp5_161+
 OMcp5_36*ORcp5_131-OPcp5_133*RLcp5_334-OPcp5_134*RLcp5_361-OPcp5_16*RLcp5_331+OPcp5_333*RLcp5_134+OPcp5_334*RLcp5_161+
 OPcp5_36*RLcp5_131;
    ACcp5_361 = qdd[3]+OMcp5_133*ORcp5_234+OMcp5_134*ORcp5_261+OMcp5_16*ORcp5_231-OMcp5_233*ORcp5_134-OMcp5_234*ORcp5_161-
 OMcp5_26*ORcp5_131+OPcp5_133*RLcp5_234+OPcp5_134*RLcp5_261+OPcp5_16*RLcp5_231-OPcp5_233*RLcp5_134-OPcp5_234*RLcp5_161-
 OPcp5_26*RLcp5_131;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_161;
    sens->P[2] = POcp5_261;
    sens->P[3] = POcp5_361;
    sens->R[1][1] = ROcp5_134;
    sens->R[1][2] = ROcp5_234;
    sens->R[1][3] = ROcp5_334;
    sens->R[2][1] = ROcp5_434;
    sens->R[2][2] = ROcp5_534;
    sens->R[2][3] = ROcp5_634;
    sens->R[3][1] = ROcp5_733;
    sens->R[3][2] = ROcp5_833;
    sens->R[3][3] = ROcp5_933;
    sens->V[1] = VIcp5_161;
    sens->V[2] = VIcp5_261;
    sens->V[3] = VIcp5_361;
    sens->OM[1] = OMcp5_134;
    sens->OM[2] = OMcp5_234;
    sens->OM[3] = OMcp5_334;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp5_161_5;
    sens->J[1][6] = JTcp5_161_6;
    sens->J[1][31] = JTcp5_161_7;
    sens->J[1][32] = JTcp5_161_8;
    sens->J[1][33] = JTcp5_161_9;
    sens->J[1][34] = JTcp5_161_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp5_261_4;
    sens->J[2][5] = JTcp5_261_5;
    sens->J[2][6] = JTcp5_261_6;
    sens->J[2][31] = JTcp5_261_7;
    sens->J[2][32] = JTcp5_261_8;
    sens->J[2][33] = JTcp5_261_9;
    sens->J[2][34] = JTcp5_261_10;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp5_361_4;
    sens->J[3][5] = JTcp5_361_5;
    sens->J[3][6] = JTcp5_361_6;
    sens->J[3][31] = JTcp5_361_7;
    sens->J[3][32] = JTcp5_361_8;
    sens->J[3][33] = JTcp5_361_9;
    sens->J[3][34] = JTcp5_361_10;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp5_46;
    sens->J[4][32] = ROcp5_131;
    sens->J[4][33] = ROcp5_432;
    sens->J[4][34] = ROcp5_733;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp5_85;
    sens->J[5][31] = ROcp5_56;
    sens->J[5][32] = ROcp5_231;
    sens->J[5][33] = ROcp5_532;
    sens->J[5][34] = ROcp5_833;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp5_95;
    sens->J[6][31] = ROcp5_66;
    sens->J[6][32] = ROcp5_331;
    sens->J[6][33] = ROcp5_632;
    sens->J[6][34] = ROcp5_933;
    sens->A[1] = ACcp5_161;
    sens->A[2] = ACcp5_261;
    sens->A[3] = ACcp5_361;
    sens->OMP[1] = OPcp5_134;
    sens->OMP[2] = OPcp5_234;
    sens->OMP[3] = OPcp5_334;
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp6_25 = qd[5]*C4;
    OMcp6_35 = qd[5]*S4;
    OMcp6_16 = qd[4]+qd[6]*S5;
    OMcp6_26 = OMcp6_25+ROcp6_85*qd[6];
    OMcp6_36 = OMcp6_35+ROcp6_95*qd[6];
    OPcp6_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp6_26 = ROcp6_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp6_35*S5-ROcp6_95*qd[4]);
    OPcp6_36 = ROcp6_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp6_25*S5-ROcp6_85*qd[4]);

// = = Block_1_0_0_7_0_4 = = 
 
// Sensor Kinematics 


    ROcp6_131 = ROcp6_16*C31-S31*S5;
    ROcp6_231 = ROcp6_26*C31-ROcp6_85*S31;
    ROcp6_331 = ROcp6_36*C31-ROcp6_95*S31;
    ROcp6_731 = ROcp6_16*S31+C31*S5;
    ROcp6_831 = ROcp6_26*S31+ROcp6_85*C31;
    ROcp6_931 = ROcp6_36*S31+ROcp6_95*C31;
    ROcp6_432 = ROcp6_46*C32+ROcp6_731*S32;
    ROcp6_532 = ROcp6_56*C32+ROcp6_831*S32;
    ROcp6_632 = ROcp6_66*C32+ROcp6_931*S32;
    ROcp6_732 = -(ROcp6_46*S32-ROcp6_731*C32);
    ROcp6_832 = -(ROcp6_56*S32-ROcp6_831*C32);
    ROcp6_932 = -(ROcp6_66*S32-ROcp6_931*C32);
    ROcp6_133 = ROcp6_131*C33-ROcp6_732*S33;
    ROcp6_233 = ROcp6_231*C33-ROcp6_832*S33;
    ROcp6_333 = ROcp6_331*C33-ROcp6_932*S33;
    ROcp6_733 = ROcp6_131*S33+ROcp6_732*C33;
    ROcp6_833 = ROcp6_231*S33+ROcp6_832*C33;
    ROcp6_933 = ROcp6_331*S33+ROcp6_932*C33;
    ROcp6_134 = ROcp6_133*C34+ROcp6_432*S34;
    ROcp6_234 = ROcp6_233*C34+ROcp6_532*S34;
    ROcp6_334 = ROcp6_333*C34+ROcp6_632*S34;
    ROcp6_434 = -(ROcp6_133*S34-ROcp6_432*C34);
    ROcp6_534 = -(ROcp6_233*S34-ROcp6_532*C34);
    ROcp6_634 = -(ROcp6_333*S34-ROcp6_632*C34);
    RLcp6_131 = ROcp6_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp6_231 = ROcp6_26*s->dpt[1][3]+ROcp6_85*s->dpt[3][3];
    RLcp6_331 = ROcp6_36*s->dpt[1][3]+ROcp6_95*s->dpt[3][3];
    OMcp6_131 = OMcp6_16+ROcp6_46*qd[31];
    OMcp6_231 = OMcp6_26+ROcp6_56*qd[31];
    OMcp6_331 = OMcp6_36+ROcp6_66*qd[31];
    ORcp6_131 = OMcp6_26*RLcp6_331-OMcp6_36*RLcp6_231;
    ORcp6_231 = -(OMcp6_16*RLcp6_331-OMcp6_36*RLcp6_131);
    ORcp6_331 = OMcp6_16*RLcp6_231-OMcp6_26*RLcp6_131;
    OMcp6_132 = OMcp6_131+ROcp6_131*qd[32];
    OMcp6_232 = OMcp6_231+ROcp6_231*qd[32];
    OMcp6_332 = OMcp6_331+ROcp6_331*qd[32];
    OMcp6_133 = OMcp6_132+ROcp6_432*qd[33];
    OMcp6_233 = OMcp6_232+ROcp6_532*qd[33];
    OMcp6_333 = OMcp6_332+ROcp6_632*qd[33];
    OPcp6_133 = OPcp6_16+ROcp6_131*qdd[32]+ROcp6_432*qdd[33]+ROcp6_46*qdd[31]+qd[31]*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56)
 +qd[32]*(OMcp6_231*ROcp6_331-OMcp6_331*ROcp6_231)+qd[33]*(OMcp6_232*ROcp6_632-OMcp6_332*ROcp6_532);
    OPcp6_233 = OPcp6_26+ROcp6_231*qdd[32]+ROcp6_532*qdd[33]+ROcp6_56*qdd[31]-qd[31]*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46)
 -qd[32]*(OMcp6_131*ROcp6_331-OMcp6_331*ROcp6_131)-qd[33]*(OMcp6_132*ROcp6_632-OMcp6_332*ROcp6_432);
    OPcp6_333 = OPcp6_36+ROcp6_331*qdd[32]+ROcp6_632*qdd[33]+ROcp6_66*qdd[31]+qd[31]*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46)
 +qd[32]*(OMcp6_131*ROcp6_231-OMcp6_231*ROcp6_131)+qd[33]*(OMcp6_132*ROcp6_532-OMcp6_232*ROcp6_432);
    RLcp6_134 = ROcp6_733*s->dpt[3][21];
    RLcp6_234 = ROcp6_833*s->dpt[3][21];
    RLcp6_334 = ROcp6_933*s->dpt[3][21];
    OMcp6_134 = OMcp6_133+ROcp6_733*qd[34];
    OMcp6_234 = OMcp6_233+ROcp6_833*qd[34];
    OMcp6_334 = OMcp6_333+ROcp6_933*qd[34];
    ORcp6_134 = OMcp6_233*RLcp6_334-OMcp6_333*RLcp6_234;
    ORcp6_234 = -(OMcp6_133*RLcp6_334-OMcp6_333*RLcp6_134);
    ORcp6_334 = OMcp6_133*RLcp6_234-OMcp6_233*RLcp6_134;
    OPcp6_134 = OPcp6_133+ROcp6_733*qdd[34]+qd[34]*(OMcp6_233*ROcp6_933-OMcp6_333*ROcp6_833);
    OPcp6_234 = OPcp6_233+ROcp6_833*qdd[34]-qd[34]*(OMcp6_133*ROcp6_933-OMcp6_333*ROcp6_733);
    OPcp6_334 = OPcp6_333+ROcp6_933*qdd[34]+qd[34]*(OMcp6_133*ROcp6_833-OMcp6_233*ROcp6_733);

// = = Block_1_0_0_7_0_5 = = 
 
// Sensor Kinematics 


    ROcp6_435 = ROcp6_434*C35+ROcp6_733*S35;
    ROcp6_535 = ROcp6_534*C35+ROcp6_833*S35;
    ROcp6_635 = ROcp6_634*C35+ROcp6_933*S35;
    ROcp6_735 = -(ROcp6_434*S35-ROcp6_733*C35);
    ROcp6_835 = -(ROcp6_534*S35-ROcp6_833*C35);
    ROcp6_935 = -(ROcp6_634*S35-ROcp6_933*C35);
    ROcp6_136 = ROcp6_134*C36+ROcp6_435*S36;
    ROcp6_236 = ROcp6_234*C36+ROcp6_535*S36;
    ROcp6_336 = ROcp6_334*C36+ROcp6_635*S36;
    ROcp6_436 = -(ROcp6_134*S36-ROcp6_435*C36);
    ROcp6_536 = -(ROcp6_234*S36-ROcp6_535*C36);
    ROcp6_636 = -(ROcp6_334*S36-ROcp6_635*C36);
    ROcp6_137 = ROcp6_136*C37-ROcp6_735*S37;
    ROcp6_237 = ROcp6_236*C37-ROcp6_835*S37;
    ROcp6_337 = ROcp6_336*C37-ROcp6_935*S37;
    ROcp6_737 = ROcp6_136*S37+ROcp6_735*C37;
    ROcp6_837 = ROcp6_236*S37+ROcp6_835*C37;
    ROcp6_937 = ROcp6_336*S37+ROcp6_935*C37;
    RLcp6_135 = ROcp6_134*s->dpt[1][23]+ROcp6_434*s->dpt[2][23]+ROcp6_733*s->dpt[3][23];
    RLcp6_235 = ROcp6_234*s->dpt[1][23]+ROcp6_534*s->dpt[2][23]+ROcp6_833*s->dpt[3][23];
    RLcp6_335 = ROcp6_334*s->dpt[1][23]+ROcp6_634*s->dpt[2][23]+ROcp6_933*s->dpt[3][23];
    ORcp6_135 = OMcp6_234*RLcp6_335-OMcp6_334*RLcp6_235;
    ORcp6_235 = -(OMcp6_134*RLcp6_335-OMcp6_334*RLcp6_135);
    ORcp6_335 = OMcp6_134*RLcp6_235-OMcp6_234*RLcp6_135;
    OMcp6_137 = OMcp6_134+ROcp6_436*qd[37];
    OMcp6_237 = OMcp6_234+ROcp6_536*qd[37];
    OMcp6_337 = OMcp6_334+ROcp6_636*qd[37];
    OPcp6_137 = OPcp6_134+ROcp6_436*qdd[37]+qd[37]*(OMcp6_234*ROcp6_636-OMcp6_334*ROcp6_536);
    OPcp6_237 = OPcp6_234+ROcp6_536*qdd[37]-qd[37]*(OMcp6_134*ROcp6_636-OMcp6_334*ROcp6_436);
    OPcp6_337 = OPcp6_334+ROcp6_636*qdd[37]+qd[37]*(OMcp6_134*ROcp6_536-OMcp6_234*ROcp6_436);
    RLcp6_162 = ROcp6_137*s->dpt[1][28]+ROcp6_436*s->dpt[2][28]+ROcp6_737*s->dpt[3][28];
    RLcp6_262 = ROcp6_237*s->dpt[1][28]+ROcp6_536*s->dpt[2][28]+ROcp6_837*s->dpt[3][28];
    RLcp6_362 = ROcp6_337*s->dpt[1][28]+ROcp6_636*s->dpt[2][28]+ROcp6_937*s->dpt[3][28];
    POcp6_162 = RLcp6_131+RLcp6_134+RLcp6_135+RLcp6_162+q[1];
    POcp6_262 = RLcp6_231+RLcp6_234+RLcp6_235+RLcp6_262+q[2];
    POcp6_362 = RLcp6_331+RLcp6_334+RLcp6_335+RLcp6_362+q[3];
    JTcp6_262_4 = -(RLcp6_331+RLcp6_334+RLcp6_335+RLcp6_362);
    JTcp6_362_4 = RLcp6_231+RLcp6_234+RLcp6_235+RLcp6_262;
    JTcp6_162_5 = C4*(RLcp6_331+RLcp6_334+RLcp6_335+RLcp6_362)-S4*(RLcp6_231+RLcp6_234)-S4*(RLcp6_235+RLcp6_262);
    JTcp6_262_5 = S4*(RLcp6_131+RLcp6_134+RLcp6_135+RLcp6_162);
    JTcp6_362_5 = -C4*(RLcp6_131+RLcp6_134+RLcp6_135+RLcp6_162);
    JTcp6_162_6 = ROcp6_85*(RLcp6_331+RLcp6_334+RLcp6_335+RLcp6_362)-ROcp6_95*(RLcp6_231+RLcp6_234)-ROcp6_95*(RLcp6_235+
 RLcp6_262);
    JTcp6_262_6 = RLcp6_162*ROcp6_95-RLcp6_335*S5-RLcp6_362*S5+ROcp6_95*(RLcp6_131+RLcp6_134+RLcp6_135)-S5*(RLcp6_331+
 RLcp6_334);
    JTcp6_362_6 = RLcp6_235*S5-ROcp6_85*(RLcp6_131+RLcp6_134+RLcp6_135)+S5*(RLcp6_231+RLcp6_234)-RLcp6_162*ROcp6_85+
 RLcp6_262*S5;
    JTcp6_162_7 = ROcp6_56*(RLcp6_334+RLcp6_335)-ROcp6_66*(RLcp6_234+RLcp6_235)-RLcp6_262*ROcp6_66+RLcp6_362*ROcp6_56;
    JTcp6_262_7 = RLcp6_162*ROcp6_66-RLcp6_362*ROcp6_46-ROcp6_46*(RLcp6_334+RLcp6_335)+ROcp6_66*(RLcp6_134+RLcp6_135);
    JTcp6_362_7 = ROcp6_46*(RLcp6_234+RLcp6_235)-ROcp6_56*(RLcp6_134+RLcp6_135)-RLcp6_162*ROcp6_56+RLcp6_262*ROcp6_46;
    JTcp6_162_8 = ROcp6_231*(RLcp6_334+RLcp6_335)-ROcp6_331*(RLcp6_234+RLcp6_235)-RLcp6_262*ROcp6_331+RLcp6_362*ROcp6_231;
    JTcp6_262_8 = RLcp6_162*ROcp6_331-RLcp6_362*ROcp6_131-ROcp6_131*(RLcp6_334+RLcp6_335)+ROcp6_331*(RLcp6_134+RLcp6_135);
    JTcp6_362_8 = ROcp6_131*(RLcp6_234+RLcp6_235)-ROcp6_231*(RLcp6_134+RLcp6_135)-RLcp6_162*ROcp6_231+RLcp6_262*ROcp6_131;
    JTcp6_162_9 = ROcp6_532*(RLcp6_334+RLcp6_335)-ROcp6_632*(RLcp6_234+RLcp6_235)-RLcp6_262*ROcp6_632+RLcp6_362*ROcp6_532;
    JTcp6_262_9 = RLcp6_162*ROcp6_632-RLcp6_362*ROcp6_432-ROcp6_432*(RLcp6_334+RLcp6_335)+ROcp6_632*(RLcp6_134+RLcp6_135);
    JTcp6_362_9 = ROcp6_432*(RLcp6_234+RLcp6_235)-ROcp6_532*(RLcp6_134+RLcp6_135)-RLcp6_162*ROcp6_532+RLcp6_262*ROcp6_432;
    JTcp6_162_10 = ROcp6_833*(RLcp6_335+RLcp6_362)-ROcp6_933*(RLcp6_235+RLcp6_262);
    JTcp6_262_10 = -(ROcp6_733*(RLcp6_335+RLcp6_362)-ROcp6_933*(RLcp6_135+RLcp6_162));
    JTcp6_362_10 = ROcp6_733*(RLcp6_235+RLcp6_262)-ROcp6_833*(RLcp6_135+RLcp6_162);
    JTcp6_162_11 = -(RLcp6_262*ROcp6_334-RLcp6_362*ROcp6_234);
    JTcp6_262_11 = RLcp6_162*ROcp6_334-RLcp6_362*ROcp6_134;
    JTcp6_362_11 = -(RLcp6_162*ROcp6_234-RLcp6_262*ROcp6_134);
    JTcp6_162_12 = -(RLcp6_262*ROcp6_935-RLcp6_362*ROcp6_835);
    JTcp6_262_12 = RLcp6_162*ROcp6_935-RLcp6_362*ROcp6_735;
    JTcp6_362_12 = -(RLcp6_162*ROcp6_835-RLcp6_262*ROcp6_735);
    JTcp6_162_13 = -(RLcp6_262*ROcp6_636-RLcp6_362*ROcp6_536);
    JTcp6_262_13 = RLcp6_162*ROcp6_636-RLcp6_362*ROcp6_436;
    JTcp6_362_13 = -(RLcp6_162*ROcp6_536-RLcp6_262*ROcp6_436);
    ORcp6_162 = OMcp6_237*RLcp6_362-OMcp6_337*RLcp6_262;
    ORcp6_262 = -(OMcp6_137*RLcp6_362-OMcp6_337*RLcp6_162);
    ORcp6_362 = OMcp6_137*RLcp6_262-OMcp6_237*RLcp6_162;
    VIcp6_162 = ORcp6_131+ORcp6_134+ORcp6_135+ORcp6_162+qd[1];
    VIcp6_262 = ORcp6_231+ORcp6_234+ORcp6_235+ORcp6_262+qd[2];
    VIcp6_362 = ORcp6_331+ORcp6_334+ORcp6_335+ORcp6_362+qd[3];
    ACcp6_162 = qdd[1]+OMcp6_233*ORcp6_334+OMcp6_234*ORcp6_335+OMcp6_237*ORcp6_362+OMcp6_26*ORcp6_331-OMcp6_333*ORcp6_234-
 OMcp6_334*ORcp6_235-OMcp6_337*ORcp6_262-OMcp6_36*ORcp6_231+OPcp6_233*RLcp6_334+OPcp6_234*RLcp6_335+OPcp6_237*RLcp6_362+
 OPcp6_26*RLcp6_331-OPcp6_333*RLcp6_234-OPcp6_334*RLcp6_235-OPcp6_337*RLcp6_262-OPcp6_36*RLcp6_231;
    ACcp6_262 = qdd[2]-OMcp6_133*ORcp6_334-OMcp6_134*ORcp6_335-OMcp6_137*ORcp6_362-OMcp6_16*ORcp6_331+OMcp6_333*ORcp6_134+
 OMcp6_334*ORcp6_135+OMcp6_337*ORcp6_162+OMcp6_36*ORcp6_131-OPcp6_133*RLcp6_334-OPcp6_134*RLcp6_335-OPcp6_137*RLcp6_362-
 OPcp6_16*RLcp6_331+OPcp6_333*RLcp6_134+OPcp6_334*RLcp6_135+OPcp6_337*RLcp6_162+OPcp6_36*RLcp6_131;
    ACcp6_362 = qdd[3]+OMcp6_133*ORcp6_234+OMcp6_134*ORcp6_235+OMcp6_137*ORcp6_262+OMcp6_16*ORcp6_231-OMcp6_233*ORcp6_134-
 OMcp6_234*ORcp6_135-OMcp6_237*ORcp6_162-OMcp6_26*ORcp6_131+OPcp6_133*RLcp6_234+OPcp6_134*RLcp6_235+OPcp6_137*RLcp6_262+
 OPcp6_16*RLcp6_231-OPcp6_233*RLcp6_134-OPcp6_234*RLcp6_135-OPcp6_237*RLcp6_162-OPcp6_26*RLcp6_131;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_162;
    sens->P[2] = POcp6_262;
    sens->P[3] = POcp6_362;
    sens->R[1][1] = ROcp6_137;
    sens->R[1][2] = ROcp6_237;
    sens->R[1][3] = ROcp6_337;
    sens->R[2][1] = ROcp6_436;
    sens->R[2][2] = ROcp6_536;
    sens->R[2][3] = ROcp6_636;
    sens->R[3][1] = ROcp6_737;
    sens->R[3][2] = ROcp6_837;
    sens->R[3][3] = ROcp6_937;
    sens->V[1] = VIcp6_162;
    sens->V[2] = VIcp6_262;
    sens->V[3] = VIcp6_362;
    sens->OM[1] = OMcp6_137;
    sens->OM[2] = OMcp6_237;
    sens->OM[3] = OMcp6_337;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp6_162_5;
    sens->J[1][6] = JTcp6_162_6;
    sens->J[1][31] = JTcp6_162_7;
    sens->J[1][32] = JTcp6_162_8;
    sens->J[1][33] = JTcp6_162_9;
    sens->J[1][34] = JTcp6_162_10;
    sens->J[1][35] = JTcp6_162_11;
    sens->J[1][36] = JTcp6_162_12;
    sens->J[1][37] = JTcp6_162_13;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp6_262_4;
    sens->J[2][5] = JTcp6_262_5;
    sens->J[2][6] = JTcp6_262_6;
    sens->J[2][31] = JTcp6_262_7;
    sens->J[2][32] = JTcp6_262_8;
    sens->J[2][33] = JTcp6_262_9;
    sens->J[2][34] = JTcp6_262_10;
    sens->J[2][35] = JTcp6_262_11;
    sens->J[2][36] = JTcp6_262_12;
    sens->J[2][37] = JTcp6_262_13;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp6_362_4;
    sens->J[3][5] = JTcp6_362_5;
    sens->J[3][6] = JTcp6_362_6;
    sens->J[3][31] = JTcp6_362_7;
    sens->J[3][32] = JTcp6_362_8;
    sens->J[3][33] = JTcp6_362_9;
    sens->J[3][34] = JTcp6_362_10;
    sens->J[3][35] = JTcp6_362_11;
    sens->J[3][36] = JTcp6_362_12;
    sens->J[3][37] = JTcp6_362_13;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp6_46;
    sens->J[4][32] = ROcp6_131;
    sens->J[4][33] = ROcp6_432;
    sens->J[4][34] = ROcp6_733;
    sens->J[4][35] = ROcp6_134;
    sens->J[4][36] = ROcp6_735;
    sens->J[4][37] = ROcp6_436;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp6_85;
    sens->J[5][31] = ROcp6_56;
    sens->J[5][32] = ROcp6_231;
    sens->J[5][33] = ROcp6_532;
    sens->J[5][34] = ROcp6_833;
    sens->J[5][35] = ROcp6_234;
    sens->J[5][36] = ROcp6_835;
    sens->J[5][37] = ROcp6_536;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp6_95;
    sens->J[6][31] = ROcp6_66;
    sens->J[6][32] = ROcp6_331;
    sens->J[6][33] = ROcp6_632;
    sens->J[6][34] = ROcp6_933;
    sens->J[6][35] = ROcp6_334;
    sens->J[6][36] = ROcp6_935;
    sens->J[6][37] = ROcp6_636;
    sens->A[1] = ACcp6_162;
    sens->A[2] = ACcp6_262;
    sens->A[3] = ACcp6_362;
    sens->OMP[1] = OPcp6_137;
    sens->OMP[2] = OPcp6_237;
    sens->OMP[3] = OPcp6_337;
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp7_25 = qd[5]*C4;
    OMcp7_35 = qd[5]*S4;
    OMcp7_16 = qd[4]+qd[6]*S5;
    OMcp7_26 = OMcp7_25+ROcp7_85*qd[6];
    OMcp7_36 = OMcp7_35+ROcp7_95*qd[6];
    OPcp7_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp7_26 = ROcp7_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp7_35*S5-ROcp7_95*qd[4]);
    OPcp7_36 = ROcp7_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp7_25*S5-ROcp7_85*qd[4]);

// = = Block_1_0_0_8_0_4 = = 
 
// Sensor Kinematics 


    ROcp7_131 = ROcp7_16*C31-S31*S5;
    ROcp7_231 = ROcp7_26*C31-ROcp7_85*S31;
    ROcp7_331 = ROcp7_36*C31-ROcp7_95*S31;
    ROcp7_731 = ROcp7_16*S31+C31*S5;
    ROcp7_831 = ROcp7_26*S31+ROcp7_85*C31;
    ROcp7_931 = ROcp7_36*S31+ROcp7_95*C31;
    ROcp7_432 = ROcp7_46*C32+ROcp7_731*S32;
    ROcp7_532 = ROcp7_56*C32+ROcp7_831*S32;
    ROcp7_632 = ROcp7_66*C32+ROcp7_931*S32;
    ROcp7_732 = -(ROcp7_46*S32-ROcp7_731*C32);
    ROcp7_832 = -(ROcp7_56*S32-ROcp7_831*C32);
    ROcp7_932 = -(ROcp7_66*S32-ROcp7_931*C32);
    ROcp7_133 = ROcp7_131*C33-ROcp7_732*S33;
    ROcp7_233 = ROcp7_231*C33-ROcp7_832*S33;
    ROcp7_333 = ROcp7_331*C33-ROcp7_932*S33;
    ROcp7_733 = ROcp7_131*S33+ROcp7_732*C33;
    ROcp7_833 = ROcp7_231*S33+ROcp7_832*C33;
    ROcp7_933 = ROcp7_331*S33+ROcp7_932*C33;
    ROcp7_134 = ROcp7_133*C34+ROcp7_432*S34;
    ROcp7_234 = ROcp7_233*C34+ROcp7_532*S34;
    ROcp7_334 = ROcp7_333*C34+ROcp7_632*S34;
    ROcp7_434 = -(ROcp7_133*S34-ROcp7_432*C34);
    ROcp7_534 = -(ROcp7_233*S34-ROcp7_532*C34);
    ROcp7_634 = -(ROcp7_333*S34-ROcp7_632*C34);
    RLcp7_131 = ROcp7_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp7_231 = ROcp7_26*s->dpt[1][3]+ROcp7_85*s->dpt[3][3];
    RLcp7_331 = ROcp7_36*s->dpt[1][3]+ROcp7_95*s->dpt[3][3];
    OMcp7_131 = OMcp7_16+ROcp7_46*qd[31];
    OMcp7_231 = OMcp7_26+ROcp7_56*qd[31];
    OMcp7_331 = OMcp7_36+ROcp7_66*qd[31];
    ORcp7_131 = OMcp7_26*RLcp7_331-OMcp7_36*RLcp7_231;
    ORcp7_231 = -(OMcp7_16*RLcp7_331-OMcp7_36*RLcp7_131);
    ORcp7_331 = OMcp7_16*RLcp7_231-OMcp7_26*RLcp7_131;
    OMcp7_132 = OMcp7_131+ROcp7_131*qd[32];
    OMcp7_232 = OMcp7_231+ROcp7_231*qd[32];
    OMcp7_332 = OMcp7_331+ROcp7_331*qd[32];
    OMcp7_133 = OMcp7_132+ROcp7_432*qd[33];
    OMcp7_233 = OMcp7_232+ROcp7_532*qd[33];
    OMcp7_333 = OMcp7_332+ROcp7_632*qd[33];
    OPcp7_133 = OPcp7_16+ROcp7_131*qdd[32]+ROcp7_432*qdd[33]+ROcp7_46*qdd[31]+qd[31]*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56)
 +qd[32]*(OMcp7_231*ROcp7_331-OMcp7_331*ROcp7_231)+qd[33]*(OMcp7_232*ROcp7_632-OMcp7_332*ROcp7_532);
    OPcp7_233 = OPcp7_26+ROcp7_231*qdd[32]+ROcp7_532*qdd[33]+ROcp7_56*qdd[31]-qd[31]*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46)
 -qd[32]*(OMcp7_131*ROcp7_331-OMcp7_331*ROcp7_131)-qd[33]*(OMcp7_132*ROcp7_632-OMcp7_332*ROcp7_432);
    OPcp7_333 = OPcp7_36+ROcp7_331*qdd[32]+ROcp7_632*qdd[33]+ROcp7_66*qdd[31]+qd[31]*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46)
 +qd[32]*(OMcp7_131*ROcp7_231-OMcp7_231*ROcp7_131)+qd[33]*(OMcp7_132*ROcp7_532-OMcp7_232*ROcp7_432);
    RLcp7_134 = ROcp7_733*s->dpt[3][21];
    RLcp7_234 = ROcp7_833*s->dpt[3][21];
    RLcp7_334 = ROcp7_933*s->dpt[3][21];
    OMcp7_134 = OMcp7_133+ROcp7_733*qd[34];
    OMcp7_234 = OMcp7_233+ROcp7_833*qd[34];
    OMcp7_334 = OMcp7_333+ROcp7_933*qd[34];
    ORcp7_134 = OMcp7_233*RLcp7_334-OMcp7_333*RLcp7_234;
    ORcp7_234 = -(OMcp7_133*RLcp7_334-OMcp7_333*RLcp7_134);
    ORcp7_334 = OMcp7_133*RLcp7_234-OMcp7_233*RLcp7_134;
    OPcp7_134 = OPcp7_133+ROcp7_733*qdd[34]+qd[34]*(OMcp7_233*ROcp7_933-OMcp7_333*ROcp7_833);
    OPcp7_234 = OPcp7_233+ROcp7_833*qdd[34]-qd[34]*(OMcp7_133*ROcp7_933-OMcp7_333*ROcp7_733);
    OPcp7_334 = OPcp7_333+ROcp7_933*qdd[34]+qd[34]*(OMcp7_133*ROcp7_833-OMcp7_233*ROcp7_733);

// = = Block_1_0_0_8_0_5 = = 
 
// Sensor Kinematics 


    ROcp7_435 = ROcp7_434*C35+ROcp7_733*S35;
    ROcp7_535 = ROcp7_534*C35+ROcp7_833*S35;
    ROcp7_635 = ROcp7_634*C35+ROcp7_933*S35;
    ROcp7_735 = -(ROcp7_434*S35-ROcp7_733*C35);
    ROcp7_835 = -(ROcp7_534*S35-ROcp7_833*C35);
    ROcp7_935 = -(ROcp7_634*S35-ROcp7_933*C35);
    ROcp7_136 = ROcp7_134*C36+ROcp7_435*S36;
    ROcp7_236 = ROcp7_234*C36+ROcp7_535*S36;
    ROcp7_336 = ROcp7_334*C36+ROcp7_635*S36;
    ROcp7_436 = -(ROcp7_134*S36-ROcp7_435*C36);
    ROcp7_536 = -(ROcp7_234*S36-ROcp7_535*C36);
    ROcp7_636 = -(ROcp7_334*S36-ROcp7_635*C36);
    ROcp7_137 = ROcp7_136*C37-ROcp7_735*S37;
    ROcp7_237 = ROcp7_236*C37-ROcp7_835*S37;
    ROcp7_337 = ROcp7_336*C37-ROcp7_935*S37;
    ROcp7_737 = ROcp7_136*S37+ROcp7_735*C37;
    ROcp7_837 = ROcp7_236*S37+ROcp7_835*C37;
    ROcp7_937 = ROcp7_336*S37+ROcp7_935*C37;
    ROcp7_438 = ROcp7_436*C38+ROcp7_737*S38;
    ROcp7_538 = ROcp7_536*C38+ROcp7_837*S38;
    ROcp7_638 = ROcp7_636*C38+ROcp7_937*S38;
    ROcp7_738 = -(ROcp7_436*S38-ROcp7_737*C38);
    ROcp7_838 = -(ROcp7_536*S38-ROcp7_837*C38);
    ROcp7_938 = -(ROcp7_636*S38-ROcp7_937*C38);
    RLcp7_135 = ROcp7_134*s->dpt[1][23]+ROcp7_434*s->dpt[2][23]+ROcp7_733*s->dpt[3][23];
    RLcp7_235 = ROcp7_234*s->dpt[1][23]+ROcp7_534*s->dpt[2][23]+ROcp7_833*s->dpt[3][23];
    RLcp7_335 = ROcp7_334*s->dpt[1][23]+ROcp7_634*s->dpt[2][23]+ROcp7_933*s->dpt[3][23];
    ORcp7_135 = OMcp7_234*RLcp7_335-OMcp7_334*RLcp7_235;
    ORcp7_235 = -(OMcp7_134*RLcp7_335-OMcp7_334*RLcp7_135);
    ORcp7_335 = OMcp7_134*RLcp7_235-OMcp7_234*RLcp7_135;
    OMcp7_137 = OMcp7_134+ROcp7_436*qd[37];
    OMcp7_237 = OMcp7_234+ROcp7_536*qd[37];
    OMcp7_337 = OMcp7_334+ROcp7_636*qd[37];
    OPcp7_137 = OPcp7_134+ROcp7_436*qdd[37]+qd[37]*(OMcp7_234*ROcp7_636-OMcp7_334*ROcp7_536);
    OPcp7_237 = OPcp7_234+ROcp7_536*qdd[37]-qd[37]*(OMcp7_134*ROcp7_636-OMcp7_334*ROcp7_436);
    OPcp7_337 = OPcp7_334+ROcp7_636*qdd[37]+qd[37]*(OMcp7_134*ROcp7_536-OMcp7_234*ROcp7_436);
    RLcp7_138 = ROcp7_436*s->dpt[2][27]+ROcp7_737*s->dpt[3][27];
    RLcp7_238 = ROcp7_536*s->dpt[2][27]+ROcp7_837*s->dpt[3][27];
    RLcp7_338 = ROcp7_636*s->dpt[2][27]+ROcp7_937*s->dpt[3][27];
    OMcp7_138 = OMcp7_137+ROcp7_137*qd[38];
    OMcp7_238 = OMcp7_237+ROcp7_237*qd[38];
    OMcp7_338 = OMcp7_337+ROcp7_337*qd[38];
    ORcp7_138 = OMcp7_237*RLcp7_338-OMcp7_337*RLcp7_238;
    ORcp7_238 = -(OMcp7_137*RLcp7_338-OMcp7_337*RLcp7_138);
    ORcp7_338 = OMcp7_137*RLcp7_238-OMcp7_237*RLcp7_138;
    OPcp7_138 = OPcp7_137+ROcp7_137*qdd[38]+qd[38]*(OMcp7_237*ROcp7_337-OMcp7_337*ROcp7_237);
    OPcp7_238 = OPcp7_237+ROcp7_237*qdd[38]-qd[38]*(OMcp7_137*ROcp7_337-OMcp7_337*ROcp7_137);
    OPcp7_338 = OPcp7_337+ROcp7_337*qdd[38]+qd[38]*(OMcp7_137*ROcp7_237-OMcp7_237*ROcp7_137);
    RLcp7_163 = ROcp7_137*s->dpt[1][30]+ROcp7_438*s->dpt[2][30]+ROcp7_738*s->dpt[3][30];
    RLcp7_263 = ROcp7_237*s->dpt[1][30]+ROcp7_538*s->dpt[2][30]+ROcp7_838*s->dpt[3][30];
    RLcp7_363 = ROcp7_337*s->dpt[1][30]+ROcp7_638*s->dpt[2][30]+ROcp7_938*s->dpt[3][30];
    POcp7_163 = RLcp7_131+RLcp7_134+RLcp7_135+RLcp7_138+RLcp7_163+q[1];
    POcp7_263 = RLcp7_231+RLcp7_234+RLcp7_235+RLcp7_238+RLcp7_263+q[2];
    POcp7_363 = RLcp7_331+RLcp7_334+RLcp7_335+RLcp7_338+RLcp7_363+q[3];
    JTcp7_263_4 = -(RLcp7_331+RLcp7_334+RLcp7_335+RLcp7_338+RLcp7_363);
    JTcp7_363_4 = RLcp7_231+RLcp7_234+RLcp7_235+RLcp7_238+RLcp7_263;
    JTcp7_163_5 = C4*(RLcp7_331+RLcp7_334+RLcp7_335+RLcp7_338)-S4*(RLcp7_231+RLcp7_234)-S4*(RLcp7_235+RLcp7_238)-RLcp7_263
 *S4+RLcp7_363*C4;
    JTcp7_263_5 = S4*(RLcp7_131+RLcp7_134+RLcp7_135+RLcp7_138+RLcp7_163);
    JTcp7_363_5 = -C4*(RLcp7_131+RLcp7_134+RLcp7_135+RLcp7_138+RLcp7_163);
    JTcp7_163_6 = ROcp7_85*(RLcp7_331+RLcp7_334+RLcp7_335+RLcp7_338)-ROcp7_95*(RLcp7_231+RLcp7_234)-ROcp7_95*(RLcp7_235+
 RLcp7_238)-RLcp7_263*ROcp7_95+RLcp7_363*ROcp7_85;
    JTcp7_263_6 = -(RLcp7_363*S5-ROcp7_95*(RLcp7_131+RLcp7_134+RLcp7_135+RLcp7_138+RLcp7_163)+S5*(RLcp7_331+RLcp7_334)+S5*
 (RLcp7_335+RLcp7_338));
    JTcp7_363_6 = RLcp7_263*S5-ROcp7_85*(RLcp7_131+RLcp7_134+RLcp7_135+RLcp7_138+RLcp7_163)+S5*(RLcp7_231+RLcp7_234)+S5*(
 RLcp7_235+RLcp7_238);
    JTcp7_163_7 = ROcp7_56*(RLcp7_334+RLcp7_335+RLcp7_338+RLcp7_363)-ROcp7_66*(RLcp7_234+RLcp7_235)-ROcp7_66*(RLcp7_238+
 RLcp7_263);
    JTcp7_263_7 = -(ROcp7_46*(RLcp7_334+RLcp7_335+RLcp7_338+RLcp7_363)-ROcp7_66*(RLcp7_134+RLcp7_135)-ROcp7_66*(RLcp7_138+
 RLcp7_163));
    JTcp7_363_7 = ROcp7_46*(RLcp7_234+RLcp7_235+RLcp7_238+RLcp7_263)-ROcp7_56*(RLcp7_134+RLcp7_135)-ROcp7_56*(RLcp7_138+
 RLcp7_163);
    JTcp7_163_8 = ROcp7_231*(RLcp7_334+RLcp7_335+RLcp7_338+RLcp7_363)-ROcp7_331*(RLcp7_234+RLcp7_235)-ROcp7_331*(RLcp7_238
 +RLcp7_263);
    JTcp7_263_8 = -(ROcp7_131*(RLcp7_334+RLcp7_335+RLcp7_338+RLcp7_363)-ROcp7_331*(RLcp7_134+RLcp7_135)-ROcp7_331*(
 RLcp7_138+RLcp7_163));
    JTcp7_363_8 = ROcp7_131*(RLcp7_234+RLcp7_235+RLcp7_238+RLcp7_263)-ROcp7_231*(RLcp7_134+RLcp7_135)-ROcp7_231*(RLcp7_138
 +RLcp7_163);
    JTcp7_163_9 = ROcp7_532*(RLcp7_334+RLcp7_335+RLcp7_338+RLcp7_363)-ROcp7_632*(RLcp7_234+RLcp7_235)-ROcp7_632*(RLcp7_238
 +RLcp7_263);
    JTcp7_263_9 = -(ROcp7_432*(RLcp7_334+RLcp7_335+RLcp7_338+RLcp7_363)-ROcp7_632*(RLcp7_134+RLcp7_135)-ROcp7_632*(
 RLcp7_138+RLcp7_163));
    JTcp7_363_9 = ROcp7_432*(RLcp7_234+RLcp7_235+RLcp7_238+RLcp7_263)-ROcp7_532*(RLcp7_134+RLcp7_135)-ROcp7_532*(RLcp7_138
 +RLcp7_163);
    JTcp7_163_10 = ROcp7_833*(RLcp7_335+RLcp7_338)-ROcp7_933*(RLcp7_235+RLcp7_238)-RLcp7_263*ROcp7_933+RLcp7_363*ROcp7_833;
    JTcp7_263_10 = RLcp7_163*ROcp7_933-RLcp7_363*ROcp7_733-ROcp7_733*(RLcp7_335+RLcp7_338)+ROcp7_933*(RLcp7_135+RLcp7_138);
    JTcp7_363_10 = ROcp7_733*(RLcp7_235+RLcp7_238)-ROcp7_833*(RLcp7_135+RLcp7_138)-RLcp7_163*ROcp7_833+RLcp7_263*ROcp7_733;
    JTcp7_163_11 = ROcp7_234*(RLcp7_338+RLcp7_363)-ROcp7_334*(RLcp7_238+RLcp7_263);
    JTcp7_263_11 = -(ROcp7_134*(RLcp7_338+RLcp7_363)-ROcp7_334*(RLcp7_138+RLcp7_163));
    JTcp7_363_11 = ROcp7_134*(RLcp7_238+RLcp7_263)-ROcp7_234*(RLcp7_138+RLcp7_163);
    JTcp7_163_12 = ROcp7_835*(RLcp7_338+RLcp7_363)-ROcp7_935*(RLcp7_238+RLcp7_263);
    JTcp7_263_12 = -(ROcp7_735*(RLcp7_338+RLcp7_363)-ROcp7_935*(RLcp7_138+RLcp7_163));
    JTcp7_363_12 = ROcp7_735*(RLcp7_238+RLcp7_263)-ROcp7_835*(RLcp7_138+RLcp7_163);
    JTcp7_163_13 = ROcp7_536*(RLcp7_338+RLcp7_363)-ROcp7_636*(RLcp7_238+RLcp7_263);
    JTcp7_263_13 = -(ROcp7_436*(RLcp7_338+RLcp7_363)-ROcp7_636*(RLcp7_138+RLcp7_163));
    JTcp7_363_13 = ROcp7_436*(RLcp7_238+RLcp7_263)-ROcp7_536*(RLcp7_138+RLcp7_163);
    JTcp7_163_14 = -(RLcp7_263*ROcp7_337-RLcp7_363*ROcp7_237);
    JTcp7_263_14 = RLcp7_163*ROcp7_337-RLcp7_363*ROcp7_137;
    JTcp7_363_14 = -(RLcp7_163*ROcp7_237-RLcp7_263*ROcp7_137);
    ORcp7_163 = OMcp7_238*RLcp7_363-OMcp7_338*RLcp7_263;
    ORcp7_263 = -(OMcp7_138*RLcp7_363-OMcp7_338*RLcp7_163);
    ORcp7_363 = OMcp7_138*RLcp7_263-OMcp7_238*RLcp7_163;
    VIcp7_163 = ORcp7_131+ORcp7_134+ORcp7_135+ORcp7_138+ORcp7_163+qd[1];
    VIcp7_263 = ORcp7_231+ORcp7_234+ORcp7_235+ORcp7_238+ORcp7_263+qd[2];
    VIcp7_363 = ORcp7_331+ORcp7_334+ORcp7_335+ORcp7_338+ORcp7_363+qd[3];
    ACcp7_163 = qdd[1]+OMcp7_233*ORcp7_334+OMcp7_234*ORcp7_335+OMcp7_237*ORcp7_338+OMcp7_238*ORcp7_363+OMcp7_26*ORcp7_331-
 OMcp7_333*ORcp7_234-OMcp7_334*ORcp7_235-OMcp7_337*ORcp7_238-OMcp7_338*ORcp7_263-OMcp7_36*ORcp7_231+OPcp7_233*RLcp7_334+
 OPcp7_234*RLcp7_335+OPcp7_237*RLcp7_338+OPcp7_238*RLcp7_363+OPcp7_26*RLcp7_331-OPcp7_333*RLcp7_234-OPcp7_334*RLcp7_235-
 OPcp7_337*RLcp7_238-OPcp7_338*RLcp7_263-OPcp7_36*RLcp7_231;
    ACcp7_263 = qdd[2]-OMcp7_133*ORcp7_334-OMcp7_134*ORcp7_335-OMcp7_137*ORcp7_338-OMcp7_138*ORcp7_363-OMcp7_16*ORcp7_331+
 OMcp7_333*ORcp7_134+OMcp7_334*ORcp7_135+OMcp7_337*ORcp7_138+OMcp7_338*ORcp7_163+OMcp7_36*ORcp7_131-OPcp7_133*RLcp7_334-
 OPcp7_134*RLcp7_335-OPcp7_137*RLcp7_338-OPcp7_138*RLcp7_363-OPcp7_16*RLcp7_331+OPcp7_333*RLcp7_134+OPcp7_334*RLcp7_135+
 OPcp7_337*RLcp7_138+OPcp7_338*RLcp7_163+OPcp7_36*RLcp7_131;
    ACcp7_363 = qdd[3]+OMcp7_133*ORcp7_234+OMcp7_134*ORcp7_235+OMcp7_137*ORcp7_238+OMcp7_138*ORcp7_263+OMcp7_16*ORcp7_231-
 OMcp7_233*ORcp7_134-OMcp7_234*ORcp7_135-OMcp7_237*ORcp7_138-OMcp7_238*ORcp7_163-OMcp7_26*ORcp7_131+OPcp7_133*RLcp7_234+
 OPcp7_134*RLcp7_235+OPcp7_137*RLcp7_238+OPcp7_138*RLcp7_263+OPcp7_16*RLcp7_231-OPcp7_233*RLcp7_134-OPcp7_234*RLcp7_135-
 OPcp7_237*RLcp7_138-OPcp7_238*RLcp7_163-OPcp7_26*RLcp7_131;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_163;
    sens->P[2] = POcp7_263;
    sens->P[3] = POcp7_363;
    sens->R[1][1] = ROcp7_137;
    sens->R[1][2] = ROcp7_237;
    sens->R[1][3] = ROcp7_337;
    sens->R[2][1] = ROcp7_438;
    sens->R[2][2] = ROcp7_538;
    sens->R[2][3] = ROcp7_638;
    sens->R[3][1] = ROcp7_738;
    sens->R[3][2] = ROcp7_838;
    sens->R[3][3] = ROcp7_938;
    sens->V[1] = VIcp7_163;
    sens->V[2] = VIcp7_263;
    sens->V[3] = VIcp7_363;
    sens->OM[1] = OMcp7_138;
    sens->OM[2] = OMcp7_238;
    sens->OM[3] = OMcp7_338;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp7_163_5;
    sens->J[1][6] = JTcp7_163_6;
    sens->J[1][31] = JTcp7_163_7;
    sens->J[1][32] = JTcp7_163_8;
    sens->J[1][33] = JTcp7_163_9;
    sens->J[1][34] = JTcp7_163_10;
    sens->J[1][35] = JTcp7_163_11;
    sens->J[1][36] = JTcp7_163_12;
    sens->J[1][37] = JTcp7_163_13;
    sens->J[1][38] = JTcp7_163_14;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp7_263_4;
    sens->J[2][5] = JTcp7_263_5;
    sens->J[2][6] = JTcp7_263_6;
    sens->J[2][31] = JTcp7_263_7;
    sens->J[2][32] = JTcp7_263_8;
    sens->J[2][33] = JTcp7_263_9;
    sens->J[2][34] = JTcp7_263_10;
    sens->J[2][35] = JTcp7_263_11;
    sens->J[2][36] = JTcp7_263_12;
    sens->J[2][37] = JTcp7_263_13;
    sens->J[2][38] = JTcp7_263_14;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp7_363_4;
    sens->J[3][5] = JTcp7_363_5;
    sens->J[3][6] = JTcp7_363_6;
    sens->J[3][31] = JTcp7_363_7;
    sens->J[3][32] = JTcp7_363_8;
    sens->J[3][33] = JTcp7_363_9;
    sens->J[3][34] = JTcp7_363_10;
    sens->J[3][35] = JTcp7_363_11;
    sens->J[3][36] = JTcp7_363_12;
    sens->J[3][37] = JTcp7_363_13;
    sens->J[3][38] = JTcp7_363_14;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp7_46;
    sens->J[4][32] = ROcp7_131;
    sens->J[4][33] = ROcp7_432;
    sens->J[4][34] = ROcp7_733;
    sens->J[4][35] = ROcp7_134;
    sens->J[4][36] = ROcp7_735;
    sens->J[4][37] = ROcp7_436;
    sens->J[4][38] = ROcp7_137;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp7_85;
    sens->J[5][31] = ROcp7_56;
    sens->J[5][32] = ROcp7_231;
    sens->J[5][33] = ROcp7_532;
    sens->J[5][34] = ROcp7_833;
    sens->J[5][35] = ROcp7_234;
    sens->J[5][36] = ROcp7_835;
    sens->J[5][37] = ROcp7_536;
    sens->J[5][38] = ROcp7_237;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp7_95;
    sens->J[6][31] = ROcp7_66;
    sens->J[6][32] = ROcp7_331;
    sens->J[6][33] = ROcp7_632;
    sens->J[6][34] = ROcp7_933;
    sens->J[6][35] = ROcp7_334;
    sens->J[6][36] = ROcp7_935;
    sens->J[6][37] = ROcp7_636;
    sens->J[6][38] = ROcp7_337;
    sens->A[1] = ACcp7_163;
    sens->A[2] = ACcp7_263;
    sens->A[3] = ACcp7_363;
    sens->OMP[1] = OPcp7_138;
    sens->OMP[2] = OPcp7_238;
    sens->OMP[3] = OPcp7_338;
 
// 
break;
case 9:
 


// = = Block_1_0_0_9_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp8_25 = qd[5]*C4;
    OMcp8_35 = qd[5]*S4;
    OMcp8_16 = qd[4]+qd[6]*S5;
    OMcp8_26 = OMcp8_25+ROcp8_85*qd[6];
    OMcp8_36 = OMcp8_35+ROcp8_95*qd[6];
    OPcp8_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp8_26 = ROcp8_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp8_35*S5-ROcp8_95*qd[4]);
    OPcp8_36 = ROcp8_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp8_25*S5-ROcp8_85*qd[4]);

// = = Block_1_0_0_9_0_4 = = 
 
// Sensor Kinematics 


    ROcp8_131 = ROcp8_16*C31-S31*S5;
    ROcp8_231 = ROcp8_26*C31-ROcp8_85*S31;
    ROcp8_331 = ROcp8_36*C31-ROcp8_95*S31;
    ROcp8_731 = ROcp8_16*S31+C31*S5;
    ROcp8_831 = ROcp8_26*S31+ROcp8_85*C31;
    ROcp8_931 = ROcp8_36*S31+ROcp8_95*C31;
    ROcp8_432 = ROcp8_46*C32+ROcp8_731*S32;
    ROcp8_532 = ROcp8_56*C32+ROcp8_831*S32;
    ROcp8_632 = ROcp8_66*C32+ROcp8_931*S32;
    ROcp8_732 = -(ROcp8_46*S32-ROcp8_731*C32);
    ROcp8_832 = -(ROcp8_56*S32-ROcp8_831*C32);
    ROcp8_932 = -(ROcp8_66*S32-ROcp8_931*C32);
    ROcp8_133 = ROcp8_131*C33-ROcp8_732*S33;
    ROcp8_233 = ROcp8_231*C33-ROcp8_832*S33;
    ROcp8_333 = ROcp8_331*C33-ROcp8_932*S33;
    ROcp8_733 = ROcp8_131*S33+ROcp8_732*C33;
    ROcp8_833 = ROcp8_231*S33+ROcp8_832*C33;
    ROcp8_933 = ROcp8_331*S33+ROcp8_932*C33;
    ROcp8_134 = ROcp8_133*C34+ROcp8_432*S34;
    ROcp8_234 = ROcp8_233*C34+ROcp8_532*S34;
    ROcp8_334 = ROcp8_333*C34+ROcp8_632*S34;
    ROcp8_434 = -(ROcp8_133*S34-ROcp8_432*C34);
    ROcp8_534 = -(ROcp8_233*S34-ROcp8_532*C34);
    ROcp8_634 = -(ROcp8_333*S34-ROcp8_632*C34);
    RLcp8_131 = ROcp8_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp8_231 = ROcp8_26*s->dpt[1][3]+ROcp8_85*s->dpt[3][3];
    RLcp8_331 = ROcp8_36*s->dpt[1][3]+ROcp8_95*s->dpt[3][3];
    OMcp8_131 = OMcp8_16+ROcp8_46*qd[31];
    OMcp8_231 = OMcp8_26+ROcp8_56*qd[31];
    OMcp8_331 = OMcp8_36+ROcp8_66*qd[31];
    ORcp8_131 = OMcp8_26*RLcp8_331-OMcp8_36*RLcp8_231;
    ORcp8_231 = -(OMcp8_16*RLcp8_331-OMcp8_36*RLcp8_131);
    ORcp8_331 = OMcp8_16*RLcp8_231-OMcp8_26*RLcp8_131;
    OMcp8_132 = OMcp8_131+ROcp8_131*qd[32];
    OMcp8_232 = OMcp8_231+ROcp8_231*qd[32];
    OMcp8_332 = OMcp8_331+ROcp8_331*qd[32];
    OMcp8_133 = OMcp8_132+ROcp8_432*qd[33];
    OMcp8_233 = OMcp8_232+ROcp8_532*qd[33];
    OMcp8_333 = OMcp8_332+ROcp8_632*qd[33];
    OPcp8_133 = OPcp8_16+ROcp8_131*qdd[32]+ROcp8_432*qdd[33]+ROcp8_46*qdd[31]+qd[31]*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56)
 +qd[32]*(OMcp8_231*ROcp8_331-OMcp8_331*ROcp8_231)+qd[33]*(OMcp8_232*ROcp8_632-OMcp8_332*ROcp8_532);
    OPcp8_233 = OPcp8_26+ROcp8_231*qdd[32]+ROcp8_532*qdd[33]+ROcp8_56*qdd[31]-qd[31]*(OMcp8_16*ROcp8_66-OMcp8_36*ROcp8_46)
 -qd[32]*(OMcp8_131*ROcp8_331-OMcp8_331*ROcp8_131)-qd[33]*(OMcp8_132*ROcp8_632-OMcp8_332*ROcp8_432);
    OPcp8_333 = OPcp8_36+ROcp8_331*qdd[32]+ROcp8_632*qdd[33]+ROcp8_66*qdd[31]+qd[31]*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46)
 +qd[32]*(OMcp8_131*ROcp8_231-OMcp8_231*ROcp8_131)+qd[33]*(OMcp8_132*ROcp8_532-OMcp8_232*ROcp8_432);
    RLcp8_134 = ROcp8_733*s->dpt[3][21];
    RLcp8_234 = ROcp8_833*s->dpt[3][21];
    RLcp8_334 = ROcp8_933*s->dpt[3][21];
    OMcp8_134 = OMcp8_133+ROcp8_733*qd[34];
    OMcp8_234 = OMcp8_233+ROcp8_833*qd[34];
    OMcp8_334 = OMcp8_333+ROcp8_933*qd[34];
    ORcp8_134 = OMcp8_233*RLcp8_334-OMcp8_333*RLcp8_234;
    ORcp8_234 = -(OMcp8_133*RLcp8_334-OMcp8_333*RLcp8_134);
    ORcp8_334 = OMcp8_133*RLcp8_234-OMcp8_233*RLcp8_134;
    OPcp8_134 = OPcp8_133+ROcp8_733*qdd[34]+qd[34]*(OMcp8_233*ROcp8_933-OMcp8_333*ROcp8_833);
    OPcp8_234 = OPcp8_233+ROcp8_833*qdd[34]-qd[34]*(OMcp8_133*ROcp8_933-OMcp8_333*ROcp8_733);
    OPcp8_334 = OPcp8_333+ROcp8_933*qdd[34]+qd[34]*(OMcp8_133*ROcp8_833-OMcp8_233*ROcp8_733);

// = = Block_1_0_0_9_0_5 = = 
 
// Sensor Kinematics 


    ROcp8_435 = ROcp8_434*C35+ROcp8_733*S35;
    ROcp8_535 = ROcp8_534*C35+ROcp8_833*S35;
    ROcp8_635 = ROcp8_634*C35+ROcp8_933*S35;
    ROcp8_735 = -(ROcp8_434*S35-ROcp8_733*C35);
    ROcp8_835 = -(ROcp8_534*S35-ROcp8_833*C35);
    ROcp8_935 = -(ROcp8_634*S35-ROcp8_933*C35);
    ROcp8_136 = ROcp8_134*C36+ROcp8_435*S36;
    ROcp8_236 = ROcp8_234*C36+ROcp8_535*S36;
    ROcp8_336 = ROcp8_334*C36+ROcp8_635*S36;
    ROcp8_436 = -(ROcp8_134*S36-ROcp8_435*C36);
    ROcp8_536 = -(ROcp8_234*S36-ROcp8_535*C36);
    ROcp8_636 = -(ROcp8_334*S36-ROcp8_635*C36);
    ROcp8_137 = ROcp8_136*C37-ROcp8_735*S37;
    ROcp8_237 = ROcp8_236*C37-ROcp8_835*S37;
    ROcp8_337 = ROcp8_336*C37-ROcp8_935*S37;
    ROcp8_737 = ROcp8_136*S37+ROcp8_735*C37;
    ROcp8_837 = ROcp8_236*S37+ROcp8_835*C37;
    ROcp8_937 = ROcp8_336*S37+ROcp8_935*C37;
    ROcp8_438 = ROcp8_436*C38+ROcp8_737*S38;
    ROcp8_538 = ROcp8_536*C38+ROcp8_837*S38;
    ROcp8_638 = ROcp8_636*C38+ROcp8_937*S38;
    ROcp8_738 = -(ROcp8_436*S38-ROcp8_737*C38);
    ROcp8_838 = -(ROcp8_536*S38-ROcp8_837*C38);
    ROcp8_938 = -(ROcp8_636*S38-ROcp8_937*C38);
    ROcp8_139 = ROcp8_137*C39+ROcp8_438*S39;
    ROcp8_239 = ROcp8_237*C39+ROcp8_538*S39;
    ROcp8_339 = ROcp8_337*C39+ROcp8_638*S39;
    ROcp8_439 = -(ROcp8_137*S39-ROcp8_438*C39);
    ROcp8_539 = -(ROcp8_237*S39-ROcp8_538*C39);
    ROcp8_639 = -(ROcp8_337*S39-ROcp8_638*C39);
    RLcp8_135 = ROcp8_134*s->dpt[1][23]+ROcp8_434*s->dpt[2][23]+ROcp8_733*s->dpt[3][23];
    RLcp8_235 = ROcp8_234*s->dpt[1][23]+ROcp8_534*s->dpt[2][23]+ROcp8_833*s->dpt[3][23];
    RLcp8_335 = ROcp8_334*s->dpt[1][23]+ROcp8_634*s->dpt[2][23]+ROcp8_933*s->dpt[3][23];
    ORcp8_135 = OMcp8_234*RLcp8_335-OMcp8_334*RLcp8_235;
    ORcp8_235 = -(OMcp8_134*RLcp8_335-OMcp8_334*RLcp8_135);
    ORcp8_335 = OMcp8_134*RLcp8_235-OMcp8_234*RLcp8_135;
    OMcp8_137 = OMcp8_134+ROcp8_436*qd[37];
    OMcp8_237 = OMcp8_234+ROcp8_536*qd[37];
    OMcp8_337 = OMcp8_334+ROcp8_636*qd[37];
    OPcp8_137 = OPcp8_134+ROcp8_436*qdd[37]+qd[37]*(OMcp8_234*ROcp8_636-OMcp8_334*ROcp8_536);
    OPcp8_237 = OPcp8_234+ROcp8_536*qdd[37]-qd[37]*(OMcp8_134*ROcp8_636-OMcp8_334*ROcp8_436);
    OPcp8_337 = OPcp8_334+ROcp8_636*qdd[37]+qd[37]*(OMcp8_134*ROcp8_536-OMcp8_234*ROcp8_436);
    RLcp8_138 = ROcp8_436*s->dpt[2][27]+ROcp8_737*s->dpt[3][27];
    RLcp8_238 = ROcp8_536*s->dpt[2][27]+ROcp8_837*s->dpt[3][27];
    RLcp8_338 = ROcp8_636*s->dpt[2][27]+ROcp8_937*s->dpt[3][27];
    OMcp8_138 = OMcp8_137+ROcp8_137*qd[38];
    OMcp8_238 = OMcp8_237+ROcp8_237*qd[38];
    OMcp8_338 = OMcp8_337+ROcp8_337*qd[38];
    ORcp8_138 = OMcp8_237*RLcp8_338-OMcp8_337*RLcp8_238;
    ORcp8_238 = -(OMcp8_137*RLcp8_338-OMcp8_337*RLcp8_138);
    ORcp8_338 = OMcp8_137*RLcp8_238-OMcp8_237*RLcp8_138;
    OPcp8_138 = OPcp8_137+ROcp8_137*qdd[38]+qd[38]*(OMcp8_237*ROcp8_337-OMcp8_337*ROcp8_237);
    OPcp8_238 = OPcp8_237+ROcp8_237*qdd[38]-qd[38]*(OMcp8_137*ROcp8_337-OMcp8_337*ROcp8_137);
    OPcp8_338 = OPcp8_337+ROcp8_337*qdd[38]+qd[38]*(OMcp8_137*ROcp8_237-OMcp8_237*ROcp8_137);
    RLcp8_139 = ROcp8_738*s->dpt[3][29];
    RLcp8_239 = ROcp8_838*s->dpt[3][29];
    RLcp8_339 = ROcp8_938*s->dpt[3][29];
    OMcp8_139 = OMcp8_138+ROcp8_738*qd[39];
    OMcp8_239 = OMcp8_238+ROcp8_838*qd[39];
    OMcp8_339 = OMcp8_338+ROcp8_938*qd[39];
    ORcp8_139 = OMcp8_238*RLcp8_339-OMcp8_338*RLcp8_239;
    ORcp8_239 = -(OMcp8_138*RLcp8_339-OMcp8_338*RLcp8_139);
    ORcp8_339 = OMcp8_138*RLcp8_239-OMcp8_238*RLcp8_139;
    OPcp8_139 = OPcp8_138+ROcp8_738*qdd[39]+qd[39]*(OMcp8_238*ROcp8_938-OMcp8_338*ROcp8_838);
    OPcp8_239 = OPcp8_238+ROcp8_838*qdd[39]-qd[39]*(OMcp8_138*ROcp8_938-OMcp8_338*ROcp8_738);
    OPcp8_339 = OPcp8_338+ROcp8_938*qdd[39]+qd[39]*(OMcp8_138*ROcp8_838-OMcp8_238*ROcp8_738);
    RLcp8_164 = ROcp8_139*s->dpt[1][32]+ROcp8_439*s->dpt[2][32]+ROcp8_738*s->dpt[3][32];
    RLcp8_264 = ROcp8_239*s->dpt[1][32]+ROcp8_539*s->dpt[2][32]+ROcp8_838*s->dpt[3][32];
    RLcp8_364 = ROcp8_339*s->dpt[1][32]+ROcp8_639*s->dpt[2][32]+ROcp8_938*s->dpt[3][32];
    POcp8_164 = RLcp8_131+RLcp8_134+RLcp8_135+RLcp8_138+RLcp8_139+RLcp8_164+q[1];
    POcp8_264 = RLcp8_231+RLcp8_234+RLcp8_235+RLcp8_238+RLcp8_239+RLcp8_264+q[2];
    POcp8_364 = RLcp8_331+RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339+RLcp8_364+q[3];
    JTcp8_264_4 = -(RLcp8_331+RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339+RLcp8_364);
    JTcp8_364_4 = RLcp8_231+RLcp8_234+RLcp8_235+RLcp8_238+RLcp8_239+RLcp8_264;
    JTcp8_164_5 = C4*(RLcp8_331+RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339+RLcp8_364)-S4*(RLcp8_231+RLcp8_234)-S4*(RLcp8_235+
 RLcp8_238)-S4*(RLcp8_239+RLcp8_264);
    JTcp8_264_5 = S4*(RLcp8_131+RLcp8_134+RLcp8_135+RLcp8_138+RLcp8_139+RLcp8_164);
    JTcp8_364_5 = -C4*(RLcp8_131+RLcp8_134+RLcp8_135+RLcp8_138+RLcp8_139+RLcp8_164);
    JTcp8_164_6 = ROcp8_85*(RLcp8_331+RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339+RLcp8_364)-ROcp8_95*(RLcp8_231+RLcp8_234)-
 ROcp8_95*(RLcp8_235+RLcp8_238)-ROcp8_95*(RLcp8_239+RLcp8_264);
    JTcp8_264_6 = RLcp8_164*ROcp8_95-RLcp8_339*S5-RLcp8_364*S5+ROcp8_95*(RLcp8_131+RLcp8_134+RLcp8_135+RLcp8_138+RLcp8_139
 )-S5*(RLcp8_331+RLcp8_334)-S5*(RLcp8_335+RLcp8_338);
    JTcp8_364_6 = RLcp8_239*S5-ROcp8_85*(RLcp8_131+RLcp8_134+RLcp8_135+RLcp8_138+RLcp8_139)+S5*(RLcp8_231+RLcp8_234)+S5*(
 RLcp8_235+RLcp8_238)-RLcp8_164*ROcp8_85+RLcp8_264*S5;
    JTcp8_164_7 = ROcp8_56*(RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339)-ROcp8_66*(RLcp8_234+RLcp8_235)-ROcp8_66*(RLcp8_238+
 RLcp8_239)-RLcp8_264*ROcp8_66+RLcp8_364*ROcp8_56;
    JTcp8_264_7 = RLcp8_164*ROcp8_66-RLcp8_364*ROcp8_46-ROcp8_46*(RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339)+ROcp8_66*(
 RLcp8_134+RLcp8_135)+ROcp8_66*(RLcp8_138+RLcp8_139);
    JTcp8_364_7 = ROcp8_46*(RLcp8_234+RLcp8_235+RLcp8_238+RLcp8_239)-ROcp8_56*(RLcp8_134+RLcp8_135)-ROcp8_56*(RLcp8_138+
 RLcp8_139)-RLcp8_164*ROcp8_56+RLcp8_264*ROcp8_46;
    JTcp8_164_8 = ROcp8_231*(RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339)-ROcp8_331*(RLcp8_234+RLcp8_235)-ROcp8_331*(RLcp8_238
 +RLcp8_239)-RLcp8_264*ROcp8_331+RLcp8_364*ROcp8_231;
    JTcp8_264_8 = RLcp8_164*ROcp8_331-RLcp8_364*ROcp8_131-ROcp8_131*(RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339)+ROcp8_331*(
 RLcp8_134+RLcp8_135)+ROcp8_331*(RLcp8_138+RLcp8_139);
    JTcp8_364_8 = ROcp8_131*(RLcp8_234+RLcp8_235+RLcp8_238+RLcp8_239)-ROcp8_231*(RLcp8_134+RLcp8_135)-ROcp8_231*(RLcp8_138
 +RLcp8_139)-RLcp8_164*ROcp8_231+RLcp8_264*ROcp8_131;
    JTcp8_164_9 = ROcp8_532*(RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339)-ROcp8_632*(RLcp8_234+RLcp8_235)-ROcp8_632*(RLcp8_238
 +RLcp8_239)-RLcp8_264*ROcp8_632+RLcp8_364*ROcp8_532;
    JTcp8_264_9 = RLcp8_164*ROcp8_632-RLcp8_364*ROcp8_432-ROcp8_432*(RLcp8_334+RLcp8_335+RLcp8_338+RLcp8_339)+ROcp8_632*(
 RLcp8_134+RLcp8_135)+ROcp8_632*(RLcp8_138+RLcp8_139);
    JTcp8_364_9 = ROcp8_432*(RLcp8_234+RLcp8_235+RLcp8_238+RLcp8_239)-ROcp8_532*(RLcp8_134+RLcp8_135)-ROcp8_532*(RLcp8_138
 +RLcp8_139)-RLcp8_164*ROcp8_532+RLcp8_264*ROcp8_432;
    JTcp8_164_10 = ROcp8_833*(RLcp8_335+RLcp8_338+RLcp8_339+RLcp8_364)-ROcp8_933*(RLcp8_235+RLcp8_238)-ROcp8_933*(
 RLcp8_239+RLcp8_264);
    JTcp8_264_10 = -(ROcp8_733*(RLcp8_335+RLcp8_338+RLcp8_339+RLcp8_364)-ROcp8_933*(RLcp8_135+RLcp8_138)-ROcp8_933*(
 RLcp8_139+RLcp8_164));
    JTcp8_364_10 = ROcp8_733*(RLcp8_235+RLcp8_238+RLcp8_239+RLcp8_264)-ROcp8_833*(RLcp8_135+RLcp8_138)-ROcp8_833*(
 RLcp8_139+RLcp8_164);
    JTcp8_164_11 = ROcp8_234*(RLcp8_338+RLcp8_339)-ROcp8_334*(RLcp8_238+RLcp8_239)-RLcp8_264*ROcp8_334+RLcp8_364*ROcp8_234;
    JTcp8_264_11 = RLcp8_164*ROcp8_334-RLcp8_364*ROcp8_134-ROcp8_134*(RLcp8_338+RLcp8_339)+ROcp8_334*(RLcp8_138+RLcp8_139);
    JTcp8_364_11 = ROcp8_134*(RLcp8_238+RLcp8_239)-ROcp8_234*(RLcp8_138+RLcp8_139)-RLcp8_164*ROcp8_234+RLcp8_264*ROcp8_134;
    JTcp8_164_12 = ROcp8_835*(RLcp8_338+RLcp8_339)-ROcp8_935*(RLcp8_238+RLcp8_239)-RLcp8_264*ROcp8_935+RLcp8_364*ROcp8_835;
    JTcp8_264_12 = RLcp8_164*ROcp8_935-RLcp8_364*ROcp8_735-ROcp8_735*(RLcp8_338+RLcp8_339)+ROcp8_935*(RLcp8_138+RLcp8_139);
    JTcp8_364_12 = ROcp8_735*(RLcp8_238+RLcp8_239)-ROcp8_835*(RLcp8_138+RLcp8_139)-RLcp8_164*ROcp8_835+RLcp8_264*ROcp8_735;
    JTcp8_164_13 = ROcp8_536*(RLcp8_338+RLcp8_339)-ROcp8_636*(RLcp8_238+RLcp8_239)-RLcp8_264*ROcp8_636+RLcp8_364*ROcp8_536;
    JTcp8_264_13 = RLcp8_164*ROcp8_636-RLcp8_364*ROcp8_436-ROcp8_436*(RLcp8_338+RLcp8_339)+ROcp8_636*(RLcp8_138+RLcp8_139);
    JTcp8_364_13 = ROcp8_436*(RLcp8_238+RLcp8_239)-ROcp8_536*(RLcp8_138+RLcp8_139)-RLcp8_164*ROcp8_536+RLcp8_264*ROcp8_436;
    JTcp8_164_14 = ROcp8_237*(RLcp8_339+RLcp8_364)-ROcp8_337*(RLcp8_239+RLcp8_264);
    JTcp8_264_14 = -(ROcp8_137*(RLcp8_339+RLcp8_364)-ROcp8_337*(RLcp8_139+RLcp8_164));
    JTcp8_364_14 = ROcp8_137*(RLcp8_239+RLcp8_264)-ROcp8_237*(RLcp8_139+RLcp8_164);
    JTcp8_164_15 = -(RLcp8_264*ROcp8_938-RLcp8_364*ROcp8_838);
    JTcp8_264_15 = RLcp8_164*ROcp8_938-RLcp8_364*ROcp8_738;
    JTcp8_364_15 = -(RLcp8_164*ROcp8_838-RLcp8_264*ROcp8_738);
    ORcp8_164 = OMcp8_239*RLcp8_364-OMcp8_339*RLcp8_264;
    ORcp8_264 = -(OMcp8_139*RLcp8_364-OMcp8_339*RLcp8_164);
    ORcp8_364 = OMcp8_139*RLcp8_264-OMcp8_239*RLcp8_164;
    VIcp8_164 = ORcp8_131+ORcp8_134+ORcp8_135+ORcp8_138+ORcp8_139+ORcp8_164+qd[1];
    VIcp8_264 = ORcp8_231+ORcp8_234+ORcp8_235+ORcp8_238+ORcp8_239+ORcp8_264+qd[2];
    VIcp8_364 = ORcp8_331+ORcp8_334+ORcp8_335+ORcp8_338+ORcp8_339+ORcp8_364+qd[3];
    ACcp8_164 = qdd[1]+OMcp8_233*ORcp8_334+OMcp8_234*ORcp8_335+OMcp8_237*ORcp8_338+OMcp8_238*ORcp8_339+OMcp8_239*ORcp8_364
 +OMcp8_26*ORcp8_331-OMcp8_333*ORcp8_234-OMcp8_334*ORcp8_235-OMcp8_337*ORcp8_238-OMcp8_338*ORcp8_239-OMcp8_339*ORcp8_264-
 OMcp8_36*ORcp8_231+OPcp8_233*RLcp8_334+OPcp8_234*RLcp8_335+OPcp8_237*RLcp8_338+OPcp8_238*RLcp8_339+OPcp8_239*RLcp8_364+
 OPcp8_26*RLcp8_331-OPcp8_333*RLcp8_234-OPcp8_334*RLcp8_235-OPcp8_337*RLcp8_238-OPcp8_338*RLcp8_239-OPcp8_339*RLcp8_264-
 OPcp8_36*RLcp8_231;
    ACcp8_264 = qdd[2]-OMcp8_133*ORcp8_334-OMcp8_134*ORcp8_335-OMcp8_137*ORcp8_338-OMcp8_138*ORcp8_339-OMcp8_139*ORcp8_364
 -OMcp8_16*ORcp8_331+OMcp8_333*ORcp8_134+OMcp8_334*ORcp8_135+OMcp8_337*ORcp8_138+OMcp8_338*ORcp8_139+OMcp8_339*ORcp8_164+
 OMcp8_36*ORcp8_131-OPcp8_133*RLcp8_334-OPcp8_134*RLcp8_335-OPcp8_137*RLcp8_338-OPcp8_138*RLcp8_339-OPcp8_139*RLcp8_364-
 OPcp8_16*RLcp8_331+OPcp8_333*RLcp8_134+OPcp8_334*RLcp8_135+OPcp8_337*RLcp8_138+OPcp8_338*RLcp8_139+OPcp8_339*RLcp8_164+
 OPcp8_36*RLcp8_131;
    ACcp8_364 = qdd[3]+OMcp8_133*ORcp8_234+OMcp8_134*ORcp8_235+OMcp8_137*ORcp8_238+OMcp8_138*ORcp8_239+OMcp8_139*ORcp8_264
 +OMcp8_16*ORcp8_231-OMcp8_233*ORcp8_134-OMcp8_234*ORcp8_135-OMcp8_237*ORcp8_138-OMcp8_238*ORcp8_139-OMcp8_239*ORcp8_164-
 OMcp8_26*ORcp8_131+OPcp8_133*RLcp8_234+OPcp8_134*RLcp8_235+OPcp8_137*RLcp8_238+OPcp8_138*RLcp8_239+OPcp8_139*RLcp8_264+
 OPcp8_16*RLcp8_231-OPcp8_233*RLcp8_134-OPcp8_234*RLcp8_135-OPcp8_237*RLcp8_138-OPcp8_238*RLcp8_139-OPcp8_239*RLcp8_164-
 OPcp8_26*RLcp8_131;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_164;
    sens->P[2] = POcp8_264;
    sens->P[3] = POcp8_364;
    sens->R[1][1] = ROcp8_139;
    sens->R[1][2] = ROcp8_239;
    sens->R[1][3] = ROcp8_339;
    sens->R[2][1] = ROcp8_439;
    sens->R[2][2] = ROcp8_539;
    sens->R[2][3] = ROcp8_639;
    sens->R[3][1] = ROcp8_738;
    sens->R[3][2] = ROcp8_838;
    sens->R[3][3] = ROcp8_938;
    sens->V[1] = VIcp8_164;
    sens->V[2] = VIcp8_264;
    sens->V[3] = VIcp8_364;
    sens->OM[1] = OMcp8_139;
    sens->OM[2] = OMcp8_239;
    sens->OM[3] = OMcp8_339;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp8_164_5;
    sens->J[1][6] = JTcp8_164_6;
    sens->J[1][31] = JTcp8_164_7;
    sens->J[1][32] = JTcp8_164_8;
    sens->J[1][33] = JTcp8_164_9;
    sens->J[1][34] = JTcp8_164_10;
    sens->J[1][35] = JTcp8_164_11;
    sens->J[1][36] = JTcp8_164_12;
    sens->J[1][37] = JTcp8_164_13;
    sens->J[1][38] = JTcp8_164_14;
    sens->J[1][39] = JTcp8_164_15;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp8_264_4;
    sens->J[2][5] = JTcp8_264_5;
    sens->J[2][6] = JTcp8_264_6;
    sens->J[2][31] = JTcp8_264_7;
    sens->J[2][32] = JTcp8_264_8;
    sens->J[2][33] = JTcp8_264_9;
    sens->J[2][34] = JTcp8_264_10;
    sens->J[2][35] = JTcp8_264_11;
    sens->J[2][36] = JTcp8_264_12;
    sens->J[2][37] = JTcp8_264_13;
    sens->J[2][38] = JTcp8_264_14;
    sens->J[2][39] = JTcp8_264_15;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp8_364_4;
    sens->J[3][5] = JTcp8_364_5;
    sens->J[3][6] = JTcp8_364_6;
    sens->J[3][31] = JTcp8_364_7;
    sens->J[3][32] = JTcp8_364_8;
    sens->J[3][33] = JTcp8_364_9;
    sens->J[3][34] = JTcp8_364_10;
    sens->J[3][35] = JTcp8_364_11;
    sens->J[3][36] = JTcp8_364_12;
    sens->J[3][37] = JTcp8_364_13;
    sens->J[3][38] = JTcp8_364_14;
    sens->J[3][39] = JTcp8_364_15;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp8_46;
    sens->J[4][32] = ROcp8_131;
    sens->J[4][33] = ROcp8_432;
    sens->J[4][34] = ROcp8_733;
    sens->J[4][35] = ROcp8_134;
    sens->J[4][36] = ROcp8_735;
    sens->J[4][37] = ROcp8_436;
    sens->J[4][38] = ROcp8_137;
    sens->J[4][39] = ROcp8_738;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp8_85;
    sens->J[5][31] = ROcp8_56;
    sens->J[5][32] = ROcp8_231;
    sens->J[5][33] = ROcp8_532;
    sens->J[5][34] = ROcp8_833;
    sens->J[5][35] = ROcp8_234;
    sens->J[5][36] = ROcp8_835;
    sens->J[5][37] = ROcp8_536;
    sens->J[5][38] = ROcp8_237;
    sens->J[5][39] = ROcp8_838;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp8_95;
    sens->J[6][31] = ROcp8_66;
    sens->J[6][32] = ROcp8_331;
    sens->J[6][33] = ROcp8_632;
    sens->J[6][34] = ROcp8_933;
    sens->J[6][35] = ROcp8_334;
    sens->J[6][36] = ROcp8_935;
    sens->J[6][37] = ROcp8_636;
    sens->J[6][38] = ROcp8_337;
    sens->J[6][39] = ROcp8_938;
    sens->A[1] = ACcp8_164;
    sens->A[2] = ACcp8_264;
    sens->A[3] = ACcp8_364;
    sens->OMP[1] = OPcp8_139;
    sens->OMP[2] = OPcp8_239;
    sens->OMP[3] = OPcp8_339;
 
// 
break;
case 10:
 


// = = Block_1_0_0_10_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp9_25 = qd[5]*C4;
    OMcp9_35 = qd[5]*S4;
    OMcp9_16 = qd[4]+qd[6]*S5;
    OMcp9_26 = OMcp9_25+ROcp9_85*qd[6];
    OMcp9_36 = OMcp9_35+ROcp9_95*qd[6];
    OPcp9_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp9_26 = ROcp9_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp9_35*S5-ROcp9_95*qd[4]);
    OPcp9_36 = ROcp9_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp9_25*S5-ROcp9_85*qd[4]);

// = = Block_1_0_0_10_0_4 = = 
 
// Sensor Kinematics 


    ROcp9_131 = ROcp9_16*C31-S31*S5;
    ROcp9_231 = ROcp9_26*C31-ROcp9_85*S31;
    ROcp9_331 = ROcp9_36*C31-ROcp9_95*S31;
    ROcp9_731 = ROcp9_16*S31+C31*S5;
    ROcp9_831 = ROcp9_26*S31+ROcp9_85*C31;
    ROcp9_931 = ROcp9_36*S31+ROcp9_95*C31;
    ROcp9_432 = ROcp9_46*C32+ROcp9_731*S32;
    ROcp9_532 = ROcp9_56*C32+ROcp9_831*S32;
    ROcp9_632 = ROcp9_66*C32+ROcp9_931*S32;
    ROcp9_732 = -(ROcp9_46*S32-ROcp9_731*C32);
    ROcp9_832 = -(ROcp9_56*S32-ROcp9_831*C32);
    ROcp9_932 = -(ROcp9_66*S32-ROcp9_931*C32);
    ROcp9_133 = ROcp9_131*C33-ROcp9_732*S33;
    ROcp9_233 = ROcp9_231*C33-ROcp9_832*S33;
    ROcp9_333 = ROcp9_331*C33-ROcp9_932*S33;
    ROcp9_733 = ROcp9_131*S33+ROcp9_732*C33;
    ROcp9_833 = ROcp9_231*S33+ROcp9_832*C33;
    ROcp9_933 = ROcp9_331*S33+ROcp9_932*C33;
    ROcp9_134 = ROcp9_133*C34+ROcp9_432*S34;
    ROcp9_234 = ROcp9_233*C34+ROcp9_532*S34;
    ROcp9_334 = ROcp9_333*C34+ROcp9_632*S34;
    ROcp9_434 = -(ROcp9_133*S34-ROcp9_432*C34);
    ROcp9_534 = -(ROcp9_233*S34-ROcp9_532*C34);
    ROcp9_634 = -(ROcp9_333*S34-ROcp9_632*C34);
    RLcp9_131 = ROcp9_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp9_231 = ROcp9_26*s->dpt[1][3]+ROcp9_85*s->dpt[3][3];
    RLcp9_331 = ROcp9_36*s->dpt[1][3]+ROcp9_95*s->dpt[3][3];
    OMcp9_131 = OMcp9_16+ROcp9_46*qd[31];
    OMcp9_231 = OMcp9_26+ROcp9_56*qd[31];
    OMcp9_331 = OMcp9_36+ROcp9_66*qd[31];
    ORcp9_131 = OMcp9_26*RLcp9_331-OMcp9_36*RLcp9_231;
    ORcp9_231 = -(OMcp9_16*RLcp9_331-OMcp9_36*RLcp9_131);
    ORcp9_331 = OMcp9_16*RLcp9_231-OMcp9_26*RLcp9_131;
    OMcp9_132 = OMcp9_131+ROcp9_131*qd[32];
    OMcp9_232 = OMcp9_231+ROcp9_231*qd[32];
    OMcp9_332 = OMcp9_331+ROcp9_331*qd[32];
    OMcp9_133 = OMcp9_132+ROcp9_432*qd[33];
    OMcp9_233 = OMcp9_232+ROcp9_532*qd[33];
    OMcp9_333 = OMcp9_332+ROcp9_632*qd[33];
    OPcp9_133 = OPcp9_16+ROcp9_131*qdd[32]+ROcp9_432*qdd[33]+ROcp9_46*qdd[31]+qd[31]*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56)
 +qd[32]*(OMcp9_231*ROcp9_331-OMcp9_331*ROcp9_231)+qd[33]*(OMcp9_232*ROcp9_632-OMcp9_332*ROcp9_532);
    OPcp9_233 = OPcp9_26+ROcp9_231*qdd[32]+ROcp9_532*qdd[33]+ROcp9_56*qdd[31]-qd[31]*(OMcp9_16*ROcp9_66-OMcp9_36*ROcp9_46)
 -qd[32]*(OMcp9_131*ROcp9_331-OMcp9_331*ROcp9_131)-qd[33]*(OMcp9_132*ROcp9_632-OMcp9_332*ROcp9_432);
    OPcp9_333 = OPcp9_36+ROcp9_331*qdd[32]+ROcp9_632*qdd[33]+ROcp9_66*qdd[31]+qd[31]*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46)
 +qd[32]*(OMcp9_131*ROcp9_231-OMcp9_231*ROcp9_131)+qd[33]*(OMcp9_132*ROcp9_532-OMcp9_232*ROcp9_432);
    RLcp9_134 = ROcp9_733*s->dpt[3][21];
    RLcp9_234 = ROcp9_833*s->dpt[3][21];
    RLcp9_334 = ROcp9_933*s->dpt[3][21];
    OMcp9_134 = OMcp9_133+ROcp9_733*qd[34];
    OMcp9_234 = OMcp9_233+ROcp9_833*qd[34];
    OMcp9_334 = OMcp9_333+ROcp9_933*qd[34];
    ORcp9_134 = OMcp9_233*RLcp9_334-OMcp9_333*RLcp9_234;
    ORcp9_234 = -(OMcp9_133*RLcp9_334-OMcp9_333*RLcp9_134);
    ORcp9_334 = OMcp9_133*RLcp9_234-OMcp9_233*RLcp9_134;
    OPcp9_134 = OPcp9_133+ROcp9_733*qdd[34]+qd[34]*(OMcp9_233*ROcp9_933-OMcp9_333*ROcp9_833);
    OPcp9_234 = OPcp9_233+ROcp9_833*qdd[34]-qd[34]*(OMcp9_133*ROcp9_933-OMcp9_333*ROcp9_733);
    OPcp9_334 = OPcp9_333+ROcp9_933*qdd[34]+qd[34]*(OMcp9_133*ROcp9_833-OMcp9_233*ROcp9_733);

// = = Block_1_0_0_10_0_5 = = 
 
// Sensor Kinematics 


    ROcp9_435 = ROcp9_434*C35+ROcp9_733*S35;
    ROcp9_535 = ROcp9_534*C35+ROcp9_833*S35;
    ROcp9_635 = ROcp9_634*C35+ROcp9_933*S35;
    ROcp9_735 = -(ROcp9_434*S35-ROcp9_733*C35);
    ROcp9_835 = -(ROcp9_534*S35-ROcp9_833*C35);
    ROcp9_935 = -(ROcp9_634*S35-ROcp9_933*C35);
    ROcp9_136 = ROcp9_134*C36+ROcp9_435*S36;
    ROcp9_236 = ROcp9_234*C36+ROcp9_535*S36;
    ROcp9_336 = ROcp9_334*C36+ROcp9_635*S36;
    ROcp9_436 = -(ROcp9_134*S36-ROcp9_435*C36);
    ROcp9_536 = -(ROcp9_234*S36-ROcp9_535*C36);
    ROcp9_636 = -(ROcp9_334*S36-ROcp9_635*C36);
    ROcp9_137 = ROcp9_136*C37-ROcp9_735*S37;
    ROcp9_237 = ROcp9_236*C37-ROcp9_835*S37;
    ROcp9_337 = ROcp9_336*C37-ROcp9_935*S37;
    ROcp9_737 = ROcp9_136*S37+ROcp9_735*C37;
    ROcp9_837 = ROcp9_236*S37+ROcp9_835*C37;
    ROcp9_937 = ROcp9_336*S37+ROcp9_935*C37;
    ROcp9_438 = ROcp9_436*C38+ROcp9_737*S38;
    ROcp9_538 = ROcp9_536*C38+ROcp9_837*S38;
    ROcp9_638 = ROcp9_636*C38+ROcp9_937*S38;
    ROcp9_738 = -(ROcp9_436*S38-ROcp9_737*C38);
    ROcp9_838 = -(ROcp9_536*S38-ROcp9_837*C38);
    ROcp9_938 = -(ROcp9_636*S38-ROcp9_937*C38);
    ROcp9_139 = ROcp9_137*C39+ROcp9_438*S39;
    ROcp9_239 = ROcp9_237*C39+ROcp9_538*S39;
    ROcp9_339 = ROcp9_337*C39+ROcp9_638*S39;
    ROcp9_439 = -(ROcp9_137*S39-ROcp9_438*C39);
    ROcp9_539 = -(ROcp9_237*S39-ROcp9_538*C39);
    ROcp9_639 = -(ROcp9_337*S39-ROcp9_638*C39);
    ROcp9_140 = ROcp9_139*C40-ROcp9_738*S40;
    ROcp9_240 = ROcp9_239*C40-ROcp9_838*S40;
    ROcp9_340 = ROcp9_339*C40-ROcp9_938*S40;
    ROcp9_740 = ROcp9_139*S40+ROcp9_738*C40;
    ROcp9_840 = ROcp9_239*S40+ROcp9_838*C40;
    ROcp9_940 = ROcp9_339*S40+ROcp9_938*C40;
    RLcp9_135 = ROcp9_134*s->dpt[1][23]+ROcp9_434*s->dpt[2][23]+ROcp9_733*s->dpt[3][23];
    RLcp9_235 = ROcp9_234*s->dpt[1][23]+ROcp9_534*s->dpt[2][23]+ROcp9_833*s->dpt[3][23];
    RLcp9_335 = ROcp9_334*s->dpt[1][23]+ROcp9_634*s->dpt[2][23]+ROcp9_933*s->dpt[3][23];
    ORcp9_135 = OMcp9_234*RLcp9_335-OMcp9_334*RLcp9_235;
    ORcp9_235 = -(OMcp9_134*RLcp9_335-OMcp9_334*RLcp9_135);
    ORcp9_335 = OMcp9_134*RLcp9_235-OMcp9_234*RLcp9_135;
    OMcp9_137 = OMcp9_134+ROcp9_436*qd[37];
    OMcp9_237 = OMcp9_234+ROcp9_536*qd[37];
    OMcp9_337 = OMcp9_334+ROcp9_636*qd[37];
    OPcp9_137 = OPcp9_134+ROcp9_436*qdd[37]+qd[37]*(OMcp9_234*ROcp9_636-OMcp9_334*ROcp9_536);
    OPcp9_237 = OPcp9_234+ROcp9_536*qdd[37]-qd[37]*(OMcp9_134*ROcp9_636-OMcp9_334*ROcp9_436);
    OPcp9_337 = OPcp9_334+ROcp9_636*qdd[37]+qd[37]*(OMcp9_134*ROcp9_536-OMcp9_234*ROcp9_436);
    RLcp9_138 = ROcp9_436*s->dpt[2][27]+ROcp9_737*s->dpt[3][27];
    RLcp9_238 = ROcp9_536*s->dpt[2][27]+ROcp9_837*s->dpt[3][27];
    RLcp9_338 = ROcp9_636*s->dpt[2][27]+ROcp9_937*s->dpt[3][27];
    OMcp9_138 = OMcp9_137+ROcp9_137*qd[38];
    OMcp9_238 = OMcp9_237+ROcp9_237*qd[38];
    OMcp9_338 = OMcp9_337+ROcp9_337*qd[38];
    ORcp9_138 = OMcp9_237*RLcp9_338-OMcp9_337*RLcp9_238;
    ORcp9_238 = -(OMcp9_137*RLcp9_338-OMcp9_337*RLcp9_138);
    ORcp9_338 = OMcp9_137*RLcp9_238-OMcp9_237*RLcp9_138;
    OPcp9_138 = OPcp9_137+ROcp9_137*qdd[38]+qd[38]*(OMcp9_237*ROcp9_337-OMcp9_337*ROcp9_237);
    OPcp9_238 = OPcp9_237+ROcp9_237*qdd[38]-qd[38]*(OMcp9_137*ROcp9_337-OMcp9_337*ROcp9_137);
    OPcp9_338 = OPcp9_337+ROcp9_337*qdd[38]+qd[38]*(OMcp9_137*ROcp9_237-OMcp9_237*ROcp9_137);
    RLcp9_139 = ROcp9_738*s->dpt[3][29];
    RLcp9_239 = ROcp9_838*s->dpt[3][29];
    RLcp9_339 = ROcp9_938*s->dpt[3][29];
    OMcp9_139 = OMcp9_138+ROcp9_738*qd[39];
    OMcp9_239 = OMcp9_238+ROcp9_838*qd[39];
    OMcp9_339 = OMcp9_338+ROcp9_938*qd[39];
    ORcp9_139 = OMcp9_238*RLcp9_339-OMcp9_338*RLcp9_239;
    ORcp9_239 = -(OMcp9_138*RLcp9_339-OMcp9_338*RLcp9_139);
    ORcp9_339 = OMcp9_138*RLcp9_239-OMcp9_238*RLcp9_139;
    OPcp9_139 = OPcp9_138+ROcp9_738*qdd[39]+qd[39]*(OMcp9_238*ROcp9_938-OMcp9_338*ROcp9_838);
    OPcp9_239 = OPcp9_238+ROcp9_838*qdd[39]-qd[39]*(OMcp9_138*ROcp9_938-OMcp9_338*ROcp9_738);
    OPcp9_339 = OPcp9_338+ROcp9_938*qdd[39]+qd[39]*(OMcp9_138*ROcp9_838-OMcp9_238*ROcp9_738);
    RLcp9_140 = ROcp9_139*s->dpt[1][31]+ROcp9_738*s->dpt[3][31];
    RLcp9_240 = ROcp9_239*s->dpt[1][31]+ROcp9_838*s->dpt[3][31];
    RLcp9_340 = ROcp9_339*s->dpt[1][31]+ROcp9_938*s->dpt[3][31];
    OMcp9_140 = OMcp9_139+ROcp9_439*qd[40];
    OMcp9_240 = OMcp9_239+ROcp9_539*qd[40];
    OMcp9_340 = OMcp9_339+ROcp9_639*qd[40];
    ORcp9_140 = OMcp9_239*RLcp9_340-OMcp9_339*RLcp9_240;
    ORcp9_240 = -(OMcp9_139*RLcp9_340-OMcp9_339*RLcp9_140);
    ORcp9_340 = OMcp9_139*RLcp9_240-OMcp9_239*RLcp9_140;
    OPcp9_140 = OPcp9_139+ROcp9_439*qdd[40]+qd[40]*(OMcp9_239*ROcp9_639-OMcp9_339*ROcp9_539);
    OPcp9_240 = OPcp9_239+ROcp9_539*qdd[40]-qd[40]*(OMcp9_139*ROcp9_639-OMcp9_339*ROcp9_439);
    OPcp9_340 = OPcp9_339+ROcp9_639*qdd[40]+qd[40]*(OMcp9_139*ROcp9_539-OMcp9_239*ROcp9_439);
    RLcp9_165 = ROcp9_140*s->dpt[1][34]+ROcp9_439*s->dpt[2][34]+ROcp9_740*s->dpt[3][34];
    RLcp9_265 = ROcp9_240*s->dpt[1][34]+ROcp9_539*s->dpt[2][34]+ROcp9_840*s->dpt[3][34];
    RLcp9_365 = ROcp9_340*s->dpt[1][34]+ROcp9_639*s->dpt[2][34]+ROcp9_940*s->dpt[3][34];
    POcp9_165 = RLcp9_131+RLcp9_134+RLcp9_135+RLcp9_138+RLcp9_139+RLcp9_140+RLcp9_165+q[1];
    POcp9_265 = RLcp9_231+RLcp9_234+RLcp9_235+RLcp9_238+RLcp9_239+RLcp9_240+RLcp9_265+q[2];
    POcp9_365 = RLcp9_331+RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365+q[3];
    JTcp9_265_4 = -(RLcp9_331+RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365);
    JTcp9_365_4 = RLcp9_231+RLcp9_234+RLcp9_235+RLcp9_238+RLcp9_239+RLcp9_240+RLcp9_265;
    JTcp9_165_5 = C4*(RLcp9_331+RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340)-S4*(RLcp9_231+RLcp9_234)-S4*(RLcp9_235+
 RLcp9_238)-S4*(RLcp9_239+RLcp9_240)-RLcp9_265*S4+RLcp9_365*C4;
    JTcp9_265_5 = S4*(RLcp9_131+RLcp9_134+RLcp9_135+RLcp9_138+RLcp9_139+RLcp9_140+RLcp9_165);
    JTcp9_365_5 = -C4*(RLcp9_131+RLcp9_134+RLcp9_135+RLcp9_138+RLcp9_139+RLcp9_140+RLcp9_165);
    JTcp9_165_6 = ROcp9_85*(RLcp9_331+RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340)-ROcp9_95*(RLcp9_231+RLcp9_234)-
 ROcp9_95*(RLcp9_235+RLcp9_238)-ROcp9_95*(RLcp9_239+RLcp9_240)-RLcp9_265*ROcp9_95+RLcp9_365*ROcp9_85;
    JTcp9_265_6 = -(RLcp9_365*S5-ROcp9_95*(RLcp9_131+RLcp9_134+RLcp9_135+RLcp9_138+RLcp9_139+RLcp9_140+RLcp9_165)+S5*(
 RLcp9_331+RLcp9_334)+S5*(RLcp9_335+RLcp9_338)+S5*(RLcp9_339+RLcp9_340));
    JTcp9_365_6 = RLcp9_265*S5-ROcp9_85*(RLcp9_131+RLcp9_134+RLcp9_135+RLcp9_138+RLcp9_139+RLcp9_140+RLcp9_165)+S5*(
 RLcp9_231+RLcp9_234)+S5*(RLcp9_235+RLcp9_238)+S5*(RLcp9_239+RLcp9_240);
    JTcp9_165_7 = ROcp9_56*(RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_66*(RLcp9_234+RLcp9_235)-
 ROcp9_66*(RLcp9_238+RLcp9_239)-ROcp9_66*(RLcp9_240+RLcp9_265);
    JTcp9_265_7 = -(ROcp9_46*(RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_66*(RLcp9_134+RLcp9_135)-
 ROcp9_66*(RLcp9_138+RLcp9_139)-ROcp9_66*(RLcp9_140+RLcp9_165));
    JTcp9_365_7 = ROcp9_46*(RLcp9_234+RLcp9_235+RLcp9_238+RLcp9_239+RLcp9_240+RLcp9_265)-ROcp9_56*(RLcp9_134+RLcp9_135)-
 ROcp9_56*(RLcp9_138+RLcp9_139)-ROcp9_56*(RLcp9_140+RLcp9_165);
    JTcp9_165_8 = ROcp9_231*(RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_331*(RLcp9_234+RLcp9_235)-
 ROcp9_331*(RLcp9_238+RLcp9_239)-ROcp9_331*(RLcp9_240+RLcp9_265);
    JTcp9_265_8 = -(ROcp9_131*(RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_331*(RLcp9_134+RLcp9_135
 )-ROcp9_331*(RLcp9_138+RLcp9_139)-ROcp9_331*(RLcp9_140+RLcp9_165));
    JTcp9_365_8 = ROcp9_131*(RLcp9_234+RLcp9_235+RLcp9_238+RLcp9_239+RLcp9_240+RLcp9_265)-ROcp9_231*(RLcp9_134+RLcp9_135)-
 ROcp9_231*(RLcp9_138+RLcp9_139)-ROcp9_231*(RLcp9_140+RLcp9_165);
    JTcp9_165_9 = ROcp9_532*(RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_632*(RLcp9_234+RLcp9_235)-
 ROcp9_632*(RLcp9_238+RLcp9_239)-ROcp9_632*(RLcp9_240+RLcp9_265);
    JTcp9_265_9 = -(ROcp9_432*(RLcp9_334+RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_632*(RLcp9_134+RLcp9_135
 )-ROcp9_632*(RLcp9_138+RLcp9_139)-ROcp9_632*(RLcp9_140+RLcp9_165));
    JTcp9_365_9 = ROcp9_432*(RLcp9_234+RLcp9_235+RLcp9_238+RLcp9_239+RLcp9_240+RLcp9_265)-ROcp9_532*(RLcp9_134+RLcp9_135)-
 ROcp9_532*(RLcp9_138+RLcp9_139)-ROcp9_532*(RLcp9_140+RLcp9_165);
    JTcp9_165_10 = ROcp9_833*(RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340)-ROcp9_933*(RLcp9_235+RLcp9_238)-ROcp9_933*(
 RLcp9_239+RLcp9_240)-RLcp9_265*ROcp9_933+RLcp9_365*ROcp9_833;
    JTcp9_265_10 = RLcp9_165*ROcp9_933-RLcp9_365*ROcp9_733-ROcp9_733*(RLcp9_335+RLcp9_338+RLcp9_339+RLcp9_340)+ROcp9_933*(
 RLcp9_135+RLcp9_138)+ROcp9_933*(RLcp9_139+RLcp9_140);
    JTcp9_365_10 = ROcp9_733*(RLcp9_235+RLcp9_238+RLcp9_239+RLcp9_240)-ROcp9_833*(RLcp9_135+RLcp9_138)-ROcp9_833*(
 RLcp9_139+RLcp9_140)-RLcp9_165*ROcp9_833+RLcp9_265*ROcp9_733;
    JTcp9_165_11 = ROcp9_234*(RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_334*(RLcp9_238+RLcp9_239)-ROcp9_334*(
 RLcp9_240+RLcp9_265);
    JTcp9_265_11 = -(ROcp9_134*(RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_334*(RLcp9_138+RLcp9_139)-ROcp9_334*(
 RLcp9_140+RLcp9_165));
    JTcp9_365_11 = ROcp9_134*(RLcp9_238+RLcp9_239+RLcp9_240+RLcp9_265)-ROcp9_234*(RLcp9_138+RLcp9_139)-ROcp9_234*(
 RLcp9_140+RLcp9_165);
    JTcp9_165_12 = ROcp9_835*(RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_935*(RLcp9_238+RLcp9_239)-ROcp9_935*(
 RLcp9_240+RLcp9_265);
    JTcp9_265_12 = -(ROcp9_735*(RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_935*(RLcp9_138+RLcp9_139)-ROcp9_935*(
 RLcp9_140+RLcp9_165));
    JTcp9_365_12 = ROcp9_735*(RLcp9_238+RLcp9_239+RLcp9_240+RLcp9_265)-ROcp9_835*(RLcp9_138+RLcp9_139)-ROcp9_835*(
 RLcp9_140+RLcp9_165);
    JTcp9_165_13 = ROcp9_536*(RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_636*(RLcp9_238+RLcp9_239)-ROcp9_636*(
 RLcp9_240+RLcp9_265);
    JTcp9_265_13 = -(ROcp9_436*(RLcp9_338+RLcp9_339+RLcp9_340+RLcp9_365)-ROcp9_636*(RLcp9_138+RLcp9_139)-ROcp9_636*(
 RLcp9_140+RLcp9_165));
    JTcp9_365_13 = ROcp9_436*(RLcp9_238+RLcp9_239+RLcp9_240+RLcp9_265)-ROcp9_536*(RLcp9_138+RLcp9_139)-ROcp9_536*(
 RLcp9_140+RLcp9_165);
    JTcp9_165_14 = ROcp9_237*(RLcp9_339+RLcp9_340)-ROcp9_337*(RLcp9_239+RLcp9_240)-RLcp9_265*ROcp9_337+RLcp9_365*ROcp9_237;
    JTcp9_265_14 = RLcp9_165*ROcp9_337-RLcp9_365*ROcp9_137-ROcp9_137*(RLcp9_339+RLcp9_340)+ROcp9_337*(RLcp9_139+RLcp9_140);
    JTcp9_365_14 = ROcp9_137*(RLcp9_239+RLcp9_240)-ROcp9_237*(RLcp9_139+RLcp9_140)-RLcp9_165*ROcp9_237+RLcp9_265*ROcp9_137;
    JTcp9_165_15 = ROcp9_838*(RLcp9_340+RLcp9_365)-ROcp9_938*(RLcp9_240+RLcp9_265);
    JTcp9_265_15 = -(ROcp9_738*(RLcp9_340+RLcp9_365)-ROcp9_938*(RLcp9_140+RLcp9_165));
    JTcp9_365_15 = ROcp9_738*(RLcp9_240+RLcp9_265)-ROcp9_838*(RLcp9_140+RLcp9_165);
    JTcp9_165_16 = -(RLcp9_265*ROcp9_639-RLcp9_365*ROcp9_539);
    JTcp9_265_16 = RLcp9_165*ROcp9_639-RLcp9_365*ROcp9_439;
    JTcp9_365_16 = -(RLcp9_165*ROcp9_539-RLcp9_265*ROcp9_439);
    ORcp9_165 = OMcp9_240*RLcp9_365-OMcp9_340*RLcp9_265;
    ORcp9_265 = -(OMcp9_140*RLcp9_365-OMcp9_340*RLcp9_165);
    ORcp9_365 = OMcp9_140*RLcp9_265-OMcp9_240*RLcp9_165;
    VIcp9_165 = ORcp9_131+ORcp9_134+ORcp9_135+ORcp9_138+ORcp9_139+ORcp9_140+ORcp9_165+qd[1];
    VIcp9_265 = ORcp9_231+ORcp9_234+ORcp9_235+ORcp9_238+ORcp9_239+ORcp9_240+ORcp9_265+qd[2];
    VIcp9_365 = ORcp9_331+ORcp9_334+ORcp9_335+ORcp9_338+ORcp9_339+ORcp9_340+ORcp9_365+qd[3];
    ACcp9_165 = qdd[1]+OMcp9_233*ORcp9_334+OMcp9_234*ORcp9_335+OMcp9_237*ORcp9_338+OMcp9_238*ORcp9_339+OMcp9_239*ORcp9_340
 +OMcp9_240*ORcp9_365+OMcp9_26*ORcp9_331-OMcp9_333*ORcp9_234-OMcp9_334*ORcp9_235-OMcp9_337*ORcp9_238-OMcp9_338*ORcp9_239-
 OMcp9_339*ORcp9_240-OMcp9_340*ORcp9_265-OMcp9_36*ORcp9_231+OPcp9_233*RLcp9_334+OPcp9_234*RLcp9_335+OPcp9_237*RLcp9_338+
 OPcp9_238*RLcp9_339+OPcp9_239*RLcp9_340+OPcp9_240*RLcp9_365+OPcp9_26*RLcp9_331-OPcp9_333*RLcp9_234-OPcp9_334*RLcp9_235-
 OPcp9_337*RLcp9_238-OPcp9_338*RLcp9_239-OPcp9_339*RLcp9_240-OPcp9_340*RLcp9_265-OPcp9_36*RLcp9_231;
    ACcp9_265 = qdd[2]-OMcp9_133*ORcp9_334-OMcp9_134*ORcp9_335-OMcp9_137*ORcp9_338-OMcp9_138*ORcp9_339-OMcp9_139*ORcp9_340
 -OMcp9_140*ORcp9_365-OMcp9_16*ORcp9_331+OMcp9_333*ORcp9_134+OMcp9_334*ORcp9_135+OMcp9_337*ORcp9_138+OMcp9_338*ORcp9_139+
 OMcp9_339*ORcp9_140+OMcp9_340*ORcp9_165+OMcp9_36*ORcp9_131-OPcp9_133*RLcp9_334-OPcp9_134*RLcp9_335-OPcp9_137*RLcp9_338-
 OPcp9_138*RLcp9_339-OPcp9_139*RLcp9_340-OPcp9_140*RLcp9_365-OPcp9_16*RLcp9_331+OPcp9_333*RLcp9_134+OPcp9_334*RLcp9_135+
 OPcp9_337*RLcp9_138+OPcp9_338*RLcp9_139+OPcp9_339*RLcp9_140+OPcp9_340*RLcp9_165+OPcp9_36*RLcp9_131;
    ACcp9_365 = qdd[3]+OMcp9_133*ORcp9_234+OMcp9_134*ORcp9_235+OMcp9_137*ORcp9_238+OMcp9_138*ORcp9_239+OMcp9_139*ORcp9_240
 +OMcp9_140*ORcp9_265+OMcp9_16*ORcp9_231-OMcp9_233*ORcp9_134-OMcp9_234*ORcp9_135-OMcp9_237*ORcp9_138-OMcp9_238*ORcp9_139-
 OMcp9_239*ORcp9_140-OMcp9_240*ORcp9_165-OMcp9_26*ORcp9_131+OPcp9_133*RLcp9_234+OPcp9_134*RLcp9_235+OPcp9_137*RLcp9_238+
 OPcp9_138*RLcp9_239+OPcp9_139*RLcp9_240+OPcp9_140*RLcp9_265+OPcp9_16*RLcp9_231-OPcp9_233*RLcp9_134-OPcp9_234*RLcp9_135-
 OPcp9_237*RLcp9_138-OPcp9_238*RLcp9_139-OPcp9_239*RLcp9_140-OPcp9_240*RLcp9_165-OPcp9_26*RLcp9_131;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_165;
    sens->P[2] = POcp9_265;
    sens->P[3] = POcp9_365;
    sens->R[1][1] = ROcp9_140;
    sens->R[1][2] = ROcp9_240;
    sens->R[1][3] = ROcp9_340;
    sens->R[2][1] = ROcp9_439;
    sens->R[2][2] = ROcp9_539;
    sens->R[2][3] = ROcp9_639;
    sens->R[3][1] = ROcp9_740;
    sens->R[3][2] = ROcp9_840;
    sens->R[3][3] = ROcp9_940;
    sens->V[1] = VIcp9_165;
    sens->V[2] = VIcp9_265;
    sens->V[3] = VIcp9_365;
    sens->OM[1] = OMcp9_140;
    sens->OM[2] = OMcp9_240;
    sens->OM[3] = OMcp9_340;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp9_165_5;
    sens->J[1][6] = JTcp9_165_6;
    sens->J[1][31] = JTcp9_165_7;
    sens->J[1][32] = JTcp9_165_8;
    sens->J[1][33] = JTcp9_165_9;
    sens->J[1][34] = JTcp9_165_10;
    sens->J[1][35] = JTcp9_165_11;
    sens->J[1][36] = JTcp9_165_12;
    sens->J[1][37] = JTcp9_165_13;
    sens->J[1][38] = JTcp9_165_14;
    sens->J[1][39] = JTcp9_165_15;
    sens->J[1][40] = JTcp9_165_16;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp9_265_4;
    sens->J[2][5] = JTcp9_265_5;
    sens->J[2][6] = JTcp9_265_6;
    sens->J[2][31] = JTcp9_265_7;
    sens->J[2][32] = JTcp9_265_8;
    sens->J[2][33] = JTcp9_265_9;
    sens->J[2][34] = JTcp9_265_10;
    sens->J[2][35] = JTcp9_265_11;
    sens->J[2][36] = JTcp9_265_12;
    sens->J[2][37] = JTcp9_265_13;
    sens->J[2][38] = JTcp9_265_14;
    sens->J[2][39] = JTcp9_265_15;
    sens->J[2][40] = JTcp9_265_16;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp9_365_4;
    sens->J[3][5] = JTcp9_365_5;
    sens->J[3][6] = JTcp9_365_6;
    sens->J[3][31] = JTcp9_365_7;
    sens->J[3][32] = JTcp9_365_8;
    sens->J[3][33] = JTcp9_365_9;
    sens->J[3][34] = JTcp9_365_10;
    sens->J[3][35] = JTcp9_365_11;
    sens->J[3][36] = JTcp9_365_12;
    sens->J[3][37] = JTcp9_365_13;
    sens->J[3][38] = JTcp9_365_14;
    sens->J[3][39] = JTcp9_365_15;
    sens->J[3][40] = JTcp9_365_16;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp9_46;
    sens->J[4][32] = ROcp9_131;
    sens->J[4][33] = ROcp9_432;
    sens->J[4][34] = ROcp9_733;
    sens->J[4][35] = ROcp9_134;
    sens->J[4][36] = ROcp9_735;
    sens->J[4][37] = ROcp9_436;
    sens->J[4][38] = ROcp9_137;
    sens->J[4][39] = ROcp9_738;
    sens->J[4][40] = ROcp9_439;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp9_85;
    sens->J[5][31] = ROcp9_56;
    sens->J[5][32] = ROcp9_231;
    sens->J[5][33] = ROcp9_532;
    sens->J[5][34] = ROcp9_833;
    sens->J[5][35] = ROcp9_234;
    sens->J[5][36] = ROcp9_835;
    sens->J[5][37] = ROcp9_536;
    sens->J[5][38] = ROcp9_237;
    sens->J[5][39] = ROcp9_838;
    sens->J[5][40] = ROcp9_539;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp9_95;
    sens->J[6][31] = ROcp9_66;
    sens->J[6][32] = ROcp9_331;
    sens->J[6][33] = ROcp9_632;
    sens->J[6][34] = ROcp9_933;
    sens->J[6][35] = ROcp9_334;
    sens->J[6][36] = ROcp9_935;
    sens->J[6][37] = ROcp9_636;
    sens->J[6][38] = ROcp9_337;
    sens->J[6][39] = ROcp9_938;
    sens->J[6][40] = ROcp9_639;
    sens->A[1] = ACcp9_165;
    sens->A[2] = ACcp9_265;
    sens->A[3] = ACcp9_365;
    sens->OMP[1] = OPcp9_140;
    sens->OMP[2] = OPcp9_240;
    sens->OMP[3] = OPcp9_340;
 
// 
break;
case 11:
 


// = = Block_1_0_0_11_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp10_25 = qd[5]*C4;
    OMcp10_35 = qd[5]*S4;
    OMcp10_16 = qd[4]+qd[6]*S5;
    OMcp10_26 = OMcp10_25+ROcp10_85*qd[6];
    OMcp10_36 = OMcp10_35+ROcp10_95*qd[6];
    OPcp10_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp10_26 = ROcp10_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp10_35*S5-ROcp10_95*qd[4]);
    OPcp10_36 = ROcp10_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp10_25*S5-ROcp10_85*qd[4]);

// = = Block_1_0_0_11_0_4 = = 
 
// Sensor Kinematics 


    ROcp10_131 = ROcp10_16*C31-S31*S5;
    ROcp10_231 = ROcp10_26*C31-ROcp10_85*S31;
    ROcp10_331 = ROcp10_36*C31-ROcp10_95*S31;
    ROcp10_731 = ROcp10_16*S31+C31*S5;
    ROcp10_831 = ROcp10_26*S31+ROcp10_85*C31;
    ROcp10_931 = ROcp10_36*S31+ROcp10_95*C31;
    ROcp10_432 = ROcp10_46*C32+ROcp10_731*S32;
    ROcp10_532 = ROcp10_56*C32+ROcp10_831*S32;
    ROcp10_632 = ROcp10_66*C32+ROcp10_931*S32;
    ROcp10_732 = -(ROcp10_46*S32-ROcp10_731*C32);
    ROcp10_832 = -(ROcp10_56*S32-ROcp10_831*C32);
    ROcp10_932 = -(ROcp10_66*S32-ROcp10_931*C32);
    ROcp10_133 = ROcp10_131*C33-ROcp10_732*S33;
    ROcp10_233 = ROcp10_231*C33-ROcp10_832*S33;
    ROcp10_333 = ROcp10_331*C33-ROcp10_932*S33;
    ROcp10_733 = ROcp10_131*S33+ROcp10_732*C33;
    ROcp10_833 = ROcp10_231*S33+ROcp10_832*C33;
    ROcp10_933 = ROcp10_331*S33+ROcp10_932*C33;
    ROcp10_134 = ROcp10_133*C34+ROcp10_432*S34;
    ROcp10_234 = ROcp10_233*C34+ROcp10_532*S34;
    ROcp10_334 = ROcp10_333*C34+ROcp10_632*S34;
    ROcp10_434 = -(ROcp10_133*S34-ROcp10_432*C34);
    ROcp10_534 = -(ROcp10_233*S34-ROcp10_532*C34);
    ROcp10_634 = -(ROcp10_333*S34-ROcp10_632*C34);
    RLcp10_131 = ROcp10_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp10_231 = ROcp10_26*s->dpt[1][3]+ROcp10_85*s->dpt[3][3];
    RLcp10_331 = ROcp10_36*s->dpt[1][3]+ROcp10_95*s->dpt[3][3];
    OMcp10_131 = OMcp10_16+ROcp10_46*qd[31];
    OMcp10_231 = OMcp10_26+ROcp10_56*qd[31];
    OMcp10_331 = OMcp10_36+ROcp10_66*qd[31];
    ORcp10_131 = OMcp10_26*RLcp10_331-OMcp10_36*RLcp10_231;
    ORcp10_231 = -(OMcp10_16*RLcp10_331-OMcp10_36*RLcp10_131);
    ORcp10_331 = OMcp10_16*RLcp10_231-OMcp10_26*RLcp10_131;
    OMcp10_132 = OMcp10_131+ROcp10_131*qd[32];
    OMcp10_232 = OMcp10_231+ROcp10_231*qd[32];
    OMcp10_332 = OMcp10_331+ROcp10_331*qd[32];
    OMcp10_133 = OMcp10_132+ROcp10_432*qd[33];
    OMcp10_233 = OMcp10_232+ROcp10_532*qd[33];
    OMcp10_333 = OMcp10_332+ROcp10_632*qd[33];
    OPcp10_133 = OPcp10_16+ROcp10_131*qdd[32]+ROcp10_432*qdd[33]+ROcp10_46*qdd[31]+qd[31]*(OMcp10_26*ROcp10_66-OMcp10_36*
 ROcp10_56)+qd[32]*(OMcp10_231*ROcp10_331-OMcp10_331*ROcp10_231)+qd[33]*(OMcp10_232*ROcp10_632-OMcp10_332*ROcp10_532);
    OPcp10_233 = OPcp10_26+ROcp10_231*qdd[32]+ROcp10_532*qdd[33]+ROcp10_56*qdd[31]-qd[31]*(OMcp10_16*ROcp10_66-OMcp10_36*
 ROcp10_46)-qd[32]*(OMcp10_131*ROcp10_331-OMcp10_331*ROcp10_131)-qd[33]*(OMcp10_132*ROcp10_632-OMcp10_332*ROcp10_432);
    OPcp10_333 = OPcp10_36+ROcp10_331*qdd[32]+ROcp10_632*qdd[33]+ROcp10_66*qdd[31]+qd[31]*(OMcp10_16*ROcp10_56-OMcp10_26*
 ROcp10_46)+qd[32]*(OMcp10_131*ROcp10_231-OMcp10_231*ROcp10_131)+qd[33]*(OMcp10_132*ROcp10_532-OMcp10_232*ROcp10_432);
    RLcp10_134 = ROcp10_733*s->dpt[3][21];
    RLcp10_234 = ROcp10_833*s->dpt[3][21];
    RLcp10_334 = ROcp10_933*s->dpt[3][21];
    OMcp10_134 = OMcp10_133+ROcp10_733*qd[34];
    OMcp10_234 = OMcp10_233+ROcp10_833*qd[34];
    OMcp10_334 = OMcp10_333+ROcp10_933*qd[34];
    ORcp10_134 = OMcp10_233*RLcp10_334-OMcp10_333*RLcp10_234;
    ORcp10_234 = -(OMcp10_133*RLcp10_334-OMcp10_333*RLcp10_134);
    ORcp10_334 = OMcp10_133*RLcp10_234-OMcp10_233*RLcp10_134;
    OPcp10_134 = OPcp10_133+ROcp10_733*qdd[34]+qd[34]*(OMcp10_233*ROcp10_933-OMcp10_333*ROcp10_833);
    OPcp10_234 = OPcp10_233+ROcp10_833*qdd[34]-qd[34]*(OMcp10_133*ROcp10_933-OMcp10_333*ROcp10_733);
    OPcp10_334 = OPcp10_333+ROcp10_933*qdd[34]+qd[34]*(OMcp10_133*ROcp10_833-OMcp10_233*ROcp10_733);

// = = Block_1_0_0_11_0_5 = = 
 
// Sensor Kinematics 


    ROcp10_435 = ROcp10_434*C35+ROcp10_733*S35;
    ROcp10_535 = ROcp10_534*C35+ROcp10_833*S35;
    ROcp10_635 = ROcp10_634*C35+ROcp10_933*S35;
    ROcp10_735 = -(ROcp10_434*S35-ROcp10_733*C35);
    ROcp10_835 = -(ROcp10_534*S35-ROcp10_833*C35);
    ROcp10_935 = -(ROcp10_634*S35-ROcp10_933*C35);
    ROcp10_136 = ROcp10_134*C36+ROcp10_435*S36;
    ROcp10_236 = ROcp10_234*C36+ROcp10_535*S36;
    ROcp10_336 = ROcp10_334*C36+ROcp10_635*S36;
    ROcp10_436 = -(ROcp10_134*S36-ROcp10_435*C36);
    ROcp10_536 = -(ROcp10_234*S36-ROcp10_535*C36);
    ROcp10_636 = -(ROcp10_334*S36-ROcp10_635*C36);
    ROcp10_137 = ROcp10_136*C37-ROcp10_735*S37;
    ROcp10_237 = ROcp10_236*C37-ROcp10_835*S37;
    ROcp10_337 = ROcp10_336*C37-ROcp10_935*S37;
    ROcp10_737 = ROcp10_136*S37+ROcp10_735*C37;
    ROcp10_837 = ROcp10_236*S37+ROcp10_835*C37;
    ROcp10_937 = ROcp10_336*S37+ROcp10_935*C37;
    ROcp10_438 = ROcp10_436*C38+ROcp10_737*S38;
    ROcp10_538 = ROcp10_536*C38+ROcp10_837*S38;
    ROcp10_638 = ROcp10_636*C38+ROcp10_937*S38;
    ROcp10_738 = -(ROcp10_436*S38-ROcp10_737*C38);
    ROcp10_838 = -(ROcp10_536*S38-ROcp10_837*C38);
    ROcp10_938 = -(ROcp10_636*S38-ROcp10_937*C38);
    ROcp10_139 = ROcp10_137*C39+ROcp10_438*S39;
    ROcp10_239 = ROcp10_237*C39+ROcp10_538*S39;
    ROcp10_339 = ROcp10_337*C39+ROcp10_638*S39;
    ROcp10_439 = -(ROcp10_137*S39-ROcp10_438*C39);
    ROcp10_539 = -(ROcp10_237*S39-ROcp10_538*C39);
    ROcp10_639 = -(ROcp10_337*S39-ROcp10_638*C39);
    ROcp10_140 = ROcp10_139*C40-ROcp10_738*S40;
    ROcp10_240 = ROcp10_239*C40-ROcp10_838*S40;
    ROcp10_340 = ROcp10_339*C40-ROcp10_938*S40;
    ROcp10_740 = ROcp10_139*S40+ROcp10_738*C40;
    ROcp10_840 = ROcp10_239*S40+ROcp10_838*C40;
    ROcp10_940 = ROcp10_339*S40+ROcp10_938*C40;
    ROcp10_141 = ROcp10_140*C41+ROcp10_439*S41;
    ROcp10_241 = ROcp10_240*C41+ROcp10_539*S41;
    ROcp10_341 = ROcp10_340*C41+ROcp10_639*S41;
    ROcp10_441 = -(ROcp10_140*S41-ROcp10_439*C41);
    ROcp10_541 = -(ROcp10_240*S41-ROcp10_539*C41);
    ROcp10_641 = -(ROcp10_340*S41-ROcp10_639*C41);
    RLcp10_135 = ROcp10_134*s->dpt[1][23]+ROcp10_434*s->dpt[2][23]+ROcp10_733*s->dpt[3][23];
    RLcp10_235 = ROcp10_234*s->dpt[1][23]+ROcp10_534*s->dpt[2][23]+ROcp10_833*s->dpt[3][23];
    RLcp10_335 = ROcp10_334*s->dpt[1][23]+ROcp10_634*s->dpt[2][23]+ROcp10_933*s->dpt[3][23];
    ORcp10_135 = OMcp10_234*RLcp10_335-OMcp10_334*RLcp10_235;
    ORcp10_235 = -(OMcp10_134*RLcp10_335-OMcp10_334*RLcp10_135);
    ORcp10_335 = OMcp10_134*RLcp10_235-OMcp10_234*RLcp10_135;
    OMcp10_137 = OMcp10_134+ROcp10_436*qd[37];
    OMcp10_237 = OMcp10_234+ROcp10_536*qd[37];
    OMcp10_337 = OMcp10_334+ROcp10_636*qd[37];
    OPcp10_137 = OPcp10_134+ROcp10_436*qdd[37]+qd[37]*(OMcp10_234*ROcp10_636-OMcp10_334*ROcp10_536);
    OPcp10_237 = OPcp10_234+ROcp10_536*qdd[37]-qd[37]*(OMcp10_134*ROcp10_636-OMcp10_334*ROcp10_436);
    OPcp10_337 = OPcp10_334+ROcp10_636*qdd[37]+qd[37]*(OMcp10_134*ROcp10_536-OMcp10_234*ROcp10_436);
    RLcp10_138 = ROcp10_436*s->dpt[2][27]+ROcp10_737*s->dpt[3][27];
    RLcp10_238 = ROcp10_536*s->dpt[2][27]+ROcp10_837*s->dpt[3][27];
    RLcp10_338 = ROcp10_636*s->dpt[2][27]+ROcp10_937*s->dpt[3][27];
    OMcp10_138 = OMcp10_137+ROcp10_137*qd[38];
    OMcp10_238 = OMcp10_237+ROcp10_237*qd[38];
    OMcp10_338 = OMcp10_337+ROcp10_337*qd[38];
    ORcp10_138 = OMcp10_237*RLcp10_338-OMcp10_337*RLcp10_238;
    ORcp10_238 = -(OMcp10_137*RLcp10_338-OMcp10_337*RLcp10_138);
    ORcp10_338 = OMcp10_137*RLcp10_238-OMcp10_237*RLcp10_138;
    OPcp10_138 = OPcp10_137+ROcp10_137*qdd[38]+qd[38]*(OMcp10_237*ROcp10_337-OMcp10_337*ROcp10_237);
    OPcp10_238 = OPcp10_237+ROcp10_237*qdd[38]-qd[38]*(OMcp10_137*ROcp10_337-OMcp10_337*ROcp10_137);
    OPcp10_338 = OPcp10_337+ROcp10_337*qdd[38]+qd[38]*(OMcp10_137*ROcp10_237-OMcp10_237*ROcp10_137);
    RLcp10_139 = ROcp10_738*s->dpt[3][29];
    RLcp10_239 = ROcp10_838*s->dpt[3][29];
    RLcp10_339 = ROcp10_938*s->dpt[3][29];
    OMcp10_139 = OMcp10_138+ROcp10_738*qd[39];
    OMcp10_239 = OMcp10_238+ROcp10_838*qd[39];
    OMcp10_339 = OMcp10_338+ROcp10_938*qd[39];
    ORcp10_139 = OMcp10_238*RLcp10_339-OMcp10_338*RLcp10_239;
    ORcp10_239 = -(OMcp10_138*RLcp10_339-OMcp10_338*RLcp10_139);
    ORcp10_339 = OMcp10_138*RLcp10_239-OMcp10_238*RLcp10_139;
    OPcp10_139 = OPcp10_138+ROcp10_738*qdd[39]+qd[39]*(OMcp10_238*ROcp10_938-OMcp10_338*ROcp10_838);
    OPcp10_239 = OPcp10_238+ROcp10_838*qdd[39]-qd[39]*(OMcp10_138*ROcp10_938-OMcp10_338*ROcp10_738);
    OPcp10_339 = OPcp10_338+ROcp10_938*qdd[39]+qd[39]*(OMcp10_138*ROcp10_838-OMcp10_238*ROcp10_738);
    RLcp10_140 = ROcp10_139*s->dpt[1][31]+ROcp10_738*s->dpt[3][31];
    RLcp10_240 = ROcp10_239*s->dpt[1][31]+ROcp10_838*s->dpt[3][31];
    RLcp10_340 = ROcp10_339*s->dpt[1][31]+ROcp10_938*s->dpt[3][31];
    OMcp10_140 = OMcp10_139+ROcp10_439*qd[40];
    OMcp10_240 = OMcp10_239+ROcp10_539*qd[40];
    OMcp10_340 = OMcp10_339+ROcp10_639*qd[40];
    ORcp10_140 = OMcp10_239*RLcp10_340-OMcp10_339*RLcp10_240;
    ORcp10_240 = -(OMcp10_139*RLcp10_340-OMcp10_339*RLcp10_140);
    ORcp10_340 = OMcp10_139*RLcp10_240-OMcp10_239*RLcp10_140;
    OPcp10_140 = OPcp10_139+ROcp10_439*qdd[40]+qd[40]*(OMcp10_239*ROcp10_639-OMcp10_339*ROcp10_539);
    OPcp10_240 = OPcp10_239+ROcp10_539*qdd[40]-qd[40]*(OMcp10_139*ROcp10_639-OMcp10_339*ROcp10_439);
    OPcp10_340 = OPcp10_339+ROcp10_639*qdd[40]+qd[40]*(OMcp10_139*ROcp10_539-OMcp10_239*ROcp10_439);
    RLcp10_141 = ROcp10_140*s->dpt[1][33]+ROcp10_740*s->dpt[3][33];
    RLcp10_241 = ROcp10_240*s->dpt[1][33]+ROcp10_840*s->dpt[3][33];
    RLcp10_341 = ROcp10_340*s->dpt[1][33]+ROcp10_940*s->dpt[3][33];
    OMcp10_141 = OMcp10_140+ROcp10_740*qd[41];
    OMcp10_241 = OMcp10_240+ROcp10_840*qd[41];
    OMcp10_341 = OMcp10_340+ROcp10_940*qd[41];
    ORcp10_141 = OMcp10_240*RLcp10_341-OMcp10_340*RLcp10_241;
    ORcp10_241 = -(OMcp10_140*RLcp10_341-OMcp10_340*RLcp10_141);
    ORcp10_341 = OMcp10_140*RLcp10_241-OMcp10_240*RLcp10_141;
    OPcp10_141 = OPcp10_140+ROcp10_740*qdd[41]+qd[41]*(OMcp10_240*ROcp10_940-OMcp10_340*ROcp10_840);
    OPcp10_241 = OPcp10_240+ROcp10_840*qdd[41]-qd[41]*(OMcp10_140*ROcp10_940-OMcp10_340*ROcp10_740);
    OPcp10_341 = OPcp10_340+ROcp10_940*qdd[41]+qd[41]*(OMcp10_140*ROcp10_840-OMcp10_240*ROcp10_740);
    RLcp10_166 = ROcp10_141*s->dpt[1][36]+ROcp10_441*s->dpt[2][36]+ROcp10_740*s->dpt[3][36];
    RLcp10_266 = ROcp10_241*s->dpt[1][36]+ROcp10_541*s->dpt[2][36]+ROcp10_840*s->dpt[3][36];
    RLcp10_366 = ROcp10_341*s->dpt[1][36]+ROcp10_641*s->dpt[2][36]+ROcp10_940*s->dpt[3][36];
    POcp10_166 = RLcp10_131+RLcp10_134+RLcp10_135+RLcp10_138+RLcp10_139+RLcp10_140+RLcp10_141+RLcp10_166+q[1];
    POcp10_266 = RLcp10_231+RLcp10_234+RLcp10_235+RLcp10_238+RLcp10_239+RLcp10_240+RLcp10_241+RLcp10_266+q[2];
    POcp10_366 = RLcp10_331+RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341+RLcp10_366+q[3];
    JTcp10_266_4 = -(RLcp10_331+RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341+RLcp10_366);
    JTcp10_366_4 = RLcp10_231+RLcp10_234+RLcp10_235+RLcp10_238+RLcp10_239+RLcp10_240+RLcp10_241+RLcp10_266;
    JTcp10_166_5 = C4*(RLcp10_331+RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341+RLcp10_366)-S4*(
 RLcp10_231+RLcp10_234)-S4*(RLcp10_235+RLcp10_238)-S4*(RLcp10_239+RLcp10_240)-S4*(RLcp10_241+RLcp10_266);
    JTcp10_266_5 = S4*(RLcp10_131+RLcp10_134+RLcp10_135+RLcp10_138+RLcp10_139+RLcp10_140+RLcp10_141+RLcp10_166);
    JTcp10_366_5 = -C4*(RLcp10_131+RLcp10_134+RLcp10_135+RLcp10_138+RLcp10_139+RLcp10_140+RLcp10_141+RLcp10_166);
    JTcp10_166_6 = ROcp10_85*(RLcp10_331+RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341+RLcp10_366)-
 ROcp10_95*(RLcp10_231+RLcp10_234)-ROcp10_95*(RLcp10_235+RLcp10_238)-ROcp10_95*(RLcp10_239+RLcp10_240)-ROcp10_95*(RLcp10_241+
 RLcp10_266);
    JTcp10_266_6 = RLcp10_166*ROcp10_95-RLcp10_341*S5-RLcp10_366*S5+ROcp10_95*(RLcp10_131+RLcp10_134+RLcp10_135+RLcp10_138
 +RLcp10_139+RLcp10_140+RLcp10_141)-S5*(RLcp10_331+RLcp10_334)-S5*(RLcp10_335+RLcp10_338)-S5*(RLcp10_339+RLcp10_340);
    JTcp10_366_6 = RLcp10_241*S5-ROcp10_85*(RLcp10_131+RLcp10_134+RLcp10_135+RLcp10_138+RLcp10_139+RLcp10_140+RLcp10_141)+
 S5*(RLcp10_231+RLcp10_234)+S5*(RLcp10_235+RLcp10_238)+S5*(RLcp10_239+RLcp10_240)-RLcp10_166*ROcp10_85+RLcp10_266*S5;
    JTcp10_166_7 = ROcp10_56*(RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341)-ROcp10_66*(RLcp10_234+
 RLcp10_235)-ROcp10_66*(RLcp10_238+RLcp10_239)-ROcp10_66*(RLcp10_240+RLcp10_241)-RLcp10_266*ROcp10_66+RLcp10_366*ROcp10_56;
    JTcp10_266_7 = RLcp10_166*ROcp10_66-RLcp10_366*ROcp10_46-ROcp10_46*(RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+
 RLcp10_340+RLcp10_341)+ROcp10_66*(RLcp10_134+RLcp10_135)+ROcp10_66*(RLcp10_138+RLcp10_139)+ROcp10_66*(RLcp10_140+RLcp10_141);
    JTcp10_366_7 = ROcp10_46*(RLcp10_234+RLcp10_235+RLcp10_238+RLcp10_239+RLcp10_240+RLcp10_241)-ROcp10_56*(RLcp10_134+
 RLcp10_135)-ROcp10_56*(RLcp10_138+RLcp10_139)-ROcp10_56*(RLcp10_140+RLcp10_141)-RLcp10_166*ROcp10_56+RLcp10_266*ROcp10_46;
    JTcp10_166_8 = ROcp10_231*(RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341)-ROcp10_331*(RLcp10_234+
 RLcp10_235)-ROcp10_331*(RLcp10_238+RLcp10_239)-ROcp10_331*(RLcp10_240+RLcp10_241)-RLcp10_266*ROcp10_331+RLcp10_366*
 ROcp10_231;
    JTcp10_266_8 = RLcp10_166*ROcp10_331-RLcp10_366*ROcp10_131-ROcp10_131*(RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+
 RLcp10_340+RLcp10_341)+ROcp10_331*(RLcp10_134+RLcp10_135)+ROcp10_331*(RLcp10_138+RLcp10_139)+ROcp10_331*(RLcp10_140+
 RLcp10_141);
    JTcp10_366_8 = ROcp10_131*(RLcp10_234+RLcp10_235+RLcp10_238+RLcp10_239+RLcp10_240+RLcp10_241)-ROcp10_231*(RLcp10_134+
 RLcp10_135)-ROcp10_231*(RLcp10_138+RLcp10_139)-ROcp10_231*(RLcp10_140+RLcp10_141)-RLcp10_166*ROcp10_231+RLcp10_266*
 ROcp10_131;
    JTcp10_166_9 = ROcp10_532*(RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341)-ROcp10_632*(RLcp10_234+
 RLcp10_235)-ROcp10_632*(RLcp10_238+RLcp10_239)-ROcp10_632*(RLcp10_240+RLcp10_241)-RLcp10_266*ROcp10_632+RLcp10_366*
 ROcp10_532;
    JTcp10_266_9 = RLcp10_166*ROcp10_632-RLcp10_366*ROcp10_432-ROcp10_432*(RLcp10_334+RLcp10_335+RLcp10_338+RLcp10_339+
 RLcp10_340+RLcp10_341)+ROcp10_632*(RLcp10_134+RLcp10_135)+ROcp10_632*(RLcp10_138+RLcp10_139)+ROcp10_632*(RLcp10_140+
 RLcp10_141);
    JTcp10_366_9 = ROcp10_432*(RLcp10_234+RLcp10_235+RLcp10_238+RLcp10_239+RLcp10_240+RLcp10_241)-ROcp10_532*(RLcp10_134+
 RLcp10_135)-ROcp10_532*(RLcp10_138+RLcp10_139)-ROcp10_532*(RLcp10_140+RLcp10_141)-RLcp10_166*ROcp10_532+RLcp10_266*
 ROcp10_432;
    JTcp10_166_10 = ROcp10_833*(RLcp10_335+RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341+RLcp10_366)-ROcp10_933*(RLcp10_235+
 RLcp10_238)-ROcp10_933*(RLcp10_239+RLcp10_240)-ROcp10_933*(RLcp10_241+RLcp10_266);
    JTcp10_266_10 = -(ROcp10_733*(RLcp10_335+RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341+RLcp10_366)-ROcp10_933*(
 RLcp10_135+RLcp10_138)-ROcp10_933*(RLcp10_139+RLcp10_140)-ROcp10_933*(RLcp10_141+RLcp10_166));
    JTcp10_366_10 = ROcp10_733*(RLcp10_235+RLcp10_238+RLcp10_239+RLcp10_240+RLcp10_241+RLcp10_266)-ROcp10_833*(RLcp10_135+
 RLcp10_138)-ROcp10_833*(RLcp10_139+RLcp10_140)-ROcp10_833*(RLcp10_141+RLcp10_166);
    JTcp10_166_11 = ROcp10_234*(RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341)-ROcp10_334*(RLcp10_238+RLcp10_239)-ROcp10_334
 *(RLcp10_240+RLcp10_241)-RLcp10_266*ROcp10_334+RLcp10_366*ROcp10_234;
    JTcp10_266_11 = RLcp10_166*ROcp10_334-RLcp10_366*ROcp10_134-ROcp10_134*(RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341)+
 ROcp10_334*(RLcp10_138+RLcp10_139)+ROcp10_334*(RLcp10_140+RLcp10_141);
    JTcp10_366_11 = ROcp10_134*(RLcp10_238+RLcp10_239+RLcp10_240+RLcp10_241)-ROcp10_234*(RLcp10_138+RLcp10_139)-ROcp10_234
 *(RLcp10_140+RLcp10_141)-RLcp10_166*ROcp10_234+RLcp10_266*ROcp10_134;
    JTcp10_166_12 = ROcp10_835*(RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341)-ROcp10_935*(RLcp10_238+RLcp10_239)-ROcp10_935
 *(RLcp10_240+RLcp10_241)-RLcp10_266*ROcp10_935+RLcp10_366*ROcp10_835;
    JTcp10_266_12 = RLcp10_166*ROcp10_935-RLcp10_366*ROcp10_735-ROcp10_735*(RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341)+
 ROcp10_935*(RLcp10_138+RLcp10_139)+ROcp10_935*(RLcp10_140+RLcp10_141);
    JTcp10_366_12 = ROcp10_735*(RLcp10_238+RLcp10_239+RLcp10_240+RLcp10_241)-ROcp10_835*(RLcp10_138+RLcp10_139)-ROcp10_835
 *(RLcp10_140+RLcp10_141)-RLcp10_166*ROcp10_835+RLcp10_266*ROcp10_735;
    JTcp10_166_13 = ROcp10_536*(RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341)-ROcp10_636*(RLcp10_238+RLcp10_239)-ROcp10_636
 *(RLcp10_240+RLcp10_241)-RLcp10_266*ROcp10_636+RLcp10_366*ROcp10_536;
    JTcp10_266_13 = RLcp10_166*ROcp10_636-RLcp10_366*ROcp10_436-ROcp10_436*(RLcp10_338+RLcp10_339+RLcp10_340+RLcp10_341)+
 ROcp10_636*(RLcp10_138+RLcp10_139)+ROcp10_636*(RLcp10_140+RLcp10_141);
    JTcp10_366_13 = ROcp10_436*(RLcp10_238+RLcp10_239+RLcp10_240+RLcp10_241)-ROcp10_536*(RLcp10_138+RLcp10_139)-ROcp10_536
 *(RLcp10_140+RLcp10_141)-RLcp10_166*ROcp10_536+RLcp10_266*ROcp10_436;
    JTcp10_166_14 = ROcp10_237*(RLcp10_339+RLcp10_340+RLcp10_341+RLcp10_366)-ROcp10_337*(RLcp10_239+RLcp10_240)-ROcp10_337
 *(RLcp10_241+RLcp10_266);
    JTcp10_266_14 = -(ROcp10_137*(RLcp10_339+RLcp10_340+RLcp10_341+RLcp10_366)-ROcp10_337*(RLcp10_139+RLcp10_140)-
 ROcp10_337*(RLcp10_141+RLcp10_166));
    JTcp10_366_14 = ROcp10_137*(RLcp10_239+RLcp10_240+RLcp10_241+RLcp10_266)-ROcp10_237*(RLcp10_139+RLcp10_140)-ROcp10_237
 *(RLcp10_141+RLcp10_166);
    JTcp10_166_15 = ROcp10_838*(RLcp10_340+RLcp10_341)-ROcp10_938*(RLcp10_240+RLcp10_241)-RLcp10_266*ROcp10_938+RLcp10_366
 *ROcp10_838;
    JTcp10_266_15 = RLcp10_166*ROcp10_938-RLcp10_366*ROcp10_738-ROcp10_738*(RLcp10_340+RLcp10_341)+ROcp10_938*(RLcp10_140+
 RLcp10_141);
    JTcp10_366_15 = ROcp10_738*(RLcp10_240+RLcp10_241)-ROcp10_838*(RLcp10_140+RLcp10_141)-RLcp10_166*ROcp10_838+RLcp10_266
 *ROcp10_738;
    JTcp10_166_16 = ROcp10_539*(RLcp10_341+RLcp10_366)-ROcp10_639*(RLcp10_241+RLcp10_266);
    JTcp10_266_16 = -(ROcp10_439*(RLcp10_341+RLcp10_366)-ROcp10_639*(RLcp10_141+RLcp10_166));
    JTcp10_366_16 = ROcp10_439*(RLcp10_241+RLcp10_266)-ROcp10_539*(RLcp10_141+RLcp10_166);
    JTcp10_166_17 = -(RLcp10_266*ROcp10_940-RLcp10_366*ROcp10_840);
    JTcp10_266_17 = RLcp10_166*ROcp10_940-RLcp10_366*ROcp10_740;
    JTcp10_366_17 = -(RLcp10_166*ROcp10_840-RLcp10_266*ROcp10_740);
    ORcp10_166 = OMcp10_241*RLcp10_366-OMcp10_341*RLcp10_266;
    ORcp10_266 = -(OMcp10_141*RLcp10_366-OMcp10_341*RLcp10_166);
    ORcp10_366 = OMcp10_141*RLcp10_266-OMcp10_241*RLcp10_166;
    VIcp10_166 = ORcp10_131+ORcp10_134+ORcp10_135+ORcp10_138+ORcp10_139+ORcp10_140+ORcp10_141+ORcp10_166+qd[1];
    VIcp10_266 = ORcp10_231+ORcp10_234+ORcp10_235+ORcp10_238+ORcp10_239+ORcp10_240+ORcp10_241+ORcp10_266+qd[2];
    VIcp10_366 = ORcp10_331+ORcp10_334+ORcp10_335+ORcp10_338+ORcp10_339+ORcp10_340+ORcp10_341+ORcp10_366+qd[3];
    ACcp10_166 = qdd[1]+OMcp10_233*ORcp10_334+OMcp10_234*ORcp10_335+OMcp10_237*ORcp10_338+OMcp10_238*ORcp10_339+OMcp10_239
 *ORcp10_340+OMcp10_240*ORcp10_341+OMcp10_241*ORcp10_366+OMcp10_26*ORcp10_331-OMcp10_333*ORcp10_234-OMcp10_334*ORcp10_235-
 OMcp10_337*ORcp10_238-OMcp10_338*ORcp10_239-OMcp10_339*ORcp10_240-OMcp10_340*ORcp10_241-OMcp10_341*ORcp10_266-OMcp10_36*
 ORcp10_231+OPcp10_233*RLcp10_334+OPcp10_234*RLcp10_335+OPcp10_237*RLcp10_338+OPcp10_238*RLcp10_339+OPcp10_239*RLcp10_340+
 OPcp10_240*RLcp10_341+OPcp10_241*RLcp10_366+OPcp10_26*RLcp10_331-OPcp10_333*RLcp10_234-OPcp10_334*RLcp10_235-OPcp10_337*
 RLcp10_238-OPcp10_338*RLcp10_239-OPcp10_339*RLcp10_240-OPcp10_340*RLcp10_241-OPcp10_341*RLcp10_266-OPcp10_36*RLcp10_231;
    ACcp10_266 = qdd[2]-OMcp10_133*ORcp10_334-OMcp10_134*ORcp10_335-OMcp10_137*ORcp10_338-OMcp10_138*ORcp10_339-OMcp10_139
 *ORcp10_340-OMcp10_140*ORcp10_341-OMcp10_141*ORcp10_366-OMcp10_16*ORcp10_331+OMcp10_333*ORcp10_134+OMcp10_334*ORcp10_135+
 OMcp10_337*ORcp10_138+OMcp10_338*ORcp10_139+OMcp10_339*ORcp10_140+OMcp10_340*ORcp10_141+OMcp10_341*ORcp10_166+OMcp10_36*
 ORcp10_131-OPcp10_133*RLcp10_334-OPcp10_134*RLcp10_335-OPcp10_137*RLcp10_338-OPcp10_138*RLcp10_339-OPcp10_139*RLcp10_340-
 OPcp10_140*RLcp10_341-OPcp10_141*RLcp10_366-OPcp10_16*RLcp10_331+OPcp10_333*RLcp10_134+OPcp10_334*RLcp10_135+OPcp10_337*
 RLcp10_138+OPcp10_338*RLcp10_139+OPcp10_339*RLcp10_140+OPcp10_340*RLcp10_141+OPcp10_341*RLcp10_166+OPcp10_36*RLcp10_131;
    ACcp10_366 = qdd[3]+OMcp10_133*ORcp10_234+OMcp10_134*ORcp10_235+OMcp10_137*ORcp10_238+OMcp10_138*ORcp10_239+OMcp10_139
 *ORcp10_240+OMcp10_140*ORcp10_241+OMcp10_141*ORcp10_266+OMcp10_16*ORcp10_231-OMcp10_233*ORcp10_134-OMcp10_234*ORcp10_135-
 OMcp10_237*ORcp10_138-OMcp10_238*ORcp10_139-OMcp10_239*ORcp10_140-OMcp10_240*ORcp10_141-OMcp10_241*ORcp10_166-OMcp10_26*
 ORcp10_131+OPcp10_133*RLcp10_234+OPcp10_134*RLcp10_235+OPcp10_137*RLcp10_238+OPcp10_138*RLcp10_239+OPcp10_139*RLcp10_240+
 OPcp10_140*RLcp10_241+OPcp10_141*RLcp10_266+OPcp10_16*RLcp10_231-OPcp10_233*RLcp10_134-OPcp10_234*RLcp10_135-OPcp10_237*
 RLcp10_138-OPcp10_238*RLcp10_139-OPcp10_239*RLcp10_140-OPcp10_240*RLcp10_141-OPcp10_241*RLcp10_166-OPcp10_26*RLcp10_131;

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_166;
    sens->P[2] = POcp10_266;
    sens->P[3] = POcp10_366;
    sens->R[1][1] = ROcp10_141;
    sens->R[1][2] = ROcp10_241;
    sens->R[1][3] = ROcp10_341;
    sens->R[2][1] = ROcp10_441;
    sens->R[2][2] = ROcp10_541;
    sens->R[2][3] = ROcp10_641;
    sens->R[3][1] = ROcp10_740;
    sens->R[3][2] = ROcp10_840;
    sens->R[3][3] = ROcp10_940;
    sens->V[1] = VIcp10_166;
    sens->V[2] = VIcp10_266;
    sens->V[3] = VIcp10_366;
    sens->OM[1] = OMcp10_141;
    sens->OM[2] = OMcp10_241;
    sens->OM[3] = OMcp10_341;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp10_166_5;
    sens->J[1][6] = JTcp10_166_6;
    sens->J[1][31] = JTcp10_166_7;
    sens->J[1][32] = JTcp10_166_8;
    sens->J[1][33] = JTcp10_166_9;
    sens->J[1][34] = JTcp10_166_10;
    sens->J[1][35] = JTcp10_166_11;
    sens->J[1][36] = JTcp10_166_12;
    sens->J[1][37] = JTcp10_166_13;
    sens->J[1][38] = JTcp10_166_14;
    sens->J[1][39] = JTcp10_166_15;
    sens->J[1][40] = JTcp10_166_16;
    sens->J[1][41] = JTcp10_166_17;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp10_266_4;
    sens->J[2][5] = JTcp10_266_5;
    sens->J[2][6] = JTcp10_266_6;
    sens->J[2][31] = JTcp10_266_7;
    sens->J[2][32] = JTcp10_266_8;
    sens->J[2][33] = JTcp10_266_9;
    sens->J[2][34] = JTcp10_266_10;
    sens->J[2][35] = JTcp10_266_11;
    sens->J[2][36] = JTcp10_266_12;
    sens->J[2][37] = JTcp10_266_13;
    sens->J[2][38] = JTcp10_266_14;
    sens->J[2][39] = JTcp10_266_15;
    sens->J[2][40] = JTcp10_266_16;
    sens->J[2][41] = JTcp10_266_17;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp10_366_4;
    sens->J[3][5] = JTcp10_366_5;
    sens->J[3][6] = JTcp10_366_6;
    sens->J[3][31] = JTcp10_366_7;
    sens->J[3][32] = JTcp10_366_8;
    sens->J[3][33] = JTcp10_366_9;
    sens->J[3][34] = JTcp10_366_10;
    sens->J[3][35] = JTcp10_366_11;
    sens->J[3][36] = JTcp10_366_12;
    sens->J[3][37] = JTcp10_366_13;
    sens->J[3][38] = JTcp10_366_14;
    sens->J[3][39] = JTcp10_366_15;
    sens->J[3][40] = JTcp10_366_16;
    sens->J[3][41] = JTcp10_366_17;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp10_46;
    sens->J[4][32] = ROcp10_131;
    sens->J[4][33] = ROcp10_432;
    sens->J[4][34] = ROcp10_733;
    sens->J[4][35] = ROcp10_134;
    sens->J[4][36] = ROcp10_735;
    sens->J[4][37] = ROcp10_436;
    sens->J[4][38] = ROcp10_137;
    sens->J[4][39] = ROcp10_738;
    sens->J[4][40] = ROcp10_439;
    sens->J[4][41] = ROcp10_740;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp10_85;
    sens->J[5][31] = ROcp10_56;
    sens->J[5][32] = ROcp10_231;
    sens->J[5][33] = ROcp10_532;
    sens->J[5][34] = ROcp10_833;
    sens->J[5][35] = ROcp10_234;
    sens->J[5][36] = ROcp10_835;
    sens->J[5][37] = ROcp10_536;
    sens->J[5][38] = ROcp10_237;
    sens->J[5][39] = ROcp10_838;
    sens->J[5][40] = ROcp10_539;
    sens->J[5][41] = ROcp10_840;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp10_95;
    sens->J[6][31] = ROcp10_66;
    sens->J[6][32] = ROcp10_331;
    sens->J[6][33] = ROcp10_632;
    sens->J[6][34] = ROcp10_933;
    sens->J[6][35] = ROcp10_334;
    sens->J[6][36] = ROcp10_935;
    sens->J[6][37] = ROcp10_636;
    sens->J[6][38] = ROcp10_337;
    sens->J[6][39] = ROcp10_938;
    sens->J[6][40] = ROcp10_639;
    sens->J[6][41] = ROcp10_940;
    sens->A[1] = ACcp10_166;
    sens->A[2] = ACcp10_266;
    sens->A[3] = ACcp10_366;
    sens->OMP[1] = OPcp10_141;
    sens->OMP[2] = OPcp10_241;
    sens->OMP[3] = OPcp10_341;
 
// 
break;
case 12:
 


// = = Block_1_0_0_12_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp11_25 = qd[5]*C4;
    OMcp11_35 = qd[5]*S4;
    OMcp11_16 = qd[4]+qd[6]*S5;
    OMcp11_26 = OMcp11_25+ROcp11_85*qd[6];
    OMcp11_36 = OMcp11_35+ROcp11_95*qd[6];
    OPcp11_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp11_26 = ROcp11_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp11_35*S5-ROcp11_95*qd[4]);
    OPcp11_36 = ROcp11_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp11_25*S5-ROcp11_85*qd[4]);

// = = Block_1_0_0_12_0_4 = = 
 
// Sensor Kinematics 


    ROcp11_131 = ROcp11_16*C31-S31*S5;
    ROcp11_231 = ROcp11_26*C31-ROcp11_85*S31;
    ROcp11_331 = ROcp11_36*C31-ROcp11_95*S31;
    ROcp11_731 = ROcp11_16*S31+C31*S5;
    ROcp11_831 = ROcp11_26*S31+ROcp11_85*C31;
    ROcp11_931 = ROcp11_36*S31+ROcp11_95*C31;
    ROcp11_432 = ROcp11_46*C32+ROcp11_731*S32;
    ROcp11_532 = ROcp11_56*C32+ROcp11_831*S32;
    ROcp11_632 = ROcp11_66*C32+ROcp11_931*S32;
    ROcp11_732 = -(ROcp11_46*S32-ROcp11_731*C32);
    ROcp11_832 = -(ROcp11_56*S32-ROcp11_831*C32);
    ROcp11_932 = -(ROcp11_66*S32-ROcp11_931*C32);
    ROcp11_133 = ROcp11_131*C33-ROcp11_732*S33;
    ROcp11_233 = ROcp11_231*C33-ROcp11_832*S33;
    ROcp11_333 = ROcp11_331*C33-ROcp11_932*S33;
    ROcp11_733 = ROcp11_131*S33+ROcp11_732*C33;
    ROcp11_833 = ROcp11_231*S33+ROcp11_832*C33;
    ROcp11_933 = ROcp11_331*S33+ROcp11_932*C33;
    ROcp11_134 = ROcp11_133*C34+ROcp11_432*S34;
    ROcp11_234 = ROcp11_233*C34+ROcp11_532*S34;
    ROcp11_334 = ROcp11_333*C34+ROcp11_632*S34;
    ROcp11_434 = -(ROcp11_133*S34-ROcp11_432*C34);
    ROcp11_534 = -(ROcp11_233*S34-ROcp11_532*C34);
    ROcp11_634 = -(ROcp11_333*S34-ROcp11_632*C34);
    RLcp11_131 = ROcp11_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp11_231 = ROcp11_26*s->dpt[1][3]+ROcp11_85*s->dpt[3][3];
    RLcp11_331 = ROcp11_36*s->dpt[1][3]+ROcp11_95*s->dpt[3][3];
    OMcp11_131 = OMcp11_16+ROcp11_46*qd[31];
    OMcp11_231 = OMcp11_26+ROcp11_56*qd[31];
    OMcp11_331 = OMcp11_36+ROcp11_66*qd[31];
    ORcp11_131 = OMcp11_26*RLcp11_331-OMcp11_36*RLcp11_231;
    ORcp11_231 = -(OMcp11_16*RLcp11_331-OMcp11_36*RLcp11_131);
    ORcp11_331 = OMcp11_16*RLcp11_231-OMcp11_26*RLcp11_131;
    OMcp11_132 = OMcp11_131+ROcp11_131*qd[32];
    OMcp11_232 = OMcp11_231+ROcp11_231*qd[32];
    OMcp11_332 = OMcp11_331+ROcp11_331*qd[32];
    OMcp11_133 = OMcp11_132+ROcp11_432*qd[33];
    OMcp11_233 = OMcp11_232+ROcp11_532*qd[33];
    OMcp11_333 = OMcp11_332+ROcp11_632*qd[33];
    OPcp11_133 = OPcp11_16+ROcp11_131*qdd[32]+ROcp11_432*qdd[33]+ROcp11_46*qdd[31]+qd[31]*(OMcp11_26*ROcp11_66-OMcp11_36*
 ROcp11_56)+qd[32]*(OMcp11_231*ROcp11_331-OMcp11_331*ROcp11_231)+qd[33]*(OMcp11_232*ROcp11_632-OMcp11_332*ROcp11_532);
    OPcp11_233 = OPcp11_26+ROcp11_231*qdd[32]+ROcp11_532*qdd[33]+ROcp11_56*qdd[31]-qd[31]*(OMcp11_16*ROcp11_66-OMcp11_36*
 ROcp11_46)-qd[32]*(OMcp11_131*ROcp11_331-OMcp11_331*ROcp11_131)-qd[33]*(OMcp11_132*ROcp11_632-OMcp11_332*ROcp11_432);
    OPcp11_333 = OPcp11_36+ROcp11_331*qdd[32]+ROcp11_632*qdd[33]+ROcp11_66*qdd[31]+qd[31]*(OMcp11_16*ROcp11_56-OMcp11_26*
 ROcp11_46)+qd[32]*(OMcp11_131*ROcp11_231-OMcp11_231*ROcp11_131)+qd[33]*(OMcp11_132*ROcp11_532-OMcp11_232*ROcp11_432);
    RLcp11_134 = ROcp11_733*s->dpt[3][21];
    RLcp11_234 = ROcp11_833*s->dpt[3][21];
    RLcp11_334 = ROcp11_933*s->dpt[3][21];
    OMcp11_134 = OMcp11_133+ROcp11_733*qd[34];
    OMcp11_234 = OMcp11_233+ROcp11_833*qd[34];
    OMcp11_334 = OMcp11_333+ROcp11_933*qd[34];
    ORcp11_134 = OMcp11_233*RLcp11_334-OMcp11_333*RLcp11_234;
    ORcp11_234 = -(OMcp11_133*RLcp11_334-OMcp11_333*RLcp11_134);
    ORcp11_334 = OMcp11_133*RLcp11_234-OMcp11_233*RLcp11_134;
    OPcp11_134 = OPcp11_133+ROcp11_733*qdd[34]+qd[34]*(OMcp11_233*ROcp11_933-OMcp11_333*ROcp11_833);
    OPcp11_234 = OPcp11_233+ROcp11_833*qdd[34]-qd[34]*(OMcp11_133*ROcp11_933-OMcp11_333*ROcp11_733);
    OPcp11_334 = OPcp11_333+ROcp11_933*qdd[34]+qd[34]*(OMcp11_133*ROcp11_833-OMcp11_233*ROcp11_733);

// = = Block_1_0_0_12_0_5 = = 
 
// Sensor Kinematics 


    ROcp11_435 = ROcp11_434*C35+ROcp11_733*S35;
    ROcp11_535 = ROcp11_534*C35+ROcp11_833*S35;
    ROcp11_635 = ROcp11_634*C35+ROcp11_933*S35;
    ROcp11_735 = -(ROcp11_434*S35-ROcp11_733*C35);
    ROcp11_835 = -(ROcp11_534*S35-ROcp11_833*C35);
    ROcp11_935 = -(ROcp11_634*S35-ROcp11_933*C35);
    ROcp11_136 = ROcp11_134*C36+ROcp11_435*S36;
    ROcp11_236 = ROcp11_234*C36+ROcp11_535*S36;
    ROcp11_336 = ROcp11_334*C36+ROcp11_635*S36;
    ROcp11_436 = -(ROcp11_134*S36-ROcp11_435*C36);
    ROcp11_536 = -(ROcp11_234*S36-ROcp11_535*C36);
    ROcp11_636 = -(ROcp11_334*S36-ROcp11_635*C36);
    ROcp11_137 = ROcp11_136*C37-ROcp11_735*S37;
    ROcp11_237 = ROcp11_236*C37-ROcp11_835*S37;
    ROcp11_337 = ROcp11_336*C37-ROcp11_935*S37;
    ROcp11_737 = ROcp11_136*S37+ROcp11_735*C37;
    ROcp11_837 = ROcp11_236*S37+ROcp11_835*C37;
    ROcp11_937 = ROcp11_336*S37+ROcp11_935*C37;
    ROcp11_438 = ROcp11_436*C38+ROcp11_737*S38;
    ROcp11_538 = ROcp11_536*C38+ROcp11_837*S38;
    ROcp11_638 = ROcp11_636*C38+ROcp11_937*S38;
    ROcp11_738 = -(ROcp11_436*S38-ROcp11_737*C38);
    ROcp11_838 = -(ROcp11_536*S38-ROcp11_837*C38);
    ROcp11_938 = -(ROcp11_636*S38-ROcp11_937*C38);
    ROcp11_139 = ROcp11_137*C39+ROcp11_438*S39;
    ROcp11_239 = ROcp11_237*C39+ROcp11_538*S39;
    ROcp11_339 = ROcp11_337*C39+ROcp11_638*S39;
    ROcp11_439 = -(ROcp11_137*S39-ROcp11_438*C39);
    ROcp11_539 = -(ROcp11_237*S39-ROcp11_538*C39);
    ROcp11_639 = -(ROcp11_337*S39-ROcp11_638*C39);
    ROcp11_140 = ROcp11_139*C40-ROcp11_738*S40;
    ROcp11_240 = ROcp11_239*C40-ROcp11_838*S40;
    ROcp11_340 = ROcp11_339*C40-ROcp11_938*S40;
    ROcp11_740 = ROcp11_139*S40+ROcp11_738*C40;
    ROcp11_840 = ROcp11_239*S40+ROcp11_838*C40;
    ROcp11_940 = ROcp11_339*S40+ROcp11_938*C40;
    ROcp11_141 = ROcp11_140*C41+ROcp11_439*S41;
    ROcp11_241 = ROcp11_240*C41+ROcp11_539*S41;
    ROcp11_341 = ROcp11_340*C41+ROcp11_639*S41;
    ROcp11_441 = -(ROcp11_140*S41-ROcp11_439*C41);
    ROcp11_541 = -(ROcp11_240*S41-ROcp11_539*C41);
    ROcp11_641 = -(ROcp11_340*S41-ROcp11_639*C41);
    ROcp11_142 = ROcp11_141*C42-ROcp11_740*S42;
    ROcp11_242 = ROcp11_241*C42-ROcp11_840*S42;
    ROcp11_342 = ROcp11_341*C42-ROcp11_940*S42;
    ROcp11_742 = ROcp11_141*S42+ROcp11_740*C42;
    ROcp11_842 = ROcp11_241*S42+ROcp11_840*C42;
    ROcp11_942 = ROcp11_341*S42+ROcp11_940*C42;
    RLcp11_135 = ROcp11_134*s->dpt[1][23]+ROcp11_434*s->dpt[2][23]+ROcp11_733*s->dpt[3][23];
    RLcp11_235 = ROcp11_234*s->dpt[1][23]+ROcp11_534*s->dpt[2][23]+ROcp11_833*s->dpt[3][23];
    RLcp11_335 = ROcp11_334*s->dpt[1][23]+ROcp11_634*s->dpt[2][23]+ROcp11_933*s->dpt[3][23];
    ORcp11_135 = OMcp11_234*RLcp11_335-OMcp11_334*RLcp11_235;
    ORcp11_235 = -(OMcp11_134*RLcp11_335-OMcp11_334*RLcp11_135);
    ORcp11_335 = OMcp11_134*RLcp11_235-OMcp11_234*RLcp11_135;
    OMcp11_137 = OMcp11_134+ROcp11_436*qd[37];
    OMcp11_237 = OMcp11_234+ROcp11_536*qd[37];
    OMcp11_337 = OMcp11_334+ROcp11_636*qd[37];
    OPcp11_137 = OPcp11_134+ROcp11_436*qdd[37]+qd[37]*(OMcp11_234*ROcp11_636-OMcp11_334*ROcp11_536);
    OPcp11_237 = OPcp11_234+ROcp11_536*qdd[37]-qd[37]*(OMcp11_134*ROcp11_636-OMcp11_334*ROcp11_436);
    OPcp11_337 = OPcp11_334+ROcp11_636*qdd[37]+qd[37]*(OMcp11_134*ROcp11_536-OMcp11_234*ROcp11_436);
    RLcp11_138 = ROcp11_436*s->dpt[2][27]+ROcp11_737*s->dpt[3][27];
    RLcp11_238 = ROcp11_536*s->dpt[2][27]+ROcp11_837*s->dpt[3][27];
    RLcp11_338 = ROcp11_636*s->dpt[2][27]+ROcp11_937*s->dpt[3][27];
    OMcp11_138 = OMcp11_137+ROcp11_137*qd[38];
    OMcp11_238 = OMcp11_237+ROcp11_237*qd[38];
    OMcp11_338 = OMcp11_337+ROcp11_337*qd[38];
    ORcp11_138 = OMcp11_237*RLcp11_338-OMcp11_337*RLcp11_238;
    ORcp11_238 = -(OMcp11_137*RLcp11_338-OMcp11_337*RLcp11_138);
    ORcp11_338 = OMcp11_137*RLcp11_238-OMcp11_237*RLcp11_138;
    OPcp11_138 = OPcp11_137+ROcp11_137*qdd[38]+qd[38]*(OMcp11_237*ROcp11_337-OMcp11_337*ROcp11_237);
    OPcp11_238 = OPcp11_237+ROcp11_237*qdd[38]-qd[38]*(OMcp11_137*ROcp11_337-OMcp11_337*ROcp11_137);
    OPcp11_338 = OPcp11_337+ROcp11_337*qdd[38]+qd[38]*(OMcp11_137*ROcp11_237-OMcp11_237*ROcp11_137);
    RLcp11_139 = ROcp11_738*s->dpt[3][29];
    RLcp11_239 = ROcp11_838*s->dpt[3][29];
    RLcp11_339 = ROcp11_938*s->dpt[3][29];
    OMcp11_139 = OMcp11_138+ROcp11_738*qd[39];
    OMcp11_239 = OMcp11_238+ROcp11_838*qd[39];
    OMcp11_339 = OMcp11_338+ROcp11_938*qd[39];
    ORcp11_139 = OMcp11_238*RLcp11_339-OMcp11_338*RLcp11_239;
    ORcp11_239 = -(OMcp11_138*RLcp11_339-OMcp11_338*RLcp11_139);
    ORcp11_339 = OMcp11_138*RLcp11_239-OMcp11_238*RLcp11_139;
    OPcp11_139 = OPcp11_138+ROcp11_738*qdd[39]+qd[39]*(OMcp11_238*ROcp11_938-OMcp11_338*ROcp11_838);
    OPcp11_239 = OPcp11_238+ROcp11_838*qdd[39]-qd[39]*(OMcp11_138*ROcp11_938-OMcp11_338*ROcp11_738);
    OPcp11_339 = OPcp11_338+ROcp11_938*qdd[39]+qd[39]*(OMcp11_138*ROcp11_838-OMcp11_238*ROcp11_738);
    RLcp11_140 = ROcp11_139*s->dpt[1][31]+ROcp11_738*s->dpt[3][31];
    RLcp11_240 = ROcp11_239*s->dpt[1][31]+ROcp11_838*s->dpt[3][31];
    RLcp11_340 = ROcp11_339*s->dpt[1][31]+ROcp11_938*s->dpt[3][31];
    OMcp11_140 = OMcp11_139+ROcp11_439*qd[40];
    OMcp11_240 = OMcp11_239+ROcp11_539*qd[40];
    OMcp11_340 = OMcp11_339+ROcp11_639*qd[40];
    ORcp11_140 = OMcp11_239*RLcp11_340-OMcp11_339*RLcp11_240;
    ORcp11_240 = -(OMcp11_139*RLcp11_340-OMcp11_339*RLcp11_140);
    ORcp11_340 = OMcp11_139*RLcp11_240-OMcp11_239*RLcp11_140;
    OPcp11_140 = OPcp11_139+ROcp11_439*qdd[40]+qd[40]*(OMcp11_239*ROcp11_639-OMcp11_339*ROcp11_539);
    OPcp11_240 = OPcp11_239+ROcp11_539*qdd[40]-qd[40]*(OMcp11_139*ROcp11_639-OMcp11_339*ROcp11_439);
    OPcp11_340 = OPcp11_339+ROcp11_639*qdd[40]+qd[40]*(OMcp11_139*ROcp11_539-OMcp11_239*ROcp11_439);
    RLcp11_141 = ROcp11_140*s->dpt[1][33]+ROcp11_740*s->dpt[3][33];
    RLcp11_241 = ROcp11_240*s->dpt[1][33]+ROcp11_840*s->dpt[3][33];
    RLcp11_341 = ROcp11_340*s->dpt[1][33]+ROcp11_940*s->dpt[3][33];
    OMcp11_141 = OMcp11_140+ROcp11_740*qd[41];
    OMcp11_241 = OMcp11_240+ROcp11_840*qd[41];
    OMcp11_341 = OMcp11_340+ROcp11_940*qd[41];
    ORcp11_141 = OMcp11_240*RLcp11_341-OMcp11_340*RLcp11_241;
    ORcp11_241 = -(OMcp11_140*RLcp11_341-OMcp11_340*RLcp11_141);
    ORcp11_341 = OMcp11_140*RLcp11_241-OMcp11_240*RLcp11_141;
    OMcp11_142 = OMcp11_141+ROcp11_441*qd[42];
    OMcp11_242 = OMcp11_241+ROcp11_541*qd[42];
    OMcp11_342 = OMcp11_341+ROcp11_641*qd[42];
    OPcp11_142 = OPcp11_140+ROcp11_441*qdd[42]+ROcp11_740*qdd[41]+qd[41]*(OMcp11_240*ROcp11_940-OMcp11_340*ROcp11_840)+
 qd[42]*(OMcp11_241*ROcp11_641-OMcp11_341*ROcp11_541);
    OPcp11_242 = OPcp11_240+ROcp11_541*qdd[42]+ROcp11_840*qdd[41]-qd[41]*(OMcp11_140*ROcp11_940-OMcp11_340*ROcp11_740)-
 qd[42]*(OMcp11_141*ROcp11_641-OMcp11_341*ROcp11_441);
    OPcp11_342 = OPcp11_340+ROcp11_641*qdd[42]+ROcp11_940*qdd[41]+qd[41]*(OMcp11_140*ROcp11_840-OMcp11_240*ROcp11_740)+
 qd[42]*(OMcp11_141*ROcp11_541-OMcp11_241*ROcp11_441);
    RLcp11_167 = ROcp11_142*s->dpt[1][38]+ROcp11_441*s->dpt[2][38]+ROcp11_742*s->dpt[3][38];
    RLcp11_267 = ROcp11_242*s->dpt[1][38]+ROcp11_541*s->dpt[2][38]+ROcp11_842*s->dpt[3][38];
    RLcp11_367 = ROcp11_342*s->dpt[1][38]+ROcp11_641*s->dpt[2][38]+ROcp11_942*s->dpt[3][38];
    POcp11_167 = RLcp11_131+RLcp11_134+RLcp11_135+RLcp11_138+RLcp11_139+RLcp11_140+RLcp11_141+RLcp11_167+q[1];
    POcp11_267 = RLcp11_231+RLcp11_234+RLcp11_235+RLcp11_238+RLcp11_239+RLcp11_240+RLcp11_241+RLcp11_267+q[2];
    POcp11_367 = RLcp11_331+RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341+RLcp11_367+q[3];
    JTcp11_267_4 = -(RLcp11_331+RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341+RLcp11_367);
    JTcp11_367_4 = RLcp11_231+RLcp11_234+RLcp11_235+RLcp11_238+RLcp11_239+RLcp11_240+RLcp11_241+RLcp11_267;
    JTcp11_167_5 = C4*(RLcp11_331+RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341+RLcp11_367)-S4*(
 RLcp11_231+RLcp11_234)-S4*(RLcp11_235+RLcp11_238)-S4*(RLcp11_239+RLcp11_240)-S4*(RLcp11_241+RLcp11_267);
    JTcp11_267_5 = S4*(RLcp11_131+RLcp11_134+RLcp11_135+RLcp11_138+RLcp11_139+RLcp11_140+RLcp11_141+RLcp11_167);
    JTcp11_367_5 = -C4*(RLcp11_131+RLcp11_134+RLcp11_135+RLcp11_138+RLcp11_139+RLcp11_140+RLcp11_141+RLcp11_167);
    JTcp11_167_6 = ROcp11_85*(RLcp11_331+RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341+RLcp11_367)-
 ROcp11_95*(RLcp11_231+RLcp11_234)-ROcp11_95*(RLcp11_235+RLcp11_238)-ROcp11_95*(RLcp11_239+RLcp11_240)-ROcp11_95*(RLcp11_241+
 RLcp11_267);
    JTcp11_267_6 = RLcp11_167*ROcp11_95-RLcp11_341*S5-RLcp11_367*S5+ROcp11_95*(RLcp11_131+RLcp11_134+RLcp11_135+RLcp11_138
 +RLcp11_139+RLcp11_140+RLcp11_141)-S5*(RLcp11_331+RLcp11_334)-S5*(RLcp11_335+RLcp11_338)-S5*(RLcp11_339+RLcp11_340);
    JTcp11_367_6 = RLcp11_241*S5-ROcp11_85*(RLcp11_131+RLcp11_134+RLcp11_135+RLcp11_138+RLcp11_139+RLcp11_140+RLcp11_141)+
 S5*(RLcp11_231+RLcp11_234)+S5*(RLcp11_235+RLcp11_238)+S5*(RLcp11_239+RLcp11_240)-RLcp11_167*ROcp11_85+RLcp11_267*S5;
    JTcp11_167_7 = ROcp11_56*(RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341)-ROcp11_66*(RLcp11_234+
 RLcp11_235)-ROcp11_66*(RLcp11_238+RLcp11_239)-ROcp11_66*(RLcp11_240+RLcp11_241)-RLcp11_267*ROcp11_66+RLcp11_367*ROcp11_56;
    JTcp11_267_7 = RLcp11_167*ROcp11_66-RLcp11_367*ROcp11_46-ROcp11_46*(RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+
 RLcp11_340+RLcp11_341)+ROcp11_66*(RLcp11_134+RLcp11_135)+ROcp11_66*(RLcp11_138+RLcp11_139)+ROcp11_66*(RLcp11_140+RLcp11_141);
    JTcp11_367_7 = ROcp11_46*(RLcp11_234+RLcp11_235+RLcp11_238+RLcp11_239+RLcp11_240+RLcp11_241)-ROcp11_56*(RLcp11_134+
 RLcp11_135)-ROcp11_56*(RLcp11_138+RLcp11_139)-ROcp11_56*(RLcp11_140+RLcp11_141)-RLcp11_167*ROcp11_56+RLcp11_267*ROcp11_46;
    JTcp11_167_8 = ROcp11_231*(RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341)-ROcp11_331*(RLcp11_234+
 RLcp11_235)-ROcp11_331*(RLcp11_238+RLcp11_239)-ROcp11_331*(RLcp11_240+RLcp11_241)-RLcp11_267*ROcp11_331+RLcp11_367*
 ROcp11_231;
    JTcp11_267_8 = RLcp11_167*ROcp11_331-RLcp11_367*ROcp11_131-ROcp11_131*(RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+
 RLcp11_340+RLcp11_341)+ROcp11_331*(RLcp11_134+RLcp11_135)+ROcp11_331*(RLcp11_138+RLcp11_139)+ROcp11_331*(RLcp11_140+
 RLcp11_141);
    JTcp11_367_8 = ROcp11_131*(RLcp11_234+RLcp11_235+RLcp11_238+RLcp11_239+RLcp11_240+RLcp11_241)-ROcp11_231*(RLcp11_134+
 RLcp11_135)-ROcp11_231*(RLcp11_138+RLcp11_139)-ROcp11_231*(RLcp11_140+RLcp11_141)-RLcp11_167*ROcp11_231+RLcp11_267*
 ROcp11_131;
    JTcp11_167_9 = ROcp11_532*(RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341)-ROcp11_632*(RLcp11_234+
 RLcp11_235)-ROcp11_632*(RLcp11_238+RLcp11_239)-ROcp11_632*(RLcp11_240+RLcp11_241)-RLcp11_267*ROcp11_632+RLcp11_367*
 ROcp11_532;
    JTcp11_267_9 = RLcp11_167*ROcp11_632-RLcp11_367*ROcp11_432-ROcp11_432*(RLcp11_334+RLcp11_335+RLcp11_338+RLcp11_339+
 RLcp11_340+RLcp11_341)+ROcp11_632*(RLcp11_134+RLcp11_135)+ROcp11_632*(RLcp11_138+RLcp11_139)+ROcp11_632*(RLcp11_140+
 RLcp11_141);
    JTcp11_367_9 = ROcp11_432*(RLcp11_234+RLcp11_235+RLcp11_238+RLcp11_239+RLcp11_240+RLcp11_241)-ROcp11_532*(RLcp11_134+
 RLcp11_135)-ROcp11_532*(RLcp11_138+RLcp11_139)-ROcp11_532*(RLcp11_140+RLcp11_141)-RLcp11_167*ROcp11_532+RLcp11_267*
 ROcp11_432;
    JTcp11_167_10 = ROcp11_833*(RLcp11_335+RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341+RLcp11_367)-ROcp11_933*(RLcp11_235+
 RLcp11_238)-ROcp11_933*(RLcp11_239+RLcp11_240)-ROcp11_933*(RLcp11_241+RLcp11_267);
    JTcp11_267_10 = -(ROcp11_733*(RLcp11_335+RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341+RLcp11_367)-ROcp11_933*(
 RLcp11_135+RLcp11_138)-ROcp11_933*(RLcp11_139+RLcp11_140)-ROcp11_933*(RLcp11_141+RLcp11_167));
    JTcp11_367_10 = ROcp11_733*(RLcp11_235+RLcp11_238+RLcp11_239+RLcp11_240+RLcp11_241+RLcp11_267)-ROcp11_833*(RLcp11_135+
 RLcp11_138)-ROcp11_833*(RLcp11_139+RLcp11_140)-ROcp11_833*(RLcp11_141+RLcp11_167);
    JTcp11_167_11 = ROcp11_234*(RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341)-ROcp11_334*(RLcp11_238+RLcp11_239)-ROcp11_334
 *(RLcp11_240+RLcp11_241)-RLcp11_267*ROcp11_334+RLcp11_367*ROcp11_234;
    JTcp11_267_11 = RLcp11_167*ROcp11_334-RLcp11_367*ROcp11_134-ROcp11_134*(RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341)+
 ROcp11_334*(RLcp11_138+RLcp11_139)+ROcp11_334*(RLcp11_140+RLcp11_141);
    JTcp11_367_11 = ROcp11_134*(RLcp11_238+RLcp11_239+RLcp11_240+RLcp11_241)-ROcp11_234*(RLcp11_138+RLcp11_139)-ROcp11_234
 *(RLcp11_140+RLcp11_141)-RLcp11_167*ROcp11_234+RLcp11_267*ROcp11_134;
    JTcp11_167_12 = ROcp11_835*(RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341)-ROcp11_935*(RLcp11_238+RLcp11_239)-ROcp11_935
 *(RLcp11_240+RLcp11_241)-RLcp11_267*ROcp11_935+RLcp11_367*ROcp11_835;
    JTcp11_267_12 = RLcp11_167*ROcp11_935-RLcp11_367*ROcp11_735-ROcp11_735*(RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341)+
 ROcp11_935*(RLcp11_138+RLcp11_139)+ROcp11_935*(RLcp11_140+RLcp11_141);
    JTcp11_367_12 = ROcp11_735*(RLcp11_238+RLcp11_239+RLcp11_240+RLcp11_241)-ROcp11_835*(RLcp11_138+RLcp11_139)-ROcp11_835
 *(RLcp11_140+RLcp11_141)-RLcp11_167*ROcp11_835+RLcp11_267*ROcp11_735;
    JTcp11_167_13 = ROcp11_536*(RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341)-ROcp11_636*(RLcp11_238+RLcp11_239)-ROcp11_636
 *(RLcp11_240+RLcp11_241)-RLcp11_267*ROcp11_636+RLcp11_367*ROcp11_536;
    JTcp11_267_13 = RLcp11_167*ROcp11_636-RLcp11_367*ROcp11_436-ROcp11_436*(RLcp11_338+RLcp11_339+RLcp11_340+RLcp11_341)+
 ROcp11_636*(RLcp11_138+RLcp11_139)+ROcp11_636*(RLcp11_140+RLcp11_141);
    JTcp11_367_13 = ROcp11_436*(RLcp11_238+RLcp11_239+RLcp11_240+RLcp11_241)-ROcp11_536*(RLcp11_138+RLcp11_139)-ROcp11_536
 *(RLcp11_140+RLcp11_141)-RLcp11_167*ROcp11_536+RLcp11_267*ROcp11_436;
    JTcp11_167_14 = ROcp11_237*(RLcp11_339+RLcp11_340+RLcp11_341+RLcp11_367)-ROcp11_337*(RLcp11_239+RLcp11_240)-ROcp11_337
 *(RLcp11_241+RLcp11_267);
    JTcp11_267_14 = -(ROcp11_137*(RLcp11_339+RLcp11_340+RLcp11_341+RLcp11_367)-ROcp11_337*(RLcp11_139+RLcp11_140)-
 ROcp11_337*(RLcp11_141+RLcp11_167));
    JTcp11_367_14 = ROcp11_137*(RLcp11_239+RLcp11_240+RLcp11_241+RLcp11_267)-ROcp11_237*(RLcp11_139+RLcp11_140)-ROcp11_237
 *(RLcp11_141+RLcp11_167);
    JTcp11_167_15 = ROcp11_838*(RLcp11_340+RLcp11_341)-ROcp11_938*(RLcp11_240+RLcp11_241)-RLcp11_267*ROcp11_938+RLcp11_367
 *ROcp11_838;
    JTcp11_267_15 = RLcp11_167*ROcp11_938-RLcp11_367*ROcp11_738-ROcp11_738*(RLcp11_340+RLcp11_341)+ROcp11_938*(RLcp11_140+
 RLcp11_141);
    JTcp11_367_15 = ROcp11_738*(RLcp11_240+RLcp11_241)-ROcp11_838*(RLcp11_140+RLcp11_141)-RLcp11_167*ROcp11_838+RLcp11_267
 *ROcp11_738;
    JTcp11_167_16 = ROcp11_539*(RLcp11_341+RLcp11_367)-ROcp11_639*(RLcp11_241+RLcp11_267);
    JTcp11_267_16 = -(ROcp11_439*(RLcp11_341+RLcp11_367)-ROcp11_639*(RLcp11_141+RLcp11_167));
    JTcp11_367_16 = ROcp11_439*(RLcp11_241+RLcp11_267)-ROcp11_539*(RLcp11_141+RLcp11_167);
    JTcp11_167_17 = -(RLcp11_267*ROcp11_940-RLcp11_367*ROcp11_840);
    JTcp11_267_17 = RLcp11_167*ROcp11_940-RLcp11_367*ROcp11_740;
    JTcp11_367_17 = -(RLcp11_167*ROcp11_840-RLcp11_267*ROcp11_740);
    JTcp11_167_18 = -(RLcp11_267*ROcp11_641-RLcp11_367*ROcp11_541);
    JTcp11_267_18 = RLcp11_167*ROcp11_641-RLcp11_367*ROcp11_441;
    JTcp11_367_18 = -(RLcp11_167*ROcp11_541-RLcp11_267*ROcp11_441);
    ORcp11_167 = OMcp11_242*RLcp11_367-OMcp11_342*RLcp11_267;
    ORcp11_267 = -(OMcp11_142*RLcp11_367-OMcp11_342*RLcp11_167);
    ORcp11_367 = OMcp11_142*RLcp11_267-OMcp11_242*RLcp11_167;
    VIcp11_167 = ORcp11_131+ORcp11_134+ORcp11_135+ORcp11_138+ORcp11_139+ORcp11_140+ORcp11_141+ORcp11_167+qd[1];
    VIcp11_267 = ORcp11_231+ORcp11_234+ORcp11_235+ORcp11_238+ORcp11_239+ORcp11_240+ORcp11_241+ORcp11_267+qd[2];
    VIcp11_367 = ORcp11_331+ORcp11_334+ORcp11_335+ORcp11_338+ORcp11_339+ORcp11_340+ORcp11_341+ORcp11_367+qd[3];
    ACcp11_167 = qdd[1]+OMcp11_233*ORcp11_334+OMcp11_234*ORcp11_335+OMcp11_237*ORcp11_338+OMcp11_238*ORcp11_339+OMcp11_239
 *ORcp11_340+OMcp11_240*ORcp11_341+OMcp11_242*ORcp11_367+OMcp11_26*ORcp11_331-OMcp11_333*ORcp11_234-OMcp11_334*ORcp11_235-
 OMcp11_337*ORcp11_238-OMcp11_338*ORcp11_239-OMcp11_339*ORcp11_240-OMcp11_340*ORcp11_241-OMcp11_342*ORcp11_267-OMcp11_36*
 ORcp11_231+OPcp11_233*RLcp11_334+OPcp11_234*RLcp11_335+OPcp11_237*RLcp11_338+OPcp11_238*RLcp11_339+OPcp11_239*RLcp11_340+
 OPcp11_240*RLcp11_341+OPcp11_242*RLcp11_367+OPcp11_26*RLcp11_331-OPcp11_333*RLcp11_234-OPcp11_334*RLcp11_235-OPcp11_337*
 RLcp11_238-OPcp11_338*RLcp11_239-OPcp11_339*RLcp11_240-OPcp11_340*RLcp11_241-OPcp11_342*RLcp11_267-OPcp11_36*RLcp11_231;
    ACcp11_267 = qdd[2]-OMcp11_133*ORcp11_334-OMcp11_134*ORcp11_335-OMcp11_137*ORcp11_338-OMcp11_138*ORcp11_339-OMcp11_139
 *ORcp11_340-OMcp11_140*ORcp11_341-OMcp11_142*ORcp11_367-OMcp11_16*ORcp11_331+OMcp11_333*ORcp11_134+OMcp11_334*ORcp11_135+
 OMcp11_337*ORcp11_138+OMcp11_338*ORcp11_139+OMcp11_339*ORcp11_140+OMcp11_340*ORcp11_141+OMcp11_342*ORcp11_167+OMcp11_36*
 ORcp11_131-OPcp11_133*RLcp11_334-OPcp11_134*RLcp11_335-OPcp11_137*RLcp11_338-OPcp11_138*RLcp11_339-OPcp11_139*RLcp11_340-
 OPcp11_140*RLcp11_341-OPcp11_142*RLcp11_367-OPcp11_16*RLcp11_331+OPcp11_333*RLcp11_134+OPcp11_334*RLcp11_135+OPcp11_337*
 RLcp11_138+OPcp11_338*RLcp11_139+OPcp11_339*RLcp11_140+OPcp11_340*RLcp11_141+OPcp11_342*RLcp11_167+OPcp11_36*RLcp11_131;
    ACcp11_367 = qdd[3]+OMcp11_133*ORcp11_234+OMcp11_134*ORcp11_235+OMcp11_137*ORcp11_238+OMcp11_138*ORcp11_239+OMcp11_139
 *ORcp11_240+OMcp11_140*ORcp11_241+OMcp11_142*ORcp11_267+OMcp11_16*ORcp11_231-OMcp11_233*ORcp11_134-OMcp11_234*ORcp11_135-
 OMcp11_237*ORcp11_138-OMcp11_238*ORcp11_139-OMcp11_239*ORcp11_140-OMcp11_240*ORcp11_141-OMcp11_242*ORcp11_167-OMcp11_26*
 ORcp11_131+OPcp11_133*RLcp11_234+OPcp11_134*RLcp11_235+OPcp11_137*RLcp11_238+OPcp11_138*RLcp11_239+OPcp11_139*RLcp11_240+
 OPcp11_140*RLcp11_241+OPcp11_142*RLcp11_267+OPcp11_16*RLcp11_231-OPcp11_233*RLcp11_134-OPcp11_234*RLcp11_135-OPcp11_237*
 RLcp11_138-OPcp11_238*RLcp11_139-OPcp11_239*RLcp11_140-OPcp11_240*RLcp11_141-OPcp11_242*RLcp11_167-OPcp11_26*RLcp11_131;

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_167;
    sens->P[2] = POcp11_267;
    sens->P[3] = POcp11_367;
    sens->R[1][1] = ROcp11_142;
    sens->R[1][2] = ROcp11_242;
    sens->R[1][3] = ROcp11_342;
    sens->R[2][1] = ROcp11_441;
    sens->R[2][2] = ROcp11_541;
    sens->R[2][3] = ROcp11_641;
    sens->R[3][1] = ROcp11_742;
    sens->R[3][2] = ROcp11_842;
    sens->R[3][3] = ROcp11_942;
    sens->V[1] = VIcp11_167;
    sens->V[2] = VIcp11_267;
    sens->V[3] = VIcp11_367;
    sens->OM[1] = OMcp11_142;
    sens->OM[2] = OMcp11_242;
    sens->OM[3] = OMcp11_342;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp11_167_5;
    sens->J[1][6] = JTcp11_167_6;
    sens->J[1][31] = JTcp11_167_7;
    sens->J[1][32] = JTcp11_167_8;
    sens->J[1][33] = JTcp11_167_9;
    sens->J[1][34] = JTcp11_167_10;
    sens->J[1][35] = JTcp11_167_11;
    sens->J[1][36] = JTcp11_167_12;
    sens->J[1][37] = JTcp11_167_13;
    sens->J[1][38] = JTcp11_167_14;
    sens->J[1][39] = JTcp11_167_15;
    sens->J[1][40] = JTcp11_167_16;
    sens->J[1][41] = JTcp11_167_17;
    sens->J[1][42] = JTcp11_167_18;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp11_267_4;
    sens->J[2][5] = JTcp11_267_5;
    sens->J[2][6] = JTcp11_267_6;
    sens->J[2][31] = JTcp11_267_7;
    sens->J[2][32] = JTcp11_267_8;
    sens->J[2][33] = JTcp11_267_9;
    sens->J[2][34] = JTcp11_267_10;
    sens->J[2][35] = JTcp11_267_11;
    sens->J[2][36] = JTcp11_267_12;
    sens->J[2][37] = JTcp11_267_13;
    sens->J[2][38] = JTcp11_267_14;
    sens->J[2][39] = JTcp11_267_15;
    sens->J[2][40] = JTcp11_267_16;
    sens->J[2][41] = JTcp11_267_17;
    sens->J[2][42] = JTcp11_267_18;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp11_367_4;
    sens->J[3][5] = JTcp11_367_5;
    sens->J[3][6] = JTcp11_367_6;
    sens->J[3][31] = JTcp11_367_7;
    sens->J[3][32] = JTcp11_367_8;
    sens->J[3][33] = JTcp11_367_9;
    sens->J[3][34] = JTcp11_367_10;
    sens->J[3][35] = JTcp11_367_11;
    sens->J[3][36] = JTcp11_367_12;
    sens->J[3][37] = JTcp11_367_13;
    sens->J[3][38] = JTcp11_367_14;
    sens->J[3][39] = JTcp11_367_15;
    sens->J[3][40] = JTcp11_367_16;
    sens->J[3][41] = JTcp11_367_17;
    sens->J[3][42] = JTcp11_367_18;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp11_46;
    sens->J[4][32] = ROcp11_131;
    sens->J[4][33] = ROcp11_432;
    sens->J[4][34] = ROcp11_733;
    sens->J[4][35] = ROcp11_134;
    sens->J[4][36] = ROcp11_735;
    sens->J[4][37] = ROcp11_436;
    sens->J[4][38] = ROcp11_137;
    sens->J[4][39] = ROcp11_738;
    sens->J[4][40] = ROcp11_439;
    sens->J[4][41] = ROcp11_740;
    sens->J[4][42] = ROcp11_441;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp11_85;
    sens->J[5][31] = ROcp11_56;
    sens->J[5][32] = ROcp11_231;
    sens->J[5][33] = ROcp11_532;
    sens->J[5][34] = ROcp11_833;
    sens->J[5][35] = ROcp11_234;
    sens->J[5][36] = ROcp11_835;
    sens->J[5][37] = ROcp11_536;
    sens->J[5][38] = ROcp11_237;
    sens->J[5][39] = ROcp11_838;
    sens->J[5][40] = ROcp11_539;
    sens->J[5][41] = ROcp11_840;
    sens->J[5][42] = ROcp11_541;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp11_95;
    sens->J[6][31] = ROcp11_66;
    sens->J[6][32] = ROcp11_331;
    sens->J[6][33] = ROcp11_632;
    sens->J[6][34] = ROcp11_933;
    sens->J[6][35] = ROcp11_334;
    sens->J[6][36] = ROcp11_935;
    sens->J[6][37] = ROcp11_636;
    sens->J[6][38] = ROcp11_337;
    sens->J[6][39] = ROcp11_938;
    sens->J[6][40] = ROcp11_639;
    sens->J[6][41] = ROcp11_940;
    sens->J[6][42] = ROcp11_641;
    sens->A[1] = ACcp11_167;
    sens->A[2] = ACcp11_267;
    sens->A[3] = ACcp11_367;
    sens->OMP[1] = OPcp11_142;
    sens->OMP[2] = OPcp11_242;
    sens->OMP[3] = OPcp11_342;
 
// 
break;
case 13:
 


// = = Block_1_0_0_13_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp12_25 = qd[5]*C4;
    OMcp12_35 = qd[5]*S4;
    OMcp12_16 = qd[4]+qd[6]*S5;
    OMcp12_26 = OMcp12_25+ROcp12_85*qd[6];
    OMcp12_36 = OMcp12_35+ROcp12_95*qd[6];
    OPcp12_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp12_26 = ROcp12_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp12_35*S5-ROcp12_95*qd[4]);
    OPcp12_36 = ROcp12_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp12_25*S5-ROcp12_85*qd[4]);

// = = Block_1_0_0_13_0_4 = = 
 
// Sensor Kinematics 


    ROcp12_131 = ROcp12_16*C31-S31*S5;
    ROcp12_231 = ROcp12_26*C31-ROcp12_85*S31;
    ROcp12_331 = ROcp12_36*C31-ROcp12_95*S31;
    ROcp12_731 = ROcp12_16*S31+C31*S5;
    ROcp12_831 = ROcp12_26*S31+ROcp12_85*C31;
    ROcp12_931 = ROcp12_36*S31+ROcp12_95*C31;
    ROcp12_432 = ROcp12_46*C32+ROcp12_731*S32;
    ROcp12_532 = ROcp12_56*C32+ROcp12_831*S32;
    ROcp12_632 = ROcp12_66*C32+ROcp12_931*S32;
    ROcp12_732 = -(ROcp12_46*S32-ROcp12_731*C32);
    ROcp12_832 = -(ROcp12_56*S32-ROcp12_831*C32);
    ROcp12_932 = -(ROcp12_66*S32-ROcp12_931*C32);
    ROcp12_133 = ROcp12_131*C33-ROcp12_732*S33;
    ROcp12_233 = ROcp12_231*C33-ROcp12_832*S33;
    ROcp12_333 = ROcp12_331*C33-ROcp12_932*S33;
    ROcp12_733 = ROcp12_131*S33+ROcp12_732*C33;
    ROcp12_833 = ROcp12_231*S33+ROcp12_832*C33;
    ROcp12_933 = ROcp12_331*S33+ROcp12_932*C33;
    ROcp12_134 = ROcp12_133*C34+ROcp12_432*S34;
    ROcp12_234 = ROcp12_233*C34+ROcp12_532*S34;
    ROcp12_334 = ROcp12_333*C34+ROcp12_632*S34;
    ROcp12_434 = -(ROcp12_133*S34-ROcp12_432*C34);
    ROcp12_534 = -(ROcp12_233*S34-ROcp12_532*C34);
    ROcp12_634 = -(ROcp12_333*S34-ROcp12_632*C34);
    RLcp12_131 = ROcp12_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp12_231 = ROcp12_26*s->dpt[1][3]+ROcp12_85*s->dpt[3][3];
    RLcp12_331 = ROcp12_36*s->dpt[1][3]+ROcp12_95*s->dpt[3][3];
    OMcp12_131 = OMcp12_16+ROcp12_46*qd[31];
    OMcp12_231 = OMcp12_26+ROcp12_56*qd[31];
    OMcp12_331 = OMcp12_36+ROcp12_66*qd[31];
    ORcp12_131 = OMcp12_26*RLcp12_331-OMcp12_36*RLcp12_231;
    ORcp12_231 = -(OMcp12_16*RLcp12_331-OMcp12_36*RLcp12_131);
    ORcp12_331 = OMcp12_16*RLcp12_231-OMcp12_26*RLcp12_131;
    OMcp12_132 = OMcp12_131+ROcp12_131*qd[32];
    OMcp12_232 = OMcp12_231+ROcp12_231*qd[32];
    OMcp12_332 = OMcp12_331+ROcp12_331*qd[32];
    OMcp12_133 = OMcp12_132+ROcp12_432*qd[33];
    OMcp12_233 = OMcp12_232+ROcp12_532*qd[33];
    OMcp12_333 = OMcp12_332+ROcp12_632*qd[33];
    OPcp12_133 = OPcp12_16+ROcp12_131*qdd[32]+ROcp12_432*qdd[33]+ROcp12_46*qdd[31]+qd[31]*(OMcp12_26*ROcp12_66-OMcp12_36*
 ROcp12_56)+qd[32]*(OMcp12_231*ROcp12_331-OMcp12_331*ROcp12_231)+qd[33]*(OMcp12_232*ROcp12_632-OMcp12_332*ROcp12_532);
    OPcp12_233 = OPcp12_26+ROcp12_231*qdd[32]+ROcp12_532*qdd[33]+ROcp12_56*qdd[31]-qd[31]*(OMcp12_16*ROcp12_66-OMcp12_36*
 ROcp12_46)-qd[32]*(OMcp12_131*ROcp12_331-OMcp12_331*ROcp12_131)-qd[33]*(OMcp12_132*ROcp12_632-OMcp12_332*ROcp12_432);
    OPcp12_333 = OPcp12_36+ROcp12_331*qdd[32]+ROcp12_632*qdd[33]+ROcp12_66*qdd[31]+qd[31]*(OMcp12_16*ROcp12_56-OMcp12_26*
 ROcp12_46)+qd[32]*(OMcp12_131*ROcp12_231-OMcp12_231*ROcp12_131)+qd[33]*(OMcp12_132*ROcp12_532-OMcp12_232*ROcp12_432);
    RLcp12_134 = ROcp12_733*s->dpt[3][21];
    RLcp12_234 = ROcp12_833*s->dpt[3][21];
    RLcp12_334 = ROcp12_933*s->dpt[3][21];
    OMcp12_134 = OMcp12_133+ROcp12_733*qd[34];
    OMcp12_234 = OMcp12_233+ROcp12_833*qd[34];
    OMcp12_334 = OMcp12_333+ROcp12_933*qd[34];
    ORcp12_134 = OMcp12_233*RLcp12_334-OMcp12_333*RLcp12_234;
    ORcp12_234 = -(OMcp12_133*RLcp12_334-OMcp12_333*RLcp12_134);
    ORcp12_334 = OMcp12_133*RLcp12_234-OMcp12_233*RLcp12_134;
    OPcp12_134 = OPcp12_133+ROcp12_733*qdd[34]+qd[34]*(OMcp12_233*ROcp12_933-OMcp12_333*ROcp12_833);
    OPcp12_234 = OPcp12_233+ROcp12_833*qdd[34]-qd[34]*(OMcp12_133*ROcp12_933-OMcp12_333*ROcp12_733);
    OPcp12_334 = OPcp12_333+ROcp12_933*qdd[34]+qd[34]*(OMcp12_133*ROcp12_833-OMcp12_233*ROcp12_733);

// = = Block_1_0_0_13_0_5 = = 
 
// Sensor Kinematics 


    ROcp12_435 = ROcp12_434*C35+ROcp12_733*S35;
    ROcp12_535 = ROcp12_534*C35+ROcp12_833*S35;
    ROcp12_635 = ROcp12_634*C35+ROcp12_933*S35;
    ROcp12_735 = -(ROcp12_434*S35-ROcp12_733*C35);
    ROcp12_835 = -(ROcp12_534*S35-ROcp12_833*C35);
    ROcp12_935 = -(ROcp12_634*S35-ROcp12_933*C35);
    ROcp12_136 = ROcp12_134*C36+ROcp12_435*S36;
    ROcp12_236 = ROcp12_234*C36+ROcp12_535*S36;
    ROcp12_336 = ROcp12_334*C36+ROcp12_635*S36;
    ROcp12_436 = -(ROcp12_134*S36-ROcp12_435*C36);
    ROcp12_536 = -(ROcp12_234*S36-ROcp12_535*C36);
    ROcp12_636 = -(ROcp12_334*S36-ROcp12_635*C36);
    ROcp12_137 = ROcp12_136*C37-ROcp12_735*S37;
    ROcp12_237 = ROcp12_236*C37-ROcp12_835*S37;
    ROcp12_337 = ROcp12_336*C37-ROcp12_935*S37;
    ROcp12_737 = ROcp12_136*S37+ROcp12_735*C37;
    ROcp12_837 = ROcp12_236*S37+ROcp12_835*C37;
    ROcp12_937 = ROcp12_336*S37+ROcp12_935*C37;
    ROcp12_438 = ROcp12_436*C38+ROcp12_737*S38;
    ROcp12_538 = ROcp12_536*C38+ROcp12_837*S38;
    ROcp12_638 = ROcp12_636*C38+ROcp12_937*S38;
    ROcp12_738 = -(ROcp12_436*S38-ROcp12_737*C38);
    ROcp12_838 = -(ROcp12_536*S38-ROcp12_837*C38);
    ROcp12_938 = -(ROcp12_636*S38-ROcp12_937*C38);
    ROcp12_139 = ROcp12_137*C39+ROcp12_438*S39;
    ROcp12_239 = ROcp12_237*C39+ROcp12_538*S39;
    ROcp12_339 = ROcp12_337*C39+ROcp12_638*S39;
    ROcp12_439 = -(ROcp12_137*S39-ROcp12_438*C39);
    ROcp12_539 = -(ROcp12_237*S39-ROcp12_538*C39);
    ROcp12_639 = -(ROcp12_337*S39-ROcp12_638*C39);
    ROcp12_140 = ROcp12_139*C40-ROcp12_738*S40;
    ROcp12_240 = ROcp12_239*C40-ROcp12_838*S40;
    ROcp12_340 = ROcp12_339*C40-ROcp12_938*S40;
    ROcp12_740 = ROcp12_139*S40+ROcp12_738*C40;
    ROcp12_840 = ROcp12_239*S40+ROcp12_838*C40;
    ROcp12_940 = ROcp12_339*S40+ROcp12_938*C40;
    ROcp12_141 = ROcp12_140*C41+ROcp12_439*S41;
    ROcp12_241 = ROcp12_240*C41+ROcp12_539*S41;
    ROcp12_341 = ROcp12_340*C41+ROcp12_639*S41;
    ROcp12_441 = -(ROcp12_140*S41-ROcp12_439*C41);
    ROcp12_541 = -(ROcp12_240*S41-ROcp12_539*C41);
    ROcp12_641 = -(ROcp12_340*S41-ROcp12_639*C41);
    ROcp12_142 = ROcp12_141*C42-ROcp12_740*S42;
    ROcp12_242 = ROcp12_241*C42-ROcp12_840*S42;
    ROcp12_342 = ROcp12_341*C42-ROcp12_940*S42;
    ROcp12_742 = ROcp12_141*S42+ROcp12_740*C42;
    ROcp12_842 = ROcp12_241*S42+ROcp12_840*C42;
    ROcp12_942 = ROcp12_341*S42+ROcp12_940*C42;
    ROcp12_443 = ROcp12_441*C43+ROcp12_742*S43;
    ROcp12_543 = ROcp12_541*C43+ROcp12_842*S43;
    ROcp12_643 = ROcp12_641*C43+ROcp12_942*S43;
    ROcp12_743 = -(ROcp12_441*S43-ROcp12_742*C43);
    ROcp12_843 = -(ROcp12_541*S43-ROcp12_842*C43);
    ROcp12_943 = -(ROcp12_641*S43-ROcp12_942*C43);
    RLcp12_135 = ROcp12_134*s->dpt[1][23]+ROcp12_434*s->dpt[2][23]+ROcp12_733*s->dpt[3][23];
    RLcp12_235 = ROcp12_234*s->dpt[1][23]+ROcp12_534*s->dpt[2][23]+ROcp12_833*s->dpt[3][23];
    RLcp12_335 = ROcp12_334*s->dpt[1][23]+ROcp12_634*s->dpt[2][23]+ROcp12_933*s->dpt[3][23];
    ORcp12_135 = OMcp12_234*RLcp12_335-OMcp12_334*RLcp12_235;
    ORcp12_235 = -(OMcp12_134*RLcp12_335-OMcp12_334*RLcp12_135);
    ORcp12_335 = OMcp12_134*RLcp12_235-OMcp12_234*RLcp12_135;
    OMcp12_137 = OMcp12_134+ROcp12_436*qd[37];
    OMcp12_237 = OMcp12_234+ROcp12_536*qd[37];
    OMcp12_337 = OMcp12_334+ROcp12_636*qd[37];
    OPcp12_137 = OPcp12_134+ROcp12_436*qdd[37]+qd[37]*(OMcp12_234*ROcp12_636-OMcp12_334*ROcp12_536);
    OPcp12_237 = OPcp12_234+ROcp12_536*qdd[37]-qd[37]*(OMcp12_134*ROcp12_636-OMcp12_334*ROcp12_436);
    OPcp12_337 = OPcp12_334+ROcp12_636*qdd[37]+qd[37]*(OMcp12_134*ROcp12_536-OMcp12_234*ROcp12_436);
    RLcp12_138 = ROcp12_436*s->dpt[2][27]+ROcp12_737*s->dpt[3][27];
    RLcp12_238 = ROcp12_536*s->dpt[2][27]+ROcp12_837*s->dpt[3][27];
    RLcp12_338 = ROcp12_636*s->dpt[2][27]+ROcp12_937*s->dpt[3][27];
    OMcp12_138 = OMcp12_137+ROcp12_137*qd[38];
    OMcp12_238 = OMcp12_237+ROcp12_237*qd[38];
    OMcp12_338 = OMcp12_337+ROcp12_337*qd[38];
    ORcp12_138 = OMcp12_237*RLcp12_338-OMcp12_337*RLcp12_238;
    ORcp12_238 = -(OMcp12_137*RLcp12_338-OMcp12_337*RLcp12_138);
    ORcp12_338 = OMcp12_137*RLcp12_238-OMcp12_237*RLcp12_138;
    OPcp12_138 = OPcp12_137+ROcp12_137*qdd[38]+qd[38]*(OMcp12_237*ROcp12_337-OMcp12_337*ROcp12_237);
    OPcp12_238 = OPcp12_237+ROcp12_237*qdd[38]-qd[38]*(OMcp12_137*ROcp12_337-OMcp12_337*ROcp12_137);
    OPcp12_338 = OPcp12_337+ROcp12_337*qdd[38]+qd[38]*(OMcp12_137*ROcp12_237-OMcp12_237*ROcp12_137);
    RLcp12_139 = ROcp12_738*s->dpt[3][29];
    RLcp12_239 = ROcp12_838*s->dpt[3][29];
    RLcp12_339 = ROcp12_938*s->dpt[3][29];
    OMcp12_139 = OMcp12_138+ROcp12_738*qd[39];
    OMcp12_239 = OMcp12_238+ROcp12_838*qd[39];
    OMcp12_339 = OMcp12_338+ROcp12_938*qd[39];
    ORcp12_139 = OMcp12_238*RLcp12_339-OMcp12_338*RLcp12_239;
    ORcp12_239 = -(OMcp12_138*RLcp12_339-OMcp12_338*RLcp12_139);
    ORcp12_339 = OMcp12_138*RLcp12_239-OMcp12_238*RLcp12_139;
    OPcp12_139 = OPcp12_138+ROcp12_738*qdd[39]+qd[39]*(OMcp12_238*ROcp12_938-OMcp12_338*ROcp12_838);
    OPcp12_239 = OPcp12_238+ROcp12_838*qdd[39]-qd[39]*(OMcp12_138*ROcp12_938-OMcp12_338*ROcp12_738);
    OPcp12_339 = OPcp12_338+ROcp12_938*qdd[39]+qd[39]*(OMcp12_138*ROcp12_838-OMcp12_238*ROcp12_738);
    RLcp12_140 = ROcp12_139*s->dpt[1][31]+ROcp12_738*s->dpt[3][31];
    RLcp12_240 = ROcp12_239*s->dpt[1][31]+ROcp12_838*s->dpt[3][31];
    RLcp12_340 = ROcp12_339*s->dpt[1][31]+ROcp12_938*s->dpt[3][31];
    OMcp12_140 = OMcp12_139+ROcp12_439*qd[40];
    OMcp12_240 = OMcp12_239+ROcp12_539*qd[40];
    OMcp12_340 = OMcp12_339+ROcp12_639*qd[40];
    ORcp12_140 = OMcp12_239*RLcp12_340-OMcp12_339*RLcp12_240;
    ORcp12_240 = -(OMcp12_139*RLcp12_340-OMcp12_339*RLcp12_140);
    ORcp12_340 = OMcp12_139*RLcp12_240-OMcp12_239*RLcp12_140;
    OPcp12_140 = OPcp12_139+ROcp12_439*qdd[40]+qd[40]*(OMcp12_239*ROcp12_639-OMcp12_339*ROcp12_539);
    OPcp12_240 = OPcp12_239+ROcp12_539*qdd[40]-qd[40]*(OMcp12_139*ROcp12_639-OMcp12_339*ROcp12_439);
    OPcp12_340 = OPcp12_339+ROcp12_639*qdd[40]+qd[40]*(OMcp12_139*ROcp12_539-OMcp12_239*ROcp12_439);
    RLcp12_141 = ROcp12_140*s->dpt[1][33]+ROcp12_740*s->dpt[3][33];
    RLcp12_241 = ROcp12_240*s->dpt[1][33]+ROcp12_840*s->dpt[3][33];
    RLcp12_341 = ROcp12_340*s->dpt[1][33]+ROcp12_940*s->dpt[3][33];
    OMcp12_141 = OMcp12_140+ROcp12_740*qd[41];
    OMcp12_241 = OMcp12_240+ROcp12_840*qd[41];
    OMcp12_341 = OMcp12_340+ROcp12_940*qd[41];
    ORcp12_141 = OMcp12_240*RLcp12_341-OMcp12_340*RLcp12_241;
    ORcp12_241 = -(OMcp12_140*RLcp12_341-OMcp12_340*RLcp12_141);
    ORcp12_341 = OMcp12_140*RLcp12_241-OMcp12_240*RLcp12_141;
    OMcp12_142 = OMcp12_141+ROcp12_441*qd[42];
    OMcp12_242 = OMcp12_241+ROcp12_541*qd[42];
    OMcp12_342 = OMcp12_341+ROcp12_641*qd[42];
    OPcp12_142 = OPcp12_140+ROcp12_441*qdd[42]+ROcp12_740*qdd[41]+qd[41]*(OMcp12_240*ROcp12_940-OMcp12_340*ROcp12_840)+
 qd[42]*(OMcp12_241*ROcp12_641-OMcp12_341*ROcp12_541);
    OPcp12_242 = OPcp12_240+ROcp12_541*qdd[42]+ROcp12_840*qdd[41]-qd[41]*(OMcp12_140*ROcp12_940-OMcp12_340*ROcp12_740)-
 qd[42]*(OMcp12_141*ROcp12_641-OMcp12_341*ROcp12_441);
    OPcp12_342 = OPcp12_340+ROcp12_641*qdd[42]+ROcp12_940*qdd[41]+qd[41]*(OMcp12_140*ROcp12_840-OMcp12_240*ROcp12_740)+
 qd[42]*(OMcp12_141*ROcp12_541-OMcp12_241*ROcp12_441);
    RLcp12_143 = ROcp12_742*s->dpt[3][37];
    RLcp12_243 = ROcp12_842*s->dpt[3][37];
    RLcp12_343 = ROcp12_942*s->dpt[3][37];
    OMcp12_143 = OMcp12_142+ROcp12_142*qd[43];
    OMcp12_243 = OMcp12_242+ROcp12_242*qd[43];
    OMcp12_343 = OMcp12_342+ROcp12_342*qd[43];
    ORcp12_143 = OMcp12_242*RLcp12_343-OMcp12_342*RLcp12_243;
    ORcp12_243 = -(OMcp12_142*RLcp12_343-OMcp12_342*RLcp12_143);
    ORcp12_343 = OMcp12_142*RLcp12_243-OMcp12_242*RLcp12_143;
    OPcp12_143 = OPcp12_142+ROcp12_142*qdd[43]+qd[43]*(OMcp12_242*ROcp12_342-OMcp12_342*ROcp12_242);
    OPcp12_243 = OPcp12_242+ROcp12_242*qdd[43]-qd[43]*(OMcp12_142*ROcp12_342-OMcp12_342*ROcp12_142);
    OPcp12_343 = OPcp12_342+ROcp12_342*qdd[43]+qd[43]*(OMcp12_142*ROcp12_242-OMcp12_242*ROcp12_142);
    RLcp12_168 = ROcp12_142*s->dpt[1][39]+ROcp12_443*s->dpt[2][39]+ROcp12_743*s->dpt[3][39];
    RLcp12_268 = ROcp12_242*s->dpt[1][39]+ROcp12_543*s->dpt[2][39]+ROcp12_843*s->dpt[3][39];
    RLcp12_368 = ROcp12_342*s->dpt[1][39]+ROcp12_643*s->dpt[2][39]+ROcp12_943*s->dpt[3][39];
    POcp12_168 = RLcp12_131+RLcp12_134+RLcp12_135+RLcp12_138+RLcp12_139+RLcp12_140+RLcp12_141+RLcp12_143+RLcp12_168+q[1];
    POcp12_268 = RLcp12_231+RLcp12_234+RLcp12_235+RLcp12_238+RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243+RLcp12_268+q[2];
    POcp12_368 = RLcp12_331+RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368+q[3];
    JTcp12_268_4 = -(RLcp12_331+RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368);
    JTcp12_368_4 = RLcp12_231+RLcp12_234+RLcp12_235+RLcp12_238+RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243+RLcp12_268;
    JTcp12_168_5 = C4*(RLcp12_331+RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343)-S4*(
 RLcp12_231+RLcp12_234)-S4*(RLcp12_235+RLcp12_238)-S4*(RLcp12_239+RLcp12_240)-S4*(RLcp12_241+RLcp12_243)-RLcp12_268*S4+
 RLcp12_368*C4;
    JTcp12_268_5 = S4*(RLcp12_131+RLcp12_134+RLcp12_135+RLcp12_138+RLcp12_139+RLcp12_140+RLcp12_141+RLcp12_143+RLcp12_168);
    JTcp12_368_5 = -C4*(RLcp12_131+RLcp12_134+RLcp12_135+RLcp12_138+RLcp12_139+RLcp12_140+RLcp12_141+RLcp12_143+RLcp12_168
 );
    JTcp12_168_6 = ROcp12_85*(RLcp12_331+RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343)-
 ROcp12_95*(RLcp12_231+RLcp12_234)-ROcp12_95*(RLcp12_235+RLcp12_238)-ROcp12_95*(RLcp12_239+RLcp12_240)-ROcp12_95*(RLcp12_241+
 RLcp12_243)-RLcp12_268*ROcp12_95+RLcp12_368*ROcp12_85;
    JTcp12_268_6 = -(RLcp12_368*S5-ROcp12_95*(RLcp12_131+RLcp12_134+RLcp12_135+RLcp12_138+RLcp12_139+RLcp12_140+RLcp12_141
 +RLcp12_143+RLcp12_168)+S5*(RLcp12_331+RLcp12_334)+S5*(RLcp12_335+RLcp12_338)+S5*(RLcp12_339+RLcp12_340)+S5*(RLcp12_341+
 RLcp12_343));
    JTcp12_368_6 = RLcp12_268*S5-ROcp12_85*(RLcp12_131+RLcp12_134+RLcp12_135+RLcp12_138+RLcp12_139+RLcp12_140+RLcp12_141+
 RLcp12_143+RLcp12_168)+S5*(RLcp12_231+RLcp12_234)+S5*(RLcp12_235+RLcp12_238)+S5*(RLcp12_239+RLcp12_240)+S5*(RLcp12_241+
 RLcp12_243);
    JTcp12_168_7 = ROcp12_56*(RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-
 ROcp12_66*(RLcp12_234+RLcp12_235)-ROcp12_66*(RLcp12_238+RLcp12_239)-ROcp12_66*(RLcp12_240+RLcp12_241)-ROcp12_66*(RLcp12_243+
 RLcp12_268);
    JTcp12_268_7 = -(ROcp12_46*(RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-
 ROcp12_66*(RLcp12_134+RLcp12_135)-ROcp12_66*(RLcp12_138+RLcp12_139)-ROcp12_66*(RLcp12_140+RLcp12_141)-ROcp12_66*(RLcp12_143+
 RLcp12_168));
    JTcp12_368_7 = ROcp12_46*(RLcp12_234+RLcp12_235+RLcp12_238+RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243+RLcp12_268)-
 ROcp12_56*(RLcp12_134+RLcp12_135)-ROcp12_56*(RLcp12_138+RLcp12_139)-ROcp12_56*(RLcp12_140+RLcp12_141)-ROcp12_56*(RLcp12_143+
 RLcp12_168);
    JTcp12_168_8 = ROcp12_231*(RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-
 ROcp12_331*(RLcp12_234+RLcp12_235)-ROcp12_331*(RLcp12_238+RLcp12_239)-ROcp12_331*(RLcp12_240+RLcp12_241)-ROcp12_331*(
 RLcp12_243+RLcp12_268);
    JTcp12_268_8 = -(ROcp12_131*(RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-
 ROcp12_331*(RLcp12_134+RLcp12_135)-ROcp12_331*(RLcp12_138+RLcp12_139)-ROcp12_331*(RLcp12_140+RLcp12_141)-ROcp12_331*(
 RLcp12_143+RLcp12_168));
    JTcp12_368_8 = ROcp12_131*(RLcp12_234+RLcp12_235+RLcp12_238+RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243+RLcp12_268)-
 ROcp12_231*(RLcp12_134+RLcp12_135)-ROcp12_231*(RLcp12_138+RLcp12_139)-ROcp12_231*(RLcp12_140+RLcp12_141)-ROcp12_231*(
 RLcp12_143+RLcp12_168);
    JTcp12_168_9 = ROcp12_532*(RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-
 ROcp12_632*(RLcp12_234+RLcp12_235)-ROcp12_632*(RLcp12_238+RLcp12_239)-ROcp12_632*(RLcp12_240+RLcp12_241)-ROcp12_632*(
 RLcp12_243+RLcp12_268);
    JTcp12_268_9 = -(ROcp12_432*(RLcp12_334+RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-
 ROcp12_632*(RLcp12_134+RLcp12_135)-ROcp12_632*(RLcp12_138+RLcp12_139)-ROcp12_632*(RLcp12_140+RLcp12_141)-ROcp12_632*(
 RLcp12_143+RLcp12_168));
    JTcp12_368_9 = ROcp12_432*(RLcp12_234+RLcp12_235+RLcp12_238+RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243+RLcp12_268)-
 ROcp12_532*(RLcp12_134+RLcp12_135)-ROcp12_532*(RLcp12_138+RLcp12_139)-ROcp12_532*(RLcp12_140+RLcp12_141)-ROcp12_532*(
 RLcp12_143+RLcp12_168);
    JTcp12_168_10 = ROcp12_833*(RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343)-ROcp12_933*(RLcp12_235+
 RLcp12_238)-ROcp12_933*(RLcp12_239+RLcp12_240)-ROcp12_933*(RLcp12_241+RLcp12_243)-RLcp12_268*ROcp12_933+RLcp12_368*
 ROcp12_833;
    JTcp12_268_10 = RLcp12_168*ROcp12_933-RLcp12_368*ROcp12_733-ROcp12_733*(RLcp12_335+RLcp12_338+RLcp12_339+RLcp12_340+
 RLcp12_341+RLcp12_343)+ROcp12_933*(RLcp12_135+RLcp12_138)+ROcp12_933*(RLcp12_139+RLcp12_140)+ROcp12_933*(RLcp12_141+
 RLcp12_143);
    JTcp12_368_10 = ROcp12_733*(RLcp12_235+RLcp12_238+RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243)-ROcp12_833*(RLcp12_135+
 RLcp12_138)-ROcp12_833*(RLcp12_139+RLcp12_140)-ROcp12_833*(RLcp12_141+RLcp12_143)-RLcp12_168*ROcp12_833+RLcp12_268*
 ROcp12_733;
    JTcp12_168_11 = ROcp12_234*(RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-ROcp12_334*(RLcp12_238+
 RLcp12_239)-ROcp12_334*(RLcp12_240+RLcp12_241)-ROcp12_334*(RLcp12_243+RLcp12_268);
    JTcp12_268_11 = -(ROcp12_134*(RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-ROcp12_334*(
 RLcp12_138+RLcp12_139)-ROcp12_334*(RLcp12_140+RLcp12_141)-ROcp12_334*(RLcp12_143+RLcp12_168));
    JTcp12_368_11 = ROcp12_134*(RLcp12_238+RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243+RLcp12_268)-ROcp12_234*(RLcp12_138+
 RLcp12_139)-ROcp12_234*(RLcp12_140+RLcp12_141)-ROcp12_234*(RLcp12_143+RLcp12_168);
    JTcp12_168_12 = ROcp12_835*(RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-ROcp12_935*(RLcp12_238+
 RLcp12_239)-ROcp12_935*(RLcp12_240+RLcp12_241)-ROcp12_935*(RLcp12_243+RLcp12_268);
    JTcp12_268_12 = -(ROcp12_735*(RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-ROcp12_935*(
 RLcp12_138+RLcp12_139)-ROcp12_935*(RLcp12_140+RLcp12_141)-ROcp12_935*(RLcp12_143+RLcp12_168));
    JTcp12_368_12 = ROcp12_735*(RLcp12_238+RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243+RLcp12_268)-ROcp12_835*(RLcp12_138+
 RLcp12_139)-ROcp12_835*(RLcp12_140+RLcp12_141)-ROcp12_835*(RLcp12_143+RLcp12_168);
    JTcp12_168_13 = ROcp12_536*(RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-ROcp12_636*(RLcp12_238+
 RLcp12_239)-ROcp12_636*(RLcp12_240+RLcp12_241)-ROcp12_636*(RLcp12_243+RLcp12_268);
    JTcp12_268_13 = -(ROcp12_436*(RLcp12_338+RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-ROcp12_636*(
 RLcp12_138+RLcp12_139)-ROcp12_636*(RLcp12_140+RLcp12_141)-ROcp12_636*(RLcp12_143+RLcp12_168));
    JTcp12_368_13 = ROcp12_436*(RLcp12_238+RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243+RLcp12_268)-ROcp12_536*(RLcp12_138+
 RLcp12_139)-ROcp12_536*(RLcp12_140+RLcp12_141)-ROcp12_536*(RLcp12_143+RLcp12_168);
    JTcp12_168_14 = ROcp12_237*(RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343)-ROcp12_337*(RLcp12_239+RLcp12_240)-ROcp12_337
 *(RLcp12_241+RLcp12_243)-RLcp12_268*ROcp12_337+RLcp12_368*ROcp12_237;
    JTcp12_268_14 = RLcp12_168*ROcp12_337-RLcp12_368*ROcp12_137-ROcp12_137*(RLcp12_339+RLcp12_340+RLcp12_341+RLcp12_343)+
 ROcp12_337*(RLcp12_139+RLcp12_140)+ROcp12_337*(RLcp12_141+RLcp12_143);
    JTcp12_368_14 = ROcp12_137*(RLcp12_239+RLcp12_240+RLcp12_241+RLcp12_243)-ROcp12_237*(RLcp12_139+RLcp12_140)-ROcp12_237
 *(RLcp12_141+RLcp12_143)-RLcp12_168*ROcp12_237+RLcp12_268*ROcp12_137;
    JTcp12_168_15 = ROcp12_838*(RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-ROcp12_938*(RLcp12_240+RLcp12_241)-ROcp12_938
 *(RLcp12_243+RLcp12_268);
    JTcp12_268_15 = -(ROcp12_738*(RLcp12_340+RLcp12_341+RLcp12_343+RLcp12_368)-ROcp12_938*(RLcp12_140+RLcp12_141)-
 ROcp12_938*(RLcp12_143+RLcp12_168));
    JTcp12_368_15 = ROcp12_738*(RLcp12_240+RLcp12_241+RLcp12_243+RLcp12_268)-ROcp12_838*(RLcp12_140+RLcp12_141)-ROcp12_838
 *(RLcp12_143+RLcp12_168);
    JTcp12_168_16 = ROcp12_539*(RLcp12_341+RLcp12_343)-ROcp12_639*(RLcp12_241+RLcp12_243)-RLcp12_268*ROcp12_639+RLcp12_368
 *ROcp12_539;
    JTcp12_268_16 = RLcp12_168*ROcp12_639-RLcp12_368*ROcp12_439-ROcp12_439*(RLcp12_341+RLcp12_343)+ROcp12_639*(RLcp12_141+
 RLcp12_143);
    JTcp12_368_16 = ROcp12_439*(RLcp12_241+RLcp12_243)-ROcp12_539*(RLcp12_141+RLcp12_143)-RLcp12_168*ROcp12_539+RLcp12_268
 *ROcp12_439;
    JTcp12_168_17 = ROcp12_840*(RLcp12_343+RLcp12_368)-ROcp12_940*(RLcp12_243+RLcp12_268);
    JTcp12_268_17 = -(ROcp12_740*(RLcp12_343+RLcp12_368)-ROcp12_940*(RLcp12_143+RLcp12_168));
    JTcp12_368_17 = ROcp12_740*(RLcp12_243+RLcp12_268)-ROcp12_840*(RLcp12_143+RLcp12_168);
    JTcp12_168_18 = ROcp12_541*(RLcp12_343+RLcp12_368)-ROcp12_641*(RLcp12_243+RLcp12_268);
    JTcp12_268_18 = -(ROcp12_441*(RLcp12_343+RLcp12_368)-ROcp12_641*(RLcp12_143+RLcp12_168));
    JTcp12_368_18 = ROcp12_441*(RLcp12_243+RLcp12_268)-ROcp12_541*(RLcp12_143+RLcp12_168);
    JTcp12_168_19 = -(RLcp12_268*ROcp12_342-RLcp12_368*ROcp12_242);
    JTcp12_268_19 = RLcp12_168*ROcp12_342-RLcp12_368*ROcp12_142;
    JTcp12_368_19 = -(RLcp12_168*ROcp12_242-RLcp12_268*ROcp12_142);
    ORcp12_168 = OMcp12_243*RLcp12_368-OMcp12_343*RLcp12_268;
    ORcp12_268 = -(OMcp12_143*RLcp12_368-OMcp12_343*RLcp12_168);
    ORcp12_368 = OMcp12_143*RLcp12_268-OMcp12_243*RLcp12_168;
    VIcp12_168 = ORcp12_131+ORcp12_134+ORcp12_135+ORcp12_138+ORcp12_139+ORcp12_140+ORcp12_141+ORcp12_143+ORcp12_168+qd[1];
    VIcp12_268 = ORcp12_231+ORcp12_234+ORcp12_235+ORcp12_238+ORcp12_239+ORcp12_240+ORcp12_241+ORcp12_243+ORcp12_268+qd[2];
    VIcp12_368 = ORcp12_331+ORcp12_334+ORcp12_335+ORcp12_338+ORcp12_339+ORcp12_340+ORcp12_341+ORcp12_343+ORcp12_368+qd[3];
    ACcp12_168 = qdd[1]+OMcp12_233*ORcp12_334+OMcp12_234*ORcp12_335+OMcp12_237*ORcp12_338+OMcp12_238*ORcp12_339+OMcp12_239
 *ORcp12_340+OMcp12_240*ORcp12_341+OMcp12_242*ORcp12_343+OMcp12_243*ORcp12_368+OMcp12_26*ORcp12_331-OMcp12_333*ORcp12_234-
 OMcp12_334*ORcp12_235-OMcp12_337*ORcp12_238-OMcp12_338*ORcp12_239-OMcp12_339*ORcp12_240-OMcp12_340*ORcp12_241-OMcp12_342*
 ORcp12_243-OMcp12_343*ORcp12_268-OMcp12_36*ORcp12_231+OPcp12_233*RLcp12_334+OPcp12_234*RLcp12_335+OPcp12_237*RLcp12_338+
 OPcp12_238*RLcp12_339+OPcp12_239*RLcp12_340+OPcp12_240*RLcp12_341+OPcp12_242*RLcp12_343+OPcp12_243*RLcp12_368+OPcp12_26*
 RLcp12_331-OPcp12_333*RLcp12_234-OPcp12_334*RLcp12_235-OPcp12_337*RLcp12_238-OPcp12_338*RLcp12_239-OPcp12_339*RLcp12_240-
 OPcp12_340*RLcp12_241-OPcp12_342*RLcp12_243-OPcp12_343*RLcp12_268-OPcp12_36*RLcp12_231;
    ACcp12_268 = qdd[2]-OMcp12_133*ORcp12_334-OMcp12_134*ORcp12_335-OMcp12_137*ORcp12_338-OMcp12_138*ORcp12_339-OMcp12_139
 *ORcp12_340-OMcp12_140*ORcp12_341-OMcp12_142*ORcp12_343-OMcp12_143*ORcp12_368-OMcp12_16*ORcp12_331+OMcp12_333*ORcp12_134+
 OMcp12_334*ORcp12_135+OMcp12_337*ORcp12_138+OMcp12_338*ORcp12_139+OMcp12_339*ORcp12_140+OMcp12_340*ORcp12_141+OMcp12_342*
 ORcp12_143+OMcp12_343*ORcp12_168+OMcp12_36*ORcp12_131-OPcp12_133*RLcp12_334-OPcp12_134*RLcp12_335-OPcp12_137*RLcp12_338-
 OPcp12_138*RLcp12_339-OPcp12_139*RLcp12_340-OPcp12_140*RLcp12_341-OPcp12_142*RLcp12_343-OPcp12_143*RLcp12_368-OPcp12_16*
 RLcp12_331+OPcp12_333*RLcp12_134+OPcp12_334*RLcp12_135+OPcp12_337*RLcp12_138+OPcp12_338*RLcp12_139+OPcp12_339*RLcp12_140+
 OPcp12_340*RLcp12_141+OPcp12_342*RLcp12_143+OPcp12_343*RLcp12_168+OPcp12_36*RLcp12_131;
    ACcp12_368 = qdd[3]+OMcp12_133*ORcp12_234+OMcp12_134*ORcp12_235+OMcp12_137*ORcp12_238+OMcp12_138*ORcp12_239+OMcp12_139
 *ORcp12_240+OMcp12_140*ORcp12_241+OMcp12_142*ORcp12_243+OMcp12_143*ORcp12_268+OMcp12_16*ORcp12_231-OMcp12_233*ORcp12_134-
 OMcp12_234*ORcp12_135-OMcp12_237*ORcp12_138-OMcp12_238*ORcp12_139-OMcp12_239*ORcp12_140-OMcp12_240*ORcp12_141-OMcp12_242*
 ORcp12_143-OMcp12_243*ORcp12_168-OMcp12_26*ORcp12_131+OPcp12_133*RLcp12_234+OPcp12_134*RLcp12_235+OPcp12_137*RLcp12_238+
 OPcp12_138*RLcp12_239+OPcp12_139*RLcp12_240+OPcp12_140*RLcp12_241+OPcp12_142*RLcp12_243+OPcp12_143*RLcp12_268+OPcp12_16*
 RLcp12_231-OPcp12_233*RLcp12_134-OPcp12_234*RLcp12_135-OPcp12_237*RLcp12_138-OPcp12_238*RLcp12_139-OPcp12_239*RLcp12_140-
 OPcp12_240*RLcp12_141-OPcp12_242*RLcp12_143-OPcp12_243*RLcp12_168-OPcp12_26*RLcp12_131;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_168;
    sens->P[2] = POcp12_268;
    sens->P[3] = POcp12_368;
    sens->R[1][1] = ROcp12_142;
    sens->R[1][2] = ROcp12_242;
    sens->R[1][3] = ROcp12_342;
    sens->R[2][1] = ROcp12_443;
    sens->R[2][2] = ROcp12_543;
    sens->R[2][3] = ROcp12_643;
    sens->R[3][1] = ROcp12_743;
    sens->R[3][2] = ROcp12_843;
    sens->R[3][3] = ROcp12_943;
    sens->V[1] = VIcp12_168;
    sens->V[2] = VIcp12_268;
    sens->V[3] = VIcp12_368;
    sens->OM[1] = OMcp12_143;
    sens->OM[2] = OMcp12_243;
    sens->OM[3] = OMcp12_343;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp12_168_5;
    sens->J[1][6] = JTcp12_168_6;
    sens->J[1][31] = JTcp12_168_7;
    sens->J[1][32] = JTcp12_168_8;
    sens->J[1][33] = JTcp12_168_9;
    sens->J[1][34] = JTcp12_168_10;
    sens->J[1][35] = JTcp12_168_11;
    sens->J[1][36] = JTcp12_168_12;
    sens->J[1][37] = JTcp12_168_13;
    sens->J[1][38] = JTcp12_168_14;
    sens->J[1][39] = JTcp12_168_15;
    sens->J[1][40] = JTcp12_168_16;
    sens->J[1][41] = JTcp12_168_17;
    sens->J[1][42] = JTcp12_168_18;
    sens->J[1][43] = JTcp12_168_19;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp12_268_4;
    sens->J[2][5] = JTcp12_268_5;
    sens->J[2][6] = JTcp12_268_6;
    sens->J[2][31] = JTcp12_268_7;
    sens->J[2][32] = JTcp12_268_8;
    sens->J[2][33] = JTcp12_268_9;
    sens->J[2][34] = JTcp12_268_10;
    sens->J[2][35] = JTcp12_268_11;
    sens->J[2][36] = JTcp12_268_12;
    sens->J[2][37] = JTcp12_268_13;
    sens->J[2][38] = JTcp12_268_14;
    sens->J[2][39] = JTcp12_268_15;
    sens->J[2][40] = JTcp12_268_16;
    sens->J[2][41] = JTcp12_268_17;
    sens->J[2][42] = JTcp12_268_18;
    sens->J[2][43] = JTcp12_268_19;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp12_368_4;
    sens->J[3][5] = JTcp12_368_5;
    sens->J[3][6] = JTcp12_368_6;
    sens->J[3][31] = JTcp12_368_7;
    sens->J[3][32] = JTcp12_368_8;
    sens->J[3][33] = JTcp12_368_9;
    sens->J[3][34] = JTcp12_368_10;
    sens->J[3][35] = JTcp12_368_11;
    sens->J[3][36] = JTcp12_368_12;
    sens->J[3][37] = JTcp12_368_13;
    sens->J[3][38] = JTcp12_368_14;
    sens->J[3][39] = JTcp12_368_15;
    sens->J[3][40] = JTcp12_368_16;
    sens->J[3][41] = JTcp12_368_17;
    sens->J[3][42] = JTcp12_368_18;
    sens->J[3][43] = JTcp12_368_19;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp12_46;
    sens->J[4][32] = ROcp12_131;
    sens->J[4][33] = ROcp12_432;
    sens->J[4][34] = ROcp12_733;
    sens->J[4][35] = ROcp12_134;
    sens->J[4][36] = ROcp12_735;
    sens->J[4][37] = ROcp12_436;
    sens->J[4][38] = ROcp12_137;
    sens->J[4][39] = ROcp12_738;
    sens->J[4][40] = ROcp12_439;
    sens->J[4][41] = ROcp12_740;
    sens->J[4][42] = ROcp12_441;
    sens->J[4][43] = ROcp12_142;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp12_85;
    sens->J[5][31] = ROcp12_56;
    sens->J[5][32] = ROcp12_231;
    sens->J[5][33] = ROcp12_532;
    sens->J[5][34] = ROcp12_833;
    sens->J[5][35] = ROcp12_234;
    sens->J[5][36] = ROcp12_835;
    sens->J[5][37] = ROcp12_536;
    sens->J[5][38] = ROcp12_237;
    sens->J[5][39] = ROcp12_838;
    sens->J[5][40] = ROcp12_539;
    sens->J[5][41] = ROcp12_840;
    sens->J[5][42] = ROcp12_541;
    sens->J[5][43] = ROcp12_242;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp12_95;
    sens->J[6][31] = ROcp12_66;
    sens->J[6][32] = ROcp12_331;
    sens->J[6][33] = ROcp12_632;
    sens->J[6][34] = ROcp12_933;
    sens->J[6][35] = ROcp12_334;
    sens->J[6][36] = ROcp12_935;
    sens->J[6][37] = ROcp12_636;
    sens->J[6][38] = ROcp12_337;
    sens->J[6][39] = ROcp12_938;
    sens->J[6][40] = ROcp12_639;
    sens->J[6][41] = ROcp12_940;
    sens->J[6][42] = ROcp12_641;
    sens->J[6][43] = ROcp12_342;
    sens->A[1] = ACcp12_168;
    sens->A[2] = ACcp12_268;
    sens->A[3] = ACcp12_368;
    sens->OMP[1] = OPcp12_143;
    sens->OMP[2] = OPcp12_243;
    sens->OMP[3] = OPcp12_343;
 
// 
break;
case 14:
 


// = = Block_1_0_0_14_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp13_25 = qd[5]*C4;
    OMcp13_35 = qd[5]*S4;
    OMcp13_16 = qd[4]+qd[6]*S5;
    OMcp13_26 = OMcp13_25+ROcp13_85*qd[6];
    OMcp13_36 = OMcp13_35+ROcp13_95*qd[6];
    OPcp13_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp13_26 = ROcp13_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp13_35*S5-ROcp13_95*qd[4]);
    OPcp13_36 = ROcp13_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp13_25*S5-ROcp13_85*qd[4]);

// = = Block_1_0_0_14_0_4 = = 
 
// Sensor Kinematics 


    ROcp13_131 = ROcp13_16*C31-S31*S5;
    ROcp13_231 = ROcp13_26*C31-ROcp13_85*S31;
    ROcp13_331 = ROcp13_36*C31-ROcp13_95*S31;
    ROcp13_731 = ROcp13_16*S31+C31*S5;
    ROcp13_831 = ROcp13_26*S31+ROcp13_85*C31;
    ROcp13_931 = ROcp13_36*S31+ROcp13_95*C31;
    ROcp13_432 = ROcp13_46*C32+ROcp13_731*S32;
    ROcp13_532 = ROcp13_56*C32+ROcp13_831*S32;
    ROcp13_632 = ROcp13_66*C32+ROcp13_931*S32;
    ROcp13_732 = -(ROcp13_46*S32-ROcp13_731*C32);
    ROcp13_832 = -(ROcp13_56*S32-ROcp13_831*C32);
    ROcp13_932 = -(ROcp13_66*S32-ROcp13_931*C32);
    ROcp13_133 = ROcp13_131*C33-ROcp13_732*S33;
    ROcp13_233 = ROcp13_231*C33-ROcp13_832*S33;
    ROcp13_333 = ROcp13_331*C33-ROcp13_932*S33;
    ROcp13_733 = ROcp13_131*S33+ROcp13_732*C33;
    ROcp13_833 = ROcp13_231*S33+ROcp13_832*C33;
    ROcp13_933 = ROcp13_331*S33+ROcp13_932*C33;
    ROcp13_134 = ROcp13_133*C34+ROcp13_432*S34;
    ROcp13_234 = ROcp13_233*C34+ROcp13_532*S34;
    ROcp13_334 = ROcp13_333*C34+ROcp13_632*S34;
    ROcp13_434 = -(ROcp13_133*S34-ROcp13_432*C34);
    ROcp13_534 = -(ROcp13_233*S34-ROcp13_532*C34);
    ROcp13_634 = -(ROcp13_333*S34-ROcp13_632*C34);
    RLcp13_131 = ROcp13_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp13_231 = ROcp13_26*s->dpt[1][3]+ROcp13_85*s->dpt[3][3];
    RLcp13_331 = ROcp13_36*s->dpt[1][3]+ROcp13_95*s->dpt[3][3];
    OMcp13_131 = OMcp13_16+ROcp13_46*qd[31];
    OMcp13_231 = OMcp13_26+ROcp13_56*qd[31];
    OMcp13_331 = OMcp13_36+ROcp13_66*qd[31];
    ORcp13_131 = OMcp13_26*RLcp13_331-OMcp13_36*RLcp13_231;
    ORcp13_231 = -(OMcp13_16*RLcp13_331-OMcp13_36*RLcp13_131);
    ORcp13_331 = OMcp13_16*RLcp13_231-OMcp13_26*RLcp13_131;
    OMcp13_132 = OMcp13_131+ROcp13_131*qd[32];
    OMcp13_232 = OMcp13_231+ROcp13_231*qd[32];
    OMcp13_332 = OMcp13_331+ROcp13_331*qd[32];
    OMcp13_133 = OMcp13_132+ROcp13_432*qd[33];
    OMcp13_233 = OMcp13_232+ROcp13_532*qd[33];
    OMcp13_333 = OMcp13_332+ROcp13_632*qd[33];
    OPcp13_133 = OPcp13_16+ROcp13_131*qdd[32]+ROcp13_432*qdd[33]+ROcp13_46*qdd[31]+qd[31]*(OMcp13_26*ROcp13_66-OMcp13_36*
 ROcp13_56)+qd[32]*(OMcp13_231*ROcp13_331-OMcp13_331*ROcp13_231)+qd[33]*(OMcp13_232*ROcp13_632-OMcp13_332*ROcp13_532);
    OPcp13_233 = OPcp13_26+ROcp13_231*qdd[32]+ROcp13_532*qdd[33]+ROcp13_56*qdd[31]-qd[31]*(OMcp13_16*ROcp13_66-OMcp13_36*
 ROcp13_46)-qd[32]*(OMcp13_131*ROcp13_331-OMcp13_331*ROcp13_131)-qd[33]*(OMcp13_132*ROcp13_632-OMcp13_332*ROcp13_432);
    OPcp13_333 = OPcp13_36+ROcp13_331*qdd[32]+ROcp13_632*qdd[33]+ROcp13_66*qdd[31]+qd[31]*(OMcp13_16*ROcp13_56-OMcp13_26*
 ROcp13_46)+qd[32]*(OMcp13_131*ROcp13_231-OMcp13_231*ROcp13_131)+qd[33]*(OMcp13_132*ROcp13_532-OMcp13_232*ROcp13_432);
    RLcp13_134 = ROcp13_733*s->dpt[3][21];
    RLcp13_234 = ROcp13_833*s->dpt[3][21];
    RLcp13_334 = ROcp13_933*s->dpt[3][21];
    OMcp13_134 = OMcp13_133+ROcp13_733*qd[34];
    OMcp13_234 = OMcp13_233+ROcp13_833*qd[34];
    OMcp13_334 = OMcp13_333+ROcp13_933*qd[34];
    ORcp13_134 = OMcp13_233*RLcp13_334-OMcp13_333*RLcp13_234;
    ORcp13_234 = -(OMcp13_133*RLcp13_334-OMcp13_333*RLcp13_134);
    ORcp13_334 = OMcp13_133*RLcp13_234-OMcp13_233*RLcp13_134;
    OPcp13_134 = OPcp13_133+ROcp13_733*qdd[34]+qd[34]*(OMcp13_233*ROcp13_933-OMcp13_333*ROcp13_833);
    OPcp13_234 = OPcp13_233+ROcp13_833*qdd[34]-qd[34]*(OMcp13_133*ROcp13_933-OMcp13_333*ROcp13_733);
    OPcp13_334 = OPcp13_333+ROcp13_933*qdd[34]+qd[34]*(OMcp13_133*ROcp13_833-OMcp13_233*ROcp13_733);

// = = Block_1_0_0_14_0_5 = = 
 
// Sensor Kinematics 


    ROcp13_435 = ROcp13_434*C35+ROcp13_733*S35;
    ROcp13_535 = ROcp13_534*C35+ROcp13_833*S35;
    ROcp13_635 = ROcp13_634*C35+ROcp13_933*S35;
    ROcp13_735 = -(ROcp13_434*S35-ROcp13_733*C35);
    ROcp13_835 = -(ROcp13_534*S35-ROcp13_833*C35);
    ROcp13_935 = -(ROcp13_634*S35-ROcp13_933*C35);
    ROcp13_136 = ROcp13_134*C36+ROcp13_435*S36;
    ROcp13_236 = ROcp13_234*C36+ROcp13_535*S36;
    ROcp13_336 = ROcp13_334*C36+ROcp13_635*S36;
    ROcp13_436 = -(ROcp13_134*S36-ROcp13_435*C36);
    ROcp13_536 = -(ROcp13_234*S36-ROcp13_535*C36);
    ROcp13_636 = -(ROcp13_334*S36-ROcp13_635*C36);
    ROcp13_137 = ROcp13_136*C37-ROcp13_735*S37;
    ROcp13_237 = ROcp13_236*C37-ROcp13_835*S37;
    ROcp13_337 = ROcp13_336*C37-ROcp13_935*S37;
    ROcp13_737 = ROcp13_136*S37+ROcp13_735*C37;
    ROcp13_837 = ROcp13_236*S37+ROcp13_835*C37;
    ROcp13_937 = ROcp13_336*S37+ROcp13_935*C37;
    ROcp13_438 = ROcp13_436*C38+ROcp13_737*S38;
    ROcp13_538 = ROcp13_536*C38+ROcp13_837*S38;
    ROcp13_638 = ROcp13_636*C38+ROcp13_937*S38;
    ROcp13_738 = -(ROcp13_436*S38-ROcp13_737*C38);
    ROcp13_838 = -(ROcp13_536*S38-ROcp13_837*C38);
    ROcp13_938 = -(ROcp13_636*S38-ROcp13_937*C38);
    ROcp13_139 = ROcp13_137*C39+ROcp13_438*S39;
    ROcp13_239 = ROcp13_237*C39+ROcp13_538*S39;
    ROcp13_339 = ROcp13_337*C39+ROcp13_638*S39;
    ROcp13_439 = -(ROcp13_137*S39-ROcp13_438*C39);
    ROcp13_539 = -(ROcp13_237*S39-ROcp13_538*C39);
    ROcp13_639 = -(ROcp13_337*S39-ROcp13_638*C39);
    ROcp13_140 = ROcp13_139*C40-ROcp13_738*S40;
    ROcp13_240 = ROcp13_239*C40-ROcp13_838*S40;
    ROcp13_340 = ROcp13_339*C40-ROcp13_938*S40;
    ROcp13_740 = ROcp13_139*S40+ROcp13_738*C40;
    ROcp13_840 = ROcp13_239*S40+ROcp13_838*C40;
    ROcp13_940 = ROcp13_339*S40+ROcp13_938*C40;
    ROcp13_141 = ROcp13_140*C41+ROcp13_439*S41;
    ROcp13_241 = ROcp13_240*C41+ROcp13_539*S41;
    ROcp13_341 = ROcp13_340*C41+ROcp13_639*S41;
    ROcp13_441 = -(ROcp13_140*S41-ROcp13_439*C41);
    ROcp13_541 = -(ROcp13_240*S41-ROcp13_539*C41);
    ROcp13_641 = -(ROcp13_340*S41-ROcp13_639*C41);
    ROcp13_142 = ROcp13_141*C42-ROcp13_740*S42;
    ROcp13_242 = ROcp13_241*C42-ROcp13_840*S42;
    ROcp13_342 = ROcp13_341*C42-ROcp13_940*S42;
    ROcp13_742 = ROcp13_141*S42+ROcp13_740*C42;
    ROcp13_842 = ROcp13_241*S42+ROcp13_840*C42;
    ROcp13_942 = ROcp13_341*S42+ROcp13_940*C42;
    ROcp13_443 = ROcp13_441*C43+ROcp13_742*S43;
    ROcp13_543 = ROcp13_541*C43+ROcp13_842*S43;
    ROcp13_643 = ROcp13_641*C43+ROcp13_942*S43;
    ROcp13_743 = -(ROcp13_441*S43-ROcp13_742*C43);
    ROcp13_843 = -(ROcp13_541*S43-ROcp13_842*C43);
    ROcp13_943 = -(ROcp13_641*S43-ROcp13_942*C43);
    RLcp13_135 = ROcp13_134*s->dpt[1][23]+ROcp13_434*s->dpt[2][23]+ROcp13_733*s->dpt[3][23];
    RLcp13_235 = ROcp13_234*s->dpt[1][23]+ROcp13_534*s->dpt[2][23]+ROcp13_833*s->dpt[3][23];
    RLcp13_335 = ROcp13_334*s->dpt[1][23]+ROcp13_634*s->dpt[2][23]+ROcp13_933*s->dpt[3][23];
    ORcp13_135 = OMcp13_234*RLcp13_335-OMcp13_334*RLcp13_235;
    ORcp13_235 = -(OMcp13_134*RLcp13_335-OMcp13_334*RLcp13_135);
    ORcp13_335 = OMcp13_134*RLcp13_235-OMcp13_234*RLcp13_135;
    OMcp13_137 = OMcp13_134+ROcp13_436*qd[37];
    OMcp13_237 = OMcp13_234+ROcp13_536*qd[37];
    OMcp13_337 = OMcp13_334+ROcp13_636*qd[37];
    OPcp13_137 = OPcp13_134+ROcp13_436*qdd[37]+qd[37]*(OMcp13_234*ROcp13_636-OMcp13_334*ROcp13_536);
    OPcp13_237 = OPcp13_234+ROcp13_536*qdd[37]-qd[37]*(OMcp13_134*ROcp13_636-OMcp13_334*ROcp13_436);
    OPcp13_337 = OPcp13_334+ROcp13_636*qdd[37]+qd[37]*(OMcp13_134*ROcp13_536-OMcp13_234*ROcp13_436);
    RLcp13_138 = ROcp13_436*s->dpt[2][27]+ROcp13_737*s->dpt[3][27];
    RLcp13_238 = ROcp13_536*s->dpt[2][27]+ROcp13_837*s->dpt[3][27];
    RLcp13_338 = ROcp13_636*s->dpt[2][27]+ROcp13_937*s->dpt[3][27];
    OMcp13_138 = OMcp13_137+ROcp13_137*qd[38];
    OMcp13_238 = OMcp13_237+ROcp13_237*qd[38];
    OMcp13_338 = OMcp13_337+ROcp13_337*qd[38];
    ORcp13_138 = OMcp13_237*RLcp13_338-OMcp13_337*RLcp13_238;
    ORcp13_238 = -(OMcp13_137*RLcp13_338-OMcp13_337*RLcp13_138);
    ORcp13_338 = OMcp13_137*RLcp13_238-OMcp13_237*RLcp13_138;
    OPcp13_138 = OPcp13_137+ROcp13_137*qdd[38]+qd[38]*(OMcp13_237*ROcp13_337-OMcp13_337*ROcp13_237);
    OPcp13_238 = OPcp13_237+ROcp13_237*qdd[38]-qd[38]*(OMcp13_137*ROcp13_337-OMcp13_337*ROcp13_137);
    OPcp13_338 = OPcp13_337+ROcp13_337*qdd[38]+qd[38]*(OMcp13_137*ROcp13_237-OMcp13_237*ROcp13_137);
    RLcp13_139 = ROcp13_738*s->dpt[3][29];
    RLcp13_239 = ROcp13_838*s->dpt[3][29];
    RLcp13_339 = ROcp13_938*s->dpt[3][29];
    OMcp13_139 = OMcp13_138+ROcp13_738*qd[39];
    OMcp13_239 = OMcp13_238+ROcp13_838*qd[39];
    OMcp13_339 = OMcp13_338+ROcp13_938*qd[39];
    ORcp13_139 = OMcp13_238*RLcp13_339-OMcp13_338*RLcp13_239;
    ORcp13_239 = -(OMcp13_138*RLcp13_339-OMcp13_338*RLcp13_139);
    ORcp13_339 = OMcp13_138*RLcp13_239-OMcp13_238*RLcp13_139;
    OPcp13_139 = OPcp13_138+ROcp13_738*qdd[39]+qd[39]*(OMcp13_238*ROcp13_938-OMcp13_338*ROcp13_838);
    OPcp13_239 = OPcp13_238+ROcp13_838*qdd[39]-qd[39]*(OMcp13_138*ROcp13_938-OMcp13_338*ROcp13_738);
    OPcp13_339 = OPcp13_338+ROcp13_938*qdd[39]+qd[39]*(OMcp13_138*ROcp13_838-OMcp13_238*ROcp13_738);
    RLcp13_140 = ROcp13_139*s->dpt[1][31]+ROcp13_738*s->dpt[3][31];
    RLcp13_240 = ROcp13_239*s->dpt[1][31]+ROcp13_838*s->dpt[3][31];
    RLcp13_340 = ROcp13_339*s->dpt[1][31]+ROcp13_938*s->dpt[3][31];
    OMcp13_140 = OMcp13_139+ROcp13_439*qd[40];
    OMcp13_240 = OMcp13_239+ROcp13_539*qd[40];
    OMcp13_340 = OMcp13_339+ROcp13_639*qd[40];
    ORcp13_140 = OMcp13_239*RLcp13_340-OMcp13_339*RLcp13_240;
    ORcp13_240 = -(OMcp13_139*RLcp13_340-OMcp13_339*RLcp13_140);
    ORcp13_340 = OMcp13_139*RLcp13_240-OMcp13_239*RLcp13_140;
    OPcp13_140 = OPcp13_139+ROcp13_439*qdd[40]+qd[40]*(OMcp13_239*ROcp13_639-OMcp13_339*ROcp13_539);
    OPcp13_240 = OPcp13_239+ROcp13_539*qdd[40]-qd[40]*(OMcp13_139*ROcp13_639-OMcp13_339*ROcp13_439);
    OPcp13_340 = OPcp13_339+ROcp13_639*qdd[40]+qd[40]*(OMcp13_139*ROcp13_539-OMcp13_239*ROcp13_439);
    RLcp13_141 = ROcp13_140*s->dpt[1][33]+ROcp13_740*s->dpt[3][33];
    RLcp13_241 = ROcp13_240*s->dpt[1][33]+ROcp13_840*s->dpt[3][33];
    RLcp13_341 = ROcp13_340*s->dpt[1][33]+ROcp13_940*s->dpt[3][33];
    OMcp13_141 = OMcp13_140+ROcp13_740*qd[41];
    OMcp13_241 = OMcp13_240+ROcp13_840*qd[41];
    OMcp13_341 = OMcp13_340+ROcp13_940*qd[41];
    ORcp13_141 = OMcp13_240*RLcp13_341-OMcp13_340*RLcp13_241;
    ORcp13_241 = -(OMcp13_140*RLcp13_341-OMcp13_340*RLcp13_141);
    ORcp13_341 = OMcp13_140*RLcp13_241-OMcp13_240*RLcp13_141;
    OMcp13_142 = OMcp13_141+ROcp13_441*qd[42];
    OMcp13_242 = OMcp13_241+ROcp13_541*qd[42];
    OMcp13_342 = OMcp13_341+ROcp13_641*qd[42];
    OPcp13_142 = OPcp13_140+ROcp13_441*qdd[42]+ROcp13_740*qdd[41]+qd[41]*(OMcp13_240*ROcp13_940-OMcp13_340*ROcp13_840)+
 qd[42]*(OMcp13_241*ROcp13_641-OMcp13_341*ROcp13_541);
    OPcp13_242 = OPcp13_240+ROcp13_541*qdd[42]+ROcp13_840*qdd[41]-qd[41]*(OMcp13_140*ROcp13_940-OMcp13_340*ROcp13_740)-
 qd[42]*(OMcp13_141*ROcp13_641-OMcp13_341*ROcp13_441);
    OPcp13_342 = OPcp13_340+ROcp13_641*qdd[42]+ROcp13_940*qdd[41]+qd[41]*(OMcp13_140*ROcp13_840-OMcp13_240*ROcp13_740)+
 qd[42]*(OMcp13_141*ROcp13_541-OMcp13_241*ROcp13_441);
    RLcp13_143 = ROcp13_742*s->dpt[3][37];
    RLcp13_243 = ROcp13_842*s->dpt[3][37];
    RLcp13_343 = ROcp13_942*s->dpt[3][37];
    OMcp13_143 = OMcp13_142+ROcp13_142*qd[43];
    OMcp13_243 = OMcp13_242+ROcp13_242*qd[43];
    OMcp13_343 = OMcp13_342+ROcp13_342*qd[43];
    ORcp13_143 = OMcp13_242*RLcp13_343-OMcp13_342*RLcp13_243;
    ORcp13_243 = -(OMcp13_142*RLcp13_343-OMcp13_342*RLcp13_143);
    ORcp13_343 = OMcp13_142*RLcp13_243-OMcp13_242*RLcp13_143;
    OPcp13_143 = OPcp13_142+ROcp13_142*qdd[43]+qd[43]*(OMcp13_242*ROcp13_342-OMcp13_342*ROcp13_242);
    OPcp13_243 = OPcp13_242+ROcp13_242*qdd[43]-qd[43]*(OMcp13_142*ROcp13_342-OMcp13_342*ROcp13_142);
    OPcp13_343 = OPcp13_342+ROcp13_342*qdd[43]+qd[43]*(OMcp13_142*ROcp13_242-OMcp13_242*ROcp13_142);
    RLcp13_169 = ROcp13_743*s->dpt[3][40];
    RLcp13_269 = ROcp13_843*s->dpt[3][40];
    RLcp13_369 = ROcp13_943*s->dpt[3][40];
    POcp13_169 = RLcp13_131+RLcp13_134+RLcp13_135+RLcp13_138+RLcp13_139+RLcp13_140+RLcp13_141+RLcp13_143+RLcp13_169+q[1];
    POcp13_269 = RLcp13_231+RLcp13_234+RLcp13_235+RLcp13_238+RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243+RLcp13_269+q[2];
    POcp13_369 = RLcp13_331+RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369+q[3];
    JTcp13_269_4 = -(RLcp13_331+RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369);
    JTcp13_369_4 = RLcp13_231+RLcp13_234+RLcp13_235+RLcp13_238+RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243+RLcp13_269;
    JTcp13_169_5 = C4*(RLcp13_331+RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343)-S4*(
 RLcp13_231+RLcp13_234)-S4*(RLcp13_235+RLcp13_238)-S4*(RLcp13_239+RLcp13_240)-S4*(RLcp13_241+RLcp13_243)-RLcp13_269*S4+
 RLcp13_369*C4;
    JTcp13_269_5 = S4*(RLcp13_131+RLcp13_134+RLcp13_135+RLcp13_138+RLcp13_139+RLcp13_140+RLcp13_141+RLcp13_143+RLcp13_169);
    JTcp13_369_5 = -C4*(RLcp13_131+RLcp13_134+RLcp13_135+RLcp13_138+RLcp13_139+RLcp13_140+RLcp13_141+RLcp13_143+RLcp13_169
 );
    JTcp13_169_6 = ROcp13_85*(RLcp13_331+RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343)-
 ROcp13_95*(RLcp13_231+RLcp13_234)-ROcp13_95*(RLcp13_235+RLcp13_238)-ROcp13_95*(RLcp13_239+RLcp13_240)-ROcp13_95*(RLcp13_241+
 RLcp13_243)-RLcp13_269*ROcp13_95+RLcp13_369*ROcp13_85;
    JTcp13_269_6 = -(RLcp13_369*S5-ROcp13_95*(RLcp13_131+RLcp13_134+RLcp13_135+RLcp13_138+RLcp13_139+RLcp13_140+RLcp13_141
 +RLcp13_143+RLcp13_169)+S5*(RLcp13_331+RLcp13_334)+S5*(RLcp13_335+RLcp13_338)+S5*(RLcp13_339+RLcp13_340)+S5*(RLcp13_341+
 RLcp13_343));
    JTcp13_369_6 = RLcp13_269*S5-ROcp13_85*(RLcp13_131+RLcp13_134+RLcp13_135+RLcp13_138+RLcp13_139+RLcp13_140+RLcp13_141+
 RLcp13_143+RLcp13_169)+S5*(RLcp13_231+RLcp13_234)+S5*(RLcp13_235+RLcp13_238)+S5*(RLcp13_239+RLcp13_240)+S5*(RLcp13_241+
 RLcp13_243);
    JTcp13_169_7 = ROcp13_56*(RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-
 ROcp13_66*(RLcp13_234+RLcp13_235)-ROcp13_66*(RLcp13_238+RLcp13_239)-ROcp13_66*(RLcp13_240+RLcp13_241)-ROcp13_66*(RLcp13_243+
 RLcp13_269);
    JTcp13_269_7 = -(ROcp13_46*(RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-
 ROcp13_66*(RLcp13_134+RLcp13_135)-ROcp13_66*(RLcp13_138+RLcp13_139)-ROcp13_66*(RLcp13_140+RLcp13_141)-ROcp13_66*(RLcp13_143+
 RLcp13_169));
    JTcp13_369_7 = ROcp13_46*(RLcp13_234+RLcp13_235+RLcp13_238+RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243+RLcp13_269)-
 ROcp13_56*(RLcp13_134+RLcp13_135)-ROcp13_56*(RLcp13_138+RLcp13_139)-ROcp13_56*(RLcp13_140+RLcp13_141)-ROcp13_56*(RLcp13_143+
 RLcp13_169);
    JTcp13_169_8 = ROcp13_231*(RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-
 ROcp13_331*(RLcp13_234+RLcp13_235)-ROcp13_331*(RLcp13_238+RLcp13_239)-ROcp13_331*(RLcp13_240+RLcp13_241)-ROcp13_331*(
 RLcp13_243+RLcp13_269);
    JTcp13_269_8 = -(ROcp13_131*(RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-
 ROcp13_331*(RLcp13_134+RLcp13_135)-ROcp13_331*(RLcp13_138+RLcp13_139)-ROcp13_331*(RLcp13_140+RLcp13_141)-ROcp13_331*(
 RLcp13_143+RLcp13_169));
    JTcp13_369_8 = ROcp13_131*(RLcp13_234+RLcp13_235+RLcp13_238+RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243+RLcp13_269)-
 ROcp13_231*(RLcp13_134+RLcp13_135)-ROcp13_231*(RLcp13_138+RLcp13_139)-ROcp13_231*(RLcp13_140+RLcp13_141)-ROcp13_231*(
 RLcp13_143+RLcp13_169);
    JTcp13_169_9 = ROcp13_532*(RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-
 ROcp13_632*(RLcp13_234+RLcp13_235)-ROcp13_632*(RLcp13_238+RLcp13_239)-ROcp13_632*(RLcp13_240+RLcp13_241)-ROcp13_632*(
 RLcp13_243+RLcp13_269);
    JTcp13_269_9 = -(ROcp13_432*(RLcp13_334+RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-
 ROcp13_632*(RLcp13_134+RLcp13_135)-ROcp13_632*(RLcp13_138+RLcp13_139)-ROcp13_632*(RLcp13_140+RLcp13_141)-ROcp13_632*(
 RLcp13_143+RLcp13_169));
    JTcp13_369_9 = ROcp13_432*(RLcp13_234+RLcp13_235+RLcp13_238+RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243+RLcp13_269)-
 ROcp13_532*(RLcp13_134+RLcp13_135)-ROcp13_532*(RLcp13_138+RLcp13_139)-ROcp13_532*(RLcp13_140+RLcp13_141)-ROcp13_532*(
 RLcp13_143+RLcp13_169);
    JTcp13_169_10 = ROcp13_833*(RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343)-ROcp13_933*(RLcp13_235+
 RLcp13_238)-ROcp13_933*(RLcp13_239+RLcp13_240)-ROcp13_933*(RLcp13_241+RLcp13_243)-RLcp13_269*ROcp13_933+RLcp13_369*
 ROcp13_833;
    JTcp13_269_10 = RLcp13_169*ROcp13_933-RLcp13_369*ROcp13_733-ROcp13_733*(RLcp13_335+RLcp13_338+RLcp13_339+RLcp13_340+
 RLcp13_341+RLcp13_343)+ROcp13_933*(RLcp13_135+RLcp13_138)+ROcp13_933*(RLcp13_139+RLcp13_140)+ROcp13_933*(RLcp13_141+
 RLcp13_143);
    JTcp13_369_10 = ROcp13_733*(RLcp13_235+RLcp13_238+RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243)-ROcp13_833*(RLcp13_135+
 RLcp13_138)-ROcp13_833*(RLcp13_139+RLcp13_140)-ROcp13_833*(RLcp13_141+RLcp13_143)-RLcp13_169*ROcp13_833+RLcp13_269*
 ROcp13_733;
    JTcp13_169_11 = ROcp13_234*(RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-ROcp13_334*(RLcp13_238+
 RLcp13_239)-ROcp13_334*(RLcp13_240+RLcp13_241)-ROcp13_334*(RLcp13_243+RLcp13_269);
    JTcp13_269_11 = -(ROcp13_134*(RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-ROcp13_334*(
 RLcp13_138+RLcp13_139)-ROcp13_334*(RLcp13_140+RLcp13_141)-ROcp13_334*(RLcp13_143+RLcp13_169));
    JTcp13_369_11 = ROcp13_134*(RLcp13_238+RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243+RLcp13_269)-ROcp13_234*(RLcp13_138+
 RLcp13_139)-ROcp13_234*(RLcp13_140+RLcp13_141)-ROcp13_234*(RLcp13_143+RLcp13_169);
    JTcp13_169_12 = ROcp13_835*(RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-ROcp13_935*(RLcp13_238+
 RLcp13_239)-ROcp13_935*(RLcp13_240+RLcp13_241)-ROcp13_935*(RLcp13_243+RLcp13_269);
    JTcp13_269_12 = -(ROcp13_735*(RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-ROcp13_935*(
 RLcp13_138+RLcp13_139)-ROcp13_935*(RLcp13_140+RLcp13_141)-ROcp13_935*(RLcp13_143+RLcp13_169));
    JTcp13_369_12 = ROcp13_735*(RLcp13_238+RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243+RLcp13_269)-ROcp13_835*(RLcp13_138+
 RLcp13_139)-ROcp13_835*(RLcp13_140+RLcp13_141)-ROcp13_835*(RLcp13_143+RLcp13_169);
    JTcp13_169_13 = ROcp13_536*(RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-ROcp13_636*(RLcp13_238+
 RLcp13_239)-ROcp13_636*(RLcp13_240+RLcp13_241)-ROcp13_636*(RLcp13_243+RLcp13_269);
    JTcp13_269_13 = -(ROcp13_436*(RLcp13_338+RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-ROcp13_636*(
 RLcp13_138+RLcp13_139)-ROcp13_636*(RLcp13_140+RLcp13_141)-ROcp13_636*(RLcp13_143+RLcp13_169));
    JTcp13_369_13 = ROcp13_436*(RLcp13_238+RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243+RLcp13_269)-ROcp13_536*(RLcp13_138+
 RLcp13_139)-ROcp13_536*(RLcp13_140+RLcp13_141)-ROcp13_536*(RLcp13_143+RLcp13_169);
    JTcp13_169_14 = ROcp13_237*(RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343)-ROcp13_337*(RLcp13_239+RLcp13_240)-ROcp13_337
 *(RLcp13_241+RLcp13_243)-RLcp13_269*ROcp13_337+RLcp13_369*ROcp13_237;
    JTcp13_269_14 = RLcp13_169*ROcp13_337-RLcp13_369*ROcp13_137-ROcp13_137*(RLcp13_339+RLcp13_340+RLcp13_341+RLcp13_343)+
 ROcp13_337*(RLcp13_139+RLcp13_140)+ROcp13_337*(RLcp13_141+RLcp13_143);
    JTcp13_369_14 = ROcp13_137*(RLcp13_239+RLcp13_240+RLcp13_241+RLcp13_243)-ROcp13_237*(RLcp13_139+RLcp13_140)-ROcp13_237
 *(RLcp13_141+RLcp13_143)-RLcp13_169*ROcp13_237+RLcp13_269*ROcp13_137;
    JTcp13_169_15 = ROcp13_838*(RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-ROcp13_938*(RLcp13_240+RLcp13_241)-ROcp13_938
 *(RLcp13_243+RLcp13_269);
    JTcp13_269_15 = -(ROcp13_738*(RLcp13_340+RLcp13_341+RLcp13_343+RLcp13_369)-ROcp13_938*(RLcp13_140+RLcp13_141)-
 ROcp13_938*(RLcp13_143+RLcp13_169));
    JTcp13_369_15 = ROcp13_738*(RLcp13_240+RLcp13_241+RLcp13_243+RLcp13_269)-ROcp13_838*(RLcp13_140+RLcp13_141)-ROcp13_838
 *(RLcp13_143+RLcp13_169);
    JTcp13_169_16 = ROcp13_539*(RLcp13_341+RLcp13_343)-ROcp13_639*(RLcp13_241+RLcp13_243)-RLcp13_269*ROcp13_639+RLcp13_369
 *ROcp13_539;
    JTcp13_269_16 = RLcp13_169*ROcp13_639-RLcp13_369*ROcp13_439-ROcp13_439*(RLcp13_341+RLcp13_343)+ROcp13_639*(RLcp13_141+
 RLcp13_143);
    JTcp13_369_16 = ROcp13_439*(RLcp13_241+RLcp13_243)-ROcp13_539*(RLcp13_141+RLcp13_143)-RLcp13_169*ROcp13_539+RLcp13_269
 *ROcp13_439;
    JTcp13_169_17 = ROcp13_840*(RLcp13_343+RLcp13_369)-ROcp13_940*(RLcp13_243+RLcp13_269);
    JTcp13_269_17 = -(ROcp13_740*(RLcp13_343+RLcp13_369)-ROcp13_940*(RLcp13_143+RLcp13_169));
    JTcp13_369_17 = ROcp13_740*(RLcp13_243+RLcp13_269)-ROcp13_840*(RLcp13_143+RLcp13_169);
    JTcp13_169_18 = ROcp13_541*(RLcp13_343+RLcp13_369)-ROcp13_641*(RLcp13_243+RLcp13_269);
    JTcp13_269_18 = -(ROcp13_441*(RLcp13_343+RLcp13_369)-ROcp13_641*(RLcp13_143+RLcp13_169));
    JTcp13_369_18 = ROcp13_441*(RLcp13_243+RLcp13_269)-ROcp13_541*(RLcp13_143+RLcp13_169);
    JTcp13_169_19 = -(RLcp13_269*ROcp13_342-RLcp13_369*ROcp13_242);
    JTcp13_269_19 = RLcp13_169*ROcp13_342-RLcp13_369*ROcp13_142;
    JTcp13_369_19 = -(RLcp13_169*ROcp13_242-RLcp13_269*ROcp13_142);
    ORcp13_169 = OMcp13_243*RLcp13_369-OMcp13_343*RLcp13_269;
    ORcp13_269 = -(OMcp13_143*RLcp13_369-OMcp13_343*RLcp13_169);
    ORcp13_369 = OMcp13_143*RLcp13_269-OMcp13_243*RLcp13_169;
    VIcp13_169 = ORcp13_131+ORcp13_134+ORcp13_135+ORcp13_138+ORcp13_139+ORcp13_140+ORcp13_141+ORcp13_143+ORcp13_169+qd[1];
    VIcp13_269 = ORcp13_231+ORcp13_234+ORcp13_235+ORcp13_238+ORcp13_239+ORcp13_240+ORcp13_241+ORcp13_243+ORcp13_269+qd[2];
    VIcp13_369 = ORcp13_331+ORcp13_334+ORcp13_335+ORcp13_338+ORcp13_339+ORcp13_340+ORcp13_341+ORcp13_343+ORcp13_369+qd[3];
    ACcp13_169 = qdd[1]+OMcp13_233*ORcp13_334+OMcp13_234*ORcp13_335+OMcp13_237*ORcp13_338+OMcp13_238*ORcp13_339+OMcp13_239
 *ORcp13_340+OMcp13_240*ORcp13_341+OMcp13_242*ORcp13_343+OMcp13_243*ORcp13_369+OMcp13_26*ORcp13_331-OMcp13_333*ORcp13_234-
 OMcp13_334*ORcp13_235-OMcp13_337*ORcp13_238-OMcp13_338*ORcp13_239-OMcp13_339*ORcp13_240-OMcp13_340*ORcp13_241-OMcp13_342*
 ORcp13_243-OMcp13_343*ORcp13_269-OMcp13_36*ORcp13_231+OPcp13_233*RLcp13_334+OPcp13_234*RLcp13_335+OPcp13_237*RLcp13_338+
 OPcp13_238*RLcp13_339+OPcp13_239*RLcp13_340+OPcp13_240*RLcp13_341+OPcp13_242*RLcp13_343+OPcp13_243*RLcp13_369+OPcp13_26*
 RLcp13_331-OPcp13_333*RLcp13_234-OPcp13_334*RLcp13_235-OPcp13_337*RLcp13_238-OPcp13_338*RLcp13_239-OPcp13_339*RLcp13_240-
 OPcp13_340*RLcp13_241-OPcp13_342*RLcp13_243-OPcp13_343*RLcp13_269-OPcp13_36*RLcp13_231;
    ACcp13_269 = qdd[2]-OMcp13_133*ORcp13_334-OMcp13_134*ORcp13_335-OMcp13_137*ORcp13_338-OMcp13_138*ORcp13_339-OMcp13_139
 *ORcp13_340-OMcp13_140*ORcp13_341-OMcp13_142*ORcp13_343-OMcp13_143*ORcp13_369-OMcp13_16*ORcp13_331+OMcp13_333*ORcp13_134+
 OMcp13_334*ORcp13_135+OMcp13_337*ORcp13_138+OMcp13_338*ORcp13_139+OMcp13_339*ORcp13_140+OMcp13_340*ORcp13_141+OMcp13_342*
 ORcp13_143+OMcp13_343*ORcp13_169+OMcp13_36*ORcp13_131-OPcp13_133*RLcp13_334-OPcp13_134*RLcp13_335-OPcp13_137*RLcp13_338-
 OPcp13_138*RLcp13_339-OPcp13_139*RLcp13_340-OPcp13_140*RLcp13_341-OPcp13_142*RLcp13_343-OPcp13_143*RLcp13_369-OPcp13_16*
 RLcp13_331+OPcp13_333*RLcp13_134+OPcp13_334*RLcp13_135+OPcp13_337*RLcp13_138+OPcp13_338*RLcp13_139+OPcp13_339*RLcp13_140+
 OPcp13_340*RLcp13_141+OPcp13_342*RLcp13_143+OPcp13_343*RLcp13_169+OPcp13_36*RLcp13_131;
    ACcp13_369 = qdd[3]+OMcp13_133*ORcp13_234+OMcp13_134*ORcp13_235+OMcp13_137*ORcp13_238+OMcp13_138*ORcp13_239+OMcp13_139
 *ORcp13_240+OMcp13_140*ORcp13_241+OMcp13_142*ORcp13_243+OMcp13_143*ORcp13_269+OMcp13_16*ORcp13_231-OMcp13_233*ORcp13_134-
 OMcp13_234*ORcp13_135-OMcp13_237*ORcp13_138-OMcp13_238*ORcp13_139-OMcp13_239*ORcp13_140-OMcp13_240*ORcp13_141-OMcp13_242*
 ORcp13_143-OMcp13_243*ORcp13_169-OMcp13_26*ORcp13_131+OPcp13_133*RLcp13_234+OPcp13_134*RLcp13_235+OPcp13_137*RLcp13_238+
 OPcp13_138*RLcp13_239+OPcp13_139*RLcp13_240+OPcp13_140*RLcp13_241+OPcp13_142*RLcp13_243+OPcp13_143*RLcp13_269+OPcp13_16*
 RLcp13_231-OPcp13_233*RLcp13_134-OPcp13_234*RLcp13_135-OPcp13_237*RLcp13_138-OPcp13_238*RLcp13_139-OPcp13_239*RLcp13_140-
 OPcp13_240*RLcp13_141-OPcp13_242*RLcp13_143-OPcp13_243*RLcp13_169-OPcp13_26*RLcp13_131;

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_169;
    sens->P[2] = POcp13_269;
    sens->P[3] = POcp13_369;
    sens->R[1][1] = ROcp13_142;
    sens->R[1][2] = ROcp13_242;
    sens->R[1][3] = ROcp13_342;
    sens->R[2][1] = ROcp13_443;
    sens->R[2][2] = ROcp13_543;
    sens->R[2][3] = ROcp13_643;
    sens->R[3][1] = ROcp13_743;
    sens->R[3][2] = ROcp13_843;
    sens->R[3][3] = ROcp13_943;
    sens->V[1] = VIcp13_169;
    sens->V[2] = VIcp13_269;
    sens->V[3] = VIcp13_369;
    sens->OM[1] = OMcp13_143;
    sens->OM[2] = OMcp13_243;
    sens->OM[3] = OMcp13_343;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp13_169_5;
    sens->J[1][6] = JTcp13_169_6;
    sens->J[1][31] = JTcp13_169_7;
    sens->J[1][32] = JTcp13_169_8;
    sens->J[1][33] = JTcp13_169_9;
    sens->J[1][34] = JTcp13_169_10;
    sens->J[1][35] = JTcp13_169_11;
    sens->J[1][36] = JTcp13_169_12;
    sens->J[1][37] = JTcp13_169_13;
    sens->J[1][38] = JTcp13_169_14;
    sens->J[1][39] = JTcp13_169_15;
    sens->J[1][40] = JTcp13_169_16;
    sens->J[1][41] = JTcp13_169_17;
    sens->J[1][42] = JTcp13_169_18;
    sens->J[1][43] = JTcp13_169_19;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp13_269_4;
    sens->J[2][5] = JTcp13_269_5;
    sens->J[2][6] = JTcp13_269_6;
    sens->J[2][31] = JTcp13_269_7;
    sens->J[2][32] = JTcp13_269_8;
    sens->J[2][33] = JTcp13_269_9;
    sens->J[2][34] = JTcp13_269_10;
    sens->J[2][35] = JTcp13_269_11;
    sens->J[2][36] = JTcp13_269_12;
    sens->J[2][37] = JTcp13_269_13;
    sens->J[2][38] = JTcp13_269_14;
    sens->J[2][39] = JTcp13_269_15;
    sens->J[2][40] = JTcp13_269_16;
    sens->J[2][41] = JTcp13_269_17;
    sens->J[2][42] = JTcp13_269_18;
    sens->J[2][43] = JTcp13_269_19;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp13_369_4;
    sens->J[3][5] = JTcp13_369_5;
    sens->J[3][6] = JTcp13_369_6;
    sens->J[3][31] = JTcp13_369_7;
    sens->J[3][32] = JTcp13_369_8;
    sens->J[3][33] = JTcp13_369_9;
    sens->J[3][34] = JTcp13_369_10;
    sens->J[3][35] = JTcp13_369_11;
    sens->J[3][36] = JTcp13_369_12;
    sens->J[3][37] = JTcp13_369_13;
    sens->J[3][38] = JTcp13_369_14;
    sens->J[3][39] = JTcp13_369_15;
    sens->J[3][40] = JTcp13_369_16;
    sens->J[3][41] = JTcp13_369_17;
    sens->J[3][42] = JTcp13_369_18;
    sens->J[3][43] = JTcp13_369_19;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp13_46;
    sens->J[4][32] = ROcp13_131;
    sens->J[4][33] = ROcp13_432;
    sens->J[4][34] = ROcp13_733;
    sens->J[4][35] = ROcp13_134;
    sens->J[4][36] = ROcp13_735;
    sens->J[4][37] = ROcp13_436;
    sens->J[4][38] = ROcp13_137;
    sens->J[4][39] = ROcp13_738;
    sens->J[4][40] = ROcp13_439;
    sens->J[4][41] = ROcp13_740;
    sens->J[4][42] = ROcp13_441;
    sens->J[4][43] = ROcp13_142;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp13_85;
    sens->J[5][31] = ROcp13_56;
    sens->J[5][32] = ROcp13_231;
    sens->J[5][33] = ROcp13_532;
    sens->J[5][34] = ROcp13_833;
    sens->J[5][35] = ROcp13_234;
    sens->J[5][36] = ROcp13_835;
    sens->J[5][37] = ROcp13_536;
    sens->J[5][38] = ROcp13_237;
    sens->J[5][39] = ROcp13_838;
    sens->J[5][40] = ROcp13_539;
    sens->J[5][41] = ROcp13_840;
    sens->J[5][42] = ROcp13_541;
    sens->J[5][43] = ROcp13_242;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp13_95;
    sens->J[6][31] = ROcp13_66;
    sens->J[6][32] = ROcp13_331;
    sens->J[6][33] = ROcp13_632;
    sens->J[6][34] = ROcp13_933;
    sens->J[6][35] = ROcp13_334;
    sens->J[6][36] = ROcp13_935;
    sens->J[6][37] = ROcp13_636;
    sens->J[6][38] = ROcp13_337;
    sens->J[6][39] = ROcp13_938;
    sens->J[6][40] = ROcp13_639;
    sens->J[6][41] = ROcp13_940;
    sens->J[6][42] = ROcp13_641;
    sens->J[6][43] = ROcp13_342;
    sens->A[1] = ACcp13_169;
    sens->A[2] = ACcp13_269;
    sens->A[3] = ACcp13_369;
    sens->OMP[1] = OPcp13_143;
    sens->OMP[2] = OPcp13_243;
    sens->OMP[3] = OPcp13_343;
 
// 
break;
case 15:
 


// = = Block_1_0_0_15_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp14_25 = qd[5]*C4;
    OMcp14_35 = qd[5]*S4;
    OMcp14_16 = qd[4]+qd[6]*S5;
    OMcp14_26 = OMcp14_25+ROcp14_85*qd[6];
    OMcp14_36 = OMcp14_35+ROcp14_95*qd[6];
    OPcp14_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp14_26 = ROcp14_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp14_35*S5-ROcp14_95*qd[4]);
    OPcp14_36 = ROcp14_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp14_25*S5-ROcp14_85*qd[4]);

// = = Block_1_0_0_15_0_4 = = 
 
// Sensor Kinematics 


    ROcp14_131 = ROcp14_16*C31-S31*S5;
    ROcp14_231 = ROcp14_26*C31-ROcp14_85*S31;
    ROcp14_331 = ROcp14_36*C31-ROcp14_95*S31;
    ROcp14_731 = ROcp14_16*S31+C31*S5;
    ROcp14_831 = ROcp14_26*S31+ROcp14_85*C31;
    ROcp14_931 = ROcp14_36*S31+ROcp14_95*C31;
    ROcp14_432 = ROcp14_46*C32+ROcp14_731*S32;
    ROcp14_532 = ROcp14_56*C32+ROcp14_831*S32;
    ROcp14_632 = ROcp14_66*C32+ROcp14_931*S32;
    ROcp14_732 = -(ROcp14_46*S32-ROcp14_731*C32);
    ROcp14_832 = -(ROcp14_56*S32-ROcp14_831*C32);
    ROcp14_932 = -(ROcp14_66*S32-ROcp14_931*C32);
    ROcp14_133 = ROcp14_131*C33-ROcp14_732*S33;
    ROcp14_233 = ROcp14_231*C33-ROcp14_832*S33;
    ROcp14_333 = ROcp14_331*C33-ROcp14_932*S33;
    ROcp14_733 = ROcp14_131*S33+ROcp14_732*C33;
    ROcp14_833 = ROcp14_231*S33+ROcp14_832*C33;
    ROcp14_933 = ROcp14_331*S33+ROcp14_932*C33;
    ROcp14_134 = ROcp14_133*C34+ROcp14_432*S34;
    ROcp14_234 = ROcp14_233*C34+ROcp14_532*S34;
    ROcp14_334 = ROcp14_333*C34+ROcp14_632*S34;
    ROcp14_434 = -(ROcp14_133*S34-ROcp14_432*C34);
    ROcp14_534 = -(ROcp14_233*S34-ROcp14_532*C34);
    ROcp14_634 = -(ROcp14_333*S34-ROcp14_632*C34);
    RLcp14_131 = ROcp14_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp14_231 = ROcp14_26*s->dpt[1][3]+ROcp14_85*s->dpt[3][3];
    RLcp14_331 = ROcp14_36*s->dpt[1][3]+ROcp14_95*s->dpt[3][3];
    OMcp14_131 = OMcp14_16+ROcp14_46*qd[31];
    OMcp14_231 = OMcp14_26+ROcp14_56*qd[31];
    OMcp14_331 = OMcp14_36+ROcp14_66*qd[31];
    ORcp14_131 = OMcp14_26*RLcp14_331-OMcp14_36*RLcp14_231;
    ORcp14_231 = -(OMcp14_16*RLcp14_331-OMcp14_36*RLcp14_131);
    ORcp14_331 = OMcp14_16*RLcp14_231-OMcp14_26*RLcp14_131;
    OMcp14_132 = OMcp14_131+ROcp14_131*qd[32];
    OMcp14_232 = OMcp14_231+ROcp14_231*qd[32];
    OMcp14_332 = OMcp14_331+ROcp14_331*qd[32];
    OMcp14_133 = OMcp14_132+ROcp14_432*qd[33];
    OMcp14_233 = OMcp14_232+ROcp14_532*qd[33];
    OMcp14_333 = OMcp14_332+ROcp14_632*qd[33];
    OPcp14_133 = OPcp14_16+ROcp14_131*qdd[32]+ROcp14_432*qdd[33]+ROcp14_46*qdd[31]+qd[31]*(OMcp14_26*ROcp14_66-OMcp14_36*
 ROcp14_56)+qd[32]*(OMcp14_231*ROcp14_331-OMcp14_331*ROcp14_231)+qd[33]*(OMcp14_232*ROcp14_632-OMcp14_332*ROcp14_532);
    OPcp14_233 = OPcp14_26+ROcp14_231*qdd[32]+ROcp14_532*qdd[33]+ROcp14_56*qdd[31]-qd[31]*(OMcp14_16*ROcp14_66-OMcp14_36*
 ROcp14_46)-qd[32]*(OMcp14_131*ROcp14_331-OMcp14_331*ROcp14_131)-qd[33]*(OMcp14_132*ROcp14_632-OMcp14_332*ROcp14_432);
    OPcp14_333 = OPcp14_36+ROcp14_331*qdd[32]+ROcp14_632*qdd[33]+ROcp14_66*qdd[31]+qd[31]*(OMcp14_16*ROcp14_56-OMcp14_26*
 ROcp14_46)+qd[32]*(OMcp14_131*ROcp14_231-OMcp14_231*ROcp14_131)+qd[33]*(OMcp14_132*ROcp14_532-OMcp14_232*ROcp14_432);
    RLcp14_134 = ROcp14_733*s->dpt[3][21];
    RLcp14_234 = ROcp14_833*s->dpt[3][21];
    RLcp14_334 = ROcp14_933*s->dpt[3][21];
    OMcp14_134 = OMcp14_133+ROcp14_733*qd[34];
    OMcp14_234 = OMcp14_233+ROcp14_833*qd[34];
    OMcp14_334 = OMcp14_333+ROcp14_933*qd[34];
    ORcp14_134 = OMcp14_233*RLcp14_334-OMcp14_333*RLcp14_234;
    ORcp14_234 = -(OMcp14_133*RLcp14_334-OMcp14_333*RLcp14_134);
    ORcp14_334 = OMcp14_133*RLcp14_234-OMcp14_233*RLcp14_134;
    OPcp14_134 = OPcp14_133+ROcp14_733*qdd[34]+qd[34]*(OMcp14_233*ROcp14_933-OMcp14_333*ROcp14_833);
    OPcp14_234 = OPcp14_233+ROcp14_833*qdd[34]-qd[34]*(OMcp14_133*ROcp14_933-OMcp14_333*ROcp14_733);
    OPcp14_334 = OPcp14_333+ROcp14_933*qdd[34]+qd[34]*(OMcp14_133*ROcp14_833-OMcp14_233*ROcp14_733);

// = = Block_1_0_0_15_0_6 = = 
 
// Sensor Kinematics 


    ROcp14_444 = ROcp14_434*C44+ROcp14_733*S44;
    ROcp14_544 = ROcp14_534*C44+ROcp14_833*S44;
    ROcp14_644 = ROcp14_634*C44+ROcp14_933*S44;
    ROcp14_744 = -(ROcp14_434*S44-ROcp14_733*C44);
    ROcp14_844 = -(ROcp14_534*S44-ROcp14_833*C44);
    ROcp14_944 = -(ROcp14_634*S44-ROcp14_933*C44);
    ROcp14_145 = ROcp14_134*C45+ROcp14_444*S45;
    ROcp14_245 = ROcp14_234*C45+ROcp14_544*S45;
    ROcp14_345 = ROcp14_334*C45+ROcp14_644*S45;
    ROcp14_445 = -(ROcp14_134*S45-ROcp14_444*C45);
    ROcp14_545 = -(ROcp14_234*S45-ROcp14_544*C45);
    ROcp14_645 = -(ROcp14_334*S45-ROcp14_644*C45);
    ROcp14_146 = ROcp14_145*C46-ROcp14_744*S46;
    ROcp14_246 = ROcp14_245*C46-ROcp14_844*S46;
    ROcp14_346 = ROcp14_345*C46-ROcp14_944*S46;
    ROcp14_746 = ROcp14_145*S46+ROcp14_744*C46;
    ROcp14_846 = ROcp14_245*S46+ROcp14_844*C46;
    ROcp14_946 = ROcp14_345*S46+ROcp14_944*C46;
    RLcp14_144 = ROcp14_134*s->dpt[1][24]+ROcp14_434*s->dpt[2][24]+ROcp14_733*s->dpt[3][24];
    RLcp14_244 = ROcp14_234*s->dpt[1][24]+ROcp14_534*s->dpt[2][24]+ROcp14_833*s->dpt[3][24];
    RLcp14_344 = ROcp14_334*s->dpt[1][24]+ROcp14_634*s->dpt[2][24]+ROcp14_933*s->dpt[3][24];
    ORcp14_144 = OMcp14_234*RLcp14_344-OMcp14_334*RLcp14_244;
    ORcp14_244 = -(OMcp14_134*RLcp14_344-OMcp14_334*RLcp14_144);
    ORcp14_344 = OMcp14_134*RLcp14_244-OMcp14_234*RLcp14_144;
    OMcp14_146 = OMcp14_134+ROcp14_445*qd[46];
    OMcp14_246 = OMcp14_234+ROcp14_545*qd[46];
    OMcp14_346 = OMcp14_334+ROcp14_645*qd[46];
    OPcp14_146 = OPcp14_134+ROcp14_445*qdd[46]+qd[46]*(OMcp14_234*ROcp14_645-OMcp14_334*ROcp14_545);
    OPcp14_246 = OPcp14_234+ROcp14_545*qdd[46]-qd[46]*(OMcp14_134*ROcp14_645-OMcp14_334*ROcp14_445);
    OPcp14_346 = OPcp14_334+ROcp14_645*qdd[46]+qd[46]*(OMcp14_134*ROcp14_545-OMcp14_234*ROcp14_445);
    RLcp14_170 = ROcp14_146*s->dpt[1][42]+ROcp14_445*s->dpt[2][42]+ROcp14_746*s->dpt[3][42];
    RLcp14_270 = ROcp14_246*s->dpt[1][42]+ROcp14_545*s->dpt[2][42]+ROcp14_846*s->dpt[3][42];
    RLcp14_370 = ROcp14_346*s->dpt[1][42]+ROcp14_645*s->dpt[2][42]+ROcp14_946*s->dpt[3][42];
    POcp14_170 = RLcp14_131+RLcp14_134+RLcp14_144+RLcp14_170+q[1];
    POcp14_270 = RLcp14_231+RLcp14_234+RLcp14_244+RLcp14_270+q[2];
    POcp14_370 = RLcp14_331+RLcp14_334+RLcp14_344+RLcp14_370+q[3];
    JTcp14_270_4 = -(RLcp14_331+RLcp14_334+RLcp14_344+RLcp14_370);
    JTcp14_370_4 = RLcp14_231+RLcp14_234+RLcp14_244+RLcp14_270;
    JTcp14_170_5 = C4*(RLcp14_331+RLcp14_334+RLcp14_344+RLcp14_370)-S4*(RLcp14_231+RLcp14_234)-S4*(RLcp14_244+RLcp14_270);
    JTcp14_270_5 = S4*(RLcp14_131+RLcp14_134+RLcp14_144+RLcp14_170);
    JTcp14_370_5 = -C4*(RLcp14_131+RLcp14_134+RLcp14_144+RLcp14_170);
    JTcp14_170_6 = ROcp14_85*(RLcp14_331+RLcp14_334+RLcp14_344+RLcp14_370)-ROcp14_95*(RLcp14_231+RLcp14_234)-ROcp14_95*(
 RLcp14_244+RLcp14_270);
    JTcp14_270_6 = RLcp14_170*ROcp14_95-RLcp14_344*S5-RLcp14_370*S5+ROcp14_95*(RLcp14_131+RLcp14_134+RLcp14_144)-S5*(
 RLcp14_331+RLcp14_334);
    JTcp14_370_6 = RLcp14_244*S5-ROcp14_85*(RLcp14_131+RLcp14_134+RLcp14_144)+S5*(RLcp14_231+RLcp14_234)-RLcp14_170*
 ROcp14_85+RLcp14_270*S5;
    JTcp14_170_7 = ROcp14_56*(RLcp14_334+RLcp14_344)-ROcp14_66*(RLcp14_234+RLcp14_244)-RLcp14_270*ROcp14_66+RLcp14_370*
 ROcp14_56;
    JTcp14_270_7 = RLcp14_170*ROcp14_66-RLcp14_370*ROcp14_46-ROcp14_46*(RLcp14_334+RLcp14_344)+ROcp14_66*(RLcp14_134+
 RLcp14_144);
    JTcp14_370_7 = ROcp14_46*(RLcp14_234+RLcp14_244)-ROcp14_56*(RLcp14_134+RLcp14_144)-RLcp14_170*ROcp14_56+RLcp14_270*
 ROcp14_46;
    JTcp14_170_8 = ROcp14_231*(RLcp14_334+RLcp14_344)-ROcp14_331*(RLcp14_234+RLcp14_244)-RLcp14_270*ROcp14_331+RLcp14_370*
 ROcp14_231;
    JTcp14_270_8 = RLcp14_170*ROcp14_331-RLcp14_370*ROcp14_131-ROcp14_131*(RLcp14_334+RLcp14_344)+ROcp14_331*(RLcp14_134+
 RLcp14_144);
    JTcp14_370_8 = ROcp14_131*(RLcp14_234+RLcp14_244)-ROcp14_231*(RLcp14_134+RLcp14_144)-RLcp14_170*ROcp14_231+RLcp14_270*
 ROcp14_131;
    JTcp14_170_9 = ROcp14_532*(RLcp14_334+RLcp14_344)-ROcp14_632*(RLcp14_234+RLcp14_244)-RLcp14_270*ROcp14_632+RLcp14_370*
 ROcp14_532;
    JTcp14_270_9 = RLcp14_170*ROcp14_632-RLcp14_370*ROcp14_432-ROcp14_432*(RLcp14_334+RLcp14_344)+ROcp14_632*(RLcp14_134+
 RLcp14_144);
    JTcp14_370_9 = ROcp14_432*(RLcp14_234+RLcp14_244)-ROcp14_532*(RLcp14_134+RLcp14_144)-RLcp14_170*ROcp14_532+RLcp14_270*
 ROcp14_432;
    JTcp14_170_10 = ROcp14_833*(RLcp14_344+RLcp14_370)-ROcp14_933*(RLcp14_244+RLcp14_270);
    JTcp14_270_10 = -(ROcp14_733*(RLcp14_344+RLcp14_370)-ROcp14_933*(RLcp14_144+RLcp14_170));
    JTcp14_370_10 = ROcp14_733*(RLcp14_244+RLcp14_270)-ROcp14_833*(RLcp14_144+RLcp14_170);
    JTcp14_170_11 = -(RLcp14_270*ROcp14_334-RLcp14_370*ROcp14_234);
    JTcp14_270_11 = RLcp14_170*ROcp14_334-RLcp14_370*ROcp14_134;
    JTcp14_370_11 = -(RLcp14_170*ROcp14_234-RLcp14_270*ROcp14_134);
    JTcp14_170_12 = -(RLcp14_270*ROcp14_944-RLcp14_370*ROcp14_844);
    JTcp14_270_12 = RLcp14_170*ROcp14_944-RLcp14_370*ROcp14_744;
    JTcp14_370_12 = -(RLcp14_170*ROcp14_844-RLcp14_270*ROcp14_744);
    JTcp14_170_13 = -(RLcp14_270*ROcp14_645-RLcp14_370*ROcp14_545);
    JTcp14_270_13 = RLcp14_170*ROcp14_645-RLcp14_370*ROcp14_445;
    JTcp14_370_13 = -(RLcp14_170*ROcp14_545-RLcp14_270*ROcp14_445);
    ORcp14_170 = OMcp14_246*RLcp14_370-OMcp14_346*RLcp14_270;
    ORcp14_270 = -(OMcp14_146*RLcp14_370-OMcp14_346*RLcp14_170);
    ORcp14_370 = OMcp14_146*RLcp14_270-OMcp14_246*RLcp14_170;
    VIcp14_170 = ORcp14_131+ORcp14_134+ORcp14_144+ORcp14_170+qd[1];
    VIcp14_270 = ORcp14_231+ORcp14_234+ORcp14_244+ORcp14_270+qd[2];
    VIcp14_370 = ORcp14_331+ORcp14_334+ORcp14_344+ORcp14_370+qd[3];
    ACcp14_170 = qdd[1]+OMcp14_233*ORcp14_334+OMcp14_234*ORcp14_344+OMcp14_246*ORcp14_370+OMcp14_26*ORcp14_331-OMcp14_333*
 ORcp14_234-OMcp14_334*ORcp14_244-OMcp14_346*ORcp14_270-OMcp14_36*ORcp14_231+OPcp14_233*RLcp14_334+OPcp14_234*RLcp14_344+
 OPcp14_246*RLcp14_370+OPcp14_26*RLcp14_331-OPcp14_333*RLcp14_234-OPcp14_334*RLcp14_244-OPcp14_346*RLcp14_270-OPcp14_36*
 RLcp14_231;
    ACcp14_270 = qdd[2]-OMcp14_133*ORcp14_334-OMcp14_134*ORcp14_344-OMcp14_146*ORcp14_370-OMcp14_16*ORcp14_331+OMcp14_333*
 ORcp14_134+OMcp14_334*ORcp14_144+OMcp14_346*ORcp14_170+OMcp14_36*ORcp14_131-OPcp14_133*RLcp14_334-OPcp14_134*RLcp14_344-
 OPcp14_146*RLcp14_370-OPcp14_16*RLcp14_331+OPcp14_333*RLcp14_134+OPcp14_334*RLcp14_144+OPcp14_346*RLcp14_170+OPcp14_36*
 RLcp14_131;
    ACcp14_370 = qdd[3]+OMcp14_133*ORcp14_234+OMcp14_134*ORcp14_244+OMcp14_146*ORcp14_270+OMcp14_16*ORcp14_231-OMcp14_233*
 ORcp14_134-OMcp14_234*ORcp14_144-OMcp14_246*ORcp14_170-OMcp14_26*ORcp14_131+OPcp14_133*RLcp14_234+OPcp14_134*RLcp14_244+
 OPcp14_146*RLcp14_270+OPcp14_16*RLcp14_231-OPcp14_233*RLcp14_134-OPcp14_234*RLcp14_144-OPcp14_246*RLcp14_170-OPcp14_26*
 RLcp14_131;

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp14_170;
    sens->P[2] = POcp14_270;
    sens->P[3] = POcp14_370;
    sens->R[1][1] = ROcp14_146;
    sens->R[1][2] = ROcp14_246;
    sens->R[1][3] = ROcp14_346;
    sens->R[2][1] = ROcp14_445;
    sens->R[2][2] = ROcp14_545;
    sens->R[2][3] = ROcp14_645;
    sens->R[3][1] = ROcp14_746;
    sens->R[3][2] = ROcp14_846;
    sens->R[3][3] = ROcp14_946;
    sens->V[1] = VIcp14_170;
    sens->V[2] = VIcp14_270;
    sens->V[3] = VIcp14_370;
    sens->OM[1] = OMcp14_146;
    sens->OM[2] = OMcp14_246;
    sens->OM[3] = OMcp14_346;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp14_170_5;
    sens->J[1][6] = JTcp14_170_6;
    sens->J[1][31] = JTcp14_170_7;
    sens->J[1][32] = JTcp14_170_8;
    sens->J[1][33] = JTcp14_170_9;
    sens->J[1][34] = JTcp14_170_10;
    sens->J[1][44] = JTcp14_170_11;
    sens->J[1][45] = JTcp14_170_12;
    sens->J[1][46] = JTcp14_170_13;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp14_270_4;
    sens->J[2][5] = JTcp14_270_5;
    sens->J[2][6] = JTcp14_270_6;
    sens->J[2][31] = JTcp14_270_7;
    sens->J[2][32] = JTcp14_270_8;
    sens->J[2][33] = JTcp14_270_9;
    sens->J[2][34] = JTcp14_270_10;
    sens->J[2][44] = JTcp14_270_11;
    sens->J[2][45] = JTcp14_270_12;
    sens->J[2][46] = JTcp14_270_13;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp14_370_4;
    sens->J[3][5] = JTcp14_370_5;
    sens->J[3][6] = JTcp14_370_6;
    sens->J[3][31] = JTcp14_370_7;
    sens->J[3][32] = JTcp14_370_8;
    sens->J[3][33] = JTcp14_370_9;
    sens->J[3][34] = JTcp14_370_10;
    sens->J[3][44] = JTcp14_370_11;
    sens->J[3][45] = JTcp14_370_12;
    sens->J[3][46] = JTcp14_370_13;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp14_46;
    sens->J[4][32] = ROcp14_131;
    sens->J[4][33] = ROcp14_432;
    sens->J[4][34] = ROcp14_733;
    sens->J[4][44] = ROcp14_134;
    sens->J[4][45] = ROcp14_744;
    sens->J[4][46] = ROcp14_445;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp14_85;
    sens->J[5][31] = ROcp14_56;
    sens->J[5][32] = ROcp14_231;
    sens->J[5][33] = ROcp14_532;
    sens->J[5][34] = ROcp14_833;
    sens->J[5][44] = ROcp14_234;
    sens->J[5][45] = ROcp14_844;
    sens->J[5][46] = ROcp14_545;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp14_95;
    sens->J[6][31] = ROcp14_66;
    sens->J[6][32] = ROcp14_331;
    sens->J[6][33] = ROcp14_632;
    sens->J[6][34] = ROcp14_933;
    sens->J[6][44] = ROcp14_334;
    sens->J[6][45] = ROcp14_944;
    sens->J[6][46] = ROcp14_645;
    sens->A[1] = ACcp14_170;
    sens->A[2] = ACcp14_270;
    sens->A[3] = ACcp14_370;
    sens->OMP[1] = OPcp14_146;
    sens->OMP[2] = OPcp14_246;
    sens->OMP[3] = OPcp14_346;
 
// 
break;
case 16:
 


// = = Block_1_0_0_16_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp15_25 = qd[5]*C4;
    OMcp15_35 = qd[5]*S4;
    OMcp15_16 = qd[4]+qd[6]*S5;
    OMcp15_26 = OMcp15_25+ROcp15_85*qd[6];
    OMcp15_36 = OMcp15_35+ROcp15_95*qd[6];
    OPcp15_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp15_26 = ROcp15_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp15_35*S5-ROcp15_95*qd[4]);
    OPcp15_36 = ROcp15_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp15_25*S5-ROcp15_85*qd[4]);

// = = Block_1_0_0_16_0_4 = = 
 
// Sensor Kinematics 


    ROcp15_131 = ROcp15_16*C31-S31*S5;
    ROcp15_231 = ROcp15_26*C31-ROcp15_85*S31;
    ROcp15_331 = ROcp15_36*C31-ROcp15_95*S31;
    ROcp15_731 = ROcp15_16*S31+C31*S5;
    ROcp15_831 = ROcp15_26*S31+ROcp15_85*C31;
    ROcp15_931 = ROcp15_36*S31+ROcp15_95*C31;
    ROcp15_432 = ROcp15_46*C32+ROcp15_731*S32;
    ROcp15_532 = ROcp15_56*C32+ROcp15_831*S32;
    ROcp15_632 = ROcp15_66*C32+ROcp15_931*S32;
    ROcp15_732 = -(ROcp15_46*S32-ROcp15_731*C32);
    ROcp15_832 = -(ROcp15_56*S32-ROcp15_831*C32);
    ROcp15_932 = -(ROcp15_66*S32-ROcp15_931*C32);
    ROcp15_133 = ROcp15_131*C33-ROcp15_732*S33;
    ROcp15_233 = ROcp15_231*C33-ROcp15_832*S33;
    ROcp15_333 = ROcp15_331*C33-ROcp15_932*S33;
    ROcp15_733 = ROcp15_131*S33+ROcp15_732*C33;
    ROcp15_833 = ROcp15_231*S33+ROcp15_832*C33;
    ROcp15_933 = ROcp15_331*S33+ROcp15_932*C33;
    ROcp15_134 = ROcp15_133*C34+ROcp15_432*S34;
    ROcp15_234 = ROcp15_233*C34+ROcp15_532*S34;
    ROcp15_334 = ROcp15_333*C34+ROcp15_632*S34;
    ROcp15_434 = -(ROcp15_133*S34-ROcp15_432*C34);
    ROcp15_534 = -(ROcp15_233*S34-ROcp15_532*C34);
    ROcp15_634 = -(ROcp15_333*S34-ROcp15_632*C34);
    RLcp15_131 = ROcp15_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp15_231 = ROcp15_26*s->dpt[1][3]+ROcp15_85*s->dpt[3][3];
    RLcp15_331 = ROcp15_36*s->dpt[1][3]+ROcp15_95*s->dpt[3][3];
    OMcp15_131 = OMcp15_16+ROcp15_46*qd[31];
    OMcp15_231 = OMcp15_26+ROcp15_56*qd[31];
    OMcp15_331 = OMcp15_36+ROcp15_66*qd[31];
    ORcp15_131 = OMcp15_26*RLcp15_331-OMcp15_36*RLcp15_231;
    ORcp15_231 = -(OMcp15_16*RLcp15_331-OMcp15_36*RLcp15_131);
    ORcp15_331 = OMcp15_16*RLcp15_231-OMcp15_26*RLcp15_131;
    OMcp15_132 = OMcp15_131+ROcp15_131*qd[32];
    OMcp15_232 = OMcp15_231+ROcp15_231*qd[32];
    OMcp15_332 = OMcp15_331+ROcp15_331*qd[32];
    OMcp15_133 = OMcp15_132+ROcp15_432*qd[33];
    OMcp15_233 = OMcp15_232+ROcp15_532*qd[33];
    OMcp15_333 = OMcp15_332+ROcp15_632*qd[33];
    OPcp15_133 = OPcp15_16+ROcp15_131*qdd[32]+ROcp15_432*qdd[33]+ROcp15_46*qdd[31]+qd[31]*(OMcp15_26*ROcp15_66-OMcp15_36*
 ROcp15_56)+qd[32]*(OMcp15_231*ROcp15_331-OMcp15_331*ROcp15_231)+qd[33]*(OMcp15_232*ROcp15_632-OMcp15_332*ROcp15_532);
    OPcp15_233 = OPcp15_26+ROcp15_231*qdd[32]+ROcp15_532*qdd[33]+ROcp15_56*qdd[31]-qd[31]*(OMcp15_16*ROcp15_66-OMcp15_36*
 ROcp15_46)-qd[32]*(OMcp15_131*ROcp15_331-OMcp15_331*ROcp15_131)-qd[33]*(OMcp15_132*ROcp15_632-OMcp15_332*ROcp15_432);
    OPcp15_333 = OPcp15_36+ROcp15_331*qdd[32]+ROcp15_632*qdd[33]+ROcp15_66*qdd[31]+qd[31]*(OMcp15_16*ROcp15_56-OMcp15_26*
 ROcp15_46)+qd[32]*(OMcp15_131*ROcp15_231-OMcp15_231*ROcp15_131)+qd[33]*(OMcp15_132*ROcp15_532-OMcp15_232*ROcp15_432);
    RLcp15_134 = ROcp15_733*s->dpt[3][21];
    RLcp15_234 = ROcp15_833*s->dpt[3][21];
    RLcp15_334 = ROcp15_933*s->dpt[3][21];
    OMcp15_134 = OMcp15_133+ROcp15_733*qd[34];
    OMcp15_234 = OMcp15_233+ROcp15_833*qd[34];
    OMcp15_334 = OMcp15_333+ROcp15_933*qd[34];
    ORcp15_134 = OMcp15_233*RLcp15_334-OMcp15_333*RLcp15_234;
    ORcp15_234 = -(OMcp15_133*RLcp15_334-OMcp15_333*RLcp15_134);
    ORcp15_334 = OMcp15_133*RLcp15_234-OMcp15_233*RLcp15_134;
    OPcp15_134 = OPcp15_133+ROcp15_733*qdd[34]+qd[34]*(OMcp15_233*ROcp15_933-OMcp15_333*ROcp15_833);
    OPcp15_234 = OPcp15_233+ROcp15_833*qdd[34]-qd[34]*(OMcp15_133*ROcp15_933-OMcp15_333*ROcp15_733);
    OPcp15_334 = OPcp15_333+ROcp15_933*qdd[34]+qd[34]*(OMcp15_133*ROcp15_833-OMcp15_233*ROcp15_733);

// = = Block_1_0_0_16_0_6 = = 
 
// Sensor Kinematics 


    ROcp15_444 = ROcp15_434*C44+ROcp15_733*S44;
    ROcp15_544 = ROcp15_534*C44+ROcp15_833*S44;
    ROcp15_644 = ROcp15_634*C44+ROcp15_933*S44;
    ROcp15_744 = -(ROcp15_434*S44-ROcp15_733*C44);
    ROcp15_844 = -(ROcp15_534*S44-ROcp15_833*C44);
    ROcp15_944 = -(ROcp15_634*S44-ROcp15_933*C44);
    ROcp15_145 = ROcp15_134*C45+ROcp15_444*S45;
    ROcp15_245 = ROcp15_234*C45+ROcp15_544*S45;
    ROcp15_345 = ROcp15_334*C45+ROcp15_644*S45;
    ROcp15_445 = -(ROcp15_134*S45-ROcp15_444*C45);
    ROcp15_545 = -(ROcp15_234*S45-ROcp15_544*C45);
    ROcp15_645 = -(ROcp15_334*S45-ROcp15_644*C45);
    ROcp15_146 = ROcp15_145*C46-ROcp15_744*S46;
    ROcp15_246 = ROcp15_245*C46-ROcp15_844*S46;
    ROcp15_346 = ROcp15_345*C46-ROcp15_944*S46;
    ROcp15_746 = ROcp15_145*S46+ROcp15_744*C46;
    ROcp15_846 = ROcp15_245*S46+ROcp15_844*C46;
    ROcp15_946 = ROcp15_345*S46+ROcp15_944*C46;
    ROcp15_447 = ROcp15_445*C47+ROcp15_746*S47;
    ROcp15_547 = ROcp15_545*C47+ROcp15_846*S47;
    ROcp15_647 = ROcp15_645*C47+ROcp15_946*S47;
    ROcp15_747 = -(ROcp15_445*S47-ROcp15_746*C47);
    ROcp15_847 = -(ROcp15_545*S47-ROcp15_846*C47);
    ROcp15_947 = -(ROcp15_645*S47-ROcp15_946*C47);
    RLcp15_144 = ROcp15_134*s->dpt[1][24]+ROcp15_434*s->dpt[2][24]+ROcp15_733*s->dpt[3][24];
    RLcp15_244 = ROcp15_234*s->dpt[1][24]+ROcp15_534*s->dpt[2][24]+ROcp15_833*s->dpt[3][24];
    RLcp15_344 = ROcp15_334*s->dpt[1][24]+ROcp15_634*s->dpt[2][24]+ROcp15_933*s->dpt[3][24];
    ORcp15_144 = OMcp15_234*RLcp15_344-OMcp15_334*RLcp15_244;
    ORcp15_244 = -(OMcp15_134*RLcp15_344-OMcp15_334*RLcp15_144);
    ORcp15_344 = OMcp15_134*RLcp15_244-OMcp15_234*RLcp15_144;
    OMcp15_146 = OMcp15_134+ROcp15_445*qd[46];
    OMcp15_246 = OMcp15_234+ROcp15_545*qd[46];
    OMcp15_346 = OMcp15_334+ROcp15_645*qd[46];
    OPcp15_146 = OPcp15_134+ROcp15_445*qdd[46]+qd[46]*(OMcp15_234*ROcp15_645-OMcp15_334*ROcp15_545);
    OPcp15_246 = OPcp15_234+ROcp15_545*qdd[46]-qd[46]*(OMcp15_134*ROcp15_645-OMcp15_334*ROcp15_445);
    OPcp15_346 = OPcp15_334+ROcp15_645*qdd[46]+qd[46]*(OMcp15_134*ROcp15_545-OMcp15_234*ROcp15_445);
    RLcp15_147 = ROcp15_445*s->dpt[2][41]+ROcp15_746*s->dpt[3][41];
    RLcp15_247 = ROcp15_545*s->dpt[2][41]+ROcp15_846*s->dpt[3][41];
    RLcp15_347 = ROcp15_645*s->dpt[2][41]+ROcp15_946*s->dpt[3][41];
    OMcp15_147 = OMcp15_146+ROcp15_146*qd[47];
    OMcp15_247 = OMcp15_246+ROcp15_246*qd[47];
    OMcp15_347 = OMcp15_346+ROcp15_346*qd[47];
    ORcp15_147 = OMcp15_246*RLcp15_347-OMcp15_346*RLcp15_247;
    ORcp15_247 = -(OMcp15_146*RLcp15_347-OMcp15_346*RLcp15_147);
    ORcp15_347 = OMcp15_146*RLcp15_247-OMcp15_246*RLcp15_147;
    OPcp15_147 = OPcp15_146+ROcp15_146*qdd[47]+qd[47]*(OMcp15_246*ROcp15_346-OMcp15_346*ROcp15_246);
    OPcp15_247 = OPcp15_246+ROcp15_246*qdd[47]-qd[47]*(OMcp15_146*ROcp15_346-OMcp15_346*ROcp15_146);
    OPcp15_347 = OPcp15_346+ROcp15_346*qdd[47]+qd[47]*(OMcp15_146*ROcp15_246-OMcp15_246*ROcp15_146);
    RLcp15_171 = ROcp15_146*s->dpt[1][44]+ROcp15_447*s->dpt[2][44]+ROcp15_747*s->dpt[3][44];
    RLcp15_271 = ROcp15_246*s->dpt[1][44]+ROcp15_547*s->dpt[2][44]+ROcp15_847*s->dpt[3][44];
    RLcp15_371 = ROcp15_346*s->dpt[1][44]+ROcp15_647*s->dpt[2][44]+ROcp15_947*s->dpt[3][44];
    POcp15_171 = RLcp15_131+RLcp15_134+RLcp15_144+RLcp15_147+RLcp15_171+q[1];
    POcp15_271 = RLcp15_231+RLcp15_234+RLcp15_244+RLcp15_247+RLcp15_271+q[2];
    POcp15_371 = RLcp15_331+RLcp15_334+RLcp15_344+RLcp15_347+RLcp15_371+q[3];
    JTcp15_271_4 = -(RLcp15_331+RLcp15_334+RLcp15_344+RLcp15_347+RLcp15_371);
    JTcp15_371_4 = RLcp15_231+RLcp15_234+RLcp15_244+RLcp15_247+RLcp15_271;
    JTcp15_171_5 = C4*(RLcp15_331+RLcp15_334+RLcp15_344+RLcp15_347)-S4*(RLcp15_231+RLcp15_234)-S4*(RLcp15_244+RLcp15_247)-
 RLcp15_271*S4+RLcp15_371*C4;
    JTcp15_271_5 = S4*(RLcp15_131+RLcp15_134+RLcp15_144+RLcp15_147+RLcp15_171);
    JTcp15_371_5 = -C4*(RLcp15_131+RLcp15_134+RLcp15_144+RLcp15_147+RLcp15_171);
    JTcp15_171_6 = ROcp15_85*(RLcp15_331+RLcp15_334+RLcp15_344+RLcp15_347)-ROcp15_95*(RLcp15_231+RLcp15_234)-ROcp15_95*(
 RLcp15_244+RLcp15_247)-RLcp15_271*ROcp15_95+RLcp15_371*ROcp15_85;
    JTcp15_271_6 = -(RLcp15_371*S5-ROcp15_95*(RLcp15_131+RLcp15_134+RLcp15_144+RLcp15_147+RLcp15_171)+S5*(RLcp15_331+
 RLcp15_334)+S5*(RLcp15_344+RLcp15_347));
    JTcp15_371_6 = RLcp15_271*S5-ROcp15_85*(RLcp15_131+RLcp15_134+RLcp15_144+RLcp15_147+RLcp15_171)+S5*(RLcp15_231+
 RLcp15_234)+S5*(RLcp15_244+RLcp15_247);
    JTcp15_171_7 = ROcp15_56*(RLcp15_334+RLcp15_344+RLcp15_347+RLcp15_371)-ROcp15_66*(RLcp15_234+RLcp15_244)-ROcp15_66*(
 RLcp15_247+RLcp15_271);
    JTcp15_271_7 = -(ROcp15_46*(RLcp15_334+RLcp15_344+RLcp15_347+RLcp15_371)-ROcp15_66*(RLcp15_134+RLcp15_144)-ROcp15_66*(
 RLcp15_147+RLcp15_171));
    JTcp15_371_7 = ROcp15_46*(RLcp15_234+RLcp15_244+RLcp15_247+RLcp15_271)-ROcp15_56*(RLcp15_134+RLcp15_144)-ROcp15_56*(
 RLcp15_147+RLcp15_171);
    JTcp15_171_8 = ROcp15_231*(RLcp15_334+RLcp15_344+RLcp15_347+RLcp15_371)-ROcp15_331*(RLcp15_234+RLcp15_244)-ROcp15_331*
 (RLcp15_247+RLcp15_271);
    JTcp15_271_8 = -(ROcp15_131*(RLcp15_334+RLcp15_344+RLcp15_347+RLcp15_371)-ROcp15_331*(RLcp15_134+RLcp15_144)-
 ROcp15_331*(RLcp15_147+RLcp15_171));
    JTcp15_371_8 = ROcp15_131*(RLcp15_234+RLcp15_244+RLcp15_247+RLcp15_271)-ROcp15_231*(RLcp15_134+RLcp15_144)-ROcp15_231*
 (RLcp15_147+RLcp15_171);
    JTcp15_171_9 = ROcp15_532*(RLcp15_334+RLcp15_344+RLcp15_347+RLcp15_371)-ROcp15_632*(RLcp15_234+RLcp15_244)-ROcp15_632*
 (RLcp15_247+RLcp15_271);
    JTcp15_271_9 = -(ROcp15_432*(RLcp15_334+RLcp15_344+RLcp15_347+RLcp15_371)-ROcp15_632*(RLcp15_134+RLcp15_144)-
 ROcp15_632*(RLcp15_147+RLcp15_171));
    JTcp15_371_9 = ROcp15_432*(RLcp15_234+RLcp15_244+RLcp15_247+RLcp15_271)-ROcp15_532*(RLcp15_134+RLcp15_144)-ROcp15_532*
 (RLcp15_147+RLcp15_171);
    JTcp15_171_10 = ROcp15_833*(RLcp15_344+RLcp15_347)-ROcp15_933*(RLcp15_244+RLcp15_247)-RLcp15_271*ROcp15_933+RLcp15_371
 *ROcp15_833;
    JTcp15_271_10 = RLcp15_171*ROcp15_933-RLcp15_371*ROcp15_733-ROcp15_733*(RLcp15_344+RLcp15_347)+ROcp15_933*(RLcp15_144+
 RLcp15_147);
    JTcp15_371_10 = ROcp15_733*(RLcp15_244+RLcp15_247)-ROcp15_833*(RLcp15_144+RLcp15_147)-RLcp15_171*ROcp15_833+RLcp15_271
 *ROcp15_733;
    JTcp15_171_11 = ROcp15_234*(RLcp15_347+RLcp15_371)-ROcp15_334*(RLcp15_247+RLcp15_271);
    JTcp15_271_11 = -(ROcp15_134*(RLcp15_347+RLcp15_371)-ROcp15_334*(RLcp15_147+RLcp15_171));
    JTcp15_371_11 = ROcp15_134*(RLcp15_247+RLcp15_271)-ROcp15_234*(RLcp15_147+RLcp15_171);
    JTcp15_171_12 = ROcp15_844*(RLcp15_347+RLcp15_371)-ROcp15_944*(RLcp15_247+RLcp15_271);
    JTcp15_271_12 = -(ROcp15_744*(RLcp15_347+RLcp15_371)-ROcp15_944*(RLcp15_147+RLcp15_171));
    JTcp15_371_12 = ROcp15_744*(RLcp15_247+RLcp15_271)-ROcp15_844*(RLcp15_147+RLcp15_171);
    JTcp15_171_13 = ROcp15_545*(RLcp15_347+RLcp15_371)-ROcp15_645*(RLcp15_247+RLcp15_271);
    JTcp15_271_13 = -(ROcp15_445*(RLcp15_347+RLcp15_371)-ROcp15_645*(RLcp15_147+RLcp15_171));
    JTcp15_371_13 = ROcp15_445*(RLcp15_247+RLcp15_271)-ROcp15_545*(RLcp15_147+RLcp15_171);
    JTcp15_171_14 = -(RLcp15_271*ROcp15_346-RLcp15_371*ROcp15_246);
    JTcp15_271_14 = RLcp15_171*ROcp15_346-RLcp15_371*ROcp15_146;
    JTcp15_371_14 = -(RLcp15_171*ROcp15_246-RLcp15_271*ROcp15_146);
    ORcp15_171 = OMcp15_247*RLcp15_371-OMcp15_347*RLcp15_271;
    ORcp15_271 = -(OMcp15_147*RLcp15_371-OMcp15_347*RLcp15_171);
    ORcp15_371 = OMcp15_147*RLcp15_271-OMcp15_247*RLcp15_171;
    VIcp15_171 = ORcp15_131+ORcp15_134+ORcp15_144+ORcp15_147+ORcp15_171+qd[1];
    VIcp15_271 = ORcp15_231+ORcp15_234+ORcp15_244+ORcp15_247+ORcp15_271+qd[2];
    VIcp15_371 = ORcp15_331+ORcp15_334+ORcp15_344+ORcp15_347+ORcp15_371+qd[3];
    ACcp15_171 = qdd[1]+OMcp15_233*ORcp15_334+OMcp15_234*ORcp15_344+OMcp15_246*ORcp15_347+OMcp15_247*ORcp15_371+OMcp15_26*
 ORcp15_331-OMcp15_333*ORcp15_234-OMcp15_334*ORcp15_244-OMcp15_346*ORcp15_247-OMcp15_347*ORcp15_271-OMcp15_36*ORcp15_231+
 OPcp15_233*RLcp15_334+OPcp15_234*RLcp15_344+OPcp15_246*RLcp15_347+OPcp15_247*RLcp15_371+OPcp15_26*RLcp15_331-OPcp15_333*
 RLcp15_234-OPcp15_334*RLcp15_244-OPcp15_346*RLcp15_247-OPcp15_347*RLcp15_271-OPcp15_36*RLcp15_231;
    ACcp15_271 = qdd[2]-OMcp15_133*ORcp15_334-OMcp15_134*ORcp15_344-OMcp15_146*ORcp15_347-OMcp15_147*ORcp15_371-OMcp15_16*
 ORcp15_331+OMcp15_333*ORcp15_134+OMcp15_334*ORcp15_144+OMcp15_346*ORcp15_147+OMcp15_347*ORcp15_171+OMcp15_36*ORcp15_131-
 OPcp15_133*RLcp15_334-OPcp15_134*RLcp15_344-OPcp15_146*RLcp15_347-OPcp15_147*RLcp15_371-OPcp15_16*RLcp15_331+OPcp15_333*
 RLcp15_134+OPcp15_334*RLcp15_144+OPcp15_346*RLcp15_147+OPcp15_347*RLcp15_171+OPcp15_36*RLcp15_131;
    ACcp15_371 = qdd[3]+OMcp15_133*ORcp15_234+OMcp15_134*ORcp15_244+OMcp15_146*ORcp15_247+OMcp15_147*ORcp15_271+OMcp15_16*
 ORcp15_231-OMcp15_233*ORcp15_134-OMcp15_234*ORcp15_144-OMcp15_246*ORcp15_147-OMcp15_247*ORcp15_171-OMcp15_26*ORcp15_131+
 OPcp15_133*RLcp15_234+OPcp15_134*RLcp15_244+OPcp15_146*RLcp15_247+OPcp15_147*RLcp15_271+OPcp15_16*RLcp15_231-OPcp15_233*
 RLcp15_134-OPcp15_234*RLcp15_144-OPcp15_246*RLcp15_147-OPcp15_247*RLcp15_171-OPcp15_26*RLcp15_131;

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp15_171;
    sens->P[2] = POcp15_271;
    sens->P[3] = POcp15_371;
    sens->R[1][1] = ROcp15_146;
    sens->R[1][2] = ROcp15_246;
    sens->R[1][3] = ROcp15_346;
    sens->R[2][1] = ROcp15_447;
    sens->R[2][2] = ROcp15_547;
    sens->R[2][3] = ROcp15_647;
    sens->R[3][1] = ROcp15_747;
    sens->R[3][2] = ROcp15_847;
    sens->R[3][3] = ROcp15_947;
    sens->V[1] = VIcp15_171;
    sens->V[2] = VIcp15_271;
    sens->V[3] = VIcp15_371;
    sens->OM[1] = OMcp15_147;
    sens->OM[2] = OMcp15_247;
    sens->OM[3] = OMcp15_347;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp15_171_5;
    sens->J[1][6] = JTcp15_171_6;
    sens->J[1][31] = JTcp15_171_7;
    sens->J[1][32] = JTcp15_171_8;
    sens->J[1][33] = JTcp15_171_9;
    sens->J[1][34] = JTcp15_171_10;
    sens->J[1][44] = JTcp15_171_11;
    sens->J[1][45] = JTcp15_171_12;
    sens->J[1][46] = JTcp15_171_13;
    sens->J[1][47] = JTcp15_171_14;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp15_271_4;
    sens->J[2][5] = JTcp15_271_5;
    sens->J[2][6] = JTcp15_271_6;
    sens->J[2][31] = JTcp15_271_7;
    sens->J[2][32] = JTcp15_271_8;
    sens->J[2][33] = JTcp15_271_9;
    sens->J[2][34] = JTcp15_271_10;
    sens->J[2][44] = JTcp15_271_11;
    sens->J[2][45] = JTcp15_271_12;
    sens->J[2][46] = JTcp15_271_13;
    sens->J[2][47] = JTcp15_271_14;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp15_371_4;
    sens->J[3][5] = JTcp15_371_5;
    sens->J[3][6] = JTcp15_371_6;
    sens->J[3][31] = JTcp15_371_7;
    sens->J[3][32] = JTcp15_371_8;
    sens->J[3][33] = JTcp15_371_9;
    sens->J[3][34] = JTcp15_371_10;
    sens->J[3][44] = JTcp15_371_11;
    sens->J[3][45] = JTcp15_371_12;
    sens->J[3][46] = JTcp15_371_13;
    sens->J[3][47] = JTcp15_371_14;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp15_46;
    sens->J[4][32] = ROcp15_131;
    sens->J[4][33] = ROcp15_432;
    sens->J[4][34] = ROcp15_733;
    sens->J[4][44] = ROcp15_134;
    sens->J[4][45] = ROcp15_744;
    sens->J[4][46] = ROcp15_445;
    sens->J[4][47] = ROcp15_146;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp15_85;
    sens->J[5][31] = ROcp15_56;
    sens->J[5][32] = ROcp15_231;
    sens->J[5][33] = ROcp15_532;
    sens->J[5][34] = ROcp15_833;
    sens->J[5][44] = ROcp15_234;
    sens->J[5][45] = ROcp15_844;
    sens->J[5][46] = ROcp15_545;
    sens->J[5][47] = ROcp15_246;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp15_95;
    sens->J[6][31] = ROcp15_66;
    sens->J[6][32] = ROcp15_331;
    sens->J[6][33] = ROcp15_632;
    sens->J[6][34] = ROcp15_933;
    sens->J[6][44] = ROcp15_334;
    sens->J[6][45] = ROcp15_944;
    sens->J[6][46] = ROcp15_645;
    sens->J[6][47] = ROcp15_346;
    sens->A[1] = ACcp15_171;
    sens->A[2] = ACcp15_271;
    sens->A[3] = ACcp15_371;
    sens->OMP[1] = OPcp15_147;
    sens->OMP[2] = OPcp15_247;
    sens->OMP[3] = OPcp15_347;
 
// 
break;
case 17:
 


// = = Block_1_0_0_17_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp16_25 = qd[5]*C4;
    OMcp16_35 = qd[5]*S4;
    OMcp16_16 = qd[4]+qd[6]*S5;
    OMcp16_26 = OMcp16_25+ROcp16_85*qd[6];
    OMcp16_36 = OMcp16_35+ROcp16_95*qd[6];
    OPcp16_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp16_26 = ROcp16_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp16_35*S5-ROcp16_95*qd[4]);
    OPcp16_36 = ROcp16_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp16_25*S5-ROcp16_85*qd[4]);

// = = Block_1_0_0_17_0_4 = = 
 
// Sensor Kinematics 


    ROcp16_131 = ROcp16_16*C31-S31*S5;
    ROcp16_231 = ROcp16_26*C31-ROcp16_85*S31;
    ROcp16_331 = ROcp16_36*C31-ROcp16_95*S31;
    ROcp16_731 = ROcp16_16*S31+C31*S5;
    ROcp16_831 = ROcp16_26*S31+ROcp16_85*C31;
    ROcp16_931 = ROcp16_36*S31+ROcp16_95*C31;
    ROcp16_432 = ROcp16_46*C32+ROcp16_731*S32;
    ROcp16_532 = ROcp16_56*C32+ROcp16_831*S32;
    ROcp16_632 = ROcp16_66*C32+ROcp16_931*S32;
    ROcp16_732 = -(ROcp16_46*S32-ROcp16_731*C32);
    ROcp16_832 = -(ROcp16_56*S32-ROcp16_831*C32);
    ROcp16_932 = -(ROcp16_66*S32-ROcp16_931*C32);
    ROcp16_133 = ROcp16_131*C33-ROcp16_732*S33;
    ROcp16_233 = ROcp16_231*C33-ROcp16_832*S33;
    ROcp16_333 = ROcp16_331*C33-ROcp16_932*S33;
    ROcp16_733 = ROcp16_131*S33+ROcp16_732*C33;
    ROcp16_833 = ROcp16_231*S33+ROcp16_832*C33;
    ROcp16_933 = ROcp16_331*S33+ROcp16_932*C33;
    ROcp16_134 = ROcp16_133*C34+ROcp16_432*S34;
    ROcp16_234 = ROcp16_233*C34+ROcp16_532*S34;
    ROcp16_334 = ROcp16_333*C34+ROcp16_632*S34;
    ROcp16_434 = -(ROcp16_133*S34-ROcp16_432*C34);
    ROcp16_534 = -(ROcp16_233*S34-ROcp16_532*C34);
    ROcp16_634 = -(ROcp16_333*S34-ROcp16_632*C34);
    RLcp16_131 = ROcp16_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp16_231 = ROcp16_26*s->dpt[1][3]+ROcp16_85*s->dpt[3][3];
    RLcp16_331 = ROcp16_36*s->dpt[1][3]+ROcp16_95*s->dpt[3][3];
    OMcp16_131 = OMcp16_16+ROcp16_46*qd[31];
    OMcp16_231 = OMcp16_26+ROcp16_56*qd[31];
    OMcp16_331 = OMcp16_36+ROcp16_66*qd[31];
    ORcp16_131 = OMcp16_26*RLcp16_331-OMcp16_36*RLcp16_231;
    ORcp16_231 = -(OMcp16_16*RLcp16_331-OMcp16_36*RLcp16_131);
    ORcp16_331 = OMcp16_16*RLcp16_231-OMcp16_26*RLcp16_131;
    OMcp16_132 = OMcp16_131+ROcp16_131*qd[32];
    OMcp16_232 = OMcp16_231+ROcp16_231*qd[32];
    OMcp16_332 = OMcp16_331+ROcp16_331*qd[32];
    OMcp16_133 = OMcp16_132+ROcp16_432*qd[33];
    OMcp16_233 = OMcp16_232+ROcp16_532*qd[33];
    OMcp16_333 = OMcp16_332+ROcp16_632*qd[33];
    OPcp16_133 = OPcp16_16+ROcp16_131*qdd[32]+ROcp16_432*qdd[33]+ROcp16_46*qdd[31]+qd[31]*(OMcp16_26*ROcp16_66-OMcp16_36*
 ROcp16_56)+qd[32]*(OMcp16_231*ROcp16_331-OMcp16_331*ROcp16_231)+qd[33]*(OMcp16_232*ROcp16_632-OMcp16_332*ROcp16_532);
    OPcp16_233 = OPcp16_26+ROcp16_231*qdd[32]+ROcp16_532*qdd[33]+ROcp16_56*qdd[31]-qd[31]*(OMcp16_16*ROcp16_66-OMcp16_36*
 ROcp16_46)-qd[32]*(OMcp16_131*ROcp16_331-OMcp16_331*ROcp16_131)-qd[33]*(OMcp16_132*ROcp16_632-OMcp16_332*ROcp16_432);
    OPcp16_333 = OPcp16_36+ROcp16_331*qdd[32]+ROcp16_632*qdd[33]+ROcp16_66*qdd[31]+qd[31]*(OMcp16_16*ROcp16_56-OMcp16_26*
 ROcp16_46)+qd[32]*(OMcp16_131*ROcp16_231-OMcp16_231*ROcp16_131)+qd[33]*(OMcp16_132*ROcp16_532-OMcp16_232*ROcp16_432);
    RLcp16_134 = ROcp16_733*s->dpt[3][21];
    RLcp16_234 = ROcp16_833*s->dpt[3][21];
    RLcp16_334 = ROcp16_933*s->dpt[3][21];
    OMcp16_134 = OMcp16_133+ROcp16_733*qd[34];
    OMcp16_234 = OMcp16_233+ROcp16_833*qd[34];
    OMcp16_334 = OMcp16_333+ROcp16_933*qd[34];
    ORcp16_134 = OMcp16_233*RLcp16_334-OMcp16_333*RLcp16_234;
    ORcp16_234 = -(OMcp16_133*RLcp16_334-OMcp16_333*RLcp16_134);
    ORcp16_334 = OMcp16_133*RLcp16_234-OMcp16_233*RLcp16_134;
    OPcp16_134 = OPcp16_133+ROcp16_733*qdd[34]+qd[34]*(OMcp16_233*ROcp16_933-OMcp16_333*ROcp16_833);
    OPcp16_234 = OPcp16_233+ROcp16_833*qdd[34]-qd[34]*(OMcp16_133*ROcp16_933-OMcp16_333*ROcp16_733);
    OPcp16_334 = OPcp16_333+ROcp16_933*qdd[34]+qd[34]*(OMcp16_133*ROcp16_833-OMcp16_233*ROcp16_733);

// = = Block_1_0_0_17_0_6 = = 
 
// Sensor Kinematics 


    ROcp16_444 = ROcp16_434*C44+ROcp16_733*S44;
    ROcp16_544 = ROcp16_534*C44+ROcp16_833*S44;
    ROcp16_644 = ROcp16_634*C44+ROcp16_933*S44;
    ROcp16_744 = -(ROcp16_434*S44-ROcp16_733*C44);
    ROcp16_844 = -(ROcp16_534*S44-ROcp16_833*C44);
    ROcp16_944 = -(ROcp16_634*S44-ROcp16_933*C44);
    ROcp16_145 = ROcp16_134*C45+ROcp16_444*S45;
    ROcp16_245 = ROcp16_234*C45+ROcp16_544*S45;
    ROcp16_345 = ROcp16_334*C45+ROcp16_644*S45;
    ROcp16_445 = -(ROcp16_134*S45-ROcp16_444*C45);
    ROcp16_545 = -(ROcp16_234*S45-ROcp16_544*C45);
    ROcp16_645 = -(ROcp16_334*S45-ROcp16_644*C45);
    ROcp16_146 = ROcp16_145*C46-ROcp16_744*S46;
    ROcp16_246 = ROcp16_245*C46-ROcp16_844*S46;
    ROcp16_346 = ROcp16_345*C46-ROcp16_944*S46;
    ROcp16_746 = ROcp16_145*S46+ROcp16_744*C46;
    ROcp16_846 = ROcp16_245*S46+ROcp16_844*C46;
    ROcp16_946 = ROcp16_345*S46+ROcp16_944*C46;
    ROcp16_447 = ROcp16_445*C47+ROcp16_746*S47;
    ROcp16_547 = ROcp16_545*C47+ROcp16_846*S47;
    ROcp16_647 = ROcp16_645*C47+ROcp16_946*S47;
    ROcp16_747 = -(ROcp16_445*S47-ROcp16_746*C47);
    ROcp16_847 = -(ROcp16_545*S47-ROcp16_846*C47);
    ROcp16_947 = -(ROcp16_645*S47-ROcp16_946*C47);
    ROcp16_148 = ROcp16_146*C48+ROcp16_447*S48;
    ROcp16_248 = ROcp16_246*C48+ROcp16_547*S48;
    ROcp16_348 = ROcp16_346*C48+ROcp16_647*S48;
    ROcp16_448 = -(ROcp16_146*S48-ROcp16_447*C48);
    ROcp16_548 = -(ROcp16_246*S48-ROcp16_547*C48);
    ROcp16_648 = -(ROcp16_346*S48-ROcp16_647*C48);
    RLcp16_144 = ROcp16_134*s->dpt[1][24]+ROcp16_434*s->dpt[2][24]+ROcp16_733*s->dpt[3][24];
    RLcp16_244 = ROcp16_234*s->dpt[1][24]+ROcp16_534*s->dpt[2][24]+ROcp16_833*s->dpt[3][24];
    RLcp16_344 = ROcp16_334*s->dpt[1][24]+ROcp16_634*s->dpt[2][24]+ROcp16_933*s->dpt[3][24];
    ORcp16_144 = OMcp16_234*RLcp16_344-OMcp16_334*RLcp16_244;
    ORcp16_244 = -(OMcp16_134*RLcp16_344-OMcp16_334*RLcp16_144);
    ORcp16_344 = OMcp16_134*RLcp16_244-OMcp16_234*RLcp16_144;
    OMcp16_146 = OMcp16_134+ROcp16_445*qd[46];
    OMcp16_246 = OMcp16_234+ROcp16_545*qd[46];
    OMcp16_346 = OMcp16_334+ROcp16_645*qd[46];
    OPcp16_146 = OPcp16_134+ROcp16_445*qdd[46]+qd[46]*(OMcp16_234*ROcp16_645-OMcp16_334*ROcp16_545);
    OPcp16_246 = OPcp16_234+ROcp16_545*qdd[46]-qd[46]*(OMcp16_134*ROcp16_645-OMcp16_334*ROcp16_445);
    OPcp16_346 = OPcp16_334+ROcp16_645*qdd[46]+qd[46]*(OMcp16_134*ROcp16_545-OMcp16_234*ROcp16_445);
    RLcp16_147 = ROcp16_445*s->dpt[2][41]+ROcp16_746*s->dpt[3][41];
    RLcp16_247 = ROcp16_545*s->dpt[2][41]+ROcp16_846*s->dpt[3][41];
    RLcp16_347 = ROcp16_645*s->dpt[2][41]+ROcp16_946*s->dpt[3][41];
    OMcp16_147 = OMcp16_146+ROcp16_146*qd[47];
    OMcp16_247 = OMcp16_246+ROcp16_246*qd[47];
    OMcp16_347 = OMcp16_346+ROcp16_346*qd[47];
    ORcp16_147 = OMcp16_246*RLcp16_347-OMcp16_346*RLcp16_247;
    ORcp16_247 = -(OMcp16_146*RLcp16_347-OMcp16_346*RLcp16_147);
    ORcp16_347 = OMcp16_146*RLcp16_247-OMcp16_246*RLcp16_147;
    OPcp16_147 = OPcp16_146+ROcp16_146*qdd[47]+qd[47]*(OMcp16_246*ROcp16_346-OMcp16_346*ROcp16_246);
    OPcp16_247 = OPcp16_246+ROcp16_246*qdd[47]-qd[47]*(OMcp16_146*ROcp16_346-OMcp16_346*ROcp16_146);
    OPcp16_347 = OPcp16_346+ROcp16_346*qdd[47]+qd[47]*(OMcp16_146*ROcp16_246-OMcp16_246*ROcp16_146);
    RLcp16_148 = ROcp16_747*s->dpt[3][43];
    RLcp16_248 = ROcp16_847*s->dpt[3][43];
    RLcp16_348 = ROcp16_947*s->dpt[3][43];
    OMcp16_148 = OMcp16_147+ROcp16_747*qd[48];
    OMcp16_248 = OMcp16_247+ROcp16_847*qd[48];
    OMcp16_348 = OMcp16_347+ROcp16_947*qd[48];
    ORcp16_148 = OMcp16_247*RLcp16_348-OMcp16_347*RLcp16_248;
    ORcp16_248 = -(OMcp16_147*RLcp16_348-OMcp16_347*RLcp16_148);
    ORcp16_348 = OMcp16_147*RLcp16_248-OMcp16_247*RLcp16_148;
    OPcp16_148 = OPcp16_147+ROcp16_747*qdd[48]+qd[48]*(OMcp16_247*ROcp16_947-OMcp16_347*ROcp16_847);
    OPcp16_248 = OPcp16_247+ROcp16_847*qdd[48]-qd[48]*(OMcp16_147*ROcp16_947-OMcp16_347*ROcp16_747);
    OPcp16_348 = OPcp16_347+ROcp16_947*qdd[48]+qd[48]*(OMcp16_147*ROcp16_847-OMcp16_247*ROcp16_747);
    RLcp16_172 = ROcp16_148*s->dpt[1][47]+ROcp16_448*s->dpt[2][47]+ROcp16_747*s->dpt[3][47];
    RLcp16_272 = ROcp16_248*s->dpt[1][47]+ROcp16_548*s->dpt[2][47]+ROcp16_847*s->dpt[3][47];
    RLcp16_372 = ROcp16_348*s->dpt[1][47]+ROcp16_648*s->dpt[2][47]+ROcp16_947*s->dpt[3][47];
    POcp16_172 = RLcp16_131+RLcp16_134+RLcp16_144+RLcp16_147+RLcp16_148+RLcp16_172+q[1];
    POcp16_272 = RLcp16_231+RLcp16_234+RLcp16_244+RLcp16_247+RLcp16_248+RLcp16_272+q[2];
    POcp16_372 = RLcp16_331+RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348+RLcp16_372+q[3];
    JTcp16_272_4 = -(RLcp16_331+RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348+RLcp16_372);
    JTcp16_372_4 = RLcp16_231+RLcp16_234+RLcp16_244+RLcp16_247+RLcp16_248+RLcp16_272;
    JTcp16_172_5 = C4*(RLcp16_331+RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348+RLcp16_372)-S4*(RLcp16_231+RLcp16_234)-S4*(
 RLcp16_244+RLcp16_247)-S4*(RLcp16_248+RLcp16_272);
    JTcp16_272_5 = S4*(RLcp16_131+RLcp16_134+RLcp16_144+RLcp16_147+RLcp16_148+RLcp16_172);
    JTcp16_372_5 = -C4*(RLcp16_131+RLcp16_134+RLcp16_144+RLcp16_147+RLcp16_148+RLcp16_172);
    JTcp16_172_6 = ROcp16_85*(RLcp16_331+RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348+RLcp16_372)-ROcp16_95*(RLcp16_231+
 RLcp16_234)-ROcp16_95*(RLcp16_244+RLcp16_247)-ROcp16_95*(RLcp16_248+RLcp16_272);
    JTcp16_272_6 = RLcp16_172*ROcp16_95-RLcp16_348*S5-RLcp16_372*S5+ROcp16_95*(RLcp16_131+RLcp16_134+RLcp16_144+RLcp16_147
 +RLcp16_148)-S5*(RLcp16_331+RLcp16_334)-S5*(RLcp16_344+RLcp16_347);
    JTcp16_372_6 = RLcp16_248*S5-ROcp16_85*(RLcp16_131+RLcp16_134+RLcp16_144+RLcp16_147+RLcp16_148)+S5*(RLcp16_231+
 RLcp16_234)+S5*(RLcp16_244+RLcp16_247)-RLcp16_172*ROcp16_85+RLcp16_272*S5;
    JTcp16_172_7 = ROcp16_56*(RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348)-ROcp16_66*(RLcp16_234+RLcp16_244)-ROcp16_66*(
 RLcp16_247+RLcp16_248)-RLcp16_272*ROcp16_66+RLcp16_372*ROcp16_56;
    JTcp16_272_7 = RLcp16_172*ROcp16_66-RLcp16_372*ROcp16_46-ROcp16_46*(RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348)+
 ROcp16_66*(RLcp16_134+RLcp16_144)+ROcp16_66*(RLcp16_147+RLcp16_148);
    JTcp16_372_7 = ROcp16_46*(RLcp16_234+RLcp16_244+RLcp16_247+RLcp16_248)-ROcp16_56*(RLcp16_134+RLcp16_144)-ROcp16_56*(
 RLcp16_147+RLcp16_148)-RLcp16_172*ROcp16_56+RLcp16_272*ROcp16_46;
    JTcp16_172_8 = ROcp16_231*(RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348)-ROcp16_331*(RLcp16_234+RLcp16_244)-ROcp16_331*
 (RLcp16_247+RLcp16_248)-RLcp16_272*ROcp16_331+RLcp16_372*ROcp16_231;
    JTcp16_272_8 = RLcp16_172*ROcp16_331-RLcp16_372*ROcp16_131-ROcp16_131*(RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348)+
 ROcp16_331*(RLcp16_134+RLcp16_144)+ROcp16_331*(RLcp16_147+RLcp16_148);
    JTcp16_372_8 = ROcp16_131*(RLcp16_234+RLcp16_244+RLcp16_247+RLcp16_248)-ROcp16_231*(RLcp16_134+RLcp16_144)-ROcp16_231*
 (RLcp16_147+RLcp16_148)-RLcp16_172*ROcp16_231+RLcp16_272*ROcp16_131;
    JTcp16_172_9 = ROcp16_532*(RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348)-ROcp16_632*(RLcp16_234+RLcp16_244)-ROcp16_632*
 (RLcp16_247+RLcp16_248)-RLcp16_272*ROcp16_632+RLcp16_372*ROcp16_532;
    JTcp16_272_9 = RLcp16_172*ROcp16_632-RLcp16_372*ROcp16_432-ROcp16_432*(RLcp16_334+RLcp16_344+RLcp16_347+RLcp16_348)+
 ROcp16_632*(RLcp16_134+RLcp16_144)+ROcp16_632*(RLcp16_147+RLcp16_148);
    JTcp16_372_9 = ROcp16_432*(RLcp16_234+RLcp16_244+RLcp16_247+RLcp16_248)-ROcp16_532*(RLcp16_134+RLcp16_144)-ROcp16_532*
 (RLcp16_147+RLcp16_148)-RLcp16_172*ROcp16_532+RLcp16_272*ROcp16_432;
    JTcp16_172_10 = ROcp16_833*(RLcp16_344+RLcp16_347+RLcp16_348+RLcp16_372)-ROcp16_933*(RLcp16_244+RLcp16_247)-ROcp16_933
 *(RLcp16_248+RLcp16_272);
    JTcp16_272_10 = -(ROcp16_733*(RLcp16_344+RLcp16_347+RLcp16_348+RLcp16_372)-ROcp16_933*(RLcp16_144+RLcp16_147)-
 ROcp16_933*(RLcp16_148+RLcp16_172));
    JTcp16_372_10 = ROcp16_733*(RLcp16_244+RLcp16_247+RLcp16_248+RLcp16_272)-ROcp16_833*(RLcp16_144+RLcp16_147)-ROcp16_833
 *(RLcp16_148+RLcp16_172);
    JTcp16_172_11 = ROcp16_234*(RLcp16_347+RLcp16_348)-ROcp16_334*(RLcp16_247+RLcp16_248)-RLcp16_272*ROcp16_334+RLcp16_372
 *ROcp16_234;
    JTcp16_272_11 = RLcp16_172*ROcp16_334-RLcp16_372*ROcp16_134-ROcp16_134*(RLcp16_347+RLcp16_348)+ROcp16_334*(RLcp16_147+
 RLcp16_148);
    JTcp16_372_11 = ROcp16_134*(RLcp16_247+RLcp16_248)-ROcp16_234*(RLcp16_147+RLcp16_148)-RLcp16_172*ROcp16_234+RLcp16_272
 *ROcp16_134;
    JTcp16_172_12 = ROcp16_844*(RLcp16_347+RLcp16_348)-ROcp16_944*(RLcp16_247+RLcp16_248)-RLcp16_272*ROcp16_944+RLcp16_372
 *ROcp16_844;
    JTcp16_272_12 = RLcp16_172*ROcp16_944-RLcp16_372*ROcp16_744-ROcp16_744*(RLcp16_347+RLcp16_348)+ROcp16_944*(RLcp16_147+
 RLcp16_148);
    JTcp16_372_12 = ROcp16_744*(RLcp16_247+RLcp16_248)-ROcp16_844*(RLcp16_147+RLcp16_148)-RLcp16_172*ROcp16_844+RLcp16_272
 *ROcp16_744;
    JTcp16_172_13 = ROcp16_545*(RLcp16_347+RLcp16_348)-ROcp16_645*(RLcp16_247+RLcp16_248)-RLcp16_272*ROcp16_645+RLcp16_372
 *ROcp16_545;
    JTcp16_272_13 = RLcp16_172*ROcp16_645-RLcp16_372*ROcp16_445-ROcp16_445*(RLcp16_347+RLcp16_348)+ROcp16_645*(RLcp16_147+
 RLcp16_148);
    JTcp16_372_13 = ROcp16_445*(RLcp16_247+RLcp16_248)-ROcp16_545*(RLcp16_147+RLcp16_148)-RLcp16_172*ROcp16_545+RLcp16_272
 *ROcp16_445;
    JTcp16_172_14 = ROcp16_246*(RLcp16_348+RLcp16_372)-ROcp16_346*(RLcp16_248+RLcp16_272);
    JTcp16_272_14 = -(ROcp16_146*(RLcp16_348+RLcp16_372)-ROcp16_346*(RLcp16_148+RLcp16_172));
    JTcp16_372_14 = ROcp16_146*(RLcp16_248+RLcp16_272)-ROcp16_246*(RLcp16_148+RLcp16_172);
    JTcp16_172_15 = -(RLcp16_272*ROcp16_947-RLcp16_372*ROcp16_847);
    JTcp16_272_15 = RLcp16_172*ROcp16_947-RLcp16_372*ROcp16_747;
    JTcp16_372_15 = -(RLcp16_172*ROcp16_847-RLcp16_272*ROcp16_747);
    ORcp16_172 = OMcp16_248*RLcp16_372-OMcp16_348*RLcp16_272;
    ORcp16_272 = -(OMcp16_148*RLcp16_372-OMcp16_348*RLcp16_172);
    ORcp16_372 = OMcp16_148*RLcp16_272-OMcp16_248*RLcp16_172;
    VIcp16_172 = ORcp16_131+ORcp16_134+ORcp16_144+ORcp16_147+ORcp16_148+ORcp16_172+qd[1];
    VIcp16_272 = ORcp16_231+ORcp16_234+ORcp16_244+ORcp16_247+ORcp16_248+ORcp16_272+qd[2];
    VIcp16_372 = ORcp16_331+ORcp16_334+ORcp16_344+ORcp16_347+ORcp16_348+ORcp16_372+qd[3];
    ACcp16_172 = qdd[1]+OMcp16_233*ORcp16_334+OMcp16_234*ORcp16_344+OMcp16_246*ORcp16_347+OMcp16_247*ORcp16_348+OMcp16_248
 *ORcp16_372+OMcp16_26*ORcp16_331-OMcp16_333*ORcp16_234-OMcp16_334*ORcp16_244-OMcp16_346*ORcp16_247-OMcp16_347*ORcp16_248-
 OMcp16_348*ORcp16_272-OMcp16_36*ORcp16_231+OPcp16_233*RLcp16_334+OPcp16_234*RLcp16_344+OPcp16_246*RLcp16_347+OPcp16_247*
 RLcp16_348+OPcp16_248*RLcp16_372+OPcp16_26*RLcp16_331-OPcp16_333*RLcp16_234-OPcp16_334*RLcp16_244-OPcp16_346*RLcp16_247-
 OPcp16_347*RLcp16_248-OPcp16_348*RLcp16_272-OPcp16_36*RLcp16_231;
    ACcp16_272 = qdd[2]-OMcp16_133*ORcp16_334-OMcp16_134*ORcp16_344-OMcp16_146*ORcp16_347-OMcp16_147*ORcp16_348-OMcp16_148
 *ORcp16_372-OMcp16_16*ORcp16_331+OMcp16_333*ORcp16_134+OMcp16_334*ORcp16_144+OMcp16_346*ORcp16_147+OMcp16_347*ORcp16_148+
 OMcp16_348*ORcp16_172+OMcp16_36*ORcp16_131-OPcp16_133*RLcp16_334-OPcp16_134*RLcp16_344-OPcp16_146*RLcp16_347-OPcp16_147*
 RLcp16_348-OPcp16_148*RLcp16_372-OPcp16_16*RLcp16_331+OPcp16_333*RLcp16_134+OPcp16_334*RLcp16_144+OPcp16_346*RLcp16_147+
 OPcp16_347*RLcp16_148+OPcp16_348*RLcp16_172+OPcp16_36*RLcp16_131;
    ACcp16_372 = qdd[3]+OMcp16_133*ORcp16_234+OMcp16_134*ORcp16_244+OMcp16_146*ORcp16_247+OMcp16_147*ORcp16_248+OMcp16_148
 *ORcp16_272+OMcp16_16*ORcp16_231-OMcp16_233*ORcp16_134-OMcp16_234*ORcp16_144-OMcp16_246*ORcp16_147-OMcp16_247*ORcp16_148-
 OMcp16_248*ORcp16_172-OMcp16_26*ORcp16_131+OPcp16_133*RLcp16_234+OPcp16_134*RLcp16_244+OPcp16_146*RLcp16_247+OPcp16_147*
 RLcp16_248+OPcp16_148*RLcp16_272+OPcp16_16*RLcp16_231-OPcp16_233*RLcp16_134-OPcp16_234*RLcp16_144-OPcp16_246*RLcp16_147-
 OPcp16_247*RLcp16_148-OPcp16_248*RLcp16_172-OPcp16_26*RLcp16_131;

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp16_172;
    sens->P[2] = POcp16_272;
    sens->P[3] = POcp16_372;
    sens->R[1][1] = ROcp16_148;
    sens->R[1][2] = ROcp16_248;
    sens->R[1][3] = ROcp16_348;
    sens->R[2][1] = ROcp16_448;
    sens->R[2][2] = ROcp16_548;
    sens->R[2][3] = ROcp16_648;
    sens->R[3][1] = ROcp16_747;
    sens->R[3][2] = ROcp16_847;
    sens->R[3][3] = ROcp16_947;
    sens->V[1] = VIcp16_172;
    sens->V[2] = VIcp16_272;
    sens->V[3] = VIcp16_372;
    sens->OM[1] = OMcp16_148;
    sens->OM[2] = OMcp16_248;
    sens->OM[3] = OMcp16_348;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp16_172_5;
    sens->J[1][6] = JTcp16_172_6;
    sens->J[1][31] = JTcp16_172_7;
    sens->J[1][32] = JTcp16_172_8;
    sens->J[1][33] = JTcp16_172_9;
    sens->J[1][34] = JTcp16_172_10;
    sens->J[1][44] = JTcp16_172_11;
    sens->J[1][45] = JTcp16_172_12;
    sens->J[1][46] = JTcp16_172_13;
    sens->J[1][47] = JTcp16_172_14;
    sens->J[1][48] = JTcp16_172_15;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp16_272_4;
    sens->J[2][5] = JTcp16_272_5;
    sens->J[2][6] = JTcp16_272_6;
    sens->J[2][31] = JTcp16_272_7;
    sens->J[2][32] = JTcp16_272_8;
    sens->J[2][33] = JTcp16_272_9;
    sens->J[2][34] = JTcp16_272_10;
    sens->J[2][44] = JTcp16_272_11;
    sens->J[2][45] = JTcp16_272_12;
    sens->J[2][46] = JTcp16_272_13;
    sens->J[2][47] = JTcp16_272_14;
    sens->J[2][48] = JTcp16_272_15;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp16_372_4;
    sens->J[3][5] = JTcp16_372_5;
    sens->J[3][6] = JTcp16_372_6;
    sens->J[3][31] = JTcp16_372_7;
    sens->J[3][32] = JTcp16_372_8;
    sens->J[3][33] = JTcp16_372_9;
    sens->J[3][34] = JTcp16_372_10;
    sens->J[3][44] = JTcp16_372_11;
    sens->J[3][45] = JTcp16_372_12;
    sens->J[3][46] = JTcp16_372_13;
    sens->J[3][47] = JTcp16_372_14;
    sens->J[3][48] = JTcp16_372_15;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp16_46;
    sens->J[4][32] = ROcp16_131;
    sens->J[4][33] = ROcp16_432;
    sens->J[4][34] = ROcp16_733;
    sens->J[4][44] = ROcp16_134;
    sens->J[4][45] = ROcp16_744;
    sens->J[4][46] = ROcp16_445;
    sens->J[4][47] = ROcp16_146;
    sens->J[4][48] = ROcp16_747;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp16_85;
    sens->J[5][31] = ROcp16_56;
    sens->J[5][32] = ROcp16_231;
    sens->J[5][33] = ROcp16_532;
    sens->J[5][34] = ROcp16_833;
    sens->J[5][44] = ROcp16_234;
    sens->J[5][45] = ROcp16_844;
    sens->J[5][46] = ROcp16_545;
    sens->J[5][47] = ROcp16_246;
    sens->J[5][48] = ROcp16_847;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp16_95;
    sens->J[6][31] = ROcp16_66;
    sens->J[6][32] = ROcp16_331;
    sens->J[6][33] = ROcp16_632;
    sens->J[6][34] = ROcp16_933;
    sens->J[6][44] = ROcp16_334;
    sens->J[6][45] = ROcp16_944;
    sens->J[6][46] = ROcp16_645;
    sens->J[6][47] = ROcp16_346;
    sens->J[6][48] = ROcp16_947;
    sens->A[1] = ACcp16_172;
    sens->A[2] = ACcp16_272;
    sens->A[3] = ACcp16_372;
    sens->OMP[1] = OPcp16_148;
    sens->OMP[2] = OPcp16_248;
    sens->OMP[3] = OPcp16_348;
 
// 
break;
case 18:
 


// = = Block_1_0_0_18_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp17_25 = qd[5]*C4;
    OMcp17_35 = qd[5]*S4;
    OMcp17_16 = qd[4]+qd[6]*S5;
    OMcp17_26 = OMcp17_25+ROcp17_85*qd[6];
    OMcp17_36 = OMcp17_35+ROcp17_95*qd[6];
    OPcp17_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp17_26 = ROcp17_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp17_35*S5-ROcp17_95*qd[4]);
    OPcp17_36 = ROcp17_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp17_25*S5-ROcp17_85*qd[4]);

// = = Block_1_0_0_18_0_4 = = 
 
// Sensor Kinematics 


    ROcp17_131 = ROcp17_16*C31-S31*S5;
    ROcp17_231 = ROcp17_26*C31-ROcp17_85*S31;
    ROcp17_331 = ROcp17_36*C31-ROcp17_95*S31;
    ROcp17_731 = ROcp17_16*S31+C31*S5;
    ROcp17_831 = ROcp17_26*S31+ROcp17_85*C31;
    ROcp17_931 = ROcp17_36*S31+ROcp17_95*C31;
    ROcp17_432 = ROcp17_46*C32+ROcp17_731*S32;
    ROcp17_532 = ROcp17_56*C32+ROcp17_831*S32;
    ROcp17_632 = ROcp17_66*C32+ROcp17_931*S32;
    ROcp17_732 = -(ROcp17_46*S32-ROcp17_731*C32);
    ROcp17_832 = -(ROcp17_56*S32-ROcp17_831*C32);
    ROcp17_932 = -(ROcp17_66*S32-ROcp17_931*C32);
    ROcp17_133 = ROcp17_131*C33-ROcp17_732*S33;
    ROcp17_233 = ROcp17_231*C33-ROcp17_832*S33;
    ROcp17_333 = ROcp17_331*C33-ROcp17_932*S33;
    ROcp17_733 = ROcp17_131*S33+ROcp17_732*C33;
    ROcp17_833 = ROcp17_231*S33+ROcp17_832*C33;
    ROcp17_933 = ROcp17_331*S33+ROcp17_932*C33;
    ROcp17_134 = ROcp17_133*C34+ROcp17_432*S34;
    ROcp17_234 = ROcp17_233*C34+ROcp17_532*S34;
    ROcp17_334 = ROcp17_333*C34+ROcp17_632*S34;
    ROcp17_434 = -(ROcp17_133*S34-ROcp17_432*C34);
    ROcp17_534 = -(ROcp17_233*S34-ROcp17_532*C34);
    ROcp17_634 = -(ROcp17_333*S34-ROcp17_632*C34);
    RLcp17_131 = ROcp17_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp17_231 = ROcp17_26*s->dpt[1][3]+ROcp17_85*s->dpt[3][3];
    RLcp17_331 = ROcp17_36*s->dpt[1][3]+ROcp17_95*s->dpt[3][3];
    OMcp17_131 = OMcp17_16+ROcp17_46*qd[31];
    OMcp17_231 = OMcp17_26+ROcp17_56*qd[31];
    OMcp17_331 = OMcp17_36+ROcp17_66*qd[31];
    ORcp17_131 = OMcp17_26*RLcp17_331-OMcp17_36*RLcp17_231;
    ORcp17_231 = -(OMcp17_16*RLcp17_331-OMcp17_36*RLcp17_131);
    ORcp17_331 = OMcp17_16*RLcp17_231-OMcp17_26*RLcp17_131;
    OMcp17_132 = OMcp17_131+ROcp17_131*qd[32];
    OMcp17_232 = OMcp17_231+ROcp17_231*qd[32];
    OMcp17_332 = OMcp17_331+ROcp17_331*qd[32];
    OMcp17_133 = OMcp17_132+ROcp17_432*qd[33];
    OMcp17_233 = OMcp17_232+ROcp17_532*qd[33];
    OMcp17_333 = OMcp17_332+ROcp17_632*qd[33];
    OPcp17_133 = OPcp17_16+ROcp17_131*qdd[32]+ROcp17_432*qdd[33]+ROcp17_46*qdd[31]+qd[31]*(OMcp17_26*ROcp17_66-OMcp17_36*
 ROcp17_56)+qd[32]*(OMcp17_231*ROcp17_331-OMcp17_331*ROcp17_231)+qd[33]*(OMcp17_232*ROcp17_632-OMcp17_332*ROcp17_532);
    OPcp17_233 = OPcp17_26+ROcp17_231*qdd[32]+ROcp17_532*qdd[33]+ROcp17_56*qdd[31]-qd[31]*(OMcp17_16*ROcp17_66-OMcp17_36*
 ROcp17_46)-qd[32]*(OMcp17_131*ROcp17_331-OMcp17_331*ROcp17_131)-qd[33]*(OMcp17_132*ROcp17_632-OMcp17_332*ROcp17_432);
    OPcp17_333 = OPcp17_36+ROcp17_331*qdd[32]+ROcp17_632*qdd[33]+ROcp17_66*qdd[31]+qd[31]*(OMcp17_16*ROcp17_56-OMcp17_26*
 ROcp17_46)+qd[32]*(OMcp17_131*ROcp17_231-OMcp17_231*ROcp17_131)+qd[33]*(OMcp17_132*ROcp17_532-OMcp17_232*ROcp17_432);
    RLcp17_134 = ROcp17_733*s->dpt[3][21];
    RLcp17_234 = ROcp17_833*s->dpt[3][21];
    RLcp17_334 = ROcp17_933*s->dpt[3][21];
    OMcp17_134 = OMcp17_133+ROcp17_733*qd[34];
    OMcp17_234 = OMcp17_233+ROcp17_833*qd[34];
    OMcp17_334 = OMcp17_333+ROcp17_933*qd[34];
    ORcp17_134 = OMcp17_233*RLcp17_334-OMcp17_333*RLcp17_234;
    ORcp17_234 = -(OMcp17_133*RLcp17_334-OMcp17_333*RLcp17_134);
    ORcp17_334 = OMcp17_133*RLcp17_234-OMcp17_233*RLcp17_134;
    OPcp17_134 = OPcp17_133+ROcp17_733*qdd[34]+qd[34]*(OMcp17_233*ROcp17_933-OMcp17_333*ROcp17_833);
    OPcp17_234 = OPcp17_233+ROcp17_833*qdd[34]-qd[34]*(OMcp17_133*ROcp17_933-OMcp17_333*ROcp17_733);
    OPcp17_334 = OPcp17_333+ROcp17_933*qdd[34]+qd[34]*(OMcp17_133*ROcp17_833-OMcp17_233*ROcp17_733);

// = = Block_1_0_0_18_0_6 = = 
 
// Sensor Kinematics 


    ROcp17_444 = ROcp17_434*C44+ROcp17_733*S44;
    ROcp17_544 = ROcp17_534*C44+ROcp17_833*S44;
    ROcp17_644 = ROcp17_634*C44+ROcp17_933*S44;
    ROcp17_744 = -(ROcp17_434*S44-ROcp17_733*C44);
    ROcp17_844 = -(ROcp17_534*S44-ROcp17_833*C44);
    ROcp17_944 = -(ROcp17_634*S44-ROcp17_933*C44);
    ROcp17_145 = ROcp17_134*C45+ROcp17_444*S45;
    ROcp17_245 = ROcp17_234*C45+ROcp17_544*S45;
    ROcp17_345 = ROcp17_334*C45+ROcp17_644*S45;
    ROcp17_445 = -(ROcp17_134*S45-ROcp17_444*C45);
    ROcp17_545 = -(ROcp17_234*S45-ROcp17_544*C45);
    ROcp17_645 = -(ROcp17_334*S45-ROcp17_644*C45);
    ROcp17_146 = ROcp17_145*C46-ROcp17_744*S46;
    ROcp17_246 = ROcp17_245*C46-ROcp17_844*S46;
    ROcp17_346 = ROcp17_345*C46-ROcp17_944*S46;
    ROcp17_746 = ROcp17_145*S46+ROcp17_744*C46;
    ROcp17_846 = ROcp17_245*S46+ROcp17_844*C46;
    ROcp17_946 = ROcp17_345*S46+ROcp17_944*C46;
    ROcp17_447 = ROcp17_445*C47+ROcp17_746*S47;
    ROcp17_547 = ROcp17_545*C47+ROcp17_846*S47;
    ROcp17_647 = ROcp17_645*C47+ROcp17_946*S47;
    ROcp17_747 = -(ROcp17_445*S47-ROcp17_746*C47);
    ROcp17_847 = -(ROcp17_545*S47-ROcp17_846*C47);
    ROcp17_947 = -(ROcp17_645*S47-ROcp17_946*C47);
    ROcp17_148 = ROcp17_146*C48+ROcp17_447*S48;
    ROcp17_248 = ROcp17_246*C48+ROcp17_547*S48;
    ROcp17_348 = ROcp17_346*C48+ROcp17_647*S48;
    ROcp17_448 = -(ROcp17_146*S48-ROcp17_447*C48);
    ROcp17_548 = -(ROcp17_246*S48-ROcp17_547*C48);
    ROcp17_648 = -(ROcp17_346*S48-ROcp17_647*C48);
    ROcp17_149 = ROcp17_148*C49-ROcp17_747*S49;
    ROcp17_249 = ROcp17_248*C49-ROcp17_847*S49;
    ROcp17_349 = ROcp17_348*C49-ROcp17_947*S49;
    ROcp17_749 = ROcp17_148*S49+ROcp17_747*C49;
    ROcp17_849 = ROcp17_248*S49+ROcp17_847*C49;
    ROcp17_949 = ROcp17_348*S49+ROcp17_947*C49;
    RLcp17_144 = ROcp17_134*s->dpt[1][24]+ROcp17_434*s->dpt[2][24]+ROcp17_733*s->dpt[3][24];
    RLcp17_244 = ROcp17_234*s->dpt[1][24]+ROcp17_534*s->dpt[2][24]+ROcp17_833*s->dpt[3][24];
    RLcp17_344 = ROcp17_334*s->dpt[1][24]+ROcp17_634*s->dpt[2][24]+ROcp17_933*s->dpt[3][24];
    ORcp17_144 = OMcp17_234*RLcp17_344-OMcp17_334*RLcp17_244;
    ORcp17_244 = -(OMcp17_134*RLcp17_344-OMcp17_334*RLcp17_144);
    ORcp17_344 = OMcp17_134*RLcp17_244-OMcp17_234*RLcp17_144;
    OMcp17_146 = OMcp17_134+ROcp17_445*qd[46];
    OMcp17_246 = OMcp17_234+ROcp17_545*qd[46];
    OMcp17_346 = OMcp17_334+ROcp17_645*qd[46];
    OPcp17_146 = OPcp17_134+ROcp17_445*qdd[46]+qd[46]*(OMcp17_234*ROcp17_645-OMcp17_334*ROcp17_545);
    OPcp17_246 = OPcp17_234+ROcp17_545*qdd[46]-qd[46]*(OMcp17_134*ROcp17_645-OMcp17_334*ROcp17_445);
    OPcp17_346 = OPcp17_334+ROcp17_645*qdd[46]+qd[46]*(OMcp17_134*ROcp17_545-OMcp17_234*ROcp17_445);
    RLcp17_147 = ROcp17_445*s->dpt[2][41]+ROcp17_746*s->dpt[3][41];
    RLcp17_247 = ROcp17_545*s->dpt[2][41]+ROcp17_846*s->dpt[3][41];
    RLcp17_347 = ROcp17_645*s->dpt[2][41]+ROcp17_946*s->dpt[3][41];
    OMcp17_147 = OMcp17_146+ROcp17_146*qd[47];
    OMcp17_247 = OMcp17_246+ROcp17_246*qd[47];
    OMcp17_347 = OMcp17_346+ROcp17_346*qd[47];
    ORcp17_147 = OMcp17_246*RLcp17_347-OMcp17_346*RLcp17_247;
    ORcp17_247 = -(OMcp17_146*RLcp17_347-OMcp17_346*RLcp17_147);
    ORcp17_347 = OMcp17_146*RLcp17_247-OMcp17_246*RLcp17_147;
    OPcp17_147 = OPcp17_146+ROcp17_146*qdd[47]+qd[47]*(OMcp17_246*ROcp17_346-OMcp17_346*ROcp17_246);
    OPcp17_247 = OPcp17_246+ROcp17_246*qdd[47]-qd[47]*(OMcp17_146*ROcp17_346-OMcp17_346*ROcp17_146);
    OPcp17_347 = OPcp17_346+ROcp17_346*qdd[47]+qd[47]*(OMcp17_146*ROcp17_246-OMcp17_246*ROcp17_146);
    RLcp17_148 = ROcp17_747*s->dpt[3][43];
    RLcp17_248 = ROcp17_847*s->dpt[3][43];
    RLcp17_348 = ROcp17_947*s->dpt[3][43];
    OMcp17_148 = OMcp17_147+ROcp17_747*qd[48];
    OMcp17_248 = OMcp17_247+ROcp17_847*qd[48];
    OMcp17_348 = OMcp17_347+ROcp17_947*qd[48];
    ORcp17_148 = OMcp17_247*RLcp17_348-OMcp17_347*RLcp17_248;
    ORcp17_248 = -(OMcp17_147*RLcp17_348-OMcp17_347*RLcp17_148);
    ORcp17_348 = OMcp17_147*RLcp17_248-OMcp17_247*RLcp17_148;
    OPcp17_148 = OPcp17_147+ROcp17_747*qdd[48]+qd[48]*(OMcp17_247*ROcp17_947-OMcp17_347*ROcp17_847);
    OPcp17_248 = OPcp17_247+ROcp17_847*qdd[48]-qd[48]*(OMcp17_147*ROcp17_947-OMcp17_347*ROcp17_747);
    OPcp17_348 = OPcp17_347+ROcp17_947*qdd[48]+qd[48]*(OMcp17_147*ROcp17_847-OMcp17_247*ROcp17_747);
    RLcp17_149 = ROcp17_148*s->dpt[1][46]+ROcp17_747*s->dpt[3][46];
    RLcp17_249 = ROcp17_248*s->dpt[1][46]+ROcp17_847*s->dpt[3][46];
    RLcp17_349 = ROcp17_348*s->dpt[1][46]+ROcp17_947*s->dpt[3][46];
    OMcp17_149 = OMcp17_148+ROcp17_448*qd[49];
    OMcp17_249 = OMcp17_248+ROcp17_548*qd[49];
    OMcp17_349 = OMcp17_348+ROcp17_648*qd[49];
    ORcp17_149 = OMcp17_248*RLcp17_349-OMcp17_348*RLcp17_249;
    ORcp17_249 = -(OMcp17_148*RLcp17_349-OMcp17_348*RLcp17_149);
    ORcp17_349 = OMcp17_148*RLcp17_249-OMcp17_248*RLcp17_149;
    OPcp17_149 = OPcp17_148+ROcp17_448*qdd[49]+qd[49]*(OMcp17_248*ROcp17_648-OMcp17_348*ROcp17_548);
    OPcp17_249 = OPcp17_248+ROcp17_548*qdd[49]-qd[49]*(OMcp17_148*ROcp17_648-OMcp17_348*ROcp17_448);
    OPcp17_349 = OPcp17_348+ROcp17_648*qdd[49]+qd[49]*(OMcp17_148*ROcp17_548-OMcp17_248*ROcp17_448);
    RLcp17_173 = ROcp17_149*s->dpt[1][49]+ROcp17_448*s->dpt[2][49]+ROcp17_749*s->dpt[3][49];
    RLcp17_273 = ROcp17_249*s->dpt[1][49]+ROcp17_548*s->dpt[2][49]+ROcp17_849*s->dpt[3][49];
    RLcp17_373 = ROcp17_349*s->dpt[1][49]+ROcp17_648*s->dpt[2][49]+ROcp17_949*s->dpt[3][49];
    POcp17_173 = RLcp17_131+RLcp17_134+RLcp17_144+RLcp17_147+RLcp17_148+RLcp17_149+RLcp17_173+q[1];
    POcp17_273 = RLcp17_231+RLcp17_234+RLcp17_244+RLcp17_247+RLcp17_248+RLcp17_249+RLcp17_273+q[2];
    POcp17_373 = RLcp17_331+RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373+q[3];
    JTcp17_273_4 = -(RLcp17_331+RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373);
    JTcp17_373_4 = RLcp17_231+RLcp17_234+RLcp17_244+RLcp17_247+RLcp17_248+RLcp17_249+RLcp17_273;
    JTcp17_173_5 = C4*(RLcp17_331+RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349)-S4*(RLcp17_231+RLcp17_234)-S4*(
 RLcp17_244+RLcp17_247)-S4*(RLcp17_248+RLcp17_249)-RLcp17_273*S4+RLcp17_373*C4;
    JTcp17_273_5 = S4*(RLcp17_131+RLcp17_134+RLcp17_144+RLcp17_147+RLcp17_148+RLcp17_149+RLcp17_173);
    JTcp17_373_5 = -C4*(RLcp17_131+RLcp17_134+RLcp17_144+RLcp17_147+RLcp17_148+RLcp17_149+RLcp17_173);
    JTcp17_173_6 = ROcp17_85*(RLcp17_331+RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349)-ROcp17_95*(RLcp17_231+
 RLcp17_234)-ROcp17_95*(RLcp17_244+RLcp17_247)-ROcp17_95*(RLcp17_248+RLcp17_249)-RLcp17_273*ROcp17_95+RLcp17_373*ROcp17_85;
    JTcp17_273_6 = -(RLcp17_373*S5-ROcp17_95*(RLcp17_131+RLcp17_134+RLcp17_144+RLcp17_147+RLcp17_148+RLcp17_149+RLcp17_173
 )+S5*(RLcp17_331+RLcp17_334)+S5*(RLcp17_344+RLcp17_347)+S5*(RLcp17_348+RLcp17_349));
    JTcp17_373_6 = RLcp17_273*S5-ROcp17_85*(RLcp17_131+RLcp17_134+RLcp17_144+RLcp17_147+RLcp17_148+RLcp17_149+RLcp17_173)+
 S5*(RLcp17_231+RLcp17_234)+S5*(RLcp17_244+RLcp17_247)+S5*(RLcp17_248+RLcp17_249);
    JTcp17_173_7 = ROcp17_56*(RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_66*(RLcp17_234+
 RLcp17_244)-ROcp17_66*(RLcp17_247+RLcp17_248)-ROcp17_66*(RLcp17_249+RLcp17_273);
    JTcp17_273_7 = -(ROcp17_46*(RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_66*(RLcp17_134+
 RLcp17_144)-ROcp17_66*(RLcp17_147+RLcp17_148)-ROcp17_66*(RLcp17_149+RLcp17_173));
    JTcp17_373_7 = ROcp17_46*(RLcp17_234+RLcp17_244+RLcp17_247+RLcp17_248+RLcp17_249+RLcp17_273)-ROcp17_56*(RLcp17_134+
 RLcp17_144)-ROcp17_56*(RLcp17_147+RLcp17_148)-ROcp17_56*(RLcp17_149+RLcp17_173);
    JTcp17_173_8 = ROcp17_231*(RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_331*(RLcp17_234+
 RLcp17_244)-ROcp17_331*(RLcp17_247+RLcp17_248)-ROcp17_331*(RLcp17_249+RLcp17_273);
    JTcp17_273_8 = -(ROcp17_131*(RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_331*(RLcp17_134
 +RLcp17_144)-ROcp17_331*(RLcp17_147+RLcp17_148)-ROcp17_331*(RLcp17_149+RLcp17_173));
    JTcp17_373_8 = ROcp17_131*(RLcp17_234+RLcp17_244+RLcp17_247+RLcp17_248+RLcp17_249+RLcp17_273)-ROcp17_231*(RLcp17_134+
 RLcp17_144)-ROcp17_231*(RLcp17_147+RLcp17_148)-ROcp17_231*(RLcp17_149+RLcp17_173);
    JTcp17_173_9 = ROcp17_532*(RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_632*(RLcp17_234+
 RLcp17_244)-ROcp17_632*(RLcp17_247+RLcp17_248)-ROcp17_632*(RLcp17_249+RLcp17_273);
    JTcp17_273_9 = -(ROcp17_432*(RLcp17_334+RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_632*(RLcp17_134
 +RLcp17_144)-ROcp17_632*(RLcp17_147+RLcp17_148)-ROcp17_632*(RLcp17_149+RLcp17_173));
    JTcp17_373_9 = ROcp17_432*(RLcp17_234+RLcp17_244+RLcp17_247+RLcp17_248+RLcp17_249+RLcp17_273)-ROcp17_532*(RLcp17_134+
 RLcp17_144)-ROcp17_532*(RLcp17_147+RLcp17_148)-ROcp17_532*(RLcp17_149+RLcp17_173);
    JTcp17_173_10 = ROcp17_833*(RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349)-ROcp17_933*(RLcp17_244+RLcp17_247)-ROcp17_933
 *(RLcp17_248+RLcp17_249)-RLcp17_273*ROcp17_933+RLcp17_373*ROcp17_833;
    JTcp17_273_10 = RLcp17_173*ROcp17_933-RLcp17_373*ROcp17_733-ROcp17_733*(RLcp17_344+RLcp17_347+RLcp17_348+RLcp17_349)+
 ROcp17_933*(RLcp17_144+RLcp17_147)+ROcp17_933*(RLcp17_148+RLcp17_149);
    JTcp17_373_10 = ROcp17_733*(RLcp17_244+RLcp17_247+RLcp17_248+RLcp17_249)-ROcp17_833*(RLcp17_144+RLcp17_147)-ROcp17_833
 *(RLcp17_148+RLcp17_149)-RLcp17_173*ROcp17_833+RLcp17_273*ROcp17_733;
    JTcp17_173_11 = ROcp17_234*(RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_334*(RLcp17_247+RLcp17_248)-ROcp17_334
 *(RLcp17_249+RLcp17_273);
    JTcp17_273_11 = -(ROcp17_134*(RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_334*(RLcp17_147+RLcp17_148)-
 ROcp17_334*(RLcp17_149+RLcp17_173));
    JTcp17_373_11 = ROcp17_134*(RLcp17_247+RLcp17_248+RLcp17_249+RLcp17_273)-ROcp17_234*(RLcp17_147+RLcp17_148)-ROcp17_234
 *(RLcp17_149+RLcp17_173);
    JTcp17_173_12 = ROcp17_844*(RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_944*(RLcp17_247+RLcp17_248)-ROcp17_944
 *(RLcp17_249+RLcp17_273);
    JTcp17_273_12 = -(ROcp17_744*(RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_944*(RLcp17_147+RLcp17_148)-
 ROcp17_944*(RLcp17_149+RLcp17_173));
    JTcp17_373_12 = ROcp17_744*(RLcp17_247+RLcp17_248+RLcp17_249+RLcp17_273)-ROcp17_844*(RLcp17_147+RLcp17_148)-ROcp17_844
 *(RLcp17_149+RLcp17_173);
    JTcp17_173_13 = ROcp17_545*(RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_645*(RLcp17_247+RLcp17_248)-ROcp17_645
 *(RLcp17_249+RLcp17_273);
    JTcp17_273_13 = -(ROcp17_445*(RLcp17_347+RLcp17_348+RLcp17_349+RLcp17_373)-ROcp17_645*(RLcp17_147+RLcp17_148)-
 ROcp17_645*(RLcp17_149+RLcp17_173));
    JTcp17_373_13 = ROcp17_445*(RLcp17_247+RLcp17_248+RLcp17_249+RLcp17_273)-ROcp17_545*(RLcp17_147+RLcp17_148)-ROcp17_545
 *(RLcp17_149+RLcp17_173);
    JTcp17_173_14 = ROcp17_246*(RLcp17_348+RLcp17_349)-ROcp17_346*(RLcp17_248+RLcp17_249)-RLcp17_273*ROcp17_346+RLcp17_373
 *ROcp17_246;
    JTcp17_273_14 = RLcp17_173*ROcp17_346-RLcp17_373*ROcp17_146-ROcp17_146*(RLcp17_348+RLcp17_349)+ROcp17_346*(RLcp17_148+
 RLcp17_149);
    JTcp17_373_14 = ROcp17_146*(RLcp17_248+RLcp17_249)-ROcp17_246*(RLcp17_148+RLcp17_149)-RLcp17_173*ROcp17_246+RLcp17_273
 *ROcp17_146;
    JTcp17_173_15 = ROcp17_847*(RLcp17_349+RLcp17_373)-ROcp17_947*(RLcp17_249+RLcp17_273);
    JTcp17_273_15 = -(ROcp17_747*(RLcp17_349+RLcp17_373)-ROcp17_947*(RLcp17_149+RLcp17_173));
    JTcp17_373_15 = ROcp17_747*(RLcp17_249+RLcp17_273)-ROcp17_847*(RLcp17_149+RLcp17_173);
    JTcp17_173_16 = -(RLcp17_273*ROcp17_648-RLcp17_373*ROcp17_548);
    JTcp17_273_16 = RLcp17_173*ROcp17_648-RLcp17_373*ROcp17_448;
    JTcp17_373_16 = -(RLcp17_173*ROcp17_548-RLcp17_273*ROcp17_448);
    ORcp17_173 = OMcp17_249*RLcp17_373-OMcp17_349*RLcp17_273;
    ORcp17_273 = -(OMcp17_149*RLcp17_373-OMcp17_349*RLcp17_173);
    ORcp17_373 = OMcp17_149*RLcp17_273-OMcp17_249*RLcp17_173;
    VIcp17_173 = ORcp17_131+ORcp17_134+ORcp17_144+ORcp17_147+ORcp17_148+ORcp17_149+ORcp17_173+qd[1];
    VIcp17_273 = ORcp17_231+ORcp17_234+ORcp17_244+ORcp17_247+ORcp17_248+ORcp17_249+ORcp17_273+qd[2];
    VIcp17_373 = ORcp17_331+ORcp17_334+ORcp17_344+ORcp17_347+ORcp17_348+ORcp17_349+ORcp17_373+qd[3];
    ACcp17_173 = qdd[1]+OMcp17_233*ORcp17_334+OMcp17_234*ORcp17_344+OMcp17_246*ORcp17_347+OMcp17_247*ORcp17_348+OMcp17_248
 *ORcp17_349+OMcp17_249*ORcp17_373+OMcp17_26*ORcp17_331-OMcp17_333*ORcp17_234-OMcp17_334*ORcp17_244-OMcp17_346*ORcp17_247-
 OMcp17_347*ORcp17_248-OMcp17_348*ORcp17_249-OMcp17_349*ORcp17_273-OMcp17_36*ORcp17_231+OPcp17_233*RLcp17_334+OPcp17_234*
 RLcp17_344+OPcp17_246*RLcp17_347+OPcp17_247*RLcp17_348+OPcp17_248*RLcp17_349+OPcp17_249*RLcp17_373+OPcp17_26*RLcp17_331-
 OPcp17_333*RLcp17_234-OPcp17_334*RLcp17_244-OPcp17_346*RLcp17_247-OPcp17_347*RLcp17_248-OPcp17_348*RLcp17_249-OPcp17_349*
 RLcp17_273-OPcp17_36*RLcp17_231;
    ACcp17_273 = qdd[2]-OMcp17_133*ORcp17_334-OMcp17_134*ORcp17_344-OMcp17_146*ORcp17_347-OMcp17_147*ORcp17_348-OMcp17_148
 *ORcp17_349-OMcp17_149*ORcp17_373-OMcp17_16*ORcp17_331+OMcp17_333*ORcp17_134+OMcp17_334*ORcp17_144+OMcp17_346*ORcp17_147+
 OMcp17_347*ORcp17_148+OMcp17_348*ORcp17_149+OMcp17_349*ORcp17_173+OMcp17_36*ORcp17_131-OPcp17_133*RLcp17_334-OPcp17_134*
 RLcp17_344-OPcp17_146*RLcp17_347-OPcp17_147*RLcp17_348-OPcp17_148*RLcp17_349-OPcp17_149*RLcp17_373-OPcp17_16*RLcp17_331+
 OPcp17_333*RLcp17_134+OPcp17_334*RLcp17_144+OPcp17_346*RLcp17_147+OPcp17_347*RLcp17_148+OPcp17_348*RLcp17_149+OPcp17_349*
 RLcp17_173+OPcp17_36*RLcp17_131;
    ACcp17_373 = qdd[3]+OMcp17_133*ORcp17_234+OMcp17_134*ORcp17_244+OMcp17_146*ORcp17_247+OMcp17_147*ORcp17_248+OMcp17_148
 *ORcp17_249+OMcp17_149*ORcp17_273+OMcp17_16*ORcp17_231-OMcp17_233*ORcp17_134-OMcp17_234*ORcp17_144-OMcp17_246*ORcp17_147-
 OMcp17_247*ORcp17_148-OMcp17_248*ORcp17_149-OMcp17_249*ORcp17_173-OMcp17_26*ORcp17_131+OPcp17_133*RLcp17_234+OPcp17_134*
 RLcp17_244+OPcp17_146*RLcp17_247+OPcp17_147*RLcp17_248+OPcp17_148*RLcp17_249+OPcp17_149*RLcp17_273+OPcp17_16*RLcp17_231-
 OPcp17_233*RLcp17_134-OPcp17_234*RLcp17_144-OPcp17_246*RLcp17_147-OPcp17_247*RLcp17_148-OPcp17_248*RLcp17_149-OPcp17_249*
 RLcp17_173-OPcp17_26*RLcp17_131;

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp17_173;
    sens->P[2] = POcp17_273;
    sens->P[3] = POcp17_373;
    sens->R[1][1] = ROcp17_149;
    sens->R[1][2] = ROcp17_249;
    sens->R[1][3] = ROcp17_349;
    sens->R[2][1] = ROcp17_448;
    sens->R[2][2] = ROcp17_548;
    sens->R[2][3] = ROcp17_648;
    sens->R[3][1] = ROcp17_749;
    sens->R[3][2] = ROcp17_849;
    sens->R[3][3] = ROcp17_949;
    sens->V[1] = VIcp17_173;
    sens->V[2] = VIcp17_273;
    sens->V[3] = VIcp17_373;
    sens->OM[1] = OMcp17_149;
    sens->OM[2] = OMcp17_249;
    sens->OM[3] = OMcp17_349;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp17_173_5;
    sens->J[1][6] = JTcp17_173_6;
    sens->J[1][31] = JTcp17_173_7;
    sens->J[1][32] = JTcp17_173_8;
    sens->J[1][33] = JTcp17_173_9;
    sens->J[1][34] = JTcp17_173_10;
    sens->J[1][44] = JTcp17_173_11;
    sens->J[1][45] = JTcp17_173_12;
    sens->J[1][46] = JTcp17_173_13;
    sens->J[1][47] = JTcp17_173_14;
    sens->J[1][48] = JTcp17_173_15;
    sens->J[1][49] = JTcp17_173_16;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp17_273_4;
    sens->J[2][5] = JTcp17_273_5;
    sens->J[2][6] = JTcp17_273_6;
    sens->J[2][31] = JTcp17_273_7;
    sens->J[2][32] = JTcp17_273_8;
    sens->J[2][33] = JTcp17_273_9;
    sens->J[2][34] = JTcp17_273_10;
    sens->J[2][44] = JTcp17_273_11;
    sens->J[2][45] = JTcp17_273_12;
    sens->J[2][46] = JTcp17_273_13;
    sens->J[2][47] = JTcp17_273_14;
    sens->J[2][48] = JTcp17_273_15;
    sens->J[2][49] = JTcp17_273_16;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp17_373_4;
    sens->J[3][5] = JTcp17_373_5;
    sens->J[3][6] = JTcp17_373_6;
    sens->J[3][31] = JTcp17_373_7;
    sens->J[3][32] = JTcp17_373_8;
    sens->J[3][33] = JTcp17_373_9;
    sens->J[3][34] = JTcp17_373_10;
    sens->J[3][44] = JTcp17_373_11;
    sens->J[3][45] = JTcp17_373_12;
    sens->J[3][46] = JTcp17_373_13;
    sens->J[3][47] = JTcp17_373_14;
    sens->J[3][48] = JTcp17_373_15;
    sens->J[3][49] = JTcp17_373_16;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp17_46;
    sens->J[4][32] = ROcp17_131;
    sens->J[4][33] = ROcp17_432;
    sens->J[4][34] = ROcp17_733;
    sens->J[4][44] = ROcp17_134;
    sens->J[4][45] = ROcp17_744;
    sens->J[4][46] = ROcp17_445;
    sens->J[4][47] = ROcp17_146;
    sens->J[4][48] = ROcp17_747;
    sens->J[4][49] = ROcp17_448;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp17_85;
    sens->J[5][31] = ROcp17_56;
    sens->J[5][32] = ROcp17_231;
    sens->J[5][33] = ROcp17_532;
    sens->J[5][34] = ROcp17_833;
    sens->J[5][44] = ROcp17_234;
    sens->J[5][45] = ROcp17_844;
    sens->J[5][46] = ROcp17_545;
    sens->J[5][47] = ROcp17_246;
    sens->J[5][48] = ROcp17_847;
    sens->J[5][49] = ROcp17_548;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp17_95;
    sens->J[6][31] = ROcp17_66;
    sens->J[6][32] = ROcp17_331;
    sens->J[6][33] = ROcp17_632;
    sens->J[6][34] = ROcp17_933;
    sens->J[6][44] = ROcp17_334;
    sens->J[6][45] = ROcp17_944;
    sens->J[6][46] = ROcp17_645;
    sens->J[6][47] = ROcp17_346;
    sens->J[6][48] = ROcp17_947;
    sens->J[6][49] = ROcp17_648;
    sens->A[1] = ACcp17_173;
    sens->A[2] = ACcp17_273;
    sens->A[3] = ACcp17_373;
    sens->OMP[1] = OPcp17_149;
    sens->OMP[2] = OPcp17_249;
    sens->OMP[3] = OPcp17_349;
 
// 
break;
case 19:
 


// = = Block_1_0_0_19_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp18_25 = qd[5]*C4;
    OMcp18_35 = qd[5]*S4;
    OMcp18_16 = qd[4]+qd[6]*S5;
    OMcp18_26 = OMcp18_25+ROcp18_85*qd[6];
    OMcp18_36 = OMcp18_35+ROcp18_95*qd[6];
    OPcp18_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp18_26 = ROcp18_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp18_35*S5-ROcp18_95*qd[4]);
    OPcp18_36 = ROcp18_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp18_25*S5-ROcp18_85*qd[4]);

// = = Block_1_0_0_19_0_4 = = 
 
// Sensor Kinematics 


    ROcp18_131 = ROcp18_16*C31-S31*S5;
    ROcp18_231 = ROcp18_26*C31-ROcp18_85*S31;
    ROcp18_331 = ROcp18_36*C31-ROcp18_95*S31;
    ROcp18_731 = ROcp18_16*S31+C31*S5;
    ROcp18_831 = ROcp18_26*S31+ROcp18_85*C31;
    ROcp18_931 = ROcp18_36*S31+ROcp18_95*C31;
    ROcp18_432 = ROcp18_46*C32+ROcp18_731*S32;
    ROcp18_532 = ROcp18_56*C32+ROcp18_831*S32;
    ROcp18_632 = ROcp18_66*C32+ROcp18_931*S32;
    ROcp18_732 = -(ROcp18_46*S32-ROcp18_731*C32);
    ROcp18_832 = -(ROcp18_56*S32-ROcp18_831*C32);
    ROcp18_932 = -(ROcp18_66*S32-ROcp18_931*C32);
    ROcp18_133 = ROcp18_131*C33-ROcp18_732*S33;
    ROcp18_233 = ROcp18_231*C33-ROcp18_832*S33;
    ROcp18_333 = ROcp18_331*C33-ROcp18_932*S33;
    ROcp18_733 = ROcp18_131*S33+ROcp18_732*C33;
    ROcp18_833 = ROcp18_231*S33+ROcp18_832*C33;
    ROcp18_933 = ROcp18_331*S33+ROcp18_932*C33;
    ROcp18_134 = ROcp18_133*C34+ROcp18_432*S34;
    ROcp18_234 = ROcp18_233*C34+ROcp18_532*S34;
    ROcp18_334 = ROcp18_333*C34+ROcp18_632*S34;
    ROcp18_434 = -(ROcp18_133*S34-ROcp18_432*C34);
    ROcp18_534 = -(ROcp18_233*S34-ROcp18_532*C34);
    ROcp18_634 = -(ROcp18_333*S34-ROcp18_632*C34);
    RLcp18_131 = ROcp18_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp18_231 = ROcp18_26*s->dpt[1][3]+ROcp18_85*s->dpt[3][3];
    RLcp18_331 = ROcp18_36*s->dpt[1][3]+ROcp18_95*s->dpt[3][3];
    OMcp18_131 = OMcp18_16+ROcp18_46*qd[31];
    OMcp18_231 = OMcp18_26+ROcp18_56*qd[31];
    OMcp18_331 = OMcp18_36+ROcp18_66*qd[31];
    ORcp18_131 = OMcp18_26*RLcp18_331-OMcp18_36*RLcp18_231;
    ORcp18_231 = -(OMcp18_16*RLcp18_331-OMcp18_36*RLcp18_131);
    ORcp18_331 = OMcp18_16*RLcp18_231-OMcp18_26*RLcp18_131;
    OMcp18_132 = OMcp18_131+ROcp18_131*qd[32];
    OMcp18_232 = OMcp18_231+ROcp18_231*qd[32];
    OMcp18_332 = OMcp18_331+ROcp18_331*qd[32];
    OMcp18_133 = OMcp18_132+ROcp18_432*qd[33];
    OMcp18_233 = OMcp18_232+ROcp18_532*qd[33];
    OMcp18_333 = OMcp18_332+ROcp18_632*qd[33];
    OPcp18_133 = OPcp18_16+ROcp18_131*qdd[32]+ROcp18_432*qdd[33]+ROcp18_46*qdd[31]+qd[31]*(OMcp18_26*ROcp18_66-OMcp18_36*
 ROcp18_56)+qd[32]*(OMcp18_231*ROcp18_331-OMcp18_331*ROcp18_231)+qd[33]*(OMcp18_232*ROcp18_632-OMcp18_332*ROcp18_532);
    OPcp18_233 = OPcp18_26+ROcp18_231*qdd[32]+ROcp18_532*qdd[33]+ROcp18_56*qdd[31]-qd[31]*(OMcp18_16*ROcp18_66-OMcp18_36*
 ROcp18_46)-qd[32]*(OMcp18_131*ROcp18_331-OMcp18_331*ROcp18_131)-qd[33]*(OMcp18_132*ROcp18_632-OMcp18_332*ROcp18_432);
    OPcp18_333 = OPcp18_36+ROcp18_331*qdd[32]+ROcp18_632*qdd[33]+ROcp18_66*qdd[31]+qd[31]*(OMcp18_16*ROcp18_56-OMcp18_26*
 ROcp18_46)+qd[32]*(OMcp18_131*ROcp18_231-OMcp18_231*ROcp18_131)+qd[33]*(OMcp18_132*ROcp18_532-OMcp18_232*ROcp18_432);
    RLcp18_134 = ROcp18_733*s->dpt[3][21];
    RLcp18_234 = ROcp18_833*s->dpt[3][21];
    RLcp18_334 = ROcp18_933*s->dpt[3][21];
    OMcp18_134 = OMcp18_133+ROcp18_733*qd[34];
    OMcp18_234 = OMcp18_233+ROcp18_833*qd[34];
    OMcp18_334 = OMcp18_333+ROcp18_933*qd[34];
    ORcp18_134 = OMcp18_233*RLcp18_334-OMcp18_333*RLcp18_234;
    ORcp18_234 = -(OMcp18_133*RLcp18_334-OMcp18_333*RLcp18_134);
    ORcp18_334 = OMcp18_133*RLcp18_234-OMcp18_233*RLcp18_134;
    OPcp18_134 = OPcp18_133+ROcp18_733*qdd[34]+qd[34]*(OMcp18_233*ROcp18_933-OMcp18_333*ROcp18_833);
    OPcp18_234 = OPcp18_233+ROcp18_833*qdd[34]-qd[34]*(OMcp18_133*ROcp18_933-OMcp18_333*ROcp18_733);
    OPcp18_334 = OPcp18_333+ROcp18_933*qdd[34]+qd[34]*(OMcp18_133*ROcp18_833-OMcp18_233*ROcp18_733);

// = = Block_1_0_0_19_0_6 = = 
 
// Sensor Kinematics 


    ROcp18_444 = ROcp18_434*C44+ROcp18_733*S44;
    ROcp18_544 = ROcp18_534*C44+ROcp18_833*S44;
    ROcp18_644 = ROcp18_634*C44+ROcp18_933*S44;
    ROcp18_744 = -(ROcp18_434*S44-ROcp18_733*C44);
    ROcp18_844 = -(ROcp18_534*S44-ROcp18_833*C44);
    ROcp18_944 = -(ROcp18_634*S44-ROcp18_933*C44);
    ROcp18_145 = ROcp18_134*C45+ROcp18_444*S45;
    ROcp18_245 = ROcp18_234*C45+ROcp18_544*S45;
    ROcp18_345 = ROcp18_334*C45+ROcp18_644*S45;
    ROcp18_445 = -(ROcp18_134*S45-ROcp18_444*C45);
    ROcp18_545 = -(ROcp18_234*S45-ROcp18_544*C45);
    ROcp18_645 = -(ROcp18_334*S45-ROcp18_644*C45);
    ROcp18_146 = ROcp18_145*C46-ROcp18_744*S46;
    ROcp18_246 = ROcp18_245*C46-ROcp18_844*S46;
    ROcp18_346 = ROcp18_345*C46-ROcp18_944*S46;
    ROcp18_746 = ROcp18_145*S46+ROcp18_744*C46;
    ROcp18_846 = ROcp18_245*S46+ROcp18_844*C46;
    ROcp18_946 = ROcp18_345*S46+ROcp18_944*C46;
    ROcp18_447 = ROcp18_445*C47+ROcp18_746*S47;
    ROcp18_547 = ROcp18_545*C47+ROcp18_846*S47;
    ROcp18_647 = ROcp18_645*C47+ROcp18_946*S47;
    ROcp18_747 = -(ROcp18_445*S47-ROcp18_746*C47);
    ROcp18_847 = -(ROcp18_545*S47-ROcp18_846*C47);
    ROcp18_947 = -(ROcp18_645*S47-ROcp18_946*C47);
    ROcp18_148 = ROcp18_146*C48+ROcp18_447*S48;
    ROcp18_248 = ROcp18_246*C48+ROcp18_547*S48;
    ROcp18_348 = ROcp18_346*C48+ROcp18_647*S48;
    ROcp18_448 = -(ROcp18_146*S48-ROcp18_447*C48);
    ROcp18_548 = -(ROcp18_246*S48-ROcp18_547*C48);
    ROcp18_648 = -(ROcp18_346*S48-ROcp18_647*C48);
    ROcp18_149 = ROcp18_148*C49-ROcp18_747*S49;
    ROcp18_249 = ROcp18_248*C49-ROcp18_847*S49;
    ROcp18_349 = ROcp18_348*C49-ROcp18_947*S49;
    ROcp18_749 = ROcp18_148*S49+ROcp18_747*C49;
    ROcp18_849 = ROcp18_248*S49+ROcp18_847*C49;
    ROcp18_949 = ROcp18_348*S49+ROcp18_947*C49;
    ROcp18_150 = ROcp18_149*C50+ROcp18_448*S50;
    ROcp18_250 = ROcp18_249*C50+ROcp18_548*S50;
    ROcp18_350 = ROcp18_349*C50+ROcp18_648*S50;
    ROcp18_450 = -(ROcp18_149*S50-ROcp18_448*C50);
    ROcp18_550 = -(ROcp18_249*S50-ROcp18_548*C50);
    ROcp18_650 = -(ROcp18_349*S50-ROcp18_648*C50);
    RLcp18_144 = ROcp18_134*s->dpt[1][24]+ROcp18_434*s->dpt[2][24]+ROcp18_733*s->dpt[3][24];
    RLcp18_244 = ROcp18_234*s->dpt[1][24]+ROcp18_534*s->dpt[2][24]+ROcp18_833*s->dpt[3][24];
    RLcp18_344 = ROcp18_334*s->dpt[1][24]+ROcp18_634*s->dpt[2][24]+ROcp18_933*s->dpt[3][24];
    ORcp18_144 = OMcp18_234*RLcp18_344-OMcp18_334*RLcp18_244;
    ORcp18_244 = -(OMcp18_134*RLcp18_344-OMcp18_334*RLcp18_144);
    ORcp18_344 = OMcp18_134*RLcp18_244-OMcp18_234*RLcp18_144;
    OMcp18_146 = OMcp18_134+ROcp18_445*qd[46];
    OMcp18_246 = OMcp18_234+ROcp18_545*qd[46];
    OMcp18_346 = OMcp18_334+ROcp18_645*qd[46];
    OPcp18_146 = OPcp18_134+ROcp18_445*qdd[46]+qd[46]*(OMcp18_234*ROcp18_645-OMcp18_334*ROcp18_545);
    OPcp18_246 = OPcp18_234+ROcp18_545*qdd[46]-qd[46]*(OMcp18_134*ROcp18_645-OMcp18_334*ROcp18_445);
    OPcp18_346 = OPcp18_334+ROcp18_645*qdd[46]+qd[46]*(OMcp18_134*ROcp18_545-OMcp18_234*ROcp18_445);
    RLcp18_147 = ROcp18_445*s->dpt[2][41]+ROcp18_746*s->dpt[3][41];
    RLcp18_247 = ROcp18_545*s->dpt[2][41]+ROcp18_846*s->dpt[3][41];
    RLcp18_347 = ROcp18_645*s->dpt[2][41]+ROcp18_946*s->dpt[3][41];
    OMcp18_147 = OMcp18_146+ROcp18_146*qd[47];
    OMcp18_247 = OMcp18_246+ROcp18_246*qd[47];
    OMcp18_347 = OMcp18_346+ROcp18_346*qd[47];
    ORcp18_147 = OMcp18_246*RLcp18_347-OMcp18_346*RLcp18_247;
    ORcp18_247 = -(OMcp18_146*RLcp18_347-OMcp18_346*RLcp18_147);
    ORcp18_347 = OMcp18_146*RLcp18_247-OMcp18_246*RLcp18_147;
    OPcp18_147 = OPcp18_146+ROcp18_146*qdd[47]+qd[47]*(OMcp18_246*ROcp18_346-OMcp18_346*ROcp18_246);
    OPcp18_247 = OPcp18_246+ROcp18_246*qdd[47]-qd[47]*(OMcp18_146*ROcp18_346-OMcp18_346*ROcp18_146);
    OPcp18_347 = OPcp18_346+ROcp18_346*qdd[47]+qd[47]*(OMcp18_146*ROcp18_246-OMcp18_246*ROcp18_146);
    RLcp18_148 = ROcp18_747*s->dpt[3][43];
    RLcp18_248 = ROcp18_847*s->dpt[3][43];
    RLcp18_348 = ROcp18_947*s->dpt[3][43];
    OMcp18_148 = OMcp18_147+ROcp18_747*qd[48];
    OMcp18_248 = OMcp18_247+ROcp18_847*qd[48];
    OMcp18_348 = OMcp18_347+ROcp18_947*qd[48];
    ORcp18_148 = OMcp18_247*RLcp18_348-OMcp18_347*RLcp18_248;
    ORcp18_248 = -(OMcp18_147*RLcp18_348-OMcp18_347*RLcp18_148);
    ORcp18_348 = OMcp18_147*RLcp18_248-OMcp18_247*RLcp18_148;
    OPcp18_148 = OPcp18_147+ROcp18_747*qdd[48]+qd[48]*(OMcp18_247*ROcp18_947-OMcp18_347*ROcp18_847);
    OPcp18_248 = OPcp18_247+ROcp18_847*qdd[48]-qd[48]*(OMcp18_147*ROcp18_947-OMcp18_347*ROcp18_747);
    OPcp18_348 = OPcp18_347+ROcp18_947*qdd[48]+qd[48]*(OMcp18_147*ROcp18_847-OMcp18_247*ROcp18_747);
    RLcp18_149 = ROcp18_148*s->dpt[1][46]+ROcp18_747*s->dpt[3][46];
    RLcp18_249 = ROcp18_248*s->dpt[1][46]+ROcp18_847*s->dpt[3][46];
    RLcp18_349 = ROcp18_348*s->dpt[1][46]+ROcp18_947*s->dpt[3][46];
    OMcp18_149 = OMcp18_148+ROcp18_448*qd[49];
    OMcp18_249 = OMcp18_248+ROcp18_548*qd[49];
    OMcp18_349 = OMcp18_348+ROcp18_648*qd[49];
    ORcp18_149 = OMcp18_248*RLcp18_349-OMcp18_348*RLcp18_249;
    ORcp18_249 = -(OMcp18_148*RLcp18_349-OMcp18_348*RLcp18_149);
    ORcp18_349 = OMcp18_148*RLcp18_249-OMcp18_248*RLcp18_149;
    OPcp18_149 = OPcp18_148+ROcp18_448*qdd[49]+qd[49]*(OMcp18_248*ROcp18_648-OMcp18_348*ROcp18_548);
    OPcp18_249 = OPcp18_248+ROcp18_548*qdd[49]-qd[49]*(OMcp18_148*ROcp18_648-OMcp18_348*ROcp18_448);
    OPcp18_349 = OPcp18_348+ROcp18_648*qdd[49]+qd[49]*(OMcp18_148*ROcp18_548-OMcp18_248*ROcp18_448);
    RLcp18_150 = ROcp18_149*s->dpt[1][48]+ROcp18_749*s->dpt[3][48];
    RLcp18_250 = ROcp18_249*s->dpt[1][48]+ROcp18_849*s->dpt[3][48];
    RLcp18_350 = ROcp18_349*s->dpt[1][48]+ROcp18_949*s->dpt[3][48];
    OMcp18_150 = OMcp18_149+ROcp18_749*qd[50];
    OMcp18_250 = OMcp18_249+ROcp18_849*qd[50];
    OMcp18_350 = OMcp18_349+ROcp18_949*qd[50];
    ORcp18_150 = OMcp18_249*RLcp18_350-OMcp18_349*RLcp18_250;
    ORcp18_250 = -(OMcp18_149*RLcp18_350-OMcp18_349*RLcp18_150);
    ORcp18_350 = OMcp18_149*RLcp18_250-OMcp18_249*RLcp18_150;
    OPcp18_150 = OPcp18_149+ROcp18_749*qdd[50]+qd[50]*(OMcp18_249*ROcp18_949-OMcp18_349*ROcp18_849);
    OPcp18_250 = OPcp18_249+ROcp18_849*qdd[50]-qd[50]*(OMcp18_149*ROcp18_949-OMcp18_349*ROcp18_749);
    OPcp18_350 = OPcp18_349+ROcp18_949*qdd[50]+qd[50]*(OMcp18_149*ROcp18_849-OMcp18_249*ROcp18_749);
    RLcp18_174 = ROcp18_150*s->dpt[1][51]+ROcp18_450*s->dpt[2][51]+ROcp18_749*s->dpt[3][51];
    RLcp18_274 = ROcp18_250*s->dpt[1][51]+ROcp18_550*s->dpt[2][51]+ROcp18_849*s->dpt[3][51];
    RLcp18_374 = ROcp18_350*s->dpt[1][51]+ROcp18_650*s->dpt[2][51]+ROcp18_949*s->dpt[3][51];
    POcp18_174 = RLcp18_131+RLcp18_134+RLcp18_144+RLcp18_147+RLcp18_148+RLcp18_149+RLcp18_150+RLcp18_174+q[1];
    POcp18_274 = RLcp18_231+RLcp18_234+RLcp18_244+RLcp18_247+RLcp18_248+RLcp18_249+RLcp18_250+RLcp18_274+q[2];
    POcp18_374 = RLcp18_331+RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350+RLcp18_374+q[3];
    JTcp18_274_4 = -(RLcp18_331+RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350+RLcp18_374);
    JTcp18_374_4 = RLcp18_231+RLcp18_234+RLcp18_244+RLcp18_247+RLcp18_248+RLcp18_249+RLcp18_250+RLcp18_274;
    JTcp18_174_5 = C4*(RLcp18_331+RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350+RLcp18_374)-S4*(
 RLcp18_231+RLcp18_234)-S4*(RLcp18_244+RLcp18_247)-S4*(RLcp18_248+RLcp18_249)-S4*(RLcp18_250+RLcp18_274);
    JTcp18_274_5 = S4*(RLcp18_131+RLcp18_134+RLcp18_144+RLcp18_147+RLcp18_148+RLcp18_149+RLcp18_150+RLcp18_174);
    JTcp18_374_5 = -C4*(RLcp18_131+RLcp18_134+RLcp18_144+RLcp18_147+RLcp18_148+RLcp18_149+RLcp18_150+RLcp18_174);
    JTcp18_174_6 = ROcp18_85*(RLcp18_331+RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350+RLcp18_374)-
 ROcp18_95*(RLcp18_231+RLcp18_234)-ROcp18_95*(RLcp18_244+RLcp18_247)-ROcp18_95*(RLcp18_248+RLcp18_249)-ROcp18_95*(RLcp18_250+
 RLcp18_274);
    JTcp18_274_6 = RLcp18_174*ROcp18_95-RLcp18_350*S5-RLcp18_374*S5+ROcp18_95*(RLcp18_131+RLcp18_134+RLcp18_144+RLcp18_147
 +RLcp18_148+RLcp18_149+RLcp18_150)-S5*(RLcp18_331+RLcp18_334)-S5*(RLcp18_344+RLcp18_347)-S5*(RLcp18_348+RLcp18_349);
    JTcp18_374_6 = RLcp18_250*S5-ROcp18_85*(RLcp18_131+RLcp18_134+RLcp18_144+RLcp18_147+RLcp18_148+RLcp18_149+RLcp18_150)+
 S5*(RLcp18_231+RLcp18_234)+S5*(RLcp18_244+RLcp18_247)+S5*(RLcp18_248+RLcp18_249)-RLcp18_174*ROcp18_85+RLcp18_274*S5;
    JTcp18_174_7 = ROcp18_56*(RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350)-ROcp18_66*(RLcp18_234+
 RLcp18_244)-ROcp18_66*(RLcp18_247+RLcp18_248)-ROcp18_66*(RLcp18_249+RLcp18_250)-RLcp18_274*ROcp18_66+RLcp18_374*ROcp18_56;
    JTcp18_274_7 = RLcp18_174*ROcp18_66-RLcp18_374*ROcp18_46-ROcp18_46*(RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+
 RLcp18_349+RLcp18_350)+ROcp18_66*(RLcp18_134+RLcp18_144)+ROcp18_66*(RLcp18_147+RLcp18_148)+ROcp18_66*(RLcp18_149+RLcp18_150);
    JTcp18_374_7 = ROcp18_46*(RLcp18_234+RLcp18_244+RLcp18_247+RLcp18_248+RLcp18_249+RLcp18_250)-ROcp18_56*(RLcp18_134+
 RLcp18_144)-ROcp18_56*(RLcp18_147+RLcp18_148)-ROcp18_56*(RLcp18_149+RLcp18_150)-RLcp18_174*ROcp18_56+RLcp18_274*ROcp18_46;
    JTcp18_174_8 = ROcp18_231*(RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350)-ROcp18_331*(RLcp18_234+
 RLcp18_244)-ROcp18_331*(RLcp18_247+RLcp18_248)-ROcp18_331*(RLcp18_249+RLcp18_250)-RLcp18_274*ROcp18_331+RLcp18_374*
 ROcp18_231;
    JTcp18_274_8 = RLcp18_174*ROcp18_331-RLcp18_374*ROcp18_131-ROcp18_131*(RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+
 RLcp18_349+RLcp18_350)+ROcp18_331*(RLcp18_134+RLcp18_144)+ROcp18_331*(RLcp18_147+RLcp18_148)+ROcp18_331*(RLcp18_149+
 RLcp18_150);
    JTcp18_374_8 = ROcp18_131*(RLcp18_234+RLcp18_244+RLcp18_247+RLcp18_248+RLcp18_249+RLcp18_250)-ROcp18_231*(RLcp18_134+
 RLcp18_144)-ROcp18_231*(RLcp18_147+RLcp18_148)-ROcp18_231*(RLcp18_149+RLcp18_150)-RLcp18_174*ROcp18_231+RLcp18_274*
 ROcp18_131;
    JTcp18_174_9 = ROcp18_532*(RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350)-ROcp18_632*(RLcp18_234+
 RLcp18_244)-ROcp18_632*(RLcp18_247+RLcp18_248)-ROcp18_632*(RLcp18_249+RLcp18_250)-RLcp18_274*ROcp18_632+RLcp18_374*
 ROcp18_532;
    JTcp18_274_9 = RLcp18_174*ROcp18_632-RLcp18_374*ROcp18_432-ROcp18_432*(RLcp18_334+RLcp18_344+RLcp18_347+RLcp18_348+
 RLcp18_349+RLcp18_350)+ROcp18_632*(RLcp18_134+RLcp18_144)+ROcp18_632*(RLcp18_147+RLcp18_148)+ROcp18_632*(RLcp18_149+
 RLcp18_150);
    JTcp18_374_9 = ROcp18_432*(RLcp18_234+RLcp18_244+RLcp18_247+RLcp18_248+RLcp18_249+RLcp18_250)-ROcp18_532*(RLcp18_134+
 RLcp18_144)-ROcp18_532*(RLcp18_147+RLcp18_148)-ROcp18_532*(RLcp18_149+RLcp18_150)-RLcp18_174*ROcp18_532+RLcp18_274*
 ROcp18_432;
    JTcp18_174_10 = ROcp18_833*(RLcp18_344+RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350+RLcp18_374)-ROcp18_933*(RLcp18_244+
 RLcp18_247)-ROcp18_933*(RLcp18_248+RLcp18_249)-ROcp18_933*(RLcp18_250+RLcp18_274);
    JTcp18_274_10 = -(ROcp18_733*(RLcp18_344+RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350+RLcp18_374)-ROcp18_933*(
 RLcp18_144+RLcp18_147)-ROcp18_933*(RLcp18_148+RLcp18_149)-ROcp18_933*(RLcp18_150+RLcp18_174));
    JTcp18_374_10 = ROcp18_733*(RLcp18_244+RLcp18_247+RLcp18_248+RLcp18_249+RLcp18_250+RLcp18_274)-ROcp18_833*(RLcp18_144+
 RLcp18_147)-ROcp18_833*(RLcp18_148+RLcp18_149)-ROcp18_833*(RLcp18_150+RLcp18_174);
    JTcp18_174_11 = ROcp18_234*(RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350)-ROcp18_334*(RLcp18_247+RLcp18_248)-ROcp18_334
 *(RLcp18_249+RLcp18_250)-RLcp18_274*ROcp18_334+RLcp18_374*ROcp18_234;
    JTcp18_274_11 = RLcp18_174*ROcp18_334-RLcp18_374*ROcp18_134-ROcp18_134*(RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350)+
 ROcp18_334*(RLcp18_147+RLcp18_148)+ROcp18_334*(RLcp18_149+RLcp18_150);
    JTcp18_374_11 = ROcp18_134*(RLcp18_247+RLcp18_248+RLcp18_249+RLcp18_250)-ROcp18_234*(RLcp18_147+RLcp18_148)-ROcp18_234
 *(RLcp18_149+RLcp18_150)-RLcp18_174*ROcp18_234+RLcp18_274*ROcp18_134;
    JTcp18_174_12 = ROcp18_844*(RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350)-ROcp18_944*(RLcp18_247+RLcp18_248)-ROcp18_944
 *(RLcp18_249+RLcp18_250)-RLcp18_274*ROcp18_944+RLcp18_374*ROcp18_844;
    JTcp18_274_12 = RLcp18_174*ROcp18_944-RLcp18_374*ROcp18_744-ROcp18_744*(RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350)+
 ROcp18_944*(RLcp18_147+RLcp18_148)+ROcp18_944*(RLcp18_149+RLcp18_150);
    JTcp18_374_12 = ROcp18_744*(RLcp18_247+RLcp18_248+RLcp18_249+RLcp18_250)-ROcp18_844*(RLcp18_147+RLcp18_148)-ROcp18_844
 *(RLcp18_149+RLcp18_150)-RLcp18_174*ROcp18_844+RLcp18_274*ROcp18_744;
    JTcp18_174_13 = ROcp18_545*(RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350)-ROcp18_645*(RLcp18_247+RLcp18_248)-ROcp18_645
 *(RLcp18_249+RLcp18_250)-RLcp18_274*ROcp18_645+RLcp18_374*ROcp18_545;
    JTcp18_274_13 = RLcp18_174*ROcp18_645-RLcp18_374*ROcp18_445-ROcp18_445*(RLcp18_347+RLcp18_348+RLcp18_349+RLcp18_350)+
 ROcp18_645*(RLcp18_147+RLcp18_148)+ROcp18_645*(RLcp18_149+RLcp18_150);
    JTcp18_374_13 = ROcp18_445*(RLcp18_247+RLcp18_248+RLcp18_249+RLcp18_250)-ROcp18_545*(RLcp18_147+RLcp18_148)-ROcp18_545
 *(RLcp18_149+RLcp18_150)-RLcp18_174*ROcp18_545+RLcp18_274*ROcp18_445;
    JTcp18_174_14 = ROcp18_246*(RLcp18_348+RLcp18_349+RLcp18_350+RLcp18_374)-ROcp18_346*(RLcp18_248+RLcp18_249)-ROcp18_346
 *(RLcp18_250+RLcp18_274);
    JTcp18_274_14 = -(ROcp18_146*(RLcp18_348+RLcp18_349+RLcp18_350+RLcp18_374)-ROcp18_346*(RLcp18_148+RLcp18_149)-
 ROcp18_346*(RLcp18_150+RLcp18_174));
    JTcp18_374_14 = ROcp18_146*(RLcp18_248+RLcp18_249+RLcp18_250+RLcp18_274)-ROcp18_246*(RLcp18_148+RLcp18_149)-ROcp18_246
 *(RLcp18_150+RLcp18_174);
    JTcp18_174_15 = ROcp18_847*(RLcp18_349+RLcp18_350)-ROcp18_947*(RLcp18_249+RLcp18_250)-RLcp18_274*ROcp18_947+RLcp18_374
 *ROcp18_847;
    JTcp18_274_15 = RLcp18_174*ROcp18_947-RLcp18_374*ROcp18_747-ROcp18_747*(RLcp18_349+RLcp18_350)+ROcp18_947*(RLcp18_149+
 RLcp18_150);
    JTcp18_374_15 = ROcp18_747*(RLcp18_249+RLcp18_250)-ROcp18_847*(RLcp18_149+RLcp18_150)-RLcp18_174*ROcp18_847+RLcp18_274
 *ROcp18_747;
    JTcp18_174_16 = ROcp18_548*(RLcp18_350+RLcp18_374)-ROcp18_648*(RLcp18_250+RLcp18_274);
    JTcp18_274_16 = -(ROcp18_448*(RLcp18_350+RLcp18_374)-ROcp18_648*(RLcp18_150+RLcp18_174));
    JTcp18_374_16 = ROcp18_448*(RLcp18_250+RLcp18_274)-ROcp18_548*(RLcp18_150+RLcp18_174);
    JTcp18_174_17 = -(RLcp18_274*ROcp18_949-RLcp18_374*ROcp18_849);
    JTcp18_274_17 = RLcp18_174*ROcp18_949-RLcp18_374*ROcp18_749;
    JTcp18_374_17 = -(RLcp18_174*ROcp18_849-RLcp18_274*ROcp18_749);
    ORcp18_174 = OMcp18_250*RLcp18_374-OMcp18_350*RLcp18_274;
    ORcp18_274 = -(OMcp18_150*RLcp18_374-OMcp18_350*RLcp18_174);
    ORcp18_374 = OMcp18_150*RLcp18_274-OMcp18_250*RLcp18_174;
    VIcp18_174 = ORcp18_131+ORcp18_134+ORcp18_144+ORcp18_147+ORcp18_148+ORcp18_149+ORcp18_150+ORcp18_174+qd[1];
    VIcp18_274 = ORcp18_231+ORcp18_234+ORcp18_244+ORcp18_247+ORcp18_248+ORcp18_249+ORcp18_250+ORcp18_274+qd[2];
    VIcp18_374 = ORcp18_331+ORcp18_334+ORcp18_344+ORcp18_347+ORcp18_348+ORcp18_349+ORcp18_350+ORcp18_374+qd[3];
    ACcp18_174 = qdd[1]+OMcp18_233*ORcp18_334+OMcp18_234*ORcp18_344+OMcp18_246*ORcp18_347+OMcp18_247*ORcp18_348+OMcp18_248
 *ORcp18_349+OMcp18_249*ORcp18_350+OMcp18_250*ORcp18_374+OMcp18_26*ORcp18_331-OMcp18_333*ORcp18_234-OMcp18_334*ORcp18_244-
 OMcp18_346*ORcp18_247-OMcp18_347*ORcp18_248-OMcp18_348*ORcp18_249-OMcp18_349*ORcp18_250-OMcp18_350*ORcp18_274-OMcp18_36*
 ORcp18_231+OPcp18_233*RLcp18_334+OPcp18_234*RLcp18_344+OPcp18_246*RLcp18_347+OPcp18_247*RLcp18_348+OPcp18_248*RLcp18_349+
 OPcp18_249*RLcp18_350+OPcp18_250*RLcp18_374+OPcp18_26*RLcp18_331-OPcp18_333*RLcp18_234-OPcp18_334*RLcp18_244-OPcp18_346*
 RLcp18_247-OPcp18_347*RLcp18_248-OPcp18_348*RLcp18_249-OPcp18_349*RLcp18_250-OPcp18_350*RLcp18_274-OPcp18_36*RLcp18_231;
    ACcp18_274 = qdd[2]-OMcp18_133*ORcp18_334-OMcp18_134*ORcp18_344-OMcp18_146*ORcp18_347-OMcp18_147*ORcp18_348-OMcp18_148
 *ORcp18_349-OMcp18_149*ORcp18_350-OMcp18_150*ORcp18_374-OMcp18_16*ORcp18_331+OMcp18_333*ORcp18_134+OMcp18_334*ORcp18_144+
 OMcp18_346*ORcp18_147+OMcp18_347*ORcp18_148+OMcp18_348*ORcp18_149+OMcp18_349*ORcp18_150+OMcp18_350*ORcp18_174+OMcp18_36*
 ORcp18_131-OPcp18_133*RLcp18_334-OPcp18_134*RLcp18_344-OPcp18_146*RLcp18_347-OPcp18_147*RLcp18_348-OPcp18_148*RLcp18_349-
 OPcp18_149*RLcp18_350-OPcp18_150*RLcp18_374-OPcp18_16*RLcp18_331+OPcp18_333*RLcp18_134+OPcp18_334*RLcp18_144+OPcp18_346*
 RLcp18_147+OPcp18_347*RLcp18_148+OPcp18_348*RLcp18_149+OPcp18_349*RLcp18_150+OPcp18_350*RLcp18_174+OPcp18_36*RLcp18_131;
    ACcp18_374 = qdd[3]+OMcp18_133*ORcp18_234+OMcp18_134*ORcp18_244+OMcp18_146*ORcp18_247+OMcp18_147*ORcp18_248+OMcp18_148
 *ORcp18_249+OMcp18_149*ORcp18_250+OMcp18_150*ORcp18_274+OMcp18_16*ORcp18_231-OMcp18_233*ORcp18_134-OMcp18_234*ORcp18_144-
 OMcp18_246*ORcp18_147-OMcp18_247*ORcp18_148-OMcp18_248*ORcp18_149-OMcp18_249*ORcp18_150-OMcp18_250*ORcp18_174-OMcp18_26*
 ORcp18_131+OPcp18_133*RLcp18_234+OPcp18_134*RLcp18_244+OPcp18_146*RLcp18_247+OPcp18_147*RLcp18_248+OPcp18_148*RLcp18_249+
 OPcp18_149*RLcp18_250+OPcp18_150*RLcp18_274+OPcp18_16*RLcp18_231-OPcp18_233*RLcp18_134-OPcp18_234*RLcp18_144-OPcp18_246*
 RLcp18_147-OPcp18_247*RLcp18_148-OPcp18_248*RLcp18_149-OPcp18_249*RLcp18_150-OPcp18_250*RLcp18_174-OPcp18_26*RLcp18_131;

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp18_174;
    sens->P[2] = POcp18_274;
    sens->P[3] = POcp18_374;
    sens->R[1][1] = ROcp18_150;
    sens->R[1][2] = ROcp18_250;
    sens->R[1][3] = ROcp18_350;
    sens->R[2][1] = ROcp18_450;
    sens->R[2][2] = ROcp18_550;
    sens->R[2][3] = ROcp18_650;
    sens->R[3][1] = ROcp18_749;
    sens->R[3][2] = ROcp18_849;
    sens->R[3][3] = ROcp18_949;
    sens->V[1] = VIcp18_174;
    sens->V[2] = VIcp18_274;
    sens->V[3] = VIcp18_374;
    sens->OM[1] = OMcp18_150;
    sens->OM[2] = OMcp18_250;
    sens->OM[3] = OMcp18_350;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp18_174_5;
    sens->J[1][6] = JTcp18_174_6;
    sens->J[1][31] = JTcp18_174_7;
    sens->J[1][32] = JTcp18_174_8;
    sens->J[1][33] = JTcp18_174_9;
    sens->J[1][34] = JTcp18_174_10;
    sens->J[1][44] = JTcp18_174_11;
    sens->J[1][45] = JTcp18_174_12;
    sens->J[1][46] = JTcp18_174_13;
    sens->J[1][47] = JTcp18_174_14;
    sens->J[1][48] = JTcp18_174_15;
    sens->J[1][49] = JTcp18_174_16;
    sens->J[1][50] = JTcp18_174_17;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp18_274_4;
    sens->J[2][5] = JTcp18_274_5;
    sens->J[2][6] = JTcp18_274_6;
    sens->J[2][31] = JTcp18_274_7;
    sens->J[2][32] = JTcp18_274_8;
    sens->J[2][33] = JTcp18_274_9;
    sens->J[2][34] = JTcp18_274_10;
    sens->J[2][44] = JTcp18_274_11;
    sens->J[2][45] = JTcp18_274_12;
    sens->J[2][46] = JTcp18_274_13;
    sens->J[2][47] = JTcp18_274_14;
    sens->J[2][48] = JTcp18_274_15;
    sens->J[2][49] = JTcp18_274_16;
    sens->J[2][50] = JTcp18_274_17;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp18_374_4;
    sens->J[3][5] = JTcp18_374_5;
    sens->J[3][6] = JTcp18_374_6;
    sens->J[3][31] = JTcp18_374_7;
    sens->J[3][32] = JTcp18_374_8;
    sens->J[3][33] = JTcp18_374_9;
    sens->J[3][34] = JTcp18_374_10;
    sens->J[3][44] = JTcp18_374_11;
    sens->J[3][45] = JTcp18_374_12;
    sens->J[3][46] = JTcp18_374_13;
    sens->J[3][47] = JTcp18_374_14;
    sens->J[3][48] = JTcp18_374_15;
    sens->J[3][49] = JTcp18_374_16;
    sens->J[3][50] = JTcp18_374_17;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp18_46;
    sens->J[4][32] = ROcp18_131;
    sens->J[4][33] = ROcp18_432;
    sens->J[4][34] = ROcp18_733;
    sens->J[4][44] = ROcp18_134;
    sens->J[4][45] = ROcp18_744;
    sens->J[4][46] = ROcp18_445;
    sens->J[4][47] = ROcp18_146;
    sens->J[4][48] = ROcp18_747;
    sens->J[4][49] = ROcp18_448;
    sens->J[4][50] = ROcp18_749;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp18_85;
    sens->J[5][31] = ROcp18_56;
    sens->J[5][32] = ROcp18_231;
    sens->J[5][33] = ROcp18_532;
    sens->J[5][34] = ROcp18_833;
    sens->J[5][44] = ROcp18_234;
    sens->J[5][45] = ROcp18_844;
    sens->J[5][46] = ROcp18_545;
    sens->J[5][47] = ROcp18_246;
    sens->J[5][48] = ROcp18_847;
    sens->J[5][49] = ROcp18_548;
    sens->J[5][50] = ROcp18_849;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp18_95;
    sens->J[6][31] = ROcp18_66;
    sens->J[6][32] = ROcp18_331;
    sens->J[6][33] = ROcp18_632;
    sens->J[6][34] = ROcp18_933;
    sens->J[6][44] = ROcp18_334;
    sens->J[6][45] = ROcp18_944;
    sens->J[6][46] = ROcp18_645;
    sens->J[6][47] = ROcp18_346;
    sens->J[6][48] = ROcp18_947;
    sens->J[6][49] = ROcp18_648;
    sens->J[6][50] = ROcp18_949;
    sens->A[1] = ACcp18_174;
    sens->A[2] = ACcp18_274;
    sens->A[3] = ACcp18_374;
    sens->OMP[1] = OPcp18_150;
    sens->OMP[2] = OPcp18_250;
    sens->OMP[3] = OPcp18_350;
 
// 
break;
case 20:
 


// = = Block_1_0_0_20_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp19_25 = qd[5]*C4;
    OMcp19_35 = qd[5]*S4;
    OMcp19_16 = qd[4]+qd[6]*S5;
    OMcp19_26 = OMcp19_25+ROcp19_85*qd[6];
    OMcp19_36 = OMcp19_35+ROcp19_95*qd[6];
    OPcp19_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp19_26 = ROcp19_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp19_35*S5-ROcp19_95*qd[4]);
    OPcp19_36 = ROcp19_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp19_25*S5-ROcp19_85*qd[4]);

// = = Block_1_0_0_20_0_4 = = 
 
// Sensor Kinematics 


    ROcp19_131 = ROcp19_16*C31-S31*S5;
    ROcp19_231 = ROcp19_26*C31-ROcp19_85*S31;
    ROcp19_331 = ROcp19_36*C31-ROcp19_95*S31;
    ROcp19_731 = ROcp19_16*S31+C31*S5;
    ROcp19_831 = ROcp19_26*S31+ROcp19_85*C31;
    ROcp19_931 = ROcp19_36*S31+ROcp19_95*C31;
    ROcp19_432 = ROcp19_46*C32+ROcp19_731*S32;
    ROcp19_532 = ROcp19_56*C32+ROcp19_831*S32;
    ROcp19_632 = ROcp19_66*C32+ROcp19_931*S32;
    ROcp19_732 = -(ROcp19_46*S32-ROcp19_731*C32);
    ROcp19_832 = -(ROcp19_56*S32-ROcp19_831*C32);
    ROcp19_932 = -(ROcp19_66*S32-ROcp19_931*C32);
    ROcp19_133 = ROcp19_131*C33-ROcp19_732*S33;
    ROcp19_233 = ROcp19_231*C33-ROcp19_832*S33;
    ROcp19_333 = ROcp19_331*C33-ROcp19_932*S33;
    ROcp19_733 = ROcp19_131*S33+ROcp19_732*C33;
    ROcp19_833 = ROcp19_231*S33+ROcp19_832*C33;
    ROcp19_933 = ROcp19_331*S33+ROcp19_932*C33;
    ROcp19_134 = ROcp19_133*C34+ROcp19_432*S34;
    ROcp19_234 = ROcp19_233*C34+ROcp19_532*S34;
    ROcp19_334 = ROcp19_333*C34+ROcp19_632*S34;
    ROcp19_434 = -(ROcp19_133*S34-ROcp19_432*C34);
    ROcp19_534 = -(ROcp19_233*S34-ROcp19_532*C34);
    ROcp19_634 = -(ROcp19_333*S34-ROcp19_632*C34);
    RLcp19_131 = ROcp19_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp19_231 = ROcp19_26*s->dpt[1][3]+ROcp19_85*s->dpt[3][3];
    RLcp19_331 = ROcp19_36*s->dpt[1][3]+ROcp19_95*s->dpt[3][3];
    OMcp19_131 = OMcp19_16+ROcp19_46*qd[31];
    OMcp19_231 = OMcp19_26+ROcp19_56*qd[31];
    OMcp19_331 = OMcp19_36+ROcp19_66*qd[31];
    ORcp19_131 = OMcp19_26*RLcp19_331-OMcp19_36*RLcp19_231;
    ORcp19_231 = -(OMcp19_16*RLcp19_331-OMcp19_36*RLcp19_131);
    ORcp19_331 = OMcp19_16*RLcp19_231-OMcp19_26*RLcp19_131;
    OMcp19_132 = OMcp19_131+ROcp19_131*qd[32];
    OMcp19_232 = OMcp19_231+ROcp19_231*qd[32];
    OMcp19_332 = OMcp19_331+ROcp19_331*qd[32];
    OMcp19_133 = OMcp19_132+ROcp19_432*qd[33];
    OMcp19_233 = OMcp19_232+ROcp19_532*qd[33];
    OMcp19_333 = OMcp19_332+ROcp19_632*qd[33];
    OPcp19_133 = OPcp19_16+ROcp19_131*qdd[32]+ROcp19_432*qdd[33]+ROcp19_46*qdd[31]+qd[31]*(OMcp19_26*ROcp19_66-OMcp19_36*
 ROcp19_56)+qd[32]*(OMcp19_231*ROcp19_331-OMcp19_331*ROcp19_231)+qd[33]*(OMcp19_232*ROcp19_632-OMcp19_332*ROcp19_532);
    OPcp19_233 = OPcp19_26+ROcp19_231*qdd[32]+ROcp19_532*qdd[33]+ROcp19_56*qdd[31]-qd[31]*(OMcp19_16*ROcp19_66-OMcp19_36*
 ROcp19_46)-qd[32]*(OMcp19_131*ROcp19_331-OMcp19_331*ROcp19_131)-qd[33]*(OMcp19_132*ROcp19_632-OMcp19_332*ROcp19_432);
    OPcp19_333 = OPcp19_36+ROcp19_331*qdd[32]+ROcp19_632*qdd[33]+ROcp19_66*qdd[31]+qd[31]*(OMcp19_16*ROcp19_56-OMcp19_26*
 ROcp19_46)+qd[32]*(OMcp19_131*ROcp19_231-OMcp19_231*ROcp19_131)+qd[33]*(OMcp19_132*ROcp19_532-OMcp19_232*ROcp19_432);
    RLcp19_134 = ROcp19_733*s->dpt[3][21];
    RLcp19_234 = ROcp19_833*s->dpt[3][21];
    RLcp19_334 = ROcp19_933*s->dpt[3][21];
    OMcp19_134 = OMcp19_133+ROcp19_733*qd[34];
    OMcp19_234 = OMcp19_233+ROcp19_833*qd[34];
    OMcp19_334 = OMcp19_333+ROcp19_933*qd[34];
    ORcp19_134 = OMcp19_233*RLcp19_334-OMcp19_333*RLcp19_234;
    ORcp19_234 = -(OMcp19_133*RLcp19_334-OMcp19_333*RLcp19_134);
    ORcp19_334 = OMcp19_133*RLcp19_234-OMcp19_233*RLcp19_134;
    OPcp19_134 = OPcp19_133+ROcp19_733*qdd[34]+qd[34]*(OMcp19_233*ROcp19_933-OMcp19_333*ROcp19_833);
    OPcp19_234 = OPcp19_233+ROcp19_833*qdd[34]-qd[34]*(OMcp19_133*ROcp19_933-OMcp19_333*ROcp19_733);
    OPcp19_334 = OPcp19_333+ROcp19_933*qdd[34]+qd[34]*(OMcp19_133*ROcp19_833-OMcp19_233*ROcp19_733);

// = = Block_1_0_0_20_0_6 = = 
 
// Sensor Kinematics 


    ROcp19_444 = ROcp19_434*C44+ROcp19_733*S44;
    ROcp19_544 = ROcp19_534*C44+ROcp19_833*S44;
    ROcp19_644 = ROcp19_634*C44+ROcp19_933*S44;
    ROcp19_744 = -(ROcp19_434*S44-ROcp19_733*C44);
    ROcp19_844 = -(ROcp19_534*S44-ROcp19_833*C44);
    ROcp19_944 = -(ROcp19_634*S44-ROcp19_933*C44);
    ROcp19_145 = ROcp19_134*C45+ROcp19_444*S45;
    ROcp19_245 = ROcp19_234*C45+ROcp19_544*S45;
    ROcp19_345 = ROcp19_334*C45+ROcp19_644*S45;
    ROcp19_445 = -(ROcp19_134*S45-ROcp19_444*C45);
    ROcp19_545 = -(ROcp19_234*S45-ROcp19_544*C45);
    ROcp19_645 = -(ROcp19_334*S45-ROcp19_644*C45);
    ROcp19_146 = ROcp19_145*C46-ROcp19_744*S46;
    ROcp19_246 = ROcp19_245*C46-ROcp19_844*S46;
    ROcp19_346 = ROcp19_345*C46-ROcp19_944*S46;
    ROcp19_746 = ROcp19_145*S46+ROcp19_744*C46;
    ROcp19_846 = ROcp19_245*S46+ROcp19_844*C46;
    ROcp19_946 = ROcp19_345*S46+ROcp19_944*C46;
    ROcp19_447 = ROcp19_445*C47+ROcp19_746*S47;
    ROcp19_547 = ROcp19_545*C47+ROcp19_846*S47;
    ROcp19_647 = ROcp19_645*C47+ROcp19_946*S47;
    ROcp19_747 = -(ROcp19_445*S47-ROcp19_746*C47);
    ROcp19_847 = -(ROcp19_545*S47-ROcp19_846*C47);
    ROcp19_947 = -(ROcp19_645*S47-ROcp19_946*C47);
    ROcp19_148 = ROcp19_146*C48+ROcp19_447*S48;
    ROcp19_248 = ROcp19_246*C48+ROcp19_547*S48;
    ROcp19_348 = ROcp19_346*C48+ROcp19_647*S48;
    ROcp19_448 = -(ROcp19_146*S48-ROcp19_447*C48);
    ROcp19_548 = -(ROcp19_246*S48-ROcp19_547*C48);
    ROcp19_648 = -(ROcp19_346*S48-ROcp19_647*C48);
    ROcp19_149 = ROcp19_148*C49-ROcp19_747*S49;
    ROcp19_249 = ROcp19_248*C49-ROcp19_847*S49;
    ROcp19_349 = ROcp19_348*C49-ROcp19_947*S49;
    ROcp19_749 = ROcp19_148*S49+ROcp19_747*C49;
    ROcp19_849 = ROcp19_248*S49+ROcp19_847*C49;
    ROcp19_949 = ROcp19_348*S49+ROcp19_947*C49;
    ROcp19_150 = ROcp19_149*C50+ROcp19_448*S50;
    ROcp19_250 = ROcp19_249*C50+ROcp19_548*S50;
    ROcp19_350 = ROcp19_349*C50+ROcp19_648*S50;
    ROcp19_450 = -(ROcp19_149*S50-ROcp19_448*C50);
    ROcp19_550 = -(ROcp19_249*S50-ROcp19_548*C50);
    ROcp19_650 = -(ROcp19_349*S50-ROcp19_648*C50);
    ROcp19_151 = ROcp19_150*C51-ROcp19_749*S51;
    ROcp19_251 = ROcp19_250*C51-ROcp19_849*S51;
    ROcp19_351 = ROcp19_350*C51-ROcp19_949*S51;
    ROcp19_751 = ROcp19_150*S51+ROcp19_749*C51;
    ROcp19_851 = ROcp19_250*S51+ROcp19_849*C51;
    ROcp19_951 = ROcp19_350*S51+ROcp19_949*C51;
    RLcp19_144 = ROcp19_134*s->dpt[1][24]+ROcp19_434*s->dpt[2][24]+ROcp19_733*s->dpt[3][24];
    RLcp19_244 = ROcp19_234*s->dpt[1][24]+ROcp19_534*s->dpt[2][24]+ROcp19_833*s->dpt[3][24];
    RLcp19_344 = ROcp19_334*s->dpt[1][24]+ROcp19_634*s->dpt[2][24]+ROcp19_933*s->dpt[3][24];
    ORcp19_144 = OMcp19_234*RLcp19_344-OMcp19_334*RLcp19_244;
    ORcp19_244 = -(OMcp19_134*RLcp19_344-OMcp19_334*RLcp19_144);
    ORcp19_344 = OMcp19_134*RLcp19_244-OMcp19_234*RLcp19_144;
    OMcp19_146 = OMcp19_134+ROcp19_445*qd[46];
    OMcp19_246 = OMcp19_234+ROcp19_545*qd[46];
    OMcp19_346 = OMcp19_334+ROcp19_645*qd[46];
    OPcp19_146 = OPcp19_134+ROcp19_445*qdd[46]+qd[46]*(OMcp19_234*ROcp19_645-OMcp19_334*ROcp19_545);
    OPcp19_246 = OPcp19_234+ROcp19_545*qdd[46]-qd[46]*(OMcp19_134*ROcp19_645-OMcp19_334*ROcp19_445);
    OPcp19_346 = OPcp19_334+ROcp19_645*qdd[46]+qd[46]*(OMcp19_134*ROcp19_545-OMcp19_234*ROcp19_445);
    RLcp19_147 = ROcp19_445*s->dpt[2][41]+ROcp19_746*s->dpt[3][41];
    RLcp19_247 = ROcp19_545*s->dpt[2][41]+ROcp19_846*s->dpt[3][41];
    RLcp19_347 = ROcp19_645*s->dpt[2][41]+ROcp19_946*s->dpt[3][41];
    OMcp19_147 = OMcp19_146+ROcp19_146*qd[47];
    OMcp19_247 = OMcp19_246+ROcp19_246*qd[47];
    OMcp19_347 = OMcp19_346+ROcp19_346*qd[47];
    ORcp19_147 = OMcp19_246*RLcp19_347-OMcp19_346*RLcp19_247;
    ORcp19_247 = -(OMcp19_146*RLcp19_347-OMcp19_346*RLcp19_147);
    ORcp19_347 = OMcp19_146*RLcp19_247-OMcp19_246*RLcp19_147;
    OPcp19_147 = OPcp19_146+ROcp19_146*qdd[47]+qd[47]*(OMcp19_246*ROcp19_346-OMcp19_346*ROcp19_246);
    OPcp19_247 = OPcp19_246+ROcp19_246*qdd[47]-qd[47]*(OMcp19_146*ROcp19_346-OMcp19_346*ROcp19_146);
    OPcp19_347 = OPcp19_346+ROcp19_346*qdd[47]+qd[47]*(OMcp19_146*ROcp19_246-OMcp19_246*ROcp19_146);
    RLcp19_148 = ROcp19_747*s->dpt[3][43];
    RLcp19_248 = ROcp19_847*s->dpt[3][43];
    RLcp19_348 = ROcp19_947*s->dpt[3][43];
    OMcp19_148 = OMcp19_147+ROcp19_747*qd[48];
    OMcp19_248 = OMcp19_247+ROcp19_847*qd[48];
    OMcp19_348 = OMcp19_347+ROcp19_947*qd[48];
    ORcp19_148 = OMcp19_247*RLcp19_348-OMcp19_347*RLcp19_248;
    ORcp19_248 = -(OMcp19_147*RLcp19_348-OMcp19_347*RLcp19_148);
    ORcp19_348 = OMcp19_147*RLcp19_248-OMcp19_247*RLcp19_148;
    OPcp19_148 = OPcp19_147+ROcp19_747*qdd[48]+qd[48]*(OMcp19_247*ROcp19_947-OMcp19_347*ROcp19_847);
    OPcp19_248 = OPcp19_247+ROcp19_847*qdd[48]-qd[48]*(OMcp19_147*ROcp19_947-OMcp19_347*ROcp19_747);
    OPcp19_348 = OPcp19_347+ROcp19_947*qdd[48]+qd[48]*(OMcp19_147*ROcp19_847-OMcp19_247*ROcp19_747);
    RLcp19_149 = ROcp19_148*s->dpt[1][46]+ROcp19_747*s->dpt[3][46];
    RLcp19_249 = ROcp19_248*s->dpt[1][46]+ROcp19_847*s->dpt[3][46];
    RLcp19_349 = ROcp19_348*s->dpt[1][46]+ROcp19_947*s->dpt[3][46];
    OMcp19_149 = OMcp19_148+ROcp19_448*qd[49];
    OMcp19_249 = OMcp19_248+ROcp19_548*qd[49];
    OMcp19_349 = OMcp19_348+ROcp19_648*qd[49];
    ORcp19_149 = OMcp19_248*RLcp19_349-OMcp19_348*RLcp19_249;
    ORcp19_249 = -(OMcp19_148*RLcp19_349-OMcp19_348*RLcp19_149);
    ORcp19_349 = OMcp19_148*RLcp19_249-OMcp19_248*RLcp19_149;
    OPcp19_149 = OPcp19_148+ROcp19_448*qdd[49]+qd[49]*(OMcp19_248*ROcp19_648-OMcp19_348*ROcp19_548);
    OPcp19_249 = OPcp19_248+ROcp19_548*qdd[49]-qd[49]*(OMcp19_148*ROcp19_648-OMcp19_348*ROcp19_448);
    OPcp19_349 = OPcp19_348+ROcp19_648*qdd[49]+qd[49]*(OMcp19_148*ROcp19_548-OMcp19_248*ROcp19_448);
    RLcp19_150 = ROcp19_149*s->dpt[1][48]+ROcp19_749*s->dpt[3][48];
    RLcp19_250 = ROcp19_249*s->dpt[1][48]+ROcp19_849*s->dpt[3][48];
    RLcp19_350 = ROcp19_349*s->dpt[1][48]+ROcp19_949*s->dpt[3][48];
    OMcp19_150 = OMcp19_149+ROcp19_749*qd[50];
    OMcp19_250 = OMcp19_249+ROcp19_849*qd[50];
    OMcp19_350 = OMcp19_349+ROcp19_949*qd[50];
    ORcp19_150 = OMcp19_249*RLcp19_350-OMcp19_349*RLcp19_250;
    ORcp19_250 = -(OMcp19_149*RLcp19_350-OMcp19_349*RLcp19_150);
    ORcp19_350 = OMcp19_149*RLcp19_250-OMcp19_249*RLcp19_150;
    OMcp19_151 = OMcp19_150+ROcp19_450*qd[51];
    OMcp19_251 = OMcp19_250+ROcp19_550*qd[51];
    OMcp19_351 = OMcp19_350+ROcp19_650*qd[51];
    OPcp19_151 = OPcp19_149+ROcp19_450*qdd[51]+ROcp19_749*qdd[50]+qd[50]*(OMcp19_249*ROcp19_949-OMcp19_349*ROcp19_849)+
 qd[51]*(OMcp19_250*ROcp19_650-OMcp19_350*ROcp19_550);
    OPcp19_251 = OPcp19_249+ROcp19_550*qdd[51]+ROcp19_849*qdd[50]-qd[50]*(OMcp19_149*ROcp19_949-OMcp19_349*ROcp19_749)-
 qd[51]*(OMcp19_150*ROcp19_650-OMcp19_350*ROcp19_450);
    OPcp19_351 = OPcp19_349+ROcp19_650*qdd[51]+ROcp19_949*qdd[50]+qd[50]*(OMcp19_149*ROcp19_849-OMcp19_249*ROcp19_749)+
 qd[51]*(OMcp19_150*ROcp19_550-OMcp19_250*ROcp19_450);
    RLcp19_175 = ROcp19_151*s->dpt[1][53]+ROcp19_450*s->dpt[2][53]+ROcp19_751*s->dpt[3][53];
    RLcp19_275 = ROcp19_251*s->dpt[1][53]+ROcp19_550*s->dpt[2][53]+ROcp19_851*s->dpt[3][53];
    RLcp19_375 = ROcp19_351*s->dpt[1][53]+ROcp19_650*s->dpt[2][53]+ROcp19_951*s->dpt[3][53];
    POcp19_175 = RLcp19_131+RLcp19_134+RLcp19_144+RLcp19_147+RLcp19_148+RLcp19_149+RLcp19_150+RLcp19_175+q[1];
    POcp19_275 = RLcp19_231+RLcp19_234+RLcp19_244+RLcp19_247+RLcp19_248+RLcp19_249+RLcp19_250+RLcp19_275+q[2];
    POcp19_375 = RLcp19_331+RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350+RLcp19_375+q[3];
    JTcp19_275_4 = -(RLcp19_331+RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350+RLcp19_375);
    JTcp19_375_4 = RLcp19_231+RLcp19_234+RLcp19_244+RLcp19_247+RLcp19_248+RLcp19_249+RLcp19_250+RLcp19_275;
    JTcp19_175_5 = C4*(RLcp19_331+RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350+RLcp19_375)-S4*(
 RLcp19_231+RLcp19_234)-S4*(RLcp19_244+RLcp19_247)-S4*(RLcp19_248+RLcp19_249)-S4*(RLcp19_250+RLcp19_275);
    JTcp19_275_5 = S4*(RLcp19_131+RLcp19_134+RLcp19_144+RLcp19_147+RLcp19_148+RLcp19_149+RLcp19_150+RLcp19_175);
    JTcp19_375_5 = -C4*(RLcp19_131+RLcp19_134+RLcp19_144+RLcp19_147+RLcp19_148+RLcp19_149+RLcp19_150+RLcp19_175);
    JTcp19_175_6 = ROcp19_85*(RLcp19_331+RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350+RLcp19_375)-
 ROcp19_95*(RLcp19_231+RLcp19_234)-ROcp19_95*(RLcp19_244+RLcp19_247)-ROcp19_95*(RLcp19_248+RLcp19_249)-ROcp19_95*(RLcp19_250+
 RLcp19_275);
    JTcp19_275_6 = RLcp19_175*ROcp19_95-RLcp19_350*S5-RLcp19_375*S5+ROcp19_95*(RLcp19_131+RLcp19_134+RLcp19_144+RLcp19_147
 +RLcp19_148+RLcp19_149+RLcp19_150)-S5*(RLcp19_331+RLcp19_334)-S5*(RLcp19_344+RLcp19_347)-S5*(RLcp19_348+RLcp19_349);
    JTcp19_375_6 = RLcp19_250*S5-ROcp19_85*(RLcp19_131+RLcp19_134+RLcp19_144+RLcp19_147+RLcp19_148+RLcp19_149+RLcp19_150)+
 S5*(RLcp19_231+RLcp19_234)+S5*(RLcp19_244+RLcp19_247)+S5*(RLcp19_248+RLcp19_249)-RLcp19_175*ROcp19_85+RLcp19_275*S5;
    JTcp19_175_7 = ROcp19_56*(RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350)-ROcp19_66*(RLcp19_234+
 RLcp19_244)-ROcp19_66*(RLcp19_247+RLcp19_248)-ROcp19_66*(RLcp19_249+RLcp19_250)-RLcp19_275*ROcp19_66+RLcp19_375*ROcp19_56;
    JTcp19_275_7 = RLcp19_175*ROcp19_66-RLcp19_375*ROcp19_46-ROcp19_46*(RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+
 RLcp19_349+RLcp19_350)+ROcp19_66*(RLcp19_134+RLcp19_144)+ROcp19_66*(RLcp19_147+RLcp19_148)+ROcp19_66*(RLcp19_149+RLcp19_150);
    JTcp19_375_7 = ROcp19_46*(RLcp19_234+RLcp19_244+RLcp19_247+RLcp19_248+RLcp19_249+RLcp19_250)-ROcp19_56*(RLcp19_134+
 RLcp19_144)-ROcp19_56*(RLcp19_147+RLcp19_148)-ROcp19_56*(RLcp19_149+RLcp19_150)-RLcp19_175*ROcp19_56+RLcp19_275*ROcp19_46;
    JTcp19_175_8 = ROcp19_231*(RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350)-ROcp19_331*(RLcp19_234+
 RLcp19_244)-ROcp19_331*(RLcp19_247+RLcp19_248)-ROcp19_331*(RLcp19_249+RLcp19_250)-RLcp19_275*ROcp19_331+RLcp19_375*
 ROcp19_231;
    JTcp19_275_8 = RLcp19_175*ROcp19_331-RLcp19_375*ROcp19_131-ROcp19_131*(RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+
 RLcp19_349+RLcp19_350)+ROcp19_331*(RLcp19_134+RLcp19_144)+ROcp19_331*(RLcp19_147+RLcp19_148)+ROcp19_331*(RLcp19_149+
 RLcp19_150);
    JTcp19_375_8 = ROcp19_131*(RLcp19_234+RLcp19_244+RLcp19_247+RLcp19_248+RLcp19_249+RLcp19_250)-ROcp19_231*(RLcp19_134+
 RLcp19_144)-ROcp19_231*(RLcp19_147+RLcp19_148)-ROcp19_231*(RLcp19_149+RLcp19_150)-RLcp19_175*ROcp19_231+RLcp19_275*
 ROcp19_131;
    JTcp19_175_9 = ROcp19_532*(RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350)-ROcp19_632*(RLcp19_234+
 RLcp19_244)-ROcp19_632*(RLcp19_247+RLcp19_248)-ROcp19_632*(RLcp19_249+RLcp19_250)-RLcp19_275*ROcp19_632+RLcp19_375*
 ROcp19_532;
    JTcp19_275_9 = RLcp19_175*ROcp19_632-RLcp19_375*ROcp19_432-ROcp19_432*(RLcp19_334+RLcp19_344+RLcp19_347+RLcp19_348+
 RLcp19_349+RLcp19_350)+ROcp19_632*(RLcp19_134+RLcp19_144)+ROcp19_632*(RLcp19_147+RLcp19_148)+ROcp19_632*(RLcp19_149+
 RLcp19_150);
    JTcp19_375_9 = ROcp19_432*(RLcp19_234+RLcp19_244+RLcp19_247+RLcp19_248+RLcp19_249+RLcp19_250)-ROcp19_532*(RLcp19_134+
 RLcp19_144)-ROcp19_532*(RLcp19_147+RLcp19_148)-ROcp19_532*(RLcp19_149+RLcp19_150)-RLcp19_175*ROcp19_532+RLcp19_275*
 ROcp19_432;
    JTcp19_175_10 = ROcp19_833*(RLcp19_344+RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350+RLcp19_375)-ROcp19_933*(RLcp19_244+
 RLcp19_247)-ROcp19_933*(RLcp19_248+RLcp19_249)-ROcp19_933*(RLcp19_250+RLcp19_275);
    JTcp19_275_10 = -(ROcp19_733*(RLcp19_344+RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350+RLcp19_375)-ROcp19_933*(
 RLcp19_144+RLcp19_147)-ROcp19_933*(RLcp19_148+RLcp19_149)-ROcp19_933*(RLcp19_150+RLcp19_175));
    JTcp19_375_10 = ROcp19_733*(RLcp19_244+RLcp19_247+RLcp19_248+RLcp19_249+RLcp19_250+RLcp19_275)-ROcp19_833*(RLcp19_144+
 RLcp19_147)-ROcp19_833*(RLcp19_148+RLcp19_149)-ROcp19_833*(RLcp19_150+RLcp19_175);
    JTcp19_175_11 = ROcp19_234*(RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350)-ROcp19_334*(RLcp19_247+RLcp19_248)-ROcp19_334
 *(RLcp19_249+RLcp19_250)-RLcp19_275*ROcp19_334+RLcp19_375*ROcp19_234;
    JTcp19_275_11 = RLcp19_175*ROcp19_334-RLcp19_375*ROcp19_134-ROcp19_134*(RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350)+
 ROcp19_334*(RLcp19_147+RLcp19_148)+ROcp19_334*(RLcp19_149+RLcp19_150);
    JTcp19_375_11 = ROcp19_134*(RLcp19_247+RLcp19_248+RLcp19_249+RLcp19_250)-ROcp19_234*(RLcp19_147+RLcp19_148)-ROcp19_234
 *(RLcp19_149+RLcp19_150)-RLcp19_175*ROcp19_234+RLcp19_275*ROcp19_134;
    JTcp19_175_12 = ROcp19_844*(RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350)-ROcp19_944*(RLcp19_247+RLcp19_248)-ROcp19_944
 *(RLcp19_249+RLcp19_250)-RLcp19_275*ROcp19_944+RLcp19_375*ROcp19_844;
    JTcp19_275_12 = RLcp19_175*ROcp19_944-RLcp19_375*ROcp19_744-ROcp19_744*(RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350)+
 ROcp19_944*(RLcp19_147+RLcp19_148)+ROcp19_944*(RLcp19_149+RLcp19_150);
    JTcp19_375_12 = ROcp19_744*(RLcp19_247+RLcp19_248+RLcp19_249+RLcp19_250)-ROcp19_844*(RLcp19_147+RLcp19_148)-ROcp19_844
 *(RLcp19_149+RLcp19_150)-RLcp19_175*ROcp19_844+RLcp19_275*ROcp19_744;
    JTcp19_175_13 = ROcp19_545*(RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350)-ROcp19_645*(RLcp19_247+RLcp19_248)-ROcp19_645
 *(RLcp19_249+RLcp19_250)-RLcp19_275*ROcp19_645+RLcp19_375*ROcp19_545;
    JTcp19_275_13 = RLcp19_175*ROcp19_645-RLcp19_375*ROcp19_445-ROcp19_445*(RLcp19_347+RLcp19_348+RLcp19_349+RLcp19_350)+
 ROcp19_645*(RLcp19_147+RLcp19_148)+ROcp19_645*(RLcp19_149+RLcp19_150);
    JTcp19_375_13 = ROcp19_445*(RLcp19_247+RLcp19_248+RLcp19_249+RLcp19_250)-ROcp19_545*(RLcp19_147+RLcp19_148)-ROcp19_545
 *(RLcp19_149+RLcp19_150)-RLcp19_175*ROcp19_545+RLcp19_275*ROcp19_445;
    JTcp19_175_14 = ROcp19_246*(RLcp19_348+RLcp19_349+RLcp19_350+RLcp19_375)-ROcp19_346*(RLcp19_248+RLcp19_249)-ROcp19_346
 *(RLcp19_250+RLcp19_275);
    JTcp19_275_14 = -(ROcp19_146*(RLcp19_348+RLcp19_349+RLcp19_350+RLcp19_375)-ROcp19_346*(RLcp19_148+RLcp19_149)-
 ROcp19_346*(RLcp19_150+RLcp19_175));
    JTcp19_375_14 = ROcp19_146*(RLcp19_248+RLcp19_249+RLcp19_250+RLcp19_275)-ROcp19_246*(RLcp19_148+RLcp19_149)-ROcp19_246
 *(RLcp19_150+RLcp19_175);
    JTcp19_175_15 = ROcp19_847*(RLcp19_349+RLcp19_350)-ROcp19_947*(RLcp19_249+RLcp19_250)-RLcp19_275*ROcp19_947+RLcp19_375
 *ROcp19_847;
    JTcp19_275_15 = RLcp19_175*ROcp19_947-RLcp19_375*ROcp19_747-ROcp19_747*(RLcp19_349+RLcp19_350)+ROcp19_947*(RLcp19_149+
 RLcp19_150);
    JTcp19_375_15 = ROcp19_747*(RLcp19_249+RLcp19_250)-ROcp19_847*(RLcp19_149+RLcp19_150)-RLcp19_175*ROcp19_847+RLcp19_275
 *ROcp19_747;
    JTcp19_175_16 = ROcp19_548*(RLcp19_350+RLcp19_375)-ROcp19_648*(RLcp19_250+RLcp19_275);
    JTcp19_275_16 = -(ROcp19_448*(RLcp19_350+RLcp19_375)-ROcp19_648*(RLcp19_150+RLcp19_175));
    JTcp19_375_16 = ROcp19_448*(RLcp19_250+RLcp19_275)-ROcp19_548*(RLcp19_150+RLcp19_175);
    JTcp19_175_17 = -(RLcp19_275*ROcp19_949-RLcp19_375*ROcp19_849);
    JTcp19_275_17 = RLcp19_175*ROcp19_949-RLcp19_375*ROcp19_749;
    JTcp19_375_17 = -(RLcp19_175*ROcp19_849-RLcp19_275*ROcp19_749);
    JTcp19_175_18 = -(RLcp19_275*ROcp19_650-RLcp19_375*ROcp19_550);
    JTcp19_275_18 = RLcp19_175*ROcp19_650-RLcp19_375*ROcp19_450;
    JTcp19_375_18 = -(RLcp19_175*ROcp19_550-RLcp19_275*ROcp19_450);
    ORcp19_175 = OMcp19_251*RLcp19_375-OMcp19_351*RLcp19_275;
    ORcp19_275 = -(OMcp19_151*RLcp19_375-OMcp19_351*RLcp19_175);
    ORcp19_375 = OMcp19_151*RLcp19_275-OMcp19_251*RLcp19_175;
    VIcp19_175 = ORcp19_131+ORcp19_134+ORcp19_144+ORcp19_147+ORcp19_148+ORcp19_149+ORcp19_150+ORcp19_175+qd[1];
    VIcp19_275 = ORcp19_231+ORcp19_234+ORcp19_244+ORcp19_247+ORcp19_248+ORcp19_249+ORcp19_250+ORcp19_275+qd[2];
    VIcp19_375 = ORcp19_331+ORcp19_334+ORcp19_344+ORcp19_347+ORcp19_348+ORcp19_349+ORcp19_350+ORcp19_375+qd[3];
    ACcp19_175 = qdd[1]+OMcp19_233*ORcp19_334+OMcp19_234*ORcp19_344+OMcp19_246*ORcp19_347+OMcp19_247*ORcp19_348+OMcp19_248
 *ORcp19_349+OMcp19_249*ORcp19_350+OMcp19_251*ORcp19_375+OMcp19_26*ORcp19_331-OMcp19_333*ORcp19_234-OMcp19_334*ORcp19_244-
 OMcp19_346*ORcp19_247-OMcp19_347*ORcp19_248-OMcp19_348*ORcp19_249-OMcp19_349*ORcp19_250-OMcp19_351*ORcp19_275-OMcp19_36*
 ORcp19_231+OPcp19_233*RLcp19_334+OPcp19_234*RLcp19_344+OPcp19_246*RLcp19_347+OPcp19_247*RLcp19_348+OPcp19_248*RLcp19_349+
 OPcp19_249*RLcp19_350+OPcp19_251*RLcp19_375+OPcp19_26*RLcp19_331-OPcp19_333*RLcp19_234-OPcp19_334*RLcp19_244-OPcp19_346*
 RLcp19_247-OPcp19_347*RLcp19_248-OPcp19_348*RLcp19_249-OPcp19_349*RLcp19_250-OPcp19_351*RLcp19_275-OPcp19_36*RLcp19_231;
    ACcp19_275 = qdd[2]-OMcp19_133*ORcp19_334-OMcp19_134*ORcp19_344-OMcp19_146*ORcp19_347-OMcp19_147*ORcp19_348-OMcp19_148
 *ORcp19_349-OMcp19_149*ORcp19_350-OMcp19_151*ORcp19_375-OMcp19_16*ORcp19_331+OMcp19_333*ORcp19_134+OMcp19_334*ORcp19_144+
 OMcp19_346*ORcp19_147+OMcp19_347*ORcp19_148+OMcp19_348*ORcp19_149+OMcp19_349*ORcp19_150+OMcp19_351*ORcp19_175+OMcp19_36*
 ORcp19_131-OPcp19_133*RLcp19_334-OPcp19_134*RLcp19_344-OPcp19_146*RLcp19_347-OPcp19_147*RLcp19_348-OPcp19_148*RLcp19_349-
 OPcp19_149*RLcp19_350-OPcp19_151*RLcp19_375-OPcp19_16*RLcp19_331+OPcp19_333*RLcp19_134+OPcp19_334*RLcp19_144+OPcp19_346*
 RLcp19_147+OPcp19_347*RLcp19_148+OPcp19_348*RLcp19_149+OPcp19_349*RLcp19_150+OPcp19_351*RLcp19_175+OPcp19_36*RLcp19_131;
    ACcp19_375 = qdd[3]+OMcp19_133*ORcp19_234+OMcp19_134*ORcp19_244+OMcp19_146*ORcp19_247+OMcp19_147*ORcp19_248+OMcp19_148
 *ORcp19_249+OMcp19_149*ORcp19_250+OMcp19_151*ORcp19_275+OMcp19_16*ORcp19_231-OMcp19_233*ORcp19_134-OMcp19_234*ORcp19_144-
 OMcp19_246*ORcp19_147-OMcp19_247*ORcp19_148-OMcp19_248*ORcp19_149-OMcp19_249*ORcp19_150-OMcp19_251*ORcp19_175-OMcp19_26*
 ORcp19_131+OPcp19_133*RLcp19_234+OPcp19_134*RLcp19_244+OPcp19_146*RLcp19_247+OPcp19_147*RLcp19_248+OPcp19_148*RLcp19_249+
 OPcp19_149*RLcp19_250+OPcp19_151*RLcp19_275+OPcp19_16*RLcp19_231-OPcp19_233*RLcp19_134-OPcp19_234*RLcp19_144-OPcp19_246*
 RLcp19_147-OPcp19_247*RLcp19_148-OPcp19_248*RLcp19_149-OPcp19_249*RLcp19_150-OPcp19_251*RLcp19_175-OPcp19_26*RLcp19_131;

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp19_175;
    sens->P[2] = POcp19_275;
    sens->P[3] = POcp19_375;
    sens->R[1][1] = ROcp19_151;
    sens->R[1][2] = ROcp19_251;
    sens->R[1][3] = ROcp19_351;
    sens->R[2][1] = ROcp19_450;
    sens->R[2][2] = ROcp19_550;
    sens->R[2][3] = ROcp19_650;
    sens->R[3][1] = ROcp19_751;
    sens->R[3][2] = ROcp19_851;
    sens->R[3][3] = ROcp19_951;
    sens->V[1] = VIcp19_175;
    sens->V[2] = VIcp19_275;
    sens->V[3] = VIcp19_375;
    sens->OM[1] = OMcp19_151;
    sens->OM[2] = OMcp19_251;
    sens->OM[3] = OMcp19_351;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp19_175_5;
    sens->J[1][6] = JTcp19_175_6;
    sens->J[1][31] = JTcp19_175_7;
    sens->J[1][32] = JTcp19_175_8;
    sens->J[1][33] = JTcp19_175_9;
    sens->J[1][34] = JTcp19_175_10;
    sens->J[1][44] = JTcp19_175_11;
    sens->J[1][45] = JTcp19_175_12;
    sens->J[1][46] = JTcp19_175_13;
    sens->J[1][47] = JTcp19_175_14;
    sens->J[1][48] = JTcp19_175_15;
    sens->J[1][49] = JTcp19_175_16;
    sens->J[1][50] = JTcp19_175_17;
    sens->J[1][51] = JTcp19_175_18;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp19_275_4;
    sens->J[2][5] = JTcp19_275_5;
    sens->J[2][6] = JTcp19_275_6;
    sens->J[2][31] = JTcp19_275_7;
    sens->J[2][32] = JTcp19_275_8;
    sens->J[2][33] = JTcp19_275_9;
    sens->J[2][34] = JTcp19_275_10;
    sens->J[2][44] = JTcp19_275_11;
    sens->J[2][45] = JTcp19_275_12;
    sens->J[2][46] = JTcp19_275_13;
    sens->J[2][47] = JTcp19_275_14;
    sens->J[2][48] = JTcp19_275_15;
    sens->J[2][49] = JTcp19_275_16;
    sens->J[2][50] = JTcp19_275_17;
    sens->J[2][51] = JTcp19_275_18;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp19_375_4;
    sens->J[3][5] = JTcp19_375_5;
    sens->J[3][6] = JTcp19_375_6;
    sens->J[3][31] = JTcp19_375_7;
    sens->J[3][32] = JTcp19_375_8;
    sens->J[3][33] = JTcp19_375_9;
    sens->J[3][34] = JTcp19_375_10;
    sens->J[3][44] = JTcp19_375_11;
    sens->J[3][45] = JTcp19_375_12;
    sens->J[3][46] = JTcp19_375_13;
    sens->J[3][47] = JTcp19_375_14;
    sens->J[3][48] = JTcp19_375_15;
    sens->J[3][49] = JTcp19_375_16;
    sens->J[3][50] = JTcp19_375_17;
    sens->J[3][51] = JTcp19_375_18;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp19_46;
    sens->J[4][32] = ROcp19_131;
    sens->J[4][33] = ROcp19_432;
    sens->J[4][34] = ROcp19_733;
    sens->J[4][44] = ROcp19_134;
    sens->J[4][45] = ROcp19_744;
    sens->J[4][46] = ROcp19_445;
    sens->J[4][47] = ROcp19_146;
    sens->J[4][48] = ROcp19_747;
    sens->J[4][49] = ROcp19_448;
    sens->J[4][50] = ROcp19_749;
    sens->J[4][51] = ROcp19_450;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp19_85;
    sens->J[5][31] = ROcp19_56;
    sens->J[5][32] = ROcp19_231;
    sens->J[5][33] = ROcp19_532;
    sens->J[5][34] = ROcp19_833;
    sens->J[5][44] = ROcp19_234;
    sens->J[5][45] = ROcp19_844;
    sens->J[5][46] = ROcp19_545;
    sens->J[5][47] = ROcp19_246;
    sens->J[5][48] = ROcp19_847;
    sens->J[5][49] = ROcp19_548;
    sens->J[5][50] = ROcp19_849;
    sens->J[5][51] = ROcp19_550;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp19_95;
    sens->J[6][31] = ROcp19_66;
    sens->J[6][32] = ROcp19_331;
    sens->J[6][33] = ROcp19_632;
    sens->J[6][34] = ROcp19_933;
    sens->J[6][44] = ROcp19_334;
    sens->J[6][45] = ROcp19_944;
    sens->J[6][46] = ROcp19_645;
    sens->J[6][47] = ROcp19_346;
    sens->J[6][48] = ROcp19_947;
    sens->J[6][49] = ROcp19_648;
    sens->J[6][50] = ROcp19_949;
    sens->J[6][51] = ROcp19_650;
    sens->A[1] = ACcp19_175;
    sens->A[2] = ACcp19_275;
    sens->A[3] = ACcp19_375;
    sens->OMP[1] = OPcp19_151;
    sens->OMP[2] = OPcp19_251;
    sens->OMP[3] = OPcp19_351;
 
// 
break;
case 21:
 


// = = Block_1_0_0_21_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp20_25 = qd[5]*C4;
    OMcp20_35 = qd[5]*S4;
    OMcp20_16 = qd[4]+qd[6]*S5;
    OMcp20_26 = OMcp20_25+ROcp20_85*qd[6];
    OMcp20_36 = OMcp20_35+ROcp20_95*qd[6];
    OPcp20_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp20_26 = ROcp20_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp20_35*S5-ROcp20_95*qd[4]);
    OPcp20_36 = ROcp20_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp20_25*S5-ROcp20_85*qd[4]);

// = = Block_1_0_0_21_0_4 = = 
 
// Sensor Kinematics 


    ROcp20_131 = ROcp20_16*C31-S31*S5;
    ROcp20_231 = ROcp20_26*C31-ROcp20_85*S31;
    ROcp20_331 = ROcp20_36*C31-ROcp20_95*S31;
    ROcp20_731 = ROcp20_16*S31+C31*S5;
    ROcp20_831 = ROcp20_26*S31+ROcp20_85*C31;
    ROcp20_931 = ROcp20_36*S31+ROcp20_95*C31;
    ROcp20_432 = ROcp20_46*C32+ROcp20_731*S32;
    ROcp20_532 = ROcp20_56*C32+ROcp20_831*S32;
    ROcp20_632 = ROcp20_66*C32+ROcp20_931*S32;
    ROcp20_732 = -(ROcp20_46*S32-ROcp20_731*C32);
    ROcp20_832 = -(ROcp20_56*S32-ROcp20_831*C32);
    ROcp20_932 = -(ROcp20_66*S32-ROcp20_931*C32);
    ROcp20_133 = ROcp20_131*C33-ROcp20_732*S33;
    ROcp20_233 = ROcp20_231*C33-ROcp20_832*S33;
    ROcp20_333 = ROcp20_331*C33-ROcp20_932*S33;
    ROcp20_733 = ROcp20_131*S33+ROcp20_732*C33;
    ROcp20_833 = ROcp20_231*S33+ROcp20_832*C33;
    ROcp20_933 = ROcp20_331*S33+ROcp20_932*C33;
    ROcp20_134 = ROcp20_133*C34+ROcp20_432*S34;
    ROcp20_234 = ROcp20_233*C34+ROcp20_532*S34;
    ROcp20_334 = ROcp20_333*C34+ROcp20_632*S34;
    ROcp20_434 = -(ROcp20_133*S34-ROcp20_432*C34);
    ROcp20_534 = -(ROcp20_233*S34-ROcp20_532*C34);
    ROcp20_634 = -(ROcp20_333*S34-ROcp20_632*C34);
    RLcp20_131 = ROcp20_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp20_231 = ROcp20_26*s->dpt[1][3]+ROcp20_85*s->dpt[3][3];
    RLcp20_331 = ROcp20_36*s->dpt[1][3]+ROcp20_95*s->dpt[3][3];
    OMcp20_131 = OMcp20_16+ROcp20_46*qd[31];
    OMcp20_231 = OMcp20_26+ROcp20_56*qd[31];
    OMcp20_331 = OMcp20_36+ROcp20_66*qd[31];
    ORcp20_131 = OMcp20_26*RLcp20_331-OMcp20_36*RLcp20_231;
    ORcp20_231 = -(OMcp20_16*RLcp20_331-OMcp20_36*RLcp20_131);
    ORcp20_331 = OMcp20_16*RLcp20_231-OMcp20_26*RLcp20_131;
    OMcp20_132 = OMcp20_131+ROcp20_131*qd[32];
    OMcp20_232 = OMcp20_231+ROcp20_231*qd[32];
    OMcp20_332 = OMcp20_331+ROcp20_331*qd[32];
    OMcp20_133 = OMcp20_132+ROcp20_432*qd[33];
    OMcp20_233 = OMcp20_232+ROcp20_532*qd[33];
    OMcp20_333 = OMcp20_332+ROcp20_632*qd[33];
    OPcp20_133 = OPcp20_16+ROcp20_131*qdd[32]+ROcp20_432*qdd[33]+ROcp20_46*qdd[31]+qd[31]*(OMcp20_26*ROcp20_66-OMcp20_36*
 ROcp20_56)+qd[32]*(OMcp20_231*ROcp20_331-OMcp20_331*ROcp20_231)+qd[33]*(OMcp20_232*ROcp20_632-OMcp20_332*ROcp20_532);
    OPcp20_233 = OPcp20_26+ROcp20_231*qdd[32]+ROcp20_532*qdd[33]+ROcp20_56*qdd[31]-qd[31]*(OMcp20_16*ROcp20_66-OMcp20_36*
 ROcp20_46)-qd[32]*(OMcp20_131*ROcp20_331-OMcp20_331*ROcp20_131)-qd[33]*(OMcp20_132*ROcp20_632-OMcp20_332*ROcp20_432);
    OPcp20_333 = OPcp20_36+ROcp20_331*qdd[32]+ROcp20_632*qdd[33]+ROcp20_66*qdd[31]+qd[31]*(OMcp20_16*ROcp20_56-OMcp20_26*
 ROcp20_46)+qd[32]*(OMcp20_131*ROcp20_231-OMcp20_231*ROcp20_131)+qd[33]*(OMcp20_132*ROcp20_532-OMcp20_232*ROcp20_432);
    RLcp20_134 = ROcp20_733*s->dpt[3][21];
    RLcp20_234 = ROcp20_833*s->dpt[3][21];
    RLcp20_334 = ROcp20_933*s->dpt[3][21];
    OMcp20_134 = OMcp20_133+ROcp20_733*qd[34];
    OMcp20_234 = OMcp20_233+ROcp20_833*qd[34];
    OMcp20_334 = OMcp20_333+ROcp20_933*qd[34];
    ORcp20_134 = OMcp20_233*RLcp20_334-OMcp20_333*RLcp20_234;
    ORcp20_234 = -(OMcp20_133*RLcp20_334-OMcp20_333*RLcp20_134);
    ORcp20_334 = OMcp20_133*RLcp20_234-OMcp20_233*RLcp20_134;
    OPcp20_134 = OPcp20_133+ROcp20_733*qdd[34]+qd[34]*(OMcp20_233*ROcp20_933-OMcp20_333*ROcp20_833);
    OPcp20_234 = OPcp20_233+ROcp20_833*qdd[34]-qd[34]*(OMcp20_133*ROcp20_933-OMcp20_333*ROcp20_733);
    OPcp20_334 = OPcp20_333+ROcp20_933*qdd[34]+qd[34]*(OMcp20_133*ROcp20_833-OMcp20_233*ROcp20_733);

// = = Block_1_0_0_21_0_6 = = 
 
// Sensor Kinematics 


    ROcp20_444 = ROcp20_434*C44+ROcp20_733*S44;
    ROcp20_544 = ROcp20_534*C44+ROcp20_833*S44;
    ROcp20_644 = ROcp20_634*C44+ROcp20_933*S44;
    ROcp20_744 = -(ROcp20_434*S44-ROcp20_733*C44);
    ROcp20_844 = -(ROcp20_534*S44-ROcp20_833*C44);
    ROcp20_944 = -(ROcp20_634*S44-ROcp20_933*C44);
    ROcp20_145 = ROcp20_134*C45+ROcp20_444*S45;
    ROcp20_245 = ROcp20_234*C45+ROcp20_544*S45;
    ROcp20_345 = ROcp20_334*C45+ROcp20_644*S45;
    ROcp20_445 = -(ROcp20_134*S45-ROcp20_444*C45);
    ROcp20_545 = -(ROcp20_234*S45-ROcp20_544*C45);
    ROcp20_645 = -(ROcp20_334*S45-ROcp20_644*C45);
    ROcp20_146 = ROcp20_145*C46-ROcp20_744*S46;
    ROcp20_246 = ROcp20_245*C46-ROcp20_844*S46;
    ROcp20_346 = ROcp20_345*C46-ROcp20_944*S46;
    ROcp20_746 = ROcp20_145*S46+ROcp20_744*C46;
    ROcp20_846 = ROcp20_245*S46+ROcp20_844*C46;
    ROcp20_946 = ROcp20_345*S46+ROcp20_944*C46;
    ROcp20_447 = ROcp20_445*C47+ROcp20_746*S47;
    ROcp20_547 = ROcp20_545*C47+ROcp20_846*S47;
    ROcp20_647 = ROcp20_645*C47+ROcp20_946*S47;
    ROcp20_747 = -(ROcp20_445*S47-ROcp20_746*C47);
    ROcp20_847 = -(ROcp20_545*S47-ROcp20_846*C47);
    ROcp20_947 = -(ROcp20_645*S47-ROcp20_946*C47);
    ROcp20_148 = ROcp20_146*C48+ROcp20_447*S48;
    ROcp20_248 = ROcp20_246*C48+ROcp20_547*S48;
    ROcp20_348 = ROcp20_346*C48+ROcp20_647*S48;
    ROcp20_448 = -(ROcp20_146*S48-ROcp20_447*C48);
    ROcp20_548 = -(ROcp20_246*S48-ROcp20_547*C48);
    ROcp20_648 = -(ROcp20_346*S48-ROcp20_647*C48);
    ROcp20_149 = ROcp20_148*C49-ROcp20_747*S49;
    ROcp20_249 = ROcp20_248*C49-ROcp20_847*S49;
    ROcp20_349 = ROcp20_348*C49-ROcp20_947*S49;
    ROcp20_749 = ROcp20_148*S49+ROcp20_747*C49;
    ROcp20_849 = ROcp20_248*S49+ROcp20_847*C49;
    ROcp20_949 = ROcp20_348*S49+ROcp20_947*C49;
    ROcp20_150 = ROcp20_149*C50+ROcp20_448*S50;
    ROcp20_250 = ROcp20_249*C50+ROcp20_548*S50;
    ROcp20_350 = ROcp20_349*C50+ROcp20_648*S50;
    ROcp20_450 = -(ROcp20_149*S50-ROcp20_448*C50);
    ROcp20_550 = -(ROcp20_249*S50-ROcp20_548*C50);
    ROcp20_650 = -(ROcp20_349*S50-ROcp20_648*C50);
    ROcp20_151 = ROcp20_150*C51-ROcp20_749*S51;
    ROcp20_251 = ROcp20_250*C51-ROcp20_849*S51;
    ROcp20_351 = ROcp20_350*C51-ROcp20_949*S51;
    ROcp20_751 = ROcp20_150*S51+ROcp20_749*C51;
    ROcp20_851 = ROcp20_250*S51+ROcp20_849*C51;
    ROcp20_951 = ROcp20_350*S51+ROcp20_949*C51;
    ROcp20_452 = ROcp20_450*C52+ROcp20_751*S52;
    ROcp20_552 = ROcp20_550*C52+ROcp20_851*S52;
    ROcp20_652 = ROcp20_650*C52+ROcp20_951*S52;
    ROcp20_752 = -(ROcp20_450*S52-ROcp20_751*C52);
    ROcp20_852 = -(ROcp20_550*S52-ROcp20_851*C52);
    ROcp20_952 = -(ROcp20_650*S52-ROcp20_951*C52);
    RLcp20_144 = ROcp20_134*s->dpt[1][24]+ROcp20_434*s->dpt[2][24]+ROcp20_733*s->dpt[3][24];
    RLcp20_244 = ROcp20_234*s->dpt[1][24]+ROcp20_534*s->dpt[2][24]+ROcp20_833*s->dpt[3][24];
    RLcp20_344 = ROcp20_334*s->dpt[1][24]+ROcp20_634*s->dpt[2][24]+ROcp20_933*s->dpt[3][24];
    ORcp20_144 = OMcp20_234*RLcp20_344-OMcp20_334*RLcp20_244;
    ORcp20_244 = -(OMcp20_134*RLcp20_344-OMcp20_334*RLcp20_144);
    ORcp20_344 = OMcp20_134*RLcp20_244-OMcp20_234*RLcp20_144;
    OMcp20_146 = OMcp20_134+ROcp20_445*qd[46];
    OMcp20_246 = OMcp20_234+ROcp20_545*qd[46];
    OMcp20_346 = OMcp20_334+ROcp20_645*qd[46];
    OPcp20_146 = OPcp20_134+ROcp20_445*qdd[46]+qd[46]*(OMcp20_234*ROcp20_645-OMcp20_334*ROcp20_545);
    OPcp20_246 = OPcp20_234+ROcp20_545*qdd[46]-qd[46]*(OMcp20_134*ROcp20_645-OMcp20_334*ROcp20_445);
    OPcp20_346 = OPcp20_334+ROcp20_645*qdd[46]+qd[46]*(OMcp20_134*ROcp20_545-OMcp20_234*ROcp20_445);
    RLcp20_147 = ROcp20_445*s->dpt[2][41]+ROcp20_746*s->dpt[3][41];
    RLcp20_247 = ROcp20_545*s->dpt[2][41]+ROcp20_846*s->dpt[3][41];
    RLcp20_347 = ROcp20_645*s->dpt[2][41]+ROcp20_946*s->dpt[3][41];
    OMcp20_147 = OMcp20_146+ROcp20_146*qd[47];
    OMcp20_247 = OMcp20_246+ROcp20_246*qd[47];
    OMcp20_347 = OMcp20_346+ROcp20_346*qd[47];
    ORcp20_147 = OMcp20_246*RLcp20_347-OMcp20_346*RLcp20_247;
    ORcp20_247 = -(OMcp20_146*RLcp20_347-OMcp20_346*RLcp20_147);
    ORcp20_347 = OMcp20_146*RLcp20_247-OMcp20_246*RLcp20_147;
    OPcp20_147 = OPcp20_146+ROcp20_146*qdd[47]+qd[47]*(OMcp20_246*ROcp20_346-OMcp20_346*ROcp20_246);
    OPcp20_247 = OPcp20_246+ROcp20_246*qdd[47]-qd[47]*(OMcp20_146*ROcp20_346-OMcp20_346*ROcp20_146);
    OPcp20_347 = OPcp20_346+ROcp20_346*qdd[47]+qd[47]*(OMcp20_146*ROcp20_246-OMcp20_246*ROcp20_146);
    RLcp20_148 = ROcp20_747*s->dpt[3][43];
    RLcp20_248 = ROcp20_847*s->dpt[3][43];
    RLcp20_348 = ROcp20_947*s->dpt[3][43];
    OMcp20_148 = OMcp20_147+ROcp20_747*qd[48];
    OMcp20_248 = OMcp20_247+ROcp20_847*qd[48];
    OMcp20_348 = OMcp20_347+ROcp20_947*qd[48];
    ORcp20_148 = OMcp20_247*RLcp20_348-OMcp20_347*RLcp20_248;
    ORcp20_248 = -(OMcp20_147*RLcp20_348-OMcp20_347*RLcp20_148);
    ORcp20_348 = OMcp20_147*RLcp20_248-OMcp20_247*RLcp20_148;
    OPcp20_148 = OPcp20_147+ROcp20_747*qdd[48]+qd[48]*(OMcp20_247*ROcp20_947-OMcp20_347*ROcp20_847);
    OPcp20_248 = OPcp20_247+ROcp20_847*qdd[48]-qd[48]*(OMcp20_147*ROcp20_947-OMcp20_347*ROcp20_747);
    OPcp20_348 = OPcp20_347+ROcp20_947*qdd[48]+qd[48]*(OMcp20_147*ROcp20_847-OMcp20_247*ROcp20_747);
    RLcp20_149 = ROcp20_148*s->dpt[1][46]+ROcp20_747*s->dpt[3][46];
    RLcp20_249 = ROcp20_248*s->dpt[1][46]+ROcp20_847*s->dpt[3][46];
    RLcp20_349 = ROcp20_348*s->dpt[1][46]+ROcp20_947*s->dpt[3][46];
    OMcp20_149 = OMcp20_148+ROcp20_448*qd[49];
    OMcp20_249 = OMcp20_248+ROcp20_548*qd[49];
    OMcp20_349 = OMcp20_348+ROcp20_648*qd[49];
    ORcp20_149 = OMcp20_248*RLcp20_349-OMcp20_348*RLcp20_249;
    ORcp20_249 = -(OMcp20_148*RLcp20_349-OMcp20_348*RLcp20_149);
    ORcp20_349 = OMcp20_148*RLcp20_249-OMcp20_248*RLcp20_149;
    OPcp20_149 = OPcp20_148+ROcp20_448*qdd[49]+qd[49]*(OMcp20_248*ROcp20_648-OMcp20_348*ROcp20_548);
    OPcp20_249 = OPcp20_248+ROcp20_548*qdd[49]-qd[49]*(OMcp20_148*ROcp20_648-OMcp20_348*ROcp20_448);
    OPcp20_349 = OPcp20_348+ROcp20_648*qdd[49]+qd[49]*(OMcp20_148*ROcp20_548-OMcp20_248*ROcp20_448);
    RLcp20_150 = ROcp20_149*s->dpt[1][48]+ROcp20_749*s->dpt[3][48];
    RLcp20_250 = ROcp20_249*s->dpt[1][48]+ROcp20_849*s->dpt[3][48];
    RLcp20_350 = ROcp20_349*s->dpt[1][48]+ROcp20_949*s->dpt[3][48];
    OMcp20_150 = OMcp20_149+ROcp20_749*qd[50];
    OMcp20_250 = OMcp20_249+ROcp20_849*qd[50];
    OMcp20_350 = OMcp20_349+ROcp20_949*qd[50];
    ORcp20_150 = OMcp20_249*RLcp20_350-OMcp20_349*RLcp20_250;
    ORcp20_250 = -(OMcp20_149*RLcp20_350-OMcp20_349*RLcp20_150);
    ORcp20_350 = OMcp20_149*RLcp20_250-OMcp20_249*RLcp20_150;
    OMcp20_151 = OMcp20_150+ROcp20_450*qd[51];
    OMcp20_251 = OMcp20_250+ROcp20_550*qd[51];
    OMcp20_351 = OMcp20_350+ROcp20_650*qd[51];
    OPcp20_151 = OPcp20_149+ROcp20_450*qdd[51]+ROcp20_749*qdd[50]+qd[50]*(OMcp20_249*ROcp20_949-OMcp20_349*ROcp20_849)+
 qd[51]*(OMcp20_250*ROcp20_650-OMcp20_350*ROcp20_550);
    OPcp20_251 = OPcp20_249+ROcp20_550*qdd[51]+ROcp20_849*qdd[50]-qd[50]*(OMcp20_149*ROcp20_949-OMcp20_349*ROcp20_749)-
 qd[51]*(OMcp20_150*ROcp20_650-OMcp20_350*ROcp20_450);
    OPcp20_351 = OPcp20_349+ROcp20_650*qdd[51]+ROcp20_949*qdd[50]+qd[50]*(OMcp20_149*ROcp20_849-OMcp20_249*ROcp20_749)+
 qd[51]*(OMcp20_150*ROcp20_550-OMcp20_250*ROcp20_450);
    RLcp20_152 = ROcp20_751*s->dpt[3][52];
    RLcp20_252 = ROcp20_851*s->dpt[3][52];
    RLcp20_352 = ROcp20_951*s->dpt[3][52];
    OMcp20_152 = OMcp20_151+ROcp20_151*qd[52];
    OMcp20_252 = OMcp20_251+ROcp20_251*qd[52];
    OMcp20_352 = OMcp20_351+ROcp20_351*qd[52];
    ORcp20_152 = OMcp20_251*RLcp20_352-OMcp20_351*RLcp20_252;
    ORcp20_252 = -(OMcp20_151*RLcp20_352-OMcp20_351*RLcp20_152);
    ORcp20_352 = OMcp20_151*RLcp20_252-OMcp20_251*RLcp20_152;
    OPcp20_152 = OPcp20_151+ROcp20_151*qdd[52]+qd[52]*(OMcp20_251*ROcp20_351-OMcp20_351*ROcp20_251);
    OPcp20_252 = OPcp20_251+ROcp20_251*qdd[52]-qd[52]*(OMcp20_151*ROcp20_351-OMcp20_351*ROcp20_151);
    OPcp20_352 = OPcp20_351+ROcp20_351*qdd[52]+qd[52]*(OMcp20_151*ROcp20_251-OMcp20_251*ROcp20_151);
    RLcp20_176 = ROcp20_752*s->dpt[3][54];
    RLcp20_276 = ROcp20_852*s->dpt[3][54];
    RLcp20_376 = ROcp20_952*s->dpt[3][54];
    POcp20_176 = RLcp20_131+RLcp20_134+RLcp20_144+RLcp20_147+RLcp20_148+RLcp20_149+RLcp20_150+RLcp20_152+RLcp20_176+q[1];
    POcp20_276 = RLcp20_231+RLcp20_234+RLcp20_244+RLcp20_247+RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252+RLcp20_276+q[2];
    POcp20_376 = RLcp20_331+RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376+q[3];
    JTcp20_276_4 = -(RLcp20_331+RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376);
    JTcp20_376_4 = RLcp20_231+RLcp20_234+RLcp20_244+RLcp20_247+RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252+RLcp20_276;
    JTcp20_176_5 = C4*(RLcp20_331+RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352)-S4*(
 RLcp20_231+RLcp20_234)-S4*(RLcp20_244+RLcp20_247)-S4*(RLcp20_248+RLcp20_249)-S4*(RLcp20_250+RLcp20_252)-RLcp20_276*S4+
 RLcp20_376*C4;
    JTcp20_276_5 = S4*(RLcp20_131+RLcp20_134+RLcp20_144+RLcp20_147+RLcp20_148+RLcp20_149+RLcp20_150+RLcp20_152+RLcp20_176);
    JTcp20_376_5 = -C4*(RLcp20_131+RLcp20_134+RLcp20_144+RLcp20_147+RLcp20_148+RLcp20_149+RLcp20_150+RLcp20_152+RLcp20_176
 );
    JTcp20_176_6 = ROcp20_85*(RLcp20_331+RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352)-
 ROcp20_95*(RLcp20_231+RLcp20_234)-ROcp20_95*(RLcp20_244+RLcp20_247)-ROcp20_95*(RLcp20_248+RLcp20_249)-ROcp20_95*(RLcp20_250+
 RLcp20_252)-RLcp20_276*ROcp20_95+RLcp20_376*ROcp20_85;
    JTcp20_276_6 = -(RLcp20_376*S5-ROcp20_95*(RLcp20_131+RLcp20_134+RLcp20_144+RLcp20_147+RLcp20_148+RLcp20_149+RLcp20_150
 +RLcp20_152+RLcp20_176)+S5*(RLcp20_331+RLcp20_334)+S5*(RLcp20_344+RLcp20_347)+S5*(RLcp20_348+RLcp20_349)+S5*(RLcp20_350+
 RLcp20_352));
    JTcp20_376_6 = RLcp20_276*S5-ROcp20_85*(RLcp20_131+RLcp20_134+RLcp20_144+RLcp20_147+RLcp20_148+RLcp20_149+RLcp20_150+
 RLcp20_152+RLcp20_176)+S5*(RLcp20_231+RLcp20_234)+S5*(RLcp20_244+RLcp20_247)+S5*(RLcp20_248+RLcp20_249)+S5*(RLcp20_250+
 RLcp20_252);
    JTcp20_176_7 = ROcp20_56*(RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-
 ROcp20_66*(RLcp20_234+RLcp20_244)-ROcp20_66*(RLcp20_247+RLcp20_248)-ROcp20_66*(RLcp20_249+RLcp20_250)-ROcp20_66*(RLcp20_252+
 RLcp20_276);
    JTcp20_276_7 = -(ROcp20_46*(RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-
 ROcp20_66*(RLcp20_134+RLcp20_144)-ROcp20_66*(RLcp20_147+RLcp20_148)-ROcp20_66*(RLcp20_149+RLcp20_150)-ROcp20_66*(RLcp20_152+
 RLcp20_176));
    JTcp20_376_7 = ROcp20_46*(RLcp20_234+RLcp20_244+RLcp20_247+RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252+RLcp20_276)-
 ROcp20_56*(RLcp20_134+RLcp20_144)-ROcp20_56*(RLcp20_147+RLcp20_148)-ROcp20_56*(RLcp20_149+RLcp20_150)-ROcp20_56*(RLcp20_152+
 RLcp20_176);
    JTcp20_176_8 = ROcp20_231*(RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-
 ROcp20_331*(RLcp20_234+RLcp20_244)-ROcp20_331*(RLcp20_247+RLcp20_248)-ROcp20_331*(RLcp20_249+RLcp20_250)-ROcp20_331*(
 RLcp20_252+RLcp20_276);
    JTcp20_276_8 = -(ROcp20_131*(RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-
 ROcp20_331*(RLcp20_134+RLcp20_144)-ROcp20_331*(RLcp20_147+RLcp20_148)-ROcp20_331*(RLcp20_149+RLcp20_150)-ROcp20_331*(
 RLcp20_152+RLcp20_176));
    JTcp20_376_8 = ROcp20_131*(RLcp20_234+RLcp20_244+RLcp20_247+RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252+RLcp20_276)-
 ROcp20_231*(RLcp20_134+RLcp20_144)-ROcp20_231*(RLcp20_147+RLcp20_148)-ROcp20_231*(RLcp20_149+RLcp20_150)-ROcp20_231*(
 RLcp20_152+RLcp20_176);
    JTcp20_176_9 = ROcp20_532*(RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-
 ROcp20_632*(RLcp20_234+RLcp20_244)-ROcp20_632*(RLcp20_247+RLcp20_248)-ROcp20_632*(RLcp20_249+RLcp20_250)-ROcp20_632*(
 RLcp20_252+RLcp20_276);
    JTcp20_276_9 = -(ROcp20_432*(RLcp20_334+RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-
 ROcp20_632*(RLcp20_134+RLcp20_144)-ROcp20_632*(RLcp20_147+RLcp20_148)-ROcp20_632*(RLcp20_149+RLcp20_150)-ROcp20_632*(
 RLcp20_152+RLcp20_176));
    JTcp20_376_9 = ROcp20_432*(RLcp20_234+RLcp20_244+RLcp20_247+RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252+RLcp20_276)-
 ROcp20_532*(RLcp20_134+RLcp20_144)-ROcp20_532*(RLcp20_147+RLcp20_148)-ROcp20_532*(RLcp20_149+RLcp20_150)-ROcp20_532*(
 RLcp20_152+RLcp20_176);
    JTcp20_176_10 = ROcp20_833*(RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352)-ROcp20_933*(RLcp20_244+
 RLcp20_247)-ROcp20_933*(RLcp20_248+RLcp20_249)-ROcp20_933*(RLcp20_250+RLcp20_252)-RLcp20_276*ROcp20_933+RLcp20_376*
 ROcp20_833;
    JTcp20_276_10 = RLcp20_176*ROcp20_933-RLcp20_376*ROcp20_733-ROcp20_733*(RLcp20_344+RLcp20_347+RLcp20_348+RLcp20_349+
 RLcp20_350+RLcp20_352)+ROcp20_933*(RLcp20_144+RLcp20_147)+ROcp20_933*(RLcp20_148+RLcp20_149)+ROcp20_933*(RLcp20_150+
 RLcp20_152);
    JTcp20_376_10 = ROcp20_733*(RLcp20_244+RLcp20_247+RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252)-ROcp20_833*(RLcp20_144+
 RLcp20_147)-ROcp20_833*(RLcp20_148+RLcp20_149)-ROcp20_833*(RLcp20_150+RLcp20_152)-RLcp20_176*ROcp20_833+RLcp20_276*
 ROcp20_733;
    JTcp20_176_11 = ROcp20_234*(RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-ROcp20_334*(RLcp20_247+
 RLcp20_248)-ROcp20_334*(RLcp20_249+RLcp20_250)-ROcp20_334*(RLcp20_252+RLcp20_276);
    JTcp20_276_11 = -(ROcp20_134*(RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-ROcp20_334*(
 RLcp20_147+RLcp20_148)-ROcp20_334*(RLcp20_149+RLcp20_150)-ROcp20_334*(RLcp20_152+RLcp20_176));
    JTcp20_376_11 = ROcp20_134*(RLcp20_247+RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252+RLcp20_276)-ROcp20_234*(RLcp20_147+
 RLcp20_148)-ROcp20_234*(RLcp20_149+RLcp20_150)-ROcp20_234*(RLcp20_152+RLcp20_176);
    JTcp20_176_12 = ROcp20_844*(RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-ROcp20_944*(RLcp20_247+
 RLcp20_248)-ROcp20_944*(RLcp20_249+RLcp20_250)-ROcp20_944*(RLcp20_252+RLcp20_276);
    JTcp20_276_12 = -(ROcp20_744*(RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-ROcp20_944*(
 RLcp20_147+RLcp20_148)-ROcp20_944*(RLcp20_149+RLcp20_150)-ROcp20_944*(RLcp20_152+RLcp20_176));
    JTcp20_376_12 = ROcp20_744*(RLcp20_247+RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252+RLcp20_276)-ROcp20_844*(RLcp20_147+
 RLcp20_148)-ROcp20_844*(RLcp20_149+RLcp20_150)-ROcp20_844*(RLcp20_152+RLcp20_176);
    JTcp20_176_13 = ROcp20_545*(RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-ROcp20_645*(RLcp20_247+
 RLcp20_248)-ROcp20_645*(RLcp20_249+RLcp20_250)-ROcp20_645*(RLcp20_252+RLcp20_276);
    JTcp20_276_13 = -(ROcp20_445*(RLcp20_347+RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-ROcp20_645*(
 RLcp20_147+RLcp20_148)-ROcp20_645*(RLcp20_149+RLcp20_150)-ROcp20_645*(RLcp20_152+RLcp20_176));
    JTcp20_376_13 = ROcp20_445*(RLcp20_247+RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252+RLcp20_276)-ROcp20_545*(RLcp20_147+
 RLcp20_148)-ROcp20_545*(RLcp20_149+RLcp20_150)-ROcp20_545*(RLcp20_152+RLcp20_176);
    JTcp20_176_14 = ROcp20_246*(RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352)-ROcp20_346*(RLcp20_248+RLcp20_249)-ROcp20_346
 *(RLcp20_250+RLcp20_252)-RLcp20_276*ROcp20_346+RLcp20_376*ROcp20_246;
    JTcp20_276_14 = RLcp20_176*ROcp20_346-RLcp20_376*ROcp20_146-ROcp20_146*(RLcp20_348+RLcp20_349+RLcp20_350+RLcp20_352)+
 ROcp20_346*(RLcp20_148+RLcp20_149)+ROcp20_346*(RLcp20_150+RLcp20_152);
    JTcp20_376_14 = ROcp20_146*(RLcp20_248+RLcp20_249+RLcp20_250+RLcp20_252)-ROcp20_246*(RLcp20_148+RLcp20_149)-ROcp20_246
 *(RLcp20_150+RLcp20_152)-RLcp20_176*ROcp20_246+RLcp20_276*ROcp20_146;
    JTcp20_176_15 = ROcp20_847*(RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-ROcp20_947*(RLcp20_249+RLcp20_250)-ROcp20_947
 *(RLcp20_252+RLcp20_276);
    JTcp20_276_15 = -(ROcp20_747*(RLcp20_349+RLcp20_350+RLcp20_352+RLcp20_376)-ROcp20_947*(RLcp20_149+RLcp20_150)-
 ROcp20_947*(RLcp20_152+RLcp20_176));
    JTcp20_376_15 = ROcp20_747*(RLcp20_249+RLcp20_250+RLcp20_252+RLcp20_276)-ROcp20_847*(RLcp20_149+RLcp20_150)-ROcp20_847
 *(RLcp20_152+RLcp20_176);
    JTcp20_176_16 = ROcp20_548*(RLcp20_350+RLcp20_352)-ROcp20_648*(RLcp20_250+RLcp20_252)-RLcp20_276*ROcp20_648+RLcp20_376
 *ROcp20_548;
    JTcp20_276_16 = RLcp20_176*ROcp20_648-RLcp20_376*ROcp20_448-ROcp20_448*(RLcp20_350+RLcp20_352)+ROcp20_648*(RLcp20_150+
 RLcp20_152);
    JTcp20_376_16 = ROcp20_448*(RLcp20_250+RLcp20_252)-ROcp20_548*(RLcp20_150+RLcp20_152)-RLcp20_176*ROcp20_548+RLcp20_276
 *ROcp20_448;
    JTcp20_176_17 = ROcp20_849*(RLcp20_352+RLcp20_376)-ROcp20_949*(RLcp20_252+RLcp20_276);
    JTcp20_276_17 = -(ROcp20_749*(RLcp20_352+RLcp20_376)-ROcp20_949*(RLcp20_152+RLcp20_176));
    JTcp20_376_17 = ROcp20_749*(RLcp20_252+RLcp20_276)-ROcp20_849*(RLcp20_152+RLcp20_176);
    JTcp20_176_18 = ROcp20_550*(RLcp20_352+RLcp20_376)-ROcp20_650*(RLcp20_252+RLcp20_276);
    JTcp20_276_18 = -(ROcp20_450*(RLcp20_352+RLcp20_376)-ROcp20_650*(RLcp20_152+RLcp20_176));
    JTcp20_376_18 = ROcp20_450*(RLcp20_252+RLcp20_276)-ROcp20_550*(RLcp20_152+RLcp20_176);
    JTcp20_176_19 = -(RLcp20_276*ROcp20_351-RLcp20_376*ROcp20_251);
    JTcp20_276_19 = RLcp20_176*ROcp20_351-RLcp20_376*ROcp20_151;
    JTcp20_376_19 = -(RLcp20_176*ROcp20_251-RLcp20_276*ROcp20_151);
    ORcp20_176 = OMcp20_252*RLcp20_376-OMcp20_352*RLcp20_276;
    ORcp20_276 = -(OMcp20_152*RLcp20_376-OMcp20_352*RLcp20_176);
    ORcp20_376 = OMcp20_152*RLcp20_276-OMcp20_252*RLcp20_176;
    VIcp20_176 = ORcp20_131+ORcp20_134+ORcp20_144+ORcp20_147+ORcp20_148+ORcp20_149+ORcp20_150+ORcp20_152+ORcp20_176+qd[1];
    VIcp20_276 = ORcp20_231+ORcp20_234+ORcp20_244+ORcp20_247+ORcp20_248+ORcp20_249+ORcp20_250+ORcp20_252+ORcp20_276+qd[2];
    VIcp20_376 = ORcp20_331+ORcp20_334+ORcp20_344+ORcp20_347+ORcp20_348+ORcp20_349+ORcp20_350+ORcp20_352+ORcp20_376+qd[3];
    ACcp20_176 = qdd[1]+OMcp20_233*ORcp20_334+OMcp20_234*ORcp20_344+OMcp20_246*ORcp20_347+OMcp20_247*ORcp20_348+OMcp20_248
 *ORcp20_349+OMcp20_249*ORcp20_350+OMcp20_251*ORcp20_352+OMcp20_252*ORcp20_376+OMcp20_26*ORcp20_331-OMcp20_333*ORcp20_234-
 OMcp20_334*ORcp20_244-OMcp20_346*ORcp20_247-OMcp20_347*ORcp20_248-OMcp20_348*ORcp20_249-OMcp20_349*ORcp20_250-OMcp20_351*
 ORcp20_252-OMcp20_352*ORcp20_276-OMcp20_36*ORcp20_231+OPcp20_233*RLcp20_334+OPcp20_234*RLcp20_344+OPcp20_246*RLcp20_347+
 OPcp20_247*RLcp20_348+OPcp20_248*RLcp20_349+OPcp20_249*RLcp20_350+OPcp20_251*RLcp20_352+OPcp20_252*RLcp20_376+OPcp20_26*
 RLcp20_331-OPcp20_333*RLcp20_234-OPcp20_334*RLcp20_244-OPcp20_346*RLcp20_247-OPcp20_347*RLcp20_248-OPcp20_348*RLcp20_249-
 OPcp20_349*RLcp20_250-OPcp20_351*RLcp20_252-OPcp20_352*RLcp20_276-OPcp20_36*RLcp20_231;
    ACcp20_276 = qdd[2]-OMcp20_133*ORcp20_334-OMcp20_134*ORcp20_344-OMcp20_146*ORcp20_347-OMcp20_147*ORcp20_348-OMcp20_148
 *ORcp20_349-OMcp20_149*ORcp20_350-OMcp20_151*ORcp20_352-OMcp20_152*ORcp20_376-OMcp20_16*ORcp20_331+OMcp20_333*ORcp20_134+
 OMcp20_334*ORcp20_144+OMcp20_346*ORcp20_147+OMcp20_347*ORcp20_148+OMcp20_348*ORcp20_149+OMcp20_349*ORcp20_150+OMcp20_351*
 ORcp20_152+OMcp20_352*ORcp20_176+OMcp20_36*ORcp20_131-OPcp20_133*RLcp20_334-OPcp20_134*RLcp20_344-OPcp20_146*RLcp20_347-
 OPcp20_147*RLcp20_348-OPcp20_148*RLcp20_349-OPcp20_149*RLcp20_350-OPcp20_151*RLcp20_352-OPcp20_152*RLcp20_376-OPcp20_16*
 RLcp20_331+OPcp20_333*RLcp20_134+OPcp20_334*RLcp20_144+OPcp20_346*RLcp20_147+OPcp20_347*RLcp20_148+OPcp20_348*RLcp20_149+
 OPcp20_349*RLcp20_150+OPcp20_351*RLcp20_152+OPcp20_352*RLcp20_176+OPcp20_36*RLcp20_131;
    ACcp20_376 = qdd[3]+OMcp20_133*ORcp20_234+OMcp20_134*ORcp20_244+OMcp20_146*ORcp20_247+OMcp20_147*ORcp20_248+OMcp20_148
 *ORcp20_249+OMcp20_149*ORcp20_250+OMcp20_151*ORcp20_252+OMcp20_152*ORcp20_276+OMcp20_16*ORcp20_231-OMcp20_233*ORcp20_134-
 OMcp20_234*ORcp20_144-OMcp20_246*ORcp20_147-OMcp20_247*ORcp20_148-OMcp20_248*ORcp20_149-OMcp20_249*ORcp20_150-OMcp20_251*
 ORcp20_152-OMcp20_252*ORcp20_176-OMcp20_26*ORcp20_131+OPcp20_133*RLcp20_234+OPcp20_134*RLcp20_244+OPcp20_146*RLcp20_247+
 OPcp20_147*RLcp20_248+OPcp20_148*RLcp20_249+OPcp20_149*RLcp20_250+OPcp20_151*RLcp20_252+OPcp20_152*RLcp20_276+OPcp20_16*
 RLcp20_231-OPcp20_233*RLcp20_134-OPcp20_234*RLcp20_144-OPcp20_246*RLcp20_147-OPcp20_247*RLcp20_148-OPcp20_248*RLcp20_149-
 OPcp20_249*RLcp20_150-OPcp20_251*RLcp20_152-OPcp20_252*RLcp20_176-OPcp20_26*RLcp20_131;

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp20_176;
    sens->P[2] = POcp20_276;
    sens->P[3] = POcp20_376;
    sens->R[1][1] = ROcp20_151;
    sens->R[1][2] = ROcp20_251;
    sens->R[1][3] = ROcp20_351;
    sens->R[2][1] = ROcp20_452;
    sens->R[2][2] = ROcp20_552;
    sens->R[2][3] = ROcp20_652;
    sens->R[3][1] = ROcp20_752;
    sens->R[3][2] = ROcp20_852;
    sens->R[3][3] = ROcp20_952;
    sens->V[1] = VIcp20_176;
    sens->V[2] = VIcp20_276;
    sens->V[3] = VIcp20_376;
    sens->OM[1] = OMcp20_152;
    sens->OM[2] = OMcp20_252;
    sens->OM[3] = OMcp20_352;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp20_176_5;
    sens->J[1][6] = JTcp20_176_6;
    sens->J[1][31] = JTcp20_176_7;
    sens->J[1][32] = JTcp20_176_8;
    sens->J[1][33] = JTcp20_176_9;
    sens->J[1][34] = JTcp20_176_10;
    sens->J[1][44] = JTcp20_176_11;
    sens->J[1][45] = JTcp20_176_12;
    sens->J[1][46] = JTcp20_176_13;
    sens->J[1][47] = JTcp20_176_14;
    sens->J[1][48] = JTcp20_176_15;
    sens->J[1][49] = JTcp20_176_16;
    sens->J[1][50] = JTcp20_176_17;
    sens->J[1][51] = JTcp20_176_18;
    sens->J[1][52] = JTcp20_176_19;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp20_276_4;
    sens->J[2][5] = JTcp20_276_5;
    sens->J[2][6] = JTcp20_276_6;
    sens->J[2][31] = JTcp20_276_7;
    sens->J[2][32] = JTcp20_276_8;
    sens->J[2][33] = JTcp20_276_9;
    sens->J[2][34] = JTcp20_276_10;
    sens->J[2][44] = JTcp20_276_11;
    sens->J[2][45] = JTcp20_276_12;
    sens->J[2][46] = JTcp20_276_13;
    sens->J[2][47] = JTcp20_276_14;
    sens->J[2][48] = JTcp20_276_15;
    sens->J[2][49] = JTcp20_276_16;
    sens->J[2][50] = JTcp20_276_17;
    sens->J[2][51] = JTcp20_276_18;
    sens->J[2][52] = JTcp20_276_19;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp20_376_4;
    sens->J[3][5] = JTcp20_376_5;
    sens->J[3][6] = JTcp20_376_6;
    sens->J[3][31] = JTcp20_376_7;
    sens->J[3][32] = JTcp20_376_8;
    sens->J[3][33] = JTcp20_376_9;
    sens->J[3][34] = JTcp20_376_10;
    sens->J[3][44] = JTcp20_376_11;
    sens->J[3][45] = JTcp20_376_12;
    sens->J[3][46] = JTcp20_376_13;
    sens->J[3][47] = JTcp20_376_14;
    sens->J[3][48] = JTcp20_376_15;
    sens->J[3][49] = JTcp20_376_16;
    sens->J[3][50] = JTcp20_376_17;
    sens->J[3][51] = JTcp20_376_18;
    sens->J[3][52] = JTcp20_376_19;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp20_46;
    sens->J[4][32] = ROcp20_131;
    sens->J[4][33] = ROcp20_432;
    sens->J[4][34] = ROcp20_733;
    sens->J[4][44] = ROcp20_134;
    sens->J[4][45] = ROcp20_744;
    sens->J[4][46] = ROcp20_445;
    sens->J[4][47] = ROcp20_146;
    sens->J[4][48] = ROcp20_747;
    sens->J[4][49] = ROcp20_448;
    sens->J[4][50] = ROcp20_749;
    sens->J[4][51] = ROcp20_450;
    sens->J[4][52] = ROcp20_151;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp20_85;
    sens->J[5][31] = ROcp20_56;
    sens->J[5][32] = ROcp20_231;
    sens->J[5][33] = ROcp20_532;
    sens->J[5][34] = ROcp20_833;
    sens->J[5][44] = ROcp20_234;
    sens->J[5][45] = ROcp20_844;
    sens->J[5][46] = ROcp20_545;
    sens->J[5][47] = ROcp20_246;
    sens->J[5][48] = ROcp20_847;
    sens->J[5][49] = ROcp20_548;
    sens->J[5][50] = ROcp20_849;
    sens->J[5][51] = ROcp20_550;
    sens->J[5][52] = ROcp20_251;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp20_95;
    sens->J[6][31] = ROcp20_66;
    sens->J[6][32] = ROcp20_331;
    sens->J[6][33] = ROcp20_632;
    sens->J[6][34] = ROcp20_933;
    sens->J[6][44] = ROcp20_334;
    sens->J[6][45] = ROcp20_944;
    sens->J[6][46] = ROcp20_645;
    sens->J[6][47] = ROcp20_346;
    sens->J[6][48] = ROcp20_947;
    sens->J[6][49] = ROcp20_648;
    sens->J[6][50] = ROcp20_949;
    sens->J[6][51] = ROcp20_650;
    sens->J[6][52] = ROcp20_351;
    sens->A[1] = ACcp20_176;
    sens->A[2] = ACcp20_276;
    sens->A[3] = ACcp20_376;
    sens->OMP[1] = OPcp20_152;
    sens->OMP[2] = OPcp20_252;
    sens->OMP[3] = OPcp20_352;
 
// 
break;
case 22:
 


// = = Block_1_0_0_22_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp21_25 = qd[5]*C4;
    OMcp21_35 = qd[5]*S4;
    OMcp21_16 = qd[4]+qd[6]*S5;
    OMcp21_26 = OMcp21_25+ROcp21_85*qd[6];
    OMcp21_36 = OMcp21_35+ROcp21_95*qd[6];
    OPcp21_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp21_26 = ROcp21_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp21_35*S5-ROcp21_95*qd[4]);
    OPcp21_36 = ROcp21_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp21_25*S5-ROcp21_85*qd[4]);

// = = Block_1_0_0_22_0_4 = = 
 
// Sensor Kinematics 


    ROcp21_131 = ROcp21_16*C31-S31*S5;
    ROcp21_231 = ROcp21_26*C31-ROcp21_85*S31;
    ROcp21_331 = ROcp21_36*C31-ROcp21_95*S31;
    ROcp21_731 = ROcp21_16*S31+C31*S5;
    ROcp21_831 = ROcp21_26*S31+ROcp21_85*C31;
    ROcp21_931 = ROcp21_36*S31+ROcp21_95*C31;
    ROcp21_432 = ROcp21_46*C32+ROcp21_731*S32;
    ROcp21_532 = ROcp21_56*C32+ROcp21_831*S32;
    ROcp21_632 = ROcp21_66*C32+ROcp21_931*S32;
    ROcp21_732 = -(ROcp21_46*S32-ROcp21_731*C32);
    ROcp21_832 = -(ROcp21_56*S32-ROcp21_831*C32);
    ROcp21_932 = -(ROcp21_66*S32-ROcp21_931*C32);
    ROcp21_133 = ROcp21_131*C33-ROcp21_732*S33;
    ROcp21_233 = ROcp21_231*C33-ROcp21_832*S33;
    ROcp21_333 = ROcp21_331*C33-ROcp21_932*S33;
    ROcp21_733 = ROcp21_131*S33+ROcp21_732*C33;
    ROcp21_833 = ROcp21_231*S33+ROcp21_832*C33;
    ROcp21_933 = ROcp21_331*S33+ROcp21_932*C33;
    ROcp21_134 = ROcp21_133*C34+ROcp21_432*S34;
    ROcp21_234 = ROcp21_233*C34+ROcp21_532*S34;
    ROcp21_334 = ROcp21_333*C34+ROcp21_632*S34;
    ROcp21_434 = -(ROcp21_133*S34-ROcp21_432*C34);
    ROcp21_534 = -(ROcp21_233*S34-ROcp21_532*C34);
    ROcp21_634 = -(ROcp21_333*S34-ROcp21_632*C34);
    RLcp21_131 = ROcp21_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp21_231 = ROcp21_26*s->dpt[1][3]+ROcp21_85*s->dpt[3][3];
    RLcp21_331 = ROcp21_36*s->dpt[1][3]+ROcp21_95*s->dpt[3][3];
    OMcp21_131 = OMcp21_16+ROcp21_46*qd[31];
    OMcp21_231 = OMcp21_26+ROcp21_56*qd[31];
    OMcp21_331 = OMcp21_36+ROcp21_66*qd[31];
    ORcp21_131 = OMcp21_26*RLcp21_331-OMcp21_36*RLcp21_231;
    ORcp21_231 = -(OMcp21_16*RLcp21_331-OMcp21_36*RLcp21_131);
    ORcp21_331 = OMcp21_16*RLcp21_231-OMcp21_26*RLcp21_131;
    OMcp21_132 = OMcp21_131+ROcp21_131*qd[32];
    OMcp21_232 = OMcp21_231+ROcp21_231*qd[32];
    OMcp21_332 = OMcp21_331+ROcp21_331*qd[32];
    OMcp21_133 = OMcp21_132+ROcp21_432*qd[33];
    OMcp21_233 = OMcp21_232+ROcp21_532*qd[33];
    OMcp21_333 = OMcp21_332+ROcp21_632*qd[33];
    OPcp21_133 = OPcp21_16+ROcp21_131*qdd[32]+ROcp21_432*qdd[33]+ROcp21_46*qdd[31]+qd[31]*(OMcp21_26*ROcp21_66-OMcp21_36*
 ROcp21_56)+qd[32]*(OMcp21_231*ROcp21_331-OMcp21_331*ROcp21_231)+qd[33]*(OMcp21_232*ROcp21_632-OMcp21_332*ROcp21_532);
    OPcp21_233 = OPcp21_26+ROcp21_231*qdd[32]+ROcp21_532*qdd[33]+ROcp21_56*qdd[31]-qd[31]*(OMcp21_16*ROcp21_66-OMcp21_36*
 ROcp21_46)-qd[32]*(OMcp21_131*ROcp21_331-OMcp21_331*ROcp21_131)-qd[33]*(OMcp21_132*ROcp21_632-OMcp21_332*ROcp21_432);
    OPcp21_333 = OPcp21_36+ROcp21_331*qdd[32]+ROcp21_632*qdd[33]+ROcp21_66*qdd[31]+qd[31]*(OMcp21_16*ROcp21_56-OMcp21_26*
 ROcp21_46)+qd[32]*(OMcp21_131*ROcp21_231-OMcp21_231*ROcp21_131)+qd[33]*(OMcp21_132*ROcp21_532-OMcp21_232*ROcp21_432);
    RLcp21_134 = ROcp21_733*s->dpt[3][21];
    RLcp21_234 = ROcp21_833*s->dpt[3][21];
    RLcp21_334 = ROcp21_933*s->dpt[3][21];
    OMcp21_134 = OMcp21_133+ROcp21_733*qd[34];
    OMcp21_234 = OMcp21_233+ROcp21_833*qd[34];
    OMcp21_334 = OMcp21_333+ROcp21_933*qd[34];
    ORcp21_134 = OMcp21_233*RLcp21_334-OMcp21_333*RLcp21_234;
    ORcp21_234 = -(OMcp21_133*RLcp21_334-OMcp21_333*RLcp21_134);
    ORcp21_334 = OMcp21_133*RLcp21_234-OMcp21_233*RLcp21_134;
    OPcp21_134 = OPcp21_133+ROcp21_733*qdd[34]+qd[34]*(OMcp21_233*ROcp21_933-OMcp21_333*ROcp21_833);
    OPcp21_234 = OPcp21_233+ROcp21_833*qdd[34]-qd[34]*(OMcp21_133*ROcp21_933-OMcp21_333*ROcp21_733);
    OPcp21_334 = OPcp21_333+ROcp21_933*qdd[34]+qd[34]*(OMcp21_133*ROcp21_833-OMcp21_233*ROcp21_733);

// = = Block_1_0_0_22_0_6 = = 
 
// Sensor Kinematics 


    ROcp21_444 = ROcp21_434*C44+ROcp21_733*S44;
    ROcp21_544 = ROcp21_534*C44+ROcp21_833*S44;
    ROcp21_644 = ROcp21_634*C44+ROcp21_933*S44;
    ROcp21_744 = -(ROcp21_434*S44-ROcp21_733*C44);
    ROcp21_844 = -(ROcp21_534*S44-ROcp21_833*C44);
    ROcp21_944 = -(ROcp21_634*S44-ROcp21_933*C44);
    ROcp21_145 = ROcp21_134*C45+ROcp21_444*S45;
    ROcp21_245 = ROcp21_234*C45+ROcp21_544*S45;
    ROcp21_345 = ROcp21_334*C45+ROcp21_644*S45;
    ROcp21_445 = -(ROcp21_134*S45-ROcp21_444*C45);
    ROcp21_545 = -(ROcp21_234*S45-ROcp21_544*C45);
    ROcp21_645 = -(ROcp21_334*S45-ROcp21_644*C45);
    ROcp21_146 = ROcp21_145*C46-ROcp21_744*S46;
    ROcp21_246 = ROcp21_245*C46-ROcp21_844*S46;
    ROcp21_346 = ROcp21_345*C46-ROcp21_944*S46;
    ROcp21_746 = ROcp21_145*S46+ROcp21_744*C46;
    ROcp21_846 = ROcp21_245*S46+ROcp21_844*C46;
    ROcp21_946 = ROcp21_345*S46+ROcp21_944*C46;
    ROcp21_447 = ROcp21_445*C47+ROcp21_746*S47;
    ROcp21_547 = ROcp21_545*C47+ROcp21_846*S47;
    ROcp21_647 = ROcp21_645*C47+ROcp21_946*S47;
    ROcp21_747 = -(ROcp21_445*S47-ROcp21_746*C47);
    ROcp21_847 = -(ROcp21_545*S47-ROcp21_846*C47);
    ROcp21_947 = -(ROcp21_645*S47-ROcp21_946*C47);
    ROcp21_148 = ROcp21_146*C48+ROcp21_447*S48;
    ROcp21_248 = ROcp21_246*C48+ROcp21_547*S48;
    ROcp21_348 = ROcp21_346*C48+ROcp21_647*S48;
    ROcp21_448 = -(ROcp21_146*S48-ROcp21_447*C48);
    ROcp21_548 = -(ROcp21_246*S48-ROcp21_547*C48);
    ROcp21_648 = -(ROcp21_346*S48-ROcp21_647*C48);
    ROcp21_149 = ROcp21_148*C49-ROcp21_747*S49;
    ROcp21_249 = ROcp21_248*C49-ROcp21_847*S49;
    ROcp21_349 = ROcp21_348*C49-ROcp21_947*S49;
    ROcp21_749 = ROcp21_148*S49+ROcp21_747*C49;
    ROcp21_849 = ROcp21_248*S49+ROcp21_847*C49;
    ROcp21_949 = ROcp21_348*S49+ROcp21_947*C49;
    ROcp21_150 = ROcp21_149*C50+ROcp21_448*S50;
    ROcp21_250 = ROcp21_249*C50+ROcp21_548*S50;
    ROcp21_350 = ROcp21_349*C50+ROcp21_648*S50;
    ROcp21_450 = -(ROcp21_149*S50-ROcp21_448*C50);
    ROcp21_550 = -(ROcp21_249*S50-ROcp21_548*C50);
    ROcp21_650 = -(ROcp21_349*S50-ROcp21_648*C50);
    ROcp21_151 = ROcp21_150*C51-ROcp21_749*S51;
    ROcp21_251 = ROcp21_250*C51-ROcp21_849*S51;
    ROcp21_351 = ROcp21_350*C51-ROcp21_949*S51;
    ROcp21_751 = ROcp21_150*S51+ROcp21_749*C51;
    ROcp21_851 = ROcp21_250*S51+ROcp21_849*C51;
    ROcp21_951 = ROcp21_350*S51+ROcp21_949*C51;
    ROcp21_452 = ROcp21_450*C52+ROcp21_751*S52;
    ROcp21_552 = ROcp21_550*C52+ROcp21_851*S52;
    ROcp21_652 = ROcp21_650*C52+ROcp21_951*S52;
    ROcp21_752 = -(ROcp21_450*S52-ROcp21_751*C52);
    ROcp21_852 = -(ROcp21_550*S52-ROcp21_851*C52);
    ROcp21_952 = -(ROcp21_650*S52-ROcp21_951*C52);
    RLcp21_144 = ROcp21_134*s->dpt[1][24]+ROcp21_434*s->dpt[2][24]+ROcp21_733*s->dpt[3][24];
    RLcp21_244 = ROcp21_234*s->dpt[1][24]+ROcp21_534*s->dpt[2][24]+ROcp21_833*s->dpt[3][24];
    RLcp21_344 = ROcp21_334*s->dpt[1][24]+ROcp21_634*s->dpt[2][24]+ROcp21_933*s->dpt[3][24];
    ORcp21_144 = OMcp21_234*RLcp21_344-OMcp21_334*RLcp21_244;
    ORcp21_244 = -(OMcp21_134*RLcp21_344-OMcp21_334*RLcp21_144);
    ORcp21_344 = OMcp21_134*RLcp21_244-OMcp21_234*RLcp21_144;
    OMcp21_146 = OMcp21_134+ROcp21_445*qd[46];
    OMcp21_246 = OMcp21_234+ROcp21_545*qd[46];
    OMcp21_346 = OMcp21_334+ROcp21_645*qd[46];
    OPcp21_146 = OPcp21_134+ROcp21_445*qdd[46]+qd[46]*(OMcp21_234*ROcp21_645-OMcp21_334*ROcp21_545);
    OPcp21_246 = OPcp21_234+ROcp21_545*qdd[46]-qd[46]*(OMcp21_134*ROcp21_645-OMcp21_334*ROcp21_445);
    OPcp21_346 = OPcp21_334+ROcp21_645*qdd[46]+qd[46]*(OMcp21_134*ROcp21_545-OMcp21_234*ROcp21_445);
    RLcp21_147 = ROcp21_445*s->dpt[2][41]+ROcp21_746*s->dpt[3][41];
    RLcp21_247 = ROcp21_545*s->dpt[2][41]+ROcp21_846*s->dpt[3][41];
    RLcp21_347 = ROcp21_645*s->dpt[2][41]+ROcp21_946*s->dpt[3][41];
    OMcp21_147 = OMcp21_146+ROcp21_146*qd[47];
    OMcp21_247 = OMcp21_246+ROcp21_246*qd[47];
    OMcp21_347 = OMcp21_346+ROcp21_346*qd[47];
    ORcp21_147 = OMcp21_246*RLcp21_347-OMcp21_346*RLcp21_247;
    ORcp21_247 = -(OMcp21_146*RLcp21_347-OMcp21_346*RLcp21_147);
    ORcp21_347 = OMcp21_146*RLcp21_247-OMcp21_246*RLcp21_147;
    OPcp21_147 = OPcp21_146+ROcp21_146*qdd[47]+qd[47]*(OMcp21_246*ROcp21_346-OMcp21_346*ROcp21_246);
    OPcp21_247 = OPcp21_246+ROcp21_246*qdd[47]-qd[47]*(OMcp21_146*ROcp21_346-OMcp21_346*ROcp21_146);
    OPcp21_347 = OPcp21_346+ROcp21_346*qdd[47]+qd[47]*(OMcp21_146*ROcp21_246-OMcp21_246*ROcp21_146);
    RLcp21_148 = ROcp21_747*s->dpt[3][43];
    RLcp21_248 = ROcp21_847*s->dpt[3][43];
    RLcp21_348 = ROcp21_947*s->dpt[3][43];
    OMcp21_148 = OMcp21_147+ROcp21_747*qd[48];
    OMcp21_248 = OMcp21_247+ROcp21_847*qd[48];
    OMcp21_348 = OMcp21_347+ROcp21_947*qd[48];
    ORcp21_148 = OMcp21_247*RLcp21_348-OMcp21_347*RLcp21_248;
    ORcp21_248 = -(OMcp21_147*RLcp21_348-OMcp21_347*RLcp21_148);
    ORcp21_348 = OMcp21_147*RLcp21_248-OMcp21_247*RLcp21_148;
    OPcp21_148 = OPcp21_147+ROcp21_747*qdd[48]+qd[48]*(OMcp21_247*ROcp21_947-OMcp21_347*ROcp21_847);
    OPcp21_248 = OPcp21_247+ROcp21_847*qdd[48]-qd[48]*(OMcp21_147*ROcp21_947-OMcp21_347*ROcp21_747);
    OPcp21_348 = OPcp21_347+ROcp21_947*qdd[48]+qd[48]*(OMcp21_147*ROcp21_847-OMcp21_247*ROcp21_747);
    RLcp21_149 = ROcp21_148*s->dpt[1][46]+ROcp21_747*s->dpt[3][46];
    RLcp21_249 = ROcp21_248*s->dpt[1][46]+ROcp21_847*s->dpt[3][46];
    RLcp21_349 = ROcp21_348*s->dpt[1][46]+ROcp21_947*s->dpt[3][46];
    OMcp21_149 = OMcp21_148+ROcp21_448*qd[49];
    OMcp21_249 = OMcp21_248+ROcp21_548*qd[49];
    OMcp21_349 = OMcp21_348+ROcp21_648*qd[49];
    ORcp21_149 = OMcp21_248*RLcp21_349-OMcp21_348*RLcp21_249;
    ORcp21_249 = -(OMcp21_148*RLcp21_349-OMcp21_348*RLcp21_149);
    ORcp21_349 = OMcp21_148*RLcp21_249-OMcp21_248*RLcp21_149;
    OPcp21_149 = OPcp21_148+ROcp21_448*qdd[49]+qd[49]*(OMcp21_248*ROcp21_648-OMcp21_348*ROcp21_548);
    OPcp21_249 = OPcp21_248+ROcp21_548*qdd[49]-qd[49]*(OMcp21_148*ROcp21_648-OMcp21_348*ROcp21_448);
    OPcp21_349 = OPcp21_348+ROcp21_648*qdd[49]+qd[49]*(OMcp21_148*ROcp21_548-OMcp21_248*ROcp21_448);
    RLcp21_150 = ROcp21_149*s->dpt[1][48]+ROcp21_749*s->dpt[3][48];
    RLcp21_250 = ROcp21_249*s->dpt[1][48]+ROcp21_849*s->dpt[3][48];
    RLcp21_350 = ROcp21_349*s->dpt[1][48]+ROcp21_949*s->dpt[3][48];
    OMcp21_150 = OMcp21_149+ROcp21_749*qd[50];
    OMcp21_250 = OMcp21_249+ROcp21_849*qd[50];
    OMcp21_350 = OMcp21_349+ROcp21_949*qd[50];
    ORcp21_150 = OMcp21_249*RLcp21_350-OMcp21_349*RLcp21_250;
    ORcp21_250 = -(OMcp21_149*RLcp21_350-OMcp21_349*RLcp21_150);
    ORcp21_350 = OMcp21_149*RLcp21_250-OMcp21_249*RLcp21_150;
    OMcp21_151 = OMcp21_150+ROcp21_450*qd[51];
    OMcp21_251 = OMcp21_250+ROcp21_550*qd[51];
    OMcp21_351 = OMcp21_350+ROcp21_650*qd[51];
    OPcp21_151 = OPcp21_149+ROcp21_450*qdd[51]+ROcp21_749*qdd[50]+qd[50]*(OMcp21_249*ROcp21_949-OMcp21_349*ROcp21_849)+
 qd[51]*(OMcp21_250*ROcp21_650-OMcp21_350*ROcp21_550);
    OPcp21_251 = OPcp21_249+ROcp21_550*qdd[51]+ROcp21_849*qdd[50]-qd[50]*(OMcp21_149*ROcp21_949-OMcp21_349*ROcp21_749)-
 qd[51]*(OMcp21_150*ROcp21_650-OMcp21_350*ROcp21_450);
    OPcp21_351 = OPcp21_349+ROcp21_650*qdd[51]+ROcp21_949*qdd[50]+qd[50]*(OMcp21_149*ROcp21_849-OMcp21_249*ROcp21_749)+
 qd[51]*(OMcp21_150*ROcp21_550-OMcp21_250*ROcp21_450);
    RLcp21_152 = ROcp21_751*s->dpt[3][52];
    RLcp21_252 = ROcp21_851*s->dpt[3][52];
    RLcp21_352 = ROcp21_951*s->dpt[3][52];
    OMcp21_152 = OMcp21_151+ROcp21_151*qd[52];
    OMcp21_252 = OMcp21_251+ROcp21_251*qd[52];
    OMcp21_352 = OMcp21_351+ROcp21_351*qd[52];
    ORcp21_152 = OMcp21_251*RLcp21_352-OMcp21_351*RLcp21_252;
    ORcp21_252 = -(OMcp21_151*RLcp21_352-OMcp21_351*RLcp21_152);
    ORcp21_352 = OMcp21_151*RLcp21_252-OMcp21_251*RLcp21_152;
    OPcp21_152 = OPcp21_151+ROcp21_151*qdd[52]+qd[52]*(OMcp21_251*ROcp21_351-OMcp21_351*ROcp21_251);
    OPcp21_252 = OPcp21_251+ROcp21_251*qdd[52]-qd[52]*(OMcp21_151*ROcp21_351-OMcp21_351*ROcp21_151);
    OPcp21_352 = OPcp21_351+ROcp21_351*qdd[52]+qd[52]*(OMcp21_151*ROcp21_251-OMcp21_251*ROcp21_151);
    RLcp21_177 = ROcp21_151*s->dpt[1][55]+ROcp21_452*s->dpt[2][55]+ROcp21_752*s->dpt[3][55];
    RLcp21_277 = ROcp21_251*s->dpt[1][55]+ROcp21_552*s->dpt[2][55]+ROcp21_852*s->dpt[3][55];
    RLcp21_377 = ROcp21_351*s->dpt[1][55]+ROcp21_652*s->dpt[2][55]+ROcp21_952*s->dpt[3][55];
    POcp21_177 = RLcp21_131+RLcp21_134+RLcp21_144+RLcp21_147+RLcp21_148+RLcp21_149+RLcp21_150+RLcp21_152+RLcp21_177+q[1];
    POcp21_277 = RLcp21_231+RLcp21_234+RLcp21_244+RLcp21_247+RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252+RLcp21_277+q[2];
    POcp21_377 = RLcp21_331+RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377+q[3];
    JTcp21_277_4 = -(RLcp21_331+RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377);
    JTcp21_377_4 = RLcp21_231+RLcp21_234+RLcp21_244+RLcp21_247+RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252+RLcp21_277;
    JTcp21_177_5 = C4*(RLcp21_331+RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352)-S4*(
 RLcp21_231+RLcp21_234)-S4*(RLcp21_244+RLcp21_247)-S4*(RLcp21_248+RLcp21_249)-S4*(RLcp21_250+RLcp21_252)-RLcp21_277*S4+
 RLcp21_377*C4;
    JTcp21_277_5 = S4*(RLcp21_131+RLcp21_134+RLcp21_144+RLcp21_147+RLcp21_148+RLcp21_149+RLcp21_150+RLcp21_152+RLcp21_177);
    JTcp21_377_5 = -C4*(RLcp21_131+RLcp21_134+RLcp21_144+RLcp21_147+RLcp21_148+RLcp21_149+RLcp21_150+RLcp21_152+RLcp21_177
 );
    JTcp21_177_6 = ROcp21_85*(RLcp21_331+RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352)-
 ROcp21_95*(RLcp21_231+RLcp21_234)-ROcp21_95*(RLcp21_244+RLcp21_247)-ROcp21_95*(RLcp21_248+RLcp21_249)-ROcp21_95*(RLcp21_250+
 RLcp21_252)-RLcp21_277*ROcp21_95+RLcp21_377*ROcp21_85;
    JTcp21_277_6 = -(RLcp21_377*S5-ROcp21_95*(RLcp21_131+RLcp21_134+RLcp21_144+RLcp21_147+RLcp21_148+RLcp21_149+RLcp21_150
 +RLcp21_152+RLcp21_177)+S5*(RLcp21_331+RLcp21_334)+S5*(RLcp21_344+RLcp21_347)+S5*(RLcp21_348+RLcp21_349)+S5*(RLcp21_350+
 RLcp21_352));
    JTcp21_377_6 = RLcp21_277*S5-ROcp21_85*(RLcp21_131+RLcp21_134+RLcp21_144+RLcp21_147+RLcp21_148+RLcp21_149+RLcp21_150+
 RLcp21_152+RLcp21_177)+S5*(RLcp21_231+RLcp21_234)+S5*(RLcp21_244+RLcp21_247)+S5*(RLcp21_248+RLcp21_249)+S5*(RLcp21_250+
 RLcp21_252);
    JTcp21_177_7 = ROcp21_56*(RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-
 ROcp21_66*(RLcp21_234+RLcp21_244)-ROcp21_66*(RLcp21_247+RLcp21_248)-ROcp21_66*(RLcp21_249+RLcp21_250)-ROcp21_66*(RLcp21_252+
 RLcp21_277);
    JTcp21_277_7 = -(ROcp21_46*(RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-
 ROcp21_66*(RLcp21_134+RLcp21_144)-ROcp21_66*(RLcp21_147+RLcp21_148)-ROcp21_66*(RLcp21_149+RLcp21_150)-ROcp21_66*(RLcp21_152+
 RLcp21_177));
    JTcp21_377_7 = ROcp21_46*(RLcp21_234+RLcp21_244+RLcp21_247+RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252+RLcp21_277)-
 ROcp21_56*(RLcp21_134+RLcp21_144)-ROcp21_56*(RLcp21_147+RLcp21_148)-ROcp21_56*(RLcp21_149+RLcp21_150)-ROcp21_56*(RLcp21_152+
 RLcp21_177);
    JTcp21_177_8 = ROcp21_231*(RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-
 ROcp21_331*(RLcp21_234+RLcp21_244)-ROcp21_331*(RLcp21_247+RLcp21_248)-ROcp21_331*(RLcp21_249+RLcp21_250)-ROcp21_331*(
 RLcp21_252+RLcp21_277);
    JTcp21_277_8 = -(ROcp21_131*(RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-
 ROcp21_331*(RLcp21_134+RLcp21_144)-ROcp21_331*(RLcp21_147+RLcp21_148)-ROcp21_331*(RLcp21_149+RLcp21_150)-ROcp21_331*(
 RLcp21_152+RLcp21_177));
    JTcp21_377_8 = ROcp21_131*(RLcp21_234+RLcp21_244+RLcp21_247+RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252+RLcp21_277)-
 ROcp21_231*(RLcp21_134+RLcp21_144)-ROcp21_231*(RLcp21_147+RLcp21_148)-ROcp21_231*(RLcp21_149+RLcp21_150)-ROcp21_231*(
 RLcp21_152+RLcp21_177);
    JTcp21_177_9 = ROcp21_532*(RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-
 ROcp21_632*(RLcp21_234+RLcp21_244)-ROcp21_632*(RLcp21_247+RLcp21_248)-ROcp21_632*(RLcp21_249+RLcp21_250)-ROcp21_632*(
 RLcp21_252+RLcp21_277);
    JTcp21_277_9 = -(ROcp21_432*(RLcp21_334+RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-
 ROcp21_632*(RLcp21_134+RLcp21_144)-ROcp21_632*(RLcp21_147+RLcp21_148)-ROcp21_632*(RLcp21_149+RLcp21_150)-ROcp21_632*(
 RLcp21_152+RLcp21_177));
    JTcp21_377_9 = ROcp21_432*(RLcp21_234+RLcp21_244+RLcp21_247+RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252+RLcp21_277)-
 ROcp21_532*(RLcp21_134+RLcp21_144)-ROcp21_532*(RLcp21_147+RLcp21_148)-ROcp21_532*(RLcp21_149+RLcp21_150)-ROcp21_532*(
 RLcp21_152+RLcp21_177);
    JTcp21_177_10 = ROcp21_833*(RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352)-ROcp21_933*(RLcp21_244+
 RLcp21_247)-ROcp21_933*(RLcp21_248+RLcp21_249)-ROcp21_933*(RLcp21_250+RLcp21_252)-RLcp21_277*ROcp21_933+RLcp21_377*
 ROcp21_833;
    JTcp21_277_10 = RLcp21_177*ROcp21_933-RLcp21_377*ROcp21_733-ROcp21_733*(RLcp21_344+RLcp21_347+RLcp21_348+RLcp21_349+
 RLcp21_350+RLcp21_352)+ROcp21_933*(RLcp21_144+RLcp21_147)+ROcp21_933*(RLcp21_148+RLcp21_149)+ROcp21_933*(RLcp21_150+
 RLcp21_152);
    JTcp21_377_10 = ROcp21_733*(RLcp21_244+RLcp21_247+RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252)-ROcp21_833*(RLcp21_144+
 RLcp21_147)-ROcp21_833*(RLcp21_148+RLcp21_149)-ROcp21_833*(RLcp21_150+RLcp21_152)-RLcp21_177*ROcp21_833+RLcp21_277*
 ROcp21_733;
    JTcp21_177_11 = ROcp21_234*(RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-ROcp21_334*(RLcp21_247+
 RLcp21_248)-ROcp21_334*(RLcp21_249+RLcp21_250)-ROcp21_334*(RLcp21_252+RLcp21_277);
    JTcp21_277_11 = -(ROcp21_134*(RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-ROcp21_334*(
 RLcp21_147+RLcp21_148)-ROcp21_334*(RLcp21_149+RLcp21_150)-ROcp21_334*(RLcp21_152+RLcp21_177));
    JTcp21_377_11 = ROcp21_134*(RLcp21_247+RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252+RLcp21_277)-ROcp21_234*(RLcp21_147+
 RLcp21_148)-ROcp21_234*(RLcp21_149+RLcp21_150)-ROcp21_234*(RLcp21_152+RLcp21_177);
    JTcp21_177_12 = ROcp21_844*(RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-ROcp21_944*(RLcp21_247+
 RLcp21_248)-ROcp21_944*(RLcp21_249+RLcp21_250)-ROcp21_944*(RLcp21_252+RLcp21_277);
    JTcp21_277_12 = -(ROcp21_744*(RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-ROcp21_944*(
 RLcp21_147+RLcp21_148)-ROcp21_944*(RLcp21_149+RLcp21_150)-ROcp21_944*(RLcp21_152+RLcp21_177));
    JTcp21_377_12 = ROcp21_744*(RLcp21_247+RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252+RLcp21_277)-ROcp21_844*(RLcp21_147+
 RLcp21_148)-ROcp21_844*(RLcp21_149+RLcp21_150)-ROcp21_844*(RLcp21_152+RLcp21_177);
    JTcp21_177_13 = ROcp21_545*(RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-ROcp21_645*(RLcp21_247+
 RLcp21_248)-ROcp21_645*(RLcp21_249+RLcp21_250)-ROcp21_645*(RLcp21_252+RLcp21_277);
    JTcp21_277_13 = -(ROcp21_445*(RLcp21_347+RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-ROcp21_645*(
 RLcp21_147+RLcp21_148)-ROcp21_645*(RLcp21_149+RLcp21_150)-ROcp21_645*(RLcp21_152+RLcp21_177));
    JTcp21_377_13 = ROcp21_445*(RLcp21_247+RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252+RLcp21_277)-ROcp21_545*(RLcp21_147+
 RLcp21_148)-ROcp21_545*(RLcp21_149+RLcp21_150)-ROcp21_545*(RLcp21_152+RLcp21_177);
    JTcp21_177_14 = ROcp21_246*(RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352)-ROcp21_346*(RLcp21_248+RLcp21_249)-ROcp21_346
 *(RLcp21_250+RLcp21_252)-RLcp21_277*ROcp21_346+RLcp21_377*ROcp21_246;
    JTcp21_277_14 = RLcp21_177*ROcp21_346-RLcp21_377*ROcp21_146-ROcp21_146*(RLcp21_348+RLcp21_349+RLcp21_350+RLcp21_352)+
 ROcp21_346*(RLcp21_148+RLcp21_149)+ROcp21_346*(RLcp21_150+RLcp21_152);
    JTcp21_377_14 = ROcp21_146*(RLcp21_248+RLcp21_249+RLcp21_250+RLcp21_252)-ROcp21_246*(RLcp21_148+RLcp21_149)-ROcp21_246
 *(RLcp21_150+RLcp21_152)-RLcp21_177*ROcp21_246+RLcp21_277*ROcp21_146;
    JTcp21_177_15 = ROcp21_847*(RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-ROcp21_947*(RLcp21_249+RLcp21_250)-ROcp21_947
 *(RLcp21_252+RLcp21_277);
    JTcp21_277_15 = -(ROcp21_747*(RLcp21_349+RLcp21_350+RLcp21_352+RLcp21_377)-ROcp21_947*(RLcp21_149+RLcp21_150)-
 ROcp21_947*(RLcp21_152+RLcp21_177));
    JTcp21_377_15 = ROcp21_747*(RLcp21_249+RLcp21_250+RLcp21_252+RLcp21_277)-ROcp21_847*(RLcp21_149+RLcp21_150)-ROcp21_847
 *(RLcp21_152+RLcp21_177);
    JTcp21_177_16 = ROcp21_548*(RLcp21_350+RLcp21_352)-ROcp21_648*(RLcp21_250+RLcp21_252)-RLcp21_277*ROcp21_648+RLcp21_377
 *ROcp21_548;
    JTcp21_277_16 = RLcp21_177*ROcp21_648-RLcp21_377*ROcp21_448-ROcp21_448*(RLcp21_350+RLcp21_352)+ROcp21_648*(RLcp21_150+
 RLcp21_152);
    JTcp21_377_16 = ROcp21_448*(RLcp21_250+RLcp21_252)-ROcp21_548*(RLcp21_150+RLcp21_152)-RLcp21_177*ROcp21_548+RLcp21_277
 *ROcp21_448;
    JTcp21_177_17 = ROcp21_849*(RLcp21_352+RLcp21_377)-ROcp21_949*(RLcp21_252+RLcp21_277);
    JTcp21_277_17 = -(ROcp21_749*(RLcp21_352+RLcp21_377)-ROcp21_949*(RLcp21_152+RLcp21_177));
    JTcp21_377_17 = ROcp21_749*(RLcp21_252+RLcp21_277)-ROcp21_849*(RLcp21_152+RLcp21_177);
    JTcp21_177_18 = ROcp21_550*(RLcp21_352+RLcp21_377)-ROcp21_650*(RLcp21_252+RLcp21_277);
    JTcp21_277_18 = -(ROcp21_450*(RLcp21_352+RLcp21_377)-ROcp21_650*(RLcp21_152+RLcp21_177));
    JTcp21_377_18 = ROcp21_450*(RLcp21_252+RLcp21_277)-ROcp21_550*(RLcp21_152+RLcp21_177);
    JTcp21_177_19 = -(RLcp21_277*ROcp21_351-RLcp21_377*ROcp21_251);
    JTcp21_277_19 = RLcp21_177*ROcp21_351-RLcp21_377*ROcp21_151;
    JTcp21_377_19 = -(RLcp21_177*ROcp21_251-RLcp21_277*ROcp21_151);
    ORcp21_177 = OMcp21_252*RLcp21_377-OMcp21_352*RLcp21_277;
    ORcp21_277 = -(OMcp21_152*RLcp21_377-OMcp21_352*RLcp21_177);
    ORcp21_377 = OMcp21_152*RLcp21_277-OMcp21_252*RLcp21_177;
    VIcp21_177 = ORcp21_131+ORcp21_134+ORcp21_144+ORcp21_147+ORcp21_148+ORcp21_149+ORcp21_150+ORcp21_152+ORcp21_177+qd[1];
    VIcp21_277 = ORcp21_231+ORcp21_234+ORcp21_244+ORcp21_247+ORcp21_248+ORcp21_249+ORcp21_250+ORcp21_252+ORcp21_277+qd[2];
    VIcp21_377 = ORcp21_331+ORcp21_334+ORcp21_344+ORcp21_347+ORcp21_348+ORcp21_349+ORcp21_350+ORcp21_352+ORcp21_377+qd[3];
    ACcp21_177 = qdd[1]+OMcp21_233*ORcp21_334+OMcp21_234*ORcp21_344+OMcp21_246*ORcp21_347+OMcp21_247*ORcp21_348+OMcp21_248
 *ORcp21_349+OMcp21_249*ORcp21_350+OMcp21_251*ORcp21_352+OMcp21_252*ORcp21_377+OMcp21_26*ORcp21_331-OMcp21_333*ORcp21_234-
 OMcp21_334*ORcp21_244-OMcp21_346*ORcp21_247-OMcp21_347*ORcp21_248-OMcp21_348*ORcp21_249-OMcp21_349*ORcp21_250-OMcp21_351*
 ORcp21_252-OMcp21_352*ORcp21_277-OMcp21_36*ORcp21_231+OPcp21_233*RLcp21_334+OPcp21_234*RLcp21_344+OPcp21_246*RLcp21_347+
 OPcp21_247*RLcp21_348+OPcp21_248*RLcp21_349+OPcp21_249*RLcp21_350+OPcp21_251*RLcp21_352+OPcp21_252*RLcp21_377+OPcp21_26*
 RLcp21_331-OPcp21_333*RLcp21_234-OPcp21_334*RLcp21_244-OPcp21_346*RLcp21_247-OPcp21_347*RLcp21_248-OPcp21_348*RLcp21_249-
 OPcp21_349*RLcp21_250-OPcp21_351*RLcp21_252-OPcp21_352*RLcp21_277-OPcp21_36*RLcp21_231;
    ACcp21_277 = qdd[2]-OMcp21_133*ORcp21_334-OMcp21_134*ORcp21_344-OMcp21_146*ORcp21_347-OMcp21_147*ORcp21_348-OMcp21_148
 *ORcp21_349-OMcp21_149*ORcp21_350-OMcp21_151*ORcp21_352-OMcp21_152*ORcp21_377-OMcp21_16*ORcp21_331+OMcp21_333*ORcp21_134+
 OMcp21_334*ORcp21_144+OMcp21_346*ORcp21_147+OMcp21_347*ORcp21_148+OMcp21_348*ORcp21_149+OMcp21_349*ORcp21_150+OMcp21_351*
 ORcp21_152+OMcp21_352*ORcp21_177+OMcp21_36*ORcp21_131-OPcp21_133*RLcp21_334-OPcp21_134*RLcp21_344-OPcp21_146*RLcp21_347-
 OPcp21_147*RLcp21_348-OPcp21_148*RLcp21_349-OPcp21_149*RLcp21_350-OPcp21_151*RLcp21_352-OPcp21_152*RLcp21_377-OPcp21_16*
 RLcp21_331+OPcp21_333*RLcp21_134+OPcp21_334*RLcp21_144+OPcp21_346*RLcp21_147+OPcp21_347*RLcp21_148+OPcp21_348*RLcp21_149+
 OPcp21_349*RLcp21_150+OPcp21_351*RLcp21_152+OPcp21_352*RLcp21_177+OPcp21_36*RLcp21_131;
    ACcp21_377 = qdd[3]+OMcp21_133*ORcp21_234+OMcp21_134*ORcp21_244+OMcp21_146*ORcp21_247+OMcp21_147*ORcp21_248+OMcp21_148
 *ORcp21_249+OMcp21_149*ORcp21_250+OMcp21_151*ORcp21_252+OMcp21_152*ORcp21_277+OMcp21_16*ORcp21_231-OMcp21_233*ORcp21_134-
 OMcp21_234*ORcp21_144-OMcp21_246*ORcp21_147-OMcp21_247*ORcp21_148-OMcp21_248*ORcp21_149-OMcp21_249*ORcp21_150-OMcp21_251*
 ORcp21_152-OMcp21_252*ORcp21_177-OMcp21_26*ORcp21_131+OPcp21_133*RLcp21_234+OPcp21_134*RLcp21_244+OPcp21_146*RLcp21_247+
 OPcp21_147*RLcp21_248+OPcp21_148*RLcp21_249+OPcp21_149*RLcp21_250+OPcp21_151*RLcp21_252+OPcp21_152*RLcp21_277+OPcp21_16*
 RLcp21_231-OPcp21_233*RLcp21_134-OPcp21_234*RLcp21_144-OPcp21_246*RLcp21_147-OPcp21_247*RLcp21_148-OPcp21_248*RLcp21_149-
 OPcp21_249*RLcp21_150-OPcp21_251*RLcp21_152-OPcp21_252*RLcp21_177-OPcp21_26*RLcp21_131;

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp21_177;
    sens->P[2] = POcp21_277;
    sens->P[3] = POcp21_377;
    sens->R[1][1] = ROcp21_151;
    sens->R[1][2] = ROcp21_251;
    sens->R[1][3] = ROcp21_351;
    sens->R[2][1] = ROcp21_452;
    sens->R[2][2] = ROcp21_552;
    sens->R[2][3] = ROcp21_652;
    sens->R[3][1] = ROcp21_752;
    sens->R[3][2] = ROcp21_852;
    sens->R[3][3] = ROcp21_952;
    sens->V[1] = VIcp21_177;
    sens->V[2] = VIcp21_277;
    sens->V[3] = VIcp21_377;
    sens->OM[1] = OMcp21_152;
    sens->OM[2] = OMcp21_252;
    sens->OM[3] = OMcp21_352;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp21_177_5;
    sens->J[1][6] = JTcp21_177_6;
    sens->J[1][31] = JTcp21_177_7;
    sens->J[1][32] = JTcp21_177_8;
    sens->J[1][33] = JTcp21_177_9;
    sens->J[1][34] = JTcp21_177_10;
    sens->J[1][44] = JTcp21_177_11;
    sens->J[1][45] = JTcp21_177_12;
    sens->J[1][46] = JTcp21_177_13;
    sens->J[1][47] = JTcp21_177_14;
    sens->J[1][48] = JTcp21_177_15;
    sens->J[1][49] = JTcp21_177_16;
    sens->J[1][50] = JTcp21_177_17;
    sens->J[1][51] = JTcp21_177_18;
    sens->J[1][52] = JTcp21_177_19;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp21_277_4;
    sens->J[2][5] = JTcp21_277_5;
    sens->J[2][6] = JTcp21_277_6;
    sens->J[2][31] = JTcp21_277_7;
    sens->J[2][32] = JTcp21_277_8;
    sens->J[2][33] = JTcp21_277_9;
    sens->J[2][34] = JTcp21_277_10;
    sens->J[2][44] = JTcp21_277_11;
    sens->J[2][45] = JTcp21_277_12;
    sens->J[2][46] = JTcp21_277_13;
    sens->J[2][47] = JTcp21_277_14;
    sens->J[2][48] = JTcp21_277_15;
    sens->J[2][49] = JTcp21_277_16;
    sens->J[2][50] = JTcp21_277_17;
    sens->J[2][51] = JTcp21_277_18;
    sens->J[2][52] = JTcp21_277_19;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp21_377_4;
    sens->J[3][5] = JTcp21_377_5;
    sens->J[3][6] = JTcp21_377_6;
    sens->J[3][31] = JTcp21_377_7;
    sens->J[3][32] = JTcp21_377_8;
    sens->J[3][33] = JTcp21_377_9;
    sens->J[3][34] = JTcp21_377_10;
    sens->J[3][44] = JTcp21_377_11;
    sens->J[3][45] = JTcp21_377_12;
    sens->J[3][46] = JTcp21_377_13;
    sens->J[3][47] = JTcp21_377_14;
    sens->J[3][48] = JTcp21_377_15;
    sens->J[3][49] = JTcp21_377_16;
    sens->J[3][50] = JTcp21_377_17;
    sens->J[3][51] = JTcp21_377_18;
    sens->J[3][52] = JTcp21_377_19;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][31] = ROcp21_46;
    sens->J[4][32] = ROcp21_131;
    sens->J[4][33] = ROcp21_432;
    sens->J[4][34] = ROcp21_733;
    sens->J[4][44] = ROcp21_134;
    sens->J[4][45] = ROcp21_744;
    sens->J[4][46] = ROcp21_445;
    sens->J[4][47] = ROcp21_146;
    sens->J[4][48] = ROcp21_747;
    sens->J[4][49] = ROcp21_448;
    sens->J[4][50] = ROcp21_749;
    sens->J[4][51] = ROcp21_450;
    sens->J[4][52] = ROcp21_151;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp21_85;
    sens->J[5][31] = ROcp21_56;
    sens->J[5][32] = ROcp21_231;
    sens->J[5][33] = ROcp21_532;
    sens->J[5][34] = ROcp21_833;
    sens->J[5][44] = ROcp21_234;
    sens->J[5][45] = ROcp21_844;
    sens->J[5][46] = ROcp21_545;
    sens->J[5][47] = ROcp21_246;
    sens->J[5][48] = ROcp21_847;
    sens->J[5][49] = ROcp21_548;
    sens->J[5][50] = ROcp21_849;
    sens->J[5][51] = ROcp21_550;
    sens->J[5][52] = ROcp21_251;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp21_95;
    sens->J[6][31] = ROcp21_66;
    sens->J[6][32] = ROcp21_331;
    sens->J[6][33] = ROcp21_632;
    sens->J[6][34] = ROcp21_933;
    sens->J[6][44] = ROcp21_334;
    sens->J[6][45] = ROcp21_944;
    sens->J[6][46] = ROcp21_645;
    sens->J[6][47] = ROcp21_346;
    sens->J[6][48] = ROcp21_947;
    sens->J[6][49] = ROcp21_648;
    sens->J[6][50] = ROcp21_949;
    sens->J[6][51] = ROcp21_650;
    sens->J[6][52] = ROcp21_351;
    sens->A[1] = ACcp21_177;
    sens->A[2] = ACcp21_277;
    sens->A[3] = ACcp21_377;
    sens->OMP[1] = OPcp21_152;
    sens->OMP[2] = OPcp21_252;
    sens->OMP[3] = OPcp21_352;
 
// 
break;
case 23:
 


// = = Block_1_0_0_23_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp22_25 = qd[5]*C4;
    OMcp22_35 = qd[5]*S4;
    OMcp22_16 = qd[4]+qd[6]*S5;
    OMcp22_26 = OMcp22_25+ROcp22_85*qd[6];
    OMcp22_36 = OMcp22_35+ROcp22_95*qd[6];
    OPcp22_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp22_26 = ROcp22_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp22_35*S5-ROcp22_95*qd[4]);
    OPcp22_36 = ROcp22_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp22_25*S5-ROcp22_85*qd[4]);

// = = Block_1_0_0_23_0_2 = = 
 
// Sensor Kinematics 


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
    RLcp22_17 = ROcp22_16*s->dpt[1][1]+ROcp22_46*s->dpt[2][1];
    RLcp22_27 = ROcp22_26*s->dpt[1][1]+ROcp22_56*s->dpt[2][1];
    RLcp22_37 = ROcp22_36*s->dpt[1][1]+ROcp22_66*s->dpt[2][1];
    OMcp22_17 = OMcp22_16+ROcp22_16*qd[7];
    OMcp22_27 = OMcp22_26+ROcp22_26*qd[7];
    OMcp22_37 = OMcp22_36+ROcp22_36*qd[7];
    ORcp22_17 = OMcp22_26*RLcp22_37-OMcp22_36*RLcp22_27;
    ORcp22_27 = -(OMcp22_16*RLcp22_37-OMcp22_36*RLcp22_17);
    ORcp22_37 = OMcp22_16*RLcp22_27-OMcp22_26*RLcp22_17;
    OPcp22_17 = OPcp22_16+ROcp22_16*qdd[7]+qd[7]*(OMcp22_26*ROcp22_36-OMcp22_36*ROcp22_26);
    OPcp22_27 = OPcp22_26+ROcp22_26*qdd[7]-qd[7]*(OMcp22_16*ROcp22_36-OMcp22_36*ROcp22_16);
    OPcp22_37 = OPcp22_36+ROcp22_36*qdd[7]+qd[7]*(OMcp22_16*ROcp22_26-OMcp22_26*ROcp22_16);
    RLcp22_18 = ROcp22_16*s->dpt[1][5]+ROcp22_47*s->dpt[2][5]+ROcp22_77*s->dpt[3][5];
    RLcp22_28 = ROcp22_26*s->dpt[1][5]+ROcp22_57*s->dpt[2][5]+ROcp22_87*s->dpt[3][5];
    RLcp22_38 = ROcp22_36*s->dpt[1][5]+ROcp22_67*s->dpt[2][5]+ROcp22_97*s->dpt[3][5];
    OMcp22_18 = OMcp22_17+ROcp22_77*qd[8];
    OMcp22_28 = OMcp22_27+ROcp22_87*qd[8];
    OMcp22_38 = OMcp22_37+ROcp22_97*qd[8];
    ORcp22_18 = OMcp22_27*RLcp22_38-OMcp22_37*RLcp22_28;
    ORcp22_28 = -(OMcp22_17*RLcp22_38-OMcp22_37*RLcp22_18);
    ORcp22_38 = OMcp22_17*RLcp22_28-OMcp22_27*RLcp22_18;
    OMcp22_19 = OMcp22_18+ROcp22_48*qd[9];
    OMcp22_29 = OMcp22_28+ROcp22_58*qd[9];
    OMcp22_39 = OMcp22_38+ROcp22_68*qd[9];
    OPcp22_19 = OPcp22_17+ROcp22_48*qdd[9]+ROcp22_77*qdd[8]+qd[8]*(OMcp22_27*ROcp22_97-OMcp22_37*ROcp22_87)+qd[9]*(
 OMcp22_28*ROcp22_68-OMcp22_38*ROcp22_58);
    OPcp22_29 = OPcp22_27+ROcp22_58*qdd[9]+ROcp22_87*qdd[8]-qd[8]*(OMcp22_17*ROcp22_97-OMcp22_37*ROcp22_77)-qd[9]*(
 OMcp22_18*ROcp22_68-OMcp22_38*ROcp22_48);
    OPcp22_39 = OPcp22_37+ROcp22_68*qdd[9]+ROcp22_97*qdd[8]+qd[8]*(OMcp22_17*ROcp22_87-OMcp22_27*ROcp22_77)+qd[9]*(
 OMcp22_18*ROcp22_58-OMcp22_28*ROcp22_48);
    RLcp22_110 = ROcp22_79*s->dpt[3][7];
    RLcp22_210 = ROcp22_89*s->dpt[3][7];
    RLcp22_310 = ROcp22_99*s->dpt[3][7];
    OMcp22_110 = OMcp22_19+ROcp22_48*qd[10];
    OMcp22_210 = OMcp22_29+ROcp22_58*qd[10];
    OMcp22_310 = OMcp22_39+ROcp22_68*qd[10];
    ORcp22_110 = OMcp22_29*RLcp22_310-OMcp22_39*RLcp22_210;
    ORcp22_210 = -(OMcp22_19*RLcp22_310-OMcp22_39*RLcp22_110);
    ORcp22_310 = OMcp22_19*RLcp22_210-OMcp22_29*RLcp22_110;
    OPcp22_110 = OPcp22_19+ROcp22_48*qdd[10]+qd[10]*(OMcp22_29*ROcp22_68-OMcp22_39*ROcp22_58);
    OPcp22_210 = OPcp22_29+ROcp22_58*qdd[10]-qd[10]*(OMcp22_19*ROcp22_68-OMcp22_39*ROcp22_48);
    OPcp22_310 = OPcp22_39+ROcp22_68*qdd[10]+qd[10]*(OMcp22_19*ROcp22_58-OMcp22_29*ROcp22_48);
    RLcp22_111 = ROcp22_710*s->dpt[3][8];
    RLcp22_211 = ROcp22_810*s->dpt[3][8];
    RLcp22_311 = ROcp22_910*s->dpt[3][8];
    OMcp22_111 = OMcp22_110+ROcp22_48*qd[11];
    OMcp22_211 = OMcp22_210+ROcp22_58*qd[11];
    OMcp22_311 = OMcp22_310+ROcp22_68*qd[11];
    ORcp22_111 = OMcp22_210*RLcp22_311-OMcp22_310*RLcp22_211;
    ORcp22_211 = -(OMcp22_110*RLcp22_311-OMcp22_310*RLcp22_111);
    ORcp22_311 = OMcp22_110*RLcp22_211-OMcp22_210*RLcp22_111;
    OMcp22_112 = OMcp22_111+ROcp22_111*qd[12];
    OMcp22_212 = OMcp22_211+ROcp22_211*qd[12];
    OMcp22_312 = OMcp22_311+ROcp22_311*qd[12];
    OPcp22_112 = OPcp22_110+ROcp22_111*qdd[12]+ROcp22_48*qdd[11]+qd[11]*(OMcp22_210*ROcp22_68-OMcp22_310*ROcp22_58)+qd[12]
 *(OMcp22_211*ROcp22_311-OMcp22_311*ROcp22_211);
    OPcp22_212 = OPcp22_210+ROcp22_211*qdd[12]+ROcp22_58*qdd[11]-qd[11]*(OMcp22_110*ROcp22_68-OMcp22_310*ROcp22_48)-qd[12]
 *(OMcp22_111*ROcp22_311-OMcp22_311*ROcp22_111);
    OPcp22_312 = OPcp22_310+ROcp22_311*qdd[12]+ROcp22_68*qdd[11]+qd[11]*(OMcp22_110*ROcp22_58-OMcp22_210*ROcp22_48)+qd[12]
 *(OMcp22_111*ROcp22_211-OMcp22_211*ROcp22_111);
    RLcp22_113 = ROcp22_111*s->dpt[1][10]+ROcp22_712*s->dpt[3][10];
    RLcp22_213 = ROcp22_211*s->dpt[1][10]+ROcp22_812*s->dpt[3][10];
    RLcp22_313 = ROcp22_311*s->dpt[1][10]+ROcp22_912*s->dpt[3][10];
    ORcp22_113 = OMcp22_212*RLcp22_313-OMcp22_312*RLcp22_213;
    ORcp22_213 = -(OMcp22_112*RLcp22_313-OMcp22_312*RLcp22_113);
    ORcp22_313 = OMcp22_112*RLcp22_213-OMcp22_212*RLcp22_113;
    RLcp22_116 = ROcp22_715*q[16];
    RLcp22_216 = ROcp22_815*q[16];
    RLcp22_316 = ROcp22_915*q[16];
    ORcp22_116 = OMcp22_212*RLcp22_316-OMcp22_312*RLcp22_216;
    ORcp22_216 = -(OMcp22_112*RLcp22_316-OMcp22_312*RLcp22_116);
    ORcp22_316 = OMcp22_112*RLcp22_216-OMcp22_212*RLcp22_116;
    RLcp22_117 = ROcp22_415*q[17];
    RLcp22_217 = ROcp22_515*q[17];
    RLcp22_317 = ROcp22_615*q[17];
    ORcp22_117 = OMcp22_212*RLcp22_317-OMcp22_312*RLcp22_217;
    ORcp22_217 = -(OMcp22_112*RLcp22_317-OMcp22_312*RLcp22_117);
    ORcp22_317 = OMcp22_112*RLcp22_217-OMcp22_212*RLcp22_117;
    RLcp22_118 = ROcp22_114*q[18];
    RLcp22_218 = ROcp22_214*q[18];
    RLcp22_318 = ROcp22_314*q[18];
    ORcp22_118 = OMcp22_212*RLcp22_318-OMcp22_312*RLcp22_218;
    ORcp22_218 = -(OMcp22_112*RLcp22_318-OMcp22_312*RLcp22_118);
    ORcp22_318 = OMcp22_112*RLcp22_218-OMcp22_212*RLcp22_118;
    RLcp22_178 = ROcp22_715*s->dpt[3][11];
    RLcp22_278 = ROcp22_815*s->dpt[3][11];
    RLcp22_378 = ROcp22_915*s->dpt[3][11];
    POcp22_178 = RLcp22_110+RLcp22_111+RLcp22_113+RLcp22_116+RLcp22_117+RLcp22_118+RLcp22_17+RLcp22_178+RLcp22_18+q[1];
    POcp22_278 = RLcp22_210+RLcp22_211+RLcp22_213+RLcp22_216+RLcp22_217+RLcp22_218+RLcp22_27+RLcp22_278+RLcp22_28+q[2];
    POcp22_378 = RLcp22_310+RLcp22_311+RLcp22_313+RLcp22_316+RLcp22_317+RLcp22_318+RLcp22_37+RLcp22_378+RLcp22_38+q[3];
    ORcp22_178 = OMcp22_212*RLcp22_378-OMcp22_312*RLcp22_278;
    ORcp22_278 = -(OMcp22_112*RLcp22_378-OMcp22_312*RLcp22_178);
    ORcp22_378 = OMcp22_112*RLcp22_278-OMcp22_212*RLcp22_178;
    VIcp22_178 = ORcp22_110+ORcp22_111+ORcp22_113+ORcp22_116+ORcp22_117+ORcp22_118+ORcp22_17+ORcp22_178+ORcp22_18+qd[1];
    VIcp22_278 = ORcp22_210+ORcp22_211+ORcp22_213+ORcp22_216+ORcp22_217+ORcp22_218+ORcp22_27+ORcp22_278+ORcp22_28+qd[2];
    VIcp22_378 = ORcp22_310+ORcp22_311+ORcp22_313+ORcp22_316+ORcp22_317+ORcp22_318+ORcp22_37+ORcp22_378+ORcp22_38+qd[3];
    ACcp22_178 = qdd[1]+OMcp22_210*ORcp22_311+OMcp22_212*ORcp22_313+OMcp22_212*ORcp22_316+OMcp22_212*ORcp22_317+OMcp22_212
 *ORcp22_318+OMcp22_212*ORcp22_378+OMcp22_26*ORcp22_37+OMcp22_27*ORcp22_38+OMcp22_29*ORcp22_310-OMcp22_310*ORcp22_211-
 OMcp22_312*ORcp22_213-OMcp22_312*ORcp22_216-OMcp22_312*ORcp22_217-OMcp22_312*ORcp22_218-OMcp22_312*ORcp22_278-OMcp22_36*
 ORcp22_27-OMcp22_37*ORcp22_28-OMcp22_39*ORcp22_210+OPcp22_210*RLcp22_311+OPcp22_212*RLcp22_313+OPcp22_212*RLcp22_316+
 OPcp22_212*RLcp22_317+OPcp22_212*RLcp22_318+OPcp22_212*RLcp22_378+OPcp22_26*RLcp22_37+OPcp22_27*RLcp22_38+OPcp22_29*
 RLcp22_310-OPcp22_310*RLcp22_211-OPcp22_312*RLcp22_213-OPcp22_312*RLcp22_216-OPcp22_312*RLcp22_217-OPcp22_312*RLcp22_218-
 OPcp22_312*RLcp22_278-OPcp22_36*RLcp22_27-OPcp22_37*RLcp22_28-OPcp22_39*RLcp22_210;
    ACcp22_278 = qdd[2]-OMcp22_110*ORcp22_311-OMcp22_112*ORcp22_313-OMcp22_112*ORcp22_316-OMcp22_112*ORcp22_317-OMcp22_112
 *ORcp22_318-OMcp22_112*ORcp22_378-OMcp22_16*ORcp22_37-OMcp22_17*ORcp22_38-OMcp22_19*ORcp22_310+OMcp22_310*ORcp22_111+
 OMcp22_312*ORcp22_113+OMcp22_312*ORcp22_116+OMcp22_312*ORcp22_117+OMcp22_312*ORcp22_118+OMcp22_312*ORcp22_178+OMcp22_36*
 ORcp22_17+OMcp22_37*ORcp22_18+OMcp22_39*ORcp22_110-OPcp22_110*RLcp22_311-OPcp22_112*RLcp22_313-OPcp22_112*RLcp22_316-
 OPcp22_112*RLcp22_317-OPcp22_112*RLcp22_318-OPcp22_112*RLcp22_378-OPcp22_16*RLcp22_37-OPcp22_17*RLcp22_38-OPcp22_19*
 RLcp22_310+OPcp22_310*RLcp22_111+OPcp22_312*RLcp22_113+OPcp22_312*RLcp22_116+OPcp22_312*RLcp22_117+OPcp22_312*RLcp22_118+
 OPcp22_312*RLcp22_178+OPcp22_36*RLcp22_17+OPcp22_37*RLcp22_18+OPcp22_39*RLcp22_110;
    ACcp22_378 = qdd[3]+OMcp22_110*ORcp22_211+OMcp22_112*ORcp22_213+OMcp22_112*ORcp22_216+OMcp22_112*ORcp22_217+OMcp22_112
 *ORcp22_218+OMcp22_112*ORcp22_278+OMcp22_16*ORcp22_27+OMcp22_17*ORcp22_28+OMcp22_19*ORcp22_210-OMcp22_210*ORcp22_111-
 OMcp22_212*ORcp22_113-OMcp22_212*ORcp22_116-OMcp22_212*ORcp22_117-OMcp22_212*ORcp22_118-OMcp22_212*ORcp22_178-OMcp22_26*
 ORcp22_17-OMcp22_27*ORcp22_18-OMcp22_29*ORcp22_110+OPcp22_110*RLcp22_211+OPcp22_112*RLcp22_213+OPcp22_112*RLcp22_216+
 OPcp22_112*RLcp22_217+OPcp22_112*RLcp22_218+OPcp22_112*RLcp22_278+OPcp22_16*RLcp22_27+OPcp22_17*RLcp22_28+OPcp22_19*
 RLcp22_210-OPcp22_210*RLcp22_111-OPcp22_212*RLcp22_113-OPcp22_212*RLcp22_116-OPcp22_212*RLcp22_117-OPcp22_212*RLcp22_118-
 OPcp22_212*RLcp22_178-OPcp22_26*RLcp22_17-OPcp22_27*RLcp22_18-OPcp22_29*RLcp22_110;

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp22_178;
    sens->P[2] = POcp22_278;
    sens->P[3] = POcp22_378;
    sens->R[1][1] = ROcp22_114;
    sens->R[1][2] = ROcp22_214;
    sens->R[1][3] = ROcp22_314;
    sens->R[2][1] = ROcp22_415;
    sens->R[2][2] = ROcp22_515;
    sens->R[2][3] = ROcp22_615;
    sens->R[3][1] = ROcp22_715;
    sens->R[3][2] = ROcp22_815;
    sens->R[3][3] = ROcp22_915;
    sens->V[1] = VIcp22_178;
    sens->V[2] = VIcp22_278;
    sens->V[3] = VIcp22_378;
    sens->OM[1] = OMcp22_112;
    sens->OM[2] = OMcp22_212;
    sens->OM[3] = OMcp22_312;
    sens->A[1] = ACcp22_178;
    sens->A[2] = ACcp22_278;
    sens->A[3] = ACcp22_378;
    sens->OMP[1] = OPcp22_112;
    sens->OMP[2] = OPcp22_212;
    sens->OMP[3] = OPcp22_312;
 
// 
break;
case 24:
 


// = = Block_1_0_0_24_0_1 = = 
 
// Sensor Kinematics 


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
    OMcp23_25 = qd[5]*C4;
    OMcp23_35 = qd[5]*S4;
    OMcp23_16 = qd[4]+qd[6]*S5;
    OMcp23_26 = OMcp23_25+ROcp23_85*qd[6];
    OMcp23_36 = OMcp23_35+ROcp23_95*qd[6];
    OPcp23_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp23_26 = ROcp23_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp23_35*S5-ROcp23_95*qd[4]);
    OPcp23_36 = ROcp23_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp23_25*S5-ROcp23_85*qd[4]);

// = = Block_1_0_0_24_0_3 = = 
 
// Sensor Kinematics 


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
    RLcp23_119 = ROcp23_16*s->dpt[1][2]+ROcp23_46*s->dpt[2][2];
    RLcp23_219 = ROcp23_26*s->dpt[1][2]+ROcp23_56*s->dpt[2][2];
    RLcp23_319 = ROcp23_36*s->dpt[1][2]+ROcp23_66*s->dpt[2][2];
    OMcp23_119 = OMcp23_16+ROcp23_16*qd[19];
    OMcp23_219 = OMcp23_26+ROcp23_26*qd[19];
    OMcp23_319 = OMcp23_36+ROcp23_36*qd[19];
    ORcp23_119 = OMcp23_26*RLcp23_319-OMcp23_36*RLcp23_219;
    ORcp23_219 = -(OMcp23_16*RLcp23_319-OMcp23_36*RLcp23_119);
    ORcp23_319 = OMcp23_16*RLcp23_219-OMcp23_26*RLcp23_119;
    OPcp23_119 = OPcp23_16+ROcp23_16*qdd[19]+qd[19]*(OMcp23_26*ROcp23_36-OMcp23_36*ROcp23_26);
    OPcp23_219 = OPcp23_26+ROcp23_26*qdd[19]-qd[19]*(OMcp23_16*ROcp23_36-OMcp23_36*ROcp23_16);
    OPcp23_319 = OPcp23_36+ROcp23_36*qdd[19]+qd[19]*(OMcp23_16*ROcp23_26-OMcp23_26*ROcp23_16);
    RLcp23_120 = ROcp23_16*s->dpt[1][12]+ROcp23_419*s->dpt[2][12]+ROcp23_719*s->dpt[3][12];
    RLcp23_220 = ROcp23_26*s->dpt[1][12]+ROcp23_519*s->dpt[2][12]+ROcp23_819*s->dpt[3][12];
    RLcp23_320 = ROcp23_36*s->dpt[1][12]+ROcp23_619*s->dpt[2][12]+ROcp23_919*s->dpt[3][12];
    OMcp23_120 = OMcp23_119+ROcp23_719*qd[20];
    OMcp23_220 = OMcp23_219+ROcp23_819*qd[20];
    OMcp23_320 = OMcp23_319+ROcp23_919*qd[20];
    ORcp23_120 = OMcp23_219*RLcp23_320-OMcp23_319*RLcp23_220;
    ORcp23_220 = -(OMcp23_119*RLcp23_320-OMcp23_319*RLcp23_120);
    ORcp23_320 = OMcp23_119*RLcp23_220-OMcp23_219*RLcp23_120;
    OMcp23_121 = OMcp23_120+ROcp23_420*qd[21];
    OMcp23_221 = OMcp23_220+ROcp23_520*qd[21];
    OMcp23_321 = OMcp23_320+ROcp23_620*qd[21];
    OPcp23_121 = OPcp23_119+ROcp23_420*qdd[21]+ROcp23_719*qdd[20]+qd[20]*(OMcp23_219*ROcp23_919-OMcp23_319*ROcp23_819)+
 qd[21]*(OMcp23_220*ROcp23_620-OMcp23_320*ROcp23_520);
    OPcp23_221 = OPcp23_219+ROcp23_520*qdd[21]+ROcp23_819*qdd[20]-qd[20]*(OMcp23_119*ROcp23_919-OMcp23_319*ROcp23_719)-
 qd[21]*(OMcp23_120*ROcp23_620-OMcp23_320*ROcp23_420);
    OPcp23_321 = OPcp23_319+ROcp23_620*qdd[21]+ROcp23_919*qdd[20]+qd[20]*(OMcp23_119*ROcp23_819-OMcp23_219*ROcp23_719)+
 qd[21]*(OMcp23_120*ROcp23_520-OMcp23_220*ROcp23_420);
    RLcp23_122 = ROcp23_721*s->dpt[3][14];
    RLcp23_222 = ROcp23_821*s->dpt[3][14];
    RLcp23_322 = ROcp23_921*s->dpt[3][14];
    OMcp23_122 = OMcp23_121+ROcp23_420*qd[22];
    OMcp23_222 = OMcp23_221+ROcp23_520*qd[22];
    OMcp23_322 = OMcp23_321+ROcp23_620*qd[22];
    ORcp23_122 = OMcp23_221*RLcp23_322-OMcp23_321*RLcp23_222;
    ORcp23_222 = -(OMcp23_121*RLcp23_322-OMcp23_321*RLcp23_122);
    ORcp23_322 = OMcp23_121*RLcp23_222-OMcp23_221*RLcp23_122;
    OPcp23_122 = OPcp23_121+ROcp23_420*qdd[22]+qd[22]*(OMcp23_221*ROcp23_620-OMcp23_321*ROcp23_520);
    OPcp23_222 = OPcp23_221+ROcp23_520*qdd[22]-qd[22]*(OMcp23_121*ROcp23_620-OMcp23_321*ROcp23_420);
    OPcp23_322 = OPcp23_321+ROcp23_620*qdd[22]+qd[22]*(OMcp23_121*ROcp23_520-OMcp23_221*ROcp23_420);
    RLcp23_123 = ROcp23_722*s->dpt[3][15];
    RLcp23_223 = ROcp23_822*s->dpt[3][15];
    RLcp23_323 = ROcp23_922*s->dpt[3][15];
    OMcp23_123 = OMcp23_122+ROcp23_420*qd[23];
    OMcp23_223 = OMcp23_222+ROcp23_520*qd[23];
    OMcp23_323 = OMcp23_322+ROcp23_620*qd[23];
    ORcp23_123 = OMcp23_222*RLcp23_323-OMcp23_322*RLcp23_223;
    ORcp23_223 = -(OMcp23_122*RLcp23_323-OMcp23_322*RLcp23_123);
    ORcp23_323 = OMcp23_122*RLcp23_223-OMcp23_222*RLcp23_123;
    OMcp23_124 = OMcp23_123+ROcp23_123*qd[24];
    OMcp23_224 = OMcp23_223+ROcp23_223*qd[24];
    OMcp23_324 = OMcp23_323+ROcp23_323*qd[24];
    OPcp23_124 = OPcp23_122+ROcp23_123*qdd[24]+ROcp23_420*qdd[23]+qd[23]*(OMcp23_222*ROcp23_620-OMcp23_322*ROcp23_520)+
 qd[24]*(OMcp23_223*ROcp23_323-OMcp23_323*ROcp23_223);
    OPcp23_224 = OPcp23_222+ROcp23_223*qdd[24]+ROcp23_520*qdd[23]-qd[23]*(OMcp23_122*ROcp23_620-OMcp23_322*ROcp23_420)-
 qd[24]*(OMcp23_123*ROcp23_323-OMcp23_323*ROcp23_123);
    OPcp23_324 = OPcp23_322+ROcp23_323*qdd[24]+ROcp23_620*qdd[23]+qd[23]*(OMcp23_122*ROcp23_520-OMcp23_222*ROcp23_420)+
 qd[24]*(OMcp23_123*ROcp23_223-OMcp23_223*ROcp23_123);
    RLcp23_125 = ROcp23_123*s->dpt[1][17]+ROcp23_724*s->dpt[3][17];
    RLcp23_225 = ROcp23_223*s->dpt[1][17]+ROcp23_824*s->dpt[3][17];
    RLcp23_325 = ROcp23_323*s->dpt[1][17]+ROcp23_924*s->dpt[3][17];
    ORcp23_125 = OMcp23_224*RLcp23_325-OMcp23_324*RLcp23_225;
    ORcp23_225 = -(OMcp23_124*RLcp23_325-OMcp23_324*RLcp23_125);
    ORcp23_325 = OMcp23_124*RLcp23_225-OMcp23_224*RLcp23_125;
    RLcp23_128 = ROcp23_727*q[28];
    RLcp23_228 = ROcp23_827*q[28];
    RLcp23_328 = ROcp23_927*q[28];
    ORcp23_128 = OMcp23_224*RLcp23_328-OMcp23_324*RLcp23_228;
    ORcp23_228 = -(OMcp23_124*RLcp23_328-OMcp23_324*RLcp23_128);
    ORcp23_328 = OMcp23_124*RLcp23_228-OMcp23_224*RLcp23_128;
    RLcp23_129 = ROcp23_427*q[29];
    RLcp23_229 = ROcp23_527*q[29];
    RLcp23_329 = ROcp23_627*q[29];
    ORcp23_129 = OMcp23_224*RLcp23_329-OMcp23_324*RLcp23_229;
    ORcp23_229 = -(OMcp23_124*RLcp23_329-OMcp23_324*RLcp23_129);
    ORcp23_329 = OMcp23_124*RLcp23_229-OMcp23_224*RLcp23_129;
    RLcp23_130 = ROcp23_126*q[30];
    RLcp23_230 = ROcp23_226*q[30];
    RLcp23_330 = ROcp23_326*q[30];
    ORcp23_130 = OMcp23_224*RLcp23_330-OMcp23_324*RLcp23_230;
    ORcp23_230 = -(OMcp23_124*RLcp23_330-OMcp23_324*RLcp23_130);
    ORcp23_330 = OMcp23_124*RLcp23_230-OMcp23_224*RLcp23_130;
    RLcp23_179 = ROcp23_727*s->dpt[3][18];
    RLcp23_279 = ROcp23_827*s->dpt[3][18];
    RLcp23_379 = ROcp23_927*s->dpt[3][18];
    POcp23_179 = RLcp23_119+RLcp23_120+RLcp23_122+RLcp23_123+RLcp23_125+RLcp23_128+RLcp23_129+RLcp23_130+RLcp23_179+q[1];
    POcp23_279 = RLcp23_219+RLcp23_220+RLcp23_222+RLcp23_223+RLcp23_225+RLcp23_228+RLcp23_229+RLcp23_230+RLcp23_279+q[2];
    POcp23_379 = RLcp23_319+RLcp23_320+RLcp23_322+RLcp23_323+RLcp23_325+RLcp23_328+RLcp23_329+RLcp23_330+RLcp23_379+q[3];
    ORcp23_179 = OMcp23_224*RLcp23_379-OMcp23_324*RLcp23_279;
    ORcp23_279 = -(OMcp23_124*RLcp23_379-OMcp23_324*RLcp23_179);
    ORcp23_379 = OMcp23_124*RLcp23_279-OMcp23_224*RLcp23_179;
    VIcp23_179 = ORcp23_119+ORcp23_120+ORcp23_122+ORcp23_123+ORcp23_125+ORcp23_128+ORcp23_129+ORcp23_130+ORcp23_179+qd[1];
    VIcp23_279 = ORcp23_219+ORcp23_220+ORcp23_222+ORcp23_223+ORcp23_225+ORcp23_228+ORcp23_229+ORcp23_230+ORcp23_279+qd[2];
    VIcp23_379 = ORcp23_319+ORcp23_320+ORcp23_322+ORcp23_323+ORcp23_325+ORcp23_328+ORcp23_329+ORcp23_330+ORcp23_379+qd[3];
    ACcp23_179 = qdd[1]+OMcp23_219*ORcp23_320+OMcp23_221*ORcp23_322+OMcp23_222*ORcp23_323+OMcp23_224*ORcp23_325+OMcp23_224
 *ORcp23_328+OMcp23_224*ORcp23_329+OMcp23_224*ORcp23_330+OMcp23_224*ORcp23_379+OMcp23_26*ORcp23_319-OMcp23_319*ORcp23_220-
 OMcp23_321*ORcp23_222-OMcp23_322*ORcp23_223-OMcp23_324*ORcp23_225-OMcp23_324*ORcp23_228-OMcp23_324*ORcp23_229-OMcp23_324*
 ORcp23_230-OMcp23_324*ORcp23_279-OMcp23_36*ORcp23_219+OPcp23_219*RLcp23_320+OPcp23_221*RLcp23_322+OPcp23_222*RLcp23_323+
 OPcp23_224*RLcp23_325+OPcp23_224*RLcp23_328+OPcp23_224*RLcp23_329+OPcp23_224*RLcp23_330+OPcp23_224*RLcp23_379+OPcp23_26*
 RLcp23_319-OPcp23_319*RLcp23_220-OPcp23_321*RLcp23_222-OPcp23_322*RLcp23_223-OPcp23_324*RLcp23_225-OPcp23_324*RLcp23_228-
 OPcp23_324*RLcp23_229-OPcp23_324*RLcp23_230-OPcp23_324*RLcp23_279-OPcp23_36*RLcp23_219;
    ACcp23_279 = qdd[2]-OMcp23_119*ORcp23_320-OMcp23_121*ORcp23_322-OMcp23_122*ORcp23_323-OMcp23_124*ORcp23_325-OMcp23_124
 *ORcp23_328-OMcp23_124*ORcp23_329-OMcp23_124*ORcp23_330-OMcp23_124*ORcp23_379-OMcp23_16*ORcp23_319+OMcp23_319*ORcp23_120+
 OMcp23_321*ORcp23_122+OMcp23_322*ORcp23_123+OMcp23_324*ORcp23_125+OMcp23_324*ORcp23_128+OMcp23_324*ORcp23_129+OMcp23_324*
 ORcp23_130+OMcp23_324*ORcp23_179+OMcp23_36*ORcp23_119-OPcp23_119*RLcp23_320-OPcp23_121*RLcp23_322-OPcp23_122*RLcp23_323-
 OPcp23_124*RLcp23_325-OPcp23_124*RLcp23_328-OPcp23_124*RLcp23_329-OPcp23_124*RLcp23_330-OPcp23_124*RLcp23_379-OPcp23_16*
 RLcp23_319+OPcp23_319*RLcp23_120+OPcp23_321*RLcp23_122+OPcp23_322*RLcp23_123+OPcp23_324*RLcp23_125+OPcp23_324*RLcp23_128+
 OPcp23_324*RLcp23_129+OPcp23_324*RLcp23_130+OPcp23_324*RLcp23_179+OPcp23_36*RLcp23_119;
    ACcp23_379 = qdd[3]+OMcp23_119*ORcp23_220+OMcp23_121*ORcp23_222+OMcp23_122*ORcp23_223+OMcp23_124*ORcp23_225+OMcp23_124
 *ORcp23_228+OMcp23_124*ORcp23_229+OMcp23_124*ORcp23_230+OMcp23_124*ORcp23_279+OMcp23_16*ORcp23_219-OMcp23_219*ORcp23_120-
 OMcp23_221*ORcp23_122-OMcp23_222*ORcp23_123-OMcp23_224*ORcp23_125-OMcp23_224*ORcp23_128-OMcp23_224*ORcp23_129-OMcp23_224*
 ORcp23_130-OMcp23_224*ORcp23_179-OMcp23_26*ORcp23_119+OPcp23_119*RLcp23_220+OPcp23_121*RLcp23_222+OPcp23_122*RLcp23_223+
 OPcp23_124*RLcp23_225+OPcp23_124*RLcp23_228+OPcp23_124*RLcp23_229+OPcp23_124*RLcp23_230+OPcp23_124*RLcp23_279+OPcp23_16*
 RLcp23_219-OPcp23_219*RLcp23_120-OPcp23_221*RLcp23_122-OPcp23_222*RLcp23_123-OPcp23_224*RLcp23_125-OPcp23_224*RLcp23_128-
 OPcp23_224*RLcp23_129-OPcp23_224*RLcp23_130-OPcp23_224*RLcp23_179-OPcp23_26*RLcp23_119;

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp23_179;
    sens->P[2] = POcp23_279;
    sens->P[3] = POcp23_379;
    sens->R[1][1] = ROcp23_126;
    sens->R[1][2] = ROcp23_226;
    sens->R[1][3] = ROcp23_326;
    sens->R[2][1] = ROcp23_427;
    sens->R[2][2] = ROcp23_527;
    sens->R[2][3] = ROcp23_627;
    sens->R[3][1] = ROcp23_727;
    sens->R[3][2] = ROcp23_827;
    sens->R[3][3] = ROcp23_927;
    sens->V[1] = VIcp23_179;
    sens->V[2] = VIcp23_279;
    sens->V[3] = VIcp23_379;
    sens->OM[1] = OMcp23_124;
    sens->OM[2] = OMcp23_224;
    sens->OM[3] = OMcp23_324;
    sens->A[1] = ACcp23_179;
    sens->A[2] = ACcp23_279;
    sens->A[3] = ACcp23_379;
    sens->OMP[1] = OPcp23_124;
    sens->OMP[2] = OPcp23_224;
    sens->OMP[3] = OPcp23_324;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

