#ifndef GLOBAL_H
#define GLOBAL_H


#define PI 3.14159265f
#define JNT_NUM 7

extern float g_fSampleTime_OBC;									/*-- 数据插值时间间隔 --*/
extern float g_afJntRateMaxDeg_OBC;								/*-- 关节角速度限制 -- */
extern float gOut_fVisEndErr_OBC[6];									/*-- 当前手眼视觉伺服误差（笛卡尔空间） --*/
extern float g_afEndRotSpdMax_OBC;					/*-- 末端运动角速度限制 --*/
extern float g_afEndVelMax_OBC;									/*--  末端最大运动速度40mm/s*/
extern float g_afEndZPosErrDisLimit_OBC;								/*视觉伺服Z向测量距离限制*/
extern float g_afVisKp_OBC[6];					/*-- 视觉伺服控制比例系数 --*/
extern float g_fCapRunTime_OBC;									/*-- 进入视觉伺服控制的执行时间 --*/
extern float g_afStartTime_OBC;
extern float g_afVisThreshold_OBC[6];
extern float g_afDesiredBethingPose_OBC[6];
extern float g_EndHisStepPara[3];
/*---------函数使用的全局变量-----------*/
extern float g_EndHisStep1[6];									 /*上一次的末端运动步长*/
extern float g_EndHisStep2 [6];									 /* 上二次的末端运动步长*/
extern float g_EndHisStep3 [6];									 /* 上三次的末端运动步长*/
extern float g_afZInnerVisThreshold_OBC;								/*视觉伺服Z向内侧抓捕范围*/
extern float D_H[JNT_NUM][4];
extern float JointAngmax[JNT_NUM];
extern float JointAngmin[JNT_NUM];

int Rbt_InvMtrx( float *C, float *IC, int n );
void Rbt_PInvMtrx67( float AA[6][7], float AA_pinv[7][6]);
void Rbt_Cross(float u[], float v[], float n[]);
void Rbt_CalJcb(float DH[JNT_NUM][4], float JointAngle[JNT_NUM],float dRbtJcb[6][JNT_NUM],float T0n_c[4][4]);
void Rbt_MulMtrx(int m, int n, int p, float *A, float *B, float *C);
int Rbt_ikineItera(float DH[JNT_NUM][4], float T0n[4][4], float JntCurrent[JNT_NUM], float JntCalcted[JNT_NUM]);
void Rbt_fkine(float JointAngle[JNT_NUM], float DH[JNT_NUM][4], float T0n[4][4]);
void nfZyxEuler(float Euler_zyx[6],float TransMtrx[4][4]);
int nfVisualServoPlanPBVS(float fTempMatrix4[][4], float CurrentJntAngle[], float CurrentJntRate[],
						  float fJntAngleDesired[], float fJntRateDesired[], float fJntAccDesired[]);
int nfremotecontrol(float fPosedeltaX[6], float CurrentJntAngle[JNT_NUM],float fJntAngleDesired[JNT_NUM]);
#endif
