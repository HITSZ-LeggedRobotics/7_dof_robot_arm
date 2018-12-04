#include "global.h"

float g_fSampleTime_OBC=0.25f;									/*-- 数据插值时间间隔 --*/
float g_afJntRateMaxDeg_OBC= 10.0f;								/*-- 关节角速度限制 -- */
float gOut_fVisEndErr_OBC[6];									/*-- 当前手眼视觉伺服误差（笛卡尔空间） --*/
float g_afEndRotSpdMax_OBC = 0.08726646259972f;					/*-- 末端运动角速度限制 --*/
float g_afEndVelMax_OBC =0.05f;									/*--  末端最大运动速度40mm/s*/
float g_afEndZPosErrDisLimit_OBC = 0.01f;								/*视觉伺服Z向测量距离限制*/
float g_afVisKp_OBC[6]= {0.1f, 0.1f, 0.1f, 0.01f, 0.01f, 0.01f };					/*-- 视觉伺服控制比例系数 --*/
float g_fCapRunTime_OBC = 0.0f;									/*-- 进入视觉伺服控制的执行时间 --*/
float g_afStartTime_OBC = 3.0f;									/*-- 捕获用起始时间 --*/
//float g_afVisThreshold_OBC[6] = {0.035f, 0.035f, 0.04f, 0.087266f, 0.087266f,0.087266f};/*-- 视觉伺服抓捕范围 --*/
float g_afVisThreshold_OBC[6] = {1e-4f, 1e-4f, 1e-4f, 1e-4f, 1e-4f,1e-4f};/*-- 视觉伺服抓捕范围 --*/
float g_afDesiredBethingPose_OBC[6]= {0.0f, 0, 0, 0, 0, 0};		/*-- 手爪抓捕目标相对位姿偏置 --*/
float g_EndHisStepPara[3]={0.0f,0.5f,1.0f};                           /*视觉测量时延补偿系数*/
/* 函数使用的全局变量*/
float g_EndHisStep1[6]={ 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};				/*上一次的末端运动步长*/
float g_EndHisStep2 [6]={ 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};				/*上二次的末端运动步长*/
float g_EndHisStep3 [6]={ 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};				/*上三次的末端运动步长*/
float g_afZInnerVisThreshold_OBC = 0.06f;								/*视觉伺服Z向内侧抓捕范围*/
float D_H[JNT_NUM][4] =	{
	{0,		PI/2,		0,		323.5*0.001},
	{PI,		PI/2,		0,		0},
	{PI,		PI/2,		0,	    316*0.001},
	{PI,		PI/2,		0,		0},
    {PI,		PI/2,		0,		(284.5)*0.001},
	{PI,		PI/2,		0,		0},
	{0,			0,			0,		201*0.001}
};
float JointAngmax[JNT_NUM]={180*PI/180, 90*PI/180, 180*PI/180, 90*PI/180, 180*PI/180, 90*PI/180, 180*PI/180};
float JointAngmin[JNT_NUM]={-180*PI/180, -90*PI/180, -180*PI/180,  -90*PI/180,  -180*PI/180, -90*PI/180, -180*PI/180}; 
