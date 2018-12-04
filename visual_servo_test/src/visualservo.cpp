// visualservo.cpp : 定义控制台应用程序的入口点。
//

//#include "stdio.h"
#include "global.h"
#include "math.h"
void Rbt_Tr2EulerZyx(float dTr[3][3], float dEulerZyx[3])
{
	if( ( fabs( dTr[2][1] ) < 1e-12 )
	  &&( fabs( dTr[2][2] ) < 1e-12 ) )
	{
		dEulerZyx[2] = 0.0;
		dEulerZyx[1] = (float)(atan2 ( -dTr[2][0], dTr[2][2] ));
		dEulerZyx[0] = (float)(atan2 ( -dTr[0][1], dTr[1][1] ));
	}
	else
	{
		dEulerZyx[2] = (float)(atan2( dTr[2][1], dTr[2][2] ));
		dEulerZyx[1] = (float)(atan2( -dTr[2][0], ( cos( dEulerZyx[2] ) * dTr[2][2] + sin( dEulerZyx[2] ) * dTr[2][1] ) ));
		dEulerZyx[0] = (float)(atan2( dTr[1][0], dTr[0][0] ));
	}
}
//void Rbt_Tr2EulerZyx(float dTr[3][3], float dEulerZyx[3])
//{
//	int i;
//	float dEulerZyx1[3],dEulerZyx2[3];
//	if ( fabs( dTr[2][0] - 1 ) < 1e-12 )
//	{
//		dEulerZyx[2] = (float)(atan2 ( -dTr[1][2], dTr[1][1] ));
//		dEulerZyx[1] = -PI/2;
//		dEulerZyx[0] = 0.0;
//	}
//    else
//	{
//
//        if ( fabs( dTr[2][0] + 1 ) < 1e-12 )
//		{
//			dEulerZyx[2] = (float)(atan2 ( -dTr[1][2], dTr[1][1] ));
//			dEulerZyx[1] = PI/2;
//			dEulerZyx[0] = 0.0;
//		}
//        else
//		{
//
//			dEulerZyx1[1] = (float)(asin ( -dTr[2][0]));
//			dEulerZyx1[2] = (float)(atan2 ( dTr[2][1]*cos(dEulerZyx[1]), dTr[2][2]*cos(dEulerZyx[1]) ));
//			dEulerZyx1[0] = (float)(atan2 ( dTr[1][0]*cos(dEulerZyx[1]), dTr[0][0]*cos(dEulerZyx[1]) ));
//
//            dEulerZyx2[1] = PI-(float)(asin ( -dTr[2][0]));
//			dEulerZyx2[2] = (float)(atan2 ( dTr[2][1]*cos(dEulerZyx[1]), dTr[2][2]*cos(dEulerZyx[1]) ));
//			dEulerZyx2[0] = (float)(atan2 ( dTr[1][0]*cos(dEulerZyx[1]), dTr[0][0]*cos(dEulerZyx[1]) ));
//
//			//if(fabs(dEulerZyx1[0])+fabs(dEulerZyx1[1])+fabs(dEulerZyx1[2])>fabs(dEulerZyx2[0])+fabs(dEulerZyx2[1])+fabs(dEulerZyx2[2]))
//				for(i=1;i<3;i++)
//				{
//				  dEulerZyx[i]= dEulerZyx2[i];
//				}
//			//else
//				//for(i=1;i<3;i++)
//				//{
//				//  dEulerZyx[i]= dEulerZyx1[i];
//				//}
//
//		}
//
//	}
//}

/*矩阵求逆计算函数*/
int Rbt_InvMtrx( float *C, float *IC, int n )
{
	int i, j, k, l;
	
	/* 单位阵*/
	for (i=0; i<n; i++)
	{
		for (j=0; j<n; j++) 
		{	
			*(IC+i*n+j) = 0.0;
		}
		*(IC+i*n+i) = 1.0;
	}
	
	/* 化上三角阵*/
	for (j=0; j<n; j++)
	{	
		if(fabs(*(C+j*n+j))>1e-15) /* C[j][j]不等于0*/
		{
			/* IC阵的第j行除以C[j][j]*/
			for(k=0; k<n; k++)
			{
				*(IC+j*n+k) /= *(C+j*n+j);
			}
			/* C阵的第j行除以C[j][j]*/
			for(k=n-1; k>=j; k--)
			{
				*(C+j*n+k) /= *(C+j*n+j);
			}
			
			for(i=j+1; i<n; i++)
			{
				/* IC阵的第i行 - C[i][j]*IC阵的第j行*/
				for(k=0; k<n; k++)
				{
					*(IC+i*n+k) -= *(C+i*n+j) * *(IC+j*n+k);
				}
				/* C阵的第i行 - C[i][j]*C阵的第j行*/
				for (k=n-1; k>=j; k--)
				{
					*(C+i*n+k) -= *(C+i*n+j) * *(C+j*n+k);
				}
			}
		}
		else if (j<n-1)
		{
			
			for(l=j+1; l<n; l++)
			{
				/* 若C阵第j行后的C[l][j]不等于0，第j行加上第l行*/
				if (fabs(*(C+l*n+j)) > 1e-15)
				{
					for (k=0; k<n; k++)
					{
						*(IC+j*n+k) += *(IC+l*n+k);
					}
					for (k=n-1; k>=j; k--)
					{
						*(C+j*n+k) += *(C+l*n+k);
					}
					/* IC阵的第j行除以C[j][j]*/
					for (k=0; k<n; k++)
					{
						*(IC+j*n+k) /= *(C+j*n+j);
					}
					/* C阵的第j行除以C[j][j]*/
					for (k=n-1; k>=j; k--)
					{
						*(C+j*n+k) /= *(C+j*n+j);
					}
					/* 第i行 - C[i][j]*第j行*/
					for (i=j+1; i<n; i++)
					{
						for (k=0; k<n; k++)
						{
							*(IC+i*n+k) -= *(C+i*n+j) * *(IC+j*n+k);
						}
						for (k=n-1; k>=j; k--)
						{
							*(C+i*n+k) -= *(C+i*n+j) * *(C+j*n+k);
						}
					}
					break;
				}
			}
			
			if (l == n)  /* C[l][j] 全等于0*/
			{
				return (-1);   /*  矩阵的行列式为零，不可求逆*/
			}
		}
		else  /* C[n][n]等于0*/
		{
			return (-1);    /*  矩阵的行列式为零，不可求逆*/
		}
	}
	/* 化成单位阵*/
	for(j=n-1; j>=1; j--)
	{
		for(i=j-1; i>=0; i--)
		{
			for(k=0; k<n; k++)
			{
				*(IC+i*n+k) -= *(C+i*n+j) * *(IC+j*n+k);
			}
			*(C+i*n+j) = 0;
		}
	}
	
	return (1);
}

/*---计算广义逆矩阵， 6X7， m《=n---
  ----A_pinv = A'*(A*A')^(-1)---
  */
void Rbt_PInvMtrx67( float AA[6][7], float AA_pinv[7][6])   
{
     int i, j;
	 float AA_T[7][6], BB[6][6], BB_Inv[6][6];
	 
	 for(i=0; i<7; i++)
	 {
		 for(j=0; j<6; j++)
		 {
			 AA_T[i][j] = AA[j][i];
		 }
	 }

	Rbt_MulMtrx(6, 6, 7, AA[0], AA_T[0], BB[0]); /*---BB = AA*AA_T----*/
	Rbt_InvMtrx( BB[0], BB_Inv[0], 6 );
	Rbt_MulMtrx(7, 6, 6, AA_T[0], BB_Inv[0], AA_pinv[0]); /*---BB = AA*AA_T----*/
}

/************************************************************************/
/* n = cross(u,v)                                                       */
/************************************************************************/
void Rbt_Cross(float u[], float v[], float n[])
{
	n[0] = u[1] * v[2] - u[2] * v[1];
	n[1] = u[2] * v[0] - u[0] * v[2];
	n[2] = u[0] * v[1] - u[1] * v[0];
}

/************************************************************************/
/*		根据DH参数，计算机械臂的Jacobian矩阵，在末端坐标系中的表示  
		DH=[theta, alpha, a, d]
*/
/************************************************************************/
void Rbt_CalJcb(float DH[JNT_NUM][4], float JointAngle[JNT_NUM],float dRbtJcb[6][JNT_NUM],float T0n_c[4][4])
{

	float T1[4][4], T2[4][4], Tn[4][4],alpha_sr[JNT_NUM], a_sr[JNT_NUM], d_sr[JNT_NUM], qq[JNT_NUM], q0_sr[JNT_NUM];
	float zi_1[3],zi_1_All[3][JNT_NUM],P_i_1_n[3], P_i_All[3][JNT_NUM], Cros_Z_P[3];
	int i,j,k;

	for(i = 0; i<JNT_NUM; i++)
	{
		q0_sr[i] = DH[i][0];
		alpha_sr[i] = DH[i][1];
		a_sr[i] = DH[i][2];
		d_sr[i] = DH[i][3];
	}


	Tn[0][0] = 1;    Tn[0][1] = 0;   Tn[0][2] = 0;  Tn[0][3] = 0; 
	Tn[1][0] = 0;    Tn[1][1] = 1;   Tn[1][2] = 0;  Tn[1][3] = 0; 
	Tn[2][0] = 0;    Tn[2][1] = 0;   Tn[2][2] = 1;  Tn[2][3] = 0; 
	Tn[3][0] = 0;    Tn[3][1] = 0;   Tn[3][2] = 0;  Tn[3][3] = 1; 


	
	for (i=0; i<JNT_NUM; i++)
	{
		for(j=0;j<3;j++) 
		{
			zi_1_All[j][i] = Tn[j][2];			
		}

		qq[i] = q0_sr[i] + JointAngle[i];
		T1[0][0] = (float)(cos(qq[i]));                    T1[0][1] = -(float)(cos(alpha_sr[i])*sin(qq[i])); 
		T1[0][2] = (float)(sin(alpha_sr[i])*sin(qq[i]));   T1[0][3] = (float)(a_sr[i]*cos(qq[i]));

		T1[1][0] = (float)(sin(qq[i]));                    T1[1][1] = (float)(cos(alpha_sr[i])*cos(qq[i])); 
		T1[1][2] = -(float)(sin(alpha_sr[i])*cos(qq[i]));   T1[1][3] = (float)(a_sr[i]*sin(qq[i]));

		T1[2][0] = 0;  T1[2][1] = (float)(sin(alpha_sr[i])); T1[2][2] = (float)(cos(alpha_sr[i]));   T1[2][3] = d_sr[i];

		T1[3][0] = 0;  T1[3][1] = 0; T1[3][2] = 0;   T1[3][3] = 1;

		Rbt_MulMtrx(4,4,4, Tn[0], T1[0], T2[0]);
		
		for (j=0;j<4;j++)
		{	for (k=0;k<4;k++)
			{
				Tn[j][k] = T2[j][k];
			/*	Trans_Matrix[j][4*i+k] = T6[j][k];*/
			}
		}

		for(j=0;j<3;j++) P_i_All[j][i] = Tn[j][3];
	}


	for(j=0;j<3;j++) 
	{
		P_i_1_n[j] = P_i_All[j][JNT_NUM-1];  /*P_0_n*/
	}

	
	for (i=0; i<JNT_NUM; i++)
	{
		for(j=0;j<3;j++) 
		{
			zi_1[j] = zi_1_All[j][i];
		}

		Rbt_Cross(zi_1,P_i_1_n,Cros_Z_P);

		for(j=0;j<3;j++) 
		{
			dRbtJcb[j][i] = Cros_Z_P[j];
			dRbtJcb[j+3][i] = zi_1[j];			
			P_i_1_n[j] = P_i_All[j][JNT_NUM-1] - P_i_All[j][i];  /*P_0_n*/
		}
	}

	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			T0n_c[i][j] = Tn[i][j];   /*---返回当前的末端位姿矩阵--*/
		}
	}

}

/*
矩阵相乘计算函数: C=AB，C为MxN矩阵，A为MxP，B为PxN
输入：A[m][p]，B[p][n]
输出：C[m][n].
*/
void Rbt_MulMtrx(int m, int n, int p, float *A, float *B, float *C)
{
	int i, j, k;
	for( i=0; i<m; i++ )
	{
		for( j=0; j<n; j++ )
		{
			C[n*i+j] = 0.0;
			for( k=0; k<p; k++ )
			{
				C[n*i+j] = C[n*i+j] + A[p*i+k] * B[n*k+j];
			}
		}
	}
}


/************************************************************************/
	/*-----	数值解求解机械臂逆运动学
	

  已经过Matlab验证
    */
/************************************************************************/
int Rbt_ikineItera(float DH[JNT_NUM][4], float T0n[4][4], float JntCurrent[JNT_NUM], float JntCalcted[JNT_NUM])
{
    float DH_q0[JNT_NUM], DH_alpha[JNT_NUM], DH_a[JNT_NUM], DH_d[JNT_NUM];
	float T0n_c[4][4], dA[6], AttErrVec[3];
	float Jcb_0[6][JNT_NUM], Jcb_0_pinv[JNT_NUM][6];
	float q_Itera[JNT_NUM], dq[JNT_NUM], dp_norm, do_norm;  /*---用于迭代的中间变量---*/

	float efs;
	int solutionFlag;
	float n[3], o[3], a[3],nd[3], od[3], ad[3];
	float Cros1[3], Cros2[3], Cros3[3];

	int i, lmax, l_limit;

	lmax = 0;
    l_limit = 1000;
	dp_norm = 1;
	do_norm = 1;
	efs = (float)(1.0e-6);   
	solutionFlag = 0;
	
	for (i=0; i<JNT_NUM; i++)
	{
		DH_q0[i] = DH[i][0];
		DH_alpha[i] = DH[i][1];
		DH_a[i] = DH[i][2];
		DH_d[i] = DH[i][3];

		q_Itera[i] = JntCurrent[i];
	}


	for(i=0; i<3; i++)
	{			
		
		nd[i] = T0n[i][0];
		od[i] = T0n[i][1];
		ad[i] = T0n[i][2];
	}

	while (lmax <= l_limit)
	{
		if((dp_norm> efs) || (do_norm> efs))
		{
			solutionFlag = 0;

			Rbt_CalJcb(DH, q_Itera, Jcb_0, T0n_c);

			for(i=0; i<3; i++)
			{
				n[i] = T0n_c[i][0];
				o[i] = T0n_c[i][1];
				a[i] = T0n_c[i][2];				
			}
			
			Rbt_Cross(n, nd, Cros1);
			Rbt_Cross(o, od, Cros2);
			Rbt_Cross(a, ad, Cros3);
			
			for(i=0; i<3; i++)
			{
				AttErrVec[i] = 0.5f*(Cros1[i] + Cros2[i] + Cros3[i]);  //---姿态误差

				dA[i] = T0n[i][3] - T0n_c[i][3];
				dA[i+3] = AttErrVec[i];
			}

			Rbt_PInvMtrx67(Jcb_0, Jcb_0_pinv);

			Rbt_MulMtrx(JNT_NUM, 1, 6, Jcb_0_pinv[0], dA, dq);  // ---ARM_b用于平衡质心位置和姿态
			
			dp_norm = 0;
			do_norm = 0;
			for (i=0; i<3; i++)
			{
				dp_norm = dp_norm + dA[i]*dA[i];
				do_norm = do_norm + dA[i+3]*dA[i+3];
			}
						

			for (i=0; i<JNT_NUM; i++)
			{
				q_Itera[i] = q_Itera[i] + dq[i];
			}

			lmax = lmax+1;  /*---更新迭代次数---*/
		}
		else
		{
			solutionFlag = 1;   /*---求解成功---*/

			lmax = l_limit+10;  /*---求解成功，直接退出---*/
		}
		
	}

	if (solutionFlag==1)
	{

		/************************************************************************/
		/*  范围[-pi, pi]---                                                    */
		/************************************************************************/
		for (i=0;i<JNT_NUM;i++)
		{
		// 	q_Itera[i] = q_Itera[i]-DH_q0[i];
			while (q_Itera[i]>PI) q_Itera[i] = q_Itera[i]-2*PI;
			while (q_Itera[i]<-PI) q_Itera[i] = q_Itera[i]+2*PI;
		}

		for (i=0; i<JNT_NUM; i++)
		{
			JntCalcted[i] = q_Itera[i] ;
		}


		
	}
	else
	{
		for (i=0; i<JNT_NUM; i++)
		{
			JntCalcted[i] = JntCurrent[i] ;
		}
	}

	return solutionFlag;


}
/************************************************************************/
/*   根据DH参数计算空间机器人末端相对于其安装坐标系的位姿 
------已通过Matlab验证----2011年11月10日------*/                                                                
/************************************************************************/
void Rbt_fkine(float JointAngle[JNT_NUM], float DH[JNT_NUM][4], float T0n[4][4])
{
	float T1[4][4], T2[4][4], alpha_sr[JNT_NUM], a_sr[JNT_NUM], d_sr[JNT_NUM], qq[JNT_NUM], q0_sr[JNT_NUM];//, Tr0[4][4];
	int i,j,k;

	for(i = 0; i<JNT_NUM; i++)
	{
		q0_sr[i] = DH[i][0];
		alpha_sr[i] = DH[i][1];
		a_sr[i] = DH[i][2];
		d_sr[i] = DH[i][3];
	}	


	T0n[0][0] = 1;    T0n[0][1] = 0;   T0n[0][2] = 0;  T0n[0][3] = 0; 
	T0n[1][0] = 0;    T0n[1][1] = 1;   T0n[1][2] = 0;  T0n[1][3] = 0; 
	T0n[2][0] = 0;    T0n[2][1] = 0;   T0n[2][2] = 1;  T0n[2][3] = 0; 
	T0n[3][0] = 0;    T0n[3][1] = 0;   T0n[3][2] = 0;  T0n[3][3] = 1; 

	

	for (i=0; i<JNT_NUM; i++)
	{
		qq[i] = q0_sr[i] + JointAngle[i];
		T1[0][0] = (float)cos(qq[i]);                    T1[0][1] = -(float)(cos(alpha_sr[i])*sin(qq[i])); 
		T1[0][2] = (float)(sin(alpha_sr[i])*sin(qq[i]));   T1[0][3] = (float)(a_sr[i]*cos(qq[i]));

		T1[1][0] = (float)(sin(qq[i]));                    T1[1][1] = (float)(cos(alpha_sr[i])*cos(qq[i])); 
		T1[1][2] = -(float)(sin(alpha_sr[i])*cos(qq[i]));   T1[1][3] = (float)(a_sr[i]*sin(qq[i]));

		T1[2][0] = 0;  T1[2][1] = (float)(sin(alpha_sr[i])); T1[2][2] = (float)(cos(alpha_sr[i]));   T1[2][3] = d_sr[i];

		T1[3][0] = 0;  T1[3][1] = 0; T1[3][2] = 0;   T1[3][3] = 1;
		Rbt_MulMtrx(4,4,4, T0n[0], T1[0], T2[0]);
		for (j=0;j<4;j++)
		{	for (k=0;k<4;k++)
			{
				T0n[j][k] = T2[j][k];
			}
		}
	}


}
/*************************************************************/
/*	函数名称:		nfZyxEuler
	函数功能:		计算旋转变换矩阵
	参数:	
					Euler_zyx[3]：输入变量，欧拉角，数组中的数分别表示绕z、y、x转的角度

					TransMtrx[3][3]：输出变量，旋转后的矩阵
	返回值: 		
	                无
*/
/*************************************************************/
void nfZyxEuler(float Euler_zyx[6],float TransMtrx[4][4])
{
	double angle_z, angle_y, angle_x;
	angle_z = (double)Euler_zyx[3];
	angle_y = (double)Euler_zyx[4];
	angle_x = (double)Euler_zyx[5];

    TransMtrx[0][0]=(float)(cos(angle_z)*cos(angle_y));
    TransMtrx[0][1]=(float)(cos(angle_z)*sin(angle_y)*sin(angle_x) - sin(angle_z)*cos(angle_x));
    TransMtrx[0][2]=(float)(cos(angle_z)*sin(angle_y)*cos(angle_x) + sin(angle_z)*sin(angle_x));

    TransMtrx[1][0]=(float)(sin(angle_z)*cos(angle_y));
    TransMtrx[1][1]=(float)(sin(angle_z)*sin(angle_y)*sin(angle_x) + cos(angle_z)*cos(angle_x));
    TransMtrx[1][2]=(float)(sin(angle_z)*sin(angle_y)*cos(angle_x) - cos(angle_z)*sin(angle_x));

    TransMtrx[2][0]=(float)(-sin(angle_y));
    TransMtrx[2][1]=(float)(cos(angle_y)*sin(angle_x));
    TransMtrx[2][2]=(float)(cos(angle_y)*cos(angle_x));

	TransMtrx[0][3]=  Euler_zyx[0];
	TransMtrx[1][3]=  Euler_zyx[1];
	TransMtrx[2][3]=  Euler_zyx[2];

	TransMtrx[3][0]= 0.0f;
	TransMtrx[3][1]= 0.0f;
	TransMtrx[3][2]= 0.0f;
	TransMtrx[3][3]= 1.0f;


}

/*************************************************************/
/*	函数名称:		nfVisualServoPlanPBVS
函数功能:		基于位置的视觉伺服控制核心规划程序，根据视觉测量值、当前关节角度和当前关节角速度，计算期望关节角度、期望关节角速度和期望关节角加速度。
参    数:	
				fVisualMea[6]：输入变量，视觉测量值（MTS手柄在手爪工具坐标系的位置姿态，姿态采用欧拉角zyx描述）
				CurrentJntAngle[6]：输入变量，机械臂当前关节角度，弧度
				CurrentJntRate[6]：输入变量，机械臂当前关节角速度

				fJntAngleDesired[6]:输出变量,期望关节角度，弧度
				fJntRateDesired [6]:输出变量,期望关节速度
				fJntAccDesired [6]:输出变量,期望关节加速度

返回值: 		
				iCapFlag=1:到捕获点
				=0:没有到捕获点
*/
/*************************************************************/
int nfVisualServoPlanPBVS(float fTempMatrix4[4][4], float CurrentJntAngle[JNT_NUM], float CurrentJntRate[JNT_NUM],
						  float fJntAngleDesired[JNT_NUM], float fJntRateDesired[JNT_NUM], float fJntAccDesired[JNT_NUM])
{
	float  fEndStep,fPoseErr[6];
	float fKp[6], fK_temp;  /*控制系数*/ 
	int i,j;
	int iCapFlag;

	float fTempVec4[4];
	float fTempCtrTrMatrix[4]={0,0,0,1};
	float fTempPoseErrMatrix4[4][4];
	float fTempAdvMatrix4[4][4];
	float fTempForwMatrix4[4][4];
	//float fTempMatrix4[4][4];
	float fJntAng[JNT_NUM];

	float fEndRotMaxStepArc;
	float fMaxJntRateLimit;
	float fTempPBJntAng;


	float gOut_fVisionMeaModifyPBVS_OBC[6];								/*时延补偿手眼视觉测量值测量值*/
	float gOut_fCurAngPBVS_OBC[6];										/*进入计算的当前关节角度值*/

	float A7t[3][3],fVisualMea[3];
	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
		{
			A7t[i][j] = fTempMatrix4[i][j];
		}
	//for (i=0; i<3; i++)
	//{
	//	/*当前关节角输出*/
	//	gOut_fCurAngPBVS_OBC[i] = CurrentJntAngle[i];
	//	gOut_fCurAngPBVS_OBC[i+3] = CurrentJntAngle[i+3];

	//	/*测量位姿补偿*/
	//	fVisualMea[i] =(float) (fVisualMea[i] -g_EndHisStepPara[0]*g_EndHisStep1[i]-g_EndHisStepPara[1]*g_EndHisStep2[i]-g_EndHisStepPara[2]*g_EndHisStep3[i]);  

	//	/*视觉修正数据*/
	//	gOut_fVisionMeaModifyPBVS_OBC[i] = fVisualMea[i];
	//	gOut_fVisionMeaModifyPBVS_OBC[i+3] = fVisualMea[i+3];

	//}

	/*  期望位姿TGp7 */

	//nfZyxEuler(fVisualMea,fTempMatrix4);
	/*  期望位姿TGp7 */
	Rbt_Tr2EulerZyx(A7t, fVisualMea);

	for (i=0; i<3; i++)
	{
		fTempCtrTrMatrix[i] = -g_afDesiredBethingPose_OBC[i];
	}
	/*  抓捕对象在末端坐标系中的位置fTempVec4 */
	for (i=0; i<4; i++)
	{
		fTempVec4[i]=fTempMatrix4[i][0]*fTempCtrTrMatrix[0]+fTempMatrix4[i][1]*fTempCtrTrMatrix[1]+fTempMatrix4[i][2]*fTempCtrTrMatrix[2]+fTempMatrix4[i][3]*fTempCtrTrMatrix[3];
	}
    /*  抓捕对象在末端坐标系中的位置fPoseErr，抓捕对象在末端坐标系中的姿态欧拉角fPoseErr */
	for (i=0; i<3; i++)
	{
		fPoseErr[i] = fTempVec4[i];
		fPoseErr[i+3] =  fVisualMea[i];
	}	

	/*判断是否进入抓捕域 */
	if((fabs((double)fPoseErr[0]) <= g_afVisThreshold_OBC[0]) &&  (fabs((double)fPoseErr[1]) <= g_afVisThreshold_OBC[1]) &&  (fPoseErr[2]<= g_afVisThreshold_OBC[2]) &&  (fPoseErr[2]> -g_afZInnerVisThreshold_OBC)
		&& (fabs((double)fPoseErr[3]) <=  g_afVisThreshold_OBC[3])&& (fabs((double)fPoseErr[4]) <=  g_afVisThreshold_OBC[4]) && (fabs((double)fPoseErr[5]) <=  g_afVisThreshold_OBC[5]))
	{
		iCapFlag = 1;  
	}
	else
	{
		iCapFlag = 0;
	}

	/*计算视觉伺服位置误差控制比系数*/ 
	//if (g_fCapRunTime_OBC <= g_afStartTime_OBC)      
	//{
	//	for(i=0; i<6; i++)
	//	{
	//		fKp[i] =g_afVisKp_OBC[i]*(3.0/(g_afStartTime_OBC*g_afStartTime_OBC)-2*g_fCapRunTime_OBC/(g_afStartTime_OBC*g_afStartTime_OBC*g_afStartTime_OBC))*g_fCapRunTime_OBC*g_fCapRunTime_OBC;
	//	}
	//}    
	//else
	{
		for(i=0; i<6; i++)
		{
			fKp[i] = g_afVisKp_OBC[i];
		}  
	}

	/*伺服误差限制,和伺服速度有关  *kp小一点 */ 
	for (i=0; i<3; i++)
	{	
		if(fPoseErr[i]>g_afEndZPosErrDisLimit_OBC)
		{
			fPoseErr[i] = g_afEndZPosErrDisLimit_OBC; 
		}
		if(fPoseErr[i]<- g_afEndZPosErrDisLimit_OBC)
		{
			fPoseErr[i] = -g_afEndZPosErrDisLimit_OBC; 
		}
	}

	for (i=0; i<3; i++)
	{
		fPoseErr[i] = (float)(fKp[i]*fPoseErr[i]) ;    
		fPoseErr[i+3] = (float)(fKp[i+3]*fPoseErr[i+3]) ; 
	}

	/*末端位姿步长限制  离得近时慢一点*/
	fEndStep = (float)(sqrt((double)(fPoseErr[0]*fPoseErr[0] +fPoseErr[1]*fPoseErr[1]+fPoseErr[2]*fPoseErr[2])));

	if (fEndStep>( g_afEndVelMax_OBC/4) )
	{
		fK_temp = fEndStep/( g_afEndVelMax_OBC/4) ;
		for (i=0; i<3; i++)
		{
			fPoseErr[i] = fPoseErr[i]/(fK_temp);
		}
	}

	for (i=0; i<3; i++)
	{
		fEndRotMaxStepArc =(float)(g_afEndRotSpdMax_OBC*PI/180/4);
		if(fPoseErr[i+3]>fEndRotMaxStepArc)
		{
			fPoseErr[i+3] = fEndRotMaxStepArc; 
		}
		if(fPoseErr[i+3]<-fEndRotMaxStepArc)
		{
			fPoseErr[i+3] = -fEndRotMaxStepArc; 
		}
	}

	/*  记录上几次末端速度   */
	for(i=0; i<6; i++)
	{
		g_EndHisStep3[i] = g_EndHisStep2[i] ;
		g_EndHisStep2[i] = g_EndHisStep1[i] ;
		g_EndHisStep1[i] = fPoseErr[i];
	}

	/*伺服进给增量 */
	for(i=0;i<6;i++)
	{
		gOut_fVisEndErr_OBC[i] = fPoseErr[i];	
	}

	nfZyxEuler(fPoseErr,fTempPoseErrMatrix4);

	/*  当前位姿 */
 	Rbt_fkine(CurrentJntAngle,D_H,fTempForwMatrix4);
	//nfFkineSpaceRbt(CurrentJntAngle, fTempForwMatrix4);
 	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
		{

			fTempAdvMatrix4[i][j]=fTempForwMatrix4[i][0]*fTempPoseErrMatrix4[0][j]+fTempForwMatrix4[i][1]*fTempPoseErrMatrix4[1][j]+fTempForwMatrix4[i][2]*fTempPoseErrMatrix4[2][j]+fTempForwMatrix4[i][3]*fTempPoseErrMatrix4[3][j];
		}
	}


	//nfInvKinePosLev(fTempAdvMatrix4, CurrentJntAngle, fJntAng);
	Rbt_ikineItera(D_H, fTempAdvMatrix4, CurrentJntAngle, fJntAng);

	/* 避免奇异突变, 关节超速 */			
	fMaxJntRateLimit = (float)(g_afJntRateMaxDeg_OBC*PI/180/4);
	for (i=0; i<JNT_NUM; i++)
	{
		fTempPBJntAng = fJntAng[i]-CurrentJntAngle[i];
		if (fTempPBJntAng > fMaxJntRateLimit )
		{
			fJntAng[i] = CurrentJntAngle[i]+fMaxJntRateLimit;
		}
		if (fTempPBJntAng <- fMaxJntRateLimit )
		{
			fJntAng[i] = CurrentJntAngle[i]-fMaxJntRateLimit;
		}				
	}

	for (i=0; i<JNT_NUM; i++)
	{ 		
		fJntAngleDesired[i] = fJntAng[i];			 
		fJntRateDesired[i] = (fJntAng[i] - CurrentJntAngle[i])/g_fSampleTime_OBC;
		fJntAccDesired[i] = (fJntRateDesired[i] - CurrentJntRate[i])/g_fSampleTime_OBC;
	}

	/* 增加关节角度限制 */
	for (i=0; i<JNT_NUM; i++)
	{
		if(fJntAngleDesired[i]>JointAngmax[i])
			fJntAngleDesired[i]=JointAngmax[i];
		if(fJntAngleDesired[i]<JointAngmin[i])
			fJntAngleDesired[i]=JointAngmin[i];
	}

	return iCapFlag;  /*----返回是否到捕获点的标志----*/

}
/*************************************************************/
/*	函数名称:		nfremotecontrol
函数功能:		给定位置增量和欧拉角增量，和当前关节角，输出期望关节角。
参    数:
                fPosedeltaX[6]：输入变量，机械臂位姿增量，相对于末端坐标系
                CurrentJntAngle[6]：输入变量，机械臂当前关节角度，弧度

                fJntAngleDesired[6]:输出变量,期望关节角度，弧度

返回值:
                iCapFlag=1:未超出关节限制
                iCapFlag=0:超出关节限制
*/
/*************************************************************/
int nfremotecontrol(float fPosedeltaX[6], float CurrentJntAngle[JNT_NUM],float fJntAngleDesired[JNT_NUM])
{
    int i,j,iCapFlag;
    float fTempMatrix4[4][4],fdeltaMatrix4[4][4],fTempMatrixnew4[4][4];
    float fJntAng[JNT_NUM];
    float A7t[3][3],fEulerX[3];
    float fPoseX[6]={0};
    /*  当前位姿 */
    Rbt_fkine(CurrentJntAngle,D_H,fTempMatrix4);

    /*  计算T7t*/
    nfZyxEuler(fPosedeltaX,fdeltaMatrix4);

    Rbt_MulMtrx(4, 4, 4, fTempMatrix4[0], fdeltaMatrix4[0], fTempMatrixnew4[0]);

    /*  逆运动学*/
    Rbt_ikineItera(D_H, fTempMatrixnew4, CurrentJntAngle, fJntAng);

    for (i=0; i<JNT_NUM; i++)
    {
        fJntAngleDesired[i] = fJntAng[i];
    }

    /* 增加关节角度限制 */
    for (i=0; i<JNT_NUM; i++)
    {
        if(fJntAngleDesired[i]>JointAngmax[i])
        {
            fJntAngleDesired[i]=JointAngmax[i];
            iCapFlag=0;
        }
        else if(fJntAngleDesired[i]<JointAngmin[i])
        {
            fJntAngleDesired[i]=JointAngmin[i];
            iCapFlag=0;
        }
        else
            iCapFlag=1;
    }
    return iCapFlag;
}
