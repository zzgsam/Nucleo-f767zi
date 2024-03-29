/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
acadoWorkspace.state[0] = acadoVariables.x[0];
acadoWorkspace.state[1] = acadoVariables.x[1];
acadoWorkspace.state[2] = acadoVariables.x[2];
acadoWorkspace.state[15] = acadoVariables.u[0];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{

acadoWorkspace.state[15] = acadoVariables.u[lRun1];

ret = acado_integrate(acadoWorkspace.state, lRun1 == 0);

acadoVariables.x[lRun1 * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[lRun1 * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[lRun1 * 3 + 5] = acadoWorkspace.state[2];

acadoWorkspace.evGx[lRun1 * 9] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 9 + 1] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 9 + 2] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 9 + 3] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 9 + 4] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 9 + 5] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 9 + 6] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 9 + 7] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 9 + 8] = acadoWorkspace.state[11];

acadoWorkspace.evGu[lRun1 * 3] = acadoWorkspace.state[12];
acadoWorkspace.evGu[lRun1 * 3 + 1] = acadoWorkspace.state[13];
acadoWorkspace.evGu[lRun1 * 3 + 2] = acadoWorkspace.state[14];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 4] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 4 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 4 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 4 + 3] = acadoWorkspace.objValueOut[3];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[30];
acadoWorkspace.objValueIn[1] = acadoVariables.x[31];
acadoWorkspace.objValueIn[2] = acadoVariables.x[32];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2];
dNew[1] += + Gx1[3]*dOld[0] + Gx1[4]*dOld[1] + Gx1[5]*dOld[2];
dNew[2] += + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[3] + Gx1[2]*Gx2[6];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[7];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[8];
Gx3[3] = + Gx1[3]*Gx2[0] + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[6];
Gx3[4] = + Gx1[3]*Gx2[1] + Gx1[4]*Gx2[4] + Gx1[5]*Gx2[7];
Gx3[5] = + Gx1[3]*Gx2[2] + Gx1[4]*Gx2[5] + Gx1[5]*Gx2[8];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[3] + Gx1[8]*Gx2[6];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[4] + Gx1[8]*Gx2[7];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[5] + Gx1[8]*Gx2[8];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1] + Gx1[2]*Gu1[2];
Gu2[1] = + Gx1[3]*Gu1[0] + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[2];
Gu2[2] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[1] + Gx1[8]*Gu1[2];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] += + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = (real_t)1.0000000000000000e+00;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = acadoWorkspace.H[(iCol * 10) + (iRow)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)1.0000000000000000e+05*dOld[0];
dNew[1] = +dOld[1];
dNew[2] = +dOld[2];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = +Dy1[3];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)1.0000000000000000e+05*Dy1[0];
QDy1[1] = +Dy1[1];
QDy1[2] = +Dy1[2];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[1]*QDy1[1] + E1[2]*QDy1[2];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[1]*Gx1[3] + E1[2]*Gx1[6];
H101[1] += + E1[0]*Gx1[1] + E1[1]*Gx1[4] + E1[2]*Gx1[7];
H101[2] += + E1[0]*Gx1[2] + E1[1]*Gx1[5] + E1[2]*Gx1[8];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 3; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0];
dNew[1] += + E1[1]*U1[0];
dNew[2] += + E1[2]*U1[0];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e+05*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e+05*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000000e+05*Gx1[2];
Gx2[3] = +Gx1[3];
Gx2[4] = +Gx1[4];
Gx2[5] = +Gx1[5];
Gx2[6] = +Gx1[6];
Gx2[7] = +Gx1[7];
Gx2[8] = +Gx1[8];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e+01*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e+01*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000000e+01*Gx1[2];
Gx2[3] = + (real_t)1.0000000000000000e+01*Gx1[3];
Gx2[4] = + (real_t)1.0000000000000000e+01*Gx1[4];
Gx2[5] = + (real_t)1.0000000000000000e+01*Gx1[5];
Gx2[6] = + (real_t)1.0000000000000000e+01*Gx1[6];
Gx2[7] = + (real_t)1.0000000000000000e+01*Gx1[7];
Gx2[8] = + (real_t)1.0000000000000000e+01*Gx1[8];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e+05*Gu1[0];
Gu2[1] = +Gu1[1];
Gu2[2] = +Gu1[2];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e+01*Gu1[0];
Gu2[1] = + (real_t)1.0000000000000000e+01*Gu1[1];
Gu2[2] = + (real_t)1.0000000000000000e+01*Gu1[2];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 30 */
static const int xBoundIndices[ 30 ] = 
{ 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 9 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 3 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 3 ]), &(acadoWorkspace.E[ 6 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.evGx[ 18 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.E[ 9 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.E[ 12 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.E[ 15 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.evGx[ 27 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.E[ 18 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 21 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.E[ 24 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 9 ]), &(acadoWorkspace.E[ 27 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.evGx[ 36 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.E[ 30 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.E[ 33 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 36 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.E[ 39 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 42 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGx[ 45 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 45 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 51 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.E[ 54 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.E[ 57 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 15 ]), &(acadoWorkspace.E[ 60 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.evGx[ 54 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.E[ 63 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 66 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.E[ 69 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.E[ 75 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 78 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.E[ 81 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.evGx[ 63 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.E[ 84 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.E[ 87 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.E[ 90 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 93 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.E[ 99 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.E[ 102 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 21 ]), &(acadoWorkspace.E[ 105 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.evGx[ 72 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 108 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.E[ 111 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.E[ 114 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.E[ 117 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.E[ 123 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.E[ 126 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.E[ 129 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 132 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGx[ 81 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 135 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.E[ 138 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.E[ 141 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 147 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.E[ 150 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.E[ 153 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.E[ 156 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.E[ 159 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.E[ 162 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.QE[ 3 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QE[ 9 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.QE[ 15 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QE[ 21 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 33 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 45 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 51 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 63 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 69 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 87 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 111 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 135 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 141 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 147 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 153 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 3 ]), &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 9 ]), &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 18 ]), &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 45 ]), &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 63 ]), &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 135 ]), &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 6 ]), &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 21 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 33 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 87 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 111 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 138 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 15 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 51 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 69 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 114 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 141 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 27 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 39 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 93 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 117 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 42 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 57 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 75 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 147 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 99 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 123 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 81 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 126 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 153 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 105 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 129 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 159 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 162 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 27 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.QE[ 3 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QE[ 9 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 45 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 63 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 135 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 21 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 33 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 87 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 111 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 138 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QE[ 15 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 51 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 69 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 141 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 144 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 147 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 150 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 153 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 156 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 159 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QE[ 21 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 33 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 87 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 111 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 138 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 15 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 51 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 69 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 141 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 144 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 147 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 150 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 153 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 156 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 159 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.QE[ 15 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 51 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 69 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 141 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 144 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 147 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 150 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 153 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 156 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 159 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 147 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 150 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 153 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 156 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 159 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 147 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 150 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 153 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 156 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 159 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 153 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 156 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 159 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 153 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 156 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 159 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 159 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.QE[ 159 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.QE[ 162 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 3 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 9 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 18 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 30 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 45 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 63 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 84 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 135 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 6 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 21 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 33 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 87 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 111 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 138 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 15 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 51 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 69 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 114 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 141 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 27 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 39 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 93 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 117 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 42 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 57 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 75 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 147 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 99 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 123 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 81 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 126 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 153 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 105 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 129 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 159 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 162 ]), &(acadoWorkspace.g[ 9 ]) );
acadoWorkspace.lb[0] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-2.5500000000000000e+02 - acadoVariables.u[9];
acadoWorkspace.ub[0] = (real_t)2.5500000000000000e+02 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)2.5500000000000000e+02 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)2.5500000000000000e+02 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)2.5500000000000000e+02 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)2.5500000000000000e+02 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)2.5500000000000000e+02 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)2.5500000000000000e+02 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)2.5500000000000000e+02 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)2.5500000000000000e+02 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)2.5500000000000000e+02 - acadoVariables.u[9];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 3;
lRun4 = ((lRun3) / (3)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (3)) + ((lRun3) % (3));
acadoWorkspace.A[(lRun1 * 10) + (lRun2)] = acadoWorkspace.E[lRun5];
}
}

}

void acado_condenseFdb(  )
{
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 9 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 27 ]) );

acadoWorkspace.QDy[30] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[31] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[32] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[2];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 3 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QDy[ 9 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.QDy[ 9 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 9 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[1] += + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[2] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[3] += + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[4] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[5] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[6] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[7] += + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[8] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[9] += + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[2];

tmp = + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoVariables.x[3];
acadoWorkspace.lbA[0] = - tmp;
acadoWorkspace.ubA[0] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[2] + acadoVariables.x[4];
acadoWorkspace.lbA[1] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[1] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[2] + acadoVariables.x[5];
acadoWorkspace.lbA[2] = - tmp;
acadoWorkspace.ubA[2] = (real_t)2.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[2] + acadoVariables.x[6];
acadoWorkspace.lbA[3] = - tmp;
acadoWorkspace.ubA[3] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoVariables.x[7];
acadoWorkspace.lbA[4] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[4] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoVariables.x[8];
acadoWorkspace.lbA[5] = - tmp;
acadoWorkspace.ubA[5] = (real_t)2.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2] + acadoVariables.x[9];
acadoWorkspace.lbA[6] = - tmp;
acadoWorkspace.ubA[6] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[2] + acadoVariables.x[10];
acadoWorkspace.lbA[7] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[7] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoVariables.x[11];
acadoWorkspace.lbA[8] = - tmp;
acadoWorkspace.ubA[8] = (real_t)2.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[2] + acadoVariables.x[12];
acadoWorkspace.lbA[9] = - tmp;
acadoWorkspace.ubA[9] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoVariables.x[13];
acadoWorkspace.lbA[10] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[10] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[2] + acadoVariables.x[14];
acadoWorkspace.lbA[11] = - tmp;
acadoWorkspace.ubA[11] = (real_t)2.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoVariables.x[15];
acadoWorkspace.lbA[12] = - tmp;
acadoWorkspace.ubA[12] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[2] + acadoVariables.x[16];
acadoWorkspace.lbA[13] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[13] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[2] + acadoVariables.x[17];
acadoWorkspace.lbA[14] = - tmp;
acadoWorkspace.ubA[14] = (real_t)2.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoVariables.x[18];
acadoWorkspace.lbA[15] = - tmp;
acadoWorkspace.ubA[15] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoVariables.x[19];
acadoWorkspace.lbA[16] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[16] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[2] + acadoVariables.x[20];
acadoWorkspace.lbA[17] = - tmp;
acadoWorkspace.ubA[17] = (real_t)2.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2] + acadoVariables.x[21];
acadoWorkspace.lbA[18] = - tmp;
acadoWorkspace.ubA[18] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[2] + acadoVariables.x[22];
acadoWorkspace.lbA[19] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[19] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoVariables.x[23];
acadoWorkspace.lbA[20] = - tmp;
acadoWorkspace.ubA[20] = (real_t)2.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[2] + acadoVariables.x[24];
acadoWorkspace.lbA[21] = - tmp;
acadoWorkspace.ubA[21] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[2] + acadoVariables.x[25];
acadoWorkspace.lbA[22] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[22] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[2] + acadoVariables.x[26];
acadoWorkspace.lbA[23] = - tmp;
acadoWorkspace.ubA[23] = (real_t)2.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoVariables.x[27];
acadoWorkspace.lbA[24] = - tmp;
acadoWorkspace.ubA[24] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2] + acadoVariables.x[28];
acadoWorkspace.lbA[25] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[25] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[2] + acadoVariables.x[29];
acadoWorkspace.lbA[26] = - tmp;
acadoWorkspace.ubA[26] = (real_t)2.0000000000000000e+02 - tmp;
tmp = + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[2] + acadoVariables.x[30];
acadoWorkspace.lbA[27] = - tmp;
acadoWorkspace.ubA[27] = (real_t)1.8000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoVariables.x[31];
acadoWorkspace.lbA[28] = (real_t)-1.0000000000000000e+00 - tmp;
acadoWorkspace.ubA[28] = (real_t)1.0000000000000000e+00 - tmp;
tmp = + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[2] + acadoVariables.x[32];
acadoWorkspace.lbA[29] = - tmp;
acadoWorkspace.ubA[29] = (real_t)2.0000000000000000e+02 - tmp;

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];

acadoVariables.x[3] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2];
acadoVariables.x[4] += + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[2];
acadoVariables.x[5] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[2];
acadoVariables.x[6] += + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[2];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2];
acadoVariables.x[9] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2];
acadoVariables.x[10] += + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[2];
acadoVariables.x[11] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2];
acadoVariables.x[12] += + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[2];
acadoVariables.x[13] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2];
acadoVariables.x[14] += + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[2];
acadoVariables.x[15] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2];
acadoVariables.x[16] += + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[2];
acadoVariables.x[17] += + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[2];
acadoVariables.x[18] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2];
acadoVariables.x[19] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2];
acadoVariables.x[20] += + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[2];
acadoVariables.x[21] += + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2];
acadoVariables.x[22] += + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[2];
acadoVariables.x[23] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2];
acadoVariables.x[24] += + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[2];
acadoVariables.x[25] += + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[2];
acadoVariables.x[26] += + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[2];
acadoVariables.x[27] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2];
acadoVariables.x[28] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2];
acadoVariables.x[29] += + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[2];
acadoVariables.x[30] += + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[2];
acadoVariables.x[31] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2];
acadoVariables.x[32] += + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[2];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 3 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3 ]), acadoWorkspace.x, &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 9 ]), acadoWorkspace.x, &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 18 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 30 ]), acadoWorkspace.x, &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 45 ]), acadoWorkspace.x, &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 63 ]), acadoWorkspace.x, &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 84 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), acadoWorkspace.x, &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 135 ]), acadoWorkspace.x, &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 30 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[15] = acadoVariables.u[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[30] = xEnd[0];
acadoVariables.x[31] = xEnd[1];
acadoVariables.x[32] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[30];
acadoWorkspace.state[1] = acadoVariables.x[31];
acadoWorkspace.state[2] = acadoVariables.x[32];
if (uEnd != 0)
{
acadoWorkspace.state[15] = uEnd[0];
}
else
{
acadoWorkspace.state[15] = acadoVariables.u[9];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[30] = acadoWorkspace.state[0];
acadoVariables.x[31] = acadoWorkspace.state[1];
acadoVariables.x[32] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[9] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9];
kkt = fabs( kkt );
for (index = 0; index < 10; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 30; ++index)
{
prd = acadoWorkspace.y[index + 10];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 4 */
real_t tmpDy[ 4 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 4] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 4];
acadoWorkspace.Dy[lRun1 * 4 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 4 + 1];
acadoWorkspace.Dy[lRun1 * 4 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 4 + 2];
acadoWorkspace.Dy[lRun1 * 4 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 4 + 3];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[30];
acadoWorkspace.objValueIn[1] = acadoVariables.x[31];
acadoWorkspace.objValueIn[2] = acadoVariables.x[32];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 4]*(real_t)1.0000000000000000e+05;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 4 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 4 + 2];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 4 + 3];
objVal += + acadoWorkspace.Dy[lRun1 * 4]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 4 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 4 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 4 + 3]*tmpDy[3];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000000e+01;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)1.0000000000000000e+01;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)1.0000000000000000e+01;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

