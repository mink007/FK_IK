#include <string.h>
#include <iostream>
//#include "Kinematics.h"
//#include "MatrixUtils.h"
#include <math.h>
//#include "Eigen/Dense"
#define PI 3.141592654


float result3[3][3]; //, result1, result2;

//float a2=130.0,a3=150.0,d1=157.0,d5=155.0;
//****9/9/2023
//float a1=0.0,a2=2.0,a3=2.0,a4=0.0,a5=0.0;
//float d1=3.0,d2=0.0,d3=0.0,d4=0.0,d5=1.5;
//****9/9/2023
// d1=3.0,d2=0.0,d3=0.0,d4=0.0,d5=1.5;

float a1=0.0,a2=2.0,a3=2.0,a4=0.0,a5=0.0;
float d1=3.0,d2=0.0,d3=0.0,d4=0.0,d5=1.5;




void mul_matrix(float* mat1, float* mat2, int r1, int c1, int r2, int c2, float* result) {
    for (int i = 0; i < r1; i++) {
        for(int j = 0; j < c2; j++) {
            result[c2 * i + j] = 0;
            for (int k = 0; k < c1; k++) {
                result[c2 * i + j] = result[c2 * i + j] + mat1[c1 * i + k] * mat2[c2 * k + j];
            }
        }
    }
}



void forward_kinematic(float* qi, float* alfai, float* fk_result){
  printf("\n In Forward Kinematics Function \n");
  /*
  printf("\n");
  printf("Qis in FK\n");
  for (int i = 1; i < 6; i++) {
    printf("%f", qi[i]*180.0/PI);
    printf("\t");
  }
  */
  printf("\n");
                float mat1[4][4]= {
                                {  cosf(qi[1]),              -cosf(alfai[1])*sinf(qi[1]),               sinf(alfai[1])*sinf(qi[1]),    a1*cosf(qi[1]) },
                                {  sinf(qi[1]),               cosf(alfai[1])*cosf(qi[1]),              -sinf(alfai[1])*cosf(qi[1]),    a1*sinf(qi[1]) },
                                {  0,                                     sinf(alfai[1]),                           cosf(alfai[1]),               d1  },
                                {  0,                                                  0,                                        0,                 1 }
                                };


                float mat2[4][4]= {
                                {  cosf(qi[2]),              -cosf(alfai[2])*sinf(qi[2]),               sinf(alfai[2])*sinf(qi[2]),    a2*cosf(qi[2]) },
                                {  sinf(qi[2]),               cosf(alfai[2])*cosf(qi[2]),              -sinf(alfai[2])*cosf(qi[2]),    a2*sinf(qi[2]) },
                                {  0,                                     sinf(alfai[2]),                           cosf(alfai[2]),               d2  },
                                {  0,                                                  0,                                        0,                 1 }
                                };


                float mat3[4][4]= {
                                {  cosf(qi[3]),              -cosf(alfai[3])*sinf(qi[3]),               sinf(alfai[3])*sinf(qi[3]),    a3*cosf(qi[3]) },
                                {  sinf(qi[3]),               cosf(alfai[3])*cosf(qi[3]),              -sinf(alfai[3])*cosf(qi[3]),    a3*sinf(qi[3]) },
                                {  0,                                     sinf(alfai[3]),                           cosf(alfai[3]),               d3  },
                                {  0,                                                  0,                                        0,                 1 }
                                };


                float mat4[4][4]= {
                                {  cosf(qi[4]),              -cosf(alfai[4])*sinf(qi[4]),               sinf(alfai[4])*sinf(qi[4]),    a4*cosf(qi[4]) },
                                {  sinf(qi[4]),               cosf(alfai[4])*cosf(qi[4]),              -sinf(alfai[4])*cosf(qi[4]),    a4*sinf(qi[4]) },
                                {  0,                                     sinf(alfai[4]),                           cosf(alfai[4]),               d4  },
                                {  0,                                                  0,                                        0,                 1 }
                                };


                float mat5[4][4]= {
                                {  cosf(qi[5]),              -cosf(alfai[5])*sinf(qi[5]),               sinf(alfai[5])*sinf(qi[5]),    a5*cosf(qi[5]) },
                                {  sinf(qi[5]),               cosf(alfai[5])*cosf(qi[5]),              -sinf(alfai[5])*cosf(qi[5]),    a5*sinf(qi[5]) },
                                {  0,                                     sinf(alfai[5]),                           cosf(alfai[5]),               d5  },
                                {  0,                                                  0,                                        0,                1 } 
                                };




                float result[4][4];
                mul_matrix((float*)mat1, (float*)mat2, 4, 4, 4, 4, (float*)result);

                float result1[4][4];
                mul_matrix((float*)result, (float*)mat3, 4, 4, 4, 4, (float*)result1);

                float result2[4][4];
                mul_matrix((float*)result1, (float*)mat4, 4, 4, 4, 4, (float*)result2);

                //float result3[4][4];
                mul_matrix((float*)result2, (float*)mat5, 4, 4, 4, 4, (float*)fk_result);

                
    printf("\n Out of Forward Kinematics Function \n");         
}


float* inverse_kinematic(float* qi, float* alfai, float* fk_result){
    printf("In Inverse Kinematic func\n");

    printf("Transformation Matrix of End Effector\n");
    for (int i = 0; i < 16; i++) {
            
            printf("%f",fk_result[i]);
            printf("\t");
            if (i==3 || i==7 || i==11||i==15){
                printf("\n");
            }
    }
    printf("\n");

    float x5x = fk_result[0];
    float x5y = fk_result[1];
    float x5z = fk_result[2];
  
    float y5x = fk_result[4];
    float y5y = fk_result[5];
    float y5z = fk_result[6];
  
    float z5x = fk_result[8];
    float z5y = fk_result[9];
    float z5z = fk_result[10];

    float p5x = fk_result[3];//eef_xyz[0]; //result3[0][3];
    float p5y = fk_result[7]; //result3[1][3];
    float p5z = fk_result[11]; //result3[2][3];


    float c234  = -z5z;
    float theta234 = acosf(c234);
    printf("c234 %f ", c234);
    float s234 = sinf(theta234);
    printf(" s234 %f ", s234);
    float a = d1 - (d5*c234) - p5z;
    printf("\n a = %f ", a);

    float q1 = atan(p5y/p5x); //output is in radians
    float c1 = cosf(q1);
    float s1 = sinf(q1);
    printf(" d5 == %f ", d5);
    float b = p5x*c1 + p5y*s1 + d5*s234;

    printf("b = %f ", b);


    
    float q3_arg = (((a*a) + (b*b)  - (a2*a2) - (a3*a3))/(2*a2*a3)) ;
    printf("\n Q3_ARG = %f ", q3_arg);
    /*
    if (q3_arg >= -1.0 && q3_arg <= 1.0) {
        float q3 = acosf(q3_arg);
        // Continue with the rest of your calculations...
    } else {
        // Handle the case where q3_arg is out of the valid range (print an error, set a default value, etc.)
        printf("Error: Invalid argument for acosf\n");
        if (q3_arg > 1.0) {
            q3_arg = 1.0;
        }
        if (q3_arg < -1.0) {
            q3_arg = -1.0;
        }
    }
    */

    float q3 = acosf(q3_arg); //+ PI/2.0;

    //printf("cosf(1) %f", acosf(1.0));


    printf("a2 = %f ", a2);
    printf("a3 = %f\n", a3);

    float c3 = cosf(q3);
    float s3 = sinf(q3);
    float q2 = atan2f(a*(a2+a3*c3) - b*a3*s3,  a*a3*s3 + b*(a2 + a3*c3));


    float s23 = sinf(q2 + q3);
    float c23 = cosf(q2 + q3);

    float q4 = theta234 - q2 - q3; 

    float q5 = c234*q1 - atan2f(x5y,x5x);

    printf("=====%f=====%f=====%f=====\n", x5x, x5y, x5z);
    printf("=====%f=====%f=====%f=====\n", y5x, y5y, y5z);
    printf("=====%f=====%f=====%f=====\n", z5x, z5y, z5z);
    printf("=====%f=====%f=====%f=====\n", p5x, p5y, p5z);

    printf("Degs = q1=%f q2=%f q3=%f q4=%f q5=%f\n", q1*180.0/PI, q2*180.0/PI, q3*180.0/PI, q4*180.0/PI, q5*180.0/PI);

    float* qis = new float[6];
    qis[0] = 0.0;
    qis[1] = q1*180.0/PI;
    qis[2] = q2*180.0/PI;
    qis[3] = q3*180.0/PI;
    qis[4] = q4*180.0/PI;
    qis[5] = q5*180.0/PI;
   
    printf("Out of Inverse Kinematic func\n");
    return qis;

}

int main(){

//int r=4,c=4;

float x5x = 0.0;
float x5y = 0.0;
float x5z = 0.0;
float y5x = 0.0;
float y5y = 0.0;
float y5z = 0.0;
float z5x = 0.0;
float z5y = 0.0;
float z5z = 0.0;
float p5x = 0.0;
float p5y = 0.0;
float p5z = 0.0;

 
//This alfa settings is good.
float alfai[6] = {0.0,              -90.0,            0.0,             0.0,     -90.0,   0.0}; //0th element is not used

//float alfai[6] = {0.0,              -90.0,            0.0,             0.0,     90.0,   0.0}; //0th element is not used

//float qi[6] =    {0.0,              -0.000000   ,    -49.967673  ,      81.010702 ,   -31.043029 ,   0.0}; 
//float qi[6] =    {0.0,              -0.000000   ,    -90.0  ,     90.0 ,  0.0 ,   0.0}; 
//float qi[6] =    {0.0,              -0.000000   ,    -134.934766   ,117.934162   ,-72.937761,   0.0}; 

//float qi[6] =    {0.0,    0.000000  , -0.330042*180/PI   , -2.089309*180/PI , 0.848075*180/PI  , 0.000000 };

//float qi[6] =    {0.0,    0.000000  ,  -18.7394, -119.879, 48.5912, 0.0 };
//float qi[6] =    {0.0,              -0.000000   ,    -89.966012   ,  -42.250780   ,41.965917,   0.0}; 
//float qi[6] =    {0.0,              -45.000000 ,-109.999884 , -28.550833 , 48.646989 , 45.0};

//float qi[6] =    {0.0,              0.0 ,  -0.225449*180.0/PI , -2.182140*180.0/PI , 0.848046*180.0/PI , 0.0};


//float qi[6] = {0, 0 ,-112.89 ,82.3151 ,-34.8955, 0};

//float qi[6] = {0, -0.000000   ,-112.892717   ,99.382609   ,-20.239893   , 0 };

//float qi[6] =    {0.0,              0.0 ,  -90.0 , 90.0 , -90.0 , 0.0};

float qi[6] =    {0.0,              0.0 ,  0.0 , 0.0 , 0.0 , 0.0};



//float qi[6] =    {0.0,              0.0 ,  -0.0 , -64.26, -39.8252 , 0.0};



printf("Input Qi for all joints \n");
for (int j = 1; j < 6; j++) {
    printf("Q[%d] %f",j, qi[j]);
    printf("\t");
}
printf("\n");


//Degree To RAD:
qi[0] = qi[0] * PI/180.0;
qi[1] = qi[1] * PI/180.0;
qi[2] = qi[2] * PI/180.0;
qi[3] = qi[3] * PI/180.0;
qi[4] = qi[4] * PI/180.0;
qi[5] = qi[5] * PI/180.0;
//qi[6] = qi[6] * PI/180.0;

alfai[0] = alfai[0]*PI/180.0;
alfai[1] = alfai[1]*PI/180.0;
alfai[2] = alfai[2]*PI/180.0;
alfai[3] = alfai[3]*PI/180.0;
alfai[4] = alfai[4]*PI/180.0;
alfai[5] = alfai[5]*PI/180.0;

float fk_result[4][4]; 
forward_kinematic((float*)qi, (float*)alfai, (float*)fk_result);

  
printf("Transformation Matrix of EndEffector from FK Function\n");
for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
        printf("%f",fk_result[i][j]);
        printf("\t");
    }
    printf("\n");
}
  

float tfk_result[4][4];


///////////////////////////

tfk_result[0][0] = 0.0;
tfk_result[1][0] = 0.0;
tfk_result[2][0] = 1.0;
tfk_result[3][0] = 0.0;

tfk_result[0][1] = 0.0;
tfk_result[1][1] = -1.0;
tfk_result[2][1] = 0.0;
tfk_result[3][1] = 0.0;

tfk_result[0][2] = 1.0;
tfk_result[1][2] = 0.0;
tfk_result[2][2] = 0.0;
tfk_result[3][2] = 0.0;

tfk_result[0][3] = 3.5;
tfk_result[1][3] = 0.0;
tfk_result[2][3] = 5.0;
tfk_result[3][3] = 1.0;

  
  
printf("\n\n\n=====STARTING=======\n");
float* ik_qis = inverse_kinematic((float*)qi, (float*)alfai, (float*)tfk_result);
/*
ik_qis[0] = ik_qis[0] * PI/180.0;
ik_qis[1] = ik_qis[1] * PI/180.0;
ik_qis[2] = ik_qis[2] * PI/180.0;
ik_qis[3] = ik_qis[3] * PI/180.0;
ik_qis[4] = ik_qis[4] * PI/180.0;
ik_qis[5] = ik_qis[5] * PI/180.0;
*/
for (int j = 1; j < 6; j++) {
    printf("Q%d = %f", j, ik_qis[j]);
    printf("\t");
}


ik_qis[0] = ik_qis[0] * PI/180.0;
ik_qis[1] = ik_qis[1] * PI/180.0;
ik_qis[2] = ik_qis[2] * PI/180.0;
ik_qis[3] = ik_qis[3] * PI/180.0;
ik_qis[4] = ik_qis[4] * PI/180.0;
ik_qis[5] = ik_qis[5] * PI/180.0;

//Verify Qis from IK function
forward_kinematic((float*)ik_qis, (float*)alfai, (float*)fk_result);
printf("\nForward Kinematics Matrix after IK\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f",fk_result[i][j]);
            printf("\t");
        }
        printf("\n");
    }
printf("\n");
///////////////////////////

return 0;

}