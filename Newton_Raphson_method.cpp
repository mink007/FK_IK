#include <iostream>
#include <cmath>

const double PI = 3.14159265358979323846;
double jointWeights[5] = {1.0, 1.0, 1.0, 1.0, 1.0}; 

// Robot parameters
const double alpha[6] = {0, -90, 0, 0, 90, 0};
const double a[6] = {0.0, 0.0, 2.0, 2.0, 0.0, 0.0};
const double d[6] = {3.0, 0.0, 0.0, 0.0, 1.5, 0.0};


double alfai[6] = {0.0,              -PI/2,            0.0,             0.0,     -PI/2,   0.0}; //0th element is not used
double a1=0.0,a2=2.0,a3=2.0,a4=0.0,a5=0.0;
double d1=3.0,d2=0.0,d3=0.0,d4=0.0,d5=1.5;

// Target end-effector position and orientation (in terms of rotation matrix)
/*
const double target_x =  -0.483040;
const double target_y = 0.483041;   
const double target_z =  6.200780;
const double target_R[3][3] = {{
                                 -0.499160, -0.500840 , 0.707106  
                                },
                               {
                                -0.500840 , -0.499160 , -0.707106   
                               },
                               {
                                 0.707106,  -0.707106 , -0.001680 
                                }};
*/

const double target_x =  3.5;
const double target_y =  0.0;
const double target_z =  5.0;
const double target_R[3][3] = {{
                                 0.0,0.0,1.0 
                                },
                               {
                                0.0,-1.0,0.0   
                               },
                               {
                                1.0,0.0,0.0 
                                }};

/*
const double target_x =  2.0;
const double target_y =  0.0;
const double target_z =  3.5;
const double target_R[3][3] = {{
                                 1.0,0.0,0.0 
                                },
                               {
                                0.0,-1.0,0.0   
                               },
                               {
                                0.0,0.0,-1.0 
                                }};
*/
/*
const double target_x =  2.0;
const double target_y =  0.0;
const double target_z =  4.062;
const double target_R[3][3] = {{
                                 0.8315,0.0,0.5556 
                                },
                               {
                                0.0,-1.0,0.0   
                               },
                               {
                                0.5556, 0.0,-0.8315 
                                }};
*/
/*
const double target_x =  3.5;
const double target_y =  0.0;
const double target_z =  5.0;
const double target_R[3][3] = {{
                                 0.0,0.0,1.0 
                                },
                               {
                                0.0,-1.0,0.0   
                               },
                               {
                                1.0,0.0,0.0 
                                }};

*/

double fk_result[4][4]; 
double qi[6] ; //=    {0.0,              -0.000000   ,    -134.934766   ,117.934162   ,-72.937761,   0.0};

// Function to calculate the transformation matrix for a given joint
void calculateTransformationMatrix(double alpha, double a, double d, double theta, double(&T)[4][4]) {
    double cTheta = std::cos(theta);
    double sTheta = std::sin(theta);
    double cAlpha = std::cos(alpha);
    double sAlpha = std::sin(alpha);

    T[0][0] = cTheta;
    T[0][1] = -sTheta * cAlpha;
    T[0][2] = sTheta * sAlpha;
    T[0][3] = a * cTheta;

    T[1][0] = sTheta;
    T[1][1] = cTheta * cAlpha;
    T[1][2] = -cTheta * sAlpha;
    T[1][3] = a * sTheta;

    T[2][0] = 0.0;
    T[2][1] = sAlpha;
    T[2][2] = cAlpha;
    T[2][3] = d;

    T[3][0] = 0.0;
    T[3][1] = 0.0;
    T[3][2] = 0.0;
    T[3][3] = 1.0;
}

void mul_matrix(double* mat1, double* mat2, int r1, int c1, int r2, int c2, double* result) {
    for (int i = 0; i < r1; i++) {
        for(int j = 0; j < c2; j++) {
            result[c2 * i + j] = 0;
            for (int k = 0; k < c1; k++) {
                result[c2 * i + j] = result[c2 * i + j] + mat1[c1 * i + k] * mat2[c2 * k + j];
            }
        }
    }
}



void forwardKinematics(double* jointAngles, double(&R)[3][3], double& x, double& y, double& z, int print_trMat = 0){
  //printf("\n In Forward Kinematics Function \n");
  /*
  printf("\n");
  printf("Qis in FK\n");
  for (int i = 1; i < 6; i++) {
    printf("%f", qi[i]*180.0/PI);
    printf("\t");
  }
  */
  
  
  //printf("HERE 1\n");
  qi[0] = 0;
  //printf("HERE 2\n");
  qi[1] = jointAngles[0];
  //printf("HERE 3\n");
  qi[2] = jointAngles[1];
  qi[3] = jointAngles[2];
  qi[4] = jointAngles[3];
  qi[5] = jointAngles[4];

  /*
    printf("Input Qi for all joints \n");
    for (int j = 1; j < 6; j++) {
        printf("Q[%d] %f",j, qi[j]*180/PI);
        printf("\t");
    }
    printf("\n");
   */
   //printf("\n");
                double mat1[4][4]= {
                                {  cosf(qi[1]),              -cosf(alfai[1])*sinf(qi[1]),               sinf(alfai[1])*sinf(qi[1]),    a1*cosf(qi[1]) },
                                {  sinf(qi[1]),               cosf(alfai[1])*cosf(qi[1]),              -sinf(alfai[1])*cosf(qi[1]),    a1*sinf(qi[1]) },
                                {  0,                                     sinf(alfai[1]),                           cosf(alfai[1]),               d1  },
                                {  0,                                                  0,                                        0,                 1 }
                                };


                double mat2[4][4]= {
                                {  cosf(qi[2]),              -cosf(alfai[2])*sinf(qi[2]),               sinf(alfai[2])*sinf(qi[2]),    a2*cosf(qi[2]) },
                                {  sinf(qi[2]),               cosf(alfai[2])*cosf(qi[2]),              -sinf(alfai[2])*cosf(qi[2]),    a2*sinf(qi[2]) },
                                {  0,                                     sinf(alfai[2]),                           cosf(alfai[2]),               d2  },
                                {  0,                                                  0,                                        0,                 1 }
                                };


                double mat3[4][4]= {
                                {  cosf(qi[3]),              -cosf(alfai[3])*sinf(qi[3]),               sinf(alfai[3])*sinf(qi[3]),    a3*cosf(qi[3]) },
                                {  sinf(qi[3]),               cosf(alfai[3])*cosf(qi[3]),              -sinf(alfai[3])*cosf(qi[3]),    a3*sinf(qi[3]) },
                                {  0,                                     sinf(alfai[3]),                           cosf(alfai[3]),               d3  },
                                {  0,                                                  0,                                        0,                 1 }
                                };


                double mat4[4][4]= {
                                {  cosf(qi[4]),              -cosf(alfai[4])*sinf(qi[4]),               sinf(alfai[4])*sinf(qi[4]),    a4*cosf(qi[4]) },
                                {  sinf(qi[4]),               cosf(alfai[4])*cosf(qi[4]),              -sinf(alfai[4])*cosf(qi[4]),    a4*sinf(qi[4]) },
                                {  0,                                     sinf(alfai[4]),                           cosf(alfai[4]),               d4  },
                                {  0,                                                  0,                                        0,                 1 }
                                };


                double mat5[4][4]= {
                                {  cosf(qi[5]),              -cosf(alfai[5])*sinf(qi[5]),               sinf(alfai[5])*sinf(qi[5]),    a5*cosf(qi[5]) },
                                {  sinf(qi[5]),               cosf(alfai[5])*cosf(qi[5]),              -sinf(alfai[5])*cosf(qi[5]),    a5*sinf(qi[5]) },
                                {  0,                                     sinf(alfai[5]),                           cosf(alfai[5]),               d5  },
                                {  0,                                                  0,                                        0,                1 } 
                                };



                //printf("HERE 1");
                double result[4][4];
                mul_matrix((double*)mat1, (double*)mat2, 4, 4, 4, 4, (double*)result);

                double result1[4][4];
                mul_matrix((double*)result, (double*)mat3, 4, 4, 4, 4, (double*)result1);

                double result2[4][4];
                mul_matrix((double*)result1, (double*)mat4, 4, 4, 4, 4, (double*)result2);

                //float result3[4][4];
                mul_matrix((double*)result2, (double*)mat5, 4, 4, 4, 4, (double*)fk_result);

                
    //printf("\n Out of Forward Kinematics Function \n");      
    //printf(fk_result[0][0]);
    R[0][0] = fk_result[0][0];
    R[0][1] = fk_result[1][0];
    R[0][2] = fk_result[2][0];
    

    R[1][0] = fk_result[0][1];
    R[1][1] = fk_result[1][1];
    R[1][2] = fk_result[2][1];
    

    R[2][0] = fk_result[0][2];
    R[2][1] = fk_result[1][2];
    R[2][2] = fk_result[2][2];
    

    x = fk_result[0][3];
    y = fk_result[1][3];
    z = fk_result[2][3];

    if(print_trMat == 1) {
        printf("\n");
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                printf("%f",fk_result[i][j]);
                printf("\t");
            }
            printf("\n");
        }
    }

}



void calculateJacobian_WRONG(double* jointAngles, double(&J)[6][5]) {
    const double delta = 1e-3;

    double x, y, z;
    double R[3][3];

    forwardKinematics(jointAngles, R, x, y, z);

    double x_delta, y_delta, z_delta;
    double R_delta[3][3];

    
    double jointAngles_delta[5];
    for (int j = 0; j < 5; ++j) {
        //printf(" jointAngles[%d] = %f \t", j, jointAngles[j]*180.0/PI);
        jointAngles_delta[j] = jointAngles[j] + delta;
    }

    forwardKinematics(jointAngles_delta, R_delta, x_delta, y_delta, z_delta);
        //printf("\n");
    for (int i = 0; i < 5; ++i) {
        //jointAngles_delta[i] += delta;
        

        J[0][i] = (x_delta - x) / delta;
        J[1][i] = (y_delta - y) / delta;
        J[2][i] = (z_delta - z) / delta;

        // Calculate the angular part of the Jacobian
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                J[row + 3][i] = (R_delta[row][col] - R[row][col]) / delta;
            }
        }
    }
}

// Function to calculate the Jacobian matrix
void calculateJacobian(double* jointAngles, double(&J)[6][5]) {
    const double delta = 1e-3;

    double x, y, z;
    double R[3][3];

    forwardKinematics(jointAngles, R, x, y, z);

    double x_delta, y_delta, z_delta;
    double R_delta[3][3];

    for (int i = 0; i < 5; ++i) {
        double jointAngles_delta[5];
        for (int j = 0; j < 5; ++j) {
            //printf(" jointAngles[%d] = %f \t", j, jointAngles[j]*180.0/PI);
            jointAngles_delta[j] = jointAngles[j];
        }
        //printf("\n");

        jointAngles_delta[i] += delta;
        forwardKinematics(jointAngles_delta, R_delta, x_delta, y_delta, z_delta);

        J[0][i] = (x_delta - x) / delta;
        J[1][i] = (y_delta - y) / delta;
        J[2][i] = (z_delta - z) / delta;

        // Calculate the angular part of the Jacobian
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                J[row + 3][i] = (R_delta[row][col] - R[row][col]) / delta;
            }
        }
    }
}

// Function to calculate inverse kinematics using the Newton-Raphson method
void inverseKinematics(double* jointAngles) {
    double x, y, z;
    double R[3][3];
    double error = 1.0;

    /*
    printf("Input to IK\n");
    for (int j = 0; j < 5; ++j) {
        printf("jointAngles[%d] = %f \t", j, jointAngles[j]*180.0/PI);
    }
    printf("\n");
    */
    int iteration = 0;
    while (error > 0.2) {
        /*
        printf("In while loop of IK\n");
        // Calculate forward kinematics
        for (int j = 0; j < 5; ++j) {
            printf("jointAngles[%d] = %f \t", j, jointAngles[j]*180.0/PI);
        }
        printf("\n");
        */
        forwardKinematics(jointAngles, R, x, y, z);

        /*
        printf("\n");
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                printf("%f ", R[i][j]);
                printf("\t");
            }
            printf("\n");
        }
        printf("x = %f y = %f z = %f \n", x, y, z);
        */


        // Calculate the error in position
        double dx = target_x - x;
        double dy = target_y - y;
        double dz = target_z - z;
        
        //printf("dx = %f dy = %f dz = %f \n", dx, dy, dz);


        // Calculate the error in orientation
        double dR[3][3] = {
            {target_R[0][0] - R[0][0], target_R[0][1] - R[0][1], target_R[0][2] - R[0][2]},
            {target_R[1][0] - R[1][0], target_R[1][1] - R[1][1], target_R[1][2] - R[1][2]},
            {target_R[2][0] - R[2][0], target_R[2][1] - R[2][1], target_R[2][2] - R[2][2]}
        };

        // Check for convergence
        error = std::sqrt(dx * dx + dy * dy + dz * dz);
        //printf("Error Eucledian Distance %f \n", error);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                error += std::abs(dR[i][j]);
               // printf("Error added %f \n", error);
            }
        }

        //printf("Error added %f \n", error);

        if (error < 1e-6) {
            std::cout << "Converged! Error: " << error << std::endl;
            break;
        }

        // Calculate the Jacobian matrix
        double J[6][5];
        calculateJacobian(jointAngles, J);
        /*
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                printf(" J[%d][%d] = %f \t", i,j, J[i][j]);
            }
            printf("\n");
        }
        */
        // Calculate the change in joint angles using the pseudo-inverse
        double deltaTheta[5];
        double JT_J[5][5], inv_JT_J[5][5];

        // Calculate JT * J
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                JT_J[i][j] = 0;
                for (int k = 0; k < 6; ++k) {
                    JT_J[i][j] += J[k][i] * J[k][j];
                }
            }
        }

        // Calculate the pseudo-inverse (using a simple pseudo-inverse method)
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                inv_JT_J[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }

        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                inv_JT_J[i][j] /= JT_J[i][j];
            }
        }

        // Calculate deltaTheta
        for (int i = 0; i < 5; ++i) {
            deltaTheta[i] = 0;
            for (int j = 0; j < 5; ++j) {
                deltaTheta[i] += inv_JT_J[i][j] * J[j][i];
            }
            //deltaTheta[i] *= 0.1;  // Scale the step size
            //deltaTheta[i] *= 50;  // Scale the step size
        }

        //printf("\n");
        // Update joint angles
        /*
        for (int i = 0; i < 5; ++i) {
            
            if (!std::isnan(deltaTheta[i]) && !std::isinf(deltaTheta[i])) {
                jointAngles[i] += deltaTheta[i];
            } else {
                //std::cerr << "NaN or Inf detected. Stopping iterations." << std::endl;
                //return;
                //printf("\nNan or Inf detected %d %f \n", i, deltaTheta[i]);
                jointAngles[i] = jointAngles[i];
            }
            
            //jointAngles[i] += deltaTheta[i];
            //printf(" joinAngles[%d] = %f ", i, jointAngles[i]);
        }
        */

        
        // Update joint angles
        for (int i = 0; i < 5; ++i) {
            if (!std::isnan(deltaTheta[i]) && !std::isinf(deltaTheta[i])) {
                jointAngles[i] += jointWeights[i] * deltaTheta[i];
            } else {
                jointAngles[i] = jointAngles[i];
            }
        }

        


        //printf("\n");
        iteration++;

        if (iteration % 1 == 0){
            printf("Iteration %d; Error %f \n", iteration , error);
            for (int i = 0; i < 5; ++i) {
                 printf(" joinAngles[%d] = %f ", i, jointAngles[i]);       
            }
            printf("\n");
        }
    }

    // Print the final joint angles
    std::cout << "Final Joint Angles: ";
    for (int i = 0; i < 5; ++i) {
        std::cout << jointAngles[i]*180.0/PI << " ";
    }
    std::cout << std::endl;
    forwardKinematics(jointAngles, R, x, y, z, 1);

}



int main() {
    // Initial guess for joint angles
    //double jointAngles[5] = {0.0, -110.0*PI/180.0, 110.0*PI/180.0, -70.0*PI/180.0, 0.0};
    //double jointAngles[5] =   {0.0,     -PI/2,    PI/2.0,      -PI/2.0, 0.0};
    //double jointAngles[5] = {0.0, -1.442578, -0.976651, 0.848101, 0.0};
    //double jointAngles[5] = {0.0, -0.224374, -2.194969, 0.848075, 0.0};
    //double jointAngles[5] = {0.0, -0.327065, -2.092312, 0.848075, 0.0};
    
    //double jointAngles[5] =   {0.0,     -PI/2,    PI/2.0,          0.0, 0.0};
    //double jointAngles[5] =   {0.0,     0.0,    0.0,          0.0, 0.0};
    //double jointAngles[5] =   {0.0,     -112.89*PI/180.0,    99.38*PI/180.0,          -20.23*PI/180.0, 0.0};
    
    double jointAngles[5] =   {0.0,     -PI/2-0.01,    PI/2.0,      -PI/2.0, 0.0};
    // Perform inverse kinematics
    inverseKinematics(jointAngles);

    return 0;
}
