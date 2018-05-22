#ifndef IK_h
#define IK_h

#include "ckp_Matrix.h"
#include <cmath>
const float pi = 3.1415926535897932;


//Solving for turntable
float solve_for_theta0(float q_turntable, float w_turntable, float dt);//w is omega

//Solving for shoulder
float solve_for_theta1(Matrix p, Matrix a, Matrix q);


//Solving for elbow
float solve_for_theta2(Matrix p, Matrix a, Matrix q);


float solve_for_theta3(Matrix q, Matrix w, float psi, bool isConst, float dt);


float solve_for_psi(Matrix q);



//simple link_solve equation for 2 link manipulator. For more advanced systems, DH coordinate usage is recommended
Matrix link_solve(Matrix q, Matrix a);

#endif





















//
//Matrix DH_Solve(Matrix q, Matrix a, Matrix r, Matrix d)
//{
//    Matrix A= eye(4);
//    Matrix A_i = Matrix(4,4);
//
//    for (int i = 0; i<length(q); i++)
//    {
//        float tmp[16]= {cos(q(i,0)),  -cos(a(i,0))*sin(q(i,0)),   sin(a(i,0))*sin(q(i,0)),   r(i,0)*cos(q(i,0)),
//            sin(q(i,0)),   cos(a(i,0))*cos(q(i,0)),  -sin(a(i,0))*cos(q(i,0)),   r(i,0)*sin(q(i,0)),
//            0,              sin(a(i,0)),             cos(a(i,0)),             d(i,0),
//            0,         0,                     0,              1};
//        A_i.fillMatrix(4,4,tmp);
//        A = A*A_i;
//    }
//    return A;
//}
//
//Matrix link_solve(Matrix q, Matrix alpha, Matrix a, Matrix d)
//{
//    Matrix A= eye(4);
//    Matrix p = Matrix(3,1);
//    Matrix A_i = Matrix(4,4);
//    for (int i = 0; i< q.rows(); i++)
//    {
//        float tmp[16]= {cos(q(i,0)),  -cos(alpha(i,0))*sin(q(i,0)),   sin(alpha(i,0))*sin(q(i,0)),   a(i,0)*cos(q(i,0)),
//            sin(q(i,0)),   cos(alpha(i,0))*cos(q(i,0)),  -sin(alpha(i,0))*cos(q(i,0)),   a(i,0)*sin(q(i,0)),
//            0,              sin(alpha(i,0)),             cos(alpha(i,0)),             d(i,0),
//            0,         0,                     0,              1};
//        rotation.fillMatrix(4,4,tmp);
//        A = A*A_i;
//    }
//    p(0,0) = (0,3);
//    p(1,0) = (1,3);
//    p(2,0) = (2,3);
//
//    return p;
//}
