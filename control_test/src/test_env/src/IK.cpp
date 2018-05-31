#include "ckp_Matrix.h"
#include "IK.h"
#include <cmath>

using namespace std;
//Solving for turntable
float solve_for_theta0(float q_turntable, float w_turntable, float dt)//w is omega
{
    float qturntable_f = (w_turntable)*dt + q_turntable;
    if(qturntable_f < 0)
    {
        qturntable_f = 2*pi + qturntable_f;
    }
    else if (qturntable_f > 2*pi)
    {
      qturntable_f = qturntable_f-2*pi;
    }
    return qturntable_f;
}

//Solving for shoulder
float solve_for_theta1(Matrix p, Matrix a, Matrix q)
{
    //p is the position matrix
    float x = p(0,0);
    float y = p(1,0);

    float r = sqrt(x*x + y*y);

    float beta = acos(( (a(0,0)*a(0,0)) + r*r - (a(1,0)*a(1,0)) )/(2*a(0,0)*r) ); //always gonna be between zero and pi
    float gamma = atan2(y,x);
    //printf("ANGLES!: %f, %f", beta, gamma);
    float q1_1 = gamma - beta;
    float q1_2 = gamma + beta;

    if( abs(q(1,0)-q1_1) < abs(q(1,0)-q1_2) )
        return q1_1;
    else
        return q1_2;
}

//Solving for Elbow
float solve_for_theta2(Matrix p, Matrix a, Matrix q)
{
    //p is the position matrix
    float x = p(0,0);
    float y = p(1,0);

    float r = sqrt(x*x + y*y);
    float eta = acos( (a(0,0)*a(0,0) + a(1,0)*a(1,0) -r*r)/(2*a(0,0)*a(1,0)) ); //always gonna be between zero and pi
    float q2_1 = pi-eta;
    float q2_2 = eta - pi;

    if( abs(q(2,0)-q2_1) < abs(q(2,0)-q2_2))
    {
      if(q2_1 < 0){
        q2_1 = q2_1+(2*pi);
      }
      else if (q2_1 > 2*pi){
        q2_1 = q2_1-(2*pi);
      }
      return q2_1;
    }

    else
    {
      if(q2_2 < 0)
      {
        q2_2 = q2_2+(2*pi);
      }
      else if (q2_2 > 2*pi)
      {
        q2_2 = q2_2-(2*pi);
      }
      return q2_2;
    }
}

//Solving for Wrist Pitch
float solve_for_theta3(Matrix q, Matrix w, float psi, bool isConst, float dt)
{
    float q3_f = 0;
    if (isConst)//Solve so that the wrist stays at constant angle relative to the ground
    {
        //conversion from 0 to 2pi to -pi to pi
        if(q(1,0) > pi)
        {
            q(1,0) = -(2*pi-q(1,0));
        }

        if(q(2,0) > pi)
        {
            q(2,0) = -(2*pi-q(2,0)); //This one is more of the concern
        }

        q(3,0) = psi - q(1,0) - q(2,0);


        //conversion from -pi to pi to 0 to 2pi
        if(q(3,0) < 0)
        {
            q3_f = 2*pi + q(3,0);
        }
        else
        {
            q3_f = q(3,0);
        }
    }

    else //controlling of wrist
    {
        q3_f = w(3,0)*dt + q(3,0);
    }

    if(q3_f < 0)
      q3_f = q3_f+(2*pi);
    else if (q3_f > 2*pi)
      q3_f = q3_f-(2*pi);

    return q3_f;
}

float solve_for_psi(Matrix q)
{
    if(q(1,0) > pi)
    {
        q(1,0) = -(2*pi-q(1,0));
    }

    if(q(2,0) > pi)
    {
        q(2,0) = -(2*pi-q(2,0)); //This one is more of the concern
    }
    if(q(3,0) > pi)
    {
        q(2,0) = -(2*pi-q(3,0)); //This one is more of the concern
    }
    float psi = 0;
    psi = q(1,0) + q(2,0) + q(3,0);

    if(psi < 0)
      psi = psi+(2*pi);
    else if (psi > 2*pi)
      psi = psi-(2*pi);

    return psi;
}


//simple link_solve equation for 2 link manipulator. For more advanced systems, DH coordinate usage is recommended
Matrix link_solve(Matrix q, Matrix a)
{
    if(q(1,0) > pi)
    {
        q(1,0) = -(2*pi-q(1,0));
    }

    if(q(2,0) > pi)
    {
        q(2,0) = -(2*pi-q(2,0)); //This one is more of the concern
    }

    Matrix p = Matrix(3,1);
    p(0,0) = a(0,0)*cos(q(1,0)) + a(1,0)*cos(q(1,0)+q(2,0));
    p(1,0) = a(0,0)*sin(q(1,0)) + a(1,0)*sin(q(1,0)+q(2,0));
    p(2,0) = solve_for_psi(q);
    return p;
}

























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
