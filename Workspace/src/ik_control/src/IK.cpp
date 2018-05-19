#include "ckp_Matrix.hpp"
#include <cmath>




Matrix DH_Solve(Matrix q, Matrix a, Matrix r, Matrix d)
{
    Matrix A= eye(4);
    Matrix rotation = Matrix(4,4);

    for (int i = 0; i<length(q); i++)
    {

        float tmp[16]= {cos(q(i,0)),  -cos(a(i,0))*sin(q(i,0)),   sin(a(i,0))*sin(q(i,0)),   r(i,0)*cos(q(i,0)),
             sin(q(i,0)),   cos(a(i,0))*cos(q(i,0)),  -sin(a(i,0))*cos(q(i,0)),   r(i,0)*sin(q(i,0)),
             0,              sin(a(i,0)),             cos(a(i,0)),             d(i,0),
            0,         0,                     0,              1};
        rotation.fillMatrix(4,4,tmp);
        A = A*rotation;
    }
    return A;
}

Matrix link_solve(Matrix q, Matrix a, Matrix r, Matrix d)
{
    Matrix A= eye(4);
    Matrix XYZ = Matrix(length(q), 3);
    Matrix rotation = Matrix(4,4);
    for (int i = 0; i<length(q); i++)
    {

        float tmp[16]= {cos(q(i,0)),  -cos(a(i,0))*sin(q(i,0)),   sin(a(i,0))*sin(q(i,0)),   r(i,0)*cos(q(i,0)),
             sin(q(i,0)),   cos(a(i,0))*cos(q(i,0)),  -sin(a(i,0))*cos(q(i,0)),   r(i,0)*sin(q(i,0)),
             0,              sin(a(i,0)),             cos(a(i,0)),             d(i,0),
            0,         0,                     0,              1};
        rotation.fillMatrix(4,4,tmp);
        A = A*rotation;
        for (int j = 0; j<XYZ.cols(); j++)
        {
          XYZ(i+1,j) = A.Transpose()(3,j)
        }
    }
    return XYZ;
}

float solve_for_theta2(Matrix x, Matrix r, float q)
{
  //x is 3x1, r is 3x1, q is a number
  float R = x(0,1);
  float h = x(0,2);
  float xy = sqrt(R*R + h*h);

  beta = atan2(h,R);
  gamma = (acos())
}
