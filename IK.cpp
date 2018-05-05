Matrix DH_Solve(Matrix q, Matrix a, Matrix r, Matrix d)
{
    Matrix A= eye(4);
    Matrix rotation = Matrix(4,4);

    for (int i = 0; i<q.rows(); i++)
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
