//
//  ckp_Matrix.hpp
//  MatrixTest
//
//  Created by Chad Paik on 2018-02-26.
//  Copyright © 2018 Chad Paik. All rights reserved.
//

#ifndef ckp_Matrix_hpp
#define ckp_Matrix_hpp

#include <stdio.h>


class Matrix
{
    
    //IMPORTANT: THE row and col VARIABLES ARE ACTUAL SIZES, BUT THE INDEXES START FROM 0! THIS MEANS THAT THE LAST INDEX IS THE SIZE-1
    //The matrix is built upon floats.
private:
    float** mat;
    int row;
    int col;
    
    //Constructors and Destructors
public:
    // Default constructor
    Matrix();
    
    //constructor
    Matrix(const int n_row, const int n_col);
    
    //This destructor was giving some trouble. Commented out for now
    //    ~Matrix()
    //    {
    //        // clean up allocated memory
    //        for (int i = 0; i < row; i++)
    //        {
    //            delete mat[i];
    //        }
    //        delete mat;
    //        mat = NULL;
    //    }
    
    
    //Operator and Arithematic Methods
    //Operator Methods
public:
    //Index operator to access the index give to it by float value = Matrix(i,j)
    //INDEX STARTS FROM 0
    
    //Used for getting/setting the value of the matrix at the index i,j. Again, remember that index starts from zero.
    //Use it like: float number = MatrixA(row_index, col_index)
    float& operator()(const int i, const int j);
    
    //Comparison. It compares the sizes, and if the sizes are consistent, it checks for equality of all indices
    bool operator==(const Matrix & right);
    
    //Matrix Assignment. MatrixA = MatrixB means MatrixA turns into MatirxB. The Left matrix turns into Right matrix
    Matrix& operator= (const Matrix& right);
    
    
    //Matrix Addition Left(current) + Right(another matrix)
    // [Left]+[Right] = [Result]
    Matrix operator+(const Matrix& right)const;
    
    
    // subtraction of Matrix with Matrix
    //[Left]-[Right] = Result
    Matrix operator- (const Matrix& right);
    
    // operator multiplication
    //[Left]*[Right] = [Result]
    //Important that the inner dimension must match
    Matrix operator* (const Matrix& right);
    
    //Literally swaps rows. It does not return anything, rather it changes the row of the matrix you use.
    //For ex,  MatrixA.rowSwap(1,2) it simply swaps second row and third row(index starting from zero) of MatrixA.
    void rowSwap(int row1, int row2);
    
    //Arithmetic Methods
    
    //Gaussian Elimination
    void gElim();
    
    //Inverse
    Matrix Inv();
    
    //Ir solves the system of linear equation. The matrix should be a square matrix, and should have unique solution.
    //The parameter is the b vector.  Ax = b
    Matrix solve(Matrix & b);
    
    //Forms an augumented matrix. It basically returns a matrix that joins two matrices and turns it into 1 matrix.
    Matrix augMatrix(const Matrix & right);
    
    //Forms matrix without the selected row(i) and col(j)
    Matrix Minor(int i, int j);
    
    //Computes the Determinant. Returns Float
    float Det();
    
    //It fills the matrix with the array that is given. The array is 1D, but it takes care of the filling
    void fillMatrix(int rows, int cols, float values[]);
    
    //Elementwise constant value addition. Adds the constant given to all the values in the matrix
    //It does not return a matrix, but it changes the matrix that you are using
    //Use negative value if you want to subtract
    Matrix& addConst(const float value);
    
    // multiply a double value (elements wise)
    //It does not return a matrix, but it changes the matrix that you are using
    //For division just enter 1/value as parameter
    Matrix& multConst(const float value);
    
    //Prints out the matrix. Used for debugging
    void print() const;
    
    //Accessor Methods
public:
    // returns the number of rows
    int rows() const;
    
    // returns the number of columns
    int cols() const;
    
    
};

//EYEdentitiy matrix similar to matlab
Matrix eye(const int dim);

//Matrix with only ones as its entry
Matrix ones(const int n_row, const int n_col);



#endif /* ckp_Matrix_hpp */
