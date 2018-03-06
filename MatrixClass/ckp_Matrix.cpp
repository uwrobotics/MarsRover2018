//
//  ckp_Matrix.cpp
//  MatrixTest
//
//  Created by Chad Paik on 2018-02-26.
//  Copyright © 2018 Chad Paik. All rights reserved.
//

#include "ckp_Matrix.hpp"
#include <iostream>
#include <stdlib.h>
#include <math.h>

using namespace std;

Matrix::Matrix()
    {
        mat = NULL;
        row = 0; //Number of rows
        col = 0; //Number of cols
    }
    
    //constructor
Matrix::Matrix(const int n_row, const int n_col)
    {
        //mat is the actual array of matrix
        mat = NULL;
        
        if (n_row > 0 && n_col > 0)
        {
            this->row = n_row;
            this->col = n_col;
            
            mat = new float* [row]; //creating 1D array with the number of rows
            for (int i = 0; i < row; i++)
            {
                mat[row] = new float[col]; //creating the columns
            }
            for (int i = 0; i<row; i++)
            {
                mat[i] = new float[col];
                for (int j = 0; j<col; j++)
                {
                    mat[i][j] = 0.0; //remember, 2D array is array[#ofRows][#ofCols]
                }
            }
        }
        else
            cout<<"Cannot have negative index numbers!" << endl;
    }
    
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

    //Index operator to access the index give to it by float value = Matrix(i,j)
    //INDEX STARTS FROM 0
float& Matrix::operator()(const int i, const int j)
    {
        if ((mat != NULL) && (i >= 0) && (i <= row) && (j >= 0) && (j <= col))
            //If matrix exists, if
        {
            return mat[i][j];
        }
        else
        {
            cout<<"The Index does not exist!"<<endl;
            return mat[0][0];
        }
    }
    
    
bool Matrix::operator==(const Matrix & right){
        if (this->row == right.row && this->col == right.col){
            for (int i = 0; i<row; i++)
            {
                for (int j = 0; j-col; j++)
                {
                    if(this->mat[i][j] != right.mat[i][j])
                        return false;
                }
            }
        }
        else
            return false;
        
        return true;
    }
    
    //Matrix Assignment
Matrix& Matrix::operator= (const Matrix& right)
    {
        this->row = right.row;
        this->col = right.col;
        this->mat = new float*[right.row];
        for (int i = 0; i < right.row; i++)
        {
            mat[i] = new float[right.col];
            
            // copy the values from the matrix a
            for (int j = 0;  j< right.col; j++)
            {
                mat[i][j] = right.mat[i][j];
            }
        }
        return *this;
    }
    
    
    //Matrix Addition Left(current) + Right(another matrix)
    // [Left]+[Right] = [Result]
Matrix Matrix::operator+(const Matrix& right)const {
        // checking if dimensions match
        if (this->row == right.row && this->col == right.col){
            Matrix Result(this->row,this->col);
            for (int i = 0; i<row; i++)
            {
                for (int j = 0; j-col; j++)
                {
                    Result.mat[i][j] = this->mat[i][j] + right.mat[i][j];
                }
            }
            return Result;
        }
        // Returned empty matrix because xCode was being anal
        cout<<"Cannot add since the dimensions dont match up!"<<endl;
        return Matrix();
    }
    
    
    // subtraction of Matrix with Matrix
Matrix Matrix::operator- (const Matrix& right){
        // checking if dimensions match
        if (this->row == right.row && this->col == right.col){
            Matrix Result = Matrix(this->row,this->col);
            for (int i = 0; i<row; i++)
            {
                for (int j = 0; j-col; j++)
                {
                    Result.mat[i][j] = this->mat[i][j] - right.mat[i][j];
                }
            }
            return Result;
        }
        // Returned empty matrix because xCode was being anal
        cout<<"Cannot add since the dimensions dont match up!"<<endl;
        return Matrix();
    }
    
    // operator multiplication
Matrix Matrix::operator* (const Matrix& right){
        //checking inner dimensions
        if (this->col == right.row)
        {
            Matrix Result= Matrix(this->row, right.col);
            
            for (int i = 0; i < this->row; i++)
            {
                for (int colResult = 0; colResult < right.col; colResult++)
                {
                    for (int j = 0; j < this->col; j++)
                    {
                        Result.mat[i][colResult] += this->mat[i][j] * right.mat[j][colResult];
                    }
                }
            }
            return Result;
        }
        else
            cout<<"the innner dimensions do not match up!"<<endl;
        
        // Returned empty matrix because xCode was being anal Not sure if we can get away with not returning something if error happens
        return Matrix();
    }
    
    
    
void Matrix::rowSwap(int row1, int row2)
    {
        if(row1 > (this->row-1) || row2 > (this->row-1)){
            cout<<"Selected row(s) does not exist!!"<<endl;
            return;
        }
        float* tmp = new float [row]; //creating 1D array with the number of rows
        tmp = mat[row1];
        mat[row1] = mat[row2];
        mat[row2] = tmp;
    }
    
    //Arithmetic Methods
void Matrix::gElim()
    {
        int i = 0;
        int j = 0;
        for (i = 0; i<(this->row-1); i++) //i is the current row
        {
            int prevLeadingCo = this->mat[i][i];
            //swapping rows to have row with largest first element to be the pivot
            for(j = (i+1); j<this->row; j++)
            {
                int currentLeadingCo = this->mat[j][i];
                if(abs(currentLeadingCo) > abs(prevLeadingCo)){
                    this->rowSwap(i, j);
                    prevLeadingCo = currentLeadingCo;
                }
            }
            for(int k = (i+1); k<this->row; k++)
            {
                float f = this->mat[k][i]/this->mat[i][i];
                for(int c = i; c<this->col; c++)
                {
                    this->mat[k][c] = this->mat[k][c] - f*this->mat[i][c];
                }
            }
            //uncomment for debugging
            //this->print();
        }
        return;
    }

//Matrix Matrix::Minor(int i, int j){
//    
//}
//
//float Matrix::Det(){
//    
//}

Matrix Matrix::Inv(){
    if(this->row != this->col){
        cout<<"Not invertible!"<<endl;
        return Matrix();
    }
    Matrix Original = Matrix(this->row, this->col);
    Matrix Result = eye(this->row);
    Original = *this;
    int i = 0;
    int j = 0;
    for (i = 0; i<(Original.row-1); i++) //i is the current row
    {
        int prevLeadingCo = Original.mat[i][i];
        //swapping rows to have row with largest first element to be the pivot
        for(j = (i+1); j<Original.row; j++)
        {
            int currentLeadingCo = Original.mat[j][i];
            if(abs(currentLeadingCo) > abs(prevLeadingCo)){
                Original.rowSwap(i, j);
                Result.rowSwap(i, j);
                prevLeadingCo = currentLeadingCo;
            }
        }
        
        //Making it into Upper Triangular
        for(int k = (i+1); k<Original.row; k++)
        {
            float f = Original.mat[k][i]/Original.mat[i][i];
            for(int c = 0; c<Original.col; c++)
            {
                Original.mat[k][c] = Original.mat[k][c] - f*Original.mat[i][c];
                Result.mat[k][c] = Result.mat[k][c] - f*Result.mat[i][c];
            }
        }
        //uncomment for debugging
        //Result.print();
    }
    
    for(i = Original.row-1; i>=0; i--)
    {
        for(int k = (i-1); k>=0; k--)
        {
            float f = Original.mat[k][i]/Original.mat[i][i];
            for(int c = Original.col-1; c>=0; c--)
            {
                Original.mat[k][c] = Original.mat[k][c] - f*Original.mat[i][c];
                Result.mat[k][c] = Result.mat[k][c] - f*Result.mat[i][c];
            }
        }
        //Result.print();
    }
    for (int i = 0; i<Result.row; i++)
    {
        float factor = Original.mat[i][i];
        for (int j = 0; j<Result.col; j++)
        {
            Result.mat[i][j] = Result.mat[i][j]/factor;
            Original.mat[i][j] = Original.mat[i][j]/factor;
        }
    }
    //Original.print();
    return Result;
}
    
Matrix Matrix::solve(Matrix & b){
        Matrix Solution = Matrix(this->row, 1);
        Matrix AugMatrix;
        AugMatrix = this->augMatrix(b);
        AugMatrix.gElim();
        AugMatrix.print();
        //this is to get the actual index of row/column. The row/col variable is the size, but the index starts from 0
        int mat_row = AugMatrix.row-1;
        int mat_col = AugMatrix.col-1;
        int i = 0;
        float solved = 0;
        
        //Back Substitution
        Solution(mat_row, 0) = AugMatrix.mat[mat_row][mat_col]/AugMatrix.mat[mat_row][mat_col-1];
        for(i = mat_row-1; i>=0; i--)
        {
            for(int j = mat_row; j>i; j--)
            {
                solved+= Solution(j,0)*AugMatrix.mat[i][j];
            }
            Solution(i,0) =(AugMatrix.mat[i][mat_col] - solved )/AugMatrix.mat[i][i];
            solved = 0;
        }
        return Solution;
    }
    
Matrix Matrix::augMatrix(const Matrix & right)
    {
        if(this->row != right.row)
        {
            cout<<"Cannot join matrices with different row size!"<<endl;
            return Matrix();
        }
        //New Dimension
        int newCol = this->col + right.col;
        Matrix Result = Matrix(this->row, newCol);
        
        for (int i=0; i<this->row; i++)
        {
            for(int j=0; j<this->col; j++)
            {
                Result.mat[i][j] = this->mat[i][j];
            }
            for(int k=0; k<right.col; k++)
            {
                Result.mat[i][k+this->col] = right.mat[i][k];
            }
        }
        return Result;
    }
    
void Matrix::fillMatrix(int rows, int cols, float values[])
    {
        //Checking for size is  done as a safety factor in case people input the wrong dimension.
        
        if (this->row != rows || this->col != cols){
            cout<<"The current dimension and input dimension does not match! Please check again"<<endl;
            return;
        }
        int size = rows*cols;
        int row_index = 0;
        int col_index = 0;
        for (int i = 0; i<size; i++){
            if(i%cols == 0){
                row_index = i/cols;
                col_index = 0;
            }
            this->mat[row_index][col_index] = values[i];
            col_index++;
        }
    }
    
    //Elementwise constant value addition. Addes the constant given to all the values in the matrix
    //Use negative value if you want to subtract
Matrix& Matrix::addConst(const float value)
    {
        for (int i = 0; i < row; i++)
        {
            for (int j = 0; j < col; j++)
            {
                mat[i][j] += value;
            }
        }
        return *this;
    }
    
    // multiply a double value (elements wise)
    //For division just enter 1/value as parameter
Matrix& Matrix::multConst(const float value){
        for (int i = 0; i < row; i++)
        {
            for (int j = 0; j < col; j++)
            {
                mat[i][j] *= value;
            }
        }
        return *this;
    }
    
    
    //Accessor Methods
    // returns the number of rows
int Matrix::rows() const
    {
        return row;
    }
    
    // returns the number of columns
int Matrix::cols() const
    {
        return col;
    }
    
void Matrix::print() const
    {
        cout<<"[";
        for (int i = 0; i<row; i++){
            if (i > 0)
                cout<<endl;
            for (int j = 0; j<col; j++){
                cout<<" "<<mat[i][j]<<",";
            }
        }
        cout<<"]"<<endl;
    }
    

//EYEdentitiy matrix similar to matlab
Matrix eye(const int dim){
    Matrix Result = Matrix(dim,dim);
    
    for (int i = 0; i<dim; i++)
    {
        Result(i,i) = 1;
    }
    return Result;
}

//Matrix with only ones as its entry
Matrix ones(const int n_row, const int n_col)
{
    Matrix Result = Matrix(n_row, n_col);
    
    for (int i = 0; i <= n_row; i++)
    {
        for (int j = 0; j <= n_col; j++)
        {
            Result(i,j) = 1;
        }
    }
    return Result;
}

