//
//  main.cpp
//  MatrixTest
//
//  Created by Chad Paik on 2018-02-22.
//  Copyright © 2018 Chad Paik. All rights reserved.
//

#include <iostream>
#include <stdlib.h>
#include "ckp_Matrix.hpp"
using namespace std;




int main(int argc, const char * argv[]) {
    
    //Testing the functionality of fillMatrix() Function
    float Fill[12] = {24,6,56.7,-453,3,5,15,62,46,234,63,26.7};
    Matrix testFill1 = Matrix(4,3);
    Matrix testFill2 = Matrix(3,4);
    Matrix testFill3 = Matrix(2,6);
    Matrix testFill4 = Matrix(6,2);
    Matrix testFill5 = Matrix(1,12);
    Matrix testFill6 = Matrix(12,1);
    Matrix testFill7 = Matrix(2,2);
    Matrix testFill8 = Matrix(4,4);
    
    cout<<"Testting fillMatrix(): "<< endl <<endl;
    //Testing different dimensions with same size
    testFill1.fillMatrix(4, 3, Fill);
    cout<<"testFill1: "<<endl;
    testFill1.print();
    
    testFill2.fillMatrix(3, 4, Fill);
    cout<<"testFill2: "<<endl;
    testFill2.print();
    
    testFill3.fillMatrix(2, 6, Fill);
    cout<<"testFill3: "<<endl;
    testFill3.print();
    
    testFill4.fillMatrix(6, 2, Fill);
    cout<<"testFill4: "<<endl;
    testFill4.print();
    
    testFill5.fillMatrix(1, 12, Fill);
    cout<<"testFill5: "<<endl;
    testFill5.print();
    
    testFill6.fillMatrix(12, 1, Fill);
    cout<<"testFill6: "<<endl;
    testFill6.print();
    
    //Testing matrix smaller than the array give
    testFill7.fillMatrix(2, 2, Fill);
    cout<<"testFill7: "<<endl;
    testFill7.print();
    //Testing matrix bigger than the array given
    testFill8.fillMatrix(4, 4, Fill);
    cout<<"testFill8: "<<endl;
    testFill8.print();
    
    cout<<endl<<endl<<"Testing Matrix Multiplication: "<<endl<<endl;
    //Testing Matrix Mutiplication
    Matrix test1 = Matrix(2,3);
    Matrix test2 = Matrix(3,4);
    
    for (int i =0; i<test1.rows(); i++){
        for (int j=0; j<test1.cols(); j++){
            test1(i,j) = rand() % 10;
        }
    }
    
    for (int i =0; i<test2.rows(); i++){
        for (int j=0; j<test2.cols(); j++){
            test2(i,j) = rand() % 10;
        }
    }
    test1.print();
    test2.print();
    Matrix test3 = test1*test2;
    test3.print();
    cout<<"The resulting dimension is: "<< test3.rows() << "x" << test3.cols() <<endl;
    //Testing Swapping Rows
    cout<<endl<<endl<<"Testing Row Swapping: "<<endl<<endl;
    test1.rowSwap(0, 1);
    test2.rowSwap(0, 2);
    test3.rowSwap(0, 2);
    test1.print();
    test2.print();
    test3.print();
    
    cout<<endl<<endl<<"Testing eye(),ones(), Matrix Addtion/Subtraction: "<<endl<<endl;
    //testing eye(), Ones(), and matrix addition/subtraction
    Matrix test4 = eye(5);
    Matrix test5 = ones(5,5);
    Matrix test6 = test4+test5;
    test6.print();
    test6 = test4-test5;
    test6.print();
    
    cout<<endl<<endl<<"Testing assignment operator, =:"<<endl<<endl;
    test4.print();
    test5.print();
    test4 = test5;
    test4.print();
    test5.print();
    
    
    cout<<endl<<endl<<"Testing augMatrix() "<<endl<<endl;
    //Testing Matrix Joining
    Matrix joined = test4.augMatrix(test5);
    joined.print();
    Matrix column = Matrix(5,1);
    Matrix aug = test5.augMatrix(column);
    aug.print();
    
    cout<<endl<<endl<<"Testing Matrix Multiplication/Addition with Const: "<<endl<<endl;
    //Testing addition/multiplying with constants
    Matrix Ones = ones(4,4);
    Ones.multConst(5.67);
    Ones.print();
    Ones.addConst(4.33);
    Ones.print();
    Ones.addConst(-10);
    Ones.print();
    
    cout<<endl<<endl<<"Testing GaussianElim: "<<endl<<endl;
    test2.rowSwap(0,2);
    test2.print();
    test2.gElim();
    test2.print();
    Matrix A = Matrix(5,5);
    Matrix b = Matrix(5,1);
    float values[25] = {-1,4,5,20,6,12,5,-6,-4,-2,9,5,6,7,2,100,23,452,52,-54,25,945,-64,6,1};
    float bvalues[5] = {67,34,234,24,-52};
    b.fillMatrix(5, 1, bvalues);
    A.fillMatrix(5, 5, values);
    Matrix Answer = A.solve(b);
    Answer.print();
    
    cout<<endl<<endl<<"Testing Inv: "<<endl<<endl;
    Matrix M = Matrix(5,5);
    float MValues[25] = {9,3,1,4,-5, 3,6,7,8,11, 4,5,6,7,8, 7,8,2,4,6, 89,3,2,7,0};
    M.fillMatrix(5, 5, MValues);
    M.print();
    Matrix Inv = M.Inv();
    Inv.print();
    
    return 0;
}

