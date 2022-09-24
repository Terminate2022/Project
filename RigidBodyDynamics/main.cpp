#include <iostream>
#include <vector>
#include <time.h>
#include <math.h>
#include<string.h>
#include<bits/stdc++.h>
using namespace std;

namespace initStatesParams {
float
        pS[1][3] ={{0,0,0}},//central position of the Service Satellite
        rS[1][3] = {{0.15,0.15,0.15}}, //half length of the Service Cube
        mSS = 12,//12 kg, mass of the service satellite
        wS[1][3] = {{0,0,0}},//angular velocity
        pvS[1][3] = {{0,0,0}},//initial linear velocity of the Cube
        LS[1][3]={{rS[1-1][1-1]*2,rS[1-1][2-1]*2,rS[1-1][3-1]*2}},//x,y,z length of the target
        JcS[3][3],
        EulerParameters[]={1,0,0,0},
        R10[13],
        RS[4][4],
        R2S[4][4];
}

void initStates() {
    memset(initStatesParams::JcS,0,sizeof(initStatesParams::JcS));
    initStatesParams::JcS[1-1][1-1]=
           ((initStatesParams::mSS)*(initStatesParams::LS[1-1][2-1]*initStatesParams::LS[1-1][2-1]
            +initStatesParams::LS[1-1][3-1]*initStatesParams::LS[1-1][3-1]))/12;
    initStatesParams::JcS[2-1][2-1]=
            ((initStatesParams::mSS)*(initStatesParams::LS[1-1][1-1]*initStatesParams::LS[1-1][1-1]
            +initStatesParams::LS[1-1][3-1]*initStatesParams::LS[1-1][3-1]))/12;
    initStatesParams::JcS[3-1][3-1]=
            ((initStatesParams::mSS)*(initStatesParams::LS[1-1][1-1]*initStatesParams::LS[1-1][1-1]
            +initStatesParams::LS[1-1][2-1]*initStatesParams::LS[1-1][2-1]))/12;

    if(
            initStatesParams::EulerParameters[1-1]*initStatesParams::EulerParameters[1-1]
            +initStatesParams::EulerParameters[2-1]*initStatesParams::EulerParameters[2-1]
            +initStatesParams::EulerParameters[3-1]*initStatesParams::EulerParameters[3-1]
            +initStatesParams::EulerParameters[4-1]*initStatesParams::EulerParameters[4-1]!=1)
    {
         cout << "The initial conditions(EulerParameters) are wrong" << endl;
         return;
    }
    cout <<"pS  = ";
   for(int i=0;i<3;i++){
       cout <<" "<<initStatesParams::pS[0][i];
   }
   cout <<endl;

   cout <<"rS  = ";
  for(int i=0;i<3;i++){
      cout <<" "<<initStatesParams::rS[0][i];
  }
  cout <<endl;

  cout <<"LS  = ";
 for(int i=0;i<3;i++){
     cout <<" "<<initStatesParams::LS[0][i];
 }
 cout <<endl;

  cout <<"mSS = "<<initStatesParams::mSS<<endl;

  cout <<"wS  = ";
 for(int i=0;i<3;i++){
     cout <<" "<<initStatesParams::wS[0][i];
 }
 cout <<endl;

 cout <<"pvS  = ";
for(int i=0;i<3;i++){
    cout <<" "<<initStatesParams::pvS[0][i];
}
cout <<endl;

cout <<"EulerParameters  = ";
for(int i=0;i<4;i++){
   cout <<" "<<initStatesParams::EulerParameters[i];
}
cout <<endl;

  cout <<"JcS  = "<<endl;
 for(int i=0;i<3;i++){
     for (int j=0;j<3;j++){
         cout <<"   "<<initStatesParams::JcS[i][j];
     }
     cout <<endl;
 }
 cout <<endl;

    return;
}

void InitialConditions(){
memset(initStatesParams::R10,0,sizeof(initStatesParams::R10));
for(int i=0;i<4;i++){
    initStatesParams::RS[i][i]=initStatesParams::EulerParameters[1-1];
}
for(int i=1;i<4;i++){
    initStatesParams::RS[1-1][i]=-initStatesParams::EulerParameters[i];
}
initStatesParams::RS[2-1][3-1]=initStatesParams::EulerParameters[4-1];
initStatesParams::RS[2-1][4-1]=-initStatesParams::EulerParameters[3-1];
initStatesParams::RS[3-1][4-1]=initStatesParams::EulerParameters[2-1];
for(int i=1;i<4;i++)
{
    for(int j=0;i>j;j++)
    {
initStatesParams::RS[i][j]=-initStatesParams::RS[j][i];
    }
}
/*for(int i=0;i<4;i++){
    for (int j=0;j<4;j++){
        cout <<"   "<<initStatesParams::RS[i][j];
    }
    cout <<endl;
}
cout <<endl;*/
return;
}

namespace InitialConditions1 {
float wST[4][1]={{0},{initStatesParams::wS[0][0]},{initStatesParams::wS[0][1]},{initStatesParams::wS[0][2]}},
      EulerPdot[4][1];
}

namespace ProcessVariable {
int **c;

}
void InitMatrixRS(double a[], int x, int y){
    for(int j=0,i=0;(j<x)&(i<x*y);j++){

    for (int k=0; k<y ; i++,k++)
    {
        a[i]=initStatesParams::RS[j][k];
    }
    }
}

void InitMatrixwST(double a[], int x, int y){
    for(int j=0,i=0;j<x&i<x*y;j++){

    for (int k=0; k<y ; i++,k++)
    {
        a[i]=InitialConditions1::wST[j][k];
    }
    }
}

int **Multiply(double a[], double b[], int n, int k, int m){
    int **c = new int *[n]; // 创建动态二维数组c
    for(int i=0;i<n;i++){
        c[i]=new int[m];
    }
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            c[i][j] = 0;
            for (int v = 0; v < k; v++){
                c[i][j] += a[i*k+v] * b[v*m+j];
            }
        }
    }
    return c;
}

void disPlayEulerPdot(int **c, int n, int m){
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
              InitialConditions1::EulerPdot[i][j]=c[i][j];
        }

    }
}

void MatrixMultiplicationNum(int n, int k, int m){
    double a[n][k], b[k][m];
    InitMatrixRS(*a, n, k);
    InitMatrixwST(*b, k, m);
    ProcessVariable::c= Multiply(*a, *b, n, k, m);
    disPlayEulerPdot(ProcessVariable::c, n, m);
    return;
};

void InitialConditionsC(){
    MatrixMultiplicationNum(4,4,1);
    cout << "输出矩阵EulerPdot:" << endl;
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 1; j++){
            cout << InitialConditions1::EulerPdot[i][j] << ' ';
        }
        cout << endl;
    }
    return;
}
#if 0
#include<bits/stdc++.h>
using namespace std;

//初始化矩阵函数
void InitMatrix(int a[], int x, int y){
    cout << "输入矩阵的值：" << endl;
    for (int i = 0; i < x*y; i++){
        cin >> a[i];
    }
}

//矩阵相乘函数
int **Multiply(int a[], int b[], int n, int k, int m){
    int **c = new int *[n]; // 创建动态二维数组c
    for(int i=0;i<n;i++){
        c[i]=new int[m];
    }
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            c[i][j] = 0;
            for (int v = 0; v < k; v++){
                c[i][j] += a[i*k+v] * b[v*m+j];
            }
        }
    }
    return c;
}

//打印输出函数
void disPlay(int **c, int n, int m){
    cout << "输出矩阵c:" << endl;
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            cout << c[i][j] << ' ';
        }
        cout << endl;
    }
}

int main() {
    int n, k, m;
    cout << "请输入a矩阵的规模n,k：" << endl;
    cin >> n >> k;
    cout << "请输入b矩阵的列数m：" << endl;
    cin >> m;
    int a[n][k], b[k][m];
    int **c;
    InitMatrix(*a, n, k);
    InitMatrix(*b, k, m);
    c = Multiply(*a, *b, n, k, m);
    disPlay(c, n, m);
    getchar();
    getchar();
    return 0;
}
    temp;
    temp.mass = 0.5; temp.Ibody = temp.mass / 6.0 * Eigen::Matrix3d::Identity();
    temp.Ibodyinv = 6.0 / temp.mass * Eigen::Matrix3d::Identity();
    //
    temp.x = Eigen::Vector3d(0, 0, 0); temp.R = Eigen::Matrix3d::Identity();
    temp.P = Eigen::Vector3d(0, 0, 0); temp.L = Eigen::Vector3d(0, 0, 0);
    //
    temp.v = Eigen::Vector3d(0, 0, 0); temp.omega = Eigen::Vector3d(0, 0, 0);
    //
    temp.force = Eigen::Vector3d(0, 0, 0); temp.torque = Eigen::Vector3d(0, 0, 0);
    // 1
    Cubes.push_back(temp);
    // 2
    temp.x = Eigen::Vector3d(0, 1.5, 0);
    Cubes.push_back(temp);
    // 3
    temp.x = Eigen::Vector3d(1.5, 0, 0);
    Cubes.push_back(temp);
    if (numCubes >= 15) {
        temp.x = Eigen::Vector3d(-2.0, 0, -2.0);
        for (int i = 0; i < 10; i++) {
            Cubes.push_back(temp);
            temp.x[1] += 1.1;
        }
    }
    if (numCubes >= 25) {
        temp.x = Eigen::Vector3d(2.0, 0, -2.0);
        for (int i = 0; i < 10; i++) {
            Cubes.push_back(temp);
            temp.x[1] += 1.1;
        }
    }
    if (numCubes >= 35) {
        temp.x = Eigen::Vector3d(-2.0, 0, 2.0);
        for (int i = 0; i < 10; i++) {
            Cubes.push_back(temp);
            temp.x[1] += 1.1;
        }
    }
    if (numCubes >= 45) {
        temp.x = Eigen::Vector3d(2.0, 0, 2.0);
        for (int i = 0; i < 10; i++) {
            Cubes.push_back(temp);
            temp.x[1] += 1.1;
        }
    }
    // 4
    temp.x = Eigen::Vector3d(0.7, 3, 0);
    temp.R(0, 0) = sqrt(2) / 2.0; temp.R(1, 0) = sqrt(2) / 2.0; temp.R(2, 0) = 0.0;
    temp.R(0, 1) = -sqrt(2) / 2.0; temp.R(1, 1) = sqrt(2) / 2.0; temp.R(2, 1) = 0.0;
    Cubes.push_back(temp);
    // 5
    temp.x = Eigen::Vector3d(1.4, 5, 0);
    temp.R(0, 0) = sqrt(3) / 2.0; temp.R(1, 0) = 1.0 / 2.0; temp.R(2, 0) = 0.0;
    temp.R(0, 1) = -1.0 / 2.0; temp.R(1, 1) = sqrt(3) / 2.0; temp.R(2, 1) = 0.0;
    Cubes.push_back(temp);
}
#endif
int main()
{
    initStates();
    InitialConditions();
    InitialConditionsC();
   cout << "Hello World!" << endl;
    return 0;
}


