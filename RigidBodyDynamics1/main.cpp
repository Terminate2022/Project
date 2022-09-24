#include <iostream>
#include <vector>
#include <time.h>
#include <math.h>
#include<string.h>
#include<bits/stdc++.h>
#include <Eigen\Dense>
/*#include "stdafx.h"
#include <iostream>
#include<fstream>
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
    ofstream fout;           //创建ofstream
    fout.open("test.txt");   //关联一个文件
    fout << 12345 << endl;   //写入
    fout.close();            //关闭
    return 0;
}*/

using Eigen::MatrixXd;
using namespace std;
using namespace Eigen;


namespace initStatesParams {
double
        pS[3] ={0,0,0},//central position of the Service Satellite
        rS[3] = {0.15,0.15,0.15}, //half length of the Service Cube
        mSS = 10,//12 kg, mass of the service satellite
        wS[3] = {0,0,0},//angular velocity
        pvS[3] = {0,0,0},//initial linear velocity of the Cube
        LS[3]={rS[1-1]*2,rS[2-1]*2,rS[3-1]*2},//x,y,z length of the target
        EulerParameters[4]={1,0,0,0},
        wST[4]={0,initStatesParams::wS[0],initStatesParams::wS[1],initStatesParams::wS[2]},
        EulerPdot[4],
        wprimeST[4];
        Matrix3d JcS;
        Matrix4d RS,R2S;
        MatrixXd R10(1,13);
        Vector3d F(0,0,1);
        Vector3d MOSI(0,0,0);
        Vector3d MOS(0.5,0.5,0);
        MatrixXd RotMatrS(3,3);
}

namespace ProcessVariable {
Matrix4d met1,met2;
Vector4d u,v;
}

void initStates() {
    initStatesParams::JcS << 0, 0, 0,
                             0, 0, 0,
                             0, 0, 0;
    //initStatesParams::RotMatrS <<
    initStatesParams::JcS(0,0)=
           ((initStatesParams::mSS)*(initStatesParams::LS[1]*initStatesParams::LS[1]
            +initStatesParams::LS[2]*initStatesParams::LS[2]))/12;
    initStatesParams::JcS(1,1)=
            ((initStatesParams::mSS)*(initStatesParams::LS[0]*initStatesParams::LS[0]
            +initStatesParams::LS[2]*initStatesParams::LS[2]))/12;
    initStatesParams::JcS(2,2)=
            ((initStatesParams::mSS)*(initStatesParams::LS[0]*initStatesParams::LS[0]
            +initStatesParams::LS[1]*initStatesParams::LS[1]))/12;

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
       cout <<" "<<initStatesParams::pS[i];
   }
   cout <<endl;

   cout <<"rS  = ";
  for(int i=0;i<3;i++){
      cout <<" "<<initStatesParams::rS[i];
  }
  cout <<endl;

  cout <<"LS  = ";
 for(int i=0;i<3;i++){
     cout <<" "<<initStatesParams::LS[i];
 }
 cout <<endl;

  cout <<"mSS = "<<initStatesParams::mSS<<endl;

  cout <<"wS  = ";
 for(int i=0;i<3;i++){
     cout <<" "<<initStatesParams::wS[i];
 }
 cout <<endl;

 cout <<"pvS  = ";
for(int i=0;i<3;i++){
    cout <<" "<<initStatesParams::pvS[i];
}
cout <<endl;

cout <<"EulerParameters  = ";
for(int i=0;i<4;i++){
   cout <<" "<<initStatesParams::EulerParameters[i];
}
cout <<endl;

cout <<"JcS  = "<<endl;

cout <<initStatesParams::JcS << endl;

    return;
}

void InitialConditions(){
initStatesParams::RS <<  initStatesParams::EulerParameters[0], -initStatesParams::EulerParameters[1], -initStatesParams::EulerParameters[2], -initStatesParams::EulerParameters[3],
                         initStatesParams::EulerParameters[1],  initStatesParams::EulerParameters[0],  initStatesParams::EulerParameters[3], -initStatesParams::EulerParameters[2],
                         initStatesParams::EulerParameters[2], -initStatesParams::EulerParameters[3],  initStatesParams::EulerParameters[0],  initStatesParams::EulerParameters[1],
                         initStatesParams::EulerParameters[3],  initStatesParams::EulerParameters[2], -initStatesParams::EulerParameters[1],  initStatesParams::EulerParameters[0];

initStatesParams::R2S<<  initStatesParams::EulerParameters[0],  initStatesParams::EulerParameters[1],  initStatesParams::EulerParameters[2],  initStatesParams::EulerParameters[3],
                        -initStatesParams::EulerParameters[1],  initStatesParams::EulerParameters[0],  initStatesParams::EulerParameters[3], -initStatesParams::EulerParameters[2],
                        -initStatesParams::EulerParameters[2], -initStatesParams::EulerParameters[3],  initStatesParams::EulerParameters[0],  initStatesParams::EulerParameters[1],
                        -initStatesParams::EulerParameters[3],  initStatesParams::EulerParameters[2], -initStatesParams::EulerParameters[1],  initStatesParams::EulerParameters[0];
ProcessVariable::u<<initStatesParams::wST[0],initStatesParams::wST[1],initStatesParams::wST[2],initStatesParams::wST[3];
ProcessVariable::v=0.5*initStatesParams::RS*ProcessVariable::u;
for (int i=0; i<4; i++)
{initStatesParams::EulerPdot[i]=ProcessVariable::v(i);}
ProcessVariable::v=2*initStatesParams::R2S*ProcessVariable::v;
for (int i=0; i<4; i++)
{initStatesParams::wprimeST[i]=ProcessVariable::v(i);}
initStatesParams::R10<<  initStatesParams::pS[0],initStatesParams::pS[1],initStatesParams::pS[2],
                         initStatesParams::pvS[0],initStatesParams::pvS[1],initStatesParams::pvS[2],
                         initStatesParams::EulerParameters[0],initStatesParams::EulerParameters[1],initStatesParams::EulerParameters[2],initStatesParams::EulerParameters[3],
                         initStatesParams::wprimeST[1],initStatesParams::wprimeST[2],initStatesParams::wprimeST[3];
return;
}

namespace RK4Params {
double x0=0,xn=1, h=0.05;//此处x为时间t
MatrixXd k1(6,1), k2(6,1), k3(6,1), k4(6,1), k(6,1), yn(6,1);
MatrixXd y0(6,1);
int i, n=ceil((RK4Params::xn-RK4Params::x0)/RK4Params::h);
}

namespace RK4Params1 {
double x0=0,xn=1, h=0.05;//此处x为时间t
MatrixXd k1(4,1), k2(4,1), k3(4,1), k4(4,1), k(4,1), yn(7,1);
MatrixXd y0(7,1);
MatrixXd EulerPDiffs(4,4);
MatrixXd OmigaST(4,1);
MatrixXd OmigaS(3,1);
MatrixXd JXO(3,1);
MatrixXd OCRJOS(3,1);
int i, n=ceil((RK4Params::xn-RK4Params::x0)/RK4Params::h);
}

namespace RK4Params2 {
MatrixXd k1(3,1), k2(3,1), k3(3,1), k4(3,1), k(3,1);
MatrixXd kz(7,1);}

void RK4(){
    using namespace RK4Params;
    //RK4Params::n=ceil((RK4Params::xn-RK4Params::x0)/RK4Params::h);
    double X1[n],X2[n],X3[n],X4[n],X5[n],X6[n];
    cout<<"\nxn\tyn\n";
    cout<<"------------------\n";
    RK4Params::y0 << initStatesParams::R10(0),initStatesParams::R10(1),initStatesParams::R10(2),
                     initStatesParams::R10(3),initStatesParams::R10(4),initStatesParams::R10(5);
    //cout<<RK4Params::y0<<endl;
    cout<< x0<<"\t"<<RK4Params::y0(5)<<endl;//gai
    X1[0]=RK4Params::y0(0);
    X2[0]=RK4Params::y0(1);
    X3[0]=RK4Params::y0(2);
    X4[0]=RK4Params::y0(3);
    X5[0]=RK4Params::y0(4);
    X6[0]=RK4Params::y0(5);
    for(i=0; i < n; i++)
    {
      k1 << h*X4[i],h*X5[i],h*X6[i],
            h*initStatesParams::F(0)/initStatesParams::mSS,
            h*initStatesParams::F(1)/initStatesParams::mSS,
            h*initStatesParams::F(2)/initStatesParams::mSS;
      k2 << h*(X4[i]+0.5*k1(3)),h*(X5[i]+0.5*k1(4)),h*(X6[i]+0.5*k1(5)),
              h*initStatesParams::F(0)/initStatesParams::mSS,
              h*initStatesParams::F(1)/initStatesParams::mSS,
              h*initStatesParams::F(2)/initStatesParams::mSS;
      k3 << h*(X4[i]+0.5*k2(3)),h*(X5[i]+0.5*k2(4)),h*(X6[i]+0.5*k2(5)),
              h*initStatesParams::F(0)/initStatesParams::mSS,
              h*initStatesParams::F(1)/initStatesParams::mSS,
              h*initStatesParams::F(2)/initStatesParams::mSS;
      k4 << h*(X4[i]+k3(3)),h*(X5[i]+k3(4)),h*(X6[i]+k3(5)),
              h*initStatesParams::F(0)/initStatesParams::mSS,
              h*initStatesParams::F(1)/initStatesParams::mSS,
              h*initStatesParams::F(2)/initStatesParams::mSS;
      k = (k1+2*k2+2*k3+k4)/6;
      RK4Params::yn = RK4Params::y0 + k;
      x0 = x0+h;
      RK4Params::y0 = RK4Params::yn;
      cout<< x0<<"\t"<<RK4Params::yn(5)<< endl;//gai
      X1[i+1]=RK4Params::y0(0);
      X2[i+1]=RK4Params::y0(1);
      X3[i+1]=RK4Params::y0(2);
      X4[i+1]=RK4Params::y0(3);
      X5[i+1]=RK4Params::y0(4);
      X6[i+1]=RK4Params::y0(5);
      }
    /*X1[i]=RK4Params::y0(0);
    X2[i]=RK4Params::y0(1);
    X3[i]=RK4Params::y0(2);
    X4[i]=RK4Params::y0(3);
    X5[i]=RK4Params::y0(4);
    X6[i]=RK4Params::y0(5);*/
}

void RK41(){
    using namespace RK4Params1;
    //RK4Params::n=ceil((RK4Params::xn-RK4Params::x0)/RK4Params::h);
    double  X7[n],X8[n],X9[n],X10[n],
            X71[n],X81[n],X91[n],X101[n];
    double  X11[n],X12[n],X13[n];

    cout<<"\nxn\tyn\n";
    cout<<"------------------\n";
    RK4Params1::y0 << initStatesParams::R10(6),initStatesParams::R10(7),initStatesParams::R10(8),initStatesParams::R10(9),
                      initStatesParams::R10(10),initStatesParams::R10(11),initStatesParams::R10(12);
   //cout<<RK4Params1::y0<<endl;
   cout<< x0<<"\t"<<RK4Params1::y0(0)<<endl;//gai1
    for(i=0; i < n; i++)
    {
      X7[i]=RK4Params1::y0(0);
      X8[i]=RK4Params1::y0(1);
      X9[i]=RK4Params1::y0(2);
      X10[i]=RK4Params1::y0(3);
      X71[i]=RK4Params1::y0(0);
      X81[i]=RK4Params1::y0(1);
      X91[i]=RK4Params1::y0(2);
      X101[i]=RK4Params1::y0(3);

      X11[i]=RK4Params1::y0(4);
      X12[i]=RK4Params1::y0(5);
      X13[i]=RK4Params1::y0(6);

      RK4Params1::EulerPDiffs <<
           X71[i], -X81[i], -X91[i],-X101[i],
           X81[i],  X71[i],-X101[i],  X91[i],
           X91[i], X101[i],  X71[i], -X81[i],
          X101[i], -X91[i],  X81[i],  X71[i];

      RK4Params1::OmigaS  << X11[i],X12[i],X13[i];
      RK4Params1::OmigaST << 0,X11[i],X12[i],X13[i];
      JXO << initStatesParams::JcS*RK4Params1::OmigaS ;
      RK4Params1::OCRJOS <<  RK4Params1::OmigaS(1)*JXO(2)-RK4Params1::OmigaS(2)*JXO(1),
                            -RK4Params1::OmigaS(0)*JXO(2)+RK4Params1::OmigaS(2)*JXO(0),
                             RK4Params1::OmigaS(0)*JXO(1)-RK4Params1::OmigaS(1)*JXO(0);


      RK4Params1::k1 << h*0.5*RK4Params1::EulerPDiffs*RK4Params1::OmigaST;
      RK4Params2::k1 << h*initStatesParams::JcS.inverse()*(initStatesParams::MOS-RK4Params1::OCRJOS);
      X71[i]=RK4Params1::y0(0)+0.5*RK4Params1::k1(0);
      X81[i]=RK4Params1::y0(1)+0.5*RK4Params1::k1(1);
      X91[i]=RK4Params1::y0(2)+0.5*RK4Params1::k1(2);
      X101[i]=RK4Params1::y0(3)+0.5*RK4Params1::k1(3);
      RK4Params1::EulerPDiffs <<
           X71[i], -X81[i], -X91[i],-X101[i],
           X81[i],  X71[i],-X101[i],  X91[i],
           X91[i], X101[i],  X71[i], -X81[i],
          X101[i], -X91[i],  X81[i],  X71[i];

      RK4Params1::OmigaS  << X11[i]+0.5*RK4Params2::k1(0),X12[i]+0.5*RK4Params2::k1(1),X13[i]+0.5*RK4Params2::k1(2);
      RK4Params1::OmigaST << 0,X11[i]+0.5*RK4Params2::k1(0),X12[i]+0.5*RK4Params2::k1(1),X13[i]+0.5*RK4Params2::k1(2);
      JXO << initStatesParams::JcS*RK4Params1::OmigaS ;
      RK4Params1::OCRJOS <<  RK4Params1::OmigaS(1)*JXO(2)-RK4Params1::OmigaS(2)*JXO(1),
                            -RK4Params1::OmigaS(0)*JXO(2)+RK4Params1::OmigaS(2)*JXO(0),
                             RK4Params1::OmigaS(0)*JXO(1)-RK4Params1::OmigaS(1)*JXO(0);


      RK4Params1::k2 << h*0.5*RK4Params1::EulerPDiffs*RK4Params1::OmigaST;
      RK4Params2::k2 << h*initStatesParams::JcS.inverse()*(initStatesParams::MOS-RK4Params1::OCRJOS);
      X71[i]=RK4Params1::y0(0)+0.5*RK4Params1::k2(0);
      X81[i]=RK4Params1::y0(1)+0.5*RK4Params1::k2(1);
      X91[i]=RK4Params1::y0(2)+0.5*RK4Params1::k2(2);
      X101[i]=RK4Params1::y0(3)+0.5*RK4Params1::k2(3);
      RK4Params1::EulerPDiffs <<
           X71[i], -X81[i], -X91[i],-X101[i],
           X81[i],  X71[i],-X101[i],  X91[i],
           X91[i], X101[i],  X71[i], -X81[i],
          X101[i], -X91[i],  X81[i],  X71[i];

      RK4Params1::OmigaS  << X11[i]+0.5*RK4Params2::k2(0),X12[i]+0.5*RK4Params2::k2(1),X13[i]+0.5*RK4Params2::k2(2);
      RK4Params1::OmigaST << 0,X11[i]+0.5*RK4Params2::k2(0),X12[i]+0.5*RK4Params2::k2(1),X13[i]+0.5*RK4Params2::k2(2);
      JXO << initStatesParams::JcS*RK4Params1::OmigaS ;
      RK4Params1::OCRJOS <<  RK4Params1::OmigaS(1)*JXO(2)-RK4Params1::OmigaS(2)*JXO(1),
                            -RK4Params1::OmigaS(0)*JXO(2)+RK4Params1::OmigaS(2)*JXO(0),
                             RK4Params1::OmigaS(0)*JXO(1)-RK4Params1::OmigaS(1)*JXO(0);


      RK4Params1::k3 << h*0.5*RK4Params1::EulerPDiffs*RK4Params1::OmigaST;
      RK4Params2::k3 << h*initStatesParams::JcS.inverse()*(initStatesParams::MOS-RK4Params1::OCRJOS);
      X71[i]=RK4Params1::y0(0)+RK4Params1::k3(0);
      X81[i]=RK4Params1::y0(1)+RK4Params1::k3(1);
      X91[i]=RK4Params1::y0(2)+RK4Params1::k3(2);
      X101[i]=RK4Params1::y0(3)+RK4Params1::k3(3);
      RK4Params1::EulerPDiffs <<
           X71[i], -X81[i], -X91[i],-X101[i],
           X81[i],  X71[i],-X101[i],  X91[i],
           X91[i], X101[i],  X71[i], -X81[i],
          X101[i], -X91[i],  X81[i],  X71[i];

      RK4Params1::OmigaS  << X11[i]+RK4Params2::k3(0),X12[i]+RK4Params2::k3(1),X13[i]+RK4Params2::k3(2);
      RK4Params1::OmigaST << 0,X11[i]+RK4Params2::k3(0),X12[i]+0.5*RK4Params2::k3(1),X13[i]+RK4Params2::k3(2);
      JXO << initStatesParams::JcS*RK4Params1::OmigaS ;
      RK4Params1::OCRJOS <<  RK4Params1::OmigaS(1)*JXO(2)-RK4Params1::OmigaS(2)*JXO(1),
                            -RK4Params1::OmigaS(0)*JXO(2)+RK4Params1::OmigaS(2)*JXO(0),
                             RK4Params1::OmigaS(0)*JXO(1)-RK4Params1::OmigaS(1)*JXO(0);


      RK4Params1::k4 << h*0.5*RK4Params1::EulerPDiffs*RK4Params1::OmigaST;
      RK4Params2::k4 << h*initStatesParams::JcS.inverse()*(initStatesParams::MOS-RK4Params1::OCRJOS);

      RK4Params1::k  = (RK4Params1::k1+2*RK4Params1::k2+2*RK4Params1::k3+RK4Params1::k4)/6;
      RK4Params2::k  = (RK4Params2::k1+2*RK4Params2::k2+2*RK4Params2::k3+RK4Params2::k4)/6;

      RK4Params2::kz << RK4Params1::k(0), RK4Params1::k(1), RK4Params1::k(2), RK4Params1::k(3),
                        RK4Params2::k(0), RK4Params2::k(1), RK4Params2::k(2);

      RK4Params1::yn = RK4Params1::y0 + RK4Params2::kz;
      x0 = x0+h;
      cout<< x0<<"\t"<<RK4Params1::yn(0)<< endl;//gai1
      RK4Params1::y0 = RK4Params1::yn;
      }

   /*k1 = h * (f(x0, y0));
     k2 = h * (f((x0+h/2), (y0+k1/2)));
     k3 = h * (f((x0+h/2), (y0+k2/2)));
     k4 = h * (f((x0+h), (y0+k3)));
     k = (k1+2*k2+2*k3+k4)/6;
     yn = y0 + k;
     cout<< x0<<"\t"<< y0<<"\t"<< yn<< endl;
     x0 = x0+h;
     y0 = yn;*/

}

int main()
{
    initStates();
    InitialConditions();
    RK4();
    RK41();
    cout << "Hello World!" << endl;
    return 0;
}
