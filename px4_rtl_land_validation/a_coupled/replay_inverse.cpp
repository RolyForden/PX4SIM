#include <matrix/matrix/math.hpp>
#include <iostream>
#include <cmath>
int main(int argc,char**){
 double stamp;while(std::cin>>stamp){
  matrix::Matrix<float,6,16> G;
  for(int i=0;i<4;i++){
   float px,py,pz,ax,ay,az;std::cin>>px>>py>>pz>>ax>>ay>>az;
   matrix::Vector3f p(px,py,pz),a(ax,ay,az);a/=a.norm();
   auto force=6.5f*a;auto moment=6.5f*p.cross(a)-6.5f*(i<2?.016f:-.016f)*a;
   for(int j=0;j<3;j++){G(j,i)=moment(j,0);G(j+3,i)=force(j,0);}
  }
  for(int i=0;i<6;i++){bool small=true;for(int j=0;j<4;j++)small=small&&fabsf(G(i,j))<=.05f;if(small)G.row(i)=0.f;}
  if(argc>1){G.row(3)=0.f;G.row(4)=0.f;}
  matrix::Matrix<float,16,6> M;matrix::geninv(G,M);
  matrix::SquareMatrix<float,6> A=G*G.transpose();size_t rank=0;matrix::fullRankCholesky(A,rank);
  float error=(G*M*G-G).abs().max();
  std::cout.precision(12);std::cout<<stamp<<" "<<rank<<" "<<M.abs().max()<<" "<<error<<"\n";
 }
}
