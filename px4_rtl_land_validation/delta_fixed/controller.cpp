#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/plugin/Register.hh>
#include <Eigen/Dense>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <algorithm>
#include <sstream>
#include <iostream>
#include "cad_geometry.hh"
namespace delta_fixed {
using V=Eigen::Vector3d;using M=Eigen::Matrix3d;
static V v(const double *a){return V(a[0],a[1],a[2]);}
static M rot(int i,double q){return Eigen::AngleAxisd(q,v(cad::A[i])).toRotationMatrix();}
static M frame(V z,V a){z.normalize();V x=(a-z*z.dot(a)).normalized();M f;f.col(0)=x;f.col(1)=z.cross(x);f.col(2)=z;return f;}
static V forward(const double*q,V p){
 for(int k=0;k<20;k++){M J;V f;for(int i=0;i<3;i++){V e=v(cad::S[i])+rot(i,q[i])*(v(cad::E[i])-v(cad::S[i]));V d=p+v(cad::O[i])-e;f[i]=d.squaredNorm()-.04;J.row(i)=2*d.transpose();}if(f.norm()<1e-13)break;V step=J.colPivHouseholderQr().solve(f);if(!step.allFinite()||step.norm()>.1)return V::Constant(NAN);p-=step;}return p;
}
class Controller:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate{
 gz::sim::Model model{gz::sim::kNullEntity};gz::sim::Entity act[3]{},rod[3][2][3]{},slide[3]{},rodlinks[3][2]{},platform{},base{};
 int fd=-1;sockaddr_in dest{};double target[3]{},integ[3]{},q[3]{},last=0;V p=v(cad::P);bool ready=false;uint64_t sequence=0;
 template<class C>void set(gz::sim::Entity e,const std::vector<double>&x,gz::sim::EntityComponentManager&ecm){auto c=ecm.Component<C>(e);if(c)c->Data()=x;else ecm.CreateComponent(e,C(x));}
 void reset(gz::sim::Entity e,double q,gz::sim::EntityComponentManager&ecm){set<gz::sim::components::JointPositionReset>(e,{q},ecm);set<gz::sim::components::JointVelocityReset>(e,{0},ecm);}
 public:~Controller(){if(fd>=0)close(fd);}
 void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&,gz::sim::EntityComponentManager&,gz::sim::EventManager&)override{
 model=gz::sim::Model(e);fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(14820);if(bind(fd,(sockaddr*)&a,sizeof(a))!=0){perror("delta command bind");close(fd);fd=-1;}dest=a;dest.sin_port=htons(14821);
 }
 void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override{
 if(info.paused||fd<0)return;
 if(!ready){base=model.LinkByName(ecm,"base_link");platform=model.LinkByName(ecm,"platform");for(int i=0;i<3;i++){act[i]=model.JointByName(ecm,"act_"+std::to_string(i));slide[i]=model.JointByName(ecm,"platform_"+std::to_string(i));if(!act[i]||!slide[i])return;for(int k=0;k<2;k++){rodlinks[i][k]=model.LinkByName(ecm,"rod_"+std::to_string(i)+"_"+std::to_string(k));for(int n=0;n<3;n++)rod[i][k][n]=model.JointByName(ecm,"rod_"+std::to_string(i)+"_"+std::to_string(k)+"_"+std::to_string(n));}if(!ecm.Component<gz::sim::components::JointPosition>(act[i]))ecm.CreateComponent(act[i],gz::sim::components::JointPosition());if(!ecm.Component<gz::sim::components::JointVelocity>(act[i]))ecm.CreateComponent(act[i],gz::sim::components::JointVelocity());}ready=true;}
 struct Cmd{uint32_t magic;float q[3];}c;static_assert(sizeof(c)==16);while(recv(fd,&c,sizeof(c),0)==sizeof(c)){bool valid=c.magic==0x44454c54;for(float x:c.q)valid=valid&&std::isfinite(x)&&std::abs(x)<=.6;if(valid)for(int i=0;i<3;i++)target[i]=c.q[i];}
 double dt=std::chrono::duration<double>(info.dt).count();for(int i=0;i<3;i++){auto qp=ecm.Component<gz::sim::components::JointPosition>(act[i]);auto vp=ecm.Component<gz::sim::components::JointVelocity>(act[i]);if(!qp||qp->Data().empty())return;q[i]=qp->Data()[0];double vel=vp&&!vp->Data().empty()?vp->Data()[0]:0;double err=target[i]-q[i];integ[i]=std::clamp(integ[i]+err*dt,-.1,.1);set<gz::sim::components::JointForceCmd>(act[i],{std::clamp(8*err+integ[i]-.03*vel,-2.,2.)},ecm);}
 V next=forward(q,p);if(!next.allFinite())return;p=next;V delta=p-v(cad::P);for(int n=0;n<3;n++)reset(slide[n],delta[n],ecm);
 for(int i=0;i<3;i++){M ru=rot(i,q[i]);for(int k=0;k<2;k++){V e=v(cad::S[i])+ru*(v(cad::EP[i][k])-v(cad::S[i]));V b=p+v(cad::OP[i][k]);M f=frame(b-e,v(cad::A[i]));V angles=(ru.transpose()*f).eulerAngles(0,1,2);for(int n=0;n<3;n++)reset(rod[i][k][n],angles[n],ecm);}}
 }
 void PostUpdate(const gz::sim::UpdateInfo&info,const gz::sim::EntityComponentManager&ecm)override{
 double t=std::chrono::duration<double>(info.simTime).count();if(!ready||info.paused||t-last<.01||fd<0)return;last=t;auto bp=gz::sim::worldPose(base,ecm);auto pp=gz::sim::worldPose(platform,ecm);auto rel=bp.Inverse()*pp;double maxClosure=0,actualq[3]{};for(int i=0;i<3;i++){auto c=ecm.Component<gz::sim::components::JointPosition>(act[i]);if(c&&!c->Data().empty())actualq[i]=c->Data()[0];for(int k=0;k<2;k++){auto pose=gz::sim::worldPose(rodlinks[i][k],ecm);auto actualEnd=pose.Pos()+pose.Rot().RotateVector(gz::math::Vector3d(0,0,.2));V b=p+v(cad::OP[i][k]);auto desired=bp.Pos()+bp.Rot().RotateVector(gz::math::Vector3d(b[0],b[1],b[2]));maxClosure=std::max(maxClosure,actualEnd.Distance(desired));}}
 V f=forward(actualq,p);double pe=V(rel.Pos().X()-f.x(),rel.Pos().Y()-f.y(),rel.Pos().Z()-f.z()).norm();std::ostringstream s;s.precision(10);s<<"{\"seq\":"<<++sequence<<",\"sim_s\":"<<t<<",\"q_rad\":["<<actualq[0]<<","<<actualq[1]<<","<<actualq[2]<<"],\"target_q_rad\":["<<target[0]<<","<<target[1]<<","<<target[2]<<"],\"position_m\":["<<rel.Pos().X()<<","<<rel.Pos().Y()<<","<<rel.Pos().Z()<<"],\"closure_m\":"<<maxClosure<<",\"platform_fk_error_m\":"<<pe<<",\"base_world_m\":["<<bp.Pos().X()<<","<<bp.Pos().Y()<<","<<bp.Pos().Z()<<"]}";auto data=s.str();sendto(fd,data.data(),data.size(),0,(sockaddr*)&dest,sizeof(dest));
 }
};
}
GZ_ADD_PLUGIN(delta_fixed::Controller,gz::sim::System,delta_fixed::Controller::ISystemConfigure,delta_fixed::Controller::ISystemPreUpdate,delta_fixed::Controller::ISystemPostUpdate)
