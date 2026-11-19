#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/plugin/Register.hh>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <algorithm>
#include <sstream>
#include <iostream>
#include "cad_geometry.hh"
namespace delta_dynamics {
using V=gz::math::Vector3d;
V vec(const double*a){return {a[0],a[1],a[2]};}
class Controller:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate{
 gz::sim::Model model{gz::sim::kNullEntity};gz::sim::Entity act[3]{},rod[3][2]{},platform{},base{};
 int fd=-1;sockaddr_in dest{};double target[3]{},integ[3]{},q[3]{},tau[3]{},last=0,K=2000,D=3,payload=0,gravity=9.81;V sumForce{0,0,0},sumTorque{0,0,0};double closure=0,power=0;bool ready=false;uint64_t seq=0;
 template<class C>void set(gz::sim::Entity e,const std::vector<double>&x,gz::sim::EntityComponentManager&ecm){auto c=ecm.Component<C>(e);if(c)c->Data()=x;else ecm.CreateComponent(e,C(x));}
 public:~Controller(){if(fd>=0)close(fd);}
 void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&,gz::sim::EventManager&)override{
 model=gz::sim::Model(e);K=s->Get<double>("stiffness");D=s->Get<double>("damping");payload=s->Get<double>("payload");gravity=s->Get<double>("gravity");fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(14830);if(bind(fd,(sockaddr*)&a,sizeof(a))!=0){perror("delta dynamics bind");close(fd);fd=-1;}dest=a;dest.sin_port=htons(14831);
 }
 void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override{
 if(info.paused||fd<0)return;
 if(!ready){base=model.LinkByName(ecm,"base_link");auto pm=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("delta_platform"));if(!pm)return;platform=gz::sim::Model(pm).LinkByName(ecm,"platform");if(!platform)return;gz::sim::Link(platform).EnableVelocityChecks(ecm);for(int i=0;i<3;i++){act[i]=model.JointByName(ecm,"act_"+std::to_string(i));if(!act[i])return;for(int k=0;k<2;k++){rod[i][k]=model.LinkByName(ecm,"rod_"+std::to_string(i)+"_"+std::to_string(k));if(!rod[i][k])return;gz::sim::Link(rod[i][k]).EnableVelocityChecks(ecm);}if(!ecm.Component<gz::sim::components::JointPosition>(act[i]))ecm.CreateComponent(act[i],gz::sim::components::JointPosition());if(!ecm.Component<gz::sim::components::JointVelocity>(act[i]))ecm.CreateComponent(act[i],gz::sim::components::JointVelocity());}ready=true;}
 struct Cmd{uint32_t magic;float q[3];}c;while(recv(fd,&c,sizeof(c),0)==sizeof(c)){bool valid=c.magic==0x44454c54;for(float x:c.q)valid=valid&&std::isfinite(x)&&std::abs(x)<=.6;if(valid)for(int i=0;i<3;i++)target[i]=c.q[i];}
 const double dt=std::chrono::duration<double>(info.dt).count();for(int i=0;i<3;i++){auto qp=ecm.Component<gz::sim::components::JointPosition>(act[i]);auto vp=ecm.Component<gz::sim::components::JointVelocity>(act[i]);if(!qp||qp->Data().empty())return;q[i]=qp->Data()[0];double velocity=vp&&!vp->Data().empty()?vp->Data()[0]:0;double err=target[i]-q[i];integ[i]=std::clamp(integ[i]+err*dt,-.08,.08);tau[i]=std::clamp(30*err+20*integ[i]-.5*velocity,-2.,2.);set<gz::sim::components::JointForceCmd>(act[i],{tau[i]},ecm);}
 auto pp=gz::sim::worldPose(platform,ecm);gz::sim::Link pl(platform);sumForce=V::Zero;sumTorque=V::Zero;closure=0;power=0;
 for(int i=0;i<3;i++)for(int k=0;k<2;k++){
  gz::sim::Link rl(rod[i][k]);auto rp=gz::sim::worldPose(rod[i][k],ecm);V ro(0,0,.2),po=vec(cad::OP[i][k]);V a=rp.Pos()+rp.Rot().RotateVector(ro),b=pp.Pos()+pp.Rot().RotateVector(po);V va=rl.WorldLinearVelocity(ecm,ro).value_or(V::Zero),vb=pl.WorldLinearVelocity(ecm,po).value_or(V::Zero);V d=b-a;V relative=vb-va;V f=K*d+D*relative;
  // Apply equal/opposite force and the actual moment arm, with no pose commands.
  rl.AddWorldWrench(ecm,f,(a-rp.Pos()).Cross(f));pl.AddWorldWrench(ecm,-f,(b-pp.Pos()).Cross(-f));sumForce-=f;sumTorque+=(b-pp.Pos()).Cross(-f);closure=std::max(closure,d.Length());power+=f.Dot(va)-f.Dot(vb);
 }
 }
 void PostUpdate(const gz::sim::UpdateInfo&info,const gz::sim::EntityComponentManager&ecm)override{
 double t=std::chrono::duration<double>(info.simTime).count();if(!ready||info.paused||t-last<.01||fd<0)return;last=t;auto bp=gz::sim::worldPose(base,ecm),pp=gz::sim::worldPose(platform,ecm),p=bp.Inverse()*pp;auto pv=gz::sim::Link(platform).WorldLinearVelocity(ecm).value_or(V::Zero);double actq[3]{};for(int i=0;i<3;i++){auto a=ecm.Component<gz::sim::components::JointPosition>(act[i]);if(a&&!a->Data().empty())actq[i]=a->Data()[0];}
 std::ostringstream s;s.precision(10);s<<"{\"seq\":"<<++seq<<",\"sim_s\":"<<t<<",\"q_rad\":["<<actq[0]<<","<<actq[1]<<","<<actq[2]<<"],\"target_q_rad\":["<<target[0]<<","<<target[1]<<","<<target[2]<<"],\"position_m\":["<<p.Pos().X()<<","<<p.Pos().Y()<<","<<p.Pos().Z()<<"],\"closure_m\":"<<closure<<",\"base_world_m\":["<<bp.Pos().X()<<","<<bp.Pos().Y()<<","<<bp.Pos().Z()<<"],\"platform_rpy_rad\":["<<p.Rot().Roll()<<","<<p.Rot().Pitch()<<","<<p.Rot().Yaw()<<"],\"platform_velocity_m_s\":["<<pv.X()<<","<<pv.Y()<<","<<pv.Z()<<"],\"joint_torque_nm\":["<<tau[0]<<","<<tau[1]<<","<<tau[2]<<"],\"platform_constraint_force_n\":["<<sumForce.X()<<","<<sumForce.Y()<<","<<sumForce.Z()<<"],\"payload_kg\":"<<payload<<",\"gravity_m_s2\":"<<gravity<<",\"constraint_power_w\":"<<power<<"}";auto data=s.str();sendto(fd,data.data(),data.size(),0,(sockaddr*)&dest,sizeof(dest));
 }
};
}
GZ_ADD_PLUGIN(delta_dynamics::Controller,gz::sim::System,delta_dynamics::Controller::ISystemConfigure,delta_dynamics::Controller::ISystemPreUpdate,delta_dynamics::Controller::ISystemPostUpdate)
