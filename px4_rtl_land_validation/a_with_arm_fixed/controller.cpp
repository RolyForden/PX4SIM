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
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/math/Matrix3.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <algorithm>
#include <sstream>
#include <iostream>
#include "cad_geometry.hh"
namespace a_with_arm_fixed {
using V=gz::math::Vector3d;
V vec(const double*a){return {a[0],a[1],a[2]};}
class Controller:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate{
 gz::sim::Model model{gz::sim::kNullEntity};gz::sim::Entity act[3]{},rod[3][2]{},platform{},base{},proxy[3][2]{},endjoint[3][2]{};
 gz::sim::Entity mount{},mountJoint{},fixtureJoint{};std::vector<gz::sim::Entity> allLinks,mechLinks;
 int fd=-1;sockaddr_in dest{};double target[3]{},integ[3]{},q[3]{},tau[3]{},last=0,payload=0,gravity=9.81;V sumForce{0,0,0},sumTorque{0,0,0};double closure=0;int validWrenches=0;bool ready=false;uint64_t seq=0;
 template<class C>void set(gz::sim::Entity e,const std::vector<double>&x,gz::sim::EntityComponentManager&ecm){auto c=ecm.Component<C>(e);if(c)c->Data()=x;else ecm.CreateComponent(e,C(x));}
 struct Props{double mass=0;V center{0,0,0};gz::math::Matrix3d I=gz::math::Matrix3d::Zero;};
 Props properties(const std::vector<gz::sim::Entity>&links,const gz::sim::EntityComponentManager&ecm){Props p;
 for(auto e:links){auto c=ecm.Component<gz::sim::components::Inertial>(e);if(!c)continue;auto ip=gz::sim::worldPose(e,ecm)*c->Data().Pose();double m=c->Data().MassMatrix().Mass();p.mass+=m;p.center+=m*ip.Pos();}p.center/=p.mass;
 for(auto e:links){auto c=ecm.Component<gz::sim::components::Inertial>(e);if(!c)continue;auto ip=gz::sim::worldPose(e,ecm)*c->Data().Pose();double m=c->Data().MassMatrix().Mass();gz::math::Matrix3d Q(ip.Rot());p.I=p.I+Q*c->Data().MassMatrix().Moi()*Q.Transposed();V d=ip.Pos()-p.center;for(int a=0;a<3;a++)for(int b=0;b<3;b++)p.I(a,b)+=m*((a==b?d.SquaredLength():0)-d[a]*d[b]);}return p;}
 std::string wrench(gz::sim::Entity e,const gz::sim::EntityComponentManager&ecm){auto c=ecm.Component<gz::sim::components::JointTransmittedWrench>(e);if(!c||!c->Data().has_force()||!c->Data().has_torque())return "null";auto f=c->Data().force(),t=c->Data().torque();std::ostringstream s;s.precision(12);s<<"["<<f.x()<<","<<f.y()<<","<<f.z()<<","<<t.x()<<","<<t.y()<<","<<t.z()<<"]";return s.str();}
 public:~Controller(){if(fd>=0)close(fd);}
 void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&,gz::sim::EventManager&)override{
 model=gz::sim::Model(e);payload=s->Get<double>("payload");gravity=s->Get<double>("gravity");fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(14850);if(bind(fd,(sockaddr*)&a,sizeof(a))!=0){perror("delta dynamics bind");close(fd);fd=-1;}dest=a;dest.sin_port=htons(14851);
 }
 void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override{
 if(info.paused||fd<0)return;
 if(!ready){base=model.LinkByName(ecm,"base_link");auto pm=ecm.EntityByComponents(gz::sim::components::Model(),gz::sim::components::Name("integrated_platform"));if(!pm)return;platform=gz::sim::Model(pm).LinkByName(ecm,"platform");if(!platform)return;gz::sim::Link(platform).EnableVelocityChecks(ecm);for(int i=0;i<3;i++){act[i]=model.JointByName(ecm,"act_"+std::to_string(i));if(!act[i])return;for(int k=0;k<2;k++){rod[i][k]=model.LinkByName(ecm,"rod_"+std::to_string(i)+"_"+std::to_string(k));if(!rod[i][k])return;gz::sim::Link(rod[i][k]).EnableVelocityChecks(ecm);proxy[i][k]=model.LinkByName(ecm,"tip_"+std::to_string(i)+"_"+std::to_string(k));endjoint[i][k]=model.JointByName(ecm,"end_"+std::to_string(i)+"_"+std::to_string(k));if(!proxy[i][k]||!endjoint[i][k])return;if(!ecm.Component<gz::sim::components::JointTransmittedWrench>(endjoint[i][k]))ecm.CreateComponent(endjoint[i][k],gz::sim::components::JointTransmittedWrench());auto weld=ecm.CreateEntity();ecm.CreateComponent(weld,gz::sim::components::DetachableJoint({platform,proxy[i][k],"fixed"}));}if(!ecm.Component<gz::sim::components::JointPosition>(act[i]))ecm.CreateComponent(act[i],gz::sim::components::JointPosition());if(!ecm.Component<gz::sim::components::JointVelocity>(act[i]))ecm.CreateComponent(act[i],gz::sim::components::JointVelocity());}mount=model.LinkByName(ecm,"arm_mount");mountJoint=model.JointByName(ecm,"arm_mount_joint");fixtureJoint=model.JointByName(ecm,"fixture_fixed");if(!mount||!mountJoint||!fixtureJoint)return;
 for(auto e:{mountJoint,fixtureJoint})if(!ecm.Component<gz::sim::components::JointTransmittedWrench>(e))ecm.CreateComponent(e,gz::sim::components::JointTransmittedWrench());
 allLinks=model.Links(ecm);auto plinks=gz::sim::Model(pm).Links(ecm);allLinks.insert(allLinks.end(),plinks.begin(),plinks.end());mechLinks=plinks;mechLinks.push_back(mount);for(int i=0;i<3;i++){mechLinks.push_back(model.LinkByName(ecm,"upper_"+std::to_string(i)));for(int k=0;k<2;k++){mechLinks.push_back(rod[i][k]);mechLinks.push_back(proxy[i][k]);}}ready=true;}
 struct Cmd{uint32_t magic;float q[3];}c;while(recv(fd,&c,sizeof(c),0)==sizeof(c)){bool valid=c.magic==0x44454c54;for(float x:c.q)valid=valid&&std::isfinite(x)&&std::abs(x)<=.6;if(valid)for(int i=0;i<3;i++)target[i]=c.q[i];}
 const double dt=std::chrono::duration<double>(info.dt).count();for(int i=0;i<3;i++){auto qp=ecm.Component<gz::sim::components::JointPosition>(act[i]);auto vp=ecm.Component<gz::sim::components::JointVelocity>(act[i]);if(!qp||qp->Data().empty())return;q[i]=qp->Data()[0];double velocity=vp&&!vp->Data().empty()?vp->Data()[0]:0;double err=target[i]-q[i];integ[i]=std::clamp(integ[i]+err*dt,-.08,.08);tau[i]=std::clamp(30*err+20*integ[i]-.5*velocity,-2.,2.);set<gz::sim::components::JointForceCmd>(act[i],{tau[i]},ecm);}
 auto pp=gz::sim::worldPose(platform,ecm);gz::sim::Link pl(platform);sumForce=V::Zero;sumTorque=V::Zero;closure=0;validWrenches=0;
 for(int i=0;i<3;i++)for(int k=0;k<2;k++){
  auto rp=gz::sim::worldPose(rod[i][k],ecm);auto tp=gz::sim::worldPose(proxy[i][k],ecm);V a=rp.Pos()+rp.Rot().RotateVector(V(0,0,.2)),b=pp.Pos()+pp.Rot().RotateVector(vec(cad::OP[i][k]));closure=std::max(closure,a.Distance(b));
  auto w=ecm.Component<gz::sim::components::JointTransmittedWrench>(endjoint[i][k]);if(w&&w->Data().has_force()){++validWrenches;auto f=w->Data().force();sumForce+=tp.Rot().RotateVector(V(f.x(),f.y(),f.z()));}
 }

 }
 void PostUpdate(const gz::sim::UpdateInfo&info,const gz::sim::EntityComponentManager&ecm)override{
 double t=std::chrono::duration<double>(info.simTime).count();if(!ready||info.paused||t-last<.01||fd<0)return;last=t;auto bp=gz::sim::worldPose(base,ecm),pp=gz::sim::worldPose(platform,ecm),p=bp.Inverse()*pp;auto pv=gz::sim::Link(platform).WorldLinearVelocity(ecm).value_or(V::Zero);double actq[3]{};for(int i=0;i<3;i++){auto a=ecm.Component<gz::sim::components::JointPosition>(act[i]);if(a&&!a->Data().empty())actq[i]=a->Data()[0];}
 std::ostringstream s;s.precision(10);s<<"{\"seq\":"<<++seq<<",\"sim_s\":"<<t<<",\"q_rad\":["<<actq[0]<<","<<actq[1]<<","<<actq[2]<<"],\"target_q_rad\":["<<target[0]<<","<<target[1]<<","<<target[2]<<"],\"position_m\":["<<p.Pos().X()<<","<<p.Pos().Y()<<","<<p.Pos().Z()<<"],\"closure_m\":"<<closure<<",\"base_world_m\":["<<bp.Pos().X()<<","<<bp.Pos().Y()<<","<<bp.Pos().Z()<<"],\"platform_rpy_rad\":["<<p.Rot().Roll()<<","<<p.Rot().Pitch()<<","<<p.Rot().Yaw()<<"],\"platform_velocity_m_s\":["<<pv.X()<<","<<pv.Y()<<","<<pv.Z()<<"],\"joint_torque_nm\":["<<tau[0]<<","<<tau[1]<<","<<tau[2]<<"],\"platform_constraint_force_n\":["<<sumForce.X()<<","<<sumForce.Y()<<","<<sumForce.Z()<<"],\"payload_kg\":"<<payload<<",\"gravity_m_s2\":"<<gravity<<",\"end_wrench_valid_count\":"<<validWrenches;auto total=properties(allLinks,ecm),mechanism=properties(mechLinks,ecm);auto mp=gz::sim::worldPose(mount,ecm);V com=bp.Rot().RotateVectorReverse(total.center-bp.Pos());gz::math::Matrix3d QB(bp.Rot());auto Ibody=QB.Transposed()*total.I*QB;V expectedWorld(0,0,mechanism.mass*gravity);V ef=mp.Rot().RotateVectorReverse(expectedWorld),et=mp.Rot().RotateVectorReverse((mechanism.center-mp.Pos()).Cross(expectedWorld));
 s<<",\"mount_wrench_n_nm\":"<<wrench(mountJoint,ecm)<<",\"fixture_wrench_n_nm\":"<<wrench(fixtureJoint,ecm)<<",\"total_mass_kg\":"<<total.mass<<",\"mechanism_mass_kg\":"<<mechanism.mass<<",\"com_flu_m\":["<<com.X()<<","<<com.Y()<<","<<com.Z()<<"],\"inertia_com_flu_kg_m2\":[";for(int a=0;a<3;a++)for(int b=0;b<3;b++){if(a||b)s<<",";s<<Ibody(a,b);}s<<"],\"expected_mount_gravity_n_nm\":["<<ef.X()<<","<<ef.Y()<<","<<ef.Z()<<","<<et.X()<<","<<et.Y()<<","<<et.Z()<<"],\"mount_world_m\":["<<mp.Pos().X()<<","<<mp.Pos().Y()<<","<<mp.Pos().Z()<<"]}";
 auto data=s.str();sendto(fd,data.data(),data.size(),0,(sockaddr*)&dest,sizeof(dest));
 }
};
}
GZ_ADD_PLUGIN(a_with_arm_fixed::Controller,gz::sim::System,a_with_arm_fixed::Controller::ISystemConfigure,a_with_arm_fixed::Controller::ISystemPreUpdate,a_with_arm_fixed::Controller::ISystemPostUpdate)
