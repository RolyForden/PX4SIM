#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/plugin/Register.hh>
#include <fstream>
#include <string>
namespace native_probe {
class Probe:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPostUpdate{
 gz::sim::Model model{gz::sim::kNullEntity};gz::sim::Entity base{},bar{};std::ofstream log;double last=-1;
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override{
 model=gz::sim::Model(e);base=model.LinkByName(ecm,"base");bar=model.LinkByName(ecm,"bar");log.open(s->Get<std::string>("output"));
 if(s->Get<bool>("proxy")){auto tip=model.LinkByName(ecm,"tip");auto j=ecm.CreateEntity();ecm.CreateComponent(j,gz::sim::components::DetachableJoint({base,tip,"fixed"}));}
 }
 void PostUpdate(const gz::sim::UpdateInfo&i,const gz::sim::EntityComponentManager&ecm)override{
 double t=std::chrono::duration<double>(i.simTime).count();if(i.paused||t-last<.01)return;last=t;auto b=gz::sim::worldPose(base,ecm),p=gz::sim::worldPose(bar,ecm);double gap=0;for(double x:{-.1,.1}){auto a=b.Pos()+b.Rot().RotateVector(gz::math::Vector3d(x,0,0));auto c=p.Pos()+p.Rot().RotateVector(gz::math::Vector3d(x,0,0));gap=std::max(gap,a.Distance(c));}log<<"{\"sim_s\":"<<t<<",\"endpoint_gap_m\":"<<gap<<",\"bar_z_m\":"<<p.Pos().Z()<<"}\n";log.flush();
 }
};
}
GZ_ADD_PLUGIN(native_probe::Probe,gz::sim::System,native_probe::Probe::ISystemConfigure,native_probe::Probe::ISystemPostUpdate)
