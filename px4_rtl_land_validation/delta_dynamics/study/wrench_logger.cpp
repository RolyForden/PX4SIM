#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/plugin/Register.hh>
#include <fstream>
namespace study {
class WrenchLogger:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPostUpdate{
 gz::sim::Entity joint{};std::ofstream log;double last=-1;
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override{
 joint=gz::sim::Model(e).JointByName(ecm,s->Get<std::string>("joint"));log.open(s->Get<std::string>("output"));if(joint&&!ecm.Component<gz::sim::components::JointTransmittedWrench>(joint))ecm.CreateComponent(joint,gz::sim::components::JointTransmittedWrench());
 }
 void PostUpdate(const gz::sim::UpdateInfo&i,const gz::sim::EntityComponentManager&ecm)override{
 double t=std::chrono::duration<double>(i.simTime).count();if(i.paused||t-last<.01)return;last=t;auto c=ecm.Component<gz::sim::components::JointTransmittedWrench>(joint);bool valid=c&&c->Data().has_force()&&c->Data().has_torque();log.precision(12);log<<"{\"sim_s\":"<<t<<",\"valid\":"<<(valid?"true":"false");if(valid){auto f=c->Data().force(),m=c->Data().torque();log<<",\"force_n\":["<<f.x()<<","<<f.y()<<","<<f.z()<<"],\"torque_nm\":["<<m.x()<<","<<m.y()<<","<<m.z()<<"]";}log<<"}\n";log.flush();
 }
};
}
GZ_ADD_PLUGIN(study::WrenchLogger,gz::sim::System,study::WrenchLogger::ISystemConfigure,study::WrenchLogger::ISystemPostUpdate)
