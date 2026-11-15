#include <gz/transport/Node.hh>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/double.pb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <array>
#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
static constexpr uint32_t MAGIC=0x41555631;
struct Command {uint32_t magic; float q[8];};
struct Feedback {uint32_t magic,seq; float q[8]; double sim_s;};
static_assert(sizeof(Command)==36 && sizeof(Feedback)==48);
int main(){
 gz::transport::Node node; std::array<float,8> actual{},target{};std::mutex mu;bool ready=false;double sim_s=0;
 int fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);addr.sin_port=htons(14782);
 if(bind(fd,(sockaddr*)&addr,sizeof(addr))!=0){perror("bind");return 1;}
 sockaddr_in dest=addr;dest.sin_port=htons(14783);std::array<gz::transport::Node::Publisher,8> pubs;
 for(int i=0;i<8;i++){std::string name=(i%2?"pitch_":"yaw_")+std::to_string(i/2);pubs[i]=node.Advertise<gz::msgs::Double>("/a_air/joint/"+name+"/cmd_pos");}
 std::function<void(const gz::msgs::Model&)> cb=[&](const gz::msgs::Model&m){
  std::lock_guard<std::mutex> guard(mu);bool seen[8]{};
  for(const auto&j:m.joint())for(int i=0;i<8;i++){std::string name=(i%2?"pitch_":"yaw_")+std::to_string(i/2);if(j.name()==name){actual[i]=j.axis1().position();seen[i]=true;}}
  bool all=true;for(bool x:seen)all=all&&x;if(all){ready=true;sim_s=m.header().stamp().sec()+m.header().stamp().nsec()*1e-9;}
 };
 if(!node.Subscribe("/a_air/joint_states",cb)){return 2;}
 uint32_t seq=0;
 for(;;){
  Command c{};while(recv(fd,&c,sizeof(c),MSG_DONTWAIT)==sizeof(c)){
   bool valid=c.magic==MAGIC;for(float q:c.q)valid=valid&&std::isfinite(q)&&fabs(q)<0.81;
   if(valid)for(int i=0;i<8;i++)target[i]=c.q[i];
  }
  for(int i=0;i<8;i++){gz::msgs::Double m;m.set_data(target[i]);pubs[i].Publish(m);}
  {std::lock_guard<std::mutex> guard(mu);if(ready){Feedback f{};f.magic=MAGIC;f.seq=++seq;f.sim_s=sim_s;for(int i=0;i<8;i++)f.q[i]=actual[i];sendto(fd,&f,sizeof(f),0,(sockaddr*)&dest,sizeof(dest));}}
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
 }
}
