"""Build opt-in SITL geometry adapter; restore baseline sources/binary on exit."""
import pathlib,subprocess,shutil,difflib,hashlib,json
ROOT=pathlib.Path(__file__).resolve().parent
PX=ROOT.parents[1]/'PX4-Autopilot'
REL=pathlib.Path('src/modules/control_allocator/VehicleActuatorEffectiveness')
H=PX/REL/'ActuatorEffectivenessMultirotor.hpp';C=PX/REL/'ActuatorEffectivenessMultirotor.cpp'
original={H:H.read_bytes(),C:C.read_bytes()};binary=PX/'build/px4_sitl_default/bin/px4'
backup=ROOT/'px4_baseline.bin';shutil.copy2(binary,backup)
h=original[H].decode().replace('virtual ~ActuatorEffectivenessMultirotor() = default;','virtual ~ActuatorEffectivenessMultirotor();')
h=h.replace('ActuatorEffectivenessRotors _mc_rotors;','''ActuatorEffectivenessRotors _mc_rotors;
#if defined(__PX4_POSIX)
 int _morph_fd{-1};
 uint32_t _morph_seq{0};
 uint64_t _morph_received{0};
 float _morph_data[24]{};
 bool _morph_valid{false};
#endif''')
c=original[C].decode().replace('using namespace matrix;','''#if defined(__PX4_POSIX)
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <drivers/drv_hrt.h>
#endif
using namespace matrix;''')
c=c.replace('  _mc_rotors(this)\n{\n}', '''  _mc_rotors(this)
{
#if defined(__PX4_POSIX)
 if (std::getenv("PX4SIM_MORPH_UDP")) {
  _morph_fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(14780);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (_morph_fd >= 0 && ::bind(_morph_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
   ::close(_morph_fd); _morph_fd = -1;
  }
  PX4_INFO("SITL morph geometry UDP: %s", _morph_fd >= 0 ? "127.0.0.1:14780" : "unavailable");
 }
#endif
}

ActuatorEffectivenessMultirotor::~ActuatorEffectivenessMultirotor()
{
#if defined(__PX4_POSIX)
 if (_morph_fd >= 0) { ::close(_morph_fd); }
#endif
}''')
assert 'SITL morph geometry UDP' in c
needle='''	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}'''
replacement='''#if defined(__PX4_POSIX)
 bool changed = false;
 struct Packet { uint32_t magic; uint32_t seq; float data[24]; } packet{};
 static_assert(sizeof(Packet) == 104, "Morph packet layout");
 while (_morph_fd >= 0 && ::recv(_morph_fd, &packet, sizeof(packet), MSG_DONTWAIT) == sizeof(packet)) {
  bool valid = packet.magic == 0x41555631u;
  for (int i = 0; i < 24; ++i) { valid = valid && std::isfinite(packet.data[i]) && fabsf(packet.data[i]) < 2.f; }
  for (int i = 0; i < 4; ++i) {
   Vector3f axis(packet.data[6*i+3], packet.data[6*i+4], packet.data[6*i+5]);
   valid = valid && axis.norm() > .95f && axis.norm() < 1.05f;
  }
  if (valid) {
   memcpy(_morph_data, packet.data, sizeof(_morph_data));
   _morph_seq = packet.seq; _morph_received = hrt_absolute_time(); _morph_valid = true; changed = true;
  }
 }
 if (_morph_valid && (changed || external_update != EffectivenessUpdateReason::NO_EXTERNAL_UPDATE)) {
  auto geometry = _mc_rotors.geometry();
  if (geometry.num_rotors == 4) {
   for (int i = 0; i < 4; ++i) {
    geometry.rotors[i].position = Vector3f(_morph_data[6*i], _morph_data[6*i+1], _morph_data[6*i+2]);
    geometry.rotors[i].axis = Vector3f(_morph_data[6*i+3], _morph_data[6*i+4], _morph_data[6*i+5]);
   }
   const int added = ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry,
    configuration.effectiveness_matrices[0], 0);
   configuration.actuatorsAdded(ActuatorType::MOTORS, added);
   struct Ack { uint32_t magic; uint32_t seq; } ack{0x41555631u, _morph_seq};
   sockaddr_in dest{}; dest.sin_family = AF_INET; dest.sin_port = htons(14781);
   dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
   ::sendto(_morph_fd, &ack, sizeof(ack), MSG_DONTWAIT, reinterpret_cast<sockaddr *>(&dest), sizeof(dest));
   return added == 4;
  }
 }
#endif
 if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) { return false; }'''
assert needle in c;c=c.replace(needle,replacement)
patch=''
for file,new in [(H,h),(C,c)]:patch+=''.join(difflib.unified_diff(original[file].decode().splitlines(True),new.splitlines(True),fromfile='a/'+str(file.relative_to(PX)),tofile='b/'+str(file.relative_to(PX))))
(ROOT/'px4-morph-geometry.patch').write_text(patch)
try:
 H.write_text(h);C.write_text(c)
 with (ROOT/'px4-build.log').open('w') as log:
  subprocess.run(['docker','exec','px4-rtl-land-harmonic','bash','-lc','cd /work/PX4-Autopilot && make px4_sitl_default -j4'],stdout=log,stderr=subprocess.STDOUT,check=True)
 shutil.copy2(binary,ROOT/'px4_morph')
 (ROOT/'px4-build.json').write_text(json.dumps({'baseline_sha256':hashlib.sha256(backup.read_bytes()).hexdigest(),'morph_sha256':hashlib.sha256((ROOT/'px4_morph').read_bytes()).hexdigest(),'source_patch':'px4-morph-geometry.patch','adapter':'opt-in loopback UDP rotor geometry, PX4 native allocation and controllers'},indent=2))
 print('Built dedicated px4_morph; baseline sources/binary restored')
finally:
 for p,raw in original.items():p.write_bytes(raw)
 subprocess.run(['docker','exec','px4-rtl-land-harmonic','bash','-lc','cp /work/a_morphing/airborne/px4_baseline.bin /work/PX4-Autopilot/build/px4_sitl_default/bin/px4.restore && mv /work/PX4-Autopilot/build/px4_sitl_default/bin/px4.restore /work/PX4-Autopilot/build/px4_sitl_default/bin/px4'],check=True)
