// lowcmd_pubs.cpp — Cuenta los PUBLICADORES de rt/lowcmd: emparejados (DataWriters vivos) y ACTIVOS
// (los que de verdad mandan mensajes), con la tasa por publicador.
// Para la prueba de arranque en frío (camera-ready):
//   (b) controlador de fábrica apagado  = 0 publicadores activos antes de lanzar g1_ctrl (debug mode)
//   (c) un solo publicador              = 1 activo con g1_ctrl corriendo (~1000 msg/s)
// Nota: el DataWriter del controlador de fábrica puede seguir EMPAREJADO aunque calle en debug mode;
// por eso se cuentan mensajes por handle de publicación y no solo el emparejamiento.
// SOLO LEE. Imprime una línea por segundo: hora, uptime del Jetson, msg/s total, emparejados, activos y
// la tasa de cada handle.
// Uso: ./lowcmd_pubs <iface=eth0>
#include <dds/dds.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <thread>

using LowCmd = unitree_hg::msg::dds_::LowCmd_;

static double uptime_s() {
  std::ifstream f("/proc/uptime");
  double u = 0; f >> u; return u;
}

int main(int argc, char** argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth0";
  unitree::robot::ChannelFactory::Instance()->Init(0, iface);  // configura CycloneDDS para la interfaz

  dds::domain::DomainParticipant dp(0);
  dds::topic::Topic<LowCmd> topic(dp, "rt/lowcmd");
  dds::sub::Subscriber sub(dp);
  dds::sub::qos::DataReaderQos rqos = sub.default_datareader_qos();
  rqos << dds::core::policy::History::KeepLast(4000);   // 1 kHz x 1 s con margen
  dds::sub::DataReader<LowCmd> rd(sub, topic, rqos);

  std::printf("[pubs] iface=%s topic=rt/lowcmd. hora up msg/s emparejados activos {handle:msg/s}\n", iface.c_str());
  std::fflush(stdout);
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto samples = rd.select().max_samples(5000).take();
    std::map<std::string, size_t> per;
    size_t n = 0;
    for (const auto& s : samples) {
      if (!s.info().valid()) continue;
      ++n;
      std::ostringstream os; os << s.info().publication_handle(); ++per[os.str()];
    }
    auto st = rd.subscription_matched_status();
    std::time_t now = std::time(nullptr);
    char buf[32]; std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&now));
    std::printf("%s up=%.1f lowcmd=%zu msg/s emparejados=%d activos=%zu {", buf, uptime_s(), n, st.current_count(), per.size());
    bool first = true;
    for (const auto& kv : per) { std::printf("%s%s:%zu", first ? "" : ", ", kv.first.c_str(), kv.second); first = false; }
    std::printf("}\n");
    std::fflush(stdout);
  }
  return 0;
}
