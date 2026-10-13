// lowstate_logger.cpp — Logger NATIVO de rt/lowstate para correr EN EL JETSON del G1 (aarch64),
// sin Python ni Docker. Permite grabar las corridas del paper sin cable a la PC externa:
// el Jetson escucha el DDS interno (eth0, subred interna del robot) y se le controla por WiFi.
// SOLO LEE. No publica nada. Mismo esquema de datos que log_lowstate_run.py (ver lowstate_bin2npz.py).
//
// Uso:  ./lowstate_logger <iface=eth0> <salida.bin>
//       Ctrl+C / SIGINT / SIGTERM -> cierra el archivo limpiamente.
//       Imprime un resumen por segundo (muestras, Hz, pitch, tmax, botones) a stdout.
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using LowState = unitree_hg::msg::dds_::LowState_;

static const int NJ = 29;

#pragma pack(push, 1)
struct Rec {
  double   t_wall;        // epoch segundos (CLOCK_REALTIME)
  uint32_t tick;          // reloj del firmware (ms)
  float    q[NJ];
  float    dq[NJ];
  float    tau_est[NJ];
  int8_t   mode[NJ];
  float    vol[NJ];
  int16_t  temp[NJ][2];   // [0]=carcasa, [1]=bobinado (convencion del SDK)
  uint32_t motorstate[NJ];
  float    rpy[3];
  float    gyro[3];
  float    accel[3];
  int16_t  imu_temp;
  uint16_t buttons;       // wireless_remote bytes [2],[3]
  uint8_t  mode_machine;
  uint8_t  mode_pr;
};
#pragma pack(pop)

static std::atomic<bool> g_stop{false};
static std::atomic<uint64_t> g_n{0};
static std::mutex g_mtx;
static FILE* g_f = nullptr;
static Rec g_last{};

static void on_sig(int) { g_stop.store(true); }

static double now_wall() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

static void Handler(const void* msg) {
  const LowState* ls = static_cast<const LowState*>(msg);
  Rec r;
  r.t_wall = now_wall();
  r.tick = ls->tick();
  const auto& ms = ls->motor_state();
  for (int i = 0; i < NJ; ++i) {
    r.q[i] = ms[i].q();
    r.dq[i] = ms[i].dq();
    r.tau_est[i] = ms[i].tau_est();
    r.mode[i] = static_cast<int8_t>(ms[i].mode());
    r.vol[i] = ms[i].vol();
    r.temp[i][0] = ms[i].temperature()[0];
    r.temp[i][1] = ms[i].temperature()[1];
    r.motorstate[i] = ms[i].motorstate();
  }
  const auto& imu = ls->imu_state();
  for (int k = 0; k < 3; ++k) {
    r.rpy[k] = imu.rpy()[k];
    r.gyro[k] = imu.gyroscope()[k];
    r.accel[k] = imu.accelerometer()[k];
  }
  r.imu_temp = imu.temperature();
  const auto& wr = ls->wireless_remote();
  r.buttons = static_cast<uint16_t>(wr[2]) | (static_cast<uint16_t>(wr[3]) << 8);
  r.mode_machine = ls->mode_machine();
  r.mode_pr = ls->mode_pr();
  std::lock_guard<std::mutex> lk(g_mtx);
  if (g_f) fwrite(&r, sizeof(Rec), 1, g_f);
  g_last = r;
  g_n.fetch_add(1);
}

int main(int argc, char** argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth0";
  std::string out = (argc > 2) ? argv[2] : "lowstate.bin";
  std::signal(SIGINT, on_sig);
  std::signal(SIGTERM, on_sig);
  std::signal(SIGHUP, on_sig);

  g_f = fopen(out.c_str(), "wb");
  if (!g_f) { perror("fopen"); return 1; }
  // cabecera: magic + tamaño del registro (para validar el layout al convertir)
  const char magic[8] = {'G','1','L','S','v','1',0,0};
  uint32_t recsz = sizeof(Rec);
  fwrite(magic, 8, 1, g_f);
  fwrite(&recsz, 4, 1, g_f);

  ChannelFactory::Instance()->Init(0, iface);
  ChannelSubscriber<LowState> sub("rt/lowstate");
  sub.InitChannel(Handler, 10);
  std::printf("[logger] iface=%s out=%s recsz=%u. Ctrl+C para cerrar.\n", iface.c_str(), out.c_str(), recsz);
  std::fflush(stdout);

  uint64_t prev = 0;
  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint64_t n = g_n.load();
    Rec r;
    { std::lock_guard<std::mutex> lk(g_mtx); r = g_last; if (g_f) fflush(g_f); }
    int tmax = -99, hot = -1;
    for (int i = 0; i < NJ; ++i) for (int k = 0; k < 2; ++k) if (r.temp[i][k] > tmax) { tmax = r.temp[i][k]; hot = i; }
    int nmode0 = 0; for (int i = 0; i < NJ; ++i) if (r.mode[i] == 0) ++nmode0;
    std::printf("[%llu] %4llu Hz tick=%u mm=%u rpy=(%+.1f,%+.1f) tmax=%dC@%d mode0=%d btn=0x%04x\n",
                (unsigned long long)n, (unsigned long long)(n - prev), r.tick, r.mode_machine,
                r.rpy[0] * 57.2958f, r.rpy[1] * 57.2958f, tmax, hot, nmode0, r.buttons);
    std::fflush(stdout);
    prev = n;
  }
  { std::lock_guard<std::mutex> lk(g_mtx); fclose(g_f); g_f = nullptr; }
  std::printf("[logger] cerrado: %llu muestras -> %s\n", (unsigned long long)g_n.load(), out.c_str());
  return 0;
}
