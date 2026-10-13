// g1_launcher.cpp — lanzador con gatillo para g1_ctrl
// SOLO LEE. NUNCA publica lowcmd -> cero conflicto con fabrica.
// GATE de seguridad: solo lanza g1_ctrl si NADIE mas publica rt/lowcmd (= estamos en debug mode,
// ai_sport apagado). En modo fabrica (ai_sport publicando lowcmd) el combo NO hace nada.
// Combo LANZAR = L1+R1+A (held ~0.5s) -> systemctl --user start g1ctrl
// Combo STOP   = L1+R1+B (held ~0.5s) -> systemctl --user stop  g1ctrl
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>  // BtnUnion, REMOTE_DATA_RX

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;

static std::atomic<uint16_t> g_btn{0};
static std::atomic<bool>     g_have_state{false};
static std::atomic<double>   g_last_lowcmd{-1e9};  // ultimo instante (s) que se vio un rt/lowcmd

static double now_s() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

static void StateHandler(const void* msg) {
  auto* ls = static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
  unitree::common::REMOTE_DATA_RX rx;
  std::memcpy(&rx, &ls->wireless_remote()[0], 40);
  g_btn.store(rx.RF_RX.btn.value);
  g_have_state.store(true);
}

// Cualquiera que publique rt/lowcmd (ai_sport de fabrica, u otro) marca "control activo".
static void CmdHandler(const void*) {
  g_last_lowcmd.store(now_s());
}

int main(int argc, char** argv) {
  std::string iface = (argc > 1) ? argv[1] : "eth0";
  ChannelFactory::Instance()->Init(0, iface);

  ChannelSubscriber<unitree_hg::msg::dds_::LowState_> state_sub("rt/lowstate");
  state_sub.InitChannel(StateHandler, 1);
  ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_> cmd_sub("rt/lowcmd");
  cmd_sub.InitChannel(CmdHandler, 1);

  std::cout << "[launcher] esperando rt/lowstate en " << iface << " ..." << std::endl;
  while (!g_have_state.load()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "[launcher] listo. LANZAR=L1+R1+A(0.5s)  STOP=L1+R1+B(0.5s). "
               "Solo lanza en DEBUG MODE (rt/lowcmd en silencio)." << std::endl;

  const int    HOLD = 25;         // 25 * 20ms = 0.5s
  const double CMD_SILENCE = 0.3; // s sin lowcmd = nadie mas controla = debug mode
  int  launch_held = 0, stop_held = 0;
  bool launch_fired = false, stop_fired = false;

  const char* START = "XDG_RUNTIME_DIR=/run/user/$(id -u) systemctl --user start g1ctrl";
  const char* STOP  = "XDG_RUNTIME_DIR=/run/user/$(id -u) systemctl --user stop g1ctrl";

  while (true) {
    unitree::common::BtnUnion u; u.value = g_btn.load();
    auto& c = u.components;
    bool base   = c.L1 && c.R1;
    bool launch = base && c.A && !c.B;
    bool stop   = base && c.B && !c.A;

    // GATE: hay control de fabrica activo si vimos un lowcmd hace < CMD_SILENCE
    bool factory_active = (now_s() - g_last_lowcmd.load()) < CMD_SILENCE;

    if (launch) {
      if (++launch_held >= HOLD && !launch_fired) {
        if (factory_active) {
          std::cout << "[launcher] COMBO LANZAR ignorado: control de FABRICA activo "
                       "(rt/lowcmd en uso). Entra a debug mode (L2+R2) primero." << std::endl;
        } else {
          std::cout << "[launcher] COMBO LANZAR (debug mode OK) -> start g1ctrl" << std::endl;
          std::system(START);
        }
        launch_fired = true;  // no re-disparar hasta soltar
      }
    } else { launch_held = 0; launch_fired = false; }

    if (stop) {
      if (++stop_held >= HOLD && !stop_fired) {
        std::cout << "[launcher] COMBO STOP -> stop g1ctrl" << std::endl;
        std::system(STOP);
        stop_fired = true;
      }
    } else { stop_held = 0; stop_fired = false; }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return 0;
}
