// Accelerated experiment: production control law, allocator and MMG equations.
// No topics are published and no real actuators are connected.
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include "thruster_driver/velocity_control.hpp"
#include "thruster_driver/allocation.hpp"
#include "dutyed_tf_pub_with_disturbance/mmg_doyle_model.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto n = std::make_shared<rclcpp::Node>("velocity_benchmark");
    auto param = [&](const std::string & key, double value) {return n->declare_parameter<double>(key, value);};
    njord::sim::DoyleParams p;
#define PARAM(name) p.name = param("mmg." #name, p.name)
    PARAM(Lpp); PARAM(D); PARAM(mass_kg); PARAM(I_zG); PARAM(x_G);
    PARAM(m_x); PARAM(m_y); PARAM(J_z); PARAM(R_0); PARAM(Y_0); PARAM(Y_v); PARAM(Y_r);
    PARAM(N_0); PARAM(N_v); PARAM(N_r);
#undef PARAM
    if (p.mass_kg <= 0 || p.m_x < 0 || p.m_y < 0 || p.I_zG <= 0 || p.J_z < 0 || p.Lpp <= 0 || p.D <= 0)
      throw std::runtime_error("invalid plant parameters");
    njord::sim::MMGDoyleModel plant(p);
    using Control = njord::thruster_driver::VelocityControl;
    Control controller;
    Control::Axes kp{param("control.p.surge",28.29),param("control.p.sway",28.29),param("control.p.yaw",12.17)};
    Control::Axes ki{param("control.i.surge",0),param("control.i.sway",0),param("control.i.yaw",0)};
    Control::Axes limits{param("control.max_surge_wrench",145.7),param("control.max_sway_wrench",145.7),param("control.max_yaw_wrench",206)};
    const double reference_tau = param("control.reference_tau_sec",0.0);
    const auto mode = n->declare_parameter<std::string>("mode","velocity");
    if (mode != "velocity" && mode != "force" && mode != "thruster") throw std::runtime_error("invalid mode");
    const double cap = param("force_limit",100.0);
    const double scale = param("actuator_scale",1.0);
    const double lag = param("actuator_tau",0.0);
    auto forward = n->declare_parameter<std::vector<double>>("actuator_forward",{1.,1.,1.,1.});
    auto reverse = n->declare_parameter<std::vector<double>>("actuator_reverse",{1.,1.,1.,1.});
    if(forward.size()!=4 || reverse.size()!=4) throw std::runtime_error("actuator gains need four entries");
    for(double gain:forward) if(!std::isfinite(gain)||gain<=0) throw std::runtime_error("invalid actuator gain");
    for(double gain:reverse) if(!std::isfinite(gain)||gain<=0) throw std::runtime_error("invalid actuator gain");
    if (cap <= 0 || scale <= 0 || lag < 0) throw std::runtime_error("invalid actuator parameters");
    const std::vector<njord::thruster_driver::ThrusterGeometry> geometry = {
      {-0.1803,-0.25,2.356194,cap,false},{-0.1803,0.25,-2.356194,cap,false},
      {0.1803,-0.25,0.785398,cap,false},{0.1803,0.25,-0.785398,cap,false}};
    std::ifstream input(n->declare_parameter<std::string>("input_csv",""));
    std::ofstream out(n->declare_parameter<std::string>("output_csv",""));
    if (!input || !out) throw std::runtime_error("cannot open experiment files");
    out << std::setprecision(17) << "time_s,ref_u,ref_v,ref_r,u,v,r,tau_x,tau_y,tau_n,force_rr,force_rl,force_fr,force_fl,command_rr,command_rl,command_fr,command_fl,saturated\n";
    std::string line; std::getline(input,line);
    njord::sim::PlanarState state;
    std::array<double,4> forces{};
    double previous = -1;
    while (std::getline(input,line)) {
      for (auto & c : line) if(c==',') c=' ';
      std::istringstream row(line); double t; Control::Axes ref{};
      if (!(row>>t>>ref[0]>>ref[1]>>ref[2])) throw std::runtime_error("invalid experiment row");
      double fourth=0;
      if(mode=="thruster" && !(row>>fourth)) throw std::runtime_error("thruster mode needs four forces");
      const double dt = previous < 0 ? 0.02 : t-previous;
      if (!std::isfinite(t) || dt <= 0 || dt > 0.1) throw std::runtime_error("invalid sample interval");
      previous=t;
      auto wrench = mode != "velocity" ? ref : controller.step(ref,{state.u,state.v,state.r},dt,kp,ki,limits,reference_tau);
      auto commands = njord::thruster_driver::allocateWrench(geometry,{wrench[0],wrench[1],wrench[2]},1e-4);
      if(mode=="thruster") {
        commands = {ref[0]/cap,ref[1]/cap,ref[2]/cap,fourth/cap};
        for(auto & value:commands) value=std::clamp(value,-1.,1.);
      }
      const auto achieved = njord::thruster_driver::commandToWrench(geometry,commands);
      controller.trackApplied(wrench,{achieved[0],achieved[1],achieved[2]},dt,ki);
      bool saturated=false;
      for(size_t i=0;i<4;++i) {
        const double target=commands[i]*cap*scale*(commands[i]>=0 ? forward[i] : reverse[i]);
        forces[i] += (lag > 0 ? 1-std::exp(-dt/lag) : 1.0)*(target-forces[i]);
        saturated |= std::abs(commands[i]) > 0.999;
      }
      std::vector<double> physical;
      for(size_t i=0;i<4;++i) physical.push_back(forces[i]/cap);
      const auto applied=njord::thruster_driver::commandToWrench(geometry,physical);
      out<<t;
      for(double x:ref) out<<','<<x;
      out<<','<<state.u<<','<<state.v<<','<<state.r;
      for(double x:applied) out<<','<<x;
      for(double x:forces) out<<','<<x;
      for(double x:commands) out<<','<<x*cap;
      out<<','<<saturated<<'\n';
      auto a=plant.computeAccel(state,{applied[0],applied[1],applied[2]});
      state.u+=a.du*dt; state.v+=a.dv*dt; state.r+=a.dr*dt;
      if (!std::isfinite(state.u+state.v+state.r) || std::hypot(state.u,state.v)>10 || std::abs(state.r)>10)
        throw std::runtime_error("unstable experiment; stop and inspect parameters");
    }
    rclcpp::shutdown(); return 0;
  } catch (const std::exception & e) {
    fprintf(stderr,"velocity_benchmark: %s\n",e.what()); rclcpp::shutdown(); return 1;
  }
}
