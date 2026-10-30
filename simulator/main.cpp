#include "avoidance_sim/engine.hpp"
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv) {
  try {
    std::string scenario,output,config;avoidance_sim::RunOptions options;double duration=-1;
    for(int i=1;i<argc;++i) {
      std::string arg=argv[i];auto value=[&](){if(i+1>=argc)throw std::invalid_argument("missing value for "+arg);return std::string(argv[++i]);};
      if(arg=="--scenario")scenario=value();else if(arg=="--output")output=value();else if(arg=="--config")config=value();
      else if(arg=="--seed")options.seed=static_cast<unsigned>(std::stoul(value()));else if(arg=="--duration")duration=std::stod(value());
      else if(arg=="--no-record")options.record=false;else if(arg=="--debug")options.debug=true;
      else if(arg=="--realtime")options.realtime=true;else if(arg=="--headless"){}
      else if(arg=="--help") {std::cout<<"avoidance_sim --scenario FILE [--output DIR] [--seed N] [--duration SECONDS]\n"
        "  [--config JSON] [--headless] [--no-record] [--debug] [--realtime]\n"
        "Exit: 0 completed, 2 unresolved, 3 collision, 1 invalid input/I/O. Headless is default.\n";return 0;}
      else throw std::invalid_argument("unknown argument: "+arg);
    }
    if(scenario.empty())throw std::invalid_argument("--scenario is required (see --help)");
    auto s=avoidance_sim::load_scenario(scenario,config);
    if(duration!=-1){if(!std::isfinite(duration)||duration<=0)throw std::invalid_argument("duration must be positive");s.duration=duration;}
    const auto result=avoidance_sim::Engine{}.run(s,options);
    if(!output.empty())avoidance_sim::write_run(output,s,options,result);
    std::cout<<result.metrics.dump(2)<<'\n';
    return result.metrics["collisions"].get<int>()?3:result.metrics["outcome"]=="completed"?0:2;
  } catch(const std::exception& e){std::cerr<<"avoidance_sim: "<<e.what()<<'\n';return 1;}
}
