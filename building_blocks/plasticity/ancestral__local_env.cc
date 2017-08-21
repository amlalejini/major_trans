
#include <string>

#include "base/Ptr.h"
#include "base/vector.h"
#include "tools/Random.h"
#include "tools/random_utils.h"
#include "tools/BitSet.h"
#include "tools/Math.h"
#include "evo3/World.h"
#include "config/ArgManager.h"
#include "hardware/EventDrivenGP.h"
#include "hardware/AvidaGP.h"

#include "PABBConfig.h"

//

using hardware_t = emp::EventDrivenGP;
using state_t = emp::EventDrivenGP::State;
using affinity_t = emp::BitSet<8>;
using program_t = emp::EventDrivenGP::Program;
using inst_t = typename::emp::EventDrivenGP::inst_t;
using inst_lib_t = typename::emp::EventDrivenGP::inst_lib_t;
using event_t = typename::emp::EventDrivenGP::event_t;
using event_lib_t = typename::emp::EventDrivenGP::event_lib_t;

constexpr size_t TRAIT_ID__X_LOC = 0;
constexpr size_t TRAIT_ID__Y_LOC = 0;
constexpr size_t TRAIT_ID__DIR = 0;
constexpr size_t TRAIT_ID__RES = 0;
constexpr size_t TRAIT_ID__LAST_EXPORT = 0;
constexpr size_t TRAIT_ID__COPY_CNT = 0;
constexpr size_t TRAIT_ID__MSG_DIR = 0;

constexpr size_t NUM_NEIGHBORS = 4;


class EventDrivenGPOrg : public emp::EventDrivenGP {
public:
  const program_t & GetGenome() { return GetProgram(); }
};


void Inst_CopyInst(emp::EventDrivenGP & hw, const inst_t inst) {
  hw.SetTrait(TRAIT_ID__COPY_CNT, hw.GetTrait(TRAIT_ID__COPY_CNT) + 1);
}

// @amlalejini - TODO
void Inst_Repro(emp::EventDrivenGP & hw, const inst_t inst) {

}

// @amlalejini - TODO
void Inst_ReproRdy(emp::EventDrivenGP & hw, const inst_t inst) {

}

void Inst_Export0(emp::EventDrivenGP & hw, const inst_t inst) {
  hw.SetTrait(TRAIT_ID__LAST_EXPORT, 0);
  // @amlalejini - TODO: trigger export event.
}

void Inst_Export1(emp::EventDrivenGP & hw, const inst_t inst) {
  hw.SetTrait(TRAIT_ID__LAST_EXPORT, 1);
  // @amlalejini - TODO: trigger export event.
}

void Inst_Export2(emp::EventDrivenGP & hw, const inst_t inst) {
  hw.SetTrait(TRAIT_ID__LAST_EXPORT, 2);
  // @amlalejini - TODO: trigger export event.
}

void Inst_RotCW(emp::EventDrivenGP & hw, const inst_t inst) {
  hw.SetTrait(TRAIT_ID__DIR, emp::Mod(hw.GetTrait(TRAIT_ID__DIR) + 1, NUM_NEIGHBORS));
}

void Inst_RotCCW(emp::EventDrivenGP & hw, const inst_t inst) {
  hw.SetTrait(TRAIT_ID__DIR, emp::Mod(hw.GetTrait(TRAIT_ID__DIR) - 1, NUM_NEIGHBORS));
}

void Inst_RotDir(emp::EventDrivenGP & hw, const inst_t inst) {
  state_t & state = *hw.GetCurState();
  hw.SetTrait(TRAIT_ID__DIR, emp::Mod((int)state.AccessLocal(inst.args[0]), NUM_NEIGHBORS));
}

void Inst_GetDir(emp::EventDrivenGP & hw, const inst_t inst) {
  state_t & state = *hw.GetCurState();
  state.SetLocal(inst.args[0], hw.GetTrait(TRAIT_ID__DIR));
}

void Inst_SendMsgFacing(emp::EventDrivenGP & hw, const inst_t inst) {
  state_t & state = *hw.GetCurState();
  hw.TriggerEvent("Message", inst.affinity, state.output_mem, {"facing"});
}

void Inst_SendMsgRandom(emp::EventDrivenGP & hw, const inst_t inst) {
  state_t & state = *hw.GetCurState();
  hw.TriggerEvent("Message", inst.affinity, state.output_mem, {"random"});
}

void Inst_SendMsg(emp::EventDrivenGP & hw, const inst_t inst) {
  state_t & state = *hw.GetCurState();
  hw.SetTrait(TRAIT_ID__MSG_DIR, emp::Mod((int)state.AccessLocal(inst.args[0]), NUM_NEIGHBORS));
  hw.TriggerEvent("Message", inst.affinity, state.output_mem, {"arg"});
}

void Inst_BindEnv(emp::EventDrivenGP & hw, const inst_t inst) {
  hw.TriggerEvent("BindEnv");
}

int main(int argc, char* argv[]) {
  // Load experiment parameters.
  MajorTransConfig config;
  std::string config_fname = "ancestral__local_env.cfg";

  config.Read(config_fname);
  auto args = emp::cl::ArgManager(argc, argv);
  if (args.ProcessConfigOptions(config, std::cout, config_fname, "PABBConfig.h") == false) exit(0);
  if (args.TestUnknown() == false) exit(0);

  std::cout << "==============================" << std::endl;
  std::cout << "|    How am I configured?    |" << std::endl;
  std::cout << "==============================" << std::endl;
  config.Write(std::cout);
  std::cout << "==============================" << std::endl;

  // Localize experiment parameters.
  int RAND_SEED = config.RANDOM_SEED();
  size_t GRID_WIDTH = config.GRID_WIDTH();
  size_t GRID_HEIGHT = config.GRID_HEIGHT();
  size_t GRID_SIZE = GRID_WIDTH * GRID_HEIGHT;
  size_t GENERATIONS = config.GENERATIONS();

  constexpr size_t NUM_ENV_STATES = 3;

  // Create random number generator.
  emp::Ptr<emp::Random> random = emp::NewPtr<emp::Random>(RAND_SEED);

  // Define a convenient affinity table.
  emp::vector<affinity_t> affinity_table(256);
  for (size_t i = 0; i < affinity_table.size(); ++i) {
    affinity_table[i].SetByte(0, (uint8_t)i);
  }

  // Setup environment state affinities.
  emp::vector<affinity_t> env_state_affs = {affinity_table[0], affinity_table[15], affinity_table[255]};

  // Setup instruction set.
  emp::Ptr<inst_lib_t> inst_lib = emp::NewPtr<inst_lib_t>();
  // Standard instructions:
  inst_lib->AddInst("Inc", hardware_t::Inst_Inc, 1, "Increment value in local memory Arg1");
  inst_lib->AddInst("Dec", hardware_t::Inst_Dec, 1, "Decrement value in local memory Arg1");
  inst_lib->AddInst("Not", hardware_t::Inst_Not, 1, "Logically toggle value in local memory Arg1");
  inst_lib->AddInst("Add", hardware_t::Inst_Add, 3, "Local memory: Arg3 = Arg1 + Arg2");
  inst_lib->AddInst("Sub", hardware_t::Inst_Sub, 3, "Local memory: Arg3 = Arg1 - Arg2");
  inst_lib->AddInst("Mult", hardware_t::Inst_Mult, 3, "Local memory: Arg3 = Arg1 * Arg2");
  inst_lib->AddInst("Div", hardware_t::Inst_Div, 3, "Local memory: Arg3 = Arg1 / Arg2");
  inst_lib->AddInst("Mod", hardware_t::Inst_Mod, 3, "Local memory: Arg3 = Arg1 % Arg2");
  inst_lib->AddInst("TestEqu", hardware_t::Inst_TestEqu, 3, "Local memory: Arg3 = (Arg1 == Arg2)");
  inst_lib->AddInst("TestNEqu", hardware_t::Inst_TestNEqu, 3, "Local memory: Arg3 = (Arg1 != Arg2)");
  inst_lib->AddInst("TestLess", hardware_t::Inst_TestLess, 3, "Local memory: Arg3 = (Arg1 < Arg2)");
  inst_lib->AddInst("If", hardware_t::Inst_If, 1, "Local memory: If Arg1 != 0, proceed; else, skip block.", emp::ScopeType::BASIC, 0, {"block_def"});
  inst_lib->AddInst("While", hardware_t::Inst_While, 1, "Local memory: If Arg1 != 0, loop; else, skip block.", emp::ScopeType::BASIC, 0, {"block_def"});
  inst_lib->AddInst("Countdown", hardware_t::Inst_Countdown, 1, "Local memory: Countdown Arg1 to zero.", emp::ScopeType::BASIC, 0, {"block_def"});
  inst_lib->AddInst("Close", hardware_t::Inst_Close, 0, "Close current block if there is a block to close.", emp::ScopeType::BASIC, 0, {"block_close"});
  inst_lib->AddInst("Break", hardware_t::Inst_Break, 0, "Break out of current block.");
  inst_lib->AddInst("Call", hardware_t::Inst_Call, 0, "Call function that best matches call affinity.", emp::ScopeType::BASIC, 0, {"affinity"});
  inst_lib->AddInst("Return", hardware_t::Inst_Return, 0, "Return from current function if possible.");
  inst_lib->AddInst("SetMem", hardware_t::Inst_SetMem, 2, "Local memory: Arg1 = numerical value of Arg2");
  inst_lib->AddInst("CopyMem", hardware_t::Inst_CopyMem, 2, "Local memory: Arg1 = Arg2");
  inst_lib->AddInst("SwapMem", hardware_t::Inst_SwapMem, 2, "Local memory: Swap values of Arg1 and Arg2.");
  inst_lib->AddInst("Input", hardware_t::Inst_Input, 2, "Input memory Arg1 => Local memory Arg2.");
  inst_lib->AddInst("Output", hardware_t::Inst_Output, 2, "Local memory Arg1 => Output memory Arg2.");
  inst_lib->AddInst("Commit", hardware_t::Inst_Commit, 2, "Local memory Arg1 => Shared memory Arg2.");
  inst_lib->AddInst("Pull", hardware_t::Inst_Pull, 2, "Shared memory Arg1 => Shared memory Arg2.");
  inst_lib->AddInst("Nop", hardware_t::Inst_Nop, 0, "No operation.");
  // Custom instructions:
  inst_lib->AddInst("CopyInst", Inst_CopyInst, 0, "Simulates self-copying of instruction.");
  inst_lib->AddInst("Repro", Inst_Repro, 0, "Triggers reproduction if able.");
  inst_lib->AddInst("ReproRdy", Inst_ReproRdy, 1, "Local memory Arg1 => Ready to repro?");
  inst_lib->AddInst("Export0", Inst_Export0, 0, "Export product ID 0.");
  inst_lib->AddInst("Export1", Inst_Export1, 0, "Export product ID 1.");
  inst_lib->AddInst("Export2", Inst_Export2, 0, "Export product ID 2.");
  inst_lib->AddInst("RotCW", Inst_RotCW, 0, "Rotate orientation clockwise (90 degrees) once.");
  inst_lib->AddInst("RotCCW", Inst_RotCCW, 0, "Rotate orientation counter-clockwise (90 degrees) once.");
  inst_lib->AddInst("RotDir", Inst_RotDir, 1, "Rotate to face direction specified by Arg1 (Arg1 mod 4)");
  inst_lib->AddInst("GetDir", Inst_GetDir, 1, "Local memory Arg1 => Current direction.");
  inst_lib->AddInst("SendMsgFacing", Inst_SendMsgFacing, 0, "Send output memory as message event to faced neighbor.", emp::ScopeType::BASIC, 0, {"affinity"});
  inst_lib->AddInst("SendMsgRandom", Inst_SendMsgRandom, 0, "Send output memory as message event to random neighbor.", emp::ScopeType::BASIC, 0, {"affinity"});
  inst_lib->AddInst("SendMsg", Inst_SendMsg, 1, "Send output memory as message event to neighbor specified by local memory Arg1.", emp::ScopeType::BASIC, 0, {"affinity"});
  inst_lib->AddInst("BindEnv", Inst_BindEnv, 0, "Bind environment to appropriate function.");

  // Setup the event library.
  emp::Ptr<event_lib_t> event_lib = emp::NewPtr<event_lib_t>(*emp::EventDrivenGP::DefaultEventLib());
  // TODO: setup events
  //  [ ] Message dispatcher
  //  [ ] BindEnv handler [ ] BindEnv dispatcher

  // Setup the world.
  using org_t = EventDrivenGPOrg;
  //emp::World<emp::AvidaGP> world(random);
  // const std::function<const program_t & (org_t &)> get_genome_fun = [](org_t & hw){ return hw.GetProgram(); };
  // world.SetGetGenomeFun(get_genome_fun);

  // Setup the environment.


  random.Delete();
  return 0;
}
